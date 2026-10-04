#include "Utils.hlsli"
#if NVFLEX_XPBD
#include "KernelParams.hlsli"
#endif

StructuredBuffer<int> rigidOffsets : register(t0);
StructuredBuffer<int> rigidIndices : register(t1);
StructuredBuffer<float> rigidCoefficients : register(t2);
StructuredBuffer<int> reverseLookup : register(t5);
StructuredBuffer<float4> localNormals : register(t7);
RWByteAddressBuffer accum : register(u0);
RWStructuredBuffer<float3> localPositions : register(u1);
RWStructuredBuffer<float4> newPositions : register(u2);
RWStructuredBuffer<float4> rotations : register(u3);
RWStructuredBuffer<float3> translations : register(u4);
RWStructuredBuffer<float4> normals : register(u5);

groupshared float3 gCenter;
groupshared float3 gPrevTranslation;
groupshared float3x3 gCovariance;
groupshared float3 gReduce[64];
groupshared float4 gRotation;

// The accumulations below are deliberately written as flat left-to-right chains
// rather than the more readable grouped form. FXC contracts each "+ product" into
// a mad, so the parenthesisation decides where the intermediate roundings fall;
// grouping the cross-product terms yields a balanced tree that is a valid
// quaternion product but not the shipped one, and the difference is visible in
// rigid-pile scenes within a single frame.
//
// Three details here are load-bearing, and together they fix the lane packing of
// the cross sum in ExtractRotation -- FXC picks that packing from how QuatMul
// consumes the axis quaternion, not from the cross expression itself:
//   * `a.xyz * b.w` rather than `b.w * a.xyz`, because mul emits its operands in
//     reverse source order while mad keeps them;
//   * x and y share one four-wide product `t` that is read back with a stride-two
//     swizzle, and z stays scalar, which is exactly how the shipped code narrows;
//   * the last product of the w chain is hoisted into `zz`. That single hoist is
//     what moves the four-wide cross from lanes y,z,w to the shipped x,z,w --
//     hoisting any earlier product instead lets FXC fold it into the leading
//     three-wide mul and loses an instruction.
// Per-component arithmetic order is unchanged, so the rounding is identical.
float4 QuatMul(float4 a, float4 b) {
    float3 v = a.w * b.xyz + a.xyz * b.w;
    // The four x/y cross-term products are materialised as one four-wide multiply
    // and read back stride-two. FXC re-fuses `t` into the two mads, so the
    // instruction count is unchanged, but the `a` operands are now laid out
    // interleaved -- (a.y, a.z | a.z, a.x) instead of two consecutive pairs. That is
    // what makes the `+` term read the canonical lane of the duplicated z component
    // and the `-` term read the spare lane, which is the shipped swizzle pair.
    float4 t = a.yzzx * b.zyxz;
    float2 xy = v.xy + t.xz - t.yw;
    float zz = a.z * b.z;
    float z = v.z + a.x * b.y - a.y * b.x;
    float w = a.w * b.w - a.x * b.x - a.y * b.y - zz;
    return float4(xy, z, w);
}

float4 NormalizeQuat(float4 q) {
    // Written as a mul/add/mad chain rather than dot(q, q): the shipped code
    // squares x and y with one vector mul and folds only z and w into mads, so a
    // dp4 here would round differently.
    float2 sq = q.xy * q.xy;
    float lengthSq = sq.x + sq.y;
    lengthSq = q.z * q.z + lengthSq;
    lengthSq = q.w * q.w + lengthSq;
    float4 result;
    [branch] if (lengthSq > 0.0)
        result = q * (1.0 / sqrt(lengthSq));
    else
        result = 0.0;
    return result;
}

// The cross product is spelled out component-wise rather than via cross(): the
// intrinsic packs the result three-wide, while the shipped DXBC pairs x and y in
// one two-wide mul/mad and finishes z as a scalar. Writing it out also lets FXC
// fold the basis-axis calls below into a single masked multiply by a constant.
float3 Rotate(float4 q, float3 v) {
    float3 c;
    c.x = q.y * v.z - q.z * v.y;
    c.y = q.z * v.x - q.x * v.z;
    c.z = q.x * v.y - q.y * v.x;
    return v * (2.0 * q.w * q.w - 1.0) + c * q.w * 2.0 + q.xyz * dot(q.xyz, v) * 2.0;
}

float4 ExtractRotation(float3 c0, float3 c1, float3 c2, float4 q) {
    [loop]
    for (int i = 0; i < 4; ++i) {
        float3 r0 = Rotate(q, float3(1.0, 0.0, 0.0));
        float3 r1 = Rotate(q, float3(0.0, 1.0, 0.0));
        float3 r2 = Rotate(q, float3(0.0, 0.0, 1.0));

        float3 numerator = cross(r0, c0) + cross(r1, c1) + cross(r2, c2);
        float denominator = dot(r0, c0) + dot(r1, c1) + dot(r2, c2);
        float scale = 1.0 / abs(denominator) + 1.0e-9;
        float3 omega = numerator * scale;

        float angle = length(omega);
        if (angle < 1.0e-9)
            break;

        float3 axis = normalize(omega * (1.0 / angle));
        float s, c;
        sincos(angle * 0.5, s, c);
        q = NormalizeQuat(QuatMul(float4(axis * s, c), q));
    }
    return q;
}

float3 ReduceSum(uint threadIdx, float3 value) {
    gReduce[threadIdx] = value;
    [loop]
    for (uint stride = 32; stride > 0; stride >>= 1) {
        GroupMemoryBarrierWithGroupSync();
        if (threadIdx < stride) {
            // The partner index is formed before either load: the shipped code
            // issues the iadd ahead of both ld_structured instructions.
            uint other = threadIdx + stride;
            gReduce[threadIdx] = gReduce[threadIdx] + gReduce[other];
        }
    }
    GroupMemoryBarrierWithGroupSync();
    return gReduce[0];
}

void AccumulateDelta(uint sortedIndex, float3 difference, float coefficient) {
#if NVFLEX_XPBD
    // XPBD (Macklin, Muller, Chentanez 2016, Eq. 18): each particle is a
    // distance-to-goal constraint with the goal fixed for this iteration. It is
    // solved statelessly (the goal is re-fitted every iteration, so an
    // accumulated multiplier has no meaning) and mass-free like the PBD kernel.
    // alpha = 1 / (stiffnessMin * (stiffnessMax/stiffnessMin)^k) from the
    // stiffness k (see SolveSpringsXPBD.hlsl); k <= 0 disables the constraint
    // (no delta, no count). With C = |difference| and n = difference / C,
    // dlam = -C / (1 + alphaT) and delta = dlam * n = -difference / (1 + alphaT).
    float k = min(coefficient, 1.0);
    if (k <= 0.0)
        return;
    uint addr = sortedIndex << 4;
    float alphaT = gParams.kInvStiffnessMin * exp2(-k * gParams.kLogStiffnessRange) * gParams.kInvDt * gParams.kInvDt;
    float3 delta = -difference / (1.0 + alphaT);
#else
    // The address is formed before the delta: the shipped code issues the ishl
    // ahead of the multiply by the stiffness coefficient.
    uint addr = sortedIndex << 4;
    float3 delta = -difference * coefficient;
#endif
    InterlockedAddFp32(accum, addr + 0, delta.x);
    InterlockedAddFp32(accum, addr + 4, delta.y);
    InterlockedAddFp32(accum, addr + 8, delta.z);
    InterlockedAddFp32(accum, addr + 12, 1.0);
}

[numthreads(64, 1, 1)]
void SolveShapes(uint rigid : SV_GroupID, uint threadIdx : SV_GroupThreadID) {
    int begin = rigidOffsets[rigid];
    int count = rigidOffsets[rigid + 1] - begin;
    int iterations = int(uint(count + 63) >> 6);

    if (threadIdx == 0) {
        gCenter = 0.0;
        gPrevTranslation = translations[rigid];
        gCovariance._11_12_13 = 0.0;
        gCovariance._21_22_23 = 0.0;
        gCovariance._31_32_33 = 0.0;
    }

    [loop]
    for (int i = 0; i < iterations; ++i) {
        uint index = (uint(i) << 6) | threadIdx;
        bool valid = int(index) < count;
        uint entry = uint(begin) + index;
        float3 position = newPositions[reverseLookup[rigidIndices[entry]]].xyz;
        float3 sum = ReduceSum(threadIdx, valid ? position : 0.0);
        if (threadIdx == 0)
            gCenter = gCenter + sum;
        GroupMemoryBarrierWithGroupSync();
    }

    if (threadIdx == 0) {
        float total = float(count);
        float3 center = gCenter / total;
        gCenter = center;

        if (abs(center.x - gPrevTranslation.x) < 1.0e-5)
            gCenter.x = gPrevTranslation.x;
        if (abs(center.y - gPrevTranslation.y) < 1.0e-5)
            gCenter.y = gPrevTranslation.y;
        if (abs(center.z - gPrevTranslation.z) < 1.0e-5)
            gCenter.z = gPrevTranslation.z;
    }
    GroupMemoryBarrierWithGroupSync();

    [loop]
    for (int j = 0; j < iterations; ++j) {
        uint index = (uint(j) << 6) | threadIdx;
        float3 a, b, c;
        [branch] if (int(index) < count) {
            uint entry = uint(begin) + index;
            float3 offset = newPositions[reverseLookup[rigidIndices[entry]]].xyz - gCenter;
            float3 local = localPositions[entry];
            a = local * offset.x;
            b = local * offset.y;
            c = local * offset.z;
        } else {
            a = 0.0;
            b = 0.0;
            c = 0.0;
        }

        float3 col0 = ReduceSum(threadIdx, float3(a.x, b.x, c.x));
        GroupMemoryBarrierWithGroupSync();
        float3 col1 = ReduceSum(threadIdx, float3(a.y, b.y, c.y));
        GroupMemoryBarrierWithGroupSync();
        float3 col2 = ReduceSum(threadIdx, float3(a.z, b.z, c.z));

        if (threadIdx == 0) {
            gCovariance._11_12_13 = gCovariance._11_12_13 + col0;
            gCovariance._21_22_23 = gCovariance._21_22_23 + col1;
            gCovariance._31_32_33 = gCovariance._31_32_33 + col2;
        }
        GroupMemoryBarrierWithGroupSync();
    }

    if (threadIdx == 0) {
        float4 previous = rotations[rigid];
        float4 rotation = ExtractRotation(gCovariance._11_12_13,
                                          gCovariance._21_22_23,
                                          gCovariance._31_32_33,
                                          previous);
        rotations[rigid] = rotation;
        translations[rigid] = gCenter;
        gRotation = rotation;
    }
    GroupMemoryBarrierWithGroupSync();

    float coefficient = rigidCoefficients[rigid];
    float4 rotation = gRotation;

    [loop]
    for (int k = 0; k < iterations; ++k) {
        uint index = (uint(k) << 6) | threadIdx;
        if (int(index) < count) {
            uint entry = uint(begin) + index;
            uint particle = uint(rigidIndices[entry]);
            uint sortedIndex = uint(reverseLookup[particle]);

            float3 position = newPositions[sortedIndex].xyz;
            float3 local = localPositions[entry];
            // The rotated offset is formed before gCenter is read, so the
            // loop-invariant 2*w*w-1 term is hoisted ahead of the ld_raw.
            float3 rotated = Rotate(rotation, local);
            float3 goal = gCenter + rotated;
            float3 difference = position - goal;

            // Spelled out rather than routed through Rotate: the shipped code
            // scales the float4 it already holds for the w component and re-reads
            // the element for the cross and the dot, which is two loads of t7.
            float4 localNormal = localNormals[entry];
            float3 cn;
            cn.x = rotation.y * localNormals[entry].z - rotation.z * localNormals[entry].y;
            cn.y = rotation.z * localNormals[entry].x - rotation.x * localNormals[entry].z;
            cn.z = rotation.x * localNormals[entry].y - rotation.y * localNormals[entry].x;
            float3 rotatedNormal = localNormal.xyz * (2.0 * rotation.w * rotation.w - 1.0)
                                 + cn * rotation.w * 2.0
                                 + rotation.xyz * dot(rotation.xyz, localNormals[entry].xyz) * 2.0;
            normals[particle] = float4(rotatedNormal, localNormal.w);

            AccumulateDelta(sortedIndex, difference, coefficient);
        }
    }
}
