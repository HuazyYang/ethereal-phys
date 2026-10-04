#include <nvHLSLExtns.h>

#ifndef SOLVE_SHAPES_BLOCK_SIZE
#define SOLVE_SHAPES_BLOCK_SIZE 64
#endif
#define SOLVE_SHAPES_WARP_COUNT (SOLVE_SHAPES_BLOCK_SIZE / NV_WARP_SIZE)

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
groupshared float3 gReduce[SOLVE_SHAPES_WARP_COUNT];
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

// Component-wise cross. The shipped DXBC pairs x and y into one two-wide
// mul/mad and leaves z scalar, which the three-wide cross() intrinsic cannot
// produce, and it is also what makes the basis-axis crosses in ExtractRotation
// fold down to a single masked mul against a literal vector.
float3 Cross(float3 a, float3 b) {
    float3 r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

float3 RotateScaled(float4 q, float3 v, float scale) {
    return v * scale + Cross(q.xyz, v) * q.w * 2.0 + q.xyz * dot(q.xyz, v) * 2.0;
}

float3 Rotate(float4 q, float3 v) {
    return RotateScaled(q, v, 2.0 * q.w * q.w - 1.0);
}

float4 ExtractRotation(float3 c0, float3 c1, float3 c2, float4 q) {
    [loop]
    for (int i = 0; i < 4; ++i) {
        // Rotating the literal basis vectors lets FXC fold each cross and dot
        // into one masked mul; supplying them pre-folded loses the folding and
        // emits plain movs instead.
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

// threadIdx is never written here, but it has to be taken by reference: the
// copy the inout convention forces at the call site is what keeps the lane
// split below inside the enclosing loop. Passed by value the operand is the
// input register itself, and FXC evaluates threadIdx & 31 and threadIdx >> 5
// once in the shader prologue instead of once per iteration.
float3 ReduceSum(inout uint threadIdx, float3 value) {
#if SOLVE_SHAPES_WARP_COUNT > 1
    // Split ahead of the shuffles, not at the use below: the shipped code
    // computes both before the reduction, which is also what lets the three
    // reductions of the covariance loop share one pair of instructions.
    uint lane = threadIdx & (NV_WARP_SIZE - 1);
    int warpIndex = int(threadIdx) >> 5;
#endif
    // Warp reduction with the NVAPI down-shuffle, as in CalculateBounds.hlsl.
    [unroll]
    for (uint delta = 1; delta < NV_WARP_SIZE; delta <<= 1) {
        float3 shuffled;
        shuffled.x = asfloat(NvShflDown(asint(value.x), delta));
        shuffled.y = asfloat(NvShflDown(asint(value.y), delta));
        shuffled.z = asfloat(NvShflDown(asint(value.z), delta));
        value = value + shuffled;
    }

#if SOLVE_SHAPES_WARP_COUNT > 1
    if (lane == 0)
        gReduce[warpIndex] = value;
    GroupMemoryBarrierWithGroupSync();
    if (threadIdx == 0) {
        float3 total = gReduce[0];
        [unroll]
        for (uint warp = 1; warp < SOLVE_SHAPES_WARP_COUNT; ++warp)
            total = total + gReduce[warp];
        gReduce[0] = total;
    }
    GroupMemoryBarrierWithGroupSync();
#else
    if (threadIdx == 0)
        gReduce[0] = value;
    GroupMemoryBarrierWithGroupSync();
#endif
    return gReduce[0];
}

void AccumulateDelta(uint addr, float3 delta) {
    NvInterlockedAddFp32(accum, addr + 0, delta.x);
    NvInterlockedAddFp32(accum, addr + 4, delta.y);
    NvInterlockedAddFp32(accum, addr + 8, delta.z);
    NvInterlockedAddFp32(accum, addr + 12, 1.0);
}

void SolveShapesNVBody(uint rigid, uint threadIdx) {
    int begin = rigidOffsets[rigid];
    int count = rigidOffsets[rigid + 1] - begin;
    int iterations = int(uint(count + (SOLVE_SHAPES_BLOCK_SIZE - 1)) / SOLVE_SHAPES_BLOCK_SIZE);

    if (threadIdx == 0) {
        gCenter = 0.0;
        gPrevTranslation = translations[rigid];
        gCovariance._11_12_13 = 0.0;
        gCovariance._21_22_23 = 0.0;
        gCovariance._31_32_33 = 0.0;
    }

    [loop]
    for (int i = 0; i < iterations; ++i) {
        uint index = (uint(i) * SOLVE_SHAPES_BLOCK_SIZE) | threadIdx;
        bool valid = int(index) < count;
        uint entry = uint(begin) + index;
        float3 position = newPositions[reverseLookup[rigidIndices[entry]]].xyz;
        float3 sum = ReduceSum(threadIdx, valid ? position : 0.0);
        if (threadIdx == 0)
            gCenter = gCenter + sum;
        GroupMemoryBarrierWithGroupSync();
    }

    if (threadIdx == 0) {
        // Bound to a local so the itof lands ahead of the gCenter load, as the
        // shipped preamble has it.
        float denominator = float(count);
        float3 center = gCenter / denominator;
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
        uint index = (uint(j) * SOLVE_SHAPES_BLOCK_SIZE) | threadIdx;
        float3 a, b, c;
        [branch] if (int(index) < count) {
            uint entry = uint(begin) + index;
            float3 offset = newPositions[reverseLookup[rigidIndices[entry]]].xyz - gCenter;
            float3 local = localPositions[entry];
            a = offset.x * local;
            b = offset.y * local;
            c = offset.z * local;
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
        // Read before the covariance so the u3 load keeps its shipped position.
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
    // Bound here rather than left inside Rotate: the shipped preamble evaluates
    // 2*w*w - 1 between the gRotation and gCenter loads, which is where the
    // named local puts it.
    float scale = 2.0 * rotation.w * rotation.w - 1.0;
    float3 center = gCenter;

    [loop]
    for (int k = 0; k < iterations; ++k) {
        uint index = (uint(k) * SOLVE_SHAPES_BLOCK_SIZE) | threadIdx;
        if (int(index) < count) {
            uint entry = uint(begin) + index;
            uint particle = uint(rigidIndices[entry]);
            uint sortedIndex = uint(reverseLookup[particle]);

            float3 position = newPositions[sortedIndex].xyz;
            float3 local = localPositions[entry];
            float3 goal = center + RotateScaled(rotation, local, scale);
            float3 difference = position - goal;

            // Spelled out rather than routed through RotateScaled: the shipped
            // code scales the float4 it keeps for the w component and re-reads
            // the element for the cross and the dot, which is two loads of t7.
            float4 localNormal = localNormals[entry];
            float3 rotatedNormal = localNormal.xyz * scale
                                 + Cross(rotation.xyz, localNormals[entry].xyz) * rotation.w * 2.0
                                 + rotation.xyz * dot(rotation.xyz, localNormals[entry].xyz) * 2.0;
            normals[particle] = float4(rotatedNormal, localNormal.w);

            uint addr = sortedIndex << 4;
            AccumulateDelta(addr, -difference * coefficient);
        }
    }
}

[numthreads(SOLVE_SHAPES_BLOCK_SIZE, 1, 1)]
void SolveShapesNV(uint rigid : SV_GroupID, uint threadIdx : SV_GroupThreadID) {
    SolveShapesNVBody(rigid, threadIdx);
}
