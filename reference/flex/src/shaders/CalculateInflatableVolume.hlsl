#include "KernelParams.hlsli"

StructuredBuffer<FlexInflatable> inflatables : register(t0);
StructuredBuffer<float4> positions : register(t1);
StructuredBuffer<int> reverseLookup : register(t2);
StructuredBuffer<int> indices : register(t3);
RWStructuredBuffer<float> lambdas: register(u0);
#if NVFLEX_XPBD
RWStructuredBuffer<float> volumeLambdas : register(u1);
#endif

#define BLOCK_DIM_X      512
#define BLOCK_DIM_X_BITS 9u

#if USE_NV_SHADER_EXT

#include <nvHLSLExtns.h>
#define WAVE_SIZE      32
#define WAVE_SIZE_BITS 5

#elif USE_AMD_SHADER_EXT

#include <ags_shader_intrinsics_dx11.hlsl>
#define WAVE_SIZE      32
#define WAVE_SIZE_BITS 5

static const int g_AmdDxExtSwizzleOperations[] = {
    AmdDxExtShaderIntrinsicsSwizzle_SwapX1,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX2,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX4,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX8,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX16,
};

#endif

groupshared FlexInflatable inflatableInBlock;
groupshared float volumeOfInflatable;
groupshared float3 centerOfInflatable;

#if USE_NV_SHADER_EXT || USE_AMD_SHADER_EXT
groupshared float volumesInBlock[BLOCK_DIM_X / WAVE_SIZE];
groupshared float3 centersInBlock[BLOCK_DIM_X / WAVE_SIZE];
#else
groupshared float volumesInBlock[BLOCK_DIM_X];
groupshared float3 centersInBlock[BLOCK_DIM_X];
#endif

// Sums `center` across the whole thread group. The vendor paths reduce within a
// wave using shuffle/swizzle intrinsics and then combine the per-wave partials;
// the generic path walks a shared-memory tree over the live lanes.
float3 ReduceCenter(int threadIdx, int numTrisInBlock, float3 center) {
#if USE_NV_SHADER_EXT || USE_AMD_SHADER_EXT
    const int laneIdx = threadIdx & (WAVE_SIZE - 1);
    const int waveIdx = threadIdx >> int(WAVE_SIZE_BITS);
    float3 v1;
    int i;

#if USE_NV_SHADER_EXT
    [unroll]
    for (i = 1; i < WAVE_SIZE; i <<= 1) {
        v1.x = asfloat(NvShflDown(asint(center.x), i));
        v1.y = asfloat(NvShflDown(asint(center.y), i));
        v1.z = asfloat(NvShflDown(asint(center.z), i));
        center = center + v1;
    }
#else
    [unroll]
    for (i = WAVE_SIZE_BITS - 1; i >= 0; --i) {
        v1.x = AmdDxExtShaderIntrinsics_SwizzleF(center.x, g_AmdDxExtSwizzleOperations[i]);
        v1.y = AmdDxExtShaderIntrinsics_SwizzleF(center.y, g_AmdDxExtSwizzleOperations[i]);
        v1.z = AmdDxExtShaderIntrinsics_SwizzleF(center.z, g_AmdDxExtSwizzleOperations[i]);
        center = center + v1;
    }
#endif

    if (laneIdx == 0)
        centersInBlock[waveIdx] = center;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        const int waveCount = int(uint(numTrisInBlock) >> uint(WAVE_SIZE_BITS));
        float3 total = centersInBlock[0];
        [unroll]
        for (int k = 1; k < waveCount; ++k)
            total = total + centersInBlock[k];
        centersInBlock[0] = total;
    }
    GroupMemoryBarrierWithGroupSync();
    return centersInBlock[0];
#else
    centersInBlock[threadIdx] = center;

    for (uint j = uint(numTrisInBlock) >> 1; j > 0; j >>= 1) {
        GroupMemoryBarrierWithGroupSync();
        if (uint(threadIdx) < j) {
            uint other = threadIdx + j;
            float3 v1 = centersInBlock[threadIdx];
            float3 v2 = centersInBlock[other];
            centersInBlock[threadIdx] = v1 + v2;
        }
    }

    GroupMemoryBarrierWithGroupSync();
    return centersInBlock[0];
#endif
}

// Same reduction for the scalar volume term.
float ReduceVolume(int threadIdx, int numTrisInBlock, float vol) {
#if USE_NV_SHADER_EXT || USE_AMD_SHADER_EXT
    const int laneIdx = threadIdx & (WAVE_SIZE - 1);
    const int waveIdx = threadIdx >> int(WAVE_SIZE_BITS);
    float s1;
    int i;

#if USE_NV_SHADER_EXT
    [unroll]
    for (i = 1; i < WAVE_SIZE; i <<= 1) {
        s1 = asfloat(NvShflDown(asint(vol), i));
        vol = vol + s1;
    }
#else
    [unroll]
    for (i = WAVE_SIZE_BITS - 1; i >= 0; --i) {
        s1 = AmdDxExtShaderIntrinsics_SwizzleF(vol, g_AmdDxExtSwizzleOperations[i]);
        vol = vol + s1;
    }
#endif

    if (laneIdx == 0)
        volumesInBlock[waveIdx] = vol;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        const int waveCount = int(uint(numTrisInBlock) >> uint(WAVE_SIZE_BITS));
        float total = volumesInBlock[0];
        [unroll]
        for (int k = 1; k < waveCount; ++k)
            total = total + volumesInBlock[k];
        volumesInBlock[0] = total;
    }
    GroupMemoryBarrierWithGroupSync();
    return volumesInBlock[0];
#else
    volumesInBlock[threadIdx] = vol;

    for (uint j = uint(numTrisInBlock) >> 1; j > 0; j >>= 1) {
        GroupMemoryBarrierWithGroupSync();
        if (uint(threadIdx) < j) {
            uint other = threadIdx + j;
            float s1 = volumesInBlock[threadIdx];
            float s2 = volumesInBlock[other];
            volumesInBlock[threadIdx] = s1 + s2;
        }
    }
    GroupMemoryBarrierWithGroupSync();
    return volumesInBlock[0];
#endif
}

[numthreads(BLOCK_DIM_X, 1, 1)]
// The system values must be declared as vectors and swizzled here. Taking
// SV_GroupThreadID as a scalar parameter makes FXC 6.3 spill it into a temp
// (`mov rN, vThreadIDInGroup.x`) once the index is live across the nested
// reduction loop, which then pins `and l(31)` / `ishr l(5)` inside the loop
// body instead of the prologue. Read off a swizzle it stays an input register.
void CalculateInflatableVolume(uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID) {

    const int threadIdx = groupThreadId.x;
    const int blockIdx = groupId.x;

    if (threadIdx == 0) {
        inflatableInBlock = inflatables[blockIdx];
        volumeOfInflatable = 0.0;
        centerOfInflatable = 0.0.xxx;
    }
    GroupMemoryBarrierWithGroupSync();

    const int numBlocks = uint(inflatableInBlock.mNumTris + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS;
    const float invNumIndices = 1.0 / (3.0 * inflatableInBlock.mNumTris);

    for (int bkIdx = 0; bkIdx < numBlocks; ++bkIdx) {
        int triIdx = (bkIdx * BLOCK_DIM_X) + threadIdx;
        int idxBase = inflatableInBlock.mStartTri;
        int numTrisInBlock = min(inflatableInBlock.mNumTris - bkIdx * BLOCK_DIM_X, BLOCK_DIM_X);

        float3 center;

        if (triIdx < inflatableInBlock.mNumTris) {
            int triIdxAbs = idxBase + triIdx;
            int idx1 = indices[triIdxAbs * 3];
            int posIdx1 = reverseLookup[idx1];

            int idx2 = indices[triIdxAbs * 3 + 1];
            int posIdx2 = reverseLookup[idx2];

            int idx3 = indices[triIdxAbs * 3 + 2];
            int posIdx3 = reverseLookup[idx3];

            float3 pos1 = positions[posIdx1].xyz;
            float3 pos2 = positions[posIdx2].xyz;
            float3 pos3 = positions[posIdx3].xyz;

            center = (pos1 + pos2 + pos3) * invNumIndices;
        } else
            center = 0.0.xxx;

        center = ReduceCenter(threadIdx, numTrisInBlock, center);

        // Component-wise, not `centerOfInflatable += center`: the shipped blob
        // reads, adds and writes the three groupshared floats one at a time
        // (ld_raw/add/store_raw at byte offsets 0, 4 and 8).
        if (threadIdx == 0) {
            centerOfInflatable.x += center.x;
            centerOfInflatable.y += center.y;
            centerOfInflatable.z += center.z;
        }
        GroupMemoryBarrierWithGroupSync();
    }

    GroupMemoryBarrierWithGroupSync();

    for (int bkIdxj = 0; bkIdxj < numBlocks; ++bkIdxj) {
        int triIdx = bkIdxj * BLOCK_DIM_X + threadIdx;
        int idxBase = inflatableInBlock.mStartTri;
        int numTrisInBlock = min(inflatableInBlock.mNumTris - bkIdxj * BLOCK_DIM_X, BLOCK_DIM_X);

        float vol;

        if (triIdx < inflatableInBlock.mNumTris) {
            int triIdxAbs = triIdx + idxBase;
            int idx1 = indices[triIdxAbs * 3];
            int posIdx1 = reverseLookup[idx1];

            int idx2 = indices[triIdxAbs * 3 + 1];
            int posIdx2 = reverseLookup[idx2];

            int idx3 = indices[triIdxAbs * 3 + 2];
            int posIdx3 = reverseLookup[idx3];

            float3 pos1 = positions[posIdx1].xyz - centerOfInflatable;
            float3 pos2 = positions[posIdx2].xyz - centerOfInflatable;
            float3 pos3 = positions[posIdx3].xyz - centerOfInflatable;

            pos2 -= pos1;
            pos3 -= pos1;

            // Component-wise cross: the intrinsic makes FXC pack the result three-wide,
            // which changes the lane order the dot() below sums in. Same issue as
            // CalculateVorticity.
            float3 cr;
            cr.x = pos2.y * pos3.z - pos2.z * pos3.y;
            cr.y = pos2.z * pos3.x - pos2.x * pos3.z;
            cr.z = pos2.x * pos3.y - pos2.y * pos3.x;
            vol = dot(pos1, cr);

        } else
            vol = 0.0;

        vol = ReduceVolume(threadIdx, numTrisInBlock, vol);

        if (threadIdx == 0)
            volumeOfInflatable += vol;
        GroupMemoryBarrierWithGroupSync();
    }

    if (threadIdx == 0) {
#if NVFLEX_XPBD
        // XPBD (Macklin, Muller, Chentanez 2016, Eq. 18) on the volume
        // constraint C = V - V0. D = sum |grad C_i|^2 at the rest pose, whose
        // reciprocal the host stores as mConstraintScale; alphaT = alpha / dt^2.
        // The multiplier accumulates in volumeLambdas across the iterations of
        // one substep and is cleared by the host per substep. SolveInflatableVolume
        // applies Dx = -lambda * n per vertex, so the step dx = n * dlam is stored
        // negated. The PBD k^3 re-inflation boost is intentionally not applied.
        float C = volumeOfInflatable - inflatableInBlock.mRestVolume;
        float D = 1.0 / inflatableInBlock.mConstraintScale;
        float alphaT = gParams.kVolumeCompliance * gParams.kInvDt * gParams.kInvDt;
        float dlam = (-C - alphaT * volumeLambdas[blockIdx]) / (D + alphaT);
        volumeLambdas[blockIdx] += dlam;
        lambdas[blockIdx] = -dlam;
#else
        // `eps` is bound first so the `mul l(0.010000)` lands right after the
        // load pair, and `k3` last so the final multiply takes it as its
        // second source. Both are operand-order only; the arithmetic is the
        // same as `abs(rest) / max(abs(rest * 0.01), abs(vol))` scaled by k^3.
        float eps = inflatableInBlock.mRestVolume * 0.01;
        float k = abs(inflatableInBlock.mRestVolume) / (max(abs(volumeOfInflatable), abs(eps)));

        float k3 = k * k * k;
        lambdas[blockIdx] = ((volumeOfInflatable - inflatableInBlock.mRestVolume) * inflatableInBlock.mConstraintScale) * k3;
#endif
    }
}
