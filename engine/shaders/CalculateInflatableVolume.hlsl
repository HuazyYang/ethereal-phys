// ethereal-phys kernel source. Origin: FleX src/shaders/CalculateInflatableVolume.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "WaveReduce.hlsli"
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

groupshared FlexInflatable inflatableInBlock;
groupshared float volumeOfInflatable;
groupshared float3 centerOfInflatable;

#if EPHYS_USE_WAVE

// The wave permutation (see WaveReduce.hlsli). It replaces FleX's CalculateInflatableVolume*NV (NVAPI
// NvShflDown) and *AMD (AGS swizzle) kernels.
EPHYS_DEFINE_WAVE_GROUP_REDUCE(WaveReduceCenter, float3, BLOCK_DIM_X, WaveActiveSum, EPHYS_WAVE_ADD, 0.0.xxx)
EPHYS_DEFINE_WAVE_GROUP_REDUCE(WaveReduceVolume, float, BLOCK_DIM_X, WaveActiveSum, EPHYS_WAVE_ADD, 0.0)

// True for the thread indices whose value the groupshared tree below (EPHYS_WAVE=0) folds into slot 0.
// That tree halves numTrisInBlock, so for a count that is not a power of two it leaves some lanes out
// (count 6 -> strides 3, 1 -> lanes 0, 1, 3, 4). That is what the shipped FleX kernel computes, and the
// wave permutation sums exactly the same lanes, so that the two permutations differ in summation order
// only. (FleX's vendor kernels summed yet another subset: the first numTrisInBlock / 32 waves.) The
// strides are super-increasing, so the greedy test is exact.
bool TreeReduceCovers(uint lane, uint numTrisInBlock) {
    for (uint j = numTrisInBlock >> 1; j > 0; j >>= 1) {
        if (lane >= j)
            lane -= j;
    }
    return lane == 0;
}

float3 ReduceCenter(int threadIdx, int numTrisInBlock, float3 center) {
    if (!TreeReduceCovers(uint(threadIdx), uint(numTrisInBlock)))
        center = 0.0.xxx;
    return WaveReduceCenter(uint(threadIdx), center);
}

float ReduceVolume(int threadIdx, int numTrisInBlock, float vol) {
    if (!TreeReduceCovers(uint(threadIdx), uint(numTrisInBlock)))
        vol = 0.0;
    return WaveReduceVolume(uint(threadIdx), vol);
}

#else

groupshared float volumesInBlock[BLOCK_DIM_X];
groupshared float3 centersInBlock[BLOCK_DIM_X];

// Sums `center` across the thread group with a shared-memory tree over the live lanes.
float3 ReduceCenter(int threadIdx, int numTrisInBlock, float3 center) {
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
}

// Same reduction for the scalar volume term.
float ReduceVolume(int threadIdx, int numTrisInBlock, float vol) {
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
}

#endif

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
        float alphaT = gParamsXPBD.kVolumeCompliance * gParams.kInvDt * gParams.kInvDt;
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
