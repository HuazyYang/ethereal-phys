// ethereal-phys kernel source. Origin: FleX src/shaders/CalculateBounds.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "WaveReduce.hlsli"
#include "KernelParams.hlsli"

#ifndef REDUCE_LEVEL
#define REDUCE_LEVEL 0
#endif

#if REDUCE_LEVEL == 0
StructuredBuffer<int> activeIndices : register(t0);
StructuredBuffer<float4> positions : register(t1);
#elif (REDUCE_LEVEL == 2) || (REDUCE_LEVEL == 1)
StructuredBuffer<float4> readBoundsLower : register(t2);
StructuredBuffer<float4> readBoundsUpper : register(t3);
#endif

#if (REDUCE_LEVEL == 0) || (REDUCE_LEVEL == 1)
RWStructuredBuffer<float4> writeBoundsLower : register(u0);
RWStructuredBuffer<float4> writeBoundsUpper : register(u1);
#elif REDUCE_LEVEL == 2
RWStructuredBuffer<float3> bounds : register(u2);
#endif

static const float maxFloat = 3.402823466e+38;

#define BLOCK_DIM_X      128
#define BLOCK_HALF_DIM_X 64
#define BLOCK_DIM_X_BITS 7

float3 getLower(int idx) {
#if REDUCE_LEVEL == 0
    return idx < gParams.kNumParticles ? positions[activeIndices[idx]].xyz : maxFloat.xxx;
#elif REDUCE_LEVEL == 1
    return (idx < int((uint(gParams.kNumParticles) + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS)) ? readBoundsLower[idx].xyz : maxFloat.xxx;
#else
    uint numBlocks1 = (uint(gParams.kNumParticles) + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS;
    uint numBlocks2 = (numBlocks1 + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS;
    return idx < int(numBlocks2) ? readBoundsLower[idx].xyz : maxFloat.xxx;
#endif
}

float3 getUpper(int idx) {
#if REDUCE_LEVEL == 0
    return idx < gParams.kNumParticles ? positions[activeIndices[idx]].xyz : -maxFloat.xxx;
#elif REDUCE_LEVEL == 1
    return (idx < int((uint(gParams.kNumParticles) + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS)) ? readBoundsUpper[idx].xyz : -maxFloat.xxx;
#else
    uint numBlocks1 = (uint(gParams.kNumParticles) + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS;
    uint numBlocks2 = (numBlocks1 + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS;
    return idx < int(numBlocks2) ? readBoundsUpper[idx].xyz : -maxFloat.xxx;
#endif
}

#if EPHYS_USE_WAVE

// The wave permutation (see WaveReduce.hlsli). It replaces FleX's CalculateBounds*NV (NVAPI NvShflDown)
// and CalculateBounds*AMD (AGS swizzle) kernels; min and max are exact, so it writes the same bounds as
// the groupshared tree below.
EPHYS_DEFINE_WAVE_GROUP_REDUCE(ReduceLower, float3, BLOCK_DIM_X, WaveActiveMin, min, maxFloat.xxx)
EPHYS_DEFINE_WAVE_GROUP_REDUCE(ReduceUpper, float3, BLOCK_DIM_X, WaveActiveMax, max, (-maxFloat).xxx)

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateBounds(uint threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
#if REDUCE_LEVEL == 2
    blockIdx = 0;
#endif

    float3 v3 = ReduceLower(threadIdx, getLower(idx));
    float3 v4 = ReduceUpper(threadIdx, getUpper(idx));

    if (threadIdx == 0) {
#if REDUCE_LEVEL == 0 || REDUCE_LEVEL == 1
        writeBoundsLower[blockIdx] = float4(v3, 0.0);
        writeBoundsUpper[blockIdx] = float4(v4, 0.0);
#else
        v3 -= gParams.kRadius;
        v4 += gParams.kRadius;
        bounds[0] = v3;
        bounds[1] = v4;
#endif
    }
}

#else

groupshared float3 blockData[BLOCK_DIM_X];

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateBounds(uint threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
    uint i;
    float3 v1, v2, v3, v4;

#if REDUCE_LEVEL == 2
    blockIdx = 0;
#endif
    
    blockData[threadIdx] = getLower(idx);

    for (i = BLOCK_HALF_DIM_X; i > 0; i >>= 1u) {
        GroupMemoryBarrierWithGroupSync();
        if (threadIdx < i) {
            v1 = blockData[threadIdx];
            v2 = blockData[threadIdx + i];
            blockData[threadIdx] = min(v1, v2);
        }
    }

    GroupMemoryBarrierWithGroupSync();
    v3 = blockData[0];
    GroupMemoryBarrierWithGroupSync();

    blockData[threadIdx] = getUpper(idx);

    for (i = BLOCK_HALF_DIM_X; i > 0; i >>= 1u) {
        GroupMemoryBarrierWithGroupSync();
        if (threadIdx < i) {
            v1 = blockData[threadIdx];
            v2 = blockData[threadIdx + i];
            blockData[threadIdx] = max(v1, v2);
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
#if REDUCE_LEVEL == 0 || REDUCE_LEVEL == 1
        v4 = blockData[0];
        writeBoundsLower[blockIdx] = float4(v3, 0.0);
        writeBoundsUpper[blockIdx] = float4(v4, 0.0);
#else
        v4 = blockData[0];
        v3 -= gParams.kRadius;
        v4 += gParams.kRadius;
        bounds[0] = v3;
        bounds[1] = v4;
#endif
    }
}

#endif
