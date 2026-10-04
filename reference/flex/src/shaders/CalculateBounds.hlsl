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

#if USE_NV_SHADER_EXT

#include <nvHLSLExtns.h>
#define WARP_SIZE      32
#define WARP_SIZE_BITS 5

groupshared float3 blockData[BLOCK_DIM_X / WARP_SIZE];

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateBounds(int threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {

    float3 v0 = getLower(idx);
    const int laneIdx = (threadIdx & (WARP_SIZE - 1));
    const int waveIdx = threadIdx >> int(WARP_SIZE_BITS);
    float3 v1, v2, v3;
    uint i;

#if REDUCE_LEVEL == 2
    blockIdx = 0;
#endif

    [unroll]
    for (i = 1; i < WARP_SIZE; i <<= 1) {
        v1.x = asfloat(NvShflDown(asint(v0.x), i));
        v1.y = asfloat(NvShflDown(asint(v0.y), i));
        v1.z = asfloat(NvShflDown(asint(v0.z), i));
        v0 = min(v0, v1);
    }

    if (laneIdx == 0)
        blockData[waveIdx] = v0;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        v0 = blockData[0];
        [unroll]
        for (i = 1; i < (BLOCK_DIM_X / WARP_SIZE); ++i) {
            v1 = blockData[i];
            v0 = min(v0, v1);
        }
        blockData[0] = v0;
    }

    GroupMemoryBarrierWithGroupSync();
    v2 = blockData[0];
    GroupMemoryBarrierWithGroupSync();

    v0 = getUpper(idx);

    [unroll]
    for (i = 1; i < WARP_SIZE; i <<= 1) {
        v1.x = asfloat(NvShflDown(asint(v0.x), i));
        v1.y = asfloat(NvShflDown(asint(v0.y), i));
        v1.z = asfloat(NvShflDown(asint(v0.z), i));
        v0 = max(v0, v1);
    }

    if (laneIdx == 0)
        blockData[waveIdx] = v0;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        v0 = blockData[0];
        [unroll]
        for (i = 1; i < (BLOCK_DIM_X / WARP_SIZE); ++i) {
            v1 = blockData[i];
            v0 = max(v0, v1);
        }
        blockData[0] = v0;
    }

    GroupMemoryBarrierWithGroupSync();
    v3 = blockData[0];

    if (threadIdx == 0) {
#if REDUCE_LEVEL == 0 || REDUCE_LEVEL == 1
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v3, 0.0);
#else
        v2 -= gParams.kRadius;
        v3 += gParams.kRadius;
        bounds[0] = v2;
        bounds[1] = v3;
#endif
    }
}

#elif USE_AMD_SHADER_EXT

#include <ags_shader_intrinsics_dx11.hlsl>

#define WAVEFRONT_SIZE      32
#define WAVEFRONT_SIZE_BITS 5

// The shipped AMD variants allocate a full per-thread array (dcl_tgsm_structured
// g0, 12, 128) even though only BLOCK_DIM_X / WAVEFRONT_SIZE entries are used.
groupshared float3 blockData[BLOCK_DIM_X];

#if WAVEFRONT_SIZE != 32
#error AMD wavefront size must be 32 by configuration
#endif

static const int g_AmdDxExtSwizzleOperations[] = {
    AmdDxExtShaderIntrinsicsSwizzle_SwapX1,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX2,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX4,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX8,
    AmdDxExtShaderIntrinsicsSwizzle_SwapX16,
};

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateBounds(int threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
    float3 v0 = getLower(idx);
    const int laneIdx = (threadIdx & (WAVEFRONT_SIZE - 1));
    const int waveIdx = threadIdx >> int(WAVEFRONT_SIZE_BITS);
    float3 v1;
    float3 v2, v3;
    int i;

#if REDUCE_LEVEL == 2
    blockIdx = 0;
#endif

    [unroll]
    for (i = WAVEFRONT_SIZE_BITS - 1; i >= 0; --i) {
        v1.x = AmdDxExtShaderIntrinsics_SwizzleF(v0.x, g_AmdDxExtSwizzleOperations[i]);
        v1.y = AmdDxExtShaderIntrinsics_SwizzleF(v0.y, g_AmdDxExtSwizzleOperations[i]);
        v1.z = AmdDxExtShaderIntrinsics_SwizzleF(v0.z, g_AmdDxExtSwizzleOperations[i]);
        v0 = min(v0, v1);
    }

    if (laneIdx == 0)
        blockData[waveIdx] = v0;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        v0 = blockData[0];
        [unroll]
        for (i = 1; i < (BLOCK_DIM_X / WAVEFRONT_SIZE); ++i) {
            v1 = blockData[i];
            v0 = min(v0, v1);
        }
        blockData[0] = v0;
    }

    GroupMemoryBarrierWithGroupSync();
    v2 = blockData[0];
    GroupMemoryBarrierWithGroupSync();

    v0 = getUpper(idx);

    [unroll]
    for (i = WAVEFRONT_SIZE_BITS - 1; i >= 0; --i) {
        v1.x = AmdDxExtShaderIntrinsics_SwizzleF(v0.x, g_AmdDxExtSwizzleOperations[i]);
        v1.y = AmdDxExtShaderIntrinsics_SwizzleF(v0.y, g_AmdDxExtSwizzleOperations[i]);
        v1.z = AmdDxExtShaderIntrinsics_SwizzleF(v0.z, g_AmdDxExtSwizzleOperations[i]);
        v0 = max(v0, v1);
    }

    if (laneIdx == 0)
        blockData[waveIdx] = v0;
    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        v0 = blockData[0];
        [unroll]
        for (i = 1; i < (BLOCK_DIM_X / WAVEFRONT_SIZE); ++i) {
            v1 = blockData[i];
            v0 = max(v0, v1);
        }
        blockData[0] = v0;
    }

    GroupMemoryBarrierWithGroupSync();
    v3 = blockData[0];

    if (threadIdx == 0) {
#if REDUCE_LEVEL == 0 || REDUCE_LEVEL == 1
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v3, 0.0);
#else
        v2 -= gParams.kRadius;
        v3 += gParams.kRadius;
        bounds[0] = v2;
        bounds[1] = v3;
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
