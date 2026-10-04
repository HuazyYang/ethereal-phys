cbuffer constBVH : register(b0) {
    int gNumItems;
}

#if USE_BLOCK_SIZE
StructuredBuffer<float4> itemLowers : register(t2);
StructuredBuffer<float4> itemUppers : register(t3);
#else
StructuredBuffer<float4> itemLowers : register(t0);
StructuredBuffer<float4> itemUppers : register(t1);
#endif

RWStructuredBuffer<float4> writeBoundsLower : register(u0);
RWStructuredBuffer<float4> writeBoundsUpper : register(u1);

#define BLOCK_DIM_X 256
#define BLOCK_HALF_DIM_X 128
#define BLOCK_DIM_X_BITS 8u

static const float maxFloat = 3.402823466e+38;

#if USE_BLOCK_SIZE
int getItemCount() {
    return int((uint(gNumItems) + BLOCK_DIM_X - 1) >> BLOCK_DIM_X_BITS);
}
#else
int getItemCount() {
    return gNumItems;
}
#endif

#if USE_NV_SHADER_EXT

#include <nvHLSLExtns.h>
#define WARP_SIZE 32
#define WARP_SIZE_BITS 5

groupshared float3 blockData[BLOCK_DIM_X / WARP_SIZE];

[numthreads(BLOCK_DIM_X, 1, 1)]
void ComputeTotalBounds(int threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
    float3 v0 = idx < getItemCount() ? itemLowers[idx].xyz : maxFloat.xxx;
    const int laneIdx = (threadIdx & (WARP_SIZE - 1));
    const int waveIdx = threadIdx >> int(WARP_SIZE_BITS);
    float3 v1, v2, v3;
    int i;

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

    v0 = idx < getItemCount() ? itemUppers[idx].xyz : -maxFloat.xxx;

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
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v3, 0.0);
    }
}

#elif USE_AMD_SHADER_EXT

#include <ags_shader_intrinsics_dx11.hlsl>

#define WAVEFRONT_SIZE 32
#define WAVEFRONT_SIZE_BITS 5

groupshared float3 blockData[BLOCK_DIM_X / WAVEFRONT_SIZE];

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
void ComputeTotalBounds(int threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
    float3 v0 = idx < getItemCount() ? itemLowers[idx].xyz : maxFloat.xxx;
    const int laneIdx = (threadIdx & (WAVEFRONT_SIZE - 1));
    const int waveIdx = threadIdx >> int(WAVEFRONT_SIZE_BITS);
    float3 v1;
    float3 v2, v3;
    int i;

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

    v0 = idx < getItemCount() ? itemUppers[idx].xyz : -maxFloat.xxx;

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
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v3, 0.0);
    }
}

#else

groupshared float3 blockData[BLOCK_DIM_X];

[numthreads(BLOCK_DIM_X, 1, 1)]
void ComputeTotalBounds(uint threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {

    float3 v0 = idx < getItemCount() ? itemLowers[idx].xyz : maxFloat.xxx;
    float3 v1;
    blockData[threadIdx] = v0;

    uint i;
    for (i = BLOCK_HALF_DIM_X; i > 0; i >>= 1) {
        GroupMemoryBarrierWithGroupSync();
        if (threadIdx < i) {
            v0 = blockData[threadIdx];
            v1 = blockData[threadIdx + i];
            blockData[threadIdx] = min(v0, v1);
        }
    }

    GroupMemoryBarrierWithGroupSync();
    float3 v2 = blockData[0];
    GroupMemoryBarrierWithGroupSync();

    v0 = idx < getItemCount() ? itemUppers[idx].xyz : -maxFloat.xxx;
    blockData[threadIdx] = v0;

    for (i = BLOCK_HALF_DIM_X; i > 0; i >>= 1) {
        GroupMemoryBarrierWithGroupSync();
        if (threadIdx < i) {
            v0 = blockData[threadIdx];
            v1 = blockData[threadIdx + i];
            blockData[threadIdx] = max(v0, v1);
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx == 0) {
        v1 = blockData[0];
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v1, 0.0);
    }
}
#endif
