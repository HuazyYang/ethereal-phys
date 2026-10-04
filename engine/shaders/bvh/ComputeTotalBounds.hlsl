// ethereal-phys kernel source. Origin: FleX src/shaders/bvh/ComputeTotalBounds.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "../WaveReduce.hlsli"
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

#if EPHYS_USE_WAVE

// The wave permutation (see ../WaveReduce.hlsli). It replaces FleX's ComputeTotalBounds*NV (NVAPI
// NvShflDown) and ComputeTotalBounds*AMD (AGS swizzle) kernels; min and max are exact, so it writes the
// same bounds as the groupshared tree below.
EPHYS_DEFINE_WAVE_GROUP_REDUCE(ReduceLower, float3, BLOCK_DIM_X, WaveActiveMin, min, maxFloat.xxx)
EPHYS_DEFINE_WAVE_GROUP_REDUCE(ReduceUpper, float3, BLOCK_DIM_X, WaveActiveMax, max, (-maxFloat).xxx)

[numthreads(BLOCK_DIM_X, 1, 1)]
void ComputeTotalBounds(uint threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID, int idx: SV_DispatchThreadID) {
    float3 v0 = idx < getItemCount() ? itemLowers[idx].xyz : maxFloat.xxx;
    float3 v2 = ReduceLower(threadIdx, v0);

    v0 = idx < getItemCount() ? itemUppers[idx].xyz : -maxFloat.xxx;
    float3 v1 = ReduceUpper(threadIdx, v0);

    if (threadIdx == 0) {
        writeBoundsLower[blockIdx] = float4(v2, 0.0);
        writeBoundsUpper[blockIdx] = float4(v1, 0.0);
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
