#include "KernelParams.hlsli"

StructuredBuffer<int> activeIndices : register(t0);
StructuredBuffer<float4> newPositions : register(t1);
StructuredBuffer<float3> gBounds : register(t2);
RWStructuredBuffer<int> cellBucketStarts : register(u0);
RWStructuredBuffer<int> cellBucketEnds : register(u1);

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void ClearCellBuckets(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kNumParticles) {
        int idx = activeIndices[globalIdx];
        float3 pos = newPositions[idx].xyz;
        float3 boundLower = gBounds[0];
        int3 bucketIdx = int3((pos - boundLower) * gParams.kInvCellEdge);
        int hash = (bucketIdx.x & 127) | (((bucketIdx.z & 127) << 14) | ((bucketIdx.y & 127) << 7));
        cellBucketStarts[hash] = 0;
        cellBucketEnds[hash] = 0;
    }
}
