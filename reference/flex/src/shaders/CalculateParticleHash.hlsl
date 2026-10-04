#include "KernelParams.hlsli"

StructuredBuffer<int> activeIndices : register(t0);
StructuredBuffer<float4> newPositions : register(t1);
StructuredBuffer<float3> gBounds : register(t2);

RWBuffer<uint4> cellHash : register(u0);
RWBuffer<uint4> cellIndices : register(u1);

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateParticleHash(int globalIdx: SV_DispatchThreadID) {
    uint index = -1;
    uint hash = -1;
    [branch]
    if (globalIdx < gParams.kNumParticles) {
        index = (uint)activeIndices[globalIdx];
        float3 pos = newPositions[index].xyz;
        float3 boundLower = gBounds[0];
        pos -= boundLower;
        int3 bucket = int3(pos * gParams.kInvCellEdge);
        hash = (bucket.x & 127) | (((bucket.z & 127) << 14) | ((bucket.y & 127) << 7));
    }

    cellIndices[globalIdx] = index.xxxx;
    cellHash[globalIdx] = hash.xxxx;
}
