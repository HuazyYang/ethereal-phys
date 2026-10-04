#include "KernelParams.hlsli"

StructuredBuffer<float4> positions : register(t0);
StructuredBuffer<float4> newPositions : register(t1);
StructuredBuffer<float3> velocities : register(t2);
StructuredBuffer<int> phases : register(t3);
Buffer<uint> indices : register(t4);
StructuredBuffer<float4> normals : register(t5);
RWStructuredBuffer<float4> sortedPositions : register(u0);
RWStructuredBuffer<float4> sortedNewPositions : register(u1);
RWStructuredBuffer<float4> sortedVelocities : register(u2);
RWStructuredBuffer<int> sortedPhases : register(u3);
RWStructuredBuffer<int> reverseLookup : register(u4);
RWStructuredBuffer<float4> sortedNormals : register(u5);

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void ReorderParticles(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kNumParticles) {
        uint idx = indices[globalIdx];
        sortedPositions[globalIdx] = positions[idx];
        sortedNewPositions[globalIdx] = newPositions[idx];
        sortedVelocities[globalIdx].xyz = velocities[idx];
        sortedPhases[globalIdx] = phases[idx];
        sortedNormals[globalIdx] = normals[idx];
        reverseLookup[idx] = globalIdx;
    }
}
