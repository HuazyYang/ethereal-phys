#include "KernelParams.hlsli"

StructuredBuffer<float4> oldPositions : register(t0);
StructuredBuffer<float4> newPositions : register(t1);
RWStructuredBuffer<float4> velocities : register(u0);

[numthreads(256, 1, 1)]
void UpdateVelocities(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles)
        velocities[idx] = (newPositions[idx] - oldPositions[idx]) * gParams.kInvDt;
}
