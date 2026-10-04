#include "KernelParams.hlsli"

StructuredBuffer<float> potentials : register(t0);
StructuredBuffer<float4> sortedNewPositionsTex : register(t1);
StructuredBuffer<float4> sortedNewVelocitiesTex : register(t2);
StructuredBuffer<float> random : register(t3);
RWStructuredBuffer<int> numDiffuseParticles : register(u0);
RWStructuredBuffer<float4> diffusePositions : register(u1);
RWStructuredBuffer<float4> diffuseVelocities : register(u2);

[numthreads(256, 1, 1)]
void CreateDiffuseParticles(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float potential = potentials[idx];
        if (potential > gParams.kDiffuseThreshold) {
            int outIndex;
            InterlockedAdd(numDiffuseParticles[0], 1, outIndex);
            if (outIndex < gParams.kMaxDiffuseParticles) {
                float3 position = sortedNewPositionsTex[idx].xyz;
                float3 velocity = sortedNewVelocitiesTex[idx].xyz;
                float3 randomVec = float3(random[idx & 255],
                                          random[(idx + 1) & 255],
                                          random[(idx + 2) & 255]);

                float life = min((potential / gParams.kDiffuseThreshold) * gParams.kDiffuseLifetime,
                                 gParams.kDiffuseLifetime);
                float3 spawnPosition = position - velocity * randomVec.x * gParams.kDiffuseDt +
                                       randomVec * gParams.kFluidRestDistance * 0.25;

                diffusePositions[outIndex] = float4(spawnPosition, life);
                diffuseVelocities[outIndex] = float4(velocity, 0.0);
            }
        }
    }
}
