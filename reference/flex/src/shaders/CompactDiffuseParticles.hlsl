
StructuredBuffer<int> numDiffuseParticles : register(t0);
StructuredBuffer<float4> diffusePositions : register(t1);
StructuredBuffer<float4> diffuseVelocities : register(t2);
RWStructuredBuffer<int> numDiffuseParticlesNew : register(u0);
RWStructuredBuffer<float4> diffusePositionsNew : register(u1);
RWStructuredBuffer<float4> diffuseVelocitiesNew : register(u2);

[numthreads(256, 1, 1)]
void CompactDiffuseParticles(int particle: SV_DispatchThreadID) {

    if (particle < numDiffuseParticles[0]) {
        float4 pos = diffusePositions[particle];
        if (pos.w > 0.0) {
            int nextIdx;
            InterlockedAdd(numDiffuseParticlesNew[0], 1, nextIdx);
            diffusePositionsNew[nextIdx] = diffusePositions[particle];
            diffuseVelocitiesNew[nextIdx] = diffuseVelocities[particle];
        }
    }
}
