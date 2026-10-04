#include "KernelParams.hlsli"

RWStructuredBuffer<int> numDiffuseParticles : register(u0);
RWStructuredBuffer<int> numDiffuseParticlesNew : register(u1);

[numthreads(1, 1, 1)]
void ClampDiffuseParticleCount() {
    numDiffuseParticles[0] = (int)min((float)numDiffuseParticles[0], (float)gParams.kMaxDiffuseParticles);
    numDiffuseParticlesNew[0] = 0;
}
