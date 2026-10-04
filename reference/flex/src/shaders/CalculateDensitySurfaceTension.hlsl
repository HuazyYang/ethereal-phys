#include "KernelParams.hlsli"

StructuredBuffer<uint> contacts : register(t0);
StructuredBuffer<int> contactCounts : register(t1);
StructuredBuffer<float4> sortedNewPositions : register(t2);
StructuredBuffer<uint> phases : register(t3);
Buffer<uint> indices : register(t4);
RWStructuredBuffer<float> densities : register(u0);
RWStructuredBuffer<float4> normals : register(u1);
RWStructuredBuffer<float4> sortedNormals : register(u2);

[numthreads(256, 1, 1)]
void CalculateDensitySurfaceTension(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
    float3 position = sortedNewPositions[idx].xyz;
    int contactCount = contactCounts[idx];
    uint contactIndex = idx;
    float density = 0.0;
    float3 normal = 0.0;

    [loop]
    for (int i = 0; i < contactCount; ++i) {
        uint neighbor = contacts[contactIndex];
        contactIndex += uint(gParams.kNumParticlesAligned);

        float3 delta = position - sortedNewPositions[neighbor].xyz;
        float distSq = dot(delta, delta);
        if (distSq < gParams.kRadiusSq && distSq > 0.0) {
            float dist = sqrt(distSq);
            float phaseWeight = (phases[neighbor] & eNvFlexPhaseFluid) ? 1.0 : gParams.kSolidPressure;
            float q = 1.0 - dist * gParams.kInvRadius;
            density += phaseWeight * (q * q * gParams.kSpiky1);
            float scale = q * -gParams.kSpiky2;
            normal += (scale * delta) / dist;
        }
    }

    densities[idx] = max(density - gParams.kRestDensity, 0.0) * gParams.kLambdaScale;

    if (phases[idx] & eNvFlexPhaseFluid) {
        float4 outNormal = float4(normal * gParams.kSurfaceTension, 0.0);
        normals[indices[idx]] = outNormal;
        sortedNormals[idx] = outNormal;
    }
    }
}
