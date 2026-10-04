#include "KernelParams.hlsli"

StructuredBuffer<float4> sortedNewPositions : register(t0);
StructuredBuffer<float4> sortedNewVelocities : register(t1);
StructuredBuffer<float> sortedDensities : register(t2);
ByteAddressBuffer impulses : register(t3);
Buffer<uint> indices : register(t4);
RWStructuredBuffer<float4> positions : register(u0);
RWStructuredBuffer<float3> velocities : register(u1);
RWStructuredBuffer<float> densities : register(u2);

[numthreads(256, 1, 1)]
void Finalize(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float4 newPosition = sortedNewPositions[idx];
        if (newPosition.w > 0.0) {
            uint originalIndex = indices[idx];
            positions[originalIndex].xyz = newPosition.xyz;

            float4 impulse = asfloat(impulses.Load4(idx << 4));
            float3 oldVelocity = velocities[originalIndex];
            float3 sortedVelocity = sortedNewVelocities[idx].xyz;
            float3 velocityDelta = sortedVelocity + impulse.xyz / max(impulse.w, 1.0) - oldVelocity;
            float velocityDeltaSq = dot(velocityDelta, velocityDelta);
            float invLength = rsqrt(velocityDeltaSq);
            float maxVelocityDeltaSq = gParams.kMaxVelocityDelta * gParams.kMaxVelocityDelta;

            if (velocityDeltaSq > maxVelocityDeltaSq)
                velocityDelta = velocityDelta * (invLength * gParams.kMaxVelocityDelta);

            velocities[originalIndex] = oldVelocity + velocityDelta;
            densities[originalIndex] = sortedDensities[idx] * gParams.kInvRestDensity;
        }
    }
}
