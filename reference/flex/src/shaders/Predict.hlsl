#include "KernelParams.hlsli"

StructuredBuffer<int> activeIndices : register(t0);
StructuredBuffer<float4> positions : register(t1);
StructuredBuffer<int> phases : register(t2);
RWStructuredBuffer<float4> newPositions : register(u0);
RWStructuredBuffer<float3> velocities : register(u1);

#define BLOCK_DIM_X     256

[numthreads(BLOCK_DIM_X, 1, 1)]
void Predict(int globalIdx: SV_DispatchThreadID) {

    if (globalIdx < gParams.kNumParticles) {
        int idx = activeIndices[globalIdx];
        float4 p0 = positions[idx];
        float3 v0 = velocities[idx];
        bool alive = p0.w > 0.0;

        int phase = phases[idx];
        bool isFluid = (phase & 0x400000) != 0;
        float3 accel = gParams.kGravity * gParams.kDt;

        float3 v2 = v0 + gParams.kGravity * gParams.kDt;
        float3 v3 = v0 + accel * gParams.kBuoyancy;
        float3 v1 = isFluid ? v3 : v2;
        v1 = alive ? v1 : v0;

        float v1Norm = dot(v1, v1);
        float v1InvNorm = rsqrt(v1Norm);
        v1 = (gParams.kMaxSpeed * gParams.kMaxSpeed) < v1Norm ? gParams.kMaxSpeed * v1InvNorm * v1 : v1;
        velocities[idx] = v1;

        float3 dp = v1 * gParams.kDt;
        float4 p1 = p0 + float4(dp, 0.0);
        newPositions[idx] = p1;
    }
}
