#include "KernelParams.hlsli"

StructuredBuffer<float3> bounds : register(t0);
RWStructuredBuffer<float4> positions : register(u0);

groupshared float3 g0;
groupshared float3 g1;

[numthreads(256, 1, 1)]
void ContinuousShockPropagation(int idx: SV_GroupThreadID, int globalIdx: SV_DispatchThreadID) {
    if (idx == 0) {
        float3 b0 = bounds[0];
        g0 = b0;

        float d0 = dot(gParams.kGravity, b0);
        float3 b1 = bounds[1];
        float d1 = dot(gParams.kGravity, b1);

        if (d0 < d1)
            g0 = b1;

        float gLen2 = dot(gParams.kGravity, gParams.kGravity);
        g1 = gLen2 > 0.0 ? gParams.kGravity * rsqrt(gLen2) : 0.0.xxx;
    }
    GroupMemoryBarrierWithGroupSync();

    if (globalIdx < gParams.kNumParticles) {
        float3 gmax = g0;
        float4 pos = positions[globalIdx];
        float3 delta = -pos.xyz + gmax;
        float invM = dot(delta, g1) * gParams.kShockPropagation;
        invM = exp(invM);
        positions[globalIdx].w *= invM;
    }
}
