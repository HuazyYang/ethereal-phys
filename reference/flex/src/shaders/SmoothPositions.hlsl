#include "KernelParams.hlsli"

StructuredBuffer<uint> contacts : register(t0);
StructuredBuffer<int> contactCounts : register(t1);
StructuredBuffer<uint> phases : register(t2);
StructuredBuffer<float4> sortedNewPositionsTex : register(t3);
Buffer<uint> indices : register(t4);
RWStructuredBuffer<float4> smoothPositions : register(u0);
RWStructuredBuffer<float4> smoothPositionsOriginal : register(u1);

[numthreads(256, 1, 1)]
void SmoothPositions(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float4 original = sortedNewPositionsTex[idx];
        uint originalIndex = indices[idx];
        // Declared ahead of `result`: the declaration order decides which of the two
        // gets the mad destination and which gets the copy.
        float4 sp;
        float4 result = original;

        if (phases[idx] & eNvFlexPhaseFluid) {
            int contactCount = contactCounts[idx];
            uint contactIndex = idx;
            float weightSum = 0.0;
            float3 positionSum = 0.0;

            [loop]
            for (int i = 0; i < contactCount; ++i) {
                uint neighbor = contacts[contactIndex];
                contactIndex += uint(gParams.kNumParticlesAligned);

                float3 neighborPos = sortedNewPositionsTex[neighbor].xyz;
                float3 delta = original.xyz - neighborPos;
                float distSq = dot(delta, delta);
                if (distSq < gParams.kRadiusSq && distSq > 0.0) {
                    float q = sqrt(distSq) * gParams.kInvRadius;
                    float weight = 1.0 - q * q * q;
                    positionSum += neighborPos * weight;
                    weightSum += weight;
                }
            }

            float t = gParams.kRadius * gParams.kInvRadius * 0.5;
            float normalization = (1.0 - t * t * t) * 4.0;

            float scaledWeight = min(weightSum / normalization, 1.0);
            bool hasWeight = weightSum > 0.0;
            float3 weightedAverage = positionSum / weightSum;

            float4 smoothing = hasWeight ? float4(weightedAverage, scaledWeight)
                                         : float4(positionSum, 0.0);
            float3 average = smoothing.xyz;
            float amount = smoothing.w;

            float scale = amount * gParams.kSmoothing;
            result.xyz = original.xyz + (average - original.xyz) * scale;
            sp.xyz = result.xyz;
        } else {
            sp.xyz = original.xyz;
        }

        smoothPositionsOriginal[originalIndex] = result;
        sp.w = original.w;
        smoothPositions[idx] = sp;
    }
}
