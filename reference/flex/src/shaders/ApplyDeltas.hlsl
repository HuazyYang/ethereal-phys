#include "KernelParams.hlsli"

ByteAddressBuffer deltas : register(t0);
RWStructuredBuffer<float4> newPositions : register(u0);

[numthreads(256, 1, 1)]
void ApplyDeltas(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float4 delta = asfloat(deltas.Load4(idx << 4));
        float3 position = newPositions[idx].xyz;

        float3 result;
        [branch] if (gParams.kRelaxationMode != 0) {
            float weight = max(delta.w * gParams.kSOR, 1.0);
            result = delta.xyz * (1.0 / weight) + position;
        } else {
            result = delta.xyz * gParams.kRelaxationFactor + position;
        }

        newPositions[idx].xyz = result;
    }
}
