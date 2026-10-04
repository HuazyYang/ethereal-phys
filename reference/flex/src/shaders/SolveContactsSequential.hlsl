#include "KernelParams.hlsli"

StructuredBuffer<int> collisionCounts : register(t0);
StructuredBuffer<float4> collisionPlanes : register(t1);
StructuredBuffer<float4> prevPositions : register(t2);
StructuredBuffer<float4> collisionVelocities : register(t3);
RWStructuredBuffer<float4> positions : register(u0);

float3 ContactCorrection(float3 currentPosition, float3 prevPosition, float4 plane, uint contactIndex,
                         bool adhesionEnabled, bool dynamicFrictionEnabled) {
    float separation = dot(currentPosition, plane.xyz) + plane.w;
    float penetration = separation - gParams.kCollisionDistance;
    float3 correction = adhesionEnabled ? -((separation * gParams.kAdhesion) * plane.xyz) : 0.0;

    [branch] if (penetration < 0.0) {
        float3 normalCorrection = correction - penetration * plane.xyz;
        bool frictionEnabled = dynamicFrictionEnabled && dot(plane.xyz, plane.xyz) > 0.0;

        float3 frictionCorrection = normalCorrection;
        float3 relativeDelta = currentPosition - prevPosition;
        float3 relativeMotion = relativeDelta - collisionVelocities[contactIndex].xyz;
        float3 tangent = relativeMotion - dot(relativeMotion, plane.xyz) * plane.xyz;
        float tangentSq = dot(tangent, tangent);

        float invTangentLength = rsqrt(tangentSq);
        float staticLimit = penetration * gParams.kStaticFriction;
        bool staticRegime = tangentSq < staticLimit * staticLimit;
        float3 staticCorrection = normalCorrection - tangent;
        bool sliding = tangentSq > 0.0;
        float dynamicScale = min((-penetration * gParams.kDynamicFriction) * invTangentLength, 1.0);
        float3 dynamicCorrection = normalCorrection - tangent * dynamicScale;

        frictionCorrection = sliding ? dynamicCorrection : normalCorrection;
        if (staticRegime)
            frictionCorrection = staticCorrection;

        correction = frictionEnabled ? frictionCorrection : normalCorrection;
    }

    return correction;
}

[numthreads(256, 1, 1)]
void SolveContactsSequential(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        int count = collisionCounts[idx];
        if (count != 0) {
            float4 position = positions[idx];
            if (position.w == 0.0)
                return;

            int lastContact = count - 1;
            bool adhesionEnabled = gParams.kAdhesion != 0.0;
            bool dynamicFrictionEnabled = gParams.kDynamicFriction > 0.0;

            float3 prevPosition = prevPositions[idx].xyz;
            float3 currentPosition = position.xyz;

            [loop]
            for (int contact = lastContact; contact >= 0; --contact) {
                uint contactIndex = idx * uint(gParams.kMaxContactsPerParticle) + uint(contact);
                currentPosition += ContactCorrection(currentPosition,
                                                     prevPosition,
                                                     collisionPlanes[contactIndex],
                                                     contactIndex,
                                                     adhesionEnabled,
                                                     dynamicFrictionEnabled);
            }

            positions[idx].xyz = currentPosition;
        }
    }
}
