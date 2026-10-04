#include "KernelParams.hlsli"

StructuredBuffer<int> collisionCounts : register(t0);
StructuredBuffer<float4> collisionPlanes : register(t1);
StructuredBuffer<float4> prevPositions : register(t2);
StructuredBuffer<float4> collisionVelocities : register(t3);
RWStructuredBuffer<float4> positions : register(u0);

[numthreads(256, 1, 1)]
void SolveContactsAveraged(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        int count = collisionCounts[idx];
        if (count != 0) {
            float4 position = positions[idx];
            if (position.w == 0.0)
                return;

            int lastContact = count - 1;
            bool adhesionEnabled = gParams.kAdhesion != 0.0;
            bool dynamicFrictionEnabled = gParams.kDynamicFriction > 0.0;

            float3 relativePosition = -prevPositions[idx].xyz + position.xyz;
            float3 correctionSum = 0.0;
            float penetrationCount = 0.0;

            [loop]
            for (int contact = lastContact; contact >= 0; --contact) {
                uint contactIndex = idx * uint(gParams.kMaxContactsPerParticle) + uint(contact);
                float4 plane = collisionPlanes[contactIndex];

                float separation = dot(position.xyz, plane.xyz) + plane.w;
                float penetration = separation - gParams.kCollisionDistance;

                if (adhesionEnabled)
                    correctionSum = correctionSum - (separation * gParams.kAdhesion) * plane.xyz;

                [branch] if (penetration < 0.0) {
                    float3 normalCorrection = correctionSum - penetration * plane.xyz;
                    penetrationCount += 1.0;

                    bool frictionEnabled = dot(plane.xyz, plane.xyz) > 0.0 && dynamicFrictionEnabled;

                    float3 relativeMotion = relativePosition - collisionVelocities[contactIndex].xyz;
                    float3 tangent = relativeMotion - dot(relativeMotion, plane.xyz) * plane.xyz;
                    float tangentSq = dot(tangent, tangent);

                    float invTangentLength = rsqrt(tangentSq);
                    float staticLimit = penetration * gParams.kStaticFriction;
                    bool staticRegime = tangentSq < staticLimit * staticLimit;
                    float3 staticCorrection = normalCorrection - tangent;
                    bool sliding = tangentSq > 0.0;
                    float dynamicScale = min((-penetration * gParams.kDynamicFriction) * invTangentLength, 1.0);
                    float3 dynamicCorrection = normalCorrection - tangent * dynamicScale;

                    float3 frictionCorrection = sliding ? dynamicCorrection : normalCorrection;
                    if (staticRegime)
                        frictionCorrection = staticCorrection;

                    correctionSum = frictionEnabled ? frictionCorrection : normalCorrection;
                }
            }

            if (penetrationCount > 0.0)
                positions[idx].xyz = correctionSum * (1.0 / penetrationCount) + position.xyz;
        }
    }
}
