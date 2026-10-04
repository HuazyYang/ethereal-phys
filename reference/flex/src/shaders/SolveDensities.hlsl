#include "KernelParams.hlsli"

StructuredBuffer<uint> contacts : register(t0);
StructuredBuffer<int> contactCounts : register(t1);
StructuredBuffer<float4> sortedNewPositions : register(t2);
StructuredBuffer<uint> phases : register(t3);
StructuredBuffer<float> densities : register(t4);
StructuredBuffer<float4> sortedPositions : register(t6);
StructuredBuffer<float4> sortedNormals : register(t7);
RWByteAddressBuffer deltas : register(u0);

[numthreads(256, 1, 1)]
void SolveDensities(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float4 particle = sortedNewPositions[idx];
        if (particle.w == 0.0) {
            deltas.Store4(idx << 4, uint4(0, 0, 0, 0));
            return;
        }

        float3 oldPosition = sortedPositions[idx].xyz;
        float4 normalSelf = sortedNormals[idx];
        float selfDensity = densities[idx];
        uint selfPhase = phases[idx];
        int count = contactCounts[idx];

        bool selfFluid = (selfPhase & eNvFlexPhaseFluid) != 0;
        float4 selfNormal = float4(-normalSelf.xyz, min(normalSelf.w, 0.0));
        uint contactIndex = idx;
        float4 delta = 0.0;

        [loop]
        for (int i = 0; i < count; ++i) {
            uint neighborIndex = contacts[contactIndex];
            contactIndex += uint(gParams.kNumParticlesAligned);

            float4 neighbor = sortedNewPositions[neighborIndex];
            float3 offset = particle.xyz - neighbor.xyz;
            float distSq = dot(offset, offset);
            if (distSq <= gParams.kRadiusSq && distSq > 0.0) {
                float dist = sqrt(distSq);
                float3 dir = offset * (1.0 / dist);
                uint neighborPhase = phases[neighborIndex];
                bool neighborFluid = (neighborPhase & eNvFlexPhaseFluid) != 0;

                if (selfFluid && neighborFluid) {
                    float lambda = selfDensity + densities[neighborIndex];
                    float q = dist * gParams.kInvRadius;
                    float grad = (1.0 - dist * gParams.kInvRadius) * -gParams.kSpiky2;
                    float densityTerm = lambda * grad;
                    bool differentPhase = selfPhase != neighborPhase;

                    float3 withoutCohesion = delta.xyz - (particle.w * densityTerm) * dir;

                    float q2 = q * q;
                    float cohesion = gParams.kCohesion1 * (q * q2) + gParams.kCohesion2 * q2 - 1.0;
                    float cohesiveTerm = gParams.kCohesion * cohesion + densityTerm;
                    float3 withCohesion = delta.xyz - (particle.w * cohesiveTerm) * dir;

                    delta.xyz = differentPhase ? withoutCohesion : withCohesion;
                } else {
                    if (dist > gParams.kSolidRestDistance)
                        continue;

                    float4 neighborNormal = sortedNormals[neighborIndex];
                    float massWeight = particle.w / (particle.w + neighbor.w);
                    float4 surface = (neighborNormal.w < selfNormal.w) ? neighborNormal : selfNormal;

                    float3 normal;
                    float penetration;

                    if (surface.w == 0.0) {
                        normal = dir;
                        penetration = dist - gParams.kSolidRestDistance;
                    } else if (surface.w >= -gParams.kSolidRestDistance) {
                        normal = dir;
                        penetration = dist - gParams.kSolidRestDistance;
                        float normalDot = dot(surface.xyz, dir);
                        if (normalDot < 0.0)
                            normal = dir - (normalDot + normalDot) * surface.xyz;
                    } else {
                        normal = surface.xyz;
                        penetration = surface.w;
                    }

                    float3 corrected = delta.xyz - (massWeight * penetration) * normal;

                    bool frictionEnabled = gParams.kDynamicFriction > 0.0 && dot(normal, normal) > 0.0;

                    float3 relativeMotion = offset - (oldPosition - sortedPositions[neighborIndex].xyz);
                    float3 tangent = relativeMotion - dot(relativeMotion, normal) * normal;
                    float tangentSq = dot(tangent, tangent);

                    float invTangentLength = rsqrt(tangentSq);
                    float staticLimit = penetration * gParams.kStaticFriction;
                    bool staticRegime = tangentSq < staticLimit * staticLimit;
                    float3 weightedTangent = massWeight * tangent;
                    float3 staticCorrection = corrected - massWeight * tangent;
                    float dynamicScale = min((-penetration * gParams.kParticleFriction) * invTangentLength, 1.0);
                    float3 dynamicCorrection = corrected - weightedTangent * dynamicScale;

                    float3 frictionCorrection = staticRegime ? staticCorrection : dynamicCorrection;
                    delta.xyz = frictionEnabled ? frictionCorrection : corrected;
                }

                delta.w += 1.0;
            }
        }

        deltas.Store4(idx << 4, asuint(delta));
    }
}
