#include "KernelParams.hlsli"

StructuredBuffer<uint> contacts : register(t0);
StructuredBuffer<int> contactCounts : register(t1);
StructuredBuffer<float4> collisionPlanes : register(t2);
StructuredBuffer<int> collisionCounts : register(t3);
StructuredBuffer<float4> sortedPositions : register(t4);
StructuredBuffer<float4> sortedVelocities : register(t5);
StructuredBuffer<float4> sortedNewVelocities : register(t6);
StructuredBuffer<uint> phases : register(t7);
StructuredBuffer<float4> curl : register(t8);
RWStructuredBuffer<float4> sortedNewPositions : register(u0);
RWByteAddressBuffer deltas : register(u1);
RWStructuredBuffer<float> densities : register(u2);
RWStructuredBuffer<float> potentials : register(u3);

[numthreads(256, 1, 1)]
void SolveVelocities(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float3 position = sortedNewPositions[idx].xyz;
        float3 velocity = sortedNewVelocities[idx].xyz;
        uint phase = phases[idx];
        int count = contactCounts[idx];

        bool fluid = (phase & eNvFlexPhaseFluid) != 0;
        bool applyVorticity = gParams.kVorticityConfinement > 0.0;
        bool applyDiffuse = gParams.kDiffuseThreshold > 0.0;

        uint contactIndex = idx;
        float densityPotential = 0.0;
        float diffusePotential = 0.0;
        float3 viscosityDelta = 0.0;
        float neighborCount = 0.0;
        float3 vorticityGradient = 0.0;

        [loop]
        for (int i = 0; i < count; ++i) {
            uint neighbor = contacts[contactIndex];
            contactIndex += uint(gParams.kNumParticlesAligned);

            float3 neighborPosition = sortedNewPositions[neighbor].xyz;
            float3 offset = position - neighborPosition;
            float distSq = dot(offset, offset);
            [branch] if (distSq <= gParams.kRadiusSq && distSq > 0.0) {
                uint neighborPhase = phases[neighbor];
                neighborCount += 1.0;
                bool bothFluid = fluid && ((neighborPhase & eNvFlexPhaseFluid) != 0);

                float dist = sqrt(distSq);
                float3 dir = offset / dist;
                float3 velocityDiff = sortedNewVelocities[neighbor].xyz - velocity;

                float q = 1.0 - dist * gParams.kInvRadius;
                float q2 = q * q;
                float densityWeight = gParams.kSpiky1 * q2;

                float newDensityPotential = gParams.kSpiky1 * q2 + densityPotential;
                float3 newViscosityDelta = ((velocityDiff * gParams.kViscosity) * densityWeight) * gParams.kDt + viscosityDelta;

                float3 grad = (q * -gParams.kSpiky2) * dir;
                float3 newVorticityGradient = curl[neighbor].w * grad + vorticityGradient;
                newVorticityGradient = applyVorticity ? newVorticityGradient : vorticityGradient;

                float newDiffusePotential = (length(velocityDiff) - dot(velocityDiff, dir)) * densityWeight + diffusePotential;
                newDiffusePotential = applyDiffuse ? newDiffusePotential : diffusePotential;

                viscosityDelta = bothFluid ? newViscosityDelta : viscosityDelta;
                vorticityGradient = bothFluid ? newVorticityGradient : vorticityGradient;
                // The DXBC selects both potentials in one two-component movc.
                float2 potentialPair = bothFluid ? float2(newDensityPotential, newDiffusePotential)
                                                 : float2(densityPotential, diffusePotential);
                densityPotential = potentialPair.x;
                diffusePotential = potentialPair.y;
            }
        }

        bool confineVorticity = gParams.kVorticityConfinement > 0.0 && densityPotential > 0.0;
        float3 curlSelf = curl[idx].xyz;
        float vorticityScale = gParams.kDt * gParams.kDt * gParams.kInvRestDensity * gParams.kVorticityConfinement;

        // The shipped blob re-packs the gradient into four lanes (g.y, g.z, g.z, g.x)
        // with `mov r4.xyzw, r4.yzzx` before normalising, sums the square over that
        // layout (`dp3 r5.w, r4.xzwx, ...`), and splits the cross into one four-wide
        // product read back stride-two (`mul r6.xy` + `mad r6.xy`) plus a separate
        // scalar z term, rather than one three-wide cross().
        float4 g4 = vorticityGradient.yzzx;
        float gradientSq = dot(g4.xzw, g4.xzw);
        float4 gradientDir = (gradientSq > 0.0) ? g4 * rsqrt(gradientSq) : 0.0;

        float4 gt = gradientDir * curlSelf.zyxz;
        float2 crossXY = gt.xz - gt.yw;
        float gz = gradientDir.x * curlSelf.x;
        float crossZ = gradientDir.w * curlSelf.y - gz;

        float3 delta = confineVorticity ? (vorticityScale * float3(crossXY, crossZ) + viscosityDelta)
                                        : viscosityDelta;

        bool dissipate = gParams.kDissipation > 0.0;
        float3 scaledVelocity = velocity * gParams.kDt;
        delta = dissipate ? (-(scaledVelocity * neighborCount) * gParams.kDissipation + delta) : delta;
        delta = (gParams.kDamping > 0.0) ? (-scaledVelocity * gParams.kDamping + delta) : delta;

        if (gParams.kRestitution > 0.0) {
            int planeCount = collisionCounts[idx];
            if (planeCount != 0) {
                float3 oldVelocity = sortedVelocities[idx].xyz;
                float4 restitution = 0.0;

                [loop]
                for (int planeIndex = 0; planeIndex < planeCount; ++planeIndex) {
                    uint collisionIndex = idx * uint(gParams.kMaxContactsPerParticle) + uint(planeIndex);
                    float4 plane = collisionPlanes[collisionIndex];

                    float distance = dot(position, plane.xyz) + plane.w - gParams.kCollisionDistance * 1.001;
                    float normalVelocity = dot(oldVelocity, plane.xyz);
                    bool bounce = distance <= 0.0 && normalVelocity < 0.0;

                    float reflected = gParams.kRestitution * normalVelocity + dot(velocity, plane.xyz);
                    float4 bounced = float4(-reflected * plane.xyz + restitution.xyz, restitution.w + 1.0);
                    restitution = bounce ? bounced : restitution;
                }

                delta = delta + restitution.xyz / max(restitution.w, 1.0);
            }
        }

        if (applyDiffuse) {
            float speedSq = dot(velocity, velocity) * 3.0;
            // Reconstructed verbatim from `dp2 r0.y, cb0[11].yyyy, r5.xxxx`, which sums
            // kInvRestDensity * densityPotential twice, so the scale falls off at half the
            // rest density. This looks like an authoring slip in the shipped shader -- the
            // plain form would be `1 - kInvRestDensity * densityPotential` -- but the DXBC
            // is the authority here, so keep the doubling.
            float potentialScale = max(1.0 - dot(float2(gParams.kInvRestDensity, gParams.kInvRestDensity),
                                                 float2(densityPotential, densityPotential)), 0.0);
            float potential = (potentialScale * diffusePotential) * speedSq;
            potentials[idx] = (phase & eNvFlexPhaseFluid) ? potential : 0.0;
        }

        float3 solvedVelocity = velocity + delta;
        if (dot(solvedVelocity, solvedVelocity) < gParams.kSleepThresholdSq) {
            sortedNewPositions[idx].xyz = sortedPositions[idx].xyz;
            // Reconstructed verbatim from `mov r3.xyz, -r1.xxxx`: the shipped shader
            // broadcasts -velocity.x into all three components instead of negating the
            // whole vector, so a sleeping particle is cancelled along x and given
            // -velocity.x on y and z. This looks like a swizzle slip in the original
            // (`-velocity` is what the surrounding code implies), but the DXBC is the
            // authority here, so keep the broadcast.
            delta = -velocity.xxx;
        }

        deltas.Store4(idx << 4, asuint(float4(delta, 0.0)));
        densities[idx] = densityPotential;
    }
}
