// XPBD springs. Macklin, Muller, Chentanez, "XPBD: Position-Based Simulation
// of Compliant Constrained Dynamics", MIG 2016. Each half-spring applies the
// compliant update of Eq. 18 with the Rayleigh damping term of Eq. 26:
//
//   alphaT = alpha / dt^2,  gamma = alphaT * beta * dt  (= alpha * beta / dt)
//   dlam   = (-C - alphaT * lambda - gamma * n.(x - x_n))
//          / ((1 + gamma) * wsum + alphaT)
//   lambda += dlam,  Dx += dlam * n
//
// alpha is derived from the PBD stiffness k as recommended in "XPBD slides and
// stiffness" (blog.mmacklin.com, 2016-10-12): k in [0,1] is mapped onto the
// stiffness range [stiffnessMin, stiffnessMax] (geometrically, since material
// stiffnesses span orders of magnitude) and alpha is the reciprocal:
//   alpha = 1 / (stiffnessMin * (stiffnessMax / stiffnessMin)^k)
//         = kInvStiffnessMin * exp2(-k * kLogStiffnessRange).
// k <= 0 disables the spring. A negative stiffness marks a tether (unilateral,
// acts only when stretched). lambda lives per half-spring in halfSpringLambdas,
// persists across the iterations of one substep and is cleared by the host at
// the start of each substep; both halves of a spring compute the same dlam from
// symmetric inputs, so no synchronisation between them is needed.
//
// Same 512-thread layout as SolveSprings.hlsl: eight lanes gather one
// particle's half-springs, lane 0 reduces them and adds Dx * w plus the count
// into `deltas`, so ApplyDeltas and the count-based relaxation are unchanged.
//
// Known limitations: ApplyDeltas divides the summed Jacobi delta by the
// per-particle count (local relaxation) while lambda accumulates the full dlam,
// so the multiplier bookkeeping is approximate, and iteration independence only
// holds once the Jacobi solver has converged (about 20 iterations per substep
// for cloth; one for an isolated spring). Damping is coupled to compliance
// (gamma = alpha * beta / dt), so k = 1 springs receive no damping; this is
// inherent to Eq. 26.
#define NVFLEX_XPBD 1
#include "KernelParams.hlsli"

StructuredBuffer<int> halfSpringOpposites : register(t1);
StructuredBuffer<float> halfSpringLengths : register(t2);
StructuredBuffer<float> halfSpringStiffness : register(t3);
StructuredBuffer<int> particleSpringBegin : register(t4);
StructuredBuffer<int> particleSpringEnd : register(t5);
StructuredBuffer<int> reverseLookup : register(t6);
StructuredBuffer<float4> sortedNewPositionsTex : register(t7);
StructuredBuffer<float4> sortedPositions : register(t8);   // x_n: positions at the start of the substep
RWByteAddressBuffer deltas : register(u0);
RWStructuredBuffer<float> halfSpringLambdas : register(u1);

#define BLOCK_DIM_X 512

groupshared float3 dispInBlocks[BLOCK_DIM_X];

[numthreads(BLOCK_DIM_X, 1, 1)]
void SolveSpringsXPBD(int threadIdx: SV_GroupThreadID, int dispatchIdx: SV_DispatchThreadID) {

    uint springIdxBase = uint(dispatchIdx) / 8;
    uint springIdxOffset = uint(dispatchIdx) & 7;
    bool outOfBound = springIdxBase >= uint(gParams.kMaxParticles);
    springIdxBase = outOfBound ? 0 : springIdxBase;

    uint idxStart = particleSpringBegin[springIdxBase];
    uint idxEnd = particleSpringEnd[springIdxBase];
    uint idxRange = outOfBound ? 0 : idxEnd - idxStart;
    // The delta slot is indexed by the reverse-lookup result, not by
    // springIdxBase (see SolveSprings.hlsl).
    uint sortedIdx = reverseLookup[springIdxBase];
    float4 pos0 = sortedNewPositionsTex[sortedIdx];
    float3 x0n = sortedPositions[sortedIdx].xyz;
    float3 Dx = 0.0.xxx;

    float invDt2 = gParams.kInvDt * gParams.kInvDt;

    if (pos0.w > 0.0 && idxRange > 0) {
        for (uint springIdx = idxStart + springIdxOffset; springIdx < idxEnd; springIdx += 8) {
            uint sortedIdx1 = reverseLookup[halfSpringOpposites[springIdx]];
            float4 pos1 = sortedNewPositionsTex[sortedIdx1];
            float lambda = halfSpringLambdas[springIdx];

            float stiffness = halfSpringStiffness[springIdx];
            bool tether = stiffness < 0.0;
            float k = min(abs(stiffness), 1.0);
            float wsum = pos0.w + pos1.w;
            float3 d = pos0.xyz - pos1.xyz;
            float len = length(d);

            // Disabled spring, two pinned ends, or coincident ends: no delta and
            // no multiplier update.
            if (k <= 0.0 || wsum <= 0.0 || len <= 0.0)
                continue;

            float C = len - halfSpringLengths[springIdx];
            [branch]
            if (tether) {
                // Unilateral constraint. In this sign convention a stretched
                // spring yields dlam < 0, so the accumulated multiplier of a
                // tether is its inward pull and is never positive. A slack
                // tether with no accumulated pull contributes neither an
                // elastic nor a damping term.
                if (C <= 0.0 && lambda >= 0.0)
                    continue;
                C = max(C, 0.0);
            }

            float alphaT = gParams.kInvStiffnessMin * exp2(-k * gParams.kLogStiffnessRange) * invDt2;
            float gamma = alphaT * gParams.kSpringDamping * gParams.kDt;

            float3 n = d / len;
            float3 x1n = sortedPositions[sortedIdx1].xyz;
            float3 dv = (pos0.xyz - x0n) - (pos1.xyz - x1n);

            float dlam = (-C - alphaT * lambda - gamma * dot(n, dv)) / ((1.0 + gamma) * wsum + alphaT);
            float lambdaNew = lambda + dlam;
            // A tether never pushes: clamp its total multiplier to the pulling
            // side and apply only the clamped increment.
            lambdaNew = tether ? min(lambdaNew, 0.0) : lambdaNew;
            dlam = lambdaNew - lambda;
            halfSpringLambdas[springIdx] = lambdaNew;
            Dx += dlam * n;
        }
    }

    dispInBlocks[threadIdx] = Dx;
    GroupMemoryBarrierWithGroupSync();

    if (springIdxOffset == 0 && idxRange > 0 && pos0.w > 0.0) {
        Dx += dispInBlocks[threadIdx + 1];
        Dx += dispInBlocks[threadIdx + 2];
        Dx += dispInBlocks[threadIdx + 3];
        Dx += dispInBlocks[threadIdx + 4];
        Dx += dispInBlocks[threadIdx + 5];
        Dx += dispInBlocks[threadIdx + 6];
        Dx += dispInBlocks[threadIdx + 7];

        float4 DxTotal = asfloat(deltas.Load4(sortedIdx * 16));
        DxTotal += float4(Dx * pos0.w, float(idxRange));
        deltas.Store4(sortedIdx * 16, asuint(DxTotal));
    }
}
