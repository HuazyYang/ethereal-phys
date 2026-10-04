#include "KernelParams.hlsli"

StructuredBuffer<float4> sortedNewPositions : register(t0);
StructuredBuffer<float4> sortedNewVelocities : register(t1);
StructuredBuffer<int> sortedPhases : register(t2);
StructuredBuffer<int> contacts : register(t3);
StructuredBuffer<int> contactCounts : register(t4);
RWStructuredBuffer<float4> curl : register(u0);

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void CalculateVorticity(int globalIdx: SV_DispatchThreadID) {

    if (globalIdx < gParams.kNumParticles) {

        uint phase = (uint)sortedPhases[globalIdx];
        bool isFluid = (phase & 0x400000u) != 0;
        // Spelled `== false` rather than `!`: `!` lets FXC fold the mask test into
        // the branch, while the explicit compare keeps the shipped `ine r, r, l(0)`.
        if (isFluid == false) {
            curl[globalIdx] = 0.0.xxxx;
            return;
        }

        float3 p0 = sortedNewPositions[globalIdx].xyz;
        float3 v0 = sortedNewVelocities[globalIdx].xyz;
        int count = contactCounts[globalIdx];
        int contactIdx = globalIdx;
        float3 rotSum = 0.0.xxx;

        for (int i = 0; i < count; ++i) {
            int contact = contacts[contactIdx];
            contactIdx += gParams.kNumParticlesAligned;
            int contactPhase = sortedPhases[contact];
            bool contactFluid = (contactPhase & 0x400000) != 0;

            if (contactFluid == false)
                continue;

            float3 p1 = sortedNewPositions[contact].xyz;
            float3 d01 = p0 - p1;
            float lenSqr = dot(d01, d01);
            bool insideKernel = lenSqr <= gParams.kRadiusSq && 0 < lenSqr;
            float3 v1 = sortedNewVelocities[contact].xyz;
            float3 v10 = v1 - v0;
            float len = sqrt(lenSqr);
            float3 q10 = ((1.0 - len * gParams.kInvRadius) * -gParams.kSpiky2) * d01 / len;
            // Component-wise rather than the swizzled form. The swizzled cross makes
            // FXC rotate d01 into non-natural lanes, and d01 also feeds dot(d01, d01),
            // so the rotation changes that dp3's summation order and the result.
            float3 rot;
            rot.x = v10.y * q10.z - q10.y * v10.z;
            rot.y = v10.z * q10.x - q10.z * v10.x;
            rot.z = v10.x * q10.y - q10.x * v10.y;

            rotSum = insideKernel ? rotSum + rot : rotSum;
        }

        float rotAmount = length(rotSum);
        curl[globalIdx] = float4(rotSum, rotAmount);
    }
}
