#include "KernelParams.hlsli"

StructuredBuffer<int> halfSpringOpposites : register(t1);
StructuredBuffer<float> halfSpringLengths : register(t2);
StructuredBuffer<float> halfSpringStiffness : register(t3);
StructuredBuffer<int> particleSpringBegin : register(t4);
StructuredBuffer<int> particleSpringEnd : register(t5);
StructuredBuffer<int> reverseLookup : register(t6);
StructuredBuffer<float4> sortedNewPositionsTex : register(t7);
RWByteAddressBuffer deltas : register(u0);

#define BLOCK_DIM_X 512

groupshared float3 dispInBlocks[BLOCK_DIM_X];

[numthreads(BLOCK_DIM_X, 1, 1)]
void SolveSprings(int threadIdx: SV_GroupThreadID, int dispatchIdx: SV_DispatchThreadID) {

    uint springIdxBase = uint(dispatchIdx) / 8;
    uint springIdxOffset = uint(dispatchIdx) & 7;
    bool outOfBound = springIdxBase >= uint(gParams.kMaxParticles);
    springIdxBase = outOfBound ? 0 : springIdxBase;

    uint idxStart = particleSpringBegin[springIdxBase];
    uint idxEnd = particleSpringEnd[springIdxBase];
    uint idxRange = outOfBound ? 0 : idxEnd - idxStart;
    // The delta slot is indexed by the reverse-lookup result, not by
    // springIdxBase. The DXBC overwrites the register holding springIdxBase with
    // reverseLookup[springIdxBase] here and uses it again for the deltas address,
    // so writing to springIdxBase * 16 sends every spring delta to the wrong
    // particle. Only spring scenes (cloth, soft bodies) show it.
    uint sortedIdx = reverseLookup[springIdxBase];
    float4 pos0 = sortedNewPositionsTex[sortedIdx];
    float3 Dx = 0.0.xxx;

    if (pos0.w > 0.0 && idxRange > 0) {
        for (uint springIdx = idxStart + springIdxOffset; springIdx < idxEnd; springIdx += 8) {
            float4 pos1 = sortedNewPositionsTex[reverseLookup[halfSpringOpposites[springIdx]]];
            float3 d = pos0.xyz - pos1.xyz;
            float invM01 = pos0.w + pos1.w;
            bool massValid = invM01 > 0.0;
            float len = dot(d, d);
            bool lenValid = len > 0.0;
            invM01 = 1.0 / invM01;
            len = sqrt(len);
            float dl = halfSpringLengths[springIdx] - len;
            float k = halfSpringStiffness[springIdx];
            [branch]
            if (k < 0.0) {
                dl = min(dl, 0.0);
                k = -k;
            }

            d /= len;
            float3 dx = lenValid ? (k * invM01 * dl) * d + Dx : Dx;
            Dx = massValid ? dx : Dx;
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
