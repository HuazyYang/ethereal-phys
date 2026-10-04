#include "KernelParams.hlsli"
#include "Utils.hlsli"

#if USE_NV_SHADER_EXT
#include <nvHLSLExtns.h>
#define InterlockedAddFp32 NvInterlockedAddFp32
#endif

StructuredBuffer<FlexInflatable> inflatables : register(t0);
StructuredBuffer<float4> positions : register(t1);
StructuredBuffer<int> reverseLookup : register(t2);
StructuredBuffer<int> indices : register(t3);
StructuredBuffer<float> lambdas : register(t4);
RWByteAddressBuffer accum : register(u0);

#define BLOCK_DIM_X 512

groupshared FlexInflatable inflatableInBlock;

[numthreads(BLOCK_DIM_X, 1, 1)]
void SolveInflatableVolume(int threadIdx: SV_GroupThreadID, int blockIdx: SV_GroupID)
{
    float dx = gParams.kDt * gParams.kMaxSpeed;

    if (threadIdx == 0) {
        inflatableInBlock = inflatables[blockIdx];
    }
    GroupMemoryBarrierWithGroupSync();

    float lambda = lambdas[blockIdx];
    int idxBase = inflatableInBlock.mStartTri;
    int numTris = inflatableInBlock.mNumTris;
    float dx2 = dx * dx;

    for (int triIdx = threadIdx; triIdx < numTris; triIdx += BLOCK_DIM_X) {
        int triIdxAbs = triIdx + idxBase;

        int idx1 = indices[triIdxAbs * 3];
        int prevIdx1 = reverseLookup[idx1];

        int idx2 = indices[triIdxAbs * 3 + 1];
        int prevIdx2 = reverseLookup[idx2];

        int idx3 = indices[triIdxAbs * 3 + 2];
        int prevIdx3 = reverseLookup[idx3];

        float3 prevPos1 = positions[prevIdx1].xyz;
        float3 prevPos2 = positions[prevIdx2].xyz;
        float3 prevPos3 = positions[prevIdx3].xyz;

        prevPos2 -= prevPos1;
        prevPos3 -= prevPos1;

        float3 n;
        n.x = prevPos2.y * prevPos3.z - prevPos2.z * prevPos3.y;
        n.y = prevPos2.z * prevPos3.x - prevPos2.x * prevPos3.z;
        n.z = prevPos2.x * prevPos3.y - prevPos2.y * prevPos3.x;
        float3 Dx = -lambda * n;
        float DxMag2 = dot(Dx, Dx);
        float3 invDxMag = rsqrt(DxMag2);

        Dx = dx2 < DxMag2 ? (dx * invDxMag) * Dx : Dx;

        InterlockedAddFp32(accum, prevIdx1 * 16, Dx.x);
        InterlockedAddFp32(accum, prevIdx1 * 16 + 4, Dx.y);
        InterlockedAddFp32(accum, prevIdx1 * 16 + 8, Dx.z);
        InterlockedAddFp32(accum, prevIdx2 * 16, Dx.x);
        InterlockedAddFp32(accum, prevIdx2 * 16 + 4, Dx.y);
        InterlockedAddFp32(accum, prevIdx2 * 16 + 8, Dx.z);
        InterlockedAddFp32(accum, prevIdx3 * 16, Dx.x);
        InterlockedAddFp32(accum, prevIdx3 * 16 + 4, Dx.y);
        InterlockedAddFp32(accum, prevIdx3 * 16 + 8, Dx.z);
    }
}

