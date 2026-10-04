#include "KernelParams.hlsli"
#include "Utils.hlsli"

StructuredBuffer<int> indices : register(t1);
StructuredBuffer<float3> triNormals : register(t2);
RWByteAddressBuffer accum : register(u0);

#if USE_NV_SHADER_EXT
#include <nvHLSLExtns.h>
#define InterlockedAddFp32 NvInterlockedAddFp32
#endif

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void UpdateVertexNormals(int globalIdx: SV_DispatchThreadID) {

    if(globalIdx < gParams.kNumTriangles) {

        int idx1 = indices[globalIdx * 3];
        int idx2 = indices[globalIdx * 3 + 1];
        int idx3 = indices[globalIdx * 3 + 2];

        float3 n = triNormals[globalIdx];

        InterlockedAddFp32(accum, idx1 * 16, n.x);
        InterlockedAddFp32(accum, idx1 * 16 + 4, n.y);
        InterlockedAddFp32(accum, idx1 * 16 + 8, n.z);

        InterlockedAddFp32(accum, idx2 * 16, n.x);
        InterlockedAddFp32(accum, idx2 * 16 + 4, n.y);
        InterlockedAddFp32(accum, idx2 * 16 + 8, n.z);

        InterlockedAddFp32(accum, idx3 * 16, n.x);
        InterlockedAddFp32(accum, idx3 * 16 + 4, n.y);
        InterlockedAddFp32(accum, idx3 * 16 + 8, n.z);
    }
}
