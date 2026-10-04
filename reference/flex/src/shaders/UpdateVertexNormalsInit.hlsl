#include "KernelParams.hlsli"

RWByteAddressBuffer vertNormals : register(u0);

[numthreads(256, 1, 1)]
void UpdateVertexNormalsInit(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kMaxParticles) {
        vertNormals.Store4(globalIdx * 16, 0u.xxxx);
    }
}