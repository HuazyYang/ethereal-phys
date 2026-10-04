#include "KernelParams.hlsli"

RWByteAddressBuffer deltas : register(u0);

[numthreads(256, 1, 1)]
void UpdateTrianglesInit(int globalIdx: SV_DispatchThreadID) {

    if (globalIdx < gParams.kNumParticles) {
        deltas.Store4(globalIdx * 16, 0u.xxxx);
    }
}