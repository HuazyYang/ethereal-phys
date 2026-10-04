// ethereal-phys kernel source. Origin: FleX src/shaders/UpdateTrianglesInit.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "KernelParams.hlsli"

RWByteAddressBuffer deltas : register(u0);

[numthreads(256, 1, 1)]
void UpdateTrianglesInit(int globalIdx: SV_DispatchThreadID) {

    if (globalIdx < gParams.kNumParticles) {
        deltas.Store4(globalIdx * 16, 0u.xxxx);
    }
}