// ethereal-phys kernel source. Origin: FleX src/shaders/UpdateVertexNormalsInit.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "KernelParams.hlsli"

RWByteAddressBuffer vertNormals : register(u0);

[numthreads(256, 1, 1)]
void UpdateVertexNormalsInit(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kMaxParticles) {
        vertNormals.Store4(globalIdx * 16, 0u.xxxx);
    }
}