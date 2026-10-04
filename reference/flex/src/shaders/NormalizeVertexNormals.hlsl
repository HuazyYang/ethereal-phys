#include "KernelParams.hlsli"

Buffer<uint> indices : register(t0);
ByteAddressBuffer tempNormals : register(t1);
RWStructuredBuffer<float4> normals : register(u0);

[numthreads(256, 1, 1)]
void NormalizeVertexNormals(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kNumParticles) {
        int idx = indices[globalIdx];
        float3 n = asfloat(tempNormals.Load3(idx * 16));
        float l = dot(n, n);
        n = l > 0.0 ? n * rsqrt(l) : 0.0.xxx;

        normals[idx] = float4(-n, 0.0);
    }
}
