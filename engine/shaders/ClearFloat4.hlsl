// ethereal-phys kernel source. Origin: FleX src/shaders/ClearFloat4.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

cbuffer consts : register(b0) {
    int gLength;
    float3 _pad3;
    float4 gValue;
};

RWStructuredBuffer<float4> buf : register(u0);

[numthreads(256, 1, 1)]
void ClearFloat4(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gLength)
        buf[globalIdx] = gValue;
}
