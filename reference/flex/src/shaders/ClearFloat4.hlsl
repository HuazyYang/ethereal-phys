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
