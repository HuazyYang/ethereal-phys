
cbuffer consts : register(b0) {
    int gLength;
    int gValue;
    int _pad1;
    int _pad2;
};

RWStructuredBuffer<int> buf : register(u0);

[numthreads(256, 1, 1)]
void ClearInt(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gLength)
        buf[globalIdx] = gValue;
}
