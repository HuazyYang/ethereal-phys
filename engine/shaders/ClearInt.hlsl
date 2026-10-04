// ethereal-phys kernel source. Origin: FleX src/shaders/ClearInt.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.


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
