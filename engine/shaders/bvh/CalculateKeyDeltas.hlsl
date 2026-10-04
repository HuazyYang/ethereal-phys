// ethereal-phys kernel source. Origin: FleX src/shaders/bvh/CalculateKeyDeltas.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

cbuffer constBVH : register(b0) {
    int gNumItems;
}

Buffer<uint> keys : register(t0);
RWStructuredBuffer<int> deltas: register(u0);

[numthreads(256, 1, 1)]
void CalculateKeyDeltas(int idx: SV_DispatchThreadID) {
    if (idx < gNumItems - 1) {
        uint x = keys[idx];
        uint y = keys[idx + 1];
        deltas[idx] = x ^ y;
    }
}
