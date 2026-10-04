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
