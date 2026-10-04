cbuffer consts : register(b0) {
    uint gNumSprings;
}

RWBuffer<uint> indices : register(u0);

[numthreads(256, 1, 1)]
void SpringsGenerateIndices(uint idx : SV_DispatchThreadID) {
    if (idx < (gNumSprings << 1))
        indices[idx] = idx;
}
