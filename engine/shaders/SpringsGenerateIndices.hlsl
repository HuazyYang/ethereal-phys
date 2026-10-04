// ethereal-phys kernel source. Origin: FleX src/shaders/SpringsGenerateIndices.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

cbuffer consts : register(b0) {
    uint gNumSprings;
}

RWBuffer<uint> indices : register(u0);

[numthreads(256, 1, 1)]
void SpringsGenerateIndices(uint idx : SV_DispatchThreadID) {
    if (idx < (gNumSprings << 1))
        indices[idx] = idx;
}
