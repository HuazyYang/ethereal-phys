cbuffer consts : register(b0) {
    uint gNumSprings;
}

Buffer<uint> ids : register(t0);
RWStructuredBuffer<uint> lower : register(u0);
RWStructuredBuffer<uint> upper : register(u1);

[numthreads(256, 1, 1)]
void SpringsParticleRange(uint idx : SV_DispatchThreadID) {
    uint numHalfSprings = gNumSprings << 1;
    if (idx < numHalfSprings) {
        uint id = ids[idx];
        if (idx == 0) {
            lower[id] = 0;
        } else {
            uint prevId = ids[idx - 1];
            if (id != prevId) {
                lower[id] = idx;
                upper[prevId] = idx;
            }
        }

        if (idx == numHalfSprings - 1)
            upper[id] = idx + 1;
    }
}
