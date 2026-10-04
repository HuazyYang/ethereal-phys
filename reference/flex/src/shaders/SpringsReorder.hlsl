cbuffer consts : register(b0) {
    uint gNumSprings;
}

Buffer<uint> sortedIndices : register(t0);
StructuredBuffer<uint> indices : register(t1);
StructuredBuffer<float> lengths : register(t2);
StructuredBuffer<float> stiffness : register(t3);
RWStructuredBuffer<uint> halfOpposite : register(u0);
RWStructuredBuffer<float> halfLengths : register(u1);
RWStructuredBuffer<float> halfStiffness : register(u2);

[numthreads(256, 1, 1)]
void SpringsReorder(uint idx : SV_DispatchThreadID) {
    if (idx < (gNumSprings << 1)) {
        uint sortedIndex = sortedIndices[idx];
        uint springIndex = sortedIndex >> 1;
        uint oppositeIndex = (sortedIndex & 1) ? sortedIndex - 1 : sortedIndex + 1;

        halfOpposite[idx] = indices[oppositeIndex];
        halfLengths[idx] = lengths[springIndex];
        halfStiffness[idx] = stiffness[springIndex];
    }
}
