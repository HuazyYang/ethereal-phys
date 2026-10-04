
cbuffer constBVH : register(b0) {
    int gNumItems;
};

StructuredBuffer<float4> itemLowers : register(t0);
StructuredBuffer<float4> itemUppers : register(t1);
StructuredBuffer<float3> gridLower : register(t2);
StructuredBuffer<float3> gridInvEdges : register(t3);

RWBuffer<uint> indices : register(u0);
RWBuffer<uint> keys : register(u1);

uint EncodeMorton3(uint3 v) {
    v = (v ^ (v << 16)) & 0x010000ff;
    v = (v ^ (v << 8)) & 0x0100f00f;
    v = (v ^ (v << 4)) & 0x010c30c3;
    v = ((v ^ (v << 2)) << uint3(2, 1, 0)) & uint3(0x04924924, 0x02492492, 0x01249249);
    v.z = v.x + v.y + v.z;
    return v.z;
}

[numthreads(256, 1, 1)]
void CalculateMortonCodes(int idx: SV_DispatchThreadID) {

    float3 v0, v1;
    uint key, pos;

    if (idx < gNumItems) {

        v0 = itemLowers[idx].xyz;
        v1 = itemUppers[idx].xyz;
        v0 += v1;

        v1 = gridLower[0];
        v0 = 0.5 * v0 - v1;

        v1 = gridInvEdges[0];
        v0 = v0 * v1 * 512.0;

        v0 = trunc(v0);
        v0 = min(v0, 511.0.xxx);
        v0 = max(v0, 0.0.xxx);

        key = EncodeMorton3(uint3(v0.zyx));
        pos = idx;
    } else {
        key = ~0u;
        pos = ~0u;
    }

    keys[idx] = key;
    indices[idx] = pos;
}
