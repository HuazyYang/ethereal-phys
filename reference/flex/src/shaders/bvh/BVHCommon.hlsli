#ifndef BVHCOMMON_HLSLI
#define BVHCOMMON_HLSLI

cbuffer constBVH : register(b0) {
    int gNumItems;
}

struct PackedNodeHalf {
    float3 v;
    uint ib;
};

#endif /* BVHCOMMON_HLSLI */
