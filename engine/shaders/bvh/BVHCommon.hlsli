// ethereal-phys kernel source. Origin: FleX src/shaders/bvh/BVHCommon.hlsli at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

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
