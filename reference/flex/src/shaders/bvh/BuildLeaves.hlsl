#include "BVHCommon.hlsli"

StructuredBuffer<float4> itemLowers : register(t0);
StructuredBuffer<float4> itemUppers : register(t1);
Buffer<uint> indices : register(t2);

RWStructuredBuffer<int> rangeLefts : register(u0);
RWStructuredBuffer<int> rangeRights : register(u1);
RWStructuredBuffer<PackedNodeHalf> lowers : register(u2);
RWStructuredBuffer<PackedNodeHalf> uppers : register(u3);

[numthreads(256, 1, 1)]
void BuildLeaves(int idx: SV_DispatchThreadID) {
    if (idx < gNumItems) {
        uint leafIdx = indices[idx];
        float4 boundLower = itemLowers[leafIdx];
        float4 boundUpper = itemUppers[leafIdx];

        PackedNodeHalf nodeLower, nodeUpper;
        nodeLower.v = boundLower.xyz;  
        nodeLower.ib = (leafIdx & 0x7fffffffu) | 0x80000000;

        nodeUpper.v = boundUpper.xyz;
        nodeUpper.ib = leafIdx & 0x7fffffff;

        lowers[idx] = nodeLower;
        uppers[idx] = nodeUpper;
        rangeLefts[idx] = idx;
        rangeRights[idx] = idx;
    }
}
