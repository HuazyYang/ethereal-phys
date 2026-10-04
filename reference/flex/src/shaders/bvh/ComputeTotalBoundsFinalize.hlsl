cbuffer constBVH : register(b0) {
    int gNumItems;
}

StructuredBuffer<float4> readBoundsLower : register(t2);
StructuredBuffer<float4> readBoundsUpper : register(t3);
RWStructuredBuffer<float3> totalLower : register(u2);
RWStructuredBuffer<float3> totalUpper : register(u3);
RWStructuredBuffer<float3> totalInvEdges : register(u4);

[numthreads(1, 1, 1)]
void ComputeTotalBoundsFinalize() {
    const int numGroups = (((gNumItems + 255u) >> 8u) + 255u) >> 8u;
    float3 v0 = readBoundsLower[0].xyz;
    float3 v1 = readBoundsUpper[0].xyz;
    float3 v2;

    for (int i = 1; i < numGroups; ++i) {
        v2 = readBoundsLower[i].xyz;
        v0 = min(v0, v2);
        v2 = readBoundsUpper[i].xyz;
        v1 = max(v1, v2);
    }

    totalLower[0] = v0;
    totalUpper[0] = v1;
    totalInvEdges[0] = 1.0 / ((v1 - v0) + 0.0001);
}
