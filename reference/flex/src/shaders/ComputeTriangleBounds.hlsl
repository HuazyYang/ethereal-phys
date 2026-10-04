
cbuffer mConstTriMeshInfo : register(b0) {
    int numTris;
}

StructuredBuffer<float4> verticesArray : register(t0);
StructuredBuffer<int> indicesArray : register(t1);
RWStructuredBuffer<float4> lowers : register(u0);
RWStructuredBuffer<float4> uppers : register(u1);

[numthreads(256, 1, 1)]
void ComputeTriangleBounds(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < numTris) {
        float3 pos0 = verticesArray[indicesArray[globalIdx * 3]].xyz;
        float3 pos1 = verticesArray[indicesArray[globalIdx * 3 + 1]].xyz;
        float3 pos2 = verticesArray[indicesArray[globalIdx * 3 + 2]].xyz;
        float3 lower = min(pos0, min(pos1, pos2));
        float3 upper = max(pos0, max(pos1, pos2));
        lowers[globalIdx] = float4(lower, 0.0);
        uppers[globalIdx] = float4(upper, 0.0);
    }
}
