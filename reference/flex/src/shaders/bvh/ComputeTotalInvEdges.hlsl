StructuredBuffer<float3> totalLower : register(t0);
StructuredBuffer<float3> totalUpper : register(t1);
RWStructuredBuffer<float3> totalInvEdges : register(u0);

[numthreads(1, 1, 1)]
void ComputeTotalInvEdges() {
    totalInvEdges[0] = 1.0 / (totalUpper[0] - totalLower[0] + 0.0001);
}
