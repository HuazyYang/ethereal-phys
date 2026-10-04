// ethereal-phys kernel source. Origin: FleX src/shaders/bvh/ComputeTotalInvEdges.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

StructuredBuffer<float3> totalLower : register(t0);
StructuredBuffer<float3> totalUpper : register(t1);
RWStructuredBuffer<float3> totalInvEdges : register(u0);

[numthreads(1, 1, 1)]
void ComputeTotalInvEdges() {
    totalInvEdges[0] = 1.0 / (totalUpper[0] - totalLower[0] + 0.0001);
}
