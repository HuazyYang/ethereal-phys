#include "KernelParams.hlsli"

#define BLOCK_DIM_X         256

StructuredBuffer<int> cellStarts : register(t0);
StructuredBuffer<int> cellEnds : register(t1);
Buffer<uint> indices : register(t2);
StructuredBuffer<float4> sortedNewPositions : register(t3);
StructuredBuffer<float4> restPositions : register(t4);
StructuredBuffer<int> phases : register(t5);
StructuredBuffer<float3> bounds : register(t6);
RWStructuredBuffer<int> contacts : register(u0);
RWStructuredBuffer<int> contactCounts : register(u1);

#define BLOCK_DIM_X         256

[numthreads(BLOCK_DIM_X, 1, 1)]
void CollideParticles(uint3 dtid: SV_DispatchThreadID) {
    int globalIdx = int(dtid.x);
    if (globalIdx < gParams.kNumParticles) {
        float3 p = sortedNewPositions[globalIdx].xyz;
        int3 ipos = int3((p - bounds[0]) * gParams.kInvCellEdge);
        int xLower = ipos.x - 1, xUpper = ipos.x + 1;
        int yLower = ipos.y - 1, yUpper = ipos.y + 1;
        int zLower = ipos.z - 1, zUpper = ipos.z + 1;
        int phase = phases[globalIdx];
        int ignoreSelfCollision = phase & eNvFlexPhaseSelfCollideFilter;
        int group = phase & eNvFlexPhaseGroupMask;
        int selfCollision = phase & eNvFlexPhaseSelfCollide;
        float3 p0;
        if (ignoreSelfCollision) {
            p0 = restPositions[indices[globalIdx]].xyz;
        }

        int selfNext = globalIdx + 1;
        bool ignoreSelfEnabled = ignoreSelfCollision != 0;
        bool selfCollideEnabled = selfCollision != 0;

        int nextNeighborIdx = globalIdx;
        int numNeighbors = 0;

        for (int z = zLower; z <= zUpper; ++z) {
            for (int y = yLower; y <= yUpper; ++y) {
                for (int x = xLower; x <= xUpper; ++x) {
                    int hash = (x & 127) | (((y & 127) << 7) | ((z & 127) << 14));
                    int cellStart = cellStarts[hash];
                    int cellEnd = cellEnds[hash];

                    for (int cellId = cellStart; cellId < cellEnd;) {
                        if (cellId == globalIdx) {
                            cellId = selfNext;
                            continue;
                        }

                        int phase = phases[cellId];
                        int group2 = (phase & eNvFlexPhaseGroupMask);
                        bool sameGroup = (group == group2);
                        if (!sameGroup || selfCollideEnabled) {
                            if (ignoreSelfEnabled && sameGroup) {
                                float3 p1 = restPositions[indices[cellId]].xyz;
                                float3 v10 = p0 - p1;
                                float norm = dot(v10, v10);
                                if (norm < gParams.kRadiusSq) {
                                    cellId += 1;
                                    continue;
                                }
                            }

                            float3 p1 = sortedNewPositions[cellId].xyz;
                            float3 v01 = p - p1;
                            float norm = dot(v01, v01);
                            if (norm < gParams.kCollideParticlesRadiusSq) {
                                if (numNeighbors < gParams.kMaxNeighborsPerParticle) {
                                    contacts[nextNeighborIdx] = cellId;
                                    nextNeighborIdx += gParams.kNumParticlesAligned;
                                    numNeighbors += 1;
                                }
                            }
                        }

                        ++cellId;
                    }
                }
            }
        }

        contactCounts[globalIdx] = numNeighbors;
    }
}

