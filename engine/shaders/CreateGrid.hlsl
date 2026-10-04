// ethereal-phys kernel source. Origin: FleX src/shaders/CreateGrid.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#include "KernelParams.hlsli"

Buffer<uint> cellIds : register(t0);
RWStructuredBuffer<int> cellStarts : register(u0);
RWStructuredBuffer<int> cellEnds : register(u1);

#define BLOCK_DIM_X     256

[numthreads(BLOCK_DIM_X, 1, 1)]
void CreateGrid(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kNumParticles) {
        uint cellId = cellIds[globalIdx];
        if (globalIdx == 0)
            cellStarts[cellId] = 0;
        else {
            uint prevCellId = cellIds[globalIdx - 1];
            if (cellId != prevCellId) {
                cellStarts[cellId] = globalIdx;
                cellEnds[prevCellId] = globalIdx;
            }
        }

        if (globalIdx == gParams.kNumParticles - 1)
            cellEnds[cellId] = globalIdx + 1;
    }
}