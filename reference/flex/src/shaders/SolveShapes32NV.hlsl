#define SOLVE_SHAPES_BLOCK_SIZE 32
#include "SolveShapesNV.hlsl"

[numthreads(SOLVE_SHAPES_BLOCK_SIZE, 1, 1)]
void SolveShapes32NV(uint rigid : SV_GroupID, uint threadIdx : SV_GroupThreadID) {
    SolveShapesNVBody(rigid, threadIdx);
}
