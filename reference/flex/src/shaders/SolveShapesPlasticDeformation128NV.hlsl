#define SOLVE_SHAPES_BLOCK_SIZE 128
#include "SolveShapesPlasticDeformationNV.hlsl"

[numthreads(SOLVE_SHAPES_BLOCK_SIZE, 1, 1)]
void SolveShapesPlasticDeformation128NV(uint rigid : SV_GroupID, uint threadIdx : SV_GroupThreadID) {
    SolveShapesPlasticDeformationNVBody(rigid, threadIdx);
}
