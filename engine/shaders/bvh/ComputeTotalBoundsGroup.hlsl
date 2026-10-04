// ethereal-phys kernel source. Origin: FleX src/shaders/bvh/ComputeTotalBoundsGroup.hlsl at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#define USE_BLOCK_SIZE 1
#define ComputeTotalBounds ComputeTotalBoundsGroup

#include "ComputeTotalBounds.hlsl"