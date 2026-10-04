#ifndef ETHEREAL_PHYS_SRC_EXT_SDF_H
#define ETHEREAL_PHYS_SRC_EXT_SDF_H

// Module-private: FleX's MakeSDF (reference/flex/core/sdf.cpp), the distance field of a voxelised mesh that
// the rigid body builder takes its sample normals from.

#include <cstdint>

namespace ephys::detail::flex {

// The signed distance field of a voxel volume (fast marching, J. Sethian 1996): negative inside (non-zero
// input voxels), in voxels scaled by 1 / max(width, height, depth). output has the size of the input. A
// volume with no surface voxel leaves output at FLT_MAX, as in FleX. The 2D variant of sdf.cpp is not ported:
// no builder calls it.
void MakeSDF(const uint32_t* input, uint32_t width, uint32_t height, uint32_t depth, float* output);

}  // namespace ephys::detail::flex

#endif /* ETHEREAL_PHYS_SRC_EXT_SDF_H */
