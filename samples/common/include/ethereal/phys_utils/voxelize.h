#ifndef ETHEREAL_PHYS_UTILS_VOXELIZE_H
#define ETHEREAL_PHYS_UTILS_VOXELIZE_H

// Voxelisation and the signed distance field of a voxel volume: FleX's core/voxelize.cpp and the 3D half of
// core/sdf.cpp, what the FleX demo's CreateParticleShape (ethereal_phys_helpers) samples a closed mesh with.
//
// Part of ethereal_phys_utils, the samples' utility library (samples/common). The engine has its own,
// module-private copies for its asset builders (engine/src/ext); these give the same bits (FleX's arithmetic,
// src/flex_core.h).

#include <donut/core/math/math.h>

#include <cstdint>

namespace ephys::utils {

// Marks the voxels of a width x height x depth grid over [minExtents, maxExtents] whose centres lie inside a
// closed mesh (a parity count along z over a hierarchy of the triangles), as uint32_t(-1); the others are 0.
// The voxel (x, y, z) is volume[z * width * height + y * width + x]; numTriangleIndices is 3 per triangle.
// FleX printed "Error self-intersect" when a ray hit the triangle it left; that message is dropped.
void Voxelize(const dm::float3* vertices, int numVertices, const int* indices, int numTriangleIndices,
              uint32_t width, uint32_t height, uint32_t depth, uint32_t* volume, const dm::float3& minExtents,
              const dm::float3& maxExtents);

// The signed distance field of a voxel volume (fast marching, J. Sethian 1996): negative inside (non-zero
// input voxels), in voxels scaled by 1 / max(width, height, depth). output has the size of the input. A volume
// with no surface voxel leaves output at FLT_MAX, as in FleX.
void MakeSDF(const uint32_t* input, uint32_t width, uint32_t height, uint32_t depth, float* output);

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_VOXELIZE_H */
