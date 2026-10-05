#ifndef ETHEREAL_PHYS_SRC_EXT_VOXELIZE_H
#define ETHEREAL_PHYS_SRC_EXT_VOXELIZE_H

// Module-private: FleX's Voxelize (FleX/core/voxelize.cpp), what the rigid and soft body builders
// sample the interior of a closed mesh with. MakeSDF is in ext/sdf.h.

#include "ext/flex_core.h"

#include <cstdint>

namespace ephys::detail::flex {

// Marks the voxels of a width x height x depth grid over [minExtents, maxExtents] whose centres lie inside a
// closed mesh (single-pass parity count along z, with an AABBTree), as uint32_t(-1); the others are 0. The
// voxel (x, y, z) is volume[z * width * height + y * width + x]. numTriangleIndices is 3 per triangle.
// FleX printed "Error self-intersect" when a ray hit the triangle it left; that message is dropped.
void Voxelize(const Vec3* vertices, int numVertices, const int* indices, int numTriangleIndices, uint32_t width,
              uint32_t height, uint32_t depth, uint32_t* volume, Vec3 minExtents, Vec3 maxExtents);

}  // namespace ephys::detail::flex

#endif /* ETHEREAL_PHYS_SRC_EXT_VOXELIZE_H */
