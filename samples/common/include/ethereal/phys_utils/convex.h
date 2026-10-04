#ifndef ETHEREAL_PHYS_UTILS_CONVEX_H
#define ETHEREAL_PHYS_UTILS_CONVEX_H

// The renderable hull of a set of planes, for drawing a convex collision shape (IConvexAsset, collision.h)
// and finding its bounds: FleX's core/convex.h (Christian Sigg's ConvexMeshBuilder). Part of
// ethereal_phys_utils (samples/common), not of the engine.
//
// The planes are a x + b y + c z + d = 0, with (a, b, c) the outward normal: the convex is where
// a x + b y + c z + d <= 0 for every plane (the form IConvexAsset and FleX's convex meshes take). The hull is
// built in the dual: each plane is a point, the convex hull of those points is computed incrementally on a
// half-edge mesh, and each of its triangles becomes a vertex (the intersection of three planes) and each of
// its vertices a face polygon of the result.

#include <donut/core/math/math.h>

#include <cstdint>
#include <vector>

namespace ephys::utils {

struct ConvexMeshBuilder {
    explicit ConvexMeshBuilder(const dm::float4* planes) : mPlanes(planes) {}

    // Builds the hull of the first numPlanes planes into mVertices and mIndices (appended). Does nothing for
    // fewer than 4 planes; stops early (leaving what it built) for a degenerate set, as FleX does, and at 200
    // triangles of the dual hull or 1000 indices of the result. FleX's unused scale argument is gone.
    void operator()(uint32_t numPlanes);

    const dm::float4* mPlanes;
    std::vector<dm::float3> mVertices;  // the corners of the convex
    std::vector<uint16_t> mIndices;     // its faces as triangles, three indices into mVertices each
};

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_CONVEX_H */
