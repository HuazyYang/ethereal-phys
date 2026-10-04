#include <ethereal/phys_utils/voxelize.h>

#include "aabb_tree.h"

#include <algorithm>
#include <cstring>

// A port of reference/flex/core/voxelize.cpp in FleX's evaluation order: a copy of the engine's
// engine/src/ext/voxelize.cpp behind a donut::math signature.

namespace ephys::utils {

void Voxelize(const dm::float3* vertexArray, int numVertices, const int* indices, int numTriangleIndices,
              uint32_t width, uint32_t height, uint32_t depth, uint32_t* volume, const dm::float3& minExtentsIn,
              const dm::float3& maxExtentsIn) {
    using namespace detail;
    static_assert(sizeof(dm::float3) == sizeof(Vec3), "dm::float3 and FleX's Vec3 are three floats");
    const Vec3* vertices = reinterpret_cast<const Vec3*>(vertexArray);
    const Vec3 minExtents(minExtentsIn.x, minExtentsIn.y, minExtentsIn.z);
    const Vec3 maxExtents(maxExtentsIn.x, maxExtentsIn.y, maxExtentsIn.z);

    std::memset(volume, 0, sizeof(uint32_t) * width * height * depth);

    // a hierarchy over the mesh
    const AABBTree tree(vertices, uint32_t(numVertices), reinterpret_cast<const uint32_t*>(indices),
                        uint32_t(numTriangleIndices / 3));

    // parity count method, single pass
    const Vec3 extents(maxExtents - minExtents);
    const Vec3 delta(extents.x / float(width), extents.y / float(height), extents.z / float(depth));
    const Vec3 offset(0.5f * delta.x, 0.5f * delta.y, 0.5f * delta.z);

    // the bias that steps "off" a triangle that was hit (not very robust, FleX says)
    const float eps = 0.00001f * extents.z;

    for (uint32_t x = 0; x < width; ++x) {
        for (uint32_t y = 0; y < height; ++y) {
            bool inside = false;

            const Vec3 rayDir = Vec3(0.0f, 0.0f, 1.0f);
            Vec3 rayStart = minExtents + Vec3(float(x) * delta.x + offset.x, float(y) * delta.y + offset.y, 0.0f);

            for (;;) {
                float t, u, v, w, s;
                uint32_t tri;

                if (!tree.TraceRay(rayStart, rayDir, t, u, v, w, s, tri))
                    break;

                // the cell of the hit
                const float zpos = rayStart.z + t * rayDir.z;
                const float zhit = (zpos - minExtents.z) / delta.z;

                const uint32_t z = uint32_t(floorf((rayStart.z - minExtents.z) / delta.z + 0.5f));
                const uint32_t zend = std::min(uint32_t(floorf(zhit + 0.5f)), depth - 1);

                if (inside) {
                    // fill the column
                    for (uint32_t k = z; k < zend; ++k)
                        volume[k * width * height + y * width + x] = uint32_t(-1);
                }

                inside = !inside;

                rayStart += rayDir * (t + eps);
            }
        }
    }
}

}  // namespace ephys::utils
