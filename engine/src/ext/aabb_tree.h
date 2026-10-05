#ifndef ETHEREAL_PHYS_SRC_EXT_AABB_TREE_H
#define ETHEREAL_PHYS_SRC_EXT_AABB_TREE_H

// Module-private: FleX's AABBTree (FleX/core/aabbtree.h, .cpp), a bounding volume hierarchy over a
// triangle mesh built with the surface area heuristic, and the closest-hit ray query the voxeliser uses.
//
// Kept from FleX: the build (all three axes sorted by centroid, ties broken by face index, the SAH cost
// 0.125 + pBelow * i + pAbove * (n - i), up to 6 faces per leaf) and the recursive trace (nearer child
// first). Left out: the commented-out code, the debug drawing, TraceRaySlow and the trace statistics, and the
// static depth counter of the build (a member here, so that two builds on two threads do not share it).
//
// The tree references the vertices and indices it was built from; they must outlive it.

#include "ext/flex_core.h"

#include <cstdint>
#include <vector>

namespace ephys::detail::flex {

class AABBTree {
 public:
    AABBTree(const Vec3* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces);

    AABBTree(const AABBTree&) = delete;
    AABBTree& operator=(const AABBTree&) = delete;

    // The closest triangle the ray start + t * dir hits, t >= 0: t, the barycentric coordinates, the sign of
    // the face (IntersectRayTriTwoSided) and its index. False when nothing is hit.
    bool TraceRay(const Vec3& start, const Vec3& dir, float& outT, float& u, float& v, float& w, float& faceSign,
                  uint32_t& faceIndex) const;

 private:
    struct Node {
        // An inner node has no faces and the index of the first of its two children; a leaf has its faces.
        union {
            uint32_t m_children;
            uint32_t m_numFaces;
        };
        uint32_t* m_faces = nullptr;
        Vec3 m_minExtents;
        Vec3 m_maxExtents;

        Node() : m_numFaces(0) {}
    };

    struct Bounds {
        Vec3 m_min;
        Vec3 m_max;

        float GetSurfaceArea() const {
            const Vec3 e = m_max - m_min;
            return 2.0f * (e.x * e.y + e.x * e.z + e.y * e.z);
        }

        void Union(const Bounds& b) {
            m_min = Min(m_min, b.m_min);
            m_max = Max(m_max, b.m_max);
        }
    };

    uint32_t PartitionSAH(uint32_t* faces, uint32_t numFaces);
    void Build();
    void BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces);
    void TraceRecursive(uint32_t nodeIndex, const Vec3& start, const Vec3& dir, float& outT, float& u, float& v,
                        float& w, float& faceSign, uint32_t& faceIndex) const;
    void CalculateFaceBounds(const uint32_t* faces, uint32_t numFaces, Vec3& outMinExtents, Vec3& outMaxExtents) const;

    uint32_t m_freeNode = 0;

    const Vec3* m_vertices;
    const uint32_t m_numVerts;
    const uint32_t* m_indices;
    const uint32_t m_numFaces;

    std::vector<uint32_t> m_faces;
    std::vector<Node> m_nodes;
    std::vector<Bounds> m_faceBounds;

    uint32_t m_depth = 0;
};

}  // namespace ephys::detail::flex

#endif /* ETHEREAL_PHYS_SRC_EXT_AABB_TREE_H */
