#include "ext/aabb_tree.h"

#include <algorithm>

// A port of FleX/core/aabbtree.cpp (see aabb_tree.h for what was left out).

namespace ephys::detail::flex {
namespace {

// Orders faces by the centroid on one axis, ties by index: a strict total order, so std::sort has one result.
struct FaceSorter {
    FaceSorter(const Vec3* positions, const uint32_t* indices, uint32_t axis)
        : m_vertices(positions), m_indices(indices), m_axis(axis) {}

    bool operator()(uint32_t lhs, uint32_t rhs) const {
        const float a = GetCentroid(lhs);
        const float b = GetCentroid(rhs);
        if (a == b)
            return lhs < rhs;
        return a < b;
    }

    float GetCentroid(uint32_t face) const {
        const Vec3& a = m_vertices[m_indices[face * 3 + 0]];
        const Vec3& b = m_vertices[m_indices[face * 3 + 1]];
        const Vec3& c = m_vertices[m_indices[face * 3 + 2]];
        return (a[int(m_axis)] + b[int(m_axis)] + c[int(m_axis)]) / 3.0f;
    }

    const Vec3* m_vertices;
    const uint32_t* m_indices;
    uint32_t m_axis;
};

}  // namespace

AABBTree::AABBTree(const Vec3* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces)
    : m_vertices(vertices), m_numVerts(numVerts), m_indices(indices), m_numFaces(numFaces) {
    Build();
}

void AABBTree::CalculateFaceBounds(const uint32_t* faces, uint32_t numFaces, Vec3& outMinExtents,
                                   Vec3& outMaxExtents) const {
    Vec3 minExtents(FLT_MAX);
    Vec3 maxExtents(-FLT_MAX);

    for (uint32_t i = 0; i < numFaces; ++i) {
        const Vec3 a = m_vertices[m_indices[faces[i] * 3 + 0]];
        const Vec3 b = m_vertices[m_indices[faces[i] * 3 + 1]];
        const Vec3 c = m_vertices[m_indices[faces[i] * 3 + 2]];

        minExtents = Min(a, minExtents);
        maxExtents = Max(a, maxExtents);

        minExtents = Min(b, minExtents);
        maxExtents = Max(b, maxExtents);

        minExtents = Min(c, minExtents);
        maxExtents = Max(c, maxExtents);
    }

    outMinExtents = minExtents;
    outMaxExtents = maxExtents;
}

void AABBTree::Build() {
    const uint32_t numFaces = m_numFaces;

    m_faces.reserve(numFaces);
    m_faceBounds.reserve(numFaces);

    for (uint32_t i = 0; i < numFaces; ++i) {
        Bounds top;
        CalculateFaceBounds(&i, 1, top.m_min, top.m_max);

        m_faces.push_back(i);
        m_faceBounds.push_back(top);
    }

    m_nodes.reserve(uint32_t(float(numFaces) * 1.5f));

    m_freeNode = 1;
    if (numFaces)
        BuildRecursive(0, m_faces.data(), numFaces);

    std::vector<Bounds>().swap(m_faceBounds);
}

uint32_t AABBTree::PartitionSAH(uint32_t* faces, uint32_t numFaces) {
    uint32_t bestAxis = 0;
    uint32_t bestIndex = 0;
    float bestCost = FLT_MAX;

    for (uint32_t a = 0; a < 3; ++a) {
        // sort the faces by centroid
        std::sort(faces, faces + numFaces, FaceSorter(m_vertices, m_indices, a));

        // two passes over the data for the bounds below and above each split
        std::vector<float> cumulativeLower(numFaces);
        std::vector<float> cumulativeUpper(numFaces);

        Bounds lower;
        Bounds upper;

        for (uint32_t i = 0; i < numFaces; ++i) {
            lower.Union(m_faceBounds[faces[i]]);
            upper.Union(m_faceBounds[faces[numFaces - i - 1]]);

            cumulativeLower[i] = lower.GetSurfaceArea();
            cumulativeUpper[numFaces - i - 1] = upper.GetSurfaceArea();
        }

        const float invTotalSA = 1.0f / cumulativeUpper[0];

        // every split position
        for (uint32_t i = 0; i < numFaces - 1; ++i) {
            const float pBelow = cumulativeLower[i] * invTotalSA;
            const float pAbove = cumulativeUpper[i] * invTotalSA;

            const float cost = 0.125f + (pBelow * float(i) + pAbove * float(numFaces - i));
            if (cost <= bestCost) {
                bestCost = cost;
                bestIndex = i;
                bestAxis = a;
            }
        }
    }

    // sort again by the best axis
    std::sort(faces, faces + numFaces, FaceSorter(m_vertices, m_indices, bestAxis));

    return bestIndex + 1;
}

void AABBTree::BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces) {
    const uint32_t kMaxFacesPerLeaf = 6;

    // more nodes when they run out
    if (nodeIndex >= m_nodes.size()) {
        const uint32_t s = std::max(uint32_t(1.5f * float(m_nodes.size())), 512U);
        m_nodes.resize(s);
    }

    ++m_depth;

    // m_nodes may grow in the recursion below: no reference to a node is held across it
    CalculateFaceBounds(faces, numFaces, m_nodes[nodeIndex].m_minExtents, m_nodes[nodeIndex].m_maxExtents);

    if (numFaces <= kMaxFacesPerLeaf) {
        m_nodes[nodeIndex].m_faces = faces;
        m_nodes[nodeIndex].m_numFaces = numFaces;
    } else {
        const uint32_t leftCount = PartitionSAH(faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // two nodes
        m_nodes[nodeIndex].m_children = m_freeNode;
        m_freeNode += 2;

        const uint32_t children = m_nodes[nodeIndex].m_children;
        BuildRecursive(children + 0, faces, leftCount);
        BuildRecursive(children + 1, faces + leftCount, rightCount);
    }

    --m_depth;
}

bool AABBTree::TraceRay(const Vec3& start, const Vec3& dir, float& outT, float& u, float& v, float& w,
                        float& faceSign, uint32_t& faceIndex) const {
    outT = FLT_MAX;
    if (!m_nodes.empty())
        TraceRecursive(0, start, dir, outT, u, v, w, faceSign, faceIndex);
    return outT != FLT_MAX;
}

void AABBTree::TraceRecursive(uint32_t nodeIndex, const Vec3& start, const Vec3& dir, float& outT, float& outU,
                              float& outV, float& outW, float& faceSign, uint32_t& faceIndex) const {
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == nullptr) {
        // the nearer child first
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        float dist[2] = {FLT_MAX, FLT_MAX};
        IntersectRayAABB(start, dir, leftChild.m_minExtents, leftChild.m_maxExtents, dist[0]);
        IntersectRayAABB(start, dir, rightChild.m_minExtents, rightChild.m_maxExtents, dist[1]);

        uint32_t closest = 0;
        uint32_t furthest = 1;
        if (dist[1] < dist[0]) {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
            TraceRecursive(node.m_children + closest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);
        if (dist[furthest] < outT)
            TraceRecursive(node.m_children + furthest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);
    } else {
        float t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i) {
            const uint32_t indexStart = node.m_faces[i] * 3;

            const Vec3& a = m_vertices[m_indices[indexStart + 0]];
            const Vec3& b = m_vertices[m_indices[indexStart + 1]];
            const Vec3& c = m_vertices[m_indices[indexStart + 2]];

            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s)) {
                if (t < outT) {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

}  // namespace ephys::detail::flex
