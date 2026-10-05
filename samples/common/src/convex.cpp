#include <ethereal/phys_utils/convex.h>

#include <algorithm>
#include <cassert>
#include <utility>

// A port of FleX/core/convex.h over Donut's math. FleX's implementation lived in an anonymous
// namespace of a header; it is a source file here.

namespace ephys::utils {
namespace {

dm::float3 xyz(const dm::float4& v) { return dm::float3(v.x, v.y, v.z); }

float det(const dm::float4& v0, const dm::float4& v1, const dm::float4& v2, const dm::float4& v3) {
    const dm::float3 d0 = xyz(v0);
    const dm::float3 d1 = xyz(v1);
    const dm::float3 d2 = xyz(v2);
    const dm::float3 d3 = xyz(v3);

    return v0.w * dm::dot(dm::cross(d1, d2), d3) - v1.w * dm::dot(dm::cross(d0, d2), d3) +
           v2.w * dm::dot(dm::cross(d0, d1), d3) - v3.w * dm::dot(dm::cross(d0, d1), d2);
}

// the point the three planes meet in
dm::float3 intersect(const dm::float4& p0, const dm::float4& p1, const dm::float4& p2) {
    const dm::float3 d0 = xyz(p0);
    const dm::float3 d1 = xyz(p1);
    const dm::float3 d2 = xyz(p2);

    return (p0.w * dm::cross(d1, d2) + p1.w * dm::cross(d2, d0) + p2.w * dm::cross(d0, d1)) /
           dm::dot(d0, dm::cross(d2, d1));
}

const uint16_t sInvalid = uint16_t(-1);

// Restriction (FleX): a single patch per vertex.
struct HalfedgeMesh {
    struct Halfedge {
        explicit Halfedge(uint16_t vertex = sInvalid, uint16_t face = sInvalid, uint16_t next = sInvalid,
                          uint16_t prev = sInvalid)
            : mVertex(vertex), mFace(face), mNext(next), mPrev(prev) {}

        uint16_t mVertex;  // to
        uint16_t mFace;    // left
        uint16_t mNext;    // ccw
        uint16_t mPrev;    // cw
    };

    uint16_t findHalfedge(uint16_t v0, uint16_t v1) const {
        uint16_t h = mVertices[v0];
        const uint16_t start = h;
        while (h != sInvalid && mHalfedges[h].mVertex != v1) {
            h = mHalfedges[h ^ 1].mNext;
            if (h == start)
                return sInvalid;
        }
        return h;
    }

    void connect(uint16_t h0, uint16_t h1) {
        mHalfedges[h0].mNext = h1;
        mHalfedges[h1].mPrev = h0;
    }

    void addTriangle(uint16_t v0, uint16_t v1, uint16_t v2) {
        // new vertices
        const uint16_t n = uint16_t(std::max(v0, std::max(v1, v2)) + 1);
        if (mVertices.size() < n)
            mVertices.resize(n, sInvalid);

        // the half-edges of the triangle, and their previous and next
        const uint16_t verts[] = {v0, v1, v2};
        uint16_t handles[3], prev[3], next[3];
        for (uint16_t i = 0; i < 3; ++i) {
            const uint16_t j = uint16_t((i + 1) % 3);
            uint16_t h = findHalfedge(verts[i], verts[j]);
            if (h == sInvalid) {
                // a new edge
                h = uint16_t(mHalfedges.size());
                mHalfedges.push_back(Halfedge(verts[j]));
                mHalfedges.push_back(Halfedge(verts[i]));
            }
            handles[i] = h;
            prev[i] = mHalfedges[h].mPrev;
            next[i] = mHalfedges[h].mNext;
        }

        // the connectivity
        for (uint16_t i = 0; i < 3; ++i) {
            const uint16_t j = uint16_t((i + 1) % 3);

            mHalfedges[handles[i]].mFace = uint16_t(mFaces.size());

            // previous and next
            connect(handles[i], handles[j]);

            if (next[j] == sInvalid)  // a new next edge: connect the opposite
                connect(handles[j] ^ 1, next[i] != sInvalid ? next[i] : uint16_t(handles[i] ^ 1));

            if (prev[i] == sInvalid)  // a new previous edge: connect the opposite
                connect(prev[j] != sInvalid ? prev[j] : uint16_t(handles[j] ^ 1), handles[i] ^ 1);

            // the previous edge is a boundary: update the middle vertex
            if (mHalfedges[handles[i] ^ 1].mFace == sInvalid)
                mVertices[verts[j]] = handles[i] ^ 1;
        }

        assert(mNumTriangles < 0xffff);
        mFaces.push_back(handles[2]);
        ++mNumTriangles;
    }

    uint16_t removeTriangle(uint16_t f) {
        uint16_t result = sInvalid;

        uint16_t h = mFaces[f];
        for (uint16_t i = 0; i < 3; ++i) {
            const uint16_t v0 = mHalfedges[h ^ 1].mVertex;
            const uint16_t v1 = mHalfedges[h].mVertex;

            mHalfedges[h].mFace = sInvalid;

            if (mHalfedges[h ^ 1].mFace == sInvalid) {
                // a boundary edge: remove it
                const uint16_t v0Prev = mHalfedges[h].mPrev;
                const uint16_t v0Next = mHalfedges[h ^ 1].mNext;
                const uint16_t v1Prev = mHalfedges[h ^ 1].mPrev;
                const uint16_t v1Next = mHalfedges[h].mNext;

                connect(v0Prev, v0Next);
                connect(v1Prev, v1Next);

                // the boundary of the vertices, or their removal
                mVertices[v0] = (v0Prev ^ 1) == v0Next ? sInvalid : v0Next;
                mVertices[v1] = (v1Prev ^ 1) == v1Next ? sInvalid : v1Next;
            } else {
                mVertices[v0] = h;  // the boundary of the vertex
                result = v1;
            }

            h = mHalfedges[h].mNext;
        }

        mFaces[f] = sInvalid;
        --mNumTriangles;

        return result;
    }

    // true when vertex v is in front of face f
    bool visible(uint16_t v, uint16_t f) const {
        uint16_t h = mFaces[f];
        if (h == sInvalid)
            return false;

        const uint16_t v0 = mHalfedges[h].mVertex;
        h = mHalfedges[h].mNext;
        const uint16_t v1 = mHalfedges[h].mVertex;
        h = mHalfedges[h].mNext;
        const uint16_t v2 = mHalfedges[h].mVertex;

        return det(mPoints[v], mPoints[v0], mPoints[v1], mPoints[v2]) < -1e-3f;
    }

    std::vector<Halfedge> mHalfedges;
    std::vector<uint16_t> mVertices;  // vertex -> (boundary) half-edge
    std::vector<uint16_t> mFaces;     // face -> half-edge
    std::vector<dm::float4> mPoints;
    uint16_t mNumTriangles = 0;
};

}  // namespace

void ConvexMeshBuilder::operator()(uint32_t numPlanes) {
    if (numPlanes < 4)
        return;  // FleX: "todo: handle degenerate cases"
    numPlanes = std::min<uint32_t>(numPlanes, 0xfffe);  // the half-edge mesh indexes with uint16_t

    HalfedgeMesh mesh;

    // the points (the planes)
    mesh.mPoints.assign(mPlanes, mPlanes + numPlanes);

    // a tetrahedron to start from
    mesh.addTriangle(0, 1, 2);
    mesh.addTriangle(0, 3, 1);
    mesh.addTriangle(1, 3, 2);
    mesh.addTriangle(2, 3, 0);

    // flipped if inside out
    if (mesh.visible(3, 0))
        std::swap(mesh.mPoints[0], mesh.mPoints[1]);

    // the other points
    for (uint16_t i = 4; i < mesh.mPoints.size(); ++i) {
        // remove every visible triangle
        uint16_t v0 = sInvalid;
        for (uint16_t j = 0; j < mesh.mFaces.size(); ++j) {
            if (mesh.visible(i, j))
                v0 = std::min(v0, mesh.removeTriangle(j));
        }

        if (v0 == sInvalid)
            continue;  // no triangle removed

        if (!mesh.mNumTriangles)
            return;  // an empty mesh

        // a boundary vertex that was not deleted
        for (uint16_t h = 0; mesh.mVertices[v0] == sInvalid; h += 2) {
            if ((mesh.mHalfedges[h].mFace == sInvalid) ^ (mesh.mHalfedges[h + 1].mFace == sInvalid))
                v0 = mesh.mHalfedges[h].mVertex;
        }

        // fill the hole
        const uint16_t start = v0;
        do {
            const uint16_t h = mesh.mVertices[v0];
            const uint16_t v1 = mesh.mHalfedges[h].mVertex;
            mesh.addTriangle(v0, v1, i);
            v0 = v1;
        } while (v0 != start && mesh.mNumTriangles < 200);

        if (mesh.mNumTriangles == 200)
            return;
    }

    // the triangles become vertices (where three planes meet)
    std::vector<uint32_t> face2Vertex(mesh.mFaces.size());
    for (uint32_t i = 0; i < mesh.mFaces.size(); ++i) {
        face2Vertex[i] = uint32_t(mVertices.size());

        uint16_t h = mesh.mFaces[i];
        if (h == sInvalid)
            continue;

        const uint16_t v0 = mesh.mHalfedges[h].mVertex;
        h = mesh.mHalfedges[h].mNext;
        const uint16_t v1 = mesh.mHalfedges[h].mVertex;
        h = mesh.mHalfedges[h].mNext;
        const uint16_t v2 = mesh.mHalfedges[h].mVertex;

        mVertices.push_back(intersect(mesh.mPoints[v0], mesh.mPoints[v1], mesh.mPoints[v2]));
    }

    // the vertices become polygons (the one-ring of faces), as triangle fans
    for (uint32_t i = 0; i < mesh.mVertices.size(); ++i) {
        uint16_t h = mesh.mVertices[i];
        if (h == sInvalid || mesh.mHalfedges[h].mFace == sInvalid)
            continue;

        const uint16_t v0 = uint16_t(face2Vertex[mesh.mHalfedges[h].mFace]);
        h = mesh.mHalfedges[h].mPrev ^ 1;
        if (h == sInvalid || mesh.mHalfedges[h].mFace == sInvalid)
            continue;

        uint16_t v1 = uint16_t(face2Vertex[mesh.mHalfedges[h].mFace]);

        while (mIndices.size() < 1000) {
            h = mesh.mHalfedges[h].mPrev ^ 1;
            if (h == sInvalid || mesh.mHalfedges[h].mFace == sInvalid)
                continue;

            const uint16_t v2 = uint16_t(face2Vertex[mesh.mHalfedges[h].mFace]);

            if (v0 == v2)
                break;

            mIndices.push_back(v0);
            mIndices.push_back(v2);
            mIndices.push_back(v1);

            v1 = v2;
        }
    }
}

}  // namespace ephys::utils
