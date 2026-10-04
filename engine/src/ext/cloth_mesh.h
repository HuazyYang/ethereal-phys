#ifndef ETHEREAL_PHYS_SRC_EXT_CLOTH_MESH_H
#define ETHEREAL_PHYS_SRC_EXT_CLOTH_MESH_H

// Module-private: FleX's ClothMesh (reference/flex/core/cloth.h), the cloth half of the asset builders.
//
// It turns an indexed triangle mesh into springs: one stretch spring per edge and one bending spring across
// each edge shared by two triangles. It keeps the topology (an edge list with the two triangles of each edge,
// the three edges of each triangle, and which spring each edge carries), so that a vertex can be split in
// two along a strained edge and the springs and triangles that referred to it remapped: the tearing of
// IPhysTearingCloth. The spring arrays are the asset's arrays; tearing edits them in place and never resizes
// them (a bending spring becomes the stretch spring of the new boundary edge).
//
// Ported: the "tearable" constructor (both of FleX's builders pass tearable = true; the other path builds no
// springs), AddConstraint, SplitVertex and SeparateVertex. Not ported: IsSingularVertex, which nothing calls.
// The particles are read as 4 floats each (x y z 1/mass), FleX's Vec4.

#include "ext/flex_core.h"

#include <cstdint>
#include <vector>

namespace ephys::detail::flex {

class ClothMesh {
 public:
    struct Edge {
        int vertices[2];
        int tris[2];

        int stretchConstraint;
        int bendingConstraint;

        Edge(int a, int b);

        bool IsBoundary() const { return tris[0] == -1 || tris[1] == -1; }
        bool Contains(int index) const { return (vertices[0] == index) || (vertices[1] == index); }
        void Replace(int oldIndex, int newIndex);
        void RemoveTri(int index);
        bool AddTri(int index);

        bool operator<(const Edge& rhs) const {
            if (vertices[0] != rhs.vertices[0])
                return vertices[0] < rhs.vertices[0];
            return vertices[1] < rhs.vertices[1];
        }
    };

    struct Triangle {
        int vertices[3];
        int edges[3];

        int side;               // used during splitting
        mutable int component;  // used during singular vertex removal

        Triangle(int a, int b, int c);

        bool Contains(int v) const { return vertices[0] == v || vertices[1] == v || vertices[2] == v; }
        void ReplaceEdge(int oldIndex, int newIndex);
        int ReplaceVertex(int oldIndex, int newIndex);
        int GetOppositeVertex(int v0, int v1) const;
    };

    struct TriangleUpdate {
        int triangle;  // a position in the index array: triangle * 3 + corner
        int vertex;
    };

    struct VertexCopy {
        int srcIndex;
        int destIndex;
    };

    // particles: numVertices x 4 floats. mValid is false when the mesh is not one ClothMesh accepts (an edge
    // with more than two triangles, a triangle with a repeated vertex or edge).
    ClothMesh(const float* particles, int numVertices, const int* indices, int numIndices, float stretchStiffness,
              float bendStiffness);

    int AddConstraint(const float* particles, int a, int b, float stiffness, float give = 0.0f);

    // Gives the triangles around singularVertex that are no longer connected through an edge a vertex each
    // (the first keeps it). Appends the triangle updates and vertex copies; at most maxCopies copies.
    int SeparateVertex(int singularVertex, std::vector<TriangleUpdate>& replacements, std::vector<VertexCopy>& copies,
                       int maxCopies);

    // Splits vertex index by splitPlane: the triangles on the positive side get a new vertex. Returns its index,
    // or -1 when all triangles are on one side or maxCopies is 0.
    int SplitVertex(const float* particles, int index, Vec3 splitPlane, std::vector<int>& adjacentTris,
                    std::vector<int>& adjacentVertices, std::vector<TriangleUpdate>& replacements,
                    std::vector<VertexCopy>& copies, int maxCopies);

    std::vector<int> mConstraintIndices;
    std::vector<float> mConstraintCoefficients;
    std::vector<float> mConstraintRestLengths;

    std::vector<Edge> mEdges;
    std::vector<Triangle> mTris;

    int mNumVertices = 0;

    float mRestVolume = 0.0f;
    float mConstraintScale = 0.0f;

    bool mValid = false;
};

}  // namespace ephys::detail::flex

#endif /* ETHEREAL_PHYS_SRC_EXT_CLOTH_MESH_H */
