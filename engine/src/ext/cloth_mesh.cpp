#include "ext/cloth_mesh.h"

#include <algorithm>
#include <cassert>
#include <set>

// A port of reference/flex/core/cloth.h (see cloth_mesh.h for what was left out), in FleX's evaluation
// order. FleX's asserts on impossible states are kept as asserts.

namespace ephys::detail::flex {
namespace {

Vec4 ParticleAt(const float* particles, int i) { return Vec4(&particles[size_t(i) * 4]); }

}  // namespace

ClothMesh::Edge::Edge(int a, int b) {
    vertices[0] = Min(a, b);
    vertices[1] = Max(a, b);
    tris[0] = -1;
    tris[1] = -1;
    stretchConstraint = -1;
    bendingConstraint = -1;
}

void ClothMesh::Edge::Replace(int oldIndex, int newIndex) {
    if (vertices[0] == oldIndex)
        vertices[0] = newIndex;
    else if (vertices[1] == oldIndex)
        vertices[1] = newIndex;
    else
        assert(0);
}

void ClothMesh::Edge::RemoveTri(int index) {
    if (tris[0] == index)
        tris[0] = -1;
    else if (tris[1] == index)
        tris[1] = -1;
    else
        assert(0);
}

bool ClothMesh::Edge::AddTri(int index) {
    if (tris[0] == -1) {
        tris[0] = index;
        return true;
    }
    if (tris[1] == -1) {
        // the triangle must not reference the same edge twice
        if (index == tris[0])
            return false;
        tris[1] = index;
        return true;
    }
    return false;
}

ClothMesh::Triangle::Triangle(int a, int b, int c) {
    vertices[0] = a;
    vertices[1] = b;
    vertices[2] = c;
    edges[0] = -1;
    edges[1] = -1;
    edges[2] = -1;
    side = -1;
    component = -1;
}

void ClothMesh::Triangle::ReplaceEdge(int oldIndex, int newIndex) {
    for (int i = 0; i < 3; ++i) {
        if (edges[i] == oldIndex) {
            edges[i] = newIndex;
            return;
        }
    }
    assert(0);
}

int ClothMesh::Triangle::ReplaceVertex(int oldIndex, int newIndex) {
    for (int i = 0; i < 3; ++i) {
        if (vertices[i] == oldIndex) {
            vertices[i] = newIndex;
            return i;
        }
    }
    assert(0);
    return -1;
}

int ClothMesh::Triangle::GetOppositeVertex(int v0, int v1) const {
    for (int i = 0; i < 3; ++i) {
        if (vertices[i] != v0 && vertices[i] != v1)
            return vertices[i];
    }
    assert(0);
    return -1;
}

ClothMesh::ClothMesh(const float* particles, int numVertices, const int* indices, int numIndices,
                     float stretchStiffness, float bendStiffness) {
    mValid = false;
    mNumVertices = numVertices;

    // The simple bending model of tearable cloth, which lets a vertex split and its constraints be remapped.
    std::set<Edge> edges;

    // the unique edges
    for (int i = 0; i < numIndices; i += 3) {
        mTris.push_back(Triangle(indices[i + 0], indices[i + 1], indices[i + 2]));

        const int triIndex = i / 3;

        // FleX edits the elements of the set in place: tris is not part of the ordering.
        Edge& e1 = const_cast<Edge&>(*edges.insert(Edge(indices[i + 0], indices[i + 1])).first);
        Edge& e2 = const_cast<Edge&>(*edges.insert(Edge(indices[i + 1], indices[i + 2])).first);
        Edge& e3 = const_cast<Edge&>(*edges.insert(Edge(indices[i + 2], indices[i + 0])).first);

        if (!e1.AddTri(triIndex))
            return;
        if (!e2.AddTri(triIndex))
            return;
        if (!e3.AddTri(triIndex))
            return;
    }

    // the set as an array
    mEdges.assign(edges.begin(), edges.end());

    // the edges of each triangle
    for (int i = 0; i < numIndices; i += 3) {
        const int e1 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i + 0], indices[i + 1])) - mEdges.begin());
        const int e2 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i + 1], indices[i + 2])) - mEdges.begin());
        const int e3 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i + 2], indices[i + 0])) - mEdges.begin());

        if (e1 != e2 && e1 != e3 && e2 != e3) {
            const int triIndex = i / 3;
            mTris[size_t(triIndex)].edges[0] = e1;
            mTris[size_t(triIndex)].edges[1] = e2;
            mTris[size_t(triIndex)].edges[2] = e3;
        } else {
            // a degenerate triangle
            return;
        }
    }

    // the distance constraints
    for (size_t i = 0; i < mEdges.size(); ++i) {
        Edge& edge = mEdges[i];

        // stretch along the edge
        edge.stretchConstraint = AddConstraint(particles, edge.vertices[0], edge.vertices[1], stretchStiffness);

        const int t1 = edge.tris[0];
        const int t2 = edge.tris[1];

        // bending across the two triangles of the edge
        if (t1 != -1 && t2 != -1 && bendStiffness > 0.0f) {
            const int v1 = mTris[size_t(t1)].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
            const int v2 = mTris[size_t(t2)].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
            edge.bendingConstraint = AddConstraint(particles, v1, v2, bendStiffness);
        }
    }

    // the rest volume
    mRestVolume = 0.0f;
    mConstraintScale = 0.0f;

    std::vector<Vec3> gradients(static_cast<size_t>(numVertices));

    for (int i = 0; i < numIndices; i += 3) {
        const Vec3 v1 = XYZ(ParticleAt(particles, indices[i + 0]));
        const Vec3 v2 = XYZ(ParticleAt(particles, indices[i + 1]));
        const Vec3 v3 = XYZ(ParticleAt(particles, indices[i + 2]));

        const Vec3 n = Cross(v2 - v1, v3 - v1);
        const float signedVolume = Dot(v1, n);

        mRestVolume += signedVolume;

        gradients[size_t(indices[i + 0])] += n;
        gradients[size_t(indices[i + 1])] += n;
        gradients[size_t(indices[i + 2])] += n;
    }

    for (int i = 0; i < numVertices; ++i)
        mConstraintScale += Dot(gradients[size_t(i)], gradients[size_t(i)]);

    mConstraintScale = 1.0f / mConstraintScale;

    mValid = true;
}

int ClothMesh::AddConstraint(const float* particles, int a, int b, float stiffness, float give) {
    const int index = int(mConstraintRestLengths.size());

    mConstraintIndices.push_back(a);
    mConstraintIndices.push_back(b);

    const float restLength = Length(XYZ(ParticleAt(particles, a)) - XYZ(ParticleAt(particles, b)));

    mConstraintRestLengths.push_back(restLength * (1.0f + give));
    mConstraintCoefficients.push_back(stiffness);

    return index;
}

int ClothMesh::SeparateVertex(int singularVertex, std::vector<TriangleUpdate>& replacements,
                              std::vector<VertexCopy>& copies, int maxCopies) {
    std::vector<int> adjacentTriangles;

    // the triangles around the vertex
    for (int i = 0; i < int(mTris.size()); ++i) {
        if (mTris[size_t(i)].Contains(singularVertex))
            adjacentTriangles.push_back(i);
    }

    int componentCount = 0;

    // the first component keeps the vertex
    int newIndex = singularVertex;

    // colour the triangles connected through an edge of the vertex
    for (int i = 0; i < int(adjacentTriangles.size()); ++i) {
        if (maxCopies == 0)
            break;

        const int seed = adjacentTriangles[size_t(i)];

        // the triangle belongs to a component already
        if (mTris[size_t(seed)].component != -1)
            continue;

        std::vector<int> stack;
        stack.push_back(seed);

        while (!stack.empty()) {
            const int t = stack.back();
            stack.pop_back();

            Triangle& tri = mTris[size_t(t)];

            // back at the beginning: the component is fully connected
            if (tri.component == componentCount)
                break;

            assert(tri.component == -1);

            tri.component = componentCount;

            // the triangle
            const int v = tri.ReplaceVertex(singularVertex, newIndex);

            if (singularVertex != newIndex) {
                TriangleUpdate r;
                r.triangle = t * 3 + v;
                r.vertex = newIndex;
                replacements.push_back(r);
            }

            // its edges
            for (int e = 0; e < 3; ++e) {
                Edge& edge = mEdges[size_t(tri.edges[e])];

                if (edge.Contains(singularVertex)) {
                    // the edge goes to the new vertex
                    edge.Replace(singularVertex, newIndex);

                    const int stretching = edge.stretchConstraint;
                    if (mConstraintIndices[size_t(stretching) * 2 + 0] == singularVertex)
                        mConstraintIndices[size_t(stretching) * 2 + 0] = newIndex;
                    else if (mConstraintIndices[size_t(stretching) * 2 + 1] == singularVertex)
                        mConstraintIndices[size_t(stretching) * 2 + 1] = newIndex;
                    else
                        assert(0);

                    if (!edge.IsBoundary()) {
                        // the unprocessed neighbours
                        for (int s = 0; s < 2; ++s) {
                            assert(mTris[size_t(edge.tris[s])].component == -1 ||
                                   mTris[size_t(edge.tris[s])].component == componentCount);

                            if (edge.tris[s] != t && mTris[size_t(edge.tris[s])].component == -1)
                                stack.push_back(edge.tris[s]);
                        }
                    }
                } else {
                    const int bending = edge.bendingConstraint;

                    if (bending != -1) {
                        if (mConstraintIndices[size_t(bending) * 2 + 0] == singularVertex)
                            mConstraintIndices[size_t(bending) * 2 + 0] = newIndex;
                        else if (mConstraintIndices[size_t(bending) * 2 + 1] == singularVertex)
                            mConstraintIndices[size_t(bending) * 2 + 1] = newIndex;
                    }
                }
            }
        }

        // copy the vertex
        if (singularVertex != newIndex) {
            VertexCopy copy;
            copy.srcIndex = singularVertex;
            copy.destIndex = newIndex;
            copies.push_back(copy);

            mNumVertices++;
            maxCopies--;
        }

        // the component is done
        newIndex = mNumVertices;

        componentCount++;
    }

    // reset the components
    for (int i = 0; i < int(adjacentTriangles.size()); ++i)
        mTris[size_t(adjacentTriangles[size_t(i)])].component = -1;

    return componentCount;
}

int ClothMesh::SplitVertex(const float* particles, int index, Vec3 splitPlane, std::vector<int>& adjacentTris,
                           std::vector<int>& adjacentVertices, std::vector<TriangleUpdate>& replacements,
                           std::vector<VertexCopy>& copies, int maxCopies) {
    if (maxCopies == 0)
        return -1;

    // FleX: Dot(const Plane&, const Vector3&), the xyz of the particle
    const float w = Dot(XYZ(ParticleAt(particles, index)), splitPlane);

    int leftCount = 0;
    int rightCount = 0;

    const int newIndex = mNumVertices;

    // which side of the plane the centroid of each triangle around the vertex is on
    for (size_t i = 0; i < mTris.size(); ++i) {
        Triangle& tri = mTris[i];

        if (tri.Contains(index)) {
            const Vec4 centroid = (ParticleAt(particles, tri.vertices[0]) + ParticleAt(particles, tri.vertices[1]) +
                                   ParticleAt(particles, tri.vertices[2])) /
                                  3.0f;

            if (Dot(XYZ(centroid), splitPlane) < w) {
                tri.side = 1;
                ++leftCount;
            } else {
                tri.side = 0;
                ++rightCount;
            }

            adjacentTris.push_back(int(i));
            for (int v = 0; v < 3; ++v) {
                if (std::find(adjacentVertices.begin(), adjacentVertices.end(), tri.vertices[v]) ==
                    adjacentVertices.end())
                    adjacentVertices.push_back(tri.vertices[v]);
            }
        }
    }

    // all triangles on one side: nothing to split
    if (leftCount == 0 || rightCount == 0)
        return -1;

    // remap the triangles
    for (size_t i = 0; i < adjacentTris.size(); ++i) {
        const int triIndex = adjacentTris[i];

        Triangle& tri = mTris[size_t(triIndex)];

        // the triangles on the negative side of the plane get the new vertex
        if (tri.side == 0) {
            const int v = tri.ReplaceVertex(index, newIndex);

            TriangleUpdate update;
            update.triangle = triIndex * 3 + v;
            update.vertex = newIndex;
            replacements.push_back(update);

            // its edges and constraints
            for (int e = 0; e < 3; ++e) {
                Edge& edge = mEdges[size_t(tri.edges[e])];

                if (edge.Contains(index)) {
                    bool exposed = false;

                    if (edge.tris[0] != -1 && edge.tris[1] != -1) {
                        const Triangle& t1 = mTris[size_t(edge.tris[0])];
                        const Triangle& t2 = mTris[size_t(edge.tris[1])];

                        // Case 1: the two triangles of the edge are on opposite sides: a new exposed edge,
                        // whose stretch spring is the bending spring that crossed it
                        if (t1.side != t2.side) {
                            Edge newEdge(edge.vertices[0], edge.vertices[1]);
                            newEdge.Replace(index, newIndex);
                            newEdge.AddTri(triIndex);

                            // the old edge loses this triangle
                            edge.RemoveTri(triIndex);

                            assert(edge.bendingConstraint != -1);

                            newEdge.stretchConstraint = edge.bendingConstraint;

                            const size_t s = size_t(newEdge.stretchConstraint);
                            mConstraintIndices[s * 2 + 0] = newEdge.vertices[0];
                            mConstraintIndices[s * 2 + 1] = newEdge.vertices[1];
                            mConstraintCoefficients[s] = mConstraintCoefficients[size_t(edge.stretchConstraint)];
                            mConstraintRestLengths[s] = mConstraintRestLengths[size_t(edge.stretchConstraint)];

                            edge.bendingConstraint = -1;

                            // edge is not used after this: push_back may move it
                            tri.ReplaceEdge(tri.edges[e], int(mEdges.size()));
                            mEdges.push_back(newEdge);

                            exposed = true;
                        }
                    }

                    if (!exposed) {
                        // Case 2: both triangles on one side, or a boundary edge: remap the edge and its spring
                        edge.Replace(index, newIndex);

                        const int stretching = edge.stretchConstraint;
                        if (mConstraintIndices[size_t(stretching) * 2 + 0] == index)
                            mConstraintIndices[size_t(stretching) * 2 + 0] = newIndex;
                        else if (mConstraintIndices[size_t(stretching) * 2 + 1] == index)
                            mConstraintIndices[size_t(stretching) * 2 + 1] = newIndex;
                        else
                            assert(0);
                    }
                } else {
                    // Case 3: an edge of the triangle away from the vertex; a bending spring across it may
                    // reach the vertex
                    const int bending = edge.bendingConstraint;

                    if (bending != -1) {
                        if (mConstraintIndices[size_t(bending) * 2 + 0] == index)
                            mConstraintIndices[size_t(bending) * 2 + 0] = newIndex;
                        else if (mConstraintIndices[size_t(bending) * 2 + 1] == index)
                            mConstraintIndices[size_t(bending) * 2 + 1] = newIndex;
                    }
                }
            }
        }
    }

    // the copy of the vertex
    VertexCopy copy;
    copy.srcIndex = index;
    copy.destIndex = newIndex;
    copies.push_back(copy);

    mNumVertices++;

    return newIndex;
}

}  // namespace ephys::detail::flex
