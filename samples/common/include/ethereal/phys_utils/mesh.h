#ifndef ETHEREAL_PHYS_UTILS_MESH_H
#define ETHEREAL_PHYS_UTILS_MESH_H

// Triangle meshes for the samples: FleX's core/mesh.h and mesh.cpp (the Mesh struct, the OBJ, PLY and flat
// binary readers, the procedural primitives) over Donut's math.
//
// Part of ethereal_phys_utils, the samples' utility library (samples/common): a static library that uses
// donut::math and std:: freely. It is not part of the engine DLL and its headers are not DLL boundary
// headers; the engine's asset builders take the arrays of a Mesh as plain pointers (m_positions is
// 3 floats per vertex, m_indices 3 per triangle).
//
// The member and function names are FleX's, so that the demo's scene code ports with few edits. What changed:
// a reader returns a std::unique_ptr (FleX returned a raw new), paths are std::filesystem::path, the readers
// print nothing, and Transform takes a Donut affine transform (row vectors) instead of FleX's Matrix44.

#include <donut/core/math/math.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace ephys::utils {

struct Mesh {
    void AddMesh(const Mesh& m);

    uint32_t GetNumVertices() const { return uint32_t(m_positions.size()); }
    uint32_t GetNumFaces() const { return uint32_t(m_indices.size()) / 3; }

    void DuplicateVertex(uint32_t i);

    // Vertex normals: the normalised sum of the (area-weighted) face normals around each vertex.
    void CalculateNormals();
    // Positions as points, normals as vectors (not renormalised, as in FleX).
    void Transform(const dm::affine3& m);
    // Scales and translates so that the lower bound is the origin and the largest dimension is s.
    void Normalize(float s = 1.0f);

    void GetBounds(dm::float3& minExtents, dm::float3& maxExtents) const;

    std::vector<dm::float3> m_positions;
    std::vector<dm::float3> m_normals;
    std::vector<dm::float2> m_texcoords[2];
    std::vector<dm::float4> m_colours;

    std::vector<uint32_t> m_indices;
};

// The readers return null when the file cannot be opened or is not of the format. ImportMesh chooses by the
// extension (.ply, .obj, .bin, case-insensitive).
//
// - OBJ: v, vt, vn and f (triangles and quads; a quad becomes two triangles); a vertex is created per distinct
//   v/vt/vn combination; groups, materials and smoothing groups are skipped; normals are computed from the
//   faces (FleX's reader replaces those of the file the same way, unless the file had them for every vertex).
// - PLY: ascii and binary_big_endian; the first three properties of a vertex are its position; faces of 3 or
//   4 vertices.
// - BIN: FleX's flat binary format (ExportMeshToBin): counts, positions, normals, indices.
std::unique_ptr<Mesh> ImportMesh(const std::filesystem::path& path);
std::unique_ptr<Mesh> ImportMeshFromObj(const std::filesystem::path& path);
std::unique_ptr<Mesh> ImportMeshFromPly(const std::filesystem::path& path);
std::unique_ptr<Mesh> ImportMeshFromBin(const std::filesystem::path& path);

// Writes FleX's flat binary format. False when the file cannot be written.
bool ExportMeshToBin(const std::filesystem::path& path, const Mesh& m);

// Procedural primitives (FleX's, with its tessellation and winding).
std::unique_ptr<Mesh> CreateTriMesh(float size, float y = 0.0f);
std::unique_ptr<Mesh> CreateCubeMesh();  // unit cube centred at the origin, 24 vertices
std::unique_ptr<Mesh> CreateQuadMesh(float size, float y = 0.0f);
std::unique_ptr<Mesh> CreateDiscMesh(float radius, uint32_t segments);
std::unique_ptr<Mesh> CreateTetrahedron(float ground = 0.0f, float height = 1.0f);
std::unique_ptr<Mesh> CreateSphere(int slices, int segments, float radius = 1.0f);
std::unique_ptr<Mesh> CreateCapsule(int slices, int segments, float radius = 1.0f, float halfHeight = 1.0f);

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_MESH_H */
