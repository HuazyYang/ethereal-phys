#ifndef ETHEREAL_PHYS_HELPERS_SIM_BUFFERS_H
#define ETHEREAL_PHYS_HELPERS_SIM_BUFFERS_H

// SimBuffers: what a scene of the FleX demo authors, in host memory (the demo's SimBuffers in
// reference/flex/demo/main.cpp, plus the render content its helpers wrote into globals).
//
// Part of ethereal_phys_helpers (samples/helpers), the scene-authoring library of the samples: the FleX demo's
// helpers.h made a library. The functions of helpers.h write into a SimBuffers the way the demo's wrote into
// g_buffers; the sample framework (samples/framework) turns it into an engine domain and Donut scene-graph
// leaves (ethereal_phys_scene), and keeps the particle arrays as the host mirrors a scene's Update() reads and
// writes. The library dispatches nothing and owns no GPU object: a collision shape is a description here
// (ShapeDesc), and the framework creates the leaf (and, for a convex, the geometry asset) from it.
//
// Triangle meshes and distance fields are created once and shared by the shapes that name them, as FleX's
// NvFlexTriangleMeshId and NvFlexDistanceFieldId were: CreateTriangleMesh and CreateSDF (helpers.h) return a
// shared, immutable description (TriangleMeshId, DistanceFieldId), and the framework creates one geometry asset
// per description, however many shapes refer to it.
//
// What is not here, because it lives on the GPU only: FleX's densities, anisotropy, smoothed positions and
// diffuse particles (solver outputs and domain state, which the renderer binds without a host copy).
//
// The member names are FleX's (positions, rigidOffsets, ...), so that scene code ports with few edits. Vectors
// are donut::math: a FleX Vec4 is a dm::float4, a Vec3 a dm::float3, and a Quat a dm::float4 (x, y, z, w).

#include <ethereal/phys/collision.h>
#include <ethereal/phys/particles.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/pfm.h>

#include <donut/core/math/math.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace ephys::helpers {

using utils::Mesh;

// The number of particles a vertex of the skinned render mesh follows (FleX's g_numSkinWeights).
constexpr int c_NumSkinWeights = 4;

// A rope: the particles a tube is extruded along when it is drawn (FleX's Rope).
struct Rope {
    std::vector<int> mIndices;
};

// A triangle mesh for collision (FleX: NvFlexCreateTriangleMesh + NvFlexUpdateTriangleMesh in CreateTriangleMesh):
// the mesh, which is also what DrawShapes draws (FleX's g_meshes), and its bounds in its local space.
struct TriangleMeshData {
    Mesh mesh;
    dm::float3 lower = dm::float3(0.f);
    dm::float3 upper = dm::float3(0.f);
};
using TriangleMeshId = std::shared_ptr<const TriangleMeshData>;

// A distance field for collision (FleX: NvFlexCreateDistanceField + NvFlexUpdateDistanceField in CreateSDF): the
// volume over the unit cube (field.m_width^3 voxels, negative inside) and the mesh it was made from, normalised into
// that cube, which DrawShapes draws (FleX's g_fields).
struct DistanceFieldData {
    utils::PfmImage field;
    Mesh mesh;
};
using DistanceFieldId = std::shared_ptr<const DistanceFieldData>;

// One collision shape: an element of FleX's shape arrays (shapeGeometry, shapePositions, shapeRotations,
// shapePrevPositions, shapePrevRotations, shapeFlags), with the geometry as plain values instead of an
// NvFlexCollisionGeometry, because a convex, a triangle mesh or a distance field is a GPU asset the library
// does not create. The framework makes one PhysShapeLeaf per shape, on a node at position / rotation.
struct ShapeDesc {
    CollisionShapeType type = CollisionShapeType::Sphere;

    float radius = 0.0f;                       // Sphere, Capsule
    float halfHeight = 0.0f;                   // Capsule (along local x)
    dm::float3 halfExtents = dm::float3(0.f);  // Box

    // ConvexMesh: the planes (a x + b y + c z + d = 0, normals pointing out), their bounds and the hull as a
    // flat-shaded mesh to draw (FleX's AddRandomConvex put that mesh in g_convexes).
    std::vector<dm::float4> convexPlanes;
    dm::float3 convexLower = dm::float3(0.f);
    dm::float3 convexUpper = dm::float3(0.f);
    std::shared_ptr<const Mesh> renderMesh;

    dm::float3 scale = dm::float3(1.f);  // ConvexMesh, TriangleMesh (FleX's geo.convexMesh.scale, geo.triMesh.scale)

    TriangleMeshId triangleMesh;  // TriangleMesh (FleX's geo.triMesh.mesh)
    DistanceFieldId sdf;          // SDF (FleX's geo.sdf.field)
    float sdfScale = 1.0f;        // SDF: the edge of the unit cube of the field in the world (FleX's geo.sdf.scale)

    dm::float3 position = dm::float3(0.f);
    dm::float4 rotation = dm::float4(0.f, 0.f, 0.f, 1.f);  // quaternion x y z w
    dm::float3 prevPosition = dm::float3(0.f);
    dm::float4 prevRotation = dm::float4(0.f, 0.f, 0.f, 1.f);

    bool dynamic = false;
    bool trigger = false;
    int32_t channels = PhaseShapeChannelMask;  // PhaseShapeChannel* bits

    // FleX's shape flags word (NvFlexMakeShapeFlagsWithChannels, plus the trigger bit).
    [[nodiscard]] int32_t GetFlags() const {
        return makeShapeFlagsWithChannels(type, dynamic, channels) | (trigger ? ShapeFlagTrigger : 0);
    }
};

struct SimBuffers {
    // particles
    std::vector<dm::float4> positions;  // w: inverse mass
    std::vector<dm::float4> restPositions;
    std::vector<dm::float3> velocities;
    std::vector<int> phases;
    std::vector<dm::float4> normals;
    std::vector<int> activeIndices;

    // collision shapes; a scene that changes them after Initialize sets shapesChanged (FleX's UpdateShapes)
    std::vector<ShapeDesc> shapes;
    bool shapesChanged = false;

    // rigids (shape matching)
    std::vector<int> rigidOffsets;
    std::vector<int> rigidIndices;
    std::vector<int> rigidMeshSize;  // vertices of the skinned render mesh per rigid
    std::vector<float> rigidCoefficients;
    std::vector<float> rigidPlasticThresholds;
    std::vector<float> rigidPlasticCreeps;
    std::vector<dm::float4> rigidRotations;  // quaternion x y z w
    std::vector<dm::float3> rigidTranslations;
    std::vector<dm::float3> rigidLocalPositions;
    std::vector<dm::float4> rigidLocalNormals;

    // inflatables
    std::vector<int> inflatableTriOffsets;
    std::vector<int> inflatableTriCounts;
    std::vector<float> inflatableVolumes;
    std::vector<float> inflatableCoefficients;
    std::vector<float> inflatablePressures;

    // springs
    std::vector<int> springIndices;
    std::vector<float> springLengths;
    std::vector<float> springStiffness;

    // dynamic triangles
    std::vector<int> triangles;
    std::vector<dm::float3> triangleNormals;
    std::vector<dm::float3> uvs;

    // Render content the helpers author (FleX: the globals g_mesh, g_meshSkinIndices, g_meshSkinWeights,
    // g_meshRestPositions and g_ropes). mesh is skinned to the rigids by SkinMesh; meshRestPositions is the
    // mesh at creation, which the framework saves after Initialize.
    std::unique_ptr<Mesh> mesh;
    std::vector<int> meshSkinIndices;
    std::vector<float> meshSkinWeights;
    std::vector<dm::float3> meshRestPositions;
    std::vector<Rope> ropes;
};

}  // namespace ephys::helpers

#endif /* ETHEREAL_PHYS_HELPERS_SIM_BUFFERS_H */
