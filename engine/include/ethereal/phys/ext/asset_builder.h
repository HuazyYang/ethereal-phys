#ifndef ETHEREAL_PHYS_EXT_ASSET_BUILDER_H
#define ETHEREAL_PHYS_EXT_ASSET_BUILDER_H

// IPhysAssetBuilder: builds cloth, rigid and soft body assets (ext/asset.h) from triangle meshes on the host
// (step 9 of the migration plan). It is created by the device (IPhysDevice::createAssetBuilder).
//
// It replaces the asset half of FleX's extension library (reference/flex/extensions/flexExtCloth.cpp,
// flexExtRigid.cpp, flexExtSoft.cpp): NvFlexExtCreateWeldedMeshIndices, NvFlexExtCreateClothFromMesh,
// NvFlexExtCreateTearingClothFromMesh, NvFlexExtCreateRigidFromMesh, NvFlexExtCreateSoftFromMesh and
// NvFlexExtCreateSoftMeshSkinning, one method each, with FleX's algorithms: the same inputs give the same
// bits. Nothing here touches the GPU.
//
// What changed:
//
// - A builder that fails returns an FE_* code and a message (the device's message callback) instead of NULL,
//   and checks what FleX did not: null arrays, indices out of range, degenerate input.
// - The tuning arguments of the cloth and soft body builders are structs with FleX's argument names.
// - FleX's extension module kept one random generator (core/maths.h: seed1, seed2), zero until
//   NvFlexExtCreateSoftFromMesh seeded it, and NvFlexExtTearClothMesh drew from it. A builder is that
//   module: it owns the generator, starting at zero; createSoftFromMesh seeds it as FleX did, and the tearing
//   cloths the builder creates draw from it. Two builders do not share a generator.
//
// Mesh input. vertices are x y z, 3 floats per vertex (FleX's Vec3), except for the cloth builders, which
// take particles, 4 floats each (x y z 1/mass). indices are three per triangle, into the vertices.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/ext/asset.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// The arguments of NvFlexExtCreateClothFromMesh after the mesh.
struct ClothAssetParams {
    float stretchStiffness = 1.0f;  // the coefficient of the springs along the edges of the mesh
    float bendStiffness = 1.0f;     // the coefficient of the springs across two triangles (0: none)
    // > 0: a tether (a spring with the coefficient -tetherStiffness, which resists stretching only) from
    // each particle that is not fixed to the nearest fixed particle (1/mass 0), when there is one.
    float tetherStiffness = 0.0f;
    float tetherGive = 0.0f;  // how much a tether may stretch before it acts, relative to its length
    // > 0: the asset is inflatable, with this over-pressure; the rest volume and the stiffness are computed.
    float pressure = 0.0f;
};

static_assert(sizeof(ClothAssetParams) == 20, "ClothAssetParams layout changed: bump c_HeaderVersion");
static_assert(offsetof(ClothAssetParams, pressure) == 16);

// The arguments of NvFlexExtCreateTearingClothFromMesh after the mesh: no tethers (they would not survive a
// split).
struct TearingClothAssetParams {
    float stretchStiffness = 1.0f;
    float bendStiffness = 1.0f;
    float pressure = 0.0f;
};

static_assert(sizeof(TearingClothAssetParams) == 12, "TearingClothAssetParams layout changed: bump c_HeaderVersion");
static_assert(offsetof(TearingClothAssetParams, pressure) == 8);

// The arguments of NvFlexExtCreateSoftFromMesh after the mesh. The defaults are those of the FleX demo's soft
// body instances (demo/scenes/softbody.h) at its particle radius 0.1.
struct SoftAssetParams {
    float particleSpacing = 0.1f;  // the spacing of the particles
    // The resolution of the voxelisation that samples the interior, relative to the particle spacing; 0 for a
    // mesh that is not closed (surface sampling only).
    float volumeSampling = 4.0f;
    // How many random samples are taken on the surface: 50000 * surfaceSampling (0: none, besides the
    // vertices). Draws from the builder's random generator, which this call seeds first.
    float surfaceSampling = 0.0f;
    float clusterSpacing = 0.1f;    // the spacing of the shape-matching clusters, at least the particle spacing
    float clusterRadius = 0.0f;     // the radius of a cluster (0: the cluster spacing); controls the overlap
    float clusterStiffness = 0.5f;  // the coefficient of every cluster
    float linkRadius = 0.0f;        // > 0: a spring between every two particles closer than this
    float linkStiffness = 1.0f;     // the coefficient of those springs
    float globalStiffness = 0.0f;   // > 0: one more cluster of all particles, with this coefficient
    float clusterPlasticThreshold = 0.0f;  // the deformation above which a cluster yields ...
    float clusterPlasticCreep = 0.0f;      // ... and how fast; 0: no plastic arrays
};

static_assert(sizeof(SoftAssetParams) == 44, "SoftAssetParams layout changed: bump c_HeaderVersion");
static_assert(offsetof(SoftAssetParams, clusterPlasticCreep) == 40);

// Builds assets from meshes (IPhysDevice::createAssetBuilder). Not thread-safe: one builder is used from one
// thread at a time, and so are the tearing cloths it created (they share its random generator). A builder
// keeps the device alive (it reports through the device's message callback). An asset keeps no device alive;
// a tearing cloth keeps the device's message callback, to report its own refusals.
//
// Every method checks its arguments before it builds: FE_INVALID_ARGS and one message for a null array with
// a count, a count of 0 where the method needs something, an index out of range, and the failures of FleX's
// algorithms named at each method. A failure creates nothing and writes no output.
NVRHI_IID(IPhysAssetBuilder, "b4b65bde-837b-4918-9097-8db8daefc246")
struct IPhysAssetBuilder : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysAssetBuilder)

    // NvFlexExtCreateWeldedMeshIndices: collapses the vertices closer than threshold to each other (a render
    // mesh with seams, split by normals or texture coordinates) into one, the pre-pass createClothFromMesh
    // expects. vertices: numVertices x y z. Writes the indices of the unique vertices to uniqueVerts (room for
    // numVertices), the unique index of every vertex to originalToUniqueMap (numVertices), and their number
    // to numUniqueVerts.
    virtual nvrhi::FRESULT createWeldedMeshIndices(const float* vertices, uint32_t numVertices,
                                                   int32_t* uniqueVerts, int32_t* originalToUniqueMap,
                                                   float threshold, uint32_t& numUniqueVerts) noexcept = 0;

    // NvFlexExtCreateClothFromMesh: a cloth of the mesh's particles, with a stretch spring per edge, a bending
    // spring across every interior edge, optional tethers and an optional volume constraint. The only builder
    // with tethers and give; weld the mesh first (createWeldedMeshIndices). particles: numParticles x y z
    // 1/mass. FE_INVALID_ARGS when the mesh is not a manifold that FleX's ClothMesh accepts (an edge shared by
    // more than two triangles, a triangle that repeats a vertex or an edge).
    virtual nvrhi::FRESULT createClothFromMesh(const float* particles, uint32_t numParticles,
                                               const int32_t* indices, uint32_t numTriangles,
                                               const ClothAssetParams& params, IPhysAsset** ppAsset) noexcept = 0;

    // NvFlexExtCreateTearingClothFromMesh: the cloth above without tethers, which keeps its topology so that
    // it can tear (IPhysTearingCloth). maxParticles (at least numParticles) bounds how many particles tearing
    // may add; the storage of the particles beyond numParticles is zero. Same failures as createClothFromMesh.
    virtual nvrhi::FRESULT createTearingClothFromMesh(const float* particles, uint32_t numParticles,
                                                      uint32_t maxParticles, const int32_t* indices,
                                                      uint32_t numTriangles, const TearingClothAssetParams& params,
                                                      IPhysTearingCloth** ppCloth) noexcept = 0;

    // NvFlexExtCreateRigidFromMesh: one shape-matching constraint of stiffness 1 over particles placed in the
    // voxels a closed mesh occupies, voxelised at the spacing, and moved by expand along the gradient of the
    // mesh's distance field (negative: inwards). numTriangleIndices is 3 per triangle. A mesh that occupies no
    // voxel gives an asset without particles, as in FleX. FE_INVALID_ARGS when the voxelisation would need
    // more than 64 voxels per side (FleX returned NULL).
    virtual nvrhi::FRESULT createRigidFromMesh(const float* vertices, uint32_t numVertices, const int32_t* indices,
                                               uint32_t numTriangleIndices, float spacing, float expand,
                                               IPhysAsset** ppAsset) noexcept = 0;

    // NvFlexExtCreateSoftFromMesh: particles sampled in the interior (a voxelisation) and on the surface of a
    // mesh, clustered into overlapping shape-matching constraints, with optional links (springs), an optional
    // global cluster and optional plastic deformation. numTriangleIndices is 3 per triangle. FE_INVALID_ARGS
    // when the sampling produces no particle (FleX indexed an empty array).
    virtual nvrhi::FRESULT createSoftFromMesh(const float* vertices, uint32_t numVertices, const int32_t* indices,
                                              uint32_t numTriangleIndices, const SoftAssetParams& params,
                                              IPhysAsset** ppAsset) noexcept = 0;

    // NvFlexExtCreateSoftMeshSkinning: linear blend skinning of a render mesh to bones (the shape centers of a
    // soft body asset, 3 floats each). For each of numVertices vertices, up to four bones within maxDistance,
    // weighted by 1 / (d^falloff + 1e-4) of the squared distance d and normalised; writes 4 weights to
    // skinningWeights and 4 bone indices (-1: unused) to skinningIndices per vertex.
    virtual nvrhi::FRESULT createSoftMeshSkinning(const float* vertices, uint32_t numVertices, const float* bones,
                                                  uint32_t numBones, float falloff, float maxDistance,
                                                  float* skinningWeights, int32_t* skinningIndices) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysAssetBuilder> PhysAssetBuilderHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_EXT_ASSET_BUILDER_H */
