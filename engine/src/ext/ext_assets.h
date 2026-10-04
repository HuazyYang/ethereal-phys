#ifndef ETHEREAL_PHYS_SRC_EXT_EXT_ASSETS_H
#define ETHEREAL_PHYS_SRC_EXT_EXT_ASSETS_H

// Module-private: the algorithms of FleX's extension asset functions (reference/flex/extensions/
// flexExtCloth.cpp, flexExtRigid.cpp, flexExtSoft.cpp) over std:: containers, and how the device creates the
// asset builder (ext/asset_builder.cpp wraps them in IPhysAssetBuilder, IPhysAsset and IPhysTearingCloth).
//
// The functions take arguments the builder has checked (pointers valid for their counts, indices in range)
// and do what FleX did with them, in FleX's evaluation order: their output is compared with FleX's
// extension library bit for bit (tests/ext_parity_test.cpp).

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>

#include "ext/cloth_mesh.h"
#include "ext/flex_core.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ephys::detail {

// The arrays of an NvFlexExtAsset, owned by the library.
struct AssetData {
    std::vector<float> particles;  // maxParticles * 4
    uint32_t numParticles = 0;
    uint32_t maxParticles = 0;

    std::vector<int32_t> springIndices;
    std::vector<float> springCoefficients;
    std::vector<float> springRestLengths;

    std::vector<int32_t> shapeIndices;
    std::vector<int32_t> shapeOffsets;
    std::vector<float> shapeCoefficients;
    std::vector<float> shapeCenters;  // 3 per shape
    std::vector<float> shapePlasticThresholds;
    std::vector<float> shapePlasticCreeps;

    std::vector<int32_t> triangleIndices;

    bool inflatable = false;
    float inflatableVolume = 0.0f;
    float inflatablePressure = 0.0f;
    float inflatableStiffness = 0.0f;
};

// A tearing cloth: FleX's FlexExtTearingClothAsset, whose spring arrays are those of its ClothMesh.
struct TearingClothData {
    std::unique_ptr<flex::ClothMesh> mesh;
    std::vector<float> particles;  // maxParticles * 4
    std::vector<int32_t> triangleIndices;
    uint32_t numParticles = 0;
    uint32_t maxParticles = 0;

    bool inflatable = false;
    float inflatableVolume = 0.0f;
    float inflatablePressure = 0.0f;
    float inflatableStiffness = 0.0f;
};

// NvFlexExtCreateWeldedMeshIndices. Returns the number of unique vertices.
int createWeldedMeshIndices(const float* vertices, int numVertices, int* uniqueIndices, int* originalToUniqueMap,
                            float threshold);

// NvFlexExtCreateClothFromMesh. False when ClothMesh rejects the mesh (FleX returned NULL).
bool createClothFromMesh(const float* particles, int numVertices, const int* indices, int numTriangles,
                         const ClothAssetParams& params, AssetData& asset);

// NvFlexExtCreateTearingClothFromMesh. False when ClothMesh rejects the mesh (FleX returned NULL).
bool createTearingClothFromMesh(const float* particles, int numParticles, int maxParticles, const int* indices,
                                int numTriangles, const TearingClothAssetParams& params, TearingClothData& cloth);

// NvFlexExtTearClothMesh, with the module's random generator passed in.
void tearClothMesh(TearingClothData& cloth, flex::Random& random, float maxStrain, int maxSplits,
                   TearingParticleClone* particleCopies, int* numParticleCopies, int maxCopies,
                   TearingMeshEdit* triangleEdits, int* numTriangleEdits, int maxEdits);

// NvFlexExtCreateRigidFromMesh. False when the voxelisation needs more than 64 voxels per side (FleX returned
// NULL); an asset without particles when the mesh occupies no voxel.
bool createRigidFromMesh(const float* vertices, int numVertices, const int* indices, int numTriangleIndices,
                         float spacing, float expand, AssetData& asset);

// NvFlexExtCreateSoftFromMesh, with the module's random generator passed in. False when the sampling produced
// no particle (FleX indexed the empty sample array).
bool createSoftFromMesh(const float* vertices, int numVertices, const int* indices, int numIndices,
                        const SoftAssetParams& params, flex::Random& random, AssetData& asset);

// NvFlexExtCreateSoftMeshSkinning.
void createSoftMeshSkinning(const float* vertices, int numVertices, const float* bones, int numBones, float falloff,
                            float maxDistance, float* skinningWeights, int* skinningIndices);

// IPhysDevice::createAssetBuilder. The builder keeps the device alive.
nvrhi::FRESULT createAssetBuilder(IPhysDevice* device, IPhysAssetBuilder** ppBuilder) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_EXT_EXT_ASSETS_H */
