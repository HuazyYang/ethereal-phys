#include "ext/ext_assets.h"

#include "data_model.h"
#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <utility>

// IPhysAssetBuilder, IPhysAsset and IPhysTearingCloth (ext/asset_builder.h, ext/asset.h): the checks FleX's
// extension library did not make, around the ported algorithms of ext_cloth.cpp, ext_rigid.cpp and
// ext_soft.cpp.

namespace ephys {
namespace {

using detail::AssetData;
using detail::TearingClothData;
using detail::report;

template <typename T>
const T* viewOf(const std::vector<T>& v) noexcept {
    return v.empty() ? nullptr : v.data();
}

// Reports an error through a message callback and returns its code.
nvrhi::FRESULT failWith(IPhysMessageCallback* callback, nvrhi::FRESULT code, const char* format, ...) noexcept {
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    report(callback, PhysMessageSeverity::Error, "%s", text);
    return code;
}

NVRHI_CLASS_CLSID(PhysAsset, "df3544b7-1350-4b14-b6ef-c460939747bf")
class PhysAsset final : public nvrhi::ObjectImpl<IPhysAsset> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysAsset)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysAsset)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysAsset)
    NVRHI_IMPLEMENTS_CLASS(PhysAsset)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysAsset(AssetData&& data) : m_Data(std::move(data)) {}

    const float* getParticles() const noexcept override { return viewOf(m_Data.particles); }
    uint32_t getParticleCount() const noexcept override { return m_Data.numParticles; }
    uint32_t getMaxParticles() const noexcept override { return m_Data.maxParticles; }

    const int32_t* getSpringIndices() const noexcept override { return viewOf(m_Data.springIndices); }
    const float* getSpringCoefficients() const noexcept override { return viewOf(m_Data.springCoefficients); }
    const float* getSpringRestLengths() const noexcept override { return viewOf(m_Data.springRestLengths); }
    uint32_t getSpringCount() const noexcept override { return uint32_t(m_Data.springCoefficients.size()); }

    const int32_t* getShapeIndices() const noexcept override { return viewOf(m_Data.shapeIndices); }
    uint32_t getShapeIndexCount() const noexcept override { return uint32_t(m_Data.shapeIndices.size()); }
    const int32_t* getShapeOffsets() const noexcept override { return viewOf(m_Data.shapeOffsets); }
    const float* getShapeCoefficients() const noexcept override { return viewOf(m_Data.shapeCoefficients); }
    const float* getShapeCenters() const noexcept override { return viewOf(m_Data.shapeCenters); }
    uint32_t getShapeCount() const noexcept override { return uint32_t(m_Data.shapeOffsets.size()); }
    const float* getShapePlasticThresholds() const noexcept override { return viewOf(m_Data.shapePlasticThresholds); }
    const float* getShapePlasticCreeps() const noexcept override { return viewOf(m_Data.shapePlasticCreeps); }

    const int32_t* getTriangleIndices() const noexcept override { return viewOf(m_Data.triangleIndices); }
    uint32_t getTriangleCount() const noexcept override { return uint32_t(m_Data.triangleIndices.size() / 3); }

    bool isInflatable() const noexcept override { return m_Data.inflatable; }
    float getInflatableVolume() const noexcept override { return m_Data.inflatableVolume; }
    float getInflatablePressure() const noexcept override { return m_Data.inflatablePressure; }
    float getInflatableStiffness() const noexcept override { return m_Data.inflatableStiffness; }

 private:
    AssetData m_Data;
};

NVRHI_CLASS_CLSID(PhysTearingCloth, "27db1449-20c2-43ed-8f5c-b911b56bb9a1")
class PhysTearingCloth final : public nvrhi::ObjectImpl<IPhysTearingCloth> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysTearingCloth)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysTearingCloth)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysTearingCloth)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysAsset)
    NVRHI_IMPLEMENTS_CLASS(PhysTearingCloth)
    NVRHI_END_INTERFACE_TABLE()

    PhysTearingCloth(TearingClothData&& data, std::shared_ptr<detail::flex::Random> random,
                     IPhysMessageCallback* callback)
        : m_Data(std::move(data)), m_Random(std::move(random)), m_Callback(callback) {}

    const float* getParticles() const noexcept override { return viewOf(m_Data.particles); }
    uint32_t getParticleCount() const noexcept override { return m_Data.numParticles; }
    uint32_t getMaxParticles() const noexcept override { return m_Data.maxParticles; }

    // FleX's tearing asset pointed its spring arrays at those of its ClothMesh, as here.
    const int32_t* getSpringIndices() const noexcept override { return viewOf(m_Data.mesh->mConstraintIndices); }
    const float* getSpringCoefficients() const noexcept override {
        return viewOf(m_Data.mesh->mConstraintCoefficients);
    }
    const float* getSpringRestLengths() const noexcept override { return viewOf(m_Data.mesh->mConstraintRestLengths); }
    uint32_t getSpringCount() const noexcept override { return uint32_t(m_Data.mesh->mConstraintCoefficients.size()); }

    const int32_t* getShapeIndices() const noexcept override { return nullptr; }
    uint32_t getShapeIndexCount() const noexcept override { return 0; }
    const int32_t* getShapeOffsets() const noexcept override { return nullptr; }
    const float* getShapeCoefficients() const noexcept override { return nullptr; }
    const float* getShapeCenters() const noexcept override { return nullptr; }
    uint32_t getShapeCount() const noexcept override { return 0; }
    const float* getShapePlasticThresholds() const noexcept override { return nullptr; }
    const float* getShapePlasticCreeps() const noexcept override { return nullptr; }

    const int32_t* getTriangleIndices() const noexcept override { return viewOf(m_Data.triangleIndices); }
    uint32_t getTriangleCount() const noexcept override { return uint32_t(m_Data.triangleIndices.size() / 3); }

    bool isInflatable() const noexcept override { return m_Data.inflatable; }
    float getInflatableVolume() const noexcept override { return m_Data.inflatableVolume; }
    float getInflatablePressure() const noexcept override { return m_Data.inflatablePressure; }
    float getInflatableStiffness() const noexcept override { return m_Data.inflatableStiffness; }

    nvrhi::FRESULT setParticles(const float* particles, uint32_t first, uint32_t count) noexcept override {
        if (!particles && count)
            return failWith(m_Callback, nvrhi::FE_INVALID_ARGS, "IPhysTearingCloth::setParticles: null particles");
        if (uint64_t(first) + count > m_Data.maxParticles)
            return failWith(m_Callback, nvrhi::FE_INVALID_ARGS,
                            "IPhysTearingCloth::setParticles: particles %u .. %u exceed the %u of the asset", first,
                            first + count, m_Data.maxParticles);
        if (count)
            std::memcpy(&m_Data.particles[size_t(first) * 4], particles, size_t(count) * 4 * sizeof(float));
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT tearClothMesh(float maxStrain, uint32_t maxSplits, TearingParticleClone* particleCopies,
                                 uint32_t maxCopies, uint32_t& numParticleCopies, TearingMeshEdit* triangleEdits,
                                 uint32_t maxEdits, uint32_t& numTriangleEdits) noexcept override {
        numParticleCopies = 0;
        numTriangleEdits = 0;
        if ((!particleCopies && maxCopies) || (!triangleEdits && maxEdits))
            return failWith(m_Callback, nvrhi::FE_INVALID_ARGS,
                            "IPhysTearingCloth::tearClothMesh: a null output array with a capacity");
        try {
            int copies = 0, edits = 0;
            detail::tearClothMesh(m_Data, *m_Random, maxStrain, int(std::min<uint32_t>(maxSplits, INT32_MAX)),
                                  particleCopies, &copies, int(std::min<uint32_t>(maxCopies, INT32_MAX)),
                                  triangleEdits, &edits, int(std::min<uint32_t>(maxEdits, INT32_MAX)));
            numParticleCopies = uint32_t(copies);
            numTriangleEdits = uint32_t(edits);
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return failWith(m_Callback, nvrhi::FE_OUT_OF_MEMORY, "IPhysTearingCloth::tearClothMesh: out of memory");
        } catch (...) {
            return failWith(m_Callback, nvrhi::FE_GENERIC_ERROR, "IPhysTearingCloth::tearClothMesh: unexpected error");
        }
    }

 private:
    TearingClothData m_Data;
    std::shared_ptr<detail::flex::Random> m_Random;
    PhysMessageCallbackHandle m_Callback;
};

NVRHI_CLASS_CLSID(PhysAssetBuilder, "6337a9b9-5431-4e29-9cfa-306b31e1be9d")
class PhysAssetBuilder final : public nvrhi::ObjectImpl<IPhysAssetBuilder> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysAssetBuilder)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysAssetBuilder)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysAssetBuilder)
    NVRHI_IMPLEMENTS_CLASS(PhysAssetBuilder)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysAssetBuilder(IPhysDevice* device)
        : m_Device(device), m_Random(std::make_shared<detail::flex::Random>()) {}

    nvrhi::FRESULT createWeldedMeshIndices(const float* vertices, uint32_t numVertices, int32_t* uniqueVerts,
                                           int32_t* originalToUniqueMap, float threshold,
                                           uint32_t& numUniqueVerts) noexcept override {
        numUniqueVerts = 0;
        Call call(*this, "createWeldedMeshIndices");
        if (numVertices && (!vertices || !uniqueVerts || !originalToUniqueMap))
            return call.fail("a null array");
        if (!call.count("numVertices", numVertices) || !call.finite(vertices, numVertices, 3))
            return call.result();
        return call.run([&] {
            numUniqueVerts = uint32_t(
                detail::createWeldedMeshIndices(vertices, int(numVertices), uniqueVerts, originalToUniqueMap, threshold));
        });
    }

    nvrhi::FRESULT createClothFromMesh(const float* particles, uint32_t numParticles, const int32_t* indices,
                                       uint32_t numTriangles, const ClothAssetParams& params,
                                       IPhysAsset** ppAsset) noexcept override {
        if (ppAsset)
            *ppAsset = nullptr;
        Call call(*this, "createClothFromMesh");
        if (!ppAsset)
            return call.fail("ppAsset is null");
        if (!call.mesh(particles, numParticles, 4, indices, uint64_t(numTriangles) * 3))
            return call.result();
        return call.run([&] {
            AssetData data;
            if (!detail::createClothFromMesh(particles, int(numParticles), indices, int(numTriangles), params, data)) {
                call.fail("the mesh is not a manifold: an edge has more than two triangles, or a triangle repeats a "
                          "vertex or an edge");
                return;
            }
            *ppAsset = MAKE_RC_OBJ(PhysAsset, std::move(data));
        });
    }

    nvrhi::FRESULT createTearingClothFromMesh(const float* particles, uint32_t numParticles, uint32_t maxParticles,
                                              const int32_t* indices, uint32_t numTriangles,
                                              const TearingClothAssetParams& params,
                                              IPhysTearingCloth** ppCloth) noexcept override {
        if (ppCloth)
            *ppCloth = nullptr;
        Call call(*this, "createTearingClothFromMesh");
        if (!ppCloth)
            return call.fail("ppCloth is null");
        if (maxParticles < numParticles)
            return call.fail("maxParticles (%u) is below numParticles (%u)", maxParticles, numParticles);
        if (!call.count("maxParticles", maxParticles) ||
            !call.mesh(particles, numParticles, 4, indices, uint64_t(numTriangles) * 3))
            return call.result();
        return call.run([&] {
            TearingClothData data;
            if (!detail::createTearingClothFromMesh(particles, int(numParticles), int(maxParticles), indices,
                                                    int(numTriangles), params, data)) {
                call.fail("the mesh is not a manifold: an edge has more than two triangles, or a triangle repeats a "
                          "vertex or an edge");
                return;
            }
            *ppCloth = MAKE_RC_OBJ(PhysTearingCloth, std::move(data), m_Random, callback());
        });
    }

    nvrhi::FRESULT createRigidFromMesh(const float* vertices, uint32_t numVertices, const int32_t* indices,
                                       uint32_t numTriangleIndices, float spacing, float expand,
                                       IPhysAsset** ppAsset) noexcept override {
        if (ppAsset)
            *ppAsset = nullptr;
        Call call(*this, "createRigidFromMesh");
        if (!ppAsset)
            return call.fail("ppAsset is null");
        if (!call.triangleIndexCount(numTriangleIndices) ||
            !call.mesh(vertices, numVertices, 3, indices, numTriangleIndices))
            return call.result();
        if (!(spacing > 0.0f) || !std::isfinite(spacing) || !std::isfinite(expand))
            return call.fail("the spacing (%g) must be positive and finite, the expansion (%g) finite", spacing, expand);
        return call.run([&] {
            AssetData data;
            if (!detail::createRigidFromMesh(vertices, int(numVertices), indices, int(numTriangleIndices), spacing,
                                             expand, data)) {
                call.fail("the spacing %g needs more than 64 voxels per side for this mesh", spacing);
                return;
            }
            *ppAsset = MAKE_RC_OBJ(PhysAsset, std::move(data));
        });
    }

    nvrhi::FRESULT createSoftFromMesh(const float* vertices, uint32_t numVertices, const int32_t* indices,
                                      uint32_t numTriangleIndices, const SoftAssetParams& params,
                                      IPhysAsset** ppAsset) noexcept override {
        if (ppAsset)
            *ppAsset = nullptr;
        Call call(*this, "createSoftFromMesh");
        if (!ppAsset)
            return call.fail("ppAsset is null");
        if (!call.triangleIndexCount(numTriangleIndices) ||
            !call.mesh(vertices, numVertices, 3, indices, numTriangleIndices))
            return call.result();
        if (!(params.particleSpacing > 0.0f) || !std::isfinite(params.particleSpacing) ||
            !(params.volumeSampling >= 0.0f) || !std::isfinite(params.volumeSampling) ||
            !(params.surfaceSampling >= 0.0f) || !(params.surfaceSampling * 50000.0f < float(INT32_MAX)))
            return call.fail("the particle spacing (%g) must be positive, the volume (%g) and surface (%g) sampling "
                             "not negative, all finite",
                             params.particleSpacing, params.volumeSampling, params.surfaceSampling);
        return call.run([&] {
            AssetData data;
            if (!detail::createSoftFromMesh(vertices, int(numVertices), indices, int(numTriangleIndices), params,
                                            *m_Random, data)) {
                call.fail("the sampling produced no particle (volume sampling %g, surface sampling %g)",
                          params.volumeSampling, params.surfaceSampling);
                return;
            }
            *ppAsset = MAKE_RC_OBJ(PhysAsset, std::move(data));
        });
    }

    nvrhi::FRESULT createSoftMeshSkinning(const float* vertices, uint32_t numVertices, const float* bones,
                                          uint32_t numBones, float falloff, float maxDistance, float* skinningWeights,
                                          int32_t* skinningIndices) noexcept override {
        Call call(*this, "createSoftMeshSkinning");
        if (numVertices && (!vertices || !skinningWeights || !skinningIndices))
            return call.fail("a null array");
        if (numBones && !bones)
            return call.fail("bones is null");
        if (!call.count("numVertices", numVertices) || !call.count("numBones", numBones) ||
            !call.finite(vertices, numVertices, 3) || !call.finite(bones, numBones, 3))
            return call.result();
        return call.run([&] {
            detail::createSoftMeshSkinning(vertices, int(numVertices), bones, int(numBones), falloff, maxDistance,
                                           skinningWeights, skinningIndices);
        });
    }

 private:
    IPhysMessageCallback* callback() const noexcept { return m_Device->getDesc().messageCallback; }

    // The checks of one call; the first failure is reported once, with the name of the method.
    class Call {
     public:
        Call(const PhysAssetBuilder& builder, const char* method) : m_Callback(builder.callback()), m_Method(method) {}

        nvrhi::FRESULT result() const noexcept { return m_Result; }

        nvrhi::FRESULT fail(const char* format, ...) noexcept {
            if (NVRHI_FAILED(m_Result))
                return m_Result;
            char text[400];
            va_list args;
            va_start(args, format);
            vsnprintf(text, sizeof(text), format, args);
            va_end(args);
            report(m_Callback, PhysMessageSeverity::Error, "IPhysAssetBuilder::%s: %s", m_Method, text);
            m_Result = nvrhi::FE_INVALID_ARGS;
            return m_Result;
        }

        bool count(const char* name, uint64_t n) noexcept {
            if (n > detail::c_MaxArrayCount) {
                fail("%s (%llu) is too large", name, static_cast<unsigned long long>(n));
                return false;
            }
            return true;
        }

        bool triangleIndexCount(uint32_t numTriangleIndices) noexcept {
            if (numTriangleIndices % 3 != 0) {
                fail("numTriangleIndices (%u) is not a multiple of 3", numTriangleIndices);
                return false;
            }
            return true;
        }

        bool finite(const float* values, uint32_t count, uint32_t components) noexcept {
            for (uint64_t i = 0; i < uint64_t(count) * components; ++i) {
                if (!std::isfinite(values[i])) {
                    fail("element %llu is not finite", static_cast<unsigned long long>(i / components));
                    return false;
                }
            }
            return true;
        }

        // A mesh: numVertices vertices of `components` floats, numIndices indices into them, 3 per triangle.
        bool mesh(const float* vertices, uint32_t numVertices, uint32_t components, const int32_t* indices,
                  uint64_t numIndices) noexcept {
            if (numVertices == 0 || numIndices == 0) {
                fail("the mesh is empty (%u vertices, %llu triangles)", numVertices,
                     static_cast<unsigned long long>(numIndices / 3));
                return false;
            }
            if (!vertices || !indices) {
                fail("a null vertex or index array");
                return false;
            }
            if (!count("the vertex count", numVertices) || !count("the index count", numIndices))
                return false;
            if (!finite(vertices, numVertices, components))
                return false;
            for (uint64_t i = 0; i < numIndices; ++i) {
                if (indices[i] < 0 || uint32_t(indices[i]) >= numVertices) {
                    fail("index %llu (%d) is out of range for %u vertices", static_cast<unsigned long long>(i),
                         indices[i], numVertices);
                    return false;
                }
            }
            return true;
        }

        // Runs the build; a failure it reports through fail() stays the result.
        template <typename F>
        nvrhi::FRESULT run(F&& build) noexcept {
            try {
                build();
                return m_Result;
            } catch (const std::bad_alloc&) {
                report(m_Callback, PhysMessageSeverity::Error, "IPhysAssetBuilder::%s: out of memory", m_Method);
                return nvrhi::FE_OUT_OF_MEMORY;
            } catch (...) {
                report(m_Callback, PhysMessageSeverity::Error, "IPhysAssetBuilder::%s: unexpected error", m_Method);
                return nvrhi::FE_GENERIC_ERROR;
            }
        }

     private:
        IPhysMessageCallback* m_Callback;
        const char* m_Method;
        nvrhi::FRESULT m_Result = nvrhi::FS_OK;
    };

    PhysDeviceHandle m_Device;
    // FleX's seed1 and seed2 of the extension module: the builder's, shared with its tearing cloths.
    std::shared_ptr<detail::flex::Random> m_Random;
};

}  // namespace

nvrhi::FRESULT detail::createAssetBuilder(IPhysDevice* device, IPhysAssetBuilder** ppBuilder) noexcept {
    if (!ppBuilder)
        return nvrhi::FE_INVALID_ARGS;
    *ppBuilder = nullptr;
    try {
        *ppBuilder = MAKE_RC_OBJ(PhysAssetBuilder, device);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return failWith(device->getDesc().messageCallback, nvrhi::FE_OUT_OF_MEMORY, "createAssetBuilder: out of memory");
    } catch (...) {
        return failWith(device->getDesc().messageCallback, nvrhi::FE_GENERIC_ERROR,
                        "createAssetBuilder: unexpected error");
    }
}

}  // namespace ephys
