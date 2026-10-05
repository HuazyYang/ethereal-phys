#include "data_model.h"

#include <nvrhi/core/foundation.h>

#include <new>
#include <vector>

// ICollisionShapeSet. The data half of FleX's Solver::SetShapes and ShapeData (FleX/src/
// Solver.cpp, Data.cpp); the bounds of the shapes and the hierarchy over them (ShapeData::RebuildShapeData)
// are a solver's.

namespace ephys {
namespace {

using detail::c_MaxArrayCount;
using detail::DataObject;
using detail::slack;

NVRHI_CLASS_CLSID(CollisionShapeSet, "c3f32e58-9ffd-495a-b26e-36cdcadc5d58")
class CollisionShapeSet final : public nvrhi::ObjectImpl<ICollisionShapeSet>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(CollisionShapeSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(CollisionShapeSet)
    NVRHI_IMPLEMENTS_INTERFACE(ICollisionShapeSet)
    NVRHI_IMPLEMENTS_CLASS(CollisionShapeSet)
    NVRHI_END_INTERFACE_TABLE()

    explicit CollisionShapeSet(IPhysDevice* device)
        : DataObject(device), m_Library(device->getGeometry()) {}

    nvrhi::FRESULT reserve(uint32_t capacity) {
        if (capacity <= m_Max)
            return nvrhi::FS_OK;
        static const char* const names[c_Arrays] = {
            "ephys shape geometry",           "ephys shape positions",          "ephys shape rotations",
            "ephys shape previous positions", "ephys shape previous rotations", "ephys shape flags"};
        PhysBufferHandle arrays[c_Arrays];
        for (uint32_t i = 0; i < c_Arrays; ++i) {
            const nvrhi::FRESULT result = createArray(names[i], strideOf(i), capacity, arrays[i]);
            if (NVRHI_FAILED(result))
                return result;
        }
        for (uint32_t i = 0; i < c_Arrays; ++i)
            m_Arrays[i] = arrays[i];
        m_Max = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    void resetRevisions() { m_LayoutRevision = 0; }

    uint32_t getMaxShapes() const noexcept override { return m_Max; }
    uint32_t getNumShapes() const noexcept override { return m_Num; }
    IPhysBuffer* getGeometry() const noexcept override { return m_Arrays[Geometry]; }
    IPhysBuffer* getPositions() const noexcept override { return m_Arrays[Positions]; }
    IPhysBuffer* getRotations() const noexcept override { return m_Arrays[Rotations]; }
    IPhysBuffer* getPrevPositions() const noexcept override { return m_Arrays[PrevPositions]; }
    IPhysBuffer* getPrevRotations() const noexcept override { return m_Arrays[PrevRotations]; }
    IPhysBuffer* getFlags() const noexcept override { return m_Arrays[Flags]; }
    IGeometryLibrary* getGeometryLibrary() const noexcept override { return m_Library; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, const CollisionShapeData& data) noexcept override {
        const uint32_t numShapes = data.numShapes;
        if (numShapes == 0) {
            m_Num = 0;
            m_Assets.clear();
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        }
        if (numShapes > c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "ICollisionShapeSet::upload: %u shapes are too many",
                        numShapes);
        if (!data.geometry || !data.positions || !data.rotations || !data.flags)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "ICollisionShapeSet::upload: geometry, positions, rotations and flags are required");

        try {
            // The assets the shapes name. Each is held once, however many shapes use it.
            std::vector<nvrhi::AutoPtr<nvrhi::IObject>> assets;
            for (uint32_t i = 0; i < numShapes; ++i) {
                const uint32_t type = uint32_t(data.flags[i]) & uint32_t(ShapeFlagTypeMask);
                const CollisionGeometry& geometry = data.geometry[i];
                nvrhi::AutoPtr<nvrhi::IObject> asset;
                nvrhi::FRESULT found = nvrhi::FS_OK;
                uint32_t id = 0;
                const char* kind = nullptr;
                switch (CollisionShapeType(type)) {
                    case CollisionShapeType::Sphere:
                    case CollisionShapeType::Capsule:
                    case CollisionShapeType::Box:
                        continue;
                    case CollisionShapeType::ConvexMesh: {
                        ConvexAssetHandle convex;
                        id = geometry.convexMesh.mesh;
                        kind = "convex mesh";
                        found = m_Library->getConvex(id, &convex);
                        asset = convex.Get();
                        break;
                    }
                    case CollisionShapeType::TriangleMesh: {
                        TriangleMeshAssetHandle mesh;
                        id = geometry.triMesh.mesh;
                        kind = "triangle mesh";
                        found = m_Library->getTriangleMesh(id, &mesh);
                        asset = mesh.Get();
                        break;
                    }
                    case CollisionShapeType::Sdf: {
                        SdfAssetHandle sdf;
                        id = geometry.sdf.field;
                        kind = "distance field";
                        found = m_Library->getSdf(id, &sdf);
                        asset = sdf.Get();
                        break;
                    }
                    default:
                        return fail(nvrhi::FE_INVALID_ARGS,
                                    "ICollisionShapeSet::upload: shape %u: %u is not a shape type", i, type);
                }
                if (NVRHI_FAILED(found) || !asset)
                    return fail(nvrhi::FE_INVALID_ARGS,
                                "ICollisionShapeSet::upload: shape %u: no %s has the id %u", i, kind, id);
                bool held = false;
                for (const auto& other : assets)
                    held = held || other.Get() == asset.Get();
                if (!held)
                    assets.push_back(asset);
            }

            nvrhi::FRESULT result = numShapes > m_Max ? reserve(slack(numShapes)) : nvrhi::FS_OK;
            if (NVRHI_FAILED(result))
                return fail(result, "ICollisionShapeSet::upload: cannot create the buffers for %u shapes",
                            numShapes);

            // FleX: null previous transforms are the current ones.
            const void* const sources[c_Arrays] = {data.geometry,
                                                   data.positions,
                                                   data.rotations,
                                                   data.prevPositions ? data.prevPositions : data.positions,
                                                   data.prevRotations ? data.prevRotations : data.rotations,
                                                   data.flags};
            for (uint32_t i = 0; i < c_Arrays; ++i) {
                result = m_Device->writeBuffer(commandList, m_Arrays[i], sources[i], 0, numShapes);
                if (NVRHI_FAILED(result))
                    return result;
            }

            m_Assets.swap(assets);
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "ICollisionShapeSet::upload: out of memory");
        }

        m_Num = numShapes;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, const CollisionShapeBuffers& buffers,
                            uint32_t numShapes) noexcept override {
        if (numShapes > m_Num)
            return fail(nvrhi::FE_INVALID_ARGS, "ICollisionShapeSet::download: %u shapes; the set holds %u",
                        numShapes, m_Num);
        static const char* const names[c_Arrays] = {"geometry",      "positions",     "rotations",
                                                    "prevPositions", "prevRotations", "flags"};
        IPhysBuffer* const targets[c_Arrays] = {buffers.geometry,      buffers.positions,     buffers.rotations,
                                                buffers.prevPositions, buffers.prevRotations, buffers.flags};
        for (uint32_t i = 0; i < c_Arrays; ++i) {
            if (!targets[i])
                continue;
            if (const char* problem = problemOf(targets[i], strideOf(i), 0, numShapes, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ICollisionShapeSet::download: %s: %s", names[i], problem);
        }
        for (uint32_t i = 0; i < c_Arrays; ++i) {
            if (!targets[i])
                continue;
            const nvrhi::FRESULT result = copy(commandList, targets[i], 0, m_Arrays[i], 0, numShapes);
            if (NVRHI_FAILED(result))
                return result;
        }
        return nvrhi::FS_OK;
    }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }

 private:
    enum Array : uint32_t {
        Geometry,
        Positions,
        Rotations,
        PrevPositions,
        PrevRotations,
        Flags,
        c_Arrays
    };

    static uint32_t strideOf(uint32_t array) { return array == Flags ? 4u : 16u; }

    GeometryLibraryHandle m_Library;
    PhysBufferHandle m_Arrays[c_Arrays];
    // The assets the shapes of the last upload name: they live as long as the shapes use them.
    std::vector<nvrhi::AutoPtr<nvrhi::IObject>> m_Assets;
    uint32_t m_Max = 0;
    uint32_t m_Num = 0;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

}  // namespace

namespace detail {

nvrhi::FRESULT createCollisionShapeSet(IPhysDevice* device, uint32_t reserve,
                                       ICollisionShapeSet** ppSet) noexcept {
    if (!ppSet)
        return nvrhi::FE_INVALID_ARGS;
    *ppSet = nullptr;
    try {
        nvrhi::AutoPtr<CollisionShapeSet> set = nvrhi::TakeOver(MAKE_RC_OBJ(CollisionShapeSet, device));
        const nvrhi::FRESULT result = set->reserve(reserve);
        set->resetRevisions();
        if (NVRHI_FAILED(result)) {
            report(device->getDesc().messageCallback, PhysMessageSeverity::Error,
                   "createDomain: cannot create the collision shape set");
            return result;
        }
        *ppSet = set.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace detail
}  // namespace ephys
