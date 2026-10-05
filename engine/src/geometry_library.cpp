#include "bvh_builder.h"
#include "data_model.h"
#include "kernel_params.h"

#include <nvrhi/core/foundation.h>

#include <cstring>
#include <new>
#include <vector>

// IGeometryLibrary and the three asset kinds. FleX's Library::CreateTriangleMesh ... GetConvexMeshBounds
// (FleX/src/Library.cpp) and TriangleMeshData, ConvexMeshData, SDFData (src/Data.cpp).
//
// Ids are FleX's: a slot index plus 1. A triangle mesh takes the lowest free slot of a table that grows; a
// convex and a distance field take the top of a free-list stack over a fixed table, so the slot freed last
// is handed out first.
//
// Lifetime. An asset holds a reference to the library and a pointer to its implementation; the library
// holds plain pointers to its assets, which an asset clears in its destructor. The library holds a plain
// pointer to the device, which the device clears when it goes away (detachGeometryLibrary): the device
// owns the library, and a reference back would be a cycle.

namespace ephys {
namespace {

using detail::Bvh;
using detail::report;
using detail::slack;

class GeometryLibrary;

// ---- Assets ----------------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(TriangleMeshAsset, "86f376c2-9505-4762-90ea-52dc1d52f5d4")
class TriangleMeshAsset final : public nvrhi::ObjectImpl<ITriangleMeshAsset> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(TriangleMeshAsset)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(TriangleMeshAsset)
    NVRHI_IMPLEMENTS_INTERFACE(ITriangleMeshAsset)
    NVRHI_IMPLEMENTS_CLASS(TriangleMeshAsset)
    NVRHI_END_INTERFACE_TABLE()

    TriangleMeshAsset(GeometryLibrary* library, uint32_t slot);
    ~TriangleMeshAsset();

    uint32_t getId() const noexcept override { return m_Slot + 1; }
    uint32_t getNumVertices() const noexcept override { return m_NumVertices; }
    uint32_t getNumTriangles() const noexcept override { return m_NumTriangles; }
    const Bounds3& getBounds() const noexcept override { return m_Bounds; }

    nvrhi::FRESULT update(nvrhi::ICommandList* commandList, IPhysBuffer* vertices, IPhysBuffer* indices,
                          uint32_t numVertices, uint32_t numTriangles, const Bounds3& bounds) noexcept override;

    IPhysBuffer* getVertices() const noexcept override { return m_Vertices; }
    IPhysBuffer* getIndices() const noexcept override { return m_Indices; }
    uint32_t getNumBvhNodes() const noexcept override { return m_NumTriangles ? m_Bvh.numNodes : 0; }
    IPhysBuffer* getBvhNodeLowers() const noexcept override { return m_Bvh.nodeLowers; }
    IPhysBuffer* getBvhNodeUppers() const noexcept override { return m_Bvh.nodeUppers; }
    IPhysBuffer* getBvhRoot() const noexcept override { return m_Bvh.rootNode; }

 private:
    friend class GeometryLibrary;

    GeometryLibrary* m_Library;
    GeometryLibraryHandle m_LibraryRef;
    uint32_t m_Slot;
    PhysBufferHandle m_Vertices;
    PhysBufferHandle m_Indices;
    Bvh m_Bvh;
    uint32_t m_NumVertices = 0;
    uint32_t m_NumTriangles = 0;
    uint32_t m_MaxVertices = 0;
    uint32_t m_MaxTriangles = 0;
    Bounds3 m_Bounds;
};

NVRHI_CLASS_CLSID(SdfAsset, "a1201b99-88d9-41c9-bf53-dda116bb36a3")
class SdfAsset final : public nvrhi::ObjectImpl<ISdfAsset> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(SdfAsset)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(SdfAsset)
    NVRHI_IMPLEMENTS_INTERFACE(ISdfAsset)
    NVRHI_IMPLEMENTS_CLASS(SdfAsset)
    NVRHI_END_INTERFACE_TABLE()

    SdfAsset(GeometryLibrary* library, uint32_t slot);
    ~SdfAsset();

    uint32_t getId() const noexcept override { return m_Slot + 1; }
    uint32_t getWidth() const noexcept override { return m_Dim[0]; }
    uint32_t getHeight() const noexcept override { return m_Dim[1]; }
    uint32_t getDepth() const noexcept override { return m_Dim[2]; }

    nvrhi::FRESULT update(nvrhi::ICommandList* commandList, const float* field, uint32_t width, uint32_t height,
                          uint32_t depth) noexcept override;

    nvrhi::ITexture* getTexture() const noexcept override { return m_Texture; }

 private:
    friend class GeometryLibrary;

    GeometryLibrary* m_Library;
    GeometryLibraryHandle m_LibraryRef;
    uint32_t m_Slot;
    uint32_t m_Dim[3] = {0, 0, 0};
    nvrhi::TextureHandle m_Texture;
};

NVRHI_CLASS_CLSID(ConvexAsset, "d28908de-da83-4c17-8933-5083c12e183b")
class ConvexAsset final : public nvrhi::ObjectImpl<IConvexAsset> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(ConvexAsset)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(ConvexAsset)
    NVRHI_IMPLEMENTS_INTERFACE(IConvexAsset)
    NVRHI_IMPLEMENTS_CLASS(ConvexAsset)
    NVRHI_END_INTERFACE_TABLE()

    ConvexAsset(GeometryLibrary* library, uint32_t slot);
    ~ConvexAsset();

    uint32_t getId() const noexcept override { return m_Slot + 1; }
    uint32_t getNumPlanes() const noexcept override { return m_NumPlanes; }
    const Bounds3& getBounds() const noexcept override { return m_Bounds; }

    nvrhi::FRESULT update(nvrhi::ICommandList* commandList, IPhysBuffer* planes, uint32_t numPlanes,
                          const Bounds3& bounds) noexcept override;

 private:
    friend class GeometryLibrary;

    GeometryLibrary* m_Library;
    GeometryLibraryHandle m_LibraryRef;
    uint32_t m_Slot;
    uint32_t m_NumPlanes = 0;
    Bounds3 m_Bounds;
};

// ---- The library -----------------------------------------------------------------------------------------

// FleX's free-list stack over a fixed table (ConvexMeshData, SDFData): entries [count, capacity) are the
// free slots, and the slot freed last is at index count.
class SlotStack {
 public:
    explicit SlotStack(uint32_t capacity)
        : m_Free(capacity) {
        for (uint32_t i = 0; i < capacity; ++i)
            m_Free[i] = i;
    }

    [[nodiscard]] bool full() const { return m_Count == m_Free.size(); }
    uint32_t allocate() { return m_Free[m_Count++]; }
    void free(uint32_t slot) { m_Free[--m_Count] = slot; }

 private:
    std::vector<uint32_t> m_Free;
    uint32_t m_Count = 0;
};

NVRHI_CLASS_CLSID(GeometryLibrary, "cba615ee-95eb-48ef-a4da-dcc21759d373")
class GeometryLibrary final : public nvrhi::ObjectImpl<IGeometryLibrary> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(GeometryLibrary)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(GeometryLibrary)
    NVRHI_IMPLEMENTS_INTERFACE(IGeometryLibrary)
    NVRHI_IMPLEMENTS_CLASS(GeometryLibrary)
    NVRHI_END_INTERFACE_TABLE()

    GeometryLibrary(IPhysDevice* device, detail::KernelLibraryHost* host)
        : m_Device(device),
          m_Host(host),
          m_Callback(device->getDesc().messageCallback),
          m_Convexes(c_MaxConvexAssets, nullptr),
          m_ConvexSlots(c_MaxConvexAssets),
          m_SdfSlots(c_MaxSdfAssets) {}

    // The fixed tables (FleX creates them with the library).
    nvrhi::FRESULT init() {
        nvrhi::FRESULT result =
            createArray("ephys convex table", sizeof(ConvexMeshEntry), c_MaxConvexAssets, m_ConvexTable);
        if (NVRHI_SUCCEEDED(result))
            result =
                createArray("ephys convex planes", 16, c_MaxConvexAssets * c_MaxConvexPlanes, m_ConvexPlanes);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys sdf table", sizeof(SdfEntry), c_MaxSdfAssets, m_SdfTable);
        return result;
    }

    void detach() noexcept {
        m_Device = nullptr;
        m_Host = nullptr;
    }

    // ---- IGeometryLibrary ----

    nvrhi::FRESULT createTriangleMesh(ITriangleMeshAsset** ppMesh) noexcept override {
        if (!ppMesh)
            return nvrhi::FE_INVALID_ARGS;
        *ppMesh = nullptr;
        try {
            // TriangleMeshData::Allocate: the first free slot, or a new one.
            uint32_t slot = uint32_t(m_Meshes.size());
            for (uint32_t i = 0; i < m_Meshes.size(); ++i) {
                if (!m_Meshes[i]) {
                    slot = i;
                    break;
                }
            }
            if (slot == m_Meshes.size())
                m_Meshes.push_back(nullptr);
            TriangleMeshAsset* mesh = MAKE_RC_OBJ(TriangleMeshAsset, this, slot);
            m_Meshes[slot] = mesh;
            m_Dirty = true;
            *ppMesh = mesh;
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return nvrhi::FE_OUT_OF_MEMORY;
        } catch (...) {
            return nvrhi::FE_GENERIC_ERROR;
        }
    }

    nvrhi::FRESULT createSdf(ISdfAsset** ppSdf) noexcept override {
        if (!ppSdf)
            return nvrhi::FE_INVALID_ARGS;
        *ppSdf = nullptr;
        if (m_SdfSlots.full())
            return fail(nvrhi::FE_OUT_OF_MEMORY,
                        "IGeometryLibrary::createSdf: all %u distance fields are in use", c_MaxSdfAssets);
        try {
            SdfAsset* sdf = MAKE_RC_OBJ(SdfAsset, this, 0u);
            sdf->m_Slot = m_SdfSlots.allocate();
            m_Sdfs[sdf->m_Slot] = sdf;
            *ppSdf = sdf;
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return nvrhi::FE_OUT_OF_MEMORY;
        } catch (...) {
            return nvrhi::FE_GENERIC_ERROR;
        }
    }

    nvrhi::FRESULT createConvex(IConvexAsset** ppConvex) noexcept override {
        if (!ppConvex)
            return nvrhi::FE_INVALID_ARGS;
        *ppConvex = nullptr;
        if (m_ConvexSlots.full())
            return fail(nvrhi::FE_OUT_OF_MEMORY,
                        "IGeometryLibrary::createConvex: all %u convex meshes are in use", c_MaxConvexAssets);
        try {
            ConvexAsset* convex = MAKE_RC_OBJ(ConvexAsset, this, 0u);
            convex->m_Slot = m_ConvexSlots.allocate();
            m_Convexes[convex->m_Slot] = convex;
            *ppConvex = convex;
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return nvrhi::FE_OUT_OF_MEMORY;
        } catch (...) {
            return nvrhi::FE_GENERIC_ERROR;
        }
    }

    nvrhi::FRESULT getTriangleMesh(uint32_t id, ITriangleMeshAsset** ppMesh) noexcept override {
        if (!ppMesh)
            return nvrhi::FE_INVALID_ARGS;
        *ppMesh = nullptr;
        if (id == 0 || id > m_Meshes.size() || !m_Meshes[id - 1])
            return nvrhi::FE_NOT_FOUND;
        m_Meshes[id - 1]->AddRef();
        *ppMesh = m_Meshes[id - 1];
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT getSdf(uint32_t id, ISdfAsset** ppSdf) noexcept override {
        if (!ppSdf)
            return nvrhi::FE_INVALID_ARGS;
        *ppSdf = nullptr;
        if (id == 0 || id > c_MaxSdfAssets || !m_Sdfs[id - 1])
            return nvrhi::FE_NOT_FOUND;
        m_Sdfs[id - 1]->AddRef();
        *ppSdf = m_Sdfs[id - 1];
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT getConvex(uint32_t id, IConvexAsset** ppConvex) noexcept override {
        if (!ppConvex)
            return nvrhi::FE_INVALID_ARGS;
        *ppConvex = nullptr;
        if (id == 0 || id > c_MaxConvexAssets || !m_Convexes[id - 1])
            return nvrhi::FE_NOT_FOUND;
        m_Convexes[id - 1]->AddRef();
        *ppConvex = m_Convexes[id - 1];
        return nvrhi::FS_OK;
    }

    uint32_t getTriangleMeshIds(uint32_t* ids, uint32_t capacity) const noexcept override {
        return enumerate(m_Meshes.data(), uint32_t(m_Meshes.size()), ids, capacity);
    }
    uint32_t getSdfIds(uint32_t* ids, uint32_t capacity) const noexcept override {
        return enumerate(m_Sdfs, c_MaxSdfAssets, ids, capacity);
    }
    uint32_t getConvexIds(uint32_t* ids, uint32_t capacity) const noexcept override {
        return enumerate(m_Convexes.data(), c_MaxConvexAssets, ids, capacity);
    }

    nvrhi::FRESULT commit(nvrhi::ICommandList* commandList) noexcept override {
        if (!m_Dirty)
            return nvrhi::FS_OK;
        if (!m_Device)
            return nvrhi::FE_NOT_ALIVE_OBJECT;
        try {
            return flatten(commandList);
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IGeometryLibrary::commit: out of memory");
        }
    }

    bool needsCommit() const noexcept override { return m_Dirty; }

    IPhysBuffer* getConvexTable() const noexcept override { return m_ConvexTable; }
    IPhysBuffer* getConvexPlanes() const noexcept override { return m_ConvexPlanes; }
    IPhysBuffer* getSdfTable() const noexcept override { return m_SdfTable; }
    IPhysBuffer* getTriangleMeshTable() const noexcept override { return m_MeshTable; }
    IPhysBuffer* getTriangleVertices() const noexcept override { return m_MeshVertices; }
    IPhysBuffer* getTriangleIndices() const noexcept override { return m_MeshIndices; }
    IPhysBuffer* getTriangleBvhNodeLowers() const noexcept override { return m_MeshNodeLowers; }
    IPhysBuffer* getTriangleBvhNodeUppers() const noexcept override { return m_MeshNodeUppers; }
    IPhysBuffer* getTriangleBvhRoots() const noexcept override { return m_MeshRoots; }

    nvrhi::ITexture* getSdfTexture(uint32_t slot) const noexcept override {
        return slot < c_MaxSdfAssets && m_Sdfs[slot] ? m_Sdfs[slot]->m_Texture.Get() : nullptr;
    }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }

    // ---- For the assets ----

    void releaseTriangleMesh(uint32_t slot) noexcept {
        m_Meshes[slot] = nullptr;
        m_Dirty = true;  // its entry leaves the table with the next commit
    }

    void releaseSdf(SdfAsset* sdf) noexcept {
        if (m_Sdfs[sdf->m_Slot] != sdf)
            return;  // it never got a slot
        m_Sdfs[sdf->m_Slot] = nullptr;
        m_SdfSlots.free(sdf->m_Slot);
        if (sdf->m_Texture)
            ++m_LayoutRevision;
    }

    void releaseConvex(ConvexAsset* convex) noexcept {
        if (m_Convexes[convex->m_Slot] != convex)
            return;
        m_Convexes[convex->m_Slot] = nullptr;
        m_ConvexSlots.free(convex->m_Slot);
    }

    nvrhi::FRESULT updateTriangleMesh(TriangleMeshAsset& mesh, nvrhi::ICommandList* commandList,
                                      IPhysBuffer* vertices, IPhysBuffer* indices, uint32_t numVertices,
                                      uint32_t numTriangles, const Bounds3& bounds) noexcept {
        if (!m_Device)
            return nvrhi::FE_NOT_ALIVE_OBJECT;

        if (numVertices == 0 || numTriangles == 0) {
            // As in FleX: the mesh is empty, and its buffers stay.
            mesh.m_NumVertices = 0;
            mesh.m_NumTriangles = 0;
            m_Dirty = true;
            return nvrhi::FS_OK;
        }
        if (numVertices > detail::c_MaxArrayCount || numTriangles > detail::c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "ITriangleMeshAsset::update: %u vertices and %u triangles are too many", numVertices,
                        numTriangles);
        if (const char* problem = problemOfSource(vertices, 16, numVertices))
            return fail(nvrhi::FE_INVALID_ARGS, "ITriangleMeshAsset::update: vertices: %s", problem);
        if (const char* problem = problemOfSource(indices, 4, 3ull * numTriangles))
            return fail(nvrhi::FE_INVALID_ARGS, "ITriangleMeshAsset::update: indices: %s", problem);

        // The buffers of the mesh, and the scratch for the bounds of its triangles.
        PhysBufferHandle meshVertices = mesh.m_Vertices, meshIndices = mesh.m_Indices;
        uint32_t maxVertices = mesh.m_MaxVertices, maxTriangles = mesh.m_MaxTriangles;
        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (numVertices > maxVertices) {
            maxVertices = slack(numVertices);
            result = createArray("ephys triangle mesh vertices", 16, maxVertices, meshVertices);
        }
        if (NVRHI_SUCCEEDED(result) && numTriangles > maxTriangles) {
            maxTriangles = slack(numTriangles);
            result = createArray("ephys triangle mesh indices", 4, 3 * maxTriangles, meshIndices);
        }
        if (NVRHI_SUCCEEDED(result) && numTriangles > m_MaxTriangleBounds) {
            const uint32_t capacity = slack(numTriangles);
            PhysBufferHandle lowers, uppers;
            result = createArray("ephys triangle bounds lower", 16, capacity, lowers);
            if (NVRHI_SUCCEEDED(result))
                result = createArray("ephys triangle bounds upper", 16, capacity, uppers);
            if (NVRHI_SUCCEEDED(result)) {
                m_TriangleLowers = lowers;
                m_TriangleUppers = uppers;
                m_MaxTriangleBounds = capacity;
            }
        }
        if (NVRHI_FAILED(result))
            return fail(
                result,
                "ITriangleMeshAsset::update: cannot create the buffers for %u vertices and %u triangles",
                numVertices, numTriangles);

        CopyRegion region;
        region.elementCount = numVertices;
        result = m_Device->copyBuffer(commandList, meshVertices, vertices, region);
        if (NVRHI_FAILED(result))
            return result;
        region.elementCount = 3 * numTriangles;
        result = m_Device->copyBuffer(commandList, meshIndices, indices, region);
        if (NVRHI_FAILED(result))
            return result;

        mesh.m_Vertices = meshVertices;
        mesh.m_Indices = meshIndices;
        mesh.m_MaxVertices = maxVertices;
        mesh.m_MaxTriangles = maxTriangles;
        mesh.m_NumVertices = numVertices;
        mesh.m_NumTriangles = numTriangles;
        mesh.m_Bounds = bounds;
        m_Dirty = true;
        ++m_ContentRevision;

        // The bounds of the triangles, then the hierarchy over them: the one place where data dispatches.
        detail::BvhParams params;
        params.numItems = int32_t(numTriangles);
        KernelDispatchDesc dispatch;
        dispatch.kernel = KernelId::ComputeTriangleBounds;
        dispatch.gridDim[0] = (numTriangles + 255) / 256;
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);
        dispatch.readOnly[0] = meshVertices->getRHIBuffer();
        dispatch.readOnly[1] = meshIndices->getRHIBuffer();
        dispatch.readWrite[0] = m_TriangleLowers->getRHIBuffer();
        dispatch.readWrite[1] = m_TriangleUppers->getRHIBuffer();
        result = m_Device->getKernels()->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result))
            return failBuild(mesh, result);

        result = m_Builder.build(m_Device, commandList, mesh.m_Bvh, m_TriangleLowers->getRHIBuffer(),
                                 m_TriangleUppers->getRHIBuffer(), numTriangles, &bounds, nullptr);
        if (NVRHI_FAILED(result))
            return failBuild(mesh, result);
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT updateSdf(SdfAsset& sdf, nvrhi::ICommandList* commandList, const float* field,
                             uint32_t width, uint32_t height, uint32_t depth) noexcept {
        if (!m_Device)
            return nvrhi::FE_NOT_ALIVE_OBJECT;
        if (!field)
            return fail(nvrhi::FE_INVALID_ARGS, "ISdfAsset::update: field is null");
        if (width == 0 || height == 0 || depth == 0)
            return fail(nvrhi::FE_INVALID_ARGS, "ISdfAsset::update: a volume of %u x %u x %u voxels", width,
                        height, depth);
        // The pitches of the upload are 32-bit byte counts.
        if (uint64_t(width) * height * sizeof(float) > 0x7fffffffull ||
            uint64_t(width) * height * depth > 0x3fffffffull)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "ISdfAsset::update: a volume of %u x %u x %u voxels is too large", width, height,
                        depth);

        nvrhi::ICommandList* list = m_Host->recordingList(commandList);
        if (!list)
            return fail(nvrhi::FE_GENERIC_ERROR, "ISdfAsset::update: no command list");

        try {
            // SDFData::Update: a new texture for every update.
            nvrhi::TextureDesc desc;
            desc.dimension = nvrhi::TextureDimension::Texture3D;
            desc.width = width;
            desc.height = height;
            desc.depth = depth;
            desc.format = nvrhi::Format::R32_FLOAT;
            desc.debugName = "ephys sdf";
            // Only compute kernels sample it. NonPixelShaderResource, not ShaderResource: the latter includes
            // the pixel shader state, which a compute command list cannot leave or restore.
            desc.initialState = nvrhi::ResourceStates::NonPixelShaderResource;
            desc.keepInitialState = true;
            nvrhi::TextureHandle texture;
            const nvrhi::FRESULT created = m_Device->getRHIDevice()->createTexture(desc, &texture);
            if (NVRHI_FAILED(created) || !texture)
                return fail(NVRHI_FAILED(created) ? created : nvrhi::FE_GENERIC_ERROR,
                            "ISdfAsset::update: cannot create a texture of %u x %u x %u voxels", width, height,
                            depth);

            list->writeTexture(texture, 0, 0, field, size_t(width) * sizeof(float),
                               size_t(width) * height * sizeof(float));

            SdfEntry entry;
            entry.dim[0] = int32_t(width);
            entry.dim[1] = int32_t(height);
            entry.dim[2] = int32_t(depth);
            entry.invDim[0] = 1.0f / float(width);
            entry.invDim[1] = 1.0f / float(height);
            entry.invDim[2] = 1.0f / float(depth);
            const nvrhi::FRESULT result = m_Device->writeBuffer(commandList, m_SdfTable, &entry, sdf.m_Slot, 1);
            if (NVRHI_FAILED(result))
                return result;

            sdf.m_Texture = texture;
            sdf.m_Dim[0] = width;
            sdf.m_Dim[1] = height;
            sdf.m_Dim[2] = depth;
            ++m_LayoutRevision;
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "ISdfAsset::update: out of memory");
        }
    }

    nvrhi::FRESULT updateConvex(ConvexAsset& convex, nvrhi::ICommandList* commandList, IPhysBuffer* planes,
                                uint32_t numPlanes, const Bounds3& bounds) noexcept {
        if (!m_Device)
            return nvrhi::FE_NOT_ALIVE_OBJECT;
        if (numPlanes > c_MaxConvexPlanes)
            return fail(nvrhi::FE_INVALID_ARGS, "IConvexAsset::update: %u planes; a convex mesh has at most %u",
                        numPlanes, c_MaxConvexPlanes);
        if (numPlanes != 0)
            if (const char* problem = problemOfSource(planes, 16, numPlanes))
                return fail(nvrhi::FE_INVALID_ARGS, "IConvexAsset::update: planes: %s", problem);

        // ConvexMeshData::Update.
        ConvexMeshEntry entry;
        for (int i = 0; i < 3; ++i) {
            entry.lower[i] = bounds.lower[i];
            entry.upper[i] = bounds.upper[i];
        }
        entry.planeOffset = int32_t(c_MaxConvexPlanes * convex.m_Slot);
        entry.planeCount = int32_t(numPlanes);

        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (numPlanes != 0) {
            CopyRegion region;
            region.dstOffset = c_MaxConvexPlanes * convex.m_Slot;
            region.elementCount = numPlanes;
            result = m_Device->copyBuffer(commandList, m_ConvexPlanes, planes, region);
        }
        if (NVRHI_SUCCEEDED(result))
            result = m_Device->writeBuffer(commandList, m_ConvexTable, &entry, convex.m_Slot, 1);
        if (NVRHI_FAILED(result))
            return result;

        convex.m_NumPlanes = numPlanes;
        convex.m_Bounds = bounds;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

 private:
    nvrhi::FRESULT fail(nvrhi::FRESULT code, const char* format, ...) const noexcept {
        char text[512];
        va_list args;
        va_start(args, format);
        vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        report(m_Callback, PhysMessageSeverity::Error, "%s", text);
        return code;
    }

    // A mesh whose hierarchy could not be built is empty: a table must not point at half a hierarchy.
    nvrhi::FRESULT failBuild(TriangleMeshAsset& mesh, nvrhi::FRESULT code) noexcept {
        mesh.m_NumVertices = 0;
        mesh.m_NumTriangles = 0;
        return fail(
            code,
            "ITriangleMeshAsset::update: the bounding volume hierarchy could not be built; the mesh is empty");
    }

    static const char* problemOfSource(IPhysBuffer* buffer, uint32_t stride, uint64_t count) noexcept {
        if (!buffer)
            return "the buffer is null";
        const PhysBufferDesc& desc = buffer->getDesc();
        if (desc.elementStride != stride)
            return "the element stride of the buffer is not that of the array";
        if (count > desc.elementCount)
            return "the buffer holds fewer elements than the count";
        if (desc.use == BufferUse::Readback)
            return "a Readback buffer cannot be a copy source";
        return nullptr;
    }

    nvrhi::FRESULT createArray(const char* name, uint32_t stride, uint32_t count,
                               PhysBufferHandle& out) const noexcept {
        PhysBufferDesc desc;
        desc.elementCount = count;
        desc.elementStride = stride;
        desc.use = BufferUse::Structured;
        desc.debugName = name;
        PhysBufferHandle buffer;
        const nvrhi::FRESULT result = m_Device->createBuffer(desc, &buffer);
        if (NVRHI_FAILED(result))
            return result;
        out = buffer;
        return nvrhi::FS_OK;
    }

    template <typename Asset>
    static uint32_t enumerate(Asset* const* assets, uint32_t slots, uint32_t* ids, uint32_t capacity) noexcept {
        uint32_t count = 0;
        for (uint32_t i = 0; i < slots; ++i) {
            if (!assets[i])
                continue;
            if (ids && count < capacity)
                ids[count] = i + 1;
            ++count;
        }
        return count;
    }

    nvrhi::FRESULT copy(nvrhi::ICommandList* commandList, IPhysBuffer* dst, uint32_t dstOffset,
                        IPhysBuffer* src, uint32_t count) const noexcept {
        if (count == 0)
            return nvrhi::FS_OK;
        CopyRegion region;
        region.dstOffset = dstOffset;
        region.elementCount = count;
        return m_Device->copyBuffer(commandList, dst, src, region);
    }

    // TriangleMeshData::FlattenTriangleMeshes: the meshes one behind the other, and a table that says where
    // each one starts.
    nvrhi::FRESULT flatten(nvrhi::ICommandList* commandList) {
        const uint32_t numSlots = uint32_t(m_Meshes.size());
        uint64_t totalVertices = 0, totalTriangles = 0;
        for (const TriangleMeshAsset* mesh : m_Meshes) {
            if (!mesh)
                continue;
            totalVertices += mesh->m_NumVertices;
            totalTriangles += mesh->m_NumTriangles;
        }
        if (totalVertices > detail::c_MaxArrayCount || totalTriangles > detail::c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IGeometryLibrary::commit: the triangle meshes are too large together");

        // TriangleMeshData::Resize, into temporaries first.
        PhysBufferHandle vertices = m_MeshVertices, indices = m_MeshIndices, lowers = m_MeshNodeLowers,
                         uppers = m_MeshNodeUppers, roots = m_MeshRoots, table = m_MeshTable;
        uint32_t maxVertices = m_MaxMeshVertices, maxTriangles = m_MaxMeshTriangles, maxSlots = m_MaxMeshSlots;
        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (totalVertices > maxVertices) {
            maxVertices = slack(uint32_t(totalVertices));
            result = createArray("ephys triangle meshes vertices", 16, maxVertices, vertices);
        }
        if (NVRHI_SUCCEEDED(result) && totalTriangles > maxTriangles) {
            maxTriangles = slack(uint32_t(totalTriangles));
            result = createArray("ephys triangle meshes indices", 4, 3 * maxTriangles, indices);
            if (NVRHI_SUCCEEDED(result))
                result = createArray("ephys triangle meshes bvh lowers", sizeof(BvhNodeHalf), 2 * maxTriangles,
                                     lowers);
            if (NVRHI_SUCCEEDED(result))
                result = createArray("ephys triangle meshes bvh uppers", sizeof(BvhNodeHalf), 2 * maxTriangles,
                                     uppers);
        }
        if (NVRHI_SUCCEEDED(result) && numSlots > maxSlots) {
            maxSlots = slack(numSlots);
            result = createArray("ephys triangle meshes bvh roots", 4, maxSlots, roots);
            if (NVRHI_SUCCEEDED(result))
                result = createArray("ephys triangle mesh table", sizeof(TriangleMeshEntry), maxSlots, table);
        }
        if (NVRHI_FAILED(result))
            return fail(result, "IGeometryLibrary::commit: cannot create the triangle mesh arrays");

        if (vertices != m_MeshVertices || indices != m_MeshIndices || roots != m_MeshRoots)
            ++m_LayoutRevision;
        m_MeshVertices = vertices;
        m_MeshIndices = indices;
        m_MeshNodeLowers = lowers;
        m_MeshNodeUppers = uppers;
        m_MeshRoots = roots;
        m_MeshTable = table;
        m_MaxMeshVertices = maxVertices;
        m_MaxMeshTriangles = maxTriangles;
        m_MaxMeshSlots = maxSlots;

        std::vector<TriangleMeshEntry> entries(numSlots);
        uint32_t vertexStart = 0, indexStart = 0, nodeStart = 0;
        for (uint32_t i = 0; i < numSlots; ++i) {
            const TriangleMeshAsset* mesh = m_Meshes[i];
            if (!mesh || mesh->m_NumTriangles == 0)
                continue;
            const uint32_t numIndices = 3 * mesh->m_NumTriangles;
            const uint32_t numNodes = mesh->m_Bvh.numNodes;

            result = copy(commandList, m_MeshVertices, vertexStart, mesh->m_Vertices, mesh->m_NumVertices);
            if (NVRHI_SUCCEEDED(result))
                result = copy(commandList, m_MeshIndices, indexStart, mesh->m_Indices, numIndices);
            if (NVRHI_SUCCEEDED(result))
                result = copy(commandList, m_MeshNodeLowers, nodeStart, mesh->m_Bvh.nodeLowers, numNodes);
            if (NVRHI_SUCCEEDED(result))
                result = copy(commandList, m_MeshNodeUppers, nodeStart, mesh->m_Bvh.nodeUppers, numNodes);
            if (NVRHI_SUCCEEDED(result))
                result = copy(commandList, m_MeshRoots, i, mesh->m_Bvh.rootNode, 1);
            if (NVRHI_FAILED(result))
                return result;

            TriangleMeshEntry& entry = entries[i];
            for (int k = 0; k < 3; ++k) {
                entry.lower[k] = mesh->m_Bounds.lower[k];
                entry.upper[k] = mesh->m_Bounds.upper[k];
            }
            entry.vertexStart = int32_t(vertexStart);
            entry.indexStart = int32_t(indexStart);
            entry.nodeStart = int32_t(nodeStart);

            vertexStart += mesh->m_NumVertices;
            indexStart += numIndices;
            nodeStart += numNodes;
        }

        if (numSlots != 0) {
            result = m_Device->writeBuffer(commandList, m_MeshTable, entries.data(), 0, numSlots);
            if (NVRHI_FAILED(result))
                return result;
            ++m_ContentRevision;
        }
        m_Dirty = false;
        return nvrhi::FS_OK;
    }

    IPhysDevice* m_Device;              // null after detach(): the device owns the library
    detail::KernelLibraryHost* m_Host;  // as m_Device
    PhysMessageCallbackHandle m_Callback;

    // The assets: plain pointers, cleared by the destructor of the asset.
    std::vector<TriangleMeshAsset*> m_Meshes;  // by slot; grows
    std::vector<ConvexAsset*> m_Convexes;      // c_MaxConvexAssets slots
    SdfAsset* m_Sdfs[c_MaxSdfAssets] = {};
    SlotStack m_ConvexSlots;
    SlotStack m_SdfSlots;

    PhysBufferHandle m_ConvexTable;
    PhysBufferHandle m_ConvexPlanes;
    PhysBufferHandle m_SdfTable;

    // The flattened triangle meshes.
    PhysBufferHandle m_MeshTable;
    PhysBufferHandle m_MeshVertices;
    PhysBufferHandle m_MeshIndices;
    PhysBufferHandle m_MeshNodeLowers;
    PhysBufferHandle m_MeshNodeUppers;
    PhysBufferHandle m_MeshRoots;
    uint32_t m_MaxMeshVertices = 0;
    uint32_t m_MaxMeshTriangles = 0;
    uint32_t m_MaxMeshSlots = 0;
    bool m_Dirty = false;

    // The hierarchy builder and the bounds of the triangles of the mesh being updated (FleX's
    // mStaticTriLowers / mStaticTriUppers).
    detail::BvhBuilder m_Builder;
    PhysBufferHandle m_TriangleLowers;
    PhysBufferHandle m_TriangleUppers;
    uint32_t m_MaxTriangleBounds = 0;

    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

// ---- Assets, continued -------------------------------------------------------------------------------------

TriangleMeshAsset::TriangleMeshAsset(GeometryLibrary* library, uint32_t slot)
    : m_Library(library), m_LibraryRef(library), m_Slot(slot) {}

TriangleMeshAsset::~TriangleMeshAsset() { m_Library->releaseTriangleMesh(m_Slot); }

nvrhi::FRESULT TriangleMeshAsset::update(nvrhi::ICommandList* commandList, IPhysBuffer* vertices,
                                         IPhysBuffer* indices, uint32_t numVertices, uint32_t numTriangles,
                                         const Bounds3& bounds) noexcept {
    return m_Library->updateTriangleMesh(*this, commandList, vertices, indices, numVertices, numTriangles,
                                         bounds);
}

SdfAsset::SdfAsset(GeometryLibrary* library, uint32_t slot)
    : m_Library(library), m_LibraryRef(library), m_Slot(slot) {}

SdfAsset::~SdfAsset() { m_Library->releaseSdf(this); }

nvrhi::FRESULT SdfAsset::update(nvrhi::ICommandList* commandList, const float* field, uint32_t width,
                                uint32_t height, uint32_t depth) noexcept {
    return m_Library->updateSdf(*this, commandList, field, width, height, depth);
}

ConvexAsset::ConvexAsset(GeometryLibrary* library, uint32_t slot)
    : m_Library(library), m_LibraryRef(library), m_Slot(slot) {}

ConvexAsset::~ConvexAsset() { m_Library->releaseConvex(this); }

nvrhi::FRESULT ConvexAsset::update(nvrhi::ICommandList* commandList, IPhysBuffer* planes, uint32_t numPlanes,
                                   const Bounds3& bounds) noexcept {
    return m_Library->updateConvex(*this, commandList, planes, numPlanes, bounds);
}

}  // namespace

namespace detail {

nvrhi::FRESULT createGeometryLibrary(IPhysDevice* device, KernelLibraryHost* host,
                                     IGeometryLibrary** ppLibrary) noexcept {
    *ppLibrary = nullptr;
    try {
        nvrhi::AutoPtr<GeometryLibrary> library = nvrhi::TakeOver(MAKE_RC_OBJ(GeometryLibrary, device, host));
        const nvrhi::FRESULT result = library->init();
        if (NVRHI_FAILED(result)) {
            report(device->getDesc().messageCallback, PhysMessageSeverity::Error,
                   "the geometry library cannot create its tables");
            return result;
        }
        *ppLibrary = library.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

void detachGeometryLibrary(IGeometryLibrary* library) noexcept {
    // The library was created by this module: the cast needs no QueryInterface.
    if (library)
        static_cast<GeometryLibrary*>(library)->detach();
}

}  // namespace detail
}  // namespace ephys
