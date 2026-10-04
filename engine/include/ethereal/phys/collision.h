#ifndef ETHEREAL_PHYS_COLLISION_H
#define ETHEREAL_PHYS_COLLISION_H

// Collision geometry: the shapes of a domain (ICollisionShapeSet) and the geometry assets shapes refer to
// (ITriangleMeshAsset, ISdfAsset, IConvexAsset), kept by the geometry library of the device
// (IGeometryLibrary, IPhysDevice::getGeometry).
//
// They replace NvFlexSetShapes and the triangle mesh, distance field and convex mesh calls of
// NvFlexLibrary. What changed:
//
// - An asset is an object. NvFlexCreateTriangleMesh returned an id and NvFlexDestroyTriangleMesh freed it;
//   here the factory returns an interface and Release frees it. The id survives as getId(): it is what a
//   shape's geometry names the asset by, and what the kernels index the tables of the library with.
// - A shape set keeps the assets alive that its shapes name. In FleX, destroying a mesh that a shape still
//   used left the shape pointing at a freed table slot.
// - Bounds are kept on the host (NvFlexGetConvexMeshBounds read them back from the GPU).
//
// Data and operations. Everything here is data, with one exception the plan allows: updating a triangle
// mesh builds its bounding volume hierarchy (ITriangleMeshAsset::update), which dispatches kernels. That is
// where FleX builds it too (NvFlexUpdateTriangleMesh), and it is maintenance of the asset's own structure,
// not simulation. The bounding volume hierarchy over the SHAPES of a domain depends on the transforms of
// a step and is a solver's (FleX builds it inside the solver update).
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/params.h>
#include <ethereal/phys/particles.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// ---- Shape geometry ------------------------------------------------------------------------------------
// One 16-byte entry per shape, read by the kernels as a float4 (CollideShapes.hlsl, CollideTriangles.hlsl,
// TransformShapeBounds.hlsl: geometry.xyz is a scale or the extents, asint(geometry.w) the id of a convex
// or a triangle mesh, asint(geometry.y) the id of a distance field). The layouts are FleX's
// NvFlexCollisionGeometry and must stay what the kernels read.
//
// The structs have no default member initialisers: they are members of a union.

// A sphere around the origin of the shape.
struct SphereGeometry {
    float radius;
};

// A capsule along the x axis of the shape, centred at its origin.
struct CapsuleGeometry {
    float radius;
    float halfHeight;
};

// A box with the interior [-halfExtents, +halfExtents].
struct BoxGeometry {
    float halfExtents[3];
};

// A convex mesh, scaled per axis from its local space.
struct ConvexMeshGeometry {
    float scale[3];
    uint32_t mesh;  // IConvexAsset::getId()
};

// A triangle mesh, scaled per axis from its local space.
struct TriangleMeshGeometry {
    float scale[3];
    uint32_t mesh;  // ITriangleMeshAsset::getId()
};

// A signed distance field. Its local origin is the corner of the field at the first voxel, and the field
// covers [0, 1] in each dimension of its local space; scale is the width of the shape in world space.
struct SdfGeometry {
    float scale;
    uint32_t field;  // ISdfAsset::getId()
};

// The geometry of one shape; which member counts follows from the type in the flags of the shape.
union CollisionGeometry {
    SphereGeometry sphere;
    CapsuleGeometry capsule;
    BoxGeometry box;
    ConvexMeshGeometry convexMesh;
    TriangleMeshGeometry triMesh;
    SdfGeometry sdf;
    float raw[4];  // the entry as the kernels read it
};

static_assert(sizeof(SphereGeometry) == 4, "SphereGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(CapsuleGeometry) == 8, "CapsuleGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(BoxGeometry) == 12, "BoxGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(ConvexMeshGeometry) == 16, "ConvexMeshGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(TriangleMeshGeometry) == 16, "TriangleMeshGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(SdfGeometry) == 8, "SdfGeometry layout changed: bump c_HeaderVersion");
static_assert(sizeof(CollisionGeometry) == 16, "CollisionGeometry layout changed: bump c_HeaderVersion");
static_assert(alignof(CollisionGeometry) == 4);
static_assert(offsetof(CapsuleGeometry, halfHeight) == 4);
static_assert(offsetof(ConvexMeshGeometry, mesh) == 12);    // geometry.w
static_assert(offsetof(TriangleMeshGeometry, mesh) == 12);  // geometry.w
static_assert(offsetof(SdfGeometry, field) == 4);           // geometry.y

// The type of a shape: the low three bits of its flags.
enum class CollisionShapeType : uint8_t {
    Sphere = 0,
    Capsule = 1,
    Box = 2,
    ConvexMesh = 3,
    TriangleMesh = 4,
    Sdf = 5,
};

// The bits of the flags of a shape (FleX's NvFlexCollisionShapeFlags). Bits [24, 30] are the collision
// channels of the shape, in the positions of PhaseShapeChannel0..6 (particles.h).
enum CollisionShapeFlags : int32_t {
    ShapeFlagTypeMask = 0x7,  // the CollisionShapeType
    ShapeFlagDynamic = 0x8,   // a dynamic shape: lower priority than static shapes
    ShapeFlagTrigger = 0x10,  // a trigger volume: no collision response, but reported as a contact
};

// The flags of a shape: its type, whether it is dynamic, and a combination of PhaseShapeChannel*. A
// particle collides with the shape when they share a channel. (FleX's NvFlexMakeShapeFlagsWithChannels.)
constexpr int32_t makeShapeFlagsWithChannels(CollisionShapeType type, bool dynamic,
                                             int32_t shapeChannels) noexcept {
    return int32_t(type) | (dynamic ? int32_t(ShapeFlagDynamic) : 0) | shapeChannels;
}

// Flags with every channel set (FleX's NvFlexMakeShapeFlags).
constexpr int32_t makeShapeFlags(CollisionShapeType type, bool dynamic) noexcept {
    return makeShapeFlagsWithChannels(type, dynamic, PhaseShapeChannelMask);
}

static_assert(makeShapeFlags(CollisionShapeType::Sdf, true) == 0x7f00000d);

// ---- What the tables of the geometry library hold --------------------------------------------------------
// The elements of the buffers IGeometryLibrary and ITriangleMeshAsset expose, as the kernels declare them
// (engine/shaders/KernelParams.hlsli, bvh/BVHCommon.hlsli). Published so that a tool or a test can read
// the buffers back; an application that only simulates never needs them.

// An axis-aligned box in host memory.
struct Bounds3 {
    float lower[3] = {0.0f, 0.0f, 0.0f};
    float upper[3] = {0.0f, 0.0f, 0.0f};
};

static_assert(sizeof(Bounds3) == 24, "Bounds3 layout changed: bump c_HeaderVersion");
static_assert(offsetof(Bounds3, upper) == 12);  // after lower[3]

// One element of the convex table (FlexConvexMeshDevice).
struct ConvexMeshEntry {
    float lower[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // the bounds; w unused
    float upper[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t planeOffset = 0;  // the first plane of the convex in the plane buffer
    int32_t planeCount = 0;
    int32_t _pad[2] = {0, 0};
};

static_assert(sizeof(ConvexMeshEntry) == 48,
              "ConvexMeshEntry layout changed: bump c_HeaderVersion (and the kernels)");
static_assert(offsetof(ConvexMeshEntry, upper) == 16);
static_assert(offsetof(ConvexMeshEntry, planeOffset) == 32);  // after upper[4]
static_assert(offsetof(ConvexMeshEntry, _pad) == 40);

// One element of the distance field table (FlexSDFDevice).
struct SdfEntry {
    int32_t dim[4] = {0, 0, 0, 0};               // the voxels in x, y, z; w unused
    float invDim[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // 1 / dim
};

static_assert(sizeof(SdfEntry) == 32, "SdfEntry layout changed: bump c_HeaderVersion (and the kernels)");
static_assert(offsetof(SdfEntry, invDim) == 16);  // after dim[4]

// One element of the triangle mesh table (FlexTriangleMeshDevice): where the mesh starts in the flattened
// arrays of the library.
struct TriangleMeshEntry {
    float lower[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // the bounds; w unused
    float upper[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t indexStart = 0;   // into the flattened index array
    int32_t vertexStart = 0;  // into the flattened vertex array
    int32_t nodeStart = 0;    // into the flattened node arrays
    int32_t _pad = 0;
};

static_assert(sizeof(TriangleMeshEntry) == 48,
              "TriangleMeshEntry layout changed: bump c_HeaderVersion (and the kernels)");
static_assert(offsetof(TriangleMeshEntry, upper) == 16);
static_assert(offsetof(TriangleMeshEntry, indexStart) == 32);  // after upper[4]
static_assert(offsetof(TriangleMeshEntry, _pad) == 44);

// One half of a node of a bounding volume hierarchy (PackedNodeHalf): the nodes are two parallel arrays,
// the lower corners and the upper corners.
//
// A hierarchy over n items has 2n - 1 nodes. Nodes [0, n) are the leaves, in Morton order; nodes
// [n, 2n - 1) are internal. For a leaf, the index of the lower half has bit 31 set and the index of the
// upper half is the item (the triangle). For an internal node, the index of the lower half is its left
// child and that of the upper half its right child.
struct BvhNodeHalf {
    float corner[3] = {0.0f, 0.0f, 0.0f};
    uint32_t index = 0;
};

static_assert(sizeof(BvhNodeHalf) == 16, "BvhNodeHalf layout changed: bump c_HeaderVersion (and the kernels)");
static_assert(offsetof(BvhNodeHalf, index) == 12);  // after corner[3]

static constexpr uint32_t c_BvhLeafBit = 0x80000000u;  // BvhNodeHalf::index of the lower half of a leaf

// The limits of the geometry library, those of FleX.
static constexpr uint32_t c_MaxConvexAssets = 4096;  // convex meshes alive at a time
static constexpr uint32_t c_MaxConvexPlanes = 64;    // planes of one convex mesh
static constexpr uint32_t c_MaxSdfAssets = 16;       // distance fields alive at a time: the kernels bind one
                                                     // texture per field (CollideShapes, t15..t30)

// ---- Assets --------------------------------------------------------------------------------------------
// An asset is created by IGeometryLibrary and lives until its last reference is released; its id is then
// free for the next asset of its kind. The asset keeps the library alive. A shape set holds a reference to
// every asset its shapes name.
//
// An id is not 0: 0 in a geometry entry means "no asset".

// A static triangle mesh for particles to collide with, and its bounding volume hierarchy.
NVRHI_IID(ITriangleMeshAsset, "200072cc-3133-4d07-9872-928ab36bacd1")
struct ITriangleMeshAsset : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(ITriangleMeshAsset)

    // The id of the mesh in the library: what TriangleMeshGeometry::mesh holds. It does not change while
    // the asset lives. The library gives a new mesh the lowest id that is free, as FleX does.
    [[nodiscard]] virtual uint32_t getId() const noexcept = 0;

    [[nodiscard]] virtual uint32_t getNumVertices() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getNumTriangles() const noexcept = 0;

    // The bounds given to update, in the local space of the mesh (NvFlexGetTriangleMeshBounds). A view of
    // what the asset keeps; zero before the first update.
    [[nodiscard]] virtual const Bounds3& getBounds() const noexcept = 0;

    // Replaces the mesh (NvFlexUpdateTriangleMesh): records copies of numVertices vertices (float4
    // positions, 16 bytes) and 3 * numTriangles indices (int32) from the start of the two buffers, then
    // the kernels that compute the bounds of the triangles and build the bounding volume hierarchy over
    // them. bounds must contain the mesh: the hierarchy is built on a grid over it. The buffers of the
    // asset are replaced by larger ones when the mesh has grown.
    //
    // The tables of the library are stale afterwards until IGeometryLibrary::commit.
    //
    // With numVertices or numTriangles 0 the mesh becomes empty and nothing is recorded. FE_INVALID_ARGS
    // and a message for a null buffer, a stride that is not 16 / 4 bytes, a buffer that holds fewer
    // elements than the count, or a Readback buffer; FE_NOT_ALIVE_OBJECT after the device was released;
    // the codes of IPhysKernelLibrary::dispatch when a kernel cannot be loaded.
    virtual nvrhi::FRESULT update(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* vertices,
                                  _In_opt_ IPhysBuffer* indices, uint32_t numVertices, uint32_t numTriangles,
                                  const Bounds3& bounds) noexcept = 0;

    // The buffers of the mesh; they do not AddRef, and they are nullptr before the first update. Vertices
    // are float4, indices int32 (3 per triangle, into the vertices of this mesh).
    [[nodiscard]] virtual IPhysBuffer* getVertices() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getIndices() const noexcept = 0;

    // The bounding volume hierarchy over the triangles: two arrays of BvhNodeHalf with getNumBvhNodes()
    // elements in use, and one int32, the index of the root node. getNumBvhNodes() is 2 * getNumTriangles()
    // (as in FleX: the last of them is not a node, a hierarchy has 2n - 1).
    [[nodiscard]] virtual uint32_t getNumBvhNodes() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getBvhNodeLowers() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getBvhNodeUppers() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getBvhRoot() const noexcept = 0;
};
typedef nvrhi::AutoPtr<ITriangleMeshAsset> TriangleMeshAssetHandle;

// A signed distance field: a volume of distances to a surface, negative inside, as a 3D texture.
NVRHI_IID(ISdfAsset, "7926d44e-e488-4ea1-8646-a92019337d34")
struct ISdfAsset : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(ISdfAsset)

    // The id of the field in the library: what SdfGeometry::field holds. The library hands out the id
    // that was freed last, as FleX does.
    [[nodiscard]] virtual uint32_t getId() const noexcept = 0;

    // The size of the volume in voxels; 0 before the first update.
    [[nodiscard]] virtual uint32_t getWidth() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getHeight() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getDepth() const noexcept = 0;

    // Replaces the volume (NvFlexUpdateDistanceField): creates a 3D texture of width x height x depth
    // voxels (nvrhi::Format::R32_FLOAT) and records the upload of field into it. field holds
    // width * height * depth floats, the voxel at (x, y, z) at field[z * width * height + y * width + x],
    // and is read before the call returns.
    //
    // field is host memory, not an IPhysBuffer: nvrhi fills a texture from host memory
    // (ICommandList::writeTexture) and has no copy from a buffer to a texture.
    //
    // FE_INVALID_ARGS and a message for a null field or a dimension of 0; FE_NOT_ALIVE_OBJECT after the
    // device was released; FE_GENERIC_ERROR when the texture cannot be created.
    virtual nvrhi::FRESULT update(_In_opt_ nvrhi::ICommandList* commandList, const float* field, uint32_t width,
                                  uint32_t height, uint32_t depth) noexcept = 0;

    // The texture: what a kernel samples. Does not AddRef; nullptr before the first update, and replaced
    // by every update.
    [[nodiscard]] virtual nvrhi::ITexture* getTexture() const noexcept = 0;
};
typedef nvrhi::AutoPtr<ISdfAsset> SdfAssetHandle;

// A convex mesh: up to c_MaxConvexPlanes planes a * x + b * y + c * z + d = 0; particles are kept on
// their outside.
NVRHI_IID(IConvexAsset, "1ca3466a-07f8-43fa-a059-6cf5697c1c64")
struct IConvexAsset : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IConvexAsset)

    // The id of the convex in the library: what ConvexMeshGeometry::mesh holds. The library hands out
    // the id that was freed last, as FleX does. The planes of the convex are the c_MaxConvexPlanes
    // elements of IGeometryLibrary::getConvexPlanes() from (getId() - 1) * c_MaxConvexPlanes on.
    [[nodiscard]] virtual uint32_t getId() const noexcept = 0;

    [[nodiscard]] virtual uint32_t getNumPlanes() const noexcept = 0;

    // The bounds given to update, in the local space of the convex (NvFlexGetConvexMeshBounds). A view of
    // what the asset keeps; zero before the first update.
    [[nodiscard]] virtual const Bounds3& getBounds() const noexcept = 0;

    // Replaces the convex (NvFlexUpdateConvexMesh): records a copy of numPlanes planes (float4, 16 bytes)
    // from the start of planes into the plane buffer of the library and a write of its table entry.
    // FE_INVALID_ARGS and a message for more than c_MaxConvexPlanes planes, a null buffer with a count,
    // another stride, too few elements or a Readback buffer; FE_NOT_ALIVE_OBJECT after the device was
    // released.
    virtual nvrhi::FRESULT update(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* planes,
                                  uint32_t numPlanes, const Bounds3& bounds) noexcept = 0;
};
typedef nvrhi::AutoPtr<IConvexAsset> ConvexAssetHandle;

// ---- The geometry library --------------------------------------------------------------------------------
// The geometry assets of a device and the GPU tables the collision kernels read them from
// (IPhysDevice::getGeometry). It exists because an asset is named by an id inside a geometry entry, and
// the kernels turn that id into an index: something has to hand out the ids and pack the tables.
//
// The library does not keep its assets alive: an asset lives as long as someone holds a reference to it.
//
// Tables. An id minus 1 is an index into the table of its kind.
//
//   convexes         getConvexTable()   c_MaxConvexAssets elements of ConvexMeshEntry
//                    getConvexPlanes()  c_MaxConvexAssets * c_MaxConvexPlanes planes (float4)
//   distance fields  getSdfTable()      c_MaxSdfAssets elements of SdfEntry
//                    getSdfTexture(i)   the texture of field i + 1, for texture slot t<15 + i> of
//                                       CollideShapes
//   triangle meshes  getTriangleMeshTable(), getTriangleVertices(), getTriangleIndices(),
//                    getTriangleBvhNodeLowers(), getTriangleBvhNodeUppers(), getTriangleBvhRoots(): the
//                    meshes of all assets one behind the other ("flattened"), a TriangleMeshEntry and a
//                    root index per mesh
//
// The convex and distance field tables are written when an asset is updated. The triangle mesh arrays are
// rebuilt by commit(), when a mesh was updated, created or released since: a solver calls commit() before
// it reads them (FleX flattens the meshes at the start of a solver update). commit() records buffer
// copies and one buffer write; it dispatches no kernel.
//
// Like the device, the library is not thread-safe. It may be held past the device: it then still answers
// queries, and everything that records GPU work returns FE_NOT_ALIVE_OBJECT.
NVRHI_IID(IGeometryLibrary, "35defe45-9c27-4cd5-8520-3a073c1b9239")
struct IGeometryLibrary : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IGeometryLibrary)

    // The factories (NvFlexCreateTriangleMesh, NvFlexCreateDistanceField, NvFlexCreateConvexMesh). On
    // success FS_OK and the asset with one reference added; it is empty until its update. Releasing the
    // last reference replaces NvFlexDestroy*. FE_OUT_OF_MEMORY and a message when c_MaxSdfAssets distance
    // fields or c_MaxConvexAssets convexes are alive already (triangle meshes have no limit).
    virtual nvrhi::FRESULT createTriangleMesh(ITriangleMeshAsset** ppMesh) noexcept = 0;
    virtual nvrhi::FRESULT createSdf(ISdfAsset** ppSdf) noexcept = 0;
    virtual nvrhi::FRESULT createConvex(IConvexAsset** ppConvex) noexcept = 0;

    // The asset with an id, with one reference added; FE_NOT_FOUND and nullptr when no asset has it.
    virtual nvrhi::FRESULT getTriangleMesh(uint32_t id, ITriangleMeshAsset** ppMesh) noexcept = 0;
    virtual nvrhi::FRESULT getSdf(uint32_t id, ISdfAsset** ppSdf) noexcept = 0;
    virtual nvrhi::FRESULT getConvex(uint32_t id, IConvexAsset** ppConvex) noexcept = 0;

    // Enumeration (NvFlexGetTriangleMeshes, NvFlexGetDistanceFields, NvFlexGetConvexMeshes): stores the
    // ids of up to capacity assets in ids, in ascending order, and returns the number of assets alive.
    // ids may be null with a capacity of 0.
    virtual uint32_t getTriangleMeshIds(_Out_writes_opt_(capacity) uint32_t* ids,
                                        uint32_t capacity) const noexcept = 0;
    virtual uint32_t getSdfIds(_Out_writes_opt_(capacity) uint32_t* ids, uint32_t capacity) const noexcept = 0;
    virtual uint32_t getConvexIds(_Out_writes_opt_(capacity) uint32_t* ids,
                                  uint32_t capacity) const noexcept = 0;

    // Brings the triangle mesh arrays up to date (see above). Records nothing when they are.
    // FE_NOT_ALIVE_OBJECT after the device was released.
    virtual nvrhi::FRESULT commit(_In_opt_ nvrhi::ICommandList* commandList) noexcept = 0;

    // True when a triangle mesh changed since the last commit.
    [[nodiscard]] virtual bool needsCommit() const noexcept = 0;

    // The tables; they do not AddRef. The convex and distance field tables always exist. The triangle mesh
    // arrays are nullptr until a commit found a mesh to flatten, and commit replaces them by larger ones
    // when the meshes have grown (getLayoutRevision).
    [[nodiscard]] virtual IPhysBuffer* getConvexTable() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getConvexPlanes() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getSdfTable() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleMeshTable() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleVertices() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleIndices() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleBvhNodeLowers() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleBvhNodeUppers() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTriangleBvhRoots() const noexcept = 0;

    // The texture of the distance field with id slot + 1: nullptr when there is none, when it was never
    // updated, or for a slot of c_MaxSdfAssets or more. Does not AddRef.
    [[nodiscard]] virtual nvrhi::ITexture* getSdfTexture(uint32_t slot) const noexcept = 0;

    // Advances whenever one of the accessors above returns another object than before: a commit that
    // replaced the triangle mesh arrays, a distance field that was updated or released. Starts at 0.
    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;

    // Advances whenever the contents of a table change: an asset update, a commit that recorded
    // something. Starts at 0.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IGeometryLibrary> GeometryLibraryHandle;

// ---- The shapes of a domain --------------------------------------------------------------------------------

// The arguments of ICollisionShapeSet::upload: host memory, one element per shape in each array. All
// pointers are borrowed for the call.
struct CollisionShapeData {
    const CollisionGeometry* geometry = nullptr;
    const float* positions = nullptr;      // 4 floats per shape: the translation in world space; w unused
    const float* rotations = nullptr;      // 4 floats per shape: a quaternion, x y z imaginary
    const float* prevPositions = nullptr;  // as positions, at the start of the time step; null: positions
    const float* prevRotations = nullptr;  // as rotations, at the start of the time step; null: rotations
    const int32_t* flags = nullptr;        // makeShapeFlags()
    uint32_t numShapes = 0;
    uint32_t _pad = 0;  // explicit
};

static_assert(sizeof(CollisionShapeData) == 56, "CollisionShapeData layout changed: bump c_HeaderVersion");
static_assert(offsetof(CollisionShapeData, flags) == 40);
static_assert(offsetof(CollisionShapeData, numShapes) == 48);
static_assert(offsetof(CollisionShapeData, _pad) == 52);

// The buffers of ICollisionShapeSet::download, one per array; a null one is skipped. All pointers are
// borrowed for the call.
struct CollisionShapeBuffers {
    IPhysBuffer* geometry = nullptr;       // 16-byte elements (CollisionGeometry)
    IPhysBuffer* positions = nullptr;      // 16-byte elements
    IPhysBuffer* rotations = nullptr;      // 16-byte elements
    IPhysBuffer* prevPositions = nullptr;  // 16-byte elements
    IPhysBuffer* prevRotations = nullptr;  // 16-byte elements
    IPhysBuffer* flags = nullptr;          // 4-byte elements
};

static_assert(sizeof(CollisionShapeBuffers) == 48,
              "CollisionShapeBuffers layout changed: bump c_HeaderVersion");
static_assert(offsetof(CollisionShapeBuffers, flags) == 40);

// The collision shapes of a domain (IPhysDomain::getShapes): what the particles collide with.
//
//   array          element                 bytes
//   Geometry       CollisionGeometry          16
//   Positions      float4                     16
//   Rotations      float4 quaternion          16
//   PrevPositions  float4                     16
//   PrevRotations  float4 quaternion          16
//   Flags          int32                       4
//
// The current and the previous transform together are what continuous collision needs: a solver sweeps a
// shape from where it was at the start of the step to where it is at its end.
//
// Assets. A geometry entry names its asset by id. On upload the set looks every id up in the geometry
// library of its device and keeps a reference to each asset found, until the next upload: releasing the
// application's reference to an asset does not pull it from under a shape that uses it.
//
// Upload takes host memory, not IPhysBuffers. The set has to read the flags and the geometry to find the
// assets, and an Upload buffer is write-only memory (on D3D11 mapping it discards its contents), so a
// buffer would have to be accompanied by a host copy anyway. Shapes are few, and the application rewrites
// their transforms every frame from host memory.
//
// The bounding volume hierarchy over the shapes and their world-space bounds are not here: they depend on
// the transforms of a step, and a solver builds them.
NVRHI_IID(ICollisionShapeSet, "b5b34941-f11b-4aba-a5f2-60a609f49574")
struct ICollisionShapeSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(ICollisionShapeSet)

    [[nodiscard]] virtual uint32_t getMaxShapes() const noexcept = 0;  // the capacity; it grows on upload
    [[nodiscard]] virtual uint32_t getNumShapes() const noexcept = 0;

    // The arrays, getMaxShapes() elements each; they do not AddRef. nullptr while the capacity is 0.
    [[nodiscard]] virtual IPhysBuffer* getGeometry() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getPositions() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getRotations() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getPrevPositions() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getPrevRotations() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getFlags() const noexcept = 0;

    // The library the ids of the shapes are looked up in: that of the device. Does not AddRef.
    [[nodiscard]] virtual IGeometryLibrary* getGeometryLibrary() const noexcept = 0;

    // Replaces the set by data.numShapes shapes (NvFlexSetShapes): records writes of the six arrays and
    // advances the content revision. The memory data points at is read before the call returns. As in
    // FleX, null previous transforms mean "the current ones", and with numShapes 0 the set is emptied. A
    // set grows as the constraint sets do (constraints.h); the layout revision then advances.
    //
    // FE_INVALID_ARGS, one message and nothing changed for: a null geometry, positions, rotations or
    // flags pointer with a count; a shape whose type is not a CollisionShapeType; a convex, triangle mesh
    // or distance field shape whose id names no asset of that kind.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList,
                                  const CollisionShapeData& data) noexcept = 0;

    // Records copies of the first numShapes elements of the arrays into the buffers given; a null buffer
    // is skipped. FE_INVALID_ARGS for more than getNumShapes(), another stride, a buffer that is too small
    // or an Upload buffer. FleX has no such call.
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList,
                                    const CollisionShapeBuffers& buffers, uint32_t numShapes) noexcept = 0;

    // Advances when the buffers are replaced (an upload beyond the capacity). Starts at 0.
    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;
    // Advances with every upload. A solver rebuilds the hierarchy over the shapes when it has changed.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;
};
typedef nvrhi::AutoPtr<ICollisionShapeSet> CollisionShapeSetHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_COLLISION_H */
