#ifndef ETHEREAL_PHYS_EXT_CONTAINER_H
#define ETHEREAL_PHYS_EXT_CONTAINER_H

// IPhysContainer, IPhysInstance and IPhysSoftJoint: FleX's container (FleX/extensions/
// flexExtContainer.cpp): a particle allocator, asset instances and soft joints over one domain and its solver
// (step 9 of the migration plan). Created by IPhysDevice::createContainer.
//
// How a container sits on the data model. FleX's NvFlexExtContainer wrapped an NvFlexSolver, which was data
// and operations in one object. Here the two are a domain (domain.h) and a solver (solver.h), and a container
// drives both through their public interfaces only, as an application would:
//
//   FleX container                         here
//   NvFlexSetActive / SetActiveCount       IPhysDevice::writeBuffer into ParticleArray::ActiveIndex and
//                                          IParticleState::setActiveCount
//   NvFlexSetParticles, ..., SetNormals    IPhysDevice::writeBuffer into the particle arrays (see "Push")
//   NvFlexSetSprings / Rigids /            ISpringSet / IShapeMatchSet / ITriangleSet / IInflatableSet::upload
//   DynamicTriangles / Inflatables         with the arrays compacted from the instances and the joints
//   NvFlexUpdateSolver                     IPhysSolver::step
//   NvFlexGetParticles, ..., GetBounds,    IParticleState::download, IPhysDevice::copyBuffer of getBounds,
//   GetRigids                              IShapeMatchSet::download, into Readback buffers of the container
//
// So a container owns, while it lives, the particle arrays and the active list of its domain and the four
// constraint sets: what it pushes replaces what anyone else put there, as in FleX. The collision shapes and
// the parameters stay the application's. Its particle capacity is at most the domain's.
//
// Host mirrors. FleX kept the particle data and the compacted constraint data in host buffers that the
// application mapped (NvFlexExtMapParticleData and its siblings) and that the container copied to and from
// the solver. A container here keeps the same host arrays, the mirrors, allocated through nvrhi's process-wide
// allocator (nvrhi::GetDefaultMemAllocator). FleX's map functions returned their structs by value, which a
// virtual function of this DLL may not do (donut/nvrhi/doc/com-interface-abi.md, R1): they take the struct by
// reference and fill it (the WIDL form). The pointers in it are views of the mirrors:
//
//     THE CALLER MAY READ AND WRITE THROUGH THEM BUT MUST NEVER FREE THEM, AND MUST NOT USE THEM AFTER THE
//     MATCHING UNMAP OR AFTER THE CONTAINER IS RELEASED. THE CONTAINER OWNS THAT MEMORY.
//
// Pull and resolve. pullFromDevice records GPU copies of the particle arrays, the bounds, the rigid transforms
// and the triangle normals into Readback buffers and returns; nothing waits. The copies reach the mirrors when
// something reads the mirrors next (a map, createInstance, updateInstances, IPhysSoftJoint::setTransform, the
// compaction of a push): that call waits for the GPU first (IPhysDevice::waitForIdle) and copies them over,
// as FleX's map waited for the copies it had recorded into its host buffers. A pull recorded on an application
// command list must have been executed before that.
//
// Push. FleX uploaded the whole of every particle mirror on every push. A push here uploads the particles the
// host has written since they were last pushed: all of them while the particle data is mapped and after it
// was mapped (the application may have written anything), those of a new instance, those a soft joint moved,
// and all of them after the container was created. That is what FleX's full upload amounts to whenever its
// mirrors held what the solver had (after a pull), and it makes push, step, push without a pull in between
// keep the simulated state instead of rewinding it to stale mirrors.
//
// Command lists, as everywhere in the engine: the open command list of the application, or nullptr for the
// device's own (IPhysDevice, "The device owns one command list"). Not thread-safe, like the device.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/domain.h>
#include <ethereal/phys/ext/asset.h>
#include <ethereal/phys/solver.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

struct IPhysContainer;

// The particle mirrors of a container (NvFlexExtParticleData), filled by IPhysContainer::mapParticleData.
// Each array has IPhysContainer::getMaxParticles() elements; element i belongs to particle i of the domain.
// See the header comment: read and write, never free.
struct ContainerParticleData {
    float* particles = nullptr;      // x y z 1/mass (ParticleArray::Position)
    float* restParticles = nullptr;  // x y z w (ParticleArray::RestPosition)
    float* velocities = nullptr;     // x y z (ParticleArray::Velocity, 12 bytes per particle)
    int32_t* phases = nullptr;       // ParticleArray::Phase
    float* normals = nullptr;        // x y z w (ParticleArray::Normal)
    const float* lower = nullptr;    // the lower corner of the particle bounds, x y z (pulled)
    const float* upper = nullptr;    // the upper corner of the particle bounds, x y z (pulled)
};

static_assert(sizeof(ContainerParticleData) == 56, "ContainerParticleData layout changed: bump c_HeaderVersion");
static_assert(offsetof(ContainerParticleData, normals) == 32);
static_assert(offsetof(ContainerParticleData, upper) == 48);

// The compacted triangles of a container (NvFlexExtTriangleData), filled by IPhysContainer::mapTriangleData.
// numTriangles is new (FleX's struct had no count). Writes to indices are not sent to the device: a push
// compacts the triangles from the assets again when it compacts at all (FleX did the same).
struct ContainerTriangleData {
    int32_t* indices = nullptr;  // 3 per triangle, particle indices of the domain
    float* normals = nullptr;    // x y z per triangle (12 bytes; FleX's header said 16, its array was 12)
    uint32_t numTriangles = 0;
    uint32_t _pad = 0;  // explicit
};

static_assert(sizeof(ContainerTriangleData) == 24, "ContainerTriangleData layout changed: bump c_HeaderVersion");
static_assert(offsetof(ContainerTriangleData, numTriangles) == 16);
static_assert(offsetof(ContainerTriangleData, _pad) == 20);

// The transforms of the compacted shape-matching constraints (NvFlexExtShapeData), filled by
// IPhysContainer::mapShapeData: the shapes of the instances in the order of the instances, then one per soft
// joint. FleX left n unset.
struct ContainerShapeData {
    float* rotations = nullptr;  // x y z w per shape
    float* positions = nullptr;  // x y z per shape
    uint32_t n = 0;              // the number of shapes
    uint32_t _pad = 0;           // explicit
};

static_assert(sizeof(ContainerShapeData) == 24, "ContainerShapeData layout changed: bump c_HeaderVersion");
static_assert(offsetof(ContainerShapeData, n) == 16);
static_assert(offsetof(ContainerShapeData, _pad) == 20);

// ---- IPhysInstance ---------------------------------------------------------------------------------------

// An asset placed in a container (NvFlexExtInstance; IPhysContainer::createInstance). Releasing the last
// reference is NvFlexExtDestroyInstance: the particles go back to the container's free list and the next push
// compacts the constraints without the instance. The instance keeps its container and its asset alive (FleX
// required the application to keep the asset valid).
//
// The pointers are views of what the instance keeps, valid while it lives.
NVRHI_IID(IPhysInstance, "c5f44dab-9f3d-4001-8509-41c16911cd17")
struct IPhysInstance : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysInstance)

    // Does not AddRef: the instance keeps both alive.
    [[nodiscard]] virtual IPhysContainer* getContainer() const noexcept = 0;
    [[nodiscard]] virtual IPhysAsset* getAsset() const noexcept = 0;

    // The particles of the instance in the domain: index i of the asset is particle getParticleIndices()[i].
    // The count is the asset's particle count when the instance was created. NvFlexExtInstance::particleIndices,
    // numParticles.
    [[nodiscard]] virtual const int32_t* getParticleIndices() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getParticleCount() const noexcept = 0;

    // Where the instance's constraints are in the compacted arrays after the last compaction, or -1: its first
    // triangle (FleX set it for every instance, with or without triangles), its first shape (only for an asset
    // with shapes), its inflatable (only for an inflatable asset; FleX declared the field and never set it).
    [[nodiscard]] virtual int32_t getTriangleIndex() const noexcept = 0;
    [[nodiscard]] virtual int32_t getShapeIndex() const noexcept = 0;
    [[nodiscard]] virtual int32_t getInflatableIndex() const noexcept = 0;

    // The transform of each shape of the asset: 3 floats (translation) and 4 floats (quaternion x y z w) per
    // shape. Set at creation from the spawn transform, and copied from the container by
    // IPhysContainer::updateInstances. NvFlexExtInstance::shapeTranslations, shapeRotations.
    [[nodiscard]] virtual const float* getShapeTranslations() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapeRotations() const noexcept = 0;

    // NvFlexExtInstance::userData: a value of the application, not used by the engine. nullptr at creation.
    virtual void setUserData(void* userData) noexcept = 0;
    [[nodiscard]] virtual void* getUserData() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysInstance> PhysInstanceHandle;

// ---- IPhysSoftJoint --------------------------------------------------------------------------------------

// A soft joint (NvFlexExtSoftJoint; IPhysContainer::createSoftJoint): one more shape-matching constraint over
// particles of the container, usually particles of different instances that it holds together. Releasing the
// last reference is NvFlexExtDestroySoftJoint. The joint keeps its container alive.
NVRHI_IID(IPhysSoftJoint, "c773d444-861e-4407-af9e-51c8864cf15e")
struct IPhysSoftJoint : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysSoftJoint)

    // Does not AddRef.
    [[nodiscard]] virtual IPhysContainer* getContainer() const noexcept = 0;

    // The particles (domain indices) and their offsets from the joint's centre (x y z each), as given.
    [[nodiscard]] virtual const int32_t* getParticleIndices() const noexcept = 0;
    [[nodiscard]] virtual const float* getParticleLocalPositions() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getParticleCount() const noexcept = 0;

    // The joint's shape in the compacted shape arrays after the last compaction; -1 before the first.
    [[nodiscard]] virtual int32_t getShapeIndex() const noexcept = 0;

    // The joint's transform: 3 floats and a quaternion (x y z w). Zero and the identity at creation;
    // IPhysContainer::updateInstances sets the translation to the centre of mass of the particles the first
    // time, then copies both from the container, and setTransform sets them.
    [[nodiscard]] virtual const float* getShapeTranslation() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapeRotation() const noexcept = 0;

    [[nodiscard]] virtual float getStiffness() const noexcept = 0;

    // True once updateInstances has computed the centre of mass (NvFlexExtSoftJoint::initialized).
    [[nodiscard]] virtual bool isInitialized() const noexcept = 0;

    // NvFlexExtSoftJointSetTransform: moves the joint's particles by the rigid transform that takes the
    // joint's current transform to the new one (position: 3 floats, rotation: x y z w), in the particle
    // mirrors of the container, and makes that the joint's transform. The moved particles go to the device with
    // the next push, and the shape-matching constraints the particles belong to drag the rest of their bodies
    // after them. As in FleX, the constraint itself is not recompacted. FE_INVALID_ARGS for a null pointer.
    virtual nvrhi::FRESULT setTransform(const float* position, const float* rotation) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysSoftJoint> PhysSoftJointHandle;

// ---- IPhysContainer --------------------------------------------------------------------------------------

// A container (NvFlexExtContainer; IPhysDevice::createContainer). It keeps its solver, and through it the
// domain and the device, alive.
NVRHI_IID(IPhysContainer, "686ff788-d70f-4ac7-8e46-3a9af4965364")
struct IPhysContainer : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysContainer)

    // Do not AddRef: the container keeps them alive.
    [[nodiscard]] virtual IPhysSolver* getSolver() const noexcept = 0;
    [[nodiscard]] virtual IPhysDomain* getDomain() const noexcept = 0;

    // The capacity given at creation: particles 0 .. getMaxParticles() - 1 of the domain are the container's.
    [[nodiscard]] virtual uint32_t getMaxParticles() const noexcept = 0;

    // ---- The allocator (FleX's free list) ----

    // NvFlexExtAllocParticles: takes up to n particles off the end of the free list into indices (room for
    // n) and returns how many it took. The free list starts as 0, 1, ..., max - 1, so the first allocation
    // gets the highest indices; freed particles are appended and are the next to be handed out. The indices
    // are exactly those FleX hands out for the same calls.
    virtual uint32_t allocParticles(uint32_t n, _Out_writes_opt_(n) int32_t* indices) noexcept = 0;

    // NvFlexExtFreeParticles: appends the particles to the free list. FE_INVALID_ARGS, a message and nothing
    // freed for an index out of range, one that is free already or one given twice (FleX asserted that in
    // debug builds only).
    virtual nvrhi::FRESULT freeParticles(uint32_t n, _In_reads_opt_(n) const int32_t* indices) noexcept = 0;

    // NvFlexExtGetActiveList: the particles not on the free list, ascending. Stores up to capacity of them in
    // indices and returns how many there are. indices may be null with a capacity of 0.
    virtual uint32_t getActiveList(_Out_writes_opt_(capacity) int32_t* indices, uint32_t capacity) noexcept = 0;
    [[nodiscard]] virtual uint32_t getActiveCount() const noexcept = 0;  // getMaxParticles() - the free list

    // ---- The mirrors (see the header comment: read and write, never free) ----

    // NvFlexExtMapParticleData / UnmapParticleData. Resolves a pending pull first. Mapping twice returns the
    // same pointers; unmapping what is not mapped does nothing. While the particle data is mapped, and after
    // it, a push uploads all of it.
    virtual ContainerParticleData& mapParticleData(ContainerParticleData& retVal) noexcept = 0;
    virtual void unmapParticleData() noexcept = 0;

    // NvFlexExtMapTriangleData / UnmapTriangleData, NvFlexExtMapShapeData / UnmapShapeData. The arrays are
    // replaced when a push compacts: a push that has to compact while either is mapped is refused.
    virtual ContainerTriangleData& mapTriangleData(ContainerTriangleData& retVal) noexcept = 0;
    virtual void unmapTriangleData() noexcept = 0;
    virtual ContainerShapeData& mapShapeData(ContainerShapeData& retVal) noexcept = 0;
    virtual void unmapShapeData() noexcept = 0;

    // ---- Instances and joints ----

    // NvFlexExtCreateInstance: allocates the asset's particles, writes them into the particle mirrors (the
    // asset's positions through transform, the inverse masses times invMassScale, the asset's positions as
    // rest positions, velocity, phase, zero normals), sets the shape transforms of the instance (the shape
    // centres through transform, the rotation of transform's upper 3 x 3) and marks the container for
    // compaction. transform: 16 floats, a column-major 4 x 4 matrix for column vectors, FleX's convention
    // (Donut's row-vector affine in its memory order is the same matrix). velocity: 3 floats.
    //
    // particleData is the view mapParticleData returned (FleX required the mapped data), or nullptr: the
    // container's own mirrors, without mapping them (only the instance's particles are then pushed). Anything
    // else is FE_INVALID_ARGS: a write elsewhere would never reach the device.
    //
    // FE_OUT_OF_MEMORY and a message when the free list holds fewer particles than the asset has (FleX
    // returned NULL); FE_INVALID_ARGS for a null asset, transform, velocity or ppInstance.
    virtual nvrhi::FRESULT createInstance(_In_opt_ const ContainerParticleData* particleData, IPhysAsset* asset,
                                          const float* transform, const float* velocity, int32_t phase,
                                          float invMassScale, IPhysInstance** ppInstance) noexcept = 0;

    // NvFlexExtNotifyAssetChanged: the constraints of an asset changed (a tearing cloth, say); the next push
    // compacts. asset may be null; as in FleX every instance is compacted again either way. An asset whose
    // indices reach beyond the particles its instance was created with is left out of the compaction with a
    // message (FleX read past the instance's index array).
    virtual void notifyAssetChanged(_In_opt_ IPhysAsset* asset) noexcept = 0;

    // NvFlexExtCreateSoftJoint: a joint over n particles of the container (indices) with their offsets from
    // the joint's centre (localPositions, 3 floats each) and a stiffness; the next push compacts.
    // FE_INVALID_ARGS for n 0, a null array, or an index beyond getMaxParticles().
    virtual nvrhi::FRESULT createSoftJoint(const int32_t* indices, const float* localPositions, uint32_t n,
                                           float stiffness, IPhysSoftJoint** ppJoint) noexcept = 0;

    [[nodiscard]] virtual uint32_t getInstanceCount() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getSoftJointCount() const noexcept = 0;

    // ---- Device transfers ----

    // NvFlexExtPushToDevice: the active list when it changed (the allocator ran since), the particles the host
    // wrote (header comment), and, when needed, the compacted springs, shape-matching constraints (the
    // instances' shapes, then one per joint), triangles and inflatables. FE_INVALID_ARGS when it has to compact
    // while the triangle or the shape data is mapped; the codes of the domain's calls.
    virtual nvrhi::FRESULT pushToDevice(_In_opt_ nvrhi::ICommandList* commandList) noexcept = 0;

    // NvFlexExtPullFromDevice: records copies of the particle arrays, the particle bounds, the rigid transforms
    // and the triangle normals into Readback buffers (header comment, "Pull and resolve"). FleX did not read the
    // triangle normals back, so its NvFlexExtTriangleData::normals never held any.
    virtual nvrhi::FRESULT pullFromDevice(_In_opt_ nvrhi::ICommandList* commandList) noexcept = 0;

    // NvFlexExtTickContainer: pushToDevice, IPhysSolver::step(commandList, dt, substeps), pullFromDevice. FleX's
    // enableTimers argument is the solver's setTimersEnabled. The force fields are a stage callback of the
    // solver (IForceFieldSet), as in FleX, not a part of the tick.
    virtual nvrhi::FRESULT tickContainer(_In_opt_ nvrhi::ICommandList* commandList, float dt,
                                         uint32_t substeps) noexcept = 0;

    // NvFlexExtUpdateInstances: copies the shape transforms of the container into its instances and joints
    // (getShapeTranslations, getShapeTranslation); a joint seen for the first time gets the centre of mass of
    // its particles in the mirrors instead. Resolves a pending pull first.
    virtual void updateInstances() noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysContainer> PhysContainerHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_EXT_CONTAINER_H */
