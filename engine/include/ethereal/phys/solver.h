#ifndef ETHEREAL_PHYS_SOLVER_H
#define ETHEREAL_PHYS_SOLVER_H

// IPhysSolver: the operations half of the engine. A solver is created for one domain (domain.h), advances
// it in time, and keeps everything it derives from the domain to itself: the sorted particle arrays, the
// grid, deltas, contacts, neighbours, half-springs and the hierarchy over the collision shapes.
//
// It replaces the operations half of FleX's NvFlexSolver: NvFlexUpdateSolver, NvFlexRegisterSolverCallback
// and the NvFlexGet* calls that return what a solver computes (densities, anisotropy, smoothed positions,
// contacts, neighbours).
//
// What a solver has not:
//
// - Parameters. The domain owns them (IPhysDomain::setParams); a solver reads its domain's in every step.
// - Particle or constraint data. It reads and writes the buffers of its domain through the domain's public
//   interfaces, and leaves the results of a step there: positions, velocities and normals in the particle
//   arrays, the particle bounds in IParticleState::getBounds, the transforms of the rigids and (with
//   plastic deformation) their rest positions in IShapeMatchSet, the triangle normals in ITriangleSet, the
//   diffuse particles and their count in IDiffuseParticleState.
// - Stage methods (predict, build the grid, collide, ...). The stages of a step read and write each
//   other's scratch in a fixed order, and none of them leaves the domain in a state an application could
//   use, so a stage is not something a caller can run correctly on its own; step() is the one operation.
//   An application that wants to act in the middle of a step registers a stage callback.
// - A debug switch. FleX's NvFlexSetDebug does nothing.
//
// The constraint formulation is a creation-time choice (SolverDesc::mode): a PBD solver and an XPBD solver
// are two objects over the same kind of domain, and a caller finds out which one it holds with
// QueryInterface, not with a mode test.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/domain.h>
#include <ethereal/phys/params.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

struct IPhysSolver;

// Where in a step a stage callback runs (FleX's NvFlexSolverCallbackStage, same values).
enum class SolverStage : uint8_t {
    IterationStart = 0,  // at the beginning of each constraint iteration
    IterationEnd = 1,    // at the end of each constraint iteration
    SubstepBegin = 2,    // in each substep, after the prediction and the neighbour search
    SubstepEnd = 3,      // in each substep, after the velocities were derived from the constraint solve
    UpdateEnd = 4,       // once per step, in the last substep, right after its SubstepEnd
    Count = 5,
};

// What a stage callback is given: views of the solver's internal particle arrays and the command list the
// step is being recorded on. Everything is borrowed for the duration of the call.
//
// The arrays are in the solver's internal order (the particles sorted by grid cell); element i belongs to
// the particle sortedToOriginalMap[i] of the domain, and particle j of the domain is element
// originalToSortedMap[j]. Only the first numActive elements count. (FleX hands out the sorted cell hashes
// as one of the two maps; here both are the maps their names say.)
//
// A callback does not run kernels: it records dispatches on commandList, which run when the step does, in
// recording order. Use IPhysKernelLibrary::dispatch(context.commandList, ...) for the kernels of the
// engine, or the application's own compute pipelines. (In FleX the callback received device pointers and
// launched kernels on them; any out-of-tree callback has to be rewritten.)
//
// positions are the predicted positions the constraints work on; a change made at any stage is kept.
// velocities are derived from the positions right before SubstepEnd, so a change made to them counts at
// SubstepEnd and UpdateEnd only (at the other stages they hold the values of the previous substep and are
// overwritten).
struct SolverStageContext {
    IPhysSolver* solver = nullptr;                  // the solver that runs the callback
    nvrhi::ICommandList* commandList = nullptr;     // open; the callback records on it and leaves it open
    nvrhi::IBuffer* positions = nullptr;            // float4 x y z 1/m, a structured buffer with UAVs
    nvrhi::IBuffer* velocities = nullptr;           // float4 (w unused), a structured buffer with UAVs
    nvrhi::IBuffer* phases = nullptr;               // int32, a structured buffer with UAVs
    nvrhi::IBuffer* sortedToOriginalMap = nullptr;  // uint32, a typed buffer (R32_UINT); read only
    nvrhi::IBuffer* originalToSortedMap = nullptr;  // int32, a structured buffer; read only
    uint32_t numActive = 0;    // IParticleState::getActiveCount() of the step
    float dt = 0.0f;           // the length of a substep; at UpdateEnd the length of the whole step
    uint32_t substep = 0;      // the index of the current substep
    uint32_t numSubsteps = 0;  // the substeps of this step
    uint32_t iteration = 0;    // the index of the current iteration (IterationStart, IterationEnd); else 0
    SolverStage stage = SolverStage::Count;
    uint8_t _pad[3] = {0, 0, 0};  // explicit
};

static_assert(sizeof(SolverStage) == 1);
static_assert(sizeof(SolverStageContext) == 80, "SolverStageContext layout changed: bump c_HeaderVersion");
static_assert(offsetof(SolverStageContext, commandList) == 8);
static_assert(offsetof(SolverStageContext, originalToSortedMap) == 48);
static_assert(offsetof(SolverStageContext, numActive) == 56);
static_assert(offsetof(SolverStageContext, dt) == 60);
static_assert(offsetof(SolverStageContext, iteration) == 72);
static_assert(offsetof(SolverStageContext, stage) == 76);
static_assert(offsetof(SolverStageContext, _pad) == 77);

// Implemented by the application (or by an engine extension), as a reference-counted object; a solver
// keeps a reference while the callback is registered. It replaces FleX's NvFlexSolverCallback function
// pointer and user data. onSolverStage is called on the thread that calls IPhysSolver::step, during that
// call; it must not call step, and must not close or execute context.commandList.
NVRHI_IID(IPhysSolverStageCallback, "f0329e1c-ee6d-414b-ac1a-9e022f38d311")
struct IPhysSolverStageCallback : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysSolverStageCallback)

    virtual void onSolverStage(const SolverStageContext& context) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysSolverStageCallback> PhysSolverStageCallbackHandle;

// What a solver computes beside the state of its domain, one buffer each. P is the particle capacity of
// the domain (DomainDesc::maxParticles), C is SolverDesc::maxContactsPerParticle and N is
// SolverDesc::maxNeighborsPerParticle.
//
//   output             element         bytes  elements  order     FleX
//   SmoothedPositions  float4             16  P         domain    NvFlexGetSmoothParticles; written when
//                                                                 PhysParams::smoothing or anisotropyScale is
//                                                                 above 0, zero otherwise
//   Densities          float               4  P         domain    NvFlexGetDensities: the density of a fluid
//                                                                 particle relative to the rest density
//   Anisotropy1..3     float4             16  P         domain    NvFlexGetAnisotropy: the axes of the
//                                                                 ellipsoid of a fluid particle, xyz the
//                                                                 direction and w the scale; written when
//                                                                 anisotropyScale is above 0
//   ContactPlanes      float4             16  P * C     internal  NvFlexGetContacts: contact c of sorted
//                                                                 particle i is element i * C + c; xyz the
//                                                                 normal, w the plane distance
//   ContactVelocities  float4             16  P * C     internal  ... as the collision kernels store it: for a
//                                                                 shape, xyz the particle minus the contact point
//                                                                 at the start of the substep and w the index of
//                                                                 the shape (a float)
//   ContactCounts      int32               4  P         internal  the contacts of sorted particle i
//   Neighbors          int32               4  >= P * N  internal  NvFlexGetNeighbors: neighbour k of sorted
//                                                                 particle i is element
//                                                                 k * getNeighborStride() + i; an internal index
//   NeighborCounts     int32               4  P         internal  the neighbours of sorted particle i
//   ApiToInternal      int32               4  P         domain    the internal (sorted) index of particle j
//   InternalToApi      uint32 (typed)      4  >= P      internal  the particle of the domain at sorted index i
//
// "internal" order is the order of SolverStageContext; ApiToInternal and InternalToApi translate. The
// outputs describe the last substep of the last step; the collision and neighbour outputs are those the
// constraints of that substep were solved with.
enum class SolverOutput : uint8_t {
    SmoothedPositions = 0,
    Densities = 1,
    Anisotropy1 = 2,
    Anisotropy2 = 3,
    Anisotropy3 = 4,
    ContactPlanes = 5,
    ContactVelocities = 6,
    ContactCounts = 7,
    Neighbors = 8,
    NeighborCounts = 9,
    ApiToInternal = 10,
    InternalToApi = 11,
    Count = 12,
};

// The size of one element of a solver output in bytes; 0 for a value that is not an output.
constexpr uint32_t solverOutputStride(SolverOutput output) noexcept {
    return output == SolverOutput::SmoothedPositions || output == SolverOutput::Anisotropy1 ||
                   output == SolverOutput::Anisotropy2 || output == SolverOutput::Anisotropy3 ||
                   output == SolverOutput::ContactPlanes || output == SolverOutput::ContactVelocities
               ? 16u
           : uint8_t(output) < uint8_t(SolverOutput::Count) ? 4u
                                                            : 0u;
}

static_assert(solverOutputStride(SolverOutput::ContactPlanes) == 16 &&
              solverOutputStride(SolverOutput::InternalToApi) == 4 &&
              solverOutputStride(SolverOutput::Count) == 0);

// A solver (IPhysDevice::createSolver). Not thread-safe, like the device. The solver keeps its domain, and
// through it the device, alive.
//
// Several solvers may be created for one domain, but they are alternatives, not collaborators: each
// advances the whole domain by itself.
NVRHI_IID(IPhysSolver, "f5c9c846-6137-489a-9497-5e7e3d295162")
struct IPhysSolver : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysSolver)

    // The description the solver was created with; the object keeps it (a non-owning view).
    [[nodiscard]] virtual const SolverDesc& getDesc() const noexcept = 0;

    // The constraint formulation: getDesc().mode.
    [[nodiscard]] virtual SolverMode getMode() const noexcept = 0;

    // The domain the solver was created for. Does not AddRef: the solver keeps it alive.
    [[nodiscard]] virtual IPhysDomain* getDomain() const noexcept = 0;

    // Advances the domain by dt seconds in `substeps` equal substeps (NvFlexUpdateSolver): records the
    // kernels of the whole step, in order, on one command list. Nothing has run when the call returns.
    //
    // commandList is the open command list of the application, or nullptr for the device's own, which
    // IPhysDevice::flush executes (device.h, "The device owns one command list"). The step reads what was
    // recorded before it (uploads into the domain) and must execute before anything that reads its
    // results, so the command lists involved are executed in recording order, as for every kernel dispatch
    // (IPhysKernelLibrary::dispatch).
    //
    // The step reads the domain as it is at the call: the parameters, the active count, the constraint
    // sets and the shapes. It first brings the geometry library up to date (IGeometryLibrary::commit) and
    // rebuilds what it derives from data that changed since its last step: the half-springs
    // (ISpringSet::getContentRevision) and the hierarchy over the shapes
    // (ICollisionShapeSet::getContentRevision). Nothing is read back from the GPU during a step, so a step
    // never waits.
    //
    // With no active particle the step only does that maintenance. Every data buffer the step writes has
    // its content revision advanced (markContentChanged).
    //
    // FE_INVALID_ARGS and a message for substeps 0 or a dt that is not a positive finite number;
    // FE_NOT_ALIVE_OBJECT for a null commandList when the device has none; the codes of
    // IPhysKernelLibrary::dispatch when a kernel cannot be loaded or bound. A step that fails may have
    // recorded a part of itself: the state of the domain is then undefined until it is uploaded again.
    virtual nvrhi::FRESULT step(_In_opt_ nvrhi::ICommandList* commandList, float dt,
                                uint32_t substeps) noexcept = 0;

    // The number of substeps recorded since the solver was created (FleX's step counter).
    [[nodiscard]] virtual uint64_t getSubstepCount() const noexcept = 0;

    // Brackets every stage of a step with a command list marker of its name (NvFlexUpdateSolver's
    // enableTimers; the names are those of FleX's detail timers). Off by default. The GPU times themselves
    // are read through the timer object of step 7 of the migration plan, which attaches here.
    virtual void setTimersEnabled(bool enabled) noexcept = 0;
    [[nodiscard]] virtual bool areTimersEnabled() const noexcept = 0;

    // Registers the callback of a stage, or removes it with nullptr (NvFlexRegisterSolverCallback). The
    // solver keeps a reference to the callback. The callback that was registered before is stored in
    // *ppPrevious with one reference added (nullptr when there was none), so that callbacks can be chained;
    // with a null ppPrevious it is released. FE_INVALID_ARGS for a stage that is not one.
    virtual nvrhi::FRESULT setStageCallback(SolverStage stage, _In_opt_ IPhysSolverStageCallback* callback,
                                            _Out_opt_ IPhysSolverStageCallback** ppPrevious) noexcept = 0;

    // A buffer the solver computes (SolverOutput). Does not AddRef: the solver keeps it alive. nullptr for
    // a value that is not an output. A device buffer: read it with IPhysDevice::copyBuffer into a Readback
    // buffer, or bind getRHIBuffer() in a renderer. The object returned for InternalToApi may be another
    // one after a step; the others stay the same for the life of the solver.
    //
    // Step 7 of the migration plan wraps host readback (IPhysReadback) around these.
    [[nodiscard]] virtual IPhysBuffer* getOutput(SolverOutput output) const noexcept = 0;

    // The distance between two neighbours of one particle in SolverOutput::Neighbors: the active count of
    // the last step (FleX lays the neighbour lists out column-wise over the active particles).
    [[nodiscard]] virtual uint32_t getNeighborStride() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysSolver> PhysSolverHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_SOLVER_H */
