#ifndef ETHEREAL_PHYS_SOLVER_H
#define ETHEREAL_PHYS_SOLVER_H

// IPhysSolver: the operations half of the engine. A solver is created for one domain (domain.h), advances
// it in time, and keeps everything it derives from the domain to itself: the sorted particle arrays, the
// grid, deltas, contacts, neighbours, half-springs and the hierarchy over the collision shapes.
//
// It replaces the operations half of FleX's NvFlexSolver: NvFlexUpdateSolver, NvFlexRegisterSolverCallback
// and the NvFlexGet* calls that return what a solver computes (densities, anisotropy, smoothed positions,
// contacts, neighbours): the buffers are getOutput's, the copies of FleX's getters are IPhysReadback's and
// the GPU timers IPhysTimers' (readback.h).
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
// QueryInterface, not with a mode test. An XPBD solver answers IPhysSolverXPBD (below) as well, which holds
// what only it has: the compliance block and the Lagrange multipliers.
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

    // Brackets every stage of a step with a command list marker of its name and, where the queue has
    // timestamp queries, with an nvrhi timer query (NvFlexUpdateSolver's enableTimers; the names are those
    // of FleX's detail timers). Off by default; off, a step records neither. The GPU times are read through
    // a timer object of the solver (IPhysTimers, readback.h; IPhysDevice::createTimers).
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
    // IPhysReadback (readback.h; IPhysDevice::createReadback) records the copies FleX's NvFlexGet* calls
    // made, in FleX's order, into buffers of the application.
    [[nodiscard]] virtual IPhysBuffer* getOutput(SolverOutput output) const noexcept = 0;

    // The distance between two neighbours of one particle in SolverOutput::Neighbors: the active count of
    // the last step (FleX lays the neighbour lists out column-wise over the active particles).
    [[nodiscard]] virtual uint32_t getNeighborStride() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysSolver> PhysSolverHandle;

// The XPBD solver (SolverMode::XPBD): IPhysSolver plus what only the XPBD formulation has. A caller finds
// it with QueryInterface on an IPhysSolver; a PBD solver does not answer IID_IPhysSolverXPBD.
//
// The formulation (Macklin, Mueller, Chentanez, "XPBD: Position-Based Simulation of Compliant Constrained
// Dynamics", MIG 2016), as FleX's eNvFlexSolverXPBD has it:
//
// - Springs (with tethers, so cloth stretch, shear and bending), inflatable volumes and shape matching are
//   compliant constraints: alpha~ = alpha / dt^2 with dt the substep, so their stiffness does not depend on
//   dt, the substep count or the iteration count once the iterations have converged. Density and contact
//   constraints stay PBD.
// - A spring or rigid stiffness k of the domain is not a per-iteration multiplier here: k in [0, 1] maps
//   onto [stiffnessMin, stiffnessMax] geometrically and alpha = 1 / (stiffnessMin * (stiffnessMax /
//   stiffnessMin)^k) (XpbdComplianceParams, params.h). k <= 0 disables the constraint; a negative spring
//   stiffness still marks a tether. The domain is the one a PBD solver uses; only this reading differs.
// - Springs and inflatables carry a Lagrange multiplier each, accumulated over the iterations of a substep
//   and cleared at the start of every substep (after SubstepBegin, before the first IterationStart).
//   Shape matching has none: its goal is re-fitted every iteration, so each particle-to-goal attachment is
//   solved statelessly.
//
// Known limitations, kept from the reference (they are properties of the formulation on FleX's Jacobi
// solver, not of this port):
//
// - Under RelaxationMode::Local the summed delta of a particle is divided by its constraint count while a
//   multiplier accumulates the full increment, so the bookkeeping is approximate, and iteration
//   independence only holds once the solve has converged (about 20 iterations per substep for cloth).
// - Damping is coupled to compliance (gamma = alpha * springDamping / dt), so springs with k = 1 get none.
// - Shape-matching compliance is per particle-to-goal attachment and mass-free (as in the PBD kernel).
// - Inflatables lose PBD's k^3 re-inflation boost: a collapsed balloon re-inflates more slowly.
NVRHI_IID(IPhysSolverXPBD, "fbe24ffe-903d-4df3-939d-96918f23761c")
struct IPhysSolverXPBD : IPhysSolver {
    NVRHI_DECLARE_UUID_TRAITS(IPhysSolverXPBD)

    // The compliance block every step reads (FleX's NvFlexParams::stiffnessMin, stiffnessMax,
    // springDamping, volumeCompliance). Defaults: those of XpbdComplianceParams. The values are kept as
    // given; the clamps of params.h apply when a step reads them (stiffnessMin <= 0 counts as 1e-6,
    // stiffnessMax below stiffnessMin as stiffnessMin). getCompliance returns a view of the block the
    // solver keeps.
    virtual void setCompliance(const XpbdComplianceParams& compliance) noexcept = 0;
    [[nodiscard]] virtual const XpbdComplianceParams& getCompliance() const noexcept = 0;

    // The Lagrange multipliers, for inspection: float32 structured buffers with UAVs, owned by the solver
    // (no reference is added). nullptr while the matching constraint set of the domain is empty, and until
    // the first step after it was filled (the solver sizes them there).
    //
    // getSpringMultipliers: one per half-spring, in the solver's internal order: the two halves of every
    // spring, sorted by the particle they belong to. Both halves of a spring between two particles with
    // mass hold the same value; the half of a fixed particle (inverse mass 0) is not solved and stays 0.
    // Only the first 2 * ISpringSet::getNumSprings() elements count.
    // getInflatableMultipliers: one per inflatable, in the order of the inflatable set.
    //
    // They hold the values of the last iteration of the last substep. The object may be another one after
    // a step that grew the matching constraint set.
    [[nodiscard]] virtual nvrhi::IBuffer* getSpringMultipliers() const noexcept = 0;
    [[nodiscard]] virtual nvrhi::IBuffer* getInflatableMultipliers() const noexcept = 0;

    // Records the clear of both multiplier buffers to zero on commandList: the open command list of the
    // application, or nullptr for the device's own (as IPhysSolver::step). A step clears them itself at the
    // start of every substep, so this matters only to a caller that inspects them. FS_OK when there is
    // nothing to clear; the codes of IPhysKernelLibrary::dispatch otherwise.
    virtual nvrhi::FRESULT resetMultipliers(_In_opt_ nvrhi::ICommandList* commandList) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysSolverXPBD> PhysSolverXPBDHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_SOLVER_H */
