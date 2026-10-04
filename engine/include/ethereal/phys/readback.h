#ifndef ETHEREAL_PHYS_READBACK_H
#define ETHEREAL_PHYS_READBACK_H

// IPhysReadback and IPhysTimers: what an application reads back from a solver besides the state of its
// domain (step 7 of the migration plan).
//
// They replace FleX's "readback / diagnostics" group of NvFlex.h: NvFlexGetSmoothParticles,
// NvFlexGetDensities, NvFlexGetAnisotropy, NvFlexGetDiffuseParticles, NvFlexGetContacts, NvFlexGetNeighbors
// and NvFlexGetBounds become the read* methods of IPhysReadback; NvFlexGetTimers, NvFlexGetDetailTimers and
// NvFlexGetDeviceLatency become IPhysTimers.
//
// Both are created per solver by the device (IPhysDevice::createReadback, createTimers). They are views:
// each keeps its solver alive, and several may exist for one solver. A readback object uses nothing but the
// public interfaces of the solver and of its domain; it is an object of its own (and not a method set on
// IPhysSolver) so that IPhysSolver, and IPhysSolverXPBD which extends it, keep their vtables.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/params.h>
#include <ethereal/phys/solver.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// ---- IPhysReadback -----------------------------------------------------------------------------------------

// Records copies of what a solver computes into buffers of the application (IPhysDevice::createReadback).
// Not thread-safe, like the device.
//
// Every read* method records GPU copies and nothing else: it never waits and never maps. commandList is the
// open command list of the application, or nullptr for the device's own (device.h, "The device owns one
// command list"). The copies read what the last step left, so they execute after it, in recording order;
// the application then executes the command list (IPhysDevice::flush for the device's own), waits for it
// (IPhysDevice::waitForIdle, or a fence of its own) and maps the buffers.
//
// Destination buffers. A destination is usually a Readback buffer (BufferUse::Readback); any buffer that is
// not an Upload buffer works (a device buffer receives a device-to-device copy). A null destination is
// skipped: a method copies what it is given. A destination's element stride must be the one the method
// names; its content revision advances.
//
// The order of the elements is FleX's, getter by getter (reference/flex/src/Solver.cpp): what FleX returns
// in the order of the particles of the domain ("API order") is returned in that order here, and what FleX
// returns in the solver's internal order (the particles sorted by grid cell, SolverStageContext) is returned
// in internal order, with the two maps that translate. Nothing is reordered on the way. The table of
// SolverOutput (solver.h) says which output is in which order.
//
// Regions. The CopyRegion of a method is in particles (or diffuse particles): srcOffset is the first
// particle read, dstOffset the first written, elementCount how many. It applies to every per-particle array
// of the call, each in its own order (an API-order array is indexed by the particle of the domain, an
// internal-order array by the internal index), and to the per-particle blocks of the arrays that hold
// several elements per particle (below). A null region copies, for each destination, as many elements as
// the smaller of the destination and the source holds, from element 0 of both (FleX's NULL
// NvFlexCopyDesc, which copies the capacity).
//
// Errors. FE_INVALID_ARGS and a message for a destination of another element stride, an Upload destination,
// a region that leaves a source or a destination, and a call with no destination at all; nothing is
// recorded then. FE_GENERIC_ERROR when no command list can be opened.
NVRHI_IID(IPhysReadback, "2b5eb2b5-2f57-4945-8446-c3c8d28caf62")
struct IPhysReadback : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysReadback)

    // The solver the object reads. Does not AddRef: the object keeps it alive.
    [[nodiscard]] virtual IPhysSolver* getSolver() const noexcept = 0;

    // NvFlexGetSmoothParticles: the Laplacian-smoothed positions, float4 (16 bytes), API order. As in FleX,
    // before the solver's first step (IPhysSolver::getSubstepCount() == 0) the particle positions are
    // copied instead; afterwards the smoothed positions, which a step computes only while
    // PhysParams::smoothing or anisotropyScale is above 0 (they are zero until it does).
    virtual nvrhi::FRESULT readSmoothedPositions(_In_opt_ nvrhi::ICommandList* commandList, IPhysBuffer* positions,
                                                 _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetDensities: float (4 bytes), API order; the density of a fluid particle relative to the rest
    // density, written by the last step for every active particle with mass (FleX's Finalize).
    virtual nvrhi::FRESULT readDensities(_In_opt_ nvrhi::ICommandList* commandList, IPhysBuffer* densities,
                                         _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetAnisotropy: the three axes of the ellipsoid of each particle, float4 (16 bytes; xyz the unit
    // direction, w the scale), API order. The region applies to all three. Written while
    // PhysParams::anisotropyScale is above 0; until then the axes are those FleX initialises them to
    // ((1,0,0,0.1), (0,1,0,0.1), (0,0,1,0.1)).
    virtual nvrhi::FRESULT readAnisotropy(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* q1,
                                          _In_opt_ IPhysBuffer* q2, _In_opt_ IPhysBuffer* q3,
                                          _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetDiffuseParticles: positions (float4, w the remaining lifetime in seconds, at most
    // PhysParams::diffuseLifetime) and velocities (float4) of the diffuse
    // particles, in the order the solver compacted them, and their count (one int32, 4 bytes, at element 0
    // of `count`; the region does not apply to it). The count is on the GPU, which is why it comes back in
    // a buffer: only the first `count` particles are alive. The region is in diffuse particles. A domain
    // without diffuse particles (DomainDesc::maxDiffuseParticles 0) records nothing and returns FS_OK, as
    // FleX does.
    virtual nvrhi::FRESULT readDiffuseParticles(_In_opt_ nvrhi::ICommandList* commandList,
                                                _In_opt_ IPhysBuffer* positions, _In_opt_ IPhysBuffer* velocities,
                                                _In_opt_ IPhysBuffer* count,
                                                _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetContacts. With C = SolverDesc::maxContactsPerParticle:
    //   planes      float4 (16 bytes), internal order, C per particle: contact c of the particle at internal
    //               index i is element i * C + c; xyz the normal, w the plane distance
    //   velocities  float4 (16 bytes), the same layout: for a shape, xyz the particle minus the contact point
    //               at the start of the substep and w the index of the shape (a float); for a collision
    //               plane of PhysParams, (0, 0, 0, -1)
    //   indices     int32 (4 bytes), API order: the internal index of each particle of the domain (FleX
    //               fills this one from its reverse lookup), so the contacts of particle j start at planes
    //               element indices[j] * C
    //   counts      int32 (4 bytes), internal order: the contacts of the particle at internal index i
    // They describe the last substep of the last step. A region of n particles copies n * C elements of
    // planes and velocities, from element srcOffset * C to element dstOffset * C.
    virtual nvrhi::FRESULT readContacts(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* planes,
                                        _In_opt_ IPhysBuffer* velocities, _In_opt_ IPhysBuffer* indices,
                                        _In_opt_ IPhysBuffer* counts, _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetNeighbors. All int32 (4 bytes). With S = IPhysSolver::getNeighborStride() (the active count of
    // the last step) and N = SolverDesc::maxNeighborsPerParticle:
    //   neighbors      internal order, column-wise: neighbour k of the particle at internal index i is
    //                  element k * S + i, and is itself an internal index
    //   counts         internal order: the neighbours of the particle at internal index i
    //   apiToInternal  API order: the internal index of each particle of the domain
    //   internalToApi  internal order: the particle of the domain at each internal index (uint32 values)
    // FleX's loop over them works unchanged with stride S:
    //     i = apiToInternal[j]; for c < counts[i]: internalToApi[neighbors[c * S + i]] is a neighbour of j.
    // A region selects internal indices [srcOffset, srcOffset + n), which must lie below S, and keeps the
    // layout of the neighbour lists: column k of them goes from elements k * S + srcOffset to elements
    // k * S + dstOffset, for every k < N (N copies). A null region copies the lists as one block (all
    // columns), and the destination needs N * S elements for all of them.
    virtual nvrhi::FRESULT readNeighbors(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* neighbors,
                                         _In_opt_ IPhysBuffer* counts, _In_opt_ IPhysBuffer* apiToInternal,
                                         _In_opt_ IPhysBuffer* internalToApi,
                                         _In_opt_ const CopyRegion* region) noexcept = 0;

    // NvFlexGetBounds: the lower and the upper corner of the particles, one float3 (12 bytes) each, at
    // element 0 of the destination; the bounds of the predicted positions of the last substep, before its
    // constraint solve, grown by PhysParams::radius on every side (IParticleState::getBounds).
    virtual nvrhi::FRESULT readBounds(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* lower,
                                      _In_opt_ IPhysBuffer* upper) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysReadback> PhysReadbackHandle;

// ---- Timers ------------------------------------------------------------------------------------------------

// FleX's NvFlexTimers, member for member and in its order: the GPU seconds of the stages of one step, each
// member the sum over every substep and iteration of the stages FleX's NvFlexGetTimers adds into it:
//
//   member               stages (the names of the detail timers)
//   predict              Predicate
//   createCellIndices    CalculateParticleHash
//   sortCellIndices      SortParticleHash
//   createGrid           CreateGrid
//   reorder              RecordParticles
//   collideParticles     CollideParticles
//   collideShapes        CollideShapes
//   collideTriangles     CollideTriangles
//   collideFields        (none: FleX has no stage of that name, so it stays 0)
//   calculateDensity     CalculateDensity
//   solveDensities       SolveDensity
//   solveVelocities      UpdateVelocities, CalculateVorticity, SolveVelocities
//   solveShapes          SolveShapes
//   solveSprings         SolveSprings
//   solveContacts        SolveContact
//   solveInflatables     SolveInflatables
//   applyDeltas          ApplyDeltas
//   calculateAnisotropy  SmoothPositions, CalculateAnisotropy
//   updateDiffuse        CreateDiffuseParticles, UpdateDiffuseParticles, ClampDiffuseParticleCount,
//                        CompactDiffuseParticles
//   updateTriangles      UpdateTriangles
//   updateNormals        UpdateNormals, NormalizeNormals
//   finalize             Finalize
//   updateBounds         ComputeBounds
//   total                the sum of the 23 members above
//
// As in FleX, the stages FlattenTriangleMeshes, Rebuild shapes, ContinuousShockPropagation and
// LazyClearGrid count in no member (the detail timers have them), and total is the sum of the members, not
// the length of the step (that is IPhysTimers::getDeviceLatency).
struct PhysTimers {
    float predict = 0.0f;
    float createCellIndices = 0.0f;
    float sortCellIndices = 0.0f;
    float createGrid = 0.0f;
    float reorder = 0.0f;
    float collideParticles = 0.0f;
    float collideShapes = 0.0f;
    float collideTriangles = 0.0f;
    float collideFields = 0.0f;
    float calculateDensity = 0.0f;
    float solveDensities = 0.0f;
    float solveVelocities = 0.0f;
    float solveShapes = 0.0f;
    float solveSprings = 0.0f;
    float solveContacts = 0.0f;
    float solveInflatables = 0.0f;
    float applyDeltas = 0.0f;
    float calculateAnisotropy = 0.0f;
    float updateDiffuse = 0.0f;
    float updateTriangles = 0.0f;
    float updateNormals = 0.0f;
    float finalize = 0.0f;
    float updateBounds = 0.0f;
    float total = 0.0f;
};

static_assert(sizeof(PhysTimers) == 24 * 4, "PhysTimers layout changed: bump c_HeaderVersion");
static_assert(offsetof(PhysTimers, createCellIndices) == 4);
static_assert(offsetof(PhysTimers, collideFields) == 32);
static_assert(offsetof(PhysTimers, solveVelocities) == 44);
static_assert(offsetof(PhysTimers, calculateAnisotropy) == 68);
static_assert(offsetof(PhysTimers, updateBounds) == 88);
static_assert(offsetof(PhysTimers, total) == 92);

// FleX's NvFlexDetailTimer: the GPU seconds of one stage name over a step. name points at static storage
// of the library (valid while ethereal_phys is loaded), never at memory of the timer object.
struct DetailTimer {
    const char* name = nullptr;
    float time = 0.0f;
    uint8_t _pad[4] = {0, 0, 0, 0};  // explicit tail padding
};

static_assert(sizeof(DetailTimer) == 16, "DetailTimer layout changed: bump c_HeaderVersion");
static_assert(offsetof(DetailTimer, time) == 8);
static_assert(offsetof(DetailTimer, _pad) == 12);

// What IPhysTimers reports about itself (getStatus). Measured steps are counted from 1.
struct PhysTimerStatus {
    uint64_t recordedSteps = 0;  // steps measured (recorded with timer queries) so far: the number of the
                                 // newest
    uint64_t completedStep = 0;  // the measured step the getters describe; 0: none has completed yet. Below
                                 // recordedSteps when newer steps are still on the GPU (or not submitted):
                                 // the getters then hold older values, which is what they return.
    uint32_t pendingSteps = 0;      // measured steps whose results have not been read yet
    uint32_t queriesAllocated = 0;  // nvrhi timer queries the solver holds (0 while it never measured)
    uint32_t queriesLastStep = 0;   // the timer queries the newest measured step used
    uint32_t skippedSteps = 0;      // steps with the timers on that were not measured: their queue has no
                                    // timestamp queries, or fewer queries were free than the step needs
                                    // (the steps before it are still on the GPU)
    bool supported = false;  // the queue of the newest step with the timers on (before one: the device's
                             // execution queue) has timestamp queries; false: such a step records markers only
    bool complete = false;   // the completed step was measured in full; false: it needs more queries than
                             // the budget, and the stages after the last query are missing from the values
    uint8_t _pad[6] = {0, 0, 0, 0, 0, 0};  // explicit tail padding
};

static_assert(sizeof(PhysTimerStatus) == 40, "PhysTimerStatus layout changed: bump c_HeaderVersion");
static_assert(offsetof(PhysTimerStatus, pendingSteps) == 16);
static_assert(offsetof(PhysTimerStatus, queriesLastStep) == 24);
static_assert(offsetof(PhysTimerStatus, skippedSteps) == 28);
static_assert(offsetof(PhysTimerStatus, supported) == 32);
static_assert(offsetof(PhysTimerStatus, complete) == 33);
static_assert(offsetof(PhysTimerStatus, _pad) == 34);

// The GPU times of the stages of a solver (IPhysDevice::createTimers). Not thread-safe, like the device.
//
// Measuring. While IPhysSolver::areTimersEnabled(), every stage a step records is bracketed by an nvrhi
// timer query (ICommandList::beginTimerQuery / endTimerQuery), beside the stage marker, and the whole step by
// one more (the device latency). With the timers off a step records no query at all. The queries belong to
// the solver, not to this object: they are recorded whether or not a timer object exists, and every timer
// object of a solver reports the same results.
//
// Reading never blocks. A step's results are read only once all of its queries have finished
// (IDevice::pollTimerQuery), by poll() and at the start of every step with the timers on; the getters then
// describe the newest step read so far, and getStatus says which one that is and how many are still
// pending. They do not change between two such calls, so they always describe one and the same step. On
// Vulkan a query may be polled only once the command list that holds it was submitted, so a step there is
// read only after it is known to be: after IPhysDevice::flush for a step on the device's own command list,
// and after the next step for a step on a command list of the application. A step whose command list never
// runs stops holding back the steps after it 64 steps later. On D3D11 a poll that finds a step unfinished
// flushes the immediate context once (ID3D11DeviceContext::Flush, which does not wait), so that the results
// of an application that never presents arrive as well.
//
// Budget. A step needs one query per stage it runs (with many substeps and iterations, hundreds), and an
// nvrhi device has a fixed number of them (d3d12::DeviceDesc / vulkan::DeviceDesc::maxTimerQueries, 256 by
// default) shared with the application. A solver holds at most getQueryBudget() of them, created as they
// are needed, and a step's queries are free again once its results were read. A step that finds fewer
// free queries than the step before it needed (the steps before it are still on the GPU, or not read yet)
// is not measured at all (PhysTimerStatus::skippedSteps): every measured step is measured in full, or, when
// it needs more queries than the whole budget, up to the point where they run out
// (PhysTimerStatus::complete false). So with a GPU that runs a few frames behind, not every step is
// measured; a larger budget measures more of them.
//
// Support. A step on a queue without timestamp queries records its markers and no query, and fails
// nothing: D3D11 and D3D12 have them on the graphics and the compute queue; Vulkan on a queue when every
// queue family of its kind (graphics, compute) reports timestampValidBits above 0; no API on the copy
// queue.
//
// What is not FleX's: NvFlexGetDeviceLatency also returned the GPU clock at the start and at the end of the
// step and its frequency, which an nvrhi timer query does not expose; FleX's getters waited for the GPU,
// these do not.
NVRHI_IID(IPhysTimers, "0c9c7870-163c-45e4-b57d-1b0947158d66")
struct IPhysTimers : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysTimers)

    // The solver the object reads. Does not AddRef: the object keeps it alive.
    [[nodiscard]] virtual IPhysSolver* getSolver() const noexcept = 0;

    // True when a step recorded on a command list of `queue` can be measured (see "Support" above).
    [[nodiscard]] virtual bool isQueueSupported(nvrhi::CommandQueue queue) const noexcept = 0;

    // The most timer queries the solver holds. Default 128. Lowering it releases queries that are not in
    // use; it never takes away a query a pending step still waits for. 0 measures nothing.
    virtual void setQueryBudget(uint32_t maxQueries) noexcept = 0;
    [[nodiscard]] virtual uint32_t getQueryBudget() const noexcept = 0;

    // Reads the results of the steps that have finished on the GPU, oldest first, without waiting, and
    // makes the newest of them the one the getters describe. Returns PhysTimerStatus::completedStep.
    virtual uint64_t poll() noexcept = 0;

    virtual PhysTimerStatus& getStatus(PhysTimerStatus& retVal) const noexcept = 0;

    // NvFlexGetTimers: the times of the completed step (PhysTimerStatus::completedStep); zero before one.
    virtual PhysTimers& getTimers(PhysTimers& retVal) const noexcept = 0;

    // NvFlexGetDeviceLatency: the GPU seconds from the first to the last command of the completed step;
    // 0 before one.
    [[nodiscard]] virtual float getDeviceLatency() const noexcept = 0;

    // NvFlexGetDetailTimers: one entry per stage name the completed step ran, in the order of the names
    // (strcmp, as FleX's name map), then one named "Total" with the device latency. 0 entries before a
    // step completed. getDetailTimer returns an empty entry (null name) for an index past the count.
    [[nodiscard]] virtual uint32_t getDetailTimerCount() const noexcept = 0;
    virtual DetailTimer& getDetailTimer(DetailTimer& retVal, uint32_t index) const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysTimers> PhysTimersHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_READBACK_H */
