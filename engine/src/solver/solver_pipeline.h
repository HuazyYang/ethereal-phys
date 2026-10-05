#ifndef ETHEREAL_PHYS_SRC_SOLVER_SOLVER_PIPELINE_H
#define ETHEREAL_PHYS_SRC_SOLVER_SOLVER_PIPELINE_H

// Module-private: SolverPipeline, what the PBD and the XPBD solver have in common, and how the device
// creates a solver. The implementation classes stay in their .cpp files.
//
// SolverPipeline is FleX's Solver (FleX/src/Solver.cpp) without its data half: the scratch a
// step derives from a domain (the sorted arrays, the grid, deltas, contacts, neighbours, the half-springs,
// the bounds of the shapes and the hierarchy over them, the radix sorts) and the stages of a step, each a
// port of the Solver method of the same name. It is a helper base, not an interface: a solver class derives
// from it and from nvrhi::ObjectImpl<its interface>, and forwards the interface methods to it.
//
// Where the two formulations differ, the reference branches on NvFlexParams::solverMode. Here those points
// are virtual hooks ("seams") with the PBD behaviour as the default; the XPBD solver (solver_xpbd.cpp)
// overrides them and adds its own state (the compliance block, the Lagrange multiplier buffers). The seams
// are listed together below, under "Seams".
//
// The pipeline is written against the public interfaces of the domain, the device and the kernel library,
// with one exception: it resolves a null command list through KernelLibraryHost, as the kernel library
// does, so that a stage callback is always handed a command list.

#include <ethereal/phys/device.h>
#include <ethereal/phys/solver.h>

#include "bvh_builder.h"
#include "kernel_library.h"
#include "kernel_params.h"
#include "radix_sort.h"
#include "stage_timers.h"

#include <cstdint>

namespace ephys::detail {

// Creates the solver desc.mode asks for. Returns FS_OK and one reference in *ppSolver, or an FE_* code and
// nullptr; it reports through the message callback of the device.
nvrhi::FRESULT createSolver(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                            const SolverDesc& desc, IPhysSolver** ppSolver) noexcept;

// The two constructors createSolver dispatches to, after it validated desc (solver_pbd.cpp,
// solver_xpbd.cpp). They may throw std::bad_alloc; createSolver catches it.
nvrhi::FRESULT createSolverPBD(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                               const SolverDesc& desc, IPhysSolver** ppSolver);
nvrhi::FRESULT createSolverXPBD(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                                const SolverDesc& desc, IPhysSolver** ppSolver);

// The timer pool of an XPBD solver of this library, or null (solver_xpbd.cpp; stageTimersOf in
// stage_timers.h asks the PBD class first, then this).
StageTimerPool* stageTimersOfXPBD(IPhysSolver* solver) noexcept;

// FleX's SPHCalculateRestDensity (FleX/src/SPH.cpp): the density of a tightly packed fluid at
// rest, and the scales of the density and the surface tension constraint that follow from it.
//
// Like FleX's, it resets the seeds of the random number generator below, which is state of the module:
// what flexRandf returns to the next solver depends on whether a solver computed a rest density before.
void sphCalculateRestDensity(float restDistance, float h, float* rho, float* rhoDeriv,
                             float* surfaceDeriv) noexcept;

// FleX's Randf(min, max) (FleX/src/private/Random.h), with module-wide seeds that start at zero,
// as in FleX: until the first sphCalculateRestDensity it returns min.
float flexRandf(float min, float max) noexcept;

class SolverPipeline {
 public:
    SolverPipeline(const SolverPipeline&) = delete;
    SolverPipeline& operator=(const SolverPipeline&) = delete;

    // The timer queries of the solver (IPhysTimers reads them; IPhysDevice::createTimers).
    [[nodiscard]] StageTimerPool& stageTimers() { return m_Timers; }

 protected:
    // FleX's IterationState, plus the command list the step is recorded on.
    struct StepState {
        nvrhi::ICommandList* commandList = nullptr;
        float dt = 0.0f;   // the step
        float dta = 0.0f;  // one substep
        uint32_t substepIdx = 0;
        uint32_t numSubsteps = 0;
        uint32_t iteration = 0;
        uint32_t numBlocks = 0;      // thread groups of 256 over the active particles
        uint32_t numBlocksHalf = 0;  // thread groups of 128 over the active particles
    };

    SolverPipeline(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain, const SolverDesc& desc);
    ~SolverPipeline();

    // Solver::Init: creates and clears the scratch (on the device's own command list).
    nvrhi::FRESULT init() noexcept;

    // Solver::Update. `self` is the interface of the solver object, for SolverStageContext::solver.
    nvrhi::FRESULT update(IPhysSolver* self, nvrhi::ICommandList* commandList, float dt,
                          uint32_t substeps) noexcept;

    nvrhi::FRESULT setCallback(SolverStage stage, IPhysSolverStageCallback* callback,
                               IPhysSolverStageCallback** ppPrevious) noexcept;
    [[nodiscard]] IPhysBuffer* output(SolverOutput which) const noexcept;

    // ---- Seams: where XPBD differs from PBD (the solverMode branches of Solver.cpp) ----------------------
    //
    // The hooks run inside init() and update(), which catch what they throw. Each configure* hook is handed the dispatch the pipeline filled in for PBD (kernel id, slots, b0) and
    // may change it: another kernel id, more slots, the compliance block at b1. The defaults do nothing.

    // Solver::UpdateSubstep, before the iteration loop. XPBD clears its Lagrange multipliers here: they
    // accumulate over the iterations of a substep and start at zero in each.
    virtual nvrhi::FRESULT beginConstraintIterations(const StepState& state);

    // Solver::SolveSprings. XPBD: KernelId::SolveSpringsXPBD, the multipliers at readWrite[1], the sorted
    // positions of the start of the substep at readOnly[8].
    virtual void configureSolveSprings(const StepState& state, KernelDispatchDesc& dispatch);

    // Solver::CalculateAndSolveInflatables, its first dispatch. XPBD:
    // KernelId::CalculateInflatableVolumeXPBD, the multipliers at readWrite[1].
    virtual void configureInflatableVolume(const StepState& state, KernelDispatchDesc& dispatch);

    // Solver::SolveShapes. XPBD: KernelId::SolveShapesXPBD or SolveShapesPlasticDeformationXPBD.
    virtual void configureSolveShapes(const StepState& state, bool plastic, KernelDispatchDesc& dispatch);

    // Solver::SetSprings and SetInflatables grow the multiplier buffers with the half-spring and the
    // inflatable capacity. XPBD allocates (and clears) its buffers here.
    virtual nvrhi::FRESULT onHalfSpringCapacity(uint32_t maxHalfSprings);
    virtual nvrhi::FRESULT onInflatableCapacity(uint32_t maxInflatables);

    // ---- What a derived solver may use -------------------------------------------------------------------

    [[nodiscard]] IPhysDevice* device() const { return m_Device; }
    [[nodiscard]] IPhysKernelLibrary* kernels() const { return m_Kernels; }
    [[nodiscard]] IPhysDomain* domain() const { return m_Domain; }
    [[nodiscard]] const SolverDesc& desc() const { return m_Desc; }
    [[nodiscard]] uint64_t substepCount() const { return m_StepCounter; }
    [[nodiscard]] uint32_t neighborStride() const { return m_NeighborStride; }
    [[nodiscard]] nvrhi::IBuffer* sortedPositions() const { return rhi(m_SortedPositions); }
    // The springs and inflatables of the step being recorded (FleX's mNumSprings, mNumInflatables).
    [[nodiscard]] uint32_t stepSprings() const { return m_View.numSprings; }
    [[nodiscard]] uint32_t stepInflatables() const { return m_View.numInflatables; }

    // A structured scratch buffer of `count` elements of `stride` bytes, cleared to zero.
    nvrhi::FRESULT createScratch(const char* name, uint32_t stride, uint32_t count, PhysBufferHandle& out) {
        return createBuffer(name, BufferUse::Structured, stride, count, nullptr, out);
    }

    // Library::ClearBufferInt: records the clear of the first `count` 4-byte elements of a structured buffer.
    void clearInt(nvrhi::ICommandList* commandList, IPhysBuffer* buffer, uint32_t count, int32_t value);

    void reportError(nvrhi::FRESULT code, const char* format, ...) const;

    // IPhysSolver::setTimersEnabled / areTimersEnabled.
    void enableTimers(bool enabled) {
        m_TimersEnabled = enabled;
        m_Timers.setEnabled(enabled);
    }
    [[nodiscard]] bool timersEnabled() const { return m_TimersEnabled; }

 private:
    static nvrhi::IBuffer* rhi(IPhysBuffer* buffer) { return buffer ? buffer->getRHIBuffer() : nullptr; }
    static nvrhi::IBuffer* rhi(const PhysBufferHandle& buffer) { return rhi(buffer.Get()); }

    // Creates a buffer and fills it: with zeros, or with `count` copies of the `stride` bytes at `fill`.
    nvrhi::FRESULT createBuffer(const char* name, BufferUse use, uint32_t stride, uint32_t count, const void* fill,
                                PhysBufferHandle& out);

    // A dispatch of `groups` thread groups with the kernel parameters of the step at b0.
    KernelDispatchDesc begin(KernelId kernel, uint32_t groups) const;
    // Records it, unless a stage failed before; a failure is kept in m_Result.
    void run(const StepState& state, const KernelDispatchDesc& dispatch);
    void keep(nvrhi::FRESULT result) {
        if (NVRHI_SUCCEEDED(m_Result) && NVRHI_FAILED(result)) m_Result = result;
    }

    // The buffers of the domain a step uses, looked up at its start: a set may have replaced its buffers
    // since the last step, and the application may have substituted a particle array.
    void refreshDomainView();

    // Solver::SetParams: the part that derives the fluid rest density from the parameters.
    void refreshParams();
    // Solver::InitParams.
    void initParams(const StepState& state);

    // The rest of Solver::SetSprings (the half-springs), ShapeData::RebuildShapeData and the part of
    // Solver::SetInflatables that sizes the pressures, each when the data changed since the last step.
    void rebuildHalfSprings(const StepState& state);
    void rebuildShapeData(const StepState& state);
    void resizeInflatables();

    nvrhi::FRESULT initImpl();
    nvrhi::FRESULT updateImpl(nvrhi::ICommandList* commandList, float dt, uint32_t substeps);
    void updateSubstep(const StepState& state);

    // The stages, in the order of Solver::UpdateSubstep.
    void predict(const StepState& state);
    void computeBounds(const StepState& state);
    void calculateParticleHash(const StepState& state);
    void sortParticleHash(const StepState& state);
    void createGrid(const StepState& state);
    void reorderParticles(const StepState& state);
    void collideParticles(const StepState& state);
    void collideTriangles(const StepState& state);
    void collideShapes(const StepState& state);
    void continuousShockPropagation(const StepState& state);
    void calculateDensity(const StepState& state);
    void solveDensity(const StepState& state);
    void solveSprings(const StepState& state);
    void calculateAndSolveInflatables(const StepState& state);
    void solveShapes(const StepState& state);
    void applyDeltas(const StepState& state);
    void solveContact(const StepState& state);
    void updateVelocities(const StepState& state);
    void calculateVorticity(const StepState& state);
    void solveVelocities(const StepState& state);
    void updateTriangles(const StepState& state);
    void updateNormals(const StepState& state);
    void normalizeNormals(const StepState& state);
    void finalize(const StepState& state);
    void createDiffuse(const StepState& state);
    void updateDiffuse(const StepState& state);
    void clampDiffuse(const StepState& state);
    void compactDiffuse(const StepState& state);
    void smoothPositions(const StepState& state);
    void calculateAnisotropy(const StepState& state);
    void lazyClearGrid(const StepState& state);

    void executeCallback(const StepState& state, SolverStage stage, float dt);
    void markDomainChanged();

    // Brackets a stage with a command list marker and a timer query when the timers are on (FleX's
    // NVFLEX_PROFILE_SECTION). With the timers off it records nothing. The name is a string literal: the
    // detail timers hand it out (DetailTimer::name points at static storage).
    class StageScope {
     public:
        StageScope(SolverPipeline& pipeline, const StepState& state, const char* name)
            : m_CommandList(pipeline.m_TimersEnabled ? state.commandList : nullptr), m_Timers(&pipeline.m_Timers) {
            if (!m_CommandList) return;
            m_CommandList->beginMarker(name);
            m_Slot = m_Timers->beginStage(m_CommandList, name);
        }
        ~StageScope() {
            if (!m_CommandList) return;
            m_Timers->endStage(m_CommandList, m_Slot);
            m_CommandList->endMarker();
        }
        StageScope(const StageScope&) = delete;
        StageScope& operator=(const StageScope&) = delete;

     private:
        nvrhi::ICommandList* m_CommandList;
        StageTimerPool* m_Timers;
        int32_t m_Slot = -1;
    };

    // Brackets a whole step with the step's record in the timer pool (FleX's latency timer), when the
    // timers are on.
    class StepScope {
     public:
        StepScope(SolverPipeline& pipeline, nvrhi::ICommandList* commandList, bool ownList)
            : m_CommandList(pipeline.m_TimersEnabled ? commandList : nullptr), m_Timers(&pipeline.m_Timers) {
            if (m_CommandList) m_Timers->beginStep(m_CommandList, ownList);
        }
        ~StepScope() {
            if (m_CommandList) m_Timers->endStep(m_CommandList);
        }
        StepScope(const StepScope&) = delete;
        StepScope& operator=(const StepScope&) = delete;

     private:
        nvrhi::ICommandList* m_CommandList;
        StageTimerPool* m_Timers;
    };

    bool m_TimersEnabled = false;

    // What refreshDomainView looked up. Buffers of a set without capacity are null.
    struct DomainView {
        uint32_t numParticles = 0;
        nvrhi::IBuffer* positions = nullptr;
        nvrhi::IBuffer* velocities = nullptr;
        nvrhi::IBuffer* phases = nullptr;
        nvrhi::IBuffer* normals = nullptr;
        nvrhi::IBuffer* restPositions = nullptr;
        nvrhi::IBuffer* activeIndices = nullptr;
        nvrhi::IBuffer* bounds = nullptr;

        uint32_t numSprings = 0;

        uint32_t numRigids = 0;
        uint32_t numRigidIndices = 0;
        bool plastic = false;
        nvrhi::IBuffer* rigidOffsets = nullptr;
        nvrhi::IBuffer* rigidIndices = nullptr;
        nvrhi::IBuffer* rigidRestPositions = nullptr;
        nvrhi::IBuffer* rigidRestNormals = nullptr;
        nvrhi::IBuffer* rigidStiffness = nullptr;
        nvrhi::IBuffer* rigidThresholds = nullptr;
        nvrhi::IBuffer* rigidCreeps = nullptr;
        nvrhi::IBuffer* rigidRotations = nullptr;
        nvrhi::IBuffer* rigidTranslations = nullptr;

        uint32_t numTriangles = 0;
        nvrhi::IBuffer* triangleIndices = nullptr;
        nvrhi::IBuffer* triangleNormals = nullptr;

        uint32_t numInflatables = 0;
        nvrhi::IBuffer* inflatables = nullptr;

        uint32_t numShapes = 0;
        nvrhi::IBuffer* shapeGeometry = nullptr;
        nvrhi::IBuffer* shapePositions = nullptr;
        nvrhi::IBuffer* shapeRotations = nullptr;
        nvrhi::IBuffer* shapePrevPositions = nullptr;
        nvrhi::IBuffer* shapePrevRotations = nullptr;
        nvrhi::IBuffer* shapeFlags = nullptr;

        nvrhi::IBuffer* diffusePositions = nullptr;
        nvrhi::IBuffer* diffuseVelocities = nullptr;
        nvrhi::IBuffer* diffuseCount = nullptr;
    };

    PhysDeviceHandle m_Device;
    KernelLibraryHost* m_Host;  // the device's; valid while m_Device is
    // The command list the zero fills of new scratch buffers are recorded on (createBuffer): nullptr, the
    // device's own, at creation; the list of the step during a step, so that a buffer a step grows is cleared
    // in order before the step uses it, and a step on an application command list opens no other list (on
    // D3D11 that would be a second immediate command list).
    nvrhi::ICommandList* m_FillList = nullptr;
    PhysKernelLibraryHandle m_Kernels;
    PhysDomainHandle m_Domain;
    SolverDesc m_Desc;
    KernelVariant m_Variant = KernelVariant::Default;

    uint32_t m_MaxParticles = 0;
    uint32_t m_MaxDiffuseParticles = 0;

    DomainView m_View;
    nvrhi::FRESULT m_Result = nvrhi::FS_OK;  // the first failure of the step being recorded
    IPhysSolver* m_Self = nullptr;           // during update()

    // Parameters: the block of the step, and what Solver::SetParams keeps.
    KernelParams m_KernelParams = {};
    PhysParams m_Params;  // the parameters of the step being recorded
    uint64_t m_ParamsRevision = ~uint64_t(0);
    float m_LastRadius = 0.0f;  // of the parameters the rest density was last derived from
    float m_LastFluidRestDistance = 0.0f;
    float m_FluidRestDensity = 0.0f;
    float m_FluidDensityConstraintScale = 0.0f;
    float m_FluidSurfaceConstraintScale = 0.0f;

    uint64_t m_StepCounter = 0;
    uint32_t m_NeighborStride = 0;

    // Per particle, in the order of the domain.
    PhysBufferHandle m_NewPositions;
    PhysBufferHandle m_Densities;
    PhysBufferHandle m_Anisotropy[3];
    PhysBufferHandle m_SmoothPositions;
    PhysBufferHandle m_NormalsTemp;  // raw
    PhysBufferHandle m_ReverseLookup;

    // Per particle, sorted by grid cell.
    PhysBufferHandle m_SortedPositions;
    PhysBufferHandle m_SortedNewPositions;
    PhysBufferHandle m_SortedVelocities;
    PhysBufferHandle m_SortedNewVelocities;
    PhysBufferHandle m_SortedPhases;
    PhysBufferHandle m_SortedNormals;
    PhysBufferHandle m_SortedDensities;
    PhysBufferHandle m_Deltas;  // raw
    PhysBufferHandle m_Potentials;
    PhysBufferHandle m_Curl;
    PhysBufferHandle m_ContactPlanes;
    PhysBufferHandle m_ContactVelocities;
    PhysBufferHandle m_ContactCounts;
    PhysBufferHandle m_Neighbors;
    PhysBufferHandle m_NeighborCounts;

    // The bounds reduction, the grid and the sort of the cell hashes.
    PhysBufferHandle m_BoundsLower;
    PhysBufferHandle m_BoundsUpper;
    PhysBufferHandle m_GroupBoundsLower;
    PhysBufferHandle m_GroupBoundsUpper;
    PhysBufferHandle m_CellStarts;
    PhysBufferHandle m_CellEnds;
    RadixSort m_Sort;

    // Springs: the half-springs, sorted by particle.
    uint64_t m_SpringRevision = ~uint64_t(0);
    uint32_t m_MaxHalfSprings = 0;
    PhysBufferHandle m_HalfSpringOpposites;
    PhysBufferHandle m_HalfSpringLengths;
    PhysBufferHandle m_HalfSpringStiffness;
    PhysBufferHandle m_HalfSpringParticleBegin;
    PhysBufferHandle m_HalfSpringParticleEnd;
    RadixSort m_SpringSort;

    // Inflatables.
    uint32_t m_MaxInflatables = 0;
    PhysBufferHandle m_Pressures;

    // Shapes: their bounds and the hierarchy over them.
    uint64_t m_ShapeRevision = ~uint64_t(0);
    uint32_t m_MaxShapes = 0;
    PhysBufferHandle m_ShapeAabbMin;
    PhysBufferHandle m_ShapeAabbMax;
    Bvh m_ShapeBvh;
    BvhBuilder m_BvhBuilder;

    // Diffuse particles.
    PhysBufferHandle m_DiffusePositionsNew;
    PhysBufferHandle m_DiffuseVelocitiesNew;
    PhysBufferHandle m_NumDiffuseParticlesNew;
    PhysBufferHandle m_Random;
    float m_DiffuseRate = 0.0f;
    float m_DiffuseTimer = 0.0f;

    PhysSolverStageCallbackHandle m_Callbacks[uint32_t(SolverStage::Count)];

    // Last: destroyed first, while the device is still held.
    StageTimerPool m_Timers;
};

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_SOLVER_SOLVER_PIPELINE_H */
