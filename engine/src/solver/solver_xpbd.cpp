#include "solver_pipeline.h"

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <cmath>

// The XPBD solver: SolverPipeline with the seams overridden, behind IPhysSolverXPBD. It is the
// eNvFlexSolverXPBD half of FleX's Solver (FleX/src/Solver.cpp, branch feat-xpbd): what the
// reference does under `mParams.solverMode == eNvFlexSolverXPBD` is done here, by this class, and nowhere
// else.
//
//   reference                                         here
//   Solver::SetSprings: mHalfSpringLambdas            onHalfSpringCapacity
//     (CreateWithZero with the half-spring capacity)
//   Solver::SetInflatables: mInflatableLambdas        onInflatableCapacity
//     (CreateWithZero with the inflatable capacity)
//   Solver::InitParams: kInvStiffnessMin,             beginConstraintIterations (the compliance block,
//     kLogStiffnessRange, kSpringDamping,               KernelParamsXPBD at b1)
//     kVolumeCompliance
//   Solver::UpdateSubstep: clear both multiplier      beginConstraintIterations
//     buffers before the iteration loop
//   Solver::SolveSprings: SolveSpringsXPBD,           configureSolveSprings
//     readWrite[1] lambdas, readOnly[8] x_n
//   Solver::CalculateAndSolveInflatables:             configureInflatableVolume
//     CalculateInflatableVolumeXPBD, readWrite[1]
//   Solver::SolveShapes: SolveShapes(Plastic-         configureSolveShapes
//     Deformation)XPBD
//
// What is different from the reference, and why:
//
// - The compliance values are the solver's (IPhysSolverXPBD::setCompliance), not members of the domain's
//   parameters; FleX kept them in NvFlexParams. They reach the kernels as their own constant buffer at b1
//   (KernelParamsXPBD) instead of registers 32 and 33 of b0 (engine/shaders/PORTING.md). The values and the
//   arithmetic that derives them are the reference's.
// - The block is derived at the start of every substep instead of once per step in InitParams: the same
//   values (a caller cannot change them during a step), recomputed from a 16-byte struct.
// - The multipliers can be read and cleared through the interface. FleX had no API for them.

namespace ephys {
namespace {

using detail::KernelParamsXPBD;

NVRHI_CLASS_CLSID(PhysSolverXPBD, "49b4f19d-6202-4b6d-b034-73bf1e6773ce")
class PhysSolverXPBD final : public detail::SolverPipeline, public nvrhi::ObjectImpl<IPhysSolverXPBD> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysSolverXPBD)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysSolverXPBD)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysSolverXPBD)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysSolver)
    NVRHI_IMPLEMENTS_CLASS(PhysSolverXPBD)
    NVRHI_END_INTERFACE_TABLE()

    PhysSolverXPBD(IPhysDevice* device, detail::KernelLibraryHost* host, IPhysDomain* domain, const SolverDesc& desc)
        : SolverPipeline(device, host, domain, desc) {}

    nvrhi::FRESULT initialize() noexcept { return init(); }

    // ---- IPhysSolver ----------------------------------------------------------------------------------

    const SolverDesc& getDesc() const noexcept override { return desc(); }
    SolverMode getMode() const noexcept override { return SolverMode::XPBD; }
    IPhysDomain* getDomain() const noexcept override { return domain(); }

    nvrhi::FRESULT step(nvrhi::ICommandList* commandList, float dt, uint32_t substeps) noexcept override {
        return update(this, commandList, dt, substeps);
    }

    uint64_t getSubstepCount() const noexcept override { return substepCount(); }

    void setTimersEnabled(bool enabled) noexcept override { enableTimers(enabled); }
    bool areTimersEnabled() const noexcept override { return timersEnabled(); }

    nvrhi::FRESULT setStageCallback(SolverStage stage, IPhysSolverStageCallback* callback,
                                    IPhysSolverStageCallback** ppPrevious) noexcept override {
        return setCallback(stage, callback, ppPrevious);
    }

    IPhysBuffer* getOutput(SolverOutput which) const noexcept override { return output(which); }
    uint32_t getNeighborStride() const noexcept override { return neighborStride(); }

    // ---- IPhysSolverXPBD -------------------------------------------------------------------------------

    void setCompliance(const XpbdComplianceParams& compliance) noexcept override { m_Compliance = compliance; }
    const XpbdComplianceParams& getCompliance() const noexcept override { return m_Compliance; }

    nvrhi::IBuffer* getSpringMultipliers() const noexcept override {
        if (!m_HalfSpringLambdas || domain()->getSprings()->getNumSprings() == 0) return nullptr;
        return m_HalfSpringLambdas->getRHIBuffer();
    }

    nvrhi::IBuffer* getInflatableMultipliers() const noexcept override {
        if (!m_InflatableLambdas || domain()->getInflatables()->getNumInflatables() == 0) return nullptr;
        return m_InflatableLambdas->getRHIBuffer();
    }

    nvrhi::FRESULT resetMultipliers(nvrhi::ICommandList* commandList) noexcept override {
        nvrhi::FRESULT result = clearMultipliers(commandList, m_HalfSpringLambdas, m_MaxHalfSprings);
        if (NVRHI_SUCCEEDED(result)) result = clearMultipliers(commandList, m_InflatableLambdas, m_MaxInflatables);
        if (NVRHI_FAILED(result))
            reportError(result, "IPhysSolverXPBD::resetMultipliers: the clear was not recorded (code %d)", int(result));
        return result;
    }

 protected:
    // ---- The seams -------------------------------------------------------------------------------------

    // Solver::UpdateSubstep, the XPBD block before the iteration loop: the multipliers persist across the
    // iterations of a substep and start at zero in each (Alg. 1 of the paper). FleX clears the whole
    // capacity of a buffer, and only when its constraint set is not empty.
    nvrhi::FRESULT beginConstraintIterations(const StepState& state) override {
        m_ParamsXPBD = compliance();
        if (stepSprings()) clearInt(state.commandList, m_HalfSpringLambdas, m_MaxHalfSprings, 0);
        if (stepInflatables()) clearInt(state.commandList, m_InflatableLambdas, m_MaxInflatables, 0);
        return nvrhi::FS_OK;
    }

    // Solver::SolveSprings, eNvFlexSolverXPBD.
    void configureSolveSprings(const StepState&, KernelDispatchDesc& dispatch) override {
        dispatch.kernel = KernelId::SolveSpringsXPBD;
        dispatch.readWrite[1] = rhi(m_HalfSpringLambdas);
        dispatch.readOnly[8] = sortedPositions();
        bindCompliance(dispatch);
    }

    // Solver::CalculateAndSolveInflatables, eNvFlexSolverXPBD: the first of its two dispatches.
    void configureInflatableVolume(const StepState&, KernelDispatchDesc& dispatch) override {
        dispatch.kernel = KernelId::CalculateInflatableVolumeXPBD;
        dispatch.readWrite[1] = rhi(m_InflatableLambdas);
        bindCompliance(dispatch);
    }

    // Solver::SolveShapes, eNvFlexSolverXPBD: the generic kernels, plastic or not. (FleX's PBD path picks
    // among the vendor specialisations there; this engine has one kernel per formulation either way.)
    void configureSolveShapes(const StepState&, bool plastic, KernelDispatchDesc& dispatch) override {
        dispatch.kernel = plastic ? KernelId::SolveShapesPlasticDeformationXPBD : KernelId::SolveShapesXPBD;
        bindCompliance(dispatch);
    }

    // Solver::SetSprings: mHalfSpringLambdas grows with the half-spring arrays, created with zeros.
    nvrhi::FRESULT onHalfSpringCapacity(uint32_t maxHalfSprings) override {
        PhysBufferHandle buffer;
        const nvrhi::FRESULT result = createScratch("ephys solver half spring lambdas", 4, maxHalfSprings, buffer);
        if (NVRHI_FAILED(result)) return result;
        m_HalfSpringLambdas = buffer;
        m_MaxHalfSprings = maxHalfSprings;
        return nvrhi::FS_OK;
    }

    // Solver::SetInflatables: mInflatableLambdas grows with the inflatables, created with zeros.
    nvrhi::FRESULT onInflatableCapacity(uint32_t maxInflatables) override {
        PhysBufferHandle buffer;
        const nvrhi::FRESULT result = createScratch("ephys solver inflatable lambdas", 4, maxInflatables, buffer);
        if (NVRHI_FAILED(result)) return result;
        m_InflatableLambdas = buffer;
        m_MaxInflatables = maxInflatables;
        return nvrhi::FS_OK;
    }

 private:
    static nvrhi::IBuffer* rhi(const PhysBufferHandle& buffer) { return buffer ? buffer->getRHIBuffer() : nullptr; }

    // Solver::InitParams, the XPBD fields: [0, 1] stiffness coefficient -> [stiffnessMin, stiffnessMax] N/m,
    // geometric; compliance = 1 / stiffness (blog.mmacklin.com, "XPBD slides and stiffness"). The kernels
    // evaluate alpha = kInvStiffnessMin * exp2(-k * kLogStiffnessRange).
    KernelParamsXPBD compliance() const {
        KernelParamsXPBD p = {};
        const float stiffnessMin = std::max(m_Compliance.stiffnessMin, 1.0e-6f);
        const float stiffnessMax = std::max(m_Compliance.stiffnessMax, stiffnessMin);
        p.kInvStiffnessMin = 1.f / stiffnessMin;
        p.kLogStiffnessRange = log2f(stiffnessMax / stiffnessMin);
        p.kSpringDamping = m_Compliance.springDamping;
        p.kVolumeCompliance = m_Compliance.volumeCompliance;
        return p;
    }

    void bindCompliance(KernelDispatchDesc& dispatch) const {
        dispatch.secondConstants = &m_ParamsXPBD;
        dispatch.secondConstantsSize = sizeof(m_ParamsXPBD);
    }

    // Library::ClearBufferInt on one multiplier buffer, outside a step.
    nvrhi::FRESULT clearMultipliers(nvrhi::ICommandList* commandList, const PhysBufferHandle& buffer,
                                    uint32_t count) const noexcept {
        if (!buffer || count == 0) return nvrhi::FS_OK;
        detail::ClearIntParams params;
        params.length = int32_t(count);
        params.value = 0;
        KernelDispatchDesc dispatch;
        dispatch.kernel = KernelId::ClearInt;
        dispatch.gridDim[0] = (count + 255) / 256;
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);
        dispatch.readWrite[0] = buffer->getRHIBuffer();
        return kernels()->dispatch(commandList, dispatch);
    }

    XpbdComplianceParams m_Compliance;
    KernelParamsXPBD m_ParamsXPBD = {};  // the block of the substep being recorded

    // FleX's mHalfSpringLambdas and mInflatableLambdas, and the capacities they were created with.
    PhysBufferHandle m_HalfSpringLambdas;
    PhysBufferHandle m_InflatableLambdas;
    uint32_t m_MaxHalfSprings = 0;
    uint32_t m_MaxInflatables = 0;
};

}  // namespace

namespace detail {

StageTimerPool* stageTimersOfXPBD(IPhysSolver* solver) noexcept {
    if (!solver) return nullptr;
    nvrhi::AutoPtr<PhysSolverXPBD> xpbd;
    if (solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd)) == nvrhi::FS_OK && xpbd) return &xpbd->stageTimers();
    return nullptr;
}

nvrhi::FRESULT createSolverXPBD(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                                const SolverDesc& desc, IPhysSolver** ppSolver) {
    nvrhi::AutoPtr<PhysSolverXPBD> solver = nvrhi::TakeOver(MAKE_RC_OBJ(PhysSolverXPBD, device, host, domain, desc));
    const nvrhi::FRESULT result = solver->initialize();
    if (NVRHI_FAILED(result)) return result;
    // The IPhysSolver base of an IPhysSolverXPBD: the object is returned through its most derived interface.
    IPhysSolverXPBD* object = solver.Detach();
    *ppSolver = object;
    return nvrhi::FS_OK;
}

}  // namespace detail
}  // namespace ephys
