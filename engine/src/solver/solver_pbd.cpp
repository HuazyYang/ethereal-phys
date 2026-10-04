#include "solver_pipeline.h"

#include "data_model.h"
#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <new>

// The PBD solver: SolverPipeline with every seam at its default, behind IPhysSolver. And the factory that
// IPhysDevice::createSolver calls.
//
// The XPBD solver of step 6 is a second class of this shape: it derives from SolverPipeline and from
// nvrhi::ObjectImpl<IPhysSolverXPBD>, answers IID_IPhysSolverXPBD and IID_IPhysSolver, overrides the seams
// (solver_pipeline.h) and owns the compliance block and the multiplier buffers.

namespace ephys {
namespace {

using detail::report;

NVRHI_CLASS_CLSID(PhysSolverPBD, "86cbfefd-159a-4dd6-8e20-b609f8c7508d")
class PhysSolverPBD final : public detail::SolverPipeline, public nvrhi::ObjectImpl<IPhysSolver> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysSolverPBD)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysSolverPBD)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysSolver)
    NVRHI_IMPLEMENTS_CLASS(PhysSolverPBD)
    NVRHI_END_INTERFACE_TABLE()

    PhysSolverPBD(IPhysDevice* device, detail::KernelLibraryHost* host, IPhysDomain* domain, const SolverDesc& desc)
        : SolverPipeline(device, host, domain, desc) {}

    nvrhi::FRESULT initialize() noexcept { return init(); }

    const SolverDesc& getDesc() const noexcept override { return desc(); }
    SolverMode getMode() const noexcept override { return SolverMode::PBD; }
    IPhysDomain* getDomain() const noexcept override { return domain(); }

    nvrhi::FRESULT step(nvrhi::ICommandList* commandList, float dt, uint32_t substeps) noexcept override {
        return update(this, commandList, dt, substeps);
    }

    uint64_t getSubstepCount() const noexcept override { return substepCount(); }

    void setTimersEnabled(bool enabled) noexcept override { m_TimersEnabled = enabled; }
    bool areTimersEnabled() const noexcept override { return m_TimersEnabled; }

    nvrhi::FRESULT setStageCallback(SolverStage stage, IPhysSolverStageCallback* callback,
                                    IPhysSolverStageCallback** ppPrevious) noexcept override {
        return setCallback(stage, callback, ppPrevious);
    }

    IPhysBuffer* getOutput(SolverOutput which) const noexcept override { return output(which); }
    uint32_t getNeighborStride() const noexcept override { return neighborStride(); }
};

}  // namespace

namespace detail {

nvrhi::FRESULT createSolver(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                            const SolverDesc& desc, IPhysSolver** ppSolver) noexcept {
    if (!ppSolver) return nvrhi::FE_INVALID_ARGS;
    *ppSolver = nullptr;

    IPhysMessageCallback* callback = device->getDesc().messageCallback;
    auto fail = [&](nvrhi::FRESULT code, const char* text) {
        report(callback, PhysMessageSeverity::Error, "createSolver: %s", text);
        return code;
    };

    if (!domain) return fail(nvrhi::FE_INVALID_ARGS, "the domain is null");
    if (domain->getDevice() != device) return fail(nvrhi::FE_INVALID_ARGS, "the domain belongs to another device");
    if (desc.mode != SolverMode::PBD && desc.mode != SolverMode::XPBD)
        return fail(nvrhi::FE_INVALID_ARGS, "SolverDesc::mode is not a solver mode");
    if (uint8_t(desc.featureMode) > uint8_t(FeatureMode::SimpleFluids))
        return fail(nvrhi::FE_INVALID_ARGS, "SolverDesc::featureMode is not a feature mode");
    if (desc.maxNeighborsPerParticle == 0 || desc.maxContactsPerParticle == 0)
        return fail(nvrhi::FE_INVALID_ARGS, "maxNeighborsPerParticle and maxContactsPerParticle must not be 0");

    // The bounds of the particles are reduced in two levels of 128 and one last thread group of 128.
    const uint64_t maxParticles = domain->getParticles()->getMaxParticles();
    if (maxParticles > 128ull * 128ull * 128ull)
        return fail(nvrhi::FE_INVALID_ARGS, "a solver handles at most 2097152 particles");
    if ((maxParticles + 255) / 256 * 256 * desc.maxNeighborsPerParticle > c_MaxArrayCount ||
        maxParticles * desc.maxContactsPerParticle > c_MaxArrayCount)
        return fail(nvrhi::FE_INVALID_ARGS, "the neighbour or contact capacity is too large for this many particles");

    if (desc.mode == SolverMode::XPBD)
        return fail(nvrhi::FE_NOT_IMPLEMENT, "SolverMode::XPBD is not implemented yet (step 6 of the migration plan)");

    try {
        nvrhi::AutoPtr<PhysSolverPBD> solver = nvrhi::TakeOver(MAKE_RC_OBJ(PhysSolverPBD, device, host, domain, desc));
        // A buffer that cannot be created has been reported by the device.
        const nvrhi::FRESULT result = solver->initialize();
        if (NVRHI_FAILED(result)) return result;
        *ppSolver = solver.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return fail(nvrhi::FE_OUT_OF_MEMORY, "out of memory");
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace detail
}  // namespace ephys
