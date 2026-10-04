// IForceFieldSet (ext/force_field.h): FleX's NvFlexExtForceFieldCallback (reference/flex/extensions/dx/
// flexExt.cpp, both the ORG and the REV variant, which differ only in the D3D context they dispatch through), as
// a stage callback of the solver that records the kernel UpdateForceFields (engine/shaders/ext).

#include "ext/ext_objects.h"

#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <cmath>
#include <cstring>
#include <new>
#include <vector>

namespace ephys {
namespace {

using detail::report;

// FleX's FlexExtConstParams (flexExt_dx_common.h): the cbuffer of UpdateForceFields.hlsl.
struct ForceFieldConstants {
    int32_t numParticles = 0;
    int32_t numForceFields = 0;
    float dt = 0.0f;
    float _pad = 0.0f;
};
static_assert(sizeof(ForceFieldConstants) == 16);

class PhysForceFieldSet;

// What the solver holds: a forwarder with a pointer back to the set that the set clears when it goes. It keeps
// the callback that was registered at the stage before it (the chain), not the set: the set keeps the solver,
// and the solver keeps this, so a reference back would be a cycle.
NVRHI_CLASS_CLSID(ForceFieldStageCallback, "a63027f4-363b-406f-af2e-86ed9c78d92c")
class ForceFieldStageCallback final : public nvrhi::ObjectImpl<IPhysSolverStageCallback> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(ForceFieldStageCallback)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(ForceFieldStageCallback)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysSolverStageCallback)
    NVRHI_IMPLEMENTS_CLASS(ForceFieldStageCallback)
    NVRHI_END_INTERFACE_TABLE()

    explicit ForceFieldStageCallback(PhysForceFieldSet* owner) : m_Owner(owner) {}

    void onSolverStage(const SolverStageContext& context) noexcept override;

    void setPrevious(IPhysSolverStageCallback* previous) { m_Previous = previous; }
    [[nodiscard]] IPhysSolverStageCallback* previous() const { return m_Previous; }
    void detach() { m_Owner = nullptr; }

 private:
    PhysForceFieldSet* m_Owner;
    PhysSolverStageCallbackHandle m_Previous;
};

NVRHI_CLASS_CLSID(PhysForceFieldSet, "22c47d8a-d699-4794-9b00-19bacfa95290")
class PhysForceFieldSet final : public nvrhi::ObjectImpl<IForceFieldSet> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysForceFieldSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysForceFieldSet)
    NVRHI_IMPLEMENTS_INTERFACE(IForceFieldSet)
    NVRHI_IMPLEMENTS_CLASS(PhysForceFieldSet)
    NVRHI_END_INTERFACE_TABLE()

    PhysForceFieldSet(IPhysDevice* device, IPhysSolver* solver) : m_Device(device), m_Solver(solver) {}

    ~PhysForceFieldSet() {
        if (!m_Callback) return;
        // Give the stage back: to the callback before ours when ours is the one registered, else leave ours in
        // the chain of whoever registered after it, as a forwarder.
        PhysSolverStageCallbackHandle current;
        m_Solver->setStageCallback(SolverStage::UpdateEnd, nullptr, &current);
        if (current.Get() == static_cast<IPhysSolverStageCallback*>(m_Callback.Get()))
            m_Solver->setStageCallback(SolverStage::UpdateEnd, m_Callback->previous(), nullptr);
        else
            m_Solver->setStageCallback(SolverStage::UpdateEnd, current, nullptr);
        m_Callback->detach();
    }

    nvrhi::FRESULT registerOnSolver() {
        m_Callback = MAKE_RC_OBJ_PTR(ForceFieldStageCallback, this);
        PhysSolverStageCallbackHandle previous;
        const nvrhi::FRESULT result = m_Solver->setStageCallback(SolverStage::UpdateEnd, m_Callback, &previous);
        if (NVRHI_FAILED(result)) {
            m_Callback = nullptr;
            return result;
        }
        m_Callback->setPrevious(previous);
        return nvrhi::FS_OK;
    }

    IPhysSolver* getSolver() const noexcept override { return m_Solver; }

    nvrhi::FRESULT setForceFields(const ForceField* fields, uint32_t count) noexcept override {
        if (count && !fields) return fail("IForceFieldSet::setForceFields: a null array with a count");
        for (uint32_t i = 0; i < count; ++i) {
            const ForceField& f = fields[i];
            if (f.mode != ForceMode::Force && f.mode != ForceMode::Impulse && f.mode != ForceMode::VelocityChange)
                return fail("IForceFieldSet::setForceFields: the mode of field %u is not a ForceMode", i);
            if (!std::isfinite(f.position[0]) || !std::isfinite(f.position[1]) || !std::isfinite(f.position[2]) ||
                !std::isfinite(f.radius) || !std::isfinite(f.strength))
                return fail("IForceFieldSet::setForceFields: field %u has a value that is not finite", i);
        }
        try {
            // FleX grew its device buffer to exactly the count when it had to.
            if (count > m_Capacity) {
                PhysBufferDesc desc;
                desc.elementCount = count;
                desc.elementStride = sizeof(ForceField);
                desc.use = BufferUse::Structured;
                desc.debugName = "ephys force fields";
                PhysBufferHandle buffer;
                const nvrhi::FRESULT result = m_Device->createBuffer(desc, &buffer);
                if (NVRHI_FAILED(result)) return result;
                m_Buffer = buffer;
                m_Capacity = count;
            }
            m_Fields.assign(fields, fields + count);
            // Only the flag's byte counts: the kernel reads the low byte of the word, and the padding is zero.
            for (ForceField& f : m_Fields) std::memset(f._pad, 0, sizeof(f._pad));
            m_Dirty = count != 0;
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            report(callback(), PhysMessageSeverity::Error, "IForceFieldSet::setForceFields: out of memory");
            return nvrhi::FE_OUT_OF_MEMORY;
        }
    }

    const ForceField* getForceFields() const noexcept override { return m_Fields.empty() ? nullptr : m_Fields.data(); }
    uint32_t getForceFieldCount() const noexcept override { return uint32_t(m_Fields.size()); }

    // NvFlexExtForceFieldCallback::applyForceFields, at SolverStage::UpdateEnd.
    void apply(const SolverStageContext& context) noexcept {
        if (!context.numActive || m_Fields.empty()) return;
        const uint32_t n = uint32_t(m_Fields.size());
        if (m_Dirty) {
            if (NVRHI_FAILED(m_Device->writeBuffer(context.commandList, m_Buffer, m_Fields.data(), 0, n))) return;
            m_Dirty = false;
        }

        ForceFieldConstants constants;
        constants.numParticles = int32_t(context.numActive);
        constants.numForceFields = int32_t(n);
        constants.dt = context.dt;

        KernelDispatchDesc desc;
        desc.kernel = KernelId::UpdateForceFields;
        desc.gridDim[0] = (context.numActive + 255) / 256;
        desc.rootConstants = &constants;
        desc.rootConstantsSize = sizeof(constants);
        desc.readOnly[0] = context.positions;
        desc.readOnly[1] = m_Buffer->getRHIBuffer();
        desc.readWrite[0] = context.velocities;
        // A failure is reported by the kernel library.
        m_Device->getKernels()->dispatch(context.commandList, desc);
    }

 private:
    [[nodiscard]] IPhysMessageCallback* callback() const { return m_Device->getDesc().messageCallback; }

    template <typename... Args>
    nvrhi::FRESULT fail(const char* format, Args... args) const noexcept {
        report(callback(), PhysMessageSeverity::Error, format, args...);
        return nvrhi::FE_INVALID_ARGS;
    }

    IPhysDevice* m_Device;  // the solver keeps it alive
    PhysSolverHandle m_Solver;
    nvrhi::AutoPtr<ForceFieldStageCallback> m_Callback;
    std::vector<ForceField> m_Fields;
    PhysBufferHandle m_Buffer;
    uint32_t m_Capacity = 0;
    bool m_Dirty = false;
};

void ForceFieldStageCallback::onSolverStage(const SolverStageContext& context) noexcept {
    if (m_Owner) m_Owner->apply(context);
    if (m_Previous) m_Previous->onSolverStage(context);
}

}  // namespace

nvrhi::FRESULT detail::createForceFieldSet(IPhysDevice* device, IPhysSolver* solver, IForceFieldSet** ppSet) noexcept {
    if (!ppSet) return nvrhi::FE_INVALID_ARGS;
    *ppSet = nullptr;
    IPhysMessageCallback* callback = device->getDesc().messageCallback;
    try {
        nvrhi::AutoPtr<PhysForceFieldSet> set = MAKE_RC_OBJ_PTR(PhysForceFieldSet, device, solver);
        const nvrhi::FRESULT result = set->registerOnSolver();
        if (NVRHI_FAILED(result)) {
            report(callback, PhysMessageSeverity::Error, "createForceFieldSet: the stage callback cannot be registered");
            return result;
        }
        *ppSet = set.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        report(callback, PhysMessageSeverity::Error, "createForceFieldSet: out of memory");
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        report(callback, PhysMessageSeverity::Error, "createForceFieldSet: unexpected error");
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace ephys
