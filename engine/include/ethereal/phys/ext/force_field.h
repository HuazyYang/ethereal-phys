#ifndef ETHEREAL_PHYS_EXT_FORCE_FIELD_H
#define ETHEREAL_PHYS_EXT_FORCE_FIELD_H

// IForceFieldSet: FleX's radial force fields (NvFlexExtForceField, NvFlexExtForceFieldCallback,
// reference/flex/extensions/dx/flexExt.cpp; step 9 of the migration plan). Created for a solver by
// IPhysDevice::createForceFieldSet.
//
// How it hooks into a step. FleX's callback registered a function at eNvFlexStageUpdateEnd that launched the
// kernel flexExt.UpdateForceFields on the solver's device pointers. Here the set implements
// IPhysSolverStageCallback internally and registers it at SolverStage::UpdateEnd of its solver when it is
// created: in the last substep of every step, right after the velocities were derived from the positions,
// it records a write of the fields (when they changed) and one dispatch of the kernel UpdateForceFields
// (engine/shaders/ext/UpdateForceFields.hlsl, KernelId::UpdateForceFields) on the command list the step is
// recorded on (SolverStageContext::commandList). The kernel changes the velocities of the active particles in
// the solver's internal order; dt is the length of the whole step, as FleX passed it at that stage.
//
// What a field does to a particle at distance d < radius from its position, with s = strength, times
// (1 - d / radius) with a linear falloff, and n the direction from the field to the particle (the zero vector
// at the centre):
//
//   ForceMode::Force           v += n * s * dt / m
//   ForceMode::Impulse         v += n * s / m
//   ForceMode::VelocityChange  v += n * s
//
// (1/m is the w of the particle's position.) A fixed particle (1/m = 0) stays at rest under every mode: the
// stages of the step after UpdateEnd leave it without a velocity. A negative strength pulls particles in. Fields
// add up, in the order given.
//
// The registration chains: the callback that was registered at UpdateEnd before is called after the set's
// dispatch (FleX replaced it). Releasing the set unregisters it, giving the stage back to that callback; if
// another callback was registered after the set, the set's callback stays in its chain and only forwards.
// The set keeps its solver alive.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/solver.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// How a field changes a velocity (NvFlexExtForceMode). Four bytes: the kernel reads it as an int.
enum class ForceMode : int32_t {
    Force = 0,           // the strength is a force: scaled by dt and 1/m
    Impulse = 1,         // the strength is an impulse: scaled by 1/m
    VelocityChange = 2,  // the strength is a change of velocity
};

// One radial force field (NvFlexExtForceField), in the space of the solver's domain. The layout is FleX's and
// the kernel's (28 bytes, the element of a structured buffer): FleX's bool was followed by three bytes of
// padding that its kernel read as part of a 4-byte bool, so garbage there turned a falloff on. The padding is
// explicit here and the kernel reads the low byte only.
struct ForceField {
    float position[3] = {0.0f, 0.0f, 0.0f};  // the centre
    float radius = 0.0f;                      // particles at this distance or farther are not affected
    float strength = 0.0f;                    // negative: towards the centre
    ForceMode mode = ForceMode::Force;
    bool linearFalloff = false;  // scale the strength by 1 - distance / radius
    uint8_t _pad[3] = {0, 0, 0};  // explicit
};

static_assert(sizeof(ForceField) == 28, "ForceField layout changed: bump c_HeaderVersion (and the kernel)");
static_assert(alignof(ForceField) == 4);
static_assert(offsetof(ForceField, radius) == 12);  // after position[3]
static_assert(offsetof(ForceField, strength) == 16);
static_assert(offsetof(ForceField, mode) == 20);
static_assert(offsetof(ForceField, linearFalloff) == 24);
static_assert(offsetof(ForceField, _pad) == 25);

// The force fields of one solver. Not thread-safe, like the device.
NVRHI_IID(IForceFieldSet, "60dd1477-a4da-46e2-bf98-ca71eeeb129c")
struct IForceFieldSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IForceFieldSet)

    // The solver the set is registered on. Does not AddRef: the set keeps it alive.
    [[nodiscard]] virtual IPhysSolver* getSolver() const noexcept = 0;

    // NvFlexExtSetForceFields: replaces the fields with count fields (host memory, read before the call
    // returns; FleX also took device memory, which has no meaning here). They apply from the next step on;
    // with count 0 no kernel is dispatched. FE_INVALID_ARGS and a message for a null pointer with a count, a
    // mode that is not a ForceMode, or a value that is not finite; nothing changes then.
    virtual nvrhi::FRESULT setForceFields(_In_reads_opt_(count) const ForceField* fields, uint32_t count) noexcept = 0;

    // The fields as last set: a view of the set's copy, valid until the next setForceFields; nullptr with 0.
    [[nodiscard]] virtual const ForceField* getForceFields() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getForceFieldCount() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IForceFieldSet> ForceFieldSetHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_EXT_FORCE_FIELD_H */
