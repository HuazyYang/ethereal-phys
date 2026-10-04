#ifndef ETHEREAL_PHYS_EXT_MOVING_FRAME_H
#define ETHEREAL_PHYS_EXT_MOVING_FRAME_H

// IMovingFrame: FleX's moving frame (NvFlexExtMovingFrame, reference/flex/extensions/flexExtMovingFrame.cpp;
// step 9 of the migration plan), the helper for simulating particles in the local space of a moving parent (a
// character, an emitter) while letting through a controlled part of the inertial forces of its motion.
// Created by IPhysDevice::createMovingFrame.
//
//     frame->init(translation, rotation);
//     every frame:
//         frame->update(newTranslation, newRotation, dt);
//         container->mapParticleData(data);          // or the application's own copies of the arrays
//         frame->apply(data.particles, data.velocities, count, linearScale, angularScale, dt);
//         container->unmapParticleData();
//         ... push, step
//
// apply works on host arrays, as FleX's did: FleX's own use of it (demo/scenes/localspacecloth.h,
// localspacefluid.h) runs on the host copies of the particle arrays the demo maps every frame, and a container
// keeps exactly those (IPhysContainer::mapParticleData). On the host the arithmetic is FleX's, in its order
// (the parity test compares it with FleX's extension library bit for bit); a kernel would need another
// constant block and an index range the application has on the host anyway, and would make the result depend
// on the GPU's rounding.
//
// Rotations are quaternions x y z w (w real). Not thread-safe.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// The state of a moving frame: FleX's NvFlexExtMovingFrame, member for member.
struct MovingFrameState {
    float position[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // x y z w
    float velocity[3] = {0.0f, 0.0f, 0.0f};        // of the position, from the last two updates
    float omega[3] = {0.0f, 0.0f, 0.0f};           // the angular velocity
    float acceleration[3] = {0.0f, 0.0f, 0.0f};
    float tau[3] = {0.0f, 0.0f, 0.0f};  // the angular acceleration
    // The transform that took the frame from its previous pose to its current one: a column-major 4 x 4
    // matrix for column vectors (delta[column][row]); apply moves particles by it.
    float delta[4][4] = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 1.0f}};
};

static_assert(sizeof(MovingFrameState) == 140, "MovingFrameState layout changed: bump c_HeaderVersion");
static_assert(offsetof(MovingFrameState, rotation) == 12);  // after position[3]
static_assert(offsetof(MovingFrameState, velocity) == 28);  // after rotation[4]
static_assert(offsetof(MovingFrameState, omega) == 40);
static_assert(offsetof(MovingFrameState, acceleration) == 52);
static_assert(offsetof(MovingFrameState, tau) == 64);
static_assert(offsetof(MovingFrameState, delta) == 76);  // after tau[3]

NVRHI_IID(IMovingFrame, "0912d258-0fff-4dbd-9f87-a54f7fb44523")
struct IMovingFrame : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IMovingFrame)

    // A view of the state the frame keeps.
    [[nodiscard]] virtual const MovingFrameState& getState() const noexcept = 0;

    // NvFlexExtMovingFrameInit: the frame at a pose (translation: 3 floats, rotation: 4), at rest, with an
    // identity delta. FE_INVALID_ARGS for a null pointer.
    virtual nvrhi::FRESULT init(const float* worldTranslation, const float* worldRotation) noexcept = 0;

    // NvFlexExtMovingFrameUpdate: moves the frame to a new pose dt seconds after the last one, and derives the
    // velocity, the angular velocity, both accelerations and the delta transform from the two poses.
    // FE_INVALID_ARGS for a null pointer or a dt that is not a positive finite number (FleX divided by it).
    virtual nvrhi::FRESULT update(const float* worldTranslation, const float* worldRotation, float dt) noexcept = 0;

    // NvFlexExtMovingFrameApply: moves numParticles particles (positions: 4 floats each, x y z 1/mass;
    // velocities: 3 floats each) by the delta transform, and adds to each velocity dt times the inertial
    // acceleration of the frame, the linear part (-acceleration) scaled by linearScale and the angular part
    // (centrifugal and Euler, at the particle's position before the move) by angularScale. A scale of 0 is a
    // purely local simulation, 1 one that feels all of the frame's motion. FE_INVALID_ARGS for a null array
    // with a count.
    virtual nvrhi::FRESULT apply(_Inout_updates_opt_(numParticles * 4) float* positions,
                                 _Inout_updates_opt_(numParticles * 3) float* velocities, uint32_t numParticles,
                                 float linearScale, float angularScale, float dt) noexcept = 0;
};
typedef nvrhi::AutoPtr<IMovingFrame> MovingFrameHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_EXT_MOVING_FRAME_H */
