// IMovingFrame (ext/moving_frame.h): FleX's MovingFrame (reference/flex/extensions/flexExtMovingFrame.cpp), host
// math in FleX's order of evaluation (ext/flex_math.h).

#include "ext/ext_objects.h"
#include "ext/flex_math.h"

#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <cmath>
#include <cstring>
#include <new>

namespace ephys {
namespace {

using namespace detail::flex;
using detail::report;

// The MovingFrame of flexExtMovingFrame.cpp, loaded from and stored to MovingFrameState.
struct MovingFrame {
    Vec3 position;
    Quat rotation;
    Vec3 velocity;
    Vec3 omega;
    Vec3 acceleration;
    Vec3 tau;
    Matrix44 delta = Matrix44::Identity();

    MovingFrame(const Vec3& worldTranslation, const Quat& worldRotation)
        : position(worldTranslation), rotation(worldRotation) {}

    explicit MovingFrame(const MovingFrameState& s)
        : position(s.position), rotation(s.rotation), velocity(s.velocity), omega(s.omega),
          acceleration(s.acceleration), tau(s.tau), delta(&s.delta[0][0]) {}

    void store(MovingFrameState& s) const {
        s.position[0] = position.x, s.position[1] = position.y, s.position[2] = position.z;
        s.rotation[0] = rotation.x, s.rotation[1] = rotation.y, s.rotation[2] = rotation.z, s.rotation[3] = rotation.w;
        s.velocity[0] = velocity.x, s.velocity[1] = velocity.y, s.velocity[2] = velocity.z;
        s.omega[0] = omega.x, s.omega[1] = omega.y, s.omega[2] = omega.z;
        s.acceleration[0] = acceleration.x, s.acceleration[1] = acceleration.y, s.acceleration[2] = acceleration.z;
        s.tau[0] = tau.x, s.tau[1] = tau.y, s.tau[2] = tau.z;
        std::memcpy(s.delta, delta.data(), sizeof(s.delta));
    }

    void Move(const Vec3& newPosition, const Quat& newRotation, float dt) {
        const float invDt = 1.0f / dt;

        // calculate new velocity
        const Vec3 newVelocity = (newPosition - position) * invDt;
        const Vec3 newOmega = 2.0f * XYZ(Quat(newRotation - rotation) * Inverse(rotation) * invDt);

        // calculate new acceleration
        const Vec3 newAcceleration = (newVelocity - velocity) * invDt;
        const Vec3 newTau = (newOmega - omega) * invDt;

        // calculate delta transform
        const Matrix44 LocalFromOld = AffineInverse(TranslationMatrix(position) * RotationMatrix(rotation));
        const Matrix44 NewFromLocal = TranslationMatrix(newPosition) * RotationMatrix(newRotation);
        delta = NewFromLocal * LocalFromOld;

        position = newPosition;
        rotation = newRotation;
        velocity = newVelocity;
        omega = newOmega;
        acceleration = newAcceleration;
        tau = newTau;
    }

    Vec3 GetLinearForce() const { return -acceleration; }

    Vec3 GetAngularForce(const Vec3& p) const {
        const Vec3 d = p - position;

        // forces due to rotation
        const Vec3 centrifugalForce = -Cross(omega, Cross(omega, d));
        const Vec3 eulerForce = -Cross(tau, d);

        return centrifugalForce + eulerForce;
    }
};

NVRHI_CLASS_CLSID(PhysMovingFrame, "d50d49fb-d200-4c2b-aa6c-5daf36b60d0e")
class PhysMovingFrame final : public nvrhi::ObjectImpl<IMovingFrame> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysMovingFrame)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysMovingFrame)
    NVRHI_IMPLEMENTS_INTERFACE(IMovingFrame)
    NVRHI_IMPLEMENTS_CLASS(PhysMovingFrame)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysMovingFrame(IPhysDevice* device) : m_Device(device) {}

    const MovingFrameState& getState() const noexcept override { return m_State; }

    nvrhi::FRESULT init(const float* worldTranslation, const float* worldRotation) noexcept override {
        if (!worldTranslation || !worldRotation) return fail("IMovingFrame::init: a null pointer");
        m_State = MovingFrameState();
        MovingFrame(Vec3(worldTranslation), Quat(worldRotation)).store(m_State);
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT update(const float* worldTranslation, const float* worldRotation, float dt) noexcept override {
        if (!worldTranslation || !worldRotation) return fail("IMovingFrame::update: a null pointer");
        if (!(dt > 0.0f) || !std::isfinite(dt))
            return fail("IMovingFrame::update: dt is not a positive finite number");
        MovingFrame frame(m_State);
        frame.Move(Vec3(worldTranslation), Quat(worldRotation), dt);
        frame.store(m_State);
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT apply(float* positions, float* velocities, uint32_t numParticles, float linearScale,
                         float angularScale, float dt) noexcept override {
        if (numParticles && (!positions || !velocities))
            return fail("IMovingFrame::apply: a null array with a count");
        const MovingFrame f(m_State);

        // linear force constant for all particles
        const Vec3 linearForce = f.GetLinearForce() * linearScale;

        for (uint32_t i = 0; i < numParticles; ++i) {
            Vec3 particlePos = Vec3(&positions[size_t(i) * 4]);
            Vec3 particleVel = Vec3(&velocities[size_t(i) * 3]);

            // angular force depends on particles position
            const Vec3 angularForce = f.GetAngularForce(particlePos) * angularScale;

            // transform particle to frame's new location
            particlePos = detail::flex::XYZ(f.delta * Vec4(particlePos, 1.0f));
            particleVel += (linearForce + angularForce) * dt;

            positions[size_t(i) * 4 + 0] = particlePos.x;
            positions[size_t(i) * 4 + 1] = particlePos.y;
            positions[size_t(i) * 4 + 2] = particlePos.z;

            velocities[size_t(i) * 3 + 0] = particleVel.x;
            velocities[size_t(i) * 3 + 1] = particleVel.y;
            velocities[size_t(i) * 3 + 2] = particleVel.z;
        }
        return nvrhi::FS_OK;
    }

 private:
    nvrhi::FRESULT fail(const char* text) const noexcept {
        report(m_Device->getDesc().messageCallback, PhysMessageSeverity::Error, "%s", text);
        return nvrhi::FE_INVALID_ARGS;
    }

    PhysDeviceHandle m_Device;  // for its message callback
    MovingFrameState m_State;
};

}  // namespace

nvrhi::FRESULT detail::createMovingFrame(IPhysDevice* device, const float* worldTranslation,
                                         const float* worldRotation, IMovingFrame** ppFrame) noexcept {
    if (!ppFrame) return nvrhi::FE_INVALID_ARGS;
    *ppFrame = nullptr;
    IPhysMessageCallback* callback = device->getDesc().messageCallback;
    if (!worldTranslation || !worldRotation) {
        report(callback, PhysMessageSeverity::Error, "createMovingFrame: a null pointer");
        return nvrhi::FE_INVALID_ARGS;
    }
    try {
        nvrhi::AutoPtr<PhysMovingFrame> frame = MAKE_RC_OBJ_PTR(PhysMovingFrame, device);
        frame->init(worldTranslation, worldRotation);
        *ppFrame = frame.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        report(callback, PhysMessageSeverity::Error, "createMovingFrame: out of memory");
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        report(callback, PhysMessageSeverity::Error, "createMovingFrame: unexpected error");
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace ephys
