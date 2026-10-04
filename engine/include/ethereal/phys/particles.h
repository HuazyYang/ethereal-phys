#ifndef ETHEREAL_PHYS_PARTICLES_H
#define ETHEREAL_PHYS_PARTICLES_H

// IParticleState and IDiffuseParticleState: the particles of a domain (domain.h).
//
// They are data: they own the per-particle arrays and dispatch no kernel. A solver reads and writes the
// arrays through this interface and keeps everything it derives from them (the sorted copies, the grid, the
// deltas, contacts and neighbours) to itself.
//
// They replace the particle half of FleX's NvFlexSolver: NvFlexSet/GetParticles, Velocities, Phases,
// Normals, RestParticles, Active and ActiveCount become upload / download of one ParticleArray, and
// NvFlexSet/GetDiffuseParticles those of IDiffuseParticleState. Host memory is an IPhysBuffer with
// BufferUse::Upload (to send) or BufferUse::Readback (to receive), and a transfer is a GPU copy recorded on
// a command list, as everywhere in the engine (device.h).
//
// The densities, the anisotropy vectors and the smoothed positions of FleX (NvFlexGetDensities,
// GetAnisotropy, GetSmoothParticles) are not here: a solver produces them and they belong to it.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/params.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// The per-particle arrays. Each has getMaxParticles() elements; element i of every array belongs to
// particle i.
//
//   array         element          bytes  FleX
//   Position      float4 x y z 1/m    16  NvFlexSetParticles: the w component is the inverse mass
//   Velocity      float3              12  NvFlexSetVelocities
//   Phase         int32                4  NvFlexSetPhases: makePhase() below
//   Normal        float4              16  NvFlexSetNormals: for a particle of a rigid, w is the (negative)
//                                         signed distance inside its shape; a solver overwrites the normals
//                                         of particles of dynamic triangles
//   RestPosition  float4              16  NvFlexSetRestParticles: used by PhaseSelfCollideFilter
//   ActiveIndex   int32                4  NvFlexSetActive: the indices of the particles that are simulated;
//                                         the first getActiveCount() elements count
//
// The sizes are those of the arrays the FleX solver keeps (reference/flex/src/private/Solver.h: mPositions,
// mVelocities, mPhases, mNormals, mRestPositions, mActiveIndices) and of the buffers the kernels declare:
// a velocity is 12 bytes in the array although the kernels work on float4 copies of it.
enum class ParticleArray : uint8_t {
    Position = 0,
    Velocity = 1,
    Phase = 2,
    Normal = 3,
    RestPosition = 4,
    ActiveIndex = 5,
    Count = 6,
};

// The size of one element of a particle array in bytes; 0 for a value that is not an array.
constexpr uint32_t particleArrayStride(ParticleArray array) noexcept {
    return array == ParticleArray::Position || array == ParticleArray::Normal ||
                   array == ParticleArray::RestPosition
               ? 16u
           : array == ParticleArray::Velocity                                     ? 12u
           : array == ParticleArray::Phase || array == ParticleArray::ActiveIndex ? 4u
                                                                                  : 0u;
}

// The bits of a particle phase (FleX's NvFlexPhase). Build one with makePhase().
enum PhaseFlags : int32_t {
    PhaseGroupMask = 0x000fffff,         // bits [0, 19]: the group of the particle, for collision filtering
    PhaseFlagsMask = 0x00f00000,         // bits [20, 23]: how the particle behaves
    PhaseShapeChannelMask = 0x7f000000,  // bits [24, 30]: the shape collision channels of the particle

    PhaseSelfCollide = 1 << 20,        // the particle interacts with particles of its own group
    PhaseSelfCollideFilter = 1 << 21,  // ... but not with those closer than the radius in the rest pose;
                                       // needs ParticleArray::RestPosition
    PhaseFluid = 1 << 22,              // the particle generates fluid density constraints for its neighbours
    PhaseUnused = 1 << 23,             // reserved

    PhaseShapeChannel0 = 1 << 24,  // the particle collides with shapes that have channel 0 set
    PhaseShapeChannel1 = 1 << 25,  // (makeShapeFlagsWithChannels, collision.h)
    PhaseShapeChannel2 = 1 << 26,
    PhaseShapeChannel3 = 1 << 27,
    PhaseShapeChannel4 = 1 << 28,
    PhaseShapeChannel5 = 1 << 29,
    PhaseShapeChannel6 = 1 << 30,
};

// A phase: the group (below 2^20), a combination of PhaseSelfCollide, PhaseSelfCollideFilter and PhaseFluid,
// and a combination of PhaseShapeChannel*. A particle collides with a shape when they share a channel.
// (FleX's NvFlexMakePhaseWithChannels.)
constexpr int32_t makePhaseWithChannels(int32_t group, int32_t particleFlags, int32_t shapeChannels) noexcept {
    return (group & PhaseGroupMask) | (particleFlags & PhaseFlagsMask) |
           (shapeChannels & PhaseShapeChannelMask);
}

// A phase with every shape channel set (FleX's NvFlexMakePhase).
constexpr int32_t makePhase(int32_t group, int32_t particleFlags) noexcept {
    return makePhaseWithChannels(group, particleFlags, PhaseShapeChannelMask);
}

static_assert(particleArrayStride(ParticleArray::Position) == 16 &&
              particleArrayStride(ParticleArray::Velocity) == 12);
static_assert(particleArrayStride(ParticleArray::Phase) == 4 && particleArrayStride(ParticleArray::Count) == 0);
static_assert(makePhase(3, PhaseSelfCollide | PhaseFluid) == 0x7f500003);

// The particles of a domain (IPhysDomain::getParticles). Not thread-safe, like the device.
//
// Arrays. getArray() is the array the kernels read and write: an IPhysBuffer of getMaxParticles() elements
// (or more, for a substituted one) of particleArrayStride() bytes, usable as a structured buffer with
// UAVs. At creation the arrays are zero, except ActiveIndex, which holds 0, 1, 2, ... (as in FleX), and the
// active count is 0.
//
// Command lists. A method that records GPU work takes an nvrhi::ICommandList*: the open command list of
// the application, or nullptr for the device's own (IPhysDevice, "The device owns one command list").
//
// Rendering without a copy. The application creates a vertex buffer with canHaveUAVs, adopts it
// (IPhysDevice::adoptBuffer) and calls setArray(Position, buffer): the buffer then IS the position array.
// A solver writes the final positions straight into the buffer the renderer binds, and nothing is copied
// or registered. That needs PhysDeviceInfo::supportsRenderSharedBuffers. Where it is false (D3D11: a
// structured buffer cannot be a vertex buffer) setArray returns FE_UNSUPPORTED and the application uses
// the fallback instead: setRenderArray(Position, buffer) names its vertex buffer, and
// updateRenderArrays(commandList) copies the array into it before drawing. That copy is device to device.
// (The SPH fluid sample's fallback goes through host memory, because there the simulation and the renderer
// are two devices. Here they are one: do not copy that shape.) An application that calls setArray, falls
// back to setRenderArray when it fails, and calls updateRenderArrays before drawing, is right on every
// backend; areRenderArraysShared() says whether the last step does anything.
NVRHI_IID(IParticleState, "07ef01f1-5cbf-4d83-ab62-7b75f623ce51")
struct IParticleState : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IParticleState)

    // The capacity: DomainDesc::maxParticles. It is fixed at creation, as in FleX.
    [[nodiscard]] virtual uint32_t getMaxParticles() const noexcept = 0;

    // The number of active particles: how many elements of ParticleArray::ActiveIndex count
    // (NvFlexGetActiveCount / NvFlexSetActiveCount). FE_INVALID_ARGS for more than getMaxParticles().
    [[nodiscard]] virtual uint32_t getActiveCount() const noexcept = 0;
    virtual nvrhi::FRESULT setActiveCount(uint32_t count) noexcept = 0;

    // The array. Does not AddRef: the state keeps it alive until it is replaced (setArray, resetArray;
    // getLayoutRevision() then advances). nullptr for a value that is not an array.
    [[nodiscard]] virtual IPhysBuffer* getArray(ParticleArray array) const noexcept = 0;

    // Makes a buffer of the application the array. The state keeps a reference to it. The buffer needs at
    // least getMaxParticles() elements of particleArrayStride(array) bytes, a struct stride of that size
    // and UAVs (FE_INVALID_ARGS otherwise). The contents of the array so far are copied into it (recorded
    // on commandList), and the layout revision and the content revision advance. A buffer named with
    // setRenderArray for the array is dropped: the array is its own render array now.
    // FE_UNSUPPORTED where PhysDeviceInfo::supportsRenderSharedBuffers is false.
    virtual nvrhi::FRESULT setArray(_In_opt_ nvrhi::ICommandList* commandList, ParticleArray array,
                                    IPhysBuffer* buffer) noexcept = 0;

    // Ends setArray: the array is a buffer of the engine again, with the contents of the application's
    // buffer copied into it (recorded on commandList). FS_OK and nothing happens when the array was not
    // substituted.
    virtual nvrhi::FRESULT resetArray(_In_opt_ nvrhi::ICommandList* commandList,
                                      ParticleArray array) noexcept = 0;

    // True between setArray and resetArray.
    [[nodiscard]] virtual bool isArraySubstituted(ParticleArray array) const noexcept = 0;

    // Host to device: records a copy from hostBuffer into the array and advances the content revision
    // (NvFlexSetParticles and its siblings). region is in elements: srcOffset in hostBuffer, dstOffset in
    // the array. A null region copies from the start of both, as many elements as the smaller of
    // hostBuffer and the capacity holds (FleX's NULL NvFlexCopyDesc).
    //
    // hostBuffer is usually an Upload buffer; a device buffer works as well (a device to device copy). Its
    // element stride must be particleArrayStride(array). FE_INVALID_ARGS, a message and no copy for a
    // stride that differs, a region that leaves hostBuffer or reaches past getMaxParticles(), a null or
    // Readback hostBuffer.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList, ParticleArray array,
                                  IPhysBuffer* hostBuffer, _In_opt_ const CopyRegion* region) noexcept = 0;

    // Device to host: records a copy from the array into hostBuffer (NvFlexGetParticles and its siblings).
    // srcOffset is in the array, dstOffset in hostBuffer; a null region as in upload. hostBuffer is
    // usually a Readback buffer: map it after the command list has executed (IPhysDevice::waitForIdle).
    // The same errors as upload, with an Upload hostBuffer refused.
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList, ParticleArray array,
                                    IPhysBuffer* hostBuffer, _In_opt_ const CopyRegion* region) noexcept = 0;

    // The fallback where setArray is not supported (it works on every backend): names the buffer of the
    // application that updateRenderArrays copies the array into, or ends that with nullptr. The state
    // keeps a reference. The buffer needs at least getMaxParticles() elements of particleArrayStride(array)
    // bytes (FE_INVALID_ARGS otherwise). FE_INVALID_ARGS while the array is substituted: it is shared
    // already.
    virtual nvrhi::FRESULT setRenderArray(ParticleArray array, _In_opt_ IPhysBuffer* buffer) noexcept = 0;

    // The buffer a renderer binds for the array: the substituted buffer (setArray), else the buffer named
    // with setRenderArray, else nullptr. Does not AddRef.
    [[nodiscard]] virtual IPhysBuffer* getRenderArray(ParticleArray array) const noexcept = 0;

    // True when no array has a buffer named with setRenderArray: every render array is the array itself
    // and updateRenderArrays has nothing to do.
    [[nodiscard]] virtual bool areRenderArraysShared() const noexcept = 0;

    // Brings the buffers named with setRenderArray up to date: records one device-to-device copy of
    // getMaxParticles() elements per such array (not a host round trip). Call it after the step and
    // before the buffers are drawn. Nothing is recorded when areRenderArraysShared().
    virtual nvrhi::FRESULT updateRenderArrays(_In_opt_ nvrhi::ICommandList* commandList) noexcept = 0;

    // The bounds of the particles as a solver computed them last: two float3 elements, the lower and the
    // upper corner (NvFlexGetBounds). A device buffer, zero until a solver has stepped; read it with
    // IPhysDevice::copyBuffer into a Readback buffer. Does not AddRef.
    //
    // It is a buffer and not a box in host memory because the bounds are computed on the GPU: a host view
    // would be a readback per step that most callers do not want.
    [[nodiscard]] virtual IPhysBuffer* getBounds() const noexcept = 0;

    // Advances whenever getArray() returns another buffer object than before (setArray, resetArray). A
    // solver compares it to know when to rebuild what it derived from the buffer objects (binding sets).
    // It starts at 0.
    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;

    // Advances whenever the contents of an array or the active count change through this interface
    // (upload, setArray, resetArray, setActiveCount, markContentChanged). It starts at 0.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;

    // For a writer this interface cannot see (a solver step, an application kernel that writes
    // getArray(...)->getRHIBuffer()): advances the content revision and returns the new value.
    virtual uint64_t markContentChanged() noexcept = 0;
};
typedef nvrhi::AutoPtr<IParticleState> ParticleStateHandle;

// The diffuse particles of a domain (IPhysDomain::getDiffuseParticles): particles without mass that the
// fluid carries along (spray, foam, bubbles). A domain has them when DomainDesc::maxDiffuseParticles is not
// zero.
//
// The count lives on the GPU, because a solver creates and removes diffuse particles there: it is the one
// element of getCountBuffer(), and the host learns it by reading that buffer back.
NVRHI_IID(IDiffuseParticleState, "71531c38-c4a4-4b31-bc83-960be82688b0")
struct IDiffuseParticleState : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IDiffuseParticleState)

    // The capacity: DomainDesc::maxDiffuseParticles, fixed at creation.
    [[nodiscard]] virtual uint32_t getMaxParticles() const noexcept = 0;

    // The arrays, getMaxParticles() elements each; they do not AddRef, and they are never replaced.
    // Positions: float4, w is the lifetime (1: a new particle, 0: inactive). Velocities: float4, w unused.
    [[nodiscard]] virtual IPhysBuffer* getPositions() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getVelocities() const noexcept = 0;

    // One int32: the number of diffuse particles. Zero at creation.
    [[nodiscard]] virtual IPhysBuffer* getCountBuffer() const noexcept = 0;

    // Host to device: records copies of the first count elements of positions and velocities (16-byte
    // elements) and a write of the count, and advances the content revision (NvFlexSetDiffuseParticles).
    // A count of 0 needs no buffers. FE_INVALID_ARGS, a message and no copy for a count above
    // getMaxParticles(), a buffer that is null, too small, of another stride or a Readback buffer.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* positions,
                                  _In_opt_ IPhysBuffer* velocities, uint32_t count) noexcept = 0;

    // Device to host: records copies of the arrays into the buffers given (NvFlexGetDiffuseParticles);
    // a null buffer is skipped. positions and velocities receive as many elements as the smaller of the
    // buffer and the capacity holds, countBuffer (4-byte elements) the count.
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* positions,
                                    _In_opt_ IPhysBuffer* velocities,
                                    _In_opt_ IPhysBuffer* countBuffer) noexcept = 0;

    // As IParticleState: upload advances the revision, a solver step says so with markContentChanged.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;
    virtual uint64_t markContentChanged() noexcept = 0;
};
typedef nvrhi::AutoPtr<IDiffuseParticleState> DiffuseParticleStateHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_PARTICLES_H */
