#ifndef ETHEREAL_PHYS_DEVICE_H
#define ETHEREAL_PHYS_DEVICE_H

// IPhysDevice: the entry point of ethereal_phys, created over the application's nvrhi::IDevice.
//
// It replaces FleX's NvFlexLibrary. Simulation and rendering share one nvrhi device, so FleX's
// NvFlexInitDesc device/context pointers, GetDeviceAndContext / AcquireContext / RestoreContext and
// ComputeWaitForGraphics are gone: ordering across queues is nvrhi::IDevice::queueWaitForCommandList, with
// the instance flush() returns.
//
// ethereal_phys is a DLL, and donut/nvrhi/doc/com-interface-abi.md applies in full. The library exports
// exactly the two extern "C" functions declared at the end of this header; everything else is reached
// through interface vtables. The C++ free functions (createDevice, verifyHeaderVersion) are inline
// wrappers over those exports.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/common/export.h>
#include <ethereal/phys/domain.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys/ext/container.h>
#include <ethereal/phys/ext/force_field.h>
#include <ethereal/phys/ext/moving_frame.h>
#include <ethereal/phys/kernels.h>
#include <ethereal/phys/params.h>
#include <ethereal/phys/readback.h>
#include <ethereal/phys/solver.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

enum class PhysMessageSeverity : uint8_t {
    Info = 0,
    Warning = 1,
    Error = 2,  // a call failed; it returned an FE_* code
    Fatal = 3,  // the device is no longer usable (e.g. the GPU device was removed)
};

// Implemented by the application, as a reference-counted object; the device keeps a reference. It replaces
// FleX's NvFlexErrorCallback function pointer. message() may be called from any thread that calls into
// the engine, and must not call back into the engine.
NVRHI_IID(IPhysMessageCallback, "b43cc5c7-f89c-4d6d-acb4-e436af0b4e41")
struct IPhysMessageCallback : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysMessageCallback)

    // messageText is valid only during the call.
    virtual void message(PhysMessageSeverity severity, const char* messageText) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysMessageCallback> PhysMessageCallbackHandle;

struct PhysDeviceDesc {
    // Required. The device keeps a reference.
    nvrhi::IDevice* device = nullptr;

    // Optional. The device keeps a reference.
    IPhysMessageCallback* messageCallback = nullptr;

    // Optional, read only: the directory that holds the compiled kernels of the engine, with one
    // sub-directory per shader format (dxil, spirv, dxbc): what the ethereal_phys_shaders target writes to
    // <bin>/shaders/ethereal_phys. UTF-8; an absolute path, or one relative to the working directory of
    // the process. The device copies the string. Without it the device works, but no kernel can be
    // loaded (kernels.h).
    const char* shaderPath = nullptr;

    // Submit the engine's own command list to the compute queue when the nvrhi device has one
    // (nvrhi::Feature::ComputeQueue); otherwise, and by default, to the graphics queue.
    bool preferComputeQueue = false;
    uint8_t _pad[3] = {0, 0, 0};  // explicit

    // nvrhi::c_HeaderVersion as the caller's compiler saw it; leave the default. The library was compiled
    // against one version of the nvrhi headers and calls the caller's nvrhi objects through their vtables,
    // so etherealPhysCreateDevice refuses a caller whose nvrhi headers differ.
    uint32_t nvrhiHeaderVersion = nvrhi::c_HeaderVersion;
};

static_assert(sizeof(PhysDeviceDesc) == 32, "PhysDeviceDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(PhysDeviceDesc, messageCallback) == 8);
static_assert(offsetof(PhysDeviceDesc, shaderPath) == 16);
static_assert(offsetof(PhysDeviceDesc, preferComputeQueue) == 24);
static_assert(offsetof(PhysDeviceDesc, _pad) == 25);
static_assert(offsetof(PhysDeviceDesc, nvrhiHeaderVersion) == 28);  // after the padding

// What the device found out about the nvrhi device at creation.
struct PhysDeviceInfo {
    uint32_t headerVersion = 0;       // c_HeaderVersion the library was built with
    uint32_t nvrhiHeaderVersion = 0;  // nvrhi::c_HeaderVersion the library was built with
    nvrhi::GraphicsAPI graphicsAPI = nvrhi::GraphicsAPI::D3D12;
    // The queue the engine's own command list is submitted to (flush()).
    nvrhi::CommandQueue executionQueue = nvrhi::CommandQueue::Graphics;
    // nvrhi::Feature::WaveLaneCountMinMax: the kernels may use the wave intrinsics variants.
    bool supportsWaveOps = false;
    // BufferUse::RenderShared can be created: a buffer a kernel writes and a renderer binds as a vertex
    // buffer. False on D3D11, where a structured buffer cannot be a vertex buffer; a renderer there gets
    // a device-to-device copy instead (not a host round trip).
    bool supportsRenderSharedBuffers = false;
    uint32_t minWaveLaneCount = 0;  // 0 when supportsWaveOps is false
    uint32_t maxWaveLaneCount = 0;
};

static_assert(sizeof(PhysDeviceInfo) == 20, "PhysDeviceInfo layout changed: bump c_HeaderVersion");
static_assert(sizeof(nvrhi::GraphicsAPI) == 1 && sizeof(nvrhi::CommandQueue) == 1);
static_assert(offsetof(PhysDeviceInfo, graphicsAPI) == 8);
static_assert(offsetof(PhysDeviceInfo, executionQueue) == 9);
static_assert(offsetof(PhysDeviceInfo, supportsWaveOps) == 10);
static_assert(offsetof(PhysDeviceInfo, supportsRenderSharedBuffers) == 11);
static_assert(offsetof(PhysDeviceInfo, minWaveLaneCount) == 12);
static_assert(offsetof(PhysDeviceInfo, maxWaveLaneCount) == 16);

// The device is not thread-safe: calls on one IPhysDevice, and on the command lists handed to it, are
// made from one thread at a time.
//
// The device owns one command list (on PhysDeviceInfo::executionQueue). The methods that record GPU work
// take an nvrhi::ICommandList*: the application's open command list, or nullptr for the device's own,
// which the device opens when needed and flush() closes and executes. On D3D11 every command list is the
// immediate context and only one may be open at a time, so there flush() before opening another one.
// Releasing the last reference to the device flushes: recorded work is never dropped.
NVRHI_IID(IPhysDevice, "edcedb6a-5519-4dcb-a2c8-f320393f6c5c")
struct IPhysDevice : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysDevice)

    // Non-owning views of what the device keeps. In the desc, shaderPath points at the device's copy.
    [[nodiscard]] virtual const PhysDeviceDesc& getDesc() const noexcept = 0;
    [[nodiscard]] virtual const PhysDeviceInfo& getInfo() const noexcept = 0;

    // The nvrhi device. Does not AddRef.
    [[nodiscard]] virtual nvrhi::IDevice* getRHIDevice() const noexcept = 0;

    // Creates a buffer of desc.elementCount elements of desc.elementStride bytes (BufferUse, buffer.h).
    // FE_INVALID_ARGS for a zero count or stride, a Raw or IndirectArgs stride that is not a multiple of 4,
    // or a Typed stride that is not 4;
    // FE_UNSUPPORTED for BufferUse::RenderShared when PhysDeviceInfo::supportsRenderSharedBuffers is false.
    virtual nvrhi::FRESULT createBuffer(const PhysBufferDesc& desc, IPhysBuffer** ppBuffer) noexcept = 0;

    // Wraps a buffer the application created, e.g. a vertex buffer with canHaveUAVs that a kernel writes
    // and the renderer binds: no copy and no registration step. The IPhysBuffer keeps a reference to it.
    // elementCount * elementStride must fit into the buffer, and elementStride must equal a non-zero
    // structStride (FE_INVALID_ARGS otherwise). The buffer needs a state the command lists can track:
    // initialState + keepInitialState, or a permanent state.
    //
    // When the engine runs on the compute queue (PhysDeviceInfo::executionQueue), that resting state must
    // be one a compute command list can leave and restore: UnorderedAccess, CopySource, CopyDest, a
    // non-pixel shader resource. VertexBuffer and IndexBuffer are not (Vulkan has no vertex input stage on
    // a compute queue); a buffer that rests in one of them is adopted with a Warning, and may then only be
    // used on a graphics command list the application passes in. Give a shared vertex buffer the initial
    // state UnorderedAccess: the renderer's graphics command list moves it to VertexBuffer and back.
    virtual nvrhi::FRESULT adoptBuffer(nvrhi::IBuffer* buffer, uint32_t elementCount, uint32_t elementStride,
                                       IPhysBuffer** ppBuffer) noexcept = 0;

    // Host to device: records a write of elementCount elements from data into dst, starting at element
    // dstOffset, and advances dst's content revision. data is copied before the call returns.
    // dst must not be CPU-visible (FE_UNSUPPORTED: an Upload buffer is written through map()). On D3D11 a
    // Constant buffer is written as a whole or not at all (FE_UNSUPPORTED for a part of it).
    // commandList: see above.
    virtual nvrhi::FRESULT writeBuffer(_In_opt_ nvrhi::ICommandList* commandList, IPhysBuffer* dst,
                                       const void* data, uint32_t dstOffset, uint32_t elementCount) noexcept = 0;

    // Device to device, Upload to device, or device to Readback: records a copy of region.elementCount
    // elements and advances dst's content revision. Both buffers have the same element stride and are
    // different buffers; dst is not an Upload buffer and src is not a Readback buffer (FE_INVALID_ARGS).
    // commandList: see above.
    virtual nvrhi::FRESULT copyBuffer(_In_opt_ nvrhi::ICommandList* commandList, IPhysBuffer* dst,
                                      IPhysBuffer* src, const CopyRegion& region) noexcept = 0;

    // Closes and executes the device's own command list. Returns the instance nvrhi assigned to it, for
    // nvrhi::IDevice::queueWaitForCommandList(waitQueue, getInfo().executionQueue, instance), or 0 when
    // nothing was recorded. On D3D11 it is always 0: there is one queue and nothing to wait for.
    virtual uint64_t flush() noexcept = 0;

    // flush(), then blocks until the GPU has finished all work submitted on the nvrhi device.
    // FE_GENERIC_ERROR when the wait detects a problem (e.g. device removal).
    virtual nvrhi::FRESULT waitForIdle() noexcept = 0;

    // The kernel library of the device (kernels.h). Never null. Does not AddRef: the device keeps it
    // alive. A caller that keeps its own reference may hold it past the device; the library then still
    // answers queries and dispatches on command lists the caller passes in.
    [[nodiscard]] virtual IPhysKernelLibrary* getKernels() const noexcept = 0;

    // Creates a domain (domain.h): a particle state of desc.maxParticles particles, diffuse particles when
    // desc.maxDiffuseParticles is not 0, the four constraint sets and the shape set. The domain keeps the
    // device alive. The particle arrays are cleared on the device's own command list, so the next flush()
    // has something to submit.
    // FE_INVALID_ARGS and a message for maxParticles 0; FE_GENERIC_ERROR when a buffer cannot be created.
    virtual nvrhi::FRESULT createDomain(const DomainDesc& desc, IPhysDomain** ppDomain) noexcept = 0;

    // The geometry library of the device (collision.h): the triangle mesh, distance field and convex
    // assets that collision shapes refer to. Never null. Does not AddRef: the device keeps it alive. A
    // caller, or an asset, that holds a reference may keep it past the device; it then records no more GPU
    // work.
    [[nodiscard]] virtual IGeometryLibrary* getGeometry() const noexcept = 0;

    // Creates a solver for a domain of this device (solver.h). The solver keeps the domain alive. Its
    // scratch buffers are sized by the particle capacities of the domain and by desc, and are cleared on
    // the device's own command list, so the next flush() has something to submit.
    //
    // desc.mode selects the implementation: SolverMode::PBD answers IID_IPhysSolver, SolverMode::XPBD
    // IID_IPhysSolverXPBD as well.
    //
    // FE_INVALID_ARGS and a message for a null domain, a domain of another device, a mode or a feature
    // mode that is not one, and for maxNeighborsPerParticle or maxContactsPerParticle 0;
    // FE_GENERIC_ERROR when a buffer cannot be created.
    virtual nvrhi::FRESULT createSolver(IPhysDomain* domain, const SolverDesc& desc,
                                        IPhysSolver** ppSolver) noexcept = 0;

    // Creates the readback object of a solver of this device (readback.h): GPU copies of what the solver
    // computes into buffers of the application. The object keeps the solver alive. Several may exist for one
    // solver. FE_INVALID_ARGS and a message for a null solver or a solver of another device.
    virtual nvrhi::FRESULT createReadback(IPhysSolver* solver, IPhysReadback** ppReadback) noexcept = 0;

    // Creates a timer object of a solver of this device (readback.h): the GPU times of the stages of its
    // steps, while IPhysSolver::areTimersEnabled(). The object keeps the solver alive; every timer object of
    // a solver reports the same results (the queries are the solver's). FE_INVALID_ARGS and a message for a
    // null solver, a solver of another device, or a solver this library did not create.
    virtual nvrhi::FRESULT createTimers(IPhysSolver* solver, IPhysTimers** ppTimers) noexcept = 0;

    // Creates an asset builder (ext/asset_builder.h): FleX's extension asset functions, on the host. The
    // builder keeps the device alive and reports through its message callback. Each builder has its own
    // random generator (FleX's extension module had one), so builders do not affect each other.
    virtual nvrhi::FRESULT createAssetBuilder(IPhysAssetBuilder** ppBuilder) noexcept = 0;

    // Creates a container over a solver of this device and its domain (ext/container.h; FleX's
    // NvFlexExtCreateContainer): an allocator of particles 0 .. maxParticles - 1 of the domain, asset instances
    // and soft joints. No GPU work is recorded. The container keeps the solver alive. FE_INVALID_ARGS and a
    // message for a null solver, a solver of another device, maxParticles 0 or above the domain's capacity.
    virtual nvrhi::FRESULT createContainer(IPhysSolver* solver, uint32_t maxParticles,
                                           IPhysContainer** ppContainer) noexcept = 0;

    // Creates the force fields of a solver of this device (ext/force_field.h; FleX's
    // NvFlexExtCreateForceFieldCallback) and registers them at SolverStage::UpdateEnd of the solver, chained
    // to the callback registered there before. The set keeps the solver alive. FE_INVALID_ARGS and a message
    // for a null solver or a solver of another device.
    virtual nvrhi::FRESULT createForceFieldSet(IPhysSolver* solver, IForceFieldSet** ppSet) noexcept = 0;

    // Creates a moving frame at a pose (ext/moving_frame.h; FleX's NvFlexExtMovingFrameInit): translation 3
    // floats, rotation a quaternion x y z w. Host math only. FE_INVALID_ARGS for a null pointer.
    virtual nvrhi::FRESULT createMovingFrame(const float* worldTranslation, const float* worldRotation,
                                             IMovingFrame** ppFrame) noexcept = 0;

    // Later steps append here, each with a c_HeaderVersion bump (the vtable is not frozen before the
    // first release).
};
typedef nvrhi::AutoPtr<IPhysDevice> PhysDeviceHandle;

}  // namespace ephys

// ---- The exports of ethereal_phys.dll: these two functions and nothing else. ---------------------------

// The version of the public headers the library was built with (ephys::c_HeaderVersion at that time).
ETHEREAL_PHYS_C_API uint32_t etherealPhysGetHeaderVersion() noexcept;

// Creates a device over desc->device. headerVersion is ephys::c_HeaderVersion as the caller's compiler saw
// it. On success returns FS_OK and stores the device with one reference added; on failure returns an FE_*
// code and stores nullptr:
// - FE_UNSUPPORTED when headerVersion differs from the library's (desc is not read in that case: its
//   layout is not known to be the caller's), or when desc->nvrhiHeaderVersion differs from the nvrhi
//   headers the library was built with;
// - FE_INVALID_ARGS when desc, desc->device or ppDevice is null.
ETHEREAL_PHYS_C_API nvrhi::FRESULT etherealPhysCreateDevice(const ephys::PhysDeviceDesc* desc,
                                                            uint32_t headerVersion,
                                                            ephys::IPhysDevice** ppDevice) noexcept;

namespace ephys {

// True when the library was built with the headers this module was compiled with. createDevice checks the
// same thing; this is for a clear message at startup.
inline bool verifyHeaderVersion(uint32_t version = c_HeaderVersion) noexcept {
    return etherealPhysGetHeaderVersion() == version;
}

// Returns the device, or null on failure (the message callback of desc, when given, says why).
inline PhysDeviceHandle createDevice(const PhysDeviceDesc& desc) noexcept {
    IPhysDevice* device = nullptr;
    etherealPhysCreateDevice(&desc, c_HeaderVersion, &device);
    return nvrhi::TakeOver(device);
}

}  // namespace ephys

#endif /* ETHEREAL_PHYS_DEVICE_H */
