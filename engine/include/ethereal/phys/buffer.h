#ifndef ETHEREAL_PHYS_BUFFER_H
#define ETHEREAL_PHYS_BUFFER_H

// IPhysBuffer: an nvrhi buffer seen as an array of fixed-size elements, with a content revision.
//
// It replaces FleX's NvFlexBuffer. What survives is the element count and the element stride beside the
// nvrhi::IBuffer (the kernels index their buffers by element), the copy region (CopyRegion, params.h), and
// map/unmap on CPU-visible buffers. What is gone: NvFlexBufferType (every buffer is an nvrhi::BufferDesc,
// chosen through BufferUse), mapping a device buffer (it returned a handle masquerading as memory), and
// the register/unregister pairs: an application buffer is adopted (IPhysDevice::adoptBuffer), not
// registered.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md: no virtual returns an aggregate by value,
// every virtual is noexcept, and no two virtuals share a name.

#include <nvrhi/core/autoptr.h>
#include <nvrhi/core/types.h>

#include <cstddef>
#include <cstdint>

namespace nvrhi {
struct IBuffer;  // <nvrhi/nvrhi.h>
}

namespace ephys {

// What a buffer is for. IPhysDevice::createBuffer turns it into one nvrhi::BufferDesc recipe; every recipe
// has a tracked state (initialState + keepInitialState), as this nvrhi requires.
//
//   use           nvrhi::BufferDesc                                               initial state
//   Structured    structStride = elementStride, canHaveUAVs                       UnorderedAccess
//   Raw           canHaveRawViews, canHaveUAVs (elementStride a multiple of 4)    UnorderedAccess
//   Upload        cpuAccess = Write                                               CopySource
//   Readback      cpuAccess = Read                                                CopyDest
//   Constant      isConstantBuffer (not volatile; written with writeBuffer)       ConstantBuffer
//   RenderShared  structStride = elementStride, canHaveUAVs, isVertexBuffer       UnorderedAccess
//   IndirectArgs  canHaveRawViews, canHaveUAVs, isDrawIndirectArgs (as Raw)       IndirectArgument
//
// Upload and Readback are the only CPU-visible uses, and the only ones map() accepts. An Upload buffer is
// a copy source only, and a Readback buffer a copy destination only.
enum class BufferUse : uint8_t {
    Structured = 0,    // StructuredBuffer / RWStructuredBuffer of the kernels
    Raw = 1,           // ByteAddressBuffer / RWByteAddressBuffer of the kernels
    Upload = 2,        // host-written staging memory, the source of a copy to a device buffer
    Readback = 3,      // host-read staging memory, the destination of a copy from a device buffer
    Constant = 4,      // a constant buffer of the kernels
    RenderShared = 5,  // written by a kernel and bound by a renderer as a vertex buffer or a shader resource
    IndirectArgs = 6,  // dispatch or draw arguments written by a kernel (through a raw view)
};

struct PhysBufferDesc {
    uint32_t elementCount = 0;   // number of elements; the buffer holds elementCount * elementStride bytes
    uint32_t elementStride = 0;  // size of one element in bytes
    BufferUse use = BufferUse::Structured;
    uint8_t _pad[7] = {0, 0, 0, 0, 0, 0, 0};  // explicit: debugName is 8-byte aligned
    // Optional, read only. createBuffer copies the string; the pointer in the desc that
    // IPhysBuffer::getDesc returns points at the name the nvrhi buffer holds and lives as long as the
    // buffer. It is nullptr when the nvrhi buffer has no name (the nvrhi validation layer names every
    // buffer that was given none).
    const char* debugName = nullptr;
};

static_assert(sizeof(void*) == 8, "ethereal_phys requires a 64-bit target");
static_assert(sizeof(PhysBufferDesc) == 24, "PhysBufferDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(PhysBufferDesc, elementStride) == 4);
static_assert(offsetof(PhysBufferDesc, use) == 8);
static_assert(offsetof(PhysBufferDesc, _pad) == 9);
static_assert(offsetof(PhysBufferDesc, debugName) == 16);  // after the padding

NVRHI_IID(IPhysBuffer, "0adad42a-3f83-4680-9e51-72856feb4253")
struct IPhysBuffer : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysBuffer)

    // The description of the buffer; the object keeps it (a non-owning view). For an adopted buffer the
    // use is derived from the nvrhi::BufferDesc and debugName is the nvrhi buffer's.
    [[nodiscard]] virtual const PhysBufferDesc& getDesc() const noexcept = 0;

    // The nvrhi buffer. Does not AddRef: the IPhysBuffer keeps it alive.
    [[nodiscard]] virtual nvrhi::IBuffer* getRHIBuffer() const noexcept = 0;

    [[nodiscard]] virtual uint32_t getElementCount() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getElementStride() const noexcept = 0;

    // Maps the whole buffer into host memory and stores the address in *ppData: write-only memory for an
    // Upload buffer, read-only memory for a Readback buffer. Every other use returns FE_UNSUPPORTED and
    // stores nullptr: a device buffer has no host address (FleX returned a handle there). A second map
    // before unmap returns FE_INVALID_ARGS.
    //
    // A Readback buffer must not be mapped while a command list that writes it is still executing: wait
    // for it first (IPhysDevice::waitForIdle).
    virtual nvrhi::FRESULT map(void** ppData) noexcept = 0;

    // Ends map(). On an Upload buffer it also advances the content revision. FE_INVALID_ARGS when the
    // buffer is not mapped.
    virtual nvrhi::FRESULT unmap() noexcept = 0;

    // The content revision: a counter that advances every time the content of the buffer is replaced or
    // modified through the engine (unmap of an Upload buffer, IPhysDevice::writeBuffer and copyBuffer into
    // it, a solver step that writes it). A reader that caches something derived from the buffer compares
    // revisions instead of contents. It starts at 0.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;

    // For a writer the engine cannot see (the application's own kernel or copy into getRHIBuffer()):
    // advances the content revision and returns the new value. Thread-safe.
    virtual uint64_t markContentChanged() noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysBuffer> PhysBufferHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_BUFFER_H */
