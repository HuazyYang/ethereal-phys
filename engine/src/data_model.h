#ifndef ETHEREAL_PHYS_SRC_DATA_MODEL_H
#define ETHEREAL_PHYS_SRC_DATA_MODEL_H

// Module-private: how the device creates the objects of the data model, and what their implementations
// share. The implementation classes stay in their .cpp files; everything else holds the interfaces.
//
// The data objects are written against the public interface of the device (createBuffer, copyBuffer,
// writeBuffer): they record their GPU work the way an application would, and inherit the device's
// "null command list = the device's own" convention from it.

#include <ethereal/phys/device.h>

#include "kernel_library.h"
#include "phys_messages.h"

#include <cstdarg>
#include <cstdint>

namespace ephys::detail {

// Each factory returns FS_OK and one reference in its out-parameter, or an FE_* code and nullptr; it
// reports through the message callback of the device.

nvrhi::FRESULT createParticleState(IPhysDevice* device, uint32_t maxParticles,
                                   IParticleState** ppState) noexcept;
nvrhi::FRESULT createDiffuseParticleState(IPhysDevice* device, uint32_t maxParticles,
                                          IDiffuseParticleState** ppState) noexcept;
nvrhi::FRESULT createSpringSet(IPhysDevice* device, uint32_t reserve, ISpringSet** ppSet) noexcept;
nvrhi::FRESULT createShapeMatchSet(IPhysDevice* device, uint32_t reserveRigids, uint32_t reserveIndices,
                                   IShapeMatchSet** ppSet) noexcept;
nvrhi::FRESULT createTriangleSet(IPhysDevice* device, uint32_t reserve, ITriangleSet** ppSet) noexcept;
nvrhi::FRESULT createInflatableSet(IPhysDevice* device, uint32_t reserve, IInflatableSet** ppSet) noexcept;
nvrhi::FRESULT createCollisionShapeSet(IPhysDevice* device, uint32_t reserve,
                                       ICollisionShapeSet** ppSet) noexcept;
nvrhi::FRESULT createDomain(IPhysDevice* device, const DomainDesc& desc, IPhysDomain** ppDomain) noexcept;

// The geometry library of a device. The device owns it and the library does not keep the device alive:
// it uses `device` and `host` until detachGeometryLibrary, which the device calls when it goes away (the
// library may be referenced longer, by an asset or by the application).
nvrhi::FRESULT createGeometryLibrary(IPhysDevice* device, KernelLibraryHost* host,
                                     IGeometryLibrary** ppLibrary) noexcept;
void detachGeometryLibrary(IGeometryLibrary* library) noexcept;

// FleX's CalculateSlack: the capacity a growing array gets for n elements.
inline uint32_t slack(uint32_t n) noexcept { return n + (n >> 1); }

// The largest count a growing array accepts: slack(n) elements of 16 bytes, three per item, still fit
// the 32-bit element arithmetic.
constexpr uint32_t c_MaxArrayCount = 0x0fffffff;

// What every data object does with its device: report, check the buffers a caller hands in, create and
// copy arrays. A helper base of the implementation classes, not an interface.
class DataObject {
 protected:
    explicit DataObject(IPhysDevice* device)
        : m_Device(device) {}
    ~DataObject() = default;

    [[nodiscard]] IPhysMessageCallback* callback() const noexcept {
        return m_Device->getDesc().messageCallback;
    }

    // Reports an error and returns its code.
    nvrhi::FRESULT fail(nvrhi::FRESULT code, const char* format, ...) const noexcept {
        char text[512];
        va_list args;
        va_start(args, format);
        vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        report(callback(), PhysMessageSeverity::Error, "%s", text);
        return code;
    }

    // What is wrong with a buffer the caller gave as the source (toDevice) or the destination of a copy
    // of `count` elements of `stride` bytes starting at `offset`; nullptr when nothing is.
    [[nodiscard]] static const char* problemOf(IPhysBuffer* buffer, uint32_t stride, uint64_t offset,
                                               uint64_t count, bool toDevice) noexcept {
        if (!buffer)
            return "the buffer is null";
        const PhysBufferDesc& desc = buffer->getDesc();
        if (desc.elementStride != stride)
            return "the element stride of the buffer is not that of the array";
        if (offset + count > desc.elementCount)
            return "the range exceeds the buffer";
        if (toDevice && desc.use == BufferUse::Readback)
            return "a Readback buffer cannot be a copy source";
        if (!toDevice && desc.use == BufferUse::Upload)
            return "an Upload buffer cannot be a copy destination";
        return nullptr;
    }

    // An array of the engine: a structured buffer with UAVs.
    nvrhi::FRESULT createArray(const char* name, uint32_t stride, uint32_t count,
                               PhysBufferHandle& out) const noexcept {
        PhysBufferDesc desc;
        desc.elementCount = count;
        desc.elementStride = stride;
        desc.use = BufferUse::Structured;
        desc.debugName = name;
        PhysBufferHandle buffer;
        const nvrhi::FRESULT result = m_Device->createBuffer(desc, &buffer);
        if (NVRHI_FAILED(result))
            return result;
        out = buffer;
        return nvrhi::FS_OK;
    }

    // Records a copy of count elements. The device checks again and reports what it refuses.
    nvrhi::FRESULT copy(nvrhi::ICommandList* commandList, IPhysBuffer* dst, uint32_t dstOffset,
                        IPhysBuffer* src, uint32_t srcOffset, uint32_t count) const noexcept {
        if (count == 0)
            return nvrhi::FS_OK;
        CopyRegion region;
        region.srcOffset = srcOffset;
        region.dstOffset = dstOffset;
        region.elementCount = count;
        return m_Device->copyBuffer(commandList, dst, src, region);
    }

    PhysDeviceHandle m_Device;
};

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_DATA_MODEL_H */
