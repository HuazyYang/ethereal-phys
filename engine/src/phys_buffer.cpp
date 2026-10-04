#include "phys_buffer.h"

#include <nvrhi/core/foundation.h>

#include <atomic>
#include <new>

namespace ephys {
namespace {

NVRHI_CLASS_CLSID(PhysBuffer, "36c32679-6ee2-44db-83b0-0e2d512c96d6")
class PhysBuffer final : public nvrhi::ObjectImpl<IPhysBuffer> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysBuffer)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysBuffer)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysBuffer)
    NVRHI_IMPLEMENTS_CLASS(PhysBuffer)
    NVRHI_END_INTERFACE_TABLE()

    PhysBuffer(nvrhi::IDevice* device, nvrhi::IBuffer* buffer, const PhysBufferDesc& desc)
        : m_Device(device), m_Buffer(buffer), m_Desc(desc) {
        // The name the nvrhi buffer holds outlives every reader of m_Desc: the buffer is kept alive here.
        const nvrhi::string& name = buffer->getDesc().debugName;
        m_Desc.debugName = name.empty() ? nullptr : name.c_str();
    }

    ~PhysBuffer() {
        if (m_Mapped.load()) m_Device->unmapBuffer(m_Buffer);
    }

    const PhysBufferDesc& getDesc() const noexcept override { return m_Desc; }
    nvrhi::IBuffer* getRHIBuffer() const noexcept override { return m_Buffer; }
    uint32_t getElementCount() const noexcept override { return m_Desc.elementCount; }
    uint32_t getElementStride() const noexcept override { return m_Desc.elementStride; }

    nvrhi::FRESULT map(void** ppData) noexcept override {
        if (!ppData) return nvrhi::FE_INVALID_ARGS;
        *ppData = nullptr;
        if (m_Desc.use != BufferUse::Upload && m_Desc.use != BufferUse::Readback) return nvrhi::FE_UNSUPPORTED;
        if (m_Mapped.exchange(true)) return nvrhi::FE_INVALID_ARGS;

        void* data = m_Device->mapBuffer(
            m_Buffer, m_Desc.use == BufferUse::Upload ? nvrhi::CpuAccessMode::Write : nvrhi::CpuAccessMode::Read);
        if (!data) {
            m_Mapped.store(false);
            return nvrhi::FE_GENERIC_ERROR;
        }
        *ppData = data;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT unmap() noexcept override {
        if (!m_Mapped.exchange(false)) return nvrhi::FE_INVALID_ARGS;
        m_Device->unmapBuffer(m_Buffer);
        if (m_Desc.use == BufferUse::Upload) m_Revision.fetch_add(1);
        return nvrhi::FS_OK;
    }

    uint64_t getContentRevision() const noexcept override { return m_Revision.load(); }
    uint64_t markContentChanged() noexcept override { return m_Revision.fetch_add(1) + 1; }

 private:
    nvrhi::DeviceHandle m_Device;
    nvrhi::BufferHandle m_Buffer;
    PhysBufferDesc m_Desc;
    std::atomic<uint64_t> m_Revision{0};
    std::atomic<bool> m_Mapped{false};
};

// The BufferUse recipes (the table in buffer.h). Every recipe tracks its state automatically.
void makeBufferDesc(const PhysBufferDesc& desc, nvrhi::BufferDesc& out) {
    out.byteSize = uint64_t(desc.elementCount) * uint64_t(desc.elementStride);
    if (desc.debugName) out.debugName = desc.debugName;

    switch (desc.use) {
        case BufferUse::Structured:
            out.structStride = desc.elementStride;
            out.canHaveUAVs = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
            break;
        case BufferUse::Raw:
            out.canHaveRawViews = true;
            out.canHaveUAVs = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
            break;
        case BufferUse::Upload:
            out.cpuAccess = nvrhi::CpuAccessMode::Write;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::CopySource);
            break;
        case BufferUse::Readback:
            out.cpuAccess = nvrhi::CpuAccessMode::Read;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::CopyDest);
            break;
        case BufferUse::Constant:
            out.isConstantBuffer = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::ConstantBuffer);
            break;
        case BufferUse::RenderShared:
            out.structStride = desc.elementStride;
            out.canHaveUAVs = true;
            out.isVertexBuffer = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
            break;
        case BufferUse::IndirectArgs:
            // Raw, not structured: D3D11 refuses a structured buffer as indirect arguments.
            out.canHaveRawViews = true;
            out.canHaveUAVs = true;
            out.isDrawIndirectArgs = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::IndirectArgument);
            break;
        case BufferUse::Typed:
            // R32_UINT: the kernels declare Buffer<uint> / RWBuffer<uint>; the radix sort also reads the
            // same buffer four elements at a time through an RGBA32_UINT view.
            out.format = nvrhi::Format::R32_UINT;
            out.canHaveTypedViews = true;
            out.canHaveUAVs = true;
            out.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
            break;
    }
}

// The use an adopted buffer is treated as. Only Upload and Readback change behaviour (map, the copy
// direction checks); the rest describes the buffer to whoever reads the desc.
BufferUse useOf(const nvrhi::BufferDesc& desc) {
    if (desc.cpuAccess == nvrhi::CpuAccessMode::Write) return BufferUse::Upload;
    if (desc.cpuAccess == nvrhi::CpuAccessMode::Read) return BufferUse::Readback;
    if (desc.isConstantBuffer) return BufferUse::Constant;
    if (desc.isDrawIndirectArgs) return BufferUse::IndirectArgs;
    if (desc.isVertexBuffer || desc.isIndexBuffer) return BufferUse::RenderShared;
    if (desc.structStride != 0) return BufferUse::Structured;
    if (desc.canHaveTypedViews && !desc.canHaveRawViews) return BufferUse::Typed;
    return BufferUse::Raw;
}

nvrhi::FRESULT wrap(nvrhi::IDevice* device, nvrhi::IBuffer* buffer, const PhysBufferDesc& desc,
                    IPhysBuffer** ppBuffer, const char** ppReason) noexcept {
    try {
        *ppBuffer = MAKE_RC_OBJ(PhysBuffer, device, buffer, desc);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        *ppReason = "out of memory";
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        *ppReason = "unexpected exception";
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace

namespace detail {

nvrhi::FRESULT createPhysBuffer(nvrhi::IDevice* device, const PhysBufferDesc& desc, IPhysBuffer** ppBuffer,
                                const char** ppReason) noexcept {
    *ppBuffer = nullptr;
    *ppReason = "";

    if (desc.elementCount == 0 || desc.elementStride == 0) {
        *ppReason = "elementCount and elementStride must not be zero";
        return nvrhi::FE_INVALID_ARGS;
    }
    if (desc.use > BufferUse::Typed) {
        *ppReason = "unknown BufferUse";
        return nvrhi::FE_INVALID_ARGS;
    }
    if ((desc.use == BufferUse::Raw || desc.use == BufferUse::IndirectArgs) && (desc.elementStride % 4) != 0) {
        *ppReason = "the element stride of a Raw or IndirectArgs buffer must be a multiple of 4";
        return nvrhi::FE_INVALID_ARGS;
    }

    if (desc.use == BufferUse::Typed && desc.elementStride != 4) {
        *ppReason = "the element stride of a Typed buffer must be 4";
        return nvrhi::FE_INVALID_ARGS;
    }

    try {
        nvrhi::BufferDesc rhiDesc;
        makeBufferDesc(desc, rhiDesc);

        nvrhi::BufferHandle buffer;
        const nvrhi::FRESULT result = device->createBuffer(rhiDesc, &buffer);
        if (NVRHI_FAILED(result) || !buffer) {
            *ppReason = "nvrhi::IDevice::createBuffer failed";
            return NVRHI_FAILED(result) ? result : nvrhi::FE_GENERIC_ERROR;
        }
        return wrap(device, buffer, desc, ppBuffer, ppReason);
    } catch (const std::bad_alloc&) {
        *ppReason = "out of memory";
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        *ppReason = "unexpected exception";
        return nvrhi::FE_GENERIC_ERROR;
    }
}

nvrhi::FRESULT adoptPhysBuffer(nvrhi::IDevice* device, nvrhi::IBuffer* buffer, uint32_t elementCount,
                               uint32_t elementStride, IPhysBuffer** ppBuffer, const char** ppReason) noexcept {
    *ppBuffer = nullptr;
    *ppReason = "";

    if (!buffer) {
        *ppReason = "the buffer is null";
        return nvrhi::FE_INVALID_ARGS;
    }
    if (elementCount == 0 || elementStride == 0) {
        *ppReason = "elementCount and elementStride must not be zero";
        return nvrhi::FE_INVALID_ARGS;
    }

    const nvrhi::BufferDesc& rhiDesc = buffer->getDesc();
    if (rhiDesc.isVolatile || rhiDesc.isVirtual) {
        *ppReason = "a volatile or virtual buffer cannot be adopted";
        return nvrhi::FE_INVALID_ARGS;
    }
    if (uint64_t(elementCount) * uint64_t(elementStride) > rhiDesc.byteSize) {
        *ppReason = "elementCount * elementStride exceeds the size of the buffer";
        return nvrhi::FE_INVALID_ARGS;
    }
    if (rhiDesc.structStride != 0 && rhiDesc.structStride != elementStride) {
        *ppReason = "elementStride differs from the structStride of the buffer";
        return nvrhi::FE_INVALID_ARGS;
    }

    PhysBufferDesc desc;
    desc.elementCount = elementCount;
    desc.elementStride = elementStride;
    desc.use = useOf(rhiDesc);
    return wrap(device, buffer, desc, ppBuffer, ppReason);
}

}  // namespace detail
}  // namespace ephys
