#include <ethereal/phys/device.h>

#include "data_model.h"
#include "kernel_library.h"
#include "phys_buffer.h"
#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <new>

namespace ephys {
namespace {

using detail::report;

NVRHI_CLASS_CLSID(PhysDevice, "186f1d94-f6c1-430e-9e0a-3983ca29728b")
class PhysDevice final : public nvrhi::ObjectImpl<IPhysDevice>, private detail::KernelLibraryHost {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysDevice)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysDevice)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysDevice)
    NVRHI_IMPLEMENTS_CLASS(PhysDevice)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysDevice(const PhysDeviceDesc& desc)
        : m_Device(desc.device), m_Callback(desc.messageCallback), m_Desc(desc) {
        if (desc.shaderPath) m_ShaderPath = desc.shaderPath;
        m_Desc.shaderPath = desc.shaderPath ? m_ShaderPath.c_str() : nullptr;

        m_Info.headerVersion = c_HeaderVersion;
        m_Info.nvrhiHeaderVersion = nvrhi::c_HeaderVersion;
        m_Info.graphicsAPI = m_Device->getGraphicsAPI();
        m_Info.executionQueue =
            desc.preferComputeQueue && m_Device->queryFeatureSupport(nvrhi::Feature::ComputeQueue)
                ? nvrhi::CommandQueue::Compute
                : nvrhi::CommandQueue::Graphics;

        nvrhi::WaveLaneCountMinMaxFeatureInfo wave = {};
        if (m_Device->queryFeatureSupport(nvrhi::Feature::WaveLaneCountMinMax, &wave, sizeof(wave))) {
            m_Info.supportsWaveOps = true;
            m_Info.minWaveLaneCount = wave.minWaveLaneCount;
            m_Info.maxWaveLaneCount = wave.maxWaveLaneCount;
        }

        // D3D11 cannot create a structured buffer that is also a vertex buffer.
        m_Info.supportsRenderSharedBuffers = m_Info.graphicsAPI != nvrhi::GraphicsAPI::D3D11;

        // The kernel library loads nothing until a kernel is used. Without it there is no device.
        if (NVRHI_FAILED(detail::createKernelLibrary(m_Device, m_Callback, m_Desc.shaderPath, m_Info, this,
                                                     &m_Kernels)))
            throw std::bad_alloc();

        // The geometry library creates its fixed tables through this device: everything it uses is set up
        // by now. It refers back to the device without a reference, until the destructor detaches it.
        if (NVRHI_FAILED(detail::createGeometryLibrary(this, this, &m_Geometry))) {
            detail::detachKernelLibrary(m_Kernels);
            throw std::bad_alloc();
        }
    }

    ~PhysDevice() {
        // Work recorded on the device's command list is not dropped (and an open immediate command list
        // must not be abandoned).
        flush();
        // The library may outlive the device (a caller may hold a reference): it must not call back.
        // So may the geometry library: an asset keeps it alive.
        detail::detachGeometryLibrary(m_Geometry);
        detail::detachKernelLibrary(m_Kernels);
    }

    const PhysDeviceDesc& getDesc() const noexcept override { return m_Desc; }
    const PhysDeviceInfo& getInfo() const noexcept override { return m_Info; }
    nvrhi::IDevice* getRHIDevice() const noexcept override { return m_Device; }
    IPhysKernelLibrary* getKernels() const noexcept override { return m_Kernels; }
    IGeometryLibrary* getGeometry() const noexcept override { return m_Geometry; }

    nvrhi::FRESULT createDomain(const DomainDesc& desc, IPhysDomain** ppDomain) noexcept override {
        return detail::createDomain(this, desc, ppDomain);
    }

    nvrhi::FRESULT createBuffer(const PhysBufferDesc& desc, IPhysBuffer** ppBuffer) noexcept override {
        if (!ppBuffer) return nvrhi::FE_INVALID_ARGS;
        *ppBuffer = nullptr;

        if (desc.use == BufferUse::RenderShared && !m_Info.supportsRenderSharedBuffers) {
            report(m_Callback, PhysMessageSeverity::Error,
                   "createBuffer(%s): BufferUse::RenderShared is not supported on this graphics API",
                   nameOf(desc.debugName));
            return nvrhi::FE_UNSUPPORTED;
        }

        const char* reason = "";
        const nvrhi::FRESULT result = detail::createPhysBuffer(m_Device, desc, ppBuffer, &reason);
        if (NVRHI_FAILED(result))
            report(m_Callback, PhysMessageSeverity::Error, "createBuffer(%s): %s", nameOf(desc.debugName), reason);
        return result;
    }

    nvrhi::FRESULT adoptBuffer(nvrhi::IBuffer* buffer, uint32_t elementCount, uint32_t elementStride,
                               IPhysBuffer** ppBuffer) noexcept override {
        if (!ppBuffer) return nvrhi::FE_INVALID_ARGS;
        *ppBuffer = nullptr;

        const char* reason = "";
        const nvrhi::FRESULT result =
            detail::adoptPhysBuffer(m_Device, buffer, elementCount, elementStride, ppBuffer, &reason);
        if (NVRHI_FAILED(result)) {
            report(m_Callback, PhysMessageSeverity::Error, "adoptBuffer: %s", reason);
            return result;
        }

        const nvrhi::BufferDesc& rhiDesc = buffer->getDesc();
        if (!rhiDesc.keepInitialState && rhiDesc.cpuAccess == nvrhi::CpuAccessMode::None)
            report(m_Callback, PhysMessageSeverity::Warning,
                   "adoptBuffer(%s): the buffer has no keepInitialState; it needs a permanent state, or "
                   "beginTrackingBufferState on every command list that uses it",
                   nameOf((*ppBuffer)->getDesc().debugName));

        const nvrhi::ResourceStates graphicsOnly =
            nvrhi::ResourceStates::VertexBuffer | nvrhi::ResourceStates::IndexBuffer;
        if (m_Info.executionQueue == nvrhi::CommandQueue::Compute && rhiDesc.keepInitialState &&
            (rhiDesc.initialState & graphicsOnly) != 0)
            report(m_Callback, PhysMessageSeverity::Warning,
                   "adoptBuffer(%s): the buffer rests in a vertex or index buffer state, which the compute "
                   "queue of the engine cannot leave or restore; use it on a graphics command list only, or "
                   "give it the initial state UnorderedAccess",
                   nameOf((*ppBuffer)->getDesc().debugName));
        return result;
    }

    nvrhi::FRESULT writeBuffer(nvrhi::ICommandList* commandList, IPhysBuffer* dst, const void* data,
                               uint32_t dstOffset, uint32_t elementCount) noexcept override {
        if (!dst || (!data && elementCount != 0)) return fail(nvrhi::FE_INVALID_ARGS, "writeBuffer: null argument");

        const PhysBufferDesc& desc = dst->getDesc();
        if (desc.use == BufferUse::Upload || desc.use == BufferUse::Readback)
            return fail(nvrhi::FE_UNSUPPORTED, "writeBuffer: the destination is CPU-visible; use map()");
        if (uint64_t(dstOffset) + uint64_t(elementCount) > desc.elementCount)
            return fail(nvrhi::FE_INVALID_ARGS, "writeBuffer: the range exceeds the destination");
        if (elementCount == 0) return nvrhi::FS_OK;
        // D3D11 updates a constant buffer as a whole or not at all (UpdateSubresource takes no box for one).
        if (desc.use == BufferUse::Constant && m_Info.graphicsAPI == nvrhi::GraphicsAPI::D3D11 &&
            (dstOffset != 0 || elementCount != desc.elementCount))
            return fail(nvrhi::FE_UNSUPPORTED,
                        "writeBuffer: D3D11 cannot write part of a Constant buffer; write all of it");

        nvrhi::ICommandList* list = recordingList(commandList);
        if (!list) return fail(nvrhi::FE_GENERIC_ERROR, "writeBuffer: no command list");

        const uint64_t stride = desc.elementStride;
        list->writeBuffer(dst->getRHIBuffer(), data, size_t(elementCount * stride), dstOffset * stride);
        dst->markContentChanged();
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT copyBuffer(nvrhi::ICommandList* commandList, IPhysBuffer* dst, IPhysBuffer* src,
                              const CopyRegion& region) noexcept override {
        if (!dst || !src) return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: null buffer");

        const PhysBufferDesc& dstDesc = dst->getDesc();
        const PhysBufferDesc& srcDesc = src->getDesc();
        if (dst->getRHIBuffer() == src->getRHIBuffer())
            return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: source and destination are the same buffer");
        if (dstDesc.elementStride != srcDesc.elementStride)
            return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: the element strides differ");
        if (dstDesc.use == BufferUse::Upload)
            return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: an Upload buffer cannot be a copy destination");
        if (srcDesc.use == BufferUse::Readback)
            return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: a Readback buffer cannot be a copy source");
        if (uint64_t(region.srcOffset) + uint64_t(region.elementCount) > srcDesc.elementCount ||
            uint64_t(region.dstOffset) + uint64_t(region.elementCount) > dstDesc.elementCount)
            return fail(nvrhi::FE_INVALID_ARGS, "copyBuffer: the region exceeds a buffer");
        if (region.elementCount == 0) return nvrhi::FS_OK;

        nvrhi::ICommandList* list = recordingList(commandList);
        if (!list) return fail(nvrhi::FE_GENERIC_ERROR, "copyBuffer: no command list");

        const uint64_t stride = dstDesc.elementStride;
        list->copyBuffer(dst->getRHIBuffer(), region.dstOffset * stride, src->getRHIBuffer(),
                         region.srcOffset * stride, region.elementCount * stride);
        dst->markContentChanged();
        return nvrhi::FS_OK;
    }

    uint64_t flush() noexcept override {
        if (!m_CommandListOpen) return 0;
        m_CommandList->close();
        m_CommandListOpen = false;
        nvrhi::ICommandList* list = m_CommandList;
        return m_Device->executeCommandLists(&list, 1, m_Info.executionQueue);
    }

    nvrhi::FRESULT waitForIdle() noexcept override {
        flush();
        if (m_Device->waitForIdle()) return nvrhi::FS_OK;
        report(m_Callback, PhysMessageSeverity::Fatal, "waitForIdle: the nvrhi device reported a problem");
        return nvrhi::FE_GENERIC_ERROR;
    }

 private:
    static const char* nameOf(const char* debugName) { return debugName ? debugName : "<unnamed>"; }

    nvrhi::FRESULT fail(nvrhi::FRESULT code, const char* text) noexcept {
        report(m_Callback, PhysMessageSeverity::Error, "%s", text);
        return code;
    }

    // The command list a recording method writes to: the caller's, or the device's own, opened on demand.
    // Also what the kernel library records on (detail::KernelLibraryHost).
    nvrhi::ICommandList* recordingList(nvrhi::ICommandList* commandList) noexcept override {
        if (commandList) return commandList;

        if (!m_CommandList) {
            nvrhi::CommandListParameters params;
            params.queueType = m_Info.executionQueue;
            // A deferred command list where the backend has them, so that the application's own command
            // list may be open at the same time; D3D11 has only the immediate context.
            params.enableImmediateExecution = !m_Device->queryFeatureSupport(nvrhi::Feature::DeferredCommandLists);
            if (NVRHI_FAILED(m_Device->createCommandList(params, &m_CommandList)) || !m_CommandList) return nullptr;
        }
        if (!m_CommandListOpen) {
            m_CommandList->open();
            m_CommandListOpen = true;
        }
        return m_CommandList;
    }

    nvrhi::DeviceHandle m_Device;
    PhysMessageCallbackHandle m_Callback;
    nvrhi::string m_ShaderPath;
    PhysDeviceDesc m_Desc;
    PhysDeviceInfo m_Info;
    PhysKernelLibraryHandle m_Kernels;
    GeometryLibraryHandle m_Geometry;
    nvrhi::CommandListHandle m_CommandList;
    bool m_CommandListOpen = false;
};

}  // namespace
}  // namespace ephys

ETHEREAL_PHYS_C_API nvrhi::FRESULT etherealPhysCreateDevice(const ephys::PhysDeviceDesc* desc,
                                                            uint32_t headerVersion,
                                                            ephys::IPhysDevice** ppDevice) noexcept {
    if (ppDevice) *ppDevice = nullptr;

    // First, and before desc is read: with another header version its layout is not the caller's.
    if (headerVersion != ephys::c_HeaderVersion) return nvrhi::FE_UNSUPPORTED;

    if (!desc || !ppDevice || !desc->device) return nvrhi::FE_INVALID_ARGS;

    if (desc->nvrhiHeaderVersion != nvrhi::c_HeaderVersion) {
        ephys::report(desc->messageCallback, ephys::PhysMessageSeverity::Error,
                      "etherealPhysCreateDevice: the caller was compiled with nvrhi header version %u, "
                      "ethereal_phys with %u",
                      desc->nvrhiHeaderVersion, nvrhi::c_HeaderVersion);
        return nvrhi::FE_UNSUPPORTED;
    }

    try {
        *ppDevice = MAKE_RC_OBJ(ephys::PhysDevice, *desc);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}
