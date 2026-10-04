#include "ContextD3D12.h"
#include "Device.h"
#include "d3dx12.h"
#include <mutex>
#include <condition_variable>

#define H_VERIFY(expr)           \
    if (FAILED(hr = expr)) {       \
        NVFLEX_ASSERT(0 && #expr); \
    }

namespace NvFlex {

static uint32_t blockingRelease_refCount = 0;
static std::condition_variable blockingRelease_cv;
static std::mutex blockingRelease_mutex;

static HMODULE GetModule() {
    HMODULE module = 0;
    GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, NULL, &module);
    return module;
}

template <typename T>
static void swapIfChange(T *&currentT, T *newT) {
    if (currentT != newT) {
        currentT->Release();
        currentT = newT;
        currentT->AddRef();
    }
}

struct BlendEnumTableD3D12 {
    BlendEnumTableD3D12() {
        this->table[1] = D3D12_BLEND_ZERO;
        this->table[2] = D3D12_BLEND_ONE;
        this->table[3] = D3D12_BLEND_SRC_ALPHA;
        this->table[4] = D3D12_BLEND_INV_SRC_ALPHA;
        this->table[5] = D3D12_BLEND_DEST_ALPHA;
        this->table[6] = D3D12_BLEND_INV_DEST_ALPHA;
    }

    D3D12_BLEND table[7];
} const g_blendEnumTableD3D12;

struct BlendOpEnumTableD3D12 {
    BlendOpEnumTableD3D12() {
        this->table[1] = D3D12_BLEND_OP_ADD;
        this->table[2] = D3D12_BLEND_OP_SUBTRACT;
        this->table[3] = D3D12_BLEND_OP_REV_SUBTRACT;
        this->table[4] = D3D12_BLEND_OP_MIN;
        this->table[5] = D3D12_BLEND_OP_MAX;
    }

    D3D12_BLEND_OP table[6];
} const g_blendOpEnumTableD3D12;

struct ComparisonTableD3D12 {
    ComparisonTableD3D12() {
        this->table[1] = D3D12_COMPARISON_FUNC_NEVER;
        this->table[2] = D3D12_COMPARISON_FUNC_LESS;
        this->table[3] = D3D12_COMPARISON_FUNC_EQUAL;
        this->table[4] = D3D12_COMPARISON_FUNC_LESS_EQUAL;
        this->table[5] = D3D12_COMPARISON_FUNC_GREATER;
        this->table[6] = D3D12_COMPARISON_FUNC_NOT_EQUAL;
        this->table[7] = D3D12_COMPARISON_FUNC_GREATER_EQUAL;
        this->table[8] = D3D12_COMPARISON_FUNC_ALWAYS;
    }

    D3D12_COMPARISON_FUNC table[9];
} g_comparisonTableD3D12;

void copyViewport(NvFlexViewport *dst, const D3D12_VIEWPORT *src) {
    *(D3D12_VIEWPORT *)dst = *src;
}

D3D12_BLEND convertToD3D12(NvFlexBlendEnum blendEnum) {
    return g_blendEnumTableD3D12.table[blendEnum];
}

D3D12_BLEND_OP convertToD3D12(NvFlexBlendOpEnum blendOpEnum) {
    return g_blendOpEnumTableD3D12.table[blendOpEnum];
}

D3D12_COMPARISON_FUNC convertToD3D12(NvFlexComparisonEnum comparison) {
    return g_comparisonTableD3D12.table[comparison];
}

int64_t FlexDeferredReleaseD3D12(float timeoutMS) {
    std::unique_lock<std::mutex> lock{blockingRelease_mutex};

    if (blockingRelease_refCount)
        blockingRelease_cv.wait_for(lock,
                                    std::chrono::milliseconds{static_cast<int>(timeoutMS)});

    return blockingRelease_refCount;
}

DeferredReleaseD3D12 *createDeferredRelease(const NvFlexContextDescD3D12 *desc) {
    return new DeferredReleaseD3D12(desc);
}

extern bool FlexDedicatedDeviceAvailableD3D12(NvFlexContext *context) {
    return 1;
}

extern bool FlexDedicatedDeviceQueueAvailableD3D12(NvFlexContext *renderContext) {
    return 1;
}

namespace {

[[nodiscard]]  int transitionResourceBarrier(D3D12_RESOURCE_BARRIER *barrier, ID3D12Resource *resource,
                               D3D12_RESOURCE_STATES *state,
                               D3D12_RESOURCE_STATES dstState) {
    int count = 0;
    if (*state == D3D12_RESOURCE_STATE_UNORDERED_ACCESS &&
        dstState == D3D12_RESOURCE_STATE_UNORDERED_ACCESS) {
        *barrier = CD3DX12_RESOURCE_BARRIER::UAV(resource);
        count = 1;
    } else if(*state != dstState) {
        *barrier = CD3DX12_RESOURCE_BARRIER::Transition(resource, *state, dstState);
        *state = dstState;
        count = 1;
    }

    return count;
}

void transitionResourceBarrier(ID3D12GraphicsCommandList *commandList,
                               ID3D12Resource *resource, D3D12_RESOURCE_STATES *state,
                               D3D12_RESOURCE_STATES dstState) {
    if (*state != dstState) {
        D3D12_RESOURCE_BARRIER barrier;
        int count = transitionResourceBarrier(&barrier, resource, state, dstState);
        if(count)
            commandList->ResourceBarrier(count, &barrier);
    }
}

}  // namespace

uint64_t DeferredReleaseD3D12::getGPUBytesUsed() {
    return 0;
}

void DeferredReleaseD3D12::registerObject(NvFlexObject *object) {
    ++m_deferredObjectRefCount;
    addRef();
}

void DeferredReleaseD3D12::pushForRelease(NvFlexObject *object) {
    DeferredElement element;
    element.object = object;
    element.releaseFenceID = m_nextFenceValue;
    m_deferredReleaseObjects.push_back(element);
    --m_deferredObjectRefCount;
    release();
}

DeferredReleaseD3D12::DeferredReleaseD3D12(const NvFlexContextDescD3D12 *desc)
    : m_d3d12module{},
      m_device{},
      m_commandQueueFence{},
      m_lastFenceCompleted{},
      m_nextFenceValue{},
      m_module{},
      m_deferredObjectRefCount{},
      m_deferredReleaseObjects{} {
    m_d3d12module = LoadLibraryW(L"d3d12.dll");
    m_device = desc->device;
    m_device->AddRef();
    m_commandQueueFence = desc->commandQueueFence;
    m_commandQueueFence->AddRef();
    m_lastFenceCompleted = desc->lastFenceCompleted;
    m_nextFenceValue = desc->nextFenceValue;
}

DeferredReleaseD3D12::~DeferredReleaseD3D12() {
    SafeRelease(m_commandQueueFence);
    SafeRelease(m_device);
    FreeLibrary(m_d3d12module);
}

void DeferredReleaseD3D12::blockingRelease() {
    HANDLE fenceEvent = CreateEventW(0, FALSE, FALSE, 0);

    while (m_deferredReleaseObjects.size()) {
        uint64_t minFenceValue = m_deferredReleaseObjects[0].releaseFenceID;
        for (uint32_t i = 1; i < m_deferredReleaseObjects.size(); ++i) {
            auto &e = m_deferredReleaseObjects[i];
            if (e.releaseFenceID < minFenceValue)
                minFenceValue = e.releaseFenceID;
        }

        m_lastFenceCompleted = m_commandQueueFence->GetCompletedValue();
        if (m_lastFenceCompleted < minFenceValue) {
            m_commandQueueFence->SetEventOnCompletion(minFenceValue, fenceEvent);
            WaitForSingleObjectEx(fenceEvent, INFINITE, FALSE);
        }
        m_lastFenceCompleted = m_commandQueueFence->GetCompletedValue();
        doDeferredRelease();
    }
    CloseHandle(fenceEvent);

    HMODULE module = m_module;

    release();

    {
        std::unique_lock<std::mutex> lock(blockingRelease_mutex);
        if (!--blockingRelease_refCount)
            blockingRelease_cv.notify_all();
    }

    // FreeLibraryAndExitThread(module, 0);
}

void DeferredReleaseD3D12::contextReleaseNotify() {
    addRef();

    m_module = GetModule();

    {
        std::unique_lock<std::mutex> lock(blockingRelease_mutex);
        ++blockingRelease_refCount;
    }

    std::thread t(threadFunc, this);
    t.detach();
}

void DeferredReleaseD3D12::doDeferredRelease() {
    uint32_t listSize = m_deferredReleaseObjects.size();
    for (uint32_t i = 0; i < listSize; ++i) {
        auto &e = m_deferredReleaseObjects[i];
        if (e.releaseFenceID <= m_lastFenceCompleted) {
            SafeRelease(e.object);
        }
    }

    uint32_t readIdx = 0, writeIdx = 0;
    while (readIdx < m_deferredReleaseObjects.size()) {
        auto &e = m_deferredReleaseObjects[readIdx++];
        if (e.object)
            m_deferredReleaseObjects[writeIdx++] = e;
    }
    m_deferredReleaseObjects.resize(writeIdx);
}

void DeferredReleaseD3D12::threadFunc(DeferredReleaseD3D12 *ptr) {
    ptr->blockingRelease();
}

void DeferredReleaseD3D12::update(const NvFlexContextDescD3D12 *desc) {
    swapIfChange(m_device, desc->device);
    swapIfChange(m_commandQueueFence, desc->commandQueueFence);
    m_lastFenceCompleted = desc->lastFenceCompleted;
    m_nextFenceValue = desc->nextFenceValue;
    doDeferredRelease();
}

uint64_t ConstantBufferD3D12::getGPUBytesUsed() {
    return 0;
}

ConstantBufferD3D12::ConstantBufferD3D12(ContextD3D12 *context,
                                         const NvFlexConstantBufferDesc *desc)
    : Object{context->getDeferredRelease()},
      m_buffers{},
      m_bufferGPU{} {
    m_desc = *desc;

    auto device = context->getDevice();
    if (desc->uploadAccess) {
        context->map(this);
        context->unmap(this);
    } else {
        device->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT},
                                        D3D12_HEAP_FLAG_NONE,
                                        &CD3DX12_RESOURCE_DESC::Buffer(desc->sizeInBytes),
                                        D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER,
                                        nullptr, IID_PPV_ARGS(&m_bufferGPU));
        if (m_desc.debugName)
            m_bufferGPU->SetPrivateData(WKPDID_D3DDebugObjectName, strlen(m_desc.debugName),
                                     m_desc.debugName);
    }
}

ConstantBufferD3D12::~ConstantBufferD3D12() {
    SafeRelease(m_bufferGPU);
}

ID3D12Resource *ConstantBufferD3D12::getFront() {
    if (m_desc.uploadAccess)
        return m_buffers.front()->m_buffer;
    else
        return m_bufferGPU;
}

uint64_t VertexBufferD3D12::getGPUBytesUsed() {
    return Object::getGPUBytesUsed();
}

VertexBufferD3D12::VertexBufferD3D12(ContextD3D12 *context,
                                     const NvFlexVertexBufferDesc *desc)
    : Object(context->getDeferredRelease()),
      m_buffer{},
      m_uploadBuffers{} {
    m_desc = *desc;
    auto device = context->getDevice();

    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT}, D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(m_desc.sizeInBytes),
        D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, 0, IID_PPV_ARGS(&m_buffer));

    auto pdata = context->map(this);
    if (pdata) {
        CopyMemory(pdata, desc->data, desc->sizeInBytes);
        context->unmap(this);
    }
}

VertexBufferD3D12::~VertexBufferD3D12() {
    SafeRelease(m_buffer);
}

uint64_t IndexBufferD3D12::getGPUBytesUsed() {
    return 0;
}

IndexBufferD3D12::IndexBufferD3D12(ContextD3D12 *context, const NvFlexIndexBufferDesc *desc)
    : Object{context->getDeferredRelease()},
      m_buffer{},
      m_uploadBuffers{},
      m_format{} {
    m_desc = *desc;
    m_format = convertToDXGI(desc->format);

    context->getDevice()->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT}, D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(desc->sizeInBytes),
        D3D12_RESOURCE_STATE_INDEX_BUFFER, 0, IID_PPV_ARGS(&m_buffer));
    auto pdata = context->map(this);
    if (pdata) {
        CopyMemory(pdata, desc->data, desc->sizeInBytes);
        context->unmap(this);
    }
}

IndexBufferD3D12::~IndexBufferD3D12() {
    SafeRelease(m_buffer);
}

ResourceStateD3D12::ResourceStateD3D12(ID3D12Resource **resource,
                                       D3D12_RESOURCE_STATES *resourceState,
                                       D3D12_RESOURCE_STATES *restoreResourceState)
    : ref_resource{resource},
      ref_resourceState{resourceState},
      ref_restoreResourceState{restoreResourceState} {}

ResourceD3D12::ResourceD3D12(ID3D12Resource **resource,
                             D3D12_RESOURCE_STATES *resourceState,
                             D3D12_RESOURCE_STATES *restoreResourceState)
    : ResourceStateD3D12{resource, resourceState, restoreResourceState},
      m_srvHandle{},
      m_srvDesc{} {}

ResourceRWD3D12::ResourceRWD3D12(ID3D12Resource **resource,
                                 D3D12_RESOURCE_STATES *resourceState,
                                 D3D12_RESOURCE_STATES *restoreResourceState)
    : ResourceD3D12{resource, resourceState, restoreResourceState},
      m_uavHandle{},
      m_uavDesc{} {}

DepthStencilD3D12::DepthStencilD3D12(ID3D12Resource **resource,
                                     D3D12_RESOURCE_STATES *resourceState,
                                     D3D12_RESOURCE_STATES *restoreResourceState)
    : ResourceStateD3D12{resource, resourceState, restoreResourceState},
      m_dsvHandle{},
      m_dsvDesc{} {}

RenderTargetD3D12::RenderTargetD3D12(ID3D12Resource **resource,
                                     D3D12_RESOURCE_STATES *resourceState,
                                     D3D12_RESOURCE_STATES *restoreResourceState)
    : ResourceStateD3D12{resource, resourceState, restoreResourceState},
      m_rtvHandle{},
      m_rtvDesc{} {}

uint64_t BufferD3D12::getGPUBytesUsed() {
    return m_desc.dim * getFormatSizeInBytes(m_desc.format);
}

NvFlexContext *BufferD3D12::getContext() {
    return m_context;
}

Resource *BufferD3D12::getResource() {
    return this;
}

ResourceRW *BufferD3D12::getResourceRW() {
    return this;
}

BufferD3D12::BufferD3D12(ContextD3D12 *context, const NvFlexBufferDesc *desc)
    : Object(context->getDeferredRelease()),
      ResourceRWD3D12{&m_buffer, &m_resourceState, &m_restoreResourceState},
      m_context{context},
      m_parent{},
      m_buffer{},
      m_uploadBuffers{},
      m_readbackComputeFence{-1ull},
      m_resourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE},
      m_restoreResourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE} {
    m_desc = *desc;

    if (m_desc.format != eNvFlexFormat_unknown)
        NVFLEX_ASSERT(m_desc.structStride == 0);
    else
        NVFLEX_ASSERT(m_desc.structStride != 0);

    const bool isTypeless = isFormatTypeless(m_desc.format);
    const bool isStructured = m_desc.structStride != 0;
    const uint32_t elementSizeInBytes =
        isStructured ? m_desc.structStride : getFormatSizeInBytes(m_desc.format);

    auto device = context->getDevice();

    uint64_t size = m_desc.dim * elementSizeInBytes;

    D3D12_HEAP_TYPE heapType;
    D3D12_RESOURCE_FLAGS flags;
    switch (m_desc.memType) {
        case eNvFlexMemoryType_default:
        default:
            heapType = D3D12_HEAP_TYPE_DEFAULT;
            flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            break;
        case eNvFlexMemoryType_upload:
            heapType = D3D12_HEAP_TYPE_UPLOAD;
            flags = D3D12_RESOURCE_FLAG_NONE;
            break;
        case eNvFlexMemoryType_readback:
            heapType = D3D12_HEAP_TYPE_READBACK;
            flags = D3D12_RESOURCE_FLAG_NONE;
            m_resourceState = D3D12_RESOURCE_STATE_COPY_DEST;
            m_restoreResourceState = D3D12_RESOURCE_STATE_COPY_DEST;
            break;
    }

    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{heapType}, D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(size, flags),
        m_resourceState, 0, IID_PPV_ARGS(&m_buffer));
    if(m_desc.debugName)
        m_buffer->SetPrivateData(WKPDID_D3DDebugObjectName, strlen(m_desc.debugName),
                                 m_desc.debugName);

    if (m_desc.memType == eNvFlexMemoryType_default) {
        D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
        srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
        srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srvDesc.Format = convertToDXGI(m_desc.format);
        srvDesc.Buffer.FirstElement = 0;
        srvDesc.Buffer.NumElements = m_desc.dim;
        srvDesc.Buffer.StructureByteStride = isStructured ? elementSizeInBytes : 0;
        if(isTypeless)
            srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;

        m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
        m_srvDesc = srvDesc;
        device->CreateShaderResourceView(m_buffer, &srvDesc, m_srvHandle);

        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
        uavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
        uavDesc.Format = convertToDXGI(m_desc.format);
        uavDesc.Buffer.FirstElement = 0;
        uavDesc.Buffer.NumElements = m_desc.dim;
        uavDesc.Buffer.StructureByteStride = isStructured ? elementSizeInBytes : 0;
        if(isTypeless)
            uavDesc.Buffer.Flags = D3D12_BUFFER_UAV_FLAG_RAW;

        m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
        m_uavDesc = uavDesc;
        device->CreateUnorderedAccessView(m_buffer, nullptr, &uavDesc, m_uavHandle);
    }
}

BufferD3D12::BufferD3D12(ContextD3D12 *context, BufferD3D12 *buffer,
                         const NvFlexBufferViewDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_buffer, buffer->ref_resourceState, buffer->ref_restoreResourceState},
      m_context{context},
      m_parent{},
      m_buffer{},
      m_uploadBuffers{},
      m_readbackComputeFence{-1ull},
      m_resourceState{*buffer->ref_resourceState},
      m_restoreResourceState{*buffer->ref_restoreResourceState} {
    NVFLEX_ASSERT(buffer->m_desc.structStride == 0 &&
                  buffer->m_desc.format != eNvFlexFormat_unknown);
    m_desc = buffer->m_desc;
    uint32_t srcFormatSize = getFormatSizeInBytes(m_desc.format);
    m_desc.format = desc->format;
    uint32_t newFormatSize = getFormatSizeInBytes(m_desc.format);
    m_desc.dim = m_desc.dim * srcFormatSize / newFormatSize;

    m_parent = buffer;
    m_parent->addRef();
    m_buffer = buffer->m_buffer;
    m_buffer->AddRef();

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Buffer.FirstElement = 0;
    srvDesc.Buffer.NumElements = m_desc.dim;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;

    auto device = context->getDevice();
    device->CreateShaderResourceView(m_buffer, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Buffer.FirstElement = 0;
    uavDesc.Buffer.NumElements = m_desc.dim;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;

    device->CreateUnorderedAccessView(m_buffer, nullptr, &uavDesc, m_uavHandle);
}

BufferD3D12::BufferD3D12(ContextD3D12 *context, ID3D12Resource *nativeBuffer,
                         NvFlexUint dim, NvFlexUint structStride)
    : Object(context->getDeferredRelease()),
      ResourceRWD3D12{&m_buffer, &m_resourceState, &m_restoreResourceState},
      m_context(context),
      m_buffer(nativeBuffer) {
    m_buffer->AddRef();

    memset(&m_desc, 0, sizeof(m_desc));

    D3D12_HEAP_PROPERTIES d3dHeapProps;
    D3D12_RESOURCE_DESC d3dBufDesc;
    d3dBufDesc = m_buffer->GetDesc();
    m_buffer->GetHeapProperties(&d3dHeapProps, nullptr);

    switch (d3dHeapProps.Type) {
        case D3D12_HEAP_TYPE_DEFAULT:
        default:
            m_desc.memType = eNvFlexMemoryType_default;
            m_desc.cpuAccessMode = eNvFlexCpuAccessMode_none;
            break;
        case D3D12_HEAP_TYPE_UPLOAD:
            m_desc.memType = eNvFlexMemoryType_upload;
            m_desc.cpuAccessMode = eNvFlexCpuAccessMode_write;
            break;
        case D3D12_HEAP_TYPE_READBACK:
            m_desc.memType = eNvFlexMemoryType_readback;
            m_desc.cpuAccessMode = eNvFlexCpuAccessMode_readwrite;
            break;
    }

    m_desc.dim = dim;
    m_desc.structStride = structStride;

    auto device = context->getDevice();
    UINT size = m_desc.dim * m_desc.structStride;
}

BufferD3D12::~BufferD3D12() {
    SafeRelease(m_parent);
    SafeRelease(m_buffer);
}

uint64_t Texture1DD3D12::getGPUBytesUsed() {
    return m_desc.dim * getFormatSizeInBytes(m_desc.format);
}

Resource *Texture1DD3D12::getResource() {
    return this;
}

ResourceRW *Texture1DD3D12::getResourceRW() {
    return this;
}

Texture1DD3D12::Texture1DD3D12(ContextD3D12 *context, const NvFlexTexture1DDesc *desc)
    : Object(context->getDeferredRelease()),
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_texture{},
      m_uploadBuffers{},
      m_footPrint{},
      m_uploadHeapSize{},
      m_resourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE},
      m_restoreResourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE} {
    m_desc = *desc;

    auto device = context->getDevice();

    auto texDesc =
        CD3DX12_RESOURCE_DESC::Tex1D(convertToDXGI(m_desc.format), m_desc.dim, 1, 0,
                                     D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT}, D3D12_HEAP_FLAG_NONE, &texDesc,
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, 0, IID_PPV_ARGS(&m_texture));

    device->GetCopyableFootprints(&texDesc, 0, 1, 0, &m_footPrint, nullptr, nullptr,
                                  &m_uploadHeapSize);

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE1D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Texture1D.MostDetailedMip = 0;
    srvDesc.Texture1D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;
    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Texture1D.MipSlice = 0;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;
    device->CreateUnorderedAccessView(m_texture, 0, &uavDesc, m_uavHandle);
}

Texture1DD3D12::~Texture1DD3D12() {
    SafeRelease(m_texture);
}

uint64_t Texture2DD3D12::getGPUBytesUsed() {
    return m_desc.height * m_desc.width * getFormatSizeInBytes(m_desc.format);
}

Resource *Texture2DD3D12::getResource() {
    return this;
}

ResourceRW *Texture2DD3D12::getResourceRW() {
    return this;
}

void Texture2DD3D12::openSharedHandle(HANDLE *handleIn) {
    m_device->CreateSharedHandle(m_texture, nullptr, GENERIC_ALL, nullptr, handleIn);
}

void Texture2DD3D12::closeSharedHandle(HANDLE handleIn) {
    CloseHandle(handleIn);
}

Texture2DD3D12::Texture2DD3D12(ContextD3D12 *context, const NvFlexTexture2DDesc *desc,
                               bool createShared)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_device{},
      m_texture{},
      m_resourceState{D3D12_RESOURCE_STATE_COMMON},
      m_restoreResourceState{D3D12_RESOURCE_STATE_COMMON} {
    m_desc = *desc;
    m_device = context->getDevice();
    m_device->AddRef();

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex2D(convertToDXGI(m_desc.format), m_desc.width,
                                                m_desc.height, 1, 0, 1, 0,
                                                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    m_device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT},
        createShared ? D3D12_HEAP_FLAG_SHARED : D3D12_HEAP_FLAG_NONE, &texDesc,
        m_restoreResourceState, nullptr, IID_PPV_ARGS(&m_texture));

    createViews(context);
}

Texture2DD3D12::Texture2DD3D12(ContextD3D12 *context, Texture2D *sharedTexture,
                               bool openShared)
    : Object(context->getDeferredRelease()),
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_device{},
      m_texture{},
      m_resourceState{D3D12_RESOURCE_STATE_COMMON},
      m_restoreResourceState{D3D12_RESOURCE_STATE_COMMON} {
    m_desc = sharedTexture->m_desc;
    m_device = context->getDevice();
    m_device->AddRef();

    if (openShared) {
        HANDLE sharedHandle;
        sharedTexture->openSharedHandle(&sharedHandle);
        m_device->OpenSharedHandle(sharedHandle, IID_PPV_ARGS(&m_texture));
        sharedTexture->closeSharedHandle(sharedHandle);
    } else {
        m_texture = implSafeCast<Texture2DD3D12>(sharedTexture)->m_texture;
        m_texture->AddRef();
    }

    createViews(context);
}

Texture2DD3D12::~Texture2DD3D12() {
    SafeRelease(m_texture);
    SafeRelease(m_device);
}

void Texture2DD3D12::createViews(ContextD3D12 *context) {
    auto device = context->getDevice();

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Texture2D.MostDetailedMip = 0;
    srvDesc.Texture2D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;

    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Texture2D.MipSlice = 0;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;
    device->CreateUnorderedAccessView(m_texture, nullptr, &uavDesc, m_uavHandle);
}

uint64_t Texture3DD3D12::getGPUBytesUsed() {
    return uint64_t(m_desc.dim.z) * m_desc.dim.y * m_desc.dim.x *
           getFormatSizeInBytes(m_desc.format);
}

Resource *Texture3DD3D12::getResource() {
    return this;
}

ResourceRW *Texture3DD3D12::getResourceRW() {
    return this;
}

Texture3DD3D12::Texture3DD3D12(ContextD3D12 *context, const NvFlexTexture3DDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_texture{},
      m_uploadBuffers{},
      m_footPrint{},
      m_uploadHeapSize{},
      m_downloadBuffer{},
      m_downloadCompleteFence{-1ull},
      m_resourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE},
      m_restoreResourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE} {
    m_desc = *desc;

    auto device = context->getDevice();

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex3D(convertToDXGI(m_desc.format), m_desc.dim.x,
                                                m_desc.dim.y, m_desc.dim.z);
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT}, D3D12_HEAP_FLAG_NONE, &texDesc,
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&m_texture));

    device->GetCopyableFootprints(&texDesc, 0, 1, 0, &m_footPrint, nullptr, nullptr,
                                  &m_uploadHeapSize);

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Texture3D.MostDetailedMip = 0;
    srvDesc.Texture3D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;
    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Texture3D.MipSlice = 0;
    uavDesc.Texture3D.FirstWSlice = 0;
    uavDesc.Texture3D.WSize = m_desc.dim.z;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;
    device->CreateUnorderedAccessView(m_texture, nullptr, &uavDesc, m_uavHandle);

    if (m_desc.downloadAccess) {
        device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_READBACK}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(m_uploadHeapSize),
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_downloadBuffer));
    }
}

Texture3DD3D12::~Texture3DD3D12() {
    SafeRelease(m_texture);
    SafeRelease(m_downloadBuffer);
}

uint64_t Texture2DCrossAdapterD3D12::getGPUBytesUsed() {
    return uint64_t(m_desc.height) * m_desc.width * getFormatSizeInBytes(m_desc.format);
}

Texture2DCrossAdapterD3D12::Texture2DCrossAdapterD3D12(
    ContextD3D12 *context, Texture2DCrossAdapterD3D12 *sharedResource)
    : Object{context->getDeferredRelease()},
      ResourceStateD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_footPrint{},
      m_sharedHeap{},
      m_texture{},
      m_device{},
      m_resourceState{D3D12_RESOURCE_STATE_COMMON},
      m_restoreResourceState{D3D12_RESOURCE_STATE_COMMON} {
    m_desc = sharedResource->m_desc;
    m_device = context->getDevice();
    m_device->AddRef();

    HANDLE sharedHandle;
    sharedResource->m_device->CreateSharedHandle(sharedResource->m_sharedHeap, nullptr,
                                                 GENERIC_ALL, nullptr, &sharedHandle);
    CloseHandle(sharedHandle);

    m_device->OpenSharedHandle(sharedHandle, IID_PPV_ARGS(&m_sharedHeap));
    CloseHandle(sharedHandle);

    m_footPrint = sharedResource->m_footPrint;

    D3D12_RESOURCE_DESC texDesc = sharedResource->m_texture->GetDesc();

    m_device->CreatePlacedResource(m_sharedHeap, 0, &texDesc, m_restoreResourceState,
                                   nullptr, IID_PPV_ARGS(&m_texture));
}

Texture2DCrossAdapterD3D12::Texture2DCrossAdapterD3D12(ContextD3D12 *context,
                                                       const NvFlexTexture2DDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceStateD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_footPrint{},
      m_sharedHeap{},
      m_texture{},
      m_device{},
      m_resourceState{D3D12_RESOURCE_STATE_COMMON},
      m_restoreResourceState{D3D12_RESOURCE_STATE_COMMON} {
    m_desc = *desc;
    m_device = context->getDevice();
    m_device->AddRef();

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex2D(convertToDXGI(desc->format), desc->width,
                                                desc->height);
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_CROSS_ADAPTER;
    UINT64 heapSize;
    m_device->GetCopyableFootprints(&texDesc, 0, 1, 0, &m_footPrint, nullptr, nullptr,
                                    &heapSize);

    const UINT32 heapAlignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT;
    heapSize = (heapSize + heapAlignment - 1) / heapAlignment;

    m_device->CreateHeap(
        &CD3DX12_HEAP_DESC{heapSize, D3D12_HEAP_TYPE_DEFAULT, 0,
                           D3D12_HEAP_FLAG_SHARED | D3D12_HEAP_FLAG_SHARED_CROSS_ADAPTER},
        IID_PPV_ARGS(&m_sharedHeap));

    m_device->CreatePlacedResource(m_sharedHeap, 0, &texDesc, m_restoreResourceState, 0,
                                   IID_PPV_ARGS(&m_texture));
}

Texture2DCrossAdapterD3D12::~Texture2DCrossAdapterD3D12() {
    SafeRelease(m_texture);
    SafeRelease(m_sharedHeap);
    SafeRelease(m_device);
}

uint64_t ResourceReferenceD3D12::getGPUBytesUsed() {
    return 0;
}

ResourceReferenceD3D12::ResourceReferenceD3D12(ContextD3D12 *context,
                                               ResourceD3D12 *resource)
    : Object(context->getDeferredRelease()),
      m_resource{} {
    m_resource = *resource->ref_resource;
    m_resource->AddRef();
}

ResourceReferenceD3D12::~ResourceReferenceD3D12() {
    SafeRelease(m_resource);
}

uint64_t HeapVTRD3D12::getGPUBytesUsed() {
    return m_desc.sizeInBytes;
}

HeapVTRD3D12::HeapVTRD3D12(ContextD3D12 *context, const NvFlexHeapSparseDesc *desc)
    : Object(context->getDeferredRelease()),
      m_heap{} {
    m_desc = *desc;

    constexpr uint32_t tileSize = D3D12_TILED_RESOURCE_TILE_SIZE_IN_BYTES;
    m_numTiles = (m_desc.sizeInBytes + tileSize - 1) / tileSize;
    const UINT64 heapSize = m_numTiles * tileSize;

    context->getDevice()->CreateHeap(
        &CD3DX12_HEAP_DESC{
            heapSize, D3D12_HEAP_TYPE_DEFAULT, 0,
            D3D12_HEAP_FLAG_DENY_BUFFERS | D3D12_HEAP_FLAG_DENY_RT_DS_TEXTURES},
        IID_PPV_ARGS(&m_heap));
}

HeapVTRD3D12::~HeapVTRD3D12() {
    SafeRelease(m_heap);
}

uint64_t Texture3DVTRD3D12::getGPUBytesUsed() {
    return 0;
}

Resource *Texture3DVTRD3D12::getResource() {
    return this;
}

ResourceRW *Texture3DVTRD3D12::getResourceRW() {
    return this;
}

Texture3DVTRD3D12::Texture3DVTRD3D12(ContextD3D12 *context,
                                     const NvFlexTexture3DSparseDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_texture{},
      m_blockTable{},
      m_resourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE},
      m_restoreResourceState{D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE} {
    m_desc = *desc;

    auto device = context->getDevice();

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex3D(convertToDXGI(m_desc.format), m_desc.dim.x,
                                                m_desc.dim.y, m_desc.dim.z);
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    texDesc.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;

    device->CreateReservedResource(&texDesc, m_resourceState, nullptr,
                                   IID_PPV_ARGS(&m_texture));

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Texture3D.MostDetailedMip = 0;
    srvDesc.Texture3D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;

    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Texture3D.WSize = m_desc.dim.z;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;

    device->CreateUnorderedAccessView(m_texture, nullptr, &uavDesc, m_uavHandle);

    m_blockDim = getTileDim(m_desc.format);
    m_gridDim = m_desc.dim / m_blockDim;

    m_blockTable.init(m_gridDim);
    ZeroMemory(m_blockTable.data(), m_blockTable.dim1() * sizeof(uint32_t));
}

Texture3DVTRD3D12::~Texture3DVTRD3D12() {
    SafeRelease(m_texture);
}

uint64_t ColorBufferD3D12::getGPUBytesUsed() {
    return uint64_t(m_desc.height) * m_desc.width * getFormatSizeInBytes(m_desc.format);
}

Resource *ColorBufferD3D12::getResource() {
    return this;
}

ResourceRW *ColorBufferD3D12::getResourceRW() {
    return this;
}

RenderTarget *ColorBufferD3D12::getRenderTarget() {
    return this;
}

ColorBufferD3D12::ColorBufferD3D12(ContextD3D12 *context, const NvFlexColorBufferDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      RenderTargetD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_texture{},
      m_resourceState{D3D12_RESOURCE_STATE_RENDER_TARGET},
      m_restoreResourceState{D3D12_RESOURCE_STATE_RENDER_TARGET} {
    m_desc = *desc;
    m_rt_format = m_desc.format;
    m_viewport.topLeftX = 0.f;
    m_viewport.topLeftY = 0.f;
    m_viewport.width = m_desc.width;
    m_viewport.height = m_desc.height;
    m_viewport.minDepth = 0.f;
    m_viewport.maxDepth = 1.f;
    m_scissor.left = 0;
    m_scissor.top = 0;
    m_scissor.right = m_desc.width;
    m_scissor.bottom = m_desc.height;

    auto device = context->getDevice();

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex2D(convertToDXGI(m_desc.format), m_desc.width,
                                                m_desc.height);
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS |
                    D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

    D3D12_CLEAR_VALUE clearValue = {};
    clearValue.Format = convertToDXGI(m_desc.format);
    clearValue.Color[3] = 1.f;
    device->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT},
                                    D3D12_HEAP_FLAG_NONE, &texDesc, m_restoreResourceState,
                                    &clearValue, IID_PPV_ARGS(&m_texture));

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.Texture2D.MostDetailedMip = 0;
    srvDesc.Texture2D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;
    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    uavDesc.Format = convertToDXGI(m_desc.format);
    uavDesc.Texture2D.MipSlice = 0;

    m_uavHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_uavDesc = uavDesc;
    device->CreateUnorderedAccessView(m_texture, nullptr, &uavDesc, m_uavHandle);

    D3D12_RENDER_TARGET_VIEW_DESC rtvDesc = {};
    rtvDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    rtvDesc.Format = convertToDXGI(m_desc.format);
    rtvDesc.Texture2D.MipSlice = 0;

    m_rtvHandle = context->m_rtvDescriptorHeap->allocate(1);
    m_rtvDesc = rtvDesc;
    device->CreateRenderTargetView(m_texture, &rtvDesc, m_rtvHandle);
}

ColorBufferD3D12::~ColorBufferD3D12() {
    SafeRelease(m_texture);
}

uint64_t DepthBufferD3D12::getGPUBytesUsed() {
    return uint64_t(getFormatSizeInBytes(m_desc.format_resource)) * m_desc.height *
           m_desc.width;
}

Resource *DepthBufferD3D12::getResource() {
    return this;
}

DepthStencil *DepthBufferD3D12::getDepthStencil() {
    return this;
}

DepthBufferD3D12::DepthBufferD3D12(ContextD3D12 *context, const NvFlexDepthBufferDesc *desc)
    : Object{context->getDeferredRelease()},
      ResourceRWD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      DepthStencilD3D12{&m_texture, &m_resourceState, &m_restoreResourceState},
      m_texture{},
      m_resourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE},
      m_restoreResourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE} {
    m_desc = *desc;
    m_ds_format = m_desc.format_dsv;
    m_viewport.topLeftX = 0.f;
    m_viewport.topLeftY = 0.f;
    m_viewport.width = m_desc.width;
    m_viewport.height = m_desc.height;
    m_viewport.minDepth = 0.f;
    m_viewport.maxDepth = 1.f;

    auto texDesc = CD3DX12_RESOURCE_DESC::Tex2D(convertToDXGI(m_desc.format_resource),
                                                m_desc.width, m_desc.height);
    texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
    D3D12_CLEAR_VALUE clearValue = {};
    clearValue.Format = convertToDXGI(m_desc.format_dsv);
    clearValue.DepthStencil.Depth = 1.f;

    auto device = context->getDevice();
    device->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT},
                                    D3D12_HEAP_FLAG_NONE, &texDesc, m_restoreResourceState,
                                    &clearValue, IID_PPV_ARGS(&m_texture));

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = convertToDXGI(m_desc.format_srv);
    srvDesc.Texture2D.MostDetailedMip = 0;
    srvDesc.Texture2D.MipLevels = 1;

    m_srvHandle = context->m_cpuDescriptorHeap->allocate(1);
    m_srvDesc = srvDesc;
    device->CreateShaderResourceView(m_texture, &srvDesc, m_srvHandle);

    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Format = convertToDXGI(m_desc.format_dsv);
    dsvDesc.Texture2D.MipSlice = 0;

    m_dsvHandle = context->m_dsvDescriptorHeap->allocate(1);
    m_dsvDesc = dsvDesc;
    device->CreateDepthStencilView(m_texture, &dsvDesc, m_dsvHandle);
}

DepthBufferD3D12::~DepthBufferD3D12() {
    SafeRelease(m_texture);
}

uint64_t DepthStencilViewD3D12::getGPUBytesUsed() {
    return 0;
}

Resource *DepthStencilViewD3D12::getResource() {
    return this;
}

DepthStencil *DepthStencilViewD3D12::getDepthStencil() {
    return this;
}

NvFlexDepthBufferDesc DepthStencilViewD3D12::getDepthBufferDesc() {
    NvFlexDepthBufferDesc bufDesc;

    NvFlexFormat resourceFormat = m_ds_format;
    if (m_srvResource) {
        auto texDesc = m_srvResource->GetDesc();
        resourceFormat = convertToNvFlex(texDesc.Format);
    }

    bufDesc.format_resource = resourceFormat;
    bufDesc.format_dsv = m_ds_format;
    bufDesc.format_srv = convertToNvFlex(m_dsvDesc.Format);
    bufDesc.width = m_width;
    bufDesc.height = m_height;

    return bufDesc;
}

DepthStencilViewD3D12::DepthStencilViewD3D12(ContextD3D12 *context,
                                             const NvFlexDepthStencilViewDescD3D12 *desc)
    : Object{context->getDeferredRelease()},
      ResourceD3D12{&m_srvResource, &m_srvResourceState, &m_srvResourceResourceState},
      DepthStencilD3D12{&m_dsvResource, &m_dsvResourceState, &m_dsvRestoreResourceState},
      m_dsvResource{},
      m_dsvResourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE},
      m_dsvRestoreResourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE},
      m_srvResource{},
      m_srvResourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE},
      m_srvResourceResourceState{D3D12_RESOURCE_STATE_DEPTH_WRITE} {
    update(context, desc);
}

DepthStencilViewD3D12::~DepthStencilViewD3D12() {}

void DepthStencilViewD3D12::update(ContextD3D12 *context,
                                   const NvFlexDepthStencilViewDescD3D12 *desc) {
    m_desc = *desc;
    m_dsvResource = m_desc.dsvResource;
    m_dsvResourceState = m_desc.dsvCurrentState;
    m_dsvRestoreResourceState = m_desc.dsvCurrentState;
    m_srvResource = m_desc.srvResource;
    m_srvResourceState = m_desc.srvCurrentState;
    m_srvResourceResourceState = m_desc.srvCurrentState;

    D3D12_RESOURCE_DESC d3dResDesc;
    d3dResDesc = m_srvResource->GetDesc();
    m_ds_format = convertToNvFlex(d3dResDesc.Format);
    m_srvHandle = m_desc.srvHandle;
    m_srvDesc = m_desc.srvDesc;

    m_dsvHandle = m_desc.dsvHandle;
    m_dsvDesc = m_desc.dsvDesc;

    m_width = d3dResDesc.Width;
    m_height = d3dResDesc.Height;

    copyViewport(&m_viewport, &m_desc.viewport);
}

uint64_t RenderTargetViewD3D12::getGPUBytesUsed() {
    return 0;
}

RenderTarget *RenderTargetViewD3D12::getRenderTarget() {
    return this;
}

RenderTargetViewD3D12::RenderTargetViewD3D12(ContextD3D12 *context,
                                             const NvFlexRenderTargetViewDescD3D12 *desc)
    : Object{context->getDeferredRelease()},
      RenderTargetD3D12{&m_resource, &m_resourceState, &m_restoreResourceState},
      m_resource{},
      m_resourceState{D3D12_RESOURCE_STATE_RENDER_TARGET},
      m_restoreResourceState{D3D12_RESOURCE_STATE_RENDER_TARGET} {
    update(context, desc);
}

RenderTargetViewD3D12::~RenderTargetViewD3D12() {}

void RenderTargetViewD3D12::update(ContextD3D12 *context,
                                   const NvFlexRenderTargetViewDescD3D12 *desc) {
    m_desc = *desc;
    m_resource = m_desc.resource;
    m_resourceState = m_desc.currentState;
    m_restoreResourceState = m_desc.currentState;
    m_rt_format = convertToNvFlex(m_desc.rtvDesc.Format);
    m_rtvHandle = m_desc.rtvHandle;
    m_rtvDesc = m_desc.rtvDesc;
    m_scissor = m_desc.scissor;
    copyViewport(&m_viewport, &m_desc.viewport);
}

uint64_t ComputeShaderD3D12::getGPUBytesUsed() {
    return 0;
}

ComputeShaderD3D12::ComputeShaderD3D12(ContextD3D12 *context,
                                       const NvFlexComputeShaderDesc *desc)
    : Object{context->getDeferredRelease()},
      m_labelSizeInBytes{},
      m_cs{} {
    m_desc = *desc;
    if (!m_desc.label)
        m_desc.label = L"unlabeled";

    m_labelSizeInBytes = wcslen(m_desc.label) * sizeof(wchar_t);

    D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.pRootSignature = context->m_rootSignatureCompute;
    psoDesc.CS.pShaderBytecode = m_desc.cs;
    psoDesc.CS.BytecodeLength = m_desc.cs_length;

    context->getDevice()->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&m_cs));
}

ComputeShaderD3D12::~ComputeShaderD3D12() {
    SafeRelease(m_cs);
}

GraphicsShaderD3D12::Version::Version()
    : renderTargetFormat{},
      depthStencilFormat{},
      m_psoLH{},
      m_psoRH{} {}

GraphicsShaderD3D12::Version::~Version() {
    SafeRelease(m_psoLH);
    SafeRelease(m_psoRH);
}

uint64_t GraphicsShaderD3D12::getGPUBytesUsed() {
    return 0;
}

GraphicsShaderD3D12::GraphicsShaderD3D12(ContextD3D12 *context,
                                         const NvFlexGraphicsShaderDesc *desc)
    : Object{context->getDeferredRelease()},
      m_labelSizeInBytes{},
      m_psoDesc{},
      m_inputElementDescs{},
      m_versions{},
      m_versionIdex{} {
    m_desc = *desc;

    m_inputElementDescs.resize(m_desc.numInputElements);
    for (uint32_t i = 0; i < m_desc.numInputElements; ++i)
        m_inputElementDescs[i] = m_desc.inputElementDescs[i];
    m_desc.inputElementDescs = m_inputElementDescs.data();

    if (!m_desc.label)
        m_desc.label = L"unlabeled";

    m_labelSizeInBytes = wcslen(m_desc.label) * sizeof(wchar_t);

    createPSO(context);
}

GraphicsShaderD3D12::~GraphicsShaderD3D12() {
    for (auto &version : m_versions) {
        SafeRelease(version.m_psoLH);
        SafeRelease(version.m_psoRH);
    }
}

void GraphicsShaderD3D12::createPSO(ContextD3D12 *context) {
    ZeroMemory(&m_psoDesc, sizeof(m_psoDesc));

    m_psoDesc.pRootSignature = context->m_rootSignatureGraphics;
    m_psoDesc.VS.pShaderBytecode = m_desc.vs;
    m_psoDesc.VS.BytecodeLength = m_desc.vs_length;
    m_psoDesc.PS.pShaderBytecode = m_desc.ps;
    m_psoDesc.PS.BytecodeLength = m_desc.ps_length;

    VectorCached<D3D12_INPUT_ELEMENT_DESC, 4> elementDesc;
    elementDesc.resize(m_desc.numInputElements);

    uint32_t alignedByteOffset = 0;
    for (uint32_t i = 0; i < m_desc.numInputElements; ++i) {
        auto &e = elementDesc[i];
        auto &input = m_desc.inputElementDescs[i];
        e.SemanticName = input.semanticName;
        e.SemanticIndex = 0;
        e.Format = convertToDXGI(input.format);
        e.InputSlot = 0;
        e.AlignedByteOffset = alignedByteOffset;
        e.InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA;
        e.InstanceDataStepRate = 0;
        alignedByteOffset = -1;
    }

    m_psoDesc.InputLayout.NumElements = m_desc.numInputElements;
    m_psoDesc.InputLayout.pInputElementDescs = elementDesc.data();

    m_psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC{D3D12_DEFAULT};
    if (m_desc.uavTarget)
        m_psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    m_psoDesc.RasterizerState.DepthClipEnable = m_desc.depthClipEnable;

    D3D12_BLEND_DESC blendDesc = {};
    blendDesc.RenderTarget[0].BlendEnable = m_desc.blendState.enable;
    blendDesc.RenderTarget[0].SrcBlend = convertToD3D12(m_desc.blendState.srcBlendColor);
    blendDesc.RenderTarget[0].DestBlend = convertToD3D12(m_desc.blendState.dstBlendColor);
    blendDesc.RenderTarget[0].BlendOp = convertToD3D12(m_desc.blendState.blendOpColor);
    blendDesc.RenderTarget[0].SrcBlendAlpha =
        convertToD3D12(m_desc.blendState.srcBlendAlpha);
    blendDesc.RenderTarget[0].DestBlendAlpha =
        convertToD3D12(m_desc.blendState.dstBlendAlpha);
    blendDesc.RenderTarget[0].BlendOpAlpha = convertToD3D12(m_desc.blendState.blendOpAlpha);
    blendDesc.RenderTarget[0].RenderTargetWriteMask = 0xF;
    m_psoDesc.BlendState = blendDesc;

    D3D12_DEPTH_STENCIL_DESC depthDesc = {};
    depthDesc.DepthEnable = m_desc.depthState.depthEnable;
    depthDesc.DepthWriteMask = m_desc.depthState.depthWriteMask == eNvFlexDepthWriteMask_All
                                   ? D3D12_DEPTH_WRITE_MASK_ALL
                                   : D3D12_DEPTH_WRITE_MASK_ZERO;
    depthDesc.DepthFunc = convertToD3D12(m_desc.depthState.depthFunc);
    depthDesc.StencilEnable = FALSE;
    m_psoDesc.DepthStencilState = depthDesc;

    for (uint32_t i = 0; i < m_desc.numRenderTargets; ++i) {
        m_psoDesc.RTVFormats[i] = convertToDXGI(m_desc.renderTargetFormat[i]);
    }

    m_psoDesc.DSVFormat = convertToDXGI(m_desc.depthStencilFormat);
    m_psoDesc.SampleMask = D3D12_DEFAULT_SAMPLE_MASK;
    if (m_desc.lineList)
        m_psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    else
        m_psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;

    m_psoDesc.NumRenderTargets = m_desc.uavTarget ? 0 : m_desc.numRenderTargets;

    m_psoDesc.SampleDesc.Count = 1;
    m_psoDesc.SampleDesc.Quality = 0;

    uint32_t Back = m_versions.allocateBack();
    auto &version = m_versions[Back];
    version.renderTargetFormat = m_desc.renderTargetFormat[0];
    version.depthStencilFormat = m_desc.depthStencilFormat;

    auto device = context->getDevice();
    device->CreateGraphicsPipelineState(&m_psoDesc, IID_PPV_ARGS(&version.m_psoLH));

    m_psoDesc.RasterizerState.FrontCounterClockwise = 1;
    device->CreateGraphicsPipelineState(&m_psoDesc, IID_PPV_ARGS(&version.m_psoRH));
}

ID3D12PipelineState *GraphicsShaderD3D12::getPSO(bool frontCounterClockwise) {
    auto &version = m_versions[m_versionIdex];
    return frontCounterClockwise ? version.m_psoRH : version.m_psoLH;
}

void GraphicsShaderD3D12::setFormats(ContextD3D12 *context, NvFlexFormat renderTargetFormat,
                                     NvFlexFormat depthStencilFormat) {
    auto curVersion = &m_versions[m_versionIdex];
    if (curVersion->renderTargetFormat != renderTargetFormat ||
        curVersion->depthStencilFormat != depthStencilFormat) {
        for (m_versionIdex = 0; m_versionIdex != m_versions.size(); ++m_versionIdex) {
            curVersion = &m_versions[m_versionIdex];
            if (curVersion->renderTargetFormat == renderTargetFormat &&
                curVersion->depthStencilFormat == depthStencilFormat)
                return;
        }

        m_desc.renderTargetFormat[0] = renderTargetFormat;
        m_desc.depthStencilFormat = depthStencilFormat;
        createPSO(context);
    }
}

uint64_t TimerD3D12::getGPUBytesUsed() {
    return 0;
}

TimerD3D12::TimerD3D12(ContextD3D12 *context)
    : Object{context->getDeferredRelease()},
      m_cpuFreq{},
      m_cpuBegin{},
      m_cpuEnd{},
      m_queryHeap{},
      m_queryReadback{},
      m_queryFrequency{},
      m_queryReadbackFenceVal{-1ull},
      m_state{0} {
    auto device = context->getDevice();
    D3D12_QUERY_HEAP_DESC queryDesc = {};
    queryDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
    queryDesc.Count = 2;
    queryDesc.NodeMask = 0;
    device->CreateQueryHeap(&queryDesc, IID_PPV_ARGS(&m_queryHeap));

    auto resDesc = CD3DX12_RESOURCE_DESC::Buffer(2 * sizeof(uint64_t));
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_READBACK}, D3D12_HEAP_FLAG_NONE, &resDesc,
        D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_queryReadback));
}

TimerD3D12::~TimerD3D12() {
    SafeRelease(m_queryHeap);
    SafeRelease(m_queryReadback);
}

uint64_t EventQueueD3D12::getGPUBytesUsed() {
    return 0;
}

EventQueueD3D12::EventQueueD3D12(ContextD3D12 *context)
    : Object{context->getDeferredRelease()},
      m_pushID{},
      m_events{} {}

EventQueueD3D12::~EventQueueD3D12() {}

EventQueueD3D12::Event *EventQueueD3D12::getNewEvent(ContextD3D12 *context) {
    for (auto &ev : m_events) {
        if (ev.state == eEventStateInactive)
            return &ev;
    }
    auto idx = m_events.allocateBack();
    auto &ev = m_events[idx];
    ev.fenceID = 0;
    return &ev;
}

NvFlexResult EventQueueD3D12::pop(uint64_t *pUid, uint64_t lastFenceCompleted) {
    bool valid = 0;
    uint32_t minIdx = 0;
    uint64_t minPushID = 0;
    for (uint32_t i = 0; i < m_events.size(); ++i) {
        auto &ev = m_events[i];
        if (ev.state == eEventStateActive) {
            if (valid) {
                if (ev.pushID < minPushID) {
                    minIdx = i;
                    minPushID = ev.pushID;
                } else {
                    minIdx = i;
                    minPushID = ev.pushID;
                    valid = 1;
                }
            }
        }
    }

    if (!valid)
        return eNvFlexFail;

    auto &ev = m_events[minIdx];
    if (ev.fenceID > lastFenceCompleted)
        return eNvFlexFail;

    if (pUid)
        *pUid = ev.uid;
    ev.state = eEventStateInactive;
    return eNvFlexSuccess;
}

void EventQueueD3D12::push(ContextD3D12 *context, uint64_t uid, uint64_t nextFenceValue) {
    auto newEvent = getNewEvent(context);
    newEvent->state = eEventStateActive;
    newEvent->uid = uid;
    newEvent->pushID = ++m_pushID;
    newEvent->fenceID = nextFenceValue;
}

uint64_t FenceD3D12::getGPUBytesUsed() {
    return 0;
}

FenceD3D12::FenceD3D12(ContextD3D12 *context, FenceD3D12 *fence)
    : Object{context->getDeferredRelease()},
      m_fence{},
      m_device{} {
    m_desc = fence->m_desc;
    m_device = context->getDevice();
    m_device->AddRef();

    HANDLE sharedHandle;
    fence->m_device->CreateSharedHandle(fence->m_fence, nullptr, GENERIC_ALL, nullptr,
                                        &sharedHandle);
    m_device->OpenSharedHandle(sharedHandle, IID_PPV_ARGS(&m_fence));
    CloseHandle(sharedHandle);
}

FenceD3D12::FenceD3D12(ContextD3D12 *context, const NvFlexFenceDesc *desc)
    : Object{context->getDeferredRelease()},
      m_fence{},
      m_device{} {
    m_desc = *desc;
    m_device = context->getDevice();
    m_device->AddRef();

    D3D12_FENCE_FLAGS fenceFlags = D3D12_FENCE_FLAG_NONE;
    if (m_desc.crossAdapterShared)
        fenceFlags = D3D12_FENCE_FLAG_SHARED | D3D12_FENCE_FLAG_SHARED_CROSS_ADAPTER;

    m_device->CreateFence(0, fenceFlags, IID_PPV_ARGS(&m_fence));
}

FenceD3D12::~FenceD3D12() {
    SafeRelease(m_fence);
    SafeRelease(m_device);
}

void FenceD3D12::signalFence(ContextD3D12 *context, uint64_t fenceValue) {
    context->m_commandQueue->Signal(m_fence, fenceValue);
}

void FenceD3D12::waitOnFence(ContextD3D12 *context, uint64_t fenceValue) {
    context->m_commandQueue->Wait(m_fence, fenceValue);
}

uint64_t DescriptorHeapD3D12::getGPUBytesUsed() {
    return 0;
}

DescriptorHeapD3D12::DescriptorHeapD3D12(ContextD3D12 *context,
                                         D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                                         uint32_t minHeapSize)
    : Object{context->getDeferredRelease()},
      m_heaps{minHeapSize},
      m_descriptorSize{},
      m_heapType{heapType},
      m_device{context->getDevice()} {
    m_descriptorSize = m_device->GetDescriptorHandleIncrementSize(heapType);
}

D3D12_CPU_DESCRIPTOR_HANDLE DescriptorHeapD3D12::allocate(uint32_t numDescriptors) {
    auto heap = m_heaps.allocate(numDescriptors);
    if (!heap->heapData.m_heap) {
        D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
        heapDesc.NumDescriptors = heap->capacity;
        heapDesc.Type = m_heapType;
        heapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        m_device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&heap->heapData.m_heap));
    }

    D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle =
        heap->heapData.m_heap->GetCPUDescriptorHandleForHeapStart();
    cpuHandle.ptr += m_descriptorSize * heap->start;
    return cpuHandle;
}

DescriptorHeapD3D12 *DescriptorHeapD3D12::create(ContextD3D12 *context,
                                                 D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                                                 uint32_t minHeapSize) {
    return new DescriptorHeapD3D12(context, heapType, minHeapSize);
}

uint64_t DynamicDescriptorHeapD3D12::getGPUBytesUsed() {
    return 0;
}

DynamicDescriptorHeapD3D12::DynamicDescriptorHeapD3D12(ContextD3D12 *context,
                                                       D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                                                       uint32_t minHeapSize)
    : Object{context->getDeferredRelease()},
      m_heaps{minHeapSize},
      m_descriptorSize{},
      m_heapType{heapType},
      m_device{} {
    m_device = context->getDevice();
    m_descriptorSize = m_device->GetDescriptorHandleIncrementSize(heapType);
}

DynamicDescriptorHeapD3D12::Handles DynamicDescriptorHeapD3D12::allocate(
    uint32_t numDescriptors, uint64_t lastFenceCompleted, uint64_t nextFenceValue) {
    auto heap = m_heaps.allocate(numDescriptors, lastFenceCompleted, nextFenceValue);
    if (!heap->heapData.m_heap) {
        D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
        heapDesc.Type = m_heapType;
        heapDesc.NumDescriptors = heap->capacity;
        heapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        m_device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&heap->heapData.m_heap));
    }

    Handles handles;
    handles.cpuHandle = heap->heapData.m_heap->GetCPUDescriptorHandleForHeapStart();
    handles.cpuHandle.ptr += m_descriptorSize * heap->start;
    handles.gpuHandle = heap->heapData.m_heap->GetGPUDescriptorHandleForHeapStart();
    handles.gpuHandle.ptr += m_descriptorSize * heap->start;
    return handles;
}

DynamicDescriptorHeapD3D12 *DynamicDescriptorHeapD3D12::create(
    ContextD3D12 *context, D3D12_DESCRIPTOR_HEAP_TYPE heapType, uint32_t minHeapSize) {
    return new DynamicDescriptorHeapD3D12(context, heapType, minHeapSize);
}

NvFlexDynamicDescriptorHeapD3D12 DynamicDescriptorHeapD3D12::getInterface() {
    NvFlexDynamicDescriptorHeapD3D12 result;
    result.userdata = this;
    result.reserveDescriptors = DynamicDescriptorHeapD3D12::resverseDescriptors;
    return result;
}

NvFlexDescriptorReserveHandleD3D12 DynamicDescriptorHeapD3D12::resverseDescriptors(
    void *userdata, uint32_t numDescriptors, uint64_t lastFenceCompleted,
    uint64_t nextFenceValue) {
    auto heap = (DynamicDescriptorHeapD3D12 *)userdata;
    Handles srcHandles = heap->allocate(numDescriptors, lastFenceCompleted, nextFenceValue);
    NvFlexDescriptorReserveHandleD3D12 handle;
    handle.descriptorSize = heap->m_descriptorSize;
    handle.heap = heap->m_heaps.front()->heapData.m_heap;
    handle.cpuHandle = srcHandles.cpuHandle;
    handle.gpuHandle = srcHandles.gpuHandle;
    return handle;
}

uint32_t ContextD3D12::addRef() {
    return Object::addRef();
}

uint32_t ContextD3D12::release() {
    return Object::release();
}

uint64_t ContextD3D12::getGPUBytesUsed() {
    return Object::getGPUBytesUsed();
}

void ContextD3D12::processFenceSignal(NvFlexContext *context) {
    for (auto &fenceEvent : m_signalFenceEvents) {
        auto fenceD3D12 = implCast<FenceD3D12>(fenceEvent.fence);
        m_commandQueue->Signal(fenceD3D12->m_fence, fenceEvent.fenceValue);
    }
    m_signalFenceEvents.clear();
}

void ContextD3D12::processFenceWait(NvFlexContext *context) {
    for (auto &fenceEvent : m_waitFenceEvents) {
        auto fenceD3D12 = implCast<FenceD3D12>(fenceEvent.fence);
        m_commandQueue->Wait(fenceD3D12->m_fence, fenceEvent.fenceValue);
    }
    m_waitFenceEvents.clear();
}

void ContextD3D12::contextPush() {}

void ContextD3D12::contextPop() {}

NvFlexContextAPI ContextD3D12::getContextType() {
    return eNvFlexContextD3D12;
}

ConstantBuffer *ContextD3D12::createConstantBuffer(const NvFlexConstantBufferDesc *desc) {
    return new ConstantBufferD3D12(this, desc);
}

NvFlexMappedData ContextD3D12::map(Texture3D *buffer) {
    auto texture = implCast<Texture3DD3D12>(buffer);
    NvFlexMappedData mappedData = {};

    if (texture->m_desc.uploadAccess) {
        auto bufferData =
            texture->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
        if (!bufferData->m_buffer) {
            uint64_t size = texture->m_uploadHeapSize;
            m_device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Buffer(texture->m_uploadHeapSize),
                D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
                IID_PPV_ARGS(&bufferData->m_buffer));

            D3D12_RANGE readRange = {};
            void *pdata = 0;
            bufferData->m_buffer->Map(0, &readRange, &pdata);
            bufferData->m_mappedData = pdata;
        }

        mappedData.data = bufferData->m_mappedData;
        mappedData.rowPitch = texture->m_footPrint.Footprint.RowPitch;
        mappedData.depthPitch = mappedData.rowPitch * texture->m_footPrint.Footprint.Height;
    }

    return mappedData;
}

void *ContextD3D12::map(Buffer *bufferIn, NvFlexStagingCpuAccessFlags cpuAccessMode,
                        bool cpuNoWait) {
    auto buffer = implCast<BufferD3D12>(bufferIn);

    if(!verifyMapAccess(buffer->m_desc.memType, buffer->m_desc.cpuAccessMode, &cpuAccessMode))
        return nullptr;

    void *pdata = nullptr;
    if(buffer->m_desc.memType == eNvFlexMemoryType_upload) {
        // Version it
        auto bufferData =
            buffer->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
        if (!bufferData->m_buffer) {
            UINT elementSizeInByte = buffer->m_desc.structStride
                                         ? buffer->m_desc.structStride
                                         : getFormatSizeInBytes(buffer->m_desc.format);
            UINT64 size = elementSizeInByte * buffer->m_desc.dim;
            m_device->CreateCommittedResource(
                &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
                &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr, IID_PPV_ARGS(&bufferData->m_buffer));

            HRESULT hr;
            D3D12_RANGE readRange = {};
            H_VERIFY(bufferData->m_buffer->Map(0, &readRange, &pdata));
            bufferData->m_mappedData = pdata;
        }
    } else {
        D3D12_RANGE readRange = {};
        buffer->m_buffer->Map(0, &readRange, &pdata);
    }

    return pdata;
}

void ContextD3D12::unmap(Buffer *bufferIn) {
    auto buffer = implCast<BufferD3D12>(bufferIn);

    if (!verifyMapAccess(buffer->m_desc.memType, buffer->m_desc.cpuAccessMode, nullptr))
        return;

    if (buffer->m_desc.memType == eNvFlexMemoryType_upload) {
        // Version it
        buffer->m_uploadBuffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
        transitionResourceBarrier(m_commandList, *buffer->ref_resource,
                                  buffer->ref_resourceState,
                                  D3D12_RESOURCE_STATE_COPY_DEST);
        auto bufferData = buffer->m_uploadBuffers.front();
        m_commandList->CopyResource(buffer->m_buffer, bufferData->m_buffer);
    } else
        buffer->m_buffer->Unmap(0, nullptr);
}

void *ContextD3D12::map(ConstantBuffer *bufferIn) {
    auto buffer = implCast<ConstantBufferD3D12>(bufferIn);
    auto bufferData = buffer->m_buffers.map(m_lastFenceCompleted, m_nextFenceValue);
    if (!bufferData->m_buffer) {
        UINT64 size = buffer->m_desc.sizeInBytes;
        m_device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&bufferData->m_buffer));
        D3D12_RANGE readRange = {};
        void *pdata = 0;
        bufferData->m_buffer->Map(0, &readRange, &pdata);
        bufferData->m_mappedData = pdata;
    }
    return bufferData->m_mappedData;
}

void *ContextD3D12::map(IndexBuffer *bufferIn) {
    auto buffer = implCast<IndexBufferD3D12>(bufferIn);
    auto bufferData = buffer->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
    if (!bufferData->m_buffer) {
        UINT64 size = buffer->m_desc.sizeInBytes;
        m_device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&bufferData->m_buffer));
        D3D12_RANGE readRange = {};
        void *pdata = 0;
        bufferData->m_buffer->Map(0, &readRange, &pdata);
        bufferData->m_mappedData = pdata;
    }
    return bufferData->m_mappedData;
}

void *ContextD3D12::map(Texture1D *bufferIn) {
    auto texture = implCast<Texture1DD3D12>(bufferIn);
    if (!texture->m_desc.uploadAccess)
        return nullptr;

    auto bufferData = texture->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
    if (!bufferData->m_buffer) {
        UINT64 size = texture->m_uploadHeapSize;
        m_device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&bufferData->m_buffer));
        D3D12_RANGE readRange = {};
        void *pdata = 0;
        bufferData->m_buffer->Map(0, &readRange, &pdata);
        bufferData->m_mappedData = pdata;
    }
    return bufferData->m_mappedData;
}

void *ContextD3D12::map(VertexBuffer *bufferIn) {
    auto buffer = implCast<VertexBufferD3D12>(bufferIn);
    auto bufferData = buffer->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
    if (!bufferData->m_buffer) {
        UINT64 size = buffer->m_desc.sizeInBytes;
        m_device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&bufferData->m_buffer));
        D3D12_RANGE readRange = {};
        void *pdata = 0;
        bufferData->m_buffer->Map(0, &readRange, &pdata);
        bufferData->m_mappedData = pdata;
    }
    return bufferData->m_mappedData;
}

void ContextD3D12::unmap(ConstantBuffer *bufferIn) {
    auto buffer = implCast<ConstantBufferD3D12>(bufferIn);
    buffer->m_buffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
}

void ContextD3D12::unmap(IndexBuffer *bufferIn) {
    auto buffer = implCast<IndexBufferD3D12>(bufferIn);
    buffer->m_uploadBuffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
    D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_INDEX_BUFFER;
    transitionResourceBarrier(m_commandList, buffer->m_buffer, &state,
                              D3D12_RESOURCE_STATE_COPY_DEST);
    auto bufferData = buffer->m_uploadBuffers.front();
    m_commandList->CopyResource(buffer->m_buffer, bufferData->m_buffer);
    transitionResourceBarrier(m_commandList, buffer->m_buffer, &state,
                              D3D12_RESOURCE_STATE_INDEX_BUFFER);
}

void ContextD3D12::unmap(VertexBuffer *bufferIn) {
    auto buffer = implCast<VertexBufferD3D12>(bufferIn);
    buffer->m_uploadBuffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
    D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;
    transitionResourceBarrier(m_commandList, buffer->m_buffer, &state,
                              D3D12_RESOURCE_STATE_COPY_DEST);
    auto bufferData = buffer->m_uploadBuffers.front();
    m_commandList->CopyResource(buffer->m_buffer, bufferData->m_buffer);
    transitionResourceBarrier(m_commandList, buffer->m_buffer, &state,
                              D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
}

void ContextD3D12::unmap(Texture1D *textureIn) {
    auto texture = implCast<Texture1DD3D12>(textureIn);
    if (texture->m_desc.uploadAccess) {
        texture->m_uploadBuffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
        transitionResourceBarrier(m_commandList, *texture->ref_resource,
                                  texture->ref_resourceState,
                                  D3D12_RESOURCE_STATE_COPY_DEST);
        auto bufferData = texture->m_uploadBuffers.front();
        D3D12_TEXTURE_COPY_LOCATION dstCopy = {};
        dstCopy.pResource = texture->m_texture;
        dstCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dstCopy.SubresourceIndex = 0;
        D3D12_TEXTURE_COPY_LOCATION srcCopy = {};
        srcCopy.pResource = bufferData->m_buffer;
        srcCopy.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        srcCopy.PlacedFootprint = texture->m_footPrint;
        m_commandList->CopyTextureRegion(&dstCopy, 0, 0, 0, &srcCopy, nullptr);
    }
}

void ContextD3D12::unmap(Texture3D *textureIn) {
    auto texture = implCast<Texture3DD3D12>(textureIn);
    if (texture->m_desc.uploadAccess) {
        texture->m_uploadBuffers.unmap(m_lastFenceCompleted, m_nextFenceValue);
        transitionResourceBarrier(m_commandList, *texture->ref_resource,
                                  texture->ref_resourceState,
                                  D3D12_RESOURCE_STATE_COPY_DEST);
        auto bufferData = texture->m_uploadBuffers.front();
        D3D12_TEXTURE_COPY_LOCATION dstCopy = {};
        dstCopy.pResource = texture->m_texture;
        dstCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dstCopy.SubresourceIndex = 0;
        D3D12_TEXTURE_COPY_LOCATION srcCopy = {};
        srcCopy.pResource = bufferData->m_buffer;
        srcCopy.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        srcCopy.PlacedFootprint = texture->m_footPrint;
        m_commandList->CopyTextureRegion(&dstCopy, 0, 0, 0, &srcCopy, nullptr);
    }
}

void ContextD3D12::upload(Buffer *bufferIn, NvFlexUint offset, const void *data,
                          NvFlexUint numBytes) {
    auto buffer = implCast<BufferD3D12>(bufferIn);
    if (buffer->m_desc.memType != eNvFlexMemoryType_default) {
        NVFLEX_ASSERT(0 && "upload to a dynamic buffer or staging buffer is not allowed");
        return;
    }

    auto bufferData = buffer->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
    void *pdata = bufferData->m_mappedData;
    if (!bufferData->m_buffer) {
        UINT64 formatSizeInBytes = buffer->m_desc.structStride
                          ? buffer->m_desc.structStride
                          : getFormatSizeInBytes(buffer->m_desc.format);
        UINT64 sizeInBytes = formatSizeInBytes * buffer->m_desc.dim;
        m_device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(sizeInBytes), D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr, IID_PPV_ARGS(&bufferData->m_buffer));
        D3D12_RANGE readRange = {};
        bufferData->m_buffer->Map(0, &readRange, &pdata);
        bufferData->m_mappedData = pdata;
    }

    memcpy((uint8_t *)pdata + offset, data, numBytes);

    transitionResourceBarrier(m_commandList, *buffer->ref_resource,
                              buffer->ref_resourceState, D3D12_RESOURCE_STATE_COPY_DEST);

    m_commandList->CopyBufferRegion(buffer->m_buffer, offset, bufferData->m_buffer, offset,
                                    numBytes);

    restoreResourceState(buffer);
}

EventQueue *ContextD3D12::createEventQueue() {
    return new EventQueueD3D12(this);
}

void ContextD3D12::eventQueuePush(EventQueue *eventQueueIn, uint64_t uid) {
    auto eventQueue = implCast<EventQueueD3D12>(eventQueueIn);
    eventQueue->push(this, uid, m_nextFenceValue);
}

NvFlexResult ContextD3D12::eventQueuePop(EventQueue *eventQueueIn, uint64_t *pUid) {
    auto eventQueue = implCast<EventQueueD3D12>(eventQueueIn);
    return eventQueue->pop(pUid, m_lastFenceCompleted);
}

NvFlexResult ContextD3D12::registerD3DBuffer(void *nativeBuffer, NvFlexUint dim,
                                             NvFlexUint structStride, Buffer **ppBuffer) {
    auto bufD3D12 =
        new BufferD3D12(this, (ID3D12Resource *)nativeBuffer, dim, structStride);
    if(ppBuffer) {
        *ppBuffer = bufD3D12;
        bufD3D12->addRef();
    }
    bufD3D12->release();
    return eNvFlexSuccess;
}

DeferredRelease *ContextD3D12::getDeferredRelease() {
    return m_deferredRelease;
}

ID3D12Device *ContextD3D12::getDevice() {
    return m_device;
}

ID3D12CommandQueue *ContextD3D12::getCommandQueue() {
    return m_commandQueue;
}

ID3D12CommandList *ContextD3D12::getCommandList() {
    return m_commandList;
}

ContextD3D12::ContextD3D12(const NvFlexContextDescD3D12 *desc): ContextD3D12{nullptr, desc} {}

ContextD3D12::ContextD3D12(Device *deviceImpl, const NvFlexContextDescD3D12 *desc)
    : Context{deviceImpl},
      m_d3d12module{},
      m_dxgimodule{},
      m_device{},
      m_commandQueue{},
      m_commandList{},
      m_lastFenceCompleted{},
      m_nextFenceValue{},
      m_gpuDescriptorHeap{},
      m_rootSignatureGraphics{},
      m_rootSignatureCompute{},
      m_VTRSupportChecked{},
      m_VTRSupported{},
      m_cpuDescriptorHeap{},
      m_gpuDescriptorHeapImpl{},
      m_rtvDescriptorHeap{},
      m_dsvDescriptorHeap{},
      m_nullSRV{},
      m_nullUAV{},
      m_tileCoords{},
      m_tileRegionSize{},
      m_rangeFlags{},
      m_tilePoolCoords{},
      m_tilePoolRangeSize{},
      m_deferredRelease{},
      m_waitFenceEvents{},
      m_signalFenceEvents{},
      m_waitFenceEventsVersion{},
      m_signalFenceEventsVersion{} {
    m_deferredRelease = createDeferredRelease(desc);

    m_d3d12module = LoadLibraryW(L"d3d12.dll");
    m_dxgimodule = LoadLibraryW(L"DXGI.dll");

    updateContext(desc);

    m_cpuDescriptorHeap =
        DescriptorHeapD3D12::create(this, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, 1024);
    m_gpuDescriptorHeapImpl = DynamicDescriptorHeapD3D12::create(
        this, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, 4096);

    updateContext(desc);

    m_rtvDescriptorHeap =
        DescriptorHeapD3D12::create(this, D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 16);
    m_dsvDescriptorHeap =
        DescriptorHeapD3D12::create(this, D3D12_DESCRIPTOR_HEAP_TYPE_DSV, 16);

    {
        auto serializeRootSignature = (HRESULT(*)(
            const D3D12_ROOT_SIGNATURE_DESC *, D3D_ROOT_SIGNATURE_VERSION, ID3D10Blob **,
            ID3DBlob **))GetProcAddress(m_d3d12module, "D3D12SerializeRootSignature");
        auto createRootSignature =
            [serializeRootSignature,
             this](const D3D12_ROOT_SIGNATURE_DESC *desc) -> ID3D12RootSignature * {
            ID3DBlob *signature = 0, *error = 0;
            HRESULT hr;
            if (FAILED(hr = serializeRootSignature(desc, D3D_ROOT_SIGNATURE_VERSION_1,
                                                   &signature, &error)))
                return nullptr;

            ID3D12RootSignature *rootSignature;
            hr = m_device->CreateRootSignature(0, signature->GetBufferPointer(),
                                               signature->GetBufferSize(),
                                               IID_PPV_ARGS(&rootSignature));
            SafeRelease(signature);
            SafeRelease(error);
            return rootSignature;
        };

        CD3DX12_STATIC_SAMPLER_DESC staticSamplers[6] = {
            // clang-format off
            {
                0,
                D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            },
            {
                1,
                D3D12_FILTER_MIN_MAG_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                D3D12_TEXTURE_ADDRESS_MODE_BORDER,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            },
            {
                2,
                D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            },
            {
                3,
                D3D12_FILTER_MIN_MAG_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                D3D12_TEXTURE_ADDRESS_MODE_WRAP,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            },
            {
                4,
                D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            },
            {
                5,
                D3D12_FILTER_MIN_MAG_MIP_POINT,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                0.f,
                0,
                D3D12_COMPARISON_FUNC_NEVER,
                D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
                0.f,
                D3D12_FLOAT32_MAX,
                D3D12_SHADER_VISIBILITY_ALL,
                0
            }  // clang-format on
        };

        {
            CD3DX12_ROOT_PARAMETER rootParameters[5];

            rootParameters[0].InitAsConstantBufferView(0, 0,
                                                       D3D12_SHADER_VISIBILITY_VERTEX);
            rootParameters[1].InitAsConstantBufferView(0, 0, D3D12_SHADER_VISIBILITY_PIXEL);

            CD3DX12_DESCRIPTOR_RANGE ranges[3];
            ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, NV_FLEX_DRAW_MAX_READ_TEXTURES, 0);
            ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, NV_FLEX_DRAW_MAX_READ_TEXTURES, 0);
            ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
            rootParameters[2].InitAsDescriptorTable(1, &ranges[0],
                                                    D3D12_SHADER_VISIBILITY_VERTEX);
            rootParameters[3].InitAsDescriptorTable(1, &ranges[1],
                                                    D3D12_SHADER_VISIBILITY_PIXEL);
            rootParameters[4].InitAsDescriptorTable(1, &ranges[2],
                                                    D3D12_SHADER_VISIBILITY_PIXEL);

            D3D12_ROOT_SIGNATURE_DESC rootSigDesc = {};
            rootSigDesc.NumParameters = countof(rootParameters);
            rootSigDesc.pParameters = rootParameters;
            rootSigDesc.NumStaticSamplers = countof(staticSamplers);
            rootSigDesc.pStaticSamplers = staticSamplers;
            rootSigDesc.Flags =
                D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

            m_rootSignatureGraphics = createRootSignature(&rootSigDesc);
        }

        {
            CD3DX12_ROOT_PARAMETER rootParameters[4];

            rootParameters[0].InitAsConstantBufferView(0, 0, D3D12_SHADER_VISIBILITY_ALL);

            CD3DX12_DESCRIPTOR_RANGE ranges[2];
            ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, NV_FLEX_DISPATCH_MAX_READ_TEXTURES, 0);
            ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES, 0);
            rootParameters[1].InitAsDescriptorTable(1, &ranges[0],
                                                    D3D12_SHADER_VISIBILITY_ALL);
            rootParameters[2].InitAsDescriptorTable(1, &ranges[1],
                                                    D3D12_SHADER_VISIBILITY_ALL);

            rootParameters[3].InitAsConstantBufferView(1, 0, D3D12_SHADER_VISIBILITY_ALL);

            D3D12_ROOT_SIGNATURE_DESC rootSigDesc = {};
            rootSigDesc.NumParameters = countof(rootParameters);
            rootSigDesc.pParameters = rootParameters;
            rootSigDesc.NumStaticSamplers = countof(staticSamplers);
            rootSigDesc.pStaticSamplers = staticSamplers;
            rootSigDesc.Flags =
                D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

            m_rootSignatureCompute = createRootSignature(&rootSigDesc);
        }

        {
            D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
            uavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
            uavDesc.Format = DXGI_FORMAT_R32_UINT;
            uavDesc.Buffer.FirstElement = 0;
            uavDesc.Buffer.NumElements = 256;

            m_nullUAV = m_cpuDescriptorHeap->allocate(1);
            m_device->CreateUnorderedAccessView(nullptr, nullptr, &uavDesc, m_nullUAV);
        }

        {
            D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
            srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
            srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            srvDesc.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
            srvDesc.Texture2D.MipLevels = 1;
            srvDesc.Texture2D.MostDetailedMip = 0;
            m_nullSRV = m_cpuDescriptorHeap->allocate(1);
            m_device->CreateShaderResourceView(nullptr, &srvDesc, m_nullSRV);
        }
    }
}

ContextD3D12::~ContextD3D12() {
    SafeRelease(m_cpuDescriptorHeap);
    SafeRelease(m_gpuDescriptorHeapImpl);
    SafeRelease(m_rtvDescriptorHeap);
    SafeRelease(m_dsvDescriptorHeap);
    m_deferredRelease->contextReleaseNotify();
    SafeRelease(m_deferredRelease);
    SafeRelease(m_rootSignatureGraphics);
    SafeRelease(m_rootSignatureCompute);

    for (auto &fenceEvent : m_signalFenceEvents)
        SafeRelease(fenceEvent.fence);

    for (auto &fenceEvent : m_waitFenceEvents)
        SafeRelease(fenceEvent.fence);

    FreeLibrary(m_d3d12module);
    FreeLibrary(m_dxgimodule);
}

void ContextD3D12::updateContext(const NvFlexContextDescD3D12 *desc) {
    m_device = desc->device;
    m_commandQueue = desc->commandQueue;
    m_commandQueueFence = desc->commandQueueFence;
    m_commandList = desc->commandList;
    m_lastFenceCompleted = desc->lastFenceCompleted;
    m_nextFenceValue = desc->nextFenceValue;
    if (desc->dynamicHeapCbvSrvUav.reserveDescriptors) {
        m_gpuDescriptorHeap = desc->dynamicHeapCbvSrvUav;
    } else
        m_gpuDescriptorHeap = m_gpuDescriptorHeapImpl->getInterface();

    m_deferredRelease->update(desc);
}

void ContextD3D12::updateContextDesc(NvFlexContextDescD3D12 *desc) {
    desc->device = m_device;
    desc->commandQueue = m_commandQueue;
    desc->commandQueueFence = m_commandQueueFence;
    desc->commandList = m_commandList;
    desc->lastFenceCompleted = m_lastFenceCompleted;
    desc->nextFenceValue = m_nextFenceValue;
    desc->dynamicHeapCbvSrvUav = m_gpuDescriptorHeap;
}

NvFlexDepthStencilView *ContextD3D12::createDepthStencilView(
    const NvFlexDepthStencilViewDescD3D12 *desc) {
    return new DepthStencilViewD3D12(this, desc);
}

NvFlexRenderTargetView *ContextD3D12::createRenderTargetView(
    const NvFlexRenderTargetViewDescD3D12 *desc) {
    return new RenderTargetViewD3D12(this, desc);
}

void ContextD3D12::updateDepthStencilView(NvFlexDepthStencilView *view,
                                          const NvFlexDepthStencilViewDescD3D12 *desc) {
    auto dsv = implCast<DepthStencilViewD3D12>(view);
    dsv->update(this, desc);
}

void ContextD3D12::updateRenderTargetView(NvFlexRenderTargetView *view,
                                          const NvFlexRenderTargetViewDescD3D12 *desc) {
    auto rtv = implCast<RenderTargetViewD3D12>(view);
    rtv->update(this, desc);
}

void ContextD3D12::updateResoruceRWViewDesc(ResourceRWD3D12 *resourceRW,
                                            NvFlexResourceRWViewDescD3D12 *desc) {
    auto descD3D12 = (*resourceRW->ref_resource)->GetDesc();
    desc->resourceView.srvHandle = resourceRW->m_srvHandle;
    desc->resourceView.srvDesc = resourceRW->m_srvDesc;
    desc->uavHandle = resourceRW->m_uavHandle;
    desc->uavDesc = resourceRW->m_uavDesc;
    desc->resourceView.resource = *resourceRW->ref_resource;
    desc->resourceView.currentState = resourceRW->ref_resourceState;
}

void ContextD3D12::updateResourceViewDesc(ResourceD3D12 *resource,
                                          NvFlexResourceViewDescD3D12 *desc) {
    auto descD3D12 = (*resource->ref_resource)->GetDesc();
    desc->srvHandle = resource->m_srvHandle;
    desc->srvDesc = resource->m_srvDesc;
    desc->resource = *resource->ref_resource;
    desc->currentState = resource->ref_resourceState;
}

void ContextD3D12::setIndexBuffer(IndexBuffer *buffer, uint32_t offset) {
    auto indexBuffer = implCast<IndexBufferD3D12>(buffer);
    D3D12_INDEX_BUFFER_VIEW ibv;
    ibv.BufferLocation = indexBuffer->m_buffer->GetGPUVirtualAddress() + offset;
    ibv.Format = indexBuffer->m_format;
    ibv.SizeInBytes = indexBuffer->m_desc.sizeInBytes - offset;
    m_commandList->IASetIndexBuffer(&ibv);
}

void ContextD3D12::drawIndexedInstanced(uint32_t indicesPerInstance, uint32_t numInstances,
                                        const NvFlexDrawParams *params) {
    auto graphicsShader = implCast<GraphicsShaderD3D12>(params->shader);
    profileItemBegin(graphicsShader->m_desc.label);

    const int maxSlots = 16;
    const int maxWriteSlots = 1;

    m_commandList->SetGraphicsRootSignature(m_rootSignatureGraphics);

    auto PSO = graphicsShader->getPSO(params->frontCounterClockwise);

    m_commandList->SetPipelineState(PSO);

    if (graphicsShader->m_desc.lineList)
        m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    else
        m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    auto handles = m_gpuDescriptorHeap.reserveDescriptors(
        m_gpuDescriptorHeap.userdata, 2 * maxSlots + maxWriteSlots, m_lastFenceCompleted,
        m_nextFenceValue);

    m_commandList->SetDescriptorHeaps(1, &handles.heap);

    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D12>(params->vs_readOnly[i]);
        D3D12_CPU_DESCRIPTOR_HANDLE srcHandle = r ? r->m_srvHandle : m_nullSRV;
        m_device->CopyDescriptorsSimple(1, handles.cpuHandle, srcHandle,
                                        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        handles.cpuHandle.ptr += handles.descriptorSize;
    }

    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D12>(params->ps_readOnly[i]);
        D3D12_CPU_DESCRIPTOR_HANDLE srcHandle = r ? r->m_srvHandle : m_nullSRV;
        m_device->CopyDescriptorsSimple(1, handles.cpuHandle, srcHandle,
                                        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        handles.cpuHandle.ptr += handles.descriptorSize;
    }

    for (int i = 0; i < maxWriteSlots; ++i) {
        auto r = implCast<ResourceRWD3D12>(params->ps_readWrite[i]);
        D3D12_CPU_DESCRIPTOR_HANDLE srcHandle = r ? r->m_uavHandle : m_nullUAV;
        m_device->CopyDescriptorsSimple(1, handles.cpuHandle, srcHandle,
                                        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        handles.cpuHandle.ptr += handles.descriptorSize;
    }

    m_commandList->SetGraphicsRootDescriptorTable(2, handles.gpuHandle);
    handles.gpuHandle.ptr += maxSlots * handles.descriptorSize;
    m_commandList->SetGraphicsRootDescriptorTable(3, handles.gpuHandle);
    handles.gpuHandle.ptr += maxSlots * handles.descriptorSize;
    m_commandList->SetGraphicsRootDescriptorTable(4, handles.gpuHandle);

    if (params->rootConstantBuffer) {
        auto cb = implCast<ConstantBufferD3D12>(params->rootConstantBuffer)->getFront();
        auto cbv = cb->GetGPUVirtualAddress();
        m_commandList->SetGraphicsRootConstantBufferView(0, cbv);
        m_commandList->SetGraphicsRootConstantBufferView(1, cbv);
    }

    D3D12_RESOURCE_BARRIER barriers[2 * maxSlots + maxWriteSlots];
    uint32_t barrierIdx = 0;

    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D12>(params->vs_readOnly[i]);
        if (r && (*r->ref_resourceState & D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) ==
                     D3D12_RESOURCE_STATE_COMMON) {
            barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], *r->ref_resource,
                                      r->ref_resourceState,
                                      D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }
    }

    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D12>(params->ps_readOnly[i]);
        if (r && (*r->ref_resourceState & D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) ==
                     D3D12_RESOURCE_STATE_COMMON) {
            barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], *r->ref_resource,
                                      r->ref_resourceState,
                                      D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        }
    }

    for (int i = 0; i < maxWriteSlots; ++i) {
        auto r = implCast<ResourceRWD3D12>(params->ps_readWrite[i]);
        if (r) {
            barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], *r->ref_resource,
                                      r->ref_resourceState,
                                      D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        }
    }

    if (barrierIdx)
        m_commandList->ResourceBarrier(barrierIdx, barriers);

    m_commandList->BeginEvent(D3D12_FEATURE_ARCHITECTURE, graphicsShader->m_desc.label,
                              graphicsShader->m_labelSizeInBytes);
    m_commandList->DrawIndexedInstanced(indicesPerInstance, numInstances, 0, 0, 0);
    m_commandList->EndEvent();

    profileItemEnd();
}

void ContextD3D12::setRenderTarget(RenderTarget *rtv, DepthStencil *dsv) {
    auto rtvD3D12 = implCast<RenderTargetD3D12>(rtv);
    auto dsvD3D12 = implCast<DepthStencilD3D12>(dsv);
    if (rtvD3D12)
        transitionResourceBarrier(m_commandList, *rtvD3D12->ref_resource,
                                  rtvD3D12->ref_resourceState,
                                  D3D12_RESOURCE_STATE_RENDER_TARGET);
    if (dsvD3D12)
        transitionResourceBarrier(m_commandList, *dsvD3D12->ref_resource,
                                  dsvD3D12->ref_resourceState,
                                  D3D12_RESOURCE_STATE_DEPTH_WRITE);

    const D3D12_CPU_DESCRIPTOR_HANDLE *rtvHandle =
        rtvD3D12 ? &rtvD3D12->m_rtvHandle : nullptr;
    const D3D12_CPU_DESCRIPTOR_HANDLE *dsvHandle =
        dsvD3D12 ? &dsvD3D12->m_dsvHandle : nullptr;
    m_commandList->OMSetRenderTargets(1, rtvHandle, FALSE, dsvHandle);
    if (rtvD3D12) {
        m_commandList->RSSetViewports(1, (const D3D12_VIEWPORT *)&rtvD3D12->m_viewport);
        m_commandList->RSSetScissorRects(1, &rtvD3D12->m_scissor);
    } else if (dsvD3D12) {
        m_commandList->RSSetViewports(1, (const D3D12_VIEWPORT *)&dsvD3D12->m_viewport);
        D3D12_RECT scissor = {};
        scissor.left = 0;
        scissor.right = 0;
        scissor.right = dsvD3D12->m_width;
        scissor.bottom = dsvD3D12->m_height;
        m_commandList->RSSetScissorRects(1, &scissor);
    }
}

void ContextD3D12::setViewport(const NvFlexViewport *vp) {
    D3D12_VIEWPORT viewport = *(const D3D12_VIEWPORT *)vp;
    D3D12_RECT rect;
    rect.left = 0;
    rect.top = 0;
    rect.right = viewport.Width;
    rect.bottom = viewport.Height;
    m_commandList->RSSetViewports(1, &viewport);
    m_commandList->RSSetScissorRects(1, &rect);
}

void ContextD3D12::clearRenderTarget(RenderTarget *rtv, const float color[4]) {
    auto rtvd3d12 = implCast<RenderTargetD3D12>(rtv);
    transitionResourceBarrier(m_commandList, *rtvd3d12->ref_resource,
                              rtvd3d12->ref_resourceState,
                              D3D12_RESOURCE_STATE_RENDER_TARGET);
    m_commandList->ClearRenderTargetView(rtvd3d12->m_rtvHandle, color, 0, nullptr);
}

void ContextD3D12::clearDepthStencil(DepthStencil *dsv, float depth) {
    auto dsvd3d12 = implSafeCast<DepthStencilD3D12>(dsv);
    transitionResourceBarrier(m_commandList, *dsvd3d12->ref_resource,
                              dsvd3d12->ref_resourceState,
                              D3D12_RESOURCE_STATE_DEPTH_WRITE);
    m_commandList->ClearDepthStencilView(dsvd3d12->m_dsvHandle, D3D12_CLEAR_FLAG_DEPTH,
                                         depth, 0, 0, nullptr);
}

void ContextD3D12::restoreResourceState(Resource *resource) {
    auto resourced3d12 = implCast<ResourceD3D12>(resource);
    transitionResourceBarrier(m_commandList, *resourced3d12->ref_resource,
                              resourced3d12->ref_resourceState,
                              *resourced3d12->ref_restoreResourceState);
}

void ContextD3D12::clearState() {
    m_commandList->ClearState(nullptr);
}

int ContextD3D12::is_VTR_supported() {
    if (!m_VTRSupportChecked) {
        D3D12_FEATURE_DATA_D3D12_OPTIONS options;
        m_VTRSupported = SUCCEEDED(m_device->CheckFeatureSupport(
                             D3D12_FEATURE_D3D12_OPTIONS, &options, sizeof(options))) &&
                         options.TiledResourcesTier >= D3D12_TILED_RESOURCES_TIER_3;
        if (m_VTRSupported) {
            auto texDesc = CD3DX12_RESOURCE_DESC::Tex3D(DXGI_FORMAT_R16G16B16A16_FLOAT,
                                                        1024, 1024, 1024);
            texDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            texDesc.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
            if (FAILED(m_device->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_DEFAULT}, D3D12_HEAP_FLAG_NONE,
                    &texDesc, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, nullptr,
                    IID_ID3D12Resource, nullptr)))
                m_VTRSupported = 0;
        }
        m_VTRSupportChecked = 1;
    }

    return m_VTRSupported;
}

void ContextD3D12::updateVTRMapping(Texture3DVTR *textureIn, HeapVTR *heapIn,
                                    uint32_t *blockTableImage, uint32_t rowPitch,
                                    uint32_t depthPitch) {
    auto texture = implCast<Texture3DVTRD3D12>(textureIn);
    auto heap = implCast<HeapVTRD3D12>(heapIn);

    auto gridDim = texture->m_gridDim;
    uint32_t maxBlocks = gridDim.z * gridDim.y * gridDim.x;
    m_tileCoords.resize(maxBlocks);
    m_tileRegionSize.resize(maxBlocks);
    m_rangeFlags.resize(maxBlocks);
    m_tilePoolCoords.resize(maxBlocks);
    m_tilePoolRangeSize.resize(maxBlocks);

    uint32_t tileID;
    uint32_t index = 0;
    auto &currentImage = texture->m_blockTable;
    auto blockTableDim = currentImage.dim();
    for (uint32_t k = 0; k < blockTableDim.z; ++k)
        for (uint32_t j = 0; j < blockTableDim.y; ++j)
            for (uint32_t i = 0; i < blockTableDim.x; ++i) {
                auto newVal =
                    blockTableImage[i + (rowPitch * j + depthPitch * k) / sizeof(uint32_t)];
                auto &oldVal = currentImage(i, j, k);
                if (newVal != oldVal) {
                    tileID = ~newVal;
                    D3D12_TILED_RESOURCE_COORDINATE coords;
                    coords.X = i;
                    coords.Y = j;
                    coords.Z = k;
                    coords.Subresource = 0;
                    m_tileCoords[index] = coords;
                    D3D12_TILE_REGION_SIZE regionSize;
                    regionSize.NumTiles = 1;
                    regionSize.UseBox = FALSE;
                    regionSize.Width = 1;
                    regionSize.Height = 1;
                    regionSize.Depth = 1;
                    m_tileRegionSize[index] = regionSize;

                    m_rangeFlags[index] = newVal ? D3D12_TILE_RANGE_FLAG_REUSE_SINGLE_TILE
                                                 : D3D12_TILE_RANGE_FLAG_NULL;

                    m_tilePoolCoords[index] = tileID;
                    m_tilePoolRangeSize[index] = 1;
                    oldVal = newVal;
                    ++index;
                }
            }

    if (index) {
        m_commandQueue->UpdateTileMappings(
            texture->m_texture, index, m_tileCoords.data(), m_tileRegionSize.data(),
            heap->m_heap, index, m_rangeFlags.data(), m_tilePoolCoords.data(),
            m_tilePoolRangeSize.data(), D3D12_TILE_MAPPING_FLAG_NO_HAZARD);
    }
}

void ContextD3D12::copy(Buffer *dst, uint32_t dstOffset, Buffer *src, uint32_t srcOffset,
                        uint32_t numBytes) {
    if(!numBytes)
        return;

    auto dstBufD3D12 = implCast<BufferD3D12>(dst);
    auto srcBufD3D12 = implCast<BufferD3D12>(src);

    if (srcBufD3D12->m_desc.memType == eNvFlexMemoryType_readback) {
        HRESULT hr;
        uint8_t *sptr, *dptr;
        D3D12_RANGE readRange = {srcOffset, srcOffset + numBytes};
        H_VERIFY(srcBufD3D12->m_buffer->Map(0, &readRange, (void **)&sptr));
        NVFLEX_ASSERT(sptr);
        if (sptr) {
            auto bufferData =
                dstBufD3D12->m_uploadBuffers.map(m_lastFenceCompleted, m_nextFenceValue);
            if (!bufferData->m_buffer) {
                UINT elementSizeInByte =
                    dstBufD3D12->m_desc.structStride
                        ? dstBufD3D12->m_desc.structStride
                        : getFormatSizeInBytes(dstBufD3D12->m_desc.format);
                UINT64 size = elementSizeInByte * dstBufD3D12->m_desc.dim;
                m_device->CreateCommittedResource(
                    &CD3DX12_HEAP_PROPERTIES{D3D12_HEAP_TYPE_UPLOAD}, D3D12_HEAP_FLAG_NONE,
                    &CD3DX12_RESOURCE_DESC::Buffer(size), D3D12_RESOURCE_STATE_GENERIC_READ,
                    nullptr, IID_PPV_ARGS(&bufferData->m_buffer));

                D3D12_RANGE writeRange = {dstOffset, dstOffset + numBytes};
                H_VERIFY(bufferData->m_buffer->Map(0, &writeRange, (void **)&dptr));
                NVFLEX_ASSERT(dptr);
                memcpy(dptr, sptr, numBytes);
                bufferData->m_buffer->Unmap(0, &writeRange);

                srcBufD3D12->m_buffer->Unmap(0, nullptr);

                transitionResourceBarrier(m_commandList, dstBufD3D12->m_buffer,
                                          dstBufD3D12->ref_resourceState,
                                          D3D12_RESOURCE_STATE_COPY_DEST);
                m_commandList->CopyBufferRegion(dstBufD3D12->m_buffer, dstOffset,
                                                bufferData->m_buffer, dstOffset, numBytes);

                restoreResourceState(dstBufD3D12);
            }
        }
    } else {
        D3D12_RESOURCE_BARRIER barriers[2];
        int barrierIdx = 0;

        barrierIdx += transitionResourceBarrier(
            &barriers[barrierIdx], srcBufD3D12->m_buffer, srcBufD3D12->ref_resourceState,
            D3D12_RESOURCE_STATE_COPY_SOURCE);
        barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], dstBufD3D12->m_buffer,
                                  dstBufD3D12->ref_resourceState,
                                  D3D12_RESOURCE_STATE_COPY_DEST);
        if(barrierIdx)
            m_commandList->ResourceBarrier(barrierIdx, barriers);

        m_commandList->CopyBufferRegion(dstBufD3D12->m_buffer, dstOffset,
                                        srcBufD3D12->m_buffer, srcOffset, numBytes);

        restoreResourceState(dstBufD3D12);
    }
}

void ContextD3D12::copy(Buffer *dst, Resource *src, uint32_t offset, uint32_t numBytes) {
    auto dstBufD3D12 = implCast<BufferD3D12>(dst);
    auto srcBufD3D12 = implCast<ResourceD3D12>(src);

    transitionResourceBarrier(m_commandList, dstBufD3D12->m_buffer,
                              dstBufD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *srcBufD3D12->ref_resource,
                              srcBufD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);
    if (numBytes)
        m_commandList->CopyBufferRegion(dstBufD3D12->m_buffer, offset,
                                        *srcBufD3D12->ref_resource, offset, numBytes);
}

void ContextD3D12::copy(ConstantBuffer *dst, Buffer *src) {
    auto dstBufD3D12 = implCast<ConstantBufferD3D12>(dst);
    auto srcBufD3D12 = implCast<BufferD3D12>(src);

    D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;
    transitionResourceBarrier(m_commandList, dstBufD3D12->m_bufferGPU, &state,
                              D3D12_RESOURCE_STATE_COPY_DEST);

    transitionResourceBarrier(m_commandList, srcBufD3D12->m_buffer,
                              srcBufD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);

    m_commandList->CopyResource(dstBufD3D12->m_bufferGPU, *srcBufD3D12->ref_resource);

    transitionResourceBarrier(m_commandList, dstBufD3D12->m_bufferGPU, &state,
                              D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
}

void ContextD3D12::copy(DepthStencil *dst, Resource *src) {
    auto dstD3D12 = implCast<DepthStencilD3D12>(dst);
    auto srcD3D12 = implCast<ResourceD3D12>(src);

    transitionResourceBarrier(m_commandList, *dstD3D12->ref_resource,
                              dstD3D12->ref_resourceState, D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *srcD3D12->ref_resource,
                              srcD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);
    m_commandList->CopyResource(*dstD3D12->ref_resource, *srcD3D12->ref_resource);
}

void ContextD3D12::copy(ResourceRW *dst, Resource *src) {
    auto dstD3D12 = implCast<ResourceRWD3D12>(dst);
    auto srcD3D12 = implCast<ResourceD3D12>(src);

    transitionResourceBarrier(m_commandList, *dstD3D12->ref_resource,
                              dstD3D12->ref_resourceState, D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *srcD3D12->ref_resource,
                              srcD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);
    m_commandList->CopyResource(*dstD3D12->ref_resource, *srcD3D12->ref_resource);
}

void ContextD3D12::copy(Texture3D *dst, Texture3D *src) {
    auto dstD3D12 = implCast<Texture3DD3D12>(dst);
    auto srcD3D12 = implCast<Texture3DD3D12>(src);

    transitionResourceBarrier(m_commandList, *dstD3D12->ref_resource,
                              dstD3D12->ref_resourceState, D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *srcD3D12->ref_resource,
                              srcD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);
    m_commandList->CopyResource(*dstD3D12->ref_resource, *srcD3D12->ref_resource);
}

void ContextD3D12::copy(Texture3D *dst, Resource *src) {
    auto dstD3D12 = implCast<Texture3DD3D12>(dst);
    auto srcD3D12 = implCast<ResourceD3D12>(src);

    transitionResourceBarrier(m_commandList, *dstD3D12->ref_resource,
                              dstD3D12->ref_resourceState, D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *srcD3D12->ref_resource,
                              srcD3D12->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);
    m_commandList->CopyResource(dstD3D12->m_texture, *srcD3D12->ref_resource);
}

VertexBuffer *ContextD3D12::createVertexBuffer(const NvFlexVertexBufferDesc *desc) {
    return new VertexBufferD3D12(this, desc);
}

IndexBuffer *ContextD3D12::createIndexBuffer(const NvFlexIndexBufferDesc *desc) {
    return new IndexBufferD3D12(this, desc);
}

Buffer *ContextD3D12::createBuffer(const NvFlexBufferDesc *desc) {
    return new BufferD3D12(this, desc);
}

Buffer *ContextD3D12::createBufferView(Buffer *buffer, const NvFlexBufferViewDesc *desc) {
    return new BufferD3D12(this, implCast<BufferD3D12>(buffer), desc);
}

void ContextD3D12::download(Texture3D *buffer) {
    auto texture = implCast<Texture3DD3D12>(buffer);
    if (texture->m_desc.downloadAccess) {
        transitionResourceBarrier(m_commandList, *texture->ref_resource,
                                  texture->ref_resourceState,
                                  D3D12_RESOURCE_STATE_COPY_SOURCE);

        D3D12_TEXTURE_COPY_LOCATION srcCopy = {};
        srcCopy.pResource = texture->m_texture;
        srcCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        srcCopy.SubresourceIndex = 0;
        D3D12_TEXTURE_COPY_LOCATION dstCopy = {};
        dstCopy.pResource = texture->m_downloadBuffer;
        dstCopy.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        dstCopy.PlacedFootprint = texture->m_footPrint;

        m_commandList->CopyTextureRegion(&dstCopy, 0, 0, 0, &srcCopy, nullptr);
        texture->m_downloadCompleteFence = m_nextFenceValue;
    }
}

NvFlexMappedData ContextD3D12::mapDownload(Texture3D *bufferIn) {
    auto texture = implCast<Texture3DD3D12>(bufferIn);
    NvFlexMappedData result = {};
    if (texture->m_desc.downloadAccess) {
        if (texture->m_downloadCompleteFence <= m_lastFenceCompleted) {
            D3D12_RANGE readRange = {};
            readRange.End = texture->m_uploadHeapSize;
            void *data = 0;
            texture->m_downloadBuffer->Map(0, &readRange, &data);
            result.data = data;
            result.rowPitch = texture->m_footPrint.Footprint.RowPitch;
            result.depthPitch = result.rowPitch * texture->m_footPrint.Footprint.Height;
        }
    }
    return result;
}

void ContextD3D12::unmapDownload(Texture3D *textureIn) {
    D3D12_RANGE writeRange = {};
    auto texture = implCast<Texture3DD3D12>(textureIn);
    texture->m_downloadBuffer->Unmap(0, &writeRange);
}

void ContextD3D12::copyFromShared(Texture2D *dstTexture,
                                  Texture2DCrossAdapter *sharedTexture, uint32_t height) {
    auto dst = implCast<Texture2DD3D12>(dstTexture);
    auto src = implCast<Texture2DCrossAdapterD3D12>(sharedTexture);

    transitionResourceBarrier(m_commandList, *dst->ref_resource, dst->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *src->ref_resource, src->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);

    D3D12_TEXTURE_COPY_LOCATION srcCopy = {};
    srcCopy.pResource = *src->ref_resource;
    srcCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    srcCopy.SubresourceIndex = 0;

    D3D12_TEXTURE_COPY_LOCATION dstCopy = {};
    dstCopy.pResource = *dst->ref_resource;
    dstCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dstCopy.SubresourceIndex = 0;

    D3D12_BOX box = {};
    box.left = 0;
    box.right = src->m_desc.width;
    box.top = 0;
    box.bottom = height;
    box.front = 0;
    box.back = 1;
    if (height)
        m_commandList->CopyTextureRegion(&dstCopy, 0, 0, 0, &srcCopy, &box);
}

void ContextD3D12::copyToShared(Texture2DCrossAdapter *dstSharedTexture,
                                Texture2D *srcTexture, uint32_t height) {
    auto src = implSafeCast<Texture2DD3D12>(srcTexture);
    auto dst = implCast<Texture2DCrossAdapterD3D12>(dstSharedTexture);

    transitionResourceBarrier(m_commandList, *dst->ref_resource, dst->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_DEST);
    transitionResourceBarrier(m_commandList, *src->ref_resource, src->ref_resourceState,
                              D3D12_RESOURCE_STATE_COPY_SOURCE);

    D3D12_TEXTURE_COPY_LOCATION srcCopy = {};
    srcCopy.pResource = *src->ref_resource;
    srcCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    srcCopy.SubresourceIndex = 0;

    D3D12_TEXTURE_COPY_LOCATION dstCopy = {};
    dstCopy.pResource = *dst->ref_resource;
    dstCopy.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dstCopy.SubresourceIndex = 0;

    D3D12_BOX box = {};
    box.left = 0;
    box.right = src->m_desc.width;
    box.top = 0;
    box.bottom = height;
    box.front = 0;
    box.back = 1;

    m_commandList->CopyTextureRegion(&dstCopy, 0, 0, 0, &srcCopy, &box);
}

Fence *ContextD3D12::createFence(const NvFlexFenceDesc *desc) {
    return new FenceD3D12(this, desc);
}

Fence *ContextD3D12::shareFence(Fence *fence) {
    return new FenceD3D12(this, implCast<FenceD3D12>(fence));
}

void ContextD3D12::waitOnFence(Fence *fence, uint64_t fenceValue) {
    if (m_waitFenceEventsVersion != m_nextFenceValue) {
        m_waitFenceEventsVersion = m_nextFenceValue;
        m_waitFenceEvents.clear();
    }
    uint32_t allocIdx = m_waitFenceEvents.allocateBack();
    auto &fenceEvent = m_waitFenceEvents[allocIdx];
    fenceEvent.fence = fence;
    fenceEvent.fenceValue = fenceValue;
}

HeapVTR *ContextD3D12::createHeapVTR(const NvFlexHeapSparseDesc *desc) {
    return new HeapVTRD3D12(this, desc);
}

Texture3DVTR *ContextD3D12::createTexture3DVTR(const NvFlexTexture3DSparseDesc *desc) {
    return new Texture3DVTRD3D12(this, desc);
}

ColorBuffer *ContextD3D12::createColorBuffer(const NvFlexColorBufferDesc *desc) {
    return new ColorBufferD3D12(this, desc);
}

DepthBuffer *ContextD3D12::createDepthBuffer(const NvFlexDepthBufferDesc *desc) {
    return new DepthBufferD3D12(this, desc);
}

ComputeShader *ContextD3D12::createComputeShader(const NvFlexComputeShaderDesc *desc) {
    return new ComputeShaderD3D12(this, desc);
}

GraphicsShader *ContextD3D12::createGraphicsShader(const NvFlexGraphicsShaderDesc *desc) {
    return new GraphicsShaderD3D12(this, desc);
}

void ContextD3D12::setFormats(GraphicsShader *graphicsShader,
                              NvFlexFormat renderTargetFormat,
                              NvFlexFormat depthStencilFormat) {
    auto graphicsShaderD3D12 = implCast<GraphicsShaderD3D12>(graphicsShader);
    graphicsShaderD3D12->setFormats(this, renderTargetFormat, depthStencilFormat);
}

Texture2DCrossAdapter *ContextD3D12::createTexture2DCrossAdapter(
    const NvFlexTexture2DDesc *desc) {
    return new Texture2DCrossAdapterD3D12(this, desc);
}

Texture2DCrossAdapter *ContextD3D12::shareTexture2DCrossAdapter(
    Texture2DCrossAdapter *sharedTexture) {
    return new Texture2DCrossAdapterD3D12(
        this, implCast<Texture2DCrossAdapterD3D12>(sharedTexture));
}

void ContextD3D12::transitionToCommonState(Resource *resource) {
    auto resourceD3D12 = implCast<ResourceD3D12>(resource);
    transitionResourceBarrier(m_commandList, *resourceD3D12->ref_resource,
                              resourceD3D12->ref_resourceState,
                              *resourceD3D12->ref_restoreResourceState);
}

Texture1D *ContextD3D12::createTexture1D(const NvFlexTexture1DDesc *desc) {
    return new Texture1DD3D12(this, desc);
}

Texture2D *ContextD3D12::createTexture2D(const NvFlexTexture2DDesc *desc) {
    return new Texture2DD3D12(this, desc, false);
}

Texture2D *ContextD3D12::shareTexture2D(Texture2D *sharedTexture) {
    return new Texture2DD3D12(this, sharedTexture, false);
}

Texture2D *ContextD3D12::shareTexture2DShared(Texture2D *sharedTexture) {
    return new Texture2DD3D12(this, sharedTexture, true);
}

Texture2D *ContextD3D12::createTexture2DShared(const NvFlexTexture2DDesc *desc) {
    return new Texture2DD3D12(this, desc, true);
}

Texture3D *ContextD3D12::createTexture3D(const NvFlexTexture3DDesc *desc) {
    return new Texture3DD3D12(this, desc);
}

ResourceReference *ContextD3D12::shareResourceReference(Resource *resource) {
    return new ResourceReferenceD3D12(this, implCast<ResourceD3D12>(resource));
}

Timer *ContextD3D12::createTimer() {
    return new TimerD3D12(this);
}

void ContextD3D12::timerBegin(Timer *timer) {
    auto timerD3D12 = implCast<TimerD3D12>(timer);
    if (!timerD3D12->m_state) {
        QueryPerformanceFrequency(&timerD3D12->m_cpuFreq);
        QueryPerformanceCounter(&timerD3D12->m_cpuBegin);
        m_commandQueue->GetTimestampFrequency(&timerD3D12->m_queryFrequency);
        m_commandList->EndQuery(timerD3D12->m_queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, 0);
        timerD3D12->m_state = 1;
    }
}

void ContextD3D12::timerEnd(Timer *timer) {
    auto timerD3D12 = implCast<TimerD3D12>(timer);
    if (timerD3D12->m_state == 1) {
        QueryPerformanceCounter(&timerD3D12->m_cpuEnd);
        m_commandList->EndQuery(timerD3D12->m_queryHeap, D3D12_QUERY_TYPE_TIMESTAMP, 1);
        m_commandList->ResolveQueryData(timerD3D12->m_queryHeap, D3D12_QUERY_TYPE_TIMESTAMP,
                                        0, 2, timerD3D12->m_queryReadback, 0);
        timerD3D12->m_queryReadbackFenceVal = m_nextFenceValue;
        timerD3D12->m_state = 2;
    }
}

NvFlexResult ContextD3D12::timerGetResult(Timer *timer, float *timeGPU, float *timeCPU,
                                          NvFlexUint64 *gpuStartStamp,
                                          NvFlexUint64 *gpuEndStamp,
                                          NvFlexUint64 *gpuFreq) {
    auto timerD3D12 = implCast<TimerD3D12>(timer);
    if (timerD3D12->m_state != 2 ||
        timerD3D12->m_queryReadbackFenceVal > m_lastFenceCompleted)
        return eNvFlexFail;

    UINT64 startVal = 0;
    UINT64 endVal = 0;
    D3D12_RANGE readRange = {};
    readRange.End = 2 * sizeof(UINT64);
    UINT64 *data;
    timerD3D12->m_queryReadback->Map(0, &readRange, (void **)&data);
    if (data) {
        startVal = data[0];
        endVal = data[1];
        D3D12_RANGE writeRange = {};
        timerD3D12->m_queryReadback->Unmap(0, &writeRange);
    }

    INT64 diff = endVal - startVal;
    if (diff < 0)
        diff = 0;

    float msGPU = double(diff) / timerD3D12->m_queryFrequency;
    float msCPU = double(timerD3D12->m_cpuEnd.QuadPart - timerD3D12->m_cpuBegin.QuadPart) /
                  timerD3D12->m_cpuFreq.QuadPart;

    if (msGPU)
        *timeGPU = msGPU;
    if (msCPU)
        *timeCPU = msCPU;

    if(gpuStartStamp)
        *gpuStartStamp = startVal;
    if(gpuEndStamp)
        *gpuEndStamp = endVal;
    if(gpuFreq)
        *gpuFreq = timerD3D12->m_queryFrequency;

    timerD3D12->m_state = 0;
    return eNvFlexSuccess;
}

void ContextD3D12::signalFence(Fence *fence, uint64_t fenceValue) {
    if (m_signalFenceEventsVersion != m_nextFenceValue) {
        m_signalFenceEventsVersion = m_nextFenceValue;
        m_signalFenceEvents.clear();
    }

    uint32_t allocIdx = m_signalFenceEvents.allocateBack();
    auto &fenceEvent = m_signalFenceEvents[allocIdx];
    fenceEvent.fence = fence;
    fenceEvent.fenceValue = fenceValue;
}

void ContextD3D12::dispatch(const NvFlexDispatchParams *params) {
    auto computeShader = implCast<ComputeShaderD3D12>(params->shader);

    profileItemBegin(computeShader->m_desc.label);
    if (params->gridDim.x && params->gridDim.y && params->gridDim.z) {
        constexpr int maxReadSlot = NV_FLEX_DISPATCH_MAX_READ_TEXTURES;
        constexpr int maxWriteSlot = NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES;

        m_commandList->SetComputeRootSignature(m_rootSignatureCompute);
        m_commandList->SetPipelineState(computeShader->m_cs);

        auto handles = m_gpuDescriptorHeap.reserveDescriptors(
            m_gpuDescriptorHeap.userdata, maxReadSlot + maxWriteSlot, m_lastFenceCompleted, m_nextFenceValue);

        m_commandList->SetDescriptorHeaps(1, &handles.heap);

        for (int i = 0; i < maxReadSlot; ++i) {
            auto r = implCast<ResourceD3D12>(params->readOnly[i]);
            D3D12_CPU_DESCRIPTOR_HANDLE srcHandle;
            if (r)
                srcHandle = r->m_srvHandle;
            else
                srcHandle = m_nullSRV;

            m_device->CopyDescriptorsSimple(1, handles.cpuHandle, srcHandle,
                                            D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

            handles.cpuHandle.ptr += handles.descriptorSize;
        }

        for (int i = 0; i < maxWriteSlot; ++i) {
            auto r = implCast<ResourceRWD3D12>(params->readWrite[i]);
            D3D12_CPU_DESCRIPTOR_HANDLE srcHandle;
            if (r)
                srcHandle = r->m_uavHandle;
            else
                srcHandle = m_nullUAV;

            m_device->CopyDescriptorsSimple(1, handles.cpuHandle, srcHandle,
                                            D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

            handles.cpuHandle.ptr += handles.descriptorSize;
        }

        m_commandList->SetComputeRootDescriptorTable(1, handles.gpuHandle);
        handles.gpuHandle.ptr += maxReadSlot * handles.descriptorSize;
        m_commandList->SetComputeRootDescriptorTable(2, handles.gpuHandle);

        if (params->rootConstantBuffer) {
            auto buffer = implCast<ConstantBufferD3D12>(params->rootConstantBuffer);
            auto cb = buffer->getFront();
            m_commandList->SetComputeRootConstantBufferView(0, cb->GetGPUVirtualAddress());
        }
        if (params->secondConstantBuffer) {
            auto buffer = implCast<ConstantBufferD3D12>(params->secondConstantBuffer);
            auto cb = buffer->getFront();
            m_commandList->SetComputeRootConstantBufferView(3, cb->GetGPUVirtualAddress());
        }

        D3D12_RESOURCE_BARRIER barriers[maxReadSlot + maxWriteSlot];
        int barrierIdx = 0;

        for (int i = 0; i < maxReadSlot; ++i) {
            auto r = implCast<ResourceD3D12>(params->readOnly[i]);
            if (r &&
                (*r->ref_resourceState & D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) ==
                    D3D12_RESOURCE_STATE_COMMON) {
                barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], *r->ref_resource,
                                          r->ref_resourceState,
                                          D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            }
        }

        for (int i = 0; i < maxWriteSlot; ++i) {
            auto r = implCast<ResourceRWD3D12>(params->readWrite[i]);
            if (r) {
                barrierIdx += transitionResourceBarrier(&barriers[barrierIdx], *r->ref_resource,
                                          r->ref_resourceState,
                                          D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            }
        }

        if (barrierIdx)
            m_commandList->ResourceBarrier(barrierIdx, barriers);

        m_commandList->BeginEvent(D3D12_FEATURE_ARCHITECTURE, computeShader->m_desc.label,
                                  computeShader->m_labelSizeInBytes);
        m_commandList->Dispatch(params->gridDim.x, params->gridDim.y, params->gridDim.z);
        m_commandList->EndEvent();
    }

    profileItemEnd();


}

void ContextD3D12::setVertexBuffer(VertexBuffer *buffer, uint32_t stride, uint32_t offset) {
    auto vb = implCast<VertexBufferD3D12>(buffer);
    D3D12_VERTEX_BUFFER_VIEW vbv;
    vbv.BufferLocation = vb->m_buffer->GetGPUVirtualAddress() + offset;
    vbv.StrideInBytes = stride;
    vbv.SizeInBytes = vb->m_desc.sizeInBytes - offset;
    m_commandList->IASetVertexBuffers(0, 1, &vbv);
}

}  // namespace NvFlex
