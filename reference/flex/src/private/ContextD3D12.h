#ifndef CONTEXTD3D12_H
#define CONTEXTD3D12_H
#include "Context.h"
#include "VectorCached.h"
#include "ClientHelper.h"
#include "Image3D.h"
#include <d3d12.h>
#include <nvflex/NvFlexContextD3D12.h>

namespace NvFlex {

int64_t FlexDeferredReleaseD3D12(float timeoutMS);

// MARK: VersionedBuffer
template <typename T>
class VersionedBuffer {
 public:
    VersionedBuffer()
        : m_buffers{},
          m_frontIdx{},
          m_mappedIdx{},
          currentFrame{} {}
    VersionedBuffer(const VersionedBuffer &) = delete;
    VersionedBuffer &operator=(const VersionedBuffer &) = delete;

    T *front() { return &m_buffers[m_frontIdx].bufferData; }

    T *map(uint64_t lastFenceCompleted, uint64_t nextFenceValue) {
        uint32_t index;
        for (index = m_frontIdx + 1;
             index < m_buffers.size() &&
             m_buffers[index].releaseFenceValue > lastFenceCompleted;
             ++index)
            ;

        if (index == m_buffers.size()) {
            for (index = 0; index < m_frontIdx &&
                            m_buffers[index].releaseFenceValue > lastFenceCompleted;
                 ++index)
                ;
        }

        if (index == m_frontIdx || m_buffers.empty())
            index = m_buffers.allocateBack();

        m_mappedIdx = index;
        return &m_buffers[index].bufferData;
    }

    void unmap(uint64_t lastFenceCompleted, uint64_t nextFenceValue) {
        if (m_frontIdx != m_mappedIdx) {
            m_buffers[m_frontIdx].releaseFenceValue = nextFenceValue;
        }

        m_frontIdx = m_mappedIdx;
    }

 private:
    struct Buffer {
        Buffer()
            : releaseFenceValue{~0ull},
              bufferData{} {}

        uint64_t releaseFenceValue;
        T bufferData;
    };

    VectorCached<Buffer, 16> m_buffers;
    uint32_t m_frontIdx;
    uint32_t m_mappedIdx;
    uint64_t currentFrame;
};

// MARK: HeapAllocator
template <typename T>
struct HeapAllocator {
    struct Heap {
        Heap()
            : start{},
              allocated{},
              capacity{},
              heapData{} {}

        uint32_t start;
        uint32_t allocated;
        uint32_t capacity;
        T heapData;
    };

    HeapAllocator(uint32_t minHeapSize)
        : m_heaps{},
          m_minHeapSize{minHeapSize},
          m_frontIdx{} {}

    Heap *allocate(uint32_t numElements) {
        if (m_heaps.empty())
            goto L_ALLOCATE_ON_NEW_PAGE;

        auto &heap = m_heaps[m_frontIdx];
        if (!heap.capacity) {
            heap.capacity = m_minHeapSize;
            while (heap.capacity < numElements)
                heap.capacity *= 2;
        }

        auto newCapacity = heap.allocated + numElements;
        if (newCapacity <= heap.capacity) {
            heap.start = heap.allocated;
            heap.allocated += numElements;
            return &heap;
        } else {
        L_ALLOCATE_ON_NEW_PAGE:
            m_frontIdx = m_heaps.allocateBack();
            return allocate(numElements);
        }
    }

 private:
    VectorCached<Heap, 16> m_heaps;
    uint32_t m_minHeapSize;
    uint32_t m_frontIdx;
};

// MARK: DynamicHeapAllocator
template <typename T>
struct DynamicHeapAllocator {
    struct Heap {
        Heap()
            : start{},
              allocated{},
              capacity{},
              releaseFenceValue{-1ull},
              heapData{} {}

        uint32_t start;
        uint32_t allocated;
        uint32_t capacity;
        uint64_t releaseFenceValue;
        T heapData;
    };

    DynamicHeapAllocator(uint32_t minHeapSize)
        : m_heaps{},
          m_minHeapSize{minHeapSize},
          m_frontIdx{},
          m_fenceValue{} {};

    Heap *allocate(uint32_t numElements, uint64_t lastFenceCompleted,
                   uint64_t nextFenceValue) {
        if (!m_heaps.empty()) {
            auto &heap = m_heaps[m_frontIdx];
            if (!heap.capacity) {
                heap.capacity = m_minHeapSize;
                while (heap.capacity < numElements)
                    heap.capacity *= 2;
            }

            uint32_t newCapacity = numElements + heap.allocated;
            if (newCapacity <= heap.capacity && m_fenceValue == nextFenceValue) {
                heap.start = heap.allocated;
                heap.allocated += numElements;
                heap.releaseFenceValue = nextFenceValue;
                return &heap;
            }

            m_fenceValue = nextFenceValue;
            for (uint32_t i = 0; i < m_heaps.size(); ++i) {
                auto &curHeap = m_heaps[i];
                if (curHeap.releaseFenceValue < lastFenceCompleted) {
                    m_frontIdx = i;
                    curHeap.start = 0;
                    curHeap.allocated = 0;
                    curHeap.releaseFenceValue = nextFenceValue;
                    return allocate(numElements, lastFenceCompleted, nextFenceValue);
                }
            }
        }

        m_frontIdx = m_heaps.allocateBack();
        m_heaps[m_frontIdx].releaseFenceValue = nextFenceValue;
        return allocate(numElements, lastFenceCompleted, nextFenceValue);
    }

    Heap *front() { return &m_heaps[m_frontIdx]; }

 private:
    VectorCached<Heap, 16> m_heaps;
    uint32_t m_minHeapSize;
    uint32_t m_frontIdx;
    uint64_t m_fenceValue;
};

struct ContextD3D12;

struct DeferredReleaseD3D12 : Object, DeferredRelease {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    void registerObject(NvFlexObject *object) override;

    void pushForRelease(NvFlexObject *object) override;

    // Details
    DeferredReleaseD3D12(const NvFlexContextDescD3D12 *desc);
    ~DeferredReleaseD3D12();

    void blockingRelease();

    void contextReleaseNotify();

    void doDeferredRelease();

    static void threadFunc(DeferredReleaseD3D12 *ptr);

    void update(const NvFlexContextDescD3D12 *desc);

    struct DeferredElement {
        NvFlexObject *object;
        uint64_t releaseFenceID;
    };

    HMODULE m_d3d12module;
    ID3D12Device *m_device;
    ID3D12Fence *m_commandQueueFence;
    uint64_t m_lastFenceCompleted;
    uint64_t m_nextFenceValue;
    HMODULE m_module;
    std::atomic<uint32_t> m_deferredObjectRefCount;
    VectorCached<DeferredElement, 64> m_deferredReleaseObjects;
};

DeferredReleaseD3D12 *createDeferredRelease(const NvFlexContextDescD3D12 *desc);

struct BufferData {
    BufferData()
        : m_buffer{},
          m_mappedData{} {}

    ~BufferData() { SafeRelease(m_buffer); }

    BufferData(const BufferData &) = delete;

    BufferData(BufferData &&rhs) noexcept
        : m_buffer{rhs.m_buffer},
          m_mappedData{rhs.m_mappedData} {
        rhs.m_buffer = nullptr;
        rhs.m_mappedData = nullptr;
    }

    BufferData &operator=(const BufferData &) = delete;

    BufferData &operator=(BufferData &&rhs) noexcept {
        swap(m_buffer, rhs.m_buffer);
        swap(m_mappedData, rhs.m_mappedData);
        return *this;
    }

    ID3D12Resource *m_buffer;
    void *m_mappedData;
};

struct HeapData {
    HeapData()
        : m_heap{} {}
    ~HeapData() { SafeRelease(m_heap); }

    HeapData(const HeapData &) = delete;
    HeapData(HeapData &&rhs) noexcept
        : m_heap{rhs.m_heap} {
        rhs.m_heap = nullptr;
    }

    HeapData &operator=(const HeapData &) = delete;
    HeapData &operator=(HeapData &&rhs) noexcept {
        swap(m_heap, rhs.m_heap);
        return *this;
    }

    ID3D12DescriptorHeap *m_heap;
};

struct ConstantBufferD3D12 : Object, ConstantBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    ConstantBufferD3D12(ContextD3D12 *context, const NvFlexConstantBufferDesc *desc);
    ~ConstantBufferD3D12();

    ID3D12Resource *getFront();

    VersionedBuffer<BufferData> m_buffers;
    ID3D12Resource *m_bufferGPU;
};

struct VertexBufferD3D12 : Object, VertexBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    VertexBufferD3D12(ContextD3D12 *context, const NvFlexVertexBufferDesc *desc);
    ~VertexBufferD3D12();

    ID3D12Resource *m_buffer;
    VersionedBuffer<BufferData> m_uploadBuffers;
};

struct IndexBufferD3D12 : Object, IndexBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    IndexBufferD3D12(ContextD3D12 *context, const NvFlexIndexBufferDesc *desc);
    ~IndexBufferD3D12();

    ID3D12Resource *m_buffer;
    VersionedBuffer<BufferData> m_uploadBuffers;
    DXGI_FORMAT m_format;
};

struct ResourceStateD3D12 {
    ResourceStateD3D12(ID3D12Resource **resource, D3D12_RESOURCE_STATES *resourceState,
                       D3D12_RESOURCE_STATES *restoreResourceState);
    ID3D12Resource **const ref_resource;
    D3D12_RESOURCE_STATES *const ref_resourceState;
    D3D12_RESOURCE_STATES *const ref_restoreResourceState;
};

struct ResourceD3D12 : Resource, ResourceStateD3D12 {
    ResourceD3D12(ID3D12Resource **resource, D3D12_RESOURCE_STATES *resourceState,
                  D3D12_RESOURCE_STATES *restoreResourceState);

    D3D12_CPU_DESCRIPTOR_HANDLE m_srvHandle;
    D3D12_SHADER_RESOURCE_VIEW_DESC m_srvDesc;
};

struct ResourceRWD3D12 : ResourceRW, ResourceD3D12 {
    ResourceRWD3D12(ID3D12Resource **resource, D3D12_RESOURCE_STATES *resourceState,
                    D3D12_RESOURCE_STATES *restoreResourceState);

    D3D12_CPU_DESCRIPTOR_HANDLE m_uavHandle;
    D3D12_UNORDERED_ACCESS_VIEW_DESC m_uavDesc;
};

struct DepthStencilD3D12 : DepthStencil, ResourceStateD3D12 {
    DepthStencilD3D12(ID3D12Resource **resource, D3D12_RESOURCE_STATES *resourceState,
                      D3D12_RESOURCE_STATES *restoreResourceState);

    D3D12_CPU_DESCRIPTOR_HANDLE m_dsvHandle;
    D3D12_DEPTH_STENCIL_VIEW_DESC m_dsvDesc;
};

struct RenderTargetD3D12 : RenderTarget, ResourceStateD3D12 {
    RenderTargetD3D12(ID3D12Resource **resource, D3D12_RESOURCE_STATES *resourceState,
                      D3D12_RESOURCE_STATES *restoreResourceState);

    D3D12_CPU_DESCRIPTOR_HANDLE m_rtvHandle;
    D3D12_RENDER_TARGET_VIEW_DESC m_rtvDesc;
    D3D12_RECT m_scissor;
};

struct BufferD3D12 : Object, Buffer, ResourceRWD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    NvFlexContext *getContext() override;
    Resource *getResource() override final;
    ResourceRW *getResourceRW() override final;

    // Details
    BufferD3D12(ContextD3D12 *context, const NvFlexBufferDesc *desc);
    BufferD3D12(ContextD3D12 *context, BufferD3D12 *buffer,
                const NvFlexBufferViewDesc *desc);
    BufferD3D12(ContextD3D12 *context, ID3D12Resource *nativeBuffer, NvFlexUint dim,
                NvFlexUint structStride);
    ~BufferD3D12();

    ContextD3D12 *m_context;
    Buffer *m_parent;
    ID3D12Resource *m_buffer;
    VersionedBuffer<BufferData> m_uploadBuffers;
    uint64_t m_readbackComputeFence;

private:
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct Texture1DD3D12 : Object, Texture1D, ResourceRWD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override final;
    ResourceRW *getResourceRW() override final;

    // Details
    Texture1DD3D12(ContextD3D12 *context, const NvFlexTexture1DDesc *desc);
    ~Texture1DD3D12();

    ID3D12Resource *m_texture;
    VersionedBuffer<BufferData> m_uploadBuffers;
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT m_footPrint;
    uint64_t m_uploadHeapSize;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct Texture2DD3D12 : Object, Texture2D, ResourceRWD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override final;
    ResourceRW *getResourceRW() override final;

    void openSharedHandle(HANDLE *handleIn) override final;
    void closeSharedHandle(HANDLE handleIn) override final;

    // Details
    Texture2DD3D12(ContextD3D12 *context, const NvFlexTexture2DDesc *desc,
                   bool createShared);
    Texture2DD3D12(ContextD3D12 *context, Texture2D *sharedTexture, bool openShared);
    ~Texture2DD3D12();

    void createViews(ContextD3D12 *context);

    ID3D12Device *m_device;
    ID3D12Resource *m_texture;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct Texture3DD3D12 : Object, Texture3D, ResourceRWD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override final;
    ResourceRW *getResourceRW() override final;

    // Details
    Texture3DD3D12(ContextD3D12 *context, const NvFlexTexture3DDesc *desc);
    ~Texture3DD3D12();

    ID3D12Resource *m_texture;
    VersionedBuffer<BufferData> m_uploadBuffers;
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT m_footPrint;
    uint64_t m_uploadHeapSize;
    ID3D12Resource *m_downloadBuffer;
    uint64_t m_downloadCompleteFence;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct Texture2DCrossAdapterD3D12 : Object, Texture2DCrossAdapter, ResourceStateD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Texture2DCrossAdapterD3D12(ContextD3D12 *context,
                               Texture2DCrossAdapterD3D12 *sharedResource);
    Texture2DCrossAdapterD3D12(ContextD3D12 *context, const NvFlexTexture2DDesc *desc);
    ~Texture2DCrossAdapterD3D12();

    D3D12_PLACED_SUBRESOURCE_FOOTPRINT m_footPrint;
    ID3D12Heap *m_sharedHeap;
    ID3D12Resource *m_texture;
    ID3D12Device *m_device;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct ResourceReferenceD3D12 : Object, ResourceReference {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    ResourceReferenceD3D12(ContextD3D12 *context, ResourceD3D12 *resource);
    ~ResourceReferenceD3D12();

    ID3D12Resource *m_resource;
};

struct HeapVTRD3D12 : Object, HeapVTR {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    HeapVTRD3D12(ContextD3D12 *context, const NvFlexHeapSparseDesc *desc);
    ~HeapVTRD3D12();

    ID3D12Heap *m_heap;
};

struct Texture3DVTRD3D12 : Object, Texture3DVTR, ResourceRWD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override;
    ResourceRW *getResourceRW() override;

    // Details
    Texture3DVTRD3D12(ContextD3D12 *context, const NvFlexTexture3DSparseDesc *desc);
    ~Texture3DVTRD3D12();

    ID3D12Resource *m_texture;
    Image3D<uint32_t> m_blockTable;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct ColorBufferD3D12 : Object, ColorBuffer, ResourceRWD3D12, RenderTargetD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override;
    ResourceRW *getResourceRW() override;

    RenderTarget *getRenderTarget() override;

    // Details
    ColorBufferD3D12(ContextD3D12 *context, const NvFlexColorBufferDesc *desc);
    ~ColorBufferD3D12();

    ID3D12Resource *m_texture;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct DepthBufferD3D12 : Object, DepthBuffer, ResourceRWD3D12, DepthStencilD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override;

    DepthStencil *getDepthStencil() override;

    // Details
    DepthBufferD3D12(ContextD3D12 *context, const NvFlexDepthBufferDesc *desc);
    ~DepthBufferD3D12();

    ID3D12Resource *m_texture;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct DepthStencilViewD3D12 : Object, DepthStencilView, ResourceD3D12, DepthStencilD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override;

    DepthStencil *getDepthStencil() override;

    NvFlexDepthBufferDesc getDepthBufferDesc() override;

    // Details
    DepthStencilViewD3D12(ContextD3D12 *context,
                          const NvFlexDepthStencilViewDescD3D12 *desc);
    ~DepthStencilViewD3D12();

    void update(ContextD3D12 *context, const NvFlexDepthStencilViewDescD3D12 *desc);

    NvFlexDepthStencilViewDescD3D12 m_desc;
    ID3D12Resource *m_dsvResource;
    D3D12_RESOURCE_STATES m_dsvResourceState;
    D3D12_RESOURCE_STATES m_dsvRestoreResourceState;
    ID3D12Resource *m_srvResource;
    D3D12_RESOURCE_STATES m_srvResourceState;
    D3D12_RESOURCE_STATES m_srvResourceResourceState;
};

struct RenderTargetViewD3D12 : Object, RenderTargetView, RenderTargetD3D12 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    RenderTarget *getRenderTarget() override;

    // Details
    RenderTargetViewD3D12(ContextD3D12 *context,
                          const NvFlexRenderTargetViewDescD3D12 *desc);
    ~RenderTargetViewD3D12();

    void update(ContextD3D12 *context, const NvFlexRenderTargetViewDescD3D12 *desc);

    NvFlexRenderTargetViewDescD3D12 m_desc;
    ID3D12Resource *m_resource;
    D3D12_RESOURCE_STATES m_resourceState;
    D3D12_RESOURCE_STATES m_restoreResourceState;
};

struct ComputeShaderD3D12 : Object, ComputeShader {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    ComputeShaderD3D12(ContextD3D12 *context, const NvFlexComputeShaderDesc *desc);
    ~ComputeShaderD3D12();

    uint32_t m_labelSizeInBytes;
    ID3D12PipelineState *m_cs;
};

struct GraphicsShaderD3D12 : Object, GraphicsShader {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed();

    // Details
    GraphicsShaderD3D12(ContextD3D12 *context, const NvFlexGraphicsShaderDesc *desc);
    ~GraphicsShaderD3D12();

    void createPSO(ContextD3D12 *context);

    ID3D12PipelineState *getPSO(bool frontCounterClockwise);

    void setFormats(ContextD3D12 *context, NvFlexFormat renderTargetFormat,
                    NvFlexFormat depthStencilFormat);

    struct Version {
        Version();
        ~Version();
        NvFlexFormat renderTargetFormat;
        NvFlexFormat depthStencilFormat;
        ID3D12PipelineState *m_psoLH;
        ID3D12PipelineState *m_psoRH;
    };

    uint32_t m_labelSizeInBytes;
    D3D12_GRAPHICS_PIPELINE_STATE_DESC m_psoDesc;
    VectorCached<NvFlexInputElementDesc, 4> m_inputElementDescs;
    VectorCached<Version, 4> m_versions;
    uint32_t m_versionIdex;
};

struct TimerD3D12 : Object, Timer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    TimerD3D12(ContextD3D12 *context);
    ~TimerD3D12();

    LARGE_INTEGER m_cpuFreq;
    LARGE_INTEGER m_cpuBegin;
    LARGE_INTEGER m_cpuEnd;
    ID3D12QueryHeap *m_queryHeap;
    ID3D12Resource *m_queryReadback;
    uint64_t m_queryFrequency;
    uint64_t m_queryReadbackFenceVal;
    int m_state;
};

struct EventQueueD3D12 : Object, EventQueue {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    EventQueueD3D12(ContextD3D12 *context);
    ~EventQueueD3D12();

    enum EventState { eEventStateInactive = 0, eEventStateActive = 1 };

    struct Event {
        uint64_t uid;
        uint64_t pushID;
        EventState state;
        uint64_t fenceID;
    };

    Event *getNewEvent(ContextD3D12 *context);

    NvFlexResult pop(uint64_t *pUid, uint64_t lastFenceCompleted);
    void push(ContextD3D12 *context, uint64_t uid, uint64_t nextFenceValue);

    uint64_t m_pushID;
    VectorCached<Event, 16> m_events;
};

struct FenceD3D12 : Object, Fence {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    FenceD3D12(ContextD3D12 *context, FenceD3D12 *fence);
    FenceD3D12(ContextD3D12 *context, const NvFlexFenceDesc *desc);
    ~FenceD3D12();

    void signalFence(ContextD3D12 *context, uint64_t fenceValue);

    void waitOnFence(ContextD3D12 *context, uint64_t fenceValue);

    ID3D12Fence *m_fence;
    ID3D12Device *m_device;
};

// MARK: DescriptorHeapD3D12
struct DescriptorHeapD3D12 : Object, NvFlexContextObject {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    DescriptorHeapD3D12(ContextD3D12 *context, D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                        uint32_t minHeapSize);

    D3D12_CPU_DESCRIPTOR_HANDLE allocate(uint32_t numDescriptors);

    static DescriptorHeapD3D12 *create(ContextD3D12 *context,
                                       D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                                       uint32_t minHeapSize);

    HeapAllocator<HeapData> m_heaps;
    uint32_t m_descriptorSize;
    D3D12_DESCRIPTOR_HEAP_TYPE m_heapType;
    ID3D12Device *m_device;
};

struct DynamicDescriptorHeapD3D12 : Object, NvFlexContextObject {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    uint64_t getGPUBytesUsed() override;

    // Details
    DynamicDescriptorHeapD3D12(ContextD3D12 *context, D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                               uint32_t minHeapSize);

    struct Handles {
        D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle;
        D3D12_GPU_DESCRIPTOR_HANDLE gpuHandle;
    };

    Handles allocate(uint32_t numDescriptors, uint64_t lastFenceCompleted,
                     uint64_t nextFenceValue);

    static DynamicDescriptorHeapD3D12 *create(ContextD3D12 *context,
                                              D3D12_DESCRIPTOR_HEAP_TYPE heapType,
                                              uint32_t minHeapSize);

    NvFlexDynamicDescriptorHeapD3D12 getInterface();

    static NvFlexDescriptorReserveHandleD3D12 resverseDescriptors(
        void *userdata, uint32_t numDescriptors, uint64_t lastFenceCompleted,
        uint64_t nextFenceValue);

    DynamicHeapAllocator<HeapData> m_heaps;
    uint32_t m_descriptorSize;
    D3D12_DESCRIPTOR_HEAP_TYPE m_heapType;
    ID3D12Device *m_device;
};

struct ContextD3D12 : Object, Context {
    uint32_t addRef() override;
    uint32_t release() override;
    uint64_t getGPUBytesUsed() override;

    void processFenceSignal(NvFlexContext *context) override;

    void processFenceWait(NvFlexContext *context) override;

    void contextPush() override;

    void contextPop() override;

    NvFlexContextAPI getContextType() override;

    ConstantBuffer *createConstantBuffer(const NvFlexConstantBufferDesc *desc) override;

    NvFlexMappedData map(Texture3D *buffer) override;

    void *map(Buffer *buffer, NvFlexStagingCpuAccessFlags cpuAccessMode,
              bool cpuNoWait) override;

    void unmap(Buffer *buffer) override;

    void *map(ConstantBuffer *buffer) override;

    void *map(IndexBuffer *buffer) override;

    void *map(Texture1D *buffer) override;

    void *map(VertexBuffer *buffer) override;

    void unmap(ConstantBuffer *buffer) override;

    void unmap(IndexBuffer *buffer) override;

    void unmap(VertexBuffer *buffer) override;

    void unmap(Texture1D *texture) override;

    void unmap(Texture3D *texture) override;

    void upload(Buffer *buffer, NvFlexUint offset, const void *data, NvFlexUint numBytes) override;

    void copy(Buffer *dst, uint32_t dstOffset, Buffer *src, uint32_t srcOffset,
              uint32_t numBytes) override;

    void copy(Buffer *dst, Resource *src, uint32_t offset, uint32_t numBytes) override;

    void copy(ConstantBuffer *dst, Buffer *src) override;

    void copy(DepthStencil *dst, Resource *src) override;

    void copy(ResourceRW *dst, Resource *src) override;

    void copy(Texture3D *dst, Texture3D *src) override;

    void copy(Texture3D *dst, Resource *src) override;

    VertexBuffer *createVertexBuffer(const NvFlexVertexBufferDesc *desc) override;

    IndexBuffer *createIndexBuffer(const NvFlexIndexBufferDesc *desc) override;

    Buffer *createBuffer(const NvFlexBufferDesc *desc) override;

    Buffer *createBufferView(Buffer *buffer, const NvFlexBufferViewDesc *desc) override;

    void download(Texture3D *buffer) override;

    NvFlexMappedData mapDownload(Texture3D *buffer) override;

    void unmapDownload(Texture3D *texture) override;

    Texture1D *createTexture1D(const NvFlexTexture1DDesc *desc) override;

    Texture2D *createTexture2D(const NvFlexTexture2DDesc *desc) override;

    Texture2D *shareTexture2D(Texture2D *sharedTexture) override;

    Texture2D *shareTexture2DShared(Texture2D *sharedTexture) override;

    Texture2D *createTexture2DShared(const NvFlexTexture2DDesc *desc) override;

    Texture3D *createTexture3D(const NvFlexTexture3DDesc *desc) override;

    ResourceReference *shareResourceReference(Resource *resource) override;

    HeapVTR *createHeapVTR(const NvFlexHeapSparseDesc *desc) override;

    Texture3DVTR *createTexture3DVTR(const NvFlexTexture3DSparseDesc *desc) override;

    ColorBuffer *createColorBuffer(const NvFlexColorBufferDesc *desc) override;

    DepthBuffer *createDepthBuffer(const NvFlexDepthBufferDesc *desc) override;

    ComputeShader *createComputeShader(const NvFlexComputeShaderDesc *desc) override;

    GraphicsShader *createGraphicsShader(const NvFlexGraphicsShaderDesc *desc) override;

    void setFormats(GraphicsShader *graphicsShader, NvFlexFormat renderTargetFormat,
                    NvFlexFormat depthStencilFormat) override;

    Texture2DCrossAdapter *createTexture2DCrossAdapter(
        const NvFlexTexture2DDesc *desc) override;

    Texture2DCrossAdapter *shareTexture2DCrossAdapter(
        Texture2DCrossAdapter *sharedTexture) override;

    void transitionToCommonState(Resource *resource) override;

    void copyFromShared(Texture2D *dstTexture, Texture2DCrossAdapter *sharedTexture,
                        uint32_t height) override;

    void copyToShared(Texture2DCrossAdapter *dstSharedTexture, Texture2D *srcTexture,
                      uint32_t height) override;

    Fence *createFence(const NvFlexFenceDesc *desc) override;

    Fence *shareFence(Fence *fence) override;

    void waitOnFence(Fence *fence, uint64_t fenceValue) override;

    void signalFence(Fence *fence, uint64_t fenceValue) override;

    void dispatch(const NvFlexDispatchParams *params) override;

    void setVertexBuffer(VertexBuffer *buffer, uint32_t stride, uint32_t offset) override;

    void setIndexBuffer(IndexBuffer *buffer, uint32_t offset) override;

    void drawIndexedInstanced(uint32_t indicesPerInstance, uint32_t numInstances,
                              const NvFlexDrawParams *params) override;

    void setRenderTarget(RenderTarget *rtv, DepthStencil *dsv) override;

    void setViewport(const NvFlexViewport *vp) override;

    void clearRenderTarget(RenderTarget *rtv, const float color[4]) override;

    void clearDepthStencil(DepthStencil *dsv, float depth) override;

    void restoreResourceState(Resource *resource) override;

    void clearState() override;

    int is_VTR_supported() override;

    void updateVTRMapping(Texture3DVTR *textureIn, HeapVTR *heapIn,
                          uint32_t *blockTableImage, uint32_t rowPitch,
                          uint32_t depthPitch) override;

    Timer *createTimer() override;

    void timerBegin(Timer *timer) override;

    void timerEnd(Timer *timer) override;

    NvFlexResult timerGetResult(Timer *timer, float *timeGPU, float *timeCPU,
                                NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp,
                                NvFlexUint64 *gpuFreq) override;

    EventQueue *createEventQueue() override;

    void eventQueuePush(EventQueue *eventQueueIn, uint64_t uid) override;

    NvFlexResult eventQueuePop(EventQueue *eventQueueIn, uint64_t *pUid) override;

    NvFlexResult registerD3DBuffer(void *nativeBuffer, NvFlexUint dim,
                                   NvFlexUint structStride, Buffer **ppBuffer) override;

    // Details
    DeferredRelease *getDeferredRelease();
    ID3D12Device *getDevice();
    ID3D12CommandQueue *getCommandQueue();
    ID3D12CommandList *getCommandList();

    ContextD3D12(const NvFlexContextDescD3D12 *desc);
    ContextD3D12(Device *deviceImpl, const NvFlexContextDescD3D12 *desc);
    ~ContextD3D12();

    void updateContext(const NvFlexContextDescD3D12 *desc);

    void updateContextDesc(NvFlexContextDescD3D12 *desc);

    NvFlexDepthStencilView *createDepthStencilView(
        const NvFlexDepthStencilViewDescD3D12 *desc);

    NvFlexRenderTargetView *createRenderTargetView(
        const NvFlexRenderTargetViewDescD3D12 *desc);

    void updateDepthStencilView(NvFlexDepthStencilView *view,
                                const NvFlexDepthStencilViewDescD3D12 *desc);

    void updateRenderTargetView(NvFlexRenderTargetView *view,
                                const NvFlexRenderTargetViewDescD3D12 *desc);

    void updateResoruceRWViewDesc(ResourceRWD3D12 *resourceRW,
                                  NvFlexResourceRWViewDescD3D12 *desc);

    void updateResourceViewDesc(ResourceD3D12 *resource, NvFlexResourceViewDescD3D12 *desc);

    struct FenceEvent {
        Fence *fence;
        uint64_t fenceValue;
    };

    HMODULE m_d3d12module;
    HMODULE m_dxgimodule;
    ID3D12Device *m_device;
    ID3D12CommandQueue *m_commandQueue;
    ID3D12Fence *m_commandQueueFence;
    ID3D12GraphicsCommandList *m_commandList;
    uint64_t m_lastFenceCompleted;
    uint64_t m_nextFenceValue;
    NvFlexDynamicDescriptorHeapD3D12 m_gpuDescriptorHeap;
    ID3D12RootSignature *m_rootSignatureGraphics;
    ID3D12RootSignature *m_rootSignatureCompute;
    bool m_VTRSupportChecked;
    bool m_VTRSupported;
    DescriptorHeapD3D12 *m_cpuDescriptorHeap;
    DynamicDescriptorHeapD3D12 *m_gpuDescriptorHeapImpl;
    DescriptorHeapD3D12 *m_rtvDescriptorHeap;
    DescriptorHeapD3D12 *m_dsvDescriptorHeap;
    D3D12_CPU_DESCRIPTOR_HANDLE m_nullUAV;
    D3D12_CPU_DESCRIPTOR_HANDLE m_nullSRV;
    VectorCached<D3D12_TILED_RESOURCE_COORDINATE, 1> m_tileCoords;
    VectorCached<D3D12_TILE_REGION_SIZE, 1> m_tileRegionSize;
    VectorCached<enum D3D12_TILE_RANGE_FLAGS, 1> m_rangeFlags;
    VectorCached<unsigned int, 1> m_tilePoolCoords;
    VectorCached<unsigned int, 1> m_tilePoolRangeSize;
    DeferredReleaseD3D12 *m_deferredRelease;
    VectorCached<ContextD3D12::FenceEvent, 4> m_waitFenceEvents;
    VectorCached<ContextD3D12::FenceEvent, 4> m_signalFenceEvents;
    uint64_t m_waitFenceEventsVersion;
    uint64_t m_signalFenceEventsVersion;
};
}  // namespace NvFlex

#endif /* CONTEXTD3D12_H */
