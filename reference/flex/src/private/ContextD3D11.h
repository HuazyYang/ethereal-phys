#ifndef CONTEXTD3D11_H
#define CONTEXTD3D11_H
// Tiled resource in feature tier 2 is requrired here.
#include <d3d11_2.h>

#include "Context.h"
#include "Object.h"
#include "VectorCached.h"
#include <nvflex/NvFlexTypes.h>
#include <nvflex/NvFlexContext.h>
#include <nvflex/NvFlexContextD3D11.h>
#include "ClientHelper.h"
#include "Image3D.h"

namespace NvFlex {

struct ContextD3D11;

int64_t FlexDeferredReleaseD3D11(float timeoutMS);

void copyViewport(NvFlexViewport *dst, const D3D11_VIEWPORT *src);
D3D11_BLEND convertToD3D11(NvFlexBlendEnum blendEnum);
D3D11_BLEND_OP convertToD3D11(NvFlexBlendOpEnum blendOpEnum);
D3D11_COMPARISON_FUNC convertToD3D11(NvFlexComparisonEnum comparison);

struct ConstantBufferD3D11 : Object, ConstantBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    ConstantBufferD3D11(ContextD3D11 *ctx, const NvFlexConstantBufferDesc *desc);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11Buffer> m_buffer;
};

struct VertexBufferD3D11 : Object, VertexBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    VertexBufferD3D11(ContextD3D11 *ctx, const NvFlexVertexBufferDesc *desc);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11Buffer> m_buffer;
    ComPtr<ID3D11Buffer> m_uploadBuffer;
};

struct IndexBufferD3D11 : Object, IndexBuffer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    IndexBufferD3D11(ContextD3D11 *ctx, const NvFlexIndexBufferDesc *desc);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11Buffer> m_buffer;
    ComPtr<ID3D11Buffer> m_uploadBuffer;
    DXGI_FORMAT m_format;
};

struct ResourceD3D11 : Resource {
    ComPtr<ID3D11ShaderResourceView> m_srv;
};

struct ResourceRWD3D11 : ResourceRW, ResourceD3D11 {
    Resource *getResource() override { return this; }

    ComPtr<ID3D11UnorderedAccessView> m_uav;
};

struct DepthStencilD3D11 : DepthStencil {
    ComPtr<ID3D11DepthStencilView> m_dsv;
};

struct RenderTargetD3D11 : RenderTarget {
    ComPtr<ID3D11RenderTargetView> m_rtv;
};

struct BufferD3D11 : Object, Buffer, ResourceRWD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    BufferD3D11(ContextD3D11 *ctx, BufferD3D11 *buffer, const NvFlexBufferViewDesc *desc);

    BufferD3D11(ContextD3D11 *ctx, const NvFlexBufferDesc *desc);

    BufferD3D11(ContextD3D11 *ctx, ID3D11Buffer *d3dBuffer, NvFlexUint dim,
                NvFlexUint structStride);

    uint64_t getGPUBytesUsed() override;

    NvFlexContext *getContext() override;

    Resource *getResource() override { return this; }

    ResourceRW *getResourceRW() override { return this; }

    ContextD3D11 *m_context;
    ComPtr<ID3D11Buffer> m_buffer;
};

struct Texture1DD3D11 : Object, Texture1D, ResourceRWD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    Texture1DD3D11(ContextD3D11 *ctx, const NvFlexTexture1DDesc *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }

    ResourceRW *getResourceRW() override { return this; }

    ComPtr<ID3D11Texture1D> m_texture;
    ComPtr<ID3D11Texture1D> m_uploadTexture;
};

struct Texture2DD3D11 : Object, Texture2D, ResourceRWD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    Texture2DD3D11(ContextD3D11 *ctx, const NvFlexTexture2DDesc *desc, bool createShared);
    Texture2DD3D11(ContextD3D11 *ctx, Texture2D *sharedTexture, bool openShared);

    uint64_t getGPUBytesUsed() override;

    void openSharedHandle(HANDLE *handleIn) override;
    void closeSharedHandle(HANDLE handle) override;

    Resource *getResource() override { return this; }

    ResourceRW *getResourceRW() override { return this; }

    ComPtr<ID3D11Texture2D> m_texture;
};

struct Texture3DD3D11 : Object, Texture3D, ResourceRWD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    Texture3DD3D11(ContextD3D11 *ctx, const NvFlexTexture3DDesc *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }
    ResourceRW *getResourceRW() override { return this; }

    ComPtr<ID3D11Texture3D> m_texture;
    ComPtr<ID3D11Texture3D> m_uploadTexture;
    ComPtr<ID3D11Texture3D> m_downloadTexture;
};

struct ResourceReferenceD3D11 : Object, ResourceReference {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    ResourceReferenceD3D11(ContextD3D11 *ctx, ResourceD3D11 *resource);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11Resource> m_resource;
};

struct HeapVTRD3D11 : Object, ResourceRWD3D11, HeapVTR {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    HeapVTRD3D11(ContextD3D11 *ctx, const NvFlexHeapSparseDesc *desc);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11Buffer> m_heap;
};

struct Texture3DVTRD3D11 : Object, Texture3DVTR, ResourceRWD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    Texture3DVTRD3D11(ContextD3D11 *ctx, const NvFlexTexture3DSparseDesc *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }

    ResourceRW *getResourceRW() override { return this; }

    ComPtr<ID3D11Texture3D> m_texture;
    Image3D<uint32_t> m_blockTable;
};

struct ColorBufferD3D11 : Object, ColorBuffer, ResourceRWD3D11, RenderTargetD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    ColorBufferD3D11(ContextD3D11 *ctx, const NvFlexColorBufferDesc *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }
    ResourceRW *getResourceRW() override { return this; }
    RenderTarget *getRenderTarget() override { return this; }

    ComPtr<ID3D11Texture2D> m_texture;
};

struct DepthBufferD3D11 : Object, DepthBuffer, ResourceD3D11, DepthStencilD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    DepthBufferD3D11(ContextD3D11 *ctx, const NvFlexDepthBufferDesc *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }
    DepthStencil *getDepthStencil() override { return this; }

    ComPtr<ID3D11Texture2D> m_texture;
};

struct DepthStencilViewD3D11 : Object, DepthStencilD3D11, DepthStencilView, ResourceD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    DepthStencilViewD3D11(ContextD3D11 *ctx, const NvFlexDepthStencilViewDescD3D11 *desc);

    uint64_t getGPUBytesUsed() override;

    Resource *getResource() override { return this; }
    DepthStencil *getDepthStencil() override { return this; }

    void update(const NvFlexDepthStencilViewDescD3D11 *desc);

    NvFlexDepthBufferDesc getDepthBufferDesc() override;

    NvFlexDepthStencilViewDescD3D11 m_desc;
};

struct RenderTargetViewD3D11 : Object, RenderTargetView, RenderTargetD3D11 {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    RenderTargetViewD3D11(ContextD3D11 *ctx, const NvFlexRenderTargetViewDescD3D11 *desc);

    uint64_t getGPUBytesUsed() override;

    void update(const NvFlexRenderTargetViewDescD3D11 *desc);

    RenderTarget *getRenderTarget() override { return this; }

    NvFlexRenderTargetViewDescD3D11 m_desc;
};

struct ComputeShaderD3D11 : Object, ComputeShader {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    ComputeShaderD3D11(ContextD3D11 *ctx, const NvFlexComputeShaderDesc *desc);

    uint64_t getGPUBytesUsed() override;

    ComPtr<ID3D11ComputeShader> m_cs;
};

struct GraphicsShaderD3D11 : Object, GraphicsShader {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    GraphicsShaderD3D11(ContextD3D11 *ctx, const NvFlexGraphicsShaderDesc *desc);

    uint64_t getGPUBytesUsed() override;

    void setFormats(NvFlexFormat rtFormat, NvFlexFormat dsFormat);

    VectorCached<NvFlexInputElementDesc, 4> m_inputElementDescs;
    ComPtr<ID3D11VertexShader> m_vs;
    ComPtr<ID3D11PixelShader> m_ps;
    ComPtr<ID3D11InputLayout> m_inputLayout;
    ComPtr<ID3D11BlendState> m_blenderState;
    ComPtr<ID3D11DepthStencilState> m_depthStencilState;
    ComPtr<ID3D11RasterizerState> m_rasterizerStateLH;
    ComPtr<ID3D11RasterizerState> m_rasterizerStateRH;
};

struct TimerD3D11 : Object, Timer {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    TimerD3D11(ContextD3D11 *ctx);

    uint64_t getGPUBytesUsed() override;

    uint32_t m_state;
    ComPtr<ID3D11Query> m_disjoint;
    ComPtr<ID3D11Query> m_begin;
    ComPtr<ID3D11Query> m_end;
    LARGE_INTEGER m_cpuFreq;
    LARGE_INTEGER m_cpuBegin;
    LARGE_INTEGER m_cpuEnd;
};

struct EventQueueD3D11 : Object, EventQueue {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    EventQueueD3D11(ContextD3D11 *ctx);

    uint64_t getGPUBytesUsed() override;

    void push(uint64_t uid, ContextD3D11 *ctx);

    NvFlexResult pop(uint64_t *pUid, ContextD3D11 *ctx);

    enum EventState : uint32_t { eEventStateInactive = 0, eEventStateActive = 1 };

    struct Event {
        uint64_t uid;
        uint64_t pushID;
        EventState state;
        ComPtr<ID3D11Query> query;
    };

    Event *getNewEvent();

    uint64_t m_pushID;
    VectorCached<Event, 16> m_events;
};

struct ContextD3D11 : Object, Context {
    // Interface methods

    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()

    NvFlexContextAPI getContextType() override;

    void clearDepthStencil(DepthStencil *dsv, float depth) override;

    void clearRenderTarget(RenderTarget *rtv, const float color[4]) override;

    void contextPush() override;

    void contextPop() override;

    void copy(Buffer *dst, uint32_t dstOffset, Buffer *src, uint32_t srcOffset,
              uint32_t numBytes) override;

    void copy(Buffer *dst, Resource *src, uint32_t offset, uint32_t numBytes) override;

    void copy(ConstantBuffer *dst, Buffer *src) override;

    void copy(DepthStencil *dst, Resource *src) override;

    void copy(ResourceRW *dst, Resource *src) override;

    void copy(Texture3D *dst, Texture3D *src) override;

    void copy(Texture3D *dst, Resource *src) override;

    void copyFromShared(Texture2D *dstTexture, Texture2DCrossAdapter *sharedTexture,
                        uint32_t height) override;

    void copyToShared(Texture2DCrossAdapter *dstSharedTexture, Texture2D *srcTexture,
                      uint32_t height) override;

    Buffer *createBuffer(const NvFlexBufferDesc *desc) override;

    Buffer *createBufferView(Buffer *buffer, const NvFlexBufferViewDesc *desc) override;

    ColorBuffer *createColorBuffer(const NvFlexColorBufferDesc *desc) override;

    ComputeShader *createComputeShader(const NvFlexComputeShaderDesc *desc) override;

    ConstantBuffer *createConstantBuffer(const NvFlexConstantBufferDesc *desc) override;

    DepthBuffer *createDepthBuffer(const NvFlexDepthBufferDesc *desc) override;

    EventQueue *createEventQueue() override;

    Fence *createFence(const NvFlexFenceDesc *desc) override;

    GraphicsShader *createGraphicsShader(const NvFlexGraphicsShaderDesc *desc) override;

    HeapVTR *createHeapVTR(const NvFlexHeapSparseDesc *desc) override;

    IndexBuffer *createIndexBuffer(const NvFlexIndexBufferDesc *desc) override;

    Texture1D *createTexture1D(const NvFlexTexture1DDesc *desc) override;

    Texture2D *createTexture2D(const NvFlexTexture2DDesc *desc) override;

    Texture2DCrossAdapter *createTexture2DCrossAdapter(
        const NvFlexTexture2DDesc *desc) override;

    Texture2D *createTexture2DShared(const NvFlexTexture2DDesc *desc) override;

    Texture3D *createTexture3D(const NvFlexTexture3DDesc *desc) override;

    Texture3DVTR *createTexture3DVTR(const NvFlexTexture3DSparseDesc *desc) override;

    ResourceReference *shareResourceReference(Resource *resource) override;

    Timer *createTimer() override;

    VertexBuffer *createVertexBuffer(const NvFlexVertexBufferDesc *desc) override;

    void dispatch(const NvFlexDispatchParams *params) override;

    void download(Texture3D *buffer) override;

    void drawIndexedInstanced(uint32_t indicesPerInstance, uint32_t numInstances,
                              const NvFlexDrawParams *params) override;

    void eventQueuePush(EventQueue *eventQueueIn, uint64_t uid) override;
    NvFlexResult eventQueuePop(EventQueue *eventQueueIn, uint64_t *pUid) override;

    int is_VTR_supported() override;

    NvFlexMappedData map(Texture3D *buffer) override;

    void *map(Buffer *buffer, NvFlexStagingCpuAccessFlags cpuAccessMode,
                    bool cpuNoWait) override;

    void unmap(Buffer *buffer) override;

    void *map(ConstantBuffer *buffer) override;

    void *map(IndexBuffer *buffer) override;

    void *map(Texture1D *buffer) override;

    void *map(VertexBuffer *buffer) override;

    NvFlexMappedData mapDownload(Texture3D *buffer) override;

    void processFenceSignal(NvFlexContext *context) override;

    void processFenceWait(NvFlexContext *context) override;

    void restoreResourceState(Resource *resource) override;

    void clearState() override;

    void setFormats(GraphicsShader *graphicsShader, NvFlexFormat renderTargetFormat,
                    NvFlexFormat depthStencilFormat) override;

    void setIndexBuffer(IndexBuffer *buffer, uint32_t offset) override;

    void setVertexBuffer(VertexBuffer *buffer, uint32_t stride, uint32_t offset) override;

    void setViewport(const NvFlexViewport *vp) override;

    Fence *shareFence(Fence *fence) override;

    Texture2D *shareTexture2D(Texture2D *sharedTexture) override;

    Texture2DCrossAdapter *shareTexture2DCrossAdapter(
        Texture2DCrossAdapter *sharedTexture) override;

    Texture2D *shareTexture2DShared(Texture2D *sharedTexture) override;

    void signalFence(Fence *fence, uint64_t fenceValue) override;

    void timerBegin(Timer *timer) override;

    void timerEnd(Timer *timer) override;

    NvFlexResult timerGetResult(Timer *timer, float *timeGPU, float *timeCPU,
                                NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp,
                                NvFlexUint64 *gpuFreq) override;

    void transitionToCommonState(Resource *resource) override;

    void unmap(ConstantBuffer *buffer) override;

    void unmap(IndexBuffer *buffer) override;

    void unmap(VertexBuffer *buffer) override;

    void unmap(Texture1D *texture) override;

    void unmap(Texture3D *texture) override;

    void unmapDownload(Texture3D *texture) override;

    void upload(Buffer *buffer, NvFlexUint offset, const void *data, NvFlexUint numBytes) override;

    void updateVTRMapping(Texture3DVTR *textureIn, HeapVTR *heapIn,
                          uint32_t *blockTableImage, uint32_t rowPitch,
                          uint32_t depthPitch) override;

    void waitOnFence(Fence *fence, uint64_t fenceValue) override;

    void setRenderTarget(RenderTarget *rtv, DepthStencil *dsv) override;

    NvFlexResult registerD3DBuffer(void *nativeBuffer, NvFlexUint dim,
                                   NvFlexUint structStride, Buffer **ppBuffer) override;

    // Implement details
    static NvFlexDim extractDim(ID3D11Resource *resource);
    static NvFlexDim extractDim(ID3D11View *srv);

    ContextD3D11(const NvFlexContextDescD3D11 *pdesc);
    ContextD3D11(Device *deviceImpl, const NvFlexContextDescD3D11 *pdesc);
    ~ContextD3D11();

    ID3D11Device *getDevice();
    ID3D11DeviceContext *getContext();

    void updateContext(const NvFlexContextDescD3D11 *pdesc);

    void updateContextDesc(NvFlexContextDescD3D11 *desc);

    DepthStencilView *createDepthStencilView(const NvFlexDepthStencilViewDescD3D11 *desc);

    RenderTargetView *createRenderTargetView(const NvFlexRenderTargetViewDescD3D11 *desc);

    ResourceReference *shareResourceReference(ResourceD3D11 *resource);

    void updateDepthStencilView(NvFlexDepthStencilView *view,
                                const NvFlexDepthStencilViewDescD3D11 *desc);

    void updateRenderTargetView(NvFlexRenderTargetView *view,
                                const NvFlexRenderTargetViewDescD3D11 *desc);

    void updateResourceRWViewDesc(ResourceRWD3D11 *resourceRWIn,
                                  NvFlexResourceRWViewDescD3D11 *desc);

    void updateResourceViewDesc(ResourceD3D11 *resourceIn,
                                NvFlexResourceViewDescD3D11 *desc);

    struct State {
        ID3D11Buffer *indexBuffer = nullptr;
        DXGI_FORMAT indexFormat = DXGI_FORMAT_UNKNOWN;
        UINT indexOffset = 0;

        ID3D11InputLayout *layout = nullptr;
        D3D11_PRIMITIVE_TOPOLOGY topology = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;

        VectorCached<ID3D11Buffer *, 16> vertexBuffers;
        VectorCached<UINT, 16> vertexStrides;
        VectorCached<UINT, 16> vertexOffsets;

        ID3D11BlendState *blendState = nullptr;
        FLOAT blendFactor[4] = {0};
        UINT blendSampleMask = 0;

        ID3D11DepthStencilState *depthStencilState = nullptr;
        UINT stencilRef = 0;

        VectorCached<ID3D11RenderTargetView *, 8> renderTargetViews;
        ID3D11DepthStencilView *depthStencilView = nullptr;

        VectorCached<D3D11_RECT, 16> scissorRects;

        ID3D11RasterizerState *rasterizerState = nullptr;

        VectorCached<D3D11_VIEWPORT, 16> viewports;

        VectorCached<ID3D11Buffer *, 8> vsConstantBuffers;
        VectorCached<ID3D11SamplerState *, 8> vsSamplers;
        VectorCached<ID3D11ShaderResourceView *, 16> vsSrvs;

        VectorCached<ID3D11Buffer *, 8> psConstantBuffers;
        VectorCached<ID3D11SamplerState *, 8> psSamplers;
        VectorCached<ID3D11ShaderResourceView *, 16> psSrvs;

        VectorCached<ID3D11Buffer *, 8> csConstantBuffers;
        VectorCached<ID3D11SamplerState *, 8> csSamplers;
        VectorCached<ID3D11ShaderResourceView *, 16> csSrvs;
        VectorCached<ID3D11UnorderedAccessView *, 8> csUavs;

        ID3D11ComputeShader *computeShader = nullptr;

        ID3D11VertexShader *vertexShader = nullptr;
        ID3D11PixelShader *pixelShader = nullptr;
        ID3D11GeometryShader *geometryShader = nullptr;
        ID3D11HullShader *hullShader = nullptr;
        ID3D11DomainShader *domainShader = nullptr;

        void reset();

        State() = default;
        ~State();

     private:
        State(const State &) = delete;
        State &operator=(const State &) = delete;
    };

 protected:
    ID3D11Device *m_device;
    ID3D11DeviceContext *m_deviceContext;
    ID3DUserDefinedAnnotation *m_d3dAnnotation;
    ID3D11SamplerState *m_sampler0;
    ID3D11SamplerState *m_sampler1;
    ID3D11SamplerState *m_sampler2;
    ID3D11SamplerState *m_sampler3;
    ID3D11SamplerState *m_sampler4;
    ID3D11SamplerState *m_sampler5;
    bool m_VTRSupportChecked;
    bool m_VTRSupported;

    VectorCached<D3D11_TILED_RESOURCE_COORDINATE, 1> m_tileCoords;
    VectorCached<D3D11_TILE_REGION_SIZE, 1> m_tileRegionSize;
    VectorCached<D3D11_TILE_RANGE_FLAG, 1> m_rangeFlags;
    VectorCached<uint32_t, 1> m_tilePoolCoords;
    VectorCached<uint32_t, 1> m_tilePoolRangeSize;
    ContextD3D11::State m_state;
};

}  // namespace NvFlex

#endif /* CONTEXTD3D11_H */
