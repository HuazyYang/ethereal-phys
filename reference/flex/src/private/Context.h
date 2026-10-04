#ifndef CONTEXT_H
#define CONTEXT_H
#include "NvFlexContextImpl.h"
#include "Object.h"
#include <dxgi.h>

namespace NvFlex {

struct Device;

uint64_t FlexDeferredRelease(float timeoutMS);

IDXGIFactory1 *getDXGIFactoryD3D(IDXGIAdapter1 *pAdapter1);

struct ConstantBuffer : NvFlexConstantBuffer {
    NvFlexConstantBufferDesc m_desc;
};

struct VertexBuffer : NvFlexVertexBuffer {
    NvFlexVertexBufferDesc m_desc;
};

struct IndexBuffer : NvFlexIndexBuffer {
    NvFlexIndexBufferDesc m_desc;
};

struct Resource : NvFlexResource {};

struct ResourceRW : NvFlexResourceRW {
    virtual Resource *getResource() = 0;
};

struct DepthStencil : NvFlexDepthStencil {
    NvFlexFormat m_ds_format;
    NvFlexViewport m_viewport;
    float m_width;
    float m_height;
};

struct RenderTarget : NvFlexRenderTarget {
    NvFlexFormat m_rt_format;
    NvFlexViewport m_viewport;
};

struct Buffer : NvFlexBuffer {
    NvFlexBufferDesc m_desc;

    virtual NvFlexContext *getContext() = 0;
    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;
};

struct Texture1D : NvFlexTexture1D {
    NvFlexTexture1DDesc m_desc;

    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;
};

struct Texture2D : NvFlexTexture2D {
    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;

    virtual void openSharedHandle(HANDLE *) = 0;
    virtual void closeSharedHandle(HANDLE) = 0;

    NvFlexTexture2DDesc m_desc;
};

struct Texture3D : NvFlexTexture3D {
    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;

    NvFlexTexture3DDesc m_desc;
};

struct Texture2DCrossAdapter : NvFlexTexture2DCrossAdapter {
    NvFlexTexture2DDesc m_desc;
};

struct ResourceReference : NvFlexResourceReference {};

struct HeapVTR : NvFlexHeapSparse {
    NvFlexHeapSparseDesc m_desc;
    uint32_t m_numTiles;
};

struct Texture3DVTR : NvFlexTexture3DSparse {
    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;

    NvFlexTexture3DSparseDesc m_desc;
    NvFlexDim m_blockDim;
    NvFlexDim m_gridDim;
};

struct ColorBuffer : NvFlexColorBuffer {
    virtual Resource *getResource() = 0;
    virtual ResourceRW *getResourceRW() = 0;
    virtual RenderTarget *getRenderTarget() = 0;

    NvFlexColorBufferDesc m_desc;
};

struct DepthBuffer : NvFlexDepthBuffer {
    virtual Resource *getResource() = 0;
    virtual DepthStencil *getDepthStencil() = 0;

    NvFlexDepthBufferDesc m_desc;
};

struct DepthStencilView : NvFlexDepthStencilView {
    virtual Resource *getResource() = 0;
    virtual DepthStencil *getDepthStencil() = 0;
    virtual NvFlexDepthBufferDesc getDepthBufferDesc() = 0;
};

struct RenderTargetView : NvFlexRenderTargetView {
    virtual RenderTarget *getRenderTarget() = 0;
};

struct ComputeShader : NvFlexComputeShader {
    NvFlexComputeShaderDesc m_desc;
};

struct GraphicsShader : NvFlexGraphicsShader {
    NvFlexGraphicsShaderDesc m_desc;
};

struct Timer : NvFlexContextTimer {};

struct EventQueue : NvFlexContextEventQueue {};

struct Fence : NvFlexFence {
    NvFlexFenceDesc m_desc;
};

struct Context : NvFlexContext {
 public:
    uint64_t getGPUBytesUsed() override;  // Default to zero

    void flushRequestPush() override;

    bool flushRequestPop() override;

    virtual ConstantBuffer *createConstantBuffer(const NvFlexConstantBufferDesc *desc) = 0;

    virtual NvFlexMappedData map(Texture3D *buffer) = 0;

    virtual void *map(Buffer *buffer, NvFlexStagingCpuAccessFlags cpuAccessMode,
                      bool cpuNoWait) = 0;

    virtual void unmap(Buffer *buffer) = 0;

    virtual void *map(ConstantBuffer *buffer) = 0;

    virtual void *map(IndexBuffer *buffer) = 0;

    virtual void *map(Texture1D *buffer) = 0;

    virtual void *map(VertexBuffer *buffer) = 0;

    virtual void unmap(ConstantBuffer *buffer) = 0;

    virtual void unmap(IndexBuffer *buffer) = 0;

    virtual void unmap(VertexBuffer *buffer) = 0;

    virtual void unmap(Texture1D *texture) = 0;

    virtual void unmap(Texture3D *texture) = 0;

    virtual void upload(Buffer *buffer, NvFlexUint offset, const void *data,
                        NvFlexUint numBytes) = 0;

    virtual void copy(Buffer *dst, uint32_t dstOffset, Buffer *src, uint32_t srcOffset,
                      uint32_t numBytes) = 0;

    virtual void copy(Buffer *dst, Resource *src, uint32_t offset, uint32_t numBytes) = 0;

    virtual void copy(ConstantBuffer *dst, Buffer *src) = 0;

    virtual void copy(DepthStencil *dst, Resource *src) = 0;

    virtual void copy(ResourceRW *dst, Resource *src) = 0;

    virtual void copy(Texture3D *dst, Texture3D *src) = 0;

    virtual void copy(Texture3D *dst, Resource *src) = 0;

    virtual VertexBuffer *createVertexBuffer(const NvFlexVertexBufferDesc *desc) = 0;

    virtual IndexBuffer *createIndexBuffer(const NvFlexIndexBufferDesc *desc) = 0;

    virtual Buffer *createBuffer(const NvFlexBufferDesc *desc) = 0;

    virtual Buffer *createBufferView(Buffer *buffer, const NvFlexBufferViewDesc *desc) = 0;

    virtual void download(Texture3D *buffer) = 0;

    virtual NvFlexMappedData mapDownload(Texture3D *buffer) = 0;

    virtual void unmapDownload(Texture3D *texture) = 0;

    virtual Texture1D *createTexture1D(const NvFlexTexture1DDesc *desc) = 0;

    virtual Texture2D *createTexture2D(const NvFlexTexture2DDesc *desc) = 0;

    virtual Texture2D *shareTexture2D(Texture2D *sharedTexture) = 0;

    virtual Texture2D *shareTexture2DShared(Texture2D *sharedTexture) = 0;

    virtual Texture2D *createTexture2DShared(const NvFlexTexture2DDesc *desc) = 0;

    virtual Texture3D *createTexture3D(const NvFlexTexture3DDesc *desc) = 0;

    virtual ResourceReference *shareResourceReference(Resource *resource) = 0;

    virtual HeapVTR *createHeapVTR(const NvFlexHeapSparseDesc *desc) = 0;

    virtual Texture3DVTR *createTexture3DVTR(const NvFlexTexture3DSparseDesc *desc) = 0;

    virtual ColorBuffer *createColorBuffer(const NvFlexColorBufferDesc *desc) = 0;

    virtual DepthBuffer *createDepthBuffer(const NvFlexDepthBufferDesc *desc) = 0;

    virtual ComputeShader *createComputeShader(const NvFlexComputeShaderDesc *desc) = 0;

    virtual GraphicsShader *createGraphicsShader(const NvFlexGraphicsShaderDesc *desc) = 0;

    virtual void setFormats(GraphicsShader *graphicsShader, NvFlexFormat renderTargetFormat,
                            NvFlexFormat depthStencilFormat) = 0;

    virtual Texture2DCrossAdapter *createTexture2DCrossAdapter(const NvFlexTexture2DDesc *desc) = 0;

    virtual Texture2DCrossAdapter *shareTexture2DCrossAdapter(
        Texture2DCrossAdapter *sharedTexture) = 0;

    virtual void transitionToCommonState(Resource *resource) = 0;

    virtual void copyFromShared(Texture2D *dstTexture, Texture2DCrossAdapter *sharedTexture,
                                uint32_t height) = 0;

    virtual void copyToShared(Texture2DCrossAdapter *dstSharedTexture,
                              Texture2D *srcTexture, uint32_t height) = 0;

    virtual Fence *createFence(const NvFlexFenceDesc *desc) = 0;

    virtual Fence *shareFence(Fence *fence) = 0;

    virtual void waitOnFence(Fence *fence, uint64_t fenceValue) = 0;

    virtual void signalFence(Fence *fence, uint64_t fenceValue) = 0;

    virtual void dispatch(const NvFlexDispatchParams *params) = 0;

    virtual void setVertexBuffer(VertexBuffer *buffer, uint32_t stride,
                                 uint32_t offset) = 0;

    virtual void setIndexBuffer(IndexBuffer *buffer, uint32_t offset) = 0;

    virtual void drawIndexedInstanced(uint32_t indicesPerInstance, uint32_t numInstances,
                                      const NvFlexDrawParams *params) = 0;

    virtual void setRenderTarget(RenderTarget *rtv, DepthStencil *dsv) = 0;

    virtual void setViewport(const NvFlexViewport *vp) = 0;

    virtual void clearRenderTarget(RenderTarget *rtv, const float color[4]) = 0;

    virtual void clearDepthStencil(DepthStencil *dsv, float depth) = 0;

    virtual void restoreResourceState(Resource *resource) = 0;

    virtual void clearState() = 0;

    virtual int is_VTR_supported() = 0;

    virtual void updateVTRMapping(Texture3DVTR *textureIn, HeapVTR *heapIn,
                                  uint32_t *blockTableImage, uint32_t rowPitch,
                                  uint32_t depthPitch) = 0;

    virtual Timer *createTimer() = 0;

    virtual void timerBegin(Timer *timer) = 0;

    virtual void timerEnd(Timer *timer) = 0;

    virtual NvFlexResult timerGetResult(Timer *timer, float *timeGPU, float *timeCPU, NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp, NvFlexUint64 *gpuFreq) = 0;

    virtual EventQueue *createEventQueue() = 0;

    virtual void eventQueuePush(EventQueue *eventQueueIn, uint64_t uid) = 0;

    virtual NvFlexResult eventQueuePop(EventQueue *eventQueueIn, uint64_t *pUid) = 0;

    virtual void profileGroupBegin(const wchar_t *);
    virtual void profileGroupEnd();

    virtual void profileItemBegin(const wchar_t *);
    virtual void profileItemEnd();

    virtual NvFlexResult registerD3DBuffer(void *nativeBuffer, NvFlexUint dim, NvFlexUint structStride, Buffer **ppBuffer) = 0;

    struct Profiler : NvFlexObject {
        virtual void begin(const wchar_t *label) = 0;
        virtual void end() = 0;
    };

    // details
 protected:
    Context(Device *deviceImpl = nullptr);
    ~Context() = default;
    bool verifyMapAccess(NvFlexMemoryType memType, NvFlexCpuAccessMode cpuAccess,
                         NvFlexStagingCpuAccessFlags *accessFlags);

    Device *m_deviceImpl;
    bool m_flushRequestPending = 0;
    Profiler *m_profiler = nullptr;
};

}  // namespace NvFlex

#endif /* CONTEXT_H */
