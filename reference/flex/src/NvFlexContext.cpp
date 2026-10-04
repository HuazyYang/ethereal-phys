#include "NvFlexContextImpl.h"
#include "Types.h"
#include "Context.h"
#include "ClientHelper.h"

NV_FLEX_API NvFlexContextAPI NvFlexContextGetContextType(NvFlexContext* context) {
    return context->getContextType();
}

NV_FLEX_API void NvFlexContextFlushRequestPush(NvFlexContext* context) {
    return context->flushRequestPush();
}

NV_FLEX_API bool NvFlexContextFlushRequestPop(NvFlexContext* context) {
    return context->flushRequestPop();
}

NV_FLEX_API void NvFlexContextProcessFenceWait(NvFlexContext* context) {
    return context->processFenceWait(context);
}

NV_FLEX_API void NvFlexContextProcessFenceSignal(NvFlexContext* context) {
    return context->processFenceSignal(context);
}

NV_FLEX_API void NvFlexReleaseContext(NvFlexContext* context) {
    if(context)
        context->release();
}

NV_FLEX_API void NvFlexReleaseDepthStencilView(NvFlexDepthStencilView* view) {
    view->release();
}

NV_FLEX_API void NvFlexReleaseRenderTargetView(NvFlexRenderTargetView* view) {
    view->release();
}

NV_FLEX_API void NvFlexContextPush(NvFlexContext* context) {
    return context->contextPush();
}

NV_FLEX_API void NvFlexContextPop(NvFlexContext* context) {
    return context->contextPop();
}

NV_FLEX_API void NvFlexSetMallocFunc(void* (*malloc)(size_t size)) {
    return NvFlex::FlexSetMallocFunc(malloc);
}

NV_FLEX_API void NvFlexSetFreeFunc(void (*free)(void* ptr)) {
    return NvFlex::FlexSetFreeFunc(free);
}

NV_FLEX_API NvFlexUint NvFlexDeferredRelease(float timeoutMS) {
    return NvFlex::FlexDeferredRelease(timeoutMS);
}

NV_FLEX_API NvFlexUint NvFlexContextObjectAddRef(NvFlexContextObject* object) {
    return object->addRef();
}

NV_FLEX_API NvFlexUint NvFlexContextObjectRelease(NvFlexContextObject* object) {
    return object->release();
}

NV_FLEX_API NvFlexUint64 NvFlexContextObjectGetGPUBytesUsed(NvFlexContextObject* object) {
    return object->getGPUBytesUsed();
}

NV_FLEX_API void NvFlexConstantBufferGetDesc(NvFlexConstantBuffer* buffer,
                                             NvFlexConstantBufferDesc* desc) {
    if (buffer) *desc = NvFlex::implCast<NvFlex::ConstantBuffer>(buffer)->m_desc;
}

NV_FLEX_API NvFlexConstantBuffer* NvFlexCreateConstantBuffer(
    NvFlexContext* context, const NvFlexConstantBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createConstantBuffer(desc);
}

NV_FLEX_API void NvFlexReleaseConstantBuffer(NvFlexConstantBuffer* buffer) {
    if (buffer) buffer->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexConstantBufferGetContextObject(
    NvFlexConstantBuffer* buffer) {
    return buffer;
}

NV_FLEX_API void* NvFlexConstantBufferMap(NvFlexContext* context,
                                          NvFlexConstantBuffer* constantBuffer) {
    if (constantBuffer && context)
        return NvFlex::implCast<NvFlex::Context>(context)->map(
            NvFlex::implCast<NvFlex::ConstantBuffer>(constantBuffer));
    return nullptr;
}

NV_FLEX_API void NvFlexConstantBufferUnmap(NvFlexContext* context,
                                           NvFlexConstantBuffer* constantBuffer) {
    if (context && constantBuffer)
        NvFlex::implCast<NvFlex::Context>(context)->unmap(
            NvFlex::implCast<NvFlex::ConstantBuffer>(constantBuffer));
}

NV_FLEX_API void NvFlexVertexBufferGetDesc(NvFlexVertexBuffer* buffer,
                                           NvFlexVertexBufferDesc* desc) {
    if (buffer) *desc = NvFlex::implCast<NvFlex::VertexBuffer>(buffer)->m_desc;
}

NV_FLEX_API NvFlexVertexBuffer* NvFlexCreateVertexBuffer(
    NvFlexContext* context, const NvFlexVertexBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createVertexBuffer(desc);
}

NV_FLEX_API void NvFlexReleaseVertexBuffer(NvFlexVertexBuffer* vertexBuffer) {
    if (vertexBuffer) vertexBuffer->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexVertexBufferGetContextObject(
    NvFlexVertexBuffer* buffer) {
    return buffer;
}

NV_FLEX_API void* NvFlexVertexBufferMap(NvFlexContext* context,
                                        NvFlexVertexBuffer* vertexBuffer) {
    if (vertexBuffer)
        return NvFlex::implCast<NvFlex::Context>(context)->map(
            NvFlex::implCast<NvFlex::VertexBuffer>(vertexBuffer));
    return nullptr;
}

NV_FLEX_API void NvFlexVertexBufferUnmap(NvFlexContext* context,
                                         NvFlexVertexBuffer* vertexBuffer) {
    if (vertexBuffer)
        NvFlex::implCast<NvFlex::Context>(context)->unmap(
            NvFlex::implCast<NvFlex::VertexBuffer>(vertexBuffer));
}

NV_FLEX_API void NvFlexIndexBufferGetDesc(NvFlexIndexBuffer* index,
                                          NvFlexIndexBufferDesc* desc) {
    if (index) *desc = NvFlex::implCast<NvFlex::IndexBuffer>(index)->m_desc;
}

NV_FLEX_API NvFlexIndexBuffer* NvFlexCreateIndexBuffer(NvFlexContext* context,
                                                       const NvFlexIndexBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createIndexBuffer(desc);
}

NV_FLEX_API void NvFlexReleaseIndexBuffer(NvFlexIndexBuffer* buffer) {
    if (buffer) buffer->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexIndexBufferGetContextObject(
    NvFlexIndexBuffer* buffer) {
    return buffer;
}

NV_FLEX_API void* NvFlexIndexBufferMap(NvFlexContext* context,
                                       NvFlexIndexBuffer* indexBuffer) {
    if (indexBuffer)
        return NvFlex::implCast<NvFlex::Context>(context)->map(
            NvFlex::implCast<NvFlex::IndexBuffer>(indexBuffer));
    return nullptr;
}

NV_FLEX_API void NvFlexIndexBufferUnmap(NvFlexContext* context,
                                        NvFlexIndexBuffer* indexBuffer) {
    if (indexBuffer)
        return NvFlex::implCast<NvFlex::Context>(context)->unmap(
            NvFlex::implCast<NvFlex::IndexBuffer>(indexBuffer));
}

NV_FLEX_API NvFlexContextObject* NvFlexResourceGetContextObject(NvFlexResource* resource) {
    return resource;
}

NV_FLEX_API NvFlexContextObject* NvFlexResourceRWGetContextObject(
    NvFlexResourceRW* resourceRW) {
    return resourceRW;
}

NV_FLEX_API NvFlexResource* NvFlexResourceRWGetResource(NvFlexResourceRW* resourceRW) {
    if (resourceRW) return NvFlex::implCast<NvFlex::ResourceRW>(resourceRW)->getResource();
    return nullptr;
}

NV_FLEX_API void NvFlexRenderTargetGetDesc(NvFlexRenderTarget* rt,
                                           NvFlexRenderTargetDesc* desc) {
    if (rt) {
        auto rt2 = NvFlex::implCast<NvFlex::RenderTarget>(rt);
        desc->rt_format = rt2->m_rt_format;
        desc->viewport = rt2->m_viewport;
    }
}

NV_FLEX_API void NvFlexRenderTargetSetViewport(NvFlexRenderTarget* rt,
                                               const NvFlexViewport* viewport) {
    if (rt) NvFlex::implCast<NvFlex::RenderTarget>(rt)->m_viewport = *viewport;
}

NV_FLEX_API void NvFlexDepthStencilGetDesc(NvFlexDepthStencil* ds,
                                           NvFlexDepthStencilDesc* desc) {
    if (ds) {
        auto ds2 = NvFlex::implCast<NvFlex::DepthStencil>(ds);
        desc->ds_format = ds2->m_ds_format;
        desc->viewport = ds2->m_viewport;
        desc->width = ds2->m_width;
        desc->height = ds2->m_height;
    }
}

NV_FLEX_API void NvFlexDepthStencilSetViewport(NvFlexDepthStencil* ds,
                                               const NvFlexViewport* viewport) {
    if (ds) {
        auto ds2 = NvFlex::implCast<NvFlex::DepthStencil>(ds);
        ds2->m_viewport = *viewport;
    }
}

NV_FLEX_API void NvFlexBufferGetDesc(NvFlexBuffer* buffer, NvFlexBufferDesc* desc) {
    if (buffer) *desc = NvFlex::implCast<NvFlex::Buffer>(buffer)->m_desc;
}

NV_FLEX_API NvFlexBuffer* NvFlexCreateBuffer(NvFlexContext* context,
                                             const NvFlexBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createBuffer(desc);
}

NV_FLEX_API NvFlexBuffer* NvFlexCreateBufferView(NvFlexContext* context,
                                                 NvFlexBuffer* buffer,
                                                 const NvFlexBufferViewDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createBufferView(
        NvFlex::implCast<NvFlex::Buffer>(buffer), desc);
}

NV_FLEX_API void NvFlexReleaseBuffer(NvFlexBuffer* buffer) {
    if (buffer) buffer->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexBufferGetContextObject(NvFlexBuffer* buffer) {
    return buffer;
}

NV_FLEX_API NvFlexResource* NvFlexBufferGetResource(NvFlexBuffer* buffer) {
    return NvFlex::implCast<NvFlex::Buffer>(buffer)->getResource();
}

NV_FLEX_API NvFlexResourceRW* NvFlexBufferGetResourceRW(NvFlexBuffer* buffer) {
    return NvFlex::implCast<NvFlex::Buffer>(buffer)->getResourceRW();
}

NV_FLEX_API void NvFlexTexture1DGetDesc(NvFlexTexture1D* tex, NvFlexTexture1DDesc* desc) {
    if (tex) *desc = NvFlex::implCast<NvFlex::Texture1D>(tex)->m_desc;
}

NV_FLEX_API NvFlexTexture1D* NvFlexCreateTexture1D(NvFlexContext* context,
                                                   const NvFlexTexture1DDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture1D(desc);
}

NV_FLEX_API void NvFlexReleaseTexture1D(NvFlexTexture1D* tex) {
    if (tex) tex->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexTexture1DGetContextObject(NvFlexTexture1D* tex) {
    return tex;
}

NV_FLEX_API NvFlexResource* NvFlexTexture1DGetResource(NvFlexTexture1D* tex) {
    return NvFlex::implCast<NvFlex::Texture1D>(tex)->getResource();
}

NV_FLEX_API NvFlexResourceRW* NvFlexTexture1DGetResourceRW(NvFlexTexture1D* tex) {
    return NvFlex::implCast<NvFlex::Texture1D>(tex)->getResourceRW();
}

NV_FLEX_API void* NvFlexTexture1DMap(NvFlexContext* context, NvFlexTexture1D* tex) {
    if (tex)
        return NvFlex::implCast<NvFlex::Context>(context)->map(
            NvFlex::implCast<NvFlex::Texture1D>(tex));
    return nullptr;
}

NV_FLEX_API void NvFlexTexture1DUnmap(NvFlexContext* context, NvFlexTexture1D* tex) {
    if (tex)
        NvFlex::implCast<NvFlex::Context>(context)->unmap(
            NvFlex::implCast<NvFlex::Texture1D>(tex));
}

NV_FLEX_API void NvFlexTexture2DGetDesc(NvFlexTexture2D* tex, NvFlexTexture2DDesc* desc) {
    if (tex) {
        *desc = NvFlex::implCast<NvFlex::Texture2D>(tex)->m_desc;
    }
}

NV_FLEX_API NvFlexTexture2D* NvFlexCreateTexture2D(NvFlexContext* context,
                                                   const NvFlexTexture2DDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture2D(desc);
}

NV_FLEX_API NvFlexTexture2D* NvFlexShareTexture2D(NvFlexContext* context,
                                                  NvFlexTexture2D* sharedTexture) {
    return NvFlex::implCast<NvFlex::Context>(context)->shareTexture2D(
        NvFlex::implCast<NvFlex::Texture2D>(sharedTexture));
}

NV_FLEX_API NvFlexTexture2D* NvFlexCreateTexture2DCrossAPI(
    NvFlexContext* context, const NvFlexTexture2DDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture2DShared(desc);
}

NV_FLEX_API NvFlexTexture2D* NvFlexShareTexture2DCrossAPI(NvFlexContext* context,
                                                          NvFlexTexture2D* sharedTexture) {
    return NvFlex::implCast<NvFlex::Context>(context)->shareTexture2DShared(
        NvFlex::implCast<NvFlex::Texture2D>(sharedTexture));
}

NV_FLEX_API void NvFlexReleaseTexture2D(NvFlexTexture2D* tex) {
    if (tex) tex->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexTexture2DGetContextObject(NvFlexTexture2D* tex) {
    return tex;
}

NV_FLEX_API NvFlexResource* NvFlexTexture2DGetResource(NvFlexTexture2D* tex) {
    if (tex) return NvFlex::implCast<NvFlex::Texture2D>(tex)->getResource();
    return nullptr;
}

NV_FLEX_API NvFlexResourceRW* NvFlexTexture2DGetResourceRW(NvFlexTexture2D* tex) {
    if (tex) return NvFlex::implCast<NvFlex::Texture2D>(tex)->getResourceRW();
    return nullptr;
}

NV_FLEX_API void NvFlexTexture3DGetDesc(NvFlexTexture3D* tex, NvFlexTexture3DDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::Texture3D>(tex)->m_desc;
}

NV_FLEX_API NvFlexTexture3D* NvFlexCreateTexture3D(NvFlexContext* context,
                                                   const NvFlexTexture3DDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture3D(desc);
}

NV_FLEX_API void NvFlexReleaseTexture3D(NvFlexTexture3D* tex) {
    if (tex) tex->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexTexture3DGetContextObject(NvFlexTexture3D* tex) {
    return tex;
}

NV_FLEX_API NvFlexResource* NvFlexTexture3DGetResource(NvFlexTexture3D* tex) {
    return NvFlex::implCast<NvFlex::Texture3D>(tex)->getResource();
}

NV_FLEX_API NvFlexResourceRW* NvFlexTexture3DGetResourceRW(NvFlexTexture3D* tex) {
    return NvFlex::implCast<NvFlex::Texture3D>(tex)->getResourceRW();
}

NV_FLEX_API NvFlexMappedData NvFlexTexture3DMap(NvFlexContext* context,
                                                NvFlexTexture3D* tex) {
    return NvFlex::implCast<NvFlex::Context>(context)->map(
        NvFlex::implCast<NvFlex::Texture3D>(tex));
}

NV_FLEX_API void NvFlexTexture3DUnmap(NvFlexContext* context, NvFlexTexture3D* tex) {
    NvFlex::implCast<NvFlex::Context>(context)->unmap(
        NvFlex::implCast<NvFlex::Texture3D>(tex));
}

NV_FLEX_API void NvFlexTexture3DDownload(NvFlexContext* context, NvFlexTexture3D* tex) {
    NvFlex::implCast<NvFlex::Context>(context)->download(
        NvFlex::implCast<NvFlex::Texture3D>(tex));
}

NV_FLEX_API NvFlexMappedData NvFlexTexture3DMapDownload(NvFlexContext* context,
                                                        NvFlexTexture3D* tex) {
    return NvFlex::implCast<NvFlex::Context>(context)->mapDownload(
        NvFlex::implCast<NvFlex::Texture3D>(tex));
}

NV_FLEX_API void NvFlexTexture3DUnmapDownload(NvFlexContext* context,
                                              NvFlexTexture3D* tex) {
    NvFlex::implCast<NvFlex::Context>(context)->unmapDownload(
        NvFlex::implCast<NvFlex::Texture3D>(tex));
}

NV_FLEX_API void NvFlexHeapSparseGetDesc(NvFlexHeapSparse* heap,
                                         NvFlexHeapSparseDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::HeapVTR>(heap)->m_desc;
}

NV_FLEX_API NvFlexHeapSparse* NvFlexCreateHeapSparse(NvFlexContext* context,
                                                     const NvFlexHeapSparseDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createHeapVTR(desc);
}

NV_FLEX_API void NvFlexReleaseHeapSparse(NvFlexHeapSparse* heap) {
    if (heap) heap->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexHeapSparseGetContextObject(NvFlexHeapSparse* heap) {
    return heap;
}

NV_FLEX_API void NvFlexTexture3DSparseGetDesc(NvFlexTexture3DSparse* tex,
                                              NvFlexTexture3DSparseDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::Texture3DVTR>(tex)->m_desc;
}

NV_FLEX_API NvFlexTexture3DSparse* NvFlexCreateTexture3DSparse(
    NvFlexContext* context, const NvFlexTexture3DSparseDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture3DVTR(desc);
}

NV_FLEX_API void NvFlexReleaseTexture3DSparse(NvFlexTexture3DSparse* tex) {
    if (tex) tex->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexTexture3DSparseGetContextObject(
    NvFlexTexture3DSparse* tex) {
    return tex;
}

NV_FLEX_API NvFlexResource* NvFlexTexture3DSparseGetResource(NvFlexTexture3DSparse* tex) {
    return NvFlex::implCast<NvFlex::Texture3DVTR>(tex)->getResource();
}

NV_FLEX_API NvFlexResourceRW* NvFlexTexture3DSparseGetResourceRW(
    NvFlexTexture3DSparse* tex) {
    return NvFlex::implCast<NvFlex::Texture3DVTR>(tex)->getResourceRW();
}

NV_FLEX_API void NvFlexColorBufferGetDesc(NvFlexColorBuffer* tex,
                                          NvFlexColorBufferDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::ColorBuffer>(tex)->m_desc;
}

NV_FLEX_API NvFlexColorBuffer* NvFlexCreateColorBuffer(NvFlexContext* context,
                                                       const NvFlexColorBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createColorBuffer(desc);
}

NV_FLEX_API void NvFlexReleaseColorBuffer(NvFlexColorBuffer* tex) {
    if (tex) tex->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexColorBufferGetContextObject(NvFlexColorBuffer* tex) {
    return tex;
}

NV_FLEX_API NvFlexResource* NvFlexColorBufferGetResource(NvFlexColorBuffer* tex) {
    return NvFlex::implCast<NvFlex::ColorBuffer>(tex)->getResource();
}

NV_FLEX_API NvFlexResourceRW* NvFlexColorBufferGetResourceRW(NvFlexColorBuffer* tex) {
    return NvFlex::implCast<NvFlex::ColorBuffer>(tex)->getResourceRW();
}

NV_FLEX_API NvFlexRenderTarget* NvFlexColorBufferGetRenderTarget(NvFlexColorBuffer* tex) {
    return NvFlex::implCast<NvFlex::ColorBuffer>(tex)->getRenderTarget();
}

NV_FLEX_API void NvFlexDepthBufferGetDesc(NvFlexDepthBuffer* depthBuffer,
                                          NvFlexDepthBufferDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::DepthBuffer>(depthBuffer)->m_desc;
}

NV_FLEX_API NvFlexDepthBuffer* NvFlexCreateDepthBuffer(NvFlexContext* context,
                                                       const NvFlexDepthBufferDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createDepthBuffer(desc);
}

NV_FLEX_API void NvFlexReleaseDepthBuffer(NvFlexDepthBuffer* depthBuffer) {
    if (depthBuffer) depthBuffer->release();
}

NV_FLEX_API NvFlexContextObject* NvFlexDepthBufferGetContextObject(
    NvFlexDepthBuffer* depthBuffer) {
    return depthBuffer;
}

NV_FLEX_API NvFlexResource* NvFlexDepthBufferGetResource(NvFlexDepthBuffer* depthBuffer) {
    return NvFlex::implCast<NvFlex::DepthBuffer>(depthBuffer)->getResource();
}

NV_FLEX_API NvFlexDepthStencil* NvFlexDepthBufferGetDepthStencil(
    NvFlexDepthBuffer* depthBuffer) {
    return NvFlex::implCast<NvFlex::DepthBuffer>(depthBuffer)->getDepthStencil();
}

NV_FLEX_API NvFlexResource* NvFlexDepthStencilViewGetResource(NvFlexDepthStencilView* dsv) {
    return NvFlex::implCast<NvFlex::DepthStencilView>(dsv)->getResource();
}

NV_FLEX_API NvFlexDepthStencil* NvFlexDepthStencilViewGetDepthStencil(
    NvFlexDepthStencilView* dsv) {
    return NvFlex::implCast<NvFlex::DepthStencilView>(dsv)->getDepthStencil();
}

NV_FLEX_API void NvFlexDepthStencilViewGetDepthBufferDesc(NvFlexDepthStencilView* dsv,
                                                          NvFlexDepthBufferDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::DepthStencilView>(dsv)->getDepthBufferDesc();
}

NV_FLEX_API NvFlexRenderTarget* NvFlexRenderTargetViewGetRenderTarget(
    NvFlexRenderTargetView* rtv) {
    return NvFlex::implCast<NvFlex::RenderTargetView>(rtv)->getRenderTarget();
}

NV_FLEX_API NvFlexComputeShader* NvFlexCreateComputeShader(
    NvFlexContext* context, const NvFlexComputeShaderDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createComputeShader(desc);
}

NV_FLEX_API void NvFlexReleaseComputeShader(NvFlexComputeShader* computeShader) {
    if (computeShader) computeShader->release();
}

NV_FLEX_API void NvFlexGraphicsShaderGetDesc(NvFlexGraphicsShader* shader,
                                             NvFlexGraphicsShaderDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::GraphicsShader>(shader)->m_desc;
}

NV_FLEX_API NvFlexGraphicsShader* NvFlexCreateGraphicsShader(
    NvFlexContext* context, const NvFlexGraphicsShaderDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createGraphicsShader(desc);
}

NV_FLEX_API void NvFlexReleaseGraphicsShader(NvFlexGraphicsShader* shader) {
    if (shader) shader->release();
}

NV_FLEX_API void NvFlexGraphicsShaderSetFormats(NvFlexContext* context,
                                                NvFlexGraphicsShader* shader,
                                                NvFlexFormat renderTargetFormat,
                                                NvFlexFormat depthStencilFormat) {
    NvFlex::implCast<NvFlex::Context>(context)->setFormats(
        NvFlex::implCast<NvFlex::GraphicsShader>(shader), renderTargetFormat,
        depthStencilFormat);
}

NV_FLEX_API NvFlexContextTimer* NvFlexCreateContextTimer(NvFlexContext* context) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTimer();
}

NV_FLEX_API void NvFlexReleaseContextTimer(NvFlexContextTimer* timer) {
    if (timer) timer->release();
}

NV_FLEX_API NvFlexContextEventQueue* NvFlexCreateContextEventQueue(NvFlexContext* context) {
    return NvFlex::implCast<NvFlex::Context>(context)->createEventQueue();
}

NV_FLEX_API void NvFlexReleaseContextEventQueue(NvFlexContextEventQueue* eventQueue) {
    if (eventQueue) eventQueue->release();
}

NV_FLEX_API void NvFlexContextCopyConstantBuffer(NvFlexContext* context,
                                                 NvFlexConstantBuffer* dst,
                                                 NvFlexBuffer* src) {
    NvFlex::implCast<NvFlex::Context>(context)->copy(
        NvFlex::implCast<NvFlex::ConstantBuffer>(dst),
        NvFlex::implCast<NvFlex::Buffer>(src));
}

NV_FLEX_API void* NvFlexContextMapBuffer(NvFlexContext* context, NvFlexBuffer* buffer,
                                         NvFlexStagingCpuAccessFlags cpuAccessMode,
                                         bool cpuNoWait) {
    return NvFlex::implCast<NvFlex::Context>(context)->map(
        NvFlex::implCast<NvFlex::Buffer>(buffer), cpuAccessMode, cpuNoWait);
}

NV_FLEX_API void NvFlexContextUnmapBuffer(NvFlexContext* context, NvFlexBuffer* buffer) {
    NvFlex::implCast<NvFlex::Context>(context)->unmap(
        NvFlex::implCast<NvFlex::Buffer>(buffer));
}

NV_FLEX_API void NvFlexContextCopyBuffer(NvFlexContext* context, NvFlexBuffer* dst,
                                         NvFlexUint dstOffset, NvFlexBuffer* src,
                                         NvFlexUint srcOffset, NvFlexUint numBytes) {
    NvFlex::implCast<NvFlex::Context>(context)->copy(
        NvFlex::implCast<NvFlex::Buffer>(dst), dstOffset,
        NvFlex::implCast<NvFlex::Buffer>(src), srcOffset, numBytes);
}

NV_FLEX_API void NvFlexContextUploadBuffer(NvFlexContext* context, NvFlexBuffer* dst,
                                           NvFlexUint offsetInBytes, const void* data,
                                           NvFlexUint sizeInBytes) {
    NvFlex::implCast<NvFlex::Context>(context)->upload(
        NvFlex::implCast<NvFlex::Buffer>(dst), offsetInBytes, data, sizeInBytes);
}

NV_FLEX_API void NvFlexContextCopyTexture3D(NvFlexContext* context, NvFlexTexture3D* dst,
                                            NvFlexTexture3D* src) {
    NvFlex::implCast<NvFlex::Context>(context)->copy(
        NvFlex::implCast<NvFlex::Texture3D>(dst), NvFlex::implCast<NvFlex::Texture3D>(src));
}

NV_FLEX_API void NvFlexContextCopyResource(NvFlexContext* context,
                                           NvFlexResourceRW* resourceRW,
                                           NvFlexResource* resource) {
    NvFlex::implCast<NvFlex::Context>(context)->copy(
        NvFlex::implCast<NvFlex::ResourceRW>(resourceRW),
        NvFlex::implCast<NvFlex::Resource>(resource));
}

NV_FLEX_API void NvFlexContextDispatch(NvFlexContext* context,
                                       const NvFlexDispatchParams* params) {
    NvFlex::implCast<NvFlex::Context>(context)->dispatch(params);
}

NV_FLEX_API void NvFlexContextSetVertexBuffer(NvFlexContext* context,
                                              NvFlexVertexBuffer* vertexBuffer,
                                              NvFlexUint stride, NvFlexUint offset) {
    NvFlex::implCast<NvFlex::Context>(context)->setVertexBuffer(
        NvFlex::implCast<NvFlex::VertexBuffer>(vertexBuffer), stride, offset);
}

NV_FLEX_API void NvFlexContextSetIndexBuffer(NvFlexContext* context,
                                             NvFlexIndexBuffer* indexBuffer,
                                             NvFlexUint offset) {
    NvFlex::implCast<NvFlex::Context>(context)->setIndexBuffer(
        NvFlex::implCast<NvFlex::IndexBuffer>(indexBuffer), offset);
}

NV_FLEX_API void NvFlexContextDrawIndexedInstanced(NvFlexContext* context,
                                                   NvFlexUint indicesPerInstance,
                                                   NvFlexUint numInstances,
                                                   const NvFlexDrawParams* params) {
    NvFlex::implCast<NvFlex::Context>(context)->drawIndexedInstanced(indicesPerInstance,
                                                                     numInstances, params);
}

NV_FLEX_API void NvFlexContextSetRenderTarget(NvFlexContext* context,
                                              NvFlexRenderTarget* rt,
                                              NvFlexDepthStencil* ds) {
    NvFlex::implCast<NvFlex::Context>(context)->setRenderTarget(
        NvFlex::implCast<NvFlex::RenderTarget>(rt),
        NvFlex::implCast<NvFlex::DepthStencil>(ds));
}

NV_FLEX_API void NvFlexContextSetViewport(NvFlexContext* context,
                                          const NvFlexViewport* viewport) {
    NvFlex::implCast<NvFlex::Context>(context)->setViewport(viewport);
}

NV_FLEX_API void NvFlexContextClearRenderTarget(NvFlexContext* context,
                                                NvFlexRenderTarget* rt,
                                                const NvFlexFloat4 color) {
    NvFlex::implCast<NvFlex::Context>(context)->clearRenderTarget(
        NvFlex::implCast<NvFlex::RenderTarget>(rt), (float*)&color);
}

NV_FLEX_API void NvFlexContextClearDepthStencil(NvFlexContext* context,
                                                NvFlexDepthStencil* ds, const float depth) {
    NvFlex::implCast<NvFlex::Context>(context)->clearDepthStencil(
        NvFlex::implCast<NvFlex::DepthStencil>(ds), depth);
}

NV_FLEX_API void NvFlexContextRestoreResourceState(NvFlexContext* context,
                                                   NvFlexResource* resource) {
    NvFlex::implCast<NvFlex::Context>(context)->restoreResourceState(
        NvFlex::implCast<NvFlex::Resource>(resource));
}

NV_FLEX_API void NvFlexContextClearState(NvFlexContext* context) {
    NvFlex::implCast<NvFlex::Context>(context)->clearState();
}

NV_FLEX_API bool NvFlexContextIsSparseTextureSupported(NvFlexContext* context) {
    return NvFlex::implCast<NvFlex::Context>(context)->is_VTR_supported();
}

NV_FLEX_API void NvFlexContextUpdateSparseMapping(
    NvFlexContext* context, NvFlexTexture3DSparse* tex, NvFlexHeapSparse* heap,
    NvFlexUint* blockTableImage, NvFlexUint rowPitch, NvFlexUint depthPitch) {
    NvFlex::implCast<NvFlex::Context>(context)->updateVTRMapping(
        NvFlex::implCast<NvFlex::Texture3DVTR>(tex),
        NvFlex::implCast<NvFlex::HeapVTR>(heap), blockTableImage, rowPitch, depthPitch);
}

NV_FLEX_API void NvFlexContextTimerBegin(NvFlexContext* context,
                                         NvFlexContextTimer* timer) {
    NvFlex::implCast<NvFlex::Context>(context)->timerBegin(
        NvFlex::implCast<NvFlex::Timer>(timer));
}

NV_FLEX_API void NvFlexContextTimerEnd(NvFlexContext* context, NvFlexContextTimer* timer) {
    NvFlex::implCast<NvFlex::Context>(context)->timerEnd(
        NvFlex::implCast<NvFlex::Timer>(timer));
}

NV_FLEX_API NvFlexResult NvFlexContextTimerGetResult(NvFlexContext* context,
                                             NvFlexContextTimer* timer, float* timeGPU,
                                             float* timeCPU, NvFlexUint64* gpuStartStamp,
                                             NvFlexUint64* gpuEndStamp,
                                             NvFlexUint64* gpuFreq) {
    return NvFlex::implCast<NvFlex::Context>(context)->timerGetResult(
        NvFlex::implCast<NvFlex::Timer>(timer), timeGPU, timeCPU, gpuStartStamp, gpuEndStamp, gpuFreq);
}

NV_FLEX_API void NvFlexContextEventQueuePush(NvFlexContext* context,
                                             NvFlexContextEventQueue* eventQueue,
                                             NvFlexUint64 uid) {
    NvFlex::implCast<NvFlex::Context>(context)->eventQueuePush(
        NvFlex::implCast<NvFlex::EventQueue>(eventQueue), uid);
}

NV_FLEX_API NvFlexResult NvFlexContextEventQueuePop(NvFlexContext* context,
                                                    NvFlexContextEventQueue* eventQueue,
                                                    NvFlexUint64* pUid) {
    return (NvFlexResult)NvFlex::implCast<NvFlex::Context>(context)->eventQueuePop(
        NvFlex::implCast<NvFlex::EventQueue>(eventQueue), pUid);
}

NV_FLEX_API void NvFlexContextProfileGroupBegin(NvFlexContext* context,
                                                const wchar_t* label) {
    NvFlex::implCast<NvFlex::Context>(context)->profileGroupBegin(label);
}

NV_FLEX_API void NvFlexContextProfileGroupEnd(NvFlexContext* context) {
    NvFlex::implCast<NvFlex::Context>(context)->profileGroupEnd();
}

NV_FLEX_API void NvFlexContextProfileItemBegin(NvFlexContext* context,
                                               const wchar_t* label) {
    NvFlex::implCast<NvFlex::Context>(context)->profileItemBegin(label);
}

NV_FLEX_API void NvFlexContextProfileItemEnd(NvFlexContext* context) {
    NvFlex::implCast<NvFlex::Context>(context)->profileItemEnd();
}

NV_FLEX_API void NvFlexFenceGetDesc(NvFlexFence* fence, NvFlexFenceDesc* desc) {
    *desc = NvFlex::implCast<NvFlex::Fence>(fence)->m_desc;
}

NV_FLEX_API NvFlexFence* NvFlexCreateFence(NvFlexContext* context,
                                           const NvFlexFenceDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createFence(desc);
}

NV_FLEX_API NvFlexFence* NvFlexShareFence(NvFlexContext* context, NvFlexFence* fence) {
    return NvFlex::implCast<NvFlex::Context>(context)->shareFence(
        NvFlex::implCast<NvFlex::Fence>(fence));
}

NV_FLEX_API void NvFlexReleaseFence(NvFlexFence* fence) {
    if (fence) fence->release();
}

NV_FLEX_API void NvFlexContextWaitOnFence(NvFlexContext* context, NvFlexFence* fence,
                                          NvFlexUint64 fenceValue) {
    NvFlex::implCast<NvFlex::Context>(context)->waitOnFence(
        NvFlex::implCast<NvFlex::Fence>(fence), fenceValue);
}

NV_FLEX_API void NvFlexContextSignalFence(NvFlexContext* context, NvFlexFence* fence,
                                          NvFlexUint64 fenceValue) {
    NvFlex::implCast<NvFlex::Context>(context)->signalFence(
        NvFlex::implCast<NvFlex::Fence>(fence), fenceValue);
}

NV_FLEX_API NvFlexTexture2DCrossAdapter* NvFlexCreateTexture2DCrossAdapter(
    NvFlexContext* context, const NvFlexTexture2DDesc* desc) {
    return NvFlex::implCast<NvFlex::Context>(context)->createTexture2DCrossAdapter(desc);
}

NV_FLEX_API NvFlexTexture2DCrossAdapter* NvFlexShareTexture2DCrossAdapter(
    NvFlexContext* context, NvFlexTexture2DCrossAdapter* sharedTexture) {
    return NvFlex::implCast<NvFlex::Context>(context)->shareTexture2DCrossAdapter(
        NvFlex::implCast<NvFlex::Texture2DCrossAdapter>(sharedTexture));
}

NV_FLEX_API void NvFlexReleaseTexture2DCrossAdapter(NvFlexTexture2DCrossAdapter* tex) {
    if (tex) tex->release();
}

NV_FLEX_API void NvFlexContextTransitionToCommonState(NvFlexContext* context,
                                                      NvFlexResource* resource) {
    NvFlex::implCast<NvFlex::Context>(context)->transitionToCommonState(
        NvFlex::implCast<NvFlex::Resource>(resource));
}

NV_FLEX_API void NvFlexContextCopyToTexture2DCrossAdapter(NvFlexContext* context,
                                                          NvFlexTexture2DCrossAdapter* dst,
                                                          NvFlexTexture2D* src,
                                                          NvFlexUint height) {
    NvFlex::implCast<NvFlex::Context>(context)->copyToShared(
        NvFlex::implCast<NvFlex::Texture2DCrossAdapter>(dst),
        NvFlex::implCast<NvFlex::Texture2D>(src), height);
}

NV_FLEX_API void NvFlexContextCopyFromTexture2DCrossAdapter(
    NvFlexContext* context, NvFlexTexture2D* dst, NvFlexTexture2DCrossAdapter* src,
    NvFlexUint height) {
    NvFlex::implCast<NvFlex::Context>(context)->copyFromShared(
        NvFlex::implCast<NvFlex::Texture2D>(dst),
        NvFlex::implCast<NvFlex::Texture2DCrossAdapter>(src), height);
}

NV_FLEX_API NvFlexResourceReference* NvFlexShareResourceReference(
    NvFlexContext* context, NvFlexResource* resource) {
    return NvFlex::implCast<NvFlex::Context>(context)->shareResourceReference(
        NvFlex::implCast<NvFlex::Resource>(resource));
}

NV_FLEX_API void NvFlexReleaseResourceReference(NvFlexResourceReference* resource) {
    if (resource) resource->release();
}
