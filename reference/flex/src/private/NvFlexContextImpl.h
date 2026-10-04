#ifndef NVFLEXCONTEXTIMPL_H
#define NVFLEXCONTEXTIMPL_H
#include "NvFlexObjectImpl.h"
#include <nvflex/NvFlexContext.h>
#include <nvflex/NvFlexContextExt.h>

struct NvFlexContextObject : NvFlexObject {};

struct NvFlexConstantBuffer : NvFlexContextObject {};

struct NvFlexVertexBuffer : NvFlexContextObject {};

struct NvFlexIndexBuffer : NvFlexContextObject {};

struct NvFlexDepthStencilView : NvFlexContextObject {};

struct NvFlexRenderTargetView : NvFlexContextObject {};

struct NvFlexResource : NvFlexContextObject {};

struct NvFlexResourceRW : NvFlexContextObject {};

struct NvFlexDepthStencil : NvFlexContextObject {};

struct NvFlexRenderTarget : NvFlexContextObject {};

struct NvFlexBuffer : NvFlexContextObject {};

struct NvFlexTexture1D : NvFlexContextObject {};

struct NvFlexTexture2D : NvFlexContextObject {};

struct NvFlexTexture3D : NvFlexContextObject {};

struct NvFlexTexture2DCrossAdapter : NvFlexContextObject {};

struct NvFlexHeapSparse : NvFlexContextObject {};

struct NvFlexTexture3DSparse : NvFlexContextObject {};

struct NvFlexColorBuffer : NvFlexContextObject {};

struct NvFlexDepthBuffer : NvFlexContextObject {};

struct NvFlexComputeShader : NvFlexContextObject {};

struct NvFlexGraphicsShader : NvFlexContextObject {};

struct NvFlexContextTimer : NvFlexContextObject {};

struct NvFlexContextEventQueue : NvFlexContextObject {};

struct NvFlexResourceReference : NvFlexContextObject {};

struct NvFlexFence : NvFlexContextObject {};

struct NvFlexContext : NvFlexContextObject {
    virtual NvFlexContextAPI getContextType() = 0;

    virtual void flushRequestPush() = 0;

    virtual bool flushRequestPop() = 0;

    virtual void processFenceSignal(NvFlexContext *context) = 0;

    virtual void processFenceWait(NvFlexContext *context) = 0;

    virtual void contextPush() = 0;

    virtual void contextPop() = 0;
};

#endif /* NVFLEXCONTEXTIMPL_H */
