#include "ContextD3D11.h"

NV_FLEX_API NvFlexContext* NvFlexCreateContextD3D11(NvFlexUint version,
                                                    const NvFlexContextDescD3D11* desc) {
    return new NvFlex::ContextD3D11(desc);
}

NV_FLEX_API NvFlexDepthStencilView* NvFlexCreateDepthStencilViewD3D11(
    NvFlexContext* context, const NvFlexDepthStencilViewDescD3D11* desc) {
    return NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->createDepthStencilView(desc);
}

NV_FLEX_API NvFlexRenderTargetView* NvFlexCreateRenderTargetViewD3D11(
    NvFlexContext* context, const NvFlexRenderTargetViewDescD3D11* desc) {
    return NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->createRenderTargetView(desc);
}

NV_FLEX_API void NvFlexUpdateContextD3D11(NvFlexContext* context,
                                          const NvFlexContextDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateContext(desc);
}

NV_FLEX_API void NvFlexUpdateContextDescD3D11(NvFlexContext* context,
                                              NvFlexContextDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateContextDesc(desc);
}

NV_FLEX_API void NvFlexUpdateDepthStencilViewD3D11(
    NvFlexContext* context, NvFlexDepthStencilView* view,
    const NvFlexDepthStencilViewDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateDepthStencilView(view, desc);
}

NV_FLEX_API void NvFlexUpdateRenderTargetViewD3D11(
    NvFlexContext* context, NvFlexRenderTargetView* view,
    const NvFlexRenderTargetViewDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateRenderTargetView(view, desc);
}

NV_FLEX_API void NvFlexUpdateResourceViewDescD3D11(NvFlexContext* context,
                                                   NvFlexResource* resource,
                                                   NvFlexResourceViewDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateResourceViewDesc(
        NvFlex::implSafeCast<NvFlex::ResourceD3D11>(resource), desc);
}

NV_FLEX_API void NvFlexUpdateResourceRWViewDescD3D11(NvFlexContext* context,
                                                     NvFlexResourceRW* resourceRW,
                                                     NvFlexResourceRWViewDescD3D11* desc) {
    NvFlex::implSafeCast<NvFlex::ContextD3D11>(context)->updateResourceRWViewDesc(
        NvFlex::implSafeCast<NvFlex::ResourceRWD3D11>(resourceRW), desc);
}
