#include "Types.h"
#include "ContextD3D12.h"
#include <nvflex/NvFlexContextD3D12.h>

using namespace NvFlex;

NV_FLEX_API NvFlexContext* NvFlexCreateContextD3D12(NvFlexUint version,
                                                    const NvFlexContextDescD3D12* desc) {
    return new ContextD3D12(desc);
}

NV_FLEX_API NvFlexDepthStencilView* NvFlexCreateDepthStencilViewD3D12(
    NvFlexContext* context, const NvFlexDepthStencilViewDescD3D12* desc) {
    return implSafeCast<ContextD3D12>(context)->createDepthStencilView(desc);
}

NV_FLEX_API NvFlexRenderTargetView* NvFlexCreateRenderTargetViewD3D12(
    NvFlexContext* context, const NvFlexRenderTargetViewDescD3D12* desc) {
    return implSafeCast<ContextD3D12>(context)->createRenderTargetView(desc);
}

NV_FLEX_API void NvFlexUpdateContextD3D12(NvFlexContext* context,
                                          const NvFlexContextDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateContext(desc);
}

NV_FLEX_API void NvFlexUpdateContextDescD3D12(NvFlexContext* context,
                                              NvFlexContextDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateContextDesc(desc);
}

NV_FLEX_API void NvFlexUpdateDepthStencilViewD3D12(
    NvFlexContext* context, NvFlexDepthStencilView* view,
    const NvFlexDepthStencilViewDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateDepthStencilView(view, desc);
}

NV_FLEX_API void NvFlexUpdateRenderTargetViewD3D12(
    NvFlexContext* context, NvFlexRenderTargetView* view,
    const NvFlexRenderTargetViewDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateRenderTargetView(view, desc);
}

NV_FLEX_API void NvFlexUpdateResourceViewDescD3D12(NvFlexContext* context,
                                                   NvFlexResource* resource,
                                                   NvFlexResourceViewDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateResourceViewDesc(
        implSafeCast<ResourceD3D12>(resource), desc);
}

NV_FLEX_API void NvFlexUpdateResourceRWViewDescD3D12(NvFlexContext* context,
                                                     NvFlexResourceRW* resourceRW,
                                                     NvFlexResourceRWViewDescD3D12* desc) {
    implSafeCast<ContextD3D12>(context)->updateResoruceRWViewDesc(
        implSafeCast<ResourceRWD3D12>(resourceRW), desc);
}
