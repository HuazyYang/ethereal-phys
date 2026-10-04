#include "ContextD3D11.h"
#include "Types.h"
#include <dxgi1_4.h>
#include <d3d12.h>
#include <nvapi.h>

namespace NvFlex {

struct BlendEnumTableD3D11 {
    BlendEnumTableD3D11() {
        this->table[1] = D3D11_BLEND_ZERO;
        this->table[2] = D3D11_BLEND_ONE;
        this->table[3] = D3D11_BLEND_SRC_ALPHA;
        this->table[4] = D3D11_BLEND_INV_SRC_ALPHA;
        this->table[5] = D3D11_BLEND_DEST_ALPHA;
        this->table[6] = D3D11_BLEND_INV_DEST_ALPHA;
    }

    D3D11_BLEND table[7];
} const g_blendEnumTableD3D11;

struct BlendOpEnumTableD3D11 {
    BlendOpEnumTableD3D11() {
        this->table[1] = D3D11_BLEND_OP_ADD;
        this->table[2] = D3D11_BLEND_OP_SUBTRACT;
        this->table[3] = D3D11_BLEND_OP_REV_SUBTRACT;
        this->table[4] = D3D11_BLEND_OP_MIN;
        this->table[5] = D3D11_BLEND_OP_MAX;
    }

    D3D11_BLEND_OP table[6];
} const g_blendOpEnumTableD3D11;

struct ComparisonTableD3D11 {
    ComparisonTableD3D11() {
        this->table[1] = D3D11_COMPARISON_NEVER;
        this->table[2] = D3D11_COMPARISON_LESS;
        this->table[3] = D3D11_COMPARISON_EQUAL;
        this->table[4] = D3D11_COMPARISON_LESS_EQUAL;
        this->table[5] = D3D11_COMPARISON_GREATER;
        this->table[6] = D3D11_COMPARISON_NOT_EQUAL;
        this->table[7] = D3D11_COMPARISON_GREATER_EQUAL;
        this->table[8] = D3D11_COMPARISON_ALWAYS;
    }

    D3D11_COMPARISON_FUNC table[9];
} g_comparisonTableD3D11;

NvFlexDim ContextD3D11::extractDim(ID3D11Resource *resource) {
    D3D11_RESOURCE_DIMENSION type;
    resource->GetType(&type);
    switch (type) {
        case D3D11_RESOURCE_DIMENSION_BUFFER: {
            ComPtr<ID3D11Buffer> buffer;
            resource->QueryInterface(IID_PPV_ARGS(&buffer));
            D3D11_BUFFER_DESC buffDesc;
            buffer->GetDesc(&buffDesc);
            return {buffDesc.ByteWidth, 0, 0};
        }
        case D3D11_RESOURCE_DIMENSION_TEXTURE1D: {
            ComPtr<ID3D11Texture1D> texture;
            resource->QueryInterface(IID_PPV_ARGS(&texture));
            D3D11_TEXTURE1D_DESC texDesc;
            texture->GetDesc(&texDesc);
            return {texDesc.Width, texDesc.ArraySize, 0};
        }
        case D3D11_RESOURCE_DIMENSION_TEXTURE2D: {
            ComPtr<ID3D11Texture2D> texture;
            resource->QueryInterface(IID_PPV_ARGS(&texture));
            D3D11_TEXTURE2D_DESC texDesc;
            texture->GetDesc(&texDesc);
            return {texDesc.Width, texDesc.Height, texDesc.ArraySize};
        }
        case D3D11_RESOURCE_DIMENSION_TEXTURE3D: {
            ComPtr<ID3D11Texture3D> texture;
            resource->QueryInterface(IID_PPV_ARGS(&texture));
            D3D11_TEXTURE3D_DESC texDesc;
            texture->GetDesc(&texDesc);
            return {texDesc.Width, texDesc.Height, texDesc.Depth};
        }

        default:
            return {0, 0, 0};
    }
}

NvFlexDim ContextD3D11::extractDim(ID3D11View *srv) {
    ComPtr<ID3D11Resource> resource;
    srv->GetResource(&resource);
    return extractDim(resource.Get());
}

ContextD3D11::ContextD3D11(const NvFlexContextDescD3D11 *pdesc): ContextD3D11{nullptr, pdesc} {}

ContextD3D11::ContextD3D11(Device *deviceImpl, const NvFlexContextDescD3D11 *pdesc)
    : Context{deviceImpl}, m_device{0},
      m_deviceContext{0},
      m_d3dAnnotation{0},
      m_sampler0{0},
      m_sampler1{0},
      m_sampler2{0},
      m_sampler3{0},
      m_sampler4{0},
      m_sampler5{0},
      m_VTRSupportChecked{0},
      m_VTRSupported{0},
      m_tileCoords{},
      m_tileRegionSize{},
      m_rangeFlags{},
      m_tilePoolCoords{},
      m_tilePoolRangeSize{},
      m_state{} {
    updateContext(pdesc);

    auto createSampler = [this](D3D11_FILTER filter, D3D11_TEXTURE_ADDRESS_MODE mode =
                                                         D3D11_TEXTURE_ADDRESS_CLAMP) {
        D3D11_SAMPLER_DESC samplerDesc = {filter,
                                          mode,
                                          mode,
                                          mode,
                                          0.f,
                                          0,
                                          D3D11_COMPARISON_NEVER,
                                          {0.f, 0.f, 0.f, 0.f},
                                          0,
                                          D3D11_FLOAT32_MAX};
        ID3D11SamplerState* sampler;
        m_device->CreateSamplerState(&samplerDesc, &sampler);
        return sampler;
    };

    m_sampler0 =
        createSampler(D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT, D3D11_TEXTURE_ADDRESS_BORDER);
    m_sampler1 =
        createSampler(D3D11_FILTER_MIN_MAG_MIP_POINT, D3D11_TEXTURE_ADDRESS_BORDER);
    m_sampler2 =
        createSampler(D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT, D3D11_TEXTURE_ADDRESS_WRAP);
    m_sampler3 = createSampler(D3D11_FILTER_MIN_MAG_MIP_POINT, D3D11_TEXTURE_ADDRESS_WRAP);
    m_sampler4 =
        createSampler(D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT, D3D11_TEXTURE_ADDRESS_CLAMP);
    m_sampler5 = createSampler(D3D11_FILTER_MIN_MAG_MIP_POINT, D3D11_TEXTURE_ADDRESS_CLAMP);
}

ContextD3D11::~ContextD3D11() {
    SafeRelease(m_d3dAnnotation);
    SafeRelease(m_sampler0);
    SafeRelease(m_sampler1);
    SafeRelease(m_sampler2);
    SafeRelease(m_sampler3);
    SafeRelease(m_sampler4);
    SafeRelease(m_sampler5);
}

ID3D11Device *ContextD3D11::getDevice() {
    return m_device;
}

ID3D11DeviceContext *ContextD3D11::getContext() {
    return m_deviceContext;
}

NvFlexContextAPI ContextD3D11::getContextType() {
    return eNvFlexContextD3D11;
}

void ContextD3D11::clearDepthStencil(DepthStencil *dsv, float depth) {
    auto dsvd3d11 = implCast<DepthStencilD3D11>(dsv);
    m_deviceContext->ClearDepthStencilView(dsvd3d11->m_dsv.Get(), D3D11_CLEAR_DEPTH, depth,
                                           0);
}

void ContextD3D11::clearRenderTarget(RenderTarget *rtv, const float color[4]) {
    auto rtvd3d11 = implCast<RenderTargetD3D11>(rtv);
    m_deviceContext->ClearRenderTargetView(rtvd3d11->m_rtv.Get(), color);
}

void ContextD3D11::contextPush() {
    auto context = m_deviceContext;
    context->IAGetIndexBuffer(&m_state.indexBuffer, &m_state.indexFormat, &m_state.indexOffset);
    context->IAGetInputLayout(&m_state.layout);
    context->IAGetPrimitiveTopology(&m_state.topology);

    m_state.vertexBuffers.resize(16);
    m_state.vertexStrides.resize(16);
    m_state.vertexOffsets.resize(16);

    context->IAGetVertexBuffers(0, 16, m_state.vertexBuffers.data(),
                            m_state.vertexStrides.data(), m_state.vertexOffsets.data());

    context->OMGetBlendState(&m_state.blendState, m_state.blendFactor,
                         &m_state.blendSampleMask);

    context->OMGetDepthStencilState(&m_state.depthStencilState, &m_state.stencilRef);

    m_state.renderTargetViews.resize(8);
    context->OMGetRenderTargets(8, m_state.renderTargetViews.data(), &m_state.depthStencilView);

    m_state.scissorRects.reserve(16);
    UINT numScissorRects = 0;
    context->RSGetScissorRects(&numScissorRects, nullptr);
    m_state.scissorRects.resize(numScissorRects);
    context->RSGetScissorRects(&numScissorRects, m_state.scissorRects.data());

    context->RSGetState(&m_state.rasterizerState);

    m_state.viewports.reserve(16);
    UINT numViewports = 0;
    context->RSGetViewports(&numViewports, nullptr);
    m_state.viewports.resize(numViewports);
    context->RSGetViewports(&numViewports, m_state.viewports.data());

    m_state.vsConstantBuffers.resize(8);
    context->VSGetConstantBuffers(0, 8, m_state.vsConstantBuffers.data());
    m_state.vsSamplers.resize(8);
    context->VSGetSamplers(0, 8, m_state.vsSamplers.data());
    m_state.vsSrvs.resize(16);
    context->VSGetShaderResources(0, 16, m_state.vsSrvs.data());

    m_state.psConstantBuffers.resize(8);
    context->PSGetConstantBuffers(0, 8, m_state.psConstantBuffers.data());
    m_state.psSamplers.resize(8);
    context->PSGetSamplers(0, 8, m_state.psSamplers.data());
    m_state.psSrvs.resize(16);
    context->PSGetShaderResources(0, 16, m_state.psSrvs.data());

    m_state.csConstantBuffers.resize(8);
    context->CSGetConstantBuffers(0, 8, m_state.csConstantBuffers.data());
    m_state.csSamplers.resize(8);
    context->CSGetSamplers(0, 8, m_state.csSamplers.data());
    m_state.csSrvs.resize(NV_FLEX_DISPATCH_MAX_READ_TEXTURES);
    context->CSGetShaderResources(0, NV_FLEX_DISPATCH_MAX_READ_TEXTURES,
                                  m_state.csSrvs.data());
    m_state.csUavs.resize(NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES);
    context->CSGetUnorderedAccessViews(
        0, NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES, m_state.csUavs.data());

    context->CSGetShader(&m_state.computeShader, nullptr, nullptr);
    context->VSGetShader(&m_state.vertexShader, nullptr, nullptr);
    context->PSGetShader(&m_state.pixelShader, nullptr, nullptr);
    context->GSGetShader(&m_state.geometryShader, nullptr, nullptr);
    context->HSGetShader(&m_state.hullShader, nullptr, nullptr);
    context->DSGetShader(&m_state.domainShader, nullptr, nullptr);

    //
    context->RSSetState(nullptr);
    context->OMSetBlendState(nullptr, nullptr, UINT(-1));
    context->OMSetDepthStencilState(nullptr, 0);

    context->CSSetShader(nullptr, nullptr, 0);
    context->VSSetShader(nullptr, nullptr, 0);
    context->PSSetShader(nullptr, nullptr, 0);
    context->GSSetShader(nullptr, nullptr, 0);
    context->HSSetShader(nullptr, nullptr, 0);
    context->DSSetShader(nullptr, nullptr, 0);
}

void ContextD3D11::contextPop() {
    auto context = m_deviceContext;
    context->IASetInputLayout(m_state.layout);
    context->IASetPrimitiveTopology(m_state.topology);
    context->IASetVertexBuffers(0, m_state.vertexBuffers.size(), m_state.vertexBuffers.data(),
                            m_state.vertexStrides.data(), m_state.vertexOffsets.data());
    context->OMSetBlendState(m_state.blendState, m_state.blendFactor, m_state.blendSampleMask);
    context->OMSetDepthStencilState(m_state.depthStencilState, m_state.stencilRef);
    context->OMSetRenderTargets(8, m_state.renderTargetViews.data(), m_state.depthStencilView);
    context->RSSetScissorRects(m_state.scissorRects.size(), m_state.scissorRects.data());
    context->RSSetState(m_state.rasterizerState);
    context->RSSetViewports(m_state.viewports.size(), m_state.viewports.data());

    context->VSSetConstantBuffers(0, 8, m_state.vsConstantBuffers.data());
    context->VSSetSamplers(0, 8, m_state.vsSamplers.data());
    context->VSSetShaderResources(0, 16, m_state.vsSrvs.data());

    context->PSSetConstantBuffers(0, 8, m_state.psConstantBuffers.data());
    context->PSSetSamplers(0, 8, m_state.psSamplers.data());
    context->PSSetShaderResources(0, 16, m_state.psSrvs.data());

    context->CSSetConstantBuffers(0, 8, m_state.csConstantBuffers.data());
    context->CSSetSamplers(0, 8, m_state.csSamplers.data());
    context->CSSetShaderResources(0, NV_FLEX_DISPATCH_MAX_READ_TEXTURES, m_state.csSrvs.data());
    context->CSSetUnorderedAccessViews(0, NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES, m_state.csUavs.data(), nullptr);

    context->CSSetShader(m_state.computeShader, nullptr, 0);
    context->VSSetShader(m_state.vertexShader, nullptr, 0);
    context->HSSetShader(m_state.hullShader, nullptr, 0);
    context->DSSetShader(m_state.domainShader, nullptr, 0);
    context->GSSetShader(m_state.geometryShader, nullptr, 0);
    context->PSSetShader(m_state.pixelShader, nullptr, 0);

    m_state.reset();
}

void ContextD3D11::copy(Buffer *dst, uint32_t dstOffset, Buffer *src, uint32_t srcOffset,
                        uint32_t numBytes) {
    auto dstBufD3D11 = implCast<BufferD3D11>(dst);
    auto srcBufD3D11 = implCast<BufferD3D11>(src);
    D3D11_BOX box = {};
    box.left = srcOffset;
    box.right = srcOffset + numBytes;
    box.top = 0;
    box.bottom = 1;
    box.front = 0;
    box.back = 1;

    if (numBytes)
        m_deviceContext->CopySubresourceRegion(dstBufD3D11->m_buffer.Get(), 0, dstOffset, 0, 0,
                                               srcBufD3D11->m_buffer.Get(), 0, &box);
}

void ContextD3D11::copy(Buffer *dst, Resource *src, uint32_t offset, uint32_t numBytes) {
    auto dstBufD3D11 = implCast<BufferD3D11>(dst);
    auto srcBufD3D11 = implCast<ResourceD3D11>(src);
    D3D11_BOX box = {};
    box.left = offset;
    box.right = offset + numBytes;
    box.top = 0;
    box.bottom = 1;
    box.front = 0;
    box.back = 1;

    ComPtr<ID3D11Resource> srcResource;
    srcBufD3D11->m_srv->GetResource(&srcResource);

    if (numBytes)
        m_deviceContext->CopySubresourceRegion(dstBufD3D11->m_buffer.Get(), 0, offset, 0, 0,
                                               srcResource.Get(), 0, &box);
}

void ContextD3D11::copy(ConstantBuffer *dstIn, Buffer *srcIn) {
    auto dst = implCast<ConstantBufferD3D11>(dstIn);
    auto src = implCast<BufferD3D11>(srcIn);
    m_deviceContext->CopyResource(dst->m_buffer.Get(), src->m_buffer.Get());
}

void ContextD3D11::copy(DepthStencil *dstIn, Resource *srcIn) {
    auto dst = implCast<DepthStencilD3D11>(dstIn);
    auto src = implCast<ResourceD3D11>(srcIn);
    ComPtr<ID3D11Resource> srcResource, dstResource;
    dst->m_dsv->GetResource(&dstResource);
    src->m_srv->GetResource(&srcResource);
    m_deviceContext->CopyResource(dstResource.Get(), srcResource.Get());
}

void ContextD3D11::copy(ResourceRW *dstIn, Resource *srcIn) {
    auto dst = implCast<ResourceRWD3D11>(dstIn);
    auto src = implCast<ResourceD3D11>(srcIn);
    ComPtr<ID3D11Resource> srcResource, dstResource;
    dst->m_srv->GetResource(&dstResource);
    src->m_srv->GetResource(&srcResource);
    m_deviceContext->CopyResource(dstResource.Get(), srcResource.Get());
}

void ContextD3D11::copy(Texture3D *dstIn, Texture3D *srcIn) {
    auto dst = implCast<Texture3DD3D11>(dstIn);
    auto src = implCast<Texture3DD3D11>(srcIn);
    m_deviceContext->CopyResource(dst->m_texture.Get(), src->m_texture.Get());
}

void ContextD3D11::copy(Texture3D *dstIn, Resource *srcIn) {
    auto dst = implCast<Texture3DD3D11>(dstIn);
    auto src = implCast<ResourceD3D11>(srcIn);
    ComPtr<ID3D11Resource> srcResource;
    src->m_srv->GetResource(&srcResource);
    m_deviceContext->CopyResource(dst->m_texture.Get(), srcResource.Get());
}

void ContextD3D11::copyFromShared(Texture2D *dstTexture,
                                  Texture2DCrossAdapter *sharedTexture, uint32_t height) {}

void ContextD3D11::copyToShared(Texture2DCrossAdapter *dstSharedTexture,
                                Texture2D *srcTexture, uint32_t height) {
    return;
}

Buffer *ContextD3D11::createBuffer(const NvFlexBufferDesc *desc) {
    return new BufferD3D11(this, desc);
}

Buffer *ContextD3D11::createBufferView(Buffer *buffer, const NvFlexBufferViewDesc *desc) {
    return new BufferD3D11(this, implCast<BufferD3D11>(buffer), desc);
}

ColorBuffer *ContextD3D11::createColorBuffer(const NvFlexColorBufferDesc *desc) {
    return new ColorBufferD3D11(this, desc);
}

ComputeShader *ContextD3D11::createComputeShader(const NvFlexComputeShaderDesc *desc) {
    return new ComputeShaderD3D11(this, desc);
}

ConstantBuffer *ContextD3D11::createConstantBuffer(const NvFlexConstantBufferDesc *desc) {
    return new ConstantBufferD3D11(this, desc);
}

DepthBuffer *ContextD3D11::createDepthBuffer(const NvFlexDepthBufferDesc *desc) {
    return new DepthBufferD3D11(this, desc);
}

EventQueue *ContextD3D11::createEventQueue() {
    return new EventQueueD3D11(this);
}

Fence *ContextD3D11::createFence(const NvFlexFenceDesc *desc) {
    return nullptr;
}

GraphicsShader *ContextD3D11::createGraphicsShader(const NvFlexGraphicsShaderDesc *desc) {
    return new GraphicsShaderD3D11(this, desc);
}

HeapVTR *ContextD3D11::createHeapVTR(const NvFlexHeapSparseDesc *desc) {
    return new HeapVTRD3D11(this, desc);
}

IndexBuffer *ContextD3D11::createIndexBuffer(const NvFlexIndexBufferDesc *desc) {
    return new IndexBufferD3D11(this, desc);
}

Texture1D *ContextD3D11::createTexture1D(const NvFlexTexture1DDesc *desc) {
    return new Texture1DD3D11(this, desc);
}

Texture2D *ContextD3D11::createTexture2D(const NvFlexTexture2DDesc *desc) {
    return new Texture2DD3D11(this, desc, false);
}

Texture2DCrossAdapter *ContextD3D11::createTexture2DCrossAdapter(
    const NvFlexTexture2DDesc *desc) {
    return nullptr;
}

Texture2D *ContextD3D11::createTexture2DShared(const NvFlexTexture2DDesc *desc) {
    return new Texture2DD3D11(this, desc, true);
}

Texture3D *ContextD3D11::createTexture3D(const NvFlexTexture3DDesc *desc) {
    return new Texture3DD3D11(this, desc);
}

Texture3DVTR *ContextD3D11::createTexture3DVTR(const NvFlexTexture3DSparseDesc *desc) {
    return new Texture3DVTRD3D11(this, desc);
}

ResourceReference *ContextD3D11::shareResourceReference(Resource *resource) {
    return new ResourceReferenceD3D11(this, implCast<ResourceD3D11>(resource));
}

Timer *ContextD3D11::createTimer() {
    return new TimerD3D11(this);
}

VertexBuffer *ContextD3D11::createVertexBuffer(const NvFlexVertexBufferDesc *desc) {
    return new VertexBufferD3D11(this, desc);
}

void ContextD3D11::dispatch(const NvFlexDispatchParams *params) {
    auto shader = implCast<ComputeShaderD3D11>(params->shader);
    profileItemBegin(shader->m_desc.label);

    if (params->gridDim.x && params->gridDim.y && params->gridDim.z) {
        constexpr UINT maxReadSlots = NV_FLEX_DISPATCH_MAX_READ_TEXTURES;
        constexpr UINT maxWriteSlots = NV_FLEX_DISPATCH_MAX_WRITE_TEXTURES;
        ID3D11ShaderResourceView *srvs[maxReadSlots];
        ID3D11UnorderedAccessView *uavs[maxWriteSlots];

        auto context = getContext();
        context->CSSetShader(shader->m_cs.Get(), nullptr, 0);

        ZeroMemory(srvs, sizeof(srvs));
        ZeroMemory(uavs, sizeof(uavs));
        for (int i = 0; i < maxReadSlots; ++i) {
            auto r = implCast<ResourceD3D11>(params->readOnly[i]);
            if (r) srvs[i] = r->m_srv.Get();
        }

        for (int i = 0; i < maxWriteSlots; ++i) {
            auto r = implCast<ResourceRWD3D11>(params->readWrite[i]);
            if (r) uavs[i] = r->m_uav.Get();
        }

        context->CSSetShaderResources(0, maxReadSlots, srvs);
        context->CSSetUnorderedAccessViews(0, maxWriteSlots, uavs, nullptr);

        if (params->rootConstantBuffer) {
            auto cbv =
                implCast<ConstantBufferD3D11>(params->rootConstantBuffer)->m_buffer.Get();
            context->CSSetConstantBuffers(0, 1, &cbv);
        }
        if (params->secondConstantBuffer) {
            auto cbv =
                implCast<ConstantBufferD3D11>(params->secondConstantBuffer)->m_buffer.Get();
            context->CSSetConstantBuffers(1, 1, &cbv);
        }

        context->CSSetSamplers(0, 6, &m_sampler0);

        if (m_d3dAnnotation)
            m_d3dAnnotation->BeginEvent(shader->m_desc.label);
        context->Dispatch(params->gridDim.x, params->gridDim.y, params->gridDim.z);
        if (m_d3dAnnotation)
            m_d3dAnnotation->EndEvent();

        // Reset SRVS and UAVS
        ZeroMemory(srvs, sizeof(srvs));
        ZeroMemory(uavs, sizeof(uavs));
        context->CSSetShaderResources(0, maxReadSlots, srvs);
        context->CSSetUnorderedAccessViews(0, maxWriteSlots, uavs, nullptr);
    }
    profileItemEnd();
}

void ContextD3D11::download(Texture3D *buffer) {
    auto texD3D11 = implCast<Texture3DD3D11>(buffer);
    if (buffer->m_desc.downloadAccess) {
        m_deviceContext->CopyResource(texD3D11->m_downloadTexture.Get(),
                                      texD3D11->m_texture.Get());
    }
}

void ContextD3D11::drawIndexedInstanced(uint32_t indicesPerInstance, uint32_t numInstances,
                                        const NvFlexDrawParams *params) {
    auto shader = implCast<GraphicsShaderD3D11>(params->shader);
    profileItemBegin(shader->m_desc.label);

    constexpr UINT maxSlots = 16;
    const UINT maxWriteSlots = 1;
    BOOL isTargetUAV = FALSE;
    auto context = getContext();
    auto vs = shader->m_vs.Get();
    auto ps = shader->m_ps.Get();
    context->IASetInputLayout(shader->m_inputLayout.Get());
    context->OMSetBlendState(shader->m_blenderState.Get(), nullptr, ~0);
    context->OMSetDepthStencilState(shader->m_depthStencilState.Get(), 0);
    if (params->frontCounterClockwise)
        context->RSSetState(shader->m_rasterizerStateRH.Get());
    else
        context->RSSetState(shader->m_rasterizerStateLH.Get());

    context->VSSetShader(vs, nullptr, 0);
    context->PSSetShader(ps, nullptr, 0);

    if (shader->m_desc.lineList)
        context->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    else
        context->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    ID3D11ShaderResourceView *vs_srvs[maxSlots];
    ID3D11ShaderResourceView *ps_srvs[maxSlots];
    ID3D11UnorderedAccessView *ps_uavs[maxWriteSlots];

    ZeroMemory(vs_srvs, sizeof(vs_srvs));
    ZeroMemory(ps_srvs, sizeof(ps_srvs));

    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D11>(params->vs_readOnly[i]);
        if (r) vs_srvs[i] = r->m_srv.Get();
    }
    for (int i = 0; i < maxSlots; ++i) {
        auto r = implCast<ResourceD3D11>(params->ps_readOnly[i]);
        if (r) ps_srvs[i] = r->m_srv.Get();
    }
    for (int i = 0; i < maxWriteSlots; ++i) {
        auto r = implCast<ResourceRWD3D11>(params->ps_readWrite[i]);
        if (r) {
            ps_uavs[i] = r->m_uav.Get();
            isTargetUAV = TRUE;
        }
    }

    context->VSSetShaderResources(0, maxSlots, vs_srvs);
    context->PSSetShaderResources(0, maxSlots, ps_srvs);
    if (isTargetUAV)
        context->OMSetRenderTargetsAndUnorderedAccessViews(0, nullptr, 0, 0, maxWriteSlots,
                                                           ps_uavs, nullptr);

    if (params->rootConstantBuffer) {
        auto rootConstantBuffer = implCast<ConstantBufferD3D11>(params->rootConstantBuffer);
        auto cbv = rootConstantBuffer->m_buffer.Get();
        context->VSSetConstantBuffers(0, 1, &cbv);
        context->PSSetConstantBuffers(0, 1, &cbv);
    }

    context->VSSetSamplers(0, 6, &m_sampler0);
    context->PSSetSamplers(0, 6, &m_sampler0);

    if (m_d3dAnnotation)
        m_d3dAnnotation->BeginEvent(shader->m_desc.label);
    context->DrawIndexedInstanced(indicesPerInstance, numInstances, 0, 0, 0);
    if (m_d3dAnnotation)
        m_d3dAnnotation->EndEvent();

    ZeroMemory(ps_srvs, sizeof(ps_srvs));
    ZeroMemory(ps_uavs, sizeof(ps_uavs));
    context->VSSetShaderResources(0, maxSlots, ps_srvs);
    context->PSSetShaderResources(0, maxSlots, ps_srvs);
    if (isTargetUAV)
        context->OMSetRenderTargetsAndUnorderedAccessViews(0, nullptr, 0, 0, 1, ps_uavs,
                                                           nullptr);

    profileItemEnd();
}

void ContextD3D11::eventQueuePush(EventQueue *eventQueueIn, uint64_t uid) {
    auto eventQueue = implCast<EventQueueD3D11>(eventQueueIn);
    eventQueue->push(uid, this);
}

NvFlexResult ContextD3D11::eventQueuePop(EventQueue *eventQueueIn, uint64_t *pUid) {
    auto eventQueue = implCast<EventQueueD3D11>(eventQueueIn);
    return eventQueue->pop(pUid, this);
}

int ContextD3D11::is_VTR_supported() {
    auto checkVTR = [](ID3D11Device *device) {
        D3D11_FEATURE_DATA_D3D11_OPTIONS2 options;
        HRESULT hr = device->CheckFeatureSupport(D3D11_FEATURE_D3D11_OPTIONS2, &options,
                                                 sizeof(options));
        return SUCCEEDED(hr) && options.TiledResourcesTier >= D3D11_TILED_RESOURCES_TIER_3;
    };

    return checkVTR(getDevice());
}

NvFlexMappedData ContextD3D11::map(Texture3D *buffer) {
    auto texD3D11 = implCast<Texture3DD3D11>(buffer);
    NvFlexMappedData result;
    if (texD3D11->m_desc.uploadAccess) {
        D3D11_MAPPED_SUBRESOURCE mapped;
        getContext()->Map(texD3D11->m_uploadTexture.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0,
                          &mapped);
        memcpy(&result, &mapped, sizeof(mapped));
    } else {
        ZeroMemory(&result, sizeof(NvFlexMappedData));
    }
    return result;
}

void *ContextD3D11::map(Buffer *buffer, NvFlexStagingCpuAccessFlags cpuAccessMode,
                        bool cpuNoWait) {
    auto bufD3D11 = implCast<BufferD3D11>(buffer);

    // sanity check
    if(!verifyMapAccess(bufD3D11->m_desc.memType, bufD3D11->m_desc.cpuAccessMode, &cpuAccessMode))
        return nullptr;

    HRESULT hr;
    UINT mapAccess = 0;
    UINT flags = 0;
    if(bufD3D11->m_desc.memType == eNvFlexMemoryType_upload) {
        mapAccess = D3D11_MAP_WRITE_DISCARD;
    } else {
        if(cpuAccessMode & eNvFlexStagingCpuAccess_write)
            mapAccess |= D3D11_MAP_WRITE;
        if(cpuAccessMode & eNvFlexStagingCpuAccess_read)
            mapAccess |= D3D11_MAP_READ;

        if(cpuNoWait)
            flags = D3D11_MAP_FLAG_DO_NOT_WAIT;
    }

    D3D11_MAPPED_SUBRESOURCE mapped = {};
    hr = getContext()->Map(bufD3D11->m_buffer.Get(), 0, (D3D11_MAP)mapAccess, flags, &mapped);
    return mapped.pData;
}

void ContextD3D11::unmap(Buffer *buffer) {
    auto bufD3D11 = implCast<BufferD3D11>(buffer);

    if (!verifyMapAccess(bufD3D11->m_desc.memType, bufD3D11->m_desc.cpuAccessMode, nullptr))
        return;

    getContext()->Unmap(bufD3D11->m_buffer.Get(), 0);
}

void *ContextD3D11::map(ConstantBuffer *buffer) {
    auto bufD3D11 = implCast<ConstantBufferD3D11>(buffer);
    D3D11_MAPPED_SUBRESOURCE mapped;
    getContext()->Map(bufD3D11->m_buffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
    return mapped.pData;
}

void *ContextD3D11::map(IndexBuffer *buffer) {
    auto bufD3D11 = implCast<IndexBufferD3D11>(buffer);
    D3D11_MAPPED_SUBRESOURCE mapped;
    getContext()->Map(bufD3D11->m_uploadBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0,
                      &mapped);
    return mapped.pData;
}

void *ContextD3D11::map(Texture1D *buffer) {
    auto bufD3D11 = implCast<Texture1DD3D11>(buffer);
    if (!bufD3D11->m_desc.uploadAccess) return nullptr;

    D3D11_MAPPED_SUBRESOURCE mapped;
    getContext()->Map(bufD3D11->m_uploadTexture.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0,
                      &mapped);
    return mapped.pData;
}

void *ContextD3D11::map(VertexBuffer *buffer) {
    auto bufD3D11 = implCast<VertexBufferD3D11>(buffer);
    D3D11_MAPPED_SUBRESOURCE mapped;
    getContext()->Map(bufD3D11->m_uploadBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0,
                      &mapped);
    return mapped.pData;
}

NvFlexMappedData ContextD3D11::mapDownload(Texture3D *buffer) {
    NvFlexMappedData result = {};
    auto texD3D11 = implCast<Texture3DD3D11>(buffer);
    if (texD3D11->m_desc.downloadAccess) {
        D3D11_MAPPED_SUBRESOURCE mapped;
        getContext()->Map(texD3D11->m_downloadTexture.Get(), 0, D3D11_MAP_READ,
                          D3D11_MAP_FLAG_DO_NOT_WAIT, &mapped);
        memcpy(&result, &mapped, sizeof(mapped));
    }
    return result;
}

void ContextD3D11::processFenceSignal(NvFlexContext *context) {}

void ContextD3D11::processFenceWait(NvFlexContext *context) {}

void ContextD3D11::restoreResourceState(Resource *resource) {}

void ContextD3D11::clearState() {
    m_deviceContext->ClearState();
}

void ContextD3D11::setFormats(GraphicsShader *graphicsShader,
                              NvFlexFormat renderTargetFormat,
                              NvFlexFormat depthStencilFormat) {
    auto shader = implCast<GraphicsShaderD3D11>(graphicsShader);
    shader->setFormats(renderTargetFormat, depthStencilFormat);
}

void ContextD3D11::setIndexBuffer(IndexBuffer *buffer, uint32_t offset) {
    auto bufD3D11 = implCast<IndexBufferD3D11>(buffer);
    getContext()->IASetIndexBuffer(bufD3D11->m_buffer.Get(),
                                   convertToDXGI(buffer->m_desc.format), offset);
}

void ContextD3D11::setVertexBuffer(VertexBuffer *buffer, uint32_t stride, uint32_t offset) {
    auto bufD3D11 = implCast<VertexBufferD3D11>(buffer);
    getContext()->IASetVertexBuffers(0, 1, bufD3D11->m_buffer.GetAddressOf(), &stride,
                                     &offset);
}

void ContextD3D11::setViewport(const NvFlexViewport *vp) {
    getContext()->RSSetViewports(1, (const D3D11_VIEWPORT *)vp);
}

Fence *ContextD3D11::shareFence(Fence *fence) {
    return nullptr;
}

Texture2D *ContextD3D11::shareTexture2D(Texture2D *sharedTexture) {
    return new Texture2DD3D11(this, sharedTexture, false);
}

Texture2DCrossAdapter *ContextD3D11::shareTexture2DCrossAdapter(
    Texture2DCrossAdapter *sharedTexture) {
    return nullptr;
}

Texture2D *ContextD3D11::shareTexture2DShared(Texture2D *sharedTexture) {
    return new Texture2DD3D11(this, sharedTexture, true);
}

void ContextD3D11::signalFence(Fence *fence, uint64_t fenceValue) {}

void ContextD3D11::timerBegin(Timer *timerIn) {
    auto timer = implCast<TimerD3D11>(timerIn);
    if (!timer->m_state) {
        auto context = getContext();
        QueryPerformanceFrequency(&timer->m_cpuFreq);
        QueryPerformanceCounter(&timer->m_cpuBegin);
        context->Begin(timer->m_disjoint.Get());
        context->End(timer->m_begin.Get());
        timer->m_state = 1;
    }
}

void ContextD3D11::timerEnd(Timer *timerIn) {
    auto timer = implCast<TimerD3D11>(timerIn);
    if (timer->m_state == 1) {
        auto context = getContext();
        context->End(timer->m_end.Get());
        context->End(timer->m_disjoint.Get());
        QueryPerformanceCounter(&timer->m_cpuEnd);
        timer->m_state = 2;
    }
}

NvFlexResult ContextD3D11::timerGetResult(Timer *timerIn, float *timeGPU, float *timeCPU,
                                          NvFlexUint64 *gpuStartStamp,
                                          NvFlexUint64 *gpuEndStamp,
                                          NvFlexUint64 *gpuFreq) {
    auto timer = implCast<TimerD3D11>(timerIn);
    if (timer->m_state != 2)
        return eNvFlexFail;

    HRESULT hr;
    auto context = getContext();
    D3D11_QUERY_DATA_TIMESTAMP_DISJOINT tsDisjoint;
    UINT64 tsBegin, tsEnd;

    hr = context->GetData(timer->m_disjoint.Get(), nullptr, 0, 0);
    if (FAILED(hr))
        return eNvFlexFail;

    context->GetData(timer->m_disjoint.Get(), &tsDisjoint, sizeof(tsDisjoint), 0);
    if (tsDisjoint.Disjoint)
        return eNvFlexFail;

    hr = context->GetData(timer->m_begin.Get(), &tsBegin, sizeof(tsBegin), 0);
    if (FAILED(hr))
        return eNvFlexFail;

    hr = context->GetData(timer->m_end.Get(), &tsEnd, sizeof(tsEnd), 0);
    if (FAILED(hr))
        return eNvFlexFail;

    double Diff = double(tsEnd - tsBegin);
    double Frequency_low = tsDisjoint.Frequency;

    float timems = Diff / Frequency_low;

    float msCPU = (double)(timer->m_cpuEnd.QuadPart - timer->m_cpuBegin.QuadPart) /
                  (double)timer->m_cpuFreq.QuadPart;

    if (timeGPU)
        *timeGPU = timems;
    if (timeCPU) *timeCPU = msCPU;

    if(gpuStartStamp)
        *gpuStartStamp = tsBegin;
    if(gpuEndStamp)
        *gpuEndStamp = tsEnd;
    if(gpuFreq)
        *gpuFreq = tsDisjoint.Frequency;

    timer->m_state = 0;
    return eNvFlexSuccess;
}

void ContextD3D11::transitionToCommonState(Resource *resource) {}

void ContextD3D11::unmap(ConstantBuffer *buffer) {
    auto bufD3D11 = implCast<ConstantBufferD3D11>(buffer);
    getContext()->Unmap(bufD3D11->m_buffer.Get(), 0);
}

void ContextD3D11::unmap(IndexBuffer *buffer) {
    auto bufD3D11 = implCast<IndexBufferD3D11>(buffer);
    auto context = getContext();
    context->Unmap(bufD3D11->m_uploadBuffer.Get(), 0);
    context->CopyResource(bufD3D11->m_buffer.Get(), bufD3D11->m_uploadBuffer.Get());
}

void ContextD3D11::unmap(VertexBuffer *buffer) {
    auto bufD3D11 = implCast<VertexBufferD3D11>(buffer);
    auto context = getContext();
    context->Unmap(bufD3D11->m_uploadBuffer.Get(), 0);
    context->CopyResource(bufD3D11->m_buffer.Get(), bufD3D11->m_uploadBuffer.Get());
}

void ContextD3D11::unmap(Texture1D *buffer) {
    auto bufD3D11 = implCast<Texture1DD3D11>(buffer);
    if (bufD3D11->m_desc.uploadAccess) {
        auto context = getContext();
        context->Unmap(bufD3D11->m_uploadTexture.Get(), 0);
        context->CopyResource(bufD3D11->m_texture.Get(), bufD3D11->m_uploadTexture.Get());
    }
}

void ContextD3D11::unmap(Texture3D *texture) {
    auto texD3D11 = implCast<Texture3DD3D11>(texture);
    if (texD3D11->m_desc.uploadAccess) {
        auto context = getContext();
        context->Unmap(texD3D11->m_uploadTexture.Get(), 0);
        context->CopyResource(texD3D11->m_texture.Get(), texD3D11->m_uploadTexture.Get());
    }
}

void ContextD3D11::unmapDownload(Texture3D *texture) {
    auto texD3D11 = implCast<Texture3DD3D11>(texture);
    if (texD3D11->m_desc.downloadAccess) {
        getContext()->Unmap(texD3D11->m_downloadTexture.Get(), 0);
    }
}

void ContextD3D11::upload(Buffer *bufferIn, NvFlexUint offset, const void *data,
                          NvFlexUint numBytes) {
    auto buffer = implCast<BufferD3D11>(bufferIn);
    if(buffer->m_desc.memType != eNvFlexMemoryType_default) {
        NVFLEX_ASSERT(0 && "upload to a dynamic buffer or staging buffer is not allowed");
        return;
    }

    D3D11_BOX region = {offset, 0, 0, offset + numBytes, 1, 1};
    getContext()->UpdateSubresource(buffer->m_buffer.Get(), 0, &region, data, 0, 0);
}

void ContextD3D11::updateVTRMapping(Texture3DVTR *textureIn, HeapVTR *heapIn,
                                    uint32_t *blockTableImage, uint32_t rowPitch,
                                    uint32_t depthPitch) {
    auto texture = implCast<Texture3DVTRD3D11>(textureIn);
    auto heap = implCast<HeapVTRD3D11>(heapIn);
    auto context = getContext();

    NvFlexDim gridDim = texture->m_gridDim;
    uint32_t maxBlocks = gridDim.x * gridDim.y * gridDim.z;

    m_tileCoords.resize(maxBlocks);
    m_tileRegionSize.resize(maxBlocks);
    m_rangeFlags.resize(maxBlocks);
    m_tilePoolCoords.resize(maxBlocks);
    m_tilePoolRangeSize.resize(maxBlocks);

    uint32_t tileID;
    uint32_t index = 0;
    auto &currentImage = texture->m_blockTable;
    for (int k = 0; k < currentImage.dim().z; ++k)
        for (int j = 0; j < currentImage.dim().y; ++j)
            for (int i = 0; i < currentImage.dim().x; ++i) {
                auto newVal =
                    blockTableImage[i + (rowPitch * j + depthPitch * k) / sizeof(uint32_t)];
                auto &oldVal = currentImage(i, j, k);
                if (newVal != oldVal) {
                    tileID = ~newVal;
                    D3D11_TILED_RESOURCE_COORDINATE coords;
                    coords.X = i;
                    coords.Y = j;
                    coords.Z = k;
                    coords.Subresource = 0;
                    m_tileCoords[index] = coords;

                    D3D11_TILE_REGION_SIZE regionSize;
                    regionSize.NumTiles = 1;
                    regionSize.bUseBox = FALSE;
                    regionSize.Width = 1;
                    regionSize.Height = 1;
                    regionSize.Depth = 1;
                    m_tileRegionSize[index] = regionSize;

                    m_rangeFlags[index] =
                        newVal ? D3D11_TILE_RANGE_REUSE_SINGLE_TILE : D3D11_TILE_RANGE_NULL;
                    m_tilePoolCoords[index] = tileID;
                    m_tilePoolRangeSize[index] = 1;
                    oldVal = newVal;
                    ++index;
                }
            }

    if (index) {
        ComPtr<ID3D11DeviceContext2> deviceContext2;
        if (SUCCEEDED(m_deviceContext->QueryInterface(IID_PPV_ARGS(&deviceContext2)))) {
            deviceContext2->UpdateTileMappings(
                texture->m_texture.Get(), index, m_tileCoords.data(),
                m_tileRegionSize.data(), heap->m_heap.Get(), index,
                (const UINT *)m_rangeFlags.data(), m_tilePoolCoords.data(),
                m_tilePoolRangeSize.data(), D3D11_TILE_MAPPING_NO_OVERWRITE);
        }
    }
}

void ContextD3D11::waitOnFence(Fence *fence, uint64_t fenceValue) {}

DepthStencilView *ContextD3D11::createDepthStencilView(
    const NvFlexDepthStencilViewDescD3D11 *desc) {
    return new DepthStencilViewD3D11(this, desc);
}

RenderTargetView *ContextD3D11::createRenderTargetView(
    const NvFlexRenderTargetViewDescD3D11 *desc) {
    return new RenderTargetViewD3D11(this, desc);
}

void ContextD3D11::setRenderTarget(RenderTarget *rtvIn, DepthStencil *dsvIn) {
    auto rtv = implCast<RenderTargetD3D11>(rtvIn);
    auto dsv = implCast<DepthStencilD3D11>(dsvIn);
    ID3D11RenderTargetView *prtv = rtv ? rtv->m_rtv.Get() : nullptr;
    ID3D11DepthStencilView *pdsv = dsv ? dsv->m_dsv.Get() : nullptr;
    auto context = getContext();

    context->OMSetRenderTargets(1, &prtv, pdsv);

    if (rtv) {
        D3D11_VIEWPORT vp;
        memcpy(&vp, &rtv->m_viewport, sizeof(vp));
        context->RSSetViewports(1, &vp);
    } else if (dsv) {
        D3D11_VIEWPORT vp;
        memcpy(&vp, &dsv->m_viewport, sizeof(vp));
        context->RSSetViewports(1, &vp);
    }
}

NvFlexResult ContextD3D11::registerD3DBuffer(void *nativeBuffer, NvFlexUint dim,
                                             NvFlexUint structStride, Buffer **ppBuffer){
    BufferD3D11 *bufD3D11 = new BufferD3D11(this, (ID3D11Buffer *)nativeBuffer, dim, structStride);
    if(ppBuffer) {
        *ppBuffer = bufD3D11;
        bufD3D11->addRef();
    }
    bufD3D11->release();
    return eNvFlexSuccess;
}

ResourceReference *ContextD3D11::shareResourceReference(ResourceD3D11 *resource) {
    return new ResourceReferenceD3D11(this, resource);
}

void ContextD3D11::updateDepthStencilView(NvFlexDepthStencilView *view,
                                          const NvFlexDepthStencilViewDescD3D11 *desc) {
    auto viewD3D11 = implCast<DepthStencilViewD3D11>(view);
    viewD3D11->update(desc);
}

void ContextD3D11::updateRenderTargetView(NvFlexRenderTargetView *view,
                                          const NvFlexRenderTargetViewDescD3D11 *desc) {
    auto viewD3D11 = implCast<RenderTargetViewD3D11>(view);
    viewD3D11->update(desc);
}

void ContextD3D11::updateResourceRWViewDesc(ResourceRWD3D11 *resourceRWIn,
                                            NvFlexResourceRWViewDescD3D11 *desc) {
    desc->resourceView.srv = resourceRWIn->m_srv.Get();
    desc->uav = resourceRWIn->m_uav.Get();
}

void ContextD3D11::updateResourceViewDesc(ResourceD3D11 *resourceIn,
                                          NvFlexResourceViewDescD3D11 *desc) {
    desc->srv = resourceIn->m_srv.Get();
}

void ContextD3D11::updateContext(const NvFlexContextDescD3D11 *pdesc) {
    m_device = pdesc->device;
    m_deviceContext = pdesc->deviceContext;
    if(!m_d3dAnnotation)
        m_deviceContext->QueryInterface(IID_PPV_ARGS(&m_d3dAnnotation));
}

void ContextD3D11::updateContextDesc(NvFlexContextDescD3D11 *desc) {
    desc->device = m_device;
    desc->deviceContext = m_deviceContext;
}

BufferD3D11::BufferD3D11(ContextD3D11 *context, BufferD3D11 *buffer,
                         const NvFlexBufferViewDesc *desc)
    : Object(context->getDeferredRelease()), m_context(context) {
    NVFLEX_ASSERT(buffer->m_desc.structStride == 0 && buffer->m_desc.format != eNvFlexFormat_unknown);
    m_desc = buffer->m_desc;
    uint32_t srcFormatSize = getFormatSizeInBytes(m_desc.format);
    m_desc.format = desc->format;
    uint32_t newFormatSize = getFormatSizeInBytes(m_desc.format);
    m_desc.dim = m_desc.dim * srcFormatSize / newFormatSize;
    m_buffer = buffer->m_buffer;

    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Format = convertToDXGI(m_desc.format);
    srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    srvDesc.Buffer.FirstElement = 0;
    srvDesc.Buffer.NumElements = m_desc.dim;
    context->getDevice()->CreateShaderResourceView(m_buffer.Get(), &srvDesc, &m_srv);

    D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.Format = srvDesc.Format;
    uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    uavDesc.Buffer.FirstElement = 0;
    uavDesc.Buffer.NumElements = m_desc.dim;
    context->getDevice()->CreateUnorderedAccessView(m_buffer.Get(), &uavDesc, &m_uav);
}

BufferD3D11::BufferD3D11(ContextD3D11 *context, const NvFlexBufferDesc *desc)
    : Object(context->getDeferredRelease()), m_context(context) {
    m_desc = *desc;

    if(m_desc.format != eNvFlexFormat_unknown)
        NVFLEX_ASSERT(m_desc.structStride == 0);
    else
        NVFLEX_ASSERT(m_desc.structStride != 0);

    const bool isTypeless = isFormatTypeless(m_desc.format);
    const bool isStructured = m_desc.structStride != 0;
    const uint32_t elementSizeInBytes =
        isStructured ? m_desc.structStride : getFormatSizeInBytes(m_desc.format);

    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = elementSizeInBytes * m_desc.dim;
    bufDesc.StructureByteStride = isStructured ? elementSizeInBytes : 0;
    switch (m_desc.memType) {
        case eNvFlexMemoryType_upload:
            bufDesc.Usage = D3D11_USAGE_DYNAMIC;
            bufDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            bufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            break;
        case eNvFlexMemoryType_default:
            bufDesc.Usage = D3D11_USAGE_DEFAULT;
            bufDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
            if(isTypeless)
                bufDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
            else if(isStructured)
                bufDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            break;
        case eNvFlexMemoryType_readback:
            bufDesc.Usage = D3D11_USAGE_STAGING;
            bufDesc.BindFlags = 0;
            bufDesc.CPUAccessFlags = ((m_desc.cpuAccessMode & eNvFlexCpuAccessMode_read) ? D3D11_CPU_ACCESS_READ : 0) |
                                     ((m_desc.cpuAccessMode & eNvFlexCpuAccessMode_write) ? D3D11_CPU_ACCESS_WRITE : 0);
            if(isStructured)
                bufDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
            break;
    }

    auto device = context->getDevice();
    device->CreateBuffer(&bufDesc, nullptr, &m_buffer);
    if(m_desc.debugName)
        m_buffer->SetPrivateData(WKPDID_D3DDebugObjectName, strlen(m_desc.debugName),
                                 m_desc.debugName);

    if(m_desc.memType == eNvFlexMemoryType_default) {
        D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
        if (!isTypeless) {
            srvDesc.Format = convertToDXGI(m_desc.format);
            srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
            srvDesc.Buffer.FirstElement = 0;
            srvDesc.Buffer.NumElements = m_desc.dim;
        } else {
            srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
            srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
            srvDesc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
            srvDesc.BufferEx.NumElements = bufDesc.ByteWidth / sizeof(int);
        }
        device->CreateShaderResourceView(m_buffer.Get(), &srvDesc, &m_srv);

        D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
        if(!isTypeless) {
            uavDesc.Format = convertToDXGI(m_desc.format);
            uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
            uavDesc.Buffer.NumElements = m_desc.dim;
        } else {
            uavDesc.Format = DXGI_FORMAT_R32_TYPELESS;
            uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
            uavDesc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
            uavDesc.Buffer.NumElements = bufDesc.ByteWidth / sizeof(int);
        }

        device->CreateUnorderedAccessView(m_buffer.Get(), &uavDesc, &m_uav);
    }
}

BufferD3D11::BufferD3D11(ContextD3D11 *ctx, ID3D11Buffer *d3dBuffer, NvFlexUint dim,
                         NvFlexUint structStride): m_context(ctx), m_buffer(d3dBuffer) {
    NVFLEX_ASSERT(structStride != 0 && dim != 0);

    memset(&m_desc, 0, sizeof(m_desc));

    D3D11_BUFFER_DESC d3dBufDesc;
    d3dBuffer->GetDesc(&d3dBufDesc);
    switch (d3dBufDesc.Usage) {
        case D3D11_USAGE_DEFAULT:
        default:
            m_desc.memType = eNvFlexMemoryType_default;
            break;
        case D3D11_USAGE_DYNAMIC:
            m_desc.memType = eNvFlexMemoryType_upload;
            break;
        case D3D11_USAGE_STAGING:
            m_desc.memType = eNvFlexMemoryType_readback;
            break;
    }

    if(d3dBufDesc.CPUAccessFlags & D3D11_CPU_ACCESS_READ)
        m_desc.cpuAccessMode = (NvFlexCpuAccessMode)(m_desc.cpuAccessMode | eNvFlexCpuAccessMode_read);
    if(d3dBufDesc.CPUAccessFlags & D3D11_CPU_ACCESS_WRITE)
        m_desc.cpuAccessMode = (NvFlexCpuAccessMode)(m_desc.cpuAccessMode | eNvFlexCpuAccessMode_write);

    m_desc.dim = dim;
    m_desc.structStride = structStride;
    NVFLEX_ASSERT(m_desc.dim * m_desc.structStride <= d3dBufDesc.ByteWidth);

    auto device = ctx->getDevice();
    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = m_desc.dim * m_desc.structStride;
}

uint64_t BufferD3D11::getGPUBytesUsed() {
    return getFormatSizeInBytes(m_desc.format) * m_desc.dim;
}

NvFlexContext *BufferD3D11::getContext() {
    return m_context;
}

ConstantBufferD3D11::ConstantBufferD3D11(ContextD3D11 *context,
                                         const NvFlexConstantBufferDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    UINT byteWidth = alignUp<16>(desc->sizeInBytes);

    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = byteWidth;
    if (desc->uploadAccess) {
        bufDesc.Usage = D3D11_USAGE_DYNAMIC;
        bufDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        bufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    } else {
        bufDesc.Usage = D3D11_USAGE_DEFAULT;
        bufDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        bufDesc.CPUAccessFlags = 0;
    }
    context->getDevice()->CreateBuffer(&bufDesc, nullptr, &m_buffer);
    if(m_desc.debugName)
        m_buffer->SetPrivateData(WKPDID_D3DDebugObjectName, strlen(m_desc.debugName),
                                 m_desc.debugName);
}

uint64_t ConstantBufferD3D11::getGPUBytesUsed() {
    return alignUp<16>(m_desc.sizeInBytes);
}

VertexBufferD3D11::VertexBufferD3D11(ContextD3D11 *context, const NvFlexVertexBufferDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;

    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = desc->sizeInBytes;
    bufDesc.Usage = D3D11_USAGE_DEFAULT;
    bufDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    bufDesc.CPUAccessFlags = 0;
    bufDesc.MiscFlags = 0;

    D3D11_SUBRESOURCE_DATA initData = {};
    initData.pSysMem = desc->data;
    initData.SysMemPitch = desc->sizeInBytes;
    context->getDevice()->CreateBuffer(&bufDesc, &initData, &m_buffer);

    bufDesc.Usage = D3D11_USAGE_DYNAMIC;
    bufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bufDesc.MiscFlags = 0;
    context->getDevice()->CreateBuffer(&bufDesc, nullptr, &m_uploadBuffer);
}

uint64_t VertexBufferD3D11::getGPUBytesUsed() {
    return m_desc.sizeInBytes;
}

IndexBufferD3D11::IndexBufferD3D11(ContextD3D11 *context, const NvFlexIndexBufferDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    m_format = convertToDXGI(m_desc.format);

    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = desc->sizeInBytes;
    bufDesc.Usage = D3D11_USAGE_DEFAULT;
    bufDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    bufDesc.CPUAccessFlags = 0;
    bufDesc.MiscFlags = 0;

    D3D11_SUBRESOURCE_DATA initData = {};
    initData.pSysMem = desc->data;
    initData.SysMemPitch = desc->sizeInBytes;
    context->getDevice()->CreateBuffer(&bufDesc, &initData, &m_buffer);

    bufDesc.Usage = D3D11_USAGE_DYNAMIC;
    bufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bufDesc.MiscFlags = 0;
    context->getDevice()->CreateBuffer(&bufDesc, nullptr, &m_uploadBuffer);
}

uint64_t IndexBufferD3D11::getGPUBytesUsed() {
    return m_desc.sizeInBytes;
}

Texture1DD3D11::Texture1DD3D11(ContextD3D11 *context, const NvFlexTexture1DDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;

    D3D11_TEXTURE1D_DESC texDesc = {};
    texDesc.Format = convertToDXGI(m_desc.format);
    texDesc.Width = m_desc.dim;
    texDesc.MipLevels = 1;
    texDesc.ArraySize = 1;
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = 0;
    auto device = context->getDevice();
    device->CreateTexture1D(&texDesc, nullptr, &m_texture);

    device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
    device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);

    if (m_desc.uploadAccess) {
        texDesc.Usage = D3D11_USAGE_DYNAMIC;
        texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        texDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        device->CreateTexture1D(&texDesc, nullptr, &m_uploadTexture);
    }
}

uint64_t Texture1DD3D11::getGPUBytesUsed() {
    return getFormatSizeInBytes(m_desc.format) * m_desc.dim;
}

Texture2DD3D11::Texture2DD3D11(ContextD3D11 *context, const NvFlexTexture2DDesc *desc,
                               bool createShared)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;

    D3D11_TEXTURE2D_DESC texDesc = {};
    texDesc.Width = m_desc.width;
    texDesc.Height = m_desc.height;
    texDesc.MipLevels = 1;
    texDesc.ArraySize = 1;
    texDesc.Format = convertToDXGI(m_desc.format);
    texDesc.SampleDesc.Count = 1;
    texDesc.SampleDesc.Quality = 0;
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = createShared ? D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX : 0;
    auto device = context->getDevice();
    device->CreateTexture2D(&texDesc, nullptr, &m_texture);

    device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
    device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);

    if (createShared) {
        ComPtr<IDXGIKeyedMutex> keyedMutex;
        device->QueryInterface(IID_PPV_ARGS(&keyedMutex));
        keyedMutex->AcquireSync(0, INFINITE);
    }
}

Texture2DD3D11::Texture2DD3D11(ContextD3D11 *context, Texture2D *sharedTexture, bool openShared)
    : Object(context->getDeferredRelease()) {
    m_desc = sharedTexture->m_desc;

    auto device = context->getDevice();

    if (openShared) {
        HANDLE sharedHandle;
        sharedTexture->openSharedHandle(&sharedHandle);
        device->OpenSharedResource(sharedHandle, IID_PPV_ARGS(&m_texture));
        sharedTexture->closeSharedHandle(sharedHandle);
    } else {
        if (sharedTexture) {
            auto texD3D11 = static_cast<Texture2DD3D11 *>(sharedTexture);
            m_texture = texD3D11->m_texture;
        }
    }

    if (m_texture) {
        device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
        device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);
    }
}

void Texture2DD3D11::openSharedHandle(HANDLE *handleIn) {
    ComPtr<IDXGIResource1> resource;
    m_texture->QueryInterface(IID_PPV_ARGS(&resource));
    resource->GetSharedHandle(handleIn);
}

void Texture2DD3D11::closeSharedHandle(HANDLE handle) {}

uint64_t Texture2DD3D11::getGPUBytesUsed() {
    return getFormatSizeInBytes(m_desc.format) * m_desc.width * m_desc.height;
}

Texture3DD3D11::Texture3DD3D11(ContextD3D11 *context, const NvFlexTexture3DDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    auto device = context->getDevice();

    D3D11_TEXTURE3D_DESC texDesc = {};
    texDesc.Width = m_desc.dim.x;
    texDesc.Height = m_desc.dim.y;
    texDesc.Depth = m_desc.dim.z;
    texDesc.MipLevels = 1;
    texDesc.Format = convertToDXGI(m_desc.format);
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = 0;

    device->CreateTexture3D(&texDesc, nullptr, &m_texture);

    device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
    device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);

    if (m_desc.uploadAccess) {
        texDesc.Usage = D3D11_USAGE_DYNAMIC;
        texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        texDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        device->CreateTexture3D(&texDesc, nullptr, &m_uploadTexture);
    }

    if (m_desc.downloadAccess) {
        texDesc.Usage = D3D11_USAGE_STAGING;
        texDesc.BindFlags = 0;
        texDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        device->CreateTexture3D(&texDesc, nullptr, &m_downloadTexture);
    }
}

uint64_t Texture3DD3D11::getGPUBytesUsed() {
    uint64_t sz = getFormatSizeInBytes(m_desc.format);
    return sz * m_desc.dim.x * m_desc.dim.y * m_desc.dim.z;
}

ResourceReferenceD3D11::ResourceReferenceD3D11(ContextD3D11 *context, ResourceD3D11 *resource)
    : Object(context->getDeferredRelease()) {
    resource->m_srv->GetResource(&m_resource);
}

uint64_t ResourceReferenceD3D11::getGPUBytesUsed() {
    return 0;
}

HeapVTRD3D11::HeapVTRD3D11(ContextD3D11 *context, const NvFlexHeapSparseDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;

    constexpr uint32_t tileSize = 0x10000;  // 64K
    m_numTiles = (m_desc.sizeInBytes + tileSize - 1) / tileSize;

    D3D11_BUFFER_DESC bufDesc = {};
    bufDesc.ByteWidth = m_numTiles * tileSize;
    bufDesc.Usage = D3D11_USAGE_DEFAULT;
    bufDesc.BindFlags = 0;
    bufDesc.CPUAccessFlags = 0;
    bufDesc.MiscFlags = D3D11_RESOURCE_MISC_TILE_POOL;
    bufDesc.StructureByteStride = 0;
    context->getDevice()->CreateBuffer(&bufDesc, nullptr, &m_heap);
}

uint64_t HeapVTRD3D11::getGPUBytesUsed() {
    return m_desc.sizeInBytes;
}

Texture3DVTRD3D11::Texture3DVTRD3D11(ContextD3D11 *context,
                                     const NvFlexTexture3DSparseDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    auto device = context->getDevice();

    D3D11_TEXTURE3D_DESC texDesc = {};
    texDesc.Width = m_desc.dim.x;
    texDesc.Height = m_desc.dim.y;
    texDesc.Depth = m_desc.dim.z;
    texDesc.MipLevels = 1;
    texDesc.Format = convertToDXGI(m_desc.format);
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = D3D11_RESOURCE_MISC_TILED;

    device->CreateTexture3D(&texDesc, nullptr, &m_texture);
    device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
    device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);

    m_blockDim = getTileDim(m_desc.format);
    m_gridDim = m_desc.dim / m_blockDim;
    m_blockTable.init(m_gridDim);
    ZeroMemory(m_blockTable.data(), m_blockTable.dim1() * sizeof(uint32_t));
}

uint64_t Texture3DVTRD3D11::getGPUBytesUsed() {
    return 0;
}

ColorBufferD3D11::ColorBufferD3D11(ContextD3D11 *context, const NvFlexColorBufferDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;

    auto device = context->getDevice();

    m_rt_format = m_desc.format;
    m_viewport.topLeftX = 0.f;
    m_viewport.topLeftY = 0.f;
    m_viewport.width = (float)m_desc.width;
    m_viewport.height = (float)m_desc.height;
    m_viewport.minDepth = 0.f;
    m_viewport.maxDepth = 1.f;

    D3D11_TEXTURE2D_DESC texDesc = {};
    texDesc.Width = m_desc.width;
    texDesc.Height = m_desc.height;
    texDesc.MipLevels = 1;
    texDesc.ArraySize = 1;
    texDesc.Format = convertToDXGI(m_desc.format);
    texDesc.SampleDesc.Count = 1;
    texDesc.SampleDesc.Quality = 0;
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags =
        D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = 0;
    device->CreateTexture2D(&texDesc, nullptr, &m_texture);
    device->CreateShaderResourceView(m_texture.Get(), nullptr, &m_srv);
    device->CreateUnorderedAccessView(m_texture.Get(), nullptr, &m_uav);
    device->CreateRenderTargetView(m_texture.Get(), nullptr, &m_rtv);
}

uint64_t ColorBufferD3D11::getGPUBytesUsed() {
    uint32_t sz = getFormatSizeInBytes(m_desc.format);
    sz *= m_desc.width * m_desc.height;
    return sz;
}

DepthBufferD3D11::DepthBufferD3D11(ContextD3D11 *context, const NvFlexDepthBufferDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    m_ds_format = m_desc.format_dsv;
    m_viewport.topLeftX = 0.f;
    m_viewport.topLeftY = 0.f;
    m_viewport.width = (float)m_desc.width;
    m_viewport.height = (float)m_desc.height;
    m_viewport.minDepth = 0.f;
    m_viewport.maxDepth = 1.f;

    auto device = context->getDevice();

    D3D11_TEXTURE2D_DESC texDesc = {};
    texDesc.Width = m_desc.width;
    texDesc.Height = m_desc.height;
    texDesc.MipLevels = 1;
    texDesc.ArraySize = 1;
    texDesc.Format = convertToDXGI(m_desc.format_resource);
    texDesc.SampleDesc.Count = 1;
    texDesc.SampleDesc.Quality = 0;
    texDesc.Usage = D3D11_USAGE_DEFAULT;
    texDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;
    texDesc.CPUAccessFlags = 0;
    texDesc.MiscFlags = 0;
    device->CreateTexture2D(&texDesc, nullptr, &m_texture);

    D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = convertToDXGI(m_desc.format_dsv);
    dsvDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Flags = 0;
    dsvDesc.Texture2D.MipSlice = 0;
    device->CreateDepthStencilView(m_texture.Get(), &dsvDesc, &m_dsv);

    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Format = convertToDXGI(m_desc.format_srv);
    srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Texture2D.MipLevels = 1;
    device->CreateShaderResourceView(m_texture.Get(), &srvDesc, &m_srv);
}

uint64_t DepthBufferD3D11::getGPUBytesUsed() {
    uint64_t sz = getFormatSizeInBytes(m_desc.format_resource);
    sz *= m_desc.width * m_desc.height;
    return sz;
}

DepthStencilViewD3D11::DepthStencilViewD3D11(ContextD3D11 *context,
                                             const NvFlexDepthStencilViewDescD3D11 *desc)
    : Object(context->getDeferredRelease()) {
    update(desc);
}

uint64_t DepthStencilViewD3D11::getGPUBytesUsed() {
    return Object::getGPUBytesUsed();
}

void DepthStencilViewD3D11::update(const NvFlexDepthStencilViewDescD3D11 *desc) {
    m_desc = *desc;

    D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc;
    m_desc.dsv->GetDesc(&dsvDesc);
    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
    m_desc.srv->GetDesc(&srvDesc);

    NvFlexDim resDim = ContextD3D11::extractDim(m_desc.srv);
    m_ds_format = convertToNvFlex(dsvDesc.Format);
    m_srv = m_desc.srv;
    m_dsv = m_desc.dsv;
    m_width = resDim.x;
    m_height = resDim.y;
    copyViewport(&m_viewport, &m_desc.viewport);
}

NvFlexDepthBufferDesc DepthStencilViewD3D11::getDepthBufferDesc() {
    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
    m_srv->GetDesc(&srvDesc);

    NvFlexFormat resourceFormat = eNvFlexFormat_unknown;
    {
        ComPtr<ID3D11Resource> resource;
        ComPtr<ID3D11Texture2D> texture;
        m_srv->GetResource(&resource);
        resource->QueryInterface(IID_PPV_ARGS(&texture));
        if (texture) {
            D3D11_TEXTURE2D_DESC texDesc;
            texture->GetDesc(&texDesc);
            resourceFormat = convertToNvFlex(texDesc.Format);
        }
    }

    NvFlexDepthBufferDesc bufDesc = {};
    bufDesc.format_resource = resourceFormat;
    bufDesc.format_dsv = m_ds_format;
    bufDesc.format_srv = convertToNvFlex(srvDesc.Format);
    bufDesc.width = m_viewport.width;
    bufDesc.height = m_viewport.height;
    return bufDesc;
}

void copyViewport(NvFlexViewport *dst, const D3D11_VIEWPORT *src) {
    *(D3D11_VIEWPORT *)dst = *src;
}

D3D11_BLEND convertToD3D11(NvFlexBlendEnum blendEnum) {
    return g_blendEnumTableD3D11.table[blendEnum];
}

D3D11_BLEND_OP convertToD3D11(NvFlexBlendOpEnum blendOpEnum) {
    return g_blendOpEnumTableD3D11.table[blendOpEnum];
}

D3D11_COMPARISON_FUNC convertToD3D11(NvFlexComparisonEnum comparison) {
    return g_comparisonTableD3D11.table[comparison];
}

int64_t FlexDeferredReleaseD3D11(float timeoutMS) {
    return 0;
}

RenderTargetViewD3D11::RenderTargetViewD3D11(ContextD3D11 *context,
                                             const NvFlexRenderTargetViewDescD3D11 *desc)
    : Object(context->getDeferredRelease()) {
    update(desc);
}

uint64_t RenderTargetViewD3D11::getGPUBytesUsed() {
    return 0;
}

void RenderTargetViewD3D11::update(const NvFlexRenderTargetViewDescD3D11 *desc) {
    m_desc = *desc;

    D3D11_RENDER_TARGET_VIEW_DESC rtvDesc;
    m_desc.rtv->GetDesc(&rtvDesc);
    m_rt_format = convertToNvFlex(rtvDesc.Format);
    m_rtv = m_desc.rtv;
    copyViewport(&m_viewport, &m_desc.viewport);
}

ComputeShaderD3D11::ComputeShaderD3D11(ContextD3D11 *context,
                                       const NvFlexComputeShaderDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    if (!m_desc.label) m_desc.label = L"unlabeled";
    if(m_desc.NVAPI_Slot != -1)
        NvAPI_D3D11_SetNvShaderExtnSlot(context->getDevice(), (NvU32)m_desc.NVAPI_Slot);
    context->getDevice()->CreateComputeShader(m_desc.cs, m_desc.cs_length, nullptr, &m_cs);
    if(m_desc.NVAPI_Slot != -1)
        NvAPI_D3D11_SetNvShaderExtnSlot(context->getDevice(), -1u);
}

uint64_t ComputeShaderD3D11::getGPUBytesUsed() {
    return Object::getGPUBytesUsed();
}

GraphicsShaderD3D11::GraphicsShaderD3D11(ContextD3D11 *context,
                                         const NvFlexGraphicsShaderDesc *desc)
    : Object(context->getDeferredRelease()) {
    m_desc = *desc;
    m_inputElementDescs.resize(m_desc.numInputElements);

    auto device = context->getDevice();

    for (int i = 0; i < m_desc.numInputElements; ++i) {
        m_inputElementDescs[i] = m_desc.inputElementDescs[i];
    }
    m_desc.inputElementDescs = m_inputElementDescs.data();

    if (!m_desc.label) m_desc.label = L"unlabeled";

    device->CreateVertexShader(m_desc.vs, m_desc.vs_length, nullptr, &m_vs);
    device->CreatePixelShader(m_desc.ps, m_desc.ps_length, nullptr, &m_ps);

    VectorCached<D3D11_INPUT_ELEMENT_DESC, 4> elementDesc;
    UINT alignedByteOffset = 0;

    elementDesc.resize(m_desc.numInputElements);
    for (int i = 0; i < m_desc.numInputElements; ++i) {
        auto &input = m_desc.inputElementDescs[i];
        auto &e = elementDesc[i];
        e.SemanticName = input.semanticName;
        e.SemanticIndex = 0;
        e.Format = convertToDXGI(input.format);
        e.InputSlot = 0;
        e.AlignedByteOffset = alignedByteOffset;
        e.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
        e.InstanceDataStepRate = 0;
        alignedByteOffset = -1;
    }
    NVFLEX_ASSERT(m_desc.numInputElements);

    device->CreateInputLayout(elementDesc.data(), m_desc.numInputElements, m_desc.vs,
                              m_desc.vs_length, &m_inputLayout);

    D3D11_BLEND_DESC blendDesc = {};
    blendDesc.RenderTarget[0].BlendEnable = m_desc.blendState.enable;
    blendDesc.RenderTarget[0].SrcBlend = convertToD3D11(m_desc.blendState.srcBlendColor);
    blendDesc.RenderTarget[0].DestBlend = convertToD3D11(m_desc.blendState.dstBlendColor);
    blendDesc.RenderTarget[0].BlendOp = convertToD3D11(m_desc.blendState.blendOpColor);
    blendDesc.RenderTarget[0].SrcBlendAlpha =
        convertToD3D11(m_desc.blendState.srcBlendAlpha);
    blendDesc.RenderTarget[0].DestBlendAlpha =
        convertToD3D11(m_desc.blendState.dstBlendAlpha);
    blendDesc.RenderTarget[0].BlendOpAlpha = convertToD3D11(m_desc.blendState.blendOpAlpha);
    blendDesc.RenderTarget[0].RenderTargetWriteMask = 0xF;
    device->CreateBlendState(&blendDesc, &m_blenderState);

    D3D11_DEPTH_STENCIL_DESC depthDesc = {};
    depthDesc.DepthEnable = m_desc.depthState.depthEnable;
    depthDesc.DepthWriteMask = m_desc.depthState.depthWriteMask == eNvFlexDepthWriteMask_All
                                   ? D3D11_DEPTH_WRITE_MASK_ALL
                                   : D3D11_DEPTH_WRITE_MASK_ZERO;
    depthDesc.DepthFunc = convertToD3D11(m_desc.depthState.depthFunc);
    depthDesc.StencilEnable = FALSE;
    device->CreateDepthStencilState(&depthDesc, &m_depthStencilState);

    D3D11_RASTERIZER_DESC rastDesc = {};
    rastDesc.FillMode = D3D11_FILL_SOLID;
    rastDesc.CullMode = m_desc.uavTarget ? D3D11_CULL_NONE : D3D11_CULL_BACK;
    rastDesc.FrontCounterClockwise = FALSE;
    rastDesc.DepthBias = 0;
    rastDesc.DepthBiasClamp = 0.f;
    rastDesc.SlopeScaledDepthBias = 0.f;
    rastDesc.DepthClipEnable = m_desc.depthClipEnable;
    rastDesc.ScissorEnable = FALSE;
    rastDesc.MultisampleEnable = FALSE;
    rastDesc.AntialiasedLineEnable = FALSE;
    device->CreateRasterizerState(&rastDesc, &m_rasterizerStateLH);
    rastDesc.FrontCounterClockwise = TRUE;
    device->CreateRasterizerState(&rastDesc, &m_rasterizerStateRH);
}

uint64_t GraphicsShaderD3D11::getGPUBytesUsed() {
    return 0;
}

void GraphicsShaderD3D11::setFormats(NvFlexFormat rtFormat, NvFlexFormat dsFormat) {
    m_desc.renderTargetFormat[0] = rtFormat;
    m_desc.depthStencilFormat = dsFormat;
}

TimerD3D11::TimerD3D11(ContextD3D11 *context) : Object(context->getDeferredRelease()), m_state{0} {
    auto device = context->getDevice();

    D3D11_QUERY_DESC queryDesc = {};
    queryDesc.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
    queryDesc.MiscFlags = 0;
    device->CreateQuery(&queryDesc, &m_disjoint);

    queryDesc.Query = D3D11_QUERY_TIMESTAMP;
    device->CreateQuery(&queryDesc, &m_begin);
    device->CreateQuery(&queryDesc, &m_end);
}

uint64_t TimerD3D11::getGPUBytesUsed() {
    return 0;
}

EventQueueD3D11::EventQueueD3D11(ContextD3D11 *context)
    : Object(context->getDeferredRelease()), m_pushID(0) {}

uint64_t EventQueueD3D11::getGPUBytesUsed() {
    return 0;
}

void EventQueueD3D11::push(uint64_t uid, ContextD3D11 *context) {
    auto e = getNewEvent();
    e->state = eEventStateActive;
    e->uid = uid;
    e->pushID = ++m_pushID;

    if (!e->query) {
        D3D11_QUERY_DESC queryDesc = {};
        queryDesc.Query = D3D11_QUERY_EVENT;
        queryDesc.MiscFlags = 0;
        context->getDevice()->CreateQuery(&queryDesc, &e->query);
    }
    context->getContext()->End(e->query.Get());
}

NvFlexResult EventQueueD3D11::pop(uint64_t *pUid, ContextD3D11 *context) {
    HRESULT hr;
    int compute;
    bool valid = false;
    uint32_t minIdx = 0;
    uint64_t minPushID = 0;

    for (uint32_t i = 0; i < (uint32_t)m_events.size(); ++i) {
        auto &e = m_events[i];
        if (e.state == eEventStateActive) {
            if (valid) {
                if (e.pushID < minPushID) {
                    minIdx = i;
                    minPushID = e.pushID;
                }
            } else {
                minIdx = i;
                minPushID = e.pushID;
                valid = true;
            }
        }
    }

    if (!valid)
        return eNvFlexFail;

    auto &minEvent = m_events[minIdx];
    hr = context->getContext()->GetData(minEvent.query.Get(), &compute, sizeof(compute), 0);
    if (FAILED(hr) || compute != 1)
        return eNvFlexFail;

    if (pUid) *pUid = minEvent.uid;
    minEvent.state = eEventStateInactive;
    return eNvFlexSuccess;
}

EventQueueD3D11::Event *EventQueueD3D11::getNewEvent() {
    for (auto &ev : m_events) {
        if (ev.state == eEventStateInactive) return &ev;
    }
    Event newEvent;
    newEvent.uid = 0;
    newEvent.pushID = 0;
    newEvent.state = eEventStateInactive;
    m_events.push_back(newEvent);
    return &m_events.back();
}

void ContextD3D11::State::reset() {
    SafeRelease(indexBuffer);
    SafeRelease(layout);
    SafeRelease(vertexBuffers);

    SafeRelease(blendState);
    SafeRelease(depthStencilState);

    SafeRelease(renderTargetViews);

    SafeRelease(depthStencilView);

    SafeRelease(rasterizerState);

    SafeRelease(vsConstantBuffers);
    SafeRelease(vsSamplers);
    SafeRelease(vsSrvs);

    SafeRelease(psConstantBuffers);
    SafeRelease(psSamplers);
    SafeRelease(psSrvs);

    SafeRelease(csConstantBuffers);
    SafeRelease(csSamplers);
    SafeRelease(csSrvs);
    SafeRelease(csUavs);

    SafeRelease(computeShader);

    SafeRelease(vertexShader);
    SafeRelease(pixelShader);
    SafeRelease(geometryShader);
    SafeRelease(hullShader);
    SafeRelease(domainShader);
}

ContextD3D11::State::~State() {
    reset();
}

LUID getAdapterLUID_D3D11(ID3D11Device *d3dDevice) {
    ComPtr<IDXGIAdapter1> pAdapter1;
    LUID adapterLuid = {0, 0};
    if (d3dDevice) {
        ComPtr<IDXGIDevice> dxgiDevice;
        if (FAILED(d3dDevice->QueryInterface(IID_PPV_ARGS(&dxgiDevice))))
            return adapterLuid;

        ComPtr<IDXGIAdapter> pAdapter;
        if (FAILED(dxgiDevice->GetAdapter(&pAdapter))) {
            return adapterLuid;
        }

        if (FAILED(pAdapter->QueryInterface(IID_PPV_ARGS(&pAdapter1)))) {
            return adapterLuid;
        }
    }

    DXGI_ADAPTER_DESC1 desc;
    pAdapter1->GetDesc1(&desc);
    return desc.AdapterLuid;
}

IDXGIAdapter1 *getDXGIAdapterD3D11(ID3D11Device *d3dDevice, HMODULE dxgiModule) {
    IDXGIFactory4 *pFactory2 = 0;
    auto createDXGIFactory2 = (HRESULT(WINAPI *)(UINT, const IID *, void **))GetProcAddress(
        dxgiModule, "CreateDXGIFactory2");

    auto AdapterLUID_D3D11 = getAdapterLUID_D3D11(d3dDevice);

    UINT debugFlags = DXGI_CREATE_FACTORY_DEBUG;
    if (FAILED(createDXGIFactory2(debugFlags, &IID_IDXGIFactory2, (void **)&pFactory2)))
        return 0;

    IDXGIAdapter1 *pAdapter1 = 0;
    for (int adapterIdx = 0; SUCCEEDED(pFactory2->EnumAdapters1(adapterIdx, &pAdapter1));
         ++adapterIdx) {
        DXGI_ADAPTER_DESC1 adapterDesc1;
        pAdapter1->GetDesc1(&adapterDesc1);
        if (adapterDesc1.AdapterLuid.LowPart == AdapterLUID_D3D11.LowPart &&
            adapterDesc1.AdapterLuid.HighPart == AdapterLUID_D3D11.HighPart)
            break;
        SafeRelease(pAdapter1);
    }

    SafeRelease(pFactory2);
    return pAdapter1;
}

bool FlexDedicatedDeviceAvailableD3D11(NvFlexContext *renderContext) {
    bool foundDedicatedDevice = 0;
    HMODULE d3d12module = LoadLibraryW(L"d3d12.dll");
    HMODULE dxgimodule = LoadLibraryW(L"DXGI.dll");
    if (d3d12module && dxgimodule && renderContext) {
        NvFlexContextDescD3D11 renderContextDesc = {};
        NvFlexUpdateContextDescD3D11(renderContext, &renderContextDesc);
        auto adapterRender = getDXGIAdapterD3D11(renderContextDesc.device, dxgimodule);
        auto factory = getDXGIFactoryD3D(adapterRender);

        DXGI_ADAPTER_DESC1 renderDesc;
        adapterRender->GetDesc1(&renderDesc);

        UINT adapterIdx = 0;
        IDXGIAdapter1 *adapterTemp = 0;
        while (SUCCEEDED(factory->EnumAdapters1(adapterIdx, &adapterTemp))) {
            DXGI_ADAPTER_DESC1 adapterDesc;
            adapterTemp->GetDesc1(&adapterDesc);

            BOOL sammeLUID =
                adapterDesc.AdapterLuid.LowPart == renderDesc.AdapterLuid.LowPart &&
                adapterDesc.AdapterLuid.HighPart == renderDesc.AdapterLuid.HighPart;
            BOOL softwareAdapter = (adapterDesc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0;
            auto createDevice = (HRESULT(WINAPI *)(
                IUnknown *, D3D_FEATURE_LEVEL, const GUID *,
                void **))GetProcAddress(d3d12module, "D3D12CreateDevice");
            BOOL supportD3D12 = SUCCEEDED(
                createDevice(adapterTemp, D3D_FEATURE_LEVEL_11_0, &IID_ID3D12Device, 0));
            if (!sammeLUID && !softwareAdapter && supportD3D12)
                foundDedicatedDevice = 1;
            SafeRelease(adapterTemp);
            ++adapterIdx;
        }

        SafeRelease(factory);
        SafeRelease(adapterRender);
    }

    if (d3d12module) {
        FreeLibrary(d3d12module);
        d3d12module = 0;
    }

    if (dxgimodule) {
        FreeLibrary(dxgimodule);
        dxgimodule = 0;
    }
    return foundDedicatedDevice;
}

bool FlexDedicatedDeviceQueueAvailableD3D11(NvFlexContext *renderContext) {
    bool available = 0;
    HMODULE d3d12module = LoadLibraryW(L"d3d12.dll");
    if (d3d12module) {
        available = 1;
        FreeLibrary(d3d12module);
        d3d12module = 0;
    }
    return available;
}

}  // namespace NvFlex