#include <ethereal/phys_framework/demo_context.h>

#include <ethereal/phys_utils/extrude.h>

#include <donut/core/log.h>
#include <nvrhi/core/foundation.h>
#include <donut/engine/CommonRenderPasses.h>
#include <donut/engine/ShaderFactory.h>
#include <nvrhi/utils.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

// The FleX demo's D3D11 renderer (reference/flex/demo/d3d11: demoContextD3D11.cpp, meshRenderD3D11.cpp,
// pointRenderD3D11.cpp, fluidRenderD3D11.cpp, diffuseRenderD3D11.cpp, debugLineRenderD3D11.cpp, shadowMapD3D11.cpp,
// renderTargetD3D11.cpp and ../d3d/renderParamsD3D.cpp) over nvrhi. The passes, their render states and the values
// of their constants are FleX's; where nvrhi cannot express a state, or FleX's state was undefined, the comment
// says so.

namespace ephys::framework {

using donut::log::error;

namespace {

// ---- The constant buffers of shaders/shaderCommon.h, as the shaders read them --------------------------------

struct MeshShaderConst {
    Matrix44 modelViewProjection;
    Matrix44 modelView;
    Matrix44 objectTransform;
    Matrix44 lightTransform;

    dm::float4 clipPlane;
    dm::float4 fogColor;
    dm::float4 color;
    dm::float4 secondaryColor;

    dm::float4 shadowTaps[12];

    dm::float3 lightPos;
    float _pad0;
    dm::float3 lightDir;
    float _pad1;

    float bias;
    float expand;
    float spotMin;
    float spotMax;

    int32_t grid;
    int32_t tex;
    int32_t colorArray;
    int32_t increaseGfxLoadForAsyncComputeTesting;
};
static_assert(sizeof(MeshShaderConst) == 576);
static_assert(offsetof(MeshShaderConst, lightPos) == 512);

struct PointShaderConst {
    Matrix44 modelView;
    Matrix44 projection;
    Matrix44 lightTransform;

    dm::float4 colors[8];
    dm::float4 shadowTaps[12];

    dm::float3 lightPos;
    float _pad0;
    dm::float3 lightDir;
    float _pad1;

    float pointRadius;
    float pointScale;
    float spotMin;
    float spotMax;

    int32_t mode;
    int32_t _pad2[3];
};
static_assert(sizeof(PointShaderConst) == 576);
static_assert(offsetof(PointShaderConst, pointRadius) == 544);

struct FluidShaderConst {
    Matrix44 modelViewProjection;
    Matrix44 modelView;
    Matrix44 projection;
    Matrix44 inverseModelView;
    Matrix44 inverseProjection;

    dm::float4 invTexScale;

    dm::float3 invViewport;
    float _pad0;

    float blurRadiusWorld;
    float blurScale;
    float blurFalloff;
    int32_t debug;

    dm::float3 lightPos;
    float _pad1;
    dm::float3 lightDir;
    float _pad2;
    Matrix44 lightTransform;

    dm::float4 color;
    dm::float4 clipPosToEye;

    float spotMin;
    float spotMax;
    float ior;
    float pointRadius;

    dm::float4 shadowTaps[12];
};
static_assert(sizeof(FluidShaderConst) == 704);
static_assert(offsetof(FluidShaderConst, lightTransform) == 400);
static_assert(offsetof(FluidShaderConst, spotMin) == 496);

struct DiffuseShaderConst {
    dm::float3 lightPos;
    float pad0;
    dm::float3 lightDir;
    float pad1;
    Matrix44 lightTransform;
    dm::float4 color;

    Matrix44 modelView;
    Matrix44 modelViewProjection;
    Matrix44 projection;

    dm::float4 shadowTaps[12];

    float diffusion;
    float diffuseRadius;
    float diffuseScale;

    float spotMin;
    float spotMax;

    float motionBlurScale;

    float pad3;
    float pad4;
};
static_assert(sizeof(DiffuseShaderConst) == 528);
static_assert(offsetof(DiffuseShaderConst, diffusion) == 496);

struct LineVertex {
    dm::float3 position;
    dm::float4 color;
};
static_assert(sizeof(LineVertex) == 28);

// FleX's PassthroughVertex: the vertex of the full-screen quad, in passThroughVS's MeshVertexIn layout.
struct PassthroughVertex {
    dm::float3 position;
    dm::float3 normal;
    dm::float2 texcoords;
    dm::float4 color;
};
static_assert(sizeof(PassthroughVertex) == 48);

// RenderParamsUtilD3D::calcShadowParams' Poisson taps.
const dm::float4 c_ShadowTaps[12] = {
    dm::float4(-0.326212f, -0.40581f, 0.0f, 0.0f), dm::float4(-0.840144f, -0.07358f, 0.0f, 0.0f),
    dm::float4(-0.695914f, 0.457137f, 0.0f, 0.0f), dm::float4(-0.203345f, 0.620716f, 0.0f, 0.0f),
    dm::float4(0.96234f, -0.194983f, 0.0f, 0.0f),  dm::float4(0.473434f, -0.480026f, 0.0f, 0.0f),
    dm::float4(0.519456f, 0.767022f, 0.0f, 0.0f),  dm::float4(0.185461f, -0.893124f, 0.0f, 0.0f),
    dm::float4(0.507431f, 0.064425f, 0.0f, 0.0f),  dm::float4(0.89642f, 0.412458f, 0.0f, 0.0f),
    dm::float4(-0.32194f, -0.932615f, 0.0f, 0.0f), dm::float4(-0.791559f, -0.59771f, 0.0f, 0.0f)};

constexpr uint32_t c_ShadowResolution = 2048;   // demoContextD3D11.cpp: kShadowResolution
constexpr float c_SpotMin = 0.5f;               // DemoContextD3D11 constructor
constexpr float c_SpotMax = 1.0f;
constexpr float c_ShadowBias = 0.075f;
constexpr uint32_t c_ConstantBufferVersions = 1024;  // volatile versions in flight (Vulkan)
constexpr uint64_t c_StreamSize = 64ull << 20;       // the transient vertex / index stream

enum class Pass : uint8_t { Null, Shadow, Light };  // FleX's MESH_DRAW_*

dm::float3 normalized(const dm::float3& v) { return dm::normalize(v); }

}  // namespace

// A mesh in GPU buffers (FleX's GpuMesh): four vertex streams and 32-bit indices.
class GpuMesh {
 public:
    nvrhi::BufferHandle positions;  // float3
    nvrhi::BufferHandle normals;    // float3
    nvrhi::BufferHandle texcoords;  // float2, zero
    nvrhi::BufferHandle colors;     // float4, zero
    nvrhi::BufferHandle indices;
    uint32_t numIndices = 0;
};

struct DemoContext::Impl {
    nvrhi::DeviceHandle device;
    nvrhi::AutoPtr<donut::engine::ShaderFactory> shaderFactory;
    nvrhi::AutoPtr<donut::engine::CommonRenderPasses> commonPasses;
    uint32_t msaaSamples = 1;

    // shaders
    nvrhi::ShaderHandle meshVS, meshPS, meshShadowPS;
    nvrhi::ShaderHandle pointVS, pointGS, pointPS, pointShadowPS;
    nvrhi::ShaderHandle ellipsoidVS, ellipsoidGS, ellipsoidPS;
    nvrhi::ShaderHandle thicknessVS, thicknessGS, thicknessPS;
    nvrhi::ShaderHandle passThroughVS, blurPS, compositePS;
    nvrhi::ShaderHandle diffuseVS, diffuseGS, diffusePS;
    nvrhi::ShaderHandle lineVS, linePS;

    // input layouts
    nvrhi::InputLayoutHandle meshLayout, pointLayout, fluidLayout, thicknessLayout, passThroughLayout, diffuseLayout,
        lineLayout, clothLayout;

    // binding layouts: a constant buffer; with the shadow map; blur; composite
    nvrhi::BindingLayoutHandle cbLayout, cbShadowLayout, blurLayout, compositeLayout;

    // constant buffers (volatile)
    nvrhi::BufferHandle meshCB, pointCB, fluidCB, diffuseCB, lineCB;

    nvrhi::SamplerHandle shadowSampler;  // comparison, linear, clamp
    nvrhi::SamplerHandle linearSampler;  // linear, clamp

    // shadow map (FleX: ShadowMapD3D11, a depth buffer; its colour target was never read and is not made)
    nvrhi::TextureHandle shadowDepth;
    nvrhi::FramebufferHandle shadowFramebuffer;

    // the scene: multisampled colour and depth, and the resolved frame
    uint32_t width = 0, height = 0;
    nvrhi::TextureHandle sceneColor, sceneDepth, resolved;
    nvrhi::FramebufferHandle sceneFramebuffer;

    // the fluid renderer's targets (FluidRendererD3D11: thickness, depth and smoothed depth)
    nvrhi::TextureHandle thicknessColor, thicknessDepth, depthColor, depthDepth, smoothColor;
    nvrhi::FramebufferHandle thicknessFramebuffer, depthFramebuffer, smoothFramebuffer;

    // binding sets
    nvrhi::BindingSetHandle meshShadowSet, meshLightSet, pointShadowSet, pointLightSet, fluidSet, diffuseSet,
        lineSet, blurSet, compositeSet;

    std::unordered_map<uint32_t, nvrhi::GraphicsPipelineHandle> pipelines;

    // the full-screen quad (FluidRendererD3D11::_createScreenQuad)
    nvrhi::BufferHandle quadVertices, quadIndices;

    // the transient stream of immediate meshes and lines (FleX's m_immediateMesh / dynamic buffers)
    nvrhi::BufferHandle stream;
    uint64_t streamOffset = 0;
    std::vector<nvrhi::BufferHandle> retiredStreams;
    nvrhi::BufferHandle zeros;  // zero texcoords and colours for streams a mesh has not
    uint64_t zerosSize = 0;

    // the frame
    nvrhi::ICommandList* commandList = nullptr;
    nvrhi::CommandListHandle uploadList;  // for CreateGpuMesh outside a frame

    Matrix44 view = Matrix44::Identity();
    Matrix44 proj = Matrix44::Identity();  // D3D clip space

    // FleX's m_meshDrawParams
    Pass stage = Pass::Null;
    bool wireframe = false;
    bool cull = true;
    Matrix44 lightTransform = Matrix44::Identity();  // D3D clip space
    dm::float3 lightPos = dm::float3(0.f);
    dm::float3 lightDir = dm::float3(0.f, -1.f, 0.f);
    float bias = c_ShadowBias;
    float expand = 0.0f;
    int grid = 0;
    dm::float4 fogColor = dm::float4(0.f);

    const dm::float4* colors = nullptr;
    dm::float4 defaultColors[8] = {};

    std::vector<LineVertex> lines;

    // ---- helpers ----

    nvrhi::ShaderHandle shader(const char* file, const char* entry, nvrhi::ShaderType type) {
        std::string path = std::string("phys_framework/") + file;
        nvrhi::ShaderHandle s = shaderFactory->CreateShader(path.c_str(), entry, nullptr, type);
        if (!s)
            error("DemoContext: shader %s:%s not found", file, entry);
        return s;
    }

    nvrhi::BufferHandle volatileCB(uint32_t size, const char* name) {
        nvrhi::BufferHandle b;
        device->createBuffer(nvrhi::utils::CreateVolatileConstantBufferDesc(size, name, c_ConstantBufferVersions), &b);
        return b;
    }

    nvrhi::TextureHandle texture(uint32_t w, uint32_t h, nvrhi::Format format, uint32_t samples, bool renderTarget,
                                 bool srv, nvrhi::ResourceStates state, const char* name, bool clear,
                                 const nvrhi::Color& clearValue = nvrhi::Color(0.f)) {
        nvrhi::TextureDesc d;
        d.width = w;
        d.height = h;
        d.format = format;
        d.sampleCount = samples;
        d.dimension = samples > 1 ? nvrhi::TextureDimension::Texture2DMS : nvrhi::TextureDimension::Texture2D;
        d.isRenderTarget = renderTarget;
        d.isShaderResource = srv;
        d.initialState = state;
        d.keepInitialState = true;
        d.debugName = name;
        d.useClearValue = clear;
        d.clearValue = clearValue;
        // a depth buffer that is also read (the shadow map) is typeless underneath (D3D11: R32_TYPELESS)
        d.isTypeless = srv && nvrhi::getFormatInfo(format).hasDepth;
        nvrhi::TextureHandle t;
        device->createTexture(d, &t);
        return t;
    }

    nvrhi::FramebufferHandle framebuffer(nvrhi::ITexture* color, nvrhi::ITexture* depth) {
        nvrhi::FramebufferDesc d;
        if (color)
            d.addColorAttachment(color);
        if (depth)
            d.setDepthAttachment(depth);
        nvrhi::FramebufferHandle f;
        device->createFramebuffer(d, &f);
        return f;
    }

    nvrhi::BindingSetHandle bindingSet(nvrhi::IBindingLayout* layout, std::initializer_list<nvrhi::BindingSetItem> items) {
        nvrhi::BindingSetDesc d;
        for (const nvrhi::BindingSetItem& item : items)
            d.bindings.push_back(item);
        nvrhi::BindingSetHandle s;
        device->createBindingSet(d, layout, &s);
        return s;
    }

    // FleX's raster states: FrontCounterClockwise TRUE, depth clip on, multisample off.
    static nvrhi::RasterState raster(bool wire, nvrhi::RasterCullMode cullMode) {
        nvrhi::RasterState r;
        r.fillMode = wire ? nvrhi::RasterFillMode::Wireframe : nvrhi::RasterFillMode::Solid;
        r.cullMode = cullMode;
        r.frontCounterClockwise = true;
        r.depthClipEnable = true;
        return r;
    }

    // The application's depth state of FleX (appD3D11Ctx): test LESS_EQUAL, write.
    static nvrhi::DepthStencilState depthLessEqual(bool test = true, bool write = true) {
        nvrhi::DepthStencilState d;
        d.depthTestEnable = test;
        d.depthWriteEnable = write;
        d.depthFunc = nvrhi::ComparisonFunc::LessOrEqual;
        d.stencilEnable = false;
        return d;
    }

    enum PipelineKind : uint32_t {
        MeshLight = 1,
        MeshShadow,
        PointLight,
        PointShadow,
        Thickness,
        Ellipsoid,
        Blur,
        Composite,
        Diffuse,
        Lines,
    };

    nvrhi::IGraphicsPipeline* pipeline(PipelineKind kind, uint32_t variant = 0) {
        const uint32_t key = (uint32_t(kind) << 8) | variant;
        auto it = pipelines.find(key);
        if (it != pipelines.end())
            return it->second;

        nvrhi::GraphicsPipelineDesc d;
        nvrhi::IFramebuffer* target = sceneFramebuffer;
        d.renderState.depthStencilState = depthLessEqual();
        switch (kind) {
            case MeshLight:
            case MeshShadow: {
                const bool wire = (variant & 1) != 0;
                const bool culled = (variant & 2) != 0;
                const bool cloth = (variant & 4) != 0;
                d.VS = meshVS;
                d.PS = kind == MeshLight ? meshPS : meshShadowPS;
                d.inputLayout = cloth ? clothLayout : meshLayout;
                d.bindingLayouts = {kind == MeshLight ? cbShadowLayout : cbLayout};
                d.primType = nvrhi::PrimitiveType::TriangleList;
                d.renderState.rasterState = raster(wire, culled ? nvrhi::RasterCullMode::Back : nvrhi::RasterCullMode::None);
                target = kind == MeshLight ? sceneFramebuffer.Get() : shadowFramebuffer.Get();
                break;
            }
            case PointLight:
            case PointShadow:
                d.VS = pointVS;
                d.GS = pointGS;
                d.PS = kind == PointLight ? pointPS : pointShadowPS;
                d.inputLayout = pointLayout;
                d.bindingLayouts = {kind == PointLight ? cbShadowLayout : cbLayout};
                d.primType = nvrhi::PrimitiveType::PointList;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::Back);
                target = kind == PointLight ? sceneFramebuffer.Get() : shadowFramebuffer.Get();
                break;
            case Thickness:
                d.VS = thicknessVS;
                d.GS = thicknessGS;
                d.PS = thicknessPS;
                d.inputLayout = thicknessLayout;
                d.bindingLayouts = {cbLayout};
                d.primType = nvrhi::PrimitiveType::PointList;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::None);
                target = thicknessFramebuffer;
                break;
            case Ellipsoid:
                d.VS = ellipsoidVS;
                d.GS = ellipsoidGS;
                d.PS = ellipsoidPS;
                d.inputLayout = fluidLayout;
                d.bindingLayouts = {cbLayout};
                d.primType = nvrhi::PrimitiveType::PointList;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::None);
                target = depthFramebuffer;
                break;
            case Blur:
                d.VS = passThroughVS;
                d.PS = blurPS;
                d.inputLayout = passThroughLayout;
                d.bindingLayouts = {blurLayout};
                d.primType = nvrhi::PrimitiveType::TriangleStrip;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::None);
                // RenderTargetD3D11 of the smoothed depth: depth test off (it has no depth buffer here)
                d.renderState.depthStencilState = depthLessEqual(false, false);
                target = smoothFramebuffer;
                break;
            case Composite: {
                d.VS = passThroughVS;
                d.PS = compositePS;
                d.inputLayout = passThroughLayout;
                d.bindingLayouts = {compositeLayout};
                d.primType = nvrhi::PrimitiveType::TriangleStrip;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::None);
                // m_compositeBlendState
                auto& blend = d.renderState.blendState.targets[0];
                blend.blendEnable = true;
                blend.srcBlend = nvrhi::BlendFactor::SrcAlpha;
                blend.destBlend = nvrhi::BlendFactor::InvSrcAlpha;
                blend.blendOp = nvrhi::BlendOp::Add;
                blend.srcBlendAlpha = nvrhi::BlendFactor::One;
                blend.destBlendAlpha = nvrhi::BlendFactor::Zero;
                blend.blendOpAlpha = nvrhi::BlendOp::Add;
                break;
            }
            case Diffuse: {
                d.VS = diffuseVS;
                d.GS = diffuseGS;
                d.PS = diffusePS;
                d.inputLayout = diffuseLayout;
                d.bindingLayouts = {cbLayout};
                d.primType = nvrhi::PrimitiveType::PointList;
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::None);
                d.renderState.depthStencilState = depthLessEqual(true, false);
                auto& blend = d.renderState.blendState.targets[0];
                blend.blendEnable = true;
                blend.srcBlend = nvrhi::BlendFactor::One;
                blend.destBlend = nvrhi::BlendFactor::InvSrcAlpha;
                blend.blendOp = nvrhi::BlendOp::Add;
                blend.srcBlendAlpha = nvrhi::BlendFactor::One;
                blend.destBlendAlpha = nvrhi::BlendFactor::InvSrcAlpha;
                blend.blendOpAlpha = nvrhi::BlendOp::Add;
                break;
            }
            case Lines:
                d.VS = lineVS;
                d.PS = linePS;
                d.inputLayout = lineLayout;
                d.bindingLayouts = {cbLayout};
                d.primType = nvrhi::PrimitiveType::LineList;
                // DebugLineRenderD3D11 creates a state without depth test but never binds it: its lines are drawn
                // with the application's depth state, which is what is kept here
                d.renderState.rasterState = raster(false, nvrhi::RasterCullMode::Back);
                break;
        }

        nvrhi::GraphicsPipelineHandle p;
        device->createGraphicsPipeline1(d, target->getFramebufferInfo().getInfo(), &p);
        if (!p)
            error("DemoContext: cannot create pipeline %u.%u", uint32_t(kind), variant);
        pipelines[key] = p;
        return p;
    }

    // ---- the transient stream ----

    static uint64_t align16(uint64_t size) { return (size + 15) & ~uint64_t(15); }

    // Makes room for `size` bytes (the sum of a draw's writes): a frame that needs more than the stream holds gets
    // a larger one, so that every write of one draw lands in the same buffer. The old buffer lives until the frame
    // ends.
    void reserve(uint64_t size) {
        if (streamOffset + size <= stream->getDesc().byteSize)
            return;
        retiredStreams.push_back(stream);
        nvrhi::BufferDesc d = stream->getDesc();
        d.byteSize = std::max(d.byteSize * 2, size * 2);
        stream = nullptr;
        device->createBuffer(d, &stream);
        streamOffset = 0;
    }

    // Writes data into the stream (16-byte aligned) and returns its offset; reserve() first.
    uint64_t write(const void* data, uint64_t size) {
        const uint64_t offset = streamOffset;
        commandList->writeBuffer(stream, data, size_t(size), offset);
        streamOffset += align16(size);
        return offset;
    }

    nvrhi::IBuffer* zeroBuffer(uint64_t size) {
        if (zerosSize < size) {
            zerosSize = std::max<uint64_t>(size, 1 << 20);
            nvrhi::BufferDesc d;
            d.byteSize = zerosSize;
            d.isVertexBuffer = true;
            d.debugName = "DemoContext zeros";
            d.initialState = nvrhi::ResourceStates::VertexBuffer;
            d.keepInitialState = true;
            zeros = nullptr;
            device->createBuffer(d, &zeros);
            std::vector<uint8_t> z(size_t(zerosSize), 0);
            commandList->writeBuffer(zeros, z.data(), z.size());
        }
        return zeros;
    }

    // ---- constant buffer contents (RenderParamsUtilD3D) ----

    MeshShaderConst meshConstants(const dm::float4& color, const dm::float4& secondaryColor,
                                  const Matrix44& objectTransform, bool colorArray) const {
        MeshShaderConst c{};
        c.modelViewProjection = proj * view;  // model is the identity
        c.modelView = view;
        c.objectTransform = objectTransform;
        c.lightTransform = lightTransform;
        c.clipPlane = dm::float4(0.f);
        c.fogColor = fogColor;
        c.color = color;
        c.secondaryColor = secondaryColor;
        std::memcpy(c.shadowTaps, c_ShadowTaps, sizeof(c_ShadowTaps));
        c.lightPos = lightPos;
        c.lightDir = lightDir;
        c.bias = bias;
        c.expand = expand;
        c.spotMin = c_SpotMin;
        c.spotMax = c_SpotMax;
        c.grid = grid;
        c.tex = 0;
        c.colorArray = colorArray ? 1 : 0;
        return c;
    }

    // MeshRendererD3D11::draw: positions and normals as float3 (or float4 for cloth), texcoords, colours, indices.
    struct MeshStreams {
        nvrhi::IBuffer* positions = nullptr;
        uint64_t positionsOffset = 0;
        nvrhi::IBuffer* normals = nullptr;
        uint64_t normalsOffset = 0;
        nvrhi::IBuffer* texcoords = nullptr;
        uint64_t texcoordsOffset = 0;
        nvrhi::IBuffer* colors = nullptr;
        uint64_t colorsOffset = 0;
        nvrhi::IBuffer* indices = nullptr;
        uint32_t indicesOffset = 0;
        uint32_t numIndices = 0;
        bool cloth = false;  // float4 positions and normals
    };

    void drawMesh(const MeshStreams& s, const MeshShaderConst& constants) {
        if (stage == Pass::Null || s.numIndices == 0)
            return;
        const bool light = stage == Pass::Light;
        const uint32_t variant = (wireframe ? 1u : 0u) | (cull ? 2u : 0u) | (s.cloth ? 4u : 0u);
        nvrhi::IGraphicsPipeline* p = pipeline(light ? MeshLight : MeshShadow, variant);
        if (!p)
            return;

        commandList->writeBuffer(meshCB, &constants, sizeof(constants));

        nvrhi::IFramebuffer* fb = light ? sceneFramebuffer.Get() : shadowFramebuffer.Get();
        const nvrhi::FramebufferInfoEx& info = fb->getFramebufferInfo();

        nvrhi::GraphicsState state;
        state.pipeline = p;
        state.framebuffer = fb;
        state.bindings = {light ? meshLightSet.Get() : meshShadowSet.Get()};
        state.vertexBuffers = {nvrhi::VertexBufferBinding{s.positions, 0, s.positionsOffset},
                               nvrhi::VertexBufferBinding{s.normals, 1, s.normalsOffset},
                               nvrhi::VertexBufferBinding{s.texcoords, 2, s.texcoordsOffset},
                               nvrhi::VertexBufferBinding{s.colors, 3, s.colorsOffset}};
        state.indexBuffer = nvrhi::IndexBufferBinding{s.indices, nvrhi::Format::R32_UINT, s.indicesOffset};
        state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
        commandList->setGraphicsState(state);

        nvrhi::DrawArguments args;
        args.vertexCount = s.numIndices;
        commandList->drawIndexed(args);
    }

    // An immediate mesh: the data goes into the stream (GpuMeshD3D11::updateData on m_immediateMesh).
    void drawImmediate(const dm::float3* positions, const dm::float3* normals, const dm::float4* meshColors,
                       const uint32_t* indices, uint32_t numVertices, uint32_t numIndices,
                       const MeshShaderConst& constants) {
        if (!numVertices || !numIndices || stage == Pass::Null)
            return;
        reserve(align16(sizeof(dm::float3) * numVertices) * 2 + (meshColors ? align16(sizeof(dm::float4) * numVertices) : 0) +
                align16(sizeof(uint32_t) * numIndices));
        nvrhi::IBuffer* z = zeroBuffer(sizeof(dm::float4) * numVertices);
        MeshStreams s;
        s.positions = s.normals = s.indices = stream;
        s.positionsOffset = write(positions, sizeof(dm::float3) * numVertices);
        s.normalsOffset = write(normals, sizeof(dm::float3) * numVertices);
        s.texcoords = z;
        if (meshColors) {
            s.colors = stream;
            s.colorsOffset = write(meshColors, sizeof(dm::float4) * numVertices);
        } else {
            s.colors = z;
        }
        s.indicesOffset = uint32_t(write(indices, sizeof(uint32_t) * numIndices));
        s.numIndices = numIndices;
        drawMesh(s, constants);
    }

    nvrhi::IFramebuffer* currentTarget() const {
        return stage == Pass::Shadow ? shadowFramebuffer.Get() : sceneFramebuffer.Get();
    }
};

DemoContext::DemoContext(nvrhi::IDevice* device, donut::engine::ShaderFactory* shaderFactory,
                         donut::engine::CommonRenderPasses* commonPasses, uint32_t msaaSamples)
    : m_Impl(std::make_unique<Impl>()) {
    m_Impl->device = device;
    m_Impl->shaderFactory = shaderFactory;
    m_Impl->commonPasses = commonPasses;
    m_Impl->msaaSamples = std::max(1u, msaaSamples);
}

DemoContext::~DemoContext() = default;

bool DemoContext::Init() {
    Impl& s = *m_Impl;
    nvrhi::IDevice* device = s.device;

    s.meshVS = s.shader("meshVS.hlsl", "meshVS", nvrhi::ShaderType::Vertex);
    s.meshPS = s.shader("meshPS.hlsl", "meshPS", nvrhi::ShaderType::Pixel);
    s.meshShadowPS = s.shader("meshShadowPS.hlsl", "meshPS_Shadow", nvrhi::ShaderType::Pixel);
    s.pointVS = s.shader("pointVS.hlsl", "pointVS", nvrhi::ShaderType::Vertex);
    s.pointGS = s.shader("pointGS.hlsl", "pointGS", nvrhi::ShaderType::Geometry);
    s.pointPS = s.shader("pointPS.hlsl", "pointPS", nvrhi::ShaderType::Pixel);
    s.pointShadowPS = s.shader("pointShadowPS.hlsl", "pointShadowPS", nvrhi::ShaderType::Pixel);
    s.ellipsoidVS = s.shader("ellipsoidDepthVS.hlsl", "ellipsoidDepthVS", nvrhi::ShaderType::Vertex);
    s.ellipsoidGS = s.shader("ellipsoidDepthGS.hlsl", "ellipsoidDepthGS", nvrhi::ShaderType::Geometry);
    s.ellipsoidPS = s.shader("ellipsoidDepthPS.hlsl", "ellipsoidDepthPS", nvrhi::ShaderType::Pixel);
    s.thicknessVS = s.shader("pointThicknessVS.hlsl", "pointThicknessVS", nvrhi::ShaderType::Vertex);
    s.thicknessGS = s.shader("pointThicknessGS.hlsl", "pointThicknessGS", nvrhi::ShaderType::Geometry);
    s.thicknessPS = s.shader("pointThicknessPS.hlsl", "pointThicknessPS", nvrhi::ShaderType::Pixel);
    s.passThroughVS = s.shader("passThroughVS.hlsl", "passThroughVS", nvrhi::ShaderType::Vertex);
    s.blurPS = s.shader("blurDepthPS.hlsl", "blurDepthPS", nvrhi::ShaderType::Pixel);
    s.compositePS = s.shader("compositePS.hlsl", "compositePS", nvrhi::ShaderType::Pixel);
    s.diffuseVS = s.shader("diffuseVS.hlsl", "diffuseVS", nvrhi::ShaderType::Vertex);
    s.diffuseGS = s.shader("diffuseGS.hlsl", "diffuseGS", nvrhi::ShaderType::Geometry);
    s.diffusePS = s.shader("diffusePS.hlsl", "diffusePS", nvrhi::ShaderType::Pixel);
    s.lineVS = s.shader("debugLineVS.hlsl", "debugLineVS", nvrhi::ShaderType::Vertex);
    s.linePS = s.shader("debugLinePS.hlsl", "debugLinePS", nvrhi::ShaderType::Pixel);
    for (nvrhi::IShader* sh : {s.meshVS.Get(), s.meshPS.Get(), s.meshShadowPS.Get(), s.pointVS.Get(), s.pointGS.Get(),
                               s.pointPS.Get(), s.pointShadowPS.Get(), s.ellipsoidVS.Get(), s.ellipsoidGS.Get(),
                               s.ellipsoidPS.Get(), s.thicknessVS.Get(), s.thicknessGS.Get(), s.thicknessPS.Get(),
                               s.passThroughVS.Get(), s.blurPS.Get(), s.compositePS.Get(), s.diffuseVS.Get(),
                               s.diffuseGS.Get(), s.diffusePS.Get(), s.lineVS.Get(), s.linePS.Get()})
        if (!sh)
            return false;

    // ---- input layouts (the D3D11 renderers' input element descs) ----
    {
        // MeshRendererD3D11: one stream per attribute
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGB32_FLOAT).setBufferIndex(0).setElementStride(12),
            nvrhi::VertexAttributeDesc().setName("NORMAL").setFormat(nvrhi::Format::RGB32_FLOAT).setBufferIndex(1).setElementStride(12),
            nvrhi::VertexAttributeDesc().setName("TEXCOORD").setFormat(nvrhi::Format::RG32_FLOAT).setBufferIndex(2).setElementStride(8),
            nvrhi::VertexAttributeDesc().setName("COLOR").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(3).setElementStride(16)};
        device->createInputLayout(a, 4, s.meshVS, &s.meshLayout);
        // the same for cloth drawn from the particle arrays: float4 positions and normals, of which xyz are read
        a[0].setElementStride(16);
        a[1].setElementStride(16);
        device->createInputLayout(a, 4, s.meshVS, &s.clothLayout);
    }
    {
        // PointRendererD3D11: the position, and one buffer read as the density and as the phase
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(0).setElementStride(16),
            nvrhi::VertexAttributeDesc().setName("DENSITY").setFormat(nvrhi::Format::R32_FLOAT).setBufferIndex(1).setElementStride(4),
            nvrhi::VertexAttributeDesc().setName("PHASE").setFormat(nvrhi::Format::R32_SINT).setBufferIndex(2).setElementStride(4)};
        device->createInputLayout(a, 3, s.pointVS, &s.pointLayout);
    }
    {
        // FluidRendererD3D11 (ellipsoids): position and the three anisotropy axes, one stream each
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(0).setElementStride(16),
            nvrhi::VertexAttributeDesc().setName("U").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(1).setElementStride(16),
            nvrhi::VertexAttributeDesc().setName("V").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(2).setElementStride(16),
            nvrhi::VertexAttributeDesc().setName("W").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(3).setElementStride(16)};
        device->createInputLayout(a, 4, s.ellipsoidVS, &s.fluidLayout);
        // the thickness pass: the position only, which is all its vertex shader reads (FleX's D3D11 layout gave
        // it all four; nvrhi numbers Vulkan locations by attribute and the validation layer reports attributes a
        // shader does not consume)
        device->createInputLayout(a, 1, s.thicknessVS, &s.thicknessLayout);
    }
    {
        // the quad: PassthroughVertex, interleaved; passThroughVS reads the position and the texture coordinate
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGB32_FLOAT).setOffset(0).setElementStride(48),
            nvrhi::VertexAttributeDesc().setName("TEXCOORD").setFormat(nvrhi::Format::RG32_FLOAT).setOffset(24).setElementStride(48)};
        device->createInputLayout(a, 2, s.passThroughVS, &s.passThroughLayout);
    }
    {
        // DiffuseRendererD3D11
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(0).setElementStride(16),
            nvrhi::VertexAttributeDesc().setName("VELOCITY").setFormat(nvrhi::Format::RGBA32_FLOAT).setBufferIndex(1).setElementStride(16)};
        device->createInputLayout(a, 2, s.diffuseVS, &s.diffuseLayout);
    }
    {
        // DebugLineRenderD3D11
        nvrhi::VertexAttributeDesc a[] = {
            nvrhi::VertexAttributeDesc().setName("POSITION").setFormat(nvrhi::Format::RGB32_FLOAT).setOffset(0).setElementStride(28),
            nvrhi::VertexAttributeDesc().setName("COLOR").setFormat(nvrhi::Format::RGBA32_FLOAT).setOffset(12).setElementStride(28)};
        device->createInputLayout(a, 2, s.lineVS, &s.lineLayout);
    }

    // ---- binding layouts ----
    {
        nvrhi::BindingLayoutDesc d;
        d.visibility = nvrhi::ShaderType::All;
        d.bindings = {nvrhi::BindingLayoutItem::VolatileConstantBuffer(0)};
        device->createBindingLayout(d, &s.cbLayout);

        d.bindings = {nvrhi::BindingLayoutItem::VolatileConstantBuffer(0), nvrhi::BindingLayoutItem::Texture_SRV(0),
                      nvrhi::BindingLayoutItem::Sampler(0)};
        device->createBindingLayout(d, &s.cbShadowLayout);

        d.bindings = {nvrhi::BindingLayoutItem::VolatileConstantBuffer(0), nvrhi::BindingLayoutItem::Texture_SRV(0),
                      nvrhi::BindingLayoutItem::Texture_SRV(1), nvrhi::BindingLayoutItem::Sampler(0)};
        device->createBindingLayout(d, &s.blurLayout);

        d.bindings = {nvrhi::BindingLayoutItem::VolatileConstantBuffer(0), nvrhi::BindingLayoutItem::Texture_SRV(0),
                      nvrhi::BindingLayoutItem::Texture_SRV(1), nvrhi::BindingLayoutItem::Texture_SRV(2),
                      nvrhi::BindingLayoutItem::Sampler(0), nvrhi::BindingLayoutItem::Sampler(1)};
        device->createBindingLayout(d, &s.compositeLayout);
    }

    s.meshCB = s.volatileCB(sizeof(MeshShaderConst), "MeshShaderConst");
    s.pointCB = s.volatileCB(sizeof(PointShaderConst), "PointShaderConst");
    s.fluidCB = s.volatileCB(sizeof(FluidShaderConst), "FluidShaderConst");
    s.diffuseCB = s.volatileCB(sizeof(DiffuseShaderConst), "DiffuseShaderConst");
    s.lineCB = s.volatileCB(sizeof(Matrix44), "DebugRenderConst");

    {
        // ShadowMapD3D11's m_linearSampler: COMPARISON_MIN_MAG_MIP_LINEAR, clamp. nvrhi's comparison samplers
        // compare with LESS where FleX's used LESS_EQUAL (nvrhi has no setting for it); the two differ only
        // for a reference exactly equal to the stored depth.
        nvrhi::SamplerDesc d;
        d.setAllFilters(true).setAllAddressModes(nvrhi::SamplerAddressMode::Clamp);
        d.reductionType = nvrhi::SamplerReductionType::Comparison;
        device->createSampler(d, &s.shadowSampler);
        // RenderTargetD3D11's m_linearSampler
        nvrhi::SamplerDesc l;
        l.setAllFilters(true).setAllAddressModes(nvrhi::SamplerAddressMode::Clamp);
        device->createSampler(l, &s.linearSampler);
    }

    // the shadow map
    s.shadowDepth = s.texture(c_ShadowResolution, c_ShadowResolution, nvrhi::Format::D32, 1, true, true,
                              nvrhi::ResourceStates::ShaderResource, "shadow map", true, nvrhi::Color(1.f));
    s.shadowFramebuffer = s.framebuffer(nullptr, s.shadowDepth);

    // the quad
    {
        const PassthroughVertex vertices[4] = {
            {{-1.0f, -1.0f, 0.0f}, {0, 1, 0}, {0.0f, 0.0f}, {1, 1, 1, 1}},
            {{1.0f, -1.0f, 0.0f}, {0, 1, 0}, {1.0f, 0.0f}, {1, 1, 1, 1}},
            {{1.0f, 1.0f, 0.0f}, {0, 1, 0}, {1.0f, 1.0f}, {1, 1, 1, 1}},
            {{-1.0f, 1.0f, 0.0f}, {0, 1, 0}, {0.0f, 1.0f}, {1, 1, 1, 1}},
        };
        const uint32_t indices[4] = {0, 1, 3, 2};

        nvrhi::BufferDesc vd;
        vd.byteSize = sizeof(vertices);
        vd.isVertexBuffer = true;
        vd.debugName = "fluid quad vertices";
        vd.initialState = nvrhi::ResourceStates::VertexBuffer;
        vd.keepInitialState = true;
        device->createBuffer(vd, &s.quadVertices);
        nvrhi::BufferDesc id;
        id.byteSize = sizeof(indices);
        id.isIndexBuffer = true;
        id.debugName = "fluid quad indices";
        id.initialState = nvrhi::ResourceStates::IndexBuffer;
        id.keepInitialState = true;
        device->createBuffer(id, &s.quadIndices);

        device->createCommandList(nvrhi::CommandListParameters(), &s.uploadList);
        s.uploadList->open();
        s.uploadList->writeBuffer(s.quadVertices, vertices, sizeof(vertices));
        s.uploadList->writeBuffer(s.quadIndices, indices, sizeof(indices));
        s.uploadList->close();
        device->executeCommandList(s.uploadList);
    }

    // the stream
    {
        nvrhi::BufferDesc d;
        d.byteSize = c_StreamSize;
        d.isVertexBuffer = true;
        d.isIndexBuffer = true;
        d.debugName = "DemoContext stream";
        d.initialState = nvrhi::ResourceStates::VertexBuffer;
        d.keepInitialState = true;
        device->createBuffer(d, &s.stream);
    }

    s.meshShadowSet = s.bindingSet(s.cbLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.meshCB)});
    s.meshLightSet = s.bindingSet(s.cbShadowLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.meshCB),
                                                     nvrhi::BindingSetItem::Texture_SRV(0, s.shadowDepth),
                                                     nvrhi::BindingSetItem::Sampler(0, s.shadowSampler)});
    s.pointShadowSet = s.bindingSet(s.cbLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.pointCB)});
    s.pointLightSet = s.bindingSet(s.cbShadowLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.pointCB),
                                                      nvrhi::BindingSetItem::Texture_SRV(0, s.shadowDepth),
                                                      nvrhi::BindingSetItem::Sampler(0, s.shadowSampler)});
    s.fluidSet = s.bindingSet(s.cbLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.fluidCB)});
    s.diffuseSet = s.bindingSet(s.cbLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.diffuseCB)});
    s.lineSet = s.bindingSet(s.cbLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.lineCB)});

    return s.meshLayout && s.clothLayout && s.pointLayout && s.fluidLayout && s.thicknessLayout && s.passThroughLayout && s.diffuseLayout &&
           s.lineLayout && s.cbLayout && s.cbShadowLayout && s.blurLayout && s.compositeLayout && s.meshCB &&
           s.pointCB && s.fluidCB && s.diffuseCB && s.lineCB && s.shadowSampler && s.linearSampler &&
           s.shadowDepth && s.shadowFramebuffer && s.stream && s.meshShadowSet && s.meshLightSet &&
           s.pointShadowSet && s.pointLightSet && s.fluidSet && s.diffuseSet && s.lineSet;
}

void DemoContext::Resize(uint32_t width, uint32_t height) {
    Impl& s = *m_Impl;
    if (width == s.width && height == s.height && s.sceneFramebuffer)
        return;
    s.width = width;
    s.height = height;

    // the scene: FleX's back buffer (R8G8B8A8_UNORM, multisampled; the shaders write gamma-encoded colour) and
    // its D32 depth buffer
    s.sceneColor = s.texture(width, height, nvrhi::Format::RGBA8_UNORM, s.msaaSamples, true, true,
                             nvrhi::ResourceStates::RenderTarget, "scene colour", false);
    s.sceneDepth = s.texture(width, height, nvrhi::Format::D32, s.msaaSamples, true, false,
                             nvrhi::ResourceStates::DepthWrite, "scene depth", true, nvrhi::Color(1.f));
    s.sceneFramebuffer = s.framebuffer(s.sceneColor, s.sceneDepth);
    s.resolved = s.texture(width, height, nvrhi::Format::RGBA8_UNORM, 1, true, true,
                           nvrhi::ResourceStates::ShaderResource, "resolved frame", false);

    // the fluid renderer (FluidRendererD3D11::init: three R32F targets, the first two with a depth buffer)
    s.thicknessColor = s.texture(width, height, nvrhi::Format::R32_FLOAT, 1, true, true,
                                 nvrhi::ResourceStates::ShaderResource, "fluid thickness", true, nvrhi::Color(0.f));
    s.thicknessDepth = s.texture(width, height, nvrhi::Format::D32, 1, true, false,
                                 nvrhi::ResourceStates::DepthWrite, "fluid thickness depth", true, nvrhi::Color(1.f));
    s.depthColor = s.texture(width, height, nvrhi::Format::R32_FLOAT, 1, true, true,
                             nvrhi::ResourceStates::ShaderResource, "fluid depth", true, nvrhi::Color(0.f));
    s.depthDepth = s.texture(width, height, nvrhi::Format::D32, 1, true, false, nvrhi::ResourceStates::DepthWrite,
                             "fluid depth depth", true, nvrhi::Color(1.f));
    s.smoothColor = s.texture(width, height, nvrhi::Format::R32_FLOAT, 1, true, true,
                              nvrhi::ResourceStates::ShaderResource, "fluid smoothed depth", true, nvrhi::Color(0.f));
    s.thicknessFramebuffer = s.framebuffer(s.thicknessColor, s.thicknessDepth);
    s.depthFramebuffer = s.framebuffer(s.depthColor, s.depthDepth);
    s.smoothFramebuffer = s.framebuffer(s.smoothColor, nullptr);

    s.blurSet = s.bindingSet(s.blurLayout, {nvrhi::BindingSetItem::ConstantBuffer(0, s.fluidCB),
                                            nvrhi::BindingSetItem::Texture_SRV(0, s.depthColor),
                                            nvrhi::BindingSetItem::Texture_SRV(1, s.thicknessColor),
                                            nvrhi::BindingSetItem::Sampler(0, s.linearSampler)});
    s.compositeSet = s.bindingSet(s.compositeLayout,
                                  {nvrhi::BindingSetItem::ConstantBuffer(0, s.fluidCB),
                                   nvrhi::BindingSetItem::Texture_SRV(0, s.smoothColor),
                                   nvrhi::BindingSetItem::Texture_SRV(1, s.resolved),
                                   nvrhi::BindingSetItem::Texture_SRV(2, s.shadowDepth),
                                   nvrhi::BindingSetItem::Sampler(0, s.linearSampler),
                                   nvrhi::BindingSetItem::Sampler(1, s.shadowSampler)});
}

uint32_t DemoContext::GetWidth() const { return m_Impl->width; }
uint32_t DemoContext::GetHeight() const { return m_Impl->height; }

void DemoContext::StartFrame(nvrhi::ICommandList* commandList, const dm::float4& clearColor) {
    Impl& s = *m_Impl;
    s.commandList = commandList;
    s.streamOffset = 0;
    s.retiredStreams.clear();

    commandList->clearTextureFloat(s.sceneColor, nvrhi::AllSubresources,
                                   nvrhi::Color(clearColor.x, clearColor.y, clearColor.z, clearColor.w));
    commandList->clearDepthStencilTexture(s.sceneDepth, nvrhi::AllSubresources, true, 1.0f, false, 0);

    // DemoContextD3D11::startFrame resets the mesh state
    s.stage = Pass::Light;
    s.wireframe = false;
    s.cull = true;
    s.view = Matrix44::Identity();
    s.proj = Matrix44::Identity();
}

void DemoContext::EndFrame(nvrhi::IFramebuffer* presentFramebuffer) {
    Impl& s = *m_Impl;
    if (s.msaaSamples > 1)
        s.commandList->resolveTexture(s.resolved, nvrhi::AllSubresources, s.sceneColor, nvrhi::AllSubresources);
    else
        s.commandList->copyTexture1(s.resolved, nvrhi::TextureSlice(), s.sceneColor, nvrhi::TextureSlice());
    if (presentFramebuffer)
        s.commonPasses->BlitTexture(s.commandList, presentFramebuffer, s.resolved);
    s.commandList = nullptr;
}

nvrhi::ITexture* DemoContext::GetFrameTexture() const { return m_Impl->resolved; }

void DemoContext::SetColors(const dm::float4* colors) { m_Impl->colors = colors; }

void DemoContext::SetView(const Matrix44& view, const Matrix44& projection) {
    m_Impl->view = view;
    m_Impl->proj = ConvertToD3DProjection(projection);
}

void DemoContext::GetViewRay(int x, int y, dm::float3& origin, dm::float3& dir) const {
    // XMVector3Unproject of the D3D11 context, with y already from the top
    const Impl& s = *m_Impl;
    const Matrix44 inv = Inverse(s.proj * s.view);
    const float ndcX = (float(x) + 0.0f) / float(std::max(1u, s.width)) * 2.0f - 1.0f;
    const float ndcY = 1.0f - (float(y) + 0.0f) / float(std::max(1u, s.height)) * 2.0f;

    const dm::float4 n = inv * dm::float4(ndcX, ndcY, 0.0f, 1.0f);
    const dm::float4 f = inv * dm::float4(ndcX, ndcY, 1.0f, 1.0f);
    origin = dm::float3(n.x, n.y, n.z) / n.w;
    const dm::float3 farPoint = dm::float3(f.x, f.y, f.z) / f.w;
    dir = dm::normalize(farPoint - origin);
}

void DemoContext::SetFillMode(bool wire) { m_Impl->wireframe = wire; }
void DemoContext::SetCullMode(bool enabled) { m_Impl->cull = enabled; }

void DemoContext::BindSolidShader(const dm::float3& lightPos, const dm::float3& lightTarget,
                                  const Matrix44& lightTransform, float /*bias*/, const dm::float4& fogColor) {
    // FleX's D3D renderers ignore the bias argument and use their m_shadowBias (calcShadowParams)
    Impl& s = *m_Impl;
    s.stage = Pass::Light;
    s.grid = 0;
    s.fogColor = fogColor;
    s.lightTransform = ConvertToD3DProjection(lightTransform);
    s.lightPos = lightPos;
    s.lightDir = normalized(lightTarget - lightPos);
    s.bias = c_ShadowBias;
}

void DemoContext::UnbindSolidShader() { m_Impl->stage = Pass::Null; }

void DemoContext::ShadowBegin() {
    Impl& s = *m_Impl;
    // ShadowMapD3D11::bindAndClear
    s.commandList->clearDepthStencilTexture(s.shadowDepth, nvrhi::AllSubresources, true, 1.0f, false, 0);
    s.stage = Pass::Shadow;
}

void DemoContext::ShadowEnd() {
    Impl& s = *m_Impl;
    // back to the frame buffer, whose depth FleX clears here
    s.commandList->clearDepthStencilTexture(s.sceneDepth, nvrhi::AllSubresources, true, 1.0f, false, 0);
    s.stage = Pass::Null;
}

void DemoContext::DrawMesh(const utils::Mesh* m, const dm::float3& color) {
    Impl& s = *m_Impl;
    if (!m || m->m_positions.empty() || m->m_indices.empty())
        return;
    const bool colorArray = !m->m_colours.empty();
    const MeshShaderConst c =
        s.meshConstants(dm::float4(color, 0.0f), dm::float4(color, 0.0f), Matrix44::Identity(), colorArray);
    std::vector<dm::float3> normals;
    const dm::float3* n = m->m_normals.data();
    if (m->m_normals.size() < m->m_positions.size()) {
        normals.assign(m->m_positions.size(), dm::float3(0.0f, 1.0f, 0.0f));
        n = normals.data();
    }
    s.drawImmediate(m->m_positions.data(), n, colorArray ? m->m_colours.data() : nullptr, m->m_indices.data(),
                    m->GetNumVertices(), uint32_t(m->m_indices.size()), c);
}

void DemoContext::DrawCloth(const ClothRenderBuffers& cloth, int colorIndex, float expand, bool twosided) {
    Impl& s = *m_Impl;
    if (!cloth.numTriangles || !cloth.positions || !cloth.normals || !cloth.indices || s.stage == Pass::Null)
        return;

    if (twosided)
        s.cull = false;

    const dm::float4* colors = s.colors ? s.colors : s.defaultColors;
    s.bias = 0.0f;
    s.expand = expand;
    // FleX's Colour * 1.5f scales alpha as well
    const MeshShaderConst c =
        s.meshConstants(colors[colorIndex + 1] * 1.5f, colors[colorIndex] * 1.5f, Matrix44::Identity(), false);

    Impl::MeshStreams streams;
    streams.cloth = true;
    streams.positions = cloth.positions;
    streams.normals = cloth.normals;
    nvrhi::IBuffer* z = s.zeroBuffer(uint64_t(cloth.numPositions) * 16);
    streams.texcoords = z;
    streams.colors = z;
    streams.indices = cloth.indices;
    streams.indicesOffset = cloth.firstTriangle * 3 * uint32_t(sizeof(uint32_t));
    streams.numIndices = cloth.numTriangles * 3;
    s.drawMesh(streams, c);

    if (twosided)
        s.cull = true;
    s.bias = c_ShadowBias;
    s.expand = 0.0f;
}

void DemoContext::DrawRope(const dm::float4* positions, const int* indices, int numIndices, float radius, int color) {
    Impl& s = *m_Impl;
    if (numIndices < 2)
        return;

    std::vector<dm::float3> vertices;
    std::vector<dm::float3> normals;
    std::vector<int> triangles;

    // the curve
    std::vector<dm::float3> curve(static_cast<size_t>(numIndices));
    for (int i = 0; i < numIndices; ++i)
        curve[i] = dm::float3(positions[indices[i]].x, positions[indices[i]].y, positions[indices[i]].z);

    const int resolution = 8;
    const int smoothing = 3;

    utils::Extrude(curve.data(), int(curve.size()), vertices, normals, triangles, radius, resolution, smoothing);

    s.cull = false;
    const dm::float4* colors = s.colors ? s.colors : s.defaultColors;
    const MeshShaderConst c =
        s.meshConstants(colors[color % 8] * 1.5f, colors[color % 8] * 1.5f, Matrix44::Identity(), false);
    s.drawImmediate(vertices.data(), normals.data(), nullptr, reinterpret_cast<const uint32_t*>(triangles.data()),
                    uint32_t(vertices.size()), uint32_t(triangles.size()), c);
    s.cull = true;
}

void DemoContext::DrawPlane(const dm::float4& p, bool color) {
    Impl& s = *m_Impl;

    std::vector<dm::float3> vertices;
    std::vector<dm::float3> normals;
    std::vector<uint32_t> indices;

    dm::float3 u, v;
    const dm::float3 n(p.x, p.y, p.z);
    utils::BasisFromVector(n, u, v);

    const dm::float3 c = n * -p.w;

    dm::float4 meshColor = dm::float4(0.9f, 0.9f, 0.9f, 1.0f);
    if (color)
        meshColor = p * 0.5f + dm::float4(0.5f);

    const float kSize = 200.0f;
    const int kGrid = 3;

    // a grid of quads, for the precision of z
    for (int x = -kGrid; x <= kGrid; ++x) {
        for (int y = -kGrid; y <= kGrid; ++y) {
            const dm::float3 coff = c + u * float(x) * kSize * 2.0f + v * float(y) * kSize * 2.0f;

            const uint32_t indexStart = uint32_t(vertices.size());

            vertices.push_back(coff + u * kSize + v * kSize);
            vertices.push_back(coff - u * kSize + v * kSize);
            vertices.push_back(coff - u * kSize - v * kSize);
            vertices.push_back(coff + u * kSize - v * kSize);

            normals.push_back(n);
            normals.push_back(n);
            normals.push_back(n);
            normals.push_back(n);

            indices.push_back(indexStart + 0);
            indices.push_back(indexStart + 1);
            indices.push_back(indexStart + 2);

            indices.push_back(indexStart + 2);
            indices.push_back(indexStart + 3);
            indices.push_back(indexStart + 0);
        }
    }

    const MeshShaderConst constants = s.meshConstants(meshColor, meshColor, Matrix44::Identity(), false);
    s.drawImmediate(vertices.data(), normals.data(), nullptr, indices.data(), uint32_t(vertices.size()),
                    uint32_t(indices.size()), constants);
}

void DemoContext::DrawPlanes(const dm::float4* planes, int n, float bias) {
    Impl& s = *m_Impl;
    s.bias = 0.0f;
    s.grid = 1;
    s.expand = 0.0f;

    for (int i = 0; i < n; ++i) {
        dm::float4 p = planes[i];
        p.w -= bias;
        DrawPlane(p, false);
    }

    s.grid = 0;
    s.bias = c_ShadowBias;
}

std::shared_ptr<GpuMesh> DemoContext::CreateGpuMesh(const utils::Mesh* m, nvrhi::ICommandList* commandList) {
    Impl& s = *m_Impl;
    if (!m || m->m_positions.empty() || m->m_indices.empty())
        return nullptr;

    auto mesh = std::make_shared<GpuMesh>();
    const uint32_t numVertices = m->GetNumVertices();
    auto vertexBuffer = [&](uint64_t size, const char* name) {
        nvrhi::BufferDesc d;
        d.byteSize = size;
        d.isVertexBuffer = true;
        d.debugName = name;
        d.initialState = nvrhi::ResourceStates::VertexBuffer;
        d.keepInitialState = true;
        nvrhi::BufferHandle b;
        s.device->createBuffer(d, &b);
        return b;
    };
    mesh->positions = vertexBuffer(sizeof(dm::float3) * numVertices, "GpuMesh positions");
    mesh->normals = vertexBuffer(sizeof(dm::float3) * numVertices, "GpuMesh normals");
    mesh->texcoords = vertexBuffer(sizeof(dm::float2) * numVertices, "GpuMesh texcoords");
    mesh->colors = vertexBuffer(sizeof(dm::float4) * numVertices, "GpuMesh colours");
    {
        nvrhi::BufferDesc d;
        d.byteSize = sizeof(uint32_t) * m->m_indices.size();
        d.isIndexBuffer = true;
        d.debugName = "GpuMesh indices";
        d.initialState = nvrhi::ResourceStates::IndexBuffer;
        d.keepInitialState = true;
        s.device->createBuffer(d, &mesh->indices);
    }
    mesh->numIndices = uint32_t(m->m_indices.size());

    std::vector<dm::float3> normals(m->m_normals);
    normals.resize(numVertices, dm::float3(0.0f, 1.0f, 0.0f));
    const std::vector<uint8_t> zeros(sizeof(dm::float4) * numVertices, 0);

    nvrhi::ICommandList* cl = commandList ? commandList : s.uploadList.Get();
    if (!commandList)
        cl->open();
    cl->writeBuffer(mesh->positions, m->m_positions.data(), sizeof(dm::float3) * numVertices);
    cl->writeBuffer(mesh->normals, normals.data(), sizeof(dm::float3) * numVertices);
    cl->writeBuffer(mesh->texcoords, zeros.data(), sizeof(dm::float2) * numVertices);
    cl->writeBuffer(mesh->colors, zeros.data(), sizeof(dm::float4) * numVertices);
    cl->writeBuffer(mesh->indices, m->m_indices.data(), sizeof(uint32_t) * m->m_indices.size());
    if (!commandList) {
        cl->close();
        s.device->executeCommandList(cl);
    }
    return mesh;
}

void DemoContext::DrawGpuMesh(GpuMesh* m, const Matrix44& xform, const dm::float3& color) {
    Impl& s = *m_Impl;
    if (!m)
        return;
    const MeshShaderConst c = s.meshConstants(dm::float4(color, 0.0f), dm::float4(color, 0.0f), xform, false);
    Impl::MeshStreams streams;
    streams.positions = m->positions;
    streams.normals = m->normals;
    streams.texcoords = m->texcoords;
    streams.colors = m->colors;
    streams.indices = m->indices;
    streams.numIndices = m->numIndices;
    s.drawMesh(streams, c);
}

void DemoContext::DrawPoints(const FluidRenderBuffers& buffers, int n, int offset, float radius, float screenWidth,
                             float screenAspect, float fov, const dm::float3& lightPos, const dm::float3& lightTarget,
                             const Matrix44& lightTransform, bool showDensity) {
    Impl& s = *m_Impl;
    if (n <= 0 || !buffers.positions || !buffers.densities || !buffers.indices)
        return;

    const bool shadowPass = s.stage == Pass::Shadow;

    PointShaderConst c{};
    c.modelView = s.view;
    c.projection = s.proj;
    c.pointRadius = radius;
    c.pointScale = screenWidth / screenAspect * (1.0f / tanf(fov * 0.5f));
    c.spotMin = c_SpotMin;
    c.spotMax = c_SpotMax;
    c.mode = showDensity ? 1 : 0;
    const dm::float4* colors = s.colors ? s.colors : s.defaultColors;
    for (int i = 0; i < 8; ++i)
        c.colors[i] = colors[i];
    c.lightTransform = ConvertToD3DProjection(lightTransform);
    c.lightPos = lightPos;
    c.lightDir = normalized(lightTarget - lightPos);
    std::memcpy(c.shadowTaps, c_ShadowTaps, sizeof(c_ShadowTaps));
    if (shadowPass)
        c.mode = 2;

    nvrhi::IGraphicsPipeline* p = s.pipeline(shadowPass ? Impl::PointShadow : Impl::PointLight);
    if (!p)
        return;
    s.commandList->writeBuffer(s.pointCB, &c, sizeof(c));

    nvrhi::IFramebuffer* fb = s.currentTarget();
    const nvrhi::FramebufferInfoEx& info = fb->getFramebufferInfo();
    nvrhi::GraphicsState state;
    state.pipeline = p;
    state.framebuffer = fb;
    state.bindings = {shadowPass ? s.pointShadowSet.Get() : s.pointLightSet.Get()};
    state.vertexBuffers = {nvrhi::VertexBufferBinding{buffers.positions, 0, 0},
                           nvrhi::VertexBufferBinding{buffers.densities, 1, 0},
                           nvrhi::VertexBufferBinding{buffers.densities, 2, 0}};
    state.indexBuffer = nvrhi::IndexBufferBinding{buffers.indices, nvrhi::Format::R32_UINT, 0};
    state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
    s.commandList->setGraphicsState(state);

    nvrhi::DrawArguments args;
    args.vertexCount = uint32_t(n);
    args.startIndexLocation = uint32_t(offset);
    s.commandList->drawIndexed(args);
}

void DemoContext::RenderEllipsoids(const FluidRenderBuffers& buffers, int n, int offset, float radius,
                                   float screenWidth, float screenAspect, float fov, const dm::float3& lightPos,
                                   const dm::float3& lightTarget, const Matrix44& lightTransform,
                                   const dm::float4& color, float blur, float ior, bool debug) {
    Impl& s = *m_Impl;
    if (n <= 0 || !buffers.positions || !buffers.indices || !buffers.anisotropy[0] || !buffers.anisotropy[1] ||
        !buffers.anisotropy[2])
        return;
    nvrhi::ICommandList* cl = s.commandList;

    // calcFluidConstantBuffer
    FluidShaderConst c{};
    c.modelViewProjection = s.proj * s.view;
    c.modelView = s.view;
    c.projection = s.proj;
    c.inverseModelView = Inverse(s.view);
    c.inverseProjection = Inverse(s.proj);

    const float viewHeight = tanf(fov / 2.0f);
    (void)viewHeight;
    c.invViewport = dm::float3(1.0f / screenWidth, screenAspect / screenWidth, 1.0f);

    // larger sprites for a smoother thickness texture
    const float thicknessScale = 4.0f;
    c.pointRadius = thicknessScale * radius;

    auto drawParticles = [&](Impl::PipelineKind kind, nvrhi::IFramebuffer* fb) {
        nvrhi::IGraphicsPipeline* p = s.pipeline(kind);
        if (!p)
            return;
        cl->writeBuffer(s.fluidCB, &c, sizeof(c));
        const nvrhi::FramebufferInfoEx& info = fb->getFramebufferInfo();
        nvrhi::GraphicsState state;
        state.pipeline = p;
        state.framebuffer = fb;
        state.bindings = {s.fluidSet};
        // the thickness pass reads the positions only (its input layout has one stream)
        state.vertexBuffers = {nvrhi::VertexBufferBinding{buffers.positions, 0, 0}};
        if (kind == Impl::Ellipsoid) {
            state.vertexBuffers.push_back(nvrhi::VertexBufferBinding{buffers.anisotropy[0], 1, 0});
            state.vertexBuffers.push_back(nvrhi::VertexBufferBinding{buffers.anisotropy[1], 2, 0});
            state.vertexBuffers.push_back(nvrhi::VertexBufferBinding{buffers.anisotropy[2], 3, 0});
        }
        state.indexBuffer = nvrhi::IndexBufferBinding{buffers.indices, nvrhi::Format::R32_UINT, 0};
        state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
        cl->setGraphicsState(state);
        nvrhi::DrawArguments args;
        args.vertexCount = uint32_t(n);
        args.startIndexLocation = uint32_t(offset);
        cl->drawIndexed(args);
    };

    auto drawQuad = [&](Impl::PipelineKind kind, nvrhi::IFramebuffer* fb, nvrhi::IBindingSet* set) {
        nvrhi::IGraphicsPipeline* p = s.pipeline(kind);
        if (!p)
            return;
        cl->writeBuffer(s.fluidCB, &c, sizeof(c));
        const nvrhi::FramebufferInfoEx& info = fb->getFramebufferInfo();
        nvrhi::GraphicsState state;
        state.pipeline = p;
        state.framebuffer = fb;
        state.bindings = {set};
        state.vertexBuffers = {nvrhi::VertexBufferBinding{s.quadVertices, 0, 0}};
        state.indexBuffer = nvrhi::IndexBufferBinding{s.quadIndices, nvrhi::Format::R32_UINT, 0};
        state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
        cl->setGraphicsState(state);
        nvrhi::DrawArguments args;
        args.vertexCount = 4;
        cl->drawIndexed(args);
    };

    // thickness
    cl->clearTextureFloat(s.thicknessColor, nvrhi::AllSubresources, nvrhi::Color(0.f));
    cl->clearDepthStencilTexture(s.thicknessDepth, nvrhi::AllSubresources, true, 1.0f, false, 0);
    drawParticles(Impl::Thickness, s.thicknessFramebuffer);

    // ellipsoid depth
    cl->clearTextureFloat(s.depthColor, nvrhi::AllSubresources, nvrhi::Color(0.f));
    cl->clearDepthStencilTexture(s.depthDepth, nvrhi::AllSubresources, true, 1.0f, false, 0);
    drawParticles(Impl::Ellipsoid, s.depthFramebuffer);

    // smoothed depth
    cl->clearTextureFloat(s.smoothColor, nvrhi::AllSubresources, nvrhi::Color(0.f));
    c.blurRadiusWorld = radius * 0.5f;
    c.blurScale = screenWidth / screenAspect * (1.0f / tanf(fov * 0.5f));
    c.invTexScale = dm::float4(1.0f / screenAspect, 1.0f, 0.0f, 0.0f);
    c.blurFalloff = blur;
    c.debug = debug ? 1 : 0;
    drawQuad(Impl::Blur, s.smoothFramebuffer, s.blurSet);

    // composite (calcFluidCompositeConstantBuffer), over the scene: the refraction samples the scene as it is
    // now, resolved into the frame texture
    c.invTexScale = dm::float4(1.0f / screenWidth, screenAspect / screenWidth, 0.0f, 0.0f);
    c.clipPosToEye = dm::float4(tanf(fov * 0.5f) * screenAspect, tanf(fov * 0.5f), 0.0f, 0.0f);
    c.color = color;
    c.ior = ior;
    c.spotMin = c_SpotMin;
    c.spotMax = c_SpotMax;
    c.debug = debug ? 1 : 0;
    c.lightPos = lightPos;
    c.lightDir = -normalized(lightTarget - lightPos);
    c.lightTransform = ConvertToD3DProjection(lightTransform);
    for (int i = 0; i < 12; ++i)
        c.shadowTaps[i] = dm::float4(c_ShadowTaps[i].x, c_ShadowTaps[i].y, 0.0f, 0.0f);

    if (s.msaaSamples > 1)
        cl->resolveTexture(s.resolved, nvrhi::AllSubresources, s.sceneColor, nvrhi::AllSubresources);
    else
        cl->copyTexture1(s.resolved, nvrhi::TextureSlice(), s.sceneColor, nvrhi::TextureSlice());

    drawQuad(Impl::Composite, s.sceneFramebuffer, s.compositeSet);
}

void DemoContext::DrawDiffuse(const DiffuseRenderBuffers& buffers, float radius, float screenWidth, float screenAspect,
                              float fov, const dm::float4& /*color*/, const dm::float3& lightPos,
                              const dm::float3& lightTarget, const Matrix44& lightTransform, float motionBlur,
                              float /*inscatter*/, float /*outscatter*/, bool /*shadowEnabled*/, bool /*front*/) {
    // DemoContextD3D11::drawDiffuse: the colour, the scattering and the shadow arguments are not used by the D3D
    // renderers, and the "front" pass is the same draw as the "back" one.
    Impl& s = *m_Impl;
    if (!buffers.maxParticles || !buffers.positions || !buffers.velocities || !buffers.drawArgs)
        return;

    DiffuseShaderConst c{};
    c.modelViewProjection = s.proj * s.view;
    c.modelView = s.view;
    c.projection = s.proj;
    c.motionBlurScale = motionBlur;
    c.diffuseRadius = screenWidth / screenAspect * (1.0f / tanf(fov * 0.5f));
    c.diffuseScale = radius;
    c.spotMin = c_SpotMin;
    c.spotMax = c_SpotMax;
    c.lightTransform = ConvertToD3DProjection(lightTransform);
    c.lightPos = lightPos;
    c.lightDir = normalized(lightTarget - lightPos);
    c.color = dm::float4(1.0f, 1.0f, 1.0f, 1.0f);
    std::memcpy(c.shadowTaps, c_ShadowTaps, sizeof(c_ShadowTaps));
    // `diffusion` (the lifetime growth of the sprites) is never set by FleX's D3D renderers: it held whatever the
    // stack did. Zero here.
    c.diffusion = 0.0f;

    nvrhi::IGraphicsPipeline* p = s.pipeline(Impl::Diffuse);
    if (!p)
        return;
    s.commandList->writeBuffer(s.diffuseCB, &c, sizeof(c));

    const nvrhi::FramebufferInfoEx& info = s.sceneFramebuffer->getFramebufferInfo();
    nvrhi::GraphicsState state;
    state.pipeline = p;
    state.framebuffer = s.sceneFramebuffer;
    state.bindings = {s.diffuseSet};
    state.vertexBuffers = {nvrhi::VertexBufferBinding{buffers.positions, 0, 0},
                           nvrhi::VertexBufferBinding{buffers.velocities, 1, 0}};
    state.indirectParams = buffers.drawArgs;
    state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
    s.commandList->setGraphicsState(state);
    // the count is the solver's, on the GPU: an indirect draw where FleX read the count back
    s.commandList->drawIndirect(0);
}

void DemoContext::BeginLines() {}

void DemoContext::DrawLine(const dm::float3& p, const dm::float3& q, const dm::float4& color) {
    m_Impl->lines.push_back(LineVertex{p, color});
    m_Impl->lines.push_back(LineVertex{q, color});
}

void DemoContext::EndLines() {
    Impl& s = *m_Impl;
    if (s.lines.empty() || !s.commandList)
        return;
    // lines a scene draws in the shadow pass: FleX's D3D11 renderer drew them with the shadow map bound and no
    // colour target, so they left nothing; they are dropped here (they would land in the scene's colour target)
    if (s.stage == Pass::Shadow) {
        s.lines.clear();
        return;
    }

    nvrhi::IGraphicsPipeline* p = s.pipeline(Impl::Lines);
    if (!p) {
        s.lines.clear();
        return;
    }
    const Matrix44 projectionViewWorld = s.proj * s.view;
    s.commandList->writeBuffer(s.lineCB, &projectionViewWorld, sizeof(projectionViewWorld));
    s.reserve(Impl::align16(sizeof(LineVertex) * s.lines.size()));
    const uint64_t offset = s.write(s.lines.data(), sizeof(LineVertex) * s.lines.size());

    const nvrhi::FramebufferInfoEx& info = s.sceneFramebuffer->getFramebufferInfo();
    nvrhi::GraphicsState state;
    state.pipeline = p;
    state.framebuffer = s.sceneFramebuffer;
    state.bindings = {s.lineSet};
    state.vertexBuffers = {nvrhi::VertexBufferBinding{s.stream, 0, offset}};
    state.viewport.addViewportAndScissorRect(nvrhi::Viewport(float(info.width), float(info.height)));
    s.commandList->setGraphicsState(state);
    nvrhi::DrawArguments args;
    args.vertexCount = uint32_t(s.lines.size());
    s.commandList->draw(args);
    s.lines.clear();
}

}  // namespace ephys::framework
