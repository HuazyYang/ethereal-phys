#ifndef ETHEREAL_PHYS_FRAMEWORK_DEMO_CONTEXT_H
#define ETHEREAL_PHYS_FRAMEWORK_DEMO_CONTEXT_H

// DemoContext: the renderer of the samples, over nvrhi.
//
// Part of ethereal_phys_framework (samples/framework). It is the FleX demo's seam, reference/flex/demo/demoContext.h,
// kept as the interface the demo's frame (RenderScene, RenderDebug, DrawShapes) and its scenes draw through;
// everything below it in FleX (demo/d3d, demo/d3d11, demo/d3d12: ~85 files of D3D11 and D3D12 code) is replaced by
// this one class over nvrhi, so it runs on D3D12, Vulkan and D3D11. The shaders are FleX's (samples/framework/
// shaders, ported to shader model 6.0); the passes, their render states and their constants are those of the
// demo's D3D11 renderer (demoContextD3D11.cpp and the renderers it calls).
//
// What changed in the interface:
// - Particle data is GPU buffers, never host arrays: the particle renderers draw the engine's arrays (zero-copy
//   render arrays, or their D3D11 copy fallback) and the solver outputs copied by IPhysReadback, through
//   FluidRenderBuffers / DiffuseRenderBuffers / ClothRenderBuffers. FleX's updateFluidRenderBuffers and
//   updateDiffuseRenderBuffers (host uploads, or NvFlexGet* into registered buffers) are the framework's
//   PhysicsScene now. The diffuse count stays on the GPU: the diffuse pass is an indirect draw.
// - Recording: StartFrame takes the command list every draw of the frame is recorded on; EndFrame resolves the
//   multisampled scene and copies it to the swap chain. readFrame is GetFrameTexture.
// - One shadow map and one fluid renderer, owned by the context (FleX created them through it and passed them
//   back in).
// - The colour table the point and cloth renderers read (FleX's global g_colors) is SetColors.
// - Left out: the GPU timers of the renderer, the async compute benchmark mode, and the ImGui graph (the UI is
//   Donut's ImGui pass).

#include <ethereal/phys_framework/flex_math.h>
#include <ethereal/phys_utils/mesh.h>

#include <donut/core/math/math.h>
#include <nvrhi/nvrhi.h>

#include <cstdint>
#include <memory>

namespace donut::engine {
class ShaderFactory;
class CommonRenderPasses;
}  // namespace donut::engine

namespace ephys::framework {

// The buffers the point and fluid renderers draw (FleX's FluidRenderBuffers). Vertex buffers of maxParticles
// elements, drawn through the index buffer of active particles.
struct FluidRenderBuffers {
    nvrhi::IBuffer* positions = nullptr;      // float4: positions (w the inverse mass), or smoothed positions
    nvrhi::IBuffer* densities = nullptr;      // 4 bytes: the density (float) or the phase (int) of a particle
    nvrhi::IBuffer* anisotropy[3] = {};       // float4: the ellipsoid axes (xyz direction, w scale)
    nvrhi::IBuffer* indices = nullptr;        // uint32: the active particles (an index buffer)
};

// The diffuse particles (FleX's DiffuseRenderBuffers). drawArgs: the indirect draw arguments
// {count, 1, 0, 0}, the count written on the GPU.
struct DiffuseRenderBuffers {
    nvrhi::IBuffer* positions = nullptr;   // float4, w the lifetime
    nvrhi::IBuffer* velocities = nullptr;  // float4
    nvrhi::IBuffer* drawArgs = nullptr;
    uint32_t maxParticles = 0;
};

// A cloth drawn from the particle arrays: positions and normals are float4 per particle (the render arrays of
// the engine), indices three per triangle. numTriangles triangles from firstTriangle are drawn (FleX's DrawCloth
// took a pointer into the index array and a count: the inflatables draw one range each).
struct ClothRenderBuffers {
    nvrhi::IBuffer* positions = nullptr;
    nvrhi::IBuffer* normals = nullptr;
    nvrhi::IBuffer* indices = nullptr;
    uint32_t firstTriangle = 0;
    uint32_t numTriangles = 0;
    uint32_t numPositions = 0;
};

class GpuMesh;

class DemoContext {
 public:
    DemoContext(nvrhi::IDevice* device, donut::engine::ShaderFactory* shaderFactory,
                donut::engine::CommonRenderPasses* commonPasses, uint32_t msaaSamples);
    ~DemoContext();

    DemoContext(const DemoContext&) = delete;
    DemoContext& operator=(const DemoContext&) = delete;

    // Loads the shaders and creates what does not depend on the window size. False when a shader is missing.
    bool Init();
    // (Re)creates the scene and fluid targets for a window of width x height.
    void Resize(uint32_t width, uint32_t height);
    [[nodiscard]] uint32_t GetWidth() const;
    [[nodiscard]] uint32_t GetHeight() const;

    // ---- the frame ----
    void StartFrame(nvrhi::ICommandList* commandList, const dm::float4& clearColor);
    // Resolves the scene into GetFrameTexture() and copies that to the framebuffer (the swap chain's).
    void EndFrame(nvrhi::IFramebuffer* presentFramebuffer);
    // The resolved frame of the last EndFrame: RGBA8_UNORM, the window's size (FleX: readFrame).
    [[nodiscard]] nvrhi::ITexture* GetFrameTexture() const;

    void SetColors(const dm::float4* colors);  // 8 colours, read by DrawPoints and DrawCloth

    // ---- view and state ----
    void SetView(const Matrix44& view, const Matrix44& projection);  // projection: OpenGL style
    // The world-space ray through window pixel (x, y), y down.
    void GetViewRay(int x, int y, dm::float3& origin, dm::float3& dir) const;
    void SetFillMode(bool wire);
    void SetCullMode(bool enabled);

    // ---- solid geometry ----
    void BindSolidShader(const dm::float3& lightPos, const dm::float3& lightTarget, const Matrix44& lightTransform,
                         float bias, const dm::float4& fogColor);
    void UnbindSolidShader();

    void ShadowBegin();
    void ShadowEnd();

    void DrawMesh(const utils::Mesh* m, const dm::float3& color);
    void DrawCloth(const ClothRenderBuffers& cloth, int colorIndex, float expand, bool twosided = true);
    void DrawRope(const dm::float4* positions, const int* indices, int numIndices, float radius, int color);
    void DrawPlane(const dm::float4& p, bool color);
    void DrawPlanes(const dm::float4* planes, int n, float bias);

    // The upload is recorded on commandList, or on a list of the context executed at once when it is null (outside
    // a frame; on D3D11 no other command list may be open then).
    std::shared_ptr<GpuMesh> CreateGpuMesh(const utils::Mesh* m, nvrhi::ICommandList* commandList = nullptr);
    void DrawGpuMesh(GpuMesh* m, const Matrix44& xform, const dm::float3& color);

    // ---- particles ----
    // n active particles from offset (indices into FluidRenderBuffers::indices).
    void DrawPoints(const FluidRenderBuffers& buffers, int n, int offset, float radius, float screenWidth,
                    float screenAspect, float fov, const dm::float3& lightPos, const dm::float3& lightTarget,
                    const Matrix44& lightTransform, bool showDensity);
    // The screen-space fluid surface: thickness, ellipsoid depth, depth blur, composite.
    void RenderEllipsoids(const FluidRenderBuffers& buffers, int n, int offset, float radius, float screenWidth,
                          float screenAspect, float fov, const dm::float3& lightPos, const dm::float3& lightTarget,
                          const Matrix44& lightTransform, const dm::float4& color, float blur, float ior, bool debug);
    void DrawDiffuse(const DiffuseRenderBuffers& buffers, float radius, float screenWidth, float screenAspect,
                     float fov, const dm::float4& color, const dm::float3& lightPos, const dm::float3& lightTarget,
                     const Matrix44& lightTransform, float motionBlur, float inscatter, float outscatter,
                     bool shadowEnabled, bool front);

    // ---- debug lines ----
    void BeginLines();
    void DrawLine(const dm::float3& p, const dm::float3& q, const dm::float4& color);
    void EndLines();

 private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_DEMO_CONTEXT_H */
