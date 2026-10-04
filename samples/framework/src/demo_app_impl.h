#ifndef ETHEREAL_PHYS_FRAMEWORK_SRC_DEMO_APP_IMPL_H
#define ETHEREAL_PHYS_FRAMEWORK_SRC_DEMO_APP_IMPL_H

// DemoApp: the FleX demo's main.cpp decomposed. This class keeps the frame (UpdateFrame, RenderScene,
// RenderDebug, DrawShapes), the scene lifecycle (Init / Reset), the camera, the input and the picking; the
// renderer is DemoContext (demo_context.h), the physics PhysicsScene (physics_scene.h), the UI demo_ui.cpp, the
// validation counters ValidationMonitor (validation.h).

#include "physics_scene.h"
#include "validation.h"

#include <ethereal/phys_framework/demo_app.h>
#include <ethereal/phys_framework/demo_context.h>

#include <donut/app/DeviceManager.h>
#include <donut/core/vfs/VFS.h>
#include <donut/engine/CommonRenderPasses.h>
#include <donut/engine/ShaderFactory.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace ephys::framework {

struct DemoOptions {
    nvrhi::GraphicsAPI api = nvrhi::GraphicsAPI::D3D12;
    bool debug = false;
    int adapter = -1;
    std::string scene;  // name or index
    std::filesystem::path screenshot;
    int frames = 60;
    int smokeFrames = -1;  // --smoke: frames per scene
    std::filesystem::path smokeDir;
    uint32_t msaa = 8;
    uint32_t width = 1280;
    uint32_t height = 720;
    bool drawFluid = false;
    // --pick x,y,frame[,dy]: at simulation step `frame`, a shift + left click at window pixel (x, y), held to the
    // end of the run and dragged dy pixels up per step (FleX's particle picking, without a mouse)
    int pickX = -1, pickY = -1, pickFrame = -1, pickDrag = 0;
};

class DemoApp : public donut::app::IRenderPass {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    DemoApp(donut::app::DeviceManager* deviceManager, const DemoOptions& options, DemoAppDesc desc,
            ValidationMonitor* monitor);
    ~DemoApp() override;

    bool Init();
    [[nodiscard]] donut::engine::ShaderFactory* GetShaderFactory() const { return m_ShaderFactory; }
    [[nodiscard]] int GetExitCode() const { return m_ExitCode; }

    // ---- IRenderPass ----
    void Render(nvrhi::IFramebuffer* framebuffer) override;
    void Animate(float elapsedTimeSeconds) override;
    void BackBufferResized(uint32_t width, uint32_t height, uint32_t sampleCount) override;
    bool KeyboardUpdate(int key, int scancode, int action, int mods) override;
    bool MousePosUpdate(double xpos, double ypos) override;
    bool MouseButtonUpdate(int button, int action, int mods) override;
    bool ShouldRenderUnfocused() override { return true; }

    // the ImGui panel (demo_ui.cpp): FleX's DoUI; returns the scene to switch to, or -1
    int DoUI();

 private:
    // ---- the scene lifecycle (FleX: Init, Reset) ----
    bool InitScene(int scene, bool centerCamera = true);
    void ResetState();

    // ---- the frame (FleX: UpdateFrame and what it calls) ----
    void UpdateCamera();
    void UpdateEmitters();
    void UpdateMouse();
    void UpdateWind();
    void MapBuffers();
    void RenderScene();
    void RenderDebug();
    void DrawShapes();

    void Capture(const std::filesystem::path& path);
    void RequestExit(int code);
    void InputKey(int key);

    donut::app::DeviceManager* m_DeviceManager;
    DemoOptions m_Options;
    DemoAppDesc m_Desc;
    ValidationMonitor* m_Monitor;

    nvrhi::AutoPtr<donut::vfs::RootFileSystem> m_FileSystem;
    nvrhi::AutoPtr<donut::engine::ShaderFactory> m_ShaderFactory;
    nvrhi::AutoPtr<donut::engine::CommonRenderPasses> m_CommonPasses;
    std::unique_ptr<DemoContext> m_Context;
    PhysDeviceHandle m_Phys;
    std::unique_ptr<PhysicsScene> m_Physics;
    nvrhi::CommandListHandle m_CommandList;
    nvrhi::StagingTextureHandle m_Staging;

    DemoState g;
    int m_Scene = 0;
    int m_SelectedScene = 0;
    bool m_ResetScene = false;
    int m_NewScene = -1;
    uint32_t m_FrameIndex = 0;
    bool m_Exiting = false;
    int m_ExitCode = 0;

    // unattended runs
    bool m_CaptureThisFrame = false;
    int m_CaptureFrame = 0;  // the steps simulated before the captured frame
    std::filesystem::path m_CapturePath;
    int m_SmokeScene = -1;
    int m_SmokeFailures = 0;

    // UI state (FleX's globals of main.cpp)
    bool m_ShowHelp = true;
    bool m_TweakPanel = true;
    bool m_Profile = false;
    float m_RealDt = 0.0f;
    float m_UpdateTime = 0.0f;
    float m_SimLatency = 0.0f;
    std::string m_DeviceName;

    // the mouse (FleX: g_lastx, g_lasty, g_lastb, g_mouse*)
    int m_LastX = 0;
    int m_LastY = 0;
    int m_LastButton = -1;
    int m_Mods = 0;
    bool m_MousePicked = false;
    int m_MouseParticle = -1;
    float m_MouseT = 0.0f;
    dm::float3 m_MousePos = dm::float3(0.f);
    float m_MouseMass = 0.0f;

};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_SRC_DEMO_APP_IMPL_H */
