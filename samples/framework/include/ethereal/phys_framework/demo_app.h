#ifndef ETHEREAL_PHYS_FRAMEWORK_DEMO_APP_H
#define ETHEREAL_PHYS_FRAMEWORK_DEMO_APP_H

// The application shell of the samples: the FleX demo's main.cpp (app loop, scene registry, UI, camera, picking,
// screenshot) on Donut and the engine.
//
// Part of ethereal_phys_framework (samples/framework). A sample builds its registry and calls RunDemoApp:
//
//     int main(int argc, char** argv) {
//         ephys::framework::DemoAppDesc desc;
//         desc.title = "...";
//         desc.assetDir = ETHEREAL_PHYS_ASSETS_DIR;
//         desc.scenes.push_back(std::make_unique<MyScene>("My Scene"));
//         return ephys::framework::RunDemoApp(argc, argv, std::move(desc));
//     }
//
// Command line (all optional):
//
//   --d3d12 | --vk | --d3d11     the graphics API (Donut's switches; D3D12 by default)
//   --debug                      the debug runtime and the validation layers of the API, and nvrhi's validation
//   --adapter <i>                the adapter (DXGI) or physical device (Vulkan) to run on; on Vulkan 0 by default,
//                                because Donut's automatic choice can mix up the queue families of two GPUs
//   --scene <name>               the scene to start with (a registry name, case-insensitive), or its index
//   --screenshot <file.png>      unattended: render, save the frame after --frames simulation steps (without
//                                the UI) and exit; the exit code is 1 when a validation error or an engine error
//                                was reported, or the image is a single colour
//   --frames <n>                 with --screenshot: the simulation step count (default 60)
//   --smoke <n>                  unattended: every scene of the registry in turn, n steps each, with the same
//                                checks; --smoke-dir <dir> saves one PNG per scene there
//   --msaa <n>                   samples of the scene target (default 8, as FleX)
//   --width <w>, --height <h>    the window (default 1280 x 720)
//   --draw-fluid                 start with the fluid surface on and the points off (FleX's 'e' and 'v' keys)
//   --emit                       the emitters on from the start (FleX's space key)
//   --baseline-reductions        the solvers sum over thread groups in FleX's order, not with the wave intrinsics
//                                (SolverDesc::baselineReductions)
//   --solver pbd|xpbd            create every scene's solver with this formulation (FleX's --xpbd override); the
//                                panel's Solver choice does the same for the current scene and restarts it
//   --pick <x>,<y>,<n>[,<dy>]    at simulation step n, pick the particle under window pixel (x, y) as FleX's shift +
//                                left click does, hold it, and drag it dy pixels up per step (a test of the picking
//                                without a mouse)
//
// The simulation always advances by DemoState::dt (1/60 s) per frame in DemoState::numSubsteps substeps: the
// wall clock only drives the camera and the frame time display, so an unattended run is deterministic up to the
// GPU's own order of floating point operations.

#include <ethereal/phys_framework/scene.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace ephys::framework {

struct DemoAppDesc {
    std::string title = "ethereal-phys demo";
    std::filesystem::path assetDir;  // FleX's ../../data
    std::vector<std::unique_ptr<Scene>> scenes;
};

// Runs the application; returns the process exit code.
int RunDemoApp(int argc, const char* const* argv, DemoAppDesc desc);

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_DEMO_APP_H */
