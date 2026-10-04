// flex_demo: the FleX demo (reference/flex/demo) on ethereal-phys, Donut and nvrhi.
//
// The scene registry of the demo's main(), in its order, with the scenes ported so far (step 10a of the
// migration: one or more per feature family the framework draws). The framework (samples/framework) is the
// demo's app shell and renderer; the scenes (scenes/) are the demo's scene classes with g_x -> g.x.
//
//   flex_demo [--d3d12 | --vk | --d3d11] [--debug] [--adapter i] [--scene name]
//             [--screenshot file.png [--frames n]] [--smoke n [--smoke-dir dir]] [--draw-fluid] [--msaa n]
//
// Keys as in FleX: W A S D Q Z move, right drag looks, shift + left drag picks a particle, R resets, P pauses,
// O steps, E fluid surface, V points, M meshes, I diffuse, F springs, '.' GPU timers, H the UI, 1-9 and the
// arrows choose a scene.

#include "scenes/dam_break.h"
#include "scenes/flag_cloth.h"
#include "scenes/fluid_block.h"
#include "scenes/granular_pile.h"
#include "scenes/rigid_pile.h"

#include <ethereal/phys_framework/demo_app.h>

#include <memory>

int main(int argc, char** argv) {
    ephys::framework::DemoAppDesc desc;
    desc.title = "ethereal-phys: FleX demo";
    desc.assetDir = ETHEREAL_PHYS_ASSETS_DIR;

    auto& scenes = desc.scenes;

    // cloth scenes
    scenes.push_back(std::make_unique<FlagCloth>("Flag Cloth"));

    // regular fluids
    scenes.push_back(std::make_unique<DamBreak>("DamBreak  5cm", 0.05f));
    scenes.push_back(std::make_unique<DamBreak>("DamBreak 10cm", 0.1f));
    scenes.push_back(std::make_unique<DamBreak>("DamBreak 15cm", 0.15f));

    // rigid body scenes
    scenes.push_back(std::make_unique<RigidPile>("Rigid2", 2));
    scenes.push_back(std::make_unique<RigidPile>("Rigid4", 4));
    scenes.push_back(std::make_unique<RigidPile>("Rigid8", 12));

    // granular scenes
    scenes.push_back(std::make_unique<GranularPile>("Granular Pile"));

    // coupling scenes
    scenes.push_back(std::make_unique<FluidBlock>("Fluid Block"));

    return ephys::framework::RunDemoApp(argc, argv, std::move(desc));
}
