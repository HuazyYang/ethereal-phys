// flex_demo: the FleX demo (reference/flex/demo) on ethereal-phys, Donut and nvrhi.
//
// The scene registry of the demo's main(), in its order, with the scenes ported so far: one or more per feature
// family (steps 10a and 10b of the migration). The framework (samples/framework) is the demo's app shell and
// renderer; the scenes (scenes/) are the demo's scene classes with g_x -> g.x.
//
//   flex_demo [--d3d12 | --vk | --d3d11] [--debug] [--adapter i] [--scene name]
//             [--screenshot file.png [--frames n]] [--smoke n [--smoke-dir dir]] [--draw-fluid] [--emit]
//             [--solver pbd|xpbd] [--msaa n]
//
// Keys as in FleX: W A S D Q Z move, right drag looks, shift + left drag picks a particle, R resets, P pauses,
// O steps, E fluid surface, V points, M meshes, I diffuse, F springs, space emits, '.' GPU timers, H the UI, 1-9
// and the arrows choose a scene.

#include "scenes/collision.h"
#include "scenes/dam_break.h"
#include "scenes/flag_cloth.h"
#include "scenes/fluid_block.h"
#include "scenes/fluids.h"
#include "scenes/force_field.h"
#include "scenes/granular_pile.h"
#include "scenes/inflatable.h"
#include "scenes/local_space.h"
#include "scenes/rigid_pile.h"
#include "scenes/soft_body.h"
#include "scenes/tearing.h"
#include "scenes/xpbd.h"

#include <ethereal/phys_framework/demo_app.h>

#include <memory>

int main(int argc, char** argv) {
    ephys::framework::DemoAppDesc desc;
    desc.title = "ethereal-phys: FleX demo";
    desc.assetDir = ETHEREAL_PHYS_ASSETS_DIR;

    auto& scenes = desc.scenes;

    // soft body scenes (FleX's main(): instances of SoftBody with these settings)
    {
        SoftBody::Instance teapot("teapot.ply");
        teapot.mScale = float3(25.0f);
        teapot.mClusterSpacing = 3.0f;
        teapot.mClusterRadius = 0.0f;
        teapot.mClusterStiffness = 0.1f;
        auto softTeapotSceneNew = std::make_unique<SoftBody>("Soft Teapot");
        softTeapotSceneNew->AddInstance(teapot);

        SoftBody::Instance softbunny("bunny.ply");
        softbunny.mScale = float3(20.0f);
        softbunny.mClusterSpacing = 3.5f;
        softbunny.mClusterRadius = 0.0f;
        softbunny.mClusterStiffness = 0.2f;
        auto softBunnySceneNew = std::make_unique<SoftBody>("Soft Bunny");
        softBunnySceneNew->AddInstance(softbunny);

        scenes.push_back(std::move(softTeapotSceneNew));
        scenes.push_back(std::move(softBunnySceneNew));
    }

    // plastic scenes
    {
        SoftBody::Instance bunny1("bunny.ply");
        bunny1.mScale = float3(10.0f);
        bunny1.mClusterSpacing = 1.0f;
        bunny1.mClusterRadius = 0.0f;
        bunny1.mClusterStiffness = 0.0f;
        bunny1.mGlobalStiffness = 1.0f;
        bunny1.mClusterPlasticThreshold = 0.0015f;
        bunny1.mClusterPlasticCreep = 0.15f;
        bunny1.mTranslation[1] = 5.0f;
        SoftBody::Instance bunny2("bunny.ply");
        bunny2.mScale = float3(10.0f);
        bunny2.mClusterSpacing = 1.0f;
        bunny2.mClusterRadius = 0.0f;
        bunny2.mClusterStiffness = 0.0f;
        bunny2.mGlobalStiffness = 1.0f;
        bunny2.mClusterPlasticThreshold = 0.0015f;
        bunny2.mClusterPlasticCreep = 0.30f;
        bunny2.mTranslation[1] = 5.0f;
        bunny2.mTranslation[0] = 2.0f;
        auto plasticComparisonScene = std::make_unique<SoftBody>("Plastic Comparison");
        plasticComparisonScene->AddInstance(bunny1);
        plasticComparisonScene->AddInstance(bunny2);
        plasticComparisonScene->mPlinth = true;

        scenes.push_back(std::move(plasticComparisonScene));
    }

    // collision scenes (SDF Collision is in FleX's scenes.h but not in its registry)
    scenes.push_back(std::make_unique<SDFCollision>("SDF Collision"));
    scenes.push_back(std::make_unique<TriangleCollision>("Triangle Collision"));
    scenes.push_back(std::make_unique<LocalSpaceFluid>("Local Space Fluid"));
    scenes.push_back(std::make_unique<LocalSpaceCloth>("Local Space Cloth"));

    // cloth scenes
    scenes.push_back(std::make_unique<FlagCloth>("Flag Cloth"));
    scenes.push_back(std::make_unique<XPBDHangingCloth>("XPBD Hanging Cloth"));
    scenes.push_back(std::make_unique<XPBDInflatableBalloon>("XPBD Inflatable Balloon"));
    scenes.push_back(std::make_unique<XPBDCantileverBeam>("XPBD Cantilever Beam"));
    scenes.push_back(std::make_unique<XPBDChain>("XPBD Chain"));
    scenes.push_back(std::make_unique<InflatableScene>("Inflatables"));
    scenes.push_back(std::make_unique<Tearing>("Tearing"));
    scenes.push_back(std::make_unique<Pasta>("Pasta"));

    // viscous fluids
    scenes.push_back(std::make_unique<Viscosity>("Viscosity Med", 3.0f));
    scenes.push_back(std::make_unique<Adhesion>("Adhesion"));

    // regular fluids
    scenes.push_back(std::make_unique<Buoyancy>("Buoyancy"));
    scenes.push_back(std::make_unique<SurfaceTension>("Surface Tension Med", 10.0f));
    scenes.push_back(std::make_unique<DamBreak>("DamBreak  5cm", 0.05f));
    scenes.push_back(std::make_unique<DamBreak>("DamBreak 10cm", 0.1f));
    scenes.push_back(std::make_unique<DamBreak>("DamBreak 15cm", 0.15f));

    // misc feature scenes
    scenes.push_back(std::make_unique<ForceFieldScene>("Force Field"));

    // rigid body scenes
    scenes.push_back(std::make_unique<RigidPile>("Rigid2", 2));
    scenes.push_back(std::make_unique<RigidPile>("Rigid4", 4));
    scenes.push_back(std::make_unique<RigidPile>("Rigid8", 12));

    // granular scenes
    scenes.push_back(std::make_unique<GranularPile>("Granular Pile"));

    // coupling scenes
    scenes.push_back(std::make_unique<RigidFluidCoupling>("Rigid Fluid Coupling"));
    scenes.push_back(std::make_unique<FluidBlock>("Fluid Block"));

    return ephys::framework::RunDemoApp(argc, argv, std::move(desc));
}
