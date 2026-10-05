#ifndef FLEX_DEMO_SCENES_FLUIDS_H
#define FLEX_DEMO_SCENES_FLUIDS_H

// Fluid scenes beyond the dam breaks:
//   Viscosity           FleX/demo/scenes/viscosity.h: a viscous fluid poured on a distance field bunny by
//                       an emitter of its own (on from the start)
//   Adhesion            FleX/demo/scenes/adhesion.h: an adhesive fluid shot by the camera's emitter at a
//                       wall (space, or --emit, starts it; without it the scene is the two boxes)
//   Buoyancy            FleX/demo/scenes/bouyancy.h: three rigid spheres of different masses in a pool of
//                       fluid (FleX's file name is misspelt)
//   SurfaceTension      FleX/demo/scenes/surfacetension.h: a column of fluid without gravity that pulls
//                       itself into drops, gravity on at frame 300
//   RigidFluidCoupling  FleX/demo/scenes/rigidfluidcoupling.h: rigid boxes on convex rocks under the
//                       camera's emitter (space, or --emit)

#include "scene_common.h"

class Viscosity : public Scene {
 public:
    Viscosity(const char* name, float viscosity = 1.0f, float dissipation = 0.0f)
        : Scene(name), viscosity(viscosity), dissipation(dissipation) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.1f;
        const float restDistance = radius * 0.5f;

        g.featureMode = FeatureMode::SimpleFluids;

        g.params.radius = radius;

        g.params.numIterations = 3;
        g.params.vorticityConfinement = 0.0f;
        g.params.fluidRestDistance = restDistance;
        g.params.smoothing = 0.35f;
        g.params.relaxationFactor = 1.f;
        g.params.restitution = 0.0f;
        g.params.collisionDistance = 0.00125f;
        g.params.shapeCollisionMargin = g.params.collisionDistance * 0.25f;
        g.params.dissipation = dissipation;

        g.params.gravity[1] *= 2.0f;

        g.fluidColor = float4(1.0f, 1.0f, 1.0f, 0.0f);
        g.meshColor = float3(0.7f, 0.8f, 0.9f) * 0.7f;

        g.params.dynamicFriction = 1.0f;
        g.params.staticFriction = 0.0f;
        g.params.viscosity = 20.0f + 20.0f * viscosity;
        g.params.adhesion = 0.1f * viscosity;
        g.params.cohesion = 0.05f * viscosity;
        g.params.surfaceTension = 0.0f;

        const float shapeSize = 2.0f;
        const float3 shapeLower = float3(-shapeSize * 0.5f, 0.0f, -shapeSize * 0.5f);
        const float3 shapeUpper = shapeLower + float3(shapeSize);
        const float3 shapeCenter = (shapeLower + shapeUpper) * 0.5f;

        const DistanceFieldId sdf = CreateSDF(g.AssetPath("bunny.ply"), 128);
        AddSDF(b, sdf, shapeLower, QuatIdentity(), shapeSize);

        const float emitterSize = 1.f;

        ephys::framework::Emitter e;
        e.mEnabled = true;
        e.mWidth = int(emitterSize / restDistance);
        e.mPos = float3(shapeCenter.x - 0.2f, shapeUpper.y + 0.75f, shapeCenter.z);
        e.mDir = float3(0.0f, -1.0f, 0.0f);
        e.mRight = float3(1.0f, 0.0f, 0.0f);
        e.mSpeed = (restDistance * 2.f / g.dt);

        g.sceneUpper.z = 5.0f;

        g.emitters.push_back(e);

        g.numExtraParticles = 64 * 1024;

        g.lightDistance *= 2.5f;

        // draw options
        g.drawEllipsoids = true;

        g.emit = true;
        g.pause = false;
    }

    void DoGui(DemoState& g) override { ImGui::SliderFloat("Emitter Pos", &g.emitters.back().mPos.x, -1.0f, 1.0f); }

    float viscosity;
    float dissipation;
};

class Adhesion : public Scene {
 public:
    explicit Adhesion(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.1f;

        g.params.radius = radius;

        g.params.numIterations = 3;
        g.params.vorticityConfinement = 0.0f;
        g.params.fluidRestDistance = g.params.radius * 0.55f;
        g.params.smoothing = 0.5f;
        g.params.relaxationFactor = 1.f;
        g.params.restitution = 0.0f;
        g.params.collisionDistance = 0.01f;

        g.fluidColor = float4(0.0f, 0.8f, 0.2f, 1.0f);

        g.params.dynamicFriction = 0.5f;
        g.params.viscosity = 50.0f;
        g.params.adhesion = 0.5f;
        g.params.cohesion = 0.08f;
        g.params.surfaceTension = 0.0f;

        g.numExtraParticles = 64 * 1024;

        AddBox(b, float3(1.0f, 1.5f, 0.1f), float3(0.0f, 1.5f, 0.0f));
        AddBox(b, float3(1.0f, 0.1f, 6.0f), float3(-1.0f, 3.0f, 0.0f));

        g.emitters[0].mEnabled = true;
        g.emitters[0].mSpeed = (g.params.fluidRestDistance * 2.f / g.dt);

        g.params.numPlanes = 3;

        // draw options
        g.drawEllipsoids = true;

        g.pause = false;
    }
};

class Buoyancy : public Scene {
 public:
    explicit Buoyancy(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.1f;
        const float restDistance = radius * 0.5f;
        int group = 0;

        const int n = 3;
        const float spacing = 64 * restDistance * 0.9f / (2.0f * n);
        const float sampling = restDistance * 0.8f;
        const float3 size = float3(sampling * 12.0f);

        const float mass[] = {1.0f, 0.25f, 0.005f};

        for (int j = 0; j < 1; ++j)
            for (int i = 0; i < n; ++i)
                CreateParticleShape(b, g.AssetPath("sphere.ply"),
                                    float3(spacing - 0.5f * size.x + float(i) * spacing * 2,
                                           2.0f + float(j) * size.y * 1.2f, 0.6f),
                                    size, 0.0f, sampling, float3(0.0f, 0.0f, 0.0f), mass[i], true, 1.0f,
                                    makePhase(group++, 0), true, 0.0001f);

        g.numSolidParticles = int(b.positions.size());

        const int x = 64;
        const int y = 20;
        const int z = 32;

        CreateParticleGrid(b, float3(0.0f), x, y, z, restDistance * 0.9f, float3(0.0f), 1.0f, false, 0.0f,
                           makePhase(group++, PhaseSelfCollide | PhaseFluid));

        g.params.radius = radius;
        g.params.dynamicFriction = 0.1f;
        g.params.viscosity = 2.0f;
        g.params.numIterations = 4;
        g.params.vorticityConfinement = 180.f;
        g.params.fluidRestDistance = restDistance;
        g.params.numPlanes = 5;
        g.params.cohesion = 0.001625f;
        g.params.collisionDistance = restDistance;
        g.params.solidPressure = 0.1f;
        g.params.restitution = 0.0f;
        g.params.relaxationFactor = 1.0f;
        g.params.relaxationMode = RelaxationMode::Local;

        g.numSubsteps = 2;

        g.maxDiffuseParticles = 64 * 1024;
        g.diffuseScale = 0.25f;
        g.diffuseShadow = false;
        g.diffuseColor = float4(1.5f);
        g.diffuseMotionScale = 1.5f;
        g.params.diffuseBallistic = 35;
        g.params.diffuseThreshold *= 0.1f;

        g.drawPlaneBias = g.params.collisionDistance * 1.5f;

        g.lightDistance *= 0.65f;

        g.fluidColor = float4(0.2f, 0.6f, 0.9f, 1.0f);

        // draw options
        g.drawDensity = true;
        g.drawDiffuse = true;
        g.drawEllipsoids = true;
        g.drawPoints = false;

        g.warmup = true;
    }
};

class SurfaceTension : public Scene {
 public:
    SurfaceTension(const char* name, float surfaceTension) : Scene(name), surfaceTension(surfaceTension) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        mCounter = 0;

        const float radius = 0.1f;
        const float restDistance = radius * 0.55f;
        const int phase = makePhase(0, PhaseSelfCollide | PhaseFluid);

        CreateParticleGrid(b, float3(0.0f, 0.2f, -1.0f), 16, 64, 16, restDistance, float3(0.0f), 1.0f, false, 0.0f,
                           phase, 0.005f);

        g.params.radius = radius;

        g.params.numIterations = 3;
        g.params.vorticityConfinement = 0.0f;
        g.params.fluidRestDistance = restDistance;
        g.params.smoothing = 0.5f;
        g.params.relaxationFactor = 1.f;
        g.params.restitution = 0.0f;
        g.params.collisionDistance = 0.01f;

        g.params.dynamicFriction = 0.25f;
        g.params.viscosity = 0.5f;
        g.params.cohesion = 0.1f;
        g.params.adhesion = 0.0f;
        g.params.surfaceTension = surfaceTension;

        g.params.gravity[1] = 0.0f;

        g.numExtraParticles = 64 * 1024;

        g.emitters[0].mEnabled = true;
        g.emitters[0].mSpeed = (g.params.fluidRestDistance * 2.f / g.dt);

        g.lightDistance *= 2.0f;

        // draw options
        g.drawEllipsoids = true;
    }

    void Update(DemoState& g) override {
        if (g.frame == 300)
            g.params.gravity[1] = -9.8f;
    }

    int mCounter = 0;
    float surfaceTension;
};

class RigidFluidCoupling : public Scene {
 public:
    explicit RigidFluidCoupling(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float minSize = 0.5f;
        const float maxSize = 1.0f;

        const float radius = 0.1f;
        int group = 0;

        Randf();

        // FleX's random arguments in the order its build draws them (see scene_common.h)
        for (int i = 0; i < 5; i++) {
            const float z = Randf(0.0f, 2.0f);
            const float angle = Randf(0.0f, k2Pi * 10.0f);
            AddRandomConvex(b, 10, float3(float(i) * 2.0f, 0.0f, z), minSize, maxSize, float3(0.0f, 1.0f, 0.0f), angle);
        }

        for (int z = 0; z < 10; ++z)
            for (int x = 0; x < 50; ++x) {
                // the scale's draw first, then the jitter's z and x
                const float3 scale = float3(2.0f, 2.0f + Randf(0.0f, 4.0f), 2.0f) * radius * 0.5f;
                const float jitterZ = Randf(radius);
                const float jitterX = Randf(radius);
                CreateParticleShape(b, g.AssetPath("box.ply"),
                                    float3(float(x) * radius * 2 - 1.0f, 1.0f + radius, 1.f + float(z) * 2.0f * radius) +
                                        0.5f * float3(jitterX, 0.0f, jitterZ),
                                    scale, 0.0f, radius * 0.5f, float3(0.0f), 1.0f, true, 1.0f, makePhase(group++, 0),
                                    true, 0.0f);
            }

        // separte solid particle count
        g.numSolidParticles = int(b.positions.size());

        // number of fluid particles to allocate
        g.numExtraParticles = 64 * 1024;

        g.params.radius = radius;
        g.params.dynamicFriction = 0.5f;
        g.params.viscosity = 0.1f;
        g.params.numIterations = 3;
        g.params.vorticityConfinement = 0.0f;
        g.params.fluidRestDistance = g.params.radius * 0.55f;
        g.params.solidRestDistance = g.params.radius * 0.5f;

        g.emitters[0].mEnabled = true;
        g.emitters[0].mSpeed = 2.0f * (g.params.fluidRestDistance) / g.dt;

        // draw options
        g.drawPoints = false;
        g.drawEllipsoids = true;
    }
};

#endif /* FLEX_DEMO_SCENES_FLUIDS_H */
