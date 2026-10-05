#ifndef FLEX_DEMO_SCENES_FLUID_BLOCK_H
#define FLEX_DEMO_SCENES_FLUID_BLOCK_H

// FluidBlock: FleX/demo/scenes/fluidblock.h. A block of fluid over random convex rocks, with a stack of
// rigid tori: the screen-space fluid surface, the densities and the diffuse particles, after FleX's warm-up.

#include "scene_common.h"

class FluidBlock : public Scene {
 public:
    explicit FluidBlock(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        const float minSize = 0.5f;
        const float maxSize = 0.7f;

        const float radius = 0.1f;
        const float restDistance = radius * 0.55f;
        int group = 0;

        AddRandomConvex(g.buffers, 6, float3(5.0f, -0.1f, 0.6f), 1.0f, 1.0f, float3(1.0f, 1.0f, 0.0f), 0.0f);

        const float ly = 0.5f;

        AddRandomConvex(g.buffers, 10, float3(2.5f, ly * 0.5f, 1.f), minSize * 0.5f, maxSize * 0.5f,
                        float3(0.0f, 1.0f, 0.0f), Randf(0.0f, 2.0f * kPi));

        AddRandomConvex(g.buffers, 12, float3(3.8f, ly - 0.5f, 1.f), minSize, maxSize, float3(1.0f, 0.0f, 0.0f),
                        Randf(0.0f, 2.0f * kPi));
        AddRandomConvex(g.buffers, 12, float3(3.8f, ly - 0.5f, 2.6f), minSize, maxSize, float3(1.0f, 0.0f, 0.0f),
                        0.2f + Randf(0.0f, 2.0f * kPi));

        AddRandomConvex(g.buffers, 12, float3(4.6f, ly, 0.2f), minSize, maxSize, float3(1.0f, 0.0f, 1.0f),
                        Randf(0.0f, 2.0f * kPi));
        AddRandomConvex(g.buffers, 12, float3(4.6f, ly, 2.0f), minSize, maxSize, float3(1.0f, 0.0f, 1.0f),
                        0.2f + Randf(0.0f, 2.0f * kPi));

        const float size = 0.3f;
        for (int i = 0; i < 32; ++i)
            CreateParticleShape(g.buffers, g.AssetPath("torus.obj"),
                                float3(4.5f, 2.0f + radius * 2.0f * float(i), 1.0f), float3(size), 0.0f,
                                radius * 0.5f, float3(0.0f, 0.0f, 0.0f), 0.125f, true, 1.0f, makePhase(group++, 0),
                                true, 0.0f);

        g.numSolidParticles = int(g.buffers.positions.size());

        const float sizex = 1.76f;
        const float sizey = 2.20f;
        const float sizez = 3.50f;

        const int x = int(sizex / restDistance);
        const int y = int(sizey / restDistance);
        const int z = int(sizez / restDistance);

        CreateParticleGrid(g.buffers, float3(0.0f, restDistance * 0.5f, 0.0f), x, y, z, restDistance, float3(0.0f),
                           1.0f, false, 0.0f, makePhase(group++, PhaseSelfCollide | PhaseFluid));

        g.params.radius = radius;
        g.params.dynamicFriction = 0.0f;
        g.params.viscosity = 0.0f;
        g.params.numIterations = 3;
        g.params.vorticityConfinement = 40.f;
        g.params.fluidRestDistance = restDistance;
        g.params.numPlanes = 5;

        g.maxDiffuseParticles = 128 * 1024;
        g.diffuseScale = 0.75f;

        g.waveFloorTilt = -0.025f;

        g.lightDistance *= 0.5f;

        // draw options
        g.drawDensity = true;
        g.drawDiffuse = true;
        g.drawEllipsoids = true;
        g.drawPoints = false;

        g.warmup = true;
    }
};

#endif /* FLEX_DEMO_SCENES_FLUID_BLOCK_H */
