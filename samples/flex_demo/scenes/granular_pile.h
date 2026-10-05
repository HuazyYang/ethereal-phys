#ifndef FLEX_DEMO_SCENES_GRANULAR_PILE_H
#define FLEX_DEMO_SCENES_GRANULAR_PILE_H

// GranularPile: FleX/demo/scenes/granularpile.h. A sand castle of granular particles; after three
// seconds a rigid ball is launched into it.

#include "scene_common.h"

class GranularPile : public Scene {
 public:
    explicit GranularPile(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        // granular pile
        const float radius = 0.075f;

        const float3 lower(8.0f, 4.0f, 2.0f);

        CreateParticleShape(g.buffers, g.AssetPath("sphere.ply"), lower, float3(1.0f), 0.0f, radius, float3(0.0f),
                            0.0f, true, 1.0f, makePhase(1, 0), true, 0.00f);
        g.numSolidParticles = int(g.buffers.positions.size());

        CreateParticleShape(g.buffers, g.AssetPath("sandcastle.obj"), float3(-2.0f, -radius * 0.15f, 0.0f),
                            float3(4.0f), 0.0f, radius * 1.0001f, float3(0.0f), 1.0f, false, 0.0f,
                            makePhase(0, PhaseSelfCollide), false, 0.00f);

        g.numSubsteps = 2;

        g.params.radius = radius;
        g.params.staticFriction = 1.0f;
        g.params.dynamicFriction = 0.5f;
        g.params.viscosity = 0.0f;
        g.params.numIterations = 12;
        g.params.particleCollisionMargin = g.params.radius * 0.25f;  // 5% collision margin
        g.params.sleepThreshold = g.params.radius * 0.25f;
        g.params.shockPropagation = 6.f;
        g.params.restitution = 0.2f;
        g.params.relaxationFactor = 1.f;
        g.params.damping = 0.14f;
        g.params.numPlanes = 1;

        // draw options
        g.drawPoints = true;
        g.warmup = false;

        // hack, change the color of phase 0 particles to 'sand'
        g.colors[0] = float4(0.805f, 0.702f, 0.401f, 1.0f);
    }

    void Update(DemoState& g) override {
        // launch the ball after 3 seconds
        if (g.frame == 180) {
            g.MapBuffers();
            for (int i = 0; i < g.numSolidParticles; ++i) {
                g.buffers.positions[size_t(i)].w = 0.9f;
                g.buffers.velocities[size_t(i)] = float3(-15.0f, 0.0f, 0.0f);
            }
        }
    }
};

#endif /* FLEX_DEMO_SCENES_GRANULAR_PILE_H */
