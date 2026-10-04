#ifndef FLEX_DEMO_SCENES_FLAG_CLOTH_H
#define FLEX_DEMO_SCENES_FLAG_CLOTH_H

// FlagCloth: reference/flex/demo/scenes/flag.h. A 64 x 32 cloth hung from two corners, with tethers to them,
// blown by a noisy wind.

#include "scene_common.h"

class FlagCloth : public Scene {
 public:
    explicit FlagCloth(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        const int dimx = 64;
        const int dimz = 32;
        const float radius = 0.05f;

        const float stretchStiffness = 0.9f;
        const float bendStiffness = 1.0f;
        const float shearStiffness = 0.9f;
        const int phase = makePhase(0, PhaseSelfCollide);

        CreateSpringGrid(g.buffers, float3(0.0f, 0.0f, -3.0f), dimx, dimz, 1, radius, phase, stretchStiffness,
                         bendStiffness, shearStiffness, float3(0.0f), 1.0f);

        const int c1 = 0;
        const int c2 = dimx * (dimz - 1);

        g.buffers.positions[c1].w = 0.0f;
        g.buffers.positions[c2].w = 0.0f;

        // add tethers
        for (int i = 0; i < int(g.buffers.positions.size()); ++i) {
            // hack to rotate cloth
            std::swap(g.buffers.positions[size_t(i)].y, g.buffers.positions[size_t(i)].z);
            g.buffers.positions[size_t(i)].y *= -1.0f;

            g.buffers.velocities[size_t(i)] = RandomUnitVector() * 0.1f;

            float minSqrDist = FLT_MAX;

            if (i != c1 && i != c2) {
                const float stiffness = -0.8f;
                const float give = 0.1f;

                const float3 d = g.buffers.positions[c1].xyz() - g.buffers.positions[c2].xyz();
                const float sqrDist = dm::dot(d, d);

                if (sqrDist < minSqrDist) {
                    CreateSpring(g.buffers, c1, i, stiffness, give);
                    CreateSpring(g.buffers, c2, i, stiffness, give);

                    minSqrDist = sqrDist;
                }
            }
        }

        g.params.radius = radius * 1.0f;
        g.params.dynamicFriction = 0.25f;
        g.params.dissipation = 0.0f;
        g.params.numIterations = 4;
        g.params.drag = 0.06f;
        g.params.relaxationFactor = 1.0f;

        g.numSubsteps = 2;

        // draw options
        g.drawPoints = false;
        g.drawSprings = 0;
        // FleX doubles the wind frequency on every initialisation of the scene (Init does not reset it)
        g.windFrequency *= 2.0f;
        g.windStrength = 10.0f;
    }

    void Update(DemoState& g) override {
        const float3 kWindDir = float3(3.0f, 15.0f, 0.0f);
        const float kNoise = fabsf(Perlin1D(g.windTime * 0.05f, 2, 0.25f));
        const float3 wind = g.windStrength * kWindDir * float3(kNoise, kNoise * 0.1f, -kNoise * 0.1f);

        g.params.wind[0] = wind.x;
        g.params.wind[1] = wind.y;
        g.params.wind[2] = wind.z;
    }
};

#endif /* FLEX_DEMO_SCENES_FLAG_CLOTH_H */
