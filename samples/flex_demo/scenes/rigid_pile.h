#ifndef FLEX_DEMO_SCENES_RIGID_PILE_H
#define FLEX_DEMO_SCENES_RIGID_PILE_H

// RigidPile: reference/flex/demo/scenes/rigidpile.h. A 10 x 10 x 10 stack of rigid boxes (shape matching, a
// skinned box mesh each) dropped onto a plinth. FleX's disabled branches (the brick wall, the spring grid) are
// not ported.

#include "scene_common.h"

#include <memory>

class RigidPile : public Scene {
 public:
    RigidPile(const char* name, int brickHeight) : Scene(name), mHeight(brickHeight) {}

    void Initialize(DemoState& g) override {
        const int sx = 2;
        const int sy = mHeight;
        const int sz = 2;

        const float3 lower(0.0f, 1.5f + g.params.radius * 0.25f, 0.0f);

        const int dimx = 10;
        const int dimy = 10;
        const int dimz = 10;

        std::unique_ptr<Mesh> mesh = ephys::utils::ImportMesh(g.AssetPath("box.ply"));

        // a basic grid
        for (int y = 0; y < dimy; ++y)
            for (int z = 0; z < dimz; ++z)
                for (int x = 0; x < dimx; ++x)
                    CreateParticleShape(
                        g.buffers, mesh.get(),
                        (g.params.radius * 0.905f) * float3(float(x * sx), float(y * sy), float(z * sz)) +
                            (g.params.radius * 0.1f) * float3(float(x), float(y), float(z)) + lower,
                        g.params.radius * 0.9f * float3(float(sx), float(sy), float(sz)), 0.0f, g.params.radius * 0.9f,
                        float3(0.0f), 1.0f, true, 1.0f, makePhase(int(g.buffers.rigidOffsets.size()) + 1, 0), true,
                        0.002f);

        mesh.reset();

        AddPlinth(g.buffers);

        g.numSubsteps = 2;
        g.params.numIterations = 8;

        g.params.radius *= 1.0f;
        g.params.dynamicFriction = 0.4f;
        g.params.dissipation = 0.01f;
        g.params.particleCollisionMargin = g.params.radius * 0.05f;
        g.params.sleepThreshold = g.params.radius * 0.25f;
        g.params.shockPropagation = 3.f;

        g.windStrength = 0.0f;

        // draw options
        g.drawPoints = false;

        g.emitters[0].mEnabled = true;
        g.emitters[0].mSpeed = (g.params.radius * 2.0f / g.dt);
    }

 private:
    int mHeight;
};

#endif /* FLEX_DEMO_SCENES_RIGID_PILE_H */
