#ifndef FLEX_DEMO_SCENES_FORCE_FIELD_H
#define FLEX_DEMO_SCENES_FORCE_FIELD_H

// ForceFieldScene: reference/flex/demo/scenes/forcefield.h (FleX's class ForceField; renamed, as ephys::ForceField is
// the engine's field). A sheet of 60000 small particles swept by an attracting
// radial force field that swings from side to side and breathes, drawn as two circles.
//
// FleX's NvFlexExtCreateForceFieldCallback on the solver and NvFlexExtSetForceFields of every Update are here the
// scene's DemoState::forceFields, which the framework keeps as a PhysForceFieldLeaf of the scene graph; the scene
// service registers the engine's IForceFieldSet on the solver and sends it the field.

#include "scene_common.h"

class ForceFieldScene : public Scene {
 public:
    explicit ForceFieldScene(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        const float radius = 0.01f;
        const int phase = makePhase(0, PhaseSelfCollide);

        CreateParticleGrid(g.buffers, float3(-1.0f, radius, 0.0f), 200, 6, 50, radius * 0.5f, float3(0.0f), 1.0f, false,
                           0.0f, phase);

        g.params.radius = radius * 1.0f;
        g.params.dynamicFriction = 0.4f;
        g.params.staticFriction = 0.4f;
        g.params.particleFriction = 0.25f;
        g.params.dissipation = 0.0f;
        g.params.numIterations = 2;
        g.params.viscosity = 0.0f;

        g.numSubsteps = 2;

        // draw options
        g.drawPoints = true;

        forcefield = ephys::ForceField();
    }

    void PostInitialize(DemoState& g) override {
        // expand scene bounds to include force field
        g.sceneLower -= float3(1.0f);
        g.sceneUpper += float3(1.0f);
    }

    void DrawCircle(DemoState& g, const float3& pos, const float3& u, const float3& v, float radius, int segments) {
        g.context->BeginLines();

        float3 start = pos + radius * v;

        for (int i = 1; i <= segments; ++i) {
            const float theta = k2Pi * (float(i) / float(segments));
            const float3 end = pos + radius * sinf(theta) * u + radius * cosf(theta) * v;

            g.context->DrawLine(start, end, float4(1.0f));

            start = end;
        }

        g.context->EndLines();
    }

    void Draw(DemoState& g, int pass) override {
        (void)pass;
        const float3 position(forcefield.position[0], forcefield.position[1], forcefield.position[2]);
        DrawCircle(g, position, float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), forcefield.radius, 20);
        DrawCircle(g, position, float3(0.0f, 0.0f, 1.0f), float3(0.0f, 1.0f, 0.0f), forcefield.radius, 20);
    }

    void Update(DemoState& g) override {
        const float time = float(g.frame) * g.dt;

        forcefield.position[0] = sinf(time);
        forcefield.position[1] = 0.5f;
        forcefield.position[2] = 0.0f;
        forcefield.radius = (sinf(time * 1.5f) * 0.5f + 0.5f);
        forcefield.strength = -30.0f;
        forcefield.mode = ForceMode::Force;
        forcefield.linearFalloff = true;

        g.forceFields.assign(1, forcefield);
    }

    ephys::ForceField forcefield;
};

#endif /* FLEX_DEMO_SCENES_FORCE_FIELD_H */
