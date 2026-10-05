#ifndef FLEX_DEMO_SCENES_DAM_BREAK_H
#define FLEX_DEMO_SCENES_DAM_BREAK_H

// DamBreak: FleX/demo/scenes/dambreak.h. A block of fluid released against a wall: points by default,
// the screen-space fluid surface with 'e' (or --draw-fluid). The demo registers it three times, at particle radii
// of 5, 10 and 15 cm.

#include "scene_common.h"

class DamBreak : public Scene {
 public:
    DamBreak(const char* name, float radius) : Scene(name), mRadius(radius) {}

    void Initialize(DemoState& g) override {
        const float radius = mRadius;
        const float restDistance = mRadius * 0.65f;

        const int dx = int(ceilf(1.0f / restDistance));
        const int dy = int(ceilf(2.0f / restDistance));
        const int dz = int(ceilf(1.0f / restDistance));

        CreateParticleGrid(g.buffers, float3(0.0f, restDistance, 0.0f), dx, dy, dz, restDistance, float3(0.0f), 1.0f,
                           false, 0.0f, makePhase(0, PhaseSelfCollide | PhaseFluid), restDistance * 0.01f);

        g.sceneLower = float3(0.0f, 0.0f, -0.5f);
        g.sceneUpper = float3(3.0f, 0.0f, -0.5f);

        g.numSubsteps = 2;

        g.params.radius = radius;
        g.params.fluidRestDistance = restDistance;
        g.params.dynamicFriction = 0.f;
        g.params.restitution = 0.001f;

        g.params.numIterations = 3;
        g.params.relaxationFactor = 1.0f;

        g.params.smoothing = 0.4f;

        g.params.viscosity = 0.001f;
        g.params.cohesion = 0.1f;
        g.params.vorticityConfinement = 80.0f;
        g.params.surfaceTension = 0.0f;

        g.params.numPlanes = 5;

        // limit velocity to CFL condition
        g.params.maxSpeed = 0.5f * radius * float(g.numSubsteps) / g.dt;

        g.maxDiffuseParticles = 0;

        g.fluidColor = float4(0.113f, 0.425f, 0.55f, 1.0f);

        g.waveFrequency = 1.0f;
        g.waveAmplitude = 2.0f;
        g.waveFloorTilt = 0.0f;

        // draw options
        g.drawPoints = true;
        g.drawEllipsoids = false;
        g.drawDiffuse = true;
    }

 private:
    float mRadius;
};

#endif /* FLEX_DEMO_SCENES_DAM_BREAK_H */
