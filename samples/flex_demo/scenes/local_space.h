#ifndef FLEX_DEMO_SCENES_LOCAL_SPACE_H
#define FLEX_DEMO_SCENES_LOCAL_SPACE_H

// Simulation in a moving frame (IMovingFrame, FleX's NvFlexExtMovingFrame):
//   LocalSpaceCloth  reference/flex/demo/scenes/localspacecloth.h: a cloth draped over a box that turns and slides
//                    (the panel's Rotation and Translation), the particles moved with it and given a part of its
//                    inertia (Linear / Angular Inertia)
//   LocalSpaceFluid  reference/flex/demo/scenes/localspacefluid.h: fluid inside a torus (a triangle mesh with its
//                    faces turned inwards) that turns and slides the same way
//
// The frame is applied to the host mirrors of the particles, as FleX applied it to its mapped buffers: Update maps
// them every frame (a wait for the GPU, as in FleX), and the frame sends them back before the step.
//
// What differs from FleX: the previous transform of a shape is the scene graph's (the node's transform of the last
// frame, which the scene service sweeps from). LocalSpaceCloth's AddBox gave FleX's box a previous transform equal
// to its current one every frame; here the box turns from its last pose. With the panel's Rotation at 0 (the
// start) the two are the same.

#include "scene_common.h"

class LocalSpaceCloth : public Scene {
 public:
    explicit LocalSpaceCloth(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float stretchStiffness = 1.0f;
        const float bendStiffness = 1.0f;
        const float shearStiffness = 1.0f;

        const float radius = 0.1f;

        CreateSpringGrid(b, float3(0.5f, 1.45f, -0.5f), 32, 20, 1, radius * 0.5f,
                         makePhase(0, PhaseSelfCollide | PhaseSelfCollideFilter), stretchStiffness, bendStiffness,
                         shearStiffness, float3(0.0f), 1.0f);

        const int c1 = 1;
        const int c2 = 20;

        // add tethers
        for (int i = 0; i < int(b.positions.size()); ++i) {
            float minSqrDist = FLT_MAX;

            if (i != c1 && i != c2) {
                const float stiffness = -0.8f;
                const float give = 0.01f;

                const float sqrDist = dm::lengthSquared(b.positions[c1].xyz() - b.positions[c2].xyz());

                if (sqrDist < minSqrDist) {
                    CreateSpring(b, c1, i, stiffness, give);
                    CreateSpring(b, c2, i, stiffness, give);

                    minSqrDist = sqrDist;
                }
            }
        }

        for (float4& p : b.positions) {
            if (p.x == 0.5f)
                p.w = 0.0f;
        }

        translation = float3(0.0f, 1.0f, 0.0f);
        size = float3(0.5f, 1.1f, 0.6f);
        rotation = 0.0f;
        rotationSpeed = 0.0f;

        linearInertialScale = 0.25f;
        angularInertialScale = 0.25f;

        AddBox(b, size, translation);

        // initialize our moving frame to the center of the box
        meshFrame = nullptr;
        const float4 identity = QuatIdentity();
        if (g.phys)
            g.phys->createMovingFrame(&translation.x, &identity.x, &meshFrame);

        g.numSubsteps = 2;

        g.params.radius = radius;
        g.params.dynamicFriction = 0.f;
        g.params.restitution = 0.0f;
        g.params.shapeCollisionMargin = 0.05f;

        g.params.numIterations = 6;

        // draw options
        g.drawPoints = false;
    }

    void DoGui(DemoState& g) override {
        (void)g;
        ImGui::SliderFloat("Rotation", &rotationSpeed, 0.0f, 20.0f);
        ImGui::SliderFloat("Translation", &translation.x, -2.0f, 2.0f);

        ImGui::SliderFloat("Linear Inertia", &linearInertialScale, 0.0f, 1.0f);
        ImGui::SliderFloat("Angular Inertia", &angularInertialScale, 0.0f, 1.0f);
    }

    void Update(DemoState& g) override {
        if (!meshFrame)
            return;
        SimBuffers& b = g.buffers;

        rotation += rotationSpeed * g.dt;

        // new position of the box center
        const float3 newPosition = translation;
        const float4 newRotation = QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), rotation);

        meshFrame->update(&newPosition.x, &newRotation.x, g.dt);

        // update all the particles in the sim with inertial forces
        g.MapBuffers();
        meshFrame->apply(&b.positions[0].x, &b.velocities[0].x, uint32_t(b.positions.size()), linearInertialScale,
                         angularInertialScale, g.dt);

        // update collision shapes
        b.shapes.clear();

        AddBox(b, size, newPosition, newRotation);

        UpdateShapes(b);
    }

    float3 size = float3(0.0f);
    float3 translation = float3(0.0f);
    float rotation = 0.0f;
    float rotationSpeed = 0.0f;

    float linearInertialScale = 0.0f;
    float angularInertialScale = 0.0f;

    MovingFrameHandle meshFrame;
};

class LocalSpaceFluid : public Scene {
 public:
    explicit LocalSpaceFluid(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.05f;
        const float restDistance = radius * 0.6f;

        const int dx = int(ceilf(1.f / restDistance));
        const int dy = int(ceilf(1.f / restDistance));
        const int dz = int(ceilf(1.f / restDistance));

        CreateParticleGrid(b, float3(0.0f, 1.0f, 0.0f), dx, dy, dz, restDistance, float3(0.0f), 1.0f, false, 0.0f,
                           makePhase(0, PhaseSelfCollide | PhaseFluid), restDistance * 0.01f);

        float3 lower, upper;
        GetParticleBounds(b, lower, upper);

        const float3 center = (lower + upper) * 0.5f;

        std::unique_ptr<Mesh> shape = utils::ImportMesh(g.AssetPath("torus.obj"));
        meshFrame = nullptr;
        if (!shape)
            return;
        shape->Transform(dm::scaling(float3(0.7f)));

        // invert box faces
        for (uint32_t i = 0; i < shape->GetNumFaces(); ++i)
            std::swap(shape->m_indices[i * 3 + 0], shape->m_indices[i * 3 + 1]);

        shape->CalculateNormals();

        // shift into torus interior
        for (float4& p : b.positions) {
            p.x -= 2.1f;
        }

        const TriangleMeshId mesh = CreateTriangleMesh(shape.get());
        AddTriangleMesh(b, mesh, center, QuatIdentity(), float3(1.0f));

        // initialize our moving frame to the center of the box
        const float4 identity = QuatIdentity();
        if (g.phys)
            g.phys->createMovingFrame(&center.x, &identity.x, &meshFrame);

        g.numSubsteps = 2;

        g.params.radius = radius;
        g.params.fluidRestDistance = restDistance;
        g.params.dynamicFriction = 0.f;
        g.params.restitution = 0.0f;
        g.params.collisionDistance = 0.05f;
        g.params.shapeCollisionMargin = 0.00001f;
        g.params.maxSpeed = float(g.numSubsteps) * restDistance / g.dt;

        g.params.numIterations = 4;

        g.params.smoothing = 0.4f;
        g.params.viscosity = 0.001f;
        g.params.cohesion = 0.05f;
        g.params.surfaceTension = 0.0f;

        translation = center;
        rotation = 0.0f;
        rotationSpeed = 0.0f;

        linearInertialScale = 0.25f;
        angularInertialScale = 0.75f;

        // draw options
        g.drawPoints = false;
        g.drawEllipsoids = true;
        g.drawDiffuse = true;
    }

    void DoGui(DemoState& g) override {
        (void)g;
        ImGui::SliderFloat("Rotation", &rotationSpeed, 0.0f, 7.0f);
        ImGui::SliderFloat("Translation", &translation.x, -2.0f, 2.0f);
        ImGui::SliderFloat("Linear Inertia", &linearInertialScale, 0.0f, 1.0f);
        ImGui::SliderFloat("Angular Inertia", &angularInertialScale, 0.0f, 1.0f);
    }

    void Update(DemoState& g) override {
        if (!meshFrame || g.buffers.shapes.empty())
            return;
        SimBuffers& b = g.buffers;

        rotation += rotationSpeed * g.dt;

        // new position of the box center
        const float3 newPosition = translation;
        const float4 newRotation = QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), rotation);

        meshFrame->update(&newPosition.x, &newRotation.x, g.dt);

        // update all the particles in the sim with inertial forces
        g.MapBuffers();
        meshFrame->apply(&b.positions[0].x, &b.velocities[0].x, uint32_t(b.positions.size()), linearInertialScale,
                         angularInertialScale, g.dt);

        // update torus transform
        b.shapes[0].prevPosition = b.shapes[0].position;
        b.shapes[0].prevRotation = b.shapes[0].rotation;

        b.shapes[0].position = newPosition;
        b.shapes[0].rotation = newRotation;

        UpdateShapes(b);
    }

    float3 translation = float3(0.0f);
    float rotation = 0.0f;
    float rotationSpeed = 0.0f;

    float linearInertialScale = 0.0f;
    float angularInertialScale = 0.0f;

    MovingFrameHandle meshFrame;
};

#endif /* FLEX_DEMO_SCENES_LOCAL_SPACE_H */
