#ifndef FLEX_DEMO_SCENES_COLLISION_H
#define FLEX_DEMO_SCENES_COLLISION_H

// Collision with triangle meshes and distance fields:
//   TriangleCollision  reference/flex/demo/scenes/trianglecollision.h: particles dropped on five shallow bowls,
//                      three triangle meshes of one disc, scaled per shape
//   SDFCollision       reference/flex/demo/scenes/sdfcollision.h: a cloth dropped on three distance field bunnies of
//                      three sizes (in FleX's scenes.h but not in its registry; registered here with the collision
//                      scenes)
//   Pasta              reference/flex/demo/scenes/pasta.h: twenty ropes poured into a bowl (a triangle mesh), drawn
//                      as tubes extruded along the ropes

#include "scene_common.h"

class TriangleCollision : public Scene {
 public:
    explicit TriangleCollision(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.05f;
        CreateParticleGrid(b, float3(0.4f, 1.0f + radius * 0.5f, 0.1f), 10, 5, 10, radius, float3(0.0f), 1.0f, false,
                           0.0f, makePhase(0, PhaseSelfCollide), 0.0f);

        std::unique_ptr<Mesh> disc = utils::CreateDiscMesh(1.0f, 4);

        // create shallow bowl
        disc->m_positions[0].y -= 0.5f;
        disc->CalculateNormals();

        const TriangleMeshId mesh1 = CreateTriangleMesh(disc.get());
        AddTriangleMesh(b, mesh1, float3(0.0f, 0.5f, 0.0f), QuatIdentity(), float3(1.0f, 0.5f, 1.0f));
        AddTriangleMesh(b, mesh1, float3(1.0f, 0.5f, 1.0f), QuatIdentity(), float3(1.0f, 0.5f, 1.0f));

        const TriangleMeshId mesh2 = CreateTriangleMesh(disc.get());
        AddTriangleMesh(b, mesh2, float3(-1.0f, 0.5f, 1.0f), QuatIdentity(), float3(1.0f, 0.5f, 1.0f));
        AddTriangleMesh(b, mesh2, float3(1.0f, 0.5f, -1.0f), QuatIdentity(), float3(1.0f, 1.0f, 1.0f));

        const TriangleMeshId mesh3 = CreateTriangleMesh(disc.get());
        AddTriangleMesh(b, mesh3, float3(-1.0f, 0.5f, -1.0f), QuatIdentity(), float3(1.0f, 0.25f, 1.0f));

        g.params.radius = radius;
        g.params.dynamicFriction = 0.025f;
        g.params.dissipation = 0.0f;
        g.params.restitution = 0.0;
        g.params.numIterations = 4;
        g.params.particleCollisionMargin = g.params.radius * 0.05f;

        g.numSubsteps = 1;

        // draw options
        g.drawPoints = true;
    }
};

class SDFCollision : public Scene {
 public:
    explicit SDFCollision(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const int dim = 128;

        const DistanceFieldId sdf = CreateSDF(g.AssetPath("bunny.ply"), dim);

        AddSDF(b, sdf, float3(-1.f, 0.0f, 0.0f), QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), DegToRad(-45.0f)), 0.5f);
        AddSDF(b, sdf, float3(0.0f, 0.0f, 0.0f), QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), DegToRad(0.0f)), 1.0f);
        AddSDF(b, sdf, float3(1.0f, 0.0f, 0.0f), QuatFromAxisAngle(float3(0.0f, 1.0f, 0.0f), DegToRad(45.0f)), 2.0f);

        const float stretchStiffness = 1.0f;
        const float bendStiffness = 0.8f;
        const float shearStiffness = 0.5f;

        const int dimx = 64;
        const int dimz = 64;
        const float radius = 0.05f;
        const int phase = makePhase(0, PhaseSelfCollide | PhaseSelfCollideFilter);

        CreateSpringGrid(b, float3(-0.6f, 2.9f, -0.6f), dimx, dimz, 1, radius * 0.75f, phase, stretchStiffness,
                         bendStiffness, shearStiffness, float3(0.0f), 1.0f);

        g.params.radius = radius * 1.0f;
        g.params.dynamicFriction = 0.4f;
        g.params.staticFriction = 0.4f;
        g.params.particleFriction = 0.25f;
        g.params.dissipation = 0.0f;
        g.params.numIterations = 8;
        g.params.viscosity = 0.0f;
        g.params.drag = 0.02f;
        g.params.lift = 0.1f;
        g.params.collisionDistance = radius * 0.5f;
        g.params.relaxationFactor = 1.3f;

        g.numSubsteps = 3;

        g.windStrength = 0.0f;

        // draw options
        g.drawPoints = false;
        g.drawSprings = 0;
    }
};

class Pasta : public Scene {
 public:
    explicit Pasta(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const float radius = 0.1f;
        const float length = 15.0f;
        const int n = 20;

        for (int i = 0; i < n; ++i) {
            const float theta = k2Pi * float(i) / float(n);

            Rope r;
            CreateRope(b, r, 0.5f * float3(cosf(theta), 2.0f, sinf(theta)), float3(0.0f, 1.0f, 0.0f), 0.25f,
                       int(length / radius), length, makePhase(0, PhaseSelfCollide));
            b.ropes.push_back(r);
        }

        g.numSubsteps = 3;

        std::unique_ptr<Mesh> bowl = utils::ImportMesh(g.AssetPath("bowl.obj"));
        if (bowl) {
            bowl->Normalize(2.0f);
            bowl->CalculateNormals();
            bowl->Transform(dm::translation(float3(-1.0f, 0.0f, -1.0f)));

            const TriangleMeshId mesh = CreateTriangleMesh(bowl.get());
            AddTriangleMesh(b, mesh, float3(0.0f), QuatIdentity(), float3(1.0f));
        }

        g.params.numIterations = 6;
        g.params.radius = radius;
        g.params.dynamicFriction = 0.4f;
        g.params.dissipation = 0.001f;
        g.params.sleepThreshold = g.params.radius * 0.2f;
        g.params.relaxationFactor = 1.3f;
        g.params.restitution = 0.0f;
        g.params.shapeCollisionMargin = 0.01f;

        g.lightDistance *= 0.5f;
        g.drawPoints = false;
    }
};

#endif /* FLEX_DEMO_SCENES_COLLISION_H */
