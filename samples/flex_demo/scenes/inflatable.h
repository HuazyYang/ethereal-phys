#ifndef FLEX_DEMO_SCENES_INFLATABLE_H
#define FLEX_DEMO_SCENES_INFLATABLE_H

// InflatableScene: reference/flex/demo/scenes/inflatable.h (FleX's class Inflatable; renamed, as ephys::Inflatable
// is the engine's constraint). Eight closed cloth meshes (boxes and spheres) held at an
// over-pressure by a volume constraint each, dropped on random convex rocks; each is drawn as its own cloth.
//
// FleX built the springs and the rest volume with its core ClothMesh over the global particle array; that class is
// the engine's (module-private), and the public path to it is the asset builder's createClothFromMesh (FleX's
// NvFlexExtCreateClothFromMesh, which is that ClothMesh with the same arguments): the same springs, rest lengths,
// rest volume and constraint scale, with the indices of the mesh, offset here by its first particle.

#include "scene_common.h"

// What AddInflatable and XPBD Inflatable Balloon's AddBalloon share: the mesh's particles, its triangles (wound for
// the inflatable, the normals pointing in as FleX's were), the springs of a cloth of the mesh and the inflatable.
// Returns false when the builder refuses the mesh.
inline bool AddInflatableMesh(DemoState& g, IPhysAssetBuilder* builder, const Mesh* mesh, float stretchStiffness,
                              float bendStiffness, float overPressure, int phase) {
    SimBuffers& b = g.buffers;
    const int startVertex = int(b.positions.size());

    // add mesh to system
    std::vector<float4> particles;
    for (const float3& p : mesh->m_positions) {
        b.positions.push_back(float4(p.x, p.y, p.z, 1.0f));
        b.velocities.push_back(float3(0.0f));
        b.phases.push_back(phase);
        particles.push_back(float4(p.x, p.y, p.z, 1.0f));
    }

    const int triOffset = int(b.triangles.size());
    const int triCount = int(mesh->GetNumFaces());

    b.inflatableTriOffsets.push_back(triOffset / 3);
    b.inflatableTriCounts.push_back(triCount);
    b.inflatablePressures.push_back(overPressure);

    for (size_t i = 0; i < mesh->m_indices.size(); i += 3) {
        const uint32_t a = mesh->m_indices[i + 0];
        const uint32_t bb = mesh->m_indices[i + 1];
        const uint32_t c = mesh->m_indices[i + 2];

        const float3 n = -dm::normalize(dm::cross(mesh->m_positions[bb] - mesh->m_positions[a],
                                                  mesh->m_positions[c] - mesh->m_positions[a]));
        b.triangleNormals.push_back(n);

        b.triangles.push_back(int(a) + startVertex);
        b.triangles.push_back(int(bb) + startVertex);
        b.triangles.push_back(int(c) + startVertex);
    }

    // a cloth of the mesh (FleX: new ClothMesh(positions, n, indices, 3 * triCount, stretch, bend))
    ClothAssetParams params;
    params.stretchStiffness = stretchStiffness;
    params.bendStiffness = bendStiffness;
    params.pressure = overPressure;
    PhysAssetHandle cloth;
    if (NVRHI_FAILED(builder->createClothFromMesh(&particles[0].x, uint32_t(particles.size()),
                                                  reinterpret_cast<const int32_t*>(mesh->m_indices.data()),
                                                  uint32_t(triCount), params, &cloth)))
        return false;

    const int32_t* indices = cloth->getSpringIndices();
    for (uint32_t i = 0; i < cloth->getSpringCount() * 2; ++i)
        b.springIndices.push_back(indices[i] + startVertex);
    const float* coefficients = cloth->getSpringCoefficients();
    const float* lengths = cloth->getSpringRestLengths();
    for (uint32_t i = 0; i < cloth->getSpringCount(); ++i) {
        b.springStiffness.push_back(coefficients[i]);
        b.springLengths.push_back(lengths[i]);
    }

    // add inflatable params
    b.inflatableVolumes.push_back(cloth->getInflatableVolume());
    b.inflatableCoefficients.push_back(cloth->getInflatableStiffness());
    return true;
}

class InflatableScene : public Scene {
 public:
    explicit InflatableScene(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        mTriCounts.clear();

        const float minSize = 0.75f;
        const float maxSize = 1.0f;

        // convex rocks (FleX's two random arguments in the order its build draws them, see scene_common.h)
        for (int i = 0; i < 5; i++) {
            const float z = Randf(0.0f, 2.0f);
            const float angle = Randf(0.0f, k2Pi);
            AddRandomConvex(g.buffers, 10, float3(float(i) * 2.0f, 0.0f, z), minSize, maxSize, float3(0.0f, 1.0f, 0.0f),
                            angle);
        }

        const float radius = 0.12f;
        int group = 0;

        const char* meshes[2] = {"box_high_weld.ply", "sphere.ply"};

        mPressure = 1.0f;

        PhysAssetBuilderHandle builder;
        if (!g.phys || NVRHI_FAILED(g.phys->createAssetBuilder(&builder)))
            return;

        for (int y = 0; y < 2; ++y) {
            for (int i = 0; i < 4; ++i) {
                std::unique_ptr<Mesh> mesh = utils::ImportMesh(g.AssetPath(meshes[(i + y) & 1]));
                if (!mesh)
                    continue;
                mesh->Normalize();
                mesh->Transform(dm::translation(float3(float(i) * 2.0f, 1.0f + float(y) * 2.0f, 1.5f)));

                if (AddInflatableMesh(g, builder, mesh.get(), 0.8f, 1.0f, mPressure, makePhase(group++, 0)))
                    mTriCounts.push_back(int(mesh->GetNumFaces()));
            }
        }

        g.params.radius = radius;
        g.params.dynamicFriction = 0.4f;
        g.params.dissipation = 0.0f;
        g.params.numIterations = 10;
        g.params.particleCollisionMargin = g.params.radius * 0.05f;
        g.params.drag = 0.0f;
        g.params.collisionDistance = 0.01f;

        // better convergence with global relaxation factor
        g.params.relaxationMode = RelaxationMode::Global;
        g.params.relaxationFactor = 0.25f;

        g.windStrength = 0.0f;

        g.numSubsteps = 2;

        // draw options
        g.drawPoints = false;
        g.drawSprings = 0;
        g.drawCloth = false;
    }

    void DoGui(DemoState& g) override {
        if (ImGui::SliderFloat("Over Pressure", &mPressure, 0.25f, 3.0f)) {
            for (float& p : g.buffers.inflatablePressures)
                p = mPressure;
        }
    }

    void Sync(DemoState& g) override { g.SetInflatables(); }

    void Draw(DemoState& g, int pass) override {
        (void)pass;
        if (!g.drawMesh || !g.clothBuffers)
            return;

        ClothRenderBuffers cloth = *g.clothBuffers;
        uint32_t indexStart = 0;

        for (size_t i = 0; i < mTriCounts.size(); ++i) {
            cloth.firstTriangle = indexStart;
            cloth.numTriangles = uint32_t(mTriCounts[i]);
            g.context->DrawCloth(cloth, int(i % 6), g.params.radius * 0.35f);

            indexStart += uint32_t(mTriCounts[i]);
        }
    }

    float mPressure = 1.0f;
    std::vector<int> mTriCounts;  // FleX's mCloths[i]->mTris.size()
};

#endif /* FLEX_DEMO_SCENES_INFLATABLE_H */
