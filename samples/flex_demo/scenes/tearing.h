#ifndef FLEX_DEMO_SCENES_TEARING_H
#define FLEX_DEMO_SCENES_TEARING_H

// Tearing: reference/flex/demo/scenes/tearing.h. An irregular cloth hung by its top edge in the wind, torn every
// frame where a spring is stretched to three times its length (IPhysTearingCloth, FleX's NvFlexExtTearClothMesh):
// the clones become active particles, the triangles take the edits and the springs are sent again.
//
// What differs from FleX: FleX's Update added the clones to mCloth->numParticles after NvFlexExtTearClothMesh had
// counted them already, so its clones went to every other index (the ones between were never written but still
// made active) and the 1000 extra particles ran out twice as fast. The engine's tearing cloth counts its particles
// itself (ext/asset.h); the scene does not add them again.

#include "scene_common.h"

class Tearing : public Scene {
 public:
    explicit Tearing(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        mCloth = nullptr;
        mBuilder = nullptr;

        std::unique_ptr<Mesh> mesh = utils::ImportMesh(g.AssetPath("irregular_plane.obj"));
        if (!mesh)
            return;
        // FleX: RotationMatrix(kPi, y) * RotationMatrix(kPi * 0.5, x) * ScaleMatrix(2)
        mesh->Transform(dm::scaling(float3(2.0f)) * dm::rotation(float3(1.0f, 0.0f, 0.0f), kPi * 0.5f) *
                        dm::rotation(float3(0.0f, 1.0f, 0.0f), kPi));

        float3 lower, upper;
        mesh->GetBounds(lower, upper);

        const float radius = 0.065f;
        const int phase = makePhase(0, PhaseSelfCollide | PhaseSelfCollideFilter);

        for (size_t i = 0; i < mesh->GetNumVertices(); ++i) {
            float3 p = mesh->m_positions[i];

            float invMass = 1.0f;

            if (p.y == upper.y)
                invMass = 0.0f;

            p += float3(0.0f, 1.5f, 0.0f);

            b.positions.push_back(float4(p.x, p.y, p.z, invMass));
            b.velocities.push_back(float3(0.0f));
            b.phases.push_back(phase);
        }

        g.numExtraParticles = 1000;

        if (!g.phys || NVRHI_FAILED(g.phys->createAssetBuilder(&mBuilder)))
            return;

        TearingClothAssetParams params;
        params.stretchStiffness = 0.8f;
        params.bendStiffness = 0.8f;
        params.pressure = 0.0f;
        const uint32_t numParticles = uint32_t(b.positions.size());
        if (NVRHI_FAILED(mBuilder->createTearingClothFromMesh(
                &b.positions[0].x, numParticles, numParticles + uint32_t(g.numExtraParticles),
                reinterpret_cast<const int32_t*>(mesh->m_indices.data()), mesh->GetNumFaces(), params, &mCloth)))
            return;

        b.triangles.assign(mesh->m_indices.begin(), mesh->m_indices.end());
        b.triangleNormals.assign(mesh->GetNumFaces(), float3(0.0f, 0.0f, 1.0f));

        CopySprings(g);

        g.params.radius = radius;
        g.params.dynamicFriction = 0.025f;
        g.params.dissipation = 0.0f;
        g.params.numIterations = 16;
        g.params.particleCollisionMargin = g.params.radius * 0.05f;
        g.params.relaxationFactor = 1.0f;
        g.params.drag = 0.03f;

        g.params.relaxationMode = RelaxationMode::Global;
        g.params.relaxationFactor = 0.35f;

        g.numSubsteps = 2;

        g.pause = false;

        // draw options
        g.drawPoints = false;
    }

    void Update(DemoState& g) override {
        g.params.wind[0] = 0.1f;
        g.params.wind[1] = 0.1f;
        g.params.wind[2] = -0.2f;
        g.windStrength = 6.0f;

        if (!mCloth)
            return;

        // the tear reads the particles the solver left and writes new ones
        g.MapBuffers();
        SimBuffers& b = g.buffers;

        const float maxStrain = 3.0f;
        const uint32_t maxCopies = 2048;
        const uint32_t maxEdits = 2048;

        TearingParticleClone particleCopies[maxCopies];
        uint32_t numParticleCopies = 0;

        TearingMeshEdit triangleEdits[maxEdits];
        uint32_t numTriangleEdits = 0;

        // update asset's copy of the particles
        mCloth->setParticles(&b.positions[0].x, 0, std::min(uint32_t(b.positions.size()), mCloth->getMaxParticles()));

        mCloth->tearClothMesh(maxStrain, 4, particleCopies, maxCopies, numParticleCopies, triangleEdits, maxEdits,
                              numTriangleEdits);

        // copy particles
        for (uint32_t i = 0; i < numParticleCopies; ++i) {
            const size_t srcIndex = size_t(particleCopies[i].srcIndex);
            const size_t destIndex = size_t(particleCopies[i].destIndex);

            // override mass because picked particle has inf. mass
            b.positions[destIndex] = float4(b.positions[srcIndex].x, b.positions[srcIndex].y, b.positions[srcIndex].z, 1.0f);
            b.restPositions[destIndex] = b.restPositions[srcIndex];
            b.velocities[destIndex] = b.velocities[srcIndex];
            b.phases[destIndex] = b.phases[srcIndex];

            b.activeIndices.push_back(int(destIndex));
        }

        // apply triangle modifications to index buffer
        for (uint32_t i = 0; i < numTriangleEdits; ++i) {
            const size_t index = size_t(triangleEdits[i].triIndex);
            const int newValue = triangleEdits[i].newParticleIndex;

            b.triangles[index] = newValue;
        }

        mTorn = numParticleCopies > 0 || numTriangleEdits > 0;
        if (mTorn)
            mClones += numParticleCopies;

        // update constraints
        CopySprings(g);
    }

    void Sync(DemoState& g) override {
        // update solver data not already updated in the main loop
        g.SetSprings();
        g.SetDynamicTriangles();
        g.SetRestParticles();
    }

    void DoGui(DemoState& g) override {
        (void)g;
        ImGui::Text("Clones: %u", mClones);
    }

 private:
    void CopySprings(DemoState& g) {
        SimBuffers& b = g.buffers;
        const uint32_t n = mCloth->getSpringCount();
        b.springIndices.assign(mCloth->getSpringIndices(), mCloth->getSpringIndices() + n * 2);
        b.springStiffness.assign(mCloth->getSpringCoefficients(), mCloth->getSpringCoefficients() + n);
        b.springLengths.assign(mCloth->getSpringRestLengths(), mCloth->getSpringRestLengths() + n);
    }

    PhysAssetBuilderHandle mBuilder;  // the cloth draws from its random generator
    PhysTearingClothHandle mCloth;
    bool mTorn = false;
    uint32_t mClones = 0;
};

#endif /* FLEX_DEMO_SCENES_TEARING_H */
