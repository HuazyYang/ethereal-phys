#ifndef FLEX_DEMO_SCENES_SOFT_BODY_H
#define FLEX_DEMO_SCENES_SOFT_BODY_H

// SoftBody and SoftBodyFixed: FleX/demo/scenes/softbody.h. Soft and plastic bodies made of overlapping
// shape-matching clusters (IPhysAssetBuilder::createSoftFromMesh, FleX's NvFlexExtCreateSoftFromMesh), drawn as
// their meshes skinned to the clusters on the host (createSoftMeshSkinning). The scenes of FleX's registry are
// instances of these classes configured in main.cpp (soft_body_scenes.h here).
//
// What differs from FleX:
// - The springs of an asset (links, mLinkRadius > 0) are offset by the asset's first particle; FleX pushed the
//   asset's indices as they were, which ties every instance after the first to the particles of the first. No
//   scene of the registry has two linked instances.
// - Update maps the particle arrays (DemoState::MapBuffers) when the meshes are drawn: FleX's Draw read the rigid
//   transforms of buffers it mapped every frame; here they are downloaded when something asks.

#include "scene_common.h"

class SoftBody : public Scene {
 public:
    explicit SoftBody(const char* name) : Scene(name) {
        const float3 colorPicker[7] = {float3(0.0f, 0.5f, 1.0f),     float3(0.797f, 0.354f, 0.000f),
                                       float3(0.000f, 0.349f, 0.173f), float3(0.875f, 0.782f, 0.051f),
                                       float3(0.01f, 0.170f, 0.453f),  float3(0.673f, 0.111f, 0.000f),
                                       float3(0.612f, 0.194f, 0.394f)};
        for (int i = 0; i < 7; ++i)
            mColorPicker[i] = colorPicker[i];
    }

    float mRadius = 0.1f;
    float mRelaxationFactor = 1.0f;
    bool mPlinth = false;

    float3 mColorPicker[7];

    struct Instance {
        explicit Instance(const char* mesh) : mFile(mesh) {}

        const char* mFile;
        float3 mColor = float3(0.5f, 0.5f, 1.0f);

        float3 mScale = float3(2.0f);
        float3 mTranslation = float3(0.0f, 1.0f, 0.0f);

        float mClusterSpacing = 1.0f;
        float mClusterRadius = 0.0f;
        float mClusterStiffness = 0.5f;

        float mLinkRadius = 0.0f;
        float mLinkStiffness = 1.0f;

        float mGlobalStiffness = 0.0f;

        float mSurfaceSampling = 0.0f;
        float mVolumeSampling = 4.0f;

        float mSkinningFalloff = 2.0f;
        float mSkinningMaxDistance = 100.0f;

        float mClusterPlasticThreshold = 0.0f;
        float mClusterPlasticCreep = 0.0f;
    };

    std::vector<Instance> mInstances;

    virtual void AddInstance(const Instance& instance) { mInstances.push_back(instance); }

    virtual void AddStack(Instance instance, int xStack, int yStack, int zStack, bool rotateColors = false) {
        const float3 translation = instance.mTranslation;
        for (int x = 0; x < xStack; ++x) {
            for (int y = 0; y < yStack; ++y) {
                for (int z = 0; z < zStack; ++z) {
                    instance.mTranslation =
                        translation + float3(float(x) * (instance.mScale.x + 1), float(y) * (instance.mScale.y + 1),
                                             float(z) * (instance.mScale.z + 1)) *
                                          mRadius;
                    if (rotateColors)
                        instance.mColor = mColorPicker[(x * yStack * zStack + y * zStack + z) % 7];
                    mInstances.push_back(instance);
                }
            }
        }
    }

    void Initialize(DemoState& g) override {
        const float radius = mRadius;

        // no fluids or sdf based collision
        g.featureMode = FeatureMode::SimpleSolids;

        g.params.radius = radius;
        g.params.dynamicFriction = 0.35f;
        g.params.particleFriction = 0.25f;
        g.params.numIterations = 4;
        g.params.collisionDistance = radius * 0.75f;

        g.params.relaxationFactor = mRelaxationFactor;

        g.windStrength = 0.0f;

        g.numSubsteps = 2;

        // draw options
        g.drawPoints = false;
        g.wireframe = false;
        g.drawSprings = 0;
        g.drawBases = false;

        g.buffers.rigidOffsets.push_back(0);

        mRenderingInstances.clear();
        mPlasticDeformation = false;

        // FleX's extension library: one builder, whose random generator createSoftFromMesh seeds
        PhysAssetBuilderHandle builder;
        if (!g.phys || NVRHI_FAILED(g.phys->createAssetBuilder(&builder)))
            return;

        // build soft bodies
        for (int i = 0; i < int(mInstances.size()); i++)
            CreateSoftBody(g, builder, mInstances[size_t(i)], int(mRenderingInstances.size()));

        if (mPlinth)
            AddPlinth(g.buffers);

        // fix any particles below the ground plane in place
        for (float4& p : g.buffers.positions)
            if (p.y < 0.0f)
                p.w = 0.0f;

        // expand radius for better self collision
        g.params.radius *= 1.5f;

        g.lightDistance *= 1.5f;
    }

    void Update(DemoState& g) override {
        // Draw reads the rigid transforms of this frame
        if (g.drawMesh)
            g.MapBuffers();
    }

    void Draw(DemoState& g, int pass) override {
        (void)pass;
        if (!g.drawMesh)
            return;

        const SimBuffers& b = g.buffers;
        if (b.rigidTranslations.empty())
            return;

        for (const RenderingInstance& instance : mRenderingInstances) {
            Mesh m;
            m.m_positions.resize(instance.mMesh->m_positions.size());
            m.m_normals.resize(instance.mMesh->m_normals.size());
            m.m_indices = instance.mMesh->m_indices;

            for (size_t i = 0; i < instance.mMesh->m_positions.size(); ++i) {
                float3 softPos(0.0f);
                float3 softNormal(0.0f);

                for (int w = 0; w < 4; ++w) {
                    const int cluster = instance.mSkinningIndices[i * 4 + size_t(w)];
                    const float weight = instance.mSkinningWeights[i * 4 + size_t(w)];

                    if (cluster > -1) {
                        // offset in the global constraint array
                        const size_t rigidIndex = size_t(cluster + instance.mOffset);

                        const float3 localPos = instance.mMesh->m_positions[i] - instance.mRigidRestPoses[size_t(cluster)];

                        const float3 skinnedPos =
                            b.rigidTranslations[rigidIndex] + QuatRotate(b.rigidRotations[rigidIndex], localPos);
                        const float3 skinnedNormal = QuatRotate(b.rigidRotations[rigidIndex], instance.mMesh->m_normals[i]);

                        softPos += skinnedPos * weight;
                        softNormal += skinnedNormal * weight;
                    }
                }

                m.m_positions[i] = softPos;
                if (i < m.m_normals.size())
                    m.m_normals[i] = softNormal;
            }

            g.context->DrawMesh(&m, instance.mColor);
        }
    }

 protected:
    struct RenderingInstance {
        std::unique_ptr<Mesh> mMesh;
        std::vector<int> mSkinningIndices;
        std::vector<float> mSkinningWeights;
        std::vector<float3> mRigidRestPoses;
        float3 mColor = float3(0.0f);
        int mOffset = 0;
    };

    std::vector<RenderingInstance> mRenderingInstances;

    bool mPlasticDeformation = false;

    void CreateSoftBody(DemoState& g, IPhysAssetBuilder* builder, const Instance& instance, int group = 0) {
        SimBuffers& b = g.buffers;
        RenderingInstance renderingInstance;

        std::unique_ptr<Mesh> mesh = utils::ImportMesh(g.AssetPath(instance.mFile));
        if (!mesh)
            return;
        mesh->Normalize();
        // FleX: TranslationMatrix(mTranslation) * ScaleMatrix(mScale * mRadius)
        mesh->Transform(dm::scaling(instance.mScale * mRadius) * dm::translation(instance.mTranslation));

        renderingInstance.mMesh = std::move(mesh);
        renderingInstance.mColor = instance.mColor;
        renderingInstance.mOffset = int(b.rigidTranslations.size());

        const Mesh& m = *renderingInstance.mMesh;

        // create soft body definition
        SoftAssetParams params;
        params.particleSpacing = mRadius;
        params.volumeSampling = instance.mVolumeSampling;
        params.surfaceSampling = instance.mSurfaceSampling;
        params.clusterSpacing = instance.mClusterSpacing * mRadius;
        params.clusterRadius = instance.mClusterRadius * mRadius;
        params.clusterStiffness = instance.mClusterStiffness;
        params.linkRadius = instance.mLinkRadius * mRadius;
        params.linkStiffness = instance.mLinkStiffness;
        params.globalStiffness = instance.mGlobalStiffness;
        params.clusterPlasticThreshold = instance.mClusterPlasticThreshold;
        params.clusterPlasticCreep = instance.mClusterPlasticCreep;

        PhysAssetHandle asset;
        if (NVRHI_FAILED(builder->createSoftFromMesh(&m.m_positions[0].x, m.GetNumVertices(),
                                                     reinterpret_cast<const int32_t*>(m.m_indices.data()),
                                                     uint32_t(m.m_indices.size()), params, &asset)))
            return;

        // create skinning
        const int maxWeights = 4;

        renderingInstance.mSkinningIndices.resize(m.m_positions.size() * maxWeights);
        renderingInstance.mSkinningWeights.resize(m.m_positions.size() * maxWeights);

        const float* centers = asset->getShapeCenters();
        const int numShapes = int(asset->getShapeCount());
        for (int i = 0; i < numShapes; ++i)
            renderingInstance.mRigidRestPoses.push_back(float3(centers[i * 3 + 0], centers[i * 3 + 1], centers[i * 3 + 2]));

        builder->createSoftMeshSkinning(&m.m_positions[0].x, m.GetNumVertices(), centers, uint32_t(numShapes),
                                        instance.mSkinningFalloff, instance.mSkinningMaxDistance,
                                        renderingInstance.mSkinningWeights.data(),
                                        renderingInstance.mSkinningIndices.data());

        const int particleOffset = int(b.positions.size());
        const int indexOffset = b.rigidOffsets.back();

        // add particle data to solver
        const float* particles = asset->getParticles();
        for (uint32_t i = 0; i < asset->getParticleCount(); ++i) {
            b.positions.push_back(float4(particles[i * 4 + 0], particles[i * 4 + 1], particles[i * 4 + 2], particles[i * 4 + 3]));
            b.velocities.push_back(float3(0.0f));

            const int phase = makePhase(group, PhaseSelfCollide | PhaseSelfCollideFilter);
            b.phases.push_back(phase);
        }

        // add shape data to solver
        const int32_t* shapeIndices = asset->getShapeIndices();
        for (uint32_t i = 0; i < asset->getShapeIndexCount(); ++i)
            b.rigidIndices.push_back(shapeIndices[i] + particleOffset);

        const int32_t* shapeOffsets = asset->getShapeOffsets();
        const float* coefficients = asset->getShapeCoefficients();
        for (int i = 0; i < numShapes; ++i) {
            b.rigidOffsets.push_back(shapeOffsets[i] + indexOffset);
            b.rigidTranslations.push_back(float3(centers[i * 3 + 0], centers[i * 3 + 1], centers[i * 3 + 2]));
            b.rigidRotations.push_back(QuatIdentity());
            b.rigidCoefficients.push_back(coefficients[i]);
        }

        // add plastic deformation data to solver, if at least one asset has non-zero plastic deformation
        // coefficients; leave the arrays empty otherwise
        const float* thresholds = asset->getShapePlasticThresholds();
        const float* creeps = asset->getShapePlasticCreeps();
        if (mPlasticDeformation) {
            for (int i = 0; i < numShapes; ++i) {
                b.rigidPlasticThresholds.push_back(thresholds && creeps ? thresholds[i] : 0.0f);
                b.rigidPlasticCreeps.push_back(thresholds && creeps ? creeps[i] : 0.0f);
            }
        } else if (thresholds && creeps) {
            const size_t oldBufferSize = b.rigidCoefficients.size() - size_t(numShapes);

            b.rigidPlasticThresholds.assign(oldBufferSize, 0.0f);
            b.rigidPlasticCreeps.assign(oldBufferSize, 0.0f);

            for (int i = 0; i < numShapes; ++i) {
                b.rigidPlasticThresholds.push_back(thresholds[i]);
                b.rigidPlasticCreeps.push_back(creeps[i]);
            }

            mPlasticDeformation = true;
        }

        // add link data to the solver (offset by the asset's first particle, see the top of this file)
        const int32_t* springIndices = asset->getSpringIndices();
        const float* springCoefficients = asset->getSpringCoefficients();
        const float* springRestLengths = asset->getSpringRestLengths();
        for (uint32_t i = 0; i < asset->getSpringCount(); ++i) {
            b.springIndices.push_back(springIndices[i * 2 + 0] + particleOffset);
            b.springIndices.push_back(springIndices[i * 2 + 1] + particleOffset);

            b.springStiffness.push_back(springCoefficients[i]);
            b.springLengths.push_back(springRestLengths[i]);
        }

        std::printf("Created soft body %s: %u particles, %d clusters, %u links\n", instance.mFile,
                    asset->getParticleCount(), numShapes, asset->getSpringCount());

        mRenderingInstances.push_back(std::move(renderingInstance));
    }
};

class SoftBodyFixed : public SoftBody {
 public:
    explicit SoftBodyFixed(const char* name) : SoftBody(name) {}

    void Initialize(DemoState& g) override {
        SoftBody::Initialize(g);

        // fix any particles in the wall
        for (float4& p : g.buffers.positions)
            if (p.x < mRadius)
                p.w = 0.0f;
    }

    virtual void AddStack(Instance instance, int zStack) {
        const float clusterStiffness = instance.mClusterStiffness;
        const float3 translation = instance.mTranslation;

        for (int z = 0; z < zStack; ++z) {
            const float s = clusterStiffness * float(z + 1);
            instance.mClusterStiffness = s * s;
            instance.mTranslation = translation + float3(0.0f, 0.0f, -float(z) * (instance.mScale.z + 1)) * mRadius;
            mInstances.push_back(instance);
        }
    }

    void PostInitialize(DemoState& g) override {
        SoftBody::PostInitialize(g);

        g.params.planes[1][0] = 1.0f;
        g.params.planes[1][1] = 0.0f;
        g.params.planes[1][2] = 0.0f;
        g.params.planes[1][3] = 0.0f;
        g.params.numPlanes = 2;
    }
};

#endif /* FLEX_DEMO_SCENES_SOFT_BODY_H */
