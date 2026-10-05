#ifndef FLEX_DEMO_SCENES_XPBD_H
#define FLEX_DEMO_SCENES_XPBD_H

// The four XPBD reference scenes of FleX's feat-xpbd branch (Macklin, Mueller, Chentanez, "XPBD: Position-Based
// Simulation of Compliant Constrained Dynamics", MIG 2016): reference/flex/demo/scenes/xpbdchain.h, xpbdbeam.h,
// xpbdballoon.h and xpbdcloth.h. The long notes of FleX's files on how each scene was matched to the paper and its
// video, and the numbers measured there, are in those files and are not repeated here.
//
// FleX's g_params carried the formulation and the compliance block (solverMode, stiffnessMin, stiffnessMax,
// springDamping, volumeCompliance); here the formulation is the solver's (DemoState::solverMode, SolverDesc::mode)
// and the block is IPhysSolverXPBD's (DemoState::xpbd). The panel's Solver choice restarts a scene with the other
// formulation (FleX switched in place); --solver pbd|xpbd does the same from the command line.

#include "inflatable.h"
#include "scene_common.h"

// XPBD Chain (Sec. 6.2): twenty unit-mass particles joined by distance constraints of compliance 1e-8 m/N, hanging
// from a fixed particle and released horizontally.
class XPBDChain : public Scene {
 public:
    explicit XPBDChain(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const int numParticles = 20;
        const float segment = 0.1f;
        const float3 anchor = float3(0.0f, 2.5f, 0.0f);
        const int phase = makePhase(0, 0);

        const int baseIndex = int(b.positions.size());

        for (int i = 0; i < numParticles; ++i) {
            // particle 0 is the fixed support, the rest start level with it
            const float invMass = (i == 0) ? 0.0f : 1.0f;
            b.positions.push_back(float4(anchor.x + float(i) * segment, anchor.y, anchor.z, invMass));
            b.velocities.push_back(float3(0.0f));
            b.phases.push_back(phase);
        }

        for (int i = 0; i < numParticles - 1; ++i)
            CreateSpring(b, baseIndex + i, baseIndex + i + 1, 1.0f);

        g.params.radius = 0.05f;
        g.params.dynamicFriction = 0.0f;
        g.params.dissipation = 0.0f;
        g.params.damping = 0.0f;
        g.params.drag = 0.0f;
        g.params.relaxationFactor = 1.0f;
        g.params.maxAcceleration = FLT_MAX;  // the released chain snaps taut; do not clamp it

        g.solverMode = SolverMode::XPBD;
        g.xpbd.stiffnessMin = 1.0e8f;
        g.xpbd.stiffnessMax = 1.0e8f;
        g.xpbd.springDamping = 0.0f;
        g.params.numIterations = 50;

        g.numSubsteps = 2;

        // draw options
        g.drawPoints = true;
        g.drawSprings = 1;  // 1: draw stretch springs

        g.windStrength = 0.0f;
    }
};

// XPBD Cantilever Beam (Sec. 6.3): a 40 x 8 lattice of unit-mass particles joined by stretch, shear and bending
// springs, clamped along its left edge to a wall and bending under gravity with XPBD compliance.
class XPBDCantileverBeam : public Scene {
 public:
    explicit XPBDCantileverBeam(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const int dimx = 40;  // along the beam
        const int dimy = 8;   // through its thickness
        const float spacing = 0.05f;
        const float radius = spacing;

        // [0,1] coefficients map onto [stiffnessMin, stiffnessMax] = [1e3, 1e9] N/m geometrically; 0.54 is ~1.7e6 N/m
        const float stretchStiffness = 0.54f;
        const float shearStiffness = 0.54f;
        const float bendStiffness = 0.54f;
        const int phase = makePhase(0, 0);

        const int baseIndex = int(b.positions.size());
        CreateSpringGrid(b, float3(0.0f, 0.0f, 0.0f), dimx, dimy, 1, spacing, phase, stretchStiffness, bendStiffness,
                         shearStiffness, float3(0.0f), 1.0f);

        // CreateSpringGrid lays the grid in the x-z plane (row index along z); stand it up so the beam runs along +x
        // with its thickness along y
        const float thickness = float(dimy - 1) * spacing;
        const float wallX = 0.5f;
        const float top = 2.0f;

        for (int i = baseIndex; i < int(b.positions.size()); ++i) {
            float4& p = b.positions[size_t(i)];
            const int row = (i - baseIndex) / dimx;
            const int col = (i - baseIndex) % dimx;
            p = float4(wallX + float(col) * spacing, top - float(row) * spacing, 0.0f, p.w);
            b.velocities[size_t(i)] = float3(0.0f);

            // clamp the whole left edge
            if (col == 0)
                p.w = 0.0f;
        }

        // the wall the beam is clamped to (decoration; the clamp is the pinned edge)
        AddBox(b, float3(0.2f, 0.6f, 0.4f), float3(wallX - 0.2f - radius, top - thickness * 0.5f, 0.0f));

        g.params.radius = radius;
        g.params.dynamicFriction = 0.25f;
        g.params.dissipation = 0.0f;
        g.params.drag = 0.0f;
        g.params.damping = 0.5f;  // light viscous drag so the beam settles
        g.params.relaxationFactor = 1.0f;

        g.solverMode = SolverMode::XPBD;
        g.xpbd.stiffnessMin = 1.0e3f;
        g.xpbd.stiffnessMax = 1.0e9f;
        g.xpbd.springDamping = 0.0f;
        g.params.numIterations = 20;

        g.numSubsteps = 16;

        // draw options
        g.drawPoints = true;
        g.drawSprings = 1;  // 1: draw stretch springs

        g.windStrength = 0.0f;
    }
};

// XPBD Inflatable Balloon (Fig. 1): four balloons on the floor, closed cloth surfaces with a volume constraint
// holding the rest volume, the surface stiffness rising from left to right.
class XPBDInflatableBalloon : public Scene {
 public:
    explicit XPBDInflatableBalloon(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        mTriCounts.clear();

        // softest to stiffest, left to right: on [10, 1e6] N/m these are 316, 1.8e3, 1e4 and 1e5 N/m
        const float stiffness[4] = {0.35f, 0.5f, 0.65f, 0.85f};
        const float spacing = 2.0f;

        PhysAssetBuilderHandle builder;
        if (!g.phys || NVRHI_FAILED(g.phys->createAssetBuilder(&builder)))
            return;

        for (int i = 0; i < 4; ++i) {
            std::unique_ptr<Mesh> mesh = utils::ImportMesh(g.AssetPath("sphere.ply"));
            if (!mesh)
                continue;
            mesh->Normalize();  // unit diameter, lower corner at the origin
            mesh->Transform(dm::translation(float3((float(i) - 1.5f) * spacing - 0.5f, 0.6f, -0.5f)));

            // AddBalloon: stretch and bending springs from the surface triangles, both with the balloon's [0,1]
            // coefficient; the rest volume, no over-pressure
            if (AddInflatableMesh(g, builder, mesh.get(), stiffness[i], stiffness[i], 1.0f, makePhase(i, 0)))
                mTriCounts.push_back(int(mesh->GetNumFaces()));
        }

        g.params.radius = 0.1f;
        g.params.dynamicFriction = 0.4f;
        g.params.dissipation = 0.0f;
        g.params.damping = 0.5f;  // light viscous drag so the balloons settle
        g.params.particleCollisionMargin = g.params.radius * 0.05f;
        g.params.drag = 0.0f;
        g.params.collisionDistance = 0.01f;
        g.params.relaxationFactor = 1.0f;

        // unit-mass particles: 10 N/m flattens, 1e6 N/m is a hard shell
        g.solverMode = SolverMode::XPBD;
        g.xpbd.stiffnessMin = 10.0f;
        g.xpbd.stiffnessMax = 1.0e6f;
        g.xpbd.springDamping = 0.0f;
        g.xpbd.volumeCompliance = 0.0f;
        g.params.numIterations = 20;

        // four substeps keep the stiffest balloon inside the regime where 20 Jacobi iterations converge
        g.numSubsteps = 4;

        g.windStrength = 0.0f;

        // draw options
        g.drawPoints = false;
        g.drawSprings = 0;
        g.drawCloth = false;
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
            g.context->DrawCloth(cloth, 3, g.params.radius * 0.35f);

            indexStart += uint32_t(mTriCounts[i]);
        }
    }

 private:
    std::vector<int> mTriCounts;
};

// XPBD Hanging Cloth (Fig. 6): a 64 x 64 cloth (4096 particles, 23938 constraints) hangs by its two top corners
// below a bar in a gusting wind; with XPBD the drape is the same at 20, 40, 80 and 160 iterations.
class XPBDHangingCloth : public Scene {
 public:
    explicit XPBDHangingCloth(const char* name) : Scene(name) {}

    void Initialize(DemoState& g) override {
        SimBuffers& b = g.buffers;
        const int dimx = 64;
        const int dimy = 64;
        const float spacing = kSpacing;
        const float radius = spacing;

        // a single sheet that never folds onto itself: no self-collision
        const int phase = makePhase(0, 0);

        const int baseIndex = int(b.positions.size());
        CreateSpringGrid(b, float3(0.0f, 0.0f, 0.0f), dimx, dimy, 1, spacing, phase, kXpbdStretch, kXpbdBend, kXpbdShear,
                         float3(0.0f), 1.0f);

        // stand the grid up with row 0 on top at the pin height read off the video
        const float width = float(dimx - 1) * spacing;
        const float height = float(dimy - 1) * spacing;
        const float top = kPinHeight;
        mWidth = width;
        mHeight = height;
        mTop = top;

        for (int i = baseIndex; i < int(b.positions.size()); ++i) {
            float4& p = b.positions[size_t(i)];
            const int row = (i - baseIndex) / dimx;
            const int col = (i - baseIndex) % dimx;

            // a few millimetres of out-of-plane ripple, faded in from the pinned top row, to pick a buckling mode
            const float u = float(col) / float(dimx - 1);
            const float v = float(row) / float(dimy - 1);
            const float ripple = kSeedRipple * spacing * v * (sinf(11.0f * u) + 0.5f * sinf(7.0f * u + 2.0f));

            p = float4(float(col) * spacing - width * 0.5f, top - float(row) * spacing, ripple, p.w);
            b.velocities[size_t(i)] = float3(0.0f);
        }

        // the two top corners hang at the cloth's full rest width
        b.positions[size_t(baseIndex)].w = 0.0f;
        b.positions[size_t(baseIndex + dimx - 1)].w = 0.0f;

        // the bar: AddCapsule's half-height excludes the end caps
        AddCapsule(b, kBarRadius, 0.5f * (kBarLength * width - 2.0f * kBarRadius),
                   float3(0.0f, top + 1.1f * kBarRadius, 0.5f * kBarRadius), QuatIdentity());

        g.params.radius = radius;
        g.params.dynamicFriction = 0.25f;
        g.params.dissipation = 0.0f;

        g.params.drag = kDrag;
        g.params.lift = kLift;
        g.params.damping = kDamping;

        // XPBD's near-rigid constraints need the count-averaged (local) update
        g.params.relaxationMode = RelaxationMode::Local;
        g.params.relaxationFactor = 1.0f;

        g.solverMode = SolverMode::XPBD;
        g.xpbd.stiffnessMin = 1.0e3f;
        g.xpbd.stiffnessMax = 1.0e9f;
        g.xpbd.springDamping = 0.0f;
        g.params.numIterations = 20;

        g.numSubsteps = kXpbdSubsteps;

        // draw options: the cloth's two colours measured off the video (FleX's Init restores both)
        g.colors[3] = kClothFace;
        g.colors[4] = kClothBack;
        g.drawPoints = false;
        g.drawSprings = 0;

        g.windStrength = kWindStrength;
    }

    void Update(DemoState& g) override {
        // horizontal, across the sheet, with a little sideways drift; the gust is signed
        const float3 kWindDir = float3(kWindDrift, 0.0f, kWindDepth);
        const float gust = kGustBias + kGustAmp * Perlin1D(g.windTime * kGustRate, 4, 0.5f);
        const float3 wind = g.windStrength * gust * kWindDir;

        g.params.wind[0] = wind.x;
        g.params.wind[1] = wind.y;
        g.params.wind[2] = wind.z;
    }

    void CenterCamera(DemoState& g) override {
        // the video's framing: 0.132 W below the pins and 2.503 W in front of the cloth plane, pitch -15 degrees
        g.camPos = float3(0.0f, mTop - kCamDrop * mWidth, kCamDistance * mWidth);
        g.camAngle = float3(0.0f, -DegToRad(15.0f), 0.0f);
    }

    static constexpr float kSpacing = 0.05f;
    static constexpr float kPinHeight = 4.90f;
    static constexpr float kCamDrop = 0.132f;
    static constexpr float kCamDistance = 2.503f;
    static constexpr float kBarLength = 1.456f;
    static constexpr float kBarRadius = 0.063f;
    static inline const float4 kClothFace = float4(0.080f, 0.581f, 0.389f, 1.0f);
    static inline const float4 kClothBack = float4(0.642f, 0.631f, 0.631f, 1.0f);
    static constexpr float kXpbdStretch = 0.452f;
    static constexpr float kXpbdShear = 0.25f;
    static constexpr float kXpbdBend = 0.05f;
    static constexpr int kXpbdSubsteps = 16;
    static constexpr float kSeedRipple = 0.05f;
    static constexpr float kGustRate = 0.7f;
    static constexpr float kWindStrength = 3.0f;
    static constexpr float kGustBias = 0.0f;
    static constexpr float kGustAmp = 1.4f;
    static constexpr float kDrag = 1.0f;
    static constexpr float kLift = 0.2f;
    static constexpr float kDamping = 0.0f;
    static constexpr float kWindDrift = 0.3f;
    static constexpr float kWindDepth = -2.0f;

    float mWidth = 0.0f, mHeight = 0.0f, mTop = 0.0f;
};

#endif /* FLEX_DEMO_SCENES_XPBD_H */
