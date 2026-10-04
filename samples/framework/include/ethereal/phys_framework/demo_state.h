#ifndef ETHEREAL_PHYS_FRAMEWORK_DEMO_STATE_H
#define ETHEREAL_PHYS_FRAMEWORK_DEMO_STATE_H

// DemoState: the state the FleX demo kept in globals (reference/flex/demo/main.cpp: g_params, g_buffers,
// g_numSubsteps, the draw options, the camera, ...), in one object that the framework owns and hands to the
// scenes. A scene ports with `g_x` -> `g.x`: Initialize(DemoState& g) sets g.params, g.buffers, g.drawPoints as
// the demo's did. The defaults are set by the framework before every Initialize, to the values of FleX's Init().
//
// Part of ethereal_phys_framework (samples/framework).

#include <ethereal/phys/params.h>
#include <ethereal/phys_helpers/sim_buffers.h>

#include <donut/core/math/math.h>

#include <filesystem>
#include <functional>
#include <vector>

namespace ephys::framework {

class DemoContext;

// A particle emitter (FleX's Emitter); emitters[0] is the camera's "gun".
struct Emitter {
    dm::float3 mPos = dm::float3(0.f);
    dm::float3 mDir = dm::float3(0.f);
    dm::float3 mRight = dm::float3(0.f);
    float mSpeed = 0.0f;
    bool mEnabled = false;
    float mLeftOver = 0.0f;
    int mWidth = 8;
};

struct DemoState {
    // ---- what a scene authors ----
    helpers::SimBuffers buffers;  // g_buffers (and g_mesh, g_ropes, the skinning arrays)
    PhysParams params;            // g_params, without solverMode and the XPBD block:
    SolverMode solverMode = SolverMode::PBD;    // ... which are the solver's (SolverDesc::mode)
    XpbdComplianceParams xpbd;                  // ... and IPhysSolverXPBD's

    int numSubsteps = 2;
    float dt = 1.0f / 60.0f;  // the time step of the simulation: fixed, never the wall clock
    int frame = 0;            // simulation steps since the scene started

    int maxDiffuseParticles = 0;
    int maxNeighborsPerParticle = 96;
    int numExtraParticles = 0;
    int numExtraMultiplier = 1;
    int maxContactsPerParticle = 6;
    int numSolidParticles = 0;

    dm::float3 sceneLower = dm::float3(0.f);
    dm::float3 sceneUpper = dm::float3(0.f);

    bool warmup = false;

    // wind and waves
    float windTime = 0.0f;
    float windFrequency = 0.1f;
    float windStrength = 0.0f;
    bool wavePool = false;
    float waveTime = 0.0f;
    float wavePlane = 0.0f;
    float waveFrequency = 1.5f;
    float waveAmplitude = 1.0f;
    float waveFloorTilt = 0.0f;

    bool emit = false;
    std::vector<Emitter> emitters = std::vector<Emitter>(1);

    // ---- drawing ----
    float blur = 1.0f;
    float ior = 1.0f;
    bool drawEllipsoids = false;
    bool drawPoints = true;
    bool drawMesh = true;
    bool drawCloth = true;
    float expandCloth = 0.0f;
    bool drawOpaque = false;
    int drawSprings = 0;  // 0: none, 1: stretch, 2: tethers
    bool drawBases = false;
    bool drawContacts = false;
    bool drawNormals = false;
    bool drawDiffuse = false;
    bool drawShapeGrid = false;
    bool drawDensity = false;
    bool drawRopes = true;
    float pointScale = 1.0f;
    float ropeScale = 1.0f;
    float drawPlaneBias = 0.0f;
    bool wireframe = false;

    float diffuseScale = 0.5f;
    float diffuseMotionScale = 1.0f;
    bool diffuseShadow = false;
    float diffuseInscatter = 0.8f;
    float diffuseOutscatter = 0.53f;

    dm::float4 colors[8] = {};  // g_colors (FleX's Colour, alpha 1)
    dm::float4 fluidColor = dm::float4(0.1f, 0.4f, 0.8f, 1.0f);
    dm::float4 diffuseColor = dm::float4(1.0f);
    dm::float3 meshColor = dm::float3(0.9f);
    dm::float3 clearColor = dm::float3(0.0f);
    float lightDistance = 2.0f;
    float fogDistance = 0.005f;

    // ---- the camera (FleX's: position, yaw / pitch, velocity) ----
    dm::float3 camPos = dm::float3(6.0f, 8.0f, 18.0f);
    dm::float3 camAngle = dm::float3(0.0f, -0.349066f, 0.0f);  // x: yaw, y: pitch (radians)
    dm::float3 camVel = dm::float3(0.f);
    dm::float3 camSmoothVel = dm::float3(0.f);
    float camSpeed = 0.075f;
    float camNear = 0.01f;
    float camFar = 1000.0f;

    // ---- control ----
    bool pause = false;
    bool step = false;

    // ---- services of the framework ----

    // FleX's "../../data/<name>": the sample's asset directory.
    [[nodiscard]] std::filesystem::path AssetPath(const char* name) const { return assetDir / name; }

    // Brings the particle arrays of `buffers` (positions, velocities, normals, the rigid rotations and
    // translations) up to date with the GPU, waiting for it. FleX's Update() always ran on mapped buffers; here a
    // scene calls this before it reads or writes them, and the framework sends what the host holds back to the
    // solver before the next step (positions, velocities, phases and the active list, as FleX's frame did).
    void MapBuffers() {
        if (mapBuffers)
            mapBuffers();
    }

    // The renderer, for a scene's Draw.
    DemoContext* context = nullptr;

    std::filesystem::path assetDir;
    std::function<void()> mapBuffers;  // set by the framework
};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_DEMO_STATE_H */
