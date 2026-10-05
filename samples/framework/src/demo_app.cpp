#include "demo_app_impl.h"

#include <ethereal/phys_helpers/helpers.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/perlin.h>

#include <donut/app/ApplicationBase.h>
#include <donut/app/ImGuiRenderPass.h>
#include <donut/core/log.h>
#include <nvrhi/core/foundation.h>
#include <nvrhi/utils.h>

#include <GLFW/glfw3.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

// The FleX demo's main.cpp (reference/flex/demo/main.cpp): Init, UpdateFrame, RenderScene, RenderDebug,
// DrawShapes, the camera, the input and the picking, decomposed onto DemoContext (the renderer), PhysicsScene
// (the engine and Donut's scene graph) and the ImGui panel (demo_ui.cpp).

namespace ephys::framework {

using namespace donut;
namespace helpers = ephys::helpers;

namespace {

// FleX's g_colors at start-up (main.cpp).
const dm::float4 c_DefaultColors[8] = {
    dm::float4(0.0f, 0.5f, 1.0f, 1.0f),       dm::float4(0.797f, 0.354f, 0.000f, 1.0f),
    dm::float4(0.092f, 0.465f, 0.820f, 1.0f), dm::float4(0.000f, 0.349f, 0.173f, 1.0f),
    dm::float4(0.875f, 0.782f, 0.051f, 1.0f), dm::float4(0.000f, 0.170f, 0.453f, 1.0f),
    dm::float4(0.673f, 0.111f, 0.000f, 1.0f), dm::float4(0.612f, 0.194f, 0.394f, 1.0f)};

dm::float3 xyz(const dm::float4& v) { return dm::float3(v.x, v.y, v.z); }

dm::float3 lerp(const dm::float3& a, const dm::float3& b, float t) { return a + (b - a) * t; }

// FleX's T * R * S (column vectors) as Donut's row-vector affine transform.
dm::affine3 shapeTransform(const dm::float3& position, const dm::float4& rotation, const dm::float3& scale) {
    dm::float3 cols[3];
    helpers::QuatToMatrix33(rotation, cols);
    const dm::affine3 r(dm::float3x3(cols[0], cols[1], cols[2]), dm::float3(0.f));
    return dm::scaling(scale) * r * dm::translation(position);
}

}  // namespace

DemoApp::DemoApp(app::DeviceManager* deviceManager, const DemoOptions& options, DemoAppDesc desc,
                 ValidationMonitor* monitor)
    : IRenderPass(deviceManager),
      m_DeviceManager(deviceManager),
      m_Options(options),
      m_Desc(std::move(desc)),
      m_Monitor(monitor) {
    for (int i = 0; i < 8; ++i)
        g.colors[i] = c_DefaultColors[i];
}

DemoApp::~DemoApp() {
    if (GetDevice())
        GetDevice()->waitForIdle();
    m_Physics.reset();
    m_Context.reset();
    if (m_Phys)
        m_Phys->waitForIdle();
    m_Phys = nullptr;
}

bool DemoApp::Init() {
    nvrhi::IDevice* device = GetDevice();

    // ---- file system: Donut's shaders, the framework's shaders, the engine's kernels ----
    const std::filesystem::path exeDir = app::GetDirectoryWithExecutable();
    const char* shaderType = app::GetShaderTypeName(device->getGraphicsAPI());
    m_FileSystem = MAKE_RC_OBJ_PTR(vfs::RootFileSystem);
    m_FileSystem->mount("shaders/donut", exeDir / "shaders" / "framework" / shaderType);
    m_FileSystem->mount("shaders/phys_framework", exeDir / "shaders" / "phys_framework" / shaderType);
    m_ShaderFactory = MAKE_RC_OBJ_PTR(engine::ShaderFactory, device, m_FileSystem, "shaders");
    m_CommonPasses = MAKE_RC_OBJ_PTR(engine::CommonRenderPasses, device, m_ShaderFactory);

    m_Context = std::make_unique<DemoContext>(device, m_ShaderFactory, m_CommonPasses, m_Options.msaa);
    if (!m_Context->Init()) {
        log::error("the renderer could not be created (are the shaders of shaders/phys_framework built?)");
        return false;
    }
    int width = 0, height = 0;
    m_DeviceManager->GetWindowDimensions(width, height);
    m_Context->Resize(uint32_t(width), uint32_t(height));
    g.context = m_Context.get();
    g.assetDir = m_Desc.assetDir;
    g.mapBuffers = [this]() { MapBuffers(); };
    g.clothBuffers = &m_ClothBuffers;

    // ---- the engine ----
    struct PhysCallback final : public nvrhi::ObjectImpl<IPhysMessageCallback> {
        NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysCallback)
        NVRHI_IMPLEMENTS_INTERFACE(IPhysMessageCallback)
        NVRHI_END_INTERFACE_TABLE()
        explicit PhysCallback(ValidationMonitor* m) : monitor(m) {}
        void message(PhysMessageSeverity severity, const char* text) noexcept override {
            monitor->OnPhysMessage(severity, text);
        }
        ValidationMonitor* monitor;
    };
    auto callback = MAKE_RC_OBJ_PTR(PhysCallback, m_Monitor);
    const std::string kernels = (exeDir / "shaders" / "ethereal_phys").generic_u8string();
    PhysDeviceDesc physDesc;
    physDesc.device = device;
    physDesc.messageCallback = callback;
    physDesc.shaderPath = kernels.c_str();
    m_Phys = createDevice(physDesc);
    if (!m_Phys) {
        log::error("cannot create the physics device (the header versions of the sample and ethereal_phys.dll "
                   "must match)");
        return false;
    }
    m_DeviceName = m_DeviceManager->GetRendererString();
    g.phys = m_Phys;

    device->createCommandList(nvrhi::CommandListParameters(), &m_CommandList);

    // ---- the first scene ----
    m_Scene = 0;
    if (!m_Options.scene.empty()) {
        int found = -1;
        for (int i = 0; i < int(m_Desc.scenes.size()); ++i)
            if (_stricmp(m_Desc.scenes[i]->GetName(), m_Options.scene.c_str()) == 0)
                found = i;
        if (found < 0 && !m_Options.scene.empty() && std::isdigit((unsigned char)m_Options.scene[0]))
            found = std::atoi(m_Options.scene.c_str());
        if (found < 0 || found >= int(m_Desc.scenes.size())) {
            std::printf("Invalid scene name: %s\nAvailable scenes: {\n", m_Options.scene.c_str());
            for (const auto& s : m_Desc.scenes)
                std::printf("    %s\n", s->GetName());
            std::printf("}\n");
            return false;
        }
        m_Scene = found;
    }
    if (m_Options.smokeFrames >= 0) {
        m_SmokeScene = 0;
        m_Scene = 0;
    }
    if (!m_Options.screenshot.empty() || m_Options.smokeFrames >= 0) {
        // a comparison shot wants the render alone (FleX's --screenshot)
        m_ShowHelp = false;
        m_TweakPanel = false;
    }
    m_SelectedScene = m_Scene;
    return InitScene(m_Scene);
}

// ---- the scene lifecycle --------------------------------------------------------------------------------------

void DemoApp::ResetState() {
    // FleX's Init(): what it resets before a scene initialises (and nothing else: windFrequency, wavePool, the
    // camera when it is not centred, the clear colour and colours 1, 2, 5, 6, 7 persist, as there)
    g.emitters.assign(1, Emitter());
    g.emitters[0].mEnabled = false;
    g.emitters[0].mSpeed = 1.0f;
    g.emitters[0].mLeftOver = 0.0f;
    g.emitters[0].mWidth = 8;

    g.frame = 0;
    g.pause = false;

    g.dt = 1.0f / 60.0f;
    g.waveTime = 0.0f;
    g.windTime = 0.0f;
    g.windStrength = 1.0f;

    g.blur = 1.0f;
    g.fluidColor = dm::float4(0.1f, 0.4f, 0.8f, 1.0f);
    g.meshColor = dm::float3(0.9f, 0.9f, 0.9f);
    g.drawEllipsoids = false;
    g.drawPoints = true;
    g.drawCloth = true;
    g.expandCloth = 0.0f;

    g.drawOpaque = false;
    g.drawSprings = 0;
    g.drawDiffuse = false;
    g.drawMesh = true;
    g.drawRopes = true;
    g.drawDensity = false;
    g.ior = 1.0f;
    g.lightDistance = 2.0f;
    g.fogDistance = 0.005f;

    g.camSpeed = 0.075f;
    g.camNear = 0.01f;
    g.camFar = 1000.0f;

    g.pointScale = 1.0f;
    g.ropeScale = 1.0f;
    g.drawPlaneBias = 0.0f;

    // the simulation parameters (FleX's g_params values; solverMode and the XPBD block are the solver's)
    PhysParams& p = g.params;
    p = PhysParams();
    p.gravity[0] = 0.0f;
    p.gravity[1] = -9.8f;
    p.gravity[2] = 0.0f;

    p.wind[0] = 0.0f;
    p.wind[1] = 0.0f;
    p.wind[2] = 0.0f;

    p.radius = 0.15f;
    p.viscosity = 0.0f;
    p.dynamicFriction = 0.0f;
    p.staticFriction = 0.0f;
    p.particleFriction = 0.0f;  // scale friction between particles by default
    p.freeSurfaceDrag = 0.0f;
    p.drag = 0.0f;
    p.lift = 0.0f;
    p.numIterations = 3;
    p.fluidRestDistance = 0.0f;
    p.solidRestDistance = 0.0f;

    p.anisotropyScale = 1.0f;
    p.anisotropyMin = 0.1f;
    p.anisotropyMax = 2.0f;
    p.smoothing = 1.0f;

    p.dissipation = 0.0f;
    p.damping = 0.0f;
    p.particleCollisionMargin = 0.0f;
    p.shapeCollisionMargin = 0.0f;
    p.collisionDistance = 0.0f;
    p.sleepThreshold = 0.0f;
    p.shockPropagation = 0.0f;
    p.restitution = 0.0f;

    p.maxSpeed = FLT_MAX;
    p.maxAcceleration = 100.0f;  // approximately 10x gravity

    p.relaxationMode = RelaxationMode::Local;
    p.relaxationFactor = 1.0f;
    g.solverMode = SolverMode::PBD;
    g.featureMode = FeatureMode::Default;  // FleX's NvFlexSetSolverDescDefaults
    g.baselineReductions = m_Options.baselineReductions;
    g.xpbd = XpbdComplianceParams();
    g.xpbd.stiffnessMin = 1.0e3f;
    g.xpbd.stiffnessMax = 1.0e9f;
    g.xpbd.springDamping = 0.0f;
    g.xpbd.volumeCompliance = 0.0f;
    p.solidPressure = 1.0f;
    p.adhesion = 0.0f;
    p.cohesion = 0.025f;
    p.surfaceTension = 0.0f;
    p.vorticityConfinement = 0.0f;
    p.buoyancy = 1.0f;
    p.diffuseThreshold = 100.0f;
    p.diffuseBuoyancy = 1.0f;
    p.diffuseDrag = 0.8f;
    p.diffuseBallistic = 16;
    p.diffuseLifetime = 2.0f;

    g.numSubsteps = 2;

    // planes created after the particles
    p.numPlanes = 1;

    g.diffuseScale = 0.5f;
    g.diffuseColor = dm::float4(1.0f);
    g.diffuseMotionScale = 1.0f;
    g.diffuseShadow = false;
    g.diffuseInscatter = 0.8f;
    g.diffuseOutscatter = 0.53f;

    // the phase 0 colour back to blue, and the two colours DrawCloth reads (FleX resets those three)
    g.colors[0] = dm::float4(0.0f, 0.5f, 1.0f, 1.0f);
    g.colors[3] = dm::float4(0.000f, 0.349f, 0.173f, 1.0f);
    g.colors[4] = dm::float4(0.875f, 0.782f, 0.051f, 1.0f);

    g.numSolidParticles = 0;

    g.waveFrequency = 1.5f;
    g.waveAmplitude = 1.5f;
    g.waveFloorTilt = 0.0f;
    g.emit = false;
    g.warmup = false;

    m_MouseParticle = -1;

    g.maxDiffuseParticles = 0;  // number of diffuse particles
    g.maxNeighborsPerParticle = 96;
    g.numExtraParticles = 0;  // particles allocated but not made active
    g.maxContactsPerParticle = 6;

    g.sceneLower = dm::float3(FLT_MAX);
    g.sceneUpper = dm::float3(-FLT_MAX);

    g.forceFields.clear();
    g.syncRequests = 0;
}

bool DemoApp::InitScene(int scene, bool centerCamera) {
    helpers::RandInit();

    // the previous scene goes first: its GPU objects are released after the GPU finished with them
    m_Physics.reset();
    GetDevice()->waitForIdle();
    GetDevice()->runGarbageCollection();

    g.buffers = helpers::SimBuffers();
    ResetState();

    Scene* s = m_Desc.scenes[size_t(scene)].get();
    s->Initialize(g);

    // FleX's command-line overrides, applied after the scene's Initialize as in its Init
    if (m_Options.solverMode >= 0)
        g.solverMode = m_Options.solverMode == 1 ? SolverMode::XPBD : SolverMode::PBD;
    if (m_Options.emit)
        g.emit = true;

    if (m_Options.drawFluid) {
        // FleX's 'e' and 'v' keys
        g.drawEllipsoids = true;
        g.drawPoints = false;
    }

    helpers::SimBuffers& b = g.buffers;
    const uint32_t numParticles = uint32_t(b.positions.size());
    const uint32_t maxParticles =
        std::max(1u, numParticles + uint32_t(g.numExtraParticles * g.numExtraMultiplier));

    PhysParams& p = g.params;
    if (p.solidRestDistance == 0.0f)
        p.solidRestDistance = p.radius;

    // with fluid, solid particles are assumed to have the same radius
    if (p.fluidRestDistance > 0.0f)
        p.solidRestDistance = p.fluidRestDistance;

    // the collision distance from the rest distance, when not set
    if (p.collisionDistance == 0.0f)
        p.collisionDistance = std::max(p.solidRestDistance, p.fluidRestDistance) * 0.5f;

    // particle friction at 10% of the shape friction by default
    if (p.particleFriction == 0.0f)
        p.particleFriction = p.dynamicFriction * 0.1f;

    // a margin for detecting contacts between particles and shapes
    if (p.shapeCollisionMargin == 0.0f)
        p.shapeCollisionMargin = p.collisionDistance * 0.5f;

    // the bounds of the particles and of the shapes
    dm::float3 particleLower, particleUpper;
    helpers::GetParticleBounds(b, particleLower, particleUpper);
    dm::float3 shapeLower, shapeUpper;
    helpers::GetShapeBounds(b, shapeLower, shapeUpper);

    g.sceneLower = dm::min(dm::min(g.sceneLower, particleLower), shapeLower);
    g.sceneUpper = dm::max(dm::max(g.sceneUpper, particleUpper), shapeUpper);

    g.sceneLower -= dm::float3(p.collisionDistance);
    g.sceneUpper += dm::float3(p.collisionDistance);

    // the collision planes around the scene
    const dm::float3 up = dm::normalize(dm::float3(-g.waveFloorTilt, 1.0f, 0.0f));
    auto setPlane = [&](int i, float a, float bb, float c, float d) {
        p.planes[i][0] = a;
        p.planes[i][1] = bb;
        p.planes[i][2] = c;
        p.planes[i][3] = d;
    };
    setPlane(0, up.x, up.y, up.z, 0.0f);
    setPlane(1, 0.0f, 0.0f, 1.0f, -g.sceneLower.z);
    setPlane(2, 1.0f, 0.0f, 0.0f, -g.sceneLower.x);
    setPlane(3, -1.0f, 0.0f, 0.0f, g.sceneUpper.x);
    setPlane(4, 0.0f, 0.0f, -1.0f, g.sceneUpper.z);
    setPlane(5, 0.0f, -1.0f, 0.0f, g.sceneUpper.y);

    g.wavePlane = p.planes[2][3];

    // normals for drawing before the simulation starts: the triangle normals summed at their particles
    b.normals.assign(maxParticles, dm::float4(0.0f));
    const size_t numTris = b.triangles.size() / 3;
    for (size_t i = 0; i < numTris; ++i) {
        const dm::float3 v0 = xyz(b.positions[size_t(b.triangles[i * 3 + 0])]);
        const dm::float3 v1 = xyz(b.positions[size_t(b.triangles[i * 3 + 1])]);
        const dm::float3 v2 = xyz(b.positions[size_t(b.triangles[i * 3 + 2])]);

        const dm::float3 n = dm::cross(v1 - v0, v2 - v0);

        for (int k = 0; k < 3; ++k)
            b.normals[size_t(b.triangles[i * 3 + k])] += dm::float4(n, 0.0f);
    }
    for (uint32_t i = 0; i < maxParticles; ++i) {
        const dm::float3 n = xyz(b.normals[i]);
        const float l = dm::dot(n, n);
        b.normals[i] = dm::float4(l > 0.0f ? n * (1.0f / sqrtf(l)) : dm::float3(0.0f, 1.0f, 0.0f), 0.0f);
    }

    // the mesh positions for skinning
    if (b.mesh)
        b.meshRestPositions = b.mesh->m_positions;
    else
        b.meshRestPositions.clear();

    // the active particles (a contiguous block, as in the demo)
    b.activeIndices.resize(numParticles);
    for (uint32_t i = 0; i < numParticles; ++i)
        b.activeIndices[i] = int(i);

    // the particle arrays to the capacity
    b.positions.resize(maxParticles, dm::float4(0.0f));
    b.velocities.resize(maxParticles, dm::float3(0.0f));
    b.phases.resize(maxParticles, 0);

    // the rest positions
    b.restPositions = b.positions;

    // the rigid constraints
    if (!b.rigidOffsets.empty()) {
        const int numRigids = int(b.rigidOffsets.size()) - 1;

        // the centres of mass, when the scene did not compute them
        if (b.rigidTranslations.empty()) {
            b.rigidTranslations.resize(size_t(numRigids), dm::float3(0.0f));
            helpers::CalculateRigidCentersOfMass(b.positions.data(), int(b.positions.size()), b.rigidOffsets.data(),
                                                 b.rigidTranslations.data(), b.rigidIndices.data(), numRigids);
        }

        // the rest-space positions
        b.rigidLocalPositions.resize(size_t(b.rigidOffsets.back()));
        helpers::CalculateRigidLocalPositions(b.positions.data(), b.rigidOffsets.data(), b.rigidTranslations.data(),
                                              b.rigidIndices.data(), numRigids, b.rigidLocalPositions.data());

        b.rigidRotations.resize(size_t(numRigids), dm::float4(0.0f, 0.0f, 0.0f, 1.0f));
    }

    // ---- the engine: the domain, the scene graph, the solver, the render buffers ----
    m_Physics = std::make_unique<PhysicsScene>(GetDevice(), m_Phys, m_Context.get(),
                                               [this](PhysMessageSeverity severity, const char* text) {
                                                   m_Monitor->OnPhysMessage(severity, text);
                                               });
    if (!m_Physics->Create(g, maxParticles, numParticles)) {
        log::error("scene %s: the physics scene could not be created", s->GetName());
        m_Physics.reset();
        return false;
    }

    // the scene's post-solver initialisation
    s->PostInitialize(g);

    // the camera on the particles, after PostInitialize as in FleX's Init (a scene may grow the bounds there)
    if (centerCamera) {
        g.camPos = dm::float3((g.sceneLower.x + g.sceneUpper.x) * 0.5f, std::min(g.sceneUpper.y * 1.25f, 6.0f),
                              g.sceneUpper.z + std::min(g.sceneUpper.y, 6.0f) * 2.0f);
        g.camAngle = dm::float3(0.0f, -DegToRad(15.0f), 0.0f);

        // the scene may move it
        s->CenterCamera(g);
    }

    // the warm-up of the simulation
    if (g.warmup) {
        std::printf("Warming up sim..\n");
        m_Physics->Warmup(g);
        std::printf("Finished warm up.\n");
    }

    std::printf("scene %d: %s, %u particles (%u allocated, %d solid), %zu springs, %zu rigids, %zu triangles, "
                "%zu inflatables, %zu shapes, %s\n",
                scene, s->GetName(), numParticles, maxParticles, g.numSolidParticles, b.springLengths.size(),
                b.rigidOffsets.empty() ? size_t(0) : b.rigidOffsets.size() - 1, b.triangles.size() / 3,
                b.inflatableTriOffsets.size(), b.shapes.size(), g.solverMode == SolverMode::XPBD ? "XPBD" : "PBD");
    std::fflush(stdout);
    m_Scene = scene;
    return true;
}

// ---- the frame ------------------------------------------------------------------------------------------------

// The host mirrors of the particles, waited for (FleX's MapBuffers). A frame that mapped them sends them back
// before it steps, whether it wrote them or only read them, as FleX's frame did.
void DemoApp::MapBuffers() {
    if (m_Physics)
        m_Physics->MapBuffers(g);
}

void DemoApp::UpdateCamera() {
    const dm::float3 forward(-sinf(g.camAngle.x) * cosf(g.camAngle.y), sinf(g.camAngle.y),
                             -cosf(g.camAngle.x) * cosf(g.camAngle.y));
    const dm::float3 right(dm::normalize(dm::cross(forward, dm::float3(0.0f, 1.0f, 0.0f))));

    g.camSmoothVel = lerp(g.camSmoothVel, g.camVel, 0.1f);
    g.camPos += (forward * g.camSmoothVel.z + right * g.camSmoothVel.x + dm::cross(right, forward) * g.camSmoothVel.y);
}

void DemoApp::UpdateEmitters() {
    const float spin = DegToRad(15.0f);

    const dm::float3 forward(-sinf(g.camAngle.x + spin) * cosf(g.camAngle.y), sinf(g.camAngle.y),
                             -cosf(g.camAngle.x + spin) * cosf(g.camAngle.y));
    const dm::float3 right(dm::normalize(dm::cross(forward, dm::float3(0.0f, 1.0f, 0.0f))));

    g.emitters[0].mDir = dm::normalize(forward + dm::float3(0.0f, 0.4f, 0.0f));
    g.emitters[0].mRight = right;
    g.emitters[0].mPos = g.camPos + forward * 1.f + dm::float3(0.0f, 0.2f, 0.0f) + right * 0.65f;

    if (!g.emit)
        return;

    // the particle arrays are written: map them (FleX's were mapped)
    MapBuffers();
    helpers::SimBuffers& b = g.buffers;
    int activeCount = int(b.activeIndices.size());

    size_t e = 0;

    // skip the camera emitter when moving forward, or things get messy
    if (g.camSmoothVel.z >= 0.025f)
        e = 1;

    for (; e < g.emitters.size(); ++e) {
        Emitter& emitter = g.emitters[e];
        if (!emitter.mEnabled)
            continue;

        const dm::float3 emitterDir = emitter.mDir;
        const dm::float3 emitterRight = emitter.mRight;
        const dm::float3 emitterPos = emitter.mPos;

        const float r = g.params.fluidRestDistance;
        const int phase = makePhase(0, PhaseSelfCollide | PhaseFluid);

        const float numParticles = (emitter.mSpeed / r) * g.dt;

        // the whole number to emit
        const int n = int(numParticles + emitter.mLeftOver);

        if (n)
            emitter.mLeftOver = (numParticles + emitter.mLeftOver) - float(n);
        else
            emitter.mLeftOver += numParticles;

        // a disc of particles, n layers thick
        for (int k = 0; k < n; ++k) {
            const int emitterWidth = emitter.mWidth;
            const int numDisc = emitterWidth * emitterWidth;
            for (int i = 0; i < numDisc; ++i) {
                const float x = float(i % emitterWidth) - float(emitterWidth / 2);
                const float y = float((i / emitterWidth) % emitterWidth) - float(emitterWidth / 2);

                if ((x * x + y * y) <= float((emitterWidth / 2) * (emitterWidth / 2))) {
                    const dm::float3 upv = dm::normalize(dm::cross(emitterDir, emitterRight));
                    const dm::float3 offset = r * (emitterRight * x + upv * y) + float(k) * emitterDir * r;

                    if (activeCount < int(b.positions.size())) {
                        b.positions[size_t(activeCount)] = dm::float4(emitterPos + offset, 1.0f);
                        b.velocities[size_t(activeCount)] = emitterDir * emitter.mSpeed;
                        b.phases[size_t(activeCount)] = phase;

                        b.activeIndices.push_back(activeCount);

                        activeCount++;
                    }
                }
            }
        }
    }
}

void DemoApp::UpdateMouse() {
    // the button is up: release the particle
    if (m_LastButton == -1 && m_MouseParticle != -1) {
        MapBuffers();
        // restore its mass
        g.buffers.positions[size_t(m_MouseParticle)].w = m_MouseMass;
        m_MouseParticle = -1;
    }

    // the button went down: pick a particle
    if (m_MousePicked) {
        MapBuffers();
        dm::float3 origin, dir;
        m_Context->GetViewRay(m_LastX, m_LastY, origin, dir);

        const int numActive = int(m_Physics->GetActiveCount());

        m_MouseParticle = helpers::PickParticle(origin, dir, g.buffers.positions.data(), g.buffers.phases.data(),
                                                numActive, g.params.radius * 0.8f, m_MouseT);

        if (m_MouseParticle != -1) {
            const dm::float4& p = g.buffers.positions[size_t(m_MouseParticle)];
            const dm::float3& v = g.buffers.velocities[size_t(m_MouseParticle)];
            std::printf("picked: %d, mass: %f v: %f %f %f\n", m_MouseParticle, p.w, v.x, v.y, v.z);

            m_MousePos = origin + dir * m_MouseT;
            m_MouseMass = p.w;
            // an infinite mass drags the particle towards the point
            g.buffers.positions[size_t(m_MouseParticle)].w = 0.0f;
        }

        m_MousePicked = false;
    }

    // the picked particle follows the mouse
    if (m_MouseParticle != -1) {
        MapBuffers();
        dm::float4& particle = g.buffers.positions[size_t(m_MouseParticle)];
        const dm::float3 p = lerp(xyz(particle), m_MousePos, 0.8f);
        const dm::float3 delta = p - xyz(particle);

        particle.x = p.x;
        particle.y = p.y;
        particle.z = p.z;

        g.buffers.velocities[size_t(m_MouseParticle)] = delta / g.dt;
    }
}

void DemoApp::UpdateWind() {
    g.windTime += g.dt;

    const dm::float3 kWindDir = dm::float3(3.0f, 15.0f, 0.0f);
    const float kNoise = utils::Perlin1D(g.windTime * g.windFrequency, 10, 0.25f);
    const dm::float3 wind = g.windStrength * kWindDir * dm::float3(kNoise, fabsf(kNoise), 0.0f);

    g.params.wind[0] = wind.x;
    g.params.wind[1] = wind.y;
    g.params.wind[2] = wind.z;

    if (g.wavePool) {
        g.waveTime += g.dt;

        g.params.planes[2][3] =
            g.wavePlane + (sinf(g.waveTime * g.waveFrequency - kPi * 0.5f) * 0.5f + 0.5f) * g.waveAmplitude;
    }
}

void DemoApp::DrawShapes() {
    DemoContext& ctx = *m_Context;
    const std::vector<helpers::ShapeDesc>& shapes = g.buffers.shapes;
    for (size_t i = 0; i < shapes.size(); ++i) {
        const helpers::ShapeDesc& shape = shapes[i];

        dm::float3 color = dm::float3(0.9f);
        if (shape.trigger) {
            color = dm::float3(0.6f, 1.0f, 0.6f);
            ctx.SetFillMode(true);
        }

        // drawn with the previous transform, to match the particles' update order
        const dm::float4 rotation = shape.prevRotation;
        const dm::float3 position = shape.prevPosition;

        switch (shape.type) {
            case CollisionShapeType::Sphere: {
                std::unique_ptr<utils::Mesh> sphere = utils::CreateSphere(20, 20, shape.radius);
                sphere->Transform(shapeTransform(position, rotation, dm::float3(1.0f)));
                ctx.DrawMesh(sphere.get(), color);
                break;
            }
            case CollisionShapeType::Capsule: {
                std::unique_ptr<utils::Mesh> capsule = utils::CreateCapsule(10, 20, shape.radius, shape.halfHeight);
                // FleX: T * R * RotationMatrix(-90 degrees, z)
                capsule->Transform(dm::rotation(dm::float3(0.0f, 0.0f, 1.0f), DegToRad(-90.0f)) *
                                   shapeTransform(position, rotation, dm::float3(1.0f)));
                ctx.DrawMesh(capsule.get(), color);
                break;
            }
            case CollisionShapeType::Box: {
                std::unique_ptr<utils::Mesh> box = utils::CreateCubeMesh();
                box->Transform(shapeTransform(position, rotation, shape.halfExtents * 2.0f));
                ctx.DrawMesh(box.get(), color);
                break;
            }
            case CollisionShapeType::ConvexMesh: {
                if (GpuMesh* mesh = m_Physics->GetShapeMesh(i)) {
                    // FleX draws convexes with the current transform
                    const Matrix44 xform = TranslationMatrix(shape.position) * RotationMatrix(shape.rotation) *
                                           ScaleMatrix(shape.scale);
                    ctx.DrawGpuMesh(mesh, xform, color);
                }
                break;
            }
            case CollisionShapeType::TriangleMesh: {
                if (GpuMesh* mesh = m_Physics->GetShapeMesh(i)) {
                    const Matrix44 xform =
                        TranslationMatrix(position) * RotationMatrix(rotation) * ScaleMatrix(shape.scale);
                    ctx.DrawGpuMesh(mesh, xform, color);
                }
                break;
            }
            case CollisionShapeType::Sdf: {
                if (GpuMesh* mesh = m_Physics->GetShapeMesh(i)) {
                    const Matrix44 xform = TranslationMatrix(position) * RotationMatrix(rotation) *
                                           ScaleMatrix(dm::float3(shape.sdfScale));
                    ctx.DrawGpuMesh(mesh, xform, color);
                }
                break;
            }
            default:
                break;
        }
    }

    ctx.SetFillMode(g.wireframe);
}

void DemoApp::RenderScene() {
    DemoContext& ctx = *m_Context;
    const int numParticles = int(m_Physics->GetActiveCount());

    // ---- the view ----
    const float fov = kPi / 4.0f;
    const float screenWidth = float(ctx.GetWidth());
    const float aspect = float(ctx.GetWidth()) / float(std::max(1u, ctx.GetHeight()));

    const Matrix44 proj = ProjectionMatrix(RadToDeg(fov), aspect, g.camNear, g.camFar);
    const Matrix44 view = RotationMatrix(-g.camAngle.x, dm::float3(0.0f, 1.0f, 0.0f)) *
                          RotationMatrix(-g.camAngle.y, dm::float3(cosf(-g.camAngle.x), 0.0f, sinf(-g.camAngle.x))) *
                          TranslationMatrix(-g.camPos);

    // ---- the light ----

    // the scene bounds grown to fit most scenes
    g.sceneLower = dm::min(g.sceneLower, dm::float3(-2.0f, 0.0f, -2.0f));
    g.sceneUpper = dm::max(g.sceneUpper, dm::float3(2.0f, 2.0f, 2.0f));

    const dm::float3 sceneExtents = g.sceneUpper - g.sceneLower;
    const dm::float3 sceneCenter = 0.5f * (g.sceneUpper + g.sceneLower);

    const dm::float3 lightDir = dm::normalize(dm::float3(5.0f, 15.0f, 7.5f));
    const dm::float3 lightPos = sceneCenter + lightDir * dm::length(sceneExtents) * g.lightDistance;
    const dm::float3 lightTarget = sceneCenter;

    // tight bounds for the shadow frustum, scaled and clamped for aesthetics
    float lightFov = 2.0f * atanf(dm::length(g.sceneUpper - sceneCenter) / dm::length(lightPos - sceneCenter));
    lightFov = std::clamp(lightFov, DegToRad(25.0f), DegToRad(65.0f));

    const Matrix44 lightPerspective = ProjectionMatrix(RadToDeg(lightFov), 1.0f, 1.0f, 1000.0f);
    const Matrix44 lightView = LookAtMatrix(lightPos, lightTarget);
    const Matrix44 lightTransform = lightPerspective * lightView;

    // the radius points are drawn with
    const float radius = std::max(g.params.solidRestDistance, g.params.fluidRestDistance) * 0.5f * g.pointScale;

    helpers::SimBuffers& b = g.buffers;
    const FluidRenderBuffers fluid = m_Physics->GetFluidBuffers(g.drawEllipsoids, g.drawDensity);
    const ClothRenderBuffers cloth = m_Physics->GetClothBuffers();
    m_ClothBuffers = cloth;
    const bool hasActive = !b.activeIndices.empty();

    // ---- the shadow pass ----
    ctx.ShadowBegin();

    ctx.SetView(lightView, lightPerspective);
    ctx.SetCullMode(false);

    // the scene's own drawing
    m_Desc.scenes[size_t(m_Scene)]->Draw(g, 1);

    if (g.drawMesh)
        ctx.DrawMesh(b.mesh.get(), g.meshColor);

    DrawShapes();

    if (g.drawCloth && !b.triangles.empty())
        ctx.DrawCloth(cloth, 3, g.expandCloth);

    if (g.drawRopes) {
        for (size_t i = 0; i < b.ropes.size(); ++i)
            ctx.DrawRope(b.positions.data(), b.ropes[i].mIndices.data(), int(b.ropes[i].mIndices.size()),
                         radius * g.ropeScale, int(i));
    }

    int shadowParticles = numParticles;
    int shadowParticlesOffset = 0;

    if (!g.drawPoints) {
        shadowParticles = 0;

        if (g.drawEllipsoids) {
            shadowParticles = numParticles - g.numSolidParticles;
            shadowParticlesOffset = g.numSolidParticles;
        }
    } else {
        const int offset = g.drawMesh ? g.numSolidParticles : 0;

        shadowParticles = numParticles - offset;
        shadowParticlesOffset = offset;
    }

    if (hasActive)
        ctx.DrawPoints(fluid, shadowParticles, shadowParticlesOffset, radius, 2048, 1.0f, lightFov, lightPos,
                       lightTarget, lightTransform, g.drawDensity);

    ctx.ShadowEnd();

    // ---- the lighting pass ----
    ctx.BindSolidShader(lightPos, lightTarget, lightTransform, 0.0f, dm::float4(g.clearColor, g.fogDistance));

    ctx.SetView(view, proj);
    ctx.SetCullMode(true);

    ctx.DrawPlanes(reinterpret_cast<const dm::float4*>(g.params.planes), g.params.numPlanes, g.drawPlaneBias);

    if (g.drawMesh)
        ctx.DrawMesh(b.mesh.get(), g.meshColor);

    DrawShapes();

    if (g.drawCloth && !b.triangles.empty())
        ctx.DrawCloth(cloth, 3, g.expandCloth);

    if (g.drawRopes) {
        for (size_t i = 0; i < b.ropes.size(); ++i)
            ctx.DrawRope(b.positions.data(), b.ropes[i].mIndices.data(), int(b.ropes[i].mIndices.size()),
                         g.params.radius * 0.5f * g.ropeScale, int(i));
    }

    // the scene's own drawing
    m_Desc.scenes[size_t(m_Scene)]->Draw(g, 0);

    ctx.UnbindSolidShader();

    const DiffuseRenderBuffers diffuse = m_Physics->GetDiffuseBuffers();

    // the diffuse particles behind the fluid surface
    if (g.drawDiffuse)
        ctx.DrawDiffuse(diffuse, radius * g.diffuseScale, screenWidth, aspect, fov, g.diffuseColor, lightPos,
                        lightTarget, lightTransform, g.diffuseMotionScale, g.diffuseInscatter, g.diffuseOutscatter,
                        g.diffuseShadow, false);

    if (g.drawEllipsoids) {
        // the solid particles separately
        if (g.numSolidParticles && g.drawPoints)
            ctx.DrawPoints(fluid, g.numSolidParticles, 0, radius, screenWidth, aspect, fov, lightPos, lightTarget,
                           lightTransform, g.drawDensity);

        // the fluid surface
        ctx.RenderEllipsoids(fluid, numParticles - g.numSolidParticles, g.numSolidParticles, radius, screenWidth,
                             aspect, fov, lightPos, lightTarget, lightTransform, g.fluidColor, g.blur, g.ior,
                             g.drawOpaque);

        // the diffuse particles in front of the fluid surface
        if (g.drawDiffuse)
            ctx.DrawDiffuse(diffuse, radius * g.diffuseScale, screenWidth, aspect, fov, g.diffuseColor, lightPos,
                            lightTarget, lightTransform, g.diffuseMotionScale, g.diffuseInscatter,
                            g.diffuseOutscatter, g.diffuseShadow, true);
    } else {
        // every particle as a sphere
        if (g.drawPoints) {
            const int offset = g.drawMesh ? g.numSolidParticles : 0;

            if (hasActive)
                ctx.DrawPoints(fluid, numParticles - offset, offset, radius, screenWidth, aspect, fov, lightPos,
                               lightTarget, lightTransform, g.drawDensity);
        }
    }
}

void DemoApp::RenderDebug() {
    DemoContext& ctx = *m_Context;
    helpers::SimBuffers& b = g.buffers;

    if (m_MouseParticle != -1) {
        // the mouse spring
        ctx.BeginLines();
        ctx.DrawLine(m_MousePos, xyz(b.positions[size_t(m_MouseParticle)]), dm::float4(1.0f));
        ctx.EndLines();
    }

    // springs
    if (g.drawSprings) {
        dm::float4 color(0.0f);

        if (g.drawSprings == 1)
            color = dm::float4(0.0f, 0.0f, 1.0f, 0.8f);  // stretch
        if (g.drawSprings == 2)
            color = dm::float4(0.0f, 1.0f, 0.0f, 0.8f);  // tether

        ctx.BeginLines();

        for (size_t i = 0; i < b.springLengths.size(); ++i) {
            if (g.drawSprings == 1 && b.springStiffness[i] < 0.0f)
                continue;
            if (g.drawSprings == 2 && b.springStiffness[i] > 0.0f)
                continue;

            const int a = b.springIndices[i * 2];
            const int c = b.springIndices[i * 2 + 1];

            ctx.DrawLine(xyz(b.positions[size_t(a)]), xyz(b.positions[size_t(c)]), color);
        }

        ctx.EndLines();
    }

    if (g.drawBases) {
        for (size_t i = 0; i < b.rigidRotations.size(); ++i) {
            ctx.BeginLines();

            const float size = 0.1f;
            dm::float3 frame[3];
            helpers::QuatToMatrix33(b.rigidRotations[i], frame);

            for (int k = 0; k < 3; ++k) {
                dm::float3 color(0.0f);
                color[k] = 1.0f;

                ctx.DrawLine(b.rigidTranslations[i], b.rigidTranslations[i] + frame[k] * size,
                             dm::float4(color, 0.0f));
            }

            ctx.EndLines();
        }
    }

    if (g.drawNormals) {
        ctx.BeginLines();

        for (size_t i = 0; i < b.normals.size(); ++i) {
            ctx.DrawLine(xyz(b.positions[i]), xyz(b.positions[i]) - xyz(b.normals[i]) * b.normals[i].w,
                         dm::float4(0.0f, 1.0f, 0.0f, 0.0f));
        }

        ctx.EndLines();
    }
}

void DemoApp::Render(nvrhi::IFramebuffer* framebuffer) {
    if (m_Exiting || !m_Physics)
        return;
    nvrhi::IDevice* device = GetDevice();
    ++m_FrameIndex;

    Scene* scene = m_Desc.scenes[size_t(m_Scene)].get();

    // an unattended pick (--pick): the mouse state a shift + left click and a drag would give
    if (m_Options.pickFrame >= 0 && g.frame >= m_Options.pickFrame) {
        if (m_LastButton == -1) {
            m_LastX = m_Options.pickX;
            m_LastY = m_Options.pickY;
            m_LastButton = GLFW_MOUSE_BUTTON_LEFT;
            m_MousePicked = true;
        } else {
            m_LastY -= m_Options.pickDrag;
        }
    }

    // ---- the scene update (FleX: the map, then camera, emitters, mouse, wind, scene) ----
    UpdateCamera();

    if (!g.pause || g.step) {
        UpdateEmitters();
        UpdateMouse();
        UpdateWind();
        scene->Update(g);
    }

    // the GPU timers of the solver (FleX's g_profile)
    if (IPhysSolver* solver = m_Physics->GetSolver())
        solver->setTimersEnabled(m_Profile);
    if (m_Profile && m_Physics->GetTimers()) {
        m_Physics->GetTimers()->poll();
        const float latency = m_Physics->GetTimers()->getDeviceLatency() * 1000.0f;
        m_SimLatency = m_SimLatency == 0.0f ? latency : m_SimLatency + (latency - m_SimLatency) * 0.05f;
    }

    // what the frame draws from host memory: the skinned mesh and the debug lines
    helpers::SimBuffers& b = g.buffers;
    const bool skin = !b.meshSkinIndices.empty();
    const bool hostDebug = m_MouseParticle != -1 || g.drawSprings || g.drawBases || g.drawNormals ||
                           (g.drawRopes && !b.ropes.empty());
    if (skin || hostDebug)
        MapBuffers();
    if (skin)
        helpers::SkinMesh(b);

    // ---- render ----
    m_CommandList->open();
    m_Physics->BeginFrame(m_CommandList, g);

    m_Context->StartFrame(m_CommandList, dm::float4(g.clearColor, 1.0f));
    m_Context->SetColors(g.colors);
    RenderScene();
    RenderDebug();
    m_Context->EndFrame(framebuffer);

    // an unattended capture of this frame (FleX: g_screenshotFrame; the frame after g.frame steps)
    m_CaptureThisFrame = false;
    if (!m_Options.screenshot.empty() && g.frame >= m_Options.frames) {
        m_CaptureThisFrame = true;
        m_CapturePath = m_Options.screenshot;
    }
    if (m_SmokeScene >= 0 && g.frame >= m_Options.smokeFrames) {
        m_CaptureThisFrame = true;
        m_CapturePath.clear();
        if (!m_Options.smokeDir.empty()) {
            std::string name = scene->GetName();
            for (char& c : name)
                if (!std::isalnum((unsigned char)c))
                    c = '_';
            m_CapturePath = m_Options.smokeDir / (name + ".png");
        }
    }
    if (m_CaptureThisFrame) {
        m_CaptureFrame = g.frame;
        nvrhi::ITexture* frameTexture = m_Context->GetFrameTexture();
        if (!m_Staging) {
            nvrhi::TextureDesc d = frameTexture->getDesc();
            d.isRenderTarget = false;
            d.debugName = "screenshot staging";
            device->createStagingTexture(d, nvrhi::CpuAccessMode::Read, &m_Staging);
        }
        m_CommandList->copyTexture2(m_Staging, nvrhi::TextureSlice(), frameTexture, nvrhi::TextureSlice());
    }

    // the picked particle follows the mouse ray of this frame's view
    if (m_MouseParticle != -1) {
        dm::float3 origin, dir;
        m_Context->GetViewRay(m_LastX, m_LastY, origin, dir);
        m_MousePos = origin + dir * m_MouseT;
    }

    // ---- the physics of the frame (FleX: the Flex update) ----
    const auto updateBegin = std::chrono::high_resolution_clock::now();
    const bool advance = !g.pause || g.step;
    // the scene's changes to the constraints (FleX: SyncScene, after the particles were sent)
    scene->Sync(g);
    m_Physics->Simulate(m_CommandList, g, advance, m_FrameIndex);
    m_CommandList->close();
    device->executeCommandList(m_CommandList);
    m_Physics->Submitted();
    const float updateTime =
        std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - updateBegin).count();
    m_UpdateTime = m_UpdateTime == 0.0f ? updateTime : m_UpdateTime + (updateTime - m_UpdateTime) * 0.05f;

    if (advance) {
        g.frame++;
        g.step = false;
    }

    m_Monitor->Poll();

    if (m_CaptureThisFrame)
        Capture(m_CapturePath);
    if (m_Exiting)
        return;

    // a reset or a scene change asked for by the UI or the keyboard
    if (m_ResetScene) {
        m_ResetScene = false;
        InitScene(m_Scene, false);
    }
    if (m_NewScene != -1) {
        const int next = m_NewScene;
        m_NewScene = -1;
        InitScene(next);
    }
}

void DemoApp::Capture(const std::filesystem::path& path) {
    nvrhi::IDevice* device = GetDevice();
    device->waitForIdle();

    const nvrhi::TextureDesc& desc = m_Staging->getDesc();
    size_t rowPitch = 0;
    const auto* data = static_cast<const uint8_t*>(
        device->mapStagingTexture(m_Staging, nvrhi::TextureSlice(), nvrhi::CpuAccessMode::Read, rowPitch));
    bool ok = data != nullptr;
    bool uniform = true;
    size_t lit = 0;
    if (data) {
        std::vector<uint8_t> rgba(size_t(desc.width) * desc.height * 4);
        uint8_t lo[3] = {255, 255, 255}, hi[3] = {0, 0, 0};
        for (uint32_t y = 0; y < desc.height; ++y) {
            const uint8_t* src = data + size_t(y) * rowPitch;
            uint8_t* dst = rgba.data() + size_t(y) * desc.width * 4;
            for (uint32_t x = 0; x < desc.width; ++x, src += 4, dst += 4) {
                for (int c = 0; c < 3; ++c) {
                    dst[c] = src[c];
                    lo[c] = std::min(lo[c], src[c]);
                    hi[c] = std::max(hi[c], src[c]);
                }
                dst[3] = 255;
                if (src[0] > 8 || src[1] > 8 || src[2] > 8)
                    ++lit;
            }
        }
        device->unmapStagingTexture(m_Staging);
        uniform = hi[0] - lo[0] <= 2 && hi[1] - lo[1] <= 2 && hi[2] - lo[2] <= 2;
        if (!path.empty()) {
            const std::string name = path.string();
            ok = stbi_write_png(name.c_str(), int(desc.width), int(desc.height), 4, rgba.data(), int(desc.width) * 4) != 0;
        }
    }

    const char* sceneName = m_Desc.scenes[size_t(m_Scene)]->GetName();
    const double litFraction = double(lit) / double(std::max<size_t>(1, size_t(desc.width) * desc.height));
    std::printf("[Screenshot] %s, frame %d%s%s: %s, %.1f%% of the pixels not black, %u active particles\n", sceneName,
                m_CaptureFrame, path.empty() ? "" : " -> ", path.empty() ? "" : path.string().c_str(),
                !ok ? "NOT WRITTEN" : uniform ? "ONE COLOUR" : "ok", 100.0 * litFraction,
                m_Physics ? m_Physics->GetActiveCount() : 0u);
    std::fflush(stdout);
    const bool failed = !ok || uniform;

    if (m_SmokeScene >= 0) {
        if (failed)
            ++m_SmokeFailures;
        if (m_SmokeScene + 1 < int(m_Desc.scenes.size())) {
            ++m_SmokeScene;
            m_ResetScene = false;
            m_NewScene = -1;
            if (!InitScene(m_SmokeScene))
                RequestExit(1);
        } else {
            RequestExit(m_SmokeFailures ? 1 : 0);
        }
        return;
    }
    RequestExit(failed ? 1 : 0);
}

void DemoApp::RequestExit(int code) {
    m_ExitCode = std::max(m_ExitCode, code);
    m_Exiting = true;
    glfwSetWindowShouldClose(m_DeviceManager->GetWindow(), GLFW_TRUE);
}

void DemoApp::Animate(float elapsedTimeSeconds) { m_RealDt = elapsedTimeSeconds; }

void DemoApp::BackBufferResized(uint32_t width, uint32_t height, uint32_t /*sampleCount*/) {
    if (!m_Context || width == 0 || height == 0)
        return;
    GetDevice()->waitForIdle();
    m_Context->Resize(width, height);
    m_Staging = nullptr;
}

// ---- input ------------------------------------------------------------------------------------------------------

void DemoApp::InputKey(int key) {
    const int numScenes = int(m_Desc.scenes.size());
    if (key > '0' && key <= '9') {
        const int scene = key - '0' - 1;
        if (scene < numScenes)
            m_NewScene = scene;
        return;
    }

    const float kSpeed = g.camSpeed;

    switch (key) {
        case 'w': g.camVel.z = kSpeed; break;
        case 's': g.camVel.z = -kSpeed; break;
        case 'a': g.camVel.x = -kSpeed; break;
        case 'd': g.camVel.x = kSpeed; break;
        case 'q': g.camVel.y = kSpeed; break;
        case 'z': g.camVel.y = -kSpeed; break;
        case 'r': m_ResetScene = true; break;
        case 'y': g.wavePool = !g.wavePool; break;
        case 'p': g.pause = !g.pause; break;
        case 'o': g.step = true; break;
        case 'h': m_ShowHelp = !m_ShowHelp; break;
        case 'e': g.drawEllipsoids = !g.drawEllipsoids; break;
        case 't': g.drawOpaque = !g.drawOpaque; break;
        case 'v': g.drawPoints = !g.drawPoints; break;
        case 'f': g.drawSprings = (g.drawSprings + 1) % 3; break;
        case 'i': g.drawDiffuse = !g.drawDiffuse; break;
        case 'm': g.drawMesh = !g.drawMesh; break;
        case 'n': g.drawRopes = !g.drawRopes; break;
        case 'j':
            g.windTime = 0.0f;
            g.windStrength = 1.5f;
            g.windFrequency = 0.2f;
            break;
        case '.': m_Profile = !m_Profile; break;
        case 'g': g.params.gravity[1] = g.params.gravity[1] != 0.0f ? 0.0f : -9.8f; break;
        case '-':
            if (g.params.numPlanes)
                g.params.numPlanes--;
            break;
        case ' ': g.emit = !g.emit; break;
        default: break;
    }

    m_Desc.scenes[size_t(m_Scene)]->KeyDown(g, key);
}

bool DemoApp::KeyboardUpdate(int key, int /*scancode*/, int action, int mods) {
    m_Mods = mods;
    const int numScenes = int(m_Desc.scenes.size());
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_DOWN:
                if (m_SelectedScene < numScenes - 1)
                    m_SelectedScene++;
                return true;
            case GLFW_KEY_UP:
                if (m_SelectedScene > 0)
                    m_SelectedScene--;
                return true;
            case GLFW_KEY_LEFT:
                if (m_Scene > 0)
                    m_NewScene = m_Scene - 1;
                return true;
            case GLFW_KEY_RIGHT:
                if (m_Scene < numScenes - 1)
                    m_NewScene = m_Scene + 1;
                return true;
            case GLFW_KEY_ENTER: m_NewScene = m_SelectedScene; return true;
            case GLFW_KEY_ESCAPE: RequestExit(0); return true;
            default: break;
        }
        if (action == GLFW_PRESS && mods == 0) {
            int c = -1;
            if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
                c = 'a' + (key - GLFW_KEY_A);
            else if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
                c = '0' + (key - GLFW_KEY_0);
            else if (key == GLFW_KEY_PERIOD)
                c = '.';
            else if (key == GLFW_KEY_MINUS)
                c = '-';
            else if (key == GLFW_KEY_SPACE)
                c = ' ';
            if (c >= 0)
                InputKey(c);
        }
    } else if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W:
            case GLFW_KEY_S: g.camVel.z = 0.0f; break;
            case GLFW_KEY_A:
            case GLFW_KEY_D: g.camVel.x = 0.0f; break;
            case GLFW_KEY_Q:
            case GLFW_KEY_Z: g.camVel.y = 0.0f; break;
            default: break;
        }
    }
    return true;
}

bool DemoApp::MousePosUpdate(double xpos, double ypos) {
    const int x = int(xpos);
    const int y = int(ypos);
    const float dx = float(x - m_LastX);
    const float dy = float(y - m_LastY);
    m_LastX = x;
    m_LastY = y;

    if (m_LastButton == GLFW_MOUSE_BUTTON_RIGHT) {
        const float kSensitivity = DegToRad(0.1f);
        g.camAngle.x -= dx * kSensitivity;
        g.camAngle.y -= dy * kSensitivity;
    }
    return true;
}

bool DemoApp::MouseButtonUpdate(int button, int action, int mods) {
    m_Mods = mods;
    if (action == GLFW_RELEASE) {
        m_LastButton = -1;
    } else if (action == GLFW_PRESS) {
        m_LastButton = button;
        // shift + left: pick a particle
        if ((mods & GLFW_MOD_SHIFT) && button == GLFW_MOUSE_BUTTON_LEFT)
            m_MousePicked = true;
    }
    return true;
}

// ---- RunDemoApp ---------------------------------------------------------------------------------------------------

namespace {

class DemoGui : public app::ImGuiRenderPass {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    DemoGui(app::DeviceManager* deviceManager, DemoApp* app) : ImGuiRenderPass(deviceManager), m_App(app) {}

 protected:
    void BuildUI() override { m_App->DoUI(); }

 private:
    DemoApp* m_App;
};

bool parseArgs(int argc, const char* const* argv, DemoOptions& o) {
    o.api = app::GetGraphicsAPIFromCommandLine(argc, argv);
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (!std::strcmp(a, "--debug") || !std::strcmp(a, "-debug"))
            o.debug = true;
        else if (!std::strcmp(a, "--adapter"))
            o.adapter = std::atoi(next());
        else if (!std::strcmp(a, "--scene"))
            o.scene = next();
        else if (!std::strcmp(a, "--screenshot"))
            o.screenshot = next();
        else if (!std::strcmp(a, "--frames"))
            o.frames = std::atoi(next());
        else if (!std::strcmp(a, "--smoke"))
            o.smokeFrames = std::atoi(next());
        else if (!std::strcmp(a, "--smoke-dir"))
            o.smokeDir = next();
        else if (!std::strcmp(a, "--msaa"))
            o.msaa = uint32_t(std::max(1, std::atoi(next())));
        else if (!std::strcmp(a, "--width"))
            o.width = uint32_t(std::max(64, std::atoi(next())));
        else if (!std::strcmp(a, "--height"))
            o.height = uint32_t(std::max(64, std::atoi(next())));
        else if (!std::strcmp(a, "--draw-fluid"))
            o.drawFluid = true;
        else if (!std::strcmp(a, "--emit"))
            o.emit = true;
        else if (!std::strcmp(a, "--baseline-reductions"))
            o.baselineReductions = true;
        else if (!std::strcmp(a, "--solver")) {
            const char* v = next();
            if (!_stricmp(v, "pbd"))
                o.solverMode = 0;
            else if (!_stricmp(v, "xpbd"))
                o.solverMode = 1;
            else {
                std::printf("--solver takes pbd or xpbd\n");
                return false;
            }
        }
        else if (!std::strcmp(a, "--pick")) {
            const char* v = next();
            if (std::sscanf(v, "%d,%d,%d,%d", &o.pickX, &o.pickY, &o.pickFrame, &o.pickDrag) < 3) {
                std::printf("--pick takes x,y,frame[,dy]\n");
                return false;
            }
        }
        else if (!std::strcmp(a, "-d3d11") || !std::strcmp(a, "--d3d11") || !std::strcmp(a, "-dx11") ||
                 !std::strcmp(a, "--dx11") || !std::strcmp(a, "-d3d12") || !std::strcmp(a, "--d3d12") ||
                 !std::strcmp(a, "-dx12") || !std::strcmp(a, "--dx12") || !std::strcmp(a, "-vk") ||
                 !std::strcmp(a, "--vk") || !std::strcmp(a, "-vulkan") || !std::strcmp(a, "--vulkan"))
            continue;
        else {
            std::printf("unknown argument: %s\n", a);
            return false;
        }
    }
    return true;
}

}  // namespace

int RunDemoApp(int argc, const char* const* argv, DemoAppDesc desc) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    log::EnableOutputToMessageBox(false);
    ValidationMonitor monitor;  // the log callback from here on

    DemoOptions options;
    if (!parseArgs(argc, argv, options))
        return 2;
    const bool unattended = !options.screenshot.empty() || options.smokeFrames >= 0;

    auto deviceManager = nvrhi::TakeOver(app::DeviceManager::Create(options.api));
    if (!deviceManager) {
        log::error("this build has no device manager for the requested graphics API");
        return 1;
    }

    app::DeviceCreationParameters params;
    params.backBufferWidth = options.width;
    params.backBufferHeight = options.height;
    // the shaders write gamma-encoded colour (FleX's back buffer was R8G8B8A8_UNORM)
    params.swapChainFormat = nvrhi::Format::RGBA8_UNORM;
    params.vsyncEnabled = !unattended;
    params.enableDebugRuntime = options.debug;
    params.enableNvrhiValidationLayer = options.debug;
    params.messageCallback = monitor.GetRhiCallback();
    params.adapterIndex = options.adapter;
    if (options.api == nvrhi::GraphicsAPI::VULKAN && options.adapter < 0) {
        // name the physical device instead of letting the device manager choose (its automatic choice mixes up
        // the queue families of two GPUs; see the engine's tests/device_test.cpp)
        params.adapterIndex = 0;
    }

    if (!deviceManager->CreateWindowDeviceAndSwapChain(params, desc.title.c_str())) {
        log::error("cannot create a device and a window for the requested graphics API");
        return 1;
    }
    std::printf("%s: %s\n", desc.title.c_str(), deviceManager->GetRendererString());
    monitor.AttachDevice(deviceManager->GetDevice());

    int result = 0;
    {
        auto demo = MAKE_RC_OBJ_PTR(DemoApp, deviceManager.Get(), options, std::move(desc), &monitor);
        if (!demo->Init()) {
            log::error("initialisation failed");
            result = 1;
        } else {
            auto gui = MAKE_RC_OBJ_PTR(DemoGui, deviceManager.Get(), demo.Get());
            gui->Init(demo->GetShaderFactory());
            deviceManager->AddRenderPassToBack(demo);
            deviceManager->AddRenderPassToBack(gui);
            deviceManager->RunMessageLoop();
            deviceManager->RemoveRenderPass(gui);
            deviceManager->RemoveRenderPass(demo);
            deviceManager->GetDevice()->waitForIdle();
            result = demo->GetExitCode();
        }
        deviceManager->GetDevice()->waitForIdle();
        deviceManager->GetDevice()->runGarbageCollection();
    }
    monitor.DetachDevice();
    deviceManager->Shutdown();

    const int problems = monitor.GetProblems();
    std::printf("validation: %d problem(s) reported\n", problems);
    if (unattended && problems > 0)
        result = std::max(result, 1);
    return result;
}

}  // namespace ephys::framework
