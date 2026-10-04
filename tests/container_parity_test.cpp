// ethereal_phys_container_parity_test: the container, instances, soft joints, force fields and moving frame of the
// extension layer (ext/container.h, ext/force_field.h, ext/moving_frame.h) against FleX's extension library on a
// FleX solver.
//
//     ethereal_phys_container_parity_test <d3d11|d3d12|vulkan> --flex-dll <solver DLL> --flex-ext-dll <ext DLL>
//                                         [--shaders <dir>] [--report-only]
//
// The reference is the solver DLL and the extension DLL of the FleX checkout the build was pointed at
// (ETHEREAL_PHYS_FLEX_REFERENCE_DIR; by default bin/win64/NvFlexReleaseD3D_x64.dll and NvFlexExtReleaseD3D_x64.dll,
// what NVIDIA shipped). They are loaded at run time and called through GetProcAddress; the extension DLL imports the
// solver DLL by name from its own directory, which the loader resolves to the module loaded first, so both sides of
// the extension library talk to one solver library. The reference runs on a D3D11 device of its own with the vendor
// extensions off, as in parity_test.cpp; the engine's solver uses its baseline reductions.
//
// One script runs on both sides, call for call:
//
//   create a container over a solver; allocate and free particles in a fragmenting pattern (every handed-out index
//   and the active list compared); fill the raw particles through the mapped data; build a rigid and a cloth asset
//   (each side with its own builder: bit-identical, ext_parity_test.cpp); create four instances through the mapped
//   data and destroy one (the spawned mirrors and the instances' shape transforms compared bit for bit); register
//   two force fields; tick ten times with updateInstances after each tick (FleX's documented order), adding a soft
//   joint over particles of two rigid instances before the third tick and moving it before the sixth; then destroy
//   the cloth, spawn a rigid again into the freed particles and tick once more.
//
// Compared after ticks 1, 2, 10 and 11: the active list (exact), the mapped particle data of the active particles
// (phases and rest positions exact; positions, velocities and normals within a tolerance: the two solvers run the
// same kernels compiled by different compilers, parity_test.cpp says why), the pulled bounds, the compacted shape
// transforms and triangle indices, the instances' shape transforms and indices, the joint's transform. The moving
// frame is host math and compared bit for bit.
//
// FleX behaviour the script steers around, because it is undefined there (the engine defines it; ext/container.h):
// updateInstances before a new joint was compacted (FleX reads shape 0 or past the end), updateInstances on a joint
// whose particle data is not mapped (FleX reads the unmapped host buffer: here it is mapped around the call), the
// unset count of NvFlexExtShapeData.
//
// Exit code: 0 passed, 1 failed, 77 skipped: no device for the graphics API, or a reference DLL missing.

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys/ext/container.h>
#include <ethereal/phys/ext/force_field.h>
#include <ethereal/phys/ext/moving_frame.h>

#include "solver_support.h"

// The C API of FleX, for its declarations only: every call goes through a function pointer.
#define NV_FLEX_API extern "C"
#include <NvFlex.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <d3d11.h>
#include <dxgi1_2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::ContainerParticleData;

// ---- The reference ---------------------------------------------------------------------------------------------
// The structs of reference/flex/include/NvFlexExt.h, repeated: that header pulls in FleX's runtime loader.

struct FlexExtAsset {
    float* particles;
    int numParticles;
    int maxParticles;
    int* springIndices;
    float* springCoefficients;
    float* springRestLengths;
    int numSprings;
    int* shapeIndices;
    int numShapeIndices;
    int* shapeOffsets;
    float* shapeCoefficients;
    float* shapeCenters;
    int numShapes;
    float* shapePlasticThresholds;
    float* shapePlasticCreeps;
    int* triangleIndices;
    int numTriangles;
    bool inflatable;
    float inflatableVolume;
    float inflatablePressure;
    float inflatableStiffness;
};

struct FlexExtInstance {
    int* particleIndices;
    int numParticles;
    int triangleIndex;
    int shapeIndex;
    int inflatableIndex;
    float* shapeTranslations;
    float* shapeRotations;
    const FlexExtAsset* asset;
    void* userData;
};

struct FlexExtSoftJoint {
    int* particleIndices;
    float* particleLocalPositions;
    int shapeIndex;
    int numParticles;
    float shapeTranslations[3];
    float shapeRotations[4];
    float stiffness;
    bool initialized;
};

struct FlexExtParticleData {
    float* particles;
    float* restParticles;
    float* velocities;
    int* phases;
    float* normals;
    const float* lower;
    const float* upper;
};

struct FlexExtTriangleData {
    int* indices;
    float* normals;
};

struct FlexExtShapeData {
    float* rotations;
    float* positions;
    int n;  // FleX never sets it
};

struct FlexExtForceField {
    float mPosition[3];
    float mRadius;
    float mStrength;
    int mMode;
    bool mLinearFalloff;
};
static_assert(sizeof(FlexExtForceField) == sizeof(ephys::ForceField));

struct FlexExtMovingFrame {
    float position[3];
    float rotation[4];
    float velocity[3];
    float omega[3];
    float acceleration[3];
    float tau[3];
    float delta[4][4];
};
static_assert(sizeof(FlexExtMovingFrame) == sizeof(ephys::MovingFrameState));

#define FLEX_FUNCTIONS(F)          \
    F(NvFlexInit)                  \
    F(NvFlexGetVersion)            \
    F(NvFlexGetDeviceName)         \
    F(NvFlexSetSolverDescDefaults) \
    F(NvFlexCreateSolver)          \
    F(NvFlexDestroySolver)         \
    F(NvFlexSetParams)

struct FlexApi {
    HMODULE module = nullptr;
#define DECLARE(name) decltype(&::name) name = nullptr;
    FLEX_FUNCTIONS(DECLARE)
#undef DECLARE

    std::string load(const std::filesystem::path& file) {
        if (!std::filesystem::exists(file)) return "the file does not exist";
        const std::filesystem::path dll = std::filesystem::absolute(file).make_preferred();
        module = LoadLibraryExW(dll.wstring().c_str(), nullptr,
                                LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
        if (!module) return "LoadLibrary failed with error " + std::to_string(GetLastError());
#define RESOLVE(name)                                                        \
    name = reinterpret_cast<decltype(name)>(GetProcAddress(module, #name)); \
    if (!name) return std::string("the DLL does not export ") + #name;
        FLEX_FUNCTIONS(RESOLVE)
#undef RESOLVE
        return std::string();
    }
};

using Container = void;
using Callback = void;

struct FlexExtApi {
    HMODULE module = nullptr;
    FlexExtAsset* (*CreateRigidFromMesh)(const float*, int, const int*, int, float, float) = nullptr;
    FlexExtAsset* (*CreateClothFromMesh)(const float*, int, const int*, int, float, float, float, float, float) = nullptr;
    void (*DestroyAsset)(FlexExtAsset*) = nullptr;
    Container* (*CreateContainer)(NvFlexLibrary*, NvFlexSolver*, int) = nullptr;
    void (*DestroyContainer)(Container*) = nullptr;
    int (*AllocParticles)(Container*, int, int*) = nullptr;
    void (*FreeParticles)(Container*, int, const int*) = nullptr;
    int (*GetActiveList)(Container*, int*) = nullptr;
    FlexExtParticleData (*MapParticleData)(Container*) = nullptr;
    void (*UnmapParticleData)(Container*) = nullptr;
    FlexExtTriangleData (*MapTriangleData)(Container*) = nullptr;
    void (*UnmapTriangleData)(Container*) = nullptr;
    FlexExtShapeData (*MapShapeData)(Container*) = nullptr;
    void (*UnmapShapeData)(Container*) = nullptr;
    FlexExtInstance* (*CreateInstance)(Container*, FlexExtParticleData*, const FlexExtAsset*, const float*, float, float,
                                       float, int, float) = nullptr;
    void (*DestroyInstance)(Container*, const FlexExtInstance*) = nullptr;
    void (*NotifyAssetChanged)(Container*, const FlexExtAsset*) = nullptr;
    void (*TickContainer)(Container*, float, int, bool) = nullptr;
    void (*UpdateInstances)(Container*) = nullptr;
    Callback* (*CreateForceFieldCallback)(NvFlexSolver*) = nullptr;
    void (*DestroyForceFieldCallback)(Callback*) = nullptr;
    void (*SetForceFields)(Callback*, const FlexExtForceField*, int) = nullptr;
    FlexExtSoftJoint* (*CreateSoftJoint)(Container*, const int*, const float*, const int, const float) = nullptr;
    void (*DestroySoftJoint)(Container*, FlexExtSoftJoint*) = nullptr;
    void (*SoftJointSetTransform)(Container*, FlexExtSoftJoint*, const float*, const float*) = nullptr;
    void (*MovingFrameInit)(FlexExtMovingFrame*, const float*, const float*) = nullptr;
    void (*MovingFrameUpdate)(FlexExtMovingFrame*, const float*, const float*, float) = nullptr;
    void (*MovingFrameApply)(FlexExtMovingFrame*, float*, float*, int, float, float, float) = nullptr;

    std::string load(const std::filesystem::path& file) {
        if (!std::filesystem::exists(file)) return "the file does not exist";
        const std::filesystem::path dll = std::filesystem::absolute(file).make_preferred();
        module = LoadLibraryExW(dll.wstring().c_str(), nullptr,
                                LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
        if (!module) return "LoadLibrary failed with error " + std::to_string(GetLastError());
#define RESOLVE(name)                                                                     \
    name = reinterpret_cast<decltype(name)>(GetProcAddress(module, "NvFlexExt" #name)); \
    if (!name) return std::string("the DLL does not export NvFlexExt") + #name;
        RESOLVE(CreateRigidFromMesh)
        RESOLVE(CreateClothFromMesh)
        RESOLVE(DestroyAsset)
        RESOLVE(CreateContainer)
        RESOLVE(DestroyContainer)
        RESOLVE(AllocParticles)
        RESOLVE(FreeParticles)
        RESOLVE(GetActiveList)
        RESOLVE(MapParticleData)
        RESOLVE(UnmapParticleData)
        RESOLVE(MapTriangleData)
        RESOLVE(UnmapTriangleData)
        RESOLVE(MapShapeData)
        RESOLVE(UnmapShapeData)
        RESOLVE(CreateInstance)
        RESOLVE(DestroyInstance)
        RESOLVE(NotifyAssetChanged)
        RESOLVE(TickContainer)
        RESOLVE(UpdateInstances)
        RESOLVE(CreateForceFieldCallback)
        RESOLVE(DestroyForceFieldCallback)
        RESOLVE(SetForceFields)
        RESOLVE(CreateSoftJoint)
        RESOLVE(DestroySoftJoint)
        RESOLVE(SoftJointSetTransform)
        RESOLVE(MovingFrameInit)
        RESOLVE(MovingFrameUpdate)
        RESOLVE(MovingFrameApply)
#undef RESOLVE
        return std::string();
    }
};

int g_FlexErrors = 0;

void flexError(NvFlexErrorSeverity severity, const char* message, const char* file, int line) {
    if (severity == eNvFlexLogError) ++g_FlexErrors;
    if (severity <= eNvFlexLogWarning)
        std::printf("  (FleX %s: %s [%s:%d])\n", severity == eNvFlexLogError ? "error" : "warning", message ? message : "",
                    file ? file : "", line);
}

// The D3D11 device the reference runs on, on the DXGI adapter whose name is the engine's GPU.
class ReferenceDevice {
 public:
    ReferenceDevice() = default;
    ReferenceDevice(const ReferenceDevice&) = delete;
    ReferenceDevice& operator=(const ReferenceDevice&) = delete;
    ~ReferenceDevice() {
        if (m_Context) m_Context->Release();
        if (m_Device) m_Device->Release();
    }

    std::string create(const std::string& gpuName) {
        IDXGIFactory1* factory = nullptr;
        if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) return "CreateDXGIFactory1 failed";
        IDXGIAdapter1* chosen = nullptr;
        IDXGIAdapter1* adapter = nullptr;
        for (UINT i = 0; factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i) {
            DXGI_ADAPTER_DESC1 desc;
            adapter->GetDesc1(&desc);
            char name[256] = {};
            WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, name, sizeof(name) - 1, nullptr, nullptr);
            if (gpuName == name && !chosen) {
                chosen = adapter;
                std::printf("reference adapter %u: %s\n", i, name);
            } else {
                adapter->Release();
            }
        }
        factory->Release();
        if (!chosen) return "no DXGI adapter is \"" + gpuName + "\"";
        const D3D_FEATURE_LEVEL level = D3D_FEATURE_LEVEL_11_0;
        const HRESULT hr = D3D11CreateDevice(chosen, D3D_DRIVER_TYPE_UNKNOWN, nullptr, 0, &level, 1, D3D11_SDK_VERSION,
                                             &m_Device, nullptr, &m_Context);
        chosen->Release();
        if (FAILED(hr)) return "device creation failed with HRESULT " + std::to_string(long(hr));
        return std::string();
    }

    void* device() const { return m_Device; }
    void* context() const { return m_Context; }

 private:
    ID3D11Device* m_Device = nullptr;
    ID3D11DeviceContext* m_Context = nullptr;
};

NvFlexParams toFlex(const ephys::PhysParams& p) {
    NvFlexParams params;
    std::memset(&params, 0, sizeof(params));
    params.numIterations = p.numIterations;
    std::memcpy(params.gravity, p.gravity, sizeof(params.gravity));
    params.radius = p.radius;
    params.solidRestDistance = p.solidRestDistance;
    params.fluidRestDistance = p.fluidRestDistance;
    params.dynamicFriction = p.dynamicFriction;
    params.staticFriction = p.staticFriction;
    params.particleFriction = p.particleFriction;
    params.restitution = p.restitution;
    params.adhesion = p.adhesion;
    params.sleepThreshold = p.sleepThreshold;
    params.maxSpeed = p.maxSpeed;
    params.maxAcceleration = p.maxAcceleration;
    params.shockPropagation = p.shockPropagation;
    params.dissipation = p.dissipation;
    params.damping = p.damping;
    std::memcpy(params.wind, p.wind, sizeof(params.wind));
    params.drag = p.drag;
    params.lift = p.lift;
    params.cohesion = p.cohesion;
    params.surfaceTension = p.surfaceTension;
    params.viscosity = p.viscosity;
    params.vorticityConfinement = p.vorticityConfinement;
    params.anisotropyScale = p.anisotropyScale;
    params.anisotropyMin = p.anisotropyMin;
    params.anisotropyMax = p.anisotropyMax;
    params.smoothing = p.smoothing;
    params.solidPressure = p.solidPressure;
    params.freeSurfaceDrag = p.freeSurfaceDrag;
    params.buoyancy = p.buoyancy;
    params.diffuseThreshold = p.diffuseThreshold;
    params.diffuseBuoyancy = p.diffuseBuoyancy;
    params.diffuseDrag = p.diffuseDrag;
    params.diffuseBallistic = p.diffuseBallistic;
    params.diffuseLifetime = p.diffuseLifetime;
    params.collisionDistance = p.collisionDistance;
    params.particleCollisionMargin = p.particleCollisionMargin;
    params.shapeCollisionMargin = p.shapeCollisionMargin;
    std::memcpy(params.planes, p.planes, sizeof(params.planes));
    params.numPlanes = p.numPlanes;
    params.relaxationMode = p.relaxationMode == ephys::RelaxationMode::Global ? eNvFlexRelaxationGlobal : eNvFlexRelaxationLocal;
    params.relaxationFactor = p.relaxationFactor;
    params.solverMode = eNvFlexSolverPBD;
    return params;
}

// ---- Comparison --------------------------------------------------------------------------------------------------

struct Difference {
    double worst = 0.0;
    size_t differing = 0;
    size_t count = 0;
};

Difference compare(const float* a, const float* b, size_t n) {
    Difference d;
    d.count = n;
    for (size_t i = 0; i < n; ++i) {
        if (std::memcmp(&a[i], &b[i], 4) != 0) ++d.differing;
        const double delta = std::fabs(double(a[i]) - double(b[i]));
        if (!(delta <= d.worst)) d.worst = delta;
    }
    return d;
}

void print(const char* what, const Difference& d) {
    if (d.differing == 0)
        std::printf("    %-26s bit-identical (%zu values)\n", what, d.count);
    else
        std::printf("    %-26s max abs error %.3e (%zu of %zu values differ)\n", what, d.worst, d.differing, d.count);
}

bool g_ReportOnly = false;

// What the engine may differ by after tick 1, 2, 10 and 11 (positions; velocities are a position difference over
// the substep, so 2 / dt times that); parity_test.cpp says why the two sides do not agree to the bit. Measured on
// an AMD Radeon 780M against the shipped DLLs, max abs error in a position: D3D12 1.2e-7, 1.2e-7, 4.0e-6, 5.1e-6;
// Vulkan 1.2e-7, 1.2e-7, 3.7e-6, 4.7e-6; D3D11 bit-identical through all eleven ticks (the kernels compiled by the
// D3DCompile the reference's were). The tolerances are ten times the worst.
const double c_PositionTolerance[] = {1.0e-6, 2.0e-6, 5.0e-5, 5.0e-5};

// ---- The two sides ------------------------------------------------------------------------------------------------

struct Mesh {
    std::vector<float> vertices;  // 3 per vertex
    std::vector<int32_t> indices;
};

Mesh boxMesh(float h) {
    Mesh m;
    for (int i = 0; i < 8; ++i) m.vertices.insert(m.vertices.end(), {(i & 1) ? h : -h, (i & 2) ? h : -h, (i & 4) ? h : -h});
    const int32_t faces[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                                  {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    for (const auto& f : faces) m.indices.insert(m.indices.end(), {f[0], f[1], f[2]});
    return m;
}

void clothGrid(int n, float spacing, std::vector<float>& particles, std::vector<int32_t>& indices) {
    const float o = -0.5f * spacing * float(n - 1);
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) particles.insert(particles.end(), {o + spacing * float(x), 0.0f, o + spacing * float(z), 1.0f});
    auto at = [n](int x, int z) { return int32_t(z * n + x); };
    for (int z = 0; z + 1 < n; ++z)
        for (int x = 0; x + 1 < n; ++x) {
            indices.insert(indices.end(), {at(x, z), at(x, z + 1), at(x + 1, z + 1)});
            indices.insert(indices.end(), {at(x, z), at(x + 1, z + 1), at(x + 1, z)});
        }
}

void transform(float angle, float x, float y, float z, float m[16]) {
    const float c = std::cos(angle), s = std::sin(angle);
    const float matrix[16] = {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, x, y, z, 1};
    std::memcpy(m, matrix, sizeof(matrix));
}

struct Spawn {
    int asset;  // 0 rigid, 1 cloth
    float angle, x, y, z;
    float velocity[3];
    int32_t phase;
    float invMassScale;
};

class Script {
 public:
    Script(const Env& env, const FlexApi& api, const FlexExtApi& ext, NvFlexLibrary* library)
        : m_Env(env), m_Api(api), m_Ext(ext), m_Library(library) {}

    ~Script() {
        // the engine
        m_Joint = nullptr;
        m_Fields = nullptr;
        m_Instances.clear();
        m_Container = nullptr;
        // the reference
        if (m_FlexJoint) m_Ext.DestroySoftJoint(m_FlexContainer, m_FlexJoint);
        if (m_FlexCallback) m_Ext.DestroyForceFieldCallback(m_FlexCallback);
        for (FlexExtInstance* inst : m_FlexInstances)
            if (inst) m_Ext.DestroyInstance(m_FlexContainer, inst);
        if (m_FlexContainer) m_Ext.DestroyContainer(m_FlexContainer);
        for (FlexExtAsset* asset : m_FlexAssets)
            if (asset) m_Ext.DestroyAsset(asset);
        if (m_FlexSolver) m_Api.NvFlexDestroySolver(m_FlexSolver);
    }

    bool run() {
        if (!setUp()) return false;
        allocator();
        spawn();
        forceFields();
        for (uint32_t tick = 1; tick <= 11; ++tick) {
            if (tick == 3) createJoint();
            if (tick == 6) moveJoint();
            if (tick == 11) respawn();
            this->tick();
            if (tick == 1 || tick == 2 || tick == 10 || tick == 11) compareState(tick);
        }
        return true;
    }

    double worstPosition[4] = {0, 0, 0, 0};
    double worstVelocity[4] = {0, 0, 0, 0};

 private:
    static constexpr uint32_t M = 1024;
    static constexpr float c_Dt = 1.0f / 60.0f;
    static constexpr uint32_t c_Substeps = 2;

    bool setUp() {
        m_Params = scenes::defaultParams();
        m_Params.radius = 0.1f;
        m_Params.dynamicFriction = 0.4f;
        m_Params.numIterations = 4;
        scenes::setPlaneBox(m_Params, {-3.0f, 0.0f, -3.0f}, {3.0f, 6.0f, 3.0f}, 1);
        scenes::finishParams(m_Params);

        // the reference solver and container
        NvFlexSolverDesc desc;
        m_Api.NvFlexSetSolverDescDefaults(&desc);
        desc.maxParticles = int(M);
        desc.maxDiffuseParticles = 0;
        m_FlexSolver = m_Api.NvFlexCreateSolver(m_Library, &desc);
        CHECK(m_FlexSolver != nullptr);
        if (!m_FlexSolver) return false;
        const NvFlexParams params = toFlex(m_Params);
        m_Api.NvFlexSetParams(m_FlexSolver, &params);
        m_FlexContainer = m_Ext.CreateContainer(m_Library, m_FlexSolver, int(M));
        CHECK(m_FlexContainer != nullptr);

        // the engine's
        ephys::DomainDesc domainDesc;
        domainDesc.maxParticles = M;
        CHECK(m_Env.phys->createDomain(domainDesc, &m_Domain) == nvrhi::FS_OK);
        if (!m_Domain) return false;
        m_Domain->setParams(m_Params);
        ephys::SolverDesc solverDesc;
        solverDesc.maxNeighborsPerParticle = uint32_t(desc.maxNeighborsPerParticle);
        solverDesc.maxContactsPerParticle = uint32_t(desc.maxContactsPerParticle);
        solverDesc.baselineReductions = true;
        CHECK(m_Env.phys->createSolver(m_Domain, solverDesc, &m_Solver) == nvrhi::FS_OK);
        if (!m_Solver) return false;
        CHECK(m_Env.phys->createContainer(m_Solver, M, &m_Container) == nvrhi::FS_OK);
        m_Env.phys->flush();
        return m_FlexContainer && m_Container;
    }

    // Both active lists, compared.
    void compareActive(const char* when) {
        std::vector<int32_t> ours(M), theirs(M);
        const uint32_t n = m_Container->getActiveList(ours.data(), M);
        const int m = m_Ext.GetActiveList(m_FlexContainer, theirs.data());
        const bool same = int(n) == m && std::equal(ours.begin(), ours.begin() + n, theirs.begin());
        if (!same) std::printf("    active list differs %s: %u here, %d in the reference\n", when, n, m);
        CHECK(same);
        m_Active.assign(ours.begin(), ours.begin() + n);
    }

    std::vector<int32_t> alloc(uint32_t n) {
        std::vector<int32_t> ours(n), theirs(n);
        const uint32_t got = m_Container->allocParticles(n, ours.data());
        const int gotTheirs = m_Ext.AllocParticles(m_FlexContainer, int(n), theirs.data());
        CHECK(int(got) == gotTheirs && ours == theirs);
        ++m_Allocations;
        m_AllocMismatches += !(int(got) == gotTheirs && ours == theirs);
        return ours;
    }

    void release(const std::vector<int32_t>& indices) {
        CHECK(m_Container->freeParticles(uint32_t(indices.size()), indices.data()) == nvrhi::FS_OK);
        m_Ext.FreeParticles(m_FlexContainer, int(indices.size()), indices.data());
    }

    void allocator() {
        std::printf("  allocator\n");
        std::vector<int32_t> a = alloc(40), b = alloc(30);
        std::vector<int32_t> freedA, keptA;
        for (size_t i = 0; i < a.size(); ++i) (i % 3 == 0 ? freedA : keptA).push_back(a[i]);
        release(freedA);
        std::vector<int32_t> c = alloc(20);
        release(std::vector<int32_t>(b.begin() + 5, b.begin() + 15));
        std::vector<int32_t> d = alloc(7);
        compareActive("after the allocations");
        std::printf("    %u allocations, %u mismatches; %zu particles active\n", m_Allocations, m_AllocMismatches,
                    m_Active.size());

        // The raw particles the application owns: a loose block of granular particles beside the bodies.
        m_Raw = keptA;
        m_Raw.insert(m_Raw.end(), b.begin(), b.begin() + 5);
        m_Raw.insert(m_Raw.end(), b.begin() + 15, b.end());
        m_Raw.insert(m_Raw.end(), c.begin(), c.end());
        m_Raw.insert(m_Raw.end(), d.begin(), d.end());
        ContainerParticleData ours;
        m_Container->mapParticleData(ours);
        FlexExtParticleData theirs = m_Ext.MapParticleData(m_FlexContainer);
        for (size_t i = 0; i < m_Raw.size(); ++i) {
            const size_t k = size_t(m_Raw[i]);
            const float p[4] = {-1.8f + 0.11f * float(i % 6), 0.3f + 0.11f * float(i / 36), -0.3f + 0.11f * float((i / 6) % 6), 1.0f};
            const float v[3] = {0.0f, 0.0f, 0.0f}, n[4] = {0, 0, 0, 0};
            for (float* dst : {ours.particles, theirs.particles}) std::memcpy(&dst[k * 4], p, 16);
            for (float* dst : {ours.restParticles, theirs.restParticles}) std::memcpy(&dst[k * 4], p, 16);
            for (float* dst : {ours.velocities, theirs.velocities}) std::memcpy(&dst[k * 3], v, 12);
            for (float* dst : {ours.normals, theirs.normals}) std::memcpy(&dst[k * 4], n, 16);
            ours.phases[k] = theirs.phases[k] = ephys::makePhase(10, ephys::PhaseSelfCollide);
        }
        m_Container->unmapParticleData();
        m_Ext.UnmapParticleData(m_FlexContainer);
    }

    void spawn() {
        std::printf("  instances\n");
        // The assets, each side from its own builder.
        const Mesh box = boxMesh(0.2f);
        std::vector<float> clothParticles;
        std::vector<int32_t> clothIndices;
        clothGrid(8, 0.1f, clothParticles, clothIndices);
        ephys::PhysAssetBuilderHandle builder;
        CHECK(m_Env.phys->createAssetBuilder(&builder) == nvrhi::FS_OK);
        if (!builder) return;
        m_Assets.resize(2);
        CHECK(builder->createRigidFromMesh(box.vertices.data(), 8, box.indices.data(), 36, 0.1f, 0.0f, &m_Assets[0]) ==
              nvrhi::FS_OK);
        ephys::ClothAssetParams cloth;
        cloth.stretchStiffness = 0.8f;
        cloth.bendStiffness = 0.5f;
        CHECK(builder->createClothFromMesh(clothParticles.data(), 64, clothIndices.data(), uint32_t(clothIndices.size() / 3),
                                           cloth, &m_Assets[1]) == nvrhi::FS_OK);
        m_FlexAssets.push_back(m_Ext.CreateRigidFromMesh(box.vertices.data(), 8, box.indices.data(), 36, 0.1f, 0.0f));
        m_FlexAssets.push_back(m_Ext.CreateClothFromMesh(clothParticles.data(), 64, clothIndices.data(),
                                                         int(clothIndices.size() / 3), 0.8f, 0.5f, 0.0f, 0.0f, 0.0f));
        if (!m_Assets[0] || !m_Assets[1] || !m_FlexAssets[0] || !m_FlexAssets[1]) {
            CHECK(false);
            return;
        }
        for (int a = 0; a < 2; ++a) {
            const bool same = m_Assets[a]->getParticleCount() == uint32_t(m_FlexAssets[a]->numParticles) &&
                              std::memcmp(m_Assets[a]->getParticles(), m_FlexAssets[a]->particles,
                                          size_t(m_FlexAssets[a]->numParticles) * 16) == 0;
            CHECK(same);
        }

        const Spawn spawns[] = {
            {0, 0.3f, 0.5f, 1.0f, 0.0f, {0.0f, 0.0f, 0.0f}, ephys::makePhase(1, 0), 1.0f},
            {1, -0.2f, 0.0f, 1.5f, 0.8f, {0.0f, 0.0f, 0.0f}, ephys::makePhase(2, ephys::PhaseSelfCollide), 1.0f},
            {0, 1.0f, -0.5f, 1.2f, 0.5f, {1.0f, 0.0f, 0.0f}, ephys::makePhase(3, 0), 0.5f},
            {0, 0.0f, 1.0f, 0.8f, -0.6f, {0.0f, -1.0f, 0.0f}, ephys::makePhase(4, 0), 1.0f},
        };
        ContainerParticleData ours;
        m_Container->mapParticleData(ours);
        FlexExtParticleData theirs = m_Ext.MapParticleData(m_FlexContainer);
        for (const Spawn& s : spawns) createInstance(s, ours, theirs);
        // The spawned particles, bit for bit: FleX's matrix arithmetic on both sides.
        Difference p, rest;
        uint32_t other = 0;
        for (size_t i = 0; i < m_Instances.size(); ++i) {
            const ephys::IPhysInstance* inst = m_Instances[i];
            for (uint32_t j = 0; j < inst->getParticleCount(); ++j) {
                const size_t k = size_t(inst->getParticleIndices()[j]);
                const Difference dp = compare(&ours.particles[k * 4], &theirs.particles[k * 4], 4);
                p.differing += dp.differing, p.count += 4, p.worst = std::max(p.worst, dp.worst);
                const Difference dr = compare(&ours.restParticles[k * 4], &theirs.restParticles[k * 4], 4);
                rest.differing += dr.differing, rest.count += 4;
                other += std::memcmp(&ours.velocities[k * 3], &theirs.velocities[k * 3], 12) != 0;
                other += ours.phases[k] != theirs.phases[k];
            }
        }
        print("spawned positions", p);
        print("spawned rest positions", rest);
        Difference shapes;
        for (size_t i = 0; i < m_Instances.size(); ++i) {
            const uint32_t numShapes = m_Instances[i]->getAsset()->getShapeCount();
            const Difference t = compare(m_Instances[i]->getShapeTranslations(), m_FlexInstances[i]->shapeTranslations, numShapes * 3);
            const Difference r = compare(m_Instances[i]->getShapeRotations(), m_FlexInstances[i]->shapeRotations, numShapes * 4);
            shapes.differing += t.differing + r.differing, shapes.count += t.count + r.count;
            shapes.worst = std::max({shapes.worst, t.worst, r.worst});
        }
        print("instance shape transforms", shapes);
        CHECK(p.differing == 0 && rest.differing == 0 && other == 0 && shapes.differing == 0);
        m_Container->unmapParticleData();
        m_Ext.UnmapParticleData(m_FlexContainer);

        // One destroyed before the first tick: its particles go back to the free list on both sides.
        destroyInstance(2);
        compareActive("after the instances");
    }

    void createInstance(const Spawn& s, ContainerParticleData& ours, FlexExtParticleData& theirs) {
        float m[16];
        transform(s.angle, s.x, s.y, s.z, m);
        ephys::PhysInstanceHandle inst;
        CHECK(m_Container->createInstance(&ours, m_Assets[size_t(s.asset)], m, s.velocity, s.phase, s.invMassScale, &inst) ==
              nvrhi::FS_OK);
        FlexExtInstance* flex = m_Ext.CreateInstance(m_FlexContainer, &theirs, m_FlexAssets[size_t(s.asset)], m, s.velocity[0],
                                                     s.velocity[1], s.velocity[2], s.phase, s.invMassScale);
        CHECK(inst && flex);
        if (inst && flex) {
            const bool same = int(inst->getParticleCount()) == flex->numParticles &&
                              std::memcmp(inst->getParticleIndices(), flex->particleIndices,
                                          size_t(flex->numParticles) * 4) == 0;
            CHECK(same);
            if (!same) std::printf("    the particle indices of an instance differ\n");
        }
        m_Instances.push_back(inst);
        m_FlexInstances.push_back(flex);
    }

    void destroyInstance(size_t i) {
        m_Instances[i] = nullptr;
        m_Ext.DestroyInstance(m_FlexContainer, m_FlexInstances[i]);
        m_FlexInstances[i] = nullptr;
    }

    void forceFields() {
        FlexExtForceField fields[2];
        std::memset(fields, 0, sizeof(fields));
        const float a[3] = {0.5f, 0.3f, 0.0f}, b[3] = {-1.6f, 0.4f, -0.1f};
        std::memcpy(fields[0].mPosition, a, 12);
        fields[0].mRadius = 0.6f;
        fields[0].mStrength = -5.0f;
        fields[0].mMode = 0;  // force
        fields[0].mLinearFalloff = true;
        std::memcpy(fields[1].mPosition, b, 12);
        fields[1].mRadius = 0.5f;
        fields[1].mStrength = 0.05f;
        fields[1].mMode = 1;  // impulse
        fields[1].mLinearFalloff = false;
        m_FlexCallback = m_Ext.CreateForceFieldCallback(m_FlexSolver);
        CHECK(m_FlexCallback != nullptr);
        if (m_FlexCallback) m_Ext.SetForceFields(m_FlexCallback, fields, 2);
        CHECK(m_Env.phys->createForceFieldSet(m_Solver, &m_Fields) == nvrhi::FS_OK);
        if (m_Fields)
            CHECK(m_Fields->setForceFields(reinterpret_cast<const ephys::ForceField*>(fields), 2) == nvrhi::FS_OK);
    }

    void tick() {
        CHECK(m_Container->tickContainer(nullptr, c_Dt, c_Substeps) == nvrhi::FS_OK);
        m_Env.phys->flush();
        m_Env.device->runGarbageCollection();
        m_Ext.TickContainer(m_FlexContainer, c_Dt, int(c_Substeps), false);
        // FleX's order: update the instances after the tick. FleX reads the particle mirror of a joint seen for the
        // first time without mapping it, so it is mapped around the call there.
        m_Container->updateInstances();
        m_Ext.MapParticleData(m_FlexContainer);
        m_Ext.UpdateInstances(m_FlexContainer);
        m_Ext.UnmapParticleData(m_FlexContainer);
    }

    // A joint over eight particles of the first rigid and eight of the last, their offsets from their centroid
    // taken from the reference's mirrors (identical inputs on both sides).
    void createJoint() {
        std::printf("  soft joint created before tick 3\n");
        std::vector<int32_t> indices;
        for (size_t i : {size_t(0), m_Instances.size() - 1})
            for (uint32_t j = 0; j < 8; ++j) indices.push_back(m_Instances[i]->getParticleIndices()[j]);
        FlexExtParticleData theirs = m_Ext.MapParticleData(m_FlexContainer);
        double centroid[3] = {0, 0, 0};
        for (int32_t k : indices)
            for (int j = 0; j < 3; ++j) centroid[j] += theirs.particles[size_t(k) * 4 + j] / double(indices.size());
        std::vector<float> local;
        for (int32_t k : indices)
            for (int j = 0; j < 3; ++j) local.push_back(float(theirs.particles[size_t(k) * 4 + j] - centroid[j]));
        m_Ext.UnmapParticleData(m_FlexContainer);
        CHECK(m_Container->createSoftJoint(indices.data(), local.data(), uint32_t(indices.size()), 0.8f, &m_Joint) ==
              nvrhi::FS_OK);
        m_FlexJoint = m_Ext.CreateSoftJoint(m_FlexContainer, indices.data(), local.data(), int(indices.size()), 0.8f);
        CHECK(m_Joint && m_FlexJoint);
    }

    // Up by 0.2 from where the reference's joint is, with its rotation.
    void moveJoint() {
        if (!m_Joint || !m_FlexJoint) return;
        std::printf("  soft joint moved before tick 6\n");
        float position[3], rotation[4];
        std::memcpy(position, m_FlexJoint->shapeTranslations, 12);
        std::memcpy(rotation, m_FlexJoint->shapeRotations, 16);
        print("joint translation", compare(m_Joint->getShapeTranslation(), m_FlexJoint->shapeTranslations, 3));
        position[1] += 0.2f;
        CHECK(m_Joint->setTransform(position, rotation) == nvrhi::FS_OK);
        m_Ext.SoftJointSetTransform(m_FlexContainer, m_FlexJoint, position, rotation);
    }

    // The cloth goes, and a rigid is spawned into the freed particles.
    void respawn() {
        std::printf("  before tick 11: the cloth destroyed, a rigid spawned again\n");
        destroyInstance(1);
        compareActive("after the cloth was destroyed");
        ContainerParticleData ours;
        m_Container->mapParticleData(ours);
        FlexExtParticleData theirs = m_Ext.MapParticleData(m_FlexContainer);
        createInstance({0, 0.7f, 0.0f, 1.6f, 0.9f, {0.0f, 0.5f, 0.0f}, ephys::makePhase(5, 0), 1.0f}, ours, theirs);
        m_Container->unmapParticleData();
        m_Ext.UnmapParticleData(m_FlexContainer);
    }

    void compareState(uint32_t tick) {
        const int check = tick == 1 ? 0 : tick == 2 ? 1 : tick == 10 ? 2 : 3;
        std::printf("  after tick %u\n", tick);
        compareActive("after the tick");

        ContainerParticleData ours;
        m_Container->mapParticleData(ours);
        FlexExtParticleData theirs = m_Ext.MapParticleData(m_FlexContainer);
        Difference positions, velocities, normals, rest, invMass;
        uint32_t phases = 0;
        for (int32_t index : m_Active) {
            const size_t k = size_t(index);
            auto add = [](Difference& d, const Difference& e) {
                d.differing += e.differing, d.count += e.count, d.worst = std::max(d.worst, e.worst);
            };
            add(positions, compare(&ours.particles[k * 4], &theirs.particles[k * 4], 3));
            add(invMass, compare(&ours.particles[k * 4 + 3], &theirs.particles[k * 4 + 3], 1));
            add(velocities, compare(&ours.velocities[k * 3], &theirs.velocities[k * 3], 3));
            add(normals, compare(&ours.normals[k * 4], &theirs.normals[k * 4], 4));
            add(rest, compare(&ours.restParticles[k * 4], &theirs.restParticles[k * 4], 4));
            phases += ours.phases[k] != theirs.phases[k];
        }
        print("positions", positions);
        print("velocities", velocities);
        print("normals", normals);
        print("inverse masses", invMass);
        print("rest positions", rest);
        std::printf("    %-26s %u differ\n", "phases", phases);
        // What makes the comparison mean something: the impulse field pushes the raw block sideways (gravity and
        // the ground push vertically only), and the cloth's particles have normals.
        if (tick == 1) {
            double ourSideways = 0.0, theirSideways = 0.0, ourNormal = 0.0, theirNormal = 0.0;
            for (int32_t index : m_Raw) {
                ourSideways = std::max(ourSideways, double(std::fabs(ours.velocities[size_t(index) * 3])));
                theirSideways = std::max(theirSideways, double(std::fabs(theirs.velocities[size_t(index) * 3])));
            }
            if (m_Instances[1])
                for (uint32_t j = 0; j < m_Instances[1]->getParticleCount(); ++j) {
                    const size_t k = size_t(m_Instances[1]->getParticleIndices()[j]);
                    ourNormal = std::max(ourNormal, double(std::fabs(ours.normals[k * 4 + 1])));
                    theirNormal = std::max(theirNormal, double(std::fabs(theirs.normals[k * 4 + 1])));
                }
            std::printf("    the impulse field's sideways push on the raw block: %.4f here, %.4f in the reference; the "
                        "cloth's normals reach %.3f / %.3f in y\n",
                        ourSideways, theirSideways, ourNormal, theirNormal);
            CHECK(ourSideways > 0.01 && theirSideways > 0.01);
            CHECK(ourNormal > 0.5 && theirNormal > 0.5);
        }
        print("bounds", compare(ours.lower, theirs.lower, 3));
        print("bounds (upper)", compare(ours.upper, theirs.upper, 3));
        worstPosition[check] = positions.worst;
        worstVelocity[check] = velocities.worst;
        CHECK(phases == 0 && rest.differing == 0 && invMass.differing == 0);
        if (!g_ReportOnly) {
            CHECK(positions.worst <= c_PositionTolerance[check]);
            CHECK(velocities.worst <= 2.0 * c_PositionTolerance[check] / (c_Dt / c_Substeps));
        }
        m_Container->unmapParticleData();
        m_Ext.UnmapParticleData(m_FlexContainer);

        // The compacted shapes and triangles.
        ephys::ContainerShapeData shapes;
        m_Container->mapShapeData(shapes);
        FlexExtShapeData theirShapes = m_Ext.MapShapeData(m_FlexContainer);
        const Difference rot = compare(shapes.rotations, theirShapes.rotations, size_t(shapes.n) * 4);
        const Difference pos = compare(shapes.positions, theirShapes.positions, size_t(shapes.n) * 3);
        std::printf("    %u shapes\n", shapes.n);
        print("shape rotations", rot);
        print("shape translations", pos);
        if (!g_ReportOnly) CHECK(pos.worst <= c_PositionTolerance[check] && rot.worst <= 100.0 * c_PositionTolerance[check]);
        m_Container->unmapShapeData();
        m_Ext.UnmapShapeData(m_FlexContainer);
        ephys::ContainerTriangleData triangles;
        m_Container->mapTriangleData(triangles);
        FlexExtTriangleData theirTriangles = m_Ext.MapTriangleData(m_FlexContainer);
        const bool sameTriangles =
            triangles.numTriangles == 0 ||
            std::memcmp(triangles.indices, theirTriangles.indices, size_t(triangles.numTriangles) * 12) == 0;
        std::printf("    %u triangles, indices %s\n", triangles.numTriangles, sameTriangles ? "identical" : "DIFFER");
        CHECK(sameTriangles);
        m_Container->unmapTriangleData();
        m_Ext.UnmapTriangleData(m_FlexContainer);

        // The instances (indices into the compacted arrays, and their shape transforms after updateInstances).
        uint32_t indexMismatches = 0;
        Difference transforms;
        for (size_t i = 0; i < m_Instances.size(); ++i) {
            if (!m_Instances[i] || !m_FlexInstances[i]) continue;
            const ephys::IPhysInstance* inst = m_Instances[i];
            const FlexExtInstance* flex = m_FlexInstances[i];
            indexMismatches += inst->getTriangleIndex() != flex->triangleIndex;
            indexMismatches += inst->getShapeIndex() != flex->shapeIndex;
            const uint32_t numShapes = inst->getAsset()->getShapeCount();
            const Difference t = compare(inst->getShapeTranslations(), flex->shapeTranslations, numShapes * 3);
            transforms.differing += t.differing, transforms.count += t.count, transforms.worst = std::max(transforms.worst, t.worst);
        }
        std::printf("    instance triangle and shape indices: %u differ\n", indexMismatches);
        print("instance shape translations", transforms);
        CHECK(indexMismatches == 0);
        if (m_Joint && m_FlexJoint) {
            CHECK(m_Joint->getShapeIndex() == m_FlexJoint->shapeIndex);
            CHECK(m_Joint->isInitialized() == m_FlexJoint->initialized);
            print("joint translation", compare(m_Joint->getShapeTranslation(), m_FlexJoint->shapeTranslations, 3));
            print("joint rotation", compare(m_Joint->getShapeRotation(), m_FlexJoint->shapeRotations, 4));
        }
    }

    const Env& m_Env;
    const FlexApi& m_Api;
    const FlexExtApi& m_Ext;
    NvFlexLibrary* m_Library;
    ephys::PhysParams m_Params;

    NvFlexSolver* m_FlexSolver = nullptr;
    Container* m_FlexContainer = nullptr;
    Callback* m_FlexCallback = nullptr;
    std::vector<FlexExtAsset*> m_FlexAssets;
    std::vector<FlexExtInstance*> m_FlexInstances;
    FlexExtSoftJoint* m_FlexJoint = nullptr;

    ephys::PhysDomainHandle m_Domain;
    ephys::PhysSolverHandle m_Solver;
    ephys::PhysContainerHandle m_Container;
    ephys::ForceFieldSetHandle m_Fields;
    std::vector<ephys::PhysAssetHandle> m_Assets;
    std::vector<ephys::PhysInstanceHandle> m_Instances;
    ephys::PhysSoftJointHandle m_Joint;

    std::vector<int32_t> m_Raw, m_Active;
    uint32_t m_Allocations = 0, m_AllocMismatches = 0;
};

// The moving frame: host math, compared bit for bit with the reference's.
void movingFrame(const Env& env, const FlexExtApi& ext) {
    std::printf("moving frame\n");
    const float t[3][3] = {{0.0f, 1.0f, 0.0f}, {0.1f, 1.0f, 0.02f}, {0.27f, 1.03f, -0.01f}};
    const float half[3] = {0.0f, 0.04f, 0.1f};
    float q[3][4];
    for (int i = 0; i < 3; ++i) {
        const float axis[3] = {0.2f, 0.9f, 0.39f};
        const float s = std::sin(half[i]);
        q[i][0] = axis[0] * s, q[i][1] = axis[1] * s, q[i][2] = axis[2] * s, q[i][3] = std::cos(half[i]);
    }
    ephys::MovingFrameHandle frame;
    CHECK(env.phys->createMovingFrame(t[0], q[0], &frame) == nvrhi::FS_OK);
    if (!frame) return;
    FlexExtMovingFrame theirs;
    ext.MovingFrameInit(&theirs, t[0], q[0]);
    for (int i = 1; i < 3; ++i) {
        CHECK(frame->update(t[i], q[i], 1.0f / 60.0f) == nvrhi::FS_OK);
        ext.MovingFrameUpdate(&theirs, t[i], q[i], 1.0f / 60.0f);
    }
    const Difference state = compare(&frame->getState().position[0], &theirs.position[0], sizeof(theirs) / 4);
    print("state", state);
    std::vector<float> positions, velocities;
    for (int i = 0; i < 50; ++i) {
        positions.insert(positions.end(), {std::sin(0.3f * float(i)), 1.0f + 0.01f * float(i), std::cos(0.7f * float(i)), 1.0f});
        velocities.insert(velocities.end(), {0.1f * float(i % 3), -0.2f, 0.05f * float(i % 5)});
    }
    std::vector<float> theirPositions = positions, theirVelocities = velocities;
    CHECK(frame->apply(positions.data(), velocities.data(), 50, 0.25f, 0.5f, 1.0f / 60.0f) == nvrhi::FS_OK);
    ext.MovingFrameApply(&theirs, theirPositions.data(), theirVelocities.data(), 50, 0.25f, 0.5f, 1.0f / 60.0f);
    const Difference p = compare(positions.data(), theirPositions.data(), positions.size());
    const Difference v = compare(velocities.data(), theirVelocities.data(), velocities.size());
    print("applied positions", p);
    print("applied velocities", v);
    CHECK(state.differing == 0 && p.differing == 0 && v.differing == 0);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    Options options;
    parse(argc, argv, options);
    std::filesystem::path solverDll, extDll;
    for (size_t i = 0; i < options.rest.size(); ++i) {
        if (options.rest[i] == "--flex-dll" && i + 1 < options.rest.size())
            solverDll = std::filesystem::u8path(options.rest[++i]);
        else if (options.rest[i] == "--flex-ext-dll" && i + 1 < options.rest.size())
            extDll = std::filesystem::u8path(options.rest[++i]);
        else if (options.rest[i] == "--report-only")
            g_ReportOnly = true;
    }
    if (solverDll.empty() || extDll.empty()) {
        std::printf("usage: %s <api> --flex-dll <solver DLL> --flex-ext-dll <extension DLL> [--report-only]\n", argv[0]);
        return 2;
    }
    FlexApi api;
    std::string problem = api.load(solverDll);
    if (!problem.empty()) {
        std::printf("SKIPPED: the reference solver %s: %s\n", solverDll.u8string().c_str(), problem.c_str());
        return 77;
    }
    FlexExtApi ext;
    problem = ext.load(extDll);
    if (!problem.empty()) {
        std::printf("SKIPPED: the reference extension library %s: %s\n", extDll.u8string().c_str(), problem.c_str());
        return 77;
    }

    std::string skipped;
    double worstPosition[4] = {0, 0, 0, 0}, worstVelocity[4] = {0, 0, 0, 0};
    const int exitCode = runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        ReferenceDevice device;
        const std::string deviceProblem = device.create(env.renderer);
        if (!deviceProblem.empty()) {
            skipped = "cannot create the D3D device of the reference: " + deviceProblem;
            return;
        }
        NvFlexInitDesc initDesc;
        std::memset(&initDesc, 0, sizeof(initDesc));
        initDesc.deviceIndex = -1;
        initDesc.enableExtensions = false;
        initDesc.renderDevice = device.device();
        initDesc.renderContext = device.context();
        initDesc.runOnRenderContext = true;
        initDesc.computeType = eNvFlexD3D11;
        NvFlexLibrary* library = api.NvFlexInit(NV_FLEX_VERSION, flexError, &initDesc);
        if (!library) {
            skipped = "NvFlexInit failed";
            return;
        }
        std::printf("reference: %s and %s, version %d, device \"%s\"\n", solverDll.u8string().c_str(),
                    extDll.u8string().c_str(), api.NvFlexGetVersion(), api.NvFlexGetDeviceName(library));

        movingFrame(env, ext);
        std::printf("container script\n");
        {
            Script script(env, api, ext, library);
            script.run();
            std::memcpy(worstPosition, script.worstPosition, sizeof(worstPosition));
            std::memcpy(worstVelocity, script.worstVelocity, sizeof(worstVelocity));
        }
        CHECK(g_FlexErrors == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
        CHECK(d3d.problems() == 0);
        std::printf("\nmax abs error against the reference, engine on %s: positions / velocities after ticks 1, 2, 10, 11:\n"
                    "  %.3e / %.3e   %.3e / %.3e   %.3e / %.3e   %.3e / %.3e\n",
                    apiName(env.api), worstPosition[0], worstVelocity[0], worstPosition[1], worstVelocity[1],
                    worstPosition[2], worstVelocity[2], worstPosition[3], worstVelocity[3]);
        // NvFlexShutdown of the shipped DLL crashes (parity_test.cpp): the library is left to the end of the process.
    });
    if (!skipped.empty()) {
        std::printf("SKIPPED: %s\n", skipped.c_str());
        return 77;
    }
    return exitCode;
}
