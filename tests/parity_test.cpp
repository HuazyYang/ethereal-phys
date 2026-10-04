// ethereal_phys_parity_test: the PBD solver against a reference FleX build, on the same scenes.
//
//     ethereal_phys_parity_test <d3d11|d3d12|vulkan> --flex <FleX checkout> [--flex-dll <file>]
//                               [--flex-api <d3d11|d3d12>] [--flex-device <n>] [--scene <name>] [--adapter <n>]
//                               [--shaders <dir>] [--report-only]
//
// Each scene of solver_scenes.h is loaded into an ephys domain with a PBD solver and, from the same arrays,
// into a FleX solver. Both are stepped in lockstep; after 1, 2 and 10 steps the positions and velocities of
// all particles are compared, and after a longer run the centre of mass and the bounds. After the first
// step the stage outputs are compared as well (the particle bounds, densities, normals, contact and
// neighbour counts, the rigid transforms, the diffuse particles), which is what locates a porting error:
// a stage that disagrees after one step is a bug, not noise.
//
// The reference is a DLL of the FleX checkout the build was pointed at (the CMake cache variable
// ETHEREAL_PHYS_FLEX_REFERENCE_DIR; ctest passes it as --flex). It is loaded at run time with LoadLibrary
// and called through GetProcAddress, against the C API of reference/flex/include/NvFlex.h: nothing of FleX
// is linked, and no DLL is copied. By default that is bin/win64/NvFlexReleaseD3D_x64.dll, the solver NVIDIA
// shipped; --flex-dll names another one (the NvFlexRev build of the checkout). The reference runs on a
// D3D device of its own (NvFlexInit creates it), with the vendor extensions off, so that it dispatches the
// kernels this engine ported; the engine runs its baseline reductions (SolverDesc::baselineReductions).
//
// Exit code: 0 passed, 1 failed, 77 skipped: no device for the graphics API, no --flex, or the reference
// DLL is missing or cannot be loaded (the reason is printed).

#include <ethereal/phys/device.h>
#include <ethereal/phys/solver.h>

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
#include <d3d12.h>
#include <dxgi1_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::SolverOutput;

// ---- The reference DLL -----------------------------------------------------------------------------------------

#define FLEX_FUNCTIONS(F)        \
    F(NvFlexInit)                \
    F(NvFlexShutdown)            \
    F(NvFlexGetVersion)          \
    F(NvFlexGetDeviceName)       \
    F(NvFlexSetSolverDescDefaults) \
    F(NvFlexCreateSolver)        \
    F(NvFlexDestroySolver)       \
    F(NvFlexUpdateSolver)        \
    F(NvFlexSetParams)           \
    F(NvFlexSetActive)           \
    F(NvFlexSetActiveCount)      \
    F(NvFlexSetParticles)        \
    F(NvFlexGetParticles)        \
    F(NvFlexSetRestParticles)    \
    F(NvFlexSetVelocities)       \
    F(NvFlexGetVelocities)       \
    F(NvFlexSetPhases)           \
    F(NvFlexSetNormals)          \
    F(NvFlexGetNormals)          \
    F(NvFlexSetSprings)          \
    F(NvFlexSetRigids)           \
    F(NvFlexGetRigids)           \
    F(NvFlexCreateTriangleMesh)  \
    F(NvFlexDestroyTriangleMesh) \
    F(NvFlexUpdateTriangleMesh)  \
    F(NvFlexCreateDistanceField) \
    F(NvFlexDestroyDistanceField) \
    F(NvFlexUpdateDistanceField) \
    F(NvFlexCreateConvexMesh)    \
    F(NvFlexDestroyConvexMesh)   \
    F(NvFlexUpdateConvexMesh)    \
    F(NvFlexSetShapes)           \
    F(NvFlexSetDynamicTriangles) \
    F(NvFlexSetInflatables)      \
    F(NvFlexGetDensities)        \
    F(NvFlexGetDiffuseParticles) \
    F(NvFlexGetContacts)         \
    F(NvFlexGetNeighbors)        \
    F(NvFlexGetBounds)           \
    F(NvFlexAllocBuffer)         \
    F(NvFlexFreeBuffer)          \
    F(NvFlexMap)                 \
    F(NvFlexUnmap)

struct FlexApi {
    HMODULE module = nullptr;
#define DECLARE(name) decltype(&::name) name = nullptr;
    FLEX_FUNCTIONS(DECLARE)
#undef DECLARE

    // Loads the DLL with its own directory on the search path for what it depends on. Returns what is
    // wrong, or an empty string.
    std::string load(const std::filesystem::path& file) {
        if (!std::filesystem::exists(file)) return "the file does not exist";
        // LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR puts the directory of the DLL on the search path of this load (its
        // dependencies, e.g. amd_ags_x64.dll, sit beside it); it needs a fully qualified path with backslashes.
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

int g_FlexErrors = 0;

void flexError(NvFlexErrorSeverity severity, const char* message, const char* file, int line) {
    if (severity == eNvFlexLogError) ++g_FlexErrors;
    if (severity <= eNvFlexLogWarning) std::printf("  (FleX %s: %s [%s:%d])\n", severity == eNvFlexLogError ? "error" : "warning",
                                                   message ? message : "", file ? file : "", line);
}

// A scene in a FleX solver.
class FlexSim {
 public:
    FlexSim(const FlexApi& api, NvFlexLibrary* library) : m_Api(api), m_Library(library) {}

    ~FlexSim() {
        if (m_Solver) m_Api.NvFlexDestroySolver(m_Solver);
        for (NvFlexTriangleMeshId id : m_Meshes) m_Api.NvFlexDestroyTriangleMesh(m_Library, id);
        for (NvFlexConvexMeshId id : m_Convexes) m_Api.NvFlexDestroyConvexMesh(m_Library, id);
        for (NvFlexDistanceFieldId id : m_Sdfs) m_Api.NvFlexDestroyDistanceField(m_Library, id);
        for (NvFlexBuffer* buffer : m_Buffers) m_Api.NvFlexFreeBuffer(buffer);
    }

    bool load(const scenes::Scene& scene) {
        m_Max = scene.maxParticles();
        m_MaxDiffuse = scene.maxDiffuse;
        m_MaxContacts = scene.maxContacts;
        m_MaxNeighbors = scene.maxNeighbors;

        NvFlexSolverDesc desc;
        m_Api.NvFlexSetSolverDescDefaults(&desc);
        desc.featureMode = NvFlexFeatureMode(int(scene.featureMode));
        desc.maxParticles = int(m_Max);
        desc.maxDiffuseParticles = int(scene.maxDiffuse);
        desc.maxNeighborsPerParticle = int(scene.maxNeighbors);
        desc.maxContactsPerParticle = int(scene.maxContacts);
        m_Solver = m_Api.NvFlexCreateSolver(m_Library, &desc);
        if (!m_Solver) return false;

        const uint32_t n = scene.numParticles();
        m_Api.NvFlexSetParticles(m_Solver, host(scene.positions, 16), nullptr);
        m_Api.NvFlexSetVelocities(m_Solver, host(scene.velocities, 12), nullptr);
        m_Api.NvFlexSetPhases(m_Solver, host(scene.phases, 4), nullptr);
        if (!scene.normals.empty()) m_Api.NvFlexSetNormals(m_Solver, host(scene.normals, 16), nullptr);
        if (!scene.restPositions.empty()) m_Api.NvFlexSetRestParticles(m_Solver, host(scene.restPositions, 16), nullptr);
        std::vector<int32_t> active(n);
        for (uint32_t i = 0; i < n; ++i) active[i] = int32_t(i);
        m_Api.NvFlexSetActive(m_Solver, host(active, 4), nullptr);
        m_Api.NvFlexSetActiveCount(m_Solver, int(n));

        if (scene.numSprings())
            m_Api.NvFlexSetSprings(m_Solver, host(scene.springIndices, 4), host(scene.springLengths, 4),
                                   host(scene.springStiffness, 4), int(scene.numSprings()));

        if (scene.numRigids()) {
            const bool plastic = !scene.rigidThresholds.empty();
            m_Api.NvFlexSetRigids(m_Solver, host(scene.rigidOffsets, 4), host(scene.rigidIndices, 4),
                                  host(scene.rigidRestPositions, 12), host(scene.rigidRestNormals, 16),
                                  host(scene.rigidStiffness, 4), plastic ? host(scene.rigidThresholds, 4) : nullptr,
                                  plastic ? host(scene.rigidCreeps, 4) : nullptr, host(scene.rigidRotations, 16),
                                  host(scene.rigidTranslations, 12), int(scene.numRigids()), int(scene.rigidIndices.size()));
        }

        if (scene.numTriangles())
            m_Api.NvFlexSetDynamicTriangles(m_Solver, host(scene.triangleIndices, 4), host(scene.triangleNormals, 12),
                                            int(scene.numTriangles()));

        if (!scene.inflatables.empty()) {
            std::vector<int32_t> starts, counts;
            std::vector<float> volumes, pressures, scales;
            for (const ephys::InflatableDesc& inflatable : scene.inflatables) {
                starts.push_back(inflatable.startTri);
                counts.push_back(inflatable.numTris);
                volumes.push_back(inflatable.restVolume);
                pressures.push_back(inflatable.overPressure);
                scales.push_back(inflatable.constraintScale);
            }
            m_Api.NvFlexSetInflatables(m_Solver, host(starts, 4), host(counts, 4), host(volumes, 4), host(pressures, 4),
                                       host(scales, 4), int(scene.inflatables.size()));
        }

        for (const scenes::Mesh& mesh : scene.meshes) {
            const NvFlexTriangleMeshId id = m_Api.NvFlexCreateTriangleMesh(m_Library);
            m_Api.NvFlexUpdateTriangleMesh(m_Library, id, host(mesh.vertices, 16), host(mesh.indices, 4),
                                           int(mesh.vertices.size() / 4), int(mesh.indices.size() / 3), mesh.lower, mesh.upper);
            m_Meshes.push_back(id);
        }
        for (const scenes::Convex& convex : scene.convexes) {
            const NvFlexConvexMeshId id = m_Api.NvFlexCreateConvexMesh(m_Library);
            m_Api.NvFlexUpdateConvexMesh(m_Library, id, host(convex.planes, 16), int(convex.planes.size() / 4), convex.lower,
                                         convex.upper);
            m_Convexes.push_back(id);
        }
        for (const scenes::Sdf& sdf : scene.sdfs) {
            const NvFlexDistanceFieldId id = m_Api.NvFlexCreateDistanceField(m_Library);
            m_Api.NvFlexUpdateDistanceField(m_Library, id, sdf.dim, sdf.dim, sdf.dim, host(sdf.field, 4));
            m_Sdfs.push_back(id);
        }
        setShapes(scene, 0);

        // NvFlexParams is PhysParams with the planes in front of numPlanes and an int for the relaxation mode.
        NvFlexParams params;
        std::memset(&params, 0, sizeof(params));
        const ephys::PhysParams& p = scene.params;
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
        // The XPBD members of this header (a reference build without them does not read that far).
        params.solverMode = eNvFlexSolverPBD;
        params.stiffnessMin = 1.0e3f;
        params.stiffnessMax = 1.0e9f;
        m_Api.NvFlexSetParams(m_Solver, &params);
        return true;
    }

    void setShapes(const scenes::Scene& scene, uint32_t step) {
        const size_t n = scene.shapes.size();
        if (n == 0) return;
        std::vector<NvFlexCollisionGeometry> geometry(n);
        std::vector<float> positions(n * 4), rotations(n * 4), prevPositions(n * 4);
        std::vector<int32_t> flags(n);
        for (size_t i = 0; i < n; ++i) {
            const scenes::Shape& shape = scene.shapes[i];
            std::memset(&geometry[i], 0, sizeof(geometry[i]));
            std::memcpy(&geometry[i], shape.geometry, sizeof(shape.geometry));
            switch (shape.type) {
                case ephys::CollisionShapeType::ConvexMesh:
                    geometry[i].convexMesh.mesh = m_Convexes[size_t(shape.asset)];
                    break;
                case ephys::CollisionShapeType::TriangleMesh:
                    geometry[i].triMesh.mesh = m_Meshes[size_t(shape.asset)];
                    break;
                case ephys::CollisionShapeType::Sdf:
                    geometry[i].sdf.field = m_Sdfs[size_t(shape.asset)];
                    break;
                default:
                    break;
            }
            scene.shapePosition(shape, step, &positions[i * 4], &prevPositions[i * 4]);
            std::memcpy(&rotations[i * 4], shape.rotation, sizeof(shape.rotation));
            flags[i] = ephys::makeShapeFlags(shape.type, shape.dynamic);
        }
        m_Api.NvFlexSetShapes(m_Solver, host(geometry, 16), host(positions, 16), host(rotations, 16), host(prevPositions, 16),
                              host(rotations, 16), host(flags, 4), int(n));
    }

    void step(const scenes::Scene& scene) {
        if (m_Steps > 0 && scene.hasMovingShapes()) setShapes(scene, m_Steps);
        m_Api.NvFlexUpdateSolver(m_Solver, scene.dt, int(scene.substeps), false);
        ++m_Steps;
    }

    std::vector<float> positions(uint32_t count) {
        NvFlexBuffer* buffer = staging(m_Max, 16);
        m_Api.NvFlexGetParticles(m_Solver, buffer, nullptr);
        return read<float>(buffer, count, 16);
    }
    std::vector<float> velocities(uint32_t count) {
        NvFlexBuffer* buffer = staging(m_Max, 12);
        m_Api.NvFlexGetVelocities(m_Solver, buffer, nullptr);
        return read<float>(buffer, count, 12);
    }
    std::vector<float> normals(uint32_t count) {
        NvFlexBuffer* buffer = staging(m_Max, 16);
        m_Api.NvFlexGetNormals(m_Solver, buffer, nullptr);
        return read<float>(buffer, count, 16);
    }
    std::vector<float> densities(uint32_t count) {
        NvFlexBuffer* buffer = staging(m_Max, 4);
        m_Api.NvFlexGetDensities(m_Solver, buffer, nullptr);
        return read<float>(buffer, count, 4);
    }
    std::vector<float> bounds() {
        NvFlexBuffer* lower = staging(1, 12);
        NvFlexBuffer* upper = staging(1, 12);
        m_Api.NvFlexGetBounds(m_Solver, lower, upper);
        std::vector<float> result = read<float>(lower, 1, 12);
        const std::vector<float> high = read<float>(upper, 1, 12);
        result.insert(result.end(), high.begin(), high.end());
        return result;
    }
    // The contact counts and the neighbour counts per particle of the scene (not in the internal order).
    void counts(uint32_t count, std::vector<int32_t>& contactCounts, std::vector<int32_t>& neighborCounts) {
        NvFlexBuffer* planes = staging(m_Max * m_MaxContacts, 16);
        NvFlexBuffer* velocities = staging(m_Max * m_MaxContacts, 16);
        NvFlexBuffer* indices = staging(m_Max, 4);
        NvFlexBuffer* contacts = staging(m_Max, 4);
        m_Api.NvFlexGetContacts(m_Solver, planes, velocities, indices, contacts);
        const std::vector<int32_t> apiToInternal = read<int32_t>(indices, m_Max, 4);
        const std::vector<int32_t> sortedContacts = read<int32_t>(contacts, m_Max, 4);

        NvFlexBuffer* neighbors = staging((m_Max + 255) / 256 * 256 * m_MaxNeighbors, 4);
        NvFlexBuffer* neighborCountsBuffer = staging(m_Max, 4);
        NvFlexBuffer* toInternal = staging(m_Max, 4);
        NvFlexBuffer* toApi = staging(m_Max, 4);
        m_Api.NvFlexGetNeighbors(m_Solver, neighbors, neighborCountsBuffer, toInternal, toApi);
        const std::vector<int32_t> sortedNeighbors = read<int32_t>(neighborCountsBuffer, m_Max, 4);

        contactCounts.assign(count, -1);
        neighborCounts.assign(count, -1);
        for (uint32_t j = 0; j < count; ++j) {
            const uint32_t internal = uint32_t(apiToInternal[j]);
            if (internal >= m_Max) continue;
            contactCounts[j] = sortedContacts[internal];
            neighborCounts[j] = sortedNeighbors[internal];
        }
    }
    void rigids(const scenes::Scene& scene, std::vector<float>& rotations, std::vector<float>& translations,
                std::vector<float>& restPositions) {
        const uint32_t numRigids = scene.numRigids();
        const uint32_t numIndices = uint32_t(scene.rigidIndices.size());
        NvFlexBuffer* offsets = staging(numRigids + 1, 4);
        NvFlexBuffer* indices = staging(numIndices, 4);
        NvFlexBuffer* rest = staging(numIndices, 12);
        NvFlexBuffer* restNormals = staging(numIndices, 16);
        NvFlexBuffer* stiffness = staging(numRigids, 4);
        NvFlexBuffer* rotation = staging(numRigids, 16);
        NvFlexBuffer* translation = staging(numRigids, 12);
        m_Api.NvFlexGetRigids(m_Solver, offsets, indices, rest, restNormals, stiffness, nullptr, nullptr, rotation, translation);
        rotations = read<float>(rotation, numRigids, 16);
        translations = read<float>(translation, numRigids, 12);
        restPositions = read<float>(rest, numIndices, 12);
    }
    int32_t diffuse(std::vector<float>& positions, std::vector<float>& velocities) {
        NvFlexBuffer* p = staging(m_MaxDiffuse, 16);
        NvFlexBuffer* v = staging(m_MaxDiffuse, 16);
        NvFlexBuffer* c = staging(1, 4);
        m_Api.NvFlexGetDiffuseParticles(m_Solver, p, v, c);
        const int32_t count = read<int32_t>(c, 1, 4)[0];
        const uint32_t alive = uint32_t(std::max(0, std::min(count, int32_t(m_MaxDiffuse))));
        positions = read<float>(p, alive, 16);
        velocities = read<float>(v, alive, 16);
        return count;
    }

 private:
    // A host buffer with the contents of `values`, kept until the simulation goes.
    template <typename T>
    NvFlexBuffer* host(const std::vector<T>& values, uint32_t stride) {
        const size_t bytes = values.size() * sizeof(T);
        NvFlexBuffer* buffer = m_Api.NvFlexAllocBuffer(m_Library, int(bytes / stride), int(stride), eNvFlexBufferHost);
        m_Buffers.push_back(buffer);
        void* memory = m_Api.NvFlexMap(buffer, eNvFlexMapWait);
        std::memcpy(memory, values.data(), bytes);
        m_Api.NvFlexUnmap(buffer);
        return buffer;
    }

    NvFlexBuffer* staging(uint32_t count, uint32_t stride) {
        NvFlexBuffer* buffer = m_Api.NvFlexAllocBuffer(m_Library, int(std::max(count, 1u)), int(stride), eNvFlexBufferHost);
        m_Buffers.push_back(buffer);
        return buffer;
    }

    // Mapping waits for the copy FleX recorded into the buffer.
    template <typename T>
    std::vector<T> read(NvFlexBuffer* buffer, uint32_t count, uint32_t stride) {
        std::vector<T> values(size_t(count) * stride / sizeof(T));
        const void* memory = m_Api.NvFlexMap(buffer, eNvFlexMapWait);
        if (memory && !values.empty()) std::memcpy(values.data(), memory, values.size() * sizeof(T));
        m_Api.NvFlexUnmap(buffer);
        return values;
    }

    const FlexApi& m_Api;
    NvFlexLibrary* m_Library;
    NvFlexSolver* m_Solver = nullptr;
    std::vector<NvFlexBuffer*> m_Buffers;
    std::vector<NvFlexTriangleMeshId> m_Meshes;
    std::vector<NvFlexConvexMeshId> m_Convexes;
    std::vector<NvFlexDistanceFieldId> m_Sdfs;
    uint32_t m_Max = 0;
    uint32_t m_MaxDiffuse = 0;
    uint32_t m_MaxContacts = 0;
    uint32_t m_MaxNeighbors = 0;
    uint32_t m_Steps = 0;
};

// ---- Comparison ------------------------------------------------------------------------------------------------

struct Difference {
    float worst = 0.0f;   // the largest absolute difference
    size_t differing = 0; // elements that are not the same bits
    size_t count = 0;
    bool comparable = true;
};

template <typename T>
Difference compare(const std::vector<T>& a, const std::vector<T>& b) {
    Difference d;
    d.count = a.size();
    if (a.size() != b.size()) {
        d.comparable = false;
        d.worst = INFINITY;
        return d;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::memcmp(&a[i], &b[i], sizeof(T)) != 0) ++d.differing;
        const float delta = std::fabs(float(a[i]) - float(b[i]));
        if (!(delta <= d.worst)) d.worst = delta;
    }
    return d;
}

void print(const char* what, const Difference& d) {
    if (d.differing == 0)
        std::printf("      %-22s bit-identical (%zu values)\n", what, d.count);
    else
        std::printf("      %-22s max abs error %.3e (%zu of %zu values differ)\n", what, double(d.worst), d.differing, d.count);
}

// The mean and the extent of the particles: what is left to compare once a chaotic scene has diverged.
struct Aggregate {
    double centre[3] = {0, 0, 0};
    float lower[3] = {INFINITY, INFINITY, INFINITY};
    float upper[3] = {-INFINITY, -INFINITY, -INFINITY};
};

Aggregate aggregate(const std::vector<float>& positions) {
    Aggregate a;
    const size_t n = positions.size() / 4;
    for (size_t i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) {
            const float v = positions[i * 4 + k];
            a.centre[k] += v;
            a.lower[k] = std::min(a.lower[k], v);
            a.upper[k] = std::max(a.upper[k], v);
        }
    for (double& c : a.centre) c /= double(std::max<size_t>(n, 1));
    return a;
}

// Diffuse particles are appended with an atomic counter, so their order is not defined: sort them.
std::vector<float> sortedByPosition(const std::vector<float>& particles) {
    std::vector<std::array<float, 4>> rows(particles.size() / 4);
    for (size_t i = 0; i < rows.size(); ++i) std::memcpy(rows[i].data(), &particles[i * 4], 16);
    std::sort(rows.begin(), rows.end());
    std::vector<float> result(particles.size());
    for (size_t i = 0; i < rows.size(); ++i) std::memcpy(&result[i * 4], rows[i].data(), 16);
    return result;
}

struct Tolerance {
    float position;
    float velocity;
};

// What the two solvers may differ by after 1, 2 and 10 steps. They run the same kernels from the same
// sources, but not the same machine code: the reference was compiled by FXC to DXBC and runs on its own
// D3D11 device, the engine's kernels by DXC (DXIL, SPIR-V) or by the D3DCompile of this machine, so the
// arithmetic is the same and the rounding (fused multiply-adds, the order of a sum) is not. The values are
// set from the differences measured on two GPUs (AMD Radeon 780M, NVIDIA RTX 4050) with the engine on D3D12,
// Vulkan and D3D11: after one and two steps every scene agrees to a few units in the last place (a velocity
// is a position difference times 1/dt, so one ulp of position is about 1e-5 of velocity). After ten steps
// the scenes where a contact made in one solver and missed in the other changes the motion of a particle (a
// pile landing, a fluid splashing, particles landing on shapes) have amplified that to millimetres for a few
// particles; they are compared loosely there, every other scene stays at the level of rounding. The
// aggregates of the long run allow for a scene that has diverged.
const uint32_t c_CheckSteps[] = {1, 2, 10};
const uint32_t c_LongSteps = 120;

struct SceneResult {
    std::string name;
    Difference positions[3];
    Difference velocities[3];
    float centreError = 0.0f;
    float boundsError = 0.0f;
    bool ran = false;
};

Tolerance toleranceFor(const scenes::Scene& scene, uint32_t check) {
    const bool chaotic = scene.name == "granular" || scene.name == "fluid" || scene.name == "fluid_diffuse" ||
                         scene.name == "shapes";
    static const Tolerance c_Tolerances[3] = {{1.0e-6f, 1.0e-4f}, {2.0e-6f, 2.0e-4f}, {2.0e-5f, 5.0e-4f}};
    static const Tolerance c_Chaotic = {1.0e-2f, 1.0f};
    return check == 2 && chaotic ? c_Chaotic : c_Tolerances[check];
}

SceneResult runParity(const Env& env, D3DMessages& d3d, const FlexApi& api, NvFlexLibrary* library, scenes::Scene scene,
                      bool withoutSdf, bool reportOnly) {
    SceneResult result;
    result.name = scene.name;
    if (withoutSdf) scene = withoutSdfShapes(scene);
    const uint32_t n = scene.numParticles();
    std::printf("  %s: %u particles, %u substeps, %d iterations\n", scene.name.c_str(), n, scene.substeps,
                scene.params.numIterations);

    RigOptions options;
    options.baselineReductions = true;
    Rig rig = loadScene(env, scene, options);
    CHECK(bool(rig));
    FlexSim flex(api, library);
    const bool loaded = flex.load(scene);
    CHECK(loaded);
    if (!rig || !loaded) return result;
    result.ran = true;

    uint32_t steps = 0;
    auto advance = [&](uint32_t target) {
        for (; steps < target; ++steps) {
            stepScene(env, scene, rig);
            flex.step(scene);
        }
    };

    for (uint32_t check = 0; check < 3; ++check) {
        advance(c_CheckSteps[check]);
        const std::vector<float> ours = readPositions(env, rig, n);
        const std::vector<float> oursV = readVelocities(env, rig, n);
        const std::vector<float> theirs = flex.positions(n);
        const std::vector<float> theirsV = flex.velocities(n);
        result.positions[check] = compare(ours, theirs);
        result.velocities[check] = compare(oursV, theirsV);
        std::printf("    after %u step%s\n", c_CheckSteps[check], c_CheckSteps[check] == 1 ? "" : "s");
        print("positions", result.positions[check]);
        print("velocities", result.velocities[check]);

        if (!reportOnly) {
            const Tolerance tolerance = toleranceFor(scene, check);
            CHECK(result.positions[check].worst <= tolerance.position);
            CHECK(result.velocities[check].worst <= tolerance.velocity);
        }

        if (check != 0) continue;

        // The stage outputs after the first step.
        print("particle bounds", compare(env.fetch<float>(rig.domain->getParticles()->getBounds(), 0, 2), flex.bounds()));
        print("normals", compare(env.fetch<float>(rig.domain->getParticles()->getArray(ParticleArray::Normal), 0, n),
                                 flex.normals(n)));
        if (scene.params.fluidRestDistance > 0.0f)
            print("densities", compare(env.fetch<float>(rig.solver->getOutput(SolverOutput::Densities), 0, n), flex.densities(n)));
        {
            const std::vector<int32_t> toInternal = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ApiToInternal), 0, n);
            const std::vector<int32_t> sortedContacts = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ContactCounts), 0, n);
            const std::vector<int32_t> sortedNeighbors =
                env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::NeighborCounts), 0, n);
            std::vector<int32_t> contacts(n, -1), neighbors(n, -1), theirContacts, theirNeighbors;
            for (uint32_t j = 0; j < n && toInternal.size() == n; ++j) {
                contacts[j] = sortedContacts[size_t(toInternal[j])];
                neighbors[j] = sortedNeighbors[size_t(toInternal[j])];
            }
            flex.counts(n, theirContacts, theirNeighbors);
            const Difference contactDifference = compare(contacts, theirContacts);
            const Difference neighborDifference = compare(neighbors, theirNeighbors);
            print("contact counts", contactDifference);
            print("neighbour counts", neighborDifference);
            if (!reportOnly) {
                // A particle at the very edge of the collision margin may be in or out by rounding.
                CHECK(contactDifference.differing <= n / 100);
                CHECK(neighborDifference.differing <= n / 100);
            }
        }
        if (scene.numRigids()) {
            std::vector<float> rotations, translations, restPositions;
            flex.rigids(scene, rotations, translations, restPositions);
            ephys::IShapeMatchSet* rigids = rig.domain->getShapeMatches();
            print("rigid rotations", compare(env.fetch<float>(rigids->getRotations(), 0, scene.numRigids()), rotations));
            print("rigid translations", compare(env.fetch<float>(rigids->getTranslations(), 0, scene.numRigids()), translations));
            print("rigid rest positions",
                  compare(env.fetch<float>(rigids->getRestPositions(), 0, uint32_t(scene.rigidIndices.size())), restPositions));
        }
    }

    // Diffuse particles, after ten steps (the first ones appear after a few).
    if (scene.maxDiffuse) {
        std::vector<float> theirPositions, theirVelocities;
        const int32_t theirCount = flex.diffuse(theirPositions, theirVelocities);
        ephys::IDiffuseParticleState* diffuse = rig.domain->getDiffuseParticles();
        const int32_t ourCount = env.fetch<int32_t>(diffuse->getCountBuffer(), 0, 1)[0];
        std::printf("      diffuse particles      %d here, %d in the reference\n", ourCount, theirCount);
        if (!reportOnly) CHECK(std::abs(ourCount - theirCount) <= std::max(4, theirCount / 50));
        if (ourCount == theirCount && ourCount > 0) {
            const std::vector<float> ours = env.fetch<float>(diffuse->getPositions(), 0, uint32_t(ourCount));
            const Difference d = compare(sortedByPosition(ours), sortedByPosition(theirPositions));
            print("diffuse positions", d);
        }
    }

    // The long run: aggregates only.
    advance(c_LongSteps);
    const Aggregate ours = aggregate(readPositions(env, rig, n));
    const Aggregate theirs = aggregate(flex.positions(n));
    for (int k = 0; k < 3; ++k) {
        result.centreError = std::max(result.centreError, float(std::fabs(ours.centre[k] - theirs.centre[k])));
        result.boundsError = std::max({result.boundsError, std::fabs(ours.lower[k] - theirs.lower[k]),
                                       std::fabs(ours.upper[k] - theirs.upper[k])});
    }
    std::printf("    after %u steps: centre of mass differs by %.3e, bounds by %.3e\n", c_LongSteps, double(result.centreError),
                double(result.boundsError));
    if (!reportOnly) {
        CHECK(result.centreError < 0.01f);
        CHECK(result.boundsError < 0.1f);
    }
    CHECK(d3d.problems() == 0);
    return result;
}

// The D3D device the reference runs on: D3D11 (a device and its immediate context) or D3D12 (a device and
// a direct queue), on the DXGI adapter whose name is the engine's GPU, or on adapter `index` when it is not
// negative.
class ReferenceDevice {
 public:
    ReferenceDevice() = default;
    ReferenceDevice(const ReferenceDevice&) = delete;
    ReferenceDevice& operator=(const ReferenceDevice&) = delete;
    ~ReferenceDevice() {
        if (m_Context11) m_Context11->Release();
        if (m_Device11) m_Device11->Release();
        if (m_Queue12) m_Queue12->Release();
        if (m_Device12) m_Device12->Release();
    }

    std::string create(bool d3d12, const std::string& gpuName, int index) {
        IDXGIFactory1* factory = nullptr;
        if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) return "CreateDXGIFactory1 failed";
        IDXGIAdapter1* chosen = nullptr;
        IDXGIAdapter1* adapter = nullptr;
        for (UINT i = 0; factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i) {
            DXGI_ADAPTER_DESC1 desc;
            adapter->GetDesc1(&desc);
            char name[256] = {};
            WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, name, sizeof(name) - 1, nullptr, nullptr);
            const bool match = index >= 0 ? int(i) == index : gpuName == name;
            if (match && !chosen) {
                chosen = adapter;
                std::printf("reference adapter %u: %s\n", i, name);
            } else {
                adapter->Release();
            }
        }
        factory->Release();
        if (!chosen) return "no DXGI adapter is \"" + gpuName + "\"";

        HRESULT hr;
        if (d3d12) {
            hr = D3D12CreateDevice(chosen, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&m_Device12));
            if (SUCCEEDED(hr)) {
                D3D12_COMMAND_QUEUE_DESC queue = {};
                queue.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
                hr = m_Device12->CreateCommandQueue(&queue, IID_PPV_ARGS(&m_Queue12));
            }
        } else {
            const D3D_FEATURE_LEVEL level = D3D_FEATURE_LEVEL_11_0;
            hr = D3D11CreateDevice(chosen, D3D_DRIVER_TYPE_UNKNOWN, nullptr, 0, &level, 1, D3D11_SDK_VERSION, &m_Device11,
                                   nullptr, &m_Context11);
        }
        chosen->Release();
        if (FAILED(hr)) return "device creation failed with HRESULT " + std::to_string(long(hr));
        return std::string();
    }

    void* device() const { return m_Device11 ? static_cast<void*>(m_Device11) : static_cast<void*>(m_Device12); }
    void* context() const { return m_Context11 ? static_cast<void*>(m_Context11) : static_cast<void*>(m_Queue12); }

 private:
    ID3D11Device* m_Device11 = nullptr;
    ID3D11DeviceContext* m_Context11 = nullptr;
    ID3D12Device* m_Device12 = nullptr;
    ID3D12CommandQueue* m_Queue12 = nullptr;
};

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // nothing is lost when the process dies
    Options options;
    parse(argc, argv, options);

    std::string flexDir, flexDll, flexApi = "d3d11", onlyScene;
    int flexDevice = -1;
    bool reportOnly = false;
    bool forceWithoutSdf = false;  // leave the distance field shape out (to bisect a difference)
    for (size_t i = 0; i < options.rest.size(); ++i) {
        const std::string& arg = options.rest[i];
        const bool hasValue = i + 1 < options.rest.size();
        if (arg == "--flex" && hasValue) {
            flexDir = options.rest[++i];
        } else if (arg == "--flex-dll" && hasValue) {
            flexDll = options.rest[++i];
        } else if (arg == "--flex-api" && hasValue) {
            flexApi = options.rest[++i];
        } else if (arg == "--flex-device" && hasValue) {
            flexDevice = std::atoi(options.rest[++i].c_str());
        } else if (arg == "--scene" && hasValue) {
            onlyScene = options.rest[++i];
        } else if (arg == "--report-only") {
            reportOnly = true;
        } else if (arg == "--without-sdf") {
            forceWithoutSdf = true;
        } else {
            std::printf("usage: %s <d3d11|d3d12|vulkan> --flex <FleX checkout> [--flex-dll <file>] [--flex-api <d3d11|d3d12>]\n"
                        "       [--flex-device <n>] [--scene <name>] [--adapter <n>] [--shaders <dir>] [--report-only]\n"
                        "       [--without-sdf]\n",
                        argv[0]);
            return 1;
        }
    }

    if (flexDir.empty() && flexDll.empty()) {
        std::printf("SKIPPED: no reference FleX checkout was given (--flex; CMake: ETHEREAL_PHYS_FLEX_REFERENCE_DIR)\n");
        return 77;
    }
    const std::filesystem::path dll = flexDll.empty()
                                          ? std::filesystem::u8path(flexDir) / "bin" / "win64" / "NvFlexReleaseD3D_x64.dll"
                                          : std::filesystem::u8path(flexDll);
    FlexApi api;
    const std::string problem = api.load(dll);
    if (!problem.empty()) {
        std::printf("SKIPPED: the reference DLL %s cannot be used: %s\n", dll.u8string().c_str(), problem.c_str());
        return 77;
    }
    // The NvFlexRev build of the checkout has no NvFlexUpdateDistanceField (it loads a field from a texture
    // object of its own API): the distance field shape is left out on both sides there.
    const bool isRev = dll.filename().u8string().find("NvFlexRev") != std::string::npos;
    const bool withoutSdf = isRev || forceWithoutSdf;
    const bool shipped = !isRev;

    // D3D12 hands out one device per adapter: a D3D12 reference on the GPU of a D3D12 engine shares the
    // engine's ID3D12Device, and the debug layer messages of the reference (the shipped DLL clears structured
    // buffers with ClearUnorderedAccessViewUint, which the layer reports) land in the queue this test fails
    // on. Run the D3D12 reference against a Vulkan or D3D11 engine instead.
    if (flexApi == "d3d12" && options.api == nvrhi::GraphicsAPI::D3D12) {
        std::printf("SKIPPED: a D3D12 reference and a D3D12 engine on one GPU share one device; use --flex-api d3d11 "
                    "or another engine API\n");
        return 77;
    }

    std::vector<SceneResult> results;
    std::string skipped;  // why the reference could not run
    const int exitCode = runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        // The reference gets a D3D device of its own, on the GPU the engine runs on: the shipped DLL would
        // create one itself, the NvFlexRev build requires one (NvFlexInitDesc::renderDevice).
        ReferenceDevice device;
        const std::string deviceProblem = device.create(flexApi == "d3d12", env.renderer, flexDevice);
        if (!deviceProblem.empty()) {
            skipped = "cannot create the D3D device of the reference: " + deviceProblem;
            return;
        }

        NvFlexInitDesc initDesc;
        std::memset(&initDesc, 0, sizeof(initDesc));
        initDesc.deviceIndex = -1;
        initDesc.enableExtensions = false;  // the kernels without NVAPI / AGS: the ones this engine ported
        initDesc.renderDevice = device.device();
        initDesc.renderContext = device.context();
        initDesc.computeContext = nullptr;
        initDesc.runOnRenderContext = true;
        initDesc.computeType = flexApi == "d3d12" ? eNvFlexD3D12 : eNvFlexD3D11;
        NvFlexLibrary* library = api.NvFlexInit(NV_FLEX_VERSION, flexError, &initDesc);
        if (!library) {
            skipped = "NvFlexInit failed (" + flexApi + ")";
            return;
        }
        std::printf("reference: %s, version %d, %s, device \"%s\"%s\n", dll.u8string().c_str(), api.NvFlexGetVersion(),
                    flexApi.c_str(), api.NvFlexGetDeviceName(library), withoutSdf ? " (distance field shapes left out)" : "");

        for (const scenes::Scene& scene : scenes::all()) {
            if (!onlyScene.empty() && scene.name != onlyScene) continue;
            results.push_back(runParity(env, d3d, api, library, scene, withoutSdf, reportOnly));
        }
        CHECK(g_FlexErrors == 0);

        std::printf("\nmax abs error against the reference, engine on %s\n", apiName(env.api));
        std::printf("  %-16s | %-23s | %-23s | %-23s | %u steps\n", "scene", "1 step: pos / vel", "2 steps: pos / vel",
                    "10 steps: pos / vel", c_LongSteps);
        for (const SceneResult& r : results) {
            if (!r.ran) continue;
            std::printf("  %-16s |", r.name.c_str());
            for (int c = 0; c < 3; ++c) std::printf(" %10.3e / %10.3e |", double(r.positions[c].worst), double(r.velocities[c].worst));
            std::printf(" centre %.2e, bounds %.2e\n", double(r.centreError), double(r.boundsError));
        }

        // NvFlexShutdown of the shipped DLL dereferences an invalid address (with or without a solver ever
        // created, on a device of its own or on an external one: measured with this harness), so the library
        // is left to the end of the process. The NvFlexRev build shuts down cleanly.
        if (!shipped) api.NvFlexShutdown(library);
    });
    if (!skipped.empty()) {
        std::printf("SKIPPED: the reference %s cannot run: %s\n", dll.u8string().c_str(), skipped.c_str());
        return 77;
    }
    return exitCode;
}
