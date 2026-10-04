// Test support for the solver tests (solver_test.cpp, parity_test.cpp): a headless nvrhi device with the
// debug runtime and the nvrhi validation layer, host buffers, and a scene (solver_scenes.h) loaded into an
// ephys domain with a solver.
//
// As in the other device tests, every nvrhi error or warning, every message of the D3D debug layer and
// every engine message the test did not provoke is a failure.

#ifndef ETHEREAL_PHYS_TESTS_SOLVER_SUPPORT_H
#define ETHEREAL_PHYS_TESTS_SOLVER_SUPPORT_H

#include <ethereal/phys/device.h>

#include "d3d_messages.h"
#include "solver_scenes.h"

#include <donut/app/ApplicationBase.h>
#include <donut/app/DeviceManager.h>
#include <donut/core/log.h>

#include <nvrhi/core/foundation.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace support {

using ephys::CopyRegion;
using ephys::IPhysBuffer;
using ephys::ParticleArray;
using ephys::PhysBufferHandle;

inline int g_Failures = 0;

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            ++support::g_Failures;                                         \
            std::printf("FAILED %s(%d): %s\n", __FILE__, __LINE__, #expr); \
        }                                                                  \
    } while (false)

inline std::atomic<int> g_LogProblems{0};

inline void logCallback(donut::log::Severity severity, const char* text) {
    if (severity < donut::log::Severity::Warning) return;
    ++g_LogProblems;
    std::printf("log %s: %s\n", severity == donut::log::Severity::Warning ? "warning" : "error", text);
}

class RhiMessages final : public nvrhi::ObjectImpl<nvrhi::IMessageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(RhiMessages)
    NVRHI_IMPLEMENTS_INTERFACE(nvrhi::IMessageCallback)
    NVRHI_IMPLEMENTS_INTERFACE(nvrhi::IRHIObject)
    NVRHI_END_INTERFACE_TABLE()

    void message(nvrhi::MessageSeverity severity, const char* messageText) noexcept override {
        if (severity == nvrhi::MessageSeverity::Info) return;
        ++problems;
        std::printf("nvrhi %s: %s\n", severity == nvrhi::MessageSeverity::Warning ? "warning" : "error", messageText);
    }

    std::atomic<int> problems{0};
};

class PhysMessages final : public nvrhi::ObjectImpl<ephys::IPhysMessageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysMessages)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysMessageCallback)
    NVRHI_END_INTERFACE_TABLE()

    void message(ephys::PhysMessageSeverity severity, const char* messageText) noexcept override {
        if (severity == ephys::PhysMessageSeverity::Info) return;
        if (severity == ephys::PhysMessageSeverity::Warning) {
            ++warnings;
        } else {
            ++errors;
        }
        std::printf("  (engine %s: %s)\n", severity == ephys::PhysMessageSeverity::Warning ? "warning" : "error",
                    messageText);
    }

    std::atomic<int> errors{0};
    std::atomic<int> warnings{0};
};

// Runs a call that must fail with `expected` and must tell the message callback exactly once.
#define CHECK_FAILS(expected, messages, call)           \
    do {                                                \
        const int before = (messages)->errors.load();   \
        CHECK((call) == (expected));                    \
        CHECK((messages)->errors.load() == before + 1); \
    } while (false)

template <typename T>
nvrhi::FLONG refCount(T* object) {
    object->AddRef();
    return object->Release();
}

struct Env {
    nvrhi::IDevice* device = nullptr;
    std::string renderer;  // the name of the GPU (the device manager's renderer string)
    ephys::IPhysDevice* phys = nullptr;
    PhysMessages* messages = nullptr;
    nvrhi::GraphicsAPI api = nvrhi::GraphicsAPI::D3D12;

    PhysBufferHandle create(ephys::BufferUse use, uint32_t count, uint32_t stride) const {
        ephys::PhysBufferDesc desc;
        desc.elementCount = count;
        desc.elementStride = stride;
        desc.use = use;
        desc.debugName = use == ephys::BufferUse::Upload ? "test upload" : "test readback";
        PhysBufferHandle buffer;
        CHECK(phys->createBuffer(desc, &buffer) == nvrhi::FS_OK);
        return buffer;
    }

    // An Upload buffer filled with `bytes`, seen as elements of `stride` bytes.
    PhysBufferHandle upload(const void* data, size_t bytes, uint32_t stride) const {
        PhysBufferHandle buffer = create(ephys::BufferUse::Upload, uint32_t(bytes / stride), stride);
        if (!buffer) return buffer;
        void* memory = nullptr;
        CHECK(buffer->map(&memory) == nvrhi::FS_OK);
        if (memory) {
            std::memcpy(memory, data, bytes);
            CHECK(buffer->unmap() == nvrhi::FS_OK);
        }
        return buffer;
    }
    template <typename T>
    PhysBufferHandle upload(const std::vector<T>& values, uint32_t stride) const {
        return upload(values.data(), values.size() * sizeof(T), stride);
    }

    // count elements of a device buffer from `offset` on, as T, after everything recorded has run.
    template <typename T>
    std::vector<T> fetch(IPhysBuffer* buffer, uint32_t offset, uint32_t count) const {
        std::vector<T> values;
        CHECK(buffer != nullptr);
        if (!buffer || count == 0) return values;
        const uint32_t stride = buffer->getElementStride();
        PhysBufferHandle staging = create(ephys::BufferUse::Readback, count, stride);
        CopyRegion region;
        region.srcOffset = offset;
        region.elementCount = count;
        CHECK(phys->copyBuffer(nullptr, staging, buffer, region) == nvrhi::FS_OK);
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);
        values.resize(size_t(count) * stride / sizeof(T));
        void* memory = nullptr;
        CHECK(staging->map(&memory) == nvrhi::FS_OK);
        if (memory) {
            std::memcpy(values.data(), memory, values.size() * sizeof(T));
            CHECK(staging->unmap() == nvrhi::FS_OK);
        }
        return values;
    }
};

// ---- A scene in an ephys domain ----------------------------------------------------------------------------

struct Rig {
    ephys::PhysDomainHandle domain;
    ephys::PhysSolverHandle solver;
    std::vector<ephys::TriangleMeshAssetHandle> meshes;
    std::vector<ephys::ConvexAssetHandle> convexes;
    std::vector<ephys::SdfAssetHandle> sdfs;
    uint32_t stepsTaken = 0;

    explicit operator bool() const { return domain && solver; }
};

// The shapes of a scene as the shape set takes them, at step `step`.
inline void uploadShapes(const Env& env, const scenes::Scene& scene, Rig& rig, uint32_t step) {
    const size_t n = scene.shapes.size();
    if (n == 0) return;
    std::vector<ephys::CollisionGeometry> geometry(n);
    std::vector<float> positions(n * 4), rotations(n * 4), prevPositions(n * 4);
    std::vector<int32_t> flags(n);
    for (size_t i = 0; i < n; ++i) {
        const scenes::Shape& shape = scene.shapes[i];
        std::memcpy(geometry[i].raw, shape.geometry, sizeof(shape.geometry));
        switch (shape.type) {
            case ephys::CollisionShapeType::ConvexMesh:
                geometry[i].convexMesh.mesh = rig.convexes[size_t(shape.asset)]->getId();
                break;
            case ephys::CollisionShapeType::TriangleMesh:
                geometry[i].triMesh.mesh = rig.meshes[size_t(shape.asset)]->getId();
                break;
            case ephys::CollisionShapeType::Sdf:
                geometry[i].sdf.field = rig.sdfs[size_t(shape.asset)]->getId();
                break;
            default:
                break;
        }
        scene.shapePosition(shape, step, &positions[i * 4], &prevPositions[i * 4]);
        std::memcpy(&rotations[i * 4], shape.rotation, sizeof(shape.rotation));
        flags[i] = ephys::makeShapeFlags(shape.type, shape.dynamic);
    }
    ephys::CollisionShapeData data;
    data.geometry = geometry.data();
    data.positions = positions.data();
    data.rotations = rotations.data();
    data.prevPositions = prevPositions.data();
    data.prevRotations = rotations.data();
    data.flags = flags.data();
    data.numShapes = uint32_t(n);
    CHECK(rig.domain->getShapes()->upload(nullptr, data) == nvrhi::FS_OK);
    (void)env;
}

struct RigOptions {
    bool baselineReductions = false;
    // A buffer of the application to make the position array (IParticleState::setArray); null: none.
    IPhysBuffer* positionArray = nullptr;
    // Leave the distance field shapes out (for a reference build that cannot load one).
    bool withoutSdf = false;
};

inline scenes::Scene withoutSdfShapes(scenes::Scene scene) {
    std::vector<scenes::Shape> kept;
    for (const scenes::Shape& shape : scene.shapes)
        if (shape.type != ephys::CollisionShapeType::Sdf) kept.push_back(shape);
    scene.shapes = kept;
    scene.sdfs.clear();
    return scene;
}

// Creates a domain and a solver for a scene and uploads the scene.
inline Rig loadScene(const Env& env, const scenes::Scene& scene, const RigOptions& options = RigOptions()) {
    Rig rig;
    ephys::DomainDesc domainDesc;
    domainDesc.maxParticles = scene.maxParticles();
    domainDesc.maxDiffuseParticles = scene.maxDiffuse;
    CHECK(env.phys->createDomain(domainDesc, &rig.domain) == nvrhi::FS_OK);
    if (!rig.domain) return rig;

    const uint32_t n = scene.numParticles();
    ephys::IParticleState* particles = rig.domain->getParticles();
    if (options.positionArray)
        CHECK(particles->setArray(nullptr, ParticleArray::Position, options.positionArray) == nvrhi::FS_OK);

    auto uploadArray = [&](ParticleArray array, const void* data, size_t bytes) {
        if (bytes == 0) return;
        PhysBufferHandle source = env.upload(data, bytes, ephys::particleArrayStride(array));
        CHECK(particles->upload(nullptr, array, source, nullptr) == nvrhi::FS_OK);
    };
    uploadArray(ParticleArray::Position, scene.positions.data(), scene.positions.size() * 4);
    uploadArray(ParticleArray::Velocity, scene.velocities.data(), scene.velocities.size() * 4);
    uploadArray(ParticleArray::Phase, scene.phases.data(), scene.phases.size() * 4);
    uploadArray(ParticleArray::Normal, scene.normals.data(), scene.normals.size() * 4);
    uploadArray(ParticleArray::RestPosition, scene.restPositions.data(), scene.restPositions.size() * 4);
    CHECK(particles->setActiveCount(n) == nvrhi::FS_OK);  // the active indices are the identity already

    if (scene.numSprings()) {
        PhysBufferHandle indices = env.upload(scene.springIndices, 4);
        PhysBufferHandle lengths = env.upload(scene.springLengths, 4);
        PhysBufferHandle stiffness = env.upload(scene.springStiffness, 4);
        CHECK(rig.domain->getSprings()->upload(nullptr, indices, lengths, stiffness, scene.numSprings()) ==
              nvrhi::FS_OK);
    }

    if (scene.numRigids()) {
        ephys::ShapeMatchBuffers buffers;
        PhysBufferHandle offsets = env.upload(scene.rigidOffsets, 4);
        PhysBufferHandle indices = env.upload(scene.rigidIndices, 4);
        PhysBufferHandle restPositions = env.upload(scene.rigidRestPositions, 12);
        PhysBufferHandle restNormals = env.upload(scene.rigidRestNormals, 16);
        PhysBufferHandle stiffness = env.upload(scene.rigidStiffness, 4);
        PhysBufferHandle rotations = env.upload(scene.rigidRotations, 16);
        PhysBufferHandle translations = env.upload(scene.rigidTranslations, 12);
        PhysBufferHandle thresholds, creeps;
        buffers.offsets = offsets;
        buffers.indices = indices;
        buffers.restPositions = restPositions;
        buffers.restNormals = restNormals;
        buffers.stiffness = stiffness;
        buffers.rotations = rotations;
        buffers.translations = translations;
        if (!scene.rigidThresholds.empty()) {
            thresholds = env.upload(scene.rigidThresholds, 4);
            creeps = env.upload(scene.rigidCreeps, 4);
            buffers.plasticThresholds = thresholds;
            buffers.plasticCreeps = creeps;
        }
        CHECK(rig.domain->getShapeMatches()->upload(nullptr, buffers, scene.numRigids(),
                                                    uint32_t(scene.rigidIndices.size())) == nvrhi::FS_OK);
    }

    if (scene.numTriangles()) {
        PhysBufferHandle indices = env.upload(scene.triangleIndices, 4);
        PhysBufferHandle normals = env.upload(scene.triangleNormals, 12);
        CHECK(rig.domain->getTriangles()->upload(nullptr, indices, normals, scene.numTriangles()) == nvrhi::FS_OK);
    }

    if (!scene.inflatables.empty())
        CHECK(rig.domain->getInflatables()->upload(nullptr, scene.inflatables.data(),
                                                   uint32_t(scene.inflatables.size())) == nvrhi::FS_OK);

    ephys::IGeometryLibrary* geometry = env.phys->getGeometry();
    for (const scenes::Mesh& mesh : scene.meshes) {
        ephys::TriangleMeshAssetHandle asset;
        CHECK(geometry->createTriangleMesh(&asset) == nvrhi::FS_OK);
        if (!asset) return Rig();
        PhysBufferHandle vertices = env.upload(mesh.vertices, 16);
        PhysBufferHandle indices = env.upload(mesh.indices, 4);
        ephys::Bounds3 bounds;
        std::memcpy(bounds.lower, mesh.lower, sizeof(mesh.lower));
        std::memcpy(bounds.upper, mesh.upper, sizeof(mesh.upper));
        CHECK(asset->update(nullptr, vertices, indices, uint32_t(mesh.vertices.size() / 4),
                            uint32_t(mesh.indices.size() / 3), bounds) == nvrhi::FS_OK);
        rig.meshes.push_back(asset);
    }
    for (const scenes::Convex& convex : scene.convexes) {
        ephys::ConvexAssetHandle asset;
        CHECK(geometry->createConvex(&asset) == nvrhi::FS_OK);
        if (!asset) return Rig();
        PhysBufferHandle planes = env.upload(convex.planes, 16);
        ephys::Bounds3 bounds;
        std::memcpy(bounds.lower, convex.lower, sizeof(convex.lower));
        std::memcpy(bounds.upper, convex.upper, sizeof(convex.upper));
        CHECK(asset->update(nullptr, planes, uint32_t(convex.planes.size() / 4), bounds) == nvrhi::FS_OK);
        rig.convexes.push_back(asset);
    }
    for (const scenes::Sdf& sdf : scene.sdfs) {
        ephys::SdfAssetHandle asset;
        CHECK(geometry->createSdf(&asset) == nvrhi::FS_OK);
        if (!asset) return Rig();
        CHECK(asset->update(nullptr, sdf.field.data(), uint32_t(sdf.dim), uint32_t(sdf.dim), uint32_t(sdf.dim)) ==
              nvrhi::FS_OK);
        rig.sdfs.push_back(asset);
    }
    uploadShapes(env, scene, rig, 0);

    rig.domain->setParams(scene.params);

    ephys::SolverDesc solverDesc;
    solverDesc.featureMode = scene.featureMode;
    solverDesc.maxNeighborsPerParticle = scene.maxNeighbors;
    solverDesc.maxContactsPerParticle = scene.maxContacts;
    solverDesc.baselineReductions = options.baselineReductions;
    CHECK(env.phys->createSolver(rig.domain, solverDesc, &rig.solver) == nvrhi::FS_OK);
    return rig;
}

// One step of the scene: the shapes that move are uploaded where they are in this step.
inline void stepScene(const Env& env, const scenes::Scene& scene, Rig& rig) {
    if (rig.stepsTaken > 0 && scene.hasMovingShapes()) uploadShapes(env, scene, rig, rig.stepsTaken);
    CHECK(rig.solver->step(nullptr, scene.dt, scene.substeps) == nvrhi::FS_OK);
    env.phys->flush();
    ++rig.stepsTaken;
}

inline std::vector<float> readPositions(const Env& env, const Rig& rig, uint32_t count) {
    return env.fetch<float>(rig.domain->getParticles()->getArray(ParticleArray::Position), 0, count);
}
inline std::vector<float> readVelocities(const Env& env, const Rig& rig, uint32_t count) {
    return env.fetch<float>(rig.domain->getParticles()->getArray(ParticleArray::Velocity), 0, count);
}

// ---- The device ------------------------------------------------------------------------------------------------

struct Options {
    nvrhi::GraphicsAPI api = nvrhi::GraphicsAPI::D3D12;
    int adapter = -1;
    std::string shaderPath;
    std::vector<std::string> rest;  // what parse() did not consume
};

inline bool parse(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "d3d11") == 0) {
            options.api = nvrhi::GraphicsAPI::D3D11;
        } else if (std::strcmp(argv[i], "d3d12") == 0) {
            options.api = nvrhi::GraphicsAPI::D3D12;
        } else if (std::strcmp(argv[i], "vulkan") == 0) {
            options.api = nvrhi::GraphicsAPI::VULKAN;
        } else if (std::strcmp(argv[i], "--adapter") == 0 && i + 1 < argc) {
            options.adapter = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--shaders") == 0 && i + 1 < argc) {
            options.shaderPath = argv[++i];
        } else {
            // An option of the test itself. Those that take a value keep it (it may look like a graphics API).
            static const char* const c_WithValue[] = {"--flex", "--flex-dll", "--flex-api", "--flex-device", "--scene",
                                                      "--dump"};
            options.rest.push_back(argv[i]);
            for (const char* name : c_WithValue)
                if (std::strcmp(argv[i], name) == 0 && i + 1 < argc) {
                    options.rest.push_back(argv[++i]);
                    break;
                }
        }
    }
    if (options.shaderPath.empty())
        options.shaderPath = (donut::app::GetDirectoryWithExecutable() / "shaders" / "ethereal_phys").generic_u8string();
    return true;
}

inline const char* apiName(nvrhi::GraphicsAPI api) {
    return api == nvrhi::GraphicsAPI::D3D11 ? "d3d11" : api == nvrhi::GraphicsAPI::D3D12 ? "d3d12" : "vulkan";
}

// Creates the device, runs body, checks that nothing complained and that every reference is back.
// Returns the exit code of the test: 0 passed, 1 failed, 77 no device for this graphics API.
inline int runWithDevice(const Options& options, const std::function<void(Env&, D3DMessages&)>& body) {
    donut::log::ConsoleApplicationMode();
    donut::log::SetMinSeverity(donut::log::Severity::Warning);
    donut::log::SetCallback(logCallback);
    std::printf("kernels: %s\n", options.shaderPath.c_str());

    auto rhiMessages = MAKE_RC_OBJ_PTR(RhiMessages);
    auto physMessages = MAKE_RC_OBJ_PTR(PhysMessages);

    auto deviceManager = nvrhi::TakeOver(donut::app::DeviceManager::Create(options.api));
    if (!deviceManager) {
        std::printf("SKIPPED: this build has no device manager for the requested graphics API\n");
        return 77;
    }

    donut::app::DeviceCreationParameters params;
    params.enableDebugRuntime = true;
    params.enableNvrhiValidationLayer = true;
    params.messageCallback = rhiMessages;
    params.adapterIndex = options.adapter;

    if (options.api == nvrhi::GraphicsAPI::VULKAN && options.adapter < 0) {
        // Name the physical device instead of letting the device manager choose: see device_test.cpp (its
        // automatic choice mixes up the queue families of two GPUs).
        std::vector<donut::app::AdapterInfo> adapters;
        if (deviceManager->CreateInstance(params) && deviceManager->EnumerateAdapters(adapters) && !adapters.empty()) {
            params.adapterIndex = 0;
            std::printf("vulkan adapter 0 of %d: %s\n", int(adapters.size()), adapters[0].name.c_str());
        }
    }

    if (!deviceManager->CreateHeadlessDevice(params)) {
        std::printf("SKIPPED: cannot create a headless device for the requested graphics API\n");
        return 77;
    }
    std::printf("renderer: %s\n", deviceManager->GetRendererString());

    nvrhi::IDevice* device = deviceManager->GetDevice();
    {
        D3DMessages d3dMessages(device);
        if (options.api != nvrhi::GraphicsAPI::VULKAN) {
            CHECK(d3dMessages.attached());
            std::printf("d3d debug layer: %s\n", d3dMessages.attached() ? "watched" : "NOT available");
        }
        const nvrhi::FLONG deviceRefs = refCount(device);
        const nvrhi::FLONG messagesRefs = refCount(physMessages.Get());

        ephys::PhysDeviceDesc deviceDesc;
        deviceDesc.device = device;
        deviceDesc.messageCallback = physMessages;
        deviceDesc.shaderPath = options.shaderPath.c_str();
        ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
        CHECK(phys != nullptr);
        if (phys) {
            Env env;
            env.device = device;
            env.phys = phys;
            env.messages = physMessages;
            env.api = options.api;
            env.renderer = deviceManager->GetRendererString();

            body(env, d3dMessages);
            CHECK(d3dMessages.problems() == 0);

            // Everything the tests created is gone; the device goes with its last reference.
            CHECK(phys->waitForIdle() == nvrhi::FS_OK);
            device->runGarbageCollection();
            CHECK(phys.Detach()->Release() == 0);
            device->waitForIdle();
            device->runGarbageCollection();
            CHECK(refCount(device) == deviceRefs);
            CHECK(refCount(physMessages.Get()) == messagesRefs);
        }
        CHECK(d3dMessages.problems() == 0);
    }

    device->waitForIdle();
    device->runGarbageCollection();

    // Whatever nvrhi, its validation layer or the engine complained about beyond the provoked refusals.
    CHECK(rhiMessages->problems.load() == 0);
    CHECK(g_LogProblems.load() == 0);
    CHECK(physMessages->warnings.load() == 0);

    deviceManager->Shutdown();
    deviceManager = nullptr;

    if (g_Failures) {
        std::printf("FAILED: %d check(s)\n", g_Failures);
        return 1;
    }
    std::printf("PASSED\n");
    return 0;
}

}  // namespace support

#endif /* ETHEREAL_PHYS_TESTS_SOLVER_SUPPORT_H */
