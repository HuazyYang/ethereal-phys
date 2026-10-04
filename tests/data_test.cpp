// ethereal_phys_data_test: the data model (IPhysDomain and its parts, the geometry library and its assets)
// on a real nvrhi device, headless.
//
//     ethereal_phys_data_test <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]
//
// The nvrhi device is created by Donut's DeviceManager with the debug runtime and the nvrhi validation
// layer on. Every nvrhi error or warning is a test failure, and so is every engine message the test did not
// provoke on purpose; on D3D11 and D3D12 the messages of the debug layer count as well (d3d_messages.h).
// --shaders is the directory of the compiled kernels (default: shaders/ethereal_phys beside the
// executable): a triangle mesh update builds its bounding volume hierarchy with them.
//
// What it checks:
//   - every particle array, the diffuse particles and every constraint set go to the device and come back
//     unchanged, whole and through partial CopyRegions; an upload beyond a fixed capacity is an error with
//     one message and no effect; a growing set grows; the revisions move as the headers say;
//   - setArray makes a buffer of the application the array (D3D12, Vulkan) and is FE_UNSUPPORTED on D3D11;
//     setRenderArray and updateRenderArrays copy device to device on every backend;
//   - a mixed set of shapes comes back unchanged, keeps the assets it names alive, and refuses ids that
//     name nothing;
//   - asset ids are handed out as FleX hands them out; a convex round-trips its planes and bounds, a
//     distance field its volume; a triangle mesh update builds a valid hierarchy (every triangle is
//     reachable exactly once, every node contains its children) and commit() flattens the meshes;
//   - the hierarchy builder (engine/src/bvh_builder.cpp, compiled into this test) with bounds it computes
//     itself: the path a solver will use for the shapes;
//   - every object is gone with its last reference, and the library outlives the device without recording.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>

#include "bvh_builder.h"
#include "d3d_messages.h"

#include <donut/app/ApplicationBase.h>
#include <donut/app/DeviceManager.h>
#include <donut/core/log.h>

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace {

using ephys::BvhNodeHalf;
using ephys::CopyRegion;
using ephys::IPhysBuffer;
using ephys::ParticleArray;
using ephys::PhysBufferHandle;

int g_Failures = 0;

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            ++g_Failures;                                                  \
            std::printf("FAILED %s(%d): %s\n", __FILE__, __LINE__, #expr); \
        }                                                                  \
    } while (false)

std::atomic<int> g_LogProblems{0};

void logCallback(donut::log::Severity severity, const char* text) {
    if (severity < donut::log::Severity::Warning)
        return;
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
        if (severity == nvrhi::MessageSeverity::Info)
            return;
        ++problems;
        std::printf("nvrhi %s: %s\n", severity == nvrhi::MessageSeverity::Warning ? "warning" : "error",
                    messageText);
    }

    std::atomic<int> problems{0};
};

class PhysMessages final : public nvrhi::ObjectImpl<ephys::IPhysMessageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysMessages)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysMessageCallback)
    NVRHI_END_INTERFACE_TABLE()

    void message(ephys::PhysMessageSeverity severity, const char* messageText) noexcept override {
        if (severity == ephys::PhysMessageSeverity::Info)
            return;
        if (severity == ephys::PhysMessageSeverity::Warning) {
            ++warnings;
        } else {
            ++errors;
        }
        std::printf("  (engine %s: %s)\n",
                    severity == ephys::PhysMessageSeverity::Warning ? "warning" : "error", messageText);
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

// ---- Host buffers ------------------------------------------------------------------------------------------

using Bytes = std::vector<uint8_t>;

// count elements of stride bytes that depend on the position and on the salt.
Bytes pattern(uint32_t count, uint32_t stride, uint32_t salt) {
    Bytes bytes(size_t(count) * stride);
    uint32_t state = salt * 2654435761u + 12345u;
    for (uint8_t& b : bytes) {
        state = state * 1664525u + 1013904223u;
        b = uint8_t(state >> 24);
    }
    return bytes;
}

struct Env {
    nvrhi::IDevice* device = nullptr;
    ephys::IPhysDevice* phys = nullptr;
    PhysMessages* messages = nullptr;

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
        if (!buffer)
            return buffer;
        void* memory = nullptr;
        CHECK(buffer->map(&memory) == nvrhi::FS_OK);
        if (memory) {
            std::memcpy(memory, data, bytes);
            CHECK(buffer->unmap() == nvrhi::FS_OK);
        }
        return buffer;
    }
    PhysBufferHandle upload(const Bytes& bytes, uint32_t stride) const {
        return upload(bytes.data(), bytes.size(), stride);
    }
    template <typename T>
    PhysBufferHandle upload(const std::vector<T>& values, uint32_t stride) const {
        return upload(values.data(), values.size() * sizeof(T), stride);
    }

    PhysBufferHandle readback(uint32_t count, uint32_t stride) const {
        return create(ephys::BufferUse::Readback, count, stride);
    }

    // The bytes of a Readback buffer, after everything recorded has run.
    Bytes read(IPhysBuffer* buffer) const {
        Bytes bytes(size_t(buffer->getElementCount()) * buffer->getElementStride());
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);
        void* memory = nullptr;
        CHECK(buffer->map(&memory) == nvrhi::FS_OK);
        if (memory) {
            std::memcpy(bytes.data(), memory, bytes.size());
            CHECK(buffer->unmap() == nvrhi::FS_OK);
        }
        return bytes;
    }

    // count elements of a device buffer from `offset` on, through IPhysDevice::copyBuffer.
    Bytes fetch(IPhysBuffer* buffer, uint32_t offset, uint32_t count) const {
        CHECK(buffer != nullptr);
        if (!buffer || count == 0)
            return Bytes();
        PhysBufferHandle staging = readback(count, buffer->getElementStride());
        CopyRegion region;
        region.srcOffset = offset;
        region.elementCount = count;
        CHECK(phys->copyBuffer(nullptr, staging, buffer, region) == nvrhi::FS_OK);
        return read(staging);
    }

    template <typename T>
    std::vector<T> fetchAs(IPhysBuffer* buffer, uint32_t offset, uint32_t count) const {
        const Bytes bytes = fetch(buffer, offset, count);
        std::vector<T> values(bytes.size() / sizeof(T));
        if (!values.empty())
            std::memcpy(values.data(), bytes.data(), values.size() * sizeof(T));
        return values;
    }
};

Bytes slice(const Bytes& bytes, size_t first, size_t count, uint32_t stride) {
    return Bytes(bytes.begin() + first * stride, bytes.begin() + (first + count) * stride);
}

ephys::PhysDomainHandle createDomain(const Env& env, const ephys::DomainDesc& desc) {
    ephys::PhysDomainHandle domain;
    CHECK(env.phys->createDomain(desc, &domain) == nvrhi::FS_OK);
    CHECK(domain != nullptr);
    return domain;
}

// ---- Particles ---------------------------------------------------------------------------------------------

void testParticles(const Env& env) {
    std::printf("particle arrays\n");
    constexpr uint32_t N = 1000;

    ephys::DomainDesc desc;
    desc.maxParticles = N;
    ephys::PhysDomainHandle domain = createDomain(env, desc);
    if (!domain)
        return;

    CHECK(domain->getDesc().maxParticles == N);
    CHECK(domain->getDevice() == env.phys);
    CHECK(domain->getDiffuseParticles() == nullptr);  // maxDiffuseParticles is 0
    CHECK(domain->getSprings() && domain->getShapeMatches() && domain->getTriangles() &&
          domain->getInflatables() && domain->getShapes());

    ephys::IParticleState* particles = domain->getParticles();
    CHECK(particles != nullptr);
    if (!particles)
        return;
    CHECK(particles->getMaxParticles() == N);
    CHECK(particles->getActiveCount() == 0);
    CHECK(particles->getLayoutRevision() == 0);
    CHECK(particles->getContentRevision() == 0);
    CHECK(particles->areRenderArraysShared());
    CHECK(particles->getArray(ParticleArray::Count) == nullptr);

    // QueryInterface.
    nvrhi::AutoPtr<ephys::IParticleState> same;
    CHECK(particles->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK && same == particles);
    nvrhi::AutoPtr<ephys::ISpringSet> notSprings;
    CHECK(particles->QueryInterface(NVRHI_IID_PPV_ARGS(&notSprings)) == nvrhi::FE_NOINTERFACE);
    same = nullptr;

    // The state of a new domain: zeros, the identity in the active indices, zero bounds.
    for (uint32_t a = 0; a < uint32_t(ParticleArray::Count); ++a) {
        const ParticleArray array = ParticleArray(a);
        const uint32_t stride = ephys::particleArrayStride(array);
        IPhysBuffer* buffer = particles->getArray(array);
        CHECK(buffer != nullptr);
        if (!buffer)
            return;
        CHECK(buffer->getElementCount() == N && buffer->getElementStride() == stride);
        const nvrhi::BufferDesc& rhi = buffer->getRHIBuffer()->getDesc();
        CHECK(rhi.structStride == stride && rhi.canHaveUAVs && rhi.keepInitialState);
        CHECK(!particles->isArraySubstituted(array));
        CHECK(particles->getRenderArray(array) == nullptr);

        const Bytes initial = env.fetch(buffer, 0, N);
        Bytes expected(size_t(N) * stride, 0);
        if (array == ParticleArray::ActiveIndex)
            for (uint32_t i = 0; i < N; ++i)
                std::memcpy(&expected[size_t(i) * 4], &i, 4);
        CHECK(initial == expected);
    }
    {
        IPhysBuffer* bounds = particles->getBounds();
        CHECK(bounds && bounds->getElementCount() == 2 && bounds->getElementStride() == 12);
        if (bounds)
            CHECK(env.fetch(bounds, 0, 2) == Bytes(24, 0));
    }

    // Every array: up whole, down whole.
    Bytes contents[uint32_t(ParticleArray::Count)];
    uint64_t revision = particles->getContentRevision();
    for (uint32_t a = 0; a < uint32_t(ParticleArray::Count); ++a) {
        const ParticleArray array = ParticleArray(a);
        const uint32_t stride = ephys::particleArrayStride(array);
        contents[a] = pattern(N, stride, 100 + a);
        PhysBufferHandle source = env.upload(contents[a], stride);
        CHECK(particles->upload(nullptr, array, source, nullptr) == nvrhi::FS_OK);
        CHECK(particles->getContentRevision() == ++revision);

        PhysBufferHandle target = env.readback(N, stride);
        CHECK(particles->download(nullptr, array, target, nullptr) == nvrhi::FS_OK);
        CHECK(particles->getContentRevision() == revision);  // a download changes nothing
        CHECK(env.read(target) == contents[a]);
    }
    CHECK(particles->getLayoutRevision() == 0);

    // Partial regions, on the application's command list.
    {
        const ParticleArray array = ParticleArray::Velocity;  // the 12-byte array
        const uint32_t stride = 12;
        Bytes& expected = contents[uint32_t(array)];
        const Bytes patch = pattern(64, stride, 777);
        PhysBufferHandle source = env.upload(patch, stride);

        env.phys->flush();  // D3D11: one command list open at a time
        nvrhi::CommandListHandle commandList;
        CHECK(env.device->createCommandList(nvrhi::CommandListParameters(), &commandList) == nvrhi::FS_OK);
        if (!commandList)
            return;

        CopyRegion up;  // elements [5, 55) of the patch to [100, 150) of the array
        up.srcOffset = 5;
        up.dstOffset = 100;
        up.elementCount = 50;
        commandList->open();
        CHECK(particles->upload(commandList, array, source, &up) == nvrhi::FS_OK);
        commandList->close();
        env.device->executeCommandList(commandList);
        CHECK(env.phys->flush() == 0);  // nothing went to the device's own list
        CHECK(particles->getContentRevision() == ++revision);
        std::memcpy(&expected[size_t(100) * stride], &patch[size_t(5) * stride], size_t(50) * stride);

        CopyRegion down;  // elements [90, 170) of the array to [3, 83) of the buffer
        down.srcOffset = 90;
        down.dstOffset = 3;
        down.elementCount = 80;
        PhysBufferHandle target = env.readback(100, stride);
        commandList->open();
        CHECK(particles->download(commandList, array, target, &down) == nvrhi::FS_OK);
        commandList->close();
        env.device->executeCommandList(commandList);
        const Bytes result = env.read(target);
        CHECK(slice(result, 3, 80, stride) == slice(expected, 90, 80, stride));
        CHECK(env.fetch(particles->getArray(array), 0, N) == expected);
    }

    // A null region: as many elements as the smaller side holds.
    {
        const ParticleArray array = ParticleArray::Phase;
        Bytes& expected = contents[uint32_t(array)];
        const Bytes few = pattern(10, 4, 901);
        PhysBufferHandle source = env.upload(few, 4);
        CHECK(particles->upload(nullptr, array, source, nullptr) == nvrhi::FS_OK);
        ++revision;
        std::memcpy(expected.data(), few.data(), few.size());

        const Bytes many = pattern(N + 24, 4, 902);  // larger than the capacity
        PhysBufferHandle larger = env.upload(many, 4);
        CHECK(particles->upload(nullptr, array, larger, nullptr) == nvrhi::FS_OK);
        ++revision;
        std::memcpy(expected.data(), many.data(), size_t(N) * 4);

        PhysBufferHandle target = env.readback(N + 24, 4);
        CHECK(particles->download(nullptr, array, target, nullptr) == nvrhi::FS_OK);
        CHECK(slice(env.read(target), 0, N, 4) == expected);
        CHECK(particles->getContentRevision() == revision);
    }

    // What is refused: nothing is copied, one message each.
    {
        const ParticleArray array = ParticleArray::Position;
        PhysBufferHandle source = env.upload(pattern(64, 16, 5), 16);
        PhysBufferHandle target = env.readback(64, 16);
        PhysBufferHandle wrongStride = env.upload(pattern(64, 12, 6), 12);

        CopyRegion beyond;  // reaches past the capacity
        beyond.dstOffset = N - 10;
        beyond.elementCount = 20;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->upload(nullptr, array, source, &beyond));
        CopyRegion beyondSource;  // leaves the host buffer
        beyondSource.srcOffset = 60;
        beyondSource.elementCount = 5;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->upload(nullptr, array, source, &beyondSource));
        CopyRegion beyondDown;  // reads past the capacity
        beyondDown.srcOffset = N - 1;
        beyondDown.elementCount = 2;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->download(nullptr, array, target, &beyondDown));
        CopyRegion huge;  // offset + count wraps 32 bits
        huge.dstOffset = 0xfffffff0u;
        huge.elementCount = 0x20;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->upload(nullptr, array, source, &huge));

        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->upload(nullptr, array, wrongStride, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->upload(nullptr, array, nullptr, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->upload(nullptr, array, target, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->download(nullptr, array, source, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->upload(nullptr, ParticleArray::Count, source, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->upload(nullptr, array, particles->getArray(array), nullptr));

        CHECK(particles->getContentRevision() == revision);
        CHECK(env.phys->flush() == 0);  // nothing was recorded
        CHECK(env.fetch(particles->getArray(array), 0, N) == contents[uint32_t(array)]);
    }

    // The active count.
    CHECK(particles->setActiveCount(N) == nvrhi::FS_OK);
    CHECK(particles->getActiveCount() == N);
    CHECK(particles->getContentRevision() == ++revision);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, particles->setActiveCount(N + 1));
    CHECK(particles->getActiveCount() == N);
    CHECK(particles->getContentRevision() == revision);
    CHECK(particles->markContentChanged() == ++revision);

    // The parameters live in the domain.
    {
        CHECK(domain->getParamsRevision() == 0);
        CHECK(domain->getParams().numIterations == 1 && domain->getParams().radius == 0.0f);
        ephys::PhysParams params = domain->getParams();
        params.radius = 0.15f;
        params.numPlanes = 1;
        params.planes[0][1] = 1.0f;
        domain->setParams(params);
        CHECK(domain->getParamsRevision() == 1);
        CHECK(std::memcmp(&domain->getParams(), &params, sizeof(params)) == 0);
        CHECK(&domain->getParams() == &domain->getParams());  // a view, not a copy
    }

    // A part keeps the device alive, not the domain.
    ephys::ParticleStateHandle kept = particles;
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);
    CHECK(env.fetch(kept->getArray(ParticleArray::Position), 0, N) ==
          contents[uint32_t(ParticleArray::Position)]);
    CHECK(kept.Detach()->Release() == 0);

    // A domain needs particles.
    ephys::PhysDomainHandle none;
    ephys::DomainDesc empty;
    empty.maxParticles = 0;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createDomain(empty, &none));
    CHECK(none == nullptr);
    CHECK(env.phys->createDomain(desc, nullptr) == nvrhi::FE_INVALID_ARGS);
}

// A buffer as a renderer creates it: a vertex buffer, which a kernel may write where the API allows it.
nvrhi::BufferHandle createVertexBuffer(const Env& env, uint32_t count, uint32_t stride, bool shareable,
                                       const char* name) {
    nvrhi::BufferDesc desc;
    desc.byteSize = uint64_t(count) * stride;
    desc.isVertexBuffer = true;
    desc.debugName = name;
    if (shareable) {
        desc.canHaveUAVs = true;
        desc.structStride = stride;
        desc.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
    } else {
        desc.enableAutomaticStateTracking(nvrhi::ResourceStates::VertexBuffer);
    }
    nvrhi::BufferHandle buffer;
    CHECK(env.device->createBuffer(desc, &buffer) == nvrhi::FS_OK);
    return buffer;
}

void testRenderArrays(const Env& env) {
    std::printf("render arrays\n");
    constexpr uint32_t N = 600;
    const bool shared = env.phys->getInfo().supportsRenderSharedBuffers;

    ephys::DomainDesc desc;
    desc.maxParticles = N;
    ephys::PhysDomainHandle domain = createDomain(env, desc);
    if (!domain)
        return;
    ephys::IParticleState* particles = domain->getParticles();

    const Bytes positions = pattern(N, 16, 31);
    CHECK(particles->upload(nullptr, ParticleArray::Position, env.upload(positions, 16), nullptr) ==
          nvrhi::FS_OK);
    const uint64_t contentBefore = particles->getContentRevision();

    // ---- setArray: the application's buffer is the array ----
    nvrhi::BufferHandle vertexBuffer = createVertexBuffer(env, N + 8, 16, shared, "application positions");
    PhysBufferHandle adopted;
    CHECK(vertexBuffer && env.phys->adoptBuffer(vertexBuffer, N + 8, 16, &adopted) == nvrhi::FS_OK);
    if (!adopted)
        return;
    IPhysBuffer* const ownArray = particles->getArray(ParticleArray::Position);
    nvrhi::BufferHandle ownRhi = ownArray->getRHIBuffer();

    if (shared) {
        std::printf("  setArray: zero copy\n");
        CHECK(particles->setArray(nullptr, ParticleArray::Position, adopted) == nvrhi::FS_OK);
        CHECK(particles->getArray(ParticleArray::Position) == adopted);
        CHECK(particles->getArray(ParticleArray::Position)->getRHIBuffer() == vertexBuffer.Get());
        CHECK(particles->isArraySubstituted(ParticleArray::Position));
        CHECK(!particles->isArraySubstituted(ParticleArray::Velocity));
        CHECK(particles->getLayoutRevision() == 1);
        CHECK(particles->getContentRevision() == contentBefore + 1);
        CHECK(particles->getRenderArray(ParticleArray::Position) == adopted);
        CHECK(particles->areRenderArraysShared());
        // The same buffer again changes nothing.
        CHECK(particles->setArray(nullptr, ParticleArray::Position, adopted) == nvrhi::FS_OK);
        CHECK(particles->getLayoutRevision() == 1);

        // The contents moved into the application's buffer ...
        CHECK(env.fetch(adopted, 0, N) == positions);
        // ... and an upload now lands in it: what the renderer binds is what the simulation wrote.
        const Bytes moved = pattern(N, 16, 32);
        CHECK(particles->upload(nullptr, ParticleArray::Position, env.upload(moved, 16), nullptr) ==
              nvrhi::FS_OK);
        {
            PhysBufferHandle viaRenderer;  // another wrapper of the same buffer, as a renderer would have
            CHECK(env.phys->adoptBuffer(vertexBuffer, N, 16, &viaRenderer) == nvrhi::FS_OK);
            CHECK(env.fetch(viaRenderer, 0, N) == moved);
        }
        // The capacity still limits the array, although the buffer is larger.
        CopyRegion past;
        past.dstOffset = N;
        past.elementCount = 4;
        CHECK_FAILS(
            nvrhi::FE_INVALID_ARGS, env.messages,
            particles->upload(nullptr, ParticleArray::Position, env.upload(pattern(4, 16, 1), 16), &past));
        // No second render buffer for an array that is shared.
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setRenderArray(ParticleArray::Position, adopted));

        // Buffers that cannot be the array.
        nvrhi::BufferHandle smallBuffer = createVertexBuffer(env, N - 1, 16, true, "too small");
        PhysBufferHandle tooSmall;
        CHECK(env.phys->adoptBuffer(smallBuffer, N - 1, 16, &tooSmall) == nvrhi::FS_OK);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setArray(nullptr, ParticleArray::Velocity, adopted));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setArray(nullptr, ParticleArray::Normal, tooSmall));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setArray(nullptr, ParticleArray::Normal, nullptr));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setArray(nullptr, ParticleArray::Normal, env.readback(N, 16)));
        nvrhi::BufferHandle noUavBuffer = createVertexBuffer(env, N, 16, false, "no UAV");
        PhysBufferHandle noUav;
        CHECK(env.phys->adoptBuffer(noUavBuffer, N, 16, &noUav) == nvrhi::FS_OK);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setArray(nullptr, ParticleArray::Normal, noUav));
        CHECK(!particles->isArraySubstituted(ParticleArray::Normal));
        CHECK(particles->getLayoutRevision() == 1);

        // resetArray: an engine buffer again, with the contents.
        CHECK(particles->resetArray(nullptr, ParticleArray::Position) == nvrhi::FS_OK);
        CHECK(!particles->isArraySubstituted(ParticleArray::Position));
        CHECK(particles->getArray(ParticleArray::Position) != adopted);
        CHECK(particles->getArray(ParticleArray::Position)->getRHIBuffer() != vertexBuffer.Get());
        CHECK(particles->getArray(ParticleArray::Position)->getElementCount() == N);
        CHECK(particles->getLayoutRevision() == 2);
        CHECK(particles->getRenderArray(ParticleArray::Position) == nullptr);
        CHECK(env.fetch(particles->getArray(ParticleArray::Position), 0, N) == moved);
        CHECK(particles->resetArray(nullptr, ParticleArray::Position) == nvrhi::FS_OK);  // nothing to do
        CHECK(particles->getLayoutRevision() == 2);
        CHECK(particles->upload(nullptr, ParticleArray::Position, env.upload(positions, 16), nullptr) ==
              nvrhi::FS_OK);
    } else {
        std::printf("  setArray: unsupported on this API\n");
        CHECK_FAILS(nvrhi::FE_UNSUPPORTED, env.messages,
                    particles->setArray(nullptr, ParticleArray::Position, adopted));
        CHECK(particles->getArray(ParticleArray::Position) == ownArray);
        CHECK(!particles->isArraySubstituted(ParticleArray::Position));
        CHECK(particles->getLayoutRevision() == 0);
        CHECK(particles->getContentRevision() == contentBefore);
        CHECK(particles->resetArray(nullptr, ParticleArray::Position) == nvrhi::FS_OK);
    }
    ownRhi = nullptr;

    // ---- The fallback, on every backend: a device-to-device copy into the renderer's buffer ----
    std::printf("  setRenderArray, updateRenderArrays\n");
    {
        const Bytes velocities = pattern(N, 12, 33);
        CHECK(particles->upload(nullptr, ParticleArray::Velocity, env.upload(velocities, 12), nullptr) ==
              nvrhi::FS_OK);

        // Plain vertex buffers, without UAVs: what D3D11 can create.
        nvrhi::BufferHandle positionVB = createVertexBuffer(env, N, 16, false, "renderer positions");
        nvrhi::BufferHandle velocityVB = createVertexBuffer(env, N + 3, 12, false, "renderer velocities");
        PhysBufferHandle positionTarget, velocityTarget;
        CHECK(env.phys->adoptBuffer(positionVB, N, 16, &positionTarget) == nvrhi::FS_OK);
        CHECK(env.phys->adoptBuffer(velocityVB, N + 3, 12, &velocityTarget) == nvrhi::FS_OK);
        if (!positionTarget || !velocityTarget)
            return;

        const uint64_t layout = particles->getLayoutRevision();
        const uint64_t content = particles->getContentRevision();
        CHECK(particles->setRenderArray(ParticleArray::Position, positionTarget) == nvrhi::FS_OK);
        CHECK(!particles->areRenderArraysShared());
        CHECK(particles->setRenderArray(ParticleArray::Velocity, velocityTarget) == nvrhi::FS_OK);
        CHECK(particles->getRenderArray(ParticleArray::Position) == positionTarget);
        CHECK(particles->getRenderArray(ParticleArray::Velocity) == velocityTarget);
        CHECK(particles->getRenderArray(ParticleArray::Phase) == nullptr);
        // The array itself is untouched: the kernels keep their buffer.
        CHECK(particles->getArray(ParticleArray::Position) != positionTarget);
        CHECK(particles->getLayoutRevision() == layout && particles->getContentRevision() == content);

        CHECK(particles->updateRenderArrays(nullptr) == nvrhi::FS_OK);
        CHECK(env.fetch(positionTarget, 0, N) == positions);
        CHECK(env.fetch(velocityTarget, 0, N) == velocities);

        // After the simulation wrote the array again, the next update brings the copy up to date.
        const Bytes stepped = pattern(N, 16, 34);
        CHECK(particles->upload(nullptr, ParticleArray::Position, env.upload(stepped, 16), nullptr) ==
              nvrhi::FS_OK);
        CHECK(env.fetch(positionTarget, 0, N) == positions);  // not yet
        CHECK(particles->updateRenderArrays(nullptr) == nvrhi::FS_OK);
        CHECK(env.fetch(positionTarget, 0, N) == stepped);

        // Refused targets.
        nvrhi::BufferHandle smallVB = createVertexBuffer(env, N - 1, 16, false, "too small");
        PhysBufferHandle tooSmall;
        CHECK(env.phys->adoptBuffer(smallVB, N - 1, 16, &tooSmall) == nvrhi::FS_OK);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setRenderArray(ParticleArray::Normal, tooSmall));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setRenderArray(ParticleArray::Phase, positionTarget));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                    particles->setRenderArray(ParticleArray::Normal, env.upload(pattern(N, 16, 1), 16)));
        CHECK_FAILS(
            nvrhi::FE_INVALID_ARGS, env.messages,
            particles->setRenderArray(ParticleArray::Normal, particles->getArray(ParticleArray::Normal)));

        // Ending it.
        CHECK(particles->setRenderArray(ParticleArray::Position, nullptr) == nvrhi::FS_OK);
        CHECK(particles->setRenderArray(ParticleArray::Velocity, nullptr) == nvrhi::FS_OK);
        CHECK(particles->areRenderArraysShared());
        CHECK(particles->updateRenderArrays(nullptr) == nvrhi::FS_OK);
        CHECK(env.phys->flush() == 0);  // nothing to copy

        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
        CHECK(positionTarget.Detach()->Release() == 0);  // the state let go of it
        CHECK(velocityTarget.Detach()->Release() == 0);
    }

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);
    CHECK(adopted.Detach()->Release() == 0);
}

void testDiffuse(const Env& env) {
    std::printf("diffuse particles\n");
    constexpr uint32_t N = 300;

    ephys::DomainDesc desc;
    desc.maxParticles = 64;
    desc.maxDiffuseParticles = N;
    ephys::PhysDomainHandle domain = createDomain(env, desc);
    if (!domain)
        return;
    ephys::IDiffuseParticleState* diffuse = domain->getDiffuseParticles();
    CHECK(diffuse != nullptr);
    if (!diffuse)
        return;

    CHECK(diffuse->getMaxParticles() == N);
    CHECK(diffuse->getPositions() && diffuse->getPositions()->getElementCount() == N &&
          diffuse->getPositions()->getElementStride() == 16);
    CHECK(diffuse->getVelocities() && diffuse->getVelocities()->getElementStride() == 16);
    CHECK(diffuse->getCountBuffer() && diffuse->getCountBuffer()->getElementCount() == 1 &&
          diffuse->getCountBuffer()->getElementStride() == 4);
    CHECK(diffuse->getContentRevision() == 0);
    CHECK(env.fetch(diffuse->getPositions(), 0, N) == Bytes(size_t(N) * 16, 0));
    CHECK(env.fetchAs<int32_t>(diffuse->getCountBuffer(), 0, 1) == std::vector<int32_t>{0});

    const Bytes positions = pattern(200, 16, 41), velocities = pattern(200, 16, 42);
    PhysBufferHandle p = env.upload(positions, 16), v = env.upload(velocities, 16);
    CHECK(diffuse->upload(nullptr, p, v, 200) == nvrhi::FS_OK);
    CHECK(diffuse->getContentRevision() == 1);

    PhysBufferHandle pOut = env.readback(N, 16), vOut = env.readback(250, 16), countOut = env.readback(1, 4);
    CHECK(diffuse->download(nullptr, pOut, vOut, countOut) == nvrhi::FS_OK);
    CHECK(slice(env.read(pOut), 0, 200, 16) == positions);
    CHECK(slice(env.read(vOut), 0, 200, 16) == velocities);
    int32_t count = -1;
    std::memcpy(&count, env.read(countOut).data(), 4);
    CHECK(count == 200);
    // Null buffers are skipped.
    CHECK(diffuse->download(nullptr, nullptr, nullptr, nullptr) == nvrhi::FS_OK);
    CHECK(env.phys->flush() == 0);

    // Fewer particles: the count is what changes.
    CHECK(diffuse->upload(nullptr, nullptr, nullptr, 0) == nvrhi::FS_OK);
    CHECK(diffuse->getContentRevision() == 2);
    CHECK(env.fetchAs<int32_t>(diffuse->getCountBuffer(), 0, 1) == std::vector<int32_t>{0});

    // Refused.
    PhysBufferHandle tooMany = env.upload(pattern(N + 1, 16, 43), 16);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, diffuse->upload(nullptr, tooMany, tooMany, N + 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                diffuse->upload(nullptr, p, v, 201));  // the buffers hold 200
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, diffuse->upload(nullptr, p, nullptr, 10));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                diffuse->upload(nullptr, p, env.upload(pattern(200, 12, 1), 12), 10));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                diffuse->download(nullptr, p, nullptr, nullptr));  // an Upload buffer
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                diffuse->download(nullptr, nullptr, nullptr, env.readback(1, 16)));
    CHECK(diffuse->getContentRevision() == 2);
    CHECK(env.phys->flush() == 0);
    CHECK(diffuse->markContentChanged() == 3);

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);
}

// ---- Constraint sets ---------------------------------------------------------------------------------------

void testSprings(const Env& env, ephys::ISpringSet* springs) {
    std::printf("  springs\n");
    CHECK(springs->getMaxSprings() == 0 && springs->getNumSprings() == 0);
    CHECK(!springs->getIndices() && !springs->getRestLengths() && !springs->getStiffness());
    CHECK(springs->getLayoutRevision() == 0 && springs->getContentRevision() == 0);

    const Bytes indices = pattern(200, 4, 51), lengths = pattern(100, 4, 52), stiffness = pattern(100, 4, 53);
    PhysBufferHandle i = env.upload(indices, 4), l = env.upload(lengths, 4), k = env.upload(stiffness, 4);
    CHECK(springs->upload(nullptr, i, l, k, 100) == nvrhi::FS_OK);
    CHECK(springs->getNumSprings() == 100);
    CHECK(springs->getMaxSprings() == 150);  // 100 plus half of it: FleX's slack
    CHECK(springs->getLayoutRevision() == 1 && springs->getContentRevision() == 1);
    CHECK(springs->getIndices() && springs->getIndices()->getElementCount() == 300);
    CHECK(springs->getRestLengths() && springs->getRestLengths()->getElementCount() == 150);
    CHECK(springs->getStiffness() && springs->getStiffness()->getElementStride() == 4);

    PhysBufferHandle iOut = env.readback(200, 4), lOut = env.readback(100, 4), kOut = env.readback(100, 4);
    CHECK(springs->download(nullptr, iOut, lOut, kOut, 100) == nvrhi::FS_OK);
    CHECK(env.read(iOut) == indices && env.read(lOut) == lengths && env.read(kOut) == stiffness);
    // Part of the set, into one buffer.
    PhysBufferHandle some = env.readback(30, 4);
    CHECK(springs->download(nullptr, nullptr, some, nullptr, 30) == nvrhi::FS_OK);
    CHECK(env.read(some) == slice(lengths, 0, 30, 4));

    // Within the capacity nothing is replaced.
    nvrhi::BufferHandle sameBuffer = springs->getIndices()->getRHIBuffer();
    CHECK(springs->upload(nullptr, i, l, k, 60) == nvrhi::FS_OK);
    CHECK(springs->getNumSprings() == 60 && springs->getMaxSprings() == 150);
    CHECK(springs->getLayoutRevision() == 1 && springs->getContentRevision() == 2);
    CHECK(springs->getIndices()->getRHIBuffer() == sameBuffer.Get());

    // Beyond it the set grows.
    const Bytes moreIndices = pattern(302, 4, 54), moreLengths = pattern(151, 4, 55),
                moreStiffness = pattern(151, 4, 56);
    PhysBufferHandle mi = env.upload(moreIndices, 4), ml = env.upload(moreLengths, 4),
                     mk = env.upload(moreStiffness, 4);
    CHECK(springs->upload(nullptr, mi, ml, mk, 151) == nvrhi::FS_OK);
    CHECK(springs->getNumSprings() == 151 && springs->getMaxSprings() == 226);
    CHECK(springs->getLayoutRevision() == 2 && springs->getContentRevision() == 3);
    CHECK(springs->getIndices()->getRHIBuffer() != sameBuffer.Get());
    sameBuffer = nullptr;
    CHECK(env.fetch(springs->getIndices(), 0, 302) == moreIndices);
    CHECK(env.fetch(springs->getStiffness(), 0, 151) == moreStiffness);

    // Refused: the set is as before.
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->upload(nullptr, i, l, k, 101));  // l and k hold 100
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->upload(nullptr, l, l, k, 100));  // 2 indices per spring
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, springs->upload(nullptr, i, nullptr, k, 100));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->upload(nullptr, i, l, env.upload(pattern(100, 16, 1), 16), 100));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->upload(nullptr, iOut, l, k, 100));  // a Readback source
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, springs->upload(nullptr, i, l, k, 0x10000000u));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, springs->download(nullptr, iOut, lOut, kOut, 152));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->download(nullptr, iOut, lOut, kOut, 101));  // too small
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                springs->download(nullptr, i, nullptr, nullptr, 10));  // an Upload target
    CHECK(springs->getNumSprings() == 151 && springs->getMaxSprings() == 226);
    CHECK(springs->getLayoutRevision() == 2 && springs->getContentRevision() == 3);
    CHECK(env.phys->flush() == 0);

    // No springs: the buffers stay.
    CHECK(springs->upload(nullptr, nullptr, nullptr, nullptr, 0) == nvrhi::FS_OK);
    CHECK(springs->getNumSprings() == 0 && springs->getMaxSprings() == 226);
    CHECK(springs->getContentRevision() == 4);
    CHECK(springs->markContentChanged() == 5);
}

void testShapeMatches(const Env& env, ephys::IShapeMatchSet* rigids) {
    std::printf("  shape matching\n");
    CHECK(rigids->getMaxRigids() == 0 && rigids->getNumRigids() == 0 && rigids->getMaxIndices() == 0);
    CHECK(!rigids->getOffsets() && !rigids->getRotations() && !rigids->hasPlasticDeformation());

    constexpr uint32_t R = 5, I = 40;
    const Bytes offsets = pattern(R + 1, 4, 61), indices = pattern(I, 4, 62),
                restPositions = pattern(I, 12, 63), restNormals = pattern(I, 16, 64),
                stiffness = pattern(R, 4, 65), thresholds = pattern(R, 4, 66), creeps = pattern(R, 4, 67),
                rotations = pattern(R, 16, 68), translations = pattern(R, 12, 69);

    PhysBufferHandle held[9] = {
        env.upload(offsets, 4),      env.upload(indices, 4),    env.upload(restPositions, 12),
        env.upload(restNormals, 16), env.upload(stiffness, 4),  env.upload(thresholds, 4),
        env.upload(creeps, 4),       env.upload(rotations, 16), env.upload(translations, 12)};
    ephys::ShapeMatchBuffers up;
    up.offsets = held[0];
    up.indices = held[1];
    up.restPositions = held[2];
    up.restNormals = held[3];
    up.stiffness = held[4];
    up.plasticThresholds = held[5];
    up.plasticCreeps = held[6];
    up.rotations = held[7];
    up.translations = held[8];

    CHECK(rigids->upload(nullptr, up, R, I) == nvrhi::FS_OK);
    CHECK(rigids->getNumRigids() == R && rigids->getNumIndices() == I);
    CHECK(rigids->getMaxRigids() == 7 && rigids->getMaxIndices() == 60);
    CHECK(rigids->hasPlasticDeformation());
    CHECK(rigids->getContentRevision() == 1 && rigids->getLayoutRevision() > 0);
    CHECK(rigids->getOffsets()->getElementCount() == 8);  // one more than the rigids
    CHECK(rigids->getRestPositions()->getElementStride() == 12 &&
          rigids->getRestNormals()->getElementStride() == 16);
    CHECK(rigids->getRotations()->getElementStride() == 16 &&
          rigids->getTranslations()->getElementStride() == 12);
    CHECK(rigids->getPlasticThresholds() && rigids->getPlasticCreeps());
    CHECK(rigids->getPlasticThresholds()->getElementCount() == 7);

    PhysBufferHandle out[9] = {env.readback(R + 1, 4), env.readback(I, 4),  env.readback(I, 12),
                               env.readback(I, 16),    env.readback(R, 4),  env.readback(R, 4),
                               env.readback(R, 4),     env.readback(R, 16), env.readback(R, 12)};
    ephys::ShapeMatchBuffers down;
    down.offsets = out[0];
    down.indices = out[1];
    down.restPositions = out[2];
    down.restNormals = out[3];
    down.stiffness = out[4];
    down.plasticThresholds = out[5];
    down.plasticCreeps = out[6];
    down.rotations = out[7];
    down.translations = out[8];
    CHECK(rigids->download(nullptr, down) == nvrhi::FS_OK);
    CHECK(env.read(out[0]) == offsets && env.read(out[1]) == indices && env.read(out[2]) == restPositions);
    CHECK(env.read(out[3]) == restNormals && env.read(out[4]) == stiffness && env.read(out[5]) == thresholds);
    CHECK(env.read(out[6]) == creeps && env.read(out[7]) == rotations && env.read(out[8]) == translations);

    // Only the transforms: what an application reads after a step.
    ephys::ShapeMatchBuffers transforms;
    transforms.rotations = out[7];
    transforms.translations = out[8];
    CHECK(rigids->download(nullptr, transforms) == nvrhi::FS_OK);

    // Without rest normals and plastic coefficients: zero normals, no plastic deformation.
    const uint64_t layout = rigids->getLayoutRevision();
    ephys::ShapeMatchBuffers plain = up;
    plain.restNormals = nullptr;
    plain.plasticThresholds = nullptr;
    plain.plasticCreeps = nullptr;
    CHECK(rigids->upload(nullptr, plain, R, I) == nvrhi::FS_OK);
    CHECK(!rigids->hasPlasticDeformation());
    CHECK(!rigids->getPlasticThresholds() && !rigids->getPlasticCreeps());
    CHECK(rigids->getLayoutRevision() == layout + 1);  // two buffers went away
    CHECK(rigids->getContentRevision() == 2);
    CHECK(env.fetch(rigids->getRestNormals(), 0, I) == Bytes(size_t(I) * 16, 0));
    CHECK(env.fetch(rigids->getRestPositions(), 0, I) == restPositions);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                rigids->download(nullptr, down));  // asks for plastic arrays

    // Refused.
    ephys::ShapeMatchBuffers bad = up;
    bad.plasticCreeps = nullptr;  // one of the two
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, rigids->upload(nullptr, bad, R, I));
    bad = up;
    bad.rotations = nullptr;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, rigids->upload(nullptr, bad, R, I));
    bad = up;
    bad.restPositions = held[3];  // 16-byte elements where 12 are needed
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, rigids->upload(nullptr, bad, R, I));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                rigids->upload(nullptr, up, R + 1, I));  // the buffers hold R
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, rigids->upload(nullptr, up, R, I + 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, rigids->upload(nullptr, up, R, 0));
    CHECK(rigids->getNumRigids() == R && rigids->getContentRevision() == 2 && !rigids->hasPlasticDeformation());
    CHECK(env.phys->flush() == 0);

    // Plastic again, then none.
    CHECK(rigids->upload(nullptr, up, R, I) == nvrhi::FS_OK);
    CHECK(rigids->hasPlasticDeformation() && rigids->getLayoutRevision() == layout + 2);
    CHECK(env.fetch(rigids->getPlasticCreeps(), 0, R) == creeps);
    CHECK(rigids->upload(nullptr, ephys::ShapeMatchBuffers(), 0, 0) == nvrhi::FS_OK);
    CHECK(rigids->getNumRigids() == 0 && rigids->getNumIndices() == 0);
    CHECK(rigids->download(nullptr, down) == nvrhi::FS_OK);  // an empty set: nothing to copy
    CHECK(env.phys->flush() == 0);
}

void testTriangles(const Env& env, ephys::ITriangleSet* triangles) {
    std::printf("  triangles\n");
    CHECK(triangles->getMaxTriangles() == 0 && !triangles->getIndices() && !triangles->getNormals());

    const Bytes indices = pattern(192, 4, 71), normals = pattern(64, 12, 72);
    PhysBufferHandle i = env.upload(indices, 4), n = env.upload(normals, 12);
    CHECK(triangles->upload(nullptr, i, n, 64) == nvrhi::FS_OK);
    CHECK(triangles->getNumTriangles() == 64 && triangles->getMaxTriangles() == 96);
    CHECK(triangles->getIndices()->getElementCount() == 288 &&
          triangles->getNormals()->getElementCount() == 96);
    CHECK(triangles->getNormals()->getElementStride() == 12);
    CHECK(triangles->getLayoutRevision() == 1 && triangles->getContentRevision() == 1);

    PhysBufferHandle iOut = env.readback(192, 4), nOut = env.readback(64, 12);
    CHECK(triangles->download(nullptr, iOut, nOut, 64) == nvrhi::FS_OK);
    CHECK(env.read(iOut) == indices && env.read(nOut) == normals);

    // Without normals, as FleX allows: the indices change, the normals stay what they were.
    const Bytes other = pattern(192, 4, 73);
    CHECK(triangles->upload(nullptr, env.upload(other, 4), nullptr, 64) == nvrhi::FS_OK);
    CHECK(triangles->getContentRevision() == 2 && triangles->getLayoutRevision() == 1);
    CHECK(env.fetch(triangles->getIndices(), 0, 192) == other);
    CHECK(env.fetch(triangles->getNormals(), 0, 64) == normals);

    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, triangles->upload(nullptr, i, n, 65));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, triangles->upload(nullptr, nullptr, n, 64));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, triangles->upload(nullptr, i, i, 48));  // 4-byte normals
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, triangles->download(nullptr, iOut, nOut, 65));
    CHECK(triangles->getNumTriangles() == 64 && triangles->getContentRevision() == 2);
    CHECK(env.phys->flush() == 0);
}

void testInflatables(const Env& env, ephys::IInflatableSet* inflatables) {
    std::printf("  inflatables\n");
    CHECK(inflatables->getMaxInflatables() == 0 && !inflatables->getInflatables());

    ephys::InflatableDesc descs[3];
    descs[0].startTri = 0;
    descs[0].numTris = 120;
    descs[0].restVolume = 2.5f;
    descs[0].overPressure = 1.0f;
    descs[0].constraintScale = 0.25f;
    descs[1].startTri = 120;
    descs[1].numTris = 64;
    descs[1].restVolume = 0.75f;
    descs[1].overPressure = 1.5f;
    descs[1].constraintScale = 0.5f;
    descs[2].startTri = 184;
    descs[2].numTris = 8;
    descs[2].restVolume = 10.0f;
    descs[2].overPressure = 0.5f;
    descs[2].constraintScale = 1.0f;

    CHECK(inflatables->upload(nullptr, descs, 3) == nvrhi::FS_OK);
    CHECK(inflatables->getNumInflatables() == 3 && inflatables->getMaxInflatables() == 4);
    CHECK(inflatables->getInflatables() &&
          inflatables->getInflatables()->getElementStride() == sizeof(ephys::Inflatable));
    CHECK(inflatables->getLayoutRevision() == 1 && inflatables->getContentRevision() == 1);

    PhysBufferHandle out = env.readback(3, sizeof(ephys::Inflatable));
    CHECK(inflatables->download(nullptr, out, 3) == nvrhi::FS_OK);
    const Bytes bytes = env.read(out);
    ephys::Inflatable packed[3];
    std::memcpy(packed, bytes.data(), sizeof(packed));
    for (int i = 0; i < 3; ++i) {
        CHECK(packed[i].startTri == descs[i].startTri && packed[i].numTris == descs[i].numTris);
        // The form the kernels read: the over-pressure is folded into the volume.
        CHECK(packed[i].restVolume == descs[i].restVolume * descs[i].overPressure);
        CHECK(packed[i].constraintScale == descs[i].constraintScale);
    }

    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, inflatables->upload(nullptr, nullptr, 2));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, inflatables->download(nullptr, out, 4));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, inflatables->download(nullptr, nullptr, 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, inflatables->download(nullptr, env.readback(3, 20), 3));
    CHECK(inflatables->getNumInflatables() == 3 && inflatables->getContentRevision() == 1);

    CHECK(inflatables->upload(nullptr, nullptr, 0) == nvrhi::FS_OK);
    CHECK(inflatables->getNumInflatables() == 0 && inflatables->getContentRevision() == 2);
}

void testConstraints(const Env& env) {
    std::printf("constraint sets\n");
    ephys::DomainDesc desc;
    desc.maxParticles = 256;
    ephys::PhysDomainHandle domain = createDomain(env, desc);
    if (!domain)
        return;

    testSprings(env, domain->getSprings());
    testShapeMatches(env, domain->getShapeMatches());
    testTriangles(env, domain->getTriangles());
    testInflatables(env, domain->getInflatables());

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);

    // A domain that reserves: the buffers are there from the start, and the revisions still start at 0.
    std::printf("  reserved capacities\n");
    desc.reserveSprings = 64;
    desc.reserveRigids = 4;
    desc.reserveRigidIndices = 100;
    desc.reserveTriangles = 32;
    desc.reserveInflatables = 2;
    desc.reserveShapes = 8;
    domain = createDomain(env, desc);
    if (!domain)
        return;
    CHECK(domain->getSprings()->getMaxSprings() == 64 && domain->getSprings()->getIndices() != nullptr);
    CHECK(domain->getSprings()->getIndices()->getElementCount() == 128);
    CHECK(domain->getSprings()->getLayoutRevision() == 0);
    CHECK(domain->getShapeMatches()->getMaxRigids() == 4 && domain->getShapeMatches()->getMaxIndices() == 100);
    CHECK(domain->getShapeMatches()->getOffsets()->getElementCount() == 5);
    CHECK(domain->getShapeMatches()->getLayoutRevision() == 0);
    CHECK(domain->getTriangles()->getMaxTriangles() == 32 && domain->getTriangles()->getLayoutRevision() == 0);
    CHECK(domain->getInflatables()->getMaxInflatables() == 2 &&
          domain->getInflatables()->getLayoutRevision() == 0);
    CHECK(domain->getShapes()->getMaxShapes() == 8 && domain->getShapes()->getLayoutRevision() == 0);
    CHECK(domain->getShapes()->getGeometry() && domain->getShapes()->getFlags()->getElementStride() == 4);

    // An upload that fits replaces nothing.
    const Bytes indices = pattern(128, 4, 81), lengths = pattern(64, 4, 82);
    CHECK(domain->getSprings()->upload(nullptr, env.upload(indices, 4), env.upload(lengths, 4),
                                       env.upload(lengths, 4), 64) == nvrhi::FS_OK);
    CHECK(domain->getSprings()->getLayoutRevision() == 0 && domain->getSprings()->getContentRevision() == 1);
    CHECK(env.fetch(domain->getSprings()->getIndices(), 0, 128) == indices);

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);
}

// ---- Bounding volume hierarchies
// -----------------------------------------------------------------------------

struct Float4 {
    float x, y, z, w;
};
static_assert(sizeof(Float4) == 16);

// Checks a hierarchy over numItems boxes: every item is in exactly one leaf, the leaf has the box of its
// item, every internal node has exactly the union of its children, and 2n - 1 nodes hang under the root.
bool validateBvh(const std::vector<BvhNodeHalf>& lowers, const std::vector<BvhNodeHalf>& uppers, int32_t root,
                 const std::vector<Float4>& itemLowers, const std::vector<Float4>& itemUppers,
                 uint32_t* pDepth) {
    const uint32_t n = uint32_t(itemLowers.size());
    if (lowers.size() < 2 * n - 1 || uppers.size() < 2 * n - 1)
        return false;
    if (root < 0 || uint32_t(root) >= 2 * n - 1)
        return false;

    std::vector<uint8_t> nodeSeen(2 * n, 0), itemSeen(n, 0);
    struct Entry {
        uint32_t node;
        uint32_t depth;
    };
    std::vector<Entry> stack;
    stack.push_back({uint32_t(root), 1});
    uint32_t visited = 0, leaves = 0, depth = 0;
    while (!stack.empty()) {
        const Entry entry = stack.back();
        stack.pop_back();
        const uint32_t node = entry.node;
        if (node >= 2 * n - 1 || nodeSeen[node])
            return false;
        nodeSeen[node] = 1;
        ++visited;
        depth = std::max(depth, entry.depth);

        const BvhNodeHalf& lower = lowers[node];
        const BvhNodeHalf& upper = uppers[node];
        if (lower.index & ephys::c_BvhLeafBit) {
            // A leaf: one of the first n nodes, naming its item in both halves.
            if (node >= n)
                return false;
            const uint32_t item = upper.index;
            if (item >= n || (lower.index & ~ephys::c_BvhLeafBit) != item || itemSeen[item])
                return false;
            itemSeen[item] = 1;
            ++leaves;
            // Compared as floats: the minimum of 0 and -0 is either, on the GPU as on the host.
            const float itemLower[3] = {itemLowers[item].x, itemLowers[item].y, itemLowers[item].z};
            const float itemUpper[3] = {itemUppers[item].x, itemUppers[item].y, itemUppers[item].z};
            for (int k = 0; k < 3; ++k)
                if (lower.corner[k] != itemLower[k] || upper.corner[k] != itemUpper[k])
                    return false;
        } else {
            if (node < n)
                return false;
            const uint32_t left = lower.index, right = upper.index;
            if (left >= 2 * n - 1 || right >= 2 * n - 1 || left == right)
                return false;
            for (int k = 0; k < 3; ++k) {
                if (lower.corner[k] != std::min(lowers[left].corner[k], lowers[right].corner[k]))
                    return false;
                if (upper.corner[k] != std::max(uppers[left].corner[k], uppers[right].corner[k]))
                    return false;
            }
            stack.push_back({left, entry.depth + 1});
            stack.push_back({right, entry.depth + 1});
        }
    }
    if (pDepth)
        *pDepth = depth;
    return visited == 2 * n - 1 && leaves == n;
}

struct Mesh {
    std::vector<Float4> vertices;
    std::vector<int32_t> indices;
    ephys::Bounds3 bounds;

    uint32_t numTriangles() const { return uint32_t(indices.size() / 3); }

    void computeBounds() {
        for (int k = 0; k < 3; ++k) {
            bounds.lower[k] = 3.0e38f;
            bounds.upper[k] = -3.0e38f;
        }
        for (const Float4& v : vertices) {
            const float p[3] = {v.x, v.y, v.z};
            for (int k = 0; k < 3; ++k) {
                bounds.lower[k] = std::min(bounds.lower[k], p[k]);
                bounds.upper[k] = std::max(bounds.upper[k], p[k]);
            }
        }
    }

    // The boxes of the triangles, as ComputeTriangleBounds writes them.
    void triangleBounds(std::vector<Float4>& lowers, std::vector<Float4>& uppers) const {
        lowers.resize(numTriangles());
        uppers.resize(numTriangles());
        for (uint32_t t = 0; t < numTriangles(); ++t) {
            Float4 lower = {3.0e38f, 3.0e38f, 3.0e38f, 0.0f}, upper = {-3.0e38f, -3.0e38f, -3.0e38f, 0.0f};
            for (int c = 0; c < 3; ++c) {
                const Float4& v = vertices[size_t(indices[size_t(t) * 3 + c])];
                lower.x = std::min(lower.x, v.x);
                lower.y = std::min(lower.y, v.y);
                lower.z = std::min(lower.z, v.z);
                upper.x = std::max(upper.x, v.x);
                upper.y = std::max(upper.y, v.y);
                upper.z = std::max(upper.z, v.z);
            }
            lowers[t] = lower;
            uppers[t] = upper;
        }
    }
};

// A wavy sheet of quadsX x quadsY quads: 2 * quadsX * quadsY triangles.
Mesh makeSheet(uint32_t quadsX, uint32_t quadsY, float scale, float offset) {
    Mesh mesh;
    for (uint32_t y = 0; y <= quadsY; ++y) {
        for (uint32_t x = 0; x <= quadsX; ++x) {
            const float fx = float(x) / float(quadsX), fy = float(y) / float(quadsY);
            mesh.vertices.push_back({offset + scale * fx,
                                     scale * 0.2f * std::sin(7.0f * fx) * std::cos(5.0f * fy),
                                     offset + scale * fy, 0.0f});
        }
    }
    for (uint32_t y = 0; y < quadsY; ++y) {
        for (uint32_t x = 0; x < quadsX; ++x) {
            const int32_t a = int32_t(y * (quadsX + 1) + x), b = a + 1, c = a + int32_t(quadsX + 1), d = c + 1;
            const int32_t quad[6] = {a, b, c, b, d, c};
            mesh.indices.insert(mesh.indices.end(), quad, quad + 6);
        }
    }
    mesh.computeBounds();
    return mesh;
}

Mesh makeTriangles(uint32_t count) {
    Mesh mesh;
    for (uint32_t t = 0; t < count; ++t) {
        const float base = float(t) * 2.0f;
        mesh.vertices.push_back({base, 0.0f, 0.0f, 0.0f});
        mesh.vertices.push_back({base + 1.0f, 0.5f, 0.0f, 0.0f});
        mesh.vertices.push_back({base, 1.0f, 1.0f + float(t), 0.0f});
        for (int c = 0; c < 3; ++c)
            mesh.indices.push_back(int32_t(t * 3 + c));
    }
    mesh.computeBounds();
    return mesh;
}

nvrhi::FRESULT updateMesh(const Env& env, ephys::ITriangleMeshAsset* asset, const Mesh& mesh) {
    return asset->update(nullptr, env.upload(mesh.vertices, 16), env.upload(mesh.indices, 4),
                         uint32_t(mesh.vertices.size()), mesh.numTriangles(), mesh.bounds);
}

// Reads the hierarchy of a mesh asset back and validates it against the mesh.
void checkMeshAsset(const Env& env, ephys::ITriangleMeshAsset* asset, const Mesh& mesh, const char* what) {
    const uint32_t n = mesh.numTriangles();
    CHECK(asset->getNumTriangles() == n && asset->getNumVertices() == mesh.vertices.size());
    CHECK(asset->getNumBvhNodes() == 2 * n);
    CHECK(std::memcmp(&asset->getBounds(), &mesh.bounds, sizeof(ephys::Bounds3)) == 0);
    CHECK(asset->getVertices() && asset->getIndices() && asset->getBvhNodeLowers() &&
          asset->getBvhNodeUppers() && asset->getBvhRoot());
    if (!asset->getBvhRoot())
        return;

    CHECK(env.fetchAs<Float4>(asset->getVertices(), 0, uint32_t(mesh.vertices.size())).size() ==
          mesh.vertices.size());
    const std::vector<int32_t> indices = env.fetchAs<int32_t>(asset->getIndices(), 0, 3 * n);
    CHECK(indices == mesh.indices);

    const std::vector<BvhNodeHalf> lowers = env.fetchAs<BvhNodeHalf>(asset->getBvhNodeLowers(), 0, 2 * n);
    const std::vector<BvhNodeHalf> uppers = env.fetchAs<BvhNodeHalf>(asset->getBvhNodeUppers(), 0, 2 * n);
    const std::vector<int32_t> root = env.fetchAs<int32_t>(asset->getBvhRoot(), 0, 1);
    std::vector<Float4> itemLowers, itemUppers;
    mesh.triangleBounds(itemLowers, itemUppers);
    uint32_t depth = 0;
    const bool valid = root.size() == 1 && validateBvh(lowers, uppers, root[0], itemLowers, itemUppers, &depth);
    CHECK(valid);
    std::printf("    %s: %u triangles, %u nodes, depth %u: %s\n", what, n, 2 * n - 1, depth,
                valid ? "valid" : "INVALID");
}

// ---- Geometry assets
// -----------------------------------------------------------------------------------------

std::vector<uint32_t> ids(ephys::IGeometryLibrary* library, int kind) {
    uint32_t buffer[64] = {};
    const uint32_t count = kind == 0   ? library->getTriangleMeshIds(buffer, 64)
                           : kind == 1 ? library->getSdfIds(buffer, 64)
                                       : library->getConvexIds(buffer, 64);
    return std::vector<uint32_t>(buffer, buffer + std::min(count, 64u));
}

void testConvexes(const Env& env, ephys::IGeometryLibrary* library) {
    std::printf("  convex meshes\n");
    CHECK(ids(library, 2).empty());
    CHECK(library->getConvexTable() &&
          library->getConvexTable()->getElementCount() == ephys::c_MaxConvexAssets &&
          library->getConvexTable()->getElementStride() == sizeof(ephys::ConvexMeshEntry));
    CHECK(library->getConvexPlanes() &&
          library->getConvexPlanes()->getElementCount() == ephys::c_MaxConvexAssets * ephys::c_MaxConvexPlanes);

    // Ids: 1, 2, 3, then the slot freed last first (FleX's free-list stack).
    ephys::ConvexAssetHandle a, b, c;
    CHECK(library->createConvex(&a) == nvrhi::FS_OK && library->createConvex(&b) == nvrhi::FS_OK &&
          library->createConvex(&c) == nvrhi::FS_OK);
    if (!a || !b || !c)
        return;
    CHECK(a->getId() == 1 && b->getId() == 2 && c->getId() == 3);
    CHECK(ids(library, 2) == (std::vector<uint32_t>{1, 2, 3}));
    CHECK(library->getConvexIds(nullptr, 0) == 3);
    CHECK(b.Detach()->Release() == 0);
    CHECK(ids(library, 2) == (std::vector<uint32_t>{1, 3}));
    CHECK(library->createConvex(&b) == nvrhi::FS_OK && b->getId() == 2);
    CHECK(a.Detach()->Release() == 0);
    CHECK(c.Detach()->Release() == 0);
    CHECK(library->createConvex(&c) == nvrhi::FS_OK && c->getId() == 3);  // freed last, handed out first
    CHECK(library->createConvex(&a) == nvrhi::FS_OK && a->getId() == 1);

    // Lookup by id.
    ephys::ConvexAssetHandle found;
    CHECK(library->getConvex(2, &found) == nvrhi::FS_OK && found == b);
    CHECK(library->getConvex(0, &found) == nvrhi::FE_NOT_FOUND && found == nullptr);
    CHECK(library->getConvex(4, &found) == nvrhi::FE_NOT_FOUND);
    CHECK(library->getConvex(ephys::c_MaxConvexAssets + 1, &found) == nvrhi::FE_NOT_FOUND);

    // A box of six planes, into slot 2.
    const uint64_t content = library->getContentRevision();
    const std::vector<Float4> planes = {
        {1,  0,  0,  -1},
        {-1, 0,  0,  -1},
        {0,  1,  0,  -2},
        {0,  -1, 0,  -2},
        {0,  0,  1,  -3},
        {0,  0,  -1, -3}
    };
    ephys::Bounds3 bounds;
    bounds.lower[0] = -1.0f;
    bounds.lower[1] = -2.0f;
    bounds.lower[2] = -3.0f;
    bounds.upper[0] = 1.0f;
    bounds.upper[1] = 2.0f;
    bounds.upper[2] = 3.0f;
    CHECK(b->getNumPlanes() == 0);
    CHECK(b->update(nullptr, env.upload(planes, 16), 6, bounds) == nvrhi::FS_OK);
    CHECK(b->getNumPlanes() == 6);
    CHECK(std::memcmp(&b->getBounds(), &bounds, sizeof(bounds)) == 0);
    CHECK(library->getContentRevision() == content + 1);

    const uint32_t slot = b->getId() - 1;
    const std::vector<Float4> stored =
        env.fetchAs<Float4>(library->getConvexPlanes(), slot * ephys::c_MaxConvexPlanes, 6);
    CHECK(stored.size() == 6 && std::memcmp(stored.data(), planes.data(), 6 * sizeof(Float4)) == 0);
    const std::vector<ephys::ConvexMeshEntry> entry =
        env.fetchAs<ephys::ConvexMeshEntry>(library->getConvexTable(), slot, 1);
    CHECK(entry.size() == 1);
    if (entry.size() == 1) {
        CHECK(std::memcmp(entry[0].lower, bounds.lower, 12) == 0 &&
              std::memcmp(entry[0].upper, bounds.upper, 12) == 0);
        CHECK(entry[0].planeOffset == int32_t(slot * ephys::c_MaxConvexPlanes) && entry[0].planeCount == 6);
    }

    // Refused.
    const std::vector<Float4> many(ephys::c_MaxConvexPlanes + 1, Float4{0, 1, 0, 0});
    PhysBufferHandle manyPlanes = env.upload(many, 16);
    CHECK(b->update(nullptr, manyPlanes, ephys::c_MaxConvexPlanes, bounds) ==
          nvrhi::FS_OK);  // the limit itself
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                b->update(nullptr, manyPlanes, ephys::c_MaxConvexPlanes + 1, bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, b->update(nullptr, nullptr, 3, bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                b->update(nullptr, env.upload(pattern(8, 12, 1), 12), 6, bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, b->update(nullptr, env.upload(planes, 16), 7, bounds));
    CHECK(b->getNumPlanes() == ephys::c_MaxConvexPlanes);

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(a.Detach()->Release() == 0);
    found = nullptr;
    CHECK(b.Detach()->Release() == 0);
    CHECK(c.Detach()->Release() == 0);
    CHECK(ids(library, 2).empty());
}

// Reads a 3D R32_FLOAT texture back, slice by slice.
std::vector<float> readVolume(const Env& env, nvrhi::ITexture* texture) {
    const nvrhi::TextureDesc& desc = texture->getDesc();
    std::vector<float> values(size_t(desc.width) * desc.height * desc.depth, -1.0f);

    nvrhi::StagingTextureHandle staging;
    CHECK(env.device->createStagingTexture(desc, nvrhi::CpuAccessMode::Read, &staging) == nvrhi::FS_OK);
    nvrhi::CommandListHandle commandList;
    CHECK(env.device->createCommandList(nvrhi::CommandListParameters(), &commandList) == nvrhi::FS_OK);
    if (!staging || !commandList)
        return values;

    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);  // the upload was recorded on the engine's list
    commandList->open();
    for (uint32_t z = 0; z < desc.depth; ++z) {
        nvrhi::TextureSlice slice;
        slice.setOrigin(0, 0, z).setDepth(1);
        commandList->copyTexture2(staging, slice, texture, slice);
    }
    commandList->close();
    env.device->executeCommandList(commandList);
    env.device->waitForIdle();

    size_t rowPitch = 0;
    const uint8_t* data = static_cast<const uint8_t*>(
        env.device->mapStagingTexture(staging, nvrhi::TextureSlice(), nvrhi::CpuAccessMode::Read, rowPitch));
    CHECK(data != nullptr);
    if (!data)
        return values;
    for (uint32_t z = 0; z < desc.depth; ++z)
        for (uint32_t y = 0; y < desc.height; ++y)
            std::memcpy(&values[(size_t(z) * desc.height + y) * desc.width],
                        data + (size_t(z) * desc.height + y) * rowPitch, size_t(desc.width) * sizeof(float));
    env.device->unmapStagingTexture(staging);
    return values;
}

void testSdfs(const Env& env, ephys::IGeometryLibrary* library) {
    std::printf("  distance fields\n");
    CHECK(ids(library, 1).empty());
    CHECK(library->getSdfTable() && library->getSdfTable()->getElementCount() == ephys::c_MaxSdfAssets &&
          library->getSdfTable()->getElementStride() == sizeof(ephys::SdfEntry));

    // All sixteen slots, then one too many.
    std::vector<ephys::SdfAssetHandle> all(ephys::c_MaxSdfAssets);
    for (uint32_t i = 0; i < ephys::c_MaxSdfAssets; ++i) {
        CHECK(library->createSdf(&all[i]) == nvrhi::FS_OK);
        CHECK(all[i] && all[i]->getId() == i + 1);
    }
    ephys::SdfAssetHandle extra;
    CHECK_FAILS(nvrhi::FE_OUT_OF_MEMORY, env.messages, library->createSdf(&extra));
    CHECK(extra == nullptr);
    CHECK(library->getSdfIds(nullptr, 0) == ephys::c_MaxSdfAssets);
    // Freed last, handed out first.
    CHECK(all[4].Detach()->Release() == 0);
    CHECK(all[9].Detach()->Release() == 0);
    CHECK(library->createSdf(&extra) == nvrhi::FS_OK && extra->getId() == 10);
    CHECK(extra.Detach()->Release() == 0);
    for (auto& sdf : all)
        if (sdf)
            CHECK(sdf.Detach()->Release() == 0);
    CHECK(ids(library, 1).empty());

    // A small volume with another size in every dimension.
    constexpr uint32_t W = 8, H = 6, D = 4;
    std::vector<float> field(W * H * D);
    for (uint32_t z = 0; z < D; ++z)
        for (uint32_t y = 0; y < H; ++y)
            for (uint32_t x = 0; x < W; ++x)
                field[(z * H + y) * W + x] = float(x) + 10.0f * float(y) + 100.0f * float(z) - 0.5f;

    ephys::SdfAssetHandle sdf;
    CHECK(library->createSdf(&sdf) == nvrhi::FS_OK);
    if (!sdf)
        return;
    const uint32_t slot = sdf->getId() - 1;
    CHECK(sdf->getTexture() == nullptr && sdf->getWidth() == 0);
    CHECK(library->getSdfTexture(slot) == nullptr);

    const uint64_t layout = library->getLayoutRevision(), content = library->getContentRevision();
    CHECK(sdf->update(nullptr, field.data(), W, H, D) == nvrhi::FS_OK);
    CHECK(sdf->getWidth() == W && sdf->getHeight() == H && sdf->getDepth() == D);
    CHECK(library->getLayoutRevision() == layout + 1 && library->getContentRevision() == content + 1);
    nvrhi::ITexture* texture = sdf->getTexture();
    CHECK(texture != nullptr);
    if (!texture)
        return;
    CHECK(library->getSdfTexture(slot) == texture);
    CHECK(library->getSdfTexture(ephys::c_MaxSdfAssets) == nullptr);
    const nvrhi::TextureDesc& desc = texture->getDesc();
    CHECK(desc.dimension == nvrhi::TextureDimension::Texture3D && desc.format == nvrhi::Format::R32_FLOAT);
    CHECK(desc.width == W && desc.height == H && desc.depth == D);

    CHECK(readVolume(env, texture) == field);
    const std::vector<ephys::SdfEntry> entry = env.fetchAs<ephys::SdfEntry>(library->getSdfTable(), slot, 1);
    CHECK(entry.size() == 1);
    if (entry.size() == 1) {
        CHECK(entry[0].dim[0] == int32_t(W) && entry[0].dim[1] == int32_t(H) && entry[0].dim[2] == int32_t(D));
        CHECK(entry[0].invDim[0] == 1.0f / float(W) && entry[0].invDim[1] == 1.0f / float(H) &&
              entry[0].invDim[2] == 1.0f / float(D));
    }

    // A second update replaces the texture.
    nvrhi::TextureHandle first = texture;
    std::vector<float> cube(27, 0.25f);
    CHECK(sdf->update(nullptr, cube.data(), 3, 3, 3) == nvrhi::FS_OK);
    CHECK(sdf->getTexture() != first.Get() && sdf->getWidth() == 3);
    CHECK(library->getLayoutRevision() == layout + 2);
    CHECK(readVolume(env, sdf->getTexture()) == cube);
    first = nullptr;

    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, sdf->update(nullptr, nullptr, 3, 3, 3));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, sdf->update(nullptr, cube.data(), 3, 0, 3));
    CHECK(sdf->getWidth() == 3);

    // Releasing the field takes its texture out of the library.
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(sdf.Detach()->Release() == 0);
    CHECK(library->getSdfTexture(slot) == nullptr);
    CHECK(library->getLayoutRevision() == layout + 3);
}

void testTriangleMeshes(const Env& env, ephys::IGeometryLibrary* library) {
    std::printf("  triangle meshes\n");
    CHECK(ids(library, 0).empty());
    CHECK(!library->needsCommit());
    CHECK(library->getTriangleMeshTable() == nullptr && library->getTriangleVertices() == nullptr);

    // Ids: the lowest free one (FleX's first free table slot).
    ephys::TriangleMeshAssetHandle a, b, c, d;
    CHECK(library->createTriangleMesh(&a) == nvrhi::FS_OK && library->createTriangleMesh(&b) == nvrhi::FS_OK &&
          library->createTriangleMesh(&c) == nvrhi::FS_OK);
    if (!a || !b || !c)
        return;
    CHECK(a->getId() == 1 && b->getId() == 2 && c->getId() == 3);
    CHECK(a.Detach()->Release() == 0);
    CHECK(b.Detach()->Release() == 0);
    CHECK(ids(library, 0) == (std::vector<uint32_t>{3}));
    CHECK(library->createTriangleMesh(&a) == nvrhi::FS_OK && a->getId() == 1);  // not 2, although freed last
    CHECK(library->createTriangleMesh(&b) == nvrhi::FS_OK && b->getId() == 2);
    CHECK(library->createTriangleMesh(&d) == nvrhi::FS_OK && d->getId() == 4);
    ephys::TriangleMeshAssetHandle found;
    CHECK(library->getTriangleMesh(3, &found) == nvrhi::FS_OK && found == c);
    CHECK(library->getTriangleMesh(5, &found) == nvrhi::FE_NOT_FOUND && found == nullptr);
    CHECK(library->needsCommit());

    // A new mesh is empty.
    CHECK(a->getNumTriangles() == 0 && a->getNumBvhNodes() == 0 && a->getVertices() == nullptr &&
          a->getBvhRoot() == nullptr);

    // Four meshes: one sort block, several sort blocks, two triangles, one triangle.
    const Mesh sheet = makeSheet(16, 16, 4.0f, -2.0f);  // 512 triangles
    const Mesh large = makeSheet(48, 40, 10.0f, 1.0f);  // 3840 triangles: four sort blocks
    const Mesh pair = makeTriangles(2);
    const Mesh single = makeTriangles(1);
    CHECK(updateMesh(env, a, sheet) == nvrhi::FS_OK);
    CHECK(updateMesh(env, b, large) == nvrhi::FS_OK);
    CHECK(updateMesh(env, c, pair) == nvrhi::FS_OK);
    CHECK(updateMesh(env, d, single) == nvrhi::FS_OK);
    checkMeshAsset(env, a, sheet, "sheet");
    checkMeshAsset(env, b, large, "large sheet");
    checkMeshAsset(env, c, pair, "two triangles");
    checkMeshAsset(env, d, single, "one triangle");

    // Updating in place: a smaller mesh in the buffers of the larger one, then a larger one again.
    nvrhi::BufferHandle sameVertices = b->getVertices()->getRHIBuffer();
    CHECK(updateMesh(env, b, sheet) == nvrhi::FS_OK);
    CHECK(b->getVertices()->getRHIBuffer() == sameVertices.Get());
    checkMeshAsset(env, b, sheet, "sheet in the buffers of the large one");
    sameVertices = nullptr;
    CHECK(updateMesh(env, c, large) == nvrhi::FS_OK);
    checkMeshAsset(env, c, large, "large sheet where two triangles were");

    // Refused updates leave the mesh as it was.
    PhysBufferHandle vertices = env.upload(sheet.vertices, 16), indices = env.upload(sheet.indices, 4);
    const uint32_t nv = uint32_t(sheet.vertices.size()), nt = sheet.numTriangles();
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                a->update(nullptr, nullptr, indices, nv, nt, sheet.bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                a->update(nullptr, vertices, indices, nv + 1, nt, sheet.bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                a->update(nullptr, vertices, indices, nv, nt + 1, sheet.bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                a->update(nullptr, indices, indices, nv, nt, sheet.bounds));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                a->update(nullptr, vertices, vertices, nv, nt, sheet.bounds));
    CHECK(a->getNumTriangles() == nt);

    // commit(): the meshes one behind the other.
    std::printf("  commit\n");
    CHECK(library->needsCommit());
    const uint64_t layout = library->getLayoutRevision();
    CHECK(library->commit(nullptr) == nvrhi::FS_OK);
    CHECK(!library->needsCommit());
    CHECK(library->getLayoutRevision() == layout + 1);  // the arrays appeared
    const uint64_t content = library->getContentRevision();
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(library->commit(nullptr) == nvrhi::FS_OK);  // nothing to do
    CHECK(env.phys->flush() == 0);
    CHECK(library->getContentRevision() == content);

    CHECK(library->getTriangleMeshTable() &&
          library->getTriangleMeshTable()->getElementStride() == sizeof(ephys::TriangleMeshEntry));
    CHECK(library->getTriangleVertices() && library->getTriangleIndices() &&
          library->getTriangleBvhNodeLowers() && library->getTriangleBvhNodeUppers() &&
          library->getTriangleBvhRoots());
    if (!library->getTriangleBvhRoots())
        return;

    struct Slot {
        ephys::ITriangleMeshAsset* asset;
        const Mesh* mesh;
    };
    const Slot slots[4] = {
        {a, &sheet },
        {b, &sheet },
        {c, &large },
        {d, &single}
    };
    const std::vector<ephys::TriangleMeshEntry> table =
        env.fetchAs<ephys::TriangleMeshEntry>(library->getTriangleMeshTable(), 0, 4);
    const std::vector<int32_t> roots = env.fetchAs<int32_t>(library->getTriangleBvhRoots(), 0, 4);
    CHECK(table.size() == 4 && roots.size() == 4);
    if (table.size() != 4 || roots.size() != 4)
        return;
    int32_t vertexStart = 0, indexStart = 0, nodeStart = 0;
    for (int i = 0; i < 4; ++i) {
        const Mesh& mesh = *slots[i].mesh;
        const uint32_t n = mesh.numTriangles();
        CHECK(slots[i].asset->getId() == uint32_t(i + 1));
        CHECK(table[i].vertexStart == vertexStart && table[i].indexStart == indexStart &&
              table[i].nodeStart == nodeStart);
        CHECK(std::memcmp(table[i].lower, mesh.bounds.lower, 12) == 0 &&
              std::memcmp(table[i].upper, mesh.bounds.upper, 12) == 0);

        const std::vector<Float4> flatVertices = env.fetchAs<Float4>(
            library->getTriangleVertices(), uint32_t(vertexStart), uint32_t(mesh.vertices.size()));
        CHECK(flatVertices.size() == mesh.vertices.size() &&
              std::memcmp(flatVertices.data(), mesh.vertices.data(), mesh.vertices.size() * sizeof(Float4)) ==
                  0);
        CHECK(env.fetchAs<int32_t>(library->getTriangleIndices(), uint32_t(indexStart), 3 * n) == mesh.indices);

        // The hierarchy of the mesh, as the collision kernel finds it: nodeStart + the root of the mesh.
        const std::vector<BvhNodeHalf> lowers =
            env.fetchAs<BvhNodeHalf>(library->getTriangleBvhNodeLowers(), uint32_t(nodeStart), 2 * n);
        const std::vector<BvhNodeHalf> uppers =
            env.fetchAs<BvhNodeHalf>(library->getTriangleBvhNodeUppers(), uint32_t(nodeStart), 2 * n);
        std::vector<Float4> itemLowers, itemUppers;
        mesh.triangleBounds(itemLowers, itemUppers);
        CHECK(validateBvh(lowers, uppers, roots[i], itemLowers, itemUppers, nullptr));
        CHECK(env.fetchAs<int32_t>(slots[i].asset->getBvhRoot(), 0, 1) == std::vector<int32_t>{roots[i]});

        vertexStart += int32_t(mesh.vertices.size());
        indexStart += int32_t(3 * n);
        nodeStart += int32_t(2 * n);
    }

    // An empty update and a release both make the arrays stale; the next commit packs what is left.
    CHECK(b->update(nullptr, nullptr, nullptr, 0, 0, sheet.bounds) == nvrhi::FS_OK);
    CHECK(b->getNumTriangles() == 0 && b->getNumBvhNodes() == 0 && library->needsCommit());
    CHECK(library->commit(nullptr) == nvrhi::FS_OK);
    {
        const std::vector<ephys::TriangleMeshEntry> packed =
            env.fetchAs<ephys::TriangleMeshEntry>(library->getTriangleMeshTable(), 0, 4);
        CHECK(packed.size() == 4 && packed[2].vertexStart == int32_t(sheet.vertices.size()) &&
              packed[2].nodeStart == int32_t(2 * sheet.numTriangles()));
    }
    found = nullptr;
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(c.Detach()->Release() == 0);
    CHECK(library->needsCommit());
    CHECK(library->commit(nullptr) == nvrhi::FS_OK);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);

    CHECK(a.Detach()->Release() == 0);
    CHECK(b.Detach()->Release() == 0);
    CHECK(d.Detach()->Release() == 0);
    CHECK(ids(library, 0).empty());
    CHECK(library->commit(nullptr) == nvrhi::FS_OK);
    CHECK(!library->needsCommit());
}

// ---- Shapes
// --------------------------------------------------------------------------------------------------

void testShapes(const Env& env) {
    std::printf("collision shapes\n");
    ephys::IGeometryLibrary* library = env.phys->getGeometry();

    ephys::DomainDesc desc;
    desc.maxParticles = 64;
    ephys::PhysDomainHandle domain = createDomain(env, desc);
    if (!domain)
        return;
    ephys::ICollisionShapeSet* shapes = domain->getShapes();
    CHECK(shapes->getGeometryLibrary() == library);
    CHECK(shapes->getMaxShapes() == 0 && shapes->getNumShapes() == 0 && !shapes->getGeometry());

    // One asset of each kind. The convex takes id 2, so that an id is not just a count.
    ephys::ConvexAssetHandle spare, convex;
    ephys::TriangleMeshAssetHandle mesh;
    ephys::SdfAssetHandle sdf;
    CHECK(library->createConvex(&spare) == nvrhi::FS_OK && library->createConvex(&convex) == nvrhi::FS_OK);
    CHECK(library->createTriangleMesh(&mesh) == nvrhi::FS_OK && library->createSdf(&sdf) == nvrhi::FS_OK);
    if (!convex || !mesh || !sdf)
        return;
    const uint32_t convexId = convex->getId(), meshId = mesh->getId(), sdfId = sdf->getId();
    CHECK(convexId == 2);

    constexpr uint32_t S = 6;
    ephys::CollisionGeometry geometry[S];
    std::memset(geometry, 0, sizeof(geometry));
    geometry[0].sphere.radius = 0.5f;
    geometry[1].capsule.radius = 0.25f;
    geometry[1].capsule.halfHeight = 1.5f;
    geometry[2].box.halfExtents[0] = 1.0f;
    geometry[2].box.halfExtents[1] = 2.0f;
    geometry[2].box.halfExtents[2] = 3.0f;
    geometry[3].convexMesh.scale[0] = geometry[3].convexMesh.scale[1] = geometry[3].convexMesh.scale[2] = 2.0f;
    geometry[3].convexMesh.mesh = convexId;
    geometry[4].triMesh.scale[0] = geometry[4].triMesh.scale[1] = geometry[4].triMesh.scale[2] = 1.0f;
    geometry[4].triMesh.mesh = meshId;
    geometry[5].sdf.scale = 4.0f;
    geometry[5].sdf.field = sdfId;
    // The id of a convex is where the kernels read it: the fourth float of the entry; that of a field the
    // second.
    CHECK(std::memcmp(&geometry[3].raw[3], &convexId, 4) == 0 &&
          std::memcmp(&geometry[5].raw[1], &sdfId, 4) == 0);

    const int32_t flags[S] = {
        ephys::makeShapeFlags(ephys::CollisionShapeType::Sphere, false),
        ephys::makeShapeFlags(ephys::CollisionShapeType::Capsule, true),
        ephys::makeShapeFlagsWithChannels(ephys::CollisionShapeType::Box, false, ephys::PhaseShapeChannel0),
        ephys::makeShapeFlags(ephys::CollisionShapeType::ConvexMesh, true) | ephys::ShapeFlagTrigger,
        ephys::makeShapeFlags(ephys::CollisionShapeType::TriangleMesh, false),
        ephys::makeShapeFlags(ephys::CollisionShapeType::Sdf, false),
    };
    std::vector<float> positions(S * 4), rotations(S * 4), prevPositions(S * 4), prevRotations(S * 4);
    for (uint32_t i = 0; i < S * 4; ++i) {
        positions[i] = float(i) * 0.5f;
        rotations[i] = (i % 4 == 3) ? 1.0f : 0.0f;
        prevPositions[i] = float(i) * 0.5f - 0.125f;
        prevRotations[i] = (i % 4 == 3) ? 0.875f : 0.125f;
    }

    ephys::CollisionShapeData data;
    data.geometry = geometry;
    data.positions = positions.data();
    data.rotations = rotations.data();
    data.prevPositions = prevPositions.data();
    data.prevRotations = prevRotations.data();
    data.flags = flags;
    data.numShapes = S;

    const nvrhi::FLONG convexRefs = refCount(convex.Get());
    CHECK(shapes->upload(nullptr, data) == nvrhi::FS_OK);
    CHECK(shapes->getNumShapes() == S && shapes->getMaxShapes() == 9);
    CHECK(shapes->getLayoutRevision() == 1 && shapes->getContentRevision() == 1);
    CHECK(shapes->getGeometry()->getElementStride() == 16 && shapes->getFlags()->getElementStride() == 4);
    CHECK(refCount(convex.Get()) == convexRefs + 1);  // the set holds what its shapes name
    CHECK(refCount(spare.Get()) == 1);                // and nothing else

    PhysBufferHandle out[6] = {env.readback(S, 16), env.readback(S, 16), env.readback(S, 16),
                               env.readback(S, 16), env.readback(S, 16), env.readback(S, 4)};
    ephys::CollisionShapeBuffers down;
    down.geometry = out[0];
    down.positions = out[1];
    down.rotations = out[2];
    down.prevPositions = out[3];
    down.prevRotations = out[4];
    down.flags = out[5];
    CHECK(shapes->download(nullptr, down, S) == nvrhi::FS_OK);
    CHECK(std::memcmp(env.read(out[0]).data(), geometry, sizeof(geometry)) == 0);
    CHECK(std::memcmp(env.read(out[1]).data(), positions.data(), S * 16) == 0);
    CHECK(std::memcmp(env.read(out[2]).data(), rotations.data(), S * 16) == 0);
    CHECK(std::memcmp(env.read(out[3]).data(), prevPositions.data(), S * 16) == 0);
    CHECK(std::memcmp(env.read(out[4]).data(), prevRotations.data(), S * 16) == 0);
    CHECK(std::memcmp(env.read(out[5]).data(), flags, sizeof(flags)) == 0);

    // Null previous transforms are the current ones (FleX).
    ephys::CollisionShapeData still = data;
    still.prevPositions = nullptr;
    still.prevRotations = nullptr;
    CHECK(shapes->upload(nullptr, still) == nvrhi::FS_OK);
    CHECK(shapes->getLayoutRevision() == 1 && shapes->getContentRevision() == 2);
    CHECK(refCount(convex.Get()) == convexRefs + 1);  // held once, not once per upload
    CHECK(std::memcmp(env.fetch(shapes->getPrevPositions(), 0, S).data(), positions.data(), S * 16) == 0);
    CHECK(std::memcmp(env.fetch(shapes->getPrevRotations(), 0, S).data(), rotations.data(), S * 16) == 0);

    // Asset lifetime: the application lets go, the shapes do not.
    std::printf("  asset lifetime\n");
    CHECK(convex.Detach()->Release() == 1);
    CHECK(mesh.Detach()->Release() == 1);
    CHECK(sdf.Detach()->Release() == 1);
    CHECK(library->getConvexIds(nullptr, 0) == 2 && library->getTriangleMeshIds(nullptr, 0) == 1 &&
          library->getSdfIds(nullptr, 0) == 1);
    {
        ephys::ConvexAssetHandle alive;
        CHECK(library->getConvex(convexId, &alive) == nvrhi::FS_OK && alive && alive->getId() == convexId);
        // Its id is not handed out again while a shape names it.
        ephys::ConvexAssetHandle next;
        CHECK(library->createConvex(&next) == nvrhi::FS_OK && next->getId() != convexId);
    }

    // Refused uploads: the set, and what it holds, are as before.
    ephys::CollisionGeometry badGeometry[S];
    std::memcpy(badGeometry, geometry, sizeof(geometry));
    badGeometry[3].convexMesh.mesh = 77;  // no such convex
    ephys::CollisionShapeData bad = data;
    bad.geometry = badGeometry;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->upload(nullptr, bad));
    badGeometry[3].convexMesh.mesh = convexId;
    badGeometry[5].sdf.field = 0;  // "no asset"
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->upload(nullptr, bad));
    int32_t badFlags[S];
    std::memcpy(badFlags, flags, sizeof(flags));
    badFlags[1] = 7;  // not a shape type
    bad = data;
    bad.flags = badFlags;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->upload(nullptr, bad));
    bad = data;
    bad.rotations = nullptr;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->upload(nullptr, bad));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->download(nullptr, down, S + 1));
    down.flags = out[0];  // 16-byte elements
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, shapes->download(nullptr, down, S));
    CHECK(shapes->getNumShapes() == S && shapes->getContentRevision() == 2);
    CHECK(env.phys->flush() == 0);
    CHECK(library->getConvexIds(nullptr, 0) == 2);

    // More shapes than the capacity: the set grows.
    {
        constexpr uint32_t M = 10;
        std::vector<ephys::CollisionGeometry> spheres(M);
        std::vector<float> transforms(M * 4, 0.0f);
        std::vector<int32_t> sphereFlags(M, ephys::makeShapeFlags(ephys::CollisionShapeType::Sphere, false));
        for (uint32_t i = 0; i < M; ++i) {
            std::memset(&spheres[i], 0, sizeof(spheres[i]));
            spheres[i].sphere.radius = float(i + 1);
        }
        ephys::CollisionShapeData more;
        more.geometry = spheres.data();
        more.positions = transforms.data();
        more.rotations = transforms.data();
        more.flags = sphereFlags.data();
        more.numShapes = M;
        CHECK(shapes->upload(nullptr, more) == nvrhi::FS_OK);
        CHECK(shapes->getNumShapes() == M && shapes->getMaxShapes() == 15 && shapes->getLayoutRevision() == 2);
        CHECK(std::memcmp(env.fetch(shapes->getGeometry(), 0, M).data(), spheres.data(), M * 16) == 0);
    }

    // No shape names the assets any more: they are gone, and their ids are free.
    CHECK(library->getConvexIds(nullptr, 0) == 1);  // the spare one
    CHECK(library->getTriangleMeshIds(nullptr, 0) == 0 && library->getSdfIds(nullptr, 0) == 0);
    ephys::ConvexAssetHandle gone;
    CHECK(library->getConvex(convexId, &gone) == nvrhi::FE_NOT_FOUND);

    // An empty set.
    CHECK(shapes->upload(nullptr, ephys::CollisionShapeData()) == nvrhi::FS_OK);
    CHECK(shapes->getNumShapes() == 0 && shapes->getMaxShapes() == 15);

    // A set that still names an asset when the domain goes away lets go of it then.
    CHECK(library->createConvex(&convex) == nvrhi::FS_OK);
    geometry[3].convexMesh.mesh = convex->getId();
    data.numShapes = 4;  // sphere, capsule, box, convex
    CHECK(shapes->upload(nullptr, data) == nvrhi::FS_OK);
    CHECK(refCount(convex.Get()) == 2);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(domain.Detach()->Release() == 0);
    CHECK(convex.Detach()->Release() == 0);
    CHECK(spare.Detach()->Release() == 0);
    CHECK(library->getConvexIds(nullptr, 0) == 0);
}

void testGeometry(const Env& env) {
    std::printf("geometry library\n");
    ephys::IGeometryLibrary* library = env.phys->getGeometry();
    CHECK(library != nullptr);
    if (!library)
        return;
    nvrhi::AutoPtr<ephys::IGeometryLibrary> same;
    CHECK(library->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK && same == library);
    same = nullptr;
    CHECK(library->createConvex(nullptr) == nvrhi::FE_INVALID_ARGS);

    testConvexes(env, library);
    testSdfs(env, library);
    testTriangleMeshes(env, library);
}

// ---- The hierarchy builder, with bounds it computes
// --------------------------------------------------------------

void testBvhBuilder(const Env& env) {
    std::printf("hierarchy builder: computed bounds\n");
    ephys::detail::BvhBuilder builder;
    ephys::detail::Bvh bvh;

    std::mt19937 random(20261004u);
    std::uniform_real_distribution<float> position(-50.0f, 50.0f), extent(0.01f, 3.0f);

    // Growing and shrinking item counts through one builder: 1, one sort block, several, fewer again.
    for (const uint32_t n : {1u, 6u, 700u, 5000u, 64u}) {
        std::vector<Float4> itemLowers(n), itemUppers(n);
        for (uint32_t i = 0; i < n; ++i) {
            const float x = position(random), y = position(random), z = position(random);
            itemLowers[i] = {x, y, z, 0.0f};
            itemUppers[i] = {x + extent(random), y + extent(random), z + extent(random), 0.0f};
        }
        // Device buffers of the items, as a solver has them for its shapes.
        ephys::PhysBufferDesc desc;
        desc.elementCount = n;
        desc.elementStride = 16;
        PhysBufferHandle lowerBuffer, upperBuffer;
        desc.debugName = "test item lowers";
        CHECK(env.phys->createBuffer(desc, &lowerBuffer) == nvrhi::FS_OK);
        desc.debugName = "test item uppers";
        CHECK(env.phys->createBuffer(desc, &upperBuffer) == nvrhi::FS_OK);
        if (!lowerBuffer || !upperBuffer)
            return;
        CHECK(env.phys->writeBuffer(nullptr, lowerBuffer, itemLowers.data(), 0, n) == nvrhi::FS_OK);
        CHECK(env.phys->writeBuffer(nullptr, upperBuffer, itemUppers.data(), 0, n) == nvrhi::FS_OK);

        bool replaced = false;
        CHECK(builder.build(env.phys, nullptr, bvh, lowerBuffer->getRHIBuffer(), upperBuffer->getRHIBuffer(), n,
                            nullptr, &replaced) == nvrhi::FS_OK);
        CHECK(bvh.numNodes == 2 * n && bvh.maxNodes >= 2 * n);

        const std::vector<BvhNodeHalf> lowers = env.fetchAs<BvhNodeHalf>(bvh.nodeLowers, 0, 2 * n);
        const std::vector<BvhNodeHalf> uppers = env.fetchAs<BvhNodeHalf>(bvh.nodeUppers, 0, 2 * n);
        const std::vector<int32_t> root = env.fetchAs<int32_t>(bvh.rootNode, 0, 1);
        uint32_t depth = 0;
        const bool valid =
            root.size() == 1 && validateBvh(lowers, uppers, root[0], itemLowers, itemUppers, &depth);
        CHECK(valid);
        std::printf("  %u items, depth %u, buffers %s: %s\n", n, depth, replaced ? "replaced" : "kept",
                    valid ? "valid" : "INVALID");

        // The root contains everything.
        if (valid) {
            for (int k = 0; k < 3; ++k) {
                float lowest = 3.0e38f, highest = -3.0e38f;
                for (uint32_t i = 0; i < n; ++i) {
                    const float lower[3] = {itemLowers[i].x, itemLowers[i].y, itemLowers[i].z};
                    const float upper[3] = {itemUppers[i].x, itemUppers[i].y, itemUppers[i].z};
                    lowest = std::min(lowest, lower[k]);
                    highest = std::max(highest, upper[k]);
                }
                CHECK(lowers[size_t(root[0])].corner[k] == lowest &&
                      uppers[size_t(root[0])].corner[k] == highest);
            }
        }
    }

    CHECK(builder.build(env.phys, nullptr, bvh, nullptr, nullptr, 4, nullptr, nullptr) ==
          nvrhi::FE_INVALID_ARGS);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
}

// ---- Lifetime
// --------------------------------------------------------------------------------------------------

// The library and an asset held past the device: queries still work, recording does not.
void testLibraryOutlivesDevice(nvrhi::IDevice* device, PhysMessages* messages, const std::string& shaderPath) {
    std::printf("library and assets held past the device\n");
    ephys::PhysDeviceDesc deviceDesc;
    deviceDesc.device = device;
    deviceDesc.messageCallback = messages;
    deviceDesc.shaderPath = shaderPath.c_str();
    ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
    CHECK(phys != nullptr);
    if (!phys)
        return;

    Env env;
    env.device = device;
    env.phys = phys;
    env.messages = messages;

    ephys::GeometryLibraryHandle library = phys->getGeometry();
    ephys::TriangleMeshAssetHandle mesh;
    ephys::ConvexAssetHandle convex;
    ephys::SdfAssetHandle sdf;
    CHECK(library->createTriangleMesh(&mesh) == nvrhi::FS_OK &&
          library->createConvex(&convex) == nvrhi::FS_OK && library->createSdf(&sdf) == nvrhi::FS_OK);
    if (!mesh || !convex || !sdf)
        return;
    const Mesh sheet = makeSheet(4, 4, 1.0f, 0.0f);
    PhysBufferHandle vertices = env.upload(sheet.vertices, 16), indices = env.upload(sheet.indices, 4);
    CHECK(mesh->update(nullptr, vertices, indices, uint32_t(sheet.vertices.size()), sheet.numTriangles(),
                       sheet.bounds) == nvrhi::FS_OK);

    // The device goes: its recorded work is submitted, and the library is on its own.
    CHECK(phys.Detach()->Release() == 0);
    device->waitForIdle();

    CHECK(mesh->getNumTriangles() == sheet.numTriangles() && mesh->getId() == 1);
    CHECK(library->getTriangleMeshIds(nullptr, 0) == 1);
    CHECK(library->getConvexTable() != nullptr);
    CHECK(library->needsCommit());
    const float voxel = 1.0f;
    ephys::Bounds3 bounds;
    CHECK(mesh->update(nullptr, vertices, indices, uint32_t(sheet.vertices.size()), sheet.numTriangles(),
                       sheet.bounds) == nvrhi::FE_NOT_ALIVE_OBJECT);
    CHECK(convex->update(nullptr, nullptr, 0, bounds) == nvrhi::FE_NOT_ALIVE_OBJECT);
    CHECK(sdf->update(nullptr, &voxel, 1, 1, 1) == nvrhi::FE_NOT_ALIVE_OBJECT);
    CHECK(library->commit(nullptr) == nvrhi::FE_NOT_ALIVE_OBJECT);
    // Creating and releasing assets needs no device.
    ephys::ConvexAssetHandle another;
    CHECK(library->createConvex(&another) == nvrhi::FS_OK && another->getId() == 2);

    device->runGarbageCollection();
    CHECK(another.Detach()->Release() == 0);
    CHECK(mesh.Detach()->Release() == 0);
    CHECK(convex.Detach()->Release() == 0);
    CHECK(sdf.Detach()->Release() == 0);
    vertices = nullptr;
    indices = nullptr;
    CHECK(library.Detach()->Release() == 0);
}

}  // namespace

int main(int argc, char** argv) {
    donut::log::ConsoleApplicationMode();
    donut::log::SetMinSeverity(donut::log::Severity::Warning);
    donut::log::SetCallback(logCallback);

    nvrhi::GraphicsAPI api = nvrhi::GraphicsAPI::D3D12;
    int adapter = -1;
    std::string shaderPath;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "d3d11") == 0) {
            api = nvrhi::GraphicsAPI::D3D11;
        } else if (std::strcmp(argv[i], "d3d12") == 0) {
            api = nvrhi::GraphicsAPI::D3D12;
        } else if (std::strcmp(argv[i], "vulkan") == 0) {
            api = nvrhi::GraphicsAPI::VULKAN;
        } else if (std::strcmp(argv[i], "--adapter") == 0 && i + 1 < argc) {
            adapter = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--shaders") == 0 && i + 1 < argc) {
            shaderPath = argv[++i];
        } else {
            std::printf("usage: %s <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]\n", argv[0]);
            return 1;
        }
    }
    if (shaderPath.empty())
        shaderPath =
            (donut::app::GetDirectoryWithExecutable() / "shaders" / "ethereal_phys").generic_u8string();
    std::printf("kernels: %s\n", shaderPath.c_str());

    auto rhiMessages = MAKE_RC_OBJ_PTR(RhiMessages);
    auto physMessages = MAKE_RC_OBJ_PTR(PhysMessages);

    auto deviceManager = nvrhi::TakeOver(donut::app::DeviceManager::Create(api));
    if (!deviceManager) {
        std::printf("SKIPPED: this build has no device manager for the requested graphics API\n");
        return 77;
    }

    donut::app::DeviceCreationParameters params;
    params.enableDebugRuntime = true;
    params.enableNvrhiValidationLayer = true;
    params.messageCallback = rhiMessages;
    params.adapterIndex = adapter;

    if (api == nvrhi::GraphicsAPI::VULKAN && adapter < 0) {
        // Name the physical device instead of letting the device manager choose: see device_test.cpp (its
        // automatic choice mixes up the queue families of two GPUs).
        std::vector<donut::app::AdapterInfo> adapters;
        if (deviceManager->CreateInstance(params) && deviceManager->EnumerateAdapters(adapters) &&
            !adapters.empty()) {
            params.adapterIndex = 0;
            std::printf("vulkan adapter 0 of %d: %s\n", int(adapters.size()), adapters[0].name.c_str());
        }
    }

    if (!deviceManager->CreateHeadlessDevice(params)) {
        std::printf("SKIPPED: cannot create a headless device for the requested graphics API\n");
        return 77;
    }

    nvrhi::IDevice* device = deviceManager->GetDevice();
    {
        D3DMessages d3dMessages(device);
        if (api != nvrhi::GraphicsAPI::VULKAN) {
            CHECK(d3dMessages.attached());
            std::printf("d3d debug layer: %s\n", d3dMessages.attached() ? "watched" : "NOT available");
        }
        const nvrhi::FLONG deviceRefs = refCount(device);
        const nvrhi::FLONG messagesRefs = refCount(physMessages.Get());

        ephys::PhysDeviceDesc deviceDesc;
        deviceDesc.device = device;
        deviceDesc.messageCallback = physMessages;
        deviceDesc.shaderPath = shaderPath.c_str();
        ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
        CHECK(phys != nullptr);
        if (phys) {
            Env env;
            env.device = device;
            env.phys = phys;
            env.messages = physMessages;

            testParticles(env);
            CHECK(d3dMessages.problems() == 0);
            testRenderArrays(env);
            CHECK(d3dMessages.problems() == 0);
            testDiffuse(env);
            testConstraints(env);
            CHECK(d3dMessages.problems() == 0);
            testGeometry(env);
            CHECK(d3dMessages.problems() == 0);
            testShapes(env);
            testBvhBuilder(env);
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

        testLibraryOutlivesDevice(device, physMessages, shaderPath);

        device->waitForIdle();
        device->runGarbageCollection();
        CHECK(refCount(device) == deviceRefs);
        CHECK(refCount(physMessages.Get()) == messagesRefs);
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
