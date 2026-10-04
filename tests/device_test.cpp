// ethereal_phys_device_test: IPhysDevice and IPhysBuffer on a real nvrhi device, headless.
//
//     ethereal_phys_device_test <d3d11|d3d12|vulkan> [compute] [--adapter <n>]
//
// The nvrhi device is created by Donut's DeviceManager with the debug runtime and the nvrhi validation
// layer on. Every nvrhi error or warning is a test failure, and so is every engine message the test did not
// provoke on purpose. "compute" creates the device with a compute queue and asks the engine to use it.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>

#include "d3d_messages.h"

#include <donut/app/DeviceManager.h>
#include <donut/core/log.h>

#include <nvrhi/core/foundation.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

int g_Failures = 0;

#define CHECK(expr)                                                         \
    do {                                                                    \
        if (!(expr)) {                                                      \
            ++g_Failures;                                                   \
            std::printf("FAILED %s(%d): %s\n", __FILE__, __LINE__, #expr); \
        }                                                                   \
    } while (false)

// Donut's log: the Vulkan validation layer and, on D3D11 and Vulkan, nvrhi itself report through it (only
// the D3D12 device manager passes DeviceCreationParameters::messageCallback on). None is expected. The
// default handler breaks into the debugger on an error; this one counts and goes on, so that the test
// reports what happened.
std::atomic<int> g_LogProblems{0};

void logCallback(donut::log::Severity severity, const char* text) {
    if (severity < donut::log::Severity::Warning) return;
    ++g_LogProblems;
    std::printf("log %s: %s\n", severity == donut::log::Severity::Warning ? "warning" : "error", text);
}

// nvrhi and validation-layer messages: none is expected.
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

// Engine messages: counted, so that the test can tell the ones it provokes from the others.
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

// The strong reference count of an object, read without changing it.
template <typename T>
nvrhi::FLONG refCount(T* object) {
    object->AddRef();
    return object->Release();
}

struct Element {
    uint32_t v[4];
};
static_assert(sizeof(Element) == 16);

Element pattern(uint32_t i, uint32_t salt) {
    return Element{{i, i * 2654435761u + salt, ~i, salt}};
}

bool equal(const Element& a, const Element& b) {
    return std::memcmp(&a, &b, sizeof(Element)) == 0;
}

// Runs a call that must fail with `expected` and must tell the message callback exactly once.
#define CHECK_FAILS(expected, messages, call)                 \
    do {                                                      \
        const int before = (messages)->errors.load();        \
        CHECK((call) == (expected));                          \
        CHECK((messages)->errors.load() == before + 1);       \
    } while (false)

void testHeaderVersion(nvrhi::IDevice* device, PhysMessages* messages) {
    std::printf("header version\n");
    CHECK(etherealPhysGetHeaderVersion() == ephys::c_HeaderVersion);
    CHECK(ephys::verifyHeaderVersion());
    CHECK(!ephys::verifyHeaderVersion(ephys::c_HeaderVersion + 1));

    ephys::PhysDeviceDesc desc;
    desc.device = device;
    desc.messageCallback = messages;

    // A caller compiled against other headers gets an error and no device.
    ephys::IPhysDevice* stale = reinterpret_cast<ephys::IPhysDevice*>(uintptr_t(0x1));
    CHECK(etherealPhysCreateDevice(&desc, ephys::c_HeaderVersion + 1, &stale) == nvrhi::FE_UNSUPPORTED);
    CHECK(stale == nullptr);
    stale = reinterpret_cast<ephys::IPhysDevice*>(uintptr_t(0x1));
    CHECK(etherealPhysCreateDevice(&desc, ephys::c_HeaderVersion - 1, &stale) == nvrhi::FE_UNSUPPORTED);
    CHECK(stale == nullptr);
    // With a stale version the desc is not read at all: a null desc is still "unsupported".
    CHECK(etherealPhysCreateDevice(nullptr, ephys::c_HeaderVersion + 1, &stale) == nvrhi::FE_UNSUPPORTED);

    // A caller compiled against other nvrhi headers.
    ephys::PhysDeviceDesc otherNvrhi = desc;
    otherNvrhi.nvrhiHeaderVersion = nvrhi::c_HeaderVersion + 1;
    ephys::IPhysDevice* none = nullptr;
    CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages,
                etherealPhysCreateDevice(&otherNvrhi, ephys::c_HeaderVersion, &none));
    CHECK(none == nullptr);

    // Invalid arguments.
    ephys::PhysDeviceDesc noDevice;
    CHECK(etherealPhysCreateDevice(&noDevice, ephys::c_HeaderVersion, &none) == nvrhi::FE_INVALID_ARGS);
    CHECK(etherealPhysCreateDevice(nullptr, ephys::c_HeaderVersion, &none) == nvrhi::FE_INVALID_ARGS);
    CHECK(etherealPhysCreateDevice(&desc, ephys::c_HeaderVersion, nullptr) == nvrhi::FE_INVALID_ARGS);
    CHECK(!ephys::createDevice(noDevice));
}

// Reads `count` elements of a Readback buffer into `out`.
void readBack(ephys::IPhysBuffer* readback, std::vector<Element>& out, uint32_t count) {
    out.assign(count, Element{});
    void* data = nullptr;
    CHECK(readback->map(&data) == nvrhi::FS_OK);
    CHECK(data != nullptr);
    if (data) {
        std::memcpy(out.data(), data, size_t(count) * sizeof(Element));
        CHECK(readback->unmap() == nvrhi::FS_OK);
    }
}

void testDevice(nvrhi::IDevice* device, PhysMessages* messages, bool preferCompute) {
    const nvrhi::GraphicsAPI api = device->getGraphicsAPI();
    const nvrhi::FLONG deviceRefs = refCount(device);
    const nvrhi::FLONG messagesRefs = refCount(messages);

    ephys::PhysDeviceDesc deviceDesc;
    deviceDesc.device = device;
    deviceDesc.messageCallback = messages;
    deviceDesc.shaderPath = "shaders/ethereal_phys";
    deviceDesc.preferComputeQueue = preferCompute;

    ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
    CHECK(phys != nullptr);
    if (!phys) return;

    std::printf("device\n");
    {
        CHECK(phys->getRHIDevice() == device);
        CHECK(refCount(device) > deviceRefs);
        // The device holds the callback, and so does its kernel library.
        CHECK(refCount(messages) == messagesRefs + 2);

        const ephys::PhysDeviceDesc& desc = phys->getDesc();
        CHECK(desc.device == device);
        CHECK(desc.messageCallback == messages);
        CHECK(desc.preferComputeQueue == preferCompute);
        // The device keeps its own copy of the string.
        CHECK(desc.shaderPath != deviceDesc.shaderPath);
        CHECK(desc.shaderPath && std::strcmp(desc.shaderPath, "shaders/ethereal_phys") == 0);

        const ephys::PhysDeviceInfo& info = phys->getInfo();
        CHECK(info.headerVersion == ephys::c_HeaderVersion);
        CHECK(info.nvrhiHeaderVersion == nvrhi::c_HeaderVersion);
        CHECK(info.graphicsAPI == api);
        const bool computeQueue = preferCompute && device->queryFeatureSupport(nvrhi::Feature::ComputeQueue);
        CHECK(info.executionQueue == (computeQueue ? nvrhi::CommandQueue::Compute : nvrhi::CommandQueue::Graphics));
        CHECK(info.supportsRenderSharedBuffers == (api != nvrhi::GraphicsAPI::D3D11));
        CHECK(info.supportsWaveOps == device->queryFeatureSupport(nvrhi::Feature::WaveLaneCountMinMax));
        std::printf("  queue %s, wave ops %d (lanes %u..%u), render-shared buffers %d\n",
                    info.executionQueue == nvrhi::CommandQueue::Compute ? "compute" : "graphics",
                    int(info.supportsWaveOps), info.minWaveLaneCount, info.maxWaveLaneCount,
                    int(info.supportsRenderSharedBuffers));

        // QueryInterface: the interfaces the object implements, and one it does not.
        nvrhi::AutoPtr<ephys::IPhysDevice> same;
        CHECK(phys->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK);
        CHECK(same == phys);
        nvrhi::AutoPtr<nvrhi::IObject> object;
        CHECK(phys->QueryInterface(NVRHI_IID_PPV_ARGS(&object)) == nvrhi::FS_OK);
        nvrhi::AutoPtr<ephys::IPhysBuffer> notABuffer;
        CHECK(phys->QueryInterface(NVRHI_IID_PPV_ARGS(&notABuffer)) == nvrhi::FE_NOINTERFACE);
        CHECK(notABuffer == nullptr);

        // Nothing recorded yet.
        CHECK(phys->flush() == 0);
    }

    constexpr uint32_t N = 1024;

    ephys::PhysBufferHandle upload, structured, readback;
    std::printf("create buffers\n");
    {
        ephys::PhysBufferDesc desc;
        desc.elementCount = N;
        desc.elementStride = sizeof(Element);

        desc.use = ephys::BufferUse::Upload;
        desc.debugName = "test upload";
        CHECK(phys->createBuffer(desc, &upload) == nvrhi::FS_OK);
        desc.use = ephys::BufferUse::Structured;
        desc.debugName = "test structured";
        CHECK(phys->createBuffer(desc, &structured) == nvrhi::FS_OK);
        desc.use = ephys::BufferUse::Readback;
        desc.debugName = nullptr;
        CHECK(phys->createBuffer(desc, &readback) == nvrhi::FS_OK);
        if (!upload || !structured || !readback) return;

        CHECK(structured->getElementCount() == N);
        CHECK(structured->getElementStride() == sizeof(Element));
        CHECK(structured->getDesc().use == ephys::BufferUse::Structured);
        CHECK(structured->getDesc().elementCount == N);
        // The buffer owns the name it reports.
        CHECK(structured->getDesc().debugName != desc.debugName);
        CHECK(structured->getDesc().debugName && std::strcmp(structured->getDesc().debugName, "test structured") == 0);
        // No name was given: nullptr, or the name the nvrhi validation layer made up.
        CHECK(readback->getDesc().debugName == nullptr || readback->getDesc().debugName[0] != 0);
        CHECK(structured->getContentRevision() == 0);

        // The recipes.
        const nvrhi::BufferDesc& s = structured->getRHIBuffer()->getDesc();
        CHECK(s.byteSize == uint64_t(N) * sizeof(Element));
        CHECK(s.structStride == sizeof(Element));
        CHECK(s.canHaveUAVs && s.keepInitialState && s.cpuAccess == nvrhi::CpuAccessMode::None);
        CHECK(s.initialState == nvrhi::ResourceStates::UnorderedAccess);
        const nvrhi::BufferDesc& u = upload->getRHIBuffer()->getDesc();
        CHECK(u.cpuAccess == nvrhi::CpuAccessMode::Write && u.keepInitialState);
        const nvrhi::BufferDesc& r = readback->getRHIBuffer()->getDesc();
        CHECK(r.cpuAccess == nvrhi::CpuAccessMode::Read && r.keepInitialState);
    }

    std::printf("map, copy Upload -> Structured -> Readback on the device's command list\n");
    {
        void* data = nullptr;
        CHECK(upload->map(&data) == nvrhi::FS_OK);
        CHECK(data != nullptr);
        if (!data) return;
        // A second map before unmap is refused.
        void* again = reinterpret_cast<void*>(uintptr_t(0x1));
        CHECK(upload->map(&again) == nvrhi::FE_INVALID_ARGS);
        CHECK(again == nullptr);

        Element* elements = static_cast<Element*>(data);
        for (uint32_t i = 0; i < N; ++i) elements[i] = pattern(i, 0x1111);
        CHECK(upload->getContentRevision() == 0);
        CHECK(upload->unmap() == nvrhi::FS_OK);
        CHECK(upload->getContentRevision() == 1);
        CHECK(upload->unmap() == nvrhi::FE_INVALID_ARGS);  // not mapped

        ephys::CopyRegion all;
        all.elementCount = N;
        CHECK(phys->copyBuffer(nullptr, structured, upload, all) == nvrhi::FS_OK);
        CHECK(phys->copyBuffer(nullptr, readback, structured, all) == nvrhi::FS_OK);
        CHECK(structured->getContentRevision() == 1);
        CHECK(readback->getContentRevision() == 1);
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);
        CHECK(phys->flush() == 0);  // waitForIdle flushed

        std::vector<Element> result;
        readBack(readback, result, N);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < N; ++i) mismatches += equal(result[i], pattern(i, 0x1111)) ? 0 : 1;
        CHECK(mismatches == 0);
        CHECK(readback->getContentRevision() == 1);  // unmap of a Readback buffer changes nothing
    }

    std::printf("writeBuffer into a range, copy a sub-range on the application's command list\n");
    {
        constexpr uint32_t first = 10, count = 20;
        Element patch[count];
        for (uint32_t i = 0; i < count; ++i) patch[i] = pattern(first + i, 0x2222);
        CHECK(phys->writeBuffer(nullptr, structured, patch, first, count) == nvrhi::FS_OK);
        CHECK(structured->getContentRevision() == 2);
        const uint64_t instance = phys->flush();
        CHECK(instance != 0 || api == nvrhi::GraphicsAPI::D3D11);  // D3D11 has no instances

        // The application's own command list, opened after the device's was submitted.
        nvrhi::CommandListHandle commandList;
        CHECK(NVRHI_SUCCEEDED(device->createCommandList(nvrhi::CommandListParameters(), &commandList)));
        if (!commandList) return;
        // Elements [8, 40) of the structured buffer go to [100, 132) of the readback buffer.
        ephys::CopyRegion part;
        part.srcOffset = 8;
        part.dstOffset = 100;
        part.elementCount = 32;
        commandList->open();
        CHECK(phys->copyBuffer(commandList, readback, structured, part) == nvrhi::FS_OK);
        commandList->close();
        if (phys->getInfo().executionQueue != nvrhi::CommandQueue::Graphics)
            device->queueWaitForCommandList(nvrhi::CommandQueue::Graphics, phys->getInfo().executionQueue, instance);
        device->executeCommandList(commandList);
        CHECK(phys->flush() == 0);  // the engine's command list was not touched
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);
        CHECK(readback->getContentRevision() == 2);

        std::vector<Element> result;
        readBack(readback, result, N);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < N; ++i) {
            Element expected = pattern(i, 0x1111);  // the first copy
            if (i >= 100 && i < 132) {
                const uint32_t src = i - 100 + 8;
                expected = (src >= first && src < first + count) ? pattern(src, 0x2222) : pattern(src, 0x1111);
            }
            mismatches += equal(result[i], expected) ? 0 : 1;
        }
        CHECK(mismatches == 0);
    }

    std::printf("refused calls\n");
    {
        // A device buffer has no host address.
        void* data = reinterpret_cast<void*>(uintptr_t(0x1));
        CHECK(structured->map(&data) == nvrhi::FE_UNSUPPORTED);
        CHECK(data == nullptr);
        CHECK(structured->unmap() == nvrhi::FE_INVALID_ARGS);
        CHECK(structured->map(nullptr) == nvrhi::FE_INVALID_ARGS);

        ephys::CopyRegion all;
        all.elementCount = N;
        // No copy into an Upload buffer, none out of a Readback buffer, none onto itself.
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, upload, structured, all));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, structured, readback, all));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, structured, structured, all));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, structured, nullptr, all));
        // Out of range.
        ephys::CopyRegion tooFar = all;
        tooFar.dstOffset = 1;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, structured, upload, tooFar));
        Element one = {};
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->writeBuffer(nullptr, structured, &one, N, 1));
        CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages, phys->writeBuffer(nullptr, upload, &one, 0, 1));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->writeBuffer(nullptr, structured, nullptr, 0, 1));

        // Different strides.
        ephys::PhysBufferDesc desc;
        desc.elementCount = N;
        desc.elementStride = 8;
        ephys::PhysBufferHandle narrow;
        CHECK(phys->createBuffer(desc, &narrow) == nvrhi::FS_OK);
        if (narrow) CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->copyBuffer(nullptr, narrow, structured, all));

        // Bad descriptions.
        ephys::PhysBufferHandle none;
        desc.elementCount = 0;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->createBuffer(desc, &none));
        desc.elementCount = N;
        desc.elementStride = 0;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->createBuffer(desc, &none));
        desc.elementStride = 6;
        desc.use = ephys::BufferUse::Raw;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->createBuffer(desc, &none));
        CHECK(none == nullptr);

        // None of the refused calls recorded anything.
        CHECK(phys->flush() == 0);
        CHECK(structured->getContentRevision() == 2);
    }

    std::printf("the other uses\n");
    {
        ephys::PhysBufferDesc desc;
        desc.elementCount = 64;
        desc.elementStride = 16;

        ephys::PhysBufferHandle raw, constant, indirect, shared, typed;
        desc.use = ephys::BufferUse::Raw;
        desc.debugName = "test raw";
        CHECK(phys->createBuffer(desc, &raw) == nvrhi::FS_OK);
        if (raw) {
            const nvrhi::BufferDesc& d = raw->getRHIBuffer()->getDesc();
            CHECK(d.canHaveRawViews && d.canHaveUAVs && d.structStride == 0 && d.keepInitialState);
        }

        desc.use = ephys::BufferUse::Constant;
        desc.debugName = "test constant";
        CHECK(phys->createBuffer(desc, &constant) == nvrhi::FS_OK);
        if (constant) {
            const nvrhi::BufferDesc& d = constant->getRHIBuffer()->getDesc();
            CHECK(d.isConstantBuffer && !d.isVolatile && d.keepInitialState);
            std::vector<Element> values(64);
            for (uint32_t i = 0; i < 64; ++i) values[i] = pattern(i, 7);
            // The whole buffer on every backend; a part of it everywhere but on D3D11, which updates a
            // constant buffer as a whole or not at all.
            CHECK(phys->writeBuffer(nullptr, constant, values.data(), 0, 64) == nvrhi::FS_OK);
            CHECK(constant->getContentRevision() == 1);
            if (api == nvrhi::GraphicsAPI::D3D11) {
                CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages,
                            phys->writeBuffer(nullptr, constant, values.data(), 0, 4));
                CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages,
                            phys->writeBuffer(nullptr, constant, values.data(), 60, 4));
                CHECK(constant->getContentRevision() == 1);
            } else {
                CHECK(phys->writeBuffer(nullptr, constant, values.data(), 0, 4) == nvrhi::FS_OK);
                CHECK(phys->writeBuffer(nullptr, constant, values.data(), 60, 4) == nvrhi::FS_OK);
                CHECK(constant->getContentRevision() == 3);
            }
        }

        desc.use = ephys::BufferUse::IndirectArgs;
        desc.elementCount = 4;
        desc.elementStride = 12;
        desc.debugName = "test indirect args";
        CHECK(phys->createBuffer(desc, &indirect) == nvrhi::FS_OK);
        if (indirect) {
            const nvrhi::BufferDesc& d = indirect->getRHIBuffer()->getDesc();
            CHECK(d.isDrawIndirectArgs && d.canHaveUAVs && d.canHaveRawViews && d.keepInitialState);
            const uint32_t args[3] = {1, 1, 1};
            CHECK(phys->writeBuffer(nullptr, indirect, args, 0, 1) == nvrhi::FS_OK);
        }

        // The sort buffers of the kernels: R32_UINT elements with typed views.
        desc.use = ephys::BufferUse::Typed;
        desc.elementCount = 1024;
        desc.elementStride = 4;
        desc.debugName = "test typed";
        CHECK(phys->createBuffer(desc, &typed) == nvrhi::FS_OK);
        if (typed) {
            const nvrhi::BufferDesc& d = typed->getRHIBuffer()->getDesc();
            CHECK(d.canHaveTypedViews && d.canHaveUAVs && d.format == nvrhi::Format::R32_UINT && d.structStride == 0);
            CHECK(d.keepInitialState && d.byteSize == 4096);
            CHECK(typed->getDesc().use == ephys::BufferUse::Typed);
            std::vector<uint32_t> keys(1024, 7u);
            CHECK(phys->writeBuffer(nullptr, typed, keys.data(), 0, 1024) == nvrhi::FS_OK);
            // An adopted buffer of that kind is recognised.
            ephys::PhysBufferHandle adoptedTyped;
            CHECK(phys->adoptBuffer(typed->getRHIBuffer(), 1024, 4, &adoptedTyped) == nvrhi::FS_OK);
            CHECK(adoptedTyped && adoptedTyped->getDesc().use == ephys::BufferUse::Typed);
        }
        desc.elementStride = 8;
        ephys::PhysBufferHandle wideTyped;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->createBuffer(desc, &wideTyped));
        CHECK(wideTyped == nullptr);

        desc.use = ephys::BufferUse::RenderShared;
        desc.elementCount = 64;
        desc.elementStride = 16;
        desc.debugName = "test render shared";
        if (phys->getInfo().supportsRenderSharedBuffers) {
            CHECK(phys->createBuffer(desc, &shared) == nvrhi::FS_OK);
            if (shared) {
                const nvrhi::BufferDesc& d = shared->getRHIBuffer()->getDesc();
                CHECK(d.isVertexBuffer && d.canHaveUAVs && d.structStride == 16 && d.keepInitialState);
                ephys::CopyRegion some;
                some.elementCount = 64;
                CHECK(phys->copyBuffer(nullptr, shared, structured, some) == nvrhi::FS_OK);
            }
        } else {
            CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages, phys->createBuffer(desc, &shared));
            CHECK(shared == nullptr);
        }

        CHECK(phys->waitForIdle() == nvrhi::FS_OK);

        // Each object is gone with its last reference.
        if (raw) CHECK(raw.Detach()->Release() == 0);
        if (constant) CHECK(constant.Detach()->Release() == 0);
        if (indirect) CHECK(indirect.Detach()->Release() == 0);
        if (shared) CHECK(shared.Detach()->Release() == 0);
        if (typed) CHECK(typed.Detach()->Release() == 0);
    }

    std::printf("adoptBuffer\n");
    {
        // What a renderer creates: a vertex buffer a kernel may write.
        constexpr uint32_t vertexCount = 256;
        nvrhi::BufferDesc vertexDesc;
        vertexDesc.byteSize = uint64_t(vertexCount) * sizeof(Element);
        vertexDesc.isVertexBuffer = true;
        vertexDesc.canHaveUAVs = api != nvrhi::GraphicsAPI::D3D11;
        vertexDesc.debugName = "application vertex buffer";
        // The state the buffer rests in between command lists. A compute command list cannot leave or
        // restore VertexBuffer, so a buffer shared with an engine on the compute queue rests in
        // UnorderedAccess and the renderer's command list moves it to VertexBuffer and back.
        const bool onCompute = phys->getInfo().executionQueue == nvrhi::CommandQueue::Compute;
        vertexDesc.enableAutomaticStateTracking(onCompute ? nvrhi::ResourceStates::UnorderedAccess
                                                          : nvrhi::ResourceStates::VertexBuffer);
        nvrhi::BufferHandle vertexBuffer;
        CHECK(NVRHI_SUCCEEDED(device->createBuffer(vertexDesc, &vertexBuffer)));
        if (!vertexBuffer) return;

        if (onCompute) {
            // The other choice is adopted, with a warning.
            nvrhi::BufferDesc restingAsVertexBuffer = vertexDesc;
            restingAsVertexBuffer.enableAutomaticStateTracking(nvrhi::ResourceStates::VertexBuffer);
            nvrhi::BufferHandle other;
            CHECK(NVRHI_SUCCEEDED(device->createBuffer(restingAsVertexBuffer, &other)));
            ephys::PhysBufferHandle warned;
            const int warnings = messages->warnings.load();
            CHECK(other && phys->adoptBuffer(other, vertexCount, sizeof(Element), &warned) == nvrhi::FS_OK);
            CHECK(messages->warnings.load() == warnings + 1);
            messages->warnings.store(warnings);  // provoked
        }
        const nvrhi::FLONG vertexRefs = refCount(vertexBuffer.Get());

        ephys::PhysBufferHandle adopted;
        CHECK(phys->adoptBuffer(vertexBuffer, vertexCount, sizeof(Element), &adopted) == nvrhi::FS_OK);
        if (!adopted) return;
        CHECK(adopted->getRHIBuffer() == vertexBuffer.Get());
        CHECK(adopted->getElementCount() == vertexCount);
        CHECK(adopted->getElementStride() == sizeof(Element));
        CHECK(adopted->getDesc().use == ephys::BufferUse::RenderShared);
        CHECK(adopted->getDesc().debugName &&
              std::strcmp(adopted->getDesc().debugName, "application vertex buffer") == 0);
        CHECK(refCount(vertexBuffer.Get()) == vertexRefs + 1);
        void* data = nullptr;
        CHECK(adopted->map(&data) == nvrhi::FE_UNSUPPORTED);

        // Fewer elements than the buffer holds is fine; more is not, and neither is a null buffer.
        ephys::PhysBufferHandle half, none;
        CHECK(phys->adoptBuffer(vertexBuffer, vertexCount / 2, sizeof(Element), &half) == nvrhi::FS_OK);
        CHECK(half && half->getElementCount() == vertexCount / 2);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                    phys->adoptBuffer(vertexBuffer, vertexCount + 1, sizeof(Element), &none));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->adoptBuffer(nullptr, 1, 4, &none));
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->adoptBuffer(vertexBuffer, 0, sizeof(Element), &none));
        // The stride of a structured buffer is not the caller's to choose.
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, phys->adoptBuffer(structured->getRHIBuffer(), 1, 8, &none));
        CHECK(none == nullptr);

        // The engine writes the application's buffer directly: device to device, then back for the check.
        ephys::CopyRegion region;
        region.srcOffset = 300;
        region.elementCount = vertexCount;
        CHECK(phys->copyBuffer(nullptr, adopted, structured, region) == nvrhi::FS_OK);
        CHECK(adopted->getContentRevision() == 1);
        ephys::CopyRegion back;
        back.elementCount = vertexCount;
        CHECK(phys->copyBuffer(nullptr, readback, adopted, back) == nvrhi::FS_OK);
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);

        std::vector<Element> result;
        readBack(readback, result, vertexCount);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < vertexCount; ++i) mismatches += equal(result[i], pattern(300 + i, 0x1111)) ? 0 : 1;
        CHECK(mismatches == 0);

        // A writer the engine cannot see says so.
        CHECK(adopted->markContentChanged() == 2);
        CHECK(adopted->getContentRevision() == 2);

        // Command lists that used the buffer hold it until they are collected.
        device->runGarbageCollection();
        CHECK(half.Detach()->Release() == 0);
        CHECK(adopted.Detach()->Release() == 0);
        CHECK(refCount(vertexBuffer.Get()) == vertexRefs);
    }

    std::printf("release\n");
    {
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);
        device->runGarbageCollection();

        // The wrappers go first, then the nvrhi buffers they held.
        nvrhi::BufferHandle keptAlive = structured->getRHIBuffer();
        const nvrhi::FLONG bufferRefs = refCount(keptAlive.Get());
        CHECK(upload.Detach()->Release() == 0);
        CHECK(structured.Detach()->Release() == 0);
        CHECK(readback.Detach()->Release() == 0);
        CHECK(refCount(keptAlive.Get()) == bufferRefs - 1);
        CHECK(keptAlive.Detach()->Release() == 0);

        CHECK(phys.Detach()->Release() == 0);
        device->waitForIdle();
        device->runGarbageCollection();
        CHECK(refCount(device) == deviceRefs);
        CHECK(refCount(messages) == messagesRefs);
    }
}

// Work left on the device's command list is submitted when the device goes away, not dropped.
void testFlushOnRelease(nvrhi::IDevice* device, PhysMessages* messages) {
    std::printf("release with recorded work\n");
    ephys::PhysDeviceDesc deviceDesc;
    deviceDesc.device = device;
    deviceDesc.messageCallback = messages;
    ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
    CHECK(phys != nullptr);
    if (!phys) return;

    ephys::PhysBufferDesc desc;
    desc.elementCount = 16;
    desc.elementStride = sizeof(Element);
    ephys::PhysBufferHandle structured, readback;
    CHECK(phys->createBuffer(desc, &structured) == nvrhi::FS_OK);
    desc.use = ephys::BufferUse::Readback;
    CHECK(phys->createBuffer(desc, &readback) == nvrhi::FS_OK);
    if (!structured || !readback) return;

    Element values[16];
    for (uint32_t i = 0; i < 16; ++i) values[i] = pattern(i, 0x3333);
    ephys::CopyRegion all;
    all.elementCount = 16;
    CHECK(phys->writeBuffer(nullptr, structured, values, 0, 16) == nvrhi::FS_OK);
    CHECK(phys->copyBuffer(nullptr, readback, structured, all) == nvrhi::FS_OK);

    CHECK(phys.Detach()->Release() == 0);  // buffers outlive the device: they hold the nvrhi device
    device->waitForIdle();

    std::vector<Element> result;
    readBack(readback, result, 16);
    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < 16; ++i) mismatches += equal(result[i], values[i]) ? 0 : 1;
    CHECK(mismatches == 0);

    device->runGarbageCollection();
    CHECK(structured.Detach()->Release() == 0);
    CHECK(readback.Detach()->Release() == 0);
}

}  // namespace

int main(int argc, char** argv) {
    donut::log::ConsoleApplicationMode();
    donut::log::SetMinSeverity(donut::log::Severity::Warning);
    donut::log::SetCallback(logCallback);

    nvrhi::GraphicsAPI api = nvrhi::GraphicsAPI::D3D12;
    bool compute = false;
    int adapter = -1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "d3d11") == 0) {
            api = nvrhi::GraphicsAPI::D3D11;
        } else if (std::strcmp(argv[i], "d3d12") == 0) {
            api = nvrhi::GraphicsAPI::D3D12;
        } else if (std::strcmp(argv[i], "vulkan") == 0) {
            api = nvrhi::GraphicsAPI::VULKAN;
        } else if (std::strcmp(argv[i], "compute") == 0) {
            compute = true;
        } else if (std::strcmp(argv[i], "--adapter") == 0 && i + 1 < argc) {
            adapter = std::atoi(argv[++i]);
        } else {
            std::printf("usage: %s <d3d11|d3d12|vulkan> [compute] [--adapter <n>]\n", argv[0]);
            return 1;
        }
    }

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
    params.enableComputeQueue = compute;
    params.messageCallback = rhiMessages;
    params.adapterIndex = adapter;

    if (api == nvrhi::GraphicsAPI::VULKAN && adapter < 0) {
        // Name the physical device instead of letting the device manager choose. Its automatic choice looks
        // at every physical device in turn and keeps the first compute queue family it sees, whichever
        // device that was; on a machine with two GPUs the device then gets the other GPU's family index
        // (seen here: a transfer-only family as the "compute" queue). With an explicit index it looks at
        // one device only.
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

    nvrhi::IDevice* device = deviceManager->GetDevice();
    {
        // The D3D debug layer does not report through nvrhi or Donut's log: read it directly.
        D3DMessages d3dMessages(device);
        if (api != nvrhi::GraphicsAPI::VULKAN) CHECK(d3dMessages.attached());

        testHeaderVersion(device, physMessages);
        testDevice(device, physMessages, compute);
        testFlushOnRelease(device, physMessages);

        device->waitForIdle();
        device->runGarbageCollection();
        CHECK(d3dMessages.problems() == 0);
    }

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
