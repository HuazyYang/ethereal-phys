// ethereal_phys_kernel_test: IPhysKernelLibrary on a real nvrhi device, headless.
//
//     ethereal_phys_kernel_test <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]
//
// The nvrhi device is created by Donut's DeviceManager with the debug runtime and the nvrhi validation
// layer on. Every nvrhi error or warning is a test failure, and so is every engine message the test did not
// provoke on purpose; on D3D11 and D3D12 the messages of the debug layer count as well (d3d_messages.h).
// --shaders is the directory the ethereal_phys_shaders target wrote the kernels to
// (default: shaders/ethereal_phys beside the executable).
//
// What it checks:
//   - every kernel, in every variant the device can run, becomes a compute pipeline;
//   - every kernel can be dispatched with all of its slots left null (the dummy resources), which makes
//     the validation layers compare each binding layout with what the compiled kernel really declares;
//   - ClearInt and ClearFloat4 write what they should, through more constant blocks than the pool holds;
//   - the constant buffers arrive in the kernels with the layout of the C++ mirror (ProbeKernelParams);
//   - the radix sort (engine/src/radix_sort.cpp, compiled into this test) sorts like std::stable_sort;
//   - the wave and the baseline variants of the bounds kernels give identical bounds, and those of
//     CalculateInflatableVolume the same volume term;
//   - missing, empty, truncated and foreign blobs are errors with a message, not crashes.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>
#include <ethereal/phys/kernels.h>

#include "d3d_messages.h"
#include "kernel_params.h"
#include "radix_sort.h"

#include <KernelParamsFields.hlsli>

#include <donut/app/ApplicationBase.h>
#include <donut/app/DeviceManager.h>
#include <donut/core/log.h>

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

using ephys::KernelDispatchDesc;
using ephys::KernelId;
using ephys::KernelInfo;
using ephys::KernelVariant;

int g_Failures = 0;

#define CHECK(expr)                                                         \
    do {                                                                    \
        if (!(expr)) {                                                      \
            ++g_Failures;                                                   \
            std::printf("FAILED %s(%d): %s\n", __FILE__, __LINE__, #expr); \
        }                                                                   \
    } while (false)

std::atomic<int> g_LogProblems{0};

void logCallback(donut::log::Severity severity, const char* text) {
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
        last = messageText;
        if (!quiet)
            std::printf("  (engine %s: %s)\n", severity == ephys::PhysMessageSeverity::Warning ? "warning" : "error",
                        messageText);
    }

    std::atomic<int> errors{0};
    std::atomic<int> warnings{0};
    std::string last;
    bool quiet = false;
};

// Runs a call that must fail with `expected` and must tell the message callback exactly once.
#define CHECK_FAILS(expected, messages, call)           \
    do {                                                \
        const int before = (messages)->errors.load();   \
        CHECK((call) == (expected));                    \
        CHECK((messages)->errors.load() == before + 1); \
    } while (false)

// ---- nvrhi helpers: buffers the way the solver will create them, uploads and readbacks -----------------

struct Gpu {
    nvrhi::IDevice* device = nullptr;
    nvrhi::CommandListHandle commandList;  // the application's command list

    nvrhi::BufferHandle structured(const char* name, uint32_t stride, uint32_t count) const {
        nvrhi::BufferDesc desc;
        desc.byteSize = uint64_t(stride) * count;
        desc.structStride = stride;
        desc.canHaveUAVs = true;
        desc.debugName = name;
        desc.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
        nvrhi::BufferHandle buffer;
        CHECK(device->createBuffer(desc, &buffer) == nvrhi::FS_OK);
        return buffer;
    }

    nvrhi::BufferHandle raw(const char* name, uint32_t bytes) const {
        nvrhi::BufferDesc desc;
        desc.byteSize = bytes;
        desc.canHaveRawViews = true;
        desc.canHaveUAVs = true;
        desc.debugName = name;
        desc.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
        nvrhi::BufferHandle buffer;
        CHECK(device->createBuffer(desc, &buffer) == nvrhi::FS_OK);
        return buffer;
    }

    void upload(nvrhi::IBuffer* buffer, const void* data, size_t bytes) const {
        commandList->open();
        commandList->writeBuffer(buffer, data, bytes);
        commandList->close();
        nvrhi::ICommandList* list = commandList;
        device->executeCommandLists(&list, 1);
    }

    template <typename T>
    void upload(nvrhi::IBuffer* buffer, const std::vector<T>& data) const {
        upload(buffer, data.data(), data.size() * sizeof(T));
    }

    // Copies `count` elements of T out of a device buffer.
    template <typename T>
    std::vector<T> download(nvrhi::IBuffer* buffer, size_t count) const {
        std::vector<T> out(count);
        nvrhi::BufferDesc desc;
        desc.byteSize = count * sizeof(T);
        desc.cpuAccess = nvrhi::CpuAccessMode::Read;
        desc.debugName = "test readback";
        desc.enableAutomaticStateTracking(nvrhi::ResourceStates::CopyDest);
        nvrhi::BufferHandle staging;
        CHECK(device->createBuffer(desc, &staging) == nvrhi::FS_OK);
        if (!staging) return out;

        commandList->open();
        commandList->copyBuffer(staging, 0, buffer, 0, desc.byteSize);
        commandList->close();
        nvrhi::ICommandList* list = commandList;
        device->executeCommandLists(&list, 1);
        device->waitForIdle();

        void* data = device->mapBuffer(staging, nvrhi::CpuAccessMode::Read);
        CHECK(data != nullptr);
        if (data) {
            std::memcpy(out.data(), data, desc.byteSize);
            device->unmapBuffer(staging);
        }
        return out;
    }

    void begin() const { commandList->open(); }

    void end() const {
        commandList->close();
        nvrhi::ICommandList* list = commandList;
        device->executeCommandLists(&list, 1);
    }
};

struct Float3 {
    float x, y, z;
};
struct Float4 {
    float x, y, z, w;
};

// ---- the tests -----------------------------------------------------------------------------------------

void testCatalogue(ephys::IPhysDevice* phys, PhysMessages* messages) {
    std::printf("catalogue\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();
    CHECK(kernels != nullptr);
    if (!kernels) return;

    CHECK(kernels->getKernelCount() == uint32_t(KernelId::Count));
    CHECK(kernels->getKernelName(KernelId::Count) == nullptr);
    CHECK(std::strcmp(kernels->getKernelName(KernelId::ApplyDeltas), "ApplyDeltas") == 0);
    CHECK(std::strcmp(kernels->getKernelName(KernelId::SolveShapesXPBD), "SolveShapesXPBD") == 0);
    CHECK(std::strcmp(kernels->getKernelName(KernelId::RadixSort1CS), "radixSort1CS") == 0);

    nvrhi::AutoPtr<ephys::IPhysKernelLibrary> same;
    CHECK(kernels->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK);
    CHECK(same == kernels);
    nvrhi::AutoPtr<ephys::IPhysDevice> notADevice;
    CHECK(kernels->QueryInterface(NVRHI_IID_PPV_ARGS(&notADevice)) == nvrhi::FE_NOINTERFACE);

    KernelInfo info;
    kernels->getKernelInfo(info, KernelId::Count);
    CHECK(info.name == nullptr && info.readOnlyMask == 0 && info.threadGroupSize[0] == 0);

    // The layout of a kernel is what its source declares.
    kernels->getKernelInfo(info, KernelId::ClearInt);
    CHECK(info.readOnlyMask == 0 && info.readWriteMask == 0x1 && info.constantBufferMask == 0x1);
    CHECK(info.threadGroupSize[0] == 256 && info.threadGroupSize[1] == 1 && info.threadGroupSize[2] == 1);
    CHECK(!info.hasWaveVariant && !info.usesWaveVariant && !info.isXPBD);

    kernels->getKernelInfo(info, KernelId::CollideShapes);
    CHECK(info.readOnlyMask == 0x7fffffffu);          // t0..t30
    CHECK(info.readOnlyTextureMask == 0x7fff8000u);   // t15..t30: the SDF volumes
    CHECK(info.readWriteMask == 0x7 && info.constantBufferMask == 0x3);
    CHECK(info.threadGroupSize[0] == 128);

    kernels->getKernelInfo(info, KernelId::CalculateBounds);
    CHECK(info.hasWaveVariant);
    CHECK(info.usesWaveVariant == phys->getInfo().supportsWaveOps);
    CHECK(info.readOnlyMask == 0x3 && info.readWriteMask == 0x3);

    kernels->getKernelInfo(info, KernelId::CalculateInflatableVolumeXPBD);
    CHECK(info.isXPBD && info.hasWaveVariant && info.readWriteMask == 0x3 && info.constantBufferMask == 0x3);
    kernels->getKernelInfo(info, KernelId::CalculateInflatableVolume);
    CHECK(!info.isXPBD && info.readWriteMask == 0x1);
    // The PBD shape matching kernel reads no constant buffer at all; its XPBD twin reads both.
    kernels->getKernelInfo(info, KernelId::SolveShapes);
    CHECK(info.constantBufferMask == 0);
    kernels->getKernelInfo(info, KernelId::SolveShapesXPBD);
    CHECK(info.constantBufferMask == 0x3);

    uint32_t waveKernels = 0;
    for (uint32_t i = 0; i < kernels->getKernelCount(); ++i) {
        kernels->getKernelInfo(info, KernelId(i));
        CHECK(info.name != nullptr && info.name == kernels->getKernelName(KernelId(i)));
        CHECK(info.threadGroupSize[0] >= 1 && info.threadGroupSize[1] == 1 && info.threadGroupSize[2] == 1);
        CHECK((info.readOnlyTextureMask & ~info.readOnlyMask) == 0);
        waveKernels += info.hasWaveVariant ? 1 : 0;
    }
    // The bounds kernels (3), the BVH bounds kernels (2), the inflatable volume (PBD, XPBD) and shape
    // matching (2 x PBD, XPBD).
    CHECK(waveKernels == 11);

    // Invalid arguments: an error code and one message, never a crash.
    nvrhi::ComputePipelineHandle pipeline;
    nvrhi::BindingLayoutHandle layout;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                kernels->getPipeline(KernelId::Count, KernelVariant::Default, &pipeline));
    CHECK(!pipeline);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->getBindingLayout(KernelId(0xffff), &layout));
    CHECK(kernels->getPipeline(KernelId::ClearInt, KernelVariant::Default, nullptr) == nvrhi::FE_INVALID_ARGS);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                kernels->getPipeline(KernelId::ClearInt, KernelVariant(9), &pipeline));
    // A kernel without a wave variant has none on any device.
    CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages,
                kernels->getPipeline(KernelId::ClearInt, KernelVariant::Wave, &pipeline));
    if (!phys->getInfo().supportsWaveOps)
        CHECK_FAILS(nvrhi::FE_UNSUPPORTED, messages,
                    kernels->getPipeline(KernelId::CalculateBounds, KernelVariant::Wave, &pipeline));

    KernelDispatchDesc bad;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, bad));  // KernelId::Count
    bad.kernel = KernelId::ClearInt;
    bad.rootConstantsSize = 16;  // a size without a pointer
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, bad));
    uint8_t tooLarge[ephys::c_KernelMaxConstantBufferSize + 16] = {};
    bad.rootConstants = tooLarge;
    bad.rootConstantsSize = sizeof(tooLarge);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, bad));
    CHECK(phys->flush() == 0);  // none of them recorded anything
    // The block of another kernel: ClearInt declares 16 bytes at b0.
    bad.rootConstantsSize = sizeof(ephys::detail::KernelParams);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, bad));
    phys->flush();
}

void testPipelines(ephys::IPhysDevice* phys) {
    std::printf("pipelines\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();
    const bool wave = phys->getInfo().supportsWaveOps;

    CHECK(kernels->loadAll() == nvrhi::FS_OK);

    uint32_t pipelines = 0;
    for (uint32_t i = 0; i < kernels->getKernelCount(); ++i) {
        KernelInfo info;
        kernels->getKernelInfo(info, KernelId(i));

        nvrhi::BindingLayoutHandle layout;
        CHECK(kernels->getBindingLayout(KernelId(i), &layout) == nvrhi::FS_OK);
        CHECK(layout != nullptr);

        nvrhi::ComputePipelineHandle baseline, chosen, waved;
        CHECK(kernels->getPipeline(KernelId(i), KernelVariant::Baseline, &baseline) == nvrhi::FS_OK);
        CHECK(kernels->getPipeline(KernelId(i), KernelVariant::Default, &chosen) == nvrhi::FS_OK);
        CHECK(baseline != nullptr && chosen != nullptr);
        if (!baseline || !chosen) {
            std::printf("  no pipeline for %s\n", info.name);
            continue;
        }
        ++pipelines;
        CHECK(baseline->getDesc().bindingLayouts.size() == 1 && baseline->getDesc().bindingLayouts[0] == layout);

        if (info.hasWaveVariant && wave) {
            CHECK(kernels->getPipeline(KernelId(i), KernelVariant::Wave, &waved) == nvrhi::FS_OK);
            CHECK(waved != nullptr && waved != baseline && chosen == waved);
            ++pipelines;
        } else {
            CHECK(chosen == baseline);
        }
    }
    std::printf("  %u pipelines for %u kernels (wave variants %s)\n", pipelines, kernels->getKernelCount(),
                wave ? "on" : "off");
}

// Every kernel with nothing bound: all slots go to the dummy resources, all constants are zero, and one
// thread group runs. With zero counts in KernelParams the kernels do nothing, or touch the first element
// of a dummy. What this proves is that every binding layout matches its kernel, on every backend: the
// D3D12 debug layer and the Vulkan validation layer compare the bound descriptors with what the compiled
// shader declares (descriptor type, buffer versus texture, typed format).
void testNullDispatch(const Gpu& gpu, ephys::IPhysDevice* phys) {
    std::printf("dispatch with null slots\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();
    const bool wave = phys->getInfo().supportsWaveOps;

    uint32_t dispatches = 0;
    gpu.begin();
    for (uint32_t i = 0; i < kernels->getKernelCount(); ++i) {
        KernelInfo info;
        kernels->getKernelInfo(info, KernelId(i));
        KernelDispatchDesc desc;
        desc.kernel = KernelId(i);
        desc.variant = KernelVariant::Baseline;
        CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        ++dispatches;
        if (info.hasWaveVariant && wave) {
            desc.variant = KernelVariant::Wave;
            CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
            ++dispatches;
        }
    }
    // A zero group count records nothing and is not an error.
    KernelDispatchDesc nothing;
    nothing.kernel = KernelId::ClearInt;
    nothing.gridDim[0] = 0;
    CHECK(kernels->dispatch(gpu.commandList, nothing) == nvrhi::FS_OK);
    gpu.end();
    CHECK(gpu.device->waitForIdle());
    std::printf("  %u dispatches\n", dispatches);
}

void testClear(const Gpu& gpu, ephys::IPhysDevice* phys, PhysMessages* messages) {
    std::printf("ClearInt, ClearFloat4\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();

    constexpr uint32_t N = 1000;  // not a multiple of the group size
    constexpr uint32_t groups = (N + 255) / 256;

    // ClearInt writes [0, length) and nothing else.
    {
        nvrhi::BufferHandle buffer = gpu.structured("test ints", 4, N);
        std::vector<int32_t> initial(N);
        for (uint32_t i = 0; i < N; ++i) initial[i] = int32_t(0x7000 + i);
        gpu.upload(buffer, initial);

        ephys::detail::ClearIntParams params;
        params.length = 777;
        params.value = 0x1234;
        KernelDispatchDesc desc;
        desc.kernel = KernelId::ClearInt;
        desc.gridDim[0] = groups;
        desc.rootConstants = &params;
        desc.rootConstantsSize = sizeof(params);
        desc.readWrite[0] = buffer;
        // Slots the kernel does not declare are ignored, whatever is in them.
        desc.readOnly[5] = buffer;
        desc.readWrite[7] = buffer;

        // On the device's own command list.
        CHECK(kernels->dispatch(nullptr, desc) == nvrhi::FS_OK);
        CHECK(phys->waitForIdle() == nvrhi::FS_OK);

        const std::vector<int32_t> result = gpu.download<int32_t>(buffer, N);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < N; ++i) mismatches += result[i] == (i < 777 ? 0x1234 : initial[i]) ? 0 : 1;
        CHECK(mismatches == 0);

        // More distinct constant blocks in one command list than the pool holds: every dispatch must see
        // its own block. Dispatch i clears [0, 100 - i) to i + 1, so element j ends up as 100 - j.
        gpu.begin();
        for (int i = 0; i < 100; ++i) {
            params.length = 100 - i;
            params.value = i + 1;
            CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        }
        // And the same block again, many times: the cached binding set and constant buffer.
        params.length = 3;
        params.value = -5;
        for (int i = 0; i < 50; ++i) CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        gpu.end();

        const std::vector<int32_t> ordered = gpu.download<int32_t>(buffer, N);
        mismatches = 0;
        for (int j = 0; j < 100; ++j) mismatches += ordered[j] == (j < 3 ? -5 : 100 - j) ? 0 : 1;
        for (uint32_t j = 100; j < N; ++j) mismatches += ordered[j] == result[j] ? 0 : 1;
        CHECK(mismatches == 0);

        // A buffer of the wrong kind in a declared slot is refused with a message.
        nvrhi::BufferHandle rawBuffer = gpu.raw("test raw", 256);
        KernelDispatchDesc wrong = desc;
        wrong.readWrite[0] = rawBuffer;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, wrong));
        // And so is a structured buffer whose elements are not the size the kernel declares.
        nvrhi::BufferHandle float4s = gpu.structured("test float4s", 16, 16);
        wrong.readWrite[0] = float4s;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, kernels->dispatch(nullptr, wrong));
        phys->flush();
    }

    // ClearFloat4: the float4 value starts a register of its own in the block.
    {
        nvrhi::BufferHandle buffer = gpu.structured("test float4s", 16, N);
        std::vector<Float4> initial(N, Float4{1.0f, 2.0f, 3.0f, 4.0f});
        gpu.upload(buffer, initial);

        ephys::detail::ClearFloat4Params params;
        params.length = 300;
        params.value[0] = 0.25f;
        params.value[1] = -8.0f;
        params.value[2] = 1.0e9f;
        params.value[3] = 7.0f;
        KernelDispatchDesc desc;
        desc.kernel = KernelId::ClearFloat4;
        desc.gridDim[0] = groups;
        desc.rootConstants = &params;
        desc.rootConstantsSize = sizeof(params);
        desc.readWrite[0] = buffer;
        gpu.begin();
        CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        gpu.end();

        const std::vector<Float4> result = gpu.download<Float4>(buffer, N);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < N; ++i) {
            const Float4 expected = i < 300 ? Float4{0.25f, -8.0f, 1.0e9f, 7.0f} : initial[i];
            mismatches += std::memcmp(&result[i], &expected, sizeof(Float4)) == 0 ? 0 : 1;
        }
        CHECK(mismatches == 0);
    }
}

// The constant buffers as a kernel sees them. Every word of the blocks is distinct, the kernel reads every
// member by name, and the test expects the word at offsetof() of the C++ mirror: the two layouts agree
// exactly when the output matches.
void testKernelParams(const Gpu& gpu, ephys::IPhysDevice* phys) {
    std::printf("KernelParams layout\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();
    using namespace ephys::detail;

    uint32_t root[sizeof(KernelParams) / 4];
    for (uint32_t i = 0; i < sizeof(KernelParams) / 4; ++i) root[i] = 0xA5000000u + i;
    uint32_t second[4];
    for (uint32_t i = 0; i < 4; ++i) second[i] = 0xB6000000u + i;
    static_assert(sizeof(second) == sizeof(KernelParamsXPBD) && sizeof(second) == sizeof(SubstepParams));

    auto word = [](const uint32_t* block, size_t offset) { return block[offset / 4]; };

    for (int xpbd = 0; xpbd < 2; ++xpbd) {
        std::vector<uint32_t> expected;
#define TEST_SCALAR(name) expected.push_back(word(root, offsetof(KernelParams, name)));
#define TEST_FLOAT3(name)                                              \
    expected.push_back(word(root, offsetof(KernelParams, name) + 0)); \
    expected.push_back(word(root, offsetof(KernelParams, name) + 4)); \
    expected.push_back(word(root, offsetof(KernelParams, name) + 8));
#define TEST_FLOAT4_ARRAY(name, count)              \
    for (size_t element = 0; element < count; ++element) \
        for (size_t component = 0; component < 4; ++component) \
            expected.push_back(word(root, offsetof(KernelParams, name) + element * 16 + component * 4));
        EPHYS_KERNEL_PARAMS_FIELDS(TEST_SCALAR, TEST_FLOAT3, TEST_FLOAT4_ARRAY)
#undef TEST_SCALAR
#undef TEST_FLOAT3
#undef TEST_FLOAT4_ARRAY
        // All of KernelParams but the six words of padding (_pad0, _pad1, _pad2, _pad3).
        CHECK(expected.size() == sizeof(KernelParams) / 4 - 6);

        if (xpbd) {
#define TEST_SCALAR(name) expected.push_back(word(second, offsetof(KernelParamsXPBD, name)));
            EPHYS_KERNEL_PARAMS_XPBD_FIELDS(TEST_SCALAR)
#undef TEST_SCALAR
        } else {
#define TEST_SCALAR(name) expected.push_back(word(second, offsetof(SubstepParams, name)));
            EPHYS_SUBSTEP_PARAMS_FIELDS(TEST_SCALAR)
#undef TEST_SCALAR
        }

        const uint32_t count = uint32_t(expected.size());
        nvrhi::BufferHandle probe = gpu.structured("test probe", 4, count);
        std::vector<uint32_t> zeros(count, 0);
        gpu.upload(probe, zeros);

        KernelDispatchDesc desc;
        desc.kernel = xpbd ? KernelId::ProbeKernelParamsXPBD : KernelId::ProbeKernelParams;
        desc.rootConstants = root;
        desc.rootConstantsSize = sizeof(root);
        desc.secondConstants = second;
        desc.secondConstantsSize = sizeof(second);
        desc.readWrite[0] = probe;
        gpu.begin();
        CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        gpu.end();

        const std::vector<uint32_t> result = gpu.download<uint32_t>(probe, count);
        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < count; ++i) {
            if (result[i] != expected[i]) {
                if (mismatches < 8)
                    std::printf("  field word %u: kernel read %08x, the C++ layout says %08x\n", i, result[i],
                                expected[i]);
                ++mismatches;
            }
        }
        CHECK(mismatches == 0);
        std::printf("  %s: %u words compared\n", xpbd ? "b0 + KernelParamsXPBD" : "b0 + SubstepParams", count);
    }
}

void testRadixSort(const Gpu& gpu, ephys::IPhysDevice* phys) {
    std::printf("radix sort\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();

    struct Case {
        uint32_t numKeys;
        uint32_t bits;
        uint32_t keyMask;  // limits the key range, to get many equal keys
    };
    const Case cases[] = {
        {700, 32, 0xffffffffu},     // one block: the single-block kernel
        {1024, 32, 0x3ffu},         // exactly one block, many equal keys
        {1025, 32, 0xffffffffu},    // two blocks, one key in the second
        {5000, 32, 0xffffffffu},    // a partial last block
        {5000, 24, 0x00ffffffu},    // the cell-hash sort of the solver: 24 bits
        {5000, 32, 0xffu},          // few distinct keys: stability shows
        {200000, 32, 0xffffffffu},  // 196 blocks: the counter scan runs several passes
    };

    std::mt19937 random(20261004u);
    for (const Case& test : cases) {
        ephys::detail::RadixSort sort;
        CHECK(sort.create(phys, test.numKeys) == nvrhi::FS_OK);
        const uint32_t capacity = sort.getMaxKeys();
        CHECK(capacity >= test.numKeys && capacity % 1024 == 0);

        // The whole buffer is filled: the elements past numKeys hold keys that would sort to the front if
        // the kernels did not ignore them.
        std::vector<uint32_t> keys(capacity), values(capacity);
        for (uint32_t i = 0; i < capacity; ++i) {
            keys[i] = i < test.numKeys ? (random() & test.keyMask) : 0u;
            values[i] = i;
        }
        // Some keys at the extremes.
        keys[0] = test.keyMask;
        keys[test.numKeys / 2] = 0;
        keys[test.numKeys - 1] = test.keyMask;
        gpu.upload(sort.getKeys(), keys);
        gpu.upload(sort.getValues(), values);

        nvrhi::IBuffer* inputKeys = sort.getKeys();
        gpu.begin();
        CHECK(sort.sort(kernels, gpu.commandList, test.numKeys, test.bits) == nvrhi::FS_OK);
        gpu.end();
        // One block: one swap. Otherwise one swap per 4-bit pass, an even number here.
        CHECK((sort.getKeys() != inputKeys) == (test.numKeys <= 1024));

        const std::vector<uint32_t> sortedKeys = gpu.download<uint32_t>(sort.getKeys(), test.numKeys);
        const std::vector<uint32_t> sortedValues = gpu.download<uint32_t>(sort.getValues(), test.numKeys);

        // The reference: a stable sort of (key, value) by key.
        std::vector<uint32_t> order(test.numKeys);
        for (uint32_t i = 0; i < test.numKeys; ++i) order[i] = i;
        std::stable_sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return keys[a] < keys[b]; });

        uint32_t mismatches = 0;
        for (uint32_t i = 0; i < test.numKeys; ++i) {
            if (sortedKeys[i] != keys[order[i]] || sortedValues[i] != values[order[i]]) {
                if (mismatches < 4)
                    std::printf("  [%u] got (%08x, %u), expected (%08x, %u)\n", i, sortedKeys[i], sortedValues[i],
                                keys[order[i]], values[order[i]]);
                ++mismatches;
            }
        }
        CHECK(mismatches == 0);
        std::printf("  %u keys, %u bits, mask %08x: %u mismatches\n", test.numKeys, test.bits, test.keyMask,
                    mismatches);
    }

    ephys::detail::RadixSort sort;
    CHECK(sort.create(phys, 0) == nvrhi::FE_INVALID_ARGS);
    CHECK(sort.sort(kernels, nullptr, 10, 32) == nvrhi::FE_INVALID_ARGS);  // not created
    CHECK(sort.create(phys, 2000) == nvrhi::FS_OK);
    CHECK(sort.sort(kernels, nullptr, 2049, 32) == nvrhi::FE_INVALID_ARGS);  // beyond the capacity
    CHECK(sort.sort(kernels, nullptr, 100, 30) == nvrhi::FE_INVALID_ARGS);   // not a multiple of 4
    CHECK(sort.sort(kernels, nullptr, 0, 32) == nvrhi::FS_OK);               // nothing to do
    CHECK(phys->flush() == 0);
}

// The three bounds kernels as the solver chains them (FleX's Solver::ComputeBounds), in one variant.
// Returns lower and upper.
std::vector<Float3> computeBounds(const Gpu& gpu, ephys::IPhysKernelLibrary* kernels, KernelVariant variant,
                                  nvrhi::IBuffer* activeIndices, nvrhi::IBuffer* positions, uint32_t numParticles,
                                  float radius) {
    const uint32_t blocks = (numParticles + 127) / 128;
    const uint32_t groups = (blocks + 127) / 128;

    nvrhi::BufferHandle boundsLower = gpu.structured("test bounds lower", 16, blocks);
    nvrhi::BufferHandle boundsUpper = gpu.structured("test bounds upper", 16, blocks);
    nvrhi::BufferHandle groupLower = gpu.structured("test group bounds lower", 16, groups);
    nvrhi::BufferHandle groupUpper = gpu.structured("test group bounds upper", 16, groups);
    nvrhi::BufferHandle particleBounds = gpu.structured("test particle bounds", 12, 2);

    ephys::detail::KernelParams params = {};
    params.kNumParticles = int(numParticles);
    params.kRadius = radius;

    gpu.begin();
    KernelDispatchDesc desc;
    desc.variant = variant;
    desc.rootConstants = &params;
    desc.rootConstantsSize = sizeof(params);

    desc.kernel = KernelId::CalculateBounds;
    desc.readWrite[0] = boundsLower;
    desc.readWrite[1] = boundsUpper;
    desc.readWrite[2] = particleBounds;  // as FleX binds it: only the last kernel declares u2
    desc.readOnly[0] = activeIndices;
    desc.readOnly[1] = positions;
    desc.gridDim[0] = blocks;
    CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);

    desc.kernel = KernelId::CalculateBoundsGroup;
    desc.readWrite[0] = groupLower;
    desc.readWrite[1] = groupUpper;
    desc.readOnly[0] = nullptr;
    desc.readOnly[1] = nullptr;
    desc.readOnly[2] = boundsLower;
    desc.readOnly[3] = boundsUpper;
    desc.gridDim[0] = groups;
    CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);

    desc.kernel = KernelId::CalculateBoundsFinalize;
    desc.readWrite[0] = nullptr;
    desc.readWrite[1] = nullptr;
    desc.readOnly[2] = groupLower;
    desc.readOnly[3] = groupUpper;
    desc.gridDim[0] = 1;
    CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
    gpu.end();

    return gpu.download<Float3>(particleBounds, 2);
}

void testBounds(const Gpu& gpu, ephys::IPhysDevice* phys) {
    std::printf("bounds: baseline and wave\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();

    constexpr uint32_t N = 40001;  // 313 blocks of 128, the last one with a single particle; 3 groups
    const float radius = 0.125f;

    std::mt19937 random(7u);
    std::uniform_real_distribution<float> coordinate(-50.0f, 50.0f);
    std::vector<Float4> positions(N);
    std::vector<int32_t> activeIndices(N);
    for (uint32_t i = 0; i < N; ++i) {
        positions[i] = Float4{coordinate(random), coordinate(random) * 0.5f, coordinate(random) * 2.0f, 1.0f};
        activeIndices[i] = int32_t(N - 1 - i);  // any permutation
    }
    // The extremes sit in different blocks, one of them in the last, partial block.
    positions[activeIndices[N - 1]].x = -777.0f;
    positions[activeIndices[5000]].y = 888.0f;
    positions[activeIndices[127]].z = -999.0f;

    Float3 lower{FLT_MAX, FLT_MAX, FLT_MAX}, upper{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (const Float4& p : positions) {
        lower = Float3{std::min(lower.x, p.x), std::min(lower.y, p.y), std::min(lower.z, p.z)};
        upper = Float3{std::max(upper.x, p.x), std::max(upper.y, p.y), std::max(upper.z, p.z)};
    }
    const Float3 expected[2] = {Float3{lower.x - radius, lower.y - radius, lower.z - radius},
                                Float3{upper.x + radius, upper.y + radius, upper.z + radius}};

    nvrhi::BufferHandle positionBuffer = gpu.structured("test positions", 16, N);
    nvrhi::BufferHandle indexBuffer = gpu.structured("test active indices", 4, N);
    gpu.upload(positionBuffer, positions);
    gpu.upload(indexBuffer, activeIndices);

    const std::vector<Float3> baseline =
        computeBounds(gpu, kernels, KernelVariant::Baseline, indexBuffer, positionBuffer, N, radius);
    CHECK(std::memcmp(baseline.data(), expected, sizeof(expected)) == 0);
    std::printf("  baseline: [%g %g %g] .. [%g %g %g]\n", baseline[0].x, baseline[0].y, baseline[0].z, baseline[1].x,
                baseline[1].y, baseline[1].z);

    if (phys->getInfo().supportsWaveOps) {
        const std::vector<Float3> wave =
            computeBounds(gpu, kernels, KernelVariant::Wave, indexBuffer, positionBuffer, N, radius);
        // Min and max are exact: the wave variant gives the same bits.
        CHECK(std::memcmp(wave.data(), baseline.data(), sizeof(expected)) == 0);
        std::printf("  wave:     [%g %g %g] .. [%g %g %g]\n", wave[0].x, wave[0].y, wave[0].z, wave[1].x, wave[1].y,
                    wave[1].z);
    } else {
        std::printf("  the device has no wave intrinsics: the wave variant is not compared\n");
    }
}

// True for the lanes of a block the groupshared tree of CalculateInflatableVolume folds into its sum: it
// halves the triangle count of the block, which leaves lanes out when the count is not a power of two.
// FleX shipped that; the test pins it, and that the wave variant sums the same lanes.
bool treeCovers(uint32_t lane, uint32_t count) {
    for (uint32_t j = count >> 1; j > 0; j >>= 1)
        if (lane >= j) lane -= j;
    return lane == 0;
}

void testInflatableVolume(const Gpu& gpu, ephys::IPhysDevice* phys) {
    std::printf("inflatable volume: baseline and wave\n");
    ephys::IPhysKernelLibrary* kernels = phys->getKernels();

    struct Inflatable {
        int32_t startTri, numTris;
        float restVolume, constraintScale;
    };
    // 600 triangles: a full block of 512 and a block of 88 (not a power of two). 37: one partial block.
    // 512: no lane is left out.
    const Inflatable inflatables[] = {{0, 600, 1.0f, 1.0f}, {600, 37, 2.0f, 0.5f}, {637, 512, -3.0f, 1.0f}};
    constexpr uint32_t numInflatables = 3;
    constexpr uint32_t numTris = 600 + 37 + 512;
    constexpr uint32_t numParticles = 700;

    std::mt19937 random(99u);
    std::uniform_real_distribution<float> coordinate(-1.0f, 1.0f);
    std::vector<Float4> positions(numParticles);
    for (Float4& p : positions)
        p = Float4{coordinate(random) + 0.3f, coordinate(random), coordinate(random) - 0.2f, 1.0f};
    std::vector<int32_t> indices(numTris * 3);
    for (int32_t& index : indices) index = int32_t(random() % numParticles);
    std::vector<int32_t> reverseLookup(numParticles);
    // A permutation: 11 and 700 have no common factor.
    for (uint32_t i = 0; i < numParticles; ++i) reverseLookup[i] = int32_t((i * 11 + 3) % numParticles);

    // The kernel in double precision: with the lanes the tree covers (what the kernels must compute), and
    // with all triangles (what they must not, where the two differ).
    auto reference = [&](const Inflatable& inflatable, bool allTriangles) {
        auto position = [&](uint32_t tri, uint32_t corner) {
            return positions[reverseLookup[indices[(inflatable.startTri + tri) * 3 + corner]]];
        };
        auto covered = [&](uint32_t tri) {
            if (allTriangles) return true;
            const uint32_t block = tri / 512;
            const uint32_t inBlock = std::min<uint32_t>(inflatable.numTris - block * 512, 512);
            return treeCovers(tri % 512, inBlock);
        };
        double center[3] = {0, 0, 0};
        for (uint32_t tri = 0; tri < uint32_t(inflatable.numTris); ++tri) {
            if (!covered(tri)) continue;
            for (uint32_t corner = 0; corner < 3; ++corner) {
                const Float4 p = position(tri, corner);
                center[0] += p.x;
                center[1] += p.y;
                center[2] += p.z;
            }
        }
        for (double& c : center) c /= 3.0 * inflatable.numTris;
        double volume = 0;
        for (uint32_t tri = 0; tri < uint32_t(inflatable.numTris); ++tri) {
            if (!covered(tri)) continue;
            double p[3][3];
            for (uint32_t corner = 0; corner < 3; ++corner) {
                const Float4 q = position(tri, corner);
                p[corner][0] = q.x - center[0];
                p[corner][1] = q.y - center[1];
                p[corner][2] = q.z - center[2];
            }
            for (int a = 0; a < 3; ++a) {
                p[1][a] -= p[0][a];
                p[2][a] -= p[0][a];
            }
            const double cross[3] = {p[1][1] * p[2][2] - p[1][2] * p[2][1], p[1][2] * p[2][0] - p[1][0] * p[2][2],
                                     p[1][0] * p[2][1] - p[1][1] * p[2][0]};
            volume += p[0][0] * cross[0] + p[0][1] * cross[1] + p[0][2] * cross[2];
        }
        const double rest = inflatable.restVolume;
        const double k = std::fabs(rest) / std::max(std::fabs(volume), std::fabs(rest * 0.01));
        return (volume - rest) * inflatable.constraintScale * k * k * k;
    };

    // A relative comparison: the values span several orders of magnitude.
    auto close = [](double a, double b, double tolerance) {
        return std::fabs(a - b) <= tolerance * std::max(std::fabs(a), std::fabs(b)) + 1.0e-9;
    };

    double expected[numInflatables];
    for (uint32_t n = 0; n < numInflatables; ++n) expected[n] = reference(inflatables[n], false);
    // The test data tells the two apart: 600 and 37 triangles leave lanes out, 512 does not.
    CHECK(!close(reference(inflatables[0], true), expected[0], 1.0e-2));
    CHECK(!close(reference(inflatables[1], true), expected[1], 1.0e-2));
    CHECK(reference(inflatables[2], true) == expected[2]);

    nvrhi::BufferHandle inflatableBuffer = gpu.structured("test inflatables", 16, numInflatables);
    nvrhi::BufferHandle positionBuffer = gpu.structured("test positions", 16, numParticles);
    nvrhi::BufferHandle reverseBuffer = gpu.structured("test reverse lookup", 4, numParticles);
    nvrhi::BufferHandle indexBuffer = gpu.structured("test indices", 4, numTris * 3);
    gpu.upload(inflatableBuffer, inflatables, sizeof(inflatables));
    gpu.upload(positionBuffer, positions);
    gpu.upload(reverseBuffer, reverseLookup);
    gpu.upload(indexBuffer, indices);

    auto run = [&](KernelVariant variant) {
        nvrhi::BufferHandle lambdas = gpu.structured("test lambdas", 4, numInflatables);
        ephys::detail::KernelParams params = {};
        KernelDispatchDesc desc;
        desc.kernel = KernelId::CalculateInflatableVolume;
        desc.variant = variant;
        desc.rootConstants = &params;
        desc.rootConstantsSize = sizeof(params);
        desc.readOnly[0] = inflatableBuffer;
        desc.readOnly[1] = positionBuffer;
        desc.readOnly[2] = reverseBuffer;
        desc.readOnly[3] = indexBuffer;
        desc.readWrite[0] = lambdas;
        desc.gridDim[0] = numInflatables;  // one thread group per inflatable
        gpu.begin();
        CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
        gpu.end();
        return gpu.download<float>(lambdas, numInflatables);
    };

    const std::vector<float> baseline = run(KernelVariant::Baseline);
    for (uint32_t n = 0; n < numInflatables; ++n) {
        CHECK(close(baseline[n], expected[n], 2.0e-3));
        std::printf("  inflatable %u: reference %.6f (all triangles %.6f), baseline %.6f\n", n, expected[n],
                    reference(inflatables[n], true), double(baseline[n]));
    }
    if (phys->getInfo().supportsWaveOps) {
        const std::vector<float> wave = run(KernelVariant::Wave);
        for (uint32_t n = 0; n < numInflatables; ++n) {
            // The same lanes in another order: float rounding only.
            CHECK(close(wave[n], baseline[n], 1.0e-3));
            CHECK(close(wave[n], expected[n], 2.0e-3));
            std::printf("  inflatable %u: wave %.6f\n", n, double(wave[n]));
        }
        // Reproducible on one device.
        const std::vector<float> again = run(KernelVariant::Wave);
        CHECK(std::memcmp(again.data(), wave.data(), numInflatables * sizeof(float)) == 0);
    } else {
        std::printf("  the device has no wave intrinsics: the wave variant is not compared\n");
    }
}

// Blobs that are missing or damaged: an FE_* code and a message that names the file. No nvrhi message, no
// crash, and the kernels that are intact keep working.
void testBrokenBlobs(nvrhi::IDevice* device, PhysMessages* messages, const std::string& shaderPath) {
    std::printf("missing and damaged blobs\n");
    namespace fs = std::filesystem;

    const char* format = device->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D12
                             ? "dxil"
                             : device->getGraphicsAPI() == nvrhi::GraphicsAPI::VULKAN ? "spirv" : "dxbc";

    std::error_code error;
    const fs::path scratch = fs::temp_directory_path(error) /
                             ("ethereal_phys_kernel_test_" + std::string(format) + "_" + std::to_string(std::rand()));
    fs::create_directories(scratch / format, error);
    fs::copy(fs::u8path(shaderPath) / format, scratch / format,
             fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
    CHECK(!error);
    if (error) return;

    auto fileOf = [&](const char* name) { return scratch / format / name; };
    auto readAll = [](const fs::path& path) {
        std::ifstream in(path, std::ios::binary);
        return std::vector<char>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    };
    auto writeAll = [](const fs::path& path, const std::vector<char>& data) {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(data.data(), std::streamsize(data.size()));
    };

    // Missing.
    fs::remove(fileOf("ClearInt_ClearInt.bin"), error);
    // Empty.
    writeAll(fileOf("ClearFloat4_ClearFloat4.bin"), {});
    // Truncated: the first two thirds of a raw shader.
    std::vector<char> blob = readAll(fileOf("Predict_Predict.bin"));
    CHECK(blob.size() > 64);
    blob.resize(blob.size() * 2 / 3);
    writeAll(fileOf("Predict_Predict.bin"), blob);
    // Not a shader at all.
    writeAll(fileOf("CreateGrid_CreateGrid.bin"), std::vector<char>(4096, 'x'));
    // A permutation container cut inside its second permutation.
    blob = readAll(fileOf("CalculateBounds_CalculateBounds.bin"));
    CHECK(blob.size() > 64);
    blob.resize(blob.size() * 3 / 4);
    writeAll(fileOf("CalculateBounds_CalculateBounds.bin"), blob);
    // A permutation container that is only its magic number.
    writeAll(fileOf("SolveShapes_SolveShapes.bin"), {'N', 'V', 'S', 'P'});
    // A raw shader where a permutation container is expected (built from another manifest).
    fs::copy_file(fileOf("ApplyDeltas_ApplyDeltas.bin"), fileOf("CalculateBoundsGroup_CalculateBoundsGroup.bin"),
                  fs::copy_options::overwrite_existing, error);

    ephys::PhysDeviceDesc deviceDesc;
    deviceDesc.device = device;
    deviceDesc.messageCallback = messages;
    const std::string scratchPath = scratch.u8string();
    deviceDesc.shaderPath = scratchPath.c_str();
    ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
    CHECK(phys != nullptr);
    if (phys) {
        ephys::IPhysKernelLibrary* kernels = phys->getKernels();
        nvrhi::ComputePipelineHandle pipeline;
        messages->quiet = true;

        auto fails = [&](KernelId kernel, KernelVariant variant, nvrhi::FRESULT expected, const char* fileName) {
            const int before = messages->errors.load();
            const nvrhi::FRESULT result = kernels->getPipeline(kernel, variant, &pipeline);
            const bool ok = result == expected && !pipeline && messages->errors.load() == before + 1 &&
                            messages->last.find(fileName) != std::string::npos;
            if (!ok)
                std::printf("  %s: result %d (expected %d), message: %s\n", kernels->getKernelName(kernel), int(result),
                            int(expected), messages->last.c_str());
            CHECK(ok);
        };

        fails(KernelId::ClearInt, KernelVariant::Default, nvrhi::FE_NOT_FOUND, "ClearInt_ClearInt.bin");
        fails(KernelId::ClearFloat4, KernelVariant::Default, nvrhi::FE_GENERIC_ERROR, "ClearFloat4_ClearFloat4.bin");
        fails(KernelId::Predict, KernelVariant::Default, nvrhi::FE_GENERIC_ERROR, "Predict_Predict.bin");
        fails(KernelId::CreateGrid, KernelVariant::Default, nvrhi::FE_GENERIC_ERROR, "CreateGrid_CreateGrid.bin");
        // The container holds EPHYS_WAVE=0 whole and EPHYS_WAVE=1 cut: the first still loads.
        CHECK(kernels->getPipeline(KernelId::CalculateBounds, KernelVariant::Baseline, &pipeline) == nvrhi::FS_OK);
        CHECK(pipeline != nullptr);
        pipeline = nullptr;
        if (phys->getInfo().supportsWaveOps)
            fails(KernelId::CalculateBounds, KernelVariant::Wave, nvrhi::FE_NOT_FOUND,
                  "CalculateBounds_CalculateBounds.bin");
        fails(KernelId::SolveShapes, KernelVariant::Baseline, nvrhi::FE_NOT_FOUND, "SolveShapes_SolveShapes.bin");
        fails(KernelId::SolveShapesXPBD, KernelVariant::Baseline, nvrhi::FE_NOT_FOUND, "SolveShapes_SolveShapes.bin");
        fails(KernelId::CalculateBoundsGroup, KernelVariant::Baseline, nvrhi::FE_NOT_FOUND,
              "CalculateBoundsGroup_CalculateBoundsGroup.bin");

        // A failed kernel fails again (it is retried, not remembered as loaded), and a dispatch of it
        // records nothing.
        fails(KernelId::ClearInt, KernelVariant::Default, nvrhi::FE_NOT_FOUND, "ClearInt_ClearInt.bin");
        KernelDispatchDesc desc;
        desc.kernel = KernelId::ClearInt;
        const int before = messages->errors.load();
        CHECK(kernels->dispatch(nullptr, desc) == nvrhi::FE_NOT_FOUND);
        CHECK(messages->errors.load() == before + 1);
        CHECK(phys->flush() == 0);

        // loadAll reports every broken kernel and returns the first failure; the intact ones are loaded.
        const int beforeAll = messages->errors.load();
        CHECK(kernels->loadAll() == nvrhi::FE_NOT_FOUND);  // ClearInt comes first
        const int reported = messages->errors.load() - beforeAll;
        // ClearInt, ClearFloat4, Predict, CreateGrid, CalculateBoundsGroup (x2 with wave), SolveShapes and
        // SolveShapesXPBD (x2 with wave), CalculateBounds (wave only).
        CHECK(reported == (phys->getInfo().supportsWaveOps ? 11 : 7));
        CHECK(kernels->getPipeline(KernelId::ApplyDeltas, KernelVariant::Default, &pipeline) == nvrhi::FS_OK);
        CHECK(pipeline != nullptr);
        pipeline = nullptr;

        messages->quiet = false;
        phys = nullptr;
    }

    // No shader path at all, and a path that does not exist.
    for (int pass = 0; pass < 2; ++pass) {
        ephys::PhysDeviceDesc desc;
        desc.device = device;
        desc.messageCallback = messages;
        desc.shaderPath = pass ? "this/directory/does/not/exist" : nullptr;
        ephys::PhysDeviceHandle none = ephys::createDevice(desc);
        CHECK(none != nullptr);
        if (!none) continue;
        CHECK(none->getKernels() != nullptr);
        CHECK(std::strcmp(none->getKernels()->getKernelName(KernelId::Finalize), "Finalize") == 0);
        nvrhi::ComputePipelineHandle pipeline;
        messages->quiet = true;
        CHECK_FAILS(nvrhi::FE_NOT_FOUND, messages,
                    none->getKernels()->getPipeline(KernelId::Finalize, KernelVariant::Default, &pipeline));
        messages->quiet = false;
        CHECK(!pipeline);
    }

    device->waitForIdle();
    device->runGarbageCollection();
    fs::remove_all(scratch, error);
}

// The library may be held past its device.
void testLibraryOutlivesDevice(const Gpu& gpu, PhysMessages* messages, const std::string& shaderPath) {
    std::printf("library held past the device\n");
    ephys::PhysDeviceDesc deviceDesc;
    deviceDesc.device = gpu.device;
    deviceDesc.messageCallback = messages;
    deviceDesc.shaderPath = shaderPath.c_str();
    ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
    CHECK(phys != nullptr);
    if (!phys) return;

    ephys::PhysKernelLibraryHandle kernels = phys->getKernels();
    CHECK(phys.Detach()->Release() == 0);

    CHECK(std::strcmp(kernels->getKernelName(KernelId::ClearInt), "ClearInt") == 0);
    nvrhi::BufferHandle buffer = gpu.structured("test ints", 4, 256);
    std::vector<int32_t> zeros(256, 0);
    gpu.upload(buffer, zeros);

    ephys::detail::ClearIntParams params;
    params.length = 256;
    params.value = 42;
    KernelDispatchDesc desc;
    desc.kernel = KernelId::ClearInt;
    desc.rootConstants = &params;
    desc.rootConstantsSize = sizeof(params);
    desc.readWrite[0] = buffer;
    // The device's own command list is gone ...
    CHECK_FAILS(nvrhi::FE_NOT_ALIVE_OBJECT, messages, kernels->dispatch(nullptr, desc));
    // ... the application's still works.
    gpu.begin();
    CHECK(kernels->dispatch(gpu.commandList, desc) == nvrhi::FS_OK);
    gpu.end();
    const std::vector<int32_t> result = gpu.download<int32_t>(buffer, 256);
    CHECK(std::count(result.begin(), result.end(), 42) == 256);

    // The cached binding set holds the buffer; clearing the cache lets it go.
    buffer->AddRef();
    const nvrhi::FLONG held = buffer->Release();
    kernels->clearBindingCache();
    gpu.device->waitForIdle();
    gpu.device->runGarbageCollection();
    buffer->AddRef();
    CHECK(buffer->Release() < held);

    CHECK(kernels.Detach()->Release() == 0);
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
        shaderPath = (donut::app::GetDirectoryWithExecutable() / "shaders" / "ethereal_phys").generic_u8string();
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
        D3DMessages d3dMessages(device);
        if (api != nvrhi::GraphicsAPI::VULKAN) {
            CHECK(d3dMessages.attached());
            std::printf("d3d debug layer: %s\n", d3dMessages.attached() ? "watched" : "NOT available");
        }

        Gpu gpu;
        gpu.device = device;
        CHECK(device->createCommandList(nvrhi::CommandListParameters(), &gpu.commandList) == nvrhi::FS_OK);

        ephys::PhysDeviceDesc deviceDesc;
        deviceDesc.device = device;
        deviceDesc.messageCallback = physMessages;
        deviceDesc.shaderPath = shaderPath.c_str();
        ephys::PhysDeviceHandle phys = ephys::createDevice(deviceDesc);
        CHECK(phys != nullptr);
        if (phys && gpu.commandList) {
            const ephys::PhysDeviceInfo& info = phys->getInfo();
            std::printf("wave ops %d (lanes %u..%u)\n", int(info.supportsWaveOps), info.minWaveLaneCount,
                        info.maxWaveLaneCount);

            testCatalogue(phys, physMessages);
            testPipelines(phys);
            CHECK(d3dMessages.problems() == 0);
            testNullDispatch(gpu, phys);
            CHECK(d3dMessages.problems() == 0);
            testClear(gpu, phys, physMessages);
            testKernelParams(gpu, phys);
            testRadixSort(gpu, phys);
            testBounds(gpu, phys);
            testInflatableVolume(gpu, phys);

            CHECK(phys->waitForIdle() == nvrhi::FS_OK);
            device->runGarbageCollection();
            CHECK(phys.Detach()->Release() == 0);
        }

        if (gpu.commandList) {
            testBrokenBlobs(device, physMessages, shaderPath);
            testLibraryOutlivesDevice(gpu, physMessages, shaderPath);
        }

        device->waitForIdle();
        device->runGarbageCollection();
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
