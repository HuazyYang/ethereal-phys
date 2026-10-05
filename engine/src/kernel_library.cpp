#include "kernel_library.h"

#include "phys_messages.h"

#include <ShaderToolBlob.h>

#include <nvrhi/core/foundation.h>

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <new>
#include <string>
#include <unordered_map>
#include <vector>

namespace ephys {
namespace detail {

// ---- The kernel table (generated) -----------------------------------------------------------------------

enum class KernelBindingKind : uint8_t {
    ConstantBuffer,
    StructuredSRV,
    RawSRV,
    TypedSRV,
    TextureSRV,
    StructuredUAV,
    RawUAV,
    TypedUAV,
    Sampler,
};

struct KernelBindingDesc {
    KernelBindingKind kind;
    uint8_t slot;
    nvrhi::Format format;  // the view format of a typed buffer, UNKNOWN otherwise
    uint16_t size;         // bytes: the element of a structured buffer, or the whole constant buffer; else 0
};

// A binding layout: a run of c_KernelBindings. Kernels that declare the same resources share one.
struct KernelLayoutDesc {
    uint16_t first;
    uint16_t count;
};

struct KernelDesc {
    const char* name;
    const char* blob;   // relative to <shaderPath>/<format>/
    const char* entry;  // the HLSL entry point
    uint16_t layout;
    bool hasWave;       // the blob has the EPHYS_WAVE axis
    int8_t xpbd;        // the value of the NVFLEX_XPBD axis, -1 when the blob has no such axis
    uint32_t threadGroupSize[3];
};

#include "kernel_table.inl"

constexpr uint32_t c_KernelCount = uint32_t(KernelId::Count);
constexpr uint32_t c_LayoutCount = uint32_t(sizeof(c_KernelLayouts) / sizeof(c_KernelLayouts[0]));
static_assert(sizeof(c_KernelTable) / sizeof(c_KernelTable[0]) == c_KernelCount);

// The most bindings a layout can have: every slot of every register class and the sampler.
constexpr uint32_t c_MaxBindings =
    c_KernelConstantBufferSlots + c_KernelReadOnlySlots + c_KernelReadWriteSlots + 1;

}  // namespace detail

namespace {

using detail::c_KernelBindings;
using detail::c_KernelCount;
using detail::c_KernelLayouts;
using detail::c_KernelTable;
using detail::c_LayoutCount;
using detail::c_MaxBindings;
using detail::KernelBindingDesc;
using detail::KernelBindingKind;
using detail::KernelDesc;
using detail::KernelLayoutDesc;
using detail::report;

// The size of the dummy buffers, rounded down to whole elements for a structured one (the largest element
// a kernel declares is 48 bytes): what a kernel may touch of a slot that was left null.
constexpr uint32_t c_DummyBytes = 64;

// Constant buffers come in two sizes: for the kernels whose cbuffer is small (the clears, the radix sort,
// the BVH build, b1) and for KernelParams. Which one a binding gets follows from the size the kernel
// declares, not from the size of the block passed in: a buffer smaller than the cbuffer is an error on
// Vulkan and a warning on D3D11.
constexpr uint32_t c_SmallBlockBytes = 64;
constexpr uint32_t c_LargeBlockBytes = c_KernelMaxConstantBufferSize;
// The number of constant buffers of each size the library keeps. A step dispatches with few distinct
// blocks (one KernelParams per substep, eight radix sort passes per sort size, ...); when more are in use
// than this, the least recently used one is rewritten.
constexpr uint32_t c_BlocksPerSize = 64;

// The binding set cache: at most this many sets; entries not used for c_BindingSetMaxAge dispatches are
// dropped when it fills up.
constexpr size_t c_BindingSetCacheSize = 1024;
constexpr uint64_t c_BindingSetMaxAge = 4096;

uint64_t fnv1a(const void* data, size_t size, uint64_t hash = 14695981039346656037ull) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

const char* formatDirectory(nvrhi::GraphicsAPI api) {
    switch (api) {
        case nvrhi::GraphicsAPI::D3D11:
            return "dxbc";
        case nvrhi::GraphicsAPI::D3D12:
            return "dxil";
        case nvrhi::GraphicsAPI::VULKAN:
            return "spirv";
    }
    return "";
}

// A structural check of a compiled kernel, so that a truncated or foreign file is an error here and not
// undefined behaviour in a driver. Returns nullptr when the binary looks like a shader of the API.
const char* checkShaderBinary(nvrhi::GraphicsAPI api, const uint8_t* data, size_t size) {
    if (api == nvrhi::GraphicsAPI::VULKAN) {
        // A SPIR-V module: the magic number, a 5-word header, then instructions whose word counts add up
        // to the size of the module, the last one being OpFunctionEnd (a module cut at an instruction
        // boundary still fails that, unless it was cut between two functions).
        if (size < 20 || size % 4 != 0) return "it is not a SPIR-V module (bad size)";
        uint32_t magic;
        std::memcpy(&magic, data, 4);
        if (magic != 0x07230203u) return "it is not a SPIR-V module (bad magic number)";
        const size_t words = size / 4;
        size_t cursor = 5;
        uint32_t lastOpcode = 0;
        while (cursor < words) {
            uint32_t instruction;
            std::memcpy(&instruction, data + cursor * 4, 4);
            const uint32_t wordCount = instruction >> 16;
            if (wordCount == 0 || wordCount > words - cursor) return "the SPIR-V module is truncated or corrupt";
            lastOpcode = instruction & 0xffffu;
            cursor += wordCount;
        }
        const uint32_t opFunctionEnd = 56;
        if (lastOpcode != opFunctionEnd)
            return "the SPIR-V module is truncated or corrupt (it does not end a function)";
        return nullptr;
    }

    // A DXBC container (DXIL uses the same container): "DXBC", a 16-byte digest, a version, the total size
    // and the chunk count, then one offset per chunk.
    if (size < 32 || std::memcmp(data, "DXBC", 4) != 0) return "it is not a DXBC/DXIL container";
    uint32_t totalSize, chunkCount;
    std::memcpy(&totalSize, data + 24, 4);
    std::memcpy(&chunkCount, data + 28, 4);
    if (totalSize != size) return "the DXBC/DXIL container is truncated or corrupt (size mismatch)";
    if (chunkCount == 0 || chunkCount > (size - 32) / 4) return "the DXBC/DXIL container is corrupt (chunk table)";
    for (uint32_t i = 0; i < chunkCount; ++i) {
        uint32_t offset;
        std::memcpy(&offset, data + 32 + size_t(i) * 4, 4);
        if (offset > size - 8) return "the DXBC/DXIL container is corrupt (chunk offset)";
        uint32_t chunkSize;
        std::memcpy(&chunkSize, data + offset + 4, 4);
        if (chunkSize > size - 8 - offset) return "the DXBC/DXIL container is corrupt (chunk size)";
    }
    return nullptr;
}

bool isUAV(KernelBindingKind kind) {
    return kind == KernelBindingKind::StructuredUAV || kind == KernelBindingKind::RawUAV ||
           kind == KernelBindingKind::TypedUAV;
}

NVRHI_CLASS_CLSID(KernelLibrary, "4c70e858-8e4f-4c24-afb0-208289e01173")
class KernelLibrary final : public nvrhi::ObjectImpl<IPhysKernelLibrary> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(KernelLibrary)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(KernelLibrary)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysKernelLibrary)
    NVRHI_IMPLEMENTS_CLASS(KernelLibrary)
    NVRHI_END_INTERFACE_TABLE()

    KernelLibrary(nvrhi::IDevice* device, IPhysMessageCallback* callback, const char* shaderPath,
                  const PhysDeviceInfo& info, detail::KernelLibraryHost* host)
        : m_Device(device), m_Callback(callback), m_Host(host), m_API(info.graphicsAPI),
          m_WaveSupported(info.supportsWaveOps) {
        if (shaderPath && shaderPath[0]) {
            m_ShaderDirectory = shaderPath;
            m_ShaderDirectory += '/';
            m_ShaderDirectory += formatDirectory(m_API);
        }
    }

    void detach() noexcept { m_Host = nullptr; }

    // ---- IPhysKernelLibrary ----

    uint32_t getKernelCount() const noexcept override { return c_KernelCount; }

    const char* getKernelName(KernelId kernel) const noexcept override {
        return uint32_t(kernel) < c_KernelCount ? c_KernelTable[uint32_t(kernel)].name : nullptr;
    }

    KernelInfo& getKernelInfo(KernelInfo& retVal, KernelId kernel) const noexcept override {
        retVal = KernelInfo();
        if (uint32_t(kernel) >= c_KernelCount) return retVal;

        const KernelDesc& desc = c_KernelTable[uint32_t(kernel)];
        retVal.name = desc.name;
        for (int i = 0; i < 3; ++i) retVal.threadGroupSize[i] = desc.threadGroupSize[i];
        const KernelLayoutDesc& layout = c_KernelLayouts[desc.layout];
        for (uint32_t i = 0; i < layout.count; ++i) {
            const KernelBindingDesc& binding = c_KernelBindings[layout.first + i];
            switch (binding.kind) {
                case KernelBindingKind::ConstantBuffer:
                    retVal.constantBufferMask |= uint8_t(1u << binding.slot);
                    break;
                case KernelBindingKind::TextureSRV:
                    retVal.readOnlyTextureMask |= 1u << binding.slot;
                    retVal.readOnlyMask |= 1u << binding.slot;
                    break;
                case KernelBindingKind::StructuredSRV:
                case KernelBindingKind::RawSRV:
                case KernelBindingKind::TypedSRV:
                    retVal.readOnlyMask |= 1u << binding.slot;
                    break;
                case KernelBindingKind::StructuredUAV:
                case KernelBindingKind::RawUAV:
                case KernelBindingKind::TypedUAV:
                    retVal.readWriteMask |= uint8_t(1u << binding.slot);
                    break;
                case KernelBindingKind::Sampler:
                    break;
            }
        }
        retVal.hasWaveVariant = desc.hasWave;
        retVal.usesWaveVariant = desc.hasWave && m_WaveSupported;
        retVal.isXPBD = desc.xpbd == 1;
        return retVal;
    }

    nvrhi::FRESULT getPipeline(KernelId kernel, KernelVariant variant,
                               nvrhi::IComputePipeline** ppPipeline) noexcept override {
        if (!ppPipeline) return nvrhi::FE_INVALID_ARGS;
        *ppPipeline = nullptr;
        return guarded([&] {
            nvrhi::IComputePipeline* pipeline = nullptr;
            const nvrhi::FRESULT result = findPipeline(kernel, variant, &pipeline);
            if (NVRHI_FAILED(result)) return result;
            pipeline->AddRef();
            *ppPipeline = pipeline;
            return nvrhi::FS_OK;
        });
    }

    nvrhi::FRESULT getBindingLayout(KernelId kernel, nvrhi::IBindingLayout** ppLayout) noexcept override {
        if (!ppLayout) return nvrhi::FE_INVALID_ARGS;
        *ppLayout = nullptr;
        return guarded([&] {
            if (uint32_t(kernel) >= c_KernelCount) return invalidKernel("getBindingLayout", kernel);
            nvrhi::IBindingLayout* layout = nullptr;
            const nvrhi::FRESULT result = findLayout(c_KernelTable[uint32_t(kernel)].layout, &layout);
            if (NVRHI_FAILED(result)) return result;
            layout->AddRef();
            *ppLayout = layout;
            return nvrhi::FS_OK;
        });
    }

    nvrhi::FRESULT loadAll() noexcept override {
        return guarded([&] {
            nvrhi::FRESULT first = nvrhi::FS_OK;
            for (uint32_t i = 0; i < c_KernelCount; ++i) {
                for (int wave = 0; wave < 2; ++wave) {
                    if (wave && !(c_KernelTable[i].hasWave && m_WaveSupported)) continue;
                    nvrhi::IComputePipeline* pipeline = nullptr;
                    const nvrhi::FRESULT result = loadPipeline(i, wave != 0, &pipeline);
                    if (NVRHI_FAILED(result) && NVRHI_SUCCEEDED(first)) first = result;
                }
            }
            return first;
        });
    }

    nvrhi::FRESULT dispatch(nvrhi::ICommandList* commandList, const KernelDispatchDesc& desc) noexcept override {
        return guarded([&] { return dispatchImpl(commandList, desc); });
    }

    void clearBindingCache() noexcept override { m_BindingSets.clear(); }

 private:
    // ---- errors ----

    template <typename F>
    nvrhi::FRESULT guarded(F&& body) noexcept {
        try {
            return body();
        } catch (const std::bad_alloc&) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel library: out of memory");
            return nvrhi::FE_OUT_OF_MEMORY;
        } catch (...) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel library: unexpected exception");
            return nvrhi::FE_GENERIC_ERROR;
        }
    }

    nvrhi::FRESULT invalidKernel(const char* where, KernelId kernel) {
        report(m_Callback, PhysMessageSeverity::Error, "%s: %u is not a kernel id", where, unsigned(kernel));
        return nvrhi::FE_INVALID_ARGS;
    }

    // ---- loading ----

    nvrhi::FRESULT findLayout(uint32_t index, nvrhi::IBindingLayout** ppLayout) {
        if (!m_Layouts[index]) {
            const KernelLayoutDesc& layout = c_KernelLayouts[index];
            nvrhi::BindingLayoutDesc desc;
            desc.visibility = nvrhi::ShaderType::Compute;
            // Register space 0. The default binding offsets (t: 0, s: 128, b: 256, u: 384) are the register
            // shifts ShaderTool compiles the SPIR-V kernels with.
            for (uint32_t i = 0; i < layout.count; ++i) {
                const KernelBindingDesc& binding = c_KernelBindings[layout.first + i];
                switch (binding.kind) {
                    case KernelBindingKind::ConstantBuffer:
                        desc.addItem(nvrhi::BindingLayoutItem::ConstantBuffer(binding.slot));
                        break;
                    case KernelBindingKind::StructuredSRV:
                        desc.addItem(nvrhi::BindingLayoutItem::StructuredBuffer_SRV(binding.slot));
                        break;
                    case KernelBindingKind::RawSRV:
                        desc.addItem(nvrhi::BindingLayoutItem::RawBuffer_SRV(binding.slot));
                        break;
                    case KernelBindingKind::TypedSRV:
                        desc.addItem(nvrhi::BindingLayoutItem::TypedBuffer_SRV(binding.slot));
                        break;
                    case KernelBindingKind::TextureSRV:
                        desc.addItem(nvrhi::BindingLayoutItem::Texture_SRV(binding.slot));
                        break;
                    case KernelBindingKind::StructuredUAV:
                        desc.addItem(nvrhi::BindingLayoutItem::StructuredBuffer_UAV(binding.slot));
                        break;
                    case KernelBindingKind::RawUAV:
                        desc.addItem(nvrhi::BindingLayoutItem::RawBuffer_UAV(binding.slot));
                        break;
                    case KernelBindingKind::TypedUAV:
                        desc.addItem(nvrhi::BindingLayoutItem::TypedBuffer_UAV(binding.slot));
                        break;
                    case KernelBindingKind::Sampler:
                        desc.addItem(nvrhi::BindingLayoutItem::Sampler(binding.slot));
                        break;
                }
            }
            if (NVRHI_FAILED(m_Device->createBindingLayout(desc, &m_Layouts[index])) || !m_Layouts[index]) {
                report(m_Callback, PhysMessageSeverity::Error, "kernel library: cannot create binding layout %u",
                       index);
                return nvrhi::FE_GENERIC_ERROR;
            }
        }
        *ppLayout = m_Layouts[index];
        return nvrhi::FS_OK;
    }

    // Reads a whole file. False when it cannot be opened or read.
    static bool readFile(const std::string& path, std::vector<uint8_t>& out) {
        std::ifstream file(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
        if (!file) return false;
        const std::streamoff size = file.tellg();
        if (size < 0) return false;
        out.resize(size_t(size));
        file.seekg(0);
        if (size > 0 && !file.read(reinterpret_cast<char*>(out.data()), size)) return false;
        return true;
    }

    nvrhi::FRESULT loadPipeline(uint32_t kernel, bool wave, nvrhi::IComputePipeline** ppPipeline) {
        nvrhi::ComputePipelineHandle& slot = m_Pipelines[kernel][wave ? 1 : 0];
        if (slot) {
            *ppPipeline = slot;
            return nvrhi::FS_OK;
        }

        const KernelDesc& desc = c_KernelTable[kernel];
        const char* variantName = desc.hasWave ? (wave ? " (wave)" : " (baseline)") : "";

        if (m_ShaderDirectory.empty()) {
            report(m_Callback, PhysMessageSeverity::Error,
                   "kernel %s: PhysDeviceDesc::shaderPath was not set, so there is no directory to load the "
                   "compiled kernels from",
                   desc.name);
            return nvrhi::FE_NOT_FOUND;
        }

        const std::string path = m_ShaderDirectory + '/' + desc.blob;
        std::vector<uint8_t> blob;
        if (!readFile(path, blob)) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel %s: cannot read '%s'", desc.name, path.c_str());
            return nvrhi::FE_NOT_FOUND;
        }
        if (blob.empty()) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel %s: '%s' is empty", desc.name, path.c_str());
            return nvrhi::FE_GENERIC_ERROR;
        }

        // The permutation. A blob without axes is the raw shader; one with axes is an NVSP container keyed
        // by "NAME=VALUE" pairs (ShaderToolBlob builds the canonical key).
        ShaderToolBlobConstant constants[2];
        size_t constantCount = 0;
        if (desc.hasWave) constants[constantCount++] = {"EPHYS_WAVE", wave ? "1" : "0"};
        if (desc.xpbd >= 0) constants[constantCount++] = {"NVFLEX_XPBD", desc.xpbd ? "1" : "0"};

        const void* binary = nullptr;
        size_t binarySize = 0;
        if (ShaderToolBlobFindPermutation(blob.data(), blob.size(), constants, constantCount, &binary, &binarySize) !=
                0 ||
            !binary || binarySize == 0) {
            report(m_Callback, PhysMessageSeverity::Error,
                   "kernel %s%s: '%s' does not hold the permutation%s%s%s%s (the file is truncated, or was "
                   "built from another manifest)",
                   desc.name, variantName, path.c_str(), desc.hasWave ? " EPHYS_WAVE=" : "",
                   desc.hasWave ? (wave ? "1" : "0") : "", desc.xpbd >= 0 ? " NVFLEX_XPBD=" : "",
                   desc.xpbd >= 0 ? (desc.xpbd ? "1" : "0") : "");
            return nvrhi::FE_NOT_FOUND;
        }

        if (const char* problem = checkShaderBinary(m_API, static_cast<const uint8_t*>(binary), binarySize)) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel %s%s: '%s' is not usable: %s", desc.name,
                   variantName, path.c_str(), problem);
            return nvrhi::FE_GENERIC_ERROR;
        }

        nvrhi::IBindingLayout* layout = nullptr;
        const nvrhi::FRESULT layoutResult = findLayout(desc.layout, &layout);
        if (NVRHI_FAILED(layoutResult)) return layoutResult;

        nvrhi::ShaderDesc shaderDesc;
        shaderDesc.shaderType = nvrhi::ShaderType::Compute;
        shaderDesc.entryName = desc.entry;
        shaderDesc.debugName = desc.name;
        if (desc.hasWave) shaderDesc.debugName += variantName;
        nvrhi::ShaderHandle shader;
        if (NVRHI_FAILED(m_Device->createShader(shaderDesc, binary, binarySize, &shader)) || !shader) {
            report(m_Callback, PhysMessageSeverity::Error, "kernel %s%s: the device refused the shader in '%s'",
                   desc.name, variantName, path.c_str());
            return nvrhi::FE_GENERIC_ERROR;
        }

        nvrhi::ComputePipelineDesc pipelineDesc;
        pipelineDesc.setComputeShader(shader);
        pipelineDesc.addBindingLayout(layout);
        nvrhi::ComputePipelineHandle pipeline;
        if (NVRHI_FAILED(m_Device->createComputePipeline(pipelineDesc, &pipeline)) || !pipeline) {
            report(m_Callback, PhysMessageSeverity::Error,
                   "kernel %s%s: the device could not create a compute pipeline from '%s'", desc.name, variantName,
                   path.c_str());
            return nvrhi::FE_GENERIC_ERROR;
        }

        slot = pipeline;
        *ppPipeline = slot;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT findPipeline(KernelId kernel, KernelVariant variant, nvrhi::IComputePipeline** ppPipeline) {
        if (uint32_t(kernel) >= c_KernelCount) return invalidKernel("kernel library", kernel);
        const KernelDesc& desc = c_KernelTable[uint32_t(kernel)];

        bool wave = false;
        switch (variant) {
            case KernelVariant::Default:
                wave = desc.hasWave && m_WaveSupported;
                break;
            case KernelVariant::Baseline:
                break;
            case KernelVariant::Wave:
                if (!desc.hasWave || !m_WaveSupported) {
                    report(m_Callback, PhysMessageSeverity::Error,
                           "kernel %s: KernelVariant::Wave is not available: %s", desc.name,
                           desc.hasWave ? "the device has no wave intrinsics" : "the kernel has no wave variant");
                    return nvrhi::FE_UNSUPPORTED;
                }
                wave = true;
                break;
            default:
                report(m_Callback, PhysMessageSeverity::Error, "kernel %s: unknown KernelVariant %u", desc.name,
                       unsigned(variant));
                return nvrhi::FE_INVALID_ARGS;
        }
        return loadPipeline(uint32_t(kernel), wave, ppPipeline);
    }

    // ---- dummy resources: what a declared slot is bound to when the caller left it null ----

    //
    // A dummy buffer is made for a binding the first time one is needed, and zero-filled on the command
    // list of that dispatch. There is one per view kind, and
    // - per element stride for structured views: D3D11 compares the stride of a view with the stride the
    //   shader declares;
    // - per slot for UAVs: D3D11 does not allow one buffer in two UAV slots at a time;
    // and SRV and UAV dummies are different buffers, since a buffer cannot be both in one dispatch.

    nvrhi::FRESULT dummyBuffer(nvrhi::ICommandList* commandList, const KernelBindingDesc& binding,
                               nvrhi::IBuffer** ppBuffer) {
        const bool uav = isUAV(binding.kind);
        const bool structured =
            binding.kind == KernelBindingKind::StructuredSRV || binding.kind == KernelBindingKind::StructuredUAV;
        const bool raw = binding.kind == KernelBindingKind::RawSRV || binding.kind == KernelBindingKind::RawUAV;
        const bool typed = binding.kind == KernelBindingKind::TypedSRV || binding.kind == KernelBindingKind::TypedUAV;

        const uint32_t key = uint32_t(binding.kind) | (uav ? uint32_t(binding.slot) << 8 : 0u) |
                             (structured ? uint32_t(binding.size) << 16 : 0u);
        nvrhi::BufferHandle& slot = m_DummyBuffers[key];
        if (!slot) {
            nvrhi::BufferDesc desc;
            desc.debugName = uav ? "ephys dummy (read-write slot)" : "ephys dummy (read-only slot)";
            desc.byteSize = c_DummyBytes;
            if (structured) {
                desc.structStride = binding.size;
                // Whole elements, at least one.
                desc.byteSize = binding.size * (c_DummyBytes / binding.size > 0 ? c_DummyBytes / binding.size : 1u);
            }
            desc.canHaveRawViews = raw;
            desc.canHaveTypedViews = typed;
            if (typed) desc.format = nvrhi::Format::R32_UINT;
            desc.canHaveUAVs = uav;
            // NonPixelShaderResource, not ShaderResource: the latter includes the pixel shader state, which
            // a compute command list (the engine on a compute queue) can neither leave nor restore.
            desc.enableAutomaticStateTracking(uav ? nvrhi::ResourceStates::UnorderedAccess
                                                  : nvrhi::ResourceStates::NonPixelShaderResource);
            nvrhi::BufferHandle buffer;
            if (NVRHI_FAILED(m_Device->createBuffer(desc, &buffer)) || !buffer) {
                m_DummyBuffers.erase(key);
                report(m_Callback, PhysMessageSeverity::Error, "kernel library: cannot create a dummy buffer");
                return nvrhi::FE_GENERIC_ERROR;
            }
            const uint8_t zeros[c_DummyBytes] = {};
            commandList->writeBuffer(buffer, zeros, size_t(desc.byteSize));
            slot = buffer;
        }
        *ppBuffer = slot;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT dummyTexture(nvrhi::ICommandList* commandList, nvrhi::ITexture** ppTexture) {
        if (!m_DummyTexture) {
            nvrhi::TextureDesc desc;
            desc.dimension = nvrhi::TextureDimension::Texture3D;
            desc.width = desc.height = desc.depth = 1;
            desc.format = nvrhi::Format::R32_FLOAT;
            desc.debugName = "ephys dummy volume";
            desc.enableAutomaticStateTracking(nvrhi::ResourceStates::NonPixelShaderResource);
            nvrhi::TextureHandle texture;
            if (NVRHI_FAILED(m_Device->createTexture(desc, &texture)) || !texture) {
                report(m_Callback, PhysMessageSeverity::Error, "kernel library: cannot create the dummy volume");
                return nvrhi::FE_GENERIC_ERROR;
            }
            const float zero = 0.0f;
            commandList->writeTexture(texture, 0, 0, &zero, sizeof(zero));
            m_DummyTexture = texture;
        }
        *ppTexture = m_DummyTexture;
        return nvrhi::FS_OK;
    }

    // FleX's sampler 0, the one the kernels declare (CollideShapes: the SDF volumes): bilinear, no mip
    // interpolation, border addressing with a zero border.
    nvrhi::FRESULT findSampler(nvrhi::ISampler** ppSampler) {
        if (!m_Sampler) {
            nvrhi::SamplerDesc desc;
            desc.setMinFilter(true).setMagFilter(true).setMipFilter(false);
            desc.setAllAddressModes(nvrhi::SamplerAddressMode::Border);
            desc.setBorderColor(nvrhi::Color(0.0f));
            if (NVRHI_FAILED(m_Device->createSampler(desc, &m_Sampler)) || !m_Sampler) {
                report(m_Callback, PhysMessageSeverity::Error, "kernel library: cannot create the sampler");
                return nvrhi::FE_GENERIC_ERROR;
            }
        }
        *ppSampler = m_Sampler;
        return nvrhi::FS_OK;
    }

    // ---- constant buffers ----
    //
    // A pool of ordinary constant buffers, each remembering the bytes it holds. A dispatch whose block
    // equals the contents of a pooled buffer binds that buffer and writes nothing; otherwise the least
    // recently used buffer of the size is rewritten on the command list, ahead of the dispatch.

    struct ConstantBlock {
        nvrhi::BufferHandle buffer;
        uint64_t hash = 0;
        uint64_t lastUse = 0;
        std::array<uint8_t, c_LargeBlockBytes> content = {};
    };

    struct ConstantPool {
        uint32_t blockBytes = 0;
        std::vector<ConstantBlock> blocks;
    };

    // declaredSize is the size of the cbuffer in the kernel; the buffer bound is at least that large.
    nvrhi::FRESULT acquireConstants(nvrhi::ICommandList* commandList, const void* data, uint32_t size,
                                    uint32_t declaredSize, nvrhi::IBuffer** ppBuffer) {
        ConstantPool& pool = declaredSize <= c_SmallBlockBytes ? m_SmallBlocks : m_LargeBlocks;

        std::array<uint8_t, c_LargeBlockBytes> content = {};
        if (size) std::memcpy(content.data(), data, size);
        const uint64_t hash = fnv1a(content.data(), pool.blockBytes);

        ConstantBlock* oldest = nullptr;
        for (ConstantBlock& block : pool.blocks) {
            if (block.hash == hash && std::memcmp(block.content.data(), content.data(), pool.blockBytes) == 0) {
                block.lastUse = m_Tick;
                *ppBuffer = block.buffer;
                return nvrhi::FS_OK;
            }
            // A block bound by this very dispatch (the other constant buffer slot) is not a candidate.
            if (block.lastUse != m_Tick && (!oldest || block.lastUse < oldest->lastUse)) oldest = &block;
        }

        ConstantBlock* block = oldest;
        if (pool.blocks.size() < c_BlocksPerSize) {
            nvrhi::BufferDesc desc;
            desc.byteSize = pool.blockBytes;
            desc.isConstantBuffer = true;
            desc.debugName = pool.blockBytes == c_SmallBlockBytes ? "ephys kernel constants (small)"
                                                                 : "ephys kernel constants";
            desc.enableAutomaticStateTracking(nvrhi::ResourceStates::ConstantBuffer);
            nvrhi::BufferHandle buffer;
            if (NVRHI_FAILED(m_Device->createBuffer(desc, &buffer)) || !buffer) {
                report(m_Callback, PhysMessageSeverity::Error, "kernel library: cannot create a constant buffer");
                return nvrhi::FE_GENERIC_ERROR;
            }
            pool.blocks.emplace_back();
            block = &pool.blocks.back();
            block->buffer = buffer;
        }

        commandList->writeBuffer(block->buffer, content.data(), pool.blockBytes);
        block->content = content;
        block->hash = hash;
        block->lastUse = m_Tick;
        *ppBuffer = block->buffer;
        return nvrhi::FS_OK;
    }

    // ---- binding sets ----

    struct BindingKey {
        uint32_t layout = 0;
        uint32_t count = 0;
        std::array<const void*, c_MaxBindings> resources = {};

        bool operator==(const BindingKey& other) const {
            return layout == other.layout && count == other.count &&
                   std::memcmp(resources.data(), other.resources.data(), count * sizeof(void*)) == 0;
        }
    };

    struct BindingKeyHash {
        size_t operator()(const BindingKey& key) const {
            return size_t(fnv1a(key.resources.data(), key.count * sizeof(void*), 14695981039346656037ull ^ key.layout));
        }
    };

    struct BindingEntry {
        nvrhi::BindingSetHandle set;
        uint64_t lastUse = 0;
    };

    // A buffer the caller put into a slot must be usable as the view the kernel declares there. nvrhi's
    // validation layer checks the same, but only when it is enabled.
    bool checkBuffer(const KernelDesc& kernel, const KernelBindingDesc& binding, nvrhi::IBuffer* buffer) {
        const nvrhi::BufferDesc& desc = buffer->getDesc();
        const char* problem = nullptr;
        switch (binding.kind) {
            case KernelBindingKind::StructuredSRV:
            case KernelBindingKind::StructuredUAV:
                if (desc.structStride == 0) {
                    problem = "the kernel declares a structured buffer; the buffer has no struct stride";
                } else if (desc.structStride != binding.size) {
                    report(m_Callback, PhysMessageSeverity::Error,
                           "kernel %s: %s%u] (%s): the kernel declares elements of %u bytes; the struct stride of "
                           "the buffer is %u",
                           kernel.name, isUAV(binding.kind) ? "readWrite[" : "readOnly[", unsigned(binding.slot),
                           desc.debugName.empty() ? "<unnamed>" : desc.debugName.c_str(), unsigned(binding.size),
                           unsigned(desc.structStride));
                    return false;
                }
                break;
            case KernelBindingKind::RawSRV:
            case KernelBindingKind::RawUAV:
                if (!desc.canHaveRawViews)
                    problem = "the kernel declares a raw buffer; the buffer cannot have raw views";
                break;
            case KernelBindingKind::TypedSRV:
            case KernelBindingKind::TypedUAV:
                if (!desc.canHaveTypedViews)
                    problem = "the kernel declares a typed buffer; the buffer cannot have typed views";
                break;
            default:
                break;
        }
        if (!problem && isUAV(binding.kind) && !desc.canHaveUAVs) problem = "the buffer cannot have UAVs";
        if (!problem) return true;
        report(m_Callback, PhysMessageSeverity::Error, "kernel %s: %s%u] (%s): %s", kernel.name,
               isUAV(binding.kind) ? "readWrite[" : "readOnly[", unsigned(binding.slot),
               desc.debugName.empty() ? "<unnamed>" : desc.debugName.c_str(), problem);
        return false;
    }

    nvrhi::FRESULT dispatchImpl(nvrhi::ICommandList* commandList, const KernelDispatchDesc& desc) {
        if (uint32_t(desc.kernel) >= c_KernelCount) return invalidKernel("dispatch", desc.kernel);
        const KernelDesc& kernel = c_KernelTable[uint32_t(desc.kernel)];

        const void* const constants[c_KernelConstantBufferSlots] = {desc.rootConstants, desc.secondConstants};
        const uint32_t constantsSize[c_KernelConstantBufferSlots] = {desc.rootConstantsSize, desc.secondConstantsSize};
        for (uint32_t i = 0; i < c_KernelConstantBufferSlots; ++i) {
            if (constantsSize[i] > c_KernelMaxConstantBufferSize || (constantsSize[i] != 0 && !constants[i])) {
                report(m_Callback, PhysMessageSeverity::Error,
                       "kernel %s: constant block b%u: %u bytes at %s (at most %u bytes, and a pointer for a size)",
                       kernel.name, i, constantsSize[i], constants[i] ? "a pointer" : "nullptr",
                       c_KernelMaxConstantBufferSize);
                return nvrhi::FE_INVALID_ARGS;
            }
        }

        nvrhi::IComputePipeline* pipeline = nullptr;
        nvrhi::FRESULT result = findPipeline(desc.kernel, desc.variant, &pipeline);
        if (NVRHI_FAILED(result)) return result;

        if (desc.gridDim[0] == 0 || desc.gridDim[1] == 0 || desc.gridDim[2] == 0) return nvrhi::FS_OK;

        if (!commandList) {
            if (!m_Host) {
                report(m_Callback, PhysMessageSeverity::Error,
                       "kernel %s: dispatch on the command list of a device that was released", kernel.name);
                return nvrhi::FE_NOT_ALIVE_OBJECT;
            }
            commandList = m_Host->recordingList(nullptr);
            if (!commandList) {
                report(m_Callback, PhysMessageSeverity::Error, "kernel %s: no command list", kernel.name);
                return nvrhi::FE_GENERIC_ERROR;
            }
        }

        ++m_Tick;

        // What every binding of the layout is bound to, in layout order: also the key of the binding set.
        const KernelLayoutDesc& layout = c_KernelLayouts[kernel.layout];
        BindingKey key;
        key.layout = kernel.layout;
        key.count = layout.count;
        for (uint32_t i = 0; i < layout.count; ++i) {
            const KernelBindingDesc& binding = c_KernelBindings[layout.first + i];
            const void* resource = nullptr;
            switch (binding.kind) {
                case KernelBindingKind::ConstantBuffer: {
                    nvrhi::IBuffer* buffer = nullptr;
                    if (constantsSize[binding.slot] > binding.size) {
                        report(m_Callback, PhysMessageSeverity::Error,
                               "kernel %s: constant block b%u is %u bytes; the kernel declares %u", kernel.name,
                               unsigned(binding.slot), constantsSize[binding.slot], unsigned(binding.size));
                        return nvrhi::FE_INVALID_ARGS;
                    }
                    result = acquireConstants(commandList, constants[binding.slot], constantsSize[binding.slot],
                                              binding.size, &buffer);
                    if (NVRHI_FAILED(result)) return result;
                    resource = buffer;
                    break;
                }
                case KernelBindingKind::StructuredSRV:
                case KernelBindingKind::RawSRV:
                case KernelBindingKind::TypedSRV:
                case KernelBindingKind::StructuredUAV:
                case KernelBindingKind::RawUAV:
                case KernelBindingKind::TypedUAV: {
                    nvrhi::IBuffer* buffer =
                        isUAV(binding.kind) ? desc.readWrite[binding.slot] : desc.readOnly[binding.slot];
                    if (!buffer) {
                        result = dummyBuffer(commandList, binding, &buffer);
                        if (NVRHI_FAILED(result)) return result;
                    }
                    resource = buffer;
                    break;
                }
                case KernelBindingKind::TextureSRV: {
                    nvrhi::ITexture* texture = desc.readOnlyTextures[binding.slot];
                    if (!texture) {
                        result = dummyTexture(commandList, &texture);
                        if (NVRHI_FAILED(result)) return result;
                    }
                    resource = texture;
                    break;
                }
                case KernelBindingKind::Sampler: {
                    nvrhi::ISampler* sampler = nullptr;
                    result = findSampler(&sampler);
                    if (NVRHI_FAILED(result)) return result;
                    resource = sampler;
                    break;
                }
            }
            key.resources[i] = resource;
        }

        nvrhi::IBindingSet* bindingSet = nullptr;
        auto found = m_BindingSets.find(key);
        if (found != m_BindingSets.end()) {
            found->second.lastUse = m_Tick;
            bindingSet = found->second.set;
        } else {
            nvrhi::BindingSetDesc setDesc;
            setDesc.bindings.reserve(layout.count);
            for (uint32_t i = 0; i < layout.count; ++i) {
                const KernelBindingDesc& binding = c_KernelBindings[layout.first + i];
                // The key holds the pointers as const void*; these are the objects they came from.
                nvrhi::IBuffer* buffer = static_cast<nvrhi::IBuffer*>(const_cast<void*>(key.resources[i]));
                switch (binding.kind) {
                    case KernelBindingKind::ConstantBuffer:
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::ConstantBuffer(binding.slot, buffer));
                        break;
                    case KernelBindingKind::StructuredSRV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::StructuredBuffer_SRV(binding.slot, buffer));
                        break;
                    case KernelBindingKind::RawSRV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::RawBuffer_SRV(binding.slot, buffer));
                        break;
                    case KernelBindingKind::TypedSRV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(
                            nvrhi::BindingSetItem::TypedBuffer_SRV(binding.slot, buffer, binding.format));
                        break;
                    case KernelBindingKind::TextureSRV:
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::Texture_SRV(
                            binding.slot, static_cast<nvrhi::ITexture*>(const_cast<void*>(key.resources[i]))));
                        break;
                    case KernelBindingKind::StructuredUAV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::StructuredBuffer_UAV(binding.slot, buffer));
                        break;
                    case KernelBindingKind::RawUAV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::RawBuffer_UAV(binding.slot, buffer));
                        break;
                    case KernelBindingKind::TypedUAV:
                        if (!checkBuffer(kernel, binding, buffer)) return nvrhi::FE_INVALID_ARGS;
                        setDesc.bindings.push_back(
                            nvrhi::BindingSetItem::TypedBuffer_UAV(binding.slot, buffer, binding.format));
                        break;
                    case KernelBindingKind::Sampler:
                        setDesc.bindings.push_back(nvrhi::BindingSetItem::Sampler(binding.slot, m_Sampler));
                        break;
                }
            }

            nvrhi::IBindingLayout* bindingLayout = nullptr;
            result = findLayout(kernel.layout, &bindingLayout);
            if (NVRHI_FAILED(result)) return result;

            nvrhi::BindingSetHandle set;
            if (NVRHI_FAILED(m_Device->createBindingSet(setDesc, bindingLayout, &set)) || !set) {
                report(m_Callback, PhysMessageSeverity::Error,
                       "kernel %s: nvrhi refused the bindings (see the nvrhi message callback)", kernel.name);
                return nvrhi::FE_GENERIC_ERROR;
            }

            if (m_BindingSets.size() >= c_BindingSetCacheSize) trimBindingSets();
            BindingEntry& entry = m_BindingSets[key];
            entry.set = set;
            entry.lastUse = m_Tick;
            bindingSet = entry.set;
        }

        nvrhi::ComputeState state;
        state.pipeline = pipeline;
        state.bindings.push_back(bindingSet);
        commandList->setComputeState(state);
        commandList->dispatch(desc.gridDim[0], desc.gridDim[1], desc.gridDim[2]);
        return nvrhi::FS_OK;
    }

    // Drops the binding sets that were not used recently; all of them when every one was.
    void trimBindingSets() {
        for (auto it = m_BindingSets.begin(); it != m_BindingSets.end();) {
            if (m_Tick - it->second.lastUse > c_BindingSetMaxAge)
                it = m_BindingSets.erase(it);
            else
                ++it;
        }
        if (m_BindingSets.size() >= c_BindingSetCacheSize) m_BindingSets.clear();
    }

    nvrhi::DeviceHandle m_Device;
    PhysMessageCallbackHandle m_Callback;
    detail::KernelLibraryHost* m_Host;
    nvrhi::GraphicsAPI m_API;
    bool m_WaveSupported;
    std::string m_ShaderDirectory;  // <shaderPath>/<dxil|spirv|dxbc>, empty when no shaderPath was given

    nvrhi::BindingLayoutHandle m_Layouts[c_LayoutCount];
    nvrhi::ComputePipelineHandle m_Pipelines[c_KernelCount][2];  // [kernel][baseline, wave]

    std::map<uint32_t, nvrhi::BufferHandle> m_DummyBuffers;  // by view kind, element stride and UAV slot
    nvrhi::TextureHandle m_DummyTexture;
    nvrhi::SamplerHandle m_Sampler;

    ConstantPool m_SmallBlocks{c_SmallBlockBytes, {}};
    ConstantPool m_LargeBlocks{c_LargeBlockBytes, {}};

    std::unordered_map<BindingKey, BindingEntry, BindingKeyHash> m_BindingSets;
    uint64_t m_Tick = 0;  // the number of dispatches recorded
};

}  // namespace

namespace detail {

nvrhi::FRESULT createKernelLibrary(nvrhi::IDevice* device, IPhysMessageCallback* callback, const char* shaderPath,
                                   const PhysDeviceInfo& info, KernelLibraryHost* host,
                                   IPhysKernelLibrary** ppLibrary) noexcept {
    if (!ppLibrary) return nvrhi::FE_INVALID_ARGS;
    *ppLibrary = nullptr;
    try {
        *ppLibrary = MAKE_RC_OBJ(KernelLibrary, device, callback, shaderPath, info, host);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

void detachKernelLibrary(IPhysKernelLibrary* library) noexcept {
    // Only ever called by the device, with the object createKernelLibrary made.
    if (library) static_cast<KernelLibrary*>(library)->detach();
}

}  // namespace detail
}  // namespace ephys
