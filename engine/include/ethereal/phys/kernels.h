#ifndef ETHEREAL_PHYS_KERNELS_H
#define ETHEREAL_PHYS_KERNELS_H

// IPhysKernelLibrary: the compute kernels of the engine (engine/shaders), and the one way to dispatch them.
//
// It replaces FleX's NvFlexComputeShader objects (the ~110 mShader* members of NvFlexLibrary) and
// NvFlexContextDispatch. A dispatch is described by KernelDispatchDesc, which mirrors FleX's
// NvFlexDispatchParams: 32 read-only slots (t0..t31), 8 read-write slots (u0..u7), two constant buffers
// (b0, b1) and the thread-group counts. A call site of the FleX solver ports mechanically:
//
//     NvFlexDispatchParams params = {};                 KernelDispatchDesc desc;
//     params.shader = mLib->mShaderApplyDeltas;         desc.kernel = KernelId::ApplyDeltas;
//     params.readWrite[0] = ...(mSortedNewPositions);   desc.readWrite[0] = sortedNewPositions;
//     params.readOnly[0] = ...(mDeltas);                desc.readOnly[0] = deltas;
//     params.gridDim = make_dim(numBlocks, 1, 1);       desc.gridDim[0] = numBlocks;
//     params.rootConstantBuffer = mKernelParams;        desc.rootConstants = &kernelParams;
//                                                       desc.rootConstantsSize = sizeof(kernelParams);
//     NvFlexContextDispatch(mLib->mContext, &params);   kernels->dispatch(commandList, desc);
//
// What differs from FleX:
//
// - A kernel is a KernelId, not a shader object, and there is one id per logical kernel. The vendor
//   variants of FleX (*NV through NVAPI, *AMD through AGS) are gone; the kernels that had them have one
//   portable variant written with the SM 6.0 wave intrinsics, chosen automatically (KernelVariant).
// - Constant buffers are passed as bytes. The library owns the GPU buffers (see dispatch).
// - Each kernel has its own binding layout, made from the resources its source declares. One fixed layout
//   for all kernels is not possible: the same register is a structured buffer in one kernel and a raw
//   buffer, a typed buffer or a 3D texture in another (engine/shaders/gen_kernel_table.py --audit). The
//   caller does not see the layouts: it fills slots, as with NvFlexDispatchParams.
//
// The interface follows donut/nvrhi/doc/com-interface-abi.md.

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

static constexpr uint32_t c_KernelReadOnlySlots = 32;           // t0..t31
static constexpr uint32_t c_KernelReadWriteSlots = 8;           // u0..u7
static constexpr uint32_t c_KernelConstantBufferSlots = 2;      // b0, b1
static constexpr uint32_t c_KernelMaxConstantBufferSize = 512;  // bytes: the size of the largest block (b0)

// One entry per logical kernel: the HLSL entry point, or the entry point plus "XPBD" for the XPBD
// permutation of a kernel (it binds other resources than its PBD twin, so it is a kernel of its own).
// The order is the order of the manifest, engine/shaders/ethereal_phys.cfg (the ethereal_phys_kernel_table
// test compares the two). ProbeKernelParams is a diagnostic kernel (engine/shaders/ProbeKernelParams.hlsl).
enum class KernelId : uint16_t {
    ClearInt = 0,
    ClearFloat4 = 1,
    ComputeTotalBounds = 2,
    ComputeTotalBoundsGroup = 3,
    ComputeTotalBoundsFinalize = 4,
    CalculateMortonCodes = 5,
    CalculateKeyDeltas = 6,
    BuildLeaves = 7,
    BuildHierarchy = 8,
    ComputeTotalInvEdges = 9,
    RadixSort1CS = 10,
    RadixSort2CS = 11,
    RadixSort3CS = 12,
    RadixSortBlockCS = 13,
    CalculateBounds = 14,
    CalculateBoundsGroup = 15,
    CalculateBoundsFinalize = 16,
    CalculateInflatableVolume = 17,
    CalculateInflatableVolumeXPBD = 18,
    SolveInflatableVolume = 19,
    SolveSprings = 20,
    SolveSpringsXPBD = 21,
    UpdateTriangles = 22,
    UpdateVertexNormals = 23,
    UpdateTrianglesInit = 24,
    UpdateVertexNormalsInit = 25,
    NormalizeVertexNormals = 26,
    CalculateVorticity = 27,
    CalculateParticleHash = 28,
    Predict = 29,
    CreateGrid = 30,
    ClearCellBuckets = 31,
    ReorderParticles = 32,
    CompactDiffuseParticles = 33,
    ComputeTriangleBounds = 34,
    ContinuousShockPropagation = 35,
    CollideParticles = 36,
    CollideShapes = 37,
    CollideTriangles = 38,
    ApplyDeltas = 39,
    CalculateAnisotropy = 40,
    CalculateDensity = 41,
    CalculateDensitySurfaceTension = 42,
    ClampDiffuseParticleCount = 43,
    CreateDiffuseParticles = 44,
    Finalize = 45,
    SmoothPositions = 46,
    SolveContactsAccumulate = 47,
    SolveContactsAveraged = 48,
    SolveContactsSequential = 49,
    SolveDensities = 50,
    SolveDensitiesNonFluid = 51,
    SolveDensitiesSurfaceTension = 52,
    SolveShapes = 53,
    SolveShapesXPBD = 54,
    SolveShapesPlasticDeformation = 55,
    SolveShapesPlasticDeformationXPBD = 56,
    SolveVelocities = 57,
    SpringsGenerateIndices = 58,
    SpringsParticleRange = 59,
    SpringsReorder = 60,
    TransformShapeBounds = 61,
    UpdateDiffuseParticles = 62,
    UpdateVelocities = 63,
    ProbeKernelParams = 64,
    ProbeKernelParamsXPBD = 65,
    Count = 66,
};

// Which permutation of a kernel a dispatch uses. Only the kernels with KernelInfo::hasWaveVariant have
// two; for every other kernel Default and Baseline mean the same thing.
enum class KernelVariant : uint8_t {
    Default = 0,   // Wave when the device supports the wave intrinsics (PhysDeviceInfo::supportsWaveOps) and
                   // the kernel has such a variant, Baseline otherwise
    Baseline = 1,  // the groupshared-memory reduction: runs everywhere, and is the arithmetic FleX shipped
    Wave = 2,      // the wave-intrinsics reduction; FE_UNSUPPORTED when the device or the kernel has none.
                   // Min/max reductions give the same result as Baseline; float sums are added in another
                   // order and differ in the last bits (reproducibly on one device)
};

// One kernel dispatch: FleX's NvFlexDispatchParams.
//
// Slots. readOnly[i] is register t<i>, readWrite[i] is u<i>. A kernel uses the slots its source declares
// (KernelInfo says which); what is put into any other slot is ignored. A declared slot that is left null
// is bound to a small zero-filled dummy resource of the right kind, as FleX left unused slots null on
// D3D11: a kernel must not be dispatched with parameters that make it read or write such a slot beyond its
// first 48 bytes.
//
// Buffers. A slot the kernel declares as StructuredBuffer<T> needs a buffer whose struct stride is the
// size of T, a ByteAddressBuffer slot one with raw views, a Buffer<uint> / Buffer<uint4> slot one with
// typed views (the view is created as R32_UINT / RGBA32_UINT, whatever the format of the buffer), and
// every read-write slot one that can have UAVs; anything else is FE_INVALID_ARGS. A buffer must not be in
// a read-only and in a read-write slot of the same dispatch, nor in two read-write slots (D3D11 does not
// allow it). The buffers need a tracked state (buffer.h).
//
// Textures. The slots a kernel declares as textures (KernelInfo::readOnlyTextureMask; today the sixteen SDF
// volumes of CollideShapes, t15..t30) are read from readOnlyTextures[i], not from readOnly[i].
//
// Constant buffers. rootConstants is b0 and secondConstants is b1: pointers to rootConstantsSize /
// secondConstantsSize bytes, copied during the call. A size is at most the size of the cbuffer the kernel
// declares there (so at most c_KernelMaxConstantBufferSize; more is FE_INVALID_ARGS: it is the block of
// another kernel). A null pointer (with size 0) binds zeros, and the bytes of the cbuffer beyond the size
// given are zero. A block for a slot the kernel does not declare is ignored.
struct KernelDispatchDesc {
    KernelId kernel = KernelId::Count;
    KernelVariant variant = KernelVariant::Default;
    uint8_t _pad = 0;                 // explicit
    uint32_t gridDim[3] = {1, 1, 1};  // thread groups in x, y, z (FleX's gridDim); a zero records nothing

    const void* rootConstants = nullptr;    // b0
    const void* secondConstants = nullptr;  // b1
    uint32_t rootConstantsSize = 0;
    uint32_t secondConstantsSize = 0;

    nvrhi::IBuffer* readOnly[c_KernelReadOnlySlots] = {};           // t0..t31
    nvrhi::IBuffer* readWrite[c_KernelReadWriteSlots] = {};         // u0..u7
    nvrhi::ITexture* readOnlyTextures[c_KernelReadOnlySlots] = {};  // t0..t31, for the texture slots
};

static_assert(sizeof(KernelId) == 2 && sizeof(KernelVariant) == 1);
static_assert(sizeof(KernelDispatchDesc) == 616, "KernelDispatchDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(KernelDispatchDesc, variant) == 2);
static_assert(offsetof(KernelDispatchDesc, _pad) == 3);
static_assert(offsetof(KernelDispatchDesc, gridDim) == 4);         // after the padding
static_assert(offsetof(KernelDispatchDesc, rootConstants) == 16);  // after gridDim[3]
static_assert(offsetof(KernelDispatchDesc, secondConstants) == 24);
static_assert(offsetof(KernelDispatchDesc, rootConstantsSize) == 32);
static_assert(offsetof(KernelDispatchDesc, secondConstantsSize) == 36);
static_assert(offsetof(KernelDispatchDesc, readOnly) == 40);
static_assert(offsetof(KernelDispatchDesc, readWrite) == 296);         // after readOnly[32]
static_assert(offsetof(KernelDispatchDesc, readOnlyTextures) == 360);  // after readWrite[8]

// What a kernel is and what it binds.
struct KernelInfo {
    const char* name = nullptr;               // a static string: the name of the kernel (getKernelName)
    uint32_t threadGroupSize[3] = {0, 0, 0};  // numthreads of the kernel
    uint32_t readOnlyMask = 0;                // bit i: the kernel declares t<i>
    uint32_t readOnlyTextureMask = 0;         // bit i: t<i> is a texture (a subset of readOnlyMask)
    uint8_t readWriteMask = 0;                // bit i: the kernel declares u<i>
    uint8_t constantBufferMask = 0;           // bit i: the kernel declares b<i>
    bool hasWaveVariant = false;              // the kernel has a wave-intrinsics permutation
    bool usesWaveVariant = false;             // KernelVariant::Default selects it on this device
    bool isXPBD = false;                      // an XPBD kernel: b1 is the compliance block (KernelParamsXPBD)
    uint8_t _pad[7] = {0, 0, 0, 0, 0, 0, 0};  // explicit
};

static_assert(sizeof(KernelInfo) == 40, "KernelInfo layout changed: bump c_HeaderVersion");
static_assert(offsetof(KernelInfo, threadGroupSize) == 8);
static_assert(offsetof(KernelInfo, readOnlyMask) == 20);  // after threadGroupSize[3]
static_assert(offsetof(KernelInfo, readOnlyTextureMask) == 24);
static_assert(offsetof(KernelInfo, readWriteMask) == 28);
static_assert(offsetof(KernelInfo, constantBufferMask) == 29);
static_assert(offsetof(KernelInfo, hasWaveVariant) == 30);
static_assert(offsetof(KernelInfo, usesWaveVariant) == 31);
static_assert(offsetof(KernelInfo, isXPBD) == 32);
static_assert(offsetof(KernelInfo, _pad) == 33);

// The kernel library of a device (IPhysDevice::getKernels). Like the device it is not thread-safe.
//
// Kernels are loaded on first use: the compiled blob is read from
// <PhysDeviceDesc::shaderPath>/<dxil|spirv|dxbc>/ (the path is UTF-8), and the shader, the binding layout
// and the compute pipeline are created. A blob that is missing, truncated or not a shader is an error:
// it is reported through the message callback of the device with the path of the file, and the call
// returns an FE_* code. A kernel that failed is tried again on its next use. loadAll() loads everything
// up front.
NVRHI_IID(IPhysKernelLibrary, "769101f1-3953-4493-b993-9019f5684d39")
struct IPhysKernelLibrary : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysKernelLibrary)

    // The number of kernels: uint32_t(KernelId::Count) of the library.
    [[nodiscard]] virtual uint32_t getKernelCount() const noexcept = 0;

    // The name of a kernel, a static string ("ApplyDeltas", "SolveShapesXPBD"); nullptr for an id that is
    // not a kernel.
    [[nodiscard]] virtual const char* getKernelName(KernelId kernel) const noexcept = 0;

    // Fills retVal and returns it. For an id that is not a kernel, retVal is a default KernelInfo (name is
    // nullptr). Loads nothing.
    virtual KernelInfo& getKernelInfo(KernelInfo& retVal, KernelId kernel) const noexcept = 0;

    // The compute pipeline of a kernel variant and the binding layout of a kernel, for a caller that
    // records a dispatch itself (an indirect dispatch, say). They load the kernel if needed. On success
    // FS_OK and the object with one reference added. FE_INVALID_ARGS for an id that is not a kernel,
    // FE_UNSUPPORTED for KernelVariant::Wave where the device or the kernel has none, FE_NOT_FOUND when
    // the blob or the permutation is missing, FE_GENERIC_ERROR when the blob is not a usable shader.
    virtual nvrhi::FRESULT getPipeline(KernelId kernel, KernelVariant variant,
                                       nvrhi::IComputePipeline** ppPipeline) noexcept = 0;
    virtual nvrhi::FRESULT getBindingLayout(KernelId kernel, nvrhi::IBindingLayout** ppLayout) noexcept = 0;

    // Loads every kernel, in every variant the device can run. Returns FS_OK, or the code of the first
    // failure after trying all of them (each failure is reported through the message callback).
    virtual nvrhi::FRESULT loadAll() noexcept = 0;

    // Records one dispatch of desc.kernel: uploads the constant blocks that changed, binds the slots and
    // dispatches desc.gridDim thread groups.
    //
    // commandList is the open command list of the application, or nullptr for the device's own (IPhysDevice,
    // "The device owns one command list"), which IPhysDevice::flush executes.
    //
    // Command lists that kernels were dispatched on must be executed in the order the dispatches were
    // recorded. The simulation buffers need that anyway (a dispatch reads what the one before it wrote),
    // and the library relies on it for its constant buffers: they are ordinary GPU buffers, written on the
    // command list right before the first dispatch that needs new contents, and reused without a write by
    // later dispatches with the same contents. (They are not nvrhi volatile constant buffers: FleX records
    // hundreds of dispatches per step, and a volatile buffer on Vulkan has a fixed number of versions per
    // unfinished command list.)
    //
    // Returns the codes of getPipeline, FE_INVALID_ARGS for a constant block that is too large or has a
    // size without a pointer and for a buffer of the wrong kind or element size in a slot,
    // FE_NOT_ALIVE_OBJECT for a null commandList after the device was released, and FE_GENERIC_ERROR when
    // nvrhi refuses the bindings (the nvrhi message callback says why). A dispatch that fails records no
    // dispatch, but may already have written a constant block to the command list.
    virtual nvrhi::FRESULT dispatch(_In_opt_ nvrhi::ICommandList* commandList,
                                    const KernelDispatchDesc& desc) noexcept = 0;

    // The library caches the binding sets of recent dispatches, and those hold references to the buffers
    // and textures that were bound. This drops them: call it after releasing simulation buffers that
    // should be freed now rather than when the cache evicts them. Constant buffers and pipelines stay.
    virtual void clearBindingCache() noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysKernelLibrary> PhysKernelLibraryHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_KERNELS_H */
