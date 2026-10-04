#include "Library.h"
#include <nvflex/NvFlexContext.h>
#include <nvflex/NvFlexContextExt.h>
#include <d3d11.h>
#include <d3d12.h>
#include "Device.h"
#include "BVHBuilder.h"
#include "Data.h"
#include <cstdarg>
#include <cmrc/cmrc.hpp>

CMRC_DECLARE(nvflex);

#include "CalculateBoundsNV.hlsl.h"
#include "CalculateBoundsGroupNV.hlsl.h"
#include "CalculateBoundsFinalizeNV.hlsl.h"
#include "CalculateInflatableVolumeNV.hlsl.h"
#include "CalculateBoundsAMD.hlsl.h"
#include "CalculateBoundsGroupAMD.hlsl.h"
#include "CalculateBoundsFinalizeAMD.hlsl.h"
#include "CalculateInflatableVolumeAMD.hlsl.h"
#include "CalculateBounds.hlsl.h"
#include "CalculateBoundsGroup.hlsl.h"
#include "CalculateBoundsFinalize.hlsl.h"
#include "CalculateInflatableVolume.hlsl.h"
#include "SolveInflatableVolumeNV.hlsl.h"
#include "SolveSpringsNV.hlsl.h"
#include "UpdateTrianglesNV.hlsl.h"
#include "UpdateVertexNormalsNV.hlsl.h"
#include "SolveInflatableVolume.hlsl.h"
#include "SolveSprings.hlsl.h"
#include "UpdateTriangles.hlsl.h"
#include "UpdateVertexNormals.hlsl.h"
#include "UpdateTrianglesInit.hlsl.h"
#include "UpdateVertexNormalsInit.hlsl.h"
#include "NormalizeVertexNormals.hlsl.h"
#include "CalculateVorticity.hlsl.h"
#include "Predict.hlsl.h"
#include "CalculateParticleHash.hlsl.h"
#include "CreateGrid.hlsl.h"
#include "ClearCellBuckets.hlsl.h"
#include "ClearInt.hlsl.h"
#include "ClearFloat4.hlsl.h"
#include "ReorderParticles.hlsl.h"
#include "CollideParticles.hlsl.h"
#include "CollideShapes.hlsl.h"
#include "ContinuousShockPropagation.hlsl.h"
#include "CalculateDensity.hlsl.h"
#include "CalculateDensitySurfaceTension.hlsl.h"
#include "SolveDensities.hlsl.h"
#include "SolveDensitiesNonFluid.hlsl.h"
#include "SolveDensitiesSurfaceTension.hlsl.h"
#include "SolveShapesNV.hlsl.h"
#include "SolveShapes32NV.hlsl.h"
#include "SolveShapes128NV.hlsl.h"
#include "SolveShapesPlasticDeformationNV.hlsl.h"
#include "SolveShapesPlasticDeformation32NV.hlsl.h"
#include "SolveShapesPlasticDeformation128NV.hlsl.h"
#include "SolveShapes.hlsl.h"
#include "SolveShapesPlasticDeformation.hlsl.h"
#include "ApplyDeltas.hlsl.h"
#include "SolveContactsSequential.hlsl.h"
#include "SolveContactsAveraged.hlsl.h"
#include "SolveContactsAccumulate.hlsl.h"
#include "CollideTriangles.hlsl.h"
#include "UpdateVelocities.hlsl.h"
#include "SolveVelocities.hlsl.h"
#include "Finalize.hlsl.h"
#include "CreateDiffuseParticles.hlsl.h"
#include "UpdateDiffuseParticles.hlsl.h"
#include "ClampDiffuseParticleCount.hlsl.h"
#include "CompactDiffuseParticles.hlsl.h"
#include "SmoothPositions.hlsl.h"
#include "CalculateAnisotropy.hlsl.h"
#include "ComputeTriangleBounds.hlsl.h"
#include "TransformShapeBounds.hlsl.h"
#include "SpringsGenerateIndices.hlsl.h"
#include "SpringsParticleRange.hlsl.h"
#include "SpringsReorder.hlsl.h"
// XPBD entries (ShadersXPBD.cfg), no shipped blob: always loaded from the generated headers
#include "SolveSpringsXPBD.hlsl.h"
#include "CalculateInflatableVolumeXPBD.hlsl.h"
#include "CalculateInflatableVolumeXPBDNV.hlsl.h"
#include "CalculateInflatableVolumeXPBDAMD.hlsl.h"
#include "SolveShapesXPBD.hlsl.h"
#include "SolveShapesPlasticDeformationXPBD.hlsl.h"

namespace NvFlex {

static NvFlexInitDesc sDefaultDesc = {-1, 1, 0, 0, 0, 1, eNvFlexD3D11};

struct ClearParamsInt {
    int lengthInWords;
    int value;
    int _pad1;
    int _pad2;
};

struct ClearParamsFloat4 {
    int lengthInWords;
    float3 _pad3;
    float4 value;
};

bool Library::Init(const NvFlexInitDesc* desc, NvFlexErrorCallback errorFunc) {
    if (!desc)
        desc = &sDefaultDesc;

    if (desc->computeType != eNvFlexD3D11 && desc->computeType != eNvFlexD3D12) {
        FlexLogError(errorFunc, eNvFlexLogError,
                     "Trying to initialize D3D Flex with a CUDA compute type.", __FILE__,
                     __LINE__);
        return 0;
    }

    if (desc->renderDevice == NULL || desc->renderContext == NULL) {
        FlexLogError(
            errorFunc, eNvFlexLogError,
            "External D3D device and device context or command queue must be present.",
            __FILE__, __LINE__);
        return 0;
    }

    mErrorFunc = errorFunc;

    if (desc->computeType == eNvFlexD3D11) {
        DeviceDesc devDesc = {};
        devDesc.d3d11.device = (ID3D11Device*)desc->renderDevice;
        devDesc.d3d11.renderContext = (ID3D11DeviceContext*)desc->renderContext;
        devDesc.d3d11.computeContext = (ID3D11DeviceContext*)desc->computeContext;
        devDesc.useComputeQueue = !desc->runOnRenderContext;

        mDevice = NvFlex::createDeviceD3D11(devDesc);
        NVFLEX_ASSERT(mDevice);
        mContext = mDevice->createContext();
        NVFLEX_ASSERT(mContext);
    } else {
        DeviceDesc devDesc = {};
        devDesc.d3d12.device = (ID3D12Device*)desc->renderDevice;
        devDesc.d3d12.renderCmdQueue = (ID3D12CommandQueue*)desc->renderContext;
        devDesc.d3d12.computeCmdQueue = (ID3D12CommandQueue*)desc->computeContext;
        devDesc.useComputeQueue = !desc->runOnRenderContext;

        mDevice = NvFlex::createDeviceD3D12(devDesc);
        NVFLEX_ASSERT(mDevice);
        mContext = mDevice->createContext();
        NVFLEX_ASSERT(mContext);
        if (!mDevice) {
            FLEX_LOG_ERROR(this, "Failed to create internal d3d12 device.");
            return 0;
        }
    }

    NvFlex::FlexDeviceCapabilities devCapabilities;
    mDevice->getDeviceCapabilities(&devCapabilities);
    strcpy(mDeviceName, devCapabilities.vendorName);
    if (devCapabilities.vendorID == 4318) {
        mGpuVendorId = VENDOR_ID_NVIDIA;
    } else if (devCapabilities.vendorID == 4098 || devCapabilities.vendorID == 4130) {
        mGpuVendorId = VENDOR_ID_AMD;
    } else {
        mGpuVendorId = VENDOR_ID_OTHERS;
    }

    mSMCount = devCapabilities.smCount;
    const bool enableExtensions =
        desc->enableExtensions && !(mGpuVendorId == VENDOR_ID_NVIDIA && mSMCount == -1);
    mIsSHFLSupported = enableExtensions && devCapabilities.isSHFLSupported;
    mIsFP32ATOMICSupported = enableExtensions && devCapabilities.isFp32AtomicSupported;
    mIsSwizzleSupported = enableExtensions && devCapabilities.isSwizzleSupported;

    NvFlexFenceDesc fenceDesc = {};
    mSyncQuery = NvFlexCreateFence(mContext, &fenceDesc);

#if 0
    auto createShader = [this](const BYTE* cs, uint64_t cs_length, const wchar_t* label,
                                  int nvapiSlot) {
        NvFlexComputeShaderDesc desc = {};
        desc.cs = cs;
        desc.cs_length = cs_length;
        desc.label = label;
        desc.NVAPI_Slot = nvapiSlot;
        return NvFlexCreateComputeShader(this->mContext, &desc);
    };
#define NVFLEX_CREATE_SHADER_ARGS(name) g_##name, sizeof(g_##name), L#name
#else
    auto createShader = [this](const char *name, const wchar_t *label, int nvapiSlot) {
        NvFlexComputeShaderDesc desc = {};
        auto file = cmrc::nvflex::get_filesystem().open(name);
        desc.cs = file.begin();
        desc.cs_length = file.size();
        desc.label = label;
        desc.NVAPI_Slot = nvapiSlot;
        return NvFlexCreateComputeShader(this->mContext, &desc);
    };
#define NVFLEX_CREATE_SHADER_ARGS(name) "g_Flex_"#name".txt", L###name

#endif

    // XPBD shaders have no shipped blob and are created from the generated headers
    auto createShaderFromHeader = [this](const BYTE *cs, uint64_t len, const wchar_t *label,
                                         int nvapiSlot) {
        NvFlexComputeShaderDesc desc = {};
        desc.cs = cs;
        desc.cs_length = len;
        desc.label = label;
        desc.NVAPI_Slot = nvapiSlot;
        return NvFlexCreateComputeShader(this->mContext, &desc);
    };
#define NVFLEX_CREATE_SHADER_HEADER_ARGS(name) g_##name, sizeof(g_##name), L#name

    if (mIsSHFLSupported) {
        mShaderCalculateBounds =
            createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsNV), 7u);
        mShaderCalculateBoundsGroup =
            createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsGroupNV), 7u);
        mShaderCalculateBoundsFinalize =
            createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsFinalizeNV), 7u);
        mShaderCalculateInflatableVolume =
            createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateInflatableVolumeNV), 7u);
    } else {
        if (mIsSwizzleSupported) {
            mShaderCalculateBounds =
                createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsAMD), 0xFFFFFFFF);
            mShaderCalculateBoundsGroup = createShader(
                NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsGroupAMD), 0xFFFFFFFF);
            mShaderCalculateBoundsFinalize = createShader(
                NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsFinalizeAMD), 0xFFFFFFFF);
            mShaderCalculateInflatableVolume = createShader(
                NVFLEX_CREATE_SHADER_ARGS(CalculateInflatableVolumeAMD), 0xFFFFFFFF);
        } else {
            mShaderCalculateBounds =
                createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBounds), 0xFFFFFFFF);
            mShaderCalculateBoundsGroup =
                createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsGroup), 0xFFFFFFFF);
            mShaderCalculateBoundsFinalize = createShader(
                NVFLEX_CREATE_SHADER_ARGS(CalculateBoundsFinalize), 0xFFFFFFFF);
            mShaderCalculateInflatableVolume = createShader(
                NVFLEX_CREATE_SHADER_ARGS(CalculateInflatableVolume), 0xFFFFFFFF);
        }
    }
    if (mIsFP32ATOMICSupported) {
        mShaderSolveInflatableVolume =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveInflatableVolumeNV), 7u);
        mShaderSolveSprings = createShader(NVFLEX_CREATE_SHADER_ARGS(SolveSpringsNV), 7u);
        mShaderUpdateTriangles =
            createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateTrianglesNV), 7u);
        mShaderUpdateVertexNormals =
            createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateVertexNormalsNV), 7u);
    } else {
        mShaderSolveInflatableVolume =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveInflatableVolume), 0xFFFFFFFF);
        mShaderSolveSprings =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveSprings), 0xFFFFFFFF);
        mShaderUpdateTriangles =
            createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateTriangles), 0xFFFFFFFF);
        mShaderUpdateVertexNormals =
            createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateVertexNormals), 0xFFFFFFFF);
    }
    mShaderUpdateTrianglesInit =
        createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateTrianglesInit), 0xFFFFFFFF);
    mShaderUpdateVertexNormalsInit =
        createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateVertexNormalsInit), 0xFFFFFFFF);
    mShaderNormalizeVertexNormals =
        createShader(NVFLEX_CREATE_SHADER_ARGS(NormalizeVertexNormals), 0xFFFFFFFF);
    mShaderCalculateVorticity =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateVorticity), 0xFFFFFFFF);
    mShaderPredict = createShader(NVFLEX_CREATE_SHADER_ARGS(Predict), 0xFFFFFFFF);
    mShaderCalculateParticleHash =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateParticleHash), 0xFFFFFFFF);
    mShaderCreateGrid = createShader(NVFLEX_CREATE_SHADER_ARGS(CreateGrid), 0xFFFFFFFF);
    mShaderClearCellBuckets =
        createShader(NVFLEX_CREATE_SHADER_ARGS(ClearCellBuckets), 0xFFFFFFFF);
    mShaderClearInt = createShader(NVFLEX_CREATE_SHADER_ARGS(ClearInt), 0xFFFFFFFF);
    NVFLEX_VERIFY(
        mShaderClearConstantBufferInt.Create(mContext, sizeof(ClearParamsInt), 1));
    mShaderClearFloat4 = createShader(NVFLEX_CREATE_SHADER_ARGS(ClearFloat4), 0xFFFFFFFF);
    NVFLEX_VERIFY(
        mShaderClearConstantBufferFloat4.Create(mContext, sizeof(ClearParamsFloat4), 1));
    mShaderReorderParticles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(ReorderParticles), 0xFFFFFFFF);
    mShaderCollideParticles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CollideParticles), 0xFFFFFFFF);
    mShaderCollideShapes =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CollideShapes), 0xFFFFFFFF);
    mShaderContinuousShockPropagation =
        createShader(NVFLEX_CREATE_SHADER_ARGS(ContinuousShockPropagation), 0xFFFFFFFF);
    mShaderCalculateDensity =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateDensity), 0xFFFFFFFF);
    mShaderCalculateDensitySurfaceTension =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateDensitySurfaceTension), 0xFFFFFFFF);
    mShaderSolveDensities =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveDensities), 0xFFFFFFFF);
    mShaderSolveDensitiesNonFluid =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveDensitiesNonFluid), 0xFFFFFFFF);
    mShaderSolveDensitiesSurfaceTension =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveDensitiesSurfaceTension), 0xFFFFFFFF);
    if (mIsSHFLSupported && mIsFP32ATOMICSupported) {
        mShaderSolveShapes = createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapesNV), 7u);
        mShaderSolveShapes32 = createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapes32NV), 7u);
        mShaderSolveShapes128 =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapes128NV), 7u);
        mShaderSolveShapesPlasticDeformation =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapesPlasticDeformationNV), 7u);
        mShaderSolveShapesPlasticDeformation32 =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapesPlasticDeformation32NV), 7u);
        mShaderSolveShapesPlasticDeformation128 =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapesPlasticDeformation128NV), 7u);
    } else {
        mShaderSolveShapes =
            createShader(NVFLEX_CREATE_SHADER_ARGS(SolveShapes), 0xFFFFFFFF);
        mShaderSolveShapesPlasticDeformation = createShader(
            NVFLEX_CREATE_SHADER_ARGS(SolveShapesPlasticDeformation), 0xFFFFFFFF);
    }
    mShaderApplyDeltas = createShader(NVFLEX_CREATE_SHADER_ARGS(ApplyDeltas), 0xFFFFFFFF);
    mShaderSolveContactsSequential =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveContactsSequential), 0xFFFFFFFF);
    mShaderSolveContactsAveraged =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveContactsAveraged), 0xFFFFFFFF);
    mShaderSolveContactsAccumulate =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveContactsAccumulate), 0xFFFFFFFF);
    mShaderCollideTriangles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CollideTriangles), 0xFFFFFFFF);
    mShaderUpdateVelocities =
        createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateVelocities), 0xFFFFFFFF);
    mShaderSolveVelocities =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SolveVelocities), 0xFFFFFFFF);
    mShaderFinalize = createShader(NVFLEX_CREATE_SHADER_ARGS(Finalize), 0xFFFFFFFF);
    mShaderCreateDiffuseParticles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CreateDiffuseParticles), 0xFFFFFFFF);
    mShaderUpdateDiffuseParticles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(UpdateDiffuseParticles), 0xFFFFFFFF);
    mShaderClampDiffuseParticleCount =
        createShader(NVFLEX_CREATE_SHADER_ARGS(ClampDiffuseParticleCount), 0xFFFFFFFF);
    mShaderCompactDiffuseParticles =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CompactDiffuseParticles), 0xFFFFFFFF);
    mShaderSmoothPositions =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SmoothPositions), 0xFFFFFFFF);
    mShaderCalculateAnisotropy =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateAnisotropy), 0xFFFFFFFF);
    mShaderComputeTriangleBounds =
        createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTriangleBounds), 0xFFFFFFFF);
    mShaderTransformShapeBounds =
        createShader(NVFLEX_CREATE_SHADER_ARGS(TransformShapeBounds), 0xFFFFFFFF);
    mShaderSpringsGenerateIndices =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SpringsGenerateIndices), 0xFFFFFFFF);
    mShaderSpringsParticleRange =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SpringsParticleRange), 0xFFFFFFFF);
    mShaderSpringsReorder =
        createShader(NVFLEX_CREATE_SHADER_ARGS(SpringsReorder), 0xFFFFFFFF);

    mShaderSolveSpringsXPBD = createShaderFromHeader(
        NVFLEX_CREATE_SHADER_HEADER_ARGS(SolveSpringsXPBD), 0xFFFFFFFF);
    if (mIsSHFLSupported) {
        mShaderCalculateInflatableVolumeXPBD = createShaderFromHeader(
            NVFLEX_CREATE_SHADER_HEADER_ARGS(CalculateInflatableVolumeXPBDNV), 7u);
    } else if (mIsSwizzleSupported) {
        mShaderCalculateInflatableVolumeXPBD = createShaderFromHeader(
            NVFLEX_CREATE_SHADER_HEADER_ARGS(CalculateInflatableVolumeXPBDAMD), 0xFFFFFFFF);
    } else {
        mShaderCalculateInflatableVolumeXPBD = createShaderFromHeader(
            NVFLEX_CREATE_SHADER_HEADER_ARGS(CalculateInflatableVolumeXPBD), 0xFFFFFFFF);
    }
    mShaderSolveShapesXPBD = createShaderFromHeader(
        NVFLEX_CREATE_SHADER_HEADER_ARGS(SolveShapesXPBD), 0xFFFFFFFF);
    mShaderSolveShapesPlasticDeformationXPBD = createShaderFromHeader(
        NVFLEX_CREATE_SHADER_HEADER_ARGS(SolveShapesPlasticDeformationXPBD), 0xFFFFFFFF);

    mDevice->prepareContext(mContext, true);

    int hashSize = 0x200000;
    NVFLEX_VERIFY(mCellBucketStarts.CreateWithZero(mContext, hashSize,
                                                   "NvFlexLibrary::mCellBucketStarts"));
    NVFLEX_VERIFY(mCellBucketEnds.CreateWithZero(mContext, hashSize,
                                                 "NvFlexLibrary::mCellBucketEnds"));

    mBVHBuilder = new LinearBVHBuilderGPU(this, mContext, mIsSHFLSupported, mSMCount,
                                          mIsSwizzleSupported);

    mMaxStaticTris = 0;

    mTriangleMeshData = new TriangleMeshData;

    mConvexMeshData = new ConvexMeshData(mContext, 4096, 64);

    mSDFData = new SDFData(mContext, 16);

    mDevice->executeContext(mContext);

    return true;
}

NvResourceTracker* Library::GetResourceTracker() {
    return &mResourceTracker;
}

NvFlexContext* Library::GetContext() {
    return mContext;
}

Device* Library::GetDevice() {
    return mDevice;
}

void Library::ClearBufferInt(NvFlexBuffer* buffer, NvFlexUint sizeInBytes, NvFlexInt val) {
    if (buffer) {
        auto ptr = (ClearParamsInt*)mShaderClearConstantBufferInt.Map(mContext);
        ptr->lengthInWords = sizeInBytes / sizeof(int);
        ptr->value = val;
        mShaderClearConstantBufferInt.Unmap(mContext);

        NvFlexDispatchParams params = {};
        params.shader = mShaderClearInt;
        params.rootConstantBuffer = mShaderClearConstantBufferInt;
        params.readWrite[0] = NvFlexBufferGetResourceRW(buffer);
        params.gridDim = make_dim(divCeil<256>(sizeInBytes / sizeof(int)), 1, 1);
        NvFlexContextDispatch(mContext, &params);
    }
}

void Library::ClearBufferFloat4(NvFlexBuffer* buffer, NvFlexUint sizeInBytes,
                                NvFlexFloat4 val) {
    if (buffer) {
        auto ptr = (ClearParamsFloat4*)mShaderClearConstantBufferFloat4.Map(mContext);
        ptr->lengthInWords = sizeInBytes / sizeof(float4);
        ptr->value = val;
        mShaderClearConstantBufferFloat4.Unmap(mContext);

        NvFlexDispatchParams params = {};
        params.shader = mShaderClearFloat4;
        params.rootConstantBuffer = mShaderClearConstantBufferFloat4;
        params.readWrite[0] = NvFlexBufferGetResourceRW(buffer);
        params.gridDim = make_dim(divCeil<256>(sizeInBytes / sizeof(float4)), 1, 1);
        NvFlexContextDispatch(mContext, &params);
    }
}

NvFlexErrorCallback Library::GetLogger() {
    return mErrorFunc;
}

NvFlexTriangleMeshId Library::CreateTriangleMesh() {
    auto newId = mTriangleMeshData->Allocate() + 1;
    mResourceTracker.add((void*)(uintptr_t)newId, NvResourceTracker::eTriangleMesh);
    return newId;
}

NvFlexConvexMeshId Library::CreateConvexMesh() {
    auto newId = mConvexMeshData->Allocate() + 1;
    mResourceTracker.add((void*)(uintptr_t)newId, NvResourceTracker::eConvexMesh);
    return newId;
}

NvFlexDistanceFieldId Library::CreateDistanceField() {
    auto newId = mSDFData->Allocate() + 1;
    mResourceTracker.add((void*)(uintptr_t)newId, NvResourceTracker::eSDF);
    return newId;
}

void Library::ReleaseTriangleMesh(NvFlexTriangleMeshId id) {
    if (id > 0) {
        mTriangleMeshData->Free(id - 1);
        mResourceTracker.remove((void*)(uintptr_t)id, NvResourceTracker::eTriangleMesh);
    } else {
        FLEX_LOG_ERROR(this, "Trying to destroy an invalid triangle mesh");
    }
}

void Library::ReleaseConvexMesh(NvFlexConvexMeshId id) {
    if (id > 0) {
        mConvexMeshData->Free(id - 1);
        mResourceTracker.remove((void*)(uintptr_t)id, NvResourceTracker::eConvexMesh);
    } else {
        FLEX_LOG_ERROR(this, "Trying to destroy an invalid convex mesh");
    }
}

void Library::ReleaseDistanceField(NvFlexDistanceFieldId id) {
    if (id > 0) {
        mSDFData->Free(id - 1);
        mResourceTracker.remove((void*)(uintptr_t)id, NvResourceTracker::eSDF);
    } else
        FLEX_LOG_ERROR(this, "Trying to destroy an invalid distance field");
}

NvFlexInt Library::GetTriangleMeshes(NvFlexTriangleMeshId* meshes, NvFlexUint n) {
    return mResourceTracker.get(meshes, n, NvResourceTracker::eTriangleMesh);
}

void Library::UpdateTriangleMesh(NvFlexTriangleMeshId meshId, NvFlexBuffer* vertices,
                                 NvFlexBuffer* indices, int numVertices, int numTris,
                                 const NvFlexFloat3* lower, const NvFlexFloat3* upper) {
    auto context = mContext;
    if (meshId) {
        auto mesh = mTriangleMeshData->mTriMeshTableHost[meshId - 1];
        if (mesh) {
            if (lower && upper) {
                if (numTris && numVertices) {
                    if (numTris > mesh->mNumTris) {
                        NvFlexUint numTrisToAlloc = CalculateSlack(numTris);
                        mesh->mIndices.Create(context, 3 * numTrisToAlloc,
                                              "NvFlexSolver::TriangleMesh::Indices");
                        mesh->mMaxTris = numTrisToAlloc;
                    }

                    if (numVertices > mesh->mMaxVertices) {
                        NvFlexUint numVertsToAlloc = CalculateSlack(numVertices);
                        mesh->mVertices.Create(context, numVertsToAlloc,
                                               "NvFlexSolver::TriangleMesh::Vertices");
                        mesh->mMaxVertices = numVertsToAlloc;
                    }

                    mesh->mNumTris = numTris;
                    mesh->mNumVertices = numVertices;
                    mesh->mNumIndices = 3 * numTris;

                    NvFlexContextCopyBuffer(context, mesh->mVertices, 0, vertices, 0,
                                            numVertices * sizeof(float4));
                    NvFlexContextCopyBuffer(context, mesh->mIndices, 0, indices, 0,
                                            mesh->mNumIndices * sizeof(int));

                    mesh->mLower = *lower;
                    mesh->mUpper = *upper;

                    if (!mConstTriMeshInfo)
                        NVFLEX_VERIFY(mConstTriMeshInfo.Create(mContext, 1, 1));

                    auto vptr = mConstTriMeshInfo.Map(context);
                    *(int*)vptr = mesh->mNumTris;
                    mConstTriMeshInfo.Unmap(context);

                    if (numTris > mMaxStaticTris) {
                        NvFlexUint numToAlloc = CalculateSlack(numTris);
                        NVFLEX_VERIFY(mStaticTriLowers.Create(
                            mContext, numToAlloc, "NvFlexLibrary::mStaticTriLower"));
                        NVFLEX_VERIFY(mStaticTriUppers.Create(
                            mContext, numToAlloc, "NvFlexLibrary::mStaticUppers"));
                        mMaxStaticTris = numToAlloc;
                    }

                    const NvFlexUint kNumBlocks = divCeil<256>(mesh->mNumTris);
                    NvFlexDispatchParams params = {};
                    params.shader = mShaderComputeTriangleBounds;
                    params.readWrite[0] = NvFlexBufferGetResourceRW(mStaticTriLowers);
                    params.readWrite[1] = NvFlexBufferGetResourceRW(mStaticTriUppers);
                    params.readOnly[0] = NvFlexBufferGetResource(mesh->mVertices);
                    params.readOnly[1] = NvFlexBufferGetResource(mesh->mIndices);
                    params.gridDim = make_dim(kNumBlocks, 1, 1);
                    params.rootConstantBuffer = mConstTriMeshInfo;
                    NvFlexContextDispatch(context, &params);

                    NvFlexBounds3f totalBounds{*lower, *upper};

                    mBVHBuilder->Build(&mesh->mBVH, mStaticTriLowers, mStaticTriUppers,
                                       nullptr, numTris, &totalBounds, nullptr);

                    mTriangleMeshData->mRebuildTriangleData = 1;
                } else {
                    mesh->mNumTris = 0;
                    mesh->mNumVertices = 0;
                }
            } else
                FLEX_LOG_ERROR(this, "Spatial bounds of mesh cannot be null");
        } else
            FLEX_LOG_ERROR(this, "Trying to update an invalid triangle mesh");
    } else
        FLEX_LOG_ERROR(this, "Trying to update an invalid triangle mesh");
}

void Library::GetTriangleMeshBounds(NvFlexTriangleMeshId meshId, NvFlexFloat3* lower,
                                    NvFlexFloat3* upper) {
    if (meshId) {
        auto meshHost = mTriangleMeshData->mTriMeshTableHost[meshId - 1];
        if (meshHost) {
            *lower = meshHost->mLower;
            *upper = meshHost->mUpper;
        } else
            FLEX_LOG_ERROR(this, "Trying to query bounds of an invalid triangle mesh");
    } else
        FLEX_LOG_ERROR(this, "Trying to query bounds of an invalid triangle mesh");
}

NvFlexUint Library::GetDistanceFields(NvFlexDistanceFieldId* sdfIds, NvFlexUint n) {
    return mResourceTracker.get((void*)sdfIds, n, NvResourceTracker::eSDF);
}

void Library::UpdateDistanceField(NvFlexDistanceFieldId id, NvFlexTexture3D* sdf) {
    if (id) {
        mSDFData->Update(mContext, id - 1, sdf);
    } else
        FLEX_LOG_ERROR(this, "Trying to update an invalid SDF");
}

NvFlexUint Library::GetConvexMeshes(NvFlexConvexMeshId* ids, NvFlexUint n) {
    return mResourceTracker.get((void*)ids, n, NvResourceTracker::eConvexMesh);
}

void Library::UpdateConvexMesh(NvFlexConvexMeshId id, NvFlexBuffer* planes, int numPlanes,
                               const NvFlexFloat3* lower, const NvFlexFloat3* upper) {
    if (id) {
        if (numPlanes <= 64) {
            return mConvexMeshData->Update(mContext, id - 1, planes, numPlanes, *lower,
                                           *upper);
        } else
            FLEX_LOG_ERROR(this,
                           "Trying to update a convex mesh with too many planes %d > %d",
                           numPlanes, 64);
    } else
        FLEX_LOG_ERROR(this, "Trying to destroy an invalid convex mesh");
}

void Library::GetConvexMeshBounds(NvFlexConvexMeshId id, NvFlexFloat3* lower,
                                  NvFlexFloat3* upper) {
    if (id) {
        auto context = mContext;
        NvFlexUint numMaxConvexes = mConvexMeshData->mConvexes.GetSize();
        if (mConvexMeshBoundsReadbackBuffer.GetSize() < numMaxConvexes) {
            mConvexMeshBoundsReadbackBuffer.Create(context, numMaxConvexes,
                                                   eNvFlexCpuAccessMode_read);
        }

        NvFlexContextCopyBuffer(
            context, mConvexMeshBoundsReadbackBuffer, (id - 1) * sizeof(ConvexMeshDevice),
            mConvexMeshData->mConvexes, (id - 1) * sizeof(ConvexMeshDevice),
            sizeof(ConvexMeshDevice));

        auto vptr = mConvexMeshBoundsReadbackBuffer.Map(
            context, eNvFlexStagingCpuAccess_read, false);
        *lower = make_float3(vptr[id - 1].mLower);
        *upper = make_float3(vptr[id - 1].mUpper);
        mConvexMeshBoundsReadbackBuffer.Unmap(context);
    } else
        FLEX_LOG_ERROR(this, "Trying to query bounds for an invalid convex mesh");
}

const char* Library::GetDeviceName() {
    return mDeviceName;
}

void Library::ExecuteContext() {
    mDevice->executeContext(mContext);
}

void Library::WaitContext() {
    mDevice->waitForContext(mContext);
}

void Library::ResetContext(bool waitForPrevious) {
    mDevice->prepareContext(mContext, waitForPrevious);
}

Library::Library()
    : mErrorFunc{},
      mDevice{},
      mContext{},
      mDeviceName{},
      mShaderPredict{},
      mShaderCalculateBounds{},
      mShaderCalculateBoundsGroup{},
      mShaderCalculateBoundsFinalize{},
      mShaderCalculateParticleHash{},
      mShaderCreateGrid{},
      mShaderClearInt{},
      mShaderClearFloat4{},
      mShaderClearConstantBufferInt{},
      mShaderClearConstantBufferFloat4{},
      mShaderClearCellBuckets{},
      mShaderReorderParticles{},
      mShaderCollideParticles{},
      mShaderCollideShapes{},
      mShaderContinuousShockPropagation{},
      mShaderCalculateDensity{},
      mShaderCalculateDensitySurfaceTension{},
      mShaderSolveDensities{},
      mShaderSolveDensitiesNonFluid{},
      mShaderSolveDensitiesSurfaceTension{},
      mShaderCalculateInflatableVolume{},
      mShaderSolveInflatableVolume{},
      mShaderSolveSprings{},
      mShaderSolveShapes{},
      mShaderSolveShapes32{},
      mShaderSolveShapes128{},
      mShaderSolveShapesPlasticDeformation{},
      mShaderSolveShapesPlasticDeformation32{},
      mShaderSolveShapesPlasticDeformation128{},
      mShaderSolveSpringsXPBD{},
      mShaderCalculateInflatableVolumeXPBD{},
      mShaderSolveShapesXPBD{},
      mShaderSolveShapesPlasticDeformationXPBD{},
      mShaderApplyDeltas{},
      mShaderSolveContactsSequential{},
      mShaderSolveContactsAveraged{},
      mShaderSolveContactsAccumulate{},
      mShaderCollideTriangles{},
      mShaderUpdateVelocities{},
      mShaderCalculateVorticity{},
      mShaderSolveVelocities{},
      mShaderUpdateTriangles{},
      mShaderUpdateTrianglesInit{},
      mShaderUpdateVertexNormalsInit{},
      mShaderUpdateVertexNormals{},
      mShaderNormalizeVertexNormals{},
      mShaderFinalize{},
      mShaderCreateDiffuseParticles{},
      mShaderUpdateDiffuseParticles{},
      mShaderClampDiffuseParticleCount{},
      mShaderCompactDiffuseParticles{},
      mShaderSmoothPositions{},
      mShaderCalculateAnisotropy{},
      mShaderComputeTriangleBounds{},
      mShaderTransformShapeBounds{},
      mShaderSpringsGenerateIndices{},
      mShaderSpringsParticleRange{},
      mShaderSpringsReorder{},
      mCellBucketStarts{},
      mCellBucketEnds{},
      mBVHBuilder{},
      mConstTriMeshInfo{},
      mStaticTriLowers{},
      mStaticTriUppers{},
      mMaxStaticTris{},
      mIsSHFLSupported{},
      mIsFP32ATOMICSupported{},
      mAGSContext{},
      mIsSwizzleSupported{},
      mEnableAmdDriverBugWorkaround{},
      mGpuVendorId{},
      mSMCount{},
      mSyncQuery{},
      mTriangleMeshData{},
      mConvexMeshData{},
      mSDFData{},
      mResourceTracker{} {}

Library::~Library() {
    NvFlexReleaseComputeShader(mShaderPredict);
    NvFlexReleaseComputeShader(mShaderCalculateBounds);
    NvFlexReleaseComputeShader(mShaderCalculateBoundsGroup);
    NvFlexReleaseComputeShader(mShaderCalculateBoundsFinalize);
    NvFlexReleaseComputeShader(mShaderCalculateParticleHash);
    NvFlexReleaseComputeShader(mShaderCreateGrid);
    NvFlexReleaseComputeShader(mShaderClearInt);
    NvFlexReleaseComputeShader(mShaderClearFloat4);
    mShaderClearConstantBufferInt = nullptr;
    mShaderClearConstantBufferFloat4 = nullptr;
    NvFlexReleaseComputeShader(mShaderClearCellBuckets);
    NvFlexReleaseComputeShader(mShaderReorderParticles);
    NvFlexReleaseComputeShader(mShaderCollideParticles);
    NvFlexReleaseComputeShader(mShaderCollideShapes);
    NvFlexReleaseComputeShader(mShaderContinuousShockPropagation);
    NvFlexReleaseComputeShader(mShaderCalculateDensity);
    NvFlexReleaseComputeShader(mShaderCalculateDensitySurfaceTension);
    NvFlexReleaseComputeShader(mShaderSolveDensities);
    NvFlexReleaseComputeShader(mShaderSolveDensitiesNonFluid);
    NvFlexReleaseComputeShader(mShaderSolveDensitiesSurfaceTension);
    NvFlexReleaseComputeShader(mShaderCalculateInflatableVolume);
    NvFlexReleaseComputeShader(mShaderSolveInflatableVolume);
    NvFlexReleaseComputeShader(mShaderSolveSprings);
    NvFlexReleaseComputeShader(mShaderSolveShapes);
    NvFlexReleaseComputeShader(mShaderSolveShapes32);
    NvFlexReleaseComputeShader(mShaderSolveShapes128);
    NvFlexReleaseComputeShader(mShaderSolveShapesPlasticDeformation);
    NvFlexReleaseComputeShader(mShaderSolveShapesPlasticDeformation32);
    NvFlexReleaseComputeShader(mShaderSolveShapesPlasticDeformation128);
    NvFlexReleaseComputeShader(mShaderSolveSpringsXPBD);
    NvFlexReleaseComputeShader(mShaderCalculateInflatableVolumeXPBD);
    NvFlexReleaseComputeShader(mShaderSolveShapesXPBD);
    NvFlexReleaseComputeShader(mShaderSolveShapesPlasticDeformationXPBD);
    NvFlexReleaseComputeShader(mShaderApplyDeltas);
    NvFlexReleaseComputeShader(mShaderSolveContactsSequential);
    NvFlexReleaseComputeShader(mShaderSolveContactsAveraged);
    NvFlexReleaseComputeShader(mShaderSolveContactsAccumulate);
    NvFlexReleaseComputeShader(mShaderCollideTriangles);
    NvFlexReleaseComputeShader(mShaderUpdateVelocities);
    NvFlexReleaseComputeShader(mShaderCalculateVorticity);
    NvFlexReleaseComputeShader(mShaderSolveVelocities);
    NvFlexReleaseComputeShader(mShaderUpdateTriangles);
    NvFlexReleaseComputeShader(mShaderUpdateTrianglesInit);
    NvFlexReleaseComputeShader(mShaderUpdateVertexNormalsInit);
    NvFlexReleaseComputeShader(mShaderUpdateVertexNormals);
    NvFlexReleaseComputeShader(mShaderNormalizeVertexNormals);
    NvFlexReleaseComputeShader(mShaderFinalize);
    NvFlexReleaseComputeShader(mShaderCreateDiffuseParticles);
    NvFlexReleaseComputeShader(mShaderUpdateDiffuseParticles);
    NvFlexReleaseComputeShader(mShaderClampDiffuseParticleCount);
    NvFlexReleaseComputeShader(mShaderCompactDiffuseParticles);
    NvFlexReleaseComputeShader(mShaderSmoothPositions);
    NvFlexReleaseComputeShader(mShaderCalculateAnisotropy);
    NvFlexReleaseComputeShader(mShaderComputeTriangleBounds);
    NvFlexReleaseComputeShader(mShaderTransformShapeBounds);
    NvFlexReleaseComputeShader(mShaderSpringsGenerateIndices);
    NvFlexReleaseComputeShader(mShaderSpringsParticleRange);
    NvFlexReleaseComputeShader(mShaderSpringsReorder);
    mCellBucketStarts = nullptr;
    mCellBucketEnds = nullptr;
    delete mBVHBuilder;
    mConstTriMeshInfo = nullptr;
    mStaticTriLowers = nullptr;
    mStaticTriUppers = nullptr;
    NvFlexReleaseFence(mSyncQuery);
    mResourceTracker.cleanup(this);
    delete mTriangleMeshData;
    delete mConvexMeshData;
    mConvexMeshBoundsReadbackBuffer = nullptr;
    delete mSDFData;

    NvFlexReleaseContext(mContext);
    SafeRelease(mDevice);

    NvFlexDeferredRelease(2000.f);
}

}  // namespace NvFlex
