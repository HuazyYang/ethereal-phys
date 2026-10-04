#include "BVHBuilder.h"
#include "RadixSort.h"
#include "Data.h"
#include "Library.h"
#include "ClientHelper.h"

#include <bvh/CalculateMortonCodes.hlsl.h>
#include <bvh/CalculateKeyDeltas.hlsl.h>
#include <bvh/BuildLeaves.hlsl.h>
#include <bvh/BuildHierarchy.hlsl.h>
#include <bvh/ComputeTotalBounds.hlsl.h>
#include <bvh/ComputeTotalBoundsNV.hlsl.h>
#include <bvh/ComputeTotalBoundsAMD.hlsl.h>
#include <bvh/ComputeTotalBoundsGroup.hlsl.h>
#include <bvh/ComputeTotalBoundsGroupNV.hlsl.h>
#include <bvh/ComputeTotalBoundsGroupAMD.hlsl.h>
#include <bvh/ComputeTotalBoundsFinalize.hlsl.h>

namespace NvFlex {

LinearBVHBuilderGPU::LinearBVHBuilderGPU(Library* lib, NvFlexContext* context,
                                         bool isSHFLSupported, int SMCount,
                                         bool isSwizzleSupported) {
    mIsSHFLSupported = isSHFLSupported;
    mSMCount = SMCount;
    mMaxNumTriMeshes = 0;
    mContext = context;
    mLib = lib;

    auto createShader = [context](const BYTE* cs, uint64_t cs_length,
                                  const wchar_t* label, int nvapiSlot) {
        NvFlexComputeShaderDesc desc = {};
        desc.cs = cs;
        desc.cs_length = cs_length;
        desc.label = label;
        desc.NVAPI_Slot = nvapiSlot;
        return NvFlexCreateComputeShader(context, &desc);
    };
#define NVFLEX_CREATE_SHADER_ARGS(name) g_##name, sizeof(g_##name), L#name

    mShaderCalculateMortonCodes =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateMortonCodes), 0xFFFFFFFF);
    mShaderCalculateKeyDeltas =
        createShader(NVFLEX_CREATE_SHADER_ARGS(CalculateKeyDeltas), 0xFFFFFFFF);
    mShaderBuildLeaves = createShader(NVFLEX_CREATE_SHADER_ARGS(BuildLeaves), 0xFFFFFFFF);
    mShaderBuildHierarchy =
        createShader(NVFLEX_CREATE_SHADER_ARGS(BuildHierarchy), 0xFFFFFFFF);
    if (isSHFLSupported) {
        mShaderComputeTotalBounds =
            createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsNV), 7u);
        mShaderComputeTotalBoundsGroup =
            createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsGroupNV), 7u);
        mShaderComputeTotalBoundsFinalize =
            createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsFinalize), 7u);
    } else {
        if (isSwizzleSupported) {
            mShaderComputeTotalBounds =
                createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsAMD), 0xFFFFFFFF);
            mShaderComputeTotalBoundsGroup = createShader(
                NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsGroupAMD), 0xFFFFFFFF);
            mShaderComputeTotalBoundsFinalize = createShader(
                NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsFinalize), 0xFFFFFFFF);
        } else {
            mShaderComputeTotalBounds =
                createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBounds), 0xFFFFFFFF);
            mShaderComputeTotalBoundsGroup = createShader(
                NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsGroup), 0xFFFFFFFF);
            mShaderComputeTotalBoundsFinalize = createShader(
                NVFLEX_CREATE_SHADER_ARGS(ComputeTotalBoundsFinalize), 0xFFFFFFFF);
        }
    }
    // mShaderComputeTotalInvEdges =
    //     createShader(NVFLEX_CREATE_SHADER_ARGS(ComputeTotalInvEdges), 0xFFFFFFFF);

    mConstantBuffer.Create(context, 1, 1);
    mTotalLower.CreateWithZero(context, 1, "LinearBVHBuilderGPU::mTotalLower");
    mTotalUpper.CreateWithZero(context, 1, "LinearBVHBuilderGPU::mTotalUpper");
    mTotalInvEdges.CreateWithZero(context, 1, "LinearBVHBuilderGPU::mTotalInvEdges");
}

LinearBVHBuilderGPU::~LinearBVHBuilderGPU() {
    NvFlexReleaseComputeShader(mShaderCalculateMortonCodes);
    NvFlexReleaseComputeShader(mShaderCalculateKeyDeltas);
    NvFlexReleaseComputeShader(mShaderBuildLeaves);
    NvFlexReleaseComputeShader(mShaderBuildHierarchy);
    NvFlexReleaseComputeShader(mShaderComputeTotalBounds);
    NvFlexReleaseComputeShader(mShaderComputeTotalBoundsGroup);
    NvFlexReleaseComputeShader(mShaderComputeTotalBoundsFinalize);
    // NvFlexReleaseComputeShader(mShaderComputeTotalInvEdges);
    SafeRelease(mBVHRadixSort);
}

void LinearBVHBuilderGPU::Build(BVH* bvh, NvFlexBuffer* itemLowers,
                                NvFlexBuffer* itemUppers, NvFlexBuffer* itemPriorities,
                                NvFlexUint numItems, const NvFlexBounds3f* totalBounds,
                                TimerPool* timerPool) {
    auto context = mContext;
    int numNodes = 2 * numItems;
    bvh->Resize(mContext, numNodes);

    if (numItems > mMaxItems) {
        int itemsToAlloc = CalculateSlack(numItems);
        int nodesToAlloc = CalculateSlack(numNodes);

        uint sortSize = alignUp<1024>(itemsToAlloc);

        mDeltas.Create(context, itemsToAlloc, "LinearBVHBuilderGPU::mDeltas");
        mRangeLefts.Create(context, nodesToAlloc, "LinearBVHBuilderGPU::mRangeLefts");
        mRangeRights.Create(context, nodesToAlloc, "LinearBVHBuilderGPU::mRangeRights");
        mNumChildren.Create(context, nodesToAlloc, "LinearBVHBuilderGPU::mNumChildren");

        uint maxGroupBounds = divCeil<256>(divCeil<256>(itemsToAlloc));

        mBoundsLower.CreateWithZero(context, divCeil<256>(itemsToAlloc),
                                    "LinearBVHBuilderGPU::mBoundsLower");
        mBoundsUpper.CreateWithZero(context, divCeil<256>(itemsToAlloc),
                                    "LinearBVHBuilderGPU::mBoundsUpper");

        mGroupBoundsLower.CreateWithZero(context, maxGroupBounds,
                                         "LinearBVHBuilderGPU::mGroupBoundsLower");
        mGroupBoundsUpper.CreateWithZero(context, maxGroupBounds,
                                         "LinearBVHBuilderGPU::mGroupBoundsUpper");

        mMaxItems = itemsToAlloc;
    }

    int kNumBlocks = divCeil<256>(alignUp<1024>(numItems));
    {
        auto vptr = mConstantBuffer.Map(context);
        *vptr = numItems;
        mConstantBuffer.Unmap(context);
    }

    if (totalBounds) {
        float3 invEdges = 1.f / (boundsDiagonal(*totalBounds) + 1e-4f);

        mTotalInvEdges.Write(context, 0, 1, &invEdges);

        mTotalLower.Write(context, 0, 1, &totalBounds->lower);
        mTotalUpper.Write(context, 0, 1, &totalBounds->upper);
    } else {
        int kNumBoundsBlocks = kNumBlocks;
        int kNumGroupBoundsBlocks = divCeil<256>(kNumBlocks);
        NvFlexDispatchParams params = {};
        params.shader = mShaderComputeTotalBounds;
        params.readOnly[0] = NvFlexBufferGetResource(itemLowers);
        params.readOnly[1] = NvFlexBufferGetResource(itemUppers);
        params.readWrite[0] = NvFlexBufferGetResourceRW(mBoundsLower);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mBoundsUpper);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mTotalLower);
        params.readWrite[3] = NvFlexBufferGetResourceRW(mTotalUpper);
        params.gridDim = make_dim(kNumBlocks, 1, 1);
        params.rootConstantBuffer = mConstantBuffer;
        NvFlexContextDispatch(mContext, &params);

        memset(&params, 0, sizeof(params));
        params.shader = mShaderComputeTotalBoundsGroup;
        params.readOnly[2] = NvFlexBufferGetResource(mBoundsLower);
        params.readOnly[3] = NvFlexBufferGetResource(mBoundsUpper);
        params.readWrite[0] = NvFlexBufferGetResourceRW(mGroupBoundsLower);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mGroupBoundsUpper);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mTotalLower);
        params.readWrite[3] = NvFlexBufferGetResourceRW(mTotalUpper);
        params.gridDim = make_dim(kNumGroupBoundsBlocks, 1, 1);
        params.rootConstantBuffer = mConstantBuffer;
        NvFlexContextDispatch(mContext, &params);

        memset(&params, 0, sizeof(params));
        params.shader = mShaderComputeTotalBoundsFinalize;
        params.readOnly[2] = NvFlexBufferGetResource(mGroupBoundsLower);
        params.readOnly[3] = NvFlexBufferGetResource(mGroupBoundsUpper);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mTotalLower);
        params.readWrite[3] = NvFlexBufferGetResourceRW(mTotalUpper);
        params.readWrite[4] = NvFlexBufferGetResourceRW(mTotalInvEdges);
        params.gridDim = make_dim(1);
        params.rootConstantBuffer = mConstantBuffer;
        NvFlexContextDispatch(mContext, &params);
    }

    if (numItems > mBVHRadixSortSize) {
        SafeRelease(mBVHRadixSort);
        RadixSortDesc radixSortDesc = {};
        radixSortDesc.maxSortBlocks = divCeil<1024>(numItems);
        mBVHRadixSort = createRadixSort(mContext, &radixSortDesc);
        mBVHRadixSortSize = alignUp<1024>(numItems);
    }

    auto sortBuffers = mBVHRadixSort->getBuffer();

    NvFlexDispatchParams params = {};
    params.shader = mShaderCalculateMortonCodes;
    params.readOnly[0] = NvFlexBufferGetResource(itemLowers);
    params.readOnly[1] = NvFlexBufferGetResource(itemUppers);
    params.readOnly[2] = NvFlexBufferGetResource(mTotalLower);
    params.readOnly[3] = NvFlexBufferGetResource(mTotalInvEdges);
    params.readWrite[0] = NvFlexBufferGetResourceRW(sortBuffers.val);
    params.readWrite[1] = NvFlexBufferGetResourceRW(sortBuffers.key);
    params.gridDim = make_dim(kNumBlocks, 1, 1);
    params.rootConstantBuffer = mConstantBuffer;
    NvFlexContextDispatch(mContext, &params);

    RadixSortParams sortParams = {};
    sortParams.numKeys = numItems;
    sortParams.numSortBlocks = divCeil<1024>(numItems);
    sortParams.bits = 32;
    mBVHRadixSort->sort(mContext, &sortParams);

    sortBuffers = mBVHRadixSort->getBuffer();

    memset(&params, 0, sizeof(params));
    params.shader = mShaderCalculateKeyDeltas;
    params.readOnly[0] = NvFlexBufferGetResource(sortBuffers.key);
    params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
    params.gridDim = make_dim(kNumBlocks, 1, 1);
    params.rootConstantBuffer = mConstantBuffer;
    NvFlexContextDispatch(mContext, &params);

    memset(&params, 0, sizeof(params));
    params.shader = mShaderBuildLeaves;
    params.readOnly[0] = NvFlexBufferGetResource(itemLowers);
    params.readOnly[1] = NvFlexBufferGetResource(itemUppers);
    params.readOnly[2] = NvFlexBufferGetResource(sortBuffers.val);
    params.readWrite[0] = NvFlexBufferGetResourceRW(mRangeLefts);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mRangeRights);
    params.readWrite[2] = NvFlexBufferGetResourceRW(bvh->mNodeLowers);
    params.readWrite[3] = NvFlexBufferGetResourceRW(bvh->mNodeUppers);
    params.gridDim = make_dim(kNumBlocks, 1, 1);
    params.rootConstantBuffer = mConstantBuffer;
    NvFlexContextDispatch(mContext, &params);

    mLib->ClearBufferInt(mNumChildren, numNodes * sizeof(int), 0);

    memset(&params, 0, sizeof(params));
    params.shader = mShaderBuildHierarchy;
    params.readOnly[0] = NvFlexBufferGetResource(itemLowers);
    params.readOnly[1] = NvFlexBufferGetResource(itemUppers);
    params.readOnly[2] = NvFlexBufferGetResource(mDeltas);
    params.readWrite[0] = NvFlexBufferGetResourceRW(mRangeLefts);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mRangeRights);
    params.readWrite[2] = NvFlexBufferGetResourceRW(bvh->mNodeLowers);
    params.readWrite[3] = NvFlexBufferGetResourceRW(bvh->mNodeUppers);
    params.readWrite[4] = NvFlexBufferGetResourceRW(mNumChildren);
    params.readWrite[5] = NvFlexBufferGetResourceRW(bvh->mRootNode);
    params.gridDim = make_dim(kNumBlocks, 1, 1);
    params.rootConstantBuffer = mConstantBuffer;
    NvFlexContextDispatch(mContext, &params);
}

}  // namespace NvFlex