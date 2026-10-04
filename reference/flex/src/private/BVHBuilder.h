#ifndef LINEARBVHBUILDERGPU_H
#define LINEARBVHBUILDERGPU_H
#include "Types.h"
#include "Allocable.h"
#include <nvflex/NvFlexContextExt.h>
#include "ResourceWrapper.h"

namespace NvFlex {

struct Library;
struct RadixSort;
struct BVH;
struct Bounds;
struct TimerPool;

struct LinearBVHBuilderGPU : Allocable {
    LinearBVHBuilderGPU(Library *lib, NvFlexContext* context, bool isSHFLSupported, int SMCount,
                        bool isSwizzleSupported);
    ~LinearBVHBuilderGPU();

    void Build(BVH* bvh, NvFlexBuffer* itemLowers, NvFlexBuffer* itemUppers,
               NvFlexBuffer* itemPriorities, NvFlexUint numItems, const NvFlexBounds3f *totalBounds, TimerPool *timerPool);

 private:
    HStructuredBuffer<float> mDeltas = {};
    HStructuredBuffer<int> mRangeLefts = {};
    HStructuredBuffer<int> mRangeRights = {};
    HStructuredBuffer<int> mNumChildren = {};
    HStructuredBuffer<float3> mTotalLower = {};
    HStructuredBuffer<float3> mTotalUpper = {};
    HStructuredBuffer<float3> mTotalInvEdges = {};
    int mMaxItems = {};
    HUploadBuffer<int> mConstantBuffer = {};
    NvFlexComputeShader* mShaderCalculateMortonCodes = {};
    NvFlexComputeShader* mShaderCalculateKeyDeltas = {};
    NvFlexComputeShader* mShaderBuildLeaves = {};
    NvFlexComputeShader* mShaderBuildHierarchy = {};
    NvFlexComputeShader* mShaderComputeTotalBounds = {};
    NvFlexComputeShader* mShaderComputeTotalBoundsGroup = {};
    NvFlexComputeShader* mShaderComputeTotalBoundsFinalize = {};
    // NvFlexComputeShader* mShaderComputeTotalInvEdges = {};
    HStructuredBuffer<float4> mBoundsLower = {};
    HStructuredBuffer<float4> mBoundsUpper = {};
    HStructuredBuffer<float4> mGroupBoundsLower = {};
    HStructuredBuffer<float4> mGroupBoundsUpper = {};
    RadixSort* mBVHRadixSort = {};
    int mBVHRadixSortSize = {};
    bool mIsSHFLSupported = {};
    int mSMCount = {};
    int mMaxNumTriMeshes = {};
    NvFlexContext* mContext = {};
    Library* mLib = {};
};

}


#endif /* LINEARBVHBUILDERGPU_H */
