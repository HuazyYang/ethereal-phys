#ifndef FLEXDATA_H
#define FLEXDATA_H
#include "Types.h"
#include "Allocable.h"
#include "NvFlexImpl.h"
#include <nvflex/NvFlexContextExt.h>
#include <VectorCached.h>
#include "ResourceWrapper.h"

namespace NvFlex {

struct Library;
struct TimerPool;

NvFlexUint CalculateSlack(NvFlexUint n);

struct BVH : Allocable {
    HStructuredBuffer<float4> mNodeLowers;
    HStructuredBuffer<float4> mNodeUppers;
    HStructuredBuffer<int> mRootNode;
    int mNumNodes;
    int mMaxNodes;
    int mMaxDepth;

    BVH();
    ~BVH();

    void Resize(NvFlexContext *context, int numNodes);
};

struct TriangleMeshHost : Allocable {
    HStructuredBuffer<float4> mVertices;
    HStructuredBuffer<int> mIndices;
    BVH mBVH;
    int mNumTris;
    int mNumVertices;
    int mNumIndices;
    int mMaxTris;
    int mMaxVertices;
    NvFlexFloat3 mLower;
    NvFlexFloat3 mUpper;

    TriangleMeshHost();
    ~TriangleMeshHost();
};

struct BVHArray : Allocable {
    HStructuredBuffer<float4> mNodeLowersArray;
    HStructuredBuffer<float4> mNodeUppersArray;
    HStructuredBuffer<int> mRootNodeArray;
    HStructuredBuffer<float> mBVHTestResult;

    BVHArray();
    ~BVHArray();
};

struct TriangleMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mIndexStart;
    int mVertexStart;
    int mNodeStart;
    int mPad;
};

struct TriangleMeshData : Allocable {
    VectorCached<TriangleMeshHost *> mTriMeshTableHost;
    HStructuredBuffer<TriangleMeshDevice> mTriMeshTableDevice;
    HStructuredBuffer<float4> mVerticesArray;
    HStructuredBuffer<int> mIndicesArray;
    BVHArray mBvhData;
    int mMaxNumVertices;
    int mMaxNumTriangles;
    int mMaxNumMeshes;
    bool mRebuildTriangleData;

    TriangleMeshData();
    ~TriangleMeshData();

    int Allocate();
    void Free(int index);
    void Resize(NvFlexContext *context, int numVertices, int numTriangles, int numMeshes);
    void Rebuild(NvFlexContext *context, TimerPool *timerPool);

private:
    void FlattenTriangleMeshes(NvFlexContext *context);
};

struct ConvexMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mPlaneOffset;
    int mPlaneCount;
    int mPad[2];
};

struct ConvexMeshData : Allocable {
    HStructuredBuffer<float4> mPlanes;
    HStructuredBuffer<ConvexMeshDevice> mConvexes;
    int mNumConvexes;
    int mMaxConvexes;
    int mMaxPlanesPerConvex;
    int *mFreeList;

    ConvexMeshData(NvFlexContext *context, int maxConvexes, int maxPlanesPerConvex);
    ~ConvexMeshData();

    int Allocate();
    void Free(int index);
    void Update(NvFlexContext *context, int index, NvFlexBuffer *planes, int numPlanes,
                NvFlexFloat3 lower, NvFlexFloat3 upper);
};

struct SDFDevice {
    int4 mDim;
    float4 mInvDim;
};

struct SDFData : Allocable {
    VectorCached<AutoPtr<NvFlexTexture3D>> mTextures;
    HStructuredBuffer<SDFDevice> mSDFs;
    int mNumSDFs;
    int mMaxSDFs;
    int *mFreeList;

    SDFData(NvFlexContext *context, int maxSDFs);
    ~SDFData();

    int Allocate();
    void Free(int index);
    void Update(NvFlexContext *context, int index, NvFlexTexture3D *data);
};

struct ShapeData: Allocable {
    HStructuredBuffer<float4> mAabbMin;
    HStructuredBuffer<float4> mAabbMax;
    HStructuredBuffer<NvFlexCollisionGeometry> mGeometry;
    HStructuredBuffer<float4> mPositions;
    HStructuredBuffer<float4> mRotations;
    HStructuredBuffer<float4> mPrevPositions;
    HStructuredBuffer<float4> mPrevRotations;
    HStructuredBuffer<NvFlexUint> mFlags;
    HUploadBuffer<NvFlexUint> mNumShapesGPU;
    BVH mBVH;
    int mNumShapes;
    int mMaxShapes;
    NvFlexFloat3 mLower;
    NvFlexFloat3 mUpper;
    bool mRebuildShapeData;

    ShapeData();
    ~ShapeData();

    void RebuildShapeData(Library *lib, TimerPool *timerPool);
};

struct InflatableDevice {
    int mStartTri;
    int mNumTris;
    float mRestVolume;
    float mConstraintScale;
};

}

#endif /* FLEXDATA_H */
