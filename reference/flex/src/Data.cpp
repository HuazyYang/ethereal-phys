#include "Data.h"
#include "Library.h"
#include "BVHBuilder.h"
#include "TimerPool.h"

namespace NvFlex {

NvFlexUint CalculateSlack(NvFlexUint n) {
    return n + (n >> 1);
}

BVH::BVH()
    : mNodeLowers{},
      mNodeUppers{},
      mRootNode{},
      mNumNodes{},
      mMaxNodes{},
      mMaxDepth{} {}

BVH::~BVH() {}

void BVH::Resize(NvFlexContext *context, int numNodes) {
    if (numNodes > mMaxNodes) {
        int numToAlloc = CalculateSlack(numNodes);

        mNodeLowers.Create(context, numToAlloc, "BVH::mNodeLowers");
        mNodeUppers.Create(context, numToAlloc, "BVH::mNodeUppers");

        mMaxNodes = numToAlloc;

        if (!mRootNode)
            mRootNode.Create(context, 1, "BVH::mRootNode");
    }

    mNumNodes = numNodes;
}

TriangleMeshHost::TriangleMeshHost()
    : mVertices{},
      mIndices{},
      mBVH{},
      mNumTris{},
      mNumVertices{},
      mNumIndices{},
      mMaxTris{},
      mMaxVertices{},
      mLower{},
      mUpper{} {}

TriangleMeshHost::~TriangleMeshHost() {}

BVHArray::BVHArray()
    : mNodeLowersArray{},
      mNodeUppersArray{},
      mRootNodeArray{},
      mBVHTestResult{} {}

BVHArray::~BVHArray() {}

TriangleMeshData::TriangleMeshData()
    : mTriMeshTableHost{},
      mTriMeshTableDevice{},
      mVerticesArray{},
      mIndicesArray{},
      mBvhData{},
      mMaxNumVertices{},
      mMaxNumTriangles{},
      mMaxNumMeshes{},
      mRebuildTriangleData{} {}

TriangleMeshData::~TriangleMeshData() {
    for (auto &mesh : mTriMeshTableHost) {
        if (mesh) {
            delete mesh;
            mesh = nullptr;
        }
    }
}

int TriangleMeshData::Allocate() {
    auto mesh = new TriangleMeshHost;
    int index = -1;
    for (int i = 0; i < (int)mTriMeshTableHost.size(); ++i) {
        if (!mTriMeshTableHost[i]) {
            mTriMeshTableHost[i] = mesh;
            index = i;
            break;
        }
    }

    if (index == -1) {
        index = (int)mTriMeshTableHost.size();
        mTriMeshTableHost.push_back(mesh);
    }
    return index;
}

void TriangleMeshData::Free(int index) {
    auto &mesh = mTriMeshTableHost[index];
    delete mesh;
    mesh = nullptr;
}

void TriangleMeshData::Resize(NvFlexContext *context, int numVertices, int numTriangles,
                              int numMeshes) {
    int numTrisToAlloc;
    int numMeshesToAlloc;
    int numToAlloc;

    if (numVertices > mMaxNumVertices) {
        numToAlloc = CalculateSlack(numVertices);

        mVerticesArray.Create(context, numToAlloc,
                              "NvFlexSolver::TriangleMesh::VerticesUber");

        mMaxNumVertices = numToAlloc;
    }

    if (numTriangles > mMaxNumTriangles) {
        numTrisToAlloc = CalculateSlack(numTriangles);

        mIndicesArray.Create(context, 3 * numTrisToAlloc,
                             "NvFlexSolver::TriangleMesh::IndicesUber");

        mBvhData.mNodeLowersArray.Create(context, 2 * numTrisToAlloc,
                                         "NvFlexSolver::TriangleMesh::BvhLowerUber");

        mBvhData.mNodeUppersArray.Create(context, 2 * numTrisToAlloc,
                                         "NvFlexSolver::TriangleMesh::BvhUpperUber");

        mMaxNumTriangles = numTrisToAlloc;
    }

    if (numMeshes > mMaxNumMeshes) {
        numMeshesToAlloc = CalculateSlack(numMeshes);

        mBvhData.mRootNodeArray.Create(context, numMeshesToAlloc,
                                       "NvFlexSolver::TriangleMesh::BvhRootsUber");

        mTriMeshTableDevice.Create(context, numMeshesToAlloc,
                                   "NvFlexSolver::TriangleMesh::OffsetUber");

        mMaxNumMeshes = numMeshesToAlloc;
    }
}

void TriangleMeshData::Rebuild(NvFlexContext *context, TimerPool *timerPool) {
    if (mRebuildTriangleData) {
        NVFLEX_PROFILE_SECTION("FlattenTriangleMeshes", timerPool);
        FlattenTriangleMeshes(context);
        mRebuildTriangleData = 0;
    }
}

void TriangleMeshData::FlattenTriangleMeshes(NvFlexContext *context) {
    NvFlexUint totalNumTriangles = 0;
    NvFlexUint totalNumVertices = 0;
    NvFlexUint totalNumMeshes = mTriMeshTableHost.size();
    for (NvFlexUint i = 0; i < totalNumMeshes; ++i) {
        auto mesh = mTriMeshTableHost[i];
        if (mesh) {
            totalNumTriangles += mesh->mNumTris;
            totalNumVertices += mesh->mNumVertices;
        }
    }

    Resize(context, totalNumVertices, totalNumTriangles, totalNumMeshes);

    NvFlexUint writeVertexOffset = 0;
    NvFlexUint writeIndexOffset = 0;
    NvFlexUint writeNodeOffset = 0;

    VectorCached<TriangleMeshDevice> meshOffsets((size_t)totalNumMeshes);

    for (NvFlexUint i = 0; i < totalNumMeshes; ++i) {
        auto mesh = mTriMeshTableHost[i];
        if (!mesh)
            continue;

        NvFlexContextCopyBuffer(context, mVerticesArray, sizeof(float4) * writeVertexOffset,
                                mesh->mVertices, 0, sizeof(float4) * mesh->mNumVertices);
        NvFlexContextCopyBuffer(context, mIndicesArray, sizeof(int) * writeIndexOffset,
                                mesh->mIndices, 0, sizeof(int) * mesh->mNumIndices);
        NvFlexContextCopyBuffer(context, mBvhData.mNodeLowersArray,
                                sizeof(float4) * writeNodeOffset, mesh->mBVH.mNodeLowers, 0,
                                sizeof(float4) * mesh->mBVH.mNumNodes);
        NvFlexContextCopyBuffer(context, mBvhData.mNodeUppersArray,
                                sizeof(float4) * writeNodeOffset, mesh->mBVH.mNodeUppers, 0,
                                sizeof(float4) * mesh->mBVH.mNumNodes);
        NvFlexContextCopyBuffer(context, mBvhData.mRootNodeArray, sizeof(int) * i,
                                mesh->mBVH.mRootNode, 0, sizeof(int));

        auto &offsets = meshOffsets[i];
        offsets.mVertexStart = writeVertexOffset;
        offsets.mIndexStart = writeIndexOffset;
        offsets.mNodeStart = writeNodeOffset;
        offsets.mLower = make_float4(mesh->mLower, 0.f);
        offsets.mUpper = make_float4(mesh->mUpper, 0.f);

        writeVertexOffset += mesh->mNumVertices;
        writeIndexOffset += mesh->mNumIndices;
        writeNodeOffset += mesh->mBVH.mNumNodes;
    }

    mTriMeshTableDevice.Write(context, 0, totalNumMeshes, meshOffsets.data());
}

ConvexMeshData::ConvexMeshData(NvFlexContext *context, int maxConvexes,
                               int maxPlanesPerConvex)
    : mPlanes{},
      mConvexes{},
      mNumConvexes{},
      mMaxConvexes{maxConvexes},
      mMaxPlanesPerConvex{maxPlanesPerConvex},
      mFreeList{} {
    mPlanes.Create(context, mMaxConvexes * mMaxPlanesPerConvex);
    mConvexes.Create(context, mMaxConvexes);

    mFreeList = (int *)NvFlex::Allocable::allocate(sizeof(int) * mMaxConvexes);
    for (int i = 0; i < mMaxConvexes; ++i)
        mFreeList[i] = i;
}

ConvexMeshData::~ConvexMeshData() {
    NvFlex::Allocable::deallocate(mFreeList);
    mNumConvexes = 0;
}

int ConvexMeshData::Allocate() {
    if (mNumConvexes >= mMaxConvexes)
        return -1;
    return mFreeList[mNumConvexes++];
}

void ConvexMeshData::Free(int index) {
    mFreeList[--mNumConvexes] = index;
}

void ConvexMeshData::Update(NvFlexContext *context, int index, NvFlexBuffer *planes,
                            int numPlanes, NvFlexFloat3 lower, NvFlexFloat3 upper) {
    ConvexMeshDevice convex;
    convex.mPlaneCount = numPlanes;
    convex.mPlaneOffset = mMaxPlanesPerConvex * index;
    convex.mLower = make_float4(lower, 0.f);
    convex.mUpper = make_float4(upper, 0.f);

    NvFlexContextCopyBuffer(context, mPlanes, mMaxPlanesPerConvex * index * sizeof(float4),
                            planes, 0, numPlanes * sizeof(float4));

    mConvexes.Write(context, index, 1, &convex);
}

SDFData::SDFData(NvFlexContext *context, int maxSDFs)
    : mSDFs{},
      mNumSDFs{},
      mMaxSDFs{maxSDFs},
      mFreeList{} {
    mTextures.resize(mMaxSDFs);
    mSDFs.Create(context, mMaxSDFs);
    mFreeList = (int *)NvFlex::Allocable::allocate(sizeof(int) * mMaxSDFs);
    for (int i = 0; i < mMaxSDFs; ++i)
        mFreeList[i] = i;
}

SDFData::~SDFData() {
    NvFlex::Allocable::deallocate(mFreeList);
    mNumSDFs = 0;
}

int SDFData::Allocate() {
    if (mNumSDFs >= mMaxSDFs)
        return -1;

    return mFreeList[mNumSDFs++];
}

void SDFData::Free(int index) {
    mTextures[index] = nullptr;
    mFreeList[--mNumSDFs] = index;
}

void SDFData::Update(NvFlexContext *context, int index, NvFlexTexture3D *data) {
    NvFlexTexture3DDesc srcDesc;
    NvFlexTexture3DGetDesc(data, &srcDesc);

    SDFDevice sdf;
    sdf.mDim = make_int4(srcDesc.dim.x, srcDesc.dim.y, srcDesc.dim.z, 0.f);
    sdf.mInvDim = make_float4(1.f / (float)srcDesc.dim.x, 1.f / (float)srcDesc.dim.y,
                              1.f / (float)srcDesc.dim.z, 0.f);

    auto &tex3D = mTextures[index];
    tex3D = nullptr;

    NvFlexTexture3DDesc texDesc = {};
    texDesc.dim = srcDesc.dim;
    texDesc.format = eNvFlexFormat_r32_float;
    tex3D = AutoPtr<NvFlexTexture3D>::TakeOver(NvFlexCreateTexture3D(context, &texDesc));
    NvFlexContextCopyResource(context, NvFlexTexture3DGetResourceRW(tex3D),
                              NvFlexTexture3DGetResource(data));

    mSDFs.Write(context, index, 1, &sdf);
}

ShapeData::ShapeData()
    : mAabbMin{},
      mAabbMax{},
      mGeometry{},
      mPositions{},
      mRotations{},
      mPrevPositions{},
      mPrevRotations{},
      mFlags{},
      mNumShapesGPU{},
      mBVH{},
      mNumShapes{},
      mMaxShapes{},
      mLower{},
      mUpper{},
      mRebuildShapeData{} {}

ShapeData::~ShapeData() {}

void ShapeData::RebuildShapeData(Library *lib, TimerPool *timerPool) {
    if (!mRebuildShapeData)
        return;

    NVFLEX_PROFILE_SECTION("Rebuild shapes", timerPool);

    auto context = lib->mContext;

    if (!mNumShapesGPU)
        mNumShapesGPU.Create(context, 1, 1);

    {
        auto vptr = mNumShapesGPU.Map(context);
        *vptr = mNumShapes;
        mNumShapesGPU.Unmap(context);
    }

    NvFlexDispatchParams params = {};
    params.shader = lib->mShaderTransformShapeBounds;
    params.readWrite[0] = mAabbMin;
    params.readWrite[1] = mAabbMax;
    params.readOnly[0] = mPositions;
    params.readOnly[1] = mRotations;
    params.readOnly[2] = mPrevPositions;
    params.readOnly[3] = mPrevRotations;
    params.readOnly[4] = mFlags;
    params.readOnly[5] = mGeometry;
    if (lib->mConvexMeshData->mConvexes)
        params.readOnly[6] = lib->mConvexMeshData->mConvexes;
    if (lib->mTriangleMeshData->mTriMeshTableDevice)
        params.readOnly[7] = lib->mTriangleMeshData->mTriMeshTableDevice;
    params.gridDim = make_dim(divCeil<256>(mNumShapes), 1, 1);
    params.rootConstantBuffer = mNumShapesGPU;
    NvFlexContextDispatch(context, &params);

    lib->mBVHBuilder->Build(&mBVH, mAabbMin, mAabbMax, mFlags, mNumShapes, nullptr,
                            timerPool);

    mRebuildShapeData = 0;
}

}  // namespace NvFlex