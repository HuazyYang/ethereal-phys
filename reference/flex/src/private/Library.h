#ifndef FLEXLIBRARY_H
#define FLEXLIBRARY_H
#include "Types.h"
#include "Object.h"
#include "NvFlexImpl.h"
#include "NvFlexContextImpl.h"
#include "ClientHelper.h"
#include "ResourceWrapper.h"

namespace NvFlex {

struct Device;
struct LinearBVHBuilderGPU;
struct TriangleMeshData;
struct ConvexMeshData;
struct SDFData;

enum GpuVendorId {
    VENDOR_ID_NVIDIA = 0x10DE,
    VENDOR_ID_AMD = 0x1002,
    VENDOR_ID_OTHERS = 0xEE
};

struct Library : Object, NvFlexLibrary {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()
    uint64_t getGPUBytesUsed() override { return 0; }
    NvFlexErrorCallback GetLogger() override;

    NvFlexTriangleMeshId CreateTriangleMesh();
    NvFlexConvexMeshId CreateConvexMesh();
    NvFlexDistanceFieldId CreateDistanceField();
    void ReleaseTriangleMesh(NvFlexTriangleMeshId id);
    void ReleaseConvexMesh(NvFlexConvexMeshId id);
    void ReleaseDistanceField(NvFlexDistanceFieldId id);

    NvFlexInt GetTriangleMeshes(NvFlexTriangleMeshId *meshes, NvFlexUint n);
    void UpdateTriangleMesh(NvFlexTriangleMeshId mesh, NvFlexBuffer *vertices,
                            NvFlexBuffer *indices, int numVertices, int numTriangles,
                            const NvFlexFloat3 *lower, const NvFlexFloat3 *upper);
    void GetTriangleMeshBounds(NvFlexTriangleMeshId meshId, NvFlexFloat3 *lower,
                               NvFlexFloat3 *upper);

    NvFlexUint GetDistanceFields(NvFlexDistanceFieldId *sdfIds, NvFlexUint n);
    void UpdateDistanceField(NvFlexDistanceFieldId id, NvFlexTexture3D *sdf);

    NvFlexUint GetConvexMeshes(NvFlexConvexMeshId *ids, NvFlexUint n);
    void UpdateConvexMesh(NvFlexConvexMeshId id, NvFlexBuffer *planes, int numPlanes,
                          const NvFlexFloat3 *lower, const NvFlexFloat3 *upper);
    void GetConvexMeshBounds(NvFlexConvexMeshId id, NvFlexFloat3 *lower,
                             NvFlexFloat3 *upper);

    const char *GetDeviceName();

    void ExecuteContext();

    void WaitContext();

    void ResetContext(bool waitForPrevious);

    // details
    Library();
    ~Library();

    bool Init(const NvFlexInitDesc *desc, NvFlexErrorCallback errorFunc);
    NvResourceTracker *GetResourceTracker();
    NvFlexContext *GetContext();
    Device *GetDevice();

    void ClearBufferInt(NvFlexBuffer *buffer, NvFlexUint sizeInBytes, NvFlexInt val);
    void ClearBufferFloat4(NvFlexBuffer *buffer, NvFlexUint sizeInBytes, NvFlexFloat4 val);

 private:
    friend struct Solver;
    friend struct ShapeData;

    NvFlexErrorCallback mErrorFunc;
    Device *mDevice;
    NvFlexContext *mContext;
    char mDeviceName[256];
    NvFlexComputeShader *mShaderPredict;
    NvFlexComputeShader *mShaderCalculateBounds;
    NvFlexComputeShader *mShaderCalculateBoundsGroup;
    NvFlexComputeShader *mShaderCalculateBoundsFinalize;
    NvFlexComputeShader *mShaderCalculateParticleHash;
    NvFlexComputeShader *mShaderCreateGrid;
    NvFlexComputeShader *mShaderClearInt;
    NvFlexComputeShader *mShaderClearFloat4;
    HUploadBuffer<>     mShaderClearConstantBufferInt;
    HUploadBuffer<>     mShaderClearConstantBufferFloat4;
    NvFlexComputeShader *mShaderClearCellBuckets;
    NvFlexComputeShader *mShaderReorderParticles;
    NvFlexComputeShader *mShaderCollideParticles;
    NvFlexComputeShader *mShaderCollideShapes;
    NvFlexComputeShader *mShaderContinuousShockPropagation;
    NvFlexComputeShader *mShaderCalculateDensity;
    NvFlexComputeShader *mShaderCalculateDensitySurfaceTension;
    NvFlexComputeShader *mShaderSolveDensities;
    NvFlexComputeShader *mShaderSolveDensitiesNonFluid;
    NvFlexComputeShader *mShaderSolveDensitiesSurfaceTension;
    NvFlexComputeShader *mShaderCalculateInflatableVolume;
    NvFlexComputeShader *mShaderSolveInflatableVolume;
    NvFlexComputeShader *mShaderSolveSprings;
    NvFlexComputeShader *mShaderSolveShapes;
    NvFlexComputeShader *mShaderSolveShapes32;
    NvFlexComputeShader *mShaderSolveShapes128;
    NvFlexComputeShader *mShaderSolveShapesPlasticDeformation;
    NvFlexComputeShader *mShaderSolveShapesPlasticDeformation32;
    NvFlexComputeShader *mShaderSolveShapesPlasticDeformation128;
    NvFlexComputeShader *mShaderSolveSpringsXPBD;
    NvFlexComputeShader *mShaderCalculateInflatableVolumeXPBD;
    NvFlexComputeShader *mShaderSolveShapesXPBD;
    NvFlexComputeShader *mShaderSolveShapesPlasticDeformationXPBD;
    NvFlexComputeShader *mShaderApplyDeltas;
    NvFlexComputeShader *mShaderSolveContactsSequential;
    NvFlexComputeShader *mShaderSolveContactsAveraged;
    NvFlexComputeShader *mShaderSolveContactsAccumulate;
    NvFlexComputeShader *mShaderCollideTriangles;
    NvFlexComputeShader *mShaderUpdateVelocities;
    NvFlexComputeShader *mShaderCalculateVorticity;
    NvFlexComputeShader *mShaderSolveVelocities;
    NvFlexComputeShader *mShaderUpdateTriangles;
    NvFlexComputeShader *mShaderUpdateTrianglesInit;
    NvFlexComputeShader *mShaderUpdateVertexNormalsInit;
    NvFlexComputeShader *mShaderUpdateVertexNormals;
    NvFlexComputeShader *mShaderNormalizeVertexNormals;
    NvFlexComputeShader *mShaderFinalize;
    NvFlexComputeShader *mShaderCreateDiffuseParticles;
    NvFlexComputeShader *mShaderUpdateDiffuseParticles;
    NvFlexComputeShader *mShaderClampDiffuseParticleCount;
    NvFlexComputeShader *mShaderCompactDiffuseParticles;
    NvFlexComputeShader *mShaderSmoothPositions;
    NvFlexComputeShader *mShaderCalculateAnisotropy;
    NvFlexComputeShader *mShaderComputeTriangleBounds;
    NvFlexComputeShader *mShaderTransformShapeBounds;
    NvFlexComputeShader *mShaderSpringsGenerateIndices;
    NvFlexComputeShader *mShaderSpringsParticleRange;
    NvFlexComputeShader *mShaderSpringsReorder;
    HStructuredBuffer<NvFlexInt> mCellBucketStarts;
    HStructuredBuffer<NvFlexInt> mCellBucketEnds;
    LinearBVHBuilderGPU *mBVHBuilder;
    HUploadBuffer<NvFlexUint>  mConstTriMeshInfo;
    HStructuredBuffer<NvFlexFloat4> mStaticTriLowers;
    HStructuredBuffer<NvFlexFloat4>  mStaticTriUppers;
    int mMaxStaticTris;
    bool mIsSHFLSupported;
    bool mIsFP32ATOMICSupported;
    void *mAGSContext;
    bool mIsSwizzleSupported;
    bool mEnableAmdDriverBugWorkaround;
    GpuVendorId mGpuVendorId;
    int mSMCount;
    NvFlexFence *mSyncQuery;
    TriangleMeshData *mTriangleMeshData;
    ConvexMeshData *mConvexMeshData;
    HStagingBuffer<struct ConvexMeshDevice> mConvexMeshBoundsReadbackBuffer;
    SDFData *mSDFData;
    NvResourceTracker mResourceTracker;
};

}  // namespace NvFlex

#endif /* FLEXLIBRARY_H */
