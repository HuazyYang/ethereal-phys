#ifndef FLEXSOLVER_H
#define FLEXSOLVER_H
#include "Types.h"
#include "NvFlexImpl.h"
#include "Object.h"
#include <nvflex/NvFlexContextExt.h>
#include "Data.h"
#include "TimerPool.h"
#include "ClientHelper.h"
#include "ResourceWrapper.h"
#include <cstddef>

namespace NvFlex {

struct Library;
struct RadixSort;
struct TimerPool;

struct KernelParams {
    float4 kPlanes[8];
    int kNumPlanes;
    float kRadius;
    float kRadiusSq;
    float kInvRadius;
    float kPoly6;
    float kSpiky1;
    float kSpiky2;
    float kAkinci1;
    float kAkinci2;
    float kCohesion1;
    float kCohesion2;
    float _pad0;
    float kRestDensity;
    float kInvRestDensity;
    float kFluidRestDistance;
    float kSolidRestDistance;
    float kLambdaScale;
    float kCohesion;
    float kSurfaceTension;
    float kSolidPressure;
    float kVorticityConfinement;
    float kAnisotropy;
    float kAnisotropyMin;
    float kAnisotropyMax;
    float kSmoothing;
    float kViscosity;
    float kFreeSurfaceDrag;
    float kBuoyancy;
    float kDiffuseThreshold;
    float kDiffuseBuoyancy;
    float kDiffuseDrag;
    int kDiffuseBallistic;
    float3 kGravity;
    float kDynamicFriction;
    float kStaticFriction;
    float kParticleFriction;
    float kAdhesion;
    float kDissipation;
    float kDamping;
    float kInertiaBias;
    float kCollisionDistance;
    float kCollisionThreshold;
    float kCollisionMargin;
    float kCollisionMarginSq;
    float _pad1;
    float _pad2;
    float kDrag;
    float kLift;
    float kPlasticThreshold_deprecated;
    float kPlasticCreep_deprecated;
    float kSleepThreshold;
    float kSleepThresholdSq;
    float kShockPropagation;
    float kRestitution;
    float kSOR;
    float3 _pad3;
    int kNumParticles;
    int kNumParticlesAligned;
    float kDt;
    float kInvDt;
    float kSubStepLength;
    float kMaxSpeed;
    float kInvCellEdge;
    float kStaticTriGridInvCellEdge;
    float kCollideParticlesRadiusSq;
    int kMaxParticles;
    int kRelaxationMode;
    int kShapesCount;
    int kNumShapes;
    int kNumRigids;
    int kNumSprings;
    int kNumTriangles;
    int kMaxDiffuseParticles;
    int kDiffuseRadixSortSize;
    float kDiffuseDt;
    float kDiffuseMaxVelocity;
    float3 kDiffuseSortAxis;
    float kRelaxationFactor;
    float3 kStaticTriGridLower;
    float kDiffuseLifetime;
    float3 kWind;
    int kMaxNeighborsPerParticle;
    int kNumGeometryEntries;
    float kMaxAcceleration;
    float kMaxVelocityDelta;
    int kMaxContactsPerParticle;
    int kSolverMode;
    float kInvStiffnessMin;    // 1 / stiffnessMin
    float kLogStiffnessRange;  // log2(stiffnessMax / stiffnessMin)
    float kSpringDamping;
    float kVolumeCompliance;
    float _padXpbd[3];         // constant buffers are sized in 16-byte registers
};
static_assert(sizeof(KernelParams) == 544,
              "KernelParams must stay 34 registers; see KernelParams.hlsli");
static_assert(offsetof(KernelParams, kSolverMode) == 512 &&
                  offsetof(KernelParams, kVolumeCompliance) == 528,
              "the XPBD fields must sit in registers 32 and 33; see KernelParams.hlsli");

struct Solver : Object, NvFlexSolver {
    NVFLEX_IMPLEMENT_OBJECT_REFERENCE()
    uint64_t getGPUBytesUsed() override { return 0; }

    NvFlexLibrary* GetLibrary();
    NvFlexSolverDesc* GetDesc() { return &mDesc; };
    NvFlexSolverCallback RegisterCallback(NvFlexSolverCallbackStage stage,
                                          const NvFlexSolverCallback* callback);
    void Update(float dt, int nubSubsteps, bool enableTimers);
    void SetParams(const NvFlexParams *params);
    const NvFlexParams& GetParams();
    void SetActive(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc *copyDesc);
    void GetActive(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetActiveCount(int n);
    int GetActiveCount();
    void SetParticles(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc* copyDesc);
    void GetParticles(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetRestParticles(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc* copyDesc);
    void GetRestParticles(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void GetSmoothParticles(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetVelocities(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc* copyDesc);
    void GetVelocities(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetPhases(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc* copyDesc);
    void GetPhases(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetNormals(NvFlexBuffer* srcBuffer, const NvFlexCopyDesc* copyDesc);
    void GetNormals(NvFlexBuffer* dstBuffer, const NvFlexCopyDesc* copyDesc);
    void SetSprings(NvFlexBuffer* indices, NvFlexBuffer *restLengths, NvFlexBuffer *stiffness, int numSprings);
    void SetRigids(NvFlexBuffer* offsets, NvFlexBuffer* indices,
                   NvFlexBuffer* restPositions, NvFlexBuffer* restNormals,
                   NvFlexBuffer* stiffness, NvFlexBuffer* thresholds, NvFlexBuffer* creeps,
                   NvFlexBuffer* rotations, NvFlexBuffer* translations, int numRigids,
                   int numIndices);
    void GetRigids(NvFlexBuffer* offsets, NvFlexBuffer* indices,
                   NvFlexBuffer* restPositions, NvFlexBuffer* restNormals,
                   NvFlexBuffer* stiffness, NvFlexBuffer* thresholds, NvFlexBuffer* creeps,
                   NvFlexBuffer* rotations, NvFlexBuffer* translations);

    void SetShapes(NvFlexBuffer* geometry, NvFlexBuffer* shapePositions,
                   NvFlexBuffer* shapeRotations, NvFlexBuffer* prevShapePositions,
                   NvFlexBuffer* prevShapeRotations, NvFlexBuffer* shapeFlags,
                   int numShapes);

    void SetDynamicTriangles(NvFlexBuffer* triangles, NvFlexBuffer* normals,
                             NvFlexUint numTris);

    void GetDynamicTriangles(NvFlexBuffer* triangles, NvFlexBuffer* normals,
                             NvFlexUint numTris);

    void SetInflatables(NvFlexBuffer* startTris, NvFlexBuffer* numTris,
                        NvFlexBuffer* restVolumes, NvFlexBuffer* overPressures,
                        NvFlexBuffer* constraintScales, NvFlexUint n);

    void GetDensities(NvFlexBuffer* d, const NvFlexCopyDesc* desc);

    void GetAnisotropy(NvFlexBuffer* q1, NvFlexBuffer* q2, NvFlexBuffer* q3,
                       const NvFlexCopyDesc* desc);

    void GetDiffuseParticles(NvFlexBuffer* p, NvFlexBuffer* v, NvFlexBuffer* count);

    void SetDiffuseParticles(NvFlexBuffer* p, NvFlexBuffer* v, NvFlexUint n);

    void GetContacts(NvFlexBuffer* planes, NvFlexBuffer* velocities, NvFlexBuffer* indices,
                     NvFlexBuffer* counts);

    void GetNeighbors(NvFlexBuffer* neighbors, NvFlexBuffer* counts,
                      NvFlexBuffer* apiToInternal, NvFlexBuffer* internalToApi);

    void GetBounds(NvFlexBuffer* lower, NvFlexBuffer* upper);

    float GetDeviceLatency(NvFlexUint64* gpuStartStamp, NvFlexUint64* gpuEndStamp,
                          NvFlexUint64* gpuFreq);

    void GetTimers(NvFlexTimers* timers);

    NvFlexUint GetDetailTimers(NvFlexDetailTimer** timers);

    // Details
    Solver(NvFlexLibrary* lib, const NvFlexSolverDesc* desc);
    ~Solver();

    bool Init();

private:
   struct IterationState {
       float dt;
       float dta;
       NvFlexUint substepIdx;
       NvFlexUint numSubsteps;
       NvFlexUint kNumBlocks;
       NvFlexUint kNumBlocksHalf;
   };

   void InitParams(const IterationState &state);
   void UpdateSubstep(const IterationState &state);

   void Predict(const IterationState &);
   void ComputeBounds(const IterationState &);
   void CalculateParticleHash(const IterationState &);
   void SortParticleHash(const IterationState &);
   void CreateGrid(const IterationState &);
   void RecorderParticles(const IterationState &);
   void CollideParticles(const IterationState &);
   void CollideTriangles(const IterationState &);
   void CollideShapes(const IterationState &);
   void ContinuousShockPropagation(const IterationState &);
   void CalculateDensity(const IterationState &);
   void SolveDensity(const IterationState &);
   void SolveSprings(const IterationState &);
   void CalculateAndSolveInflatables(const IterationState &);
   void SolveShapes(const IterationState &);
   void ApplyDeltas(const IterationState &);
   void SolveContact(const IterationState &);
   void UpdateVelocities(const IterationState &);
   void CalculateVorticity(const IterationState &);
   void SolveVelocities(const IterationState &);
   void UpdateTriangles(const IterationState &);
   void UpdateNormals(const IterationState &);
   void NormalizeNormals(const IterationState &);
   void Finalize(const IterationState &);

   void CreateDiffuse(const IterationState &);
   void UpdateDiffuse(const IterationState &);
   void ClampDiffuse(const IterationState &);
   void CompactDiffuse(const IterationState &);
   void SmoothPositions(const IterationState &);
   void CalculateAnisotropy(const IterationState &);
   void LazyClearGrid(const IterationState &);

   void ExecuteCallback(NvFlexSolverCallbackStage stage, float dt);

   void CopyBufferImpl(NvFlexBuffer* dstBuffer, NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc);
   void CopyBufferImpl(NvFlexBuffer* dstBuffer, NvFlexUint dstElemOffset, NvFlexBuffer* srcBuffer, NvFlexUint srcElemOffset, NvFlexUint numElements);

   void SpringGenerateIndices(NvFlexBuffer* indices, int numSprings);
   void SpringFindParticleRange(NvFlexBuffer* sortedIndices, int numSprings);
   void SpringRecordData(NvFlexBuffer* sortedIndices, int numSprings);

   Library* mLib;
   KernelParams mKernelParamsHost;
   HUploadBuffer<KernelParams> mKernelParams;
   HStructuredBuffer<float4> mPositions;
   HStructuredBuffer<float4> mNewPositions;
   HStructuredBuffer<float3> mVelocities;
   HStructuredBuffer<float4> mRestPositions;
   HRawBuffer<TypelessFormat_r32> mDeltas;
   HStructuredBuffer<NvFlexPhase> mPhases;
   HStructuredBuffer<int> mActiveIndices;
   HStructuredBuffer<float> mDensities;
   HStructuredBuffer<float> mPotentials;
   HStructuredBuffer<float4> mCurl;
   HStructuredBuffer<float4> mNormals;
   HStructuredBuffer<float4> mSortedNormals;
   HRawBuffer<TypelessFormat_r32> mNormalsTemp;
   HStructuredBuffer<float4> mAnisotropy1;
   HStructuredBuffer<float4> mAnisotropy2;
   HStructuredBuffer<float4> mAnisotropy3;
   HStructuredBuffer<float4> mSortedPositions;
   HStructuredBuffer<float4> mSortedVelocities;
   HStructuredBuffer<float4> mSortedNewPositions;
   HStructuredBuffer<float4> mSortedNewVelocities;
   HStructuredBuffer<NvFlexPhase> mSortedPhases;
   HStructuredBuffer<float> mSortedDensities;
   HStructuredBuffer<float4> mStaticContactPlanes;
   HStructuredBuffer<float4> mStaticContactVelocities;
   HStructuredBuffer<int> mStaticContactCounts;
   int mMaxNeighborsPerParticle;
   HStructuredBuffer<int> mParticleNeighbors;
   HStructuredBuffer<int> mParticleNeighborCounts;
   int mMaxContactsPerParticle;
   HStructuredBuffer<float4> mPlanes;
   HStructuredBuffer<int> mSpringIndices;
   HStructuredBuffer<float> mSpringLengths;
   HStructuredBuffer<float> mSpringStiffness;
   int mNumSprings;
   int mMaxSprings;
   HStructuredBuffer<int> mHalfSpringIndices;
   HStructuredBuffer<int> mHalfSpringOpposites;
   HStructuredBuffer<float> mHalfSpringLengths;
   HStructuredBuffer<float> mHalfSpringStiffness;
   HStructuredBuffer<int> mHalfSpringParticleBegin;
   HStructuredBuffer<int> mHalfSpringParticleEnd;
   HStructuredBuffer<float> mHalfSpringLambdas;
   HUploadBuffer<> mHalfSpringConstantBuffer;
   int mMaxHalfSprings;
   HStructuredBuffer<int> mRigidOffsets;
   HStructuredBuffer<int> mRigidIndices;
   HStructuredBuffer<float3> mRigidLocalPositions;
   HStructuredBuffer<float4> mRigidLocalNormals;
   HStructuredBuffer<float> mRigidCoefficients;
   HStructuredBuffer<float> mRigidPlasticThresholds;
   HStructuredBuffer<float> mRigidPlasticCreeps;
   HStructuredBuffer<float4> mRigidRotations;
   HStructuredBuffer<float3> mRigidTranslations;
   int mNumRigids;
   int mMaxRigids;
   int mNumRigidIndices;
   int mMaxRigidIndices;
   int mMaxPlasticEntries;
   HStructuredBuffer<int> mDynamicTriangles;
   HStructuredBuffer<float3> mDynamicTriangleNormals;
   int mDynamicNumTris;
   int mDynamicMaxTris;
   HStructuredBuffer<int> mDynamicTriangleEdges;
   HStructuredBuffer<int> mDynamicNumTriEdges;
   HStructuredBuffer<int> mPressures;
   HStructuredBuffer<float> mInflatableLambdas;
   HStructuredBuffer<InflatableDevice> mInflatables;
   int mNumInflatables;
   int mMaxInflatables;
   HStructuredBuffer<float4> mDiffusePositions;
   HStructuredBuffer<float4> mDiffuseVelocities;
   HStructuredBuffer<float4> mDiffusePositionsNew;
   HStructuredBuffer<float4> mDiffuseVelocitiesNew;
   HStructuredBuffer<int> mNumDiffuseParticles;
   HStructuredBuffer<int> mNumDiffuseParticlesNew;
   int mMaxDiffuseParticles;
   int mNumActiveDiffuseParticles;
   HStructuredBuffer<int> mDiffuseSortSize;
   NvFlexBuffer* mDiffuseLaunchArg;
   float mDiffuseRate;
   float mDiffuseTimer;
   HStructuredBuffer<int> mCellIds;
   HStructuredBuffer<int> mReverseLookup;
   int mNumParticles;
   int mNumParticlesAligned;
   int mMaxParticles;
   HStructuredBuffer<float3> mParticleBounds;
   HStructuredBuffer<float4> mBoundsLower;
   HStructuredBuffer<float4> mBoundsUpper;
   HStructuredBuffer<float4> mGroupBoundsLower;
   HStructuredBuffer<float4> mGroupBoundsUpper;
   NvFlexSolverDesc mDesc;
   NvFlexParams mParams;
   NvFlexBuffer* mSDFShapeDX;
   VectorCached<SDFData*> mSDFDeviceShapes;
   ShapeData* mShapes;
   bool mDebug;
   HStructuredBuffer<float> mRandom;
   float mFluidRestDensity;
   float mFluidDensityConstraintScale;
   float mFluidSurfaceConstraintScale;
   int mStepCounter;
   NvFlexSolverCallback mCallbacks[eNvFlexStageCount];
   HStructuredBuffer<float4> mSmoothPositionsOriginal;
   int mRadixSortSize;
   RadixSort* mRadixSort;
   HUploadBuffer<> mSubstepParams;
   HStructuredBuffer<float> mTestFloatResult;
   HStructuredBuffer<float4> mTestFloat4Result;
   HStructuredBuffer<int> mTestIntResult;
   TimerPool* mTimerPool;
   int mNumDetailTimers;
   NvFlexDetailTimer* mDetailTimers;
   int mLatencyTimerRecordStatus;
   bool mLatencyTimerEnable;
   int mLatencyTimerIndex;
   IterationState mIterState;
};

}

#endif /* FLEXSOLVER_H */
