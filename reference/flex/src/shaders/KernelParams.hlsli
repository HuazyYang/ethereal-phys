#ifndef KERNELPARAMS_HLSLI
#define KERNELPARAMS_HLSLI

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
#if NVFLEX_XPBD
    // XPBD-only fields, register 32. Guarded because UpdateDiffuseParticles and
    // CollideShapes index kPlanes[] dynamically, which makes FXC declare cb0 at
    // the full struct size; an unconditional append moves their bytecode from
    // cb0[32] to cb0[34]. The host uploads the 544-byte struct in every mode.
    int kSolverMode;
    float kInvStiffnessMin;    // 1 / stiffnessMin; alpha = kInvStiffnessMin * exp2(-k * kLogStiffnessRange)
    float kLogStiffnessRange;  // log2(stiffnessMax / stiffnessMin)
    float kSpringDamping;
    float kVolumeCompliance;   // register 33.x
    float3 _padXpbd;
#endif
};

struct SubstepParams {
    float kSubstepStart;
    float kSubstepEnd;
};

struct FlexInflatable {
    int mStartTri;
    int mNumTris;
    float mRestVolume;
    float mConstraintScale;
};

struct FlexConvexMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mPlaneOffset;
    int mPlaneCount;
    int mPad[2];
};

struct FlexSDFDevice {
    int4 mDim;
    float4 mInvDim;
};

struct FlexTriangleMeshDevice {
    float4 mLower;
    float4 mUpper;
    int mIndexStart;
    int mVertexStart;
    int mNodeStart;
    int mPad;
};

/**
 * Flags that control a particle's behavior and grouping, use NvFlexMakePhase() to construct a valid 32bit phase identifier
 */
static const uint eNvFlexPhaseGroupMask = 0x000fffff;           //!< Bits [ 0, 19] represent the particle group for controlling collisions
static const uint eNvFlexPhaseFlagsMask = 0x00f00000;           //!< Bits [20, 23] hold flags about how the particle behave
static const uint eNvFlexPhaseShapeChannelMask = 0x7f000000;    //!< Bits [24, 30] hold flags representing what shape collision channels particles will collide with, see NvFlexMakeShapeFlags() (highest bit reserved for now)
static const uint eNvFlexPhaseSelfCollide = 1 << 20;            //!< If set this particle will interact with particles of the same group
static const uint eNvFlexPhaseSelfCollideFilter = 1 << 21;      //!< If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using NvFlexSetRestParticles()
static const uint eNvFlexPhaseFluid = 1 << 22;                  //!< If set this particle will generate fluid density constraints for its overlapping neighbors
static const uint eNvFlexPhaseUnused = 1 << 23;                 //!< Reserved
static const uint eNvFlexPhaseShapeChannel0 = 1 << 24;          //!< Particle will collide with shapes with channel 0 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel1 = 1 << 25;          //!< Particle will collide with shapes with channel 1 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel2 = 1 << 26;          //!< Particle will collide with shapes with channel 2 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel3 = 1 << 27;          //!< Particle will collide with shapes with channel 3 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel4 = 1 << 28;          //!< Particle will collide with shapes with channel 4 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel5 = 1 << 29;          //!< Particle will collide with shapes with channel 5 set (see NvFlexMakeShapeFlags())
static const uint eNvFlexPhaseShapeChannel6 = 1 << 30; //!< Particle will collide with shapes with channel 6 set (see NvFlexMakeShapeFlags())

static const uint eNvFlexShapeSphere = 0;       //!< A sphere shape, see FlexSphereGeometry
static const uint eNvFlexShapeCapsule = 1;      //!< A capsule shape, see FlexCapsuleGeometry
static const uint eNvFlexShapeBox = 2;          //!< A box shape, see FlexBoxGeometry
static const uint eNvFlexShapeConvexMesh = 3;   //!< A convex mesh shape, see FlexConvexMeshGeometry
static const uint eNvFlexShapeTriangleMesh = 4; //!< A triangle mesh shape, see FlexTriangleMeshGeometry
static const uint eNvFlexShapeSDF = 5;          //!< A signed distance field shape, see FlexSDFGeometry

cbuffer consts: register(b0) {
    KernelParams gParams;
}

cbuffer constSubstep : register(b1) {
    SubstepParams gSubParams;
}

#endif /* KERNELPARAMS_HLSLI */
