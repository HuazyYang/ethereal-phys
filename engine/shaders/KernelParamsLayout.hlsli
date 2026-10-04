// ethereal-phys kernel source. Origin: FleX src/shaders/KernelParams.hlsli at commit 2eed78e (branch feat-xpbd).
// Changes made by the SM 6.0 port are recorded in engine/shaders/PORTING.md.

#ifndef KERNELPARAMSLAYOUT_HLSLI
#define KERNELPARAMSLAYOUT_HLSLI

// The constant-buffer structs of the kernels, shared by HLSL and C++: this file is included by
// KernelParams.hlsli and, inside namespace ephys::detail, by engine/src/kernel_params.h, which supplies
// float3 / float4 as plain float aggregates and asserts the size and the offset of every padded region.
// It must stay valid in both languages: struct definitions with int, float, float3, float4 members only,
// no initialisers, no semantics, no preprocessor conditionals that change the layout.
//
// All padding is explicit. HLSL packs a cbuffer in 16-byte registers and starts a vector that would
// straddle a register on the next one; the _pad members put every vector where that rule and the C++
// compiler (4-byte alignment throughout) agree. tests/kernel_test.cpp proves it on the GPU: the
// ProbeKernelParams kernel reads every field by name and the test compares against offsetof().

// b0 of every kernel that includes KernelParams.hlsli. 512 bytes, 32 registers: FleX's shipped layout.
// The XPBD fields that feat-xpbd appended in registers 32 and 33 are KernelParamsXPBD below, and
// kSolverMode is gone: PBD and XPBD are separate kernels (permutations), nothing branches on a mode.
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
};

// b1 of the XPBD kernels (SolveSpringsXPBD and the NVFLEX_XPBD permutations of CalculateInflatableVolume,
// SolveShapes and SolveShapesPlasticDeformation). 16 bytes, one register.
struct KernelParamsXPBD {
    float kInvStiffnessMin;    // 1 / stiffnessMin; alpha = kInvStiffnessMin * exp2(-k * kLogStiffnessRange)
    float kLogStiffnessRange;  // log2(stiffnessMax / stiffnessMin)
    float kSpringDamping;
    float kVolumeCompliance;
};

// b1 of CollideShapes and CollideTriangles. 16 bytes, one register.
struct SubstepParams {
    float kSubstepStart;
    float kSubstepEnd;
    float _padSubstep0;
    float _padSubstep1;
};

#endif /* KERNELPARAMSLAYOUT_HLSLI */
