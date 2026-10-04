#ifndef KERNELPARAMSFIELDS_HLSLI
#define KERNELPARAMSFIELDS_HLSLI

// The members of the structs of KernelParamsLayout.hlsli, in declaration order, without the padding, as
// lists of macro calls. Valid in HLSL and C++: the ProbeKernelParams kernel expands them to "read the field
// by name and store it", tests/kernel_test.cpp to "the word at offsetof(field)". A member added to a
// struct is added here as well; the test then covers it.
//
//   SCALAR(name)               an int or a float
//   FLOAT3(name)               a float3
//   FLOAT4_ARRAY(name, count)  an array of float4

#define EPHYS_KERNEL_PARAMS_FIELDS(SCALAR, FLOAT3, FLOAT4_ARRAY)                                            \
    FLOAT4_ARRAY(kPlanes, 8)                                                                                \
    SCALAR(kNumPlanes)                                                                                      \
    SCALAR(kRadius)                                                                                         \
    SCALAR(kRadiusSq)                                                                                       \
    SCALAR(kInvRadius)                                                                                      \
    SCALAR(kPoly6)                                                                                          \
    SCALAR(kSpiky1)                                                                                         \
    SCALAR(kSpiky2)                                                                                         \
    SCALAR(kAkinci1)                                                                                        \
    SCALAR(kAkinci2)                                                                                        \
    SCALAR(kCohesion1)                                                                                      \
    SCALAR(kCohesion2)                                                                                      \
    SCALAR(kRestDensity)                                                                                    \
    SCALAR(kInvRestDensity)                                                                                 \
    SCALAR(kFluidRestDistance)                                                                              \
    SCALAR(kSolidRestDistance)                                                                              \
    SCALAR(kLambdaScale)                                                                                    \
    SCALAR(kCohesion)                                                                                       \
    SCALAR(kSurfaceTension)                                                                                 \
    SCALAR(kSolidPressure)                                                                                  \
    SCALAR(kVorticityConfinement)                                                                           \
    SCALAR(kAnisotropy)                                                                                     \
    SCALAR(kAnisotropyMin)                                                                                  \
    SCALAR(kAnisotropyMax)                                                                                  \
    SCALAR(kSmoothing)                                                                                      \
    SCALAR(kViscosity)                                                                                      \
    SCALAR(kFreeSurfaceDrag)                                                                                \
    SCALAR(kBuoyancy)                                                                                       \
    SCALAR(kDiffuseThreshold)                                                                               \
    SCALAR(kDiffuseBuoyancy)                                                                                \
    SCALAR(kDiffuseDrag)                                                                                    \
    SCALAR(kDiffuseBallistic)                                                                               \
    FLOAT3(kGravity)                                                                                        \
    SCALAR(kDynamicFriction)                                                                                \
    SCALAR(kStaticFriction)                                                                                 \
    SCALAR(kParticleFriction)                                                                               \
    SCALAR(kAdhesion)                                                                                       \
    SCALAR(kDissipation)                                                                                    \
    SCALAR(kDamping)                                                                                        \
    SCALAR(kInertiaBias)                                                                                    \
    SCALAR(kCollisionDistance)                                                                              \
    SCALAR(kCollisionThreshold)                                                                             \
    SCALAR(kCollisionMargin)                                                                                \
    SCALAR(kCollisionMarginSq)                                                                              \
    SCALAR(kDrag)                                                                                           \
    SCALAR(kLift)                                                                                           \
    SCALAR(kPlasticThreshold_deprecated)                                                                    \
    SCALAR(kPlasticCreep_deprecated)                                                                        \
    SCALAR(kSleepThreshold)                                                                                 \
    SCALAR(kSleepThresholdSq)                                                                               \
    SCALAR(kShockPropagation)                                                                               \
    SCALAR(kRestitution)                                                                                    \
    SCALAR(kSOR)                                                                                            \
    SCALAR(kNumParticles)                                                                                   \
    SCALAR(kNumParticlesAligned)                                                                            \
    SCALAR(kDt)                                                                                             \
    SCALAR(kInvDt)                                                                                          \
    SCALAR(kSubStepLength)                                                                                  \
    SCALAR(kMaxSpeed)                                                                                       \
    SCALAR(kInvCellEdge)                                                                                    \
    SCALAR(kStaticTriGridInvCellEdge)                                                                       \
    SCALAR(kCollideParticlesRadiusSq)                                                                       \
    SCALAR(kMaxParticles)                                                                                   \
    SCALAR(kRelaxationMode)                                                                                 \
    SCALAR(kShapesCount)                                                                                    \
    SCALAR(kNumShapes)                                                                                      \
    SCALAR(kNumRigids)                                                                                      \
    SCALAR(kNumSprings)                                                                                     \
    SCALAR(kNumTriangles)                                                                                   \
    SCALAR(kMaxDiffuseParticles)                                                                            \
    SCALAR(kDiffuseRadixSortSize)                                                                           \
    SCALAR(kDiffuseDt)                                                                                      \
    SCALAR(kDiffuseMaxVelocity)                                                                             \
    FLOAT3(kDiffuseSortAxis)                                                                                \
    SCALAR(kRelaxationFactor)                                                                               \
    FLOAT3(kStaticTriGridLower)                                                                             \
    SCALAR(kDiffuseLifetime)                                                                                \
    FLOAT3(kWind)                                                                                           \
    SCALAR(kMaxNeighborsPerParticle)                                                                        \
    SCALAR(kNumGeometryEntries)                                                                             \
    SCALAR(kMaxAcceleration)                                                                                \
    SCALAR(kMaxVelocityDelta)                                                                               \
    SCALAR(kMaxContactsPerParticle)

#define EPHYS_KERNEL_PARAMS_XPBD_FIELDS(SCALAR)                                                             \
    SCALAR(kInvStiffnessMin)                                                                                \
    SCALAR(kLogStiffnessRange)                                                                              \
    SCALAR(kSpringDamping)                                                                                  \
    SCALAR(kVolumeCompliance)

#define EPHYS_SUBSTEP_PARAMS_FIELDS(SCALAR)                                                                 \
    SCALAR(kSubstepStart)                                                                                   \
    SCALAR(kSubstepEnd)

#endif /* KERNELPARAMSFIELDS_HLSLI */
