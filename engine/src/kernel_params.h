#ifndef ETHEREAL_PHYS_SRC_KERNEL_PARAMS_H
#define ETHEREAL_PHYS_SRC_KERNEL_PARAMS_H

// Module-private: the host side of the constant buffers of the kernels.
//
// KernelParams, KernelParamsXPBD and SubstepParams are not written twice: the struct definitions of
// engine/shaders/KernelParamsLayout.hlsli are valid C++ once float3 and float4 exist, and are included
// here. What this header adds is the proof that the C++ compiler lays them out as HLSL packs a cbuffer: all
// padding in those structs is explicit, and every member that follows a padded region or a vector has its
// offset asserted. The other half of the proof runs on the GPU (tests/kernel_test.cpp, ProbeKernelParams).
//
// The small blocks of the kernels that do not use KernelParams (the clears, the radix sort, the BVH build)
// are mirrored by hand below; their cbuffers are a few scalars each.
//
// No alignas and no #pragma pack: every member is a 4-byte scalar or an aggregate of them.

#include <cstddef>
#include <cstdint>

namespace ephys::detail {

struct float3 {
    float x, y, z;
};
struct float4 {
    float x, y, z, w;
};

static_assert(sizeof(int) == 4 && sizeof(float) == 4);
static_assert(sizeof(float3) == 12 && alignof(float3) == 4);
static_assert(sizeof(float4) == 16 && alignof(float4) == 4);

#include "KernelParamsLayout.hlsli"

// b0: 32 registers, the layout FleX shipped.
static_assert(sizeof(KernelParams) == 512, "KernelParams must stay 32 registers (KernelParamsLayout.hlsli)");
static_assert(alignof(KernelParams) == 4);
static_assert(offsetof(KernelParams, kNumPlanes) == 128);        // after kPlanes[8]
static_assert(offsetof(KernelParams, _pad0) == 172);
static_assert(offsetof(KernelParams, kRestDensity) == 176);      // after _pad0: register 11
static_assert(offsetof(KernelParams, kDiffuseBallistic) == 252);
static_assert(offsetof(KernelParams, kGravity) == 256);          // a float3 at the start of register 16
static_assert(offsetof(KernelParams, kDynamicFriction) == 268);  // fills the register of kGravity
static_assert(offsetof(KernelParams, kStaticFriction) == 272);
static_assert(offsetof(KernelParams, kCollisionMarginSq) == 308);
static_assert(offsetof(KernelParams, _pad1) == 312);
static_assert(offsetof(KernelParams, _pad2) == 316);
static_assert(offsetof(KernelParams, kDrag) == 320);             // after _pad1, _pad2: register 20
static_assert(offsetof(KernelParams, kSOR) == 352);
static_assert(offsetof(KernelParams, _pad3) == 356);             // a float3 that fills register 22
static_assert(offsetof(KernelParams, kNumParticles) == 368);     // after _pad3: register 23
static_assert(offsetof(KernelParams, kDiffuseMaxVelocity) == 444);
static_assert(offsetof(KernelParams, kDiffuseSortAxis) == 448);  // a float3 at the start of register 28
static_assert(offsetof(KernelParams, kRelaxationFactor) == 460);
static_assert(offsetof(KernelParams, kStaticTriGridLower) == 464);  // register 29
static_assert(offsetof(KernelParams, kDiffuseLifetime) == 476);
static_assert(offsetof(KernelParams, kWind) == 480);             // register 30
static_assert(offsetof(KernelParams, kMaxNeighborsPerParticle) == 492);
static_assert(offsetof(KernelParams, kNumGeometryEntries) == 496);  // register 31
static_assert(offsetof(KernelParams, kMaxContactsPerParticle) == 508);

// b1 of the XPBD kernels.
static_assert(sizeof(KernelParamsXPBD) == 16);
static_assert(offsetof(KernelParamsXPBD, kVolumeCompliance) == 12);

// b1 of CollideShapes and CollideTriangles.
static_assert(sizeof(SubstepParams) == 16);
static_assert(offsetof(SubstepParams, kSubstepEnd) == 4);
static_assert(offsetof(SubstepParams, _padSubstep0) == 8);

// b0 of ClearInt (ClearInt.hlsl). The kernel compares the thread index with length as it is: an element
// count.
struct ClearIntParams {
    int32_t length = 0;
    int32_t value = 0;
    int32_t _pad[2] = {0, 0};
};
static_assert(sizeof(ClearIntParams) == 16);

// b0 of ClearFloat4 (ClearFloat4.hlsl).
struct ClearFloat4Params {
    int32_t length = 0;
    float _pad[3] = {0.0f, 0.0f, 0.0f};
    float value[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};
static_assert(sizeof(ClearFloat4Params) == 32);
static_assert(offsetof(ClearFloat4Params, value) == 16);  // a float4 starts a register

// b0 of the radix sort kernels (radixSort*.hlsl: "params").
struct RadixSortParams {
    uint32_t gridDim[3] = {0, 0, 0};
    uint32_t passStart = 0;
    uint32_t numCounters = 0;
    uint32_t numKeys = 0;
    uint32_t _pad[2] = {0, 0};
};
static_assert(sizeof(RadixSortParams) == 32);
static_assert(offsetof(RadixSortParams, passStart) == 12);  // fills the register of gridDim
static_assert(offsetof(RadixSortParams, numCounters) == 16);

// b0 of the BVH kernels (bvh/*.hlsl: "constBVH").
struct BvhParams {
    int32_t numItems = 0;
    int32_t _pad[3] = {0, 0, 0};
};
static_assert(sizeof(BvhParams) == 16);

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_KERNEL_PARAMS_H */
