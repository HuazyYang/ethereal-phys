// ethereal-phys kernel source. New in ethereal-phys (no FleX origin).
//
// ProbeKernelParams: writes the constant buffers of the kernels, as a kernel sees them, to a buffer. Every
// member is read by name, so the output shows where the HLSL compiler put it. tests/kernel_test.cpp fills
// the buffers with one distinct word per offset and compares the output with offsetof() of the C++ mirror
// (engine/src/kernel_params.h); a packing difference between the two shows up as a wrong word.
//
// The NVFLEX_XPBD=0 permutation reads b0 (KernelParams) and b1 as SubstepParams, the NVFLEX_XPBD=1
// permutation b0 and b1 as KernelParamsXPBD. Output: one uint per scalar, in the order of
// KernelParamsFields.hlsli, b0 first.

#include "KernelParams.hlsli"
#include "KernelParamsFields.hlsli"

RWStructuredBuffer<uint> probe : register(u0);

#define PROBE_SCALAR(name) probe[cursor++] = asuint(PROBE_SOURCE.name);
#define PROBE_FLOAT3(name)                        \
    probe[cursor++] = asuint(PROBE_SOURCE.name.x); \
    probe[cursor++] = asuint(PROBE_SOURCE.name.y); \
    probe[cursor++] = asuint(PROBE_SOURCE.name.z);
#define PROBE_FLOAT4_ARRAY(name, count)                     \
    for (uint element = 0; element < count; ++element) {    \
        probe[cursor++] = asuint(PROBE_SOURCE.name[element].x); \
        probe[cursor++] = asuint(PROBE_SOURCE.name[element].y); \
        probe[cursor++] = asuint(PROBE_SOURCE.name[element].z); \
        probe[cursor++] = asuint(PROBE_SOURCE.name[element].w); \
    }

[numthreads(1, 1, 1)]
void ProbeKernelParams() {
    uint cursor = 0;

#define PROBE_SOURCE gParams
    EPHYS_KERNEL_PARAMS_FIELDS(PROBE_SCALAR, PROBE_FLOAT3, PROBE_FLOAT4_ARRAY)
#undef PROBE_SOURCE

#if NVFLEX_XPBD
#define PROBE_SOURCE gParamsXPBD
    EPHYS_KERNEL_PARAMS_XPBD_FIELDS(PROBE_SCALAR)
#undef PROBE_SOURCE
#else
#define PROBE_SOURCE gSubParams
    EPHYS_SUBSTEP_PARAMS_FIELDS(PROBE_SCALAR)
#undef PROBE_SOURCE
#endif
}
