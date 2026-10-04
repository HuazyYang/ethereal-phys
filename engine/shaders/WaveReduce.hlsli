#ifndef WAVEREDUCE_HLSLI
#define WAVEREDUCE_HLSLI

// Thread-group reductions with the SM 6.0 wave intrinsics. New in ethereal-phys: it replaces the NVAPI
// (NvShflDown) and AGS (AmdDxExtShaderIntrinsics_SwizzleF) reductions of the FleX *NV / *AMD kernels with
// one portable form, used by the EPHYS_WAVE=1 permutations.
//
//   EPHYS_DEFINE_WAVE_GROUP_REDUCE(Name, Type, GroupSize, WaveOp, Combine, Identity)
//
// defines   Type Name(uint threadIdx, Type value)   which returns, in every thread of the group, the
// reduction of `value` over all GroupSize threads. WaveOp is the wave intrinsic (WaveActiveSum,
// WaveActiveMin, WaveActiveMax), Combine(a, b) the matching binary operation and Identity its neutral
// element. It costs two group barriers where the groupshared tree costs log2(GroupSize) + 1.
//
// Name must be called from control flow that is uniform across the thread group (it contains
// GroupMemoryBarrierWithGroupSync), with threadIdx = SV_GroupThreadID.x and GroupSize = the numthreads
// product. Calls may follow one another without a barrier in between.
//
// The FleX vendor kernels assumed a wave of exactly 32 lanes holding threads 32k .. 32k+31 in lane order.
// Nothing guarantees that mapping, so this form assumes nothing about the lane count or about which
// threads share a wave. What it relies on, and why the result is the full reduction regardless:
//
//  1. Two wave intrinsics in one basic block see the same set of active lanes. So `waveValue` and
//     `waveLeader` below describe the same set of threads, and so do `activeLanes` and `reduced`.
//  2. Step 1 partitions the group into such sets. The smallest thread index of a set identifies it, and
//     exactly one thread of the set has it. That thread stores the reduction of its set; every other
//     thread stores the identity. The reduction of all GroupSize slots is therefore the group result,
//     for any partition (this also holds if a driver splits a wave after non-uniform branches).
//  3. Step 2 reduces the slots. The threads of the set that holds thread 0 each fold the slots of one
//     residue class modulo the lane count; when that set has laneCount active lanes (lane indices are
//     distinct and below the lane count, so every class is covered exactly once) and the lane count
//     divides GroupSize, the wave reduction of those partial results is the group result. Otherwise
//     thread 0 folds all slots itself. Either way thread 0 publishes the result.
//
// A sum is evaluated in a different order than the groupshared tree of the EPHYS_WAVE=0 permutation (and
// the order depends on the lane count of the device), so float sums differ from it in the last bits;
// on one device the order is fixed and the result is reproducible. Min and max are exact.

// EPHYS_WAVE is the permutation axis of the manifest (ethereal_phys.cfg). The DXBC (cs_5_0, D3D11) build has
// no wave intrinsics: there both permutations are the groupshared tree, and the kernel library never
// selects the wave permutation on a device without nvrhi::Feature::WaveLaneCountMinMax anyway.
#if EPHYS_WAVE && !TARGET_D3D11
#define EPHYS_USE_WAVE 1
#else
#define EPHYS_USE_WAVE 0
#endif

#if EPHYS_USE_WAVE

#define EPHYS_WAVE_ADD(a, b) ((a) + (b))

#define EPHYS_DEFINE_WAVE_GROUP_REDUCE(Name, Type, GroupSize, WaveOp, Combine, Identity)                    \
    groupshared Type Name##_slots[GroupSize];                                                               \
    groupshared Type Name##_total;                                                                          \
    Type Name(uint threadIdx, Type value) {                                                                 \
        const Type identity = Identity;                                                                     \
        /* Step 1: one slot per wave, at the smallest thread index of the wave. */                          \
        Type waveValue = WaveOp(value);                                                                     \
        uint waveLeader = WaveActiveMin(threadIdx);                                                         \
        if (threadIdx == waveLeader)                                                                        \
            Name##_slots[threadIdx] = waveValue;                                                            \
        else                                                                                                \
            Name##_slots[threadIdx] = identity;                                                             \
        GroupMemoryBarrierWithGroupSync();                                                                  \
        /* Step 2: the wave of thread 0 reduces the slots. */                                               \
        if (waveLeader == 0) {                                                                              \
            uint laneCount = WaveGetLaneCount();                                                            \
            uint laneIndex = WaveGetLaneIndex();                                                            \
            uint strides = uint(GroupSize) / laneCount;                                                     \
            Type partial = identity;                                                                        \
            for (uint s = 0; s < strides; ++s)                                                              \
                partial = Combine(partial, Name##_slots[laneIndex + s * laneCount]);                        \
            uint activeLanes = WaveActiveSum(1u);                                                           \
            Type reduced = WaveOp(partial);                                                                 \
            if (threadIdx == 0) {                                                                           \
                if (activeLanes != laneCount || strides * laneCount != uint(GroupSize)) {                   \
                    reduced = identity;                                                                     \
                    for (uint k = 0; k < uint(GroupSize); ++k)                                              \
                        reduced = Combine(reduced, Name##_slots[k]);                                        \
                }                                                                                           \
                Name##_total = reduced;                                                                     \
            }                                                                                               \
        }                                                                                                   \
        GroupMemoryBarrierWithGroupSync();                                                                  \
        return Name##_total;                                                                                \
    }

#endif /* EPHYS_USE_WAVE */

#endif /* WAVEREDUCE_HLSLI */
