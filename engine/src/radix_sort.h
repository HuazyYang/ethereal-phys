#ifndef ETHEREAL_PHYS_SRC_RADIX_SORT_H
#define ETHEREAL_PHYS_SRC_RADIX_SORT_H

// Module-private: the GPU radix sort of key/value pairs, the driver of the radixSort* kernels.
//
// It is FleX's RadixSortImpl (reference/flex/src/RadixSort.cpp) on IPhysKernelLibrary. The solver sorts the
// particle cell hashes and the spring indices with it, the BVH builder the Morton codes. It is written
// against the public interfaces only (kernels.h, nvrhi), so tests/kernel_test.cpp compiles this file into
// the test and runs the same code the engine runs.
//
// Usage, as in FleX: write the keys and the values into getKeys() / getValues() (a kernel writes them
// through a typed R32_UINT UAV, e.g. CalculateParticleHash), call sort(), then read the sorted pairs from
// getKeys() / getValues() again. The two calls return other buffers after a sort than before it: the sort
// ping-pongs between two pairs and the "front" pair is the current one.
//
// The buffers are typed (nvrhi::Format::R32_UINT, canHaveTypedViews): the kernels read four keys at a time
// through an RGBA32_UINT view of the same buffer.

#include <ethereal/phys/kernels.h>

#include <nvrhi/nvrhi.h>

#include <cstdint>

namespace ephys::detail {

class RadixSort {
 public:
    // The keys of one block: one thread group of 256 threads sorts four keys per thread.
    static constexpr uint32_t c_KeysPerBlock = 1024;

    // Creates the buffers for up to maxKeys pairs (rounded up to whole blocks). FE_INVALID_ARGS for zero
    // keys, FE_GENERIC_ERROR when a buffer cannot be created.
    nvrhi::FRESULT create(nvrhi::IDevice* device, uint32_t maxKeys) noexcept;

    [[nodiscard]] uint32_t getMaxKeys() const noexcept { return m_MaxBlocks * c_KeysPerBlock; }

    // The current pair of buffers. They do not AddRef.
    [[nodiscard]] nvrhi::IBuffer* getKeys() const noexcept { return m_Front.keys; }
    [[nodiscard]] nvrhi::IBuffer* getValues() const noexcept { return m_Front.values; }

    // Records the sort of the first numKeys pairs by the low `bits` bits of the key (a multiple of 4; FleX
    // sorts cell hashes with 24 and everything else with 32). The sort is stable. Elements from numKeys up
    // to the end of the last block are sorted as if their key were 0xFFFFFFFF and end up behind the
    // numKeys pairs, with whatever values the buffer held there.
    //
    // As in FleX, numKeys <= 1024 runs the single-block kernel, which always sorts all 32 bits.
    //
    // commandList: as IPhysKernelLibrary::dispatch. Returns the first failing code of the dispatches.
    nvrhi::FRESULT sort(IPhysKernelLibrary* kernels, nvrhi::ICommandList* commandList, uint32_t numKeys,
                        uint32_t bits) noexcept;

 private:
    struct Pair {
        nvrhi::BufferHandle keys;
        nvrhi::BufferHandle values;
    };

    uint32_t m_MaxBlocks = 0;
    Pair m_Front;
    Pair m_Back;
    nvrhi::BufferHandle m_FrontCounters;
    nvrhi::BufferHandle m_BackCounters;
};

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_RADIX_SORT_H */
