#include "radix_sort.h"

#include "kernel_params.h"

#include <utility>

namespace ephys::detail {

namespace {

bool createSortBuffer(nvrhi::IDevice* device, uint32_t elements, const char* name, nvrhi::BufferHandle& out) {
    nvrhi::BufferDesc desc;
    desc.byteSize = uint64_t(elements) * sizeof(uint32_t);
    desc.format = nvrhi::Format::R32_UINT;
    desc.canHaveTypedViews = true;
    desc.canHaveUAVs = true;
    desc.debugName = name;
    desc.enableAutomaticStateTracking(nvrhi::ResourceStates::UnorderedAccess);
    return NVRHI_SUCCEEDED(device->createBuffer(desc, &out)) && out;
}

}  // namespace

nvrhi::FRESULT RadixSort::create(nvrhi::IDevice* device, uint32_t maxKeys) noexcept {
    if (!device || maxKeys == 0) return nvrhi::FE_INVALID_ARGS;

    const uint32_t blocks = (maxKeys + c_KeysPerBlock - 1) / c_KeysPerBlock;
    const uint32_t keyElements = blocks * c_KeysPerBlock;
    // Per block: 16 bucket counters for the global scan and 16 for the scan inside the block.
    const uint32_t counterElements = blocks * 32;

    Pair front, back;
    nvrhi::BufferHandle frontCounters, backCounters;
    if (!createSortBuffer(device, keyElements, "ephys radix sort keys 0", front.keys) ||
        !createSortBuffer(device, keyElements, "ephys radix sort values 0", front.values) ||
        !createSortBuffer(device, keyElements, "ephys radix sort keys 1", back.keys) ||
        !createSortBuffer(device, keyElements, "ephys radix sort values 1", back.values) ||
        !createSortBuffer(device, counterElements, "ephys radix sort counters 0", frontCounters) ||
        !createSortBuffer(device, counterElements, "ephys radix sort counters 1", backCounters))
        return nvrhi::FE_GENERIC_ERROR;

    m_MaxBlocks = blocks;
    m_Front = front;
    m_Back = back;
    m_FrontCounters = frontCounters;
    m_BackCounters = backCounters;
    return nvrhi::FS_OK;
}

nvrhi::FRESULT RadixSort::sort(IPhysKernelLibrary* kernels, nvrhi::ICommandList* commandList, uint32_t numKeys,
                               uint32_t bits) noexcept {
    if (!kernels || m_MaxBlocks == 0 || bits > 32 || bits % 4 != 0) return nvrhi::FE_INVALID_ARGS;
    if (numKeys > getMaxKeys()) return nvrhi::FE_INVALID_ARGS;

    const uint32_t numBlocks = (numKeys + c_KeysPerBlock - 1) / c_KeysPerBlock;
    if (numBlocks == 0) return nvrhi::FS_OK;

    RadixSortParams params;
    params.numKeys = numKeys;

    KernelDispatchDesc dispatch;
    dispatch.rootConstants = &params;
    dispatch.rootConstantsSize = sizeof(params);

    if (numBlocks == 1) {
        // One block: the whole sort in one thread group.
        params.gridDim[0] = params.gridDim[1] = params.gridDim[2] = 1;
        params.numCounters = 16;
        params.passStart = 0;

        dispatch.kernel = KernelId::RadixSortBlockCS;
        dispatch.readOnly[0] = m_Front.keys;
        dispatch.readOnly[1] = m_Front.values;
        dispatch.readWrite[0] = m_Back.keys;
        dispatch.readWrite[1] = m_Back.values;
        const nvrhi::FRESULT result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result)) return result;
        std::swap(m_Front, m_Back);
        return nvrhi::FS_OK;
    }

    // Four bits per pass: count the buckets of every block, scan the counters, scatter.
    for (uint32_t passStart = 0; passStart < bits; passStart += 4) {
        params.gridDim[0] = numBlocks;
        params.gridDim[1] = params.gridDim[2] = 1;
        params.numCounters = 16 * numBlocks;
        params.passStart = passStart;

        dispatch = KernelDispatchDesc();
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);

        dispatch.kernel = KernelId::RadixSort1CS;
        dispatch.gridDim[0] = numBlocks;
        dispatch.readOnly[0] = m_Front.keys;
        dispatch.readWrite[0] = m_FrontCounters;
        nvrhi::FRESULT result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result)) return result;

        // Group 0 scans the global counters, group 1 the per-block ones.
        dispatch.kernel = KernelId::RadixSort2CS;
        dispatch.gridDim[0] = 2;
        dispatch.readOnly[0] = m_FrontCounters;
        dispatch.readWrite[0] = m_BackCounters;
        result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result)) return result;
        std::swap(m_FrontCounters, m_BackCounters);

        dispatch.kernel = KernelId::RadixSort3CS;
        dispatch.gridDim[0] = numBlocks;
        dispatch.readOnly[0] = m_Front.keys;
        dispatch.readOnly[1] = m_Front.values;
        dispatch.readOnly[2] = m_FrontCounters;
        dispatch.readWrite[0] = m_Back.keys;
        dispatch.readWrite[1] = m_Back.values;
        result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result)) return result;
        std::swap(m_Front, m_Back);
    }
    return nvrhi::FS_OK;
}

}  // namespace ephys::detail
