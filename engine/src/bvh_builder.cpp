#include "bvh_builder.h"

#include "data_model.h"
#include "kernel_params.h"

namespace ephys::detail {

namespace {

constexpr uint32_t divCeil(uint32_t n, uint32_t d) { return (n + d - 1) / d; }

bool createStructured(IPhysDevice* device, const char* name, uint32_t stride, uint32_t count,
                      PhysBufferHandle& out) {
    PhysBufferDesc desc;
    desc.elementCount = count;
    desc.elementStride = stride;
    desc.use = BufferUse::Structured;
    desc.debugName = name;
    PhysBufferHandle buffer;
    if (NVRHI_FAILED(device->createBuffer(desc, &buffer)) || !buffer)
        return false;
    out = buffer;
    return true;
}

}  // namespace

nvrhi::FRESULT BvhBuilder::reserve(IPhysDevice* device, uint32_t numItems) noexcept {
    if (!m_TotalLower) {
        if (!createStructured(device, "ephys bvh total lower", 12, 1, m_TotalLower) ||
            !createStructured(device, "ephys bvh total upper", 12, 1, m_TotalUpper) ||
            !createStructured(device, "ephys bvh total inverse edges", 12, 1, m_TotalInvEdges))
            return nvrhi::FE_GENERIC_ERROR;
    }
    if (numItems <= m_MaxItems)
        return nvrhi::FS_OK;

    const uint32_t items = slack(numItems);
    const uint32_t nodes = slack(2 * numItems);
    // The kernels run over whole sort blocks (1024 items), in thread groups of 256: one block bound per
    // thread group. (FleX sizes these by the item count alone and lets the kernels of the last sort block
    // write past the end, which D3D ignores; Vulkan does not promise to.)
    const uint32_t blocks = divCeil(divCeil(items, 1024) * 1024, 256);
    const uint32_t groups = divCeil(blocks, 256);

    PhysBufferHandle deltas, lefts, rights, children, lower, upper, groupLower, groupUpper;
    if (!createStructured(device, "ephys bvh key deltas", 4, items, deltas) ||
        !createStructured(device, "ephys bvh range lefts", 4, nodes, lefts) ||
        !createStructured(device, "ephys bvh range rights", 4, nodes, rights) ||
        !createStructured(device, "ephys bvh child counts", 4, nodes, children) ||
        !createStructured(device, "ephys bvh block bounds lower", 16, blocks, lower) ||
        !createStructured(device, "ephys bvh block bounds upper", 16, blocks, upper) ||
        !createStructured(device, "ephys bvh group bounds lower", 16, groups, groupLower) ||
        !createStructured(device, "ephys bvh group bounds upper", 16, groups, groupUpper))
        return nvrhi::FE_GENERIC_ERROR;

    const nvrhi::FRESULT result = m_Sort.create(device, items);
    if (NVRHI_FAILED(result))
        return result;

    m_Deltas = deltas;
    m_RangeLefts = lefts;
    m_RangeRights = rights;
    m_NumChildren = children;
    m_BoundsLower = lower;
    m_BoundsUpper = upper;
    m_GroupBoundsLower = groupLower;
    m_GroupBoundsUpper = groupUpper;
    m_MaxItems = items;
    return nvrhi::FS_OK;
}

nvrhi::FRESULT BvhBuilder::build(IPhysDevice* device, nvrhi::ICommandList* commandList, Bvh& bvh,
                                 nvrhi::IBuffer* itemLowers, nvrhi::IBuffer* itemUppers, uint32_t numItems,
                                 const Bounds3* totalBounds, bool* pReplaced) noexcept {
    if (pReplaced)
        *pReplaced = false;
    if (!device || !itemLowers || !itemUppers || numItems == 0 || numItems > c_MaxArrayCount)
        return nvrhi::FE_INVALID_ARGS;
    IPhysKernelLibrary* kernels = device->getKernels();

    // BVH::Resize.
    const uint32_t numNodes = 2 * numItems;
    if (numNodes > bvh.maxNodes) {
        const uint32_t nodes = slack(numNodes);
        PhysBufferHandle lowers, uppers, root = bvh.rootNode;
        if (!createStructured(device, "ephys bvh node lowers", sizeof(BvhNodeHalf), nodes, lowers) ||
            !createStructured(device, "ephys bvh node uppers", sizeof(BvhNodeHalf), nodes, uppers) ||
            (!root && !createStructured(device, "ephys bvh root", 4, 1, root)))
            return nvrhi::FE_GENERIC_ERROR;
        bvh.nodeLowers = lowers;
        bvh.nodeUppers = uppers;
        bvh.rootNode = root;
        bvh.maxNodes = nodes;
        if (pReplaced)
            *pReplaced = true;
    }
    bvh.numNodes = numNodes;

    nvrhi::FRESULT result = reserve(device, numItems);
    if (NVRHI_FAILED(result))
        return result;

    BvhParams params;
    params.numItems = int32_t(numItems);

    // Whole sort blocks: the Morton code kernel writes a key that sorts last for every element between
    // numItems and the end of the last block.
    const uint32_t numBlocks = divCeil(divCeil(numItems, 1024) * 1024, 256);

    KernelDispatchDesc dispatch;
    auto begin = [&](KernelId kernel, uint32_t groups) {
        dispatch = KernelDispatchDesc();
        dispatch.kernel = kernel;
        dispatch.gridDim[0] = groups;
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);
    };

    if (totalBounds) {
        float lower[3], upper[3], invEdges[3];
        for (int i = 0; i < 3; ++i) {
            lower[i] = totalBounds->lower[i];
            upper[i] = totalBounds->upper[i];
            invEdges[i] = 1.0f / ((upper[i] - lower[i]) + 1.0e-4f);
        }
        result = device->writeBuffer(commandList, m_TotalInvEdges, invEdges, 0, 1);
        if (NVRHI_SUCCEEDED(result))
            result = device->writeBuffer(commandList, m_TotalLower, lower, 0, 1);
        if (NVRHI_SUCCEEDED(result))
            result = device->writeBuffer(commandList, m_TotalUpper, upper, 0, 1);
        if (NVRHI_FAILED(result))
            return result;
    } else {
        // The bounds of each thread group, then of each group of 256 of those, then of everything.
        begin(KernelId::ComputeTotalBounds, numBlocks);
        dispatch.readOnly[0] = itemLowers;
        dispatch.readOnly[1] = itemUppers;
        dispatch.readWrite[0] = m_BoundsLower->getRHIBuffer();
        dispatch.readWrite[1] = m_BoundsUpper->getRHIBuffer();
        result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result))
            return result;

        begin(KernelId::ComputeTotalBoundsGroup, divCeil(numBlocks, 256));
        dispatch.readOnly[2] = m_BoundsLower->getRHIBuffer();
        dispatch.readOnly[3] = m_BoundsUpper->getRHIBuffer();
        dispatch.readWrite[0] = m_GroupBoundsLower->getRHIBuffer();
        dispatch.readWrite[1] = m_GroupBoundsUpper->getRHIBuffer();
        result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result))
            return result;

        begin(KernelId::ComputeTotalBoundsFinalize, 1);
        dispatch.readOnly[2] = m_GroupBoundsLower->getRHIBuffer();
        dispatch.readOnly[3] = m_GroupBoundsUpper->getRHIBuffer();
        dispatch.readWrite[2] = m_TotalLower->getRHIBuffer();
        dispatch.readWrite[3] = m_TotalUpper->getRHIBuffer();
        dispatch.readWrite[4] = m_TotalInvEdges->getRHIBuffer();
        result = kernels->dispatch(commandList, dispatch);
        if (NVRHI_FAILED(result))
            return result;
    }

    begin(KernelId::CalculateMortonCodes, numBlocks);
    dispatch.readOnly[0] = itemLowers;
    dispatch.readOnly[1] = itemUppers;
    dispatch.readOnly[2] = m_TotalLower->getRHIBuffer();
    dispatch.readOnly[3] = m_TotalInvEdges->getRHIBuffer();
    dispatch.readWrite[0] = m_Sort.getValues();
    dispatch.readWrite[1] = m_Sort.getKeys();
    result = kernels->dispatch(commandList, dispatch);
    if (NVRHI_FAILED(result))
        return result;

    result = m_Sort.sort(kernels, commandList, numItems, 32);
    if (NVRHI_FAILED(result))
        return result;

    begin(KernelId::CalculateKeyDeltas, numBlocks);
    dispatch.readOnly[0] = m_Sort.getKeys();
    dispatch.readWrite[0] = m_Deltas->getRHIBuffer();
    result = kernels->dispatch(commandList, dispatch);
    if (NVRHI_FAILED(result))
        return result;

    begin(KernelId::BuildLeaves, numBlocks);
    dispatch.readOnly[0] = itemLowers;
    dispatch.readOnly[1] = itemUppers;
    dispatch.readOnly[2] = m_Sort.getValues();
    dispatch.readWrite[0] = m_RangeLefts->getRHIBuffer();
    dispatch.readWrite[1] = m_RangeRights->getRHIBuffer();
    dispatch.readWrite[2] = bvh.nodeLowers->getRHIBuffer();
    dispatch.readWrite[3] = bvh.nodeUppers->getRHIBuffer();
    result = kernels->dispatch(commandList, dispatch);
    if (NVRHI_FAILED(result))
        return result;

    // Library::ClearBufferInt(mNumChildren, numNodes * sizeof(int), 0).
    ClearIntParams clear;
    clear.length = int32_t(numNodes);
    clear.value = 0;
    dispatch = KernelDispatchDesc();
    dispatch.kernel = KernelId::ClearInt;
    dispatch.gridDim[0] = divCeil(numNodes, 256);
    dispatch.rootConstants = &clear;
    dispatch.rootConstantsSize = sizeof(clear);
    dispatch.readWrite[0] = m_NumChildren->getRHIBuffer();
    result = kernels->dispatch(commandList, dispatch);
    if (NVRHI_FAILED(result))
        return result;

    begin(KernelId::BuildHierarchy, numBlocks);
    dispatch.readOnly[0] = itemLowers;
    dispatch.readOnly[1] = itemUppers;
    dispatch.readOnly[2] = m_Deltas->getRHIBuffer();
    dispatch.readWrite[0] = m_RangeLefts->getRHIBuffer();
    dispatch.readWrite[1] = m_RangeRights->getRHIBuffer();
    dispatch.readWrite[2] = bvh.nodeLowers->getRHIBuffer();
    dispatch.readWrite[3] = bvh.nodeUppers->getRHIBuffer();
    dispatch.readWrite[4] = m_NumChildren->getRHIBuffer();
    dispatch.readWrite[5] = bvh.rootNode->getRHIBuffer();
    return kernels->dispatch(commandList, dispatch);
}

}  // namespace ephys::detail
