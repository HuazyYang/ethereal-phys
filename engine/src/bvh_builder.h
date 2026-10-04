#ifndef ETHEREAL_PHYS_SRC_BVH_BUILDER_H
#define ETHEREAL_PHYS_SRC_BVH_BUILDER_H

// Module-private: the GPU builder of linear bounding volume hierarchies, the driver of the bvh/ kernels.
//
// It is FleX's LinearBVHBuilderGPU (reference/flex/src/BVHBuilder.cpp) on IPhysKernelLibrary: Morton codes
// of the item centres on a 512^3 grid over the total bounds, a radix sort, the leaves in Morton order, and
// the hierarchy built bottom-up from the differences of neighbouring codes (Karras 2012 as FleX has it).
//
// Two things use it. The geometry library builds the hierarchy over the triangles of a mesh when the mesh
// is updated (ITriangleMeshAsset::update), with the bounds the application gives. A solver will build the
// hierarchy over the shapes of a domain in every step that changed them, with bounds the builder computes
// (FleX's ShapeData::RebuildShapeData).
//
// Like radix_sort.cpp it is written against the public interfaces only, so tests/data_test.cpp compiles
// this file into the test and runs the code the engine runs.

#include <ethereal/phys/collision.h>
#include <ethereal/phys/device.h>

#include "radix_sort.h"

#include <cstdint>

namespace ephys::detail {

// A hierarchy: FleX's BVH. The node arrays hold BvhNodeHalf (collision.h says how nodes are laid out).
struct Bvh {
    PhysBufferHandle nodeLowers;
    PhysBufferHandle nodeUppers;
    PhysBufferHandle rootNode;  // one int32: the index of the root
    uint32_t numNodes = 0;      // 2 * the number of items of the last build
    uint32_t maxNodes = 0;
};

class BvhBuilder {
 public:
    // Records the build of the hierarchy over numItems boxes (not 0). itemLowers and itemUppers are
    // structured buffers of float4. totalBounds contains all items, or is null to have it computed from
    // them. bvh's buffers are replaced when they are too small for 2 * numItems nodes; *pReplaced then
    // says so.
    //
    // commandList: as IPhysKernelLibrary::dispatch. Returns the first failing code of a buffer creation
    // or a dispatch.
    nvrhi::FRESULT build(IPhysDevice* device, nvrhi::ICommandList* commandList, Bvh& bvh,
                         nvrhi::IBuffer* itemLowers, nvrhi::IBuffer* itemUppers, uint32_t numItems,
                         const Bounds3* totalBounds, bool* pReplaced) noexcept;

 private:
    nvrhi::FRESULT reserve(IPhysDevice* device, uint32_t numItems) noexcept;

    uint32_t m_MaxItems = 0;
    PhysBufferHandle m_Deltas;
    PhysBufferHandle m_RangeLefts;
    PhysBufferHandle m_RangeRights;
    PhysBufferHandle m_NumChildren;
    PhysBufferHandle m_BoundsLower;
    PhysBufferHandle m_BoundsUpper;
    PhysBufferHandle m_GroupBoundsLower;
    PhysBufferHandle m_GroupBoundsUpper;
    PhysBufferHandle m_TotalLower;
    PhysBufferHandle m_TotalUpper;
    PhysBufferHandle m_TotalInvEdges;
    RadixSort m_Sort;
};

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_BVH_BUILDER_H */
