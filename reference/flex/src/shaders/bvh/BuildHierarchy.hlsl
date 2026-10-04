#include "BVHCommon.hlsli"

StructuredBuffer<int> deltas : register(t2);
globallycoherent RWStructuredBuffer<int> rangeLefts : register(u0);
globallycoherent RWStructuredBuffer<int> rangeRights : register(u1);
globallycoherent RWStructuredBuffer<PackedNodeHalf> lowers : register(u2);
globallycoherent RWStructuredBuffer<PackedNodeHalf> uppers : register(u3);
globallycoherent RWStructuredBuffer<int> numChildren : register(u4);
globallycoherent RWStructuredBuffer<int> root : register(u5);

[numthreads(256, 1, 1)]
void BuildHierarchy(int idx: SV_DispatchThreadID) {
    if (idx < gNumItems) {

        int nodeIdx = idx;

        while (true) {
            int leftNode, rightNode;

            InterlockedOr(rangeLefts[nodeIdx], 0, leftNode);
            InterlockedOr(rangeRights[nodeIdx], 0, rightNode);

            if (leftNode == 0 && rightNode == gNumItems - 1) {
                root[0] = nodeIdx;
                return;
            }

            bool rightNodeValid = rightNode != gNumItems - 1;
            int similarityRight = deltas[rightNode];
            int similarityLeft = deltas[leftNode - 1];
            int prevNumChildren;

            if (leftNode == 0 || (rightNodeValid && similarityRight < similarityLeft)) {
                int parent = rightNode + gNumItems;
               lowers[parent].ib  = (lowers[parent].ib & 0x80000000) | nodeIdx;

               InterlockedExchange(rangeLefts[parent], leftNode, prevNumChildren);
               InterlockedAdd(numChildren[parent], 1, prevNumChildren);

               nodeIdx = parent;
            } else {
                int parent = leftNode + gNumItems - 1;
                uppers[parent].ib = (uppers[parent].ib & 0x80000000) | nodeIdx;

                InterlockedExchange(rangeRights[parent], rightNode, prevNumChildren);
                InterlockedAdd(numChildren[parent], 1, prevNumChildren);

                nodeIdx = parent;
            }

            if (prevNumChildren == 1) {
                int firstLeafIdx = lowers[nodeIdx].ib & 0x7fffffff;
                int lastLeafIdx = uppers[nodeIdx].ib & 0x7fffffff;

                PackedNodeHalf packedNodeLower, packedNodeUpper;

                PackedNodeHalf firstLeafLower = lowers[firstLeafIdx];
                PackedNodeHalf lastLeafLower = lowers[lastLeafIdx];
                packedNodeLower.v = min(firstLeafLower.v, lastLeafLower.v);
                packedNodeLower.ib = firstLeafIdx;

                PackedNodeHalf firstLeafUpper = uppers[firstLeafIdx];
                PackedNodeHalf lastLeafUpper = uppers[lastLeafIdx];
                packedNodeUpper.v = max(firstLeafUpper.v, lastLeafUpper.v);
                packedNodeUpper.ib = lastLeafIdx;

                lowers[nodeIdx] = packedNodeLower;
                uppers[nodeIdx] = packedNodeUpper;
            } else
                return;
        }
    }
}

