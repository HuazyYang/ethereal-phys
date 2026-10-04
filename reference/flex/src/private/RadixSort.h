#ifndef NVFLEX_RADIXSORT_H
#define NVFLEX_RADIXSORT_H
#include "NvFlexObjectImpl.h"
#include "NvFlexContextImpl.h"

namespace NvFlex {

struct RadixSortBuffer {
    NvFlexBuffer *key;
    NvFlexBuffer *val;
};

struct RadixSortDesc {
    unsigned int maxSortBlocks;
};

struct RadixSortParams {
    unsigned int numKeys;
    unsigned int numSortBlocks;
    unsigned int bits;
};

struct RadixSort : NvFlexObject {
    virtual RadixSortBuffer getBuffer() = 0;
    virtual void sort(NvFlexContext *context, const RadixSortParams *params) = 0;
};

RadixSort *createRadixSort(NvFlexContext *context, const RadixSortDesc *desc);

struct RadixSortCPUBuffer {
    NvFlexUint2 *keyVal;
};

struct RadixSortCPUParams {
    unsigned int numElements;
};

struct RadixSortCPUDesc {
    unsigned int maxElements;
};

struct RadixSortCPU : NvFlexObject {
    virtual RadixSortCPUBuffer getBuffer() = 0;
    virtual void sort(NvFlexContext *context, const RadixSortCPUParams *params) = 0;
};

RadixSortCPU *createRadixSortCPU(const RadixSortCPUDesc *desc);

}  // namespace NvFlex

#endif /* NVFLEX_RADIXSORT_H */
