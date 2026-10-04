#ifndef NVFLEXOBJECT_H
#define NVFLEXOBJECT_H
#include <cstdlib>
#include <cstdint>

struct NvFlexObject {
    virtual uint32_t addRef() = 0;

    virtual uint32_t release() = 0;

    virtual uint64_t getGPUBytesUsed() = 0;
};  // namespace NvFlex

#endif /* NVFLEXOBJECT_H */
