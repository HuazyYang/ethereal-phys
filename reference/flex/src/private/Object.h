#ifndef OBJECT_H
#define OBJECT_H
#include "NvFlexObjectImpl.h"
#include "Allocable.h"
#include "DeferredRelease.h"
#include <atomic>

namespace NvFlex {

class Object : public NvFlexObject, public Allocable {
 public:
    uint32_t addRef() override;
    uint32_t release() override;

    uint64_t getGPUBytesUsed() override;  // Default to zero

   //  virtual uint32_t addRefInternal() = 0;
   //  virtual uint32_t releaseInternal() = 0;

    DeferredRelease* getDeferredRelease() const;
 protected:
    Object();
    Object(DeferredRelease* deferredRelease);
    virtual ~Object() = default;

 private:
    Object(const Object&) = delete;
    Object& operator=(const Object&) = delete;

    std::atomic<uint32_t> m_refCount;
    DeferredRelease* m_deferredRelease;
};

// struct Object_vtbl // sizeof=0x30

}  // namespace NvFlex

#endif /* OBJECT_H */
