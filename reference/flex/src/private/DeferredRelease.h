#ifndef DEFERREDRELEASE_H
#define DEFERREDRELEASE_H
#include "NvFlexObjectImpl.h"
#include "Allocable.h"

namespace NvFlex {

struct DeferredRelease : NvFlexObject, Allocable {
    virtual void registerObject(NvFlexObject *object) = 0;

    virtual void pushForRelease(NvFlexObject *object) = 0;
};

}  // namespace NvFlex

#endif /* DEFERREDRELEASE_H */
