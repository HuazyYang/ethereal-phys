#include "Object.h"

namespace NvFlex {

uint32_t Object::addRef() {
    uint32_t ref = m_refCount.fetch_add(1, std::memory_order_relaxed) + 1;
    return ref;
}

Object::Object() : m_refCount(1), m_deferredRelease(nullptr) {}

Object::Object(DeferredRelease* deferredRelease) : m_refCount{1}, m_deferredRelease(deferredRelease) {
    if (m_deferredRelease) m_deferredRelease->registerObject(this);
}

uint32_t Object::release() {
    uint32_t ref = m_refCount.fetch_add(-1) - 1;
    if (ref) return ref;

    if (m_deferredRelease) {
        auto deferredRelease = this->m_deferredRelease;
        this->m_deferredRelease = nullptr;
        this->addRef();
        deferredRelease->pushForRelease(this);
    } else {
        delete this;
    }
    return 0;
}

DeferredRelease* Object::getDeferredRelease() const {
    return m_deferredRelease;
}

uint64_t Object::getGPUBytesUsed() {
    return 0;
}

}  // namespace NvFlex