#ifndef NVFLEXUTILS_H
#define NVFLEXUTILS_H
#include <wrl/client.h>
#include "Types.h"
#include "VectorCached.h"
#include "NvFlexObjectImpl.h"

namespace NvFlex {

template <typename T>
using ComPtr = Microsoft::WRL::ComPtr<T>;

#define NVFLEX_IMPLEMENT_OBJECT_REFERENCE() \
    uint32_t addRef() override {            \
        return Object::addRef();            \
    }                                       \
    uint32_t release() override {           \
        return Object::release();           \
    }

template <typename Timpl, typename Titf>
inline Timpl* implCast(Titf* ptr) {
    return static_cast<Timpl*>(ptr);
}

template <typename Timpl, typename Titf>
inline Timpl* implSafeCast(Titf* ptr) {
    return dynamic_cast<Timpl*>(ptr);
}

template <typename T, std::enable_if_t<std::is_base_of_v<IUnknown, T>, int> = 0>
inline void SafeRelease(T*& ptr) {
    if (ptr) {
        ptr->Release();
        ptr = nullptr;
    }
}

template <typename T, uint32_t N,
          std::enable_if_t<std::is_base_of_v<IUnknown, T>, int> = 0>
inline void SafeRelease(VectorCached<T*, N>& a) {
    for (auto& v : a) {
        if (v) {
            v->Release();
            v = nullptr;
        }
    }
}

template <typename T, std::enable_if_t<std::is_base_of_v<NvFlexObject, T>, int> = 0>
inline void SafeRelease(T*& ptr) {
    if (ptr) {
        ptr->release();
        ptr = nullptr;
    }
}

template <typename T, uint32_t N,
          std::enable_if_t<std::is_base_of_v<NvFlexObject, T>, int> = 0>
inline void SafeRelease(VectorCached<T*, N>& a) {
    for (auto& v : a) {
        if (v) {
            v->release();
            v = nullptr;
        }
    }
}

template <typename T, size_t N,
          std::enable_if_t<std::is_base_of_v<NvFlexObject, T>, int> = 0>
inline void SafeRelease(T* (&a)[N]) {
    for (auto& v : a) {
        if (v) {
            v->release();
            v = nullptr;
        }
    }
}

}  // namespace NvFlex

#endif /* NVFLEXUTILS_H */
