#ifndef RESOURCEWRAPPER_H
#define RESOURCEWRAPPER_H
#include "NvFlexContextImpl.h"
#include "Allocable.h"

namespace NvFlex {

template <typename T>
class AutoPtr : protected Allocable {
 public:
    AutoPtr() noexcept
        : ptr_{} {}
    explicit AutoPtr(T *ptr) noexcept
        : ptr_{ptr} {
        internalAddRef();
    }
    AutoPtr(const AutoPtr &rhs) noexcept
        : ptr_{rhs.ptr_} {
        internalAddRef();
    }
    AutoPtr(AutoPtr &&rhs) noexcept
        : ptr_{} {
        swap(ptr_, rhs.ptr_);
    }
    ~AutoPtr() { internalRelease(); }

    /* 
    AutoPtr& operator = (T *ptr) noexcept {
        internalRelease();
        ptr_ = ptr;
        internalAddRef();
        return *this;
    } */

    AutoPtr &operator=(const AutoPtr &rhs) noexcept {
        internalRelease();
        ptr_ = rhs.ptr_;
        internalAddRef();
        return *this;
    }

    AutoPtr &operator=(AutoPtr &&rhs) noexcept {
        internalRelease();
        swap(ptr_, rhs.ptr_);
        return *this;
    }

    AutoPtr &operator=(nullptr_t) noexcept {
        internalRelease();
        return *this;
    }

    operator T *() const noexcept { return ptr_; }

    operator bool() const noexcept { return ptr_ != nullptr; }

    T *Get() const noexcept { return ptr_; }

    T **GetAddressOf() { return &ptr_; }

    T **ReleaseAndGetAddressOf() { return &ptr_; }

    void Reset(T *ptr = nullptr) {
        internalRelease();
        ptr_ = ptr;
        internalAddRef();
    }

    void Attach(T *ptr) {
        internalRelease();
        ptr_ = ptr;
    }

    T *Detach() {
        auto p = ptr_;
        ptr_ = nullptr;
        return p;
    }

    static AutoPtr<T> TakeOver(T *ptr) {
        AutoPtr<T> ap;
        ap.ptr_ = ptr;
        return ap;
    }

 protected:
    void internalAddRef() {
        if (ptr_)
            ptr_->addRef();
    }
    void internalRelease() {
        if (ptr_) {
            ptr_->release();
            ptr_ = nullptr;
        }
    }

    using IfType = T;
    IfType *ptr_;
};

template <typename ElemType = uint8_t>
class HUploadBuffer : public AutoPtr<NvFlexConstantBuffer> {
 public:
    using AutoPtr::AutoPtr;
    using AutoPtr::operator=;

    NvFlexResult Create(NvFlexContext *context, NvFlexUint dim, bool versioned,
                        const char *debugName = nullptr) {
        Reset();
        NvFlexConstantBufferDesc cbDesc = {};
        cbDesc.sizeInBytes = dim * sizeof(ElemType);
        cbDesc.uploadAccess = versioned;
        cbDesc.debugName = debugName;
        ptr_ = NvFlexCreateConstantBuffer(context, &cbDesc);
        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }
    ElemType *Map(NvFlexContext *context) {
        return (ElemType *)NvFlexConstantBufferMap(context, ptr_);
    }
    NvFlexResult Unmap(NvFlexContext *context) {
        NvFlexConstantBufferUnmap(context, ptr_);
        return eNvFlexSuccess;
    }
};

class HBufferBase : public AutoPtr<NvFlexBuffer> {
 public:
    operator NvFlexResource *() const noexcept { return NvFlexBufferGetResource(ptr_); }
    operator NvFlexResourceRW *() const noexcept { return NvFlexBufferGetResourceRW(ptr_); }

 protected:
    using AutoPtr::AutoPtr;
    using AutoPtr::operator=;

    void *Map(NvFlexContext *context,
              NvFlexStagingCpuAccessFlags cpuAcessFlags = eNvFlexStagingCpuAccess_readwrite,
              bool cpuNoWait = false) {
        return NvFlexContextMapBuffer(context, ptr_, cpuAcessFlags, cpuNoWait);
    }
    NvFlexResult Unmap(NvFlexContext *context) { NvFlexContextUnmapBuffer(context, ptr_); }
};

template <typename ElemType>
class HStructuredBuffer : public HBufferBase {
 public:
    using HBufferBase::HBufferBase;
    using HBufferBase::operator=;

    using ElementType = ElemType;

    NvFlexResult Create(NvFlexContext *context, NvFlexUint dim,
                        const char *debugName = nullptr) {
        Reset();
        NvFlexBufferDesc bufDesc = {};
        bufDesc.dim = dim;
        bufDesc.structStride = sizeof(ElemType);
        bufDesc.debugName = debugName;
        ptr_ = NvFlexCreateBuffer(context, &bufDesc);
        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }
    NvFlexResult CreateWithZero(NvFlexContext *context, NvFlexUint dim,
                                const char *debugName = nullptr) {
        Reset();
        NvFlexBufferDesc bufDesc = {};
        bufDesc.dim = dim;
        bufDesc.structStride = sizeof(ElemType);
        bufDesc.debugName = debugName;
        ptr_ = NvFlexCreateBuffer(context, &bufDesc);

        if(ptr_) {
            NvFlexUint sizeInBytes = dim * sizeof(ElemType);
            auto data = Allocable::allocate(sizeInBytes);
            memset(data, 0, sizeInBytes);
            NvFlexContextUploadBuffer(context, ptr_, 0, data, sizeInBytes);
            Allocable::deallocate(data);
        }

        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }

    void Write(NvFlexContext *context, NvFlexUint startIdx, NvFlexUint numElems, const ElementType *data) {
        NvFlexContextUploadBuffer(context, ptr_, startIdx * sizeof(ElementType), data,
                                  numElems * sizeof(ElementType));
    }

    NvFlexUint GetSize() const {
        if (!ptr_)
            return 0;
        NvFlexBufferDesc bufDesc;
        NvFlexBufferGetDesc(ptr_, &bufDesc);
        return bufDesc.dim;
    }
};

template <typename ElemType>
class HStagingBuffer : public HStructuredBuffer<ElemType> {
 public:
    using HStructuredBuffer<ElemType>::HStructuredBuffer;
    using HStructuredBuffer<ElemType>::operator=;

    NvFlexResult Create(NvFlexContext *context, NvFlexUint dim,
                               NvFlexCpuAccessMode cpuAccess, const char *debugName = nullptr) {
        Reset();

        NvFlexBufferDesc bufDesc = {};
        bufDesc.memType = eNvFlexMemoryType_readback;
        bufDesc.dim = dim;
        bufDesc.structStride = sizeof(ElemType);
        bufDesc.cpuAccessMode = cpuAccess;
        bufDesc.debugName = debugName;
        ptr_ = NvFlexCreateBuffer(context, &bufDesc);

        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }

    ElemType *Map(NvFlexContext *context,
              NvFlexStagingCpuAccessFlags cpuAcessFlags = eNvFlexStagingCpuAccess_readwrite,
              bool cpuNoWait = false) {
        return (ElemType *)NvFlexContextMapBuffer(context, ptr_, cpuAcessFlags, cpuNoWait);
    }
    NvFlexResult Unmap(NvFlexContext *context) {
        NvFlexContextUnmapBuffer(context, ptr_);
        return eNvFlexSuccess;
    }

    using HStructuredBuffer<ElemType>::GetSize;
    using HStructuredBuffer<ElemType>::Write;
};

enum TypelessFormatType { TypelessFormat_r32, TypelessFormat_r16 };

template <TypelessFormatType ElemType>
class HRawBuffer : public HBufferBase {
 public:
    using HBufferBase::HBufferBase;
    using HBufferBase::operator=;

    static constexpr TypelessFormatType FormatType = ElemType;

    NvFlexResult Create(NvFlexContext *context, NvFlexUint dim,
                        const char *debugName = nullptr) {
        Reset();
        NvFlexBufferDesc bufDesc = {};
        switch (FormatType) {
            case TypelessFormat_r32:
            default:
                bufDesc.format = eNvFlexFormat_r32_typeless;
                break;
            case TypelessFormat_r16:
                bufDesc.format = eNvFlexFormat_r16_typeless;
                break;
        }

        bufDesc.dim = dim;
        bufDesc.debugName = debugName;
        ptr_ = NvFlexCreateBuffer(context, &bufDesc);
        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }
    NvFlexResult CreateWithZero(NvFlexContext *context, NvFlexUint dim, const char *debugName = nullptr) {
        Reset();
        NvFlexBufferDesc bufDesc = {};
        switch (FormatType) {
            case TypelessFormat_r32:
            default:
                bufDesc.format = eNvFlexFormat_r32_typeless;
                break;
            case TypelessFormat_r16:
                bufDesc.format = eNvFlexFormat_r16_typeless;
                break;
        }
        bufDesc.dim = dim;
        bufDesc.debugName = debugName;
        ptr_ = NvFlexCreateBuffer(context, &bufDesc);

        if (ptr_) {
            NvFlexUint sizeInBytes = dim * sizeof(ElemType);
            auto data = Allocable::allocate(sizeInBytes);
            memset(data, 0, sizeInBytes);
            NvFlexContextUploadBuffer(context, ptr_, 0, data, sizeInBytes);
            Allocable::deallocate(data);
        }

        return ptr_ ? eNvFlexSuccess : eNvFlexFail;
    }
};

}

#endif /* RESOURCEWRAPPER_H */
