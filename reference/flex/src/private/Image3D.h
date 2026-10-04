#ifndef IMAGE3D_H
#define IMAGE3D_H
#include "Types.h"

namespace NvFlex {

template <typename T>
struct Image3D {
    Image3D();
    Image3D(NvFlexDim dim);
    ~Image3D();

    T& operator[](ptrdiff_t i);

    const T& operator[](ptrdiff_t i) const;

    void init(NvFlexDim dim);

    size_t dim1() const;

    NvFlexDim dim() const;

    T* data();

    const T* data() const;

    const T& operator()(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k) const;

    T& operator()(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k);

 private:
    Image3D(const Image3D&) = delete;
    Image3D& operator=(const Image3D&) = delete;

    T* allocate(size_t count);
    void deallocate(T* p);

    T* m_data;
    NvFlexDim m_dim;
    NvFlexDim m_capacity;
};
template <typename T>
inline Image3D<T>::Image3D() : m_dim{0, 0, 0}, m_capacity{0, 0, 0}, m_data{0} {}
template <typename T>
inline Image3D<T>::Image3D(NvFlexDim dim) : Image3D() {
    size_t size1 = dim.z * dim.y * dim.x;
    m_data = allocate(size1);
    details::default_construct_range(m_data, m_data + size1);
    m_dim = m_capacity = dim;
}

template <typename T>
inline Image3D<T>::~Image3D() {
    size_t size1 = m_dim.z * m_dim.y * m_dim.x;
    details::destroy_range(m_data, m_data + size1);
    deallocate(m_data);
    m_data = 0;
    m_dim.x = 0;
    m_dim.y = 0;
    m_dim.z = 0;
    m_capacity = m_dim;
}

template <typename T>
inline T& Image3D<T>::operator[](ptrdiff_t i) {
    return m_data[i];
}

template <typename T>
inline const T& Image3D<T>::operator[](ptrdiff_t i) const {
    return m_data[i];
}

template <typename T>
inline void Image3D<T>::init(NvFlexDim dim) {
    if (dim.x > m_dim.x || dim.y > m_dim.y || dim.z > m_dim.z) {
        size_t old_size1 = m_dim.z * m_dim.y * m_dim.x;
        details::destroy_range(m_data, m_data + old_size1);
        deallocate(m_data);

        size_t new_size1 = dim.z * dim.y * dim.x;
        m_data = allocate(new_size1);
        details::default_construct_range(m_data, m_data + new_size1);
        m_dim = m_capacity = dim;
    } else {
        size_t old_size1 = m_dim.z * m_dim.y * m_dim.x;
        details::destroy_range(m_data, m_data + old_size1);

        size_t new_size1 = dim.z * dim.y * dim.x;
        details::default_construct_range(m_data, m_data + new_size1);

        m_dim = dim;
    }
}

template <typename T>
inline size_t Image3D<T>::dim1() const {
    return size_t(m_dim.z) * m_dim.y * m_dim.x;
}

template <typename T>
inline NvFlexDim Image3D<T>::dim() const {
    return m_dim;
}

template <typename T>
inline T* Image3D<T>::data() {
    return m_data;
}

template <typename T>
inline const T* Image3D<T>::data() const {
    return m_data;
}

template <typename T>
inline const T& Image3D<T>::operator()(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k) const {
    return m_data[i + m_dim.x * (j + k * m_dim.y)];
}

template <typename T>
inline T& Image3D<T>::operator()(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k) {
    return m_data[i + m_dim.x * (j + k * m_dim.y)];
}

template <typename T>
inline T* Image3D<T>::allocate(size_t count) {
    return (T*)Allocable::allocate(sizeof(T) * count);
}

template <typename T>
inline void Image3D<T>::deallocate(T* p) {
    Allocable::deallocate(p);
}

};  // namespace NvFlex

#endif /* IMAGE3D_H */
