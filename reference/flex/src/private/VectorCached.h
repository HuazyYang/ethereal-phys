#ifndef VECTORCACHED_H
#define VECTORCACHED_H
#include "Types.h"
#include "type_traits"
#include "Allocable.h"
#include <limits>

namespace NvFlex {

namespace details {
template <typename T, std::enable_if_t<std::is_destructible_v<T>, int> = 0>
T* destroy_range(T *start, T *end) {
    while (start != end)
        (start++)->~T();
    return start;
}

template <typename T, std::enable_if_t<!std::is_destructible_v<T>, int> = 0>
T* destroy_range(T *start, T *end) {
    return end;
}

template <typename T>
T *copy_range(const T *start, const T *end, T *dest) {
    while (start != end)
        *dest++ = *start++;
    return dest;
}

template <typename T>
T *copy_construct_range(const T *start, const T *end, T *dest) {
    while (start != end)
        ::new (dest++) T(*start++);
    return dest;
}

template<typename T>
T *copy_construct_range_one_value(T *start, const T *end, const T &val) {
    while(start != end)
        ::new (start++) T{val};
    return start;
}

template <typename T>
T *move_construct_range(T *start, T *end, T *dest) {
    while (start != end)
        ::new (dest++) T(std::move(*start++));
    return dest;
}

template <typename T,
          std::enable_if_t<!std::is_trivially_default_constructible_v<T>, int> = 0>
T *default_construct_range(T *start, T *end) {
    while (start != end)
        ::new (start++) T{};
    return start;
}

template <typename T,
          std::enable_if_t<std::is_trivially_default_constructible_v<T>, int> = 0>
T *default_construct_range(T *start, T *end) {
    if(start != end)
        memset(start, 0, size_t(end - start) * sizeof(T));
    return end;
}

template <typename T, int n = std::is_move_assignable_v<T>      ? 0
                              : std::is_move_constructible_v<T> ? 1
                              : std::is_copy_assignable_v<T>    ? 2
                              : std::is_copy_constructible_v<T> ? 3
                                                                : 4>
struct EraseRangeProxy;

template<typename T>
struct EraseRangeProxy<T, 0> {
    T* operator()(T *start, T *mid, T *end) const {
        size_t left_n = mid - start;
        size_t right_n = end - mid;
        T *right = mid;

        while (left_n < right_n) {
            mid = right;
            while (start != mid)
                *start++ = std::move(*right++);
            right_n -= left_n;
        }

        // left_n >= right_n
        while (right != end)
            *start++ = std::move(*right++);
        destroy_range(start, end);

        return start;
    }
};

template <typename T>
struct EraseRangeProxy<T, 2> {
    T *operator()(T *start, T *mid, T *end) const {
        size_t left_n = mid - start;
        size_t right_n = end - mid;
        T *right = mid;

        while (left_n < right_n) {
            mid = right;
            while (start != mid)
                *start++ = (*right++);
            right_n -= left_n;
        }

        // left_n >= right_n
        while (right != end)
            *start++ = (*right++);
        destroy_range(start, end);

        return start;
    }
};

template<typename T>
struct EraseRangeProxy<T, 1> {
    T *operator()(T *start, T *mid, T *end) const {
        size_t left_n = mid - start;
        size_t right_n = end - mid;
        T *right = mid;

        while (left_n < right_n) {
            mid = right;
            destroy_range(start, mid);
            while (start != mid)
                ::new(start++) T(std::move(*right++));
            right_n -= left_n;
        }

        // left_n >= right_n
        destroy_range(start, start + right_n);
        while (right != end)
           ::new(start++)T(std::move(*right++));
        destroy_range(start, end);

        return start;
    }
};

template <typename T>
struct EraseRangeProxy<T, 3> {
    T *operator()(T *start, T *mid, T *end) const {
        size_t left_n = mid - start;
        size_t right_n = end - mid;
        T *right = mid;

        while (left_n < right_n) {
            mid = right;
            destroy_range(start, mid);
            while (start != mid)
                ::new (start++) T(*right++);
            right_n -= left_n;
        }

        // left_n >= right_n
        destroy_range(start, start + right_n);
        while (right != end)
            ::new (start++) T(*right++);
        destroy_range(start, end);

        return start;
    }
};

}  // namespace details

template <typename T, uint32_t N = 0>
class VectorCached {
 public:
    using size_type = uint32_t;
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using reference = T &;
    using const_reference = const T &;
    static constexpr size_type cached_size = N;

    using iterator = pointer;
    using const_iterator = const_pointer;

    VectorCached() {
        m_data = allocate(cached_size);
        m_capacity = cached_size;
        m_size = 0;
    }

    template <typename U = T,
              std::enable_if_t<std::is_default_constructible_v<U>, int> = 0>
    VectorCached(size_t count) {
        m_data = allocate(count);
        auto last = details::default_construct_range(m_data, m_data + count);
        m_capacity = count;
        m_size = last - m_data;
    }

    template <typename U = T,
              std::enable_if_t<std::is_copy_constructible_v<U>, int> = 0>
    VectorCached(size_t count, const T &val) {
        m_data = allocate(count);
        auto last = details::copy_construct_range_one_value(m_data, m_data + count, val);
        m_capacity = count;
        m_size = last - m_data;
    }

    ~VectorCached() { tidy(); }

    VectorCached(const VectorCached &other) {
        if (this != &other) {
            size_type new_capacity = other.m_size;
            m_data = allocate(new_capacity);
            auto last = details::copy_construct_range(other.m_data,
                                                      other.m_data + other.m_size, m_data);
            m_capacity = new_capacity;
            m_size = last - m_data;
        }
    }

    VectorCached(VectorCached &&other) {
        if (this != &other) {
            if (other.is_cached()) {
                size_type new_capacity = other.m_size;
                m_data = allocate(new_capacity);
                auto last = details::move_construct_range(
                    other.m_data, other.m_data + other.m_size, m_data);
                m_capacity = new_capacity;
                m_size = other.m_size;
            } else {
                m_data = other.m_data;
                m_capacity = other.m_capacity;
                m_size = other.m_size;
                other.m_data = 0;
                other.m_capacity = 0;
                other.m_size = 0;
            }
        }
    }

    VectorCached &operator=(const VectorCached &other) {
        if (this != &other) {
            if (m_capacity <= other.m_size) {
                tidy();
                size_type new_capacity = other.m_size;
                m_data = allocate(new_capacity);
                m_capacity = new_capacity;
            }

            size_type old_size = m_size;

            if (old_size < other.m_size) {
                auto last =
                    details::copy_range(other.m_data, other.m_data + old_size, m_data);
                last = details::copy_construct_range(other.m_data + old_size,
                                                     other.m_data + other.m_size, last);
            } else if (other.m_size < old_size) {
                details::copy_range(other.m_data, other.m_data + other.m_size, m_data);
                details::destroy_range(m_data + other.m_size, m_data + m_size);
            } else
                details::copy_range(other.m_data, other.m_data + other.m_size, m_data);

            m_size = other.m_size;
        }
        return *this;
    }

    VectorCached &operator=(VectorCached &&other) {
        tidy();
        if (other.is_cached()) {
            size_type new_capacity = other.m_size;
            m_data = allocate(new_capacity);
            m_capacity = new_capacity;
            details::move_construct_range(other.m_data, other.m_data + other.m_size,
                                          m_data);
            m_size = other.m_size;
            details::destroy_range(other.m_data, other.m_data + other.m_size);
            other.m_size = 0;
        } else {
            std::swap(m_data, other.m_data);
            std::swap(m_capacity, other.m_capacity);
            std::swap(m_size, other.m_size);
        }
        return *this;
    }

    reference operator[](size_type idx) {
        check_range(idx);
        return m_data[idx];
    }

    value_type *data() { return m_data; }

    const value_type *data() const { return m_data; }

    size_type allocateBack() {
        reserve(m_size + 1);
        construct(m_data + m_size);
        return m_size++;
    }

    void resize(size_type new_size) {
        reserve(new_size);

        if (new_size < m_size)
            details::destroy_range(m_data + new_size, m_data + m_size);
        else if (m_size < new_size)
            details::default_construct_range(m_data + m_size, m_data + new_size);

        m_size = new_size;
    }

    void clear() { resize(0); }

    size_type size() const { return m_size; }

    bool empty() const { return !m_size; }

    void push_back(const T &val) {
        reserve(m_size + 1);
        construct(m_data + m_size, val);
        m_size += 1;
    }

    void push_back(T &&val) {
        reserve(m_size + 1);
        construct(m_data + m_size, std::forward<T>(val));
        m_size += 1;
    }

    void pop_back() {
        check_range(0);
        destroy(std::addressof(m_data[--m_size]));
    }

    T *erase(iterator start, iterator end) {
        auto last = details::EraseRangeProxy<T>()(start, end, m_data + m_size);
        m_size = last - m_data;
        return last;
    }

    T &back() {
        check_range(0);
        return m_data[m_size - 1];
    }

    const T &back() const {
        check_range(0);
        return m_data[m_size - 1];
    }

    iterator begin() { return m_data; }

    iterator end() { return m_data + m_size; }

    const_iterator begin() const { return m_data; }

    const_iterator end() const { return m_data + m_size; }

    void reserve(size_type new_capacity) {
        size_type capacity = m_capacity ? m_capacity : 1;
        for (; capacity < new_capacity; capacity *= 2);

        if (capacity > m_capacity) {
            auto new_data = allocate(capacity);

            if (new_data != m_data) {
                details::move_construct_range(m_data, m_data + m_size, new_data);
                details::destroy_range(m_data, m_data + m_size);
                deallocate(m_data,  m_capacity);
                m_data = new_data;
            }

            m_capacity = capacity;
        }
    }

 private:
    void check_range(size_type idx) const {
        if (idx >= m_size) NVFLEX_INDEX_OUT_OF_RANGE_ERROR();
    }

    // Allocator
    value_type *allocate(size_type capacity) {
        if (capacity > cached_size)
            return (value_type *)Allocable::allocate(sizeof(value_type) * capacity);
        else if(cached_size)
            return (value_type *)m_cache;
        else
            return nullptr;
    }

    void deallocate(value_type *p, size_type count) {
        if (is_cached()) {
            m_data = nullptr;
        } else
            Allocable::deallocate(p);
    }

    template <typename... Args>
    void construct(value_type *p, Args &&...args) {
        ::new (p) value_type(std::forward<Args>(args)...);
    }

    void destroy(pointer p) { details::destruct(p); }

    void tidy() {
        if (m_size) {
            details::destroy_range(m_data, m_data + m_size);
            m_size = 0;
        }
        if (m_capacity) {
            deallocate(m_data, m_capacity);
            m_data = nullptr;
            m_capacity = 0;
        }
    }

    bool is_cached() const { return (const uint8_t *)m_data == m_cache; }

    value_type *m_data;
    size_type m_capacity;
    size_type m_size;
    alignas(alignof(T)) uint8_t m_cache[cached_size ? sizeof(value_type) * cached_size : 1];
};

template <typename T, uint32_t M, uint32_t N>
class VectorCached2D {
 public:
    using size_type = uint32_t;
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using reference = T &;
    using const_reference = const T &;
    static constexpr size_type cached_size_x = M;
    static constexpr size_type cached_size_y = N;
    static constexpr size_type cached_size = cached_size_x * cached_size_y;

    // Type sanity check
    static_assert(std::is_trivially_default_constructible_v<value_type> &&
                      std::is_trivially_destructible_v<value_type> &&
                      std::is_trivially_copy_constructible_v<value_type> &&
                      std::is_trivially_move_constructible_v<value_type>,
                  "None-POD type can not instantiate VectorCached template");

    struct Row {
        value_type *m_data;
        size_type m_size;

        reference operator[](size_type idx) {
            check_range(idx);
            return m_data[idx];
        }

     private:
        void check_range(size_type idx) const {
            if (idx >= m_size) NVFLEX_INDEX_OUT_OF_RANGE_ERROR();
        }
    };

    VectorCached2D() {
        uint32_t capacityX = M, capacityY = N;
        m_sizeX = 0;
        m_sizeY = 0;
        m_capacityX = M;
        m_capacityY = N;
        m_data = allocate(capacityX, capacityY);
        size_type capacity = capacityX * capacityY;

        for (size_type i = 0; i < capacity; ++i)
            ::new (m_data + i) value_type();
    }

    ~VectorCached2D() {
        cleanup(m_data, m_capacityX, m_capacityY);
        m_data = nullptr;
        m_capacityX = 0;
        m_capacityY = 0;
        m_sizeX = 0;
        m_sizeY = 0;
    }

    VectorCached2D(const VectorCached2D &) = delete;
    VectorCached2D &operator=(const VectorCached2D &other) = delete;

    Row operator[](size_type idx) {
        check_range(idx);
        return Row{m_data + m_capacityX * idx, m_sizeX};
    }

    size_type allocateBackX() {
        reserve(m_sizeX + 1, m_sizeY);
        return m_sizeX++;
    }

    size_type allocateBackY() {
        reserve(m_sizeX, m_sizeY + 1);
        return m_sizeY++;
    }

    void resize(size_type sizeX, size_type sizeY) {
        reserve(sizeX, sizeY);
        m_sizeX = sizeX;
        m_sizeY = sizeY;
    }

    size_type sizeX() const { return m_sizeX; }

    size_type sizeY() const { return m_sizeY; }

    void reserve(size_type requestedCapacityX, size_type requestedCapacityY) {
        size_type capacityX = m_capacityX, capacityY = m_capacityY;
        while (capacityX < requestedCapacityX)
            capacityX *= 2;
        while (capacityY < requestedCapacityY)
            capacityY *= 2;

        if (capacityX > m_capacityX || capacityY > m_capacityY) {
            auto newData = allocate(capacityX, capacityY);
            auto oldData = m_data;

            for (size_type j = 0; j < capacityY; ++j)
                for (size_type i = 0; i < capacityX; ++i) {
                    if (i >= m_sizeX || j >= m_sizeY)
                        ::new (newData + (i + j * capacityX)) value_type();
                    else {
                        auto newVal =
                            ::new (newData + (i + j * capacityX)) value_type();
                        auto oldVal = &m_data[i + m_capacityX * j];
                        *newVal = std::move(*oldVal);
                    }
                }

            cleanup(m_data, m_capacityX, m_capacityY);
            m_data = newData;
            m_capacityX = capacityX;
            m_capacityY = capacityY;
        }
    }

 private:
    void check_range(size_type ix) const {
        if (ix >= m_sizeY) NVFLEX_INDEX_OUT_OF_RANGE_ERROR();
    }

    value_type *allocate(size_type capacityX, size_type capacityY) {
        if (capacityX > cached_size_x || capacityY > cached_size_y)
            return (value_type *)Allocable::allocate(sizeof(value_type) * capacityX *
                                                     capacityY);
        else
            return (value_type *)m_cache;
    }

    void cleanup(T *data, size_type capacityX, size_type capacityY) {
        size_type capacity = capacityX * capacityY;
        for (size_t i = 0; i < capacity; ++i)
            data[i].~value_type();
        if (data != (T *)m_cache) Allocable::deallocate(data);
    }

    value_type *m_data;
    size_type m_capacityX;
    size_type m_capacityY;
    size_type m_sizeX;
    size_type m_sizeY;
    alignas(alignof(T)) uint8_t m_cache[sizeof(value_type) * cached_size];
};

};  // namespace NvFlex

#endif /* VECTORCACHED_H */
