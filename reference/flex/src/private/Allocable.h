#ifndef ALLOCABLE_H
#define ALLOCABLE_H
#include <stddef.h>
#include <type_traits>

namespace NvFlex {

void FlexSetMallocFunc(void* (*malloc)(size_t));

void FlexSetFreeFunc(void (*free)(void*));

class Allocable {
 public:
    void* operator new(size_t count);
    void* operator new[](size_t count);

    void operator delete(void* ptr);
    void operator delete[](void* ptr);

    static void* allocate(size_t sz);
    static void deallocate(void* ptr);

 protected:
    Allocable() = default;
    ~Allocable() = default;
};

namespace details {
template <typename T>
typename std::enable_if<std::is_destructible<T>::value, void>::type destruct(T *ptr) {
    ptr->~T();
}

template <typename T>
typename std::enable_if<!std::is_destructible<T>::value, void>::type destruct(T *ptr) {}
}

template <typename T>
struct Allocator {
    using value_type = T;
    using pointer = value_type *;
    using const_pointer = const value_type *;
    using reference = value_type &;
    using const_reference = const value_type &;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    Allocator() noexcept {}

    template <class U>
    Allocator(const Allocator<U> &other) noexcept {}

    template <class U>
    Allocator(Allocator<U> &&other) noexcept {}

    template <class U>
    Allocator &operator=(Allocator<U> &&other) noexcept {
        return *this;
    }

    template <class U>
    struct rebind {
        typedef Allocator<U> other;
    };

    T *allocate(std::size_t count) { return (T *)Allocable::allocate(count * sizeof(T)); }

    pointer address(reference r) { return &r; }
    const_pointer address(const_reference r) { return &r; }

    void deallocate(T *p, std::size_t count) { Allocable::deallocate(p); }

    inline size_type max_size() const {
        return (std::numeric_limits<size_type>::max)() / sizeof(T);
    }

    //    construction/destruction
    template <class U, class... Args>
    void construct(U *p, Args &&...args) {
        ::new (p) U(std::forward<Args>(args)...);
    }

    inline void destroy(pointer p) { details::destruct(p); }
};

template <class T, class U>
bool operator==(const Allocator<T> &left, const Allocator<U> &right) noexcept {
    return &left.m_Allocator == &right.m_Allocator;
}

template <class T, class U, class A>
bool operator!=(const Allocator<T> &left, const Allocator<U> &right) {
    return !(left == right);
}

}  // namespace NvFlex

#endif /* ALLOCABLE_H */
