#ifndef NVFLEX_BASICMATH_H
#define NVFLEX_BASICMATH_H
#include <math.h>

namespace NvFlex {

template <typename T>
inline T min(const T &left, const T &right) {
    return left < right ? left : right;
}

template <typename T>
inline constexpr T max(const T &left, const T &right) {
    return left < right ? right : left;
}

inline int abs(int x) {
    return ::abs(x);
}

inline float abs(float x) {
    return ::fabs(x);
}

inline float sqrt(float x) {
    return ::sqrtf(x);
}

inline float ceil(float x) {
    return ::ceilf(x);
}

inline float floor(float x) {
    return ::floorl(x);
}

inline float log2(float x) {
    return ::log2(x);
}

inline float exp(float x) {
    return ::expf(x);
}

inline float pow(float a, float b) {
    return ::powf(a, b);
}

inline float infinity() {
    return INFINITY;
}

inline float pi() {
    return 3.141592654f;
}

inline float sqr(float x) {
    return x * x;
}

}  // namespace NvFlex

#endif /* NVFLEX_BASICMATH_H */
