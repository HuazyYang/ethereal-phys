#ifndef NVFLEXMATH_H
#define NVFLEXMATH_H
#include <nvflex/NvFlexTypes.h>
#include "BasicMath.h"

struct NvFlexBounds3f {
    NvFlexFloat3 lower;
    NvFlexFloat3 upper;
};

namespace NvFlex {

///
/// Scalar math
///
inline float asfloat(int x) {
    return *reinterpret_cast<float *>(&x);
}

inline float asfloat(unsigned int x) {
    return *reinterpret_cast<float *>(&x);
}

inline int asint(float x) {
    return *reinterpret_cast<int *>(&x);
}

inline unsigned int asuint(float x) {
    return *reinterpret_cast<unsigned int *>(&x);
}

///
/// Vector math
///

#define NVFLEX_VECTOR3_OPERATORS(VecType, CompType, FuncSuffix)               \
    inline VecType make_##FuncSuffix(CompType s) {                            \
        return VecType{s, s, s};                                              \
    }                                                                         \
    inline VecType make_##FuncSuffix(CompType s1, CompType s2, CompType s3) { \
        return VecType{s1, s2, s3};                                           \
    }                                                                         \
    template <typename Ty>                                                    \
    inline VecType make_##FuncSuffix(Ty s) {                                  \
        CompType s1 = static_cast<CompType>(s);                               \
        return VecType{s1, s1, s1};                                           \
    }                                                                         \
    template <typename Ty>                                                    \
    inline VecType make_##FuncSuffix(Ty s1, Ty s2, Ty s3) {                   \
        return VecType{static_cast<CompType>(s1), static_cast<CompType>(s2),  \
                       static_cast<CompType>(s3)};                            \
    }                                                                         \
    inline bool operator==(const VecType &left, const VecType &right) {       \
        return left.x == right.x && left.y == right.y && left.z == right.z;   \
    }                                                                         \
    inline bool operator!=(const VecType &left, const VecType &right) {       \
        return left.x != right.x || left.y != right.y || left.z != right.z;   \
    }                                                                         \
    inline VecType operator/(VecType v1, VecType v2) {                        \
        return VecType{v1.x / v2.x, v1.y / v2.y, v1.z / v2.z};                \
    }                                                                         \
                                                                              \
    inline VecType operator/(VecType v, CompType s) {                         \
        return VecType{v.x / s, v.y / s, v.z / s};                            \
    }                                                                         \
    inline VecType operator/(CompType s, VecType v) {                         \
        return VecType{s / v.x, s / v.y, s / v.z};                            \
    }                                                                         \
    inline VecType operator+(VecType v1, VecType v2) {                        \
        return VecType{v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};                \
    }                                                                         \
    inline VecType operator+(VecType v, CompType s) {                         \
        return VecType{v.x + s, v.y + s, v.z + s};                            \
    }                                                                         \
    inline VecType operator+(CompType s, VecType v) {                         \
        return VecType{s + v.x, s + v.y, s + v.z};                            \
    }                                                                         \
    inline VecType operator-(VecType v1, VecType v2) {                        \
        return VecType{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};                \
    }                                                                         \
    inline VecType operator-(VecType v, CompType s) {                         \
        return VecType{v.x - s, v.y - s, v.z - s};                            \
    }                                                                         \
    inline VecType operator-(CompType s, VecType v) {                         \
        return VecType{s - v.x, s - v.y, s - v.z};                            \
    }                                                                         \
    inline VecType operator*(VecType v1, VecType v2) {                        \
        return VecType{v1.x * v2.x, v1.y * v2.y, v1.z * v2.z};                \
    }                                                                         \
                                                                              \
    inline VecType operator*(VecType v, CompType s) {                         \
        return VecType{v.x * s, v.y * s, v.z * s};                            \
    }                                                                         \
                                                                              \
    inline VecType operator*(CompType s, VecType v) {                         \
        return VecType{v.x * s, v.y * s, v.z * s};                            \
    }

#define NVFLEX_VECTOR3_BIT_OPERATORS(VecType)              \
    inline VecType operator>>(VecType v, unsigned int s) { \
        return VecType{v.x >> s, v.y >> s, v.z >> s};      \
    }

#define NVFLEX_VECTOR4_OPERATORS(VecType, CompType, FuncSuffix)                            \
    inline VecType make_##FuncSuffix(CompType s) {                                         \
        return VecType{s, s, s, s};                                                        \
    }                                                                                      \
    inline VecType make_##FuncSuffix(CompType s1, CompType s2, CompType s3, CompType s4) { \
        return VecType{s1, s2, s3, s4};                                                    \
    }                                                                                      \
    template <typename Ty>                                                                 \
    inline VecType make_##FuncSuffix(Ty s) {                                               \
        CompType s1 = static_cast<CompType>(s);                                            \
        return VecType{s1, s1, s1, s1};                                                    \
    }                                                                                      \
    template <typename Ty>                                                                 \
    inline VecType make_##FuncSuffix(Ty s1, Ty s2, Ty s3, Ty s4) {                         \
        return VecType{static_cast<CompType>(s1), static_cast<CompType>(s2),               \
                       static_cast<CompType>(s3), static_cast<CompType>(s4)};              \
    }                                                                                      \
    inline bool operator==(const VecType &left, const VecType &right) {                    \
        return left.x == right.x && left.y == right.y && left.z == right.z &&              \
               left.w == right.w;                                                          \
    }                                                                                      \
                                                                                           \
    inline VecType operator/(VecType v1, VecType v2) {                                     \
        return VecType{v1.x / v2.x, v1.y / v2.y, v1.z / v2.z, v1.w / v2.w};                \
    }                                                                                      \
                                                                                           \
    inline VecType operator/(VecType v, CompType s) {                                      \
        return VecType{v.x / s, v.y / s, v.z / s, v.w / s};                                \
    }                                                                                      \
    inline VecType operator/(CompType s, VecType v) {                                      \
        return VecType{s / v.x, s / v.y, s / v.z, s / v.w};                                \
    }                                                                                      \
    inline VecType operator+(VecType v1, VecType v2) {                                     \
        return VecType{v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w};                \
    }                                                                                      \
                                                                                           \
    inline VecType operator+(VecType v, CompType s) {                                      \
        return VecType{v.x + s, v.y + s, v.z + s, v.w + s};                                \
    }                                                                                      \
                                                                                           \
    inline VecType operator+(CompType s, VecType v) {                                      \
        return VecType{v.x + s, v.y + s, v.z + s, v.w + s};                                \
    }                                                                                      \
    inline VecType operator-(VecType v1, VecType v2) {                                     \
        return VecType{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w};                \
    }                                                                                      \
                                                                                           \
    inline VecType operator-(VecType v, CompType s) {                                      \
        return VecType{v.x - s, v.y - s, v.z - s, v.w - s};                                \
    }                                                                                      \
                                                                                           \
    inline VecType operator-(CompType s, VecType v) {                                      \
        return VecType{s - v.x, s - v.y, s - v.z, s - v.w};                                \
    }                                                                                      \
    inline VecType operator*(VecType v1, VecType v2) {                                     \
        return VecType{v1.x * v2.x, v1.y * v2.y, v1.z * v2.z, v1.w * v2.w};                \
    }                                                                                      \
                                                                                           \
    inline VecType operator*(VecType v, CompType s) {                                      \
        return VecType{v.x * s, v.y * s, v.z * s, v.w * s};                                \
    }                                                                                      \
                                                                                           \
    inline VecType operator*(CompType s, VecType v) {                                      \
        return VecType{v.x * s, v.y * s, v.z * s, v.w * s};                                \
    }

#define NVFLEX_VECTOR4_BIT_OPERATORS(VecType)                   \
    inline VecType operator>>(VecType v, unsigned int s) {      \
        return VecType{v.x >> s, v.y >> s, v.z >> s, v.w >> s}; \
    }

NVFLEX_VECTOR3_OPERATORS(NvFlexDim, NvFlexUint, dim)
NVFLEX_VECTOR3_BIT_OPERATORS(NvFlexDim)

NVFLEX_VECTOR3_OPERATORS(NvFlexUint3, NvFlexUint, uint3)
NVFLEX_VECTOR3_BIT_OPERATORS(NvFlexUint3)

NVFLEX_VECTOR3_OPERATORS(NvFlexInt3, int, int3)
NVFLEX_VECTOR3_BIT_OPERATORS(NvFlexInt3)

NVFLEX_VECTOR3_OPERATORS(NvFlexFloat3, float, float3)

NVFLEX_VECTOR4_OPERATORS(NvFlexUint4, NvFlexUint, uint4)
NVFLEX_VECTOR4_BIT_OPERATORS(NvFlexUint4)

NVFLEX_VECTOR4_OPERATORS(NvFlexInt4, int, int4)
NVFLEX_VECTOR4_BIT_OPERATORS(NvFlexInt4)

NVFLEX_VECTOR4_OPERATORS(NvFlexFloat4, float, float4)

inline NvFlexDim make_dim(const NvFlexUint3 &v) {
    return NvFlexDim{v.x, v.y, v.z};
}

inline NvFlexDim make_dim(const NvFlexUint4 &v) {
    return NvFlexDim{v.x, v.y, v.z};
}

inline NvFlexUint3 make_uint3(const NvFlexDim &v) {
    return NvFlexUint3{v.x, v.y, v.z};
}

inline NvFlexInt4 make_int4(const NvFlexInt3 &v, int s) {
    return NvFlexInt4{v.x, v.y, v.z, s};
}

inline NvFlexInt4 make_int4(const NvFlexFloat4 &v) {
    return NvFlexInt4{int(v.x), int(v.y), int(v.z), int(v.w)};
}

inline NvFlexInt4 make_int4(const NvFlexFloat3 &v, float s) {
    return NvFlexInt4{int(v.x), int(v.y), int(v.z), int(s)};
}

inline NvFlexUint4 make_uint4(const NvFlexUint3 &v, NvFlexUint s) {
    return NvFlexUint4{v.x, v.y, v.z, s};
}

inline NvFlexUint4 make_uint4(const NvFlexDim dim, NvFlexUint s) {
    return NvFlexUint4{dim.x, dim.y, dim.z, s};
}

inline NvFlexFloat3 make_float3(const NvFlexDim &v) {
    return NvFlexFloat3{float(v.x), float(v.y), float(v.z)};
}

inline NvFlexUint4 make_uint4(const NvFlexFloat4 &v) {
    return NvFlexUint4{(unsigned int)(v.x), (unsigned int)(v.y), (unsigned int)(v.z),
                       (unsigned int)(v.w)};
}

inline NvFlexFloat4 make_float4(const NvFlexFloat3 &v, float s) {
    return NvFlexFloat4{v.x, v.y, v.z, s};
}

inline NvFlexFloat4 make_float4(const NvFlexDim &v, NvFlexUint s) {
    return NvFlexFloat4{float(v.x), float(v.y), float(v.z), float(s)};
}

inline NvFlexFloat4 make_float4(const NvFlexUint3 &v, NvFlexUint s) {
    return NvFlexFloat4{float(v.x), float(v.y), float(v.z), float(s)};
}

inline NvFlexFloat4 make_float4(const NvFlexUint4 &v) {
    return NvFlexFloat4{float(v.x), float(v.y), float(v.z), float(v.w)};
}

inline NvFlexFloat4 make_float4(const NvFlexInt4 &v) {
    return NvFlexFloat4{float(v.x), float(v.y), float(v.z), float(v.w)};
}

inline NvFlexInt3 make_int3(const NvFlexFloat3 &v) {
    return NvFlexInt3{int(v.x), int(v.y), int(v.z)};
}

inline NvFlexFloat3 make_float3(const NvFlexInt3 &v) {
    return NvFlexFloat3{float(v.x), float(v.y), float(v.z)};
}

inline NvFlexFloat3 make_float3(const NvFlexUint3 &v) {
    return NvFlexFloat3{float(v.x), float(v.y), float(v.z)};
}

inline NvFlexFloat3 make_float3(const NvFlexFloat4 &v) {
    return NvFlexFloat3{v.x, v.y, v.z};
}

inline unsigned int log2ui(unsigned int x) {
    for (int i = 0; i < 32; ++i) {
        if ((1 << i) >= x)
            return i;
    }
    return 0;
}

template <typename T>
inline T max3(T x, T y, T z) noexcept {
    return max(x, max(y, z));
}

NvFlexFloat3 normalize(const NvFlexFloat3 &v);

inline NvFlexFloat3 abs(const NvFlexFloat3 &v) noexcept {
    return NvFlexFloat3{abs(v.x), abs(v.y), abs(v.z)};
}

inline NvFlexFloat4 abs(const NvFlexFloat4 &v) noexcept {
    return NvFlexFloat4{abs(v.x), abs(v.y), abs(v.z), abs(v.w)};
}

inline NvFlexFloat3 min(const NvFlexFloat3 &v1, const NvFlexFloat3 &v2) noexcept {
    return NvFlexFloat3{min(v1.x, v2.x), min(v1.y, v2.y), min(v1.z, v2.z)};
}

inline NvFlexFloat4 min(const NvFlexFloat4 &v1, const NvFlexFloat4 &v2) noexcept {
    return NvFlexFloat4{min(v1.x, v2.x), min(v1.y, v2.y), min(v1.z, v2.z), min(v1.w, v2.w)};
}

inline NvFlexFloat3 max(const NvFlexFloat3 &v1, const NvFlexFloat3 &v2) noexcept {
    return NvFlexFloat3{max(v1.x, v2.x), max(v1.y, v2.y), max(v1.z, v2.z)};
}

inline NvFlexFloat4 max(const NvFlexFloat4 &v1, const NvFlexFloat4 &v2) noexcept {
    return NvFlexFloat4{max(v1.x, v2.x), max(v1.y, v2.y), max(v1.z, v2.z), max(v1.w, v2.w)};
}

inline NvFlexFloat4 floor(const NvFlexFloat4 &v) noexcept {
    return NvFlexFloat4{floor(v.x), floor(v.y), floor(v.z), floor(v.w)};
}

inline NvFlexFloat4 ceil(const NvFlexFloat4 &v) noexcept {
    return NvFlexFloat4{ceil(v.x), ceil(v.y), ceil(v.z), ceil(v.w)};
}

NvFlexFloat4 pow(const NvFlexFloat4 &a, const NvFlexFloat4 &b) noexcept;

NvFlexFloat4 pow(const NvFlexFloat4 &a, float b) noexcept;

float dot(const NvFlexFloat3 &a, const NvFlexFloat3 &b) noexcept;

inline float lerp(float a, float b, float t) noexcept {
    return (1.f - t) * a + t * b;
}

inline NvFlexFloat3 lerp(const NvFlexFloat3 a, const NvFlexFloat3 &b, float t) noexcept {
    return (1.f - t) * a + t * b;
}

// Matrix

NvFlexFloat4x4 transpose(const NvFlexFloat4x4 &m) noexcept;

float length(const NvFlexFloat3 &v) noexcept;

float vector3Length(const NvFlexFloat4 &v) noexcept;

NvFlexFloat4 transform4(const NvFlexFloat4 &v, const NvFlexFloat4x4 &m) noexcept;

NvFlexFloat4x4 matrixNormalize(const NvFlexFloat4x4 &m) noexcept;

NvFlexFloat4x4 inverse(const NvFlexFloat4x4 &m) noexcept;

NvFlexFloat4x4 identity() noexcept;

NvFlexFloat4x4 operator*(const NvFlexFloat4x4 &a, const NvFlexFloat4x4 &b) noexcept;

NvFlexFloat4x4 matrixTranslation(float x, float y, float z);

NvFlexFloat4x4 matrixScaling(float x, float y, float z);

NvFlexFloat3 boundsDiagonal(const NvFlexBounds3f &bounds);

float pi();

}  // namespace NvFlex

#endif /* NVFLEXMATH_H */
