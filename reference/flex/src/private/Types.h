#ifndef TYPES_H
#define TYPES_H
#include <nvflex/NvFlexTypes.h>
#include <cstdint>
#include <dxgi.h>
#include <type_traits>
#include <cassert>

namespace NvFlex {

#define NVFLEX_ALIGN(n) __declspec(align(n))

using BYTE = uint8_t;

template <typename T, size_t N>
char (*countofHelper(const T (&Array)[N]))[N];

#define countof(Array) sizeof(*NvFlex::countofHelper(Array))

template <int N, typename T, std::enable_if_t<(N & (N - 1)) == 0, int> = 0>
inline T alignUp(T ptr) {
    static_assert((N & (N - 1)) == 0, "alignUp dividend must be power of 2");
    constexpr T mask = ~T(N - 1);
    return (ptr + T(N - 1)) & mask;
}

template <int N, typename T, std::enable_if_t<(N & (N - 1)) == 0, int> = 0>
inline T divCeil(T sz) {
    return (sz + T(N - 1))  / T(N);
}

template <typename T>
void swap(T& left, T& right) {
    T temp = std::move(left);
    left = std::move(right);
    right = std::move(temp);
}

uint32_t getFormatSizeInBytes(NvFlexFormat format);
bool isFormatTypeless(NvFlexFormat format);

DXGI_FORMAT convertToDXGI(NvFlexFormat format);

NvFlexFormat convertToNvFlex(DXGI_FORMAT format);

NvFlexDim getTileDim(NvFlexFormat format);

///
/// Error handling
///

#if _MSC_VER
#define NVFLEX_FUNCSIG __FUNCSIG__
#else
#define NVFLEX_FUNCSIG __PRETTY_FUNCTION__
#endif

enum CommonErrorType {
    COMMON_ERROR_UNKNOWN = 0,
    COMMON_ERROR_GENERIC = 1,
    COMMON_ERROR_NOT_IMPLEMENTED = 1,
    COMMON_ERROR_INDEX_OUT_OF_RANGE = 2,
    COMMON_RESOURCE_CREATE_FAILED = 3,
    COMMON_RESOURCE_UPLOAD_FAILED = 4,
    COMMON_RESOURCE_READBACK_FAILED = 5
};

#define NVFLEX_HANDLE_ERROR(type) \
    NvFlex::HandleError(__FILE__, __LINE__, NVFLEX_FUNCSIG, type)

#define NVFLEX_NOT_IMPLEMENTED_ERROR()                      \
    NvFlex::HandleError(__FILE__, __LINE__, NVFLEX_FUNCSIG, \
                        NvFlex::COMMON_ERROR_NOT_IMPLEMENTED)

#define NVFLEX_INDEX_OUT_OF_RANGE_ERROR()                   \
    NvFlex::HandleError(__FILE__, __LINE__, NVFLEX_FUNCSIG, \
                        NvFlex::COMMON_ERROR_INDEX_OUT_OF_RANGE)

#define NVFLEX_ASSERT(expr) assert(expr)

#define NVFLEX_VERIFY(expr)            \
    do {                               \
        NvFlexResult rc = (expr);      \
        if (rc != eNvFlexSuccess) {      \
            NVFLEX_ASSERT(0 && #expr); \
        }                              \
    } while(0)

void HandleError(const char* file, uint32_t line, const char* pos, CommonErrorType errType);

}  // namespace NvFlex

#include "FlexMath.h"

using int2 = NvFlexInt2;
using int3 = NvFlexInt3;
using int4 = NvFlexInt4;
using uint2 = NvFlexUint2;
using uint3 = NvFlexUint3;
using uint4 = NvFlexUint4;
using float2 = NvFlexFloat2;
using float3 = NvFlexFloat3;
using float4 = NvFlexFloat4;

using uint = NvFlexUint;

#endif /* TYPES_H */
