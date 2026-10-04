#include "Types.h"
#include <cassert>
#include <cstdio>

namespace NvFlex {

struct DXGI_table {
    DXGI_table() {
        table[eNvFlexFormat_unknown] = {DXGI_FORMAT_UNKNOWN, 0};

        table[eNvFlexFormat_r32_float] = {DXGI_FORMAT_R32_FLOAT, 4};
        table[eNvFlexFormat_r32g32_float] = {DXGI_FORMAT_R32G32_FLOAT, 8};
        table[eNvFlexFormat_r32g32b32a32_float] = {DXGI_FORMAT_R32G32B32A32_FLOAT, 16};

        table[eNvFlexFormat_r16_float] = {DXGI_FORMAT_R16_FLOAT, 2};
        table[eNvFlexFormat_r16g16_float] = {DXGI_FORMAT_R16G16_FLOAT, 4};
        table[eNvFlexFormat_r16g16b16a16_float] = {DXGI_FORMAT_R16G16B16A16_FLOAT, 8};

        table[eNvFlexFormat_r32_uint] = {DXGI_FORMAT_R32_UINT, 4};
        table[eNvFlexFormat_r32g32_uint] = {DXGI_FORMAT_R32G32_UINT, 8};
        table[eNvFlexFormat_r32g32b32a32_uint] = {DXGI_FORMAT_R32G32B32A32_UINT, 16};

        table[eNvFlexFormat_r8_unorm] = {DXGI_FORMAT_R8_UNORM, 1};
        table[eNvFlexFormat_r8g8_unorm] = {DXGI_FORMAT_R8G8_UNORM, 2};
        table[eNvFlexFormat_r8g8b8a8_unorm] = {DXGI_FORMAT_R8G8B8A8_UNORM, 4};

        table[eNvFlexFormat_r16_unorm] = {DXGI_FORMAT_R16_UNORM, 2};
        table[eNvFlexFormat_r16g16_unorm] = {DXGI_FORMAT_R16G16_UNORM, 4};
        table[eNvFlexFormat_r16g16b16a16_unorm] = {DXGI_FORMAT_R16G16B16A16_UNORM, 8};

        table[eNvFlexFormat_d32_float] = {DXGI_FORMAT_D32_FLOAT, 4};
        table[eNvFlexFormat_d24_unorm_s8_uint] = {DXGI_FORMAT_D24_UNORM_S8_UINT, 4};

        table[eNvFlexFormat_r8_snorm] = {DXGI_FORMAT_R8_SNORM, 1};
        table[eNvFlexFormat_r8g8_snorm] = {DXGI_FORMAT_R8G8_SNORM, 2};
        table[eNvFlexFormat_r8g8b8a8_snorm] = {DXGI_FORMAT_R8G8B8A8_SNORM, 4};

        table[eNvFlexFormat_r32_typeless] = {DXGI_FORMAT_R32_TYPELESS, 4};
        table[eNvFlexFormat_r24_unorm_x8_typeless] = {DXGI_FORMAT_R24_UNORM_X8_TYPELESS, 4};
        table[eNvFlexFormat_r24g8_typeless] = {DXGI_FORMAT_R24G8_TYPELESS, 4};

        table[eNvFlexFormat_r16_typeless] = {DXGI_FORMAT_R16_TYPELESS, 2};
        table[eNvFlexFormat_d16_unorm] = {DXGI_FORMAT_D16_UNORM, 2};
    }
    struct Element {
        DXGI_FORMAT format;
        uint32_t sizeInBytes;
    } table[26];
} const g_DXGI_Table;

struct NvFlexFormatTable {
    NvFlexFormatTable() {
        table[DXGI_FORMAT_UNKNOWN] = eNvFlexFormat_unknown;
        memset(&this->table[1], 0, 69 * sizeof(NvFlexFormat));
        table[DXGI_FORMAT_R32_FLOAT] = eNvFlexFormat_r32_float;
        table[DXGI_FORMAT_R32G32_FLOAT] = eNvFlexFormat_r32g32_float;
        table[DXGI_FORMAT_R32G32B32A32_FLOAT] = eNvFlexFormat_r32g32b32a32_float;
        table[DXGI_FORMAT_R16_FLOAT] = eNvFlexFormat_r16_float;
        table[DXGI_FORMAT_R16G16_FLOAT] = eNvFlexFormat_r16g16_float;
        table[DXGI_FORMAT_R16G16B16A16_FLOAT] = eNvFlexFormat_r16g16b16a16_float;
        table[DXGI_FORMAT_R32_UINT] = eNvFlexFormat_r32_uint;
        table[DXGI_FORMAT_R32G32_UINT] = eNvFlexFormat_r32g32_uint;
        table[DXGI_FORMAT_R32G32B32A32_UINT] = eNvFlexFormat_r32g32b32a32_uint;
        table[DXGI_FORMAT_R8_UNORM] = eNvFlexFormat_r8_unorm;
        table[DXGI_FORMAT_R8G8_UNORM] = eNvFlexFormat_r8g8_unorm;
        table[DXGI_FORMAT_R8G8B8A8_UNORM] = eNvFlexFormat_r8g8b8a8_unorm;
        table[DXGI_FORMAT_R16_UNORM] = eNvFlexFormat_r16_unorm;
        table[DXGI_FORMAT_R16G16_UNORM] = eNvFlexFormat_r16g16_unorm;
        table[DXGI_FORMAT_R16G16B16A16_UNORM] = eNvFlexFormat_r16g16b16a16_unorm;
        table[DXGI_FORMAT_D32_FLOAT] = eNvFlexFormat_d32_float;
        table[DXGI_FORMAT_D24_UNORM_S8_UINT] = eNvFlexFormat_d24_unorm_s8_uint;
        table[DXGI_FORMAT_R8_SNORM] = eNvFlexFormat_r8_snorm;
        table[DXGI_FORMAT_R8G8_SNORM] = eNvFlexFormat_r8g8_snorm;
        table[DXGI_FORMAT_R8G8B8A8_SNORM] = eNvFlexFormat_r8g8b8a8_snorm;
        table[DXGI_FORMAT_R32_TYPELESS] = eNvFlexFormat_r32_typeless;
        table[DXGI_FORMAT_R24_UNORM_X8_TYPELESS] = eNvFlexFormat_r24_unorm_x8_typeless;
        table[DXGI_FORMAT_R24G8_TYPELESS] = eNvFlexFormat_r24g8_typeless;
        table[DXGI_FORMAT_R16_TYPELESS] = eNvFlexFormat_r16_typeless;
        table[DXGI_FORMAT_D16_UNORM] = eNvFlexFormat_d16_unorm;
    }

    NvFlexFormat table[70];
} const g_NvFormatTable;

uint32_t getFormatSizeInBytes(NvFlexFormat format) {
    return g_DXGI_Table.table[format].sizeInBytes;
}

bool isFormatTypeless(NvFlexFormat format) {
    return format == eNvFlexFormat_r32_typeless || format == eNvFlexFormat_r16_typeless ||
           format == eNvFlexFormat_r24_unorm_x8_typeless ||
           format == eNvFlexFormat_r24g8_typeless;
}

DXGI_FORMAT convertToDXGI(NvFlexFormat format) {
    return g_DXGI_Table.table[format].format;
}

NvFlexFormat convertToNvFlex(DXGI_FORMAT format) {
    return g_NvFormatTable.table[format];
}

NvFlexDim getTileDim(NvFlexFormat format) {
    uint32_t bytesPerPixel = getFormatSizeInBytes(format);

    switch (bytesPerPixel) {
        case 1:
            return {64, 32, 32};
        case 2:
            return {32, 32, 32};
        case 4:
            return {32, 32, 16};
        case 8:
            return {32, 16, 16};
        case 16:
            return {16, 16, 16};
        default:
            return {0, 0, 0};
    }
}

static const char* GetErrorDesc(CommonErrorType errType) {
    switch (errType) {
        default:
        case COMMON_ERROR_UNKNOWN:
            return "unknown";
        case COMMON_ERROR_NOT_IMPLEMENTED:
            return "implementation does not exist";
        case COMMON_ERROR_INDEX_OUT_OF_RANGE:
            return "index out of range";
        case COMMON_RESOURCE_CREATE_FAILED:
            return "resource create failed";
        case COMMON_RESOURCE_UPLOAD_FAILED:
            return "resource upload failed";
        case COMMON_RESOURCE_READBACK_FAILED:
            return "resource readback failed";
    }
}

void HandleError(const char* file, uint32_t line, const char* pos,
                 CommonErrorType errType) {
    fprintf(stderr, "[NvFlex] %s:%u: \"%s\": %s.\n", file, line, pos,
            GetErrorDesc(errType));
    assert(0);
}

}  // namespace NvFlex