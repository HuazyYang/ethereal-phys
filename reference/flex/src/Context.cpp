#include "Context.h"
#include "ClientHelper.h"

namespace NvFlex {

extern int64_t FlexDeferredReleaseD3D11(float timeoutMS);

extern int64_t FlexDeferredReleaseD3D12(float timeoutMS);

uint64_t Context::getGPUBytesUsed() {
    return 0;
}

void Context::flushRequestPush() {
    m_flushRequestPending = 1;
}
bool Context::flushRequestPop() {
    bool prev = m_flushRequestPending;
    m_flushRequestPending = false;
    return prev;
}

void Context::profileGroupBegin(const wchar_t *) {}
void Context::profileGroupEnd() {
}
void Context::profileItemBegin(const wchar_t *) {
}
void Context::profileItemEnd() {
}

uint64_t FlexDeferredRelease(float timeoutMS) {
    uint64_t sum = FlexDeferredReleaseD3D11(timeoutMS);
    sum += FlexDeferredReleaseD3D12(timeoutMS);
    return sum;
}
IDXGIFactory1 *getDXGIFactoryD3D(IDXGIAdapter1 *pAdapter1) {
    IDXGIFactory1 *pFactory1 = 0;
    if (pAdapter1) {
        IDXGIFactory *pFactory = 0;
        if (FAILED(pAdapter1->GetParent(IID_PPV_ARGS(&pFactory)))) return 0;

        if (FAILED(pFactory->QueryInterface(IID_PPV_ARGS(&pFactory1)))) {
            SafeRelease(pFactory);
            return 0;
        }
        SafeRelease(pFactory);
    }
    return pFactory1;
}

Context::Context(Device *deviceImpl): m_deviceImpl{deviceImpl} {}

bool Context::verifyMapAccess(NvFlexMemoryType memType, NvFlexCpuAccessMode cpuAccess,
                              NvFlexStagingCpuAccessFlags *accessFlags) {
    // sanity check
    if (memType == eNvFlexMemoryType_default) {
        NVFLEX_ASSERT(0 && "default buffer can not be mapped");
        return false;
    }
    if (memType == eNvFlexMemoryType_readback) {
        if (accessFlags) {
            NvFlexStagingCpuAccessFlags accessFlags2 = *accessFlags;
            if ((accessFlags2 & eNvFlexStagingCpuAccess_read) && !(cpuAccess & eNvFlexCpuAccessMode_read)) {
                NVFLEX_ASSERT(
                    0 && "map read staging buffer is not allowed(access right denied)");
                return false;
            }
            if ((accessFlags2 & eNvFlexStagingCpuAccess_write) && !(cpuAccess & eNvFlexCpuAccessMode_write)) {
                NVFLEX_ASSERT(
                    0 && "map write staging buffer is not allowed(access right denied)");
                return false;
            }

            if (!(accessFlags2 & eNvFlexStagingCpuAccess_readwrite)) {
                NVFLEX_ASSERT(
                    0 &&
                    "map staging buffer without read or right access mode is not allowed");
                return false;
            }
        }
    }

    return true;
}

}  // namespace NvFlex