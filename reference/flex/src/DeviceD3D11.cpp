#include "Device.h"
#include <dxgi1_2.h>
#include <d3d11_2.h>
#include <nvflex/NvFlexContext.h>
#include "ContextD3D11.h"
#include <ClientHelper.h>
#include <nvapi.h>
#include <nvShaderExtnEnums.h>
#include <amd_ags.h>
#include <string>

extern "C" NvAPI_Status NvAPI_D3D11_QueryAsyncComputeHint(void *pDev, void *hint);

namespace NvFlex {

struct DeviceD3D11 : Device {
    void getDeviceCapabilities(FlexDeviceCapabilities *capabilities) override;
    NvFlexContext *createContext() override;
    void prepareContext(NvFlexContext *context, bool waitForPrevious) override;
    void executeContext(NvFlexContext *context) override;
    void waitForContext(NvFlexContext *context) override;

    // details
    DeviceD3D11(const DeviceDesc &desc);
    ~DeviceD3D11();
    bool init();

    bool queryDeviceCapabilities();

    DeviceDesc m_desc;
    ID3D11Device *m_d3d11Device;
    ID3D11DeviceContext *m_computeContext;
    AGSContext *m_agsContext;
    std::string m_vendorName;
    NvFlexUint m_vendorID;
    NvFlexUint m_SMcount;
    bool m_useD3D11AsyncCompute;

    bool m_isSHFLSupported;
    bool m_isFp32AtomicSupported;
    bool m_isSwizzleSupported;
};

Device *createDeviceD3D11(const DeviceDesc &desc) {
    auto ctx = new DeviceD3D11(desc);
    if(!ctx->init()) {
        ctx->release();
        ctx = nullptr;
    }
    return ctx;
}

void DeviceD3D11::getDeviceCapabilities(FlexDeviceCapabilities *capabilities){
    strcpy(capabilities->vendorName, m_vendorName.c_str());
    capabilities->vendorID = m_vendorID;
    capabilities->isSHFLSupported = m_isSHFLSupported;
    capabilities->isFp32AtomicSupported = m_isFp32AtomicSupported;
    capabilities->isSwizzleSupported = m_isSwizzleSupported;
    capabilities->smCount = int(m_SMcount);
}

NvFlexContext *DeviceD3D11::createContext() {
    NvFlexContextDescD3D11 desc = {};
    desc.device = m_d3d11Device;
    desc.deviceContext = m_computeContext;
    return new ContextD3D11(this, &desc);
}

void DeviceD3D11::prepareContext(NvFlexContext *context, bool waitForPrevious) {}

void DeviceD3D11::executeContext(NvFlexContext *context) {}

void DeviceD3D11::waitForContext(NvFlexContext *context) {}

DeviceD3D11::DeviceD3D11(const DeviceDesc &desc)
    : m_desc(desc),
      m_d3d11Device{m_desc.d3d11.device},
      m_computeContext{nullptr},
      m_agsContext{},
      m_vendorName{},
      m_vendorID{},
      m_SMcount{-1u},
      m_useD3D11AsyncCompute{false},
      m_isSHFLSupported{},
      m_isFp32AtomicSupported{},
      m_isSwizzleSupported{} {}

DeviceD3D11::~DeviceD3D11() {
    if(m_agsContext) {
        agsDriverExtensionsDX11_DeInit(m_agsContext);
        agsDeInit(m_agsContext);
    }
    SafeRelease(m_computeContext);
}

bool DeviceD3D11::init() {

    if(!queryDeviceCapabilities())
        return false;

    m_computeContext = m_desc.d3d11.renderContext;
    m_computeContext->AddRef();

    if(m_vendorID == 4318) {
        NvAPI_Initialize();
        NvAPI_D3D11_IsNvShaderExtnOpCodeSupported(m_d3d11Device, NV_EXTN_OP_SHFL,
                                                  &m_isSHFLSupported);
        NvAPI_D3D11_IsNvShaderExtnOpCodeSupported(m_d3d11Device, NV_EXTN_OP_FP32_ATOMIC,
                                                  &m_isFp32AtomicSupported);
    } else if(m_vendorID == 4098 || m_vendorID == 4130) {
        AGSGPUInfo agsGPUInfo;
        agsInit(&m_agsContext, nullptr, &agsGPUInfo);
        unsigned int extensionSupported;
        if (agsDriverExtensionsDX11_Init(m_agsContext, m_d3d11Device, 7,
                                         &extensionSupported) == AGS_SUCCESS &&
            ((extensionSupported & AGS_DX11_EXTENSION_INTRINSIC_SWIZZLE) != 0 ||
            ((extensionSupported & AGS_DX11_EXTENSION_DEPTH_BOUNDS_TEST) != 0))) {
            m_isSwizzleSupported = 1;
        }
    } else {
    }

    return true;
}

bool DeviceD3D11::queryDeviceCapabilities() {
    HRESULT hr;

    DXGI_ADAPTER_DESC1 adapterDesc;
    // Query adapter vendor ID and name
    {
        ComPtr<IDXGIDevice> dxgiDevice;

        if (FAILED(hr = m_d3d11Device->QueryInterface(IID_PPV_ARGS(&dxgiDevice)))) {
            NVFLEX_ASSERT(0);
            return false;
        }

        ComPtr<IDXGIAdapter> dxgiAdapter;
        dxgiDevice->GetAdapter(&dxgiAdapter);
        ComPtr<IDXGIAdapter1> dxgiAdapter1;
        if (FAILED(hr = dxgiAdapter->QueryInterface(IID_PPV_ARGS(&dxgiAdapter1)))) {
            NVFLEX_ASSERT(0);
            return false;
        }

        dxgiAdapter1->GetDesc1(&adapterDesc);
        {
            for (int i = 0; adapterDesc.Description[i]; ++i)
                m_vendorName.push_back(adapterDesc.Description[i]);
            auto pos = m_vendorName.find_last_not_of(" \n\r\t");
            m_vendorName.erase(pos + 1);
        }
        m_vendorID = adapterDesc.VendorId;
    }

        uint smCount = -1;

    // query SM count
    {
        if (VENDOR_ID_IS_NV(m_vendorID)) {
            NvPhysicalGpuHandle nvGPUHandle[64];
            NvU32 cnt = countof(nvGPUHandle);
            NvAPI_Status ret = NvAPI_EnumPhysicalGPUs(nvGPUHandle, &cnt);
            if (ret) {
                char msg[64];
                NvAPI_GetErrorMessage(ret, msg);
                FLEX_IMPL_LOG_ERROR(m_desc.logger,
                                    "NvAPI can not enumerate physical device: %s", msg);
                return false;
            } else {
                NvU32 deviceId, subsystemID, revisionId, extDeviceId;
                for (NvU32 i = 0; i < cnt; ++i) {
                    if (!NvAPI_GPU_GetPCIIdentifiers(nvGPUHandle[i], &deviceId,
                                                     &subsystemID, &revisionId,
                                                     &extDeviceId) &&
                        adapterDesc.DeviceId == extDeviceId &&
                        adapterDesc.SubSysId == subsystemID &&
                        adapterDesc.Revision == revisionId) {
                        NvAPI_GPU_GetShaderSubPipeCount(nvGPUHandle[i], (NvU32 *)&smCount);
                        break;
                    }
                }
            }
        }
    }

    m_SMcount = smCount;

    return true;
}

}  // namespace NvFlex