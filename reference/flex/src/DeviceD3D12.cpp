#include "Device.h"
#include "ContextD3D12.h"
#include <d3d12.h>
#include <nvapi.h>
#include <nvShaderExtnEnums.h>
#include <amd_ags.h>
#include <string>

namespace NvFlex {

struct DeviceD3D12 : Device {
    void getDeviceCapabilities(FlexDeviceCapabilities *capabilities) override;
    NvFlexContext *createContext() override;
    void prepareContext(NvFlexContext *context, bool waitForPrevious) override;
    void executeContext(NvFlexContext *context) override;
    void waitForContext(NvFlexContext *context) override;

    // details
    DeviceD3D12(const DeviceDesc &desc);
    ~DeviceD3D12();
    bool init();

    bool queryDeviceCapabilities();

    DeviceDesc m_desc;
    AGSContext *m_agsContext;

    ID3D12Device *m_d3d12Device;
    ID3D12CommandQueue *m_computeQueue;
    ComPtr<ID3D12CommandAllocator> m_computeCmdAllocator;
    ComPtr<ID3D12GraphicsCommandList> m_computeCmdList;
    ComPtr<ID3D12Fence> m_d3d12Fence;
    HANDLE m_fenceEvent;

    std::string m_vendorName;
    NvFlexUint m_vendorID;
    NvFlexUint m_SMcount;
    bool m_isSHFLSupported;
    bool m_isFp32AtomicSupported;
    bool m_isSwizzleSupported;

    NvFlexUint64 m_computeQueueNextSyncPoint;
};

Device *createDeviceD3D12(const DeviceDesc &desc) {
    auto device = new DeviceD3D12(desc);
    if(!device->init()) {
        device->release();
        device = nullptr;
    }
    return device;
}

void DeviceD3D12::getDeviceCapabilities(FlexDeviceCapabilities *capabilities) {
    strcpy(capabilities->vendorName, m_vendorName.c_str());
    capabilities->vendorID = m_vendorID;
    capabilities->isSHFLSupported = m_isSHFLSupported;
    capabilities->isFp32AtomicSupported = m_isFp32AtomicSupported;
    capabilities->isSwizzleSupported = m_isSwizzleSupported;
    capabilities->smCount = int(m_SMcount);
}

NvFlexContext *DeviceD3D12::createContext() {
    NvFlexContextDescD3D12 desc = {};
    desc.device = m_d3d12Device;
    desc.commandQueue = m_computeQueue;
    desc.commandList = m_computeCmdList.Get();
    desc.commandQueueFence = m_d3d12Fence.Get();
    desc.nextFenceValue = 0;

    return new ContextD3D12(this, &desc);
}

void DeviceD3D12::prepareContext(NvFlexContext *context, bool waitForPrevious) {
    HRESULT hr;

    if (waitForPrevious)
        waitForContext(context);

    NvFlexContextDescD3D12 desc;
    NvFlexUpdateContextDescD3D12(context, &desc);

    desc.lastFenceCompleted = desc.commandQueueFence->GetCompletedValue();
    desc.nextFenceValue = desc.lastFenceCompleted + 1;

    NvFlexUpdateContextD3D12(context, &desc);

    hr = m_computeCmdAllocator->Reset();
    NVFLEX_ASSERT(SUCCEEDED(hr));
    hr = desc.commandList->Reset(m_computeCmdAllocator.Get(), nullptr);
    NVFLEX_ASSERT(SUCCEEDED(hr));
}

void DeviceD3D12::executeContext(NvFlexContext *context) {
    HRESULT hr;
    NvFlexContextDescD3D12 desc;
    NvFlexUpdateContextDescD3D12(context, &desc);

    hr = desc.commandList->Close();
    NVFLEX_ASSERT(SUCCEEDED(hr));
    ID3D12CommandList *cmdLists[] = { desc.commandList };
    desc.commandQueue->ExecuteCommandLists(1, cmdLists);
    hr = desc.commandQueue->Signal(desc.commandQueueFence, desc.nextFenceValue);
    NVFLEX_ASSERT(SUCCEEDED(hr));
}

void DeviceD3D12::waitForContext(NvFlexContext *context) {
    HRESULT hr;
    DWORD rc;
    NvFlexContextDescD3D12 desc;
    NvFlexUpdateContextDescD3D12(context, &desc);

    if (desc.commandQueueFence->GetCompletedValue() < desc.nextFenceValue) {
        if (FAILED(desc.commandQueueFence->SetEventOnCompletion(desc.nextFenceValue, m_fenceEvent))) {
            NVFLEX_ASSERT(0);
        }

        rc = WaitForSingleObject(m_fenceEvent, INFINITE);
        if (rc != WAIT_OBJECT_0) {
            NVFLEX_ASSERT(0);
        }
    }
}

DeviceD3D12::DeviceD3D12(const DeviceDesc &desc)
    : m_desc(desc),
      m_d3d12Device{desc.d3d12.device},
      m_computeQueue{nullptr},
      m_fenceEvent{},
      m_agsContext{},
      m_vendorName{},
      m_vendorID{},
      m_isSHFLSupported{},
      m_isFp32AtomicSupported{},
      m_isSwizzleSupported{} {}

DeviceD3D12::~DeviceD3D12() {
    if(m_fenceEvent) CloseHandle(m_fenceEvent);
    if (m_agsContext) {
        agsDriverExtensionsDX12_DeInit(m_agsContext);
        agsDeInit(m_agsContext);
    }

    m_computeQueue->Release();
}

bool DeviceD3D12::init() {
    HRESULT hr;

    if(!queryDeviceCapabilities())
        return false;

    bool useComputeQueue = false;
    if(m_desc.useComputeQueue) {
        if(VENDOR_ID_IS_NV(m_vendorID)) {
            NvU32 driverVersion;
            char driverName[64];
            if(!NvAPI_SYS_GetDriverAndBranchVersion(&driverVersion, driverName)) {
                if (driverVersion < 38460) {
                    FLEX_IMPL_LOG_WARNING(
                        m_desc.logger,
                        "Warning: D3D12 Async compute was requested, but driver lacks "
                        "async compute features required by Flex (%s %d)", driverName, driverVersion);
                }

                useComputeQueue = 1;

            } else {
                FLEX_IMPL_LOG_WARNING(m_desc.logger, "Can not query nvidia driver version and name");
            }
        } else if(VENDOR_ID_IS_AMD(m_vendorID)) {
            useComputeQueue = 1;
        }
    }

    // Select command queue
    D3D12_COMMAND_QUEUE_DESC queueDesc = {};
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

    if(!m_desc.d3d12.renderCmdQueue || useComputeQueue) {
        if(m_desc.d3d12.computeCmdQueue && useComputeQueue) {
            m_computeQueue = m_desc.d3d12.computeCmdQueue;
            m_computeQueue->AddRef();
        } else {
            queueDesc.Type = D3D12_COMMAND_LIST_TYPE_COMPUTE;

            if(FAILED(hr = m_d3d12Device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_computeQueue)))) {
                NVFLEX_ASSERT(0 && "Failed to create compute command queue");
            }
        }
    } else {
        m_computeQueue = m_desc.d3d12.renderCmdQueue;
        m_computeQueue->AddRef();
    }

    if (FAILED(hr = m_d3d12Device->CreateCommandAllocator(
                   queueDesc.Type, IID_PPV_ARGS(&m_computeCmdAllocator)))) {
        NVFLEX_ASSERT(0);
        return false;
    }

    if (FAILED(hr = m_d3d12Device->CreateCommandList(0, queueDesc.Type,
                                                     m_computeCmdAllocator.Get(), nullptr,
                                                     IID_PPV_ARGS(&m_computeCmdList)))) {
        NVFLEX_ASSERT(0);
        return false;
    }

    if(FAILED(m_computeCmdList->Close())) {
        NVFLEX_ASSERT(0);
        return false;
    }

    m_computeCmdAllocator->SetName(L"DeviceD3D12::m_computeCmdAllocator");
    m_computeCmdList->SetName(L"DeviceD3D12::m_computeCmdList");

    if (FAILED(hr = m_d3d12Device->CreateFence(0, D3D12_FENCE_FLAG_SHARED,
                                               IID_PPV_ARGS(&m_d3d12Fence)))) {
        NVFLEX_ASSERT(0);
        return false;
    }

    m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_fenceEvent == nullptr) {
        NVFLEX_ASSERT(0);
        return false;
    }

    {
        if (m_vendorID == 4318) {
            NvAPI_Initialize();
            NvAPI_D3D12_IsNvShaderExtnOpCodeSupported(m_d3d12Device, NV_EXTN_OP_SHFL,
                                                      &m_isSHFLSupported);
            NvAPI_D3D12_IsNvShaderExtnOpCodeSupported(m_d3d12Device, NV_EXTN_OP_FP32_ATOMIC,
                                                      &m_isFp32AtomicSupported);
        } else if (m_vendorID == 4098 || m_vendorID == 4130) {
            AGSGPUInfo agsGPUInfo;
            agsInit(&m_agsContext, nullptr, &agsGPUInfo);
            unsigned int extensionSupported;
            if (agsDriverExtensionsDX12_Init(m_agsContext, m_d3d12Device,
                                             &extensionSupported) == AGS_SUCCESS &&
                ((extensionSupported & AGS_DX11_EXTENSION_INTRINSIC_SWIZZLE) != 0 ||
                 ((extensionSupported & AGS_DX11_EXTENSION_DEPTH_BOUNDS_TEST) != 0))) {
                m_isSwizzleSupported = 1;
            }
        } else {
        }
    }

    return true;
}

bool DeviceD3D12::queryDeviceCapabilities() {
    HRESULT hr;

    DXGI_ADAPTER_DESC1 adapterDesc;
    // Query adapter vendor ID and name
    {
        ComPtr<IDXGIFactory1> dxgiFactory;
        HMODULE hdxgiLib = GetModuleHandleW(L"dxgi.dll");
        NVFLEX_ASSERT(hdxgiLib != NULL);
        auto lpfnCreateDXGIFactory1 = (HRESULT(WINAPI *)(REFIID, void **))GetProcAddress(
            hdxgiLib, "CreateDXGIFactory1");
        NVFLEX_ASSERT(lpfnCreateDXGIFactory1);

        if (FAILED(hr = lpfnCreateDXGIFactory1(IID_PPV_ARGS(&dxgiFactory)))) {
            NVFLEX_ASSERT(0);
            return false;
        }

        LUID adapterLuid = m_d3d12Device->GetAdapterLuid();

        ComPtr<IDXGIAdapter1> dxgiAdapter;
        for (UINT adapterIdx = 0;
             SUCCEEDED(dxgiFactory->EnumAdapters1(adapterIdx, &dxgiAdapter));
             ++adapterIdx) {
            dxgiAdapter->GetDesc1(&adapterDesc);
            if (memcmp(&adapterLuid, &adapterDesc.AdapterLuid, sizeof(adapterLuid)) == 0)
                break;

            dxgiAdapter = nullptr;
        }

        if (!dxgiAdapter)
            return false;

        m_vendorID = adapterDesc.VendorId;

        for (int i = 0; adapterDesc.Description[i]; ++i)
            m_vendorName.push_back(adapterDesc.Description[i]);
        auto pos = m_vendorName.find_last_not_of(" \n\r\t");
        m_vendorName.erase(pos + 1);
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