#ifndef DEVICE_H
#define DEVICE_H
#include "Types.h"
#include "Object.h"
#include "NvFlexImpl.h"

struct NvFlexContext;

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D12Device;
struct ID3D12CommandQueue;

namespace NvFlex {

struct DeviceDesc {
    union {
        struct {
            ID3D11Device *device;
            ID3D11DeviceContext* renderContext;
            ID3D11DeviceContext* computeContext;
        } d3d11;

        struct {
            ID3D12Device* device;
            ID3D12CommandQueue* renderCmdQueue;
            ID3D12CommandQueue* computeCmdQueue;
        } d3d12;
    };
    bool useComputeQueue;

    NvFlexErrorCallback logger;
};

struct FlexDeviceCapabilities {
    char vendorName[256];
    NvFlexUint vendorID;
    bool isSHFLSupported;
    bool isFp32AtomicSupported;
    bool isSwizzleSupported;
    // NVIDIA shader-subpipe (SM) count, queried through NVAPI; -1 when unknown,
    // which is always the case on non-NVIDIA adapters.
    int smCount;
};

#define VENDOR_ID_IS_NV(id) ((id) == 4318)
#define VENDOR_ID_IS_AMD(id) ((id) == 4098 || (id) == 4130)

struct Device : Object {
    virtual void getDeviceCapabilities(FlexDeviceCapabilities* capabilities) = 0;

    virtual NvFlexContext* createContext() = 0;

    virtual void prepareContext(NvFlexContext* context, bool waitForPrevious) = 0;

    virtual void executeContext(NvFlexContext* context) = 0;

    virtual void waitForContext(NvFlexContext* context) = 0;
};

Device* createDeviceD3D11(const DeviceDesc &desc);
Device* createDeviceD3D12(const DeviceDesc &desc);

}

#endif /* DEVICE_H */
