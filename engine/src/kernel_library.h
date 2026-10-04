#ifndef ETHEREAL_PHYS_SRC_KERNEL_LIBRARY_H
#define ETHEREAL_PHYS_SRC_KERNEL_LIBRARY_H

// Module-private: how the device creates its kernel library. The implementation class stays in
// kernel_library.cpp.

#include <ethereal/phys/device.h>
#include <ethereal/phys/kernels.h>

namespace ephys::detail {

// What the library needs from the device that owns it. Not an interface that crosses a module boundary:
// the device implements it and the library calls it, both inside ethereal_phys.
class KernelLibraryHost {
 public:
    // The command list a recording method writes to: commandList itself, or the device's own (opened on
    // demand) when it is null. Null when there is none.
    virtual nvrhi::ICommandList* recordingList(nvrhi::ICommandList* commandList) noexcept = 0;

 protected:
    ~KernelLibraryHost() = default;
};

// Creates the kernel library of a device. Nothing is loaded yet. shaderPath may be null: every load then
// fails with a message that says so. Returns FS_OK and one reference in *ppLibrary.
nvrhi::FRESULT createKernelLibrary(nvrhi::IDevice* device, IPhysMessageCallback* callback, const char* shaderPath,
                                   const PhysDeviceInfo& info, KernelLibraryHost* host,
                                   IPhysKernelLibrary** ppLibrary) noexcept;

// The device is going away while the library may still be referenced: the library stops using the host.
void detachKernelLibrary(IPhysKernelLibrary* library) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_KERNEL_LIBRARY_H */
