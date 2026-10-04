#ifndef ETHEREAL_PHYS_FRAMEWORK_SRC_VALIDATION_H
#define ETHEREAL_PHYS_FRAMEWORK_SRC_VALIDATION_H

// ValidationMonitor: counts what the graphics stack and the engine complain about, so that an unattended run can
// fail on it.
//
// - nvrhi's messages (its validation layer with --debug): through the IMessageCallback given to the device.
// - Donut's log: errors, and every warning (the Vulkan validation layer reports through it).
// - The D3D11 / D3D12 debug layer (with --debug): Donut's device managers only send it to the debugger, and the
//   D3D12 one breaks on errors; the monitor reads the info queue of the device instead (no break), after every
//   frame. As tests/d3d_messages.h of the engine.
// - The engine's message callback: errors and warnings.
//
// Every counted message is printed to stdout.

#include <ethereal/phys/device.h>

#include <nvrhi/nvrhi.h>

#include <atomic>

namespace ephys::framework {

class ValidationMonitor {
 public:
    ValidationMonitor();
    ~ValidationMonitor();

    ValidationMonitor(const ValidationMonitor&) = delete;
    ValidationMonitor& operator=(const ValidationMonitor&) = delete;

    nvrhi::IMessageCallback* GetRhiCallback() const;

    // Starts reading the debug layer of a D3D device (nothing for Vulkan).
    void AttachDevice(nvrhi::IDevice* device);
    void DetachDevice();

    // Reads the debug layer's new messages. Returns the number of problems so far.
    int Poll();

    void OnPhysMessage(PhysMessageSeverity severity, const char* text);

    [[nodiscard]] int GetProblems() const { return m_Problems.load(); }

 private:
    struct Impl;
    Impl* m_Impl;
    std::atomic<int> m_Problems{0};
};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_SRC_VALIDATION_H */
