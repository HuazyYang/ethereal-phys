// Test helper: the messages of the D3D11 / D3D12 debug layer.
//
// Donut's device managers turn the debug layer on (DeviceCreationParameters::enableDebugRuntime) but do
// not route its messages anywhere a test can see: they only reach the debugger output, and in a Debug
// build the D3D12 device manager makes an error a breakpoint. The Vulkan validation layer, by contrast,
// reports through Donut's log. This reads the info queue of the device directly, so that a debug-layer
// warning or error fails a device test in every configuration.
//
//     D3DMessages d3d(device);          // right after the device was created
//     ...
//     CHECK(d3d.problems() == 0);       // prints what it finds

#ifndef ETHEREAL_PHYS_TESTS_D3D_MESSAGES_H
#define ETHEREAL_PHYS_TESTS_D3D_MESSAGES_H

#include <nvrhi/nvrhi.h>

#include <cstdio>
#include <vector>

#ifdef _WIN32
#include <d3d11.h>
#include <d3d11sdklayers.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
// <winuser.h> may turn GetMessage into GetMessageA / GetMessageW. The info queue method of that name is
// then declared under the macro, and is called under it below: the macro must stay as it is.
#endif

class D3DMessages {
 public:
    explicit D3DMessages(nvrhi::IDevice* device) {
#ifdef _WIN32
        if (device->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D12) {
            if (auto* native = static_cast<ID3D12Device*>(device->getNativeObject(nvrhi::ObjectTypes::D3D12_Device)))
                native->QueryInterface(IID_PPV_ARGS(&m_Queue12));
            if (m_Queue12) {
                // Report, do not break: a test prints what went wrong and goes on.
                m_Queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, FALSE);
                m_Queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, FALSE);
                m_Queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, FALSE);
                m_Queue12->ClearStoredMessages();
            }
        } else if (device->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D11) {
            if (auto* native = static_cast<ID3D11Device*>(device->getNativeObject(nvrhi::ObjectTypes::D3D11_Device)))
                native->QueryInterface(IID_PPV_ARGS(&m_Queue11));
            if (m_Queue11) m_Queue11->ClearStoredMessages();
        }
#else
        (void)device;
#endif
    }

    ~D3DMessages() {
#ifdef _WIN32
        if (m_Queue12) m_Queue12->Release();
        if (m_Queue11) m_Queue11->Release();
#endif
    }

    D3DMessages(const D3DMessages&) = delete;
    D3DMessages& operator=(const D3DMessages&) = delete;

    // True when the debug layer of the device can be read (a D3D device with the debug runtime).
    bool attached() const {
#ifdef _WIN32
        return m_Queue12 || m_Queue11;
#else
        return false;
#endif
    }

    // The number of corruption, error and warning messages stored since the last call; prints them.
    int problems() {
        int count = 0;
#ifdef _WIN32
        std::vector<char> storage;
        if (m_Queue12) {
            const UINT64 messages = m_Queue12->GetNumStoredMessagesAllowedByRetrievalFilter();
            for (UINT64 i = 0; i < messages; ++i) {
                SIZE_T size = 0;
                m_Queue12->GetMessage(i, nullptr, &size);
                storage.resize(size);
                auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
                if (FAILED(m_Queue12->GetMessage(i, message, &size))) continue;
                if (message->Severity > D3D12_MESSAGE_SEVERITY_WARNING) continue;  // info, message
                ++count;
                std::printf("d3d12 debug layer: %s\n", message->pDescription);
            }
            m_Queue12->ClearStoredMessages();
        }
        if (m_Queue11) {
            const UINT64 messages = m_Queue11->GetNumStoredMessagesAllowedByRetrievalFilter();
            for (UINT64 i = 0; i < messages; ++i) {
                SIZE_T size = 0;
                m_Queue11->GetMessage(i, nullptr, &size);
                storage.resize(size);
                auto* message = reinterpret_cast<D3D11_MESSAGE*>(storage.data());
                if (FAILED(m_Queue11->GetMessage(i, message, &size))) continue;
                if (message->Severity > D3D11_MESSAGE_SEVERITY_WARNING) continue;
                ++count;
                std::printf("d3d11 debug layer: %s\n", message->pDescription);
            }
            m_Queue11->ClearStoredMessages();
        }
#endif
        return count;
    }

 private:
#ifdef _WIN32
    ID3D12InfoQueue* m_Queue12 = nullptr;
    ID3D11InfoQueue* m_Queue11 = nullptr;
#endif
};

#endif /* ETHEREAL_PHYS_TESTS_D3D_MESSAGES_H */
