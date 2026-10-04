#include "validation.h"

#include <donut/core/log.h>
#include <nvrhi/core/foundation.h>

#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

#ifdef _WIN32
#include <d3d11.h>
#include <d3d11sdklayers.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#endif

namespace ephys::framework {

namespace {

class RhiMessages final : public nvrhi::ObjectImpl<nvrhi::IMessageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(RhiMessages)
    NVRHI_IMPLEMENTS_INTERFACE(nvrhi::IMessageCallback)
    NVRHI_IMPLEMENTS_INTERFACE(nvrhi::IRHIObject)
    NVRHI_END_INTERFACE_TABLE()

    explicit RhiMessages(std::atomic<int>* problems) : m_Problems(problems) {}

    void message(nvrhi::MessageSeverity severity, const char* messageText) noexcept override {
        if (severity == nvrhi::MessageSeverity::Info)
            return;
        ++*m_Problems;
        std::printf("nvrhi %s: %s\n", severity == nvrhi::MessageSeverity::Warning ? "warning" : "error",
                    messageText);
        std::fflush(stdout);
    }

 private:
    std::atomic<int>* m_Problems;
};

std::atomic<int>* g_LogProblems = nullptr;
std::mutex g_LogMutex;

void logCallback(donut::log::Severity severity, const char* text) {
    const char* prefix = "";
    switch (severity) {
        case donut::log::Severity::Debug: prefix = "debug: "; break;
        case donut::log::Severity::Info: prefix = ""; break;
        case donut::log::Severity::Warning: prefix = "warning: "; break;
        case donut::log::Severity::Error: prefix = "error: "; break;
        case donut::log::Severity::Fatal: prefix = "fatal: "; break;
        default: break;
    }
    std::lock_guard<std::mutex> lock(g_LogMutex);
    std::printf("%s%s\n", prefix, text);
    std::fflush(stdout);
    if (severity >= donut::log::Severity::Warning && g_LogProblems)
        ++*g_LogProblems;
}

}  // namespace

struct ValidationMonitor::Impl {
    nvrhi::AutoPtr<RhiMessages> rhi;
#ifdef _WIN32
    ID3D12InfoQueue* queue12 = nullptr;
    ID3D11InfoQueue* queue11 = nullptr;
#endif
};

ValidationMonitor::ValidationMonitor() : m_Impl(new Impl) {
    m_Impl->rhi = MAKE_RC_OBJ_PTR(RhiMessages, &m_Problems);
    g_LogProblems = &m_Problems;
    donut::log::SetCallback(logCallback);
}

ValidationMonitor::~ValidationMonitor() {
    DetachDevice();
    donut::log::ResetCallback();
    g_LogProblems = nullptr;
    delete m_Impl;
}

nvrhi::IMessageCallback* ValidationMonitor::GetRhiCallback() const { return m_Impl->rhi; }

void ValidationMonitor::AttachDevice(nvrhi::IDevice* device) {
    DetachDevice();
#ifdef _WIN32
    if (device->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D12) {
        if (auto* native = static_cast<ID3D12Device*>(device->getNativeObject(nvrhi::ObjectTypes::D3D12_Device)))
            native->QueryInterface(IID_PPV_ARGS(&m_Impl->queue12));
        if (m_Impl->queue12) {
            // report, do not break: Donut's device manager breaks on errors in its debug configuration
            m_Impl->queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, FALSE);
            m_Impl->queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, FALSE);
            m_Impl->queue12->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, FALSE);
        }
    } else if (device->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D11) {
        if (auto* native = static_cast<ID3D11Device*>(device->getNativeObject(nvrhi::ObjectTypes::D3D11_Device)))
            native->QueryInterface(IID_PPV_ARGS(&m_Impl->queue11));
        if (m_Impl->queue11) {
            m_Impl->queue11->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_CORRUPTION, FALSE);
            m_Impl->queue11->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_ERROR, FALSE);
            m_Impl->queue11->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_WARNING, FALSE);
        }
    }
#else
    (void)device;
#endif
    Poll();
}

void ValidationMonitor::DetachDevice() {
#ifdef _WIN32
    if (m_Impl->queue12) {
        Poll();
        m_Impl->queue12->Release();
        m_Impl->queue12 = nullptr;
    }
    if (m_Impl->queue11) {
        Poll();
        m_Impl->queue11->Release();
        m_Impl->queue11 = nullptr;
    }
#endif
}

int ValidationMonitor::Poll() {
#ifdef _WIN32
    std::vector<char> storage;
    if (ID3D12InfoQueue* q = m_Impl->queue12) {
        const UINT64 messages = q->GetNumStoredMessagesAllowedByRetrievalFilter();
        for (UINT64 i = 0; i < messages; ++i) {
            SIZE_T size = 0;
            q->GetMessage(i, nullptr, &size);
            storage.resize(size);
            auto* message = reinterpret_cast<D3D12_MESSAGE*>(storage.data());
            if (FAILED(q->GetMessage(i, message, &size)))
                continue;
            if (message->Severity > D3D12_MESSAGE_SEVERITY_WARNING)
                continue;  // info, message
            ++m_Problems;
            std::printf("d3d12 debug layer: %s\n", message->pDescription);
        }
        q->ClearStoredMessages();
    }
    if (ID3D11InfoQueue* q = m_Impl->queue11) {
        const UINT64 messages = q->GetNumStoredMessagesAllowedByRetrievalFilter();
        for (UINT64 i = 0; i < messages; ++i) {
            SIZE_T size = 0;
            q->GetMessage(i, nullptr, &size);
            storage.resize(size);
            auto* message = reinterpret_cast<D3D11_MESSAGE*>(storage.data());
            if (FAILED(q->GetMessage(i, message, &size)))
                continue;
            if (message->Severity > D3D11_MESSAGE_SEVERITY_WARNING)
                continue;
            ++m_Problems;
            std::printf("d3d11 debug layer: %s\n", message->pDescription);
        }
        q->ClearStoredMessages();
    }
    std::fflush(stdout);
#endif
    return m_Problems.load();
}

void ValidationMonitor::OnPhysMessage(PhysMessageSeverity severity, const char* text) {
    if (severity == PhysMessageSeverity::Info) {
        std::printf("engine: %s\n", text);
        return;
    }
    ++m_Problems;
    std::printf("engine %s: %s\n", severity == PhysMessageSeverity::Warning ? "warning" : "error", text);
    std::fflush(stdout);
}

}  // namespace ephys::framework
