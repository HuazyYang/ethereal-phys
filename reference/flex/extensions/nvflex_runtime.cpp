#include "nvflex_runtime.h"

#include <Windows.h>

#include <cstring>
#include <filesystem>

namespace
{
FARPROC FindExport(HMODULE module, const char *logicalName)
{
    const auto base = reinterpret_cast<const unsigned char *>(module);
    const auto dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE)
        return nullptr;

    const auto nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE)
        return nullptr;

    const DWORD exportRva = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT].VirtualAddress;
    if (!exportRva)
        return nullptr;

    const auto exports = reinterpret_cast<const IMAGE_EXPORT_DIRECTORY *>(base + exportRva);
    const auto names = reinterpret_cast<const DWORD *>(base + exports->AddressOfNames);
    const std::string decoratedPrefix = std::string("?") + logicalName + "@@";

    for (DWORD index = 0; index < exports->NumberOfNames; ++index)
    {
        const char *exportName = reinterpret_cast<const char *>(base + names[index]);
        if (std::strcmp(exportName, logicalName) == 0 ||
            std::strncmp(exportName, decoratedPrefix.c_str(), decoratedPrefix.size()) == 0)
            return GetProcAddress(module, exportName);
    }

    return nullptr;
}

std::string FormatWindowsError(DWORD error)
{
    char *text = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, error, 0, reinterpret_cast<char *>(&text), 0, nullptr);
    std::string result = text ? text : "unknown Windows error";
    if (text)
        LocalFree(text);
    return result;
}

bool LoadLibraryByPath(const std::wstring &path, void **module, std::string &error)
{
    HMODULE handle = LoadLibraryExW(path.c_str(), nullptr,
        LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if (!handle)
    {
        error = "Cannot load DLL '" + std::filesystem::path(path).string() + "': " +
            FormatWindowsError(GetLastError());
        return false;
    }

    *module = handle;
    return true;
}

std::wstring ResolveLibraryPath(const std::wstring &path, NvFlexRuntime::Backend backend)
{
    std::filesystem::path basePath(path);
    if (basePath.has_extension() && basePath.extension().wstring().compare(L".dll") == 0)
        return basePath.wstring();

#if 1
    const wchar_t *dllName =
        backend == NvFlexRuntime::eBackendReversed ? L"NvFlexRev.dll" :
                                                     L"NvFlexDebugD3D_x64.dll";
#else
    const wchar_t *dllName = backend == NvFlexRuntime::eBackendReversed
                                 ? L"NvFlexRev.dll"
                                 : L"NvFlexReleaseD3D_x64.dll";
#endif
    const std::filesystem::path requestedPath = basePath / dllName;
    if (std::filesystem::exists(requestedPath))
        return requestedPath.wstring();

    wchar_t executablePath[32768] = {};
    const DWORD executablePathLength =
        GetModuleFileNameW(nullptr, executablePath, ARRAYSIZE(executablePath));
    if (executablePathLength > 0 && executablePathLength < ARRAYSIZE(executablePath))
        return (std::filesystem::path(executablePath).parent_path() / dllName).wstring();

    return requestedPath.wstring();
}
}

NvFlexRuntime &NvFlexRuntime::Get()
{
    static NvFlexRuntime runtime;
    return runtime;
}

bool NvFlexRuntime::Load(const std::wstring &path, Backend backend, std::string &error)
{
    Unload();

    const std::wstring dllPath = ResolveLibraryPath(path, backend);
    if (!LoadLibraryByPath(dllPath, &m_Module, error))
        return false;

    m_Backend = backend;

#define RESOLVE_NVFLEX_PROC(name) \
    p_##name = reinterpret_cast<decltype(p_##name)>(FindExport(static_cast<HMODULE>(m_Module), #name)); \
    if (!p_##name) { \
        error = "NvFlex DLL '" + std::filesystem::path(dllPath).string() + \
            "' is missing export: " #name; \
        Unload(); \
        return false; \
    }
    NVFLEX_RUNTIME_FUNCTIONS(RESOLVE_NVFLEX_PROC)
#undef RESOLVE_NVFLEX_PROC

    if(m_Backend == eBackendReversed) {
#define RESOLVE_NVFLEX_REV_PROC(name) \
    p_##name = reinterpret_cast<decltype(p_##name)>(FindExport(static_cast<HMODULE>(m_Module), #name)); \
    if (!p_##name) { \
        error = "NvFlex DLL '" + std::filesystem::path(dllPath).string() + \
            "' is missing export: " #name; \
        Unload(); \
        return false; \
    }
    NVFLEX_REV_RUNTIME_FUNCTIONS(RESOLVE_NVFLEX_REV_PROC)
    NVFLEX_REV_CONTEXT_FUNCTIONS(RESOLVE_NVFLEX_REV_PROC)
#undef RESOLVE_NVFLEX_REV_PROC
    }

    return true;
}

bool NvFlexRuntime::Load(const std::wstring &path, std::string &error)
{
    return Load(path, eBackendPublic, error);
}

void NvFlexRuntime::Unload()
{
#define CLEAR_NVFLEX_PROC(name) p_##name = nullptr;
    NVFLEX_RUNTIME_FUNCTIONS(CLEAR_NVFLEX_PROC)
    NVFLEX_REV_RUNTIME_FUNCTIONS(CLEAR_NVFLEX_PROC)
#undef CLEAR_NVFLEX_PROC

    if (m_Module)
    {
        FreeLibrary(static_cast<HMODULE>(m_Module));
        m_Module = nullptr;
    }

    m_Backend = eBackendPublic;
}
