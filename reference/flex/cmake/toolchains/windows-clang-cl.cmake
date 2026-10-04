set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)

# Use CMake's tool variables directly; callers can override each on the command line.
find_program(CMAKE_C_COMPILER NAMES clang-cl-19 clang-cl REQUIRED)
set(CMAKE_CXX_COMPILER "${CMAKE_C_COMPILER}" CACHE FILEPATH "C++ compiler")
find_program(CMAKE_LINKER NAMES lld-link-19 lld-link REQUIRED)
find_program(CMAKE_AR NAMES llvm-lib-19 llvm-lib REQUIRED)
find_program(CMAKE_RC_COMPILER NAMES llvm-rc-19 llvm-rc REQUIRED)
find_program(CMAKE_MT NAMES llvm-mt-19 llvm-mt REQUIRED)

get_filename_component(
    WINDOWS_CMAKE_MODULE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/.."
    ABSOLUTE)

if(DEFINED ENV{WINEPREFIX} AND NOT "$ENV{WINEPREFIX}" STREQUAL "")
    set(_windows_wineprefix "$ENV{WINEPREFIX}")
else()
    set(_windows_wineprefix "$ENV{HOME}/.wineprefixes/dev-x64")
endif()
set(WINEPREFIX "${_windows_wineprefix}" CACHE PATH
    "Wine prefix used by the Windows cross-toolchain")
if(NOT WindowsSDK_ROOT)
    set(WindowsSDK_ROOT
        "${_windows_wineprefix}/drive_c/Program Files (x86)/Windows Kits/10")
endif()
set(WindowsSDK_ROOT "${WindowsSDK_ROOT}" CACHE PATH
    "Windows 10 SDK root inside the target environment")
list(APPEND CMAKE_MODULE_PATH "${WINDOWS_CMAKE_MODULE_DIR}")
find_package(WindowsSDK 10.0.19041 REQUIRED)

if(DEFINED ENV{MSVC_ROOT} AND NOT "$ENV{MSVC_ROOT}" STREQUAL "")
    set(MSVC_ROOT "$ENV{MSVC_ROOT}")
else()
    set(MSVC_ROOT
        "${_windows_wineprefix}/drive_c/Program Files/Microsoft Visual Studio/2022/BuildTools")
endif()
file(GLOB _windows_msvc_toolsets LIST_DIRECTORIES true
    "${MSVC_ROOT}/VC/Tools/MSVC/*")
list(SORT _windows_msvc_toolsets COMPARE NATURAL ORDER DESCENDING)
if(NOT _windows_msvc_toolsets)
    message(FATAL_ERROR "MSVC toolset was not found below ${MSVC_ROOT}")
endif()
list(GET _windows_msvc_toolsets 0 MSVC_TOOLSET_ROOT)

set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
    "${MSVC_TOOLSET_ROOT}/include"
    "${WindowsSDK_INCLUDE_DIR}/shared"
    "${WindowsSDK_INCLUDE_DIR}/ucrt"
    "${WindowsSDK_INCLUDE_DIR}/um"
    "${WindowsSDK_INCLUDE_DIR}/winrt")
set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_STANDARD_INCLUDE_DIRECTORIES})
string(CONCAT CMAKE_EXE_LINKER_FLAGS_INIT
    " /libpath:\"${MSVC_TOOLSET_ROOT}/lib/x64\""
    " /libpath:\"${WindowsSDK_LIBRARY_DIR}/ucrt/x64\""
    " /libpath:\"${WindowsSDK_LIBRARY_DIR}/um/x64\"")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT}")

find_program(CMAKE_CROSSCOMPILING_EMULATOR NAMES wine REQUIRED NO_CMAKE_FIND_ROOT_PATH)

set(CMAKE_C_COMPILER_TARGET x86_64-pc-windows-msvc)
set(CMAKE_CXX_COMPILER_TARGET x86_64-pc-windows-msvc)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
set(CMAKE_C_FLAGS_INIT "-fms-compatibility-version=19.44")
set(CMAKE_CXX_FLAGS_INIT "-fms-compatibility-version=19.44")
set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release "")
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release "")

set(CMAKE_C_FLAGS_DEBUG_INIT "/Od /Ob0 -gdwarf")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "/Od /Ob0 -gdwarf")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG_INIT "/debug:dwarf")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG_INIT "/debug:dwarf")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)

list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
    WINEPREFIX
    MSVC_ROOT
    WindowsSDK_ROOT
    WindowsSDK_VERSION
)
