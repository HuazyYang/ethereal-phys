cmake_minimum_required(VERSION 3.25)

project(NvFlex VERSION 1.2.0)

# NvFlow config options
option(NVFLEX_USE_MICROSOFT_VLD "NvFlow use Microsoft VLD library to detect user space moemory leakage" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(MSVC)
    function(demo_apply_msvc_debug_optimizations)
        if(NOT DEFINED NVFLEX_MSVC_OPTIONS_STACK)
            set(NVFLEX_MSVC_OPTIONS_STACK "" PARENT_SCOPE)
        endif()

        list(APPEND NVFLEX_MSVC_OPTIONS_STACK "${CMAKE_C_FLAGS_DEBUG}" "${CMAKE_CXX_FLAGS_DEBUG}" "${CMAKE_MSVC_RUNTIME_LIBRARY}")

        string(REGEX REPLACE "(^| )[/-]Ob0( |$)" "\\1/Ob2\\2" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        string(REGEX REPLACE "(^| )[/-]Od( |$)" "\\1/O2\\2" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        string(REGEX REPLACE "(^| )[/-]RTC1( |$)" "\\1\\2" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        set(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG} PARENT_SCOPE)

        string(REGEX REPLACE "(^| )[/-]Ob0( |$)" "\\1/Ob2\\2" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
        string(REGEX REPLACE "(^| )[/-]Od( |$)" "\\1/O2\\2" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
        string(REGEX REPLACE "(^| )[/-]RTC1( |$)" "\\1\\2" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
        set(CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG} PARENT_SCOPE)

        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded" PARENT_SCOPE)
    endfunction(demo_apply_msvc_debug_optimizations)
else()
    function(demo_apply_msvc_debug_optimizations)
    endfunction(demo_apply_msvc_debug_optimizations)
endif()

if(MSVC)
    add_compile_definitions(
        _HAS_ITERATOR_DEBUGGING=0
        _ITERATOR_DEBUG_LEVEL=0)

    if(CMAKE_CROSSCOMPILING)
        add_compile_options(
            "$<$<CONFIG:Debug>:-gdwarf>"
            "$<$<COMPILE_LANGUAGE:CXX>:-fdelayed-template-parsing>"
            "$<$<COMPILE_LANGUAGE:CXX>:-Wno-address-of-temporary>")
        add_link_options("$<$<CONFIG:Debug>:/debug:dwarf>")
    endif()
endif()

include(cmake/CMakeRC.cmake)
include(cmake/CopyAsset.cmake)

add_subdirectory(thirdparty/ShaderTool EXCLUDE_FROM_ALL)
set(ShaderTool_DIR "${CMAKE_BINARY_DIR}/thirdparty/ShaderTool")
find_package(ShaderTool CONFIG REQUIRED)

# NvFlexLib
add_library(NvFlexLib SHARED IMPORTED)
set(NvFlexLib_IMPLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/win64")
set(NvFlexLib_IMPLOC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64" )
set_target_properties(NvFlexLib PROPERTIES
    IMPORTED_LOCATION_DEBUG "${NvFlexLib_IMPLOC_DIR}/NvFlexDebugD3D_x64.dll"
    IMPORTED_LOCATION_RELEASE "${NvFlexLib_IMPLOC_DIR}/NvFlexReleaseD3D_x64.dll"
    IMPORTED_LOCATION_RELWITHDEBINFO "${NvFlexLib_IMPLOC_DIR}/NvFlexDebugD3D_x64.dll"
    IMPORTED_LOCATION_MINSIZEREL "${NvFlexLib_IMPLOC_DIR}/NvFlexReleaseD3D_x64.dll"

    IMPORTED_IMPLIB_DEBUG "${NvFlexLib_IMPLIB_DIR}/NvFlexDebugD3D_x64.lib"
    IMPORTED_IMPLIB_RELEASE "${NvFlexLib_IMPLIB_DIR}/NvFlexReleaseD3D_x64.lib"
    IMPORTED_IMPLIB_RELWITHDEBINFO "${NvFlexLib_IMPLIB_DIR}/NvFlexDebugD3D_x64.lib"
    IMPORTED_IMPLIB_MINSIZEREL "${NvFlexLib_IMPLIB_DIR}/NvFlexReleaseD3D_x64.lib"

    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# SDL2
add_library(SDL2 SHARED IMPORTED)
set_target_properties(SDL2 PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL2-2.0.4/lib/x64/SDL2.dll"
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL2-2.0.4/lib/x64/SDL2.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL2-2.0.4/include"
)

add_library(SDL2main SHARED IMPORTED)
set_target_properties(SDL2main PROPERTIES
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL2-2.0.4/lib/x64/SDL2main.lib"
)

# stb
add_library(stb INTERFACE)
set_target_properties(stb PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/external/stb_image"
)

# NVTX
add_library(NVTX SHARED IMPORTED)
set_target_properties(NVTX PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/NvToolsExt/lib/x64/nvToolsExt64_1.dll"
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/external/NvToolsExt/lib/x64/nvToolsExt64_1.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/external/NvToolsExt/include"
)


# NVAPI
add_library(NVAPI STATIC IMPORTED)
set_target_properties(NVAPI PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/nvapi/lib/nvapi64.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/external/nvapi/include"
)

# AGS
add_library(AGS SHARED IMPORTED)
set_target_properties(AGS PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/ags_lib/lib/amd_ags_x64.dll"
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/external/ags_lib/lib/amd_ags_x64.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/external/ags_lib/inc"
)

add_subdirectory(src)
add_subdirectory(extensions)
add_subdirectory(demo)


