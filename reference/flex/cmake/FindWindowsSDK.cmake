include_guard(GLOBAL)

include(FindPackageHandleStandardArgs)

set(_WindowsSDK_root_candidates)
if(WindowsSDK_ROOT)
    list(APPEND _WindowsSDK_root_candidates "${WindowsSDK_ROOT}")
endif()
if(DEFINED ENV{WindowsSDK_ROOT} AND NOT "$ENV{WindowsSDK_ROOT}" STREQUAL "")
    list(APPEND _WindowsSDK_root_candidates "$ENV{WindowsSDK_ROOT}")
endif()
if(DEFINED ENV{WindowsSdkDir} AND NOT "$ENV{WindowsSdkDir}" STREQUAL "")
    list(APPEND _WindowsSDK_root_candidates "$ENV{WindowsSdkDir}")
endif()
if(CMAKE_HOST_WIN32)
    get_filename_component(_WindowsSDK_registry_root
        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows Kits\\Installed Roots;KitsRoot10]"
        ABSOLUTE)
    if(_WindowsSDK_registry_root)
        list(APPEND _WindowsSDK_root_candidates "${_WindowsSDK_registry_root}")
    endif()
endif()
list(REMOVE_DUPLICATES _WindowsSDK_root_candidates)

set(_WindowsSDK_requested_version)
if(WindowsSDK_VERSION)
    set(_WindowsSDK_requested_version "${WindowsSDK_VERSION}")
elseif(CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION)
    set(_WindowsSDK_requested_version
        "${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}")
elseif(DEFINED ENV{WindowsSDKVersion} AND
       NOT "$ENV{WindowsSDKVersion}" STREQUAL "")
    string(REGEX REPLACE "[/\\\\]+$" "" _WindowsSDK_requested_version
        "$ENV{WindowsSDKVersion}")
elseif(CMAKE_SYSTEM_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+$")
    set(_WindowsSDK_requested_version "${CMAKE_SYSTEM_VERSION}")
endif()

set(WindowsSDK_ROOT "WindowsSDK_ROOT-NOTFOUND")
set(WindowsSDK_VERSION "WindowsSDK_VERSION-NOTFOUND")
set(WindowsSDK_INCLUDE_DIR "WindowsSDK_INCLUDE_DIR-NOTFOUND")
set(WindowsSDK_LIBRARY_DIR "WindowsSDK_LIBRARY_DIR-NOTFOUND")
set(WindowsSDK_BINARY_DIR "WindowsSDK_BINARY_DIR-NOTFOUND")

foreach(_WindowsSDK_root IN LISTS _WindowsSDK_root_candidates)
    if(NOT IS_DIRECTORY "${_WindowsSDK_root}")
        continue()
    endif()

    set(_WindowsSDK_versions)
    if(_WindowsSDK_requested_version)
        list(APPEND _WindowsSDK_versions "${_WindowsSDK_requested_version}")
    else()
        file(GLOB _WindowsSDK_include_versions LIST_DIRECTORIES TRUE
            RELATIVE "${_WindowsSDK_root}/Include"
            "${_WindowsSDK_root}/Include/10.*")
        list(SORT _WindowsSDK_include_versions COMPARE NATURAL ORDER DESCENDING)
        list(APPEND _WindowsSDK_versions ${_WindowsSDK_include_versions})
    endif()

    foreach(_WindowsSDK_version IN LISTS _WindowsSDK_versions)
        set(_WindowsSDK_include "${_WindowsSDK_root}/Include/${_WindowsSDK_version}")
        set(_WindowsSDK_library "${_WindowsSDK_root}/Lib/${_WindowsSDK_version}")
        set(_WindowsSDK_binary "${_WindowsSDK_root}/bin/${_WindowsSDK_version}/x64")
        if(IS_DIRECTORY "${_WindowsSDK_include}/shared" AND
           IS_DIRECTORY "${_WindowsSDK_include}/ucrt" AND
           IS_DIRECTORY "${_WindowsSDK_include}/um" AND
           IS_DIRECTORY "${_WindowsSDK_library}/ucrt/x64" AND
           IS_DIRECTORY "${_WindowsSDK_library}/um/x64" AND
           IS_DIRECTORY "${_WindowsSDK_binary}")
            set(WindowsSDK_ROOT "${_WindowsSDK_root}")
            set(WindowsSDK_VERSION "${_WindowsSDK_version}")
            set(WindowsSDK_INCLUDE_DIR "${_WindowsSDK_include}")
            set(WindowsSDK_LIBRARY_DIR "${_WindowsSDK_library}")
            set(WindowsSDK_BINARY_DIR "${_WindowsSDK_binary}")
            break()
        endif()
    endforeach()
    if(NOT WindowsSDK_ROOT MATCHES "-NOTFOUND$")
        break()
    endif()
endforeach()

find_package_handle_standard_args(WindowsSDK
    REQUIRED_VARS WindowsSDK_ROOT WindowsSDK_INCLUDE_DIR
        WindowsSDK_LIBRARY_DIR WindowsSDK_BINARY_DIR
    VERSION_VAR WindowsSDK_VERSION)

if(WindowsSDK_FOUND AND NOT TARGET WindowsSDK::Headers)
    add_library(WindowsSDK::Headers INTERFACE IMPORTED)
    set_target_properties(WindowsSDK::Headers PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES
            "${WindowsSDK_INCLUDE_DIR}/shared;${WindowsSDK_INCLUDE_DIR}/ucrt;${WindowsSDK_INCLUDE_DIR}/um;${WindowsSDK_INCLUDE_DIR}/winrt")
endif()

mark_as_advanced(WindowsSDK_ROOT WindowsSDK_VERSION WindowsSDK_INCLUDE_DIR
    WindowsSDK_LIBRARY_DIR WindowsSDK_BINARY_DIR)
