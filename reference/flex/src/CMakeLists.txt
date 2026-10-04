if(NVFLEX_USE_MICROSOFT_VLD)
    include(../cmake/FindVLD.cmake)
    set(NVFLEX__USE_MICROSOFT_VLD 1)
else()
    set(NVFLEX__USE_MICROSOFT_VLD 0)
endif()

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

shadertool_add_shader_objects(
    TARGET NvFlexShaders
    SHADER_PROFILE 5_0
    CONFIG_FILE "shaders/Shaders.cfg"
    BACKENDS fxc
    OUTPUT_NAME_USE_ENTRY
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/__shader_objs"
    INCLUDE_DIRECTORIES
    "../external/nvapi/include"
    "../external/ags_lib/hlsl"
)
get_target_property(NvFlexShaders_SHADER_HEADER_OUTPUT_DIR NvFlexShaders OBJECT_HEADER_OUTPUT_DIR)
get_target_property(NvFlexShaders_OBJECT_HEADER_FILES NvFlexShaders OBJECT_HEADER_FILES)

# XPBD entries have no shipped DXBC blob and live in a separate manifest so the
# DXBC-equivalence tools (which iterate Shaders.cfg) stay untouched.
shadertool_add_shader_objects(
    TARGET NvFlexShadersXPBD
    SHADER_PROFILE 5_0
    CONFIG_FILE "shaders/ShadersXPBD.cfg"
    BACKENDS fxc
    OUTPUT_NAME_USE_ENTRY
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/__shader_objs"
    INCLUDE_DIRECTORIES
    "../external/nvapi/include"
    "../external/ags_lib/hlsl"
)
get_target_property(NvFlexShadersXPBD_SHADER_HEADER_OUTPUT_DIR NvFlexShadersXPBD
                    OBJECT_HEADER_OUTPUT_DIR)
get_target_property(NvFlexShadersXPBD_OBJECT_HEADER_FILES NvFlexShadersXPBD
                    OBJECT_HEADER_FILES)

file(GLOB NvFlexDXBCBinaryFiles "dxbc/*.txt")
cmrc_add_resource_library(
    NvFlexDXBCBinaryLib
    WHENCE dxbc
    NAMESPACE nvflex
    ${NvFlexDXBCBinaryFiles}
)

file(GLOB Flex_PRIVATE_HEADERS "private/*.h")
file(GLOB Flex_SOURCES "*.cpp" "*.c")

add_library(NvFlexRev SHARED
    ${Flex_PRIVATE_HEADERS}
    ${Flex_SOURCES}
)
target_compile_definitions(NvFlexRev PRIVATE "NOMINMAX=1" "_CRT_SECURE_NO_WARNINGS" "NVFLEX__USE_MICROSOFT_VLD=${NVFLEX__USE_MICROSOFT_VLD}")
set_target_properties(
    NvFlexRev PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)
target_include_directories(
    NvFlexRev
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
    PRIVATE "private" "${NvFlexShaders_SHADER_HEADER_OUTPUT_DIR}"
    "${NvFlexShadersXPBD_SHADER_HEADER_OUTPUT_DIR}"
)
target_link_libraries(NvFlexRev PRIVATE "dxguid.lib" NVTX NVAPI AGS NvFlexDXBCBinaryLib "$<${NVFLEX__USE_MICROSOFT_VLD}:vld>")
add_dependencies(NvFlexRev NvFlexShaders NvFlexShadersXPBD)



