
demo_apply_msvc_debug_optimizations()

set(DemoAppD3D_OUTNAME
    "NvFlexDemo$<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:Debug>$<$<OR:$<CONFIG:Release>,$<CONFIG:MinSizeRel>>:Release>D3D_win64")

file(GLOB DemoAppD3D_SOURCES
    "*.h" "*.cpp" "*.c"
    "d3d/*.h" "d3d/*.cpp"
    "d3d11/*.h" "d3d11/*.cpp"
    "d3d12/*.h" "d3d12/*.cpp"
    "scene/*.h" "scene/*.cpp"
    "../core/*.h" "../core/*.cpp"
)

shadertool_add_shader_objects(
    TARGET DemoAppD3DShaders
    BACKENDS fxc
    SHADER_PROFILE 5_0
    CONFIG_FILE "d3d/shaders/Shaders.cfg"
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/d3d/shaders"
)
get_target_property(DemoAppD3D_SHADER_HEADER_FILES DemoAppD3DShaders OBJECT_HEADER_FILES)
get_target_property(DemoAppD3D_SHADER_HEADER_OUTPUT_DIR DemoAppD3DShaders OBJECT_HEADER_OUTPUT_DIR)

add_executable(
    DemoAppD3D ${DemoAppD3D_SOURCES}
)
set_target_properties(DemoAppD3D PROPERTIES
    OUTPUT_NAME ${DemoAppD3D_OUTNAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)
target_include_directories(DemoAppD3D PRIVATE
    "$<BUILD_INTERFACE:${DemoAppD3D_SHADER_HEADER_OUTPUT_DIR}/../../>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/d3d12>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
)
target_compile_definitions(DemoAppD3D PRIVATE
    "_CRT_SECURE_NO_WARNINGS"
    "_ITERATOR_DEBUG_LEVEL=0"
)

target_link_libraries(DemoAppD3D PRIVATE SDL2 SDL2main stb NvFlexLib NvFlexExt)
add_dependencies(DemoAppD3D NvFlexLib NvFlexRev DemoAppD3DShaders)

if(NVFLEX_USE_MICROSOFT_VLD)
    target_link_libraries(DemoAppD3D PRIVATE vld)
    target_compile_definitions(DemoAppD3D PRIVATE "NVFLEX_USE_MICROSOFT_VLD=1")
endif()

add_custom_command(
    TARGET DemoAppD3D
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:NvFlexLib>"
        "$<TARGET_FILE:NvFlexRev>"
        "$<TARGET_FILE:NvFlexExt>"
        "$<TARGET_FILE:SDL2>"
        "$<TARGET_FILE:NVTX>"
        "$<TARGET_FILE:AGS>"
        "${CMAKE_SOURCE_DIR}/bin/win64/GFSDK_Aftermath_Lib.x64.dll"
        "$<TARGET_FILE_DIR:DemoAppD3D>"
    )
