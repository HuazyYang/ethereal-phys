demo_apply_msvc_debug_optimizations()

set(NvFlexExt_OUTNAME
    "NvFlexExt$<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:Debug>$<$<OR:$<CONFIG:Release>,$<CONFIG:MinSizeRel>>:Release>D3D_win64")

shadertool_add_shader_objects(
    TARGET NvFlexExtShaders
    BACKENDS fxc
    SHADER_PROFILE 5_0
    CONFIG_FILE "dx/shaders/Shaders.cfg"
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/shaders"
)
get_target_property(NvFlexExt_SHADER_HEADER_FILES NvFlexExtShaders OBJECT_HEADER_FILES)
get_target_property(NvFlexExt_SHADER_HEADER_OUTPUT_DIR NvFlexExtShaders OBJECT_HEADER_OUTPUT_DIR)

set(NvFlexExt_SOURCES
    "../core/aabbtree.h"
    "../core/aabbtree.cpp"
    "../core/maths.h"
    "../core/maths.cpp"
    "../core/sdf.h"
    "../core/sdf.cpp"
    "../core/voxelize.h"
    "../core/voxelize.cpp"
    "dx/flexExt_dx_common.h"
    "dx/flexExt.cpp"
    "flexExtCloth.cpp"
    "flexExtContainer.cpp"
    "flexExtMovingFrame.cpp"
    "flexExtRigid.cpp"
    "flexExtSoft.cpp"
    "../include/nvflex_runtime.h"
    "nvflex_runtime.cpp"
)

add_library(NvFlexExt SHARED ${NvFlexExt_SOURCES})
set_target_properties(NvFlexExt PROPERTIES
    OUTPUT_NAME ${NvFlexExt_OUTNAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)
target_compile_definitions(NvFlexExt PRIVATE
    "_CONSOLE"
    "WIN32"
    "_ITERATOR_DEBUG_LEVEL=0"
)
target_include_directories(NvFlexExt PRIVATE
    "$<BUILD_INTERFACE:${NvFlexExt_SHADER_HEADER_OUTPUT_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
)
add_dependencies(NvFlexExt NvFlexLib NvFlexRev NvFlexExtShaders)

