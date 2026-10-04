# ethereal_phys_framework: the sample framework, a static library.
#
# The FleX demo's main.cpp and demoContext.h, decomposed onto Donut and the engine: the application shell
# (DeviceManager, the camera and input of the demo, picking, the ImGui panel, unattended screenshot and smoke
# runs), the renderer (DemoContext: points, meshes with a shadow map, debug lines, diffuse particles and the
# screen-space fluid surface, with the demo's shaders ported to shader model 6.0), and PhysicsScene, which runs a
# scene's content through ethereal_phys_scene (a domain and shape leaves on Donut's scene graph, stepped by
# PhysSceneService). A sample registers its scenes and calls RunDemoApp (demo_app.h); ethereal_phys_add_sample
# (samples/CMakeLists.txt) builds it. Headers under include/ethereal/phys_framework; not DLL boundary headers.

set(framework_shader_sources
	shaders/shaderCommon.h
	shaders/blurDepthPS.hlsl
	shaders/compositePS.hlsl
	shaders/debugLinePS.hlsl
	shaders/debugLineVS.hlsl
	shaders/diffuseGS.hlsl
	shaders/diffusePS.hlsl
	shaders/diffuseVS.hlsl
	shaders/ellipsoidDepthGS.hlsl
	shaders/ellipsoidDepthPS.hlsl
	shaders/ellipsoidDepthVS.hlsl
	shaders/meshPS.hlsl
	shaders/meshShadowPS.hlsl
	shaders/meshVS.hlsl
	shaders/passThroughVS.hlsl
	shaders/pointGS.hlsl
	shaders/pointPS.hlsl
	shaders/pointShadowPS.hlsl
	shaders/pointThicknessGS.hlsl
	shaders/pointThicknessPS.hlsl
	shaders/pointThicknessVS.hlsl
	shaders/pointVS.hlsl)

# DXIL and SPIR-V at shader model 6.0, DXBC at 5.0 (ALL_PLATFORMS); the constant buffers keep the D3D layout on
# Vulkan, as the engine's kernels do.
ethereal_compile_shaders(
	TARGET ethereal_phys_framework_shaders
	CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/shaders/shaders.cfg
	SOURCES ${framework_shader_sources}
	FOLDER "ethereal-phys"
	OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/phys_framework"
	SHADER_MODEL 6_0
	ALL_PLATFORMS
	EXTRA_FLAGS_SPIRV -fvk-use-dx-layout)

add_library(ethereal_phys_framework STATIC
	include/ethereal/phys_framework/demo_app.h
	include/ethereal/phys_framework/demo_context.h
	include/ethereal/phys_framework/demo_state.h
	include/ethereal/phys_framework/flex_math.h
	include/ethereal/phys_framework/scene.h
	src/demo_app.cpp
	src/demo_app_impl.h
	src/demo_context.cpp
	src/demo_ui.cpp
	src/flex_math.cpp
	src/physics_scene.cpp
	src/physics_scene.h
	src/validation.cpp
	src/validation.h)

target_include_directories(ethereal_phys_framework PUBLIC include)
target_link_libraries(ethereal_phys_framework PUBLIC ethereal_phys_helpers ethereal_phys_scene ethereal_phys_utils
	ethereal_phys donut_app donut_engine)
add_dependencies(ethereal_phys_framework ethereal_phys_framework_shaders ethereal_phys_shaders)

if (MSVC)
	target_compile_options(ethereal_phys_framework PRIVATE /utf-8)
endif()

set_target_properties(ethereal_phys_framework PROPERTIES FOLDER "ethereal-phys")
