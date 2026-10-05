# Samples built on ethereal_phys.
#
#   common/      ethereal_phys_utils: what the samples need besides the engine (mesh readers and primitives, the
#                PFM distance field format, rope tubes, convex hulls, Perlin noise, the voxeliser and distance
#                field of a closed mesh)
#   helpers/     ethereal_phys_helpers: the FleX demo's scene-authoring helpers (helpers.h) as a library
#   framework/   ethereal_phys_framework: the application shell, the renderer (the demo's shaders) and the
#                physics scene on Donut's scene graph; a sample registers its scenes and calls RunDemoApp
#   <sample>/    one directory per sample, with a CMakeLists.txt that calls ethereal_phys_add_sample();
#                discovered by glob, so a new sample needs no edit here

add_subdirectory(common)
add_subdirectory(helpers)
add_subdirectory(framework)

# ethereal_phys_add_sample(<name> SOURCES <files...> [LIBRARIES <targets...>] [SMOKE_FRAMES <n>])
#
# A console executable (the window is Donut's; the log, the validation messages and the unattended results go to
# stdout) linked against the framework, with ETHEREAL_PHYS_ASSETS_DIR (the meshes of assets/) and
# ETHEREAL_PHYS_SAMPLE_NAME defined. With ETHEREAL_PHYS_BUILD_TESTS, one ctest per graphics API of the build,
# ethereal_phys_sample_<name>_<api>: every scene of the sample in turn for SMOKE_FRAMES (default 10) simulation
# steps with the debug runtime and the validation layers; it fails on a non-zero exit, which a validation error,
# an engine error or a frame of one colour gives (RunDemoApp's --smoke). ethereal_phys_sample_<name>_<api>_async is
# the same with --async-compute (D3D12 and Vulkan).
function(ethereal_phys_add_sample name)
	cmake_parse_arguments(arg "" "SMOKE_FRAMES" "SOURCES;LIBRARIES" ${ARGN})
	if (NOT arg_SOURCES)
		message(FATAL_ERROR "ethereal_phys_add_sample(${name}): SOURCES required")
	endif()
	if (NOT arg_SMOKE_FRAMES)
		set(arg_SMOKE_FRAMES 10)
	endif()

	add_executable(${name} ${arg_SOURCES})
	target_link_libraries(${name} PRIVATE ethereal_phys_framework ${arg_LIBRARIES})
	target_compile_definitions(${name} PRIVATE
		"ETHEREAL_PHYS_ASSETS_DIR=\"${ETHEREAL_PHYS_ASSETS_DIR}\""
		"ETHEREAL_PHYS_SAMPLE_NAME=\"${name}\"")
	target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
	add_dependencies(${name} ethereal_phys_framework_shaders ethereal_phys_shaders)
	set_target_properties(${name} PROPERTIES FOLDER "ethereal-phys/samples")
	if (MSVC)
		target_compile_options(${name} PRIVATE /utf-8)
	endif()

	if (ETHEREAL_PHYS_BUILD_TESTS)
		set(apis)
		if (DONUT_WITH_DX12)
			list(APPEND apis d3d12)
		endif()
		if (DONUT_WITH_VULKAN)
			list(APPEND apis vulkan)
		endif()
		if (DONUT_WITH_DX11)
			list(APPEND apis d3d11)
		endif()
		foreach(api IN LISTS apis)
			if (api STREQUAL "vulkan")
				set(switch --vk)
			else()
				set(switch --${api})
			endif()
			add_test(NAME ethereal_phys_sample_${name}_${api}
				COMMAND ${name} ${switch} --debug --smoke ${arg_SMOKE_FRAMES} --width 640 --height 360)
			set_tests_properties(ethereal_phys_sample_${name}_${api} PROPERTIES TIMEOUT 900 RUN_SERIAL TRUE)
			# The same on a compute queue (--async-compute): the physics steps there, the frame waits for it and the
			# next step for the frame. D3D11 has no compute queue.
			if (NOT api STREQUAL "d3d11")
				add_test(NAME ethereal_phys_sample_${name}_${api}_async
					COMMAND ${name} ${switch} --debug --async-compute --smoke ${arg_SMOKE_FRAMES} --width 640 --height 360)
				set_tests_properties(ethereal_phys_sample_${name}_${api}_async PROPERTIES TIMEOUT 900 RUN_SERIAL TRUE)
			endif()
		endforeach()
	endif()
endfunction()

file(GLOB sample_dirs LIST_DIRECTORIES true RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/*")
list(SORT sample_dirs)
foreach(dir IN LISTS sample_dirs)
	if (dir STREQUAL "common" OR dir STREQUAL "helpers" OR dir STREQUAL "framework")
		continue()
	endif()
	if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${dir}/CMakeLists.txt")
		add_subdirectory(${dir})
	endif()
endforeach()
