# Tests of ethereal_phys, registered with ctest of the aggregate build (the root CMakeLists.txt calls
# enable_testing()). Run them with "ctest --preset release -R ethereal_phys" or ".\build.ps1 -Test".
#
# What must hold before the ABI of the engine may change (donut/nvrhi/doc/com-interface-abi.md, section 8):
#
#   ethereal_phys_abi_lint            the interface and struct rules, on engine/include/ethereal/phys
#   ethereal_phys_abi_lint_selftest   the lint reports a planted violation of each rule
#   ethereal_phys_header_hygiene      no std:: containers, C exports only, noexcept exports, include guards
#   ethereal_phys_header_tus          every public header compiles on its own (building the target is the test)
#   ethereal_phys_guids               no placeholder, duplicated or copied IID / CLSID
#   ethereal_phys_exports             dumpbin /exports ethereal_phys.dll lists only etherealPhys* names
#   ethereal_phys_device_<api>        IPhysDevice and IPhysBuffer on a real, headless nvrhi device
#
# The kernels (engine/shaders):
#
#   ethereal_phys_kernel_table        engine/src/kernel_table.inl and the KernelId enum match the manifest
#   ethereal_phys_shader_artifacts    every manifest row became a non-empty blob with all its permutations,
#                                     for every shader format of this build
#   ethereal_phys_kernel_<api>        IPhysKernelLibrary on a real, headless nvrhi device: pipelines,
#                                     dispatches, the constant-buffer layout, the radix sort, wave variants
#
# The data model (IPhysDomain, its parts, the geometry library):
#
#   ethereal_phys_data_<api>          round trips of every particle array, constraint set and shape set,
#                                     zero-copy render arrays and their fallback, asset ids and lifetimes,
#                                     the hierarchy a triangle mesh update builds, reference counts

set(phys_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/../engine/include")
set(phys_engine_dir "${CMAKE_CURRENT_SOURCE_DIR}/../engine")
set(nvrhi_dir "${ETHEREAL_DONUT_DIR}/nvrhi")

# ---- Scripts ------------------------------------------------------------------------------------------

find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
	# nvrhi's own lint, pointed at these headers (abi_lint.py here explains how), plus the rules it leaves
	# to review.
	add_test(NAME ethereal_phys_abi_lint
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/abi_lint.py
			${nvrhi_dir}/tests ${nvrhi_dir}/include/nvrhi ${phys_include_dir}/ethereal/phys)
	add_test(NAME ethereal_phys_abi_lint_selftest
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/abi_lint.py --self-test
			${nvrhi_dir}/tests ${nvrhi_dir}/include/nvrhi ${phys_include_dir}/ethereal/phys)

	add_test(NAME ethereal_phys_header_hygiene
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/header_hygiene.py
			${phys_include_dir}/ethereal/phys)

	# The IDs of the engine and of its tests, compared with every ID of nvrhi, Donut and the samples.
	set(guid_references ${nvrhi_dir}/include ${nvrhi_dir}/src ${nvrhi_dir}/tests
		${ETHEREAL_DONUT_DIR}/include ${ETHEREAL_DONUT_DIR}/src)
	if (EXISTS "${CMAKE_SOURCE_DIR}/ethereal-samples/src")
		list(APPEND guid_references ${CMAKE_SOURCE_DIR}/ethereal-samples/src)
	endif()
	add_test(NAME ethereal_phys_guids
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_guids.py
			${phys_engine_dir} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../samples
			--reference ${guid_references})

	# The generated kernel table is what the manifest and the kernel sources say.
	add_test(NAME ethereal_phys_kernel_table
		COMMAND ${Python3_EXECUTABLE} ${phys_engine_dir}/shaders/gen_kernel_table.py --check)

	# The shader target produced something for every row. ethereal_compile_shaders creates an empty target,
	# without an error, when no shader format survives the DONUT_WITH_* switches; a build like that must
	# not pass.
	set(phys_shader_formats "")
	if (DONUT_WITH_DX12)
		list(APPEND phys_shader_formats dxil)
	endif()
	if (DONUT_WITH_VULKAN)
		list(APPEND phys_shader_formats spirv)
	endif()
	if (DONUT_WITH_DX11)
		list(APPEND phys_shader_formats dxbc)
	endif()
	add_test(NAME ethereal_phys_shader_artifacts
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_shader_artifacts.py
			${phys_engine_dir}/shaders/ethereal_phys.cfg ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR} ${phys_shader_formats})
else()
	message(WARNING "ethereal-phys: Python 3 not found; the ABI lint, the header hygiene check, the GUID "
		"check, the kernel table check and the shader artifact check are NOT registered")
endif()

# ---- Every public header compiles on its own ----------------------------------------------------------
# One compile-only translation unit per header, which includes that header alone (twice, for the include
# guard). The object library is part of the default build, so a header that is not self-contained breaks
# the build; the ctest entry rebuilds it, so that "ctest" alone also reports it.

file(GLOB_RECURSE phys_public_headers RELATIVE "${phys_include_dir}" "${phys_include_dir}/ethereal/phys/*.h")
list(SORT phys_public_headers)
set(phys_header_tus "")
foreach(header IN LISTS phys_public_headers)
	string(REGEX REPLACE "[/.]" "_" tu_name "${header}")
	set(tu "${CMAKE_CURRENT_BINARY_DIR}/header_tus/${tu_name}.cpp")
	set(tu_content "// Generated by tests/CMakeLists.txt: ${header} compiles on its own.\n#include <${header}>\n#include <${header}>\n")
	if (EXISTS "${tu}")
		file(READ "${tu}" tu_old)
	else()
		set(tu_old "")
	endif()
	if (NOT tu_old STREQUAL tu_content)
		file(WRITE "${tu}" "${tu_content}")
	endif()
	list(APPEND phys_header_tus "${tu}")
endforeach()
add_library(ethereal_phys_header_tus OBJECT ${phys_header_tus})
target_link_libraries(ethereal_phys_header_tus PRIVATE ethereal_phys)  # its usage requirements only
if (MSVC)
	target_compile_options(ethereal_phys_header_tus PRIVATE /W4 /WX /utf-8)
endif()
set_target_properties(ethereal_phys_header_tus PROPERTIES FOLDER "ethereal-phys/Tests")
add_test(NAME ethereal_phys_header_tus
	COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config $<CONFIG> --target ethereal_phys_header_tus)

# ---- The export table ---------------------------------------------------------------------------------

if (MSVC)
	get_filename_component(phys_linker_dir "${CMAKE_LINKER}" DIRECTORY)
	find_program(ETHEREAL_PHYS_DUMPBIN dumpbin HINTS "${phys_linker_dir}")
	if (ETHEREAL_PHYS_DUMPBIN)
		add_test(NAME ethereal_phys_exports
			COMMAND ${CMAKE_COMMAND}
				-DDUMPBIN=${ETHEREAL_PHYS_DUMPBIN}
				-DDLL=$<TARGET_FILE:ethereal_phys>
				-DEXPECTED=etherealPhysCreateDevice,etherealPhysGetHeaderVersion
				-P ${CMAKE_CURRENT_SOURCE_DIR}/check_exports.cmake)
	else()
		message(WARNING "ethereal-phys: dumpbin not found; the export check is NOT registered")
	endif()
endif()

# ---- A real device ------------------------------------------------------------------------------------
# Headless (no window), with the debug runtime and the nvrhi validation layer; an nvrhi error or warning
# fails the test. Exit code 77: the graphics API has no usable device on this machine.

add_executable(ethereal_phys_device_test device_test.cpp)
target_link_libraries(ethereal_phys_device_test PRIVATE ethereal_phys donut_app)
if (MSVC)
	target_compile_options(ethereal_phys_device_test PRIVATE /utf-8)  # Donut headers are not /W4 clean
endif()
set_target_properties(ethereal_phys_device_test PROPERTIES FOLDER "ethereal-phys/Tests")

set(phys_device_tests "")
if (DONUT_WITH_DX12)
	add_test(NAME ethereal_phys_device_d3d12 COMMAND ethereal_phys_device_test d3d12)
	add_test(NAME ethereal_phys_device_d3d12_compute COMMAND ethereal_phys_device_test d3d12 compute)
	list(APPEND phys_device_tests ethereal_phys_device_d3d12 ethereal_phys_device_d3d12_compute)
endif()
if (DONUT_WITH_VULKAN)
	add_test(NAME ethereal_phys_device_vulkan COMMAND ethereal_phys_device_test vulkan)
	add_test(NAME ethereal_phys_device_vulkan_compute COMMAND ethereal_phys_device_test vulkan compute)
	list(APPEND phys_device_tests ethereal_phys_device_vulkan ethereal_phys_device_vulkan_compute)
endif()
if (DONUT_WITH_DX11)
	add_test(NAME ethereal_phys_device_d3d11 COMMAND ethereal_phys_device_test d3d11)
	list(APPEND phys_device_tests ethereal_phys_device_d3d11)
endif()
if (phys_device_tests)
	set_tests_properties(${phys_device_tests} PROPERTIES TIMEOUT 120 SKIP_RETURN_CODE 77)
endif()

# ---- The kernels on a real device -----------------------------------------------------------------------
# As above: headless, debug runtime, nvrhi validation layer, any nvrhi message fails the test. On Vulkan
# the test names adapter 0 itself (kernel_test.cpp), as the device test does.
#
# engine/src/radix_sort.cpp is compiled into the test: it is the driver the solver uses, written against
# the public kernel interface, and not reachable through the exports of the DLL. engine/src and
# engine/shaders are on the include path for the constant-buffer mirrors (kernel_params.h and the .hlsli
# files it shares with the kernels).

add_executable(ethereal_phys_kernel_test
	kernel_test.cpp
	${phys_engine_dir}/src/radix_sort.cpp)
target_include_directories(ethereal_phys_kernel_test PRIVATE ${phys_engine_dir}/src ${phys_engine_dir}/shaders)
target_link_libraries(ethereal_phys_kernel_test PRIVATE ethereal_phys donut_app)
if (MSVC)
	target_compile_options(ethereal_phys_kernel_test PRIVATE /utf-8)
endif()
set_target_properties(ethereal_phys_kernel_test PROPERTIES FOLDER "ethereal-phys/Tests")

set(phys_kernel_tests "")
if (DONUT_WITH_DX12)
	add_test(NAME ethereal_phys_kernel_d3d12
		COMMAND ethereal_phys_kernel_test d3d12 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_kernel_tests ethereal_phys_kernel_d3d12)
endif()
if (DONUT_WITH_VULKAN)
	add_test(NAME ethereal_phys_kernel_vulkan
		COMMAND ethereal_phys_kernel_test vulkan --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_kernel_tests ethereal_phys_kernel_vulkan)
endif()
if (DONUT_WITH_DX11)
	add_test(NAME ethereal_phys_kernel_d3d11
		COMMAND ethereal_phys_kernel_test d3d11 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_kernel_tests ethereal_phys_kernel_d3d11)
endif()
if (phys_kernel_tests)
	set_tests_properties(${phys_kernel_tests} PROPERTIES TIMEOUT 300 SKIP_RETURN_CODE 77)
endif()

# ---- The data model on a real device ----------------------------------------------------------------------
# As above. engine/src/bvh_builder.cpp (and the radix sort it drives) is compiled into the test: it is the
# builder the geometry library uses for triangle meshes and a solver will use for the shapes, written
# against the public interfaces, and the test runs it with bounds it computes itself, which no public call
# reaches before there is a solver.

add_executable(ethereal_phys_data_test
	data_test.cpp
	${phys_engine_dir}/src/bvh_builder.cpp
	${phys_engine_dir}/src/radix_sort.cpp)
target_include_directories(ethereal_phys_data_test PRIVATE ${phys_engine_dir}/src ${phys_engine_dir}/shaders)
target_link_libraries(ethereal_phys_data_test PRIVATE ethereal_phys donut_app)
if (MSVC)
	target_compile_options(ethereal_phys_data_test PRIVATE /utf-8)
endif()
set_target_properties(ethereal_phys_data_test PROPERTIES FOLDER "ethereal-phys/Tests")

set(phys_data_tests "")
if (DONUT_WITH_DX12)
	add_test(NAME ethereal_phys_data_d3d12
		COMMAND ethereal_phys_data_test d3d12 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_data_tests ethereal_phys_data_d3d12)
endif()
if (DONUT_WITH_VULKAN)
	add_test(NAME ethereal_phys_data_vulkan
		COMMAND ethereal_phys_data_test vulkan --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_data_tests ethereal_phys_data_vulkan)
endif()
if (DONUT_WITH_DX11)
	add_test(NAME ethereal_phys_data_d3d11
		COMMAND ethereal_phys_data_test d3d11 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_data_tests ethereal_phys_data_d3d11)
endif()
if (phys_data_tests)
	set_tests_properties(${phys_data_tests} PROPERTIES TIMEOUT 300 SKIP_RETURN_CODE 77)
endif()
