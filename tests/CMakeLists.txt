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
#
# The solver (IPhysSolver):
#
#   ethereal_phys_solver_<api>        every scene of solver_scenes.h steps cleanly and plausibly, determinism,
#                                     zero copy, stage callbacks, reference counts
#   ethereal_phys_solver_d3d12_vulkan the two graphics APIs agree on every scene within a tolerance
#   ethereal_phys_parity_<api>        the solver against a reference FleX build, scene by scene; registered
#                                     only when ETHEREAL_PHYS_FLEX_REFERENCE_DIR names a FleX checkout
#
# The XPBD solver (IPhysSolverXPBD):
#
#   ethereal_phys_xpbd_<api>          the interface, the multipliers (cleared every substep), the four XPBD
#                                     reference scenes, substep convergence against PBD, compliance, determinism
#   ethereal_phys_parity_xpbd_<api>   the XPBD solver against FleX's XPBD build (NvFlexRev of branch
#                                     feat-xpbd); registered only when ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL
#                                     names that DLL

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

# ---- The solver on a real device ----------------------------------------------------------------------------
# As above. The D3D12 and the Vulkan run each write the state of every scene after ten steps to a file, and
# ethereal_phys_solver_d3d12_vulkan compares the two (a ctest fixture: it runs after both).

add_executable(ethereal_phys_solver_test solver_test.cpp solver_scenes.h solver_support.h)
target_include_directories(ethereal_phys_solver_test PRIVATE ${phys_engine_dir}/src ${phys_engine_dir}/shaders)
target_link_libraries(ethereal_phys_solver_test PRIVATE ethereal_phys donut_app)
if (MSVC)
	target_compile_options(ethereal_phys_solver_test PRIVATE /utf-8)
endif()
set_target_properties(ethereal_phys_solver_test PROPERTIES FOLDER "ethereal-phys/Tests")

set(phys_solver_tests "")
set(phys_solver_dumps "${CMAKE_CURRENT_BINARY_DIR}/solver_dumps/$<CONFIG>")
if (DONUT_WITH_DX12)
	add_test(NAME ethereal_phys_solver_d3d12
		COMMAND ethereal_phys_solver_test d3d12 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR}
			--dump ${phys_solver_dumps}/d3d12.bin)
	list(APPEND phys_solver_tests ethereal_phys_solver_d3d12)
endif()
if (DONUT_WITH_VULKAN)
	add_test(NAME ethereal_phys_solver_vulkan
		COMMAND ethereal_phys_solver_test vulkan --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR}
			--dump ${phys_solver_dumps}/vulkan.bin)
	list(APPEND phys_solver_tests ethereal_phys_solver_vulkan)
endif()
if (DONUT_WITH_DX11)
	add_test(NAME ethereal_phys_solver_d3d11
		COMMAND ethereal_phys_solver_test d3d11 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_solver_tests ethereal_phys_solver_d3d11)
endif()
if (phys_solver_tests)
	set_tests_properties(${phys_solver_tests} PROPERTIES TIMEOUT 600 SKIP_RETURN_CODE 77)
endif()
if (DONUT_WITH_DX12 AND DONUT_WITH_VULKAN)
	set_tests_properties(ethereal_phys_solver_d3d12 ethereal_phys_solver_vulkan
		PROPERTIES FIXTURES_SETUP ethereal_phys_solver_dumps)
	add_test(NAME ethereal_phys_solver_d3d12_vulkan
		COMMAND ethereal_phys_solver_test --compare ${phys_solver_dumps}/d3d12.bin ${phys_solver_dumps}/vulkan.bin)
	set_tests_properties(ethereal_phys_solver_d3d12_vulkan PROPERTIES
		FIXTURES_REQUIRED ethereal_phys_solver_dumps SKIP_RETURN_CODE 77 TIMEOUT 60)
endif()

# ---- The XPBD solver on a real device -------------------------------------------------------------------------
# As above: the interface, the multipliers, the four reference scenes of FleX's XPBD branch, substep
# convergence against PBD, compliance and determinism (xpbd_test.cpp).

add_executable(ethereal_phys_xpbd_test xpbd_test.cpp solver_scenes.h solver_support.h)
target_include_directories(ethereal_phys_xpbd_test PRIVATE ${phys_engine_dir}/src ${phys_engine_dir}/shaders)
target_link_libraries(ethereal_phys_xpbd_test PRIVATE ethereal_phys donut_app)
if (MSVC)
	target_compile_options(ethereal_phys_xpbd_test PRIVATE /utf-8)
endif()
set_target_properties(ethereal_phys_xpbd_test PROPERTIES FOLDER "ethereal-phys/Tests")

set(phys_xpbd_tests "")
if (DONUT_WITH_DX12)
	add_test(NAME ethereal_phys_xpbd_d3d12 COMMAND ethereal_phys_xpbd_test d3d12 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_xpbd_tests ethereal_phys_xpbd_d3d12)
endif()
if (DONUT_WITH_VULKAN)
	add_test(NAME ethereal_phys_xpbd_vulkan COMMAND ethereal_phys_xpbd_test vulkan --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_xpbd_tests ethereal_phys_xpbd_vulkan)
endif()
if (DONUT_WITH_DX11)
	add_test(NAME ethereal_phys_xpbd_d3d11 COMMAND ethereal_phys_xpbd_test d3d11 --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR})
	list(APPEND phys_xpbd_tests ethereal_phys_xpbd_d3d11)
endif()
if (phys_xpbd_tests)
	set_tests_properties(${phys_xpbd_tests} PROPERTIES TIMEOUT 1200 SKIP_RETURN_CODE 77)
endif()

# ---- Parity with a reference FleX build ------------------------------------------------------------------------
# The FleX checkout stays outside the aggregate; its location is this cache variable, empty by default. The
# test loads a DLL of it at run time (LoadLibrary): nothing of FleX is linked or copied. It is registered
# only when the variable is set and the DLL exists. ETHEREAL_PHYS_FLEX_REFERENCE_DLL names another DLL than
# the shipped solver (e.g. the NvFlexRev build of the checkout), relative to the checkout or absolute.

set(ETHEREAL_PHYS_FLEX_REFERENCE_DIR "" CACHE PATH
	"A FleX checkout whose solver DLL the parity tests compare the engine with; empty: no parity tests")
set(ETHEREAL_PHYS_FLEX_REFERENCE_DLL "bin/win64/NvFlexReleaseD3D_x64.dll" CACHE STRING
	"The reference solver DLL, relative to ETHEREAL_PHYS_FLEX_REFERENCE_DIR or absolute")

if (WIN32)
	# Built always (it is code that must keep compiling); without a checkout it reports SKIPPED.
	add_executable(ethereal_phys_parity_test parity_test.cpp solver_scenes.h solver_support.h)
	target_include_directories(ethereal_phys_parity_test PRIVATE
		${phys_engine_dir}/src ${phys_engine_dir}/shaders ${CMAKE_CURRENT_SOURCE_DIR}/../reference/flex/include)
	target_link_libraries(ethereal_phys_parity_test PRIVATE ethereal_phys donut_app d3d11 d3d12 dxgi)
	if (MSVC)
		target_compile_options(ethereal_phys_parity_test PRIVATE /utf-8)
	endif()
	set_target_properties(ethereal_phys_parity_test PROPERTIES FOLDER "ethereal-phys/Tests")

	if (ETHEREAL_PHYS_FLEX_REFERENCE_DIR)
		if (IS_ABSOLUTE "${ETHEREAL_PHYS_FLEX_REFERENCE_DLL}")
			set(phys_flex_dll "${ETHEREAL_PHYS_FLEX_REFERENCE_DLL}")
		else()
			set(phys_flex_dll "${ETHEREAL_PHYS_FLEX_REFERENCE_DIR}/${ETHEREAL_PHYS_FLEX_REFERENCE_DLL}")
		endif()
		if (EXISTS "${phys_flex_dll}")
			set(phys_parity_tests "")
			foreach(api IN ITEMS d3d12 vulkan d3d11)
				if (api STREQUAL "vulkan")
					set(api_option DONUT_WITH_VULKAN)
				elseif (api STREQUAL "d3d12")
					set(api_option DONUT_WITH_DX12)
				else()
					set(api_option DONUT_WITH_DX11)
				endif()
				if (${api_option})
					add_test(NAME ethereal_phys_parity_${api}
						COMMAND ethereal_phys_parity_test ${api} --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR}
							--flex ${ETHEREAL_PHYS_FLEX_REFERENCE_DIR} --flex-dll ${phys_flex_dll})
					list(APPEND phys_parity_tests ethereal_phys_parity_${api})
				endif()
			endforeach()
			if (phys_parity_tests)
				set_tests_properties(${phys_parity_tests} PROPERTIES TIMEOUT 900 SKIP_RETURN_CODE 77)
			endif()
		else()
			message(WARNING "ethereal-phys: ${phys_flex_dll} does not exist; the parity tests are NOT registered")
		endif()
	endif()

	# The XPBD solver against FleX's XPBD build: the NvFlexRev DLL of a build of the checkout's branch
	# feat-xpbd (the shipped DLL and the checkout's other branches have no eNvFlexSolverXPBD). Build it outside
	# the checkout, e.g. from "git archive feat-xpbd"; its ShaderTool submodule is the commit of
	# donut/thirdparty/shader-tool that the gitlink names, and amd_ags_x64.dll and nvToolsExt64_1.dll of its
	# external/ go beside the DLL. Empty by default: no XPBD parity tests.
	set(ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL "" CACHE FILEPATH
		"The NvFlexRev DLL of a FleX feat-xpbd build the XPBD parity tests compare the engine with; empty: none")
	if (ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL)
		if (EXISTS "${ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL}")
			set(phys_parity_xpbd_tests "")
			foreach(api IN ITEMS d3d12 vulkan d3d11)
				if (api STREQUAL "vulkan")
					set(api_option DONUT_WITH_VULKAN)
				elseif (api STREQUAL "d3d12")
					set(api_option DONUT_WITH_DX12)
				else()
					set(api_option DONUT_WITH_DX11)
				endif()
				if (${api_option})
					add_test(NAME ethereal_phys_parity_xpbd_${api}
						COMMAND ethereal_phys_parity_test ${api} --shaders ${ETHEREAL_PHYS_SHADER_OUTPUT_DIR}
							--flex-dll ${ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL} --xpbd)
					list(APPEND phys_parity_xpbd_tests ethereal_phys_parity_xpbd_${api})
				endif()
			endforeach()
			if (phys_parity_xpbd_tests)
				set_tests_properties(${phys_parity_xpbd_tests} PROPERTIES TIMEOUT 900 SKIP_RETURN_CODE 77)
			endif()
		else()
			message(WARNING "ethereal-phys: ${ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL} does not exist; the XPBD "
				"parity tests are NOT registered")
		endif()
	endif()
endif()
