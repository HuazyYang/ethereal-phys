# ethereal-phys -- a GPU physics engine (PBD and XPBD solvers) built on nvrhi and the Donut framework.
#
# This directory is not a standalone CMake project: it is added by the root
# CMakeLists.txt of the ethereal aggregate, after Donut, which provides the
# donut_* / nvrhi targets, ShaderTool and the ethereal_compile_shaders() helper.
#
#   engine/      the engine: ethereal_phys (shared library), its public headers and shaders
#   samples/     samples built on the engine
#   assets/      meshes and signed distance fields loaded by the samples
#
# reference/ (the FleX tree the engine is refactored out of) is not part of the build.

if (NOT TARGET donut_app)
	message(FATAL_ERROR "ethereal-phys must be configured from the ethereal root CMakeLists.txt "
		"(Donut has to be added first).")
endif()

# Runtime assets of the samples. Samples that look assets up by name get this directory as a
# compile definition instead of guessing it from the executable's location.
set(ETHEREAL_PHYS_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")

add_subdirectory(engine)
add_subdirectory(samples)
