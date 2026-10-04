# ethereal_phys_helpers: the scene-authoring library of the samples, a static library.
#
# The FleX demo's helpers.h (reference/flex/demo/helpers.h, 39 functions behind no header) made a library:
# SimBuffers (what a scene authors, in host memory), the authoring functions over it, and the demo's random
# generator with the sampling functions of core/maths.h its scenes call. It owns no GPU object and dispatches
# nothing: the sample framework (samples/framework) makes the engine domain and the scene-graph leaves from a
# SimBuffers. Headers under include/ethereal/phys_helpers; not DLL boundary headers (the ABI checks of the engine
# do not read them).

add_library(ethereal_phys_helpers STATIC
	include/ethereal/phys_helpers/helpers.h
	include/ethereal/phys_helpers/random.h
	include/ethereal/phys_helpers/sim_buffers.h
	src/helpers.cpp
	src/random.cpp)

target_include_directories(ethereal_phys_helpers PUBLIC include)
# ethereal_phys for the constants of its public headers (makePhase, the shape types and flags); nothing of the
# DLL is called.
target_link_libraries(ethereal_phys_helpers PUBLIC ethereal_phys_utils ethereal_phys)

if (MSVC)
	target_compile_options(ethereal_phys_helpers PRIVATE /utf-8)
endif()

set_target_properties(ethereal_phys_helpers PROPERTIES FOLDER "ethereal-phys")
