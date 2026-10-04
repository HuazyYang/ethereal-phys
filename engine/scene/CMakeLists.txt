# ethereal_phys_scene: the physics leaves of Donut's scene graph and the physics scene service, a STATIC
# library beside the ethereal_phys DLL.
#
# It cannot live in the DLL: donut::engine::SceneGraphLeaf derives from nvrhi::WeakReferenceSourceImpl, an
# implementation class the nvrhi object model declares module-private (nvrhi/core/foundation.h, "Module
# boundaries"); SceneGraph holds std:: containers; donut_core and donut_engine are static libraries. So it is
# compiled into the application, links donut_engine (with RTTI, which Donut's graph needs for its
# dynamic_casts), and reaches the engine through the COM interfaces of ethereal_phys only.
#
# Its headers (include/ethereal/phys_scene) are not DLL boundary headers: the ABI lint, the header hygiene
# check and the header translation units of tests/ stay pointed at engine/include/ethereal/phys.

add_library(ethereal_phys_scene STATIC
	include/ethereal/phys_scene/scene_graph.h
	include/ethereal/phys_scene/scene_service.h
	src/scene_graph.cpp
	src/scene_service.cpp
	src/scene_util.h)

target_include_directories(ethereal_phys_scene
	PUBLIC include
	PRIVATE src)

target_link_libraries(ethereal_phys_scene PUBLIC ethereal_phys donut_engine)

if (MSVC)
	target_compile_options(ethereal_phys_scene PRIVATE /W3 /utf-8)  # Donut headers are not /W4 clean
endif()

set_target_properties(ethereal_phys_scene PROPERTIES FOLDER "ethereal-phys")
