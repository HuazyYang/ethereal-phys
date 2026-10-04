# ethereal_phys_utils: the utility library of the samples, a static library.
#
# It holds the parts of FleX's core/ that samples need and the engine must not own: the OBJ, PLY and flat binary
# mesh readers and the procedural primitives (core/mesh.*), the PFM format of the distance field assets
# (core/pfm.*), rope tubes (core/extrude.*), the hull of a plane set (core/convex.h), Perlin noise
# (core/perlin.*), and the voxeliser and distance field of a closed mesh (core/voxelize.*, core/sdf.*, over
# core/aabbtree.*) that the demo's CreateParticleShape samples meshes with. It uses donut::math and std::
# freely: it is linked into executables, never into the engine DLL, and its headers (include/ethereal/phys_utils)
# are not DLL boundary headers, so the ABI checks of the engine do not read them. What the engine's asset
# builders need of core/ is module-private in engine/src/ext (the voxeliser and the distance field are in both
# places, as copies: the DLL does not export its own).

add_library(ethereal_phys_utils STATIC
	include/ethereal/phys_utils/convex.h
	include/ethereal/phys_utils/extrude.h
	include/ethereal/phys_utils/mesh.h
	include/ethereal/phys_utils/perlin.h
	include/ethereal/phys_utils/pfm.h
	include/ethereal/phys_utils/voxelize.h
	src/aabb_tree.cpp
	src/aabb_tree.h
	src/convex.cpp
	src/extrude.cpp
	src/flex_core.h
	src/mesh.cpp
	src/perlin.cpp
	src/pfm.cpp
	src/sdf.cpp
	src/voxelize.cpp)

target_include_directories(ethereal_phys_utils PUBLIC include)
target_link_libraries(ethereal_phys_utils PUBLIC donut_core)

if (MSVC)
	target_compile_options(ethereal_phys_utils PRIVATE /utf-8)  # Donut headers are not /W4 clean
endif()

set_target_properties(ethereal_phys_utils PROPERTIES FOLDER "ethereal-phys")
