# ethereal_phys: the physics engine, a shared library.
#
# Its public surface is COM interfaces (nvrhi's object model) and extern "C" exports prefixed
# etherealPhys; donut/nvrhi/doc/com-interface-abi.md applies in full. The export table holds
# no C++-mangled name.

add_library(ethereal_phys SHARED
	include/ethereal/phys/common/export.h
	src/version.cpp)

target_include_directories(ethereal_phys PUBLIC include)

# ETHEREAL_PHYS_BUILD_DLL selects dllexport inside the library; every other module gets dllimport
# (include/ethereal/phys/common/export.h).
target_compile_definitions(ethereal_phys PRIVATE ETHEREAL_PHYS_BUILD_DLL)

# nvrhi brings nvrhi_core (the object model: IObject, AutoPtr, ObjectImpl) as a PUBLIC dependency.
target_link_libraries(ethereal_phys PUBLIC nvrhi)

# Only the C exports are visible; inline and template code stays private.
set_target_properties(ethereal_phys PROPERTIES
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
	FOLDER "ethereal-phys")
