# ethereal_phys: the physics engine, a shared library.
#
# Its public surface is COM interfaces (nvrhi's object model) and extern "C" exports prefixed
# etherealPhys; donut/nvrhi/doc/com-interface-abi.md applies in full. The export table holds
# no C++-mangled name (tests/check_exports.cmake).

add_library(ethereal_phys SHARED
	include/ethereal/phys/common/export.h
	include/ethereal/phys/buffer.h
	include/ethereal/phys/device.h
	include/ethereal/phys/params.h
	src/phys_buffer.cpp
	src/phys_buffer.h
	src/phys_device.cpp
	src/version.cpp)

target_include_directories(ethereal_phys PUBLIC include)

# ETHEREAL_PHYS_BUILD_DLL selects dllexport inside the library; every other module gets dllimport
# (include/ethereal/phys/common/export.h).
target_compile_definitions(ethereal_phys PRIVATE ETHEREAL_PHYS_BUILD_DLL)

# How ethereal_phys links nvrhi.
#
# - nvrhi_core, PUBLIC. It is the DLL that owns the process-wide allocator behind MAKE_RC_OBJ and the nvrhi
#   containers, and its include directory is nvrhi's whole include/ tree, so it also provides <nvrhi/nvrhi.h>.
#   This is the only nvrhi binary the DLL links.
# - nvrhi, INTERFACE: usage requirements for the modules that link ethereal_phys (they create the nvrhi
#   device, so they link nvrhi anyway), nothing for the DLL itself. The DLL calls the application's nvrhi
#   objects through their interface vtables and uses header-inline code only (BufferDesc, AutoPtr,
#   nvrhi::string). It calls none of nvrhi's own functions (nvrhiVerifyHeaderVersion, nvrhiGetFormatInfo,
#   nvrhi::utils::*), so it needs no copy of the nvrhi static library, and gets none: with nvrhi linked in,
#   the DLL would carry a second copy of nvrhi's code and static data beside the application's. A reference
#   to one of those functions from engine code is a link error here, which is the intent.
#
# The C runtime. ethereal_phys keeps the runtime of the aggregate (the static CRT, CMAKE_MSVC_RUNTIME_LIBRARY
# in the root CMakeLists.txt), so the DLL has its own CRT instance and heap; nvrhi_core.dll is on the DLL
# CRT, and an application on whatever it chooses. That is safe because nothing CRT-owned crosses the
# boundary: objects (MAKE_RC_OBJ) and nvrhi::string storage come from nvrhi_core's allocator, an object is
# destroyed by the module that created it (Release reaches it through the vtable), and what the engine
# allocates with its own new/malloc it frees itself. The static CRT also keeps the option of linking the
# aggregate's static libraries (donut_core for donut::math) into the DLL later; they are built on it.
target_link_libraries(ethereal_phys
	PUBLIC nvrhi_core
	INTERFACE nvrhi)

# No RTTI, like nvrhi (ADR 0006): type tests go through QueryInterface, never dynamic_cast or typeid.
# Exceptions stay on: the implementation catches std::bad_alloc inside every noexcept entry point and
# returns an FE_* code.
nvrhi_disable_rtti(ethereal_phys)

if (MSVC)
	target_compile_options(ethereal_phys PRIVATE /W4 /utf-8)
endif()

# Only the C exports are visible; inline and template code stays private.
set_target_properties(ethereal_phys PROPERTIES
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
	FOLDER "ethereal-phys")
