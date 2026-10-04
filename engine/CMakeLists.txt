# ethereal_phys: the physics engine, a shared library.
#
# Its public surface is COM interfaces (nvrhi's object model) and extern "C" exports prefixed
# etherealPhys; donut/nvrhi/doc/com-interface-abi.md applies in full. The export table holds
# no C++-mangled name (tests/check_exports.cmake).

add_library(ethereal_phys SHARED
	include/ethereal/phys/common/export.h
	include/ethereal/phys/buffer.h
	include/ethereal/phys/collision.h
	include/ethereal/phys/constraints.h
	include/ethereal/phys/device.h
	include/ethereal/phys/domain.h
	include/ethereal/phys/ext/asset.h
	include/ethereal/phys/ext/asset_builder.h
	include/ethereal/phys/ext/container.h
	include/ethereal/phys/ext/force_field.h
	include/ethereal/phys/ext/moving_frame.h
	include/ethereal/phys/kernels.h
	include/ethereal/phys/params.h
	include/ethereal/phys/particles.h
	include/ethereal/phys/readback.h
	include/ethereal/phys/solver.h
	src/bvh_builder.cpp
	src/bvh_builder.h
	src/collision_shape_set.cpp
	src/constraint_sets.cpp
	src/data_model.h
	src/ext/aabb_tree.cpp
	src/ext/aabb_tree.h
	src/ext/asset_builder.cpp
	src/ext/cloth_mesh.cpp
	src/ext/cloth_mesh.h
	src/ext/container.cpp
	src/ext/ext_assets.h
	src/ext/ext_cloth.cpp
	src/ext/ext_rigid.cpp
	src/ext/ext_objects.h
	src/ext/ext_soft.cpp
	src/ext/flex_core.h
	src/ext/flex_math.h
	src/ext/force_field_set.cpp
	src/ext/moving_frame.cpp
	src/ext/sdf.cpp
	src/ext/sdf.h
	src/ext/voxelize.cpp
	src/ext/voxelize.h
	src/geometry_library.cpp
	src/kernel_library.cpp
	src/kernel_library.h
	src/kernel_params.h
	src/kernel_table.inl
	src/particle_state.cpp
	src/phys_buffer.cpp
	src/phys_buffer.h
	src/phys_device.cpp
	src/phys_domain.cpp
	src/phys_messages.h
	src/radix_sort.cpp
	src/radix_sort.h
	src/readback.cpp
	src/readback_impl.h
	src/solver/solver_pbd.cpp
	src/solver/solver_pipeline.cpp
	src/solver/solver_pipeline.h
	src/solver/solver_xpbd.cpp
	src/solver/stage_timers.cpp
	src/solver/stage_timers.h
	src/version.cpp)

# shaders/ is on the private include path for KernelParamsLayout.hlsli, the constant-buffer structs that
# the kernels and src/kernel_params.h share; src/ for the sources in its sub-directories (src/solver, src/ext).
#
# src/ext: the extension layer: the asset builders (IPhysAssetBuilder, ext/asset_builder.h) and the part of
# FleX's core/ they need (the bounding volume hierarchy, the voxeliser, the distance field, the cloth topology,
# FleX's vector, quaternion and matrix arithmetic), the container with its instances and soft joints
# (ext/container.h), the force fields (ext/force_field.h, a stage callback over the kernel
# shaders/ext/UpdateForceFields.hlsl) and the moving frame (ext/moving_frame.h).
target_include_directories(ethereal_phys
	PUBLIC include
	PRIVATE shaders src)

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
#
# ShaderTool::Blob, PRIVATE: the reader of the NVSP permutation container the kernels are compiled into
# (pure C, libc only). The kernel library reads the blobs itself; the DLL does not link donut_engine, whose
# ShaderFactory does the same for Donut.
target_link_libraries(ethereal_phys
	PUBLIC nvrhi_core
	INTERFACE nvrhi
	PRIVATE ShaderTool::Blob)

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

# ---- The kernels ----------------------------------------------------------------------------------------
# shaders/ethereal_phys.cfg lists them; ShaderTool compiles each row to <bin>/shaders/ethereal_phys/<format>/,
# which is what PhysDeviceDesc::shaderPath points at.
#
# - Shader model 6.0 (not the 6_5 default of the helper): the kernels need nothing newer, and the wave
#   intrinsics of the EPHYS_WAVE=1 permutations are core HLSL from 6.0 on.
# - DXIL for D3D12, SPIR-V for Vulkan and DXBC (cs_5_0, D3DCompile) for D3D11; the helper drops the formats
#   of the graphics APIs Donut was configured without.
# - SPIR-V uses the DirectX memory layout: a StructuredBuffer<float3> has a 12-byte stride, as on D3D and as
#   the host fills it (DXC's default layout would make it 16). The Vulkan device needs scalarBlockLayout,
#   which Donut's device manager enables.
file(GLOB_RECURSE ethereal_phys_shader_sources CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.hlsl"
	"${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.hlsli")

if (NOT (DONUT_WITH_DX11 OR DONUT_WITH_DX12 OR DONUT_WITH_VULKAN))
	# ethereal_compile_shaders would create an empty target and the engine would find no kernel at run time.
	message(FATAL_ERROR "ethereal-phys: no graphics API is enabled (DONUT_WITH_DX11 / DX12 / VULKAN), so no "
		"kernel would be compiled")
endif()

ethereal_compile_shaders(
	TARGET ethereal_phys_shaders
	CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ethereal_phys.cfg
	SOURCES ${ethereal_phys_shader_sources}
	FOLDER "ethereal-phys"
	SHADER_MODEL 6_0
	BACKENDS DXBC DXIL SPIRV
	EXTRA_FLAGS_SPIRV -fvk-use-dx-layout)
add_dependencies(ethereal_phys ethereal_phys_shaders)

# Where the kernels go, for the tests and the samples (the default OUTPUT_DIRECTORY of the helper).
set(ETHEREAL_PHYS_SHADER_OUTPUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders/ethereal_phys" PARENT_SCOPE)

# ---- Donut scene graph integration ------------------------------------------------------------------------
# ethereal_phys_scene (static): the physics leaves, PhysSceneGraph and PhysSceneService (scene/CMakeLists.txt).
add_subdirectory(scene)
