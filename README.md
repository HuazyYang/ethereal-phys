# ethereal-phys

A GPU physics engine for the [ethereal](https://github.com/HuazyYang/ethereal) aggregate: position based
dynamics (PBD) and extended PBD (XPBD) solvers over one particle and constraint data model, with GPU
compute through nvrhi and rendering and scene management through Donut.

The engine is being refactored out of a FleX source tree. That tree is kept, unmodified, under
`reference/flex/` as the migration source. It is reference material only: nothing in it is built, and the
CMake files of this repository do not refer to it. Code leaves it by being rewritten into `engine/`.

This repository is not a standalone CMake project. It is a submodule of the ethereal aggregate and is
configured from the aggregate's root `CMakeLists.txt`, after Donut.

| Directory | Content |
|---|---|
| `engine/` | The engine: `ethereal_phys` (shared library; COM interfaces and `etherealPhys*` C exports) |
| `engine/include/` | Public headers, under `ethereal/phys/` |
| `engine/shaders/` | The compute kernels (HLSL, shader model 6.0), their manifest `ethereal_phys.cfg`, the porting record `PORTING.md` and the FleX engineering log `AGENTS.md` |
| `engine/scene/` | Donut scene graph integration (none yet) |
| `tests/` | The ABI checks (lint, header hygiene, GUIDs, export table), the kernel table and shader artifact checks, and the device, kernel and data model tests, run by ctest |
| `samples/` | Samples built on the engine (none yet) |
| `assets/` | Meshes and signed distance fields loaded by the samples |
| `reference/flex/` | The unmodified FleX tree the engine is refactored out of; not built |
| `doc/adr/`, `doc/plans/` | Architecture decision records and implementation plans of this repository |

The migration plan spans repositories and lives in the aggregate: `docs/plans/2026-10-04-ethereal-phys.md`.

## Data and operations

Anything that owns a buffer is data; anything that dispatches a kernel is operations. The data half is in
place: a domain (`IPhysDomain`, `domain.h`) holds one particle state, optional diffuse particles
(`particles.h`), the spring, shape-matching, triangle and inflatable sets (`constraints.h`), the collision
shapes and the simulation parameters; the geometry library of the device (`IGeometryLibrary`, `collision.h`)
holds the triangle mesh, distance field and convex assets that shapes refer to. None of them dispatches a
kernel, with one exception: updating a triangle mesh builds its bounding volume hierarchy. The solvers, which
will read a domain through these interfaces only and keep their scratch to themselves, are the next step.

## Build

From the root of the aggregate:

```powershell
.\build.ps1 -Target ethereal_phys
```

That also compiles the kernels (target `ethereal_phys_shaders`) to `build/bin/shaders/ethereal_phys/<format>/`,
with `<format>` one of `dxil` (D3D12), `spirv` (Vulkan) and `dxbc` (D3D11). An application passes that
directory as `PhysDeviceDesc::shaderPath`.

## Tests

`ethereal_phys` is a DLL, and its public headers follow `donut/nvrhi/doc/com-interface-abi.md` with no
relaxation. The checks that enforce this, and the tests of the engine on a real device, are ctest tests of the
aggregate build (`ETHEREAL_PHYS_BUILD_TESTS`, on by default):

```powershell
.\build.ps1 -Target ethereal_phys,ethereal_phys_header_tus,ethereal_phys_device_test,ethereal_phys_kernel_test,ethereal_phys_data_test
ctest --preset release -R ethereal_phys
```

The kernel tests (`ethereal_phys_kernel_<api>`) create every compute pipeline, dispatch every kernel and
check the constant-buffer layout, the radix sort and the wave variants on a real device, with the
validation layers on. The data model tests (`ethereal_phys_data_<api>`) send every particle array, constraint
set and shape set to the device and read it back, exercise the zero-copy render arrays and their fallback,
the asset ids and lifetimes, and validate the hierarchy a triangle mesh update builds.

A change to a public struct layout or interface bumps `ephys::c_HeaderVersion` (`params.h`).

## License

See `LICENSE.txt`. The files under `reference/flex/` keep their original notices.
