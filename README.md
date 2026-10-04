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
| `engine/scene/` | `ethereal_phys_scene` (static library): the physics leaves of Donut's scene graph and `PhysSceneService`; headers under `ethereal/phys_scene/` |
| `tests/` | The ABI checks (lint, header hygiene, GUIDs, export table), the kernel table and shader artifact checks, and the device, kernel, data model, solver, XPBD and parity tests, run by ctest |
| `samples/` | Samples built on the engine (none yet), and `samples/common/`: `ethereal_phys_utils` (static library; mesh readers and primitives, the PFM format, rope tubes, convex hulls, Perlin noise; headers under `ethereal/phys_utils/`) |
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
kernel, with one exception: updating a triangle mesh builds its bounding volume hierarchy.

The operations half is the solver (`IPhysSolver`, `solver.h`, `IPhysDevice::createSolver`). It reads a domain
through these interfaces only, keeps its scratch (sorted arrays, grid, contacts, neighbours, half-springs, the
hierarchy over the shapes) to itself, and records a whole step on one command list with `step(commandList, dt,
substeps)`. There are two solvers over the same pipeline (`engine/src/solver/`: `SolverPipeline`, the port of
FleX's `Solver::Update`): `PhysSolverPBD`, and `PhysSolverXPBD` (`SolverMode::XPBD`), which overrides the
pipeline's seams with FleX's `eNvFlexSolverXPBD` branches. The formulation is chosen when the solver is
created; an XPBD solver answers `IPhysSolverXPBD` (`QueryInterface`), which holds what only it has: the
compliance block (`XpbdComplianceParams`) and read-only views of its Lagrange multipliers.

## Extension layer

The asset builders of FleX's extension library are `IPhysAssetBuilder` (`ext/asset_builder.h`,
`IPhysDevice::createAssetBuilder`): welding, cloth (with tethers and pressure), tearing cloth, rigid and soft
bodies from meshes, and soft body skinning, with FleX's algorithms, on the host. A builder returns an
`IPhysAsset` (`ext/asset.h`) whose arrays it owns and the application reads (they have the element types of the
constraint sets), released with `Release`; a tearing cloth is an `IPhysTearingCloth`, an asset that also splits
its mesh along strained springs. The part of FleX's `core/` they need (the bounding volume hierarchy, the
voxeliser, the distance field, the cloth topology) is module-private in `engine/src/ext/`. A builder owns the
random generator that FleX kept per extension module, shared with its tearing cloths; ADR 0003 records the
decisions.

What samples need besides the engine (mesh readers, the PFM distance fields, rope tubes, convex hulls, Perlin
noise) is the static library `ethereal_phys_utils` in `samples/common/`, over Donut's math; it is never linked
into the DLL.

## Scene graph integration

Physics lives as leaves of Donut's own scene graph (`engine/scene/`, `ethereal_phys_scene`), not in a graph of
its own. It is a static library beside the DLL, because Donut's leaf base class is an nvrhi implementation class
that may not cross a module boundary; it links `ethereal_phys` and `donut_engine`, and uses the engine through
its COM interfaces only. `PhysDomainInstance` places a domain in the scene (several may share one, as mesh
instances share a mesh), `PhysShapeLeaf` is one collision shape whose node drives its transform, and
`PhysSceneGraph` tracks both with a `ResourceTracker<IPhysDomain>` (`OnDomainAdded` / `OnDomainRemoved`).
`PhysSceneTypeFactory` creates them for Donut's JSON scene loader (types `PhysDomain` and `PhysShape`).

`PhysSceneService` runs three passes per frame after `SceneGraph::Refresh`: `SyncToPhysics` (the current and
previous world transforms of every shape into the shape sets, in each domain's simulation space),
`Step(dt, substeps)` (one solver per domain, owned by the service) and `SyncFromPhysics` (particle bounds back
to the instances through readback buffers it never waits for). A domain's node transform must be rigid with a
uniform scale; the service rejects any other with a message. `scene_graph.h` and `scene_service.h` state the
contracts; ADR 0002 records the decisions.

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
.\build.ps1 -Target ethereal_phys,ethereal_phys_header_tus,ethereal_phys_device_test,ethereal_phys_kernel_test,ethereal_phys_data_test,ethereal_phys_solver_test,ethereal_phys_xpbd_test,ethereal_phys_parity_test,ethereal_phys_readback_test,ethereal_phys_scene_test,ethereal_phys_utils_test,ethereal_phys_ext_test,ethereal_phys_ext_parity_test
ctest --preset release -R ethereal_phys
```

The kernel tests (`ethereal_phys_kernel_<api>`) create every compute pipeline, dispatch every kernel and
check the constant-buffer layout, the radix sort and the wave variants on a real device, with the
validation layers on. The data model tests (`ethereal_phys_data_<api>`) send every particle array, constraint
set and shape set to the device and read it back, exercise the zero-copy render arrays and their fallback,
the asset ids and lifetimes, and validate the hierarchy a triangle mesh update builds. The solver tests
(`ethereal_phys_solver_<api>`) step granular, fluid, cloth, rigid, inflatable, collision-shape and diffuse
scenes (`tests/solver_scenes.h`) and check plausibility, determinism, zero copy and stage callbacks;
`ethereal_phys_solver_d3d12_vulkan` compares the two graphics APIs. The XPBD tests (`ethereal_phys_xpbd_<api>`)
check `IPhysSolverXPBD`, that the multipliers are cleared at the start of every substep, the four XPBD scenes
of FleX's `feat-xpbd` branch (chain, cantilever beam, hanging cloth, balloons), compliance against
`alpha * m * g`, and substep convergence: the 64 x 64 cloth at rest takes the same shape at 8, 16 and 32
substeps within 0.1% with XPBD, while PBD stiffens with every substep.

The scene tests check the Donut integration: `ethereal_phys_scene_graph` without a device (leaf tracking, the
domain tracker and its callbacks, `Clone`, the content flag, detaching, the JSON factory), and
`ethereal_phys_scene_<api>` with `PhysSceneService` on a device (shape transforms in a domain's space, a particle
block caught by a rising, turning box, placement rejection, bounds without blocking, an application command
list, JSON domains, reference counts).

The extension tests check the asset builders and the samples' utility library: `ethereal_phys_utils` without a
device (every mesh and distance field of `assets/` loads with sane counts and bounds, the binary mesh and PFM
round trips, the primitives, `ConvexMeshBuilder` against its plane set, `Extrude`, Perlin noise), and
`ethereal_phys_ext_<api>` on a device (well-formed assets from every builder, the refusals, a cloth, a rigid
and a soft body asset stepped in a domain, a tearing cloth pulled apart and torn every frame).

The parity tests (`ethereal_phys_parity_<api>`) compare the solver with a reference FleX build, scene by scene,
after 1, 2 and 10 steps. FleX is never part of this repository or of the build: point the CMake cache variable
`ETHEREAL_PHYS_FLEX_REFERENCE_DIR` at a FleX checkout and the test loads its solver DLL at run time
(`ETHEREAL_PHYS_FLEX_REFERENCE_DLL` selects another DLL than `bin/win64/NvFlexReleaseD3D_x64.dll`). Without it
the tests are not registered:

```powershell
.\build.ps1 -CMakeArgs '-DETHEREAL_PHYS_FLEX_REFERENCE_DIR=<path to the FleX checkout>' -Target ethereal_phys_parity_test
```

The XPBD parity tests (`ethereal_phys_parity_xpbd_<api>`) compare the XPBD solver with FleX's XPBD build, the
`NvFlexRev` DLL of a build of the checkout's `feat-xpbd` branch (no shipped DLL has `eNvFlexSolverXPBD`). Point
`ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL` at that DLL (`tests/CMakeLists.txt` says how to build it outside the
checkout); empty, the tests are not registered.

The asset builder parity test (`ethereal_phys_ext_parity`) compares every builder and tearing with FleX's
extension library bit for bit, on procedural meshes and meshes of `assets/`. It is registered with
`ETHEREAL_PHYS_FLEX_REFERENCE_DIR` and loads `bin/win64/NvFlexExtReleaseD3D_x64.dll` of the checkout
(`ETHEREAL_PHYS_FLEX_EXT_REFERENCE_DLL` selects another, e.g. the NvFlexExt of a build of the checkout).

A change to a public struct layout or interface bumps `ephys::c_HeaderVersion` (`params.h`).

## License

See `LICENSE.txt`. The files under `reference/flex/` keep their original notices.
