# 0005. Sample framework: the FleX demo's renderer behind DemoContext over nvrhi, particles drawn from the GPU, physics through the scene service

- Status: Accepted
- Date: 2026-10-05

## Context

Step 10 of the migration plan (`../plans/2026-10-04-ethereal-phys.md`, "Steps 9 and 10") ports FleX's demo: its
`main.cpp` (application loop, scene registry, UI, camera, picking, screenshot in one 117 KB file), its 39
scene-authoring functions (`demo/helpers.h`), its scenes (`demo/scenes/*.h`) and its renderer, which is the interface
`demo/demoContext.h` over about 85 files of D3D11 and D3D12 code (`demo/d3d`, `demo/d3d11`, `demo/d3d12`) with HLSL in
`demo/d3d/shaders`. Step 10a is the first part with rendering on screen. Facts that constrain the design:

- The plan names `demoContext.h` as the seam: everything below it goes, replaced by Donut and nvrhi. The samples are to
  look like FleX's: points coloured by phase with a shadow map, meshes, debug lines, diffuse particles and the
  screen-space fluid surface (ellipsoid depth from the anisotropy, depth blur, thickness, composite).
- FleX's shaders are written for OpenGL conventions carried into D3D: column-vector matrices, a right-handed view
  space whose depths are negative (the fluid shaders test `eyeZ >= 0` and divide by `-depth`), an OpenGL projection
  converted to D3D's clip z (`ConvertToD3DProjection`), geometry shaders for the sprites.
- The engine runs on the application's nvrhi device (ADR 0001). Its particle arrays can be the renderer's vertex
  buffers (`IParticleState::setArray`, zero copy) where `PhysDeviceInfo::supportsRenderSharedBuffers`, and are copied
  device to device into them (`setRenderArray`, `updateRenderArrays`) where not (D3D11). The smoothed positions,
  anisotropy and densities are solver outputs (`IPhysSolver::getOutput`), copied into application buffers by
  `IPhysReadback`; the diffuse count lives on the GPU.
- The physics of a scene is to go through the scene service of step 8 (ADR 0002): physics leaves on Donut's scene
  graph, the service owning the solver and running `SyncToPhysics` / `Step` / `SyncFromPhysics` each frame. The
  solver runs on the graphics queue only (the compute-queue defect recorded in step 7), and an application must call
  the device's `runGarbageCollection` every frame.
- FleX's demo maps all its buffers every frame (a wait for the GPU) and its scenes, picking and mesh skinning read and
  write those host copies; its scenes use about 60 globals (`g_params`, `g_buffers`, `g_numSubsteps`, the draw
  options) and its helpers write into `g_buffers`.
- Every sample must run unattended (`--screenshot`) and fail on a validation error, on D3D12, Vulkan and D3D11.

## Decision

1. **Three libraries and one executable per sample.** `ethereal_phys_utils` (`samples/common`) gains FleX's voxeliser
   and 3D distance field (copies of the engine's module-private ones, behind donut::math signatures);
   `ethereal_phys_helpers` (`samples/helpers`) is `demo/helpers.h` as a library: `SimBuffers` (what a scene authors,
   with collision shapes as `ShapeDesc` descriptions instead of FleX geometry ids), the authoring functions taking the
   `SimBuffers` they write into, and the demo's process-wide random generator with the sampling functions step 9a
   left for it; it owns no GPU object. `ethereal_phys_framework` (`samples/framework`) is the rest of `main.cpp` and
   the renderer. A sample is a directory with a `CMakeLists.txt` that calls `ethereal_phys_add_sample()`, found by
   glob; the helper also registers one ctest per graphics API that runs every scene of the sample unattended with the
   validation layers.
2. **`DemoContext` is the seam, kept.** The framework's renderer is FleX's `DemoContext` interface (`demo_context.h`)
   implemented once over nvrhi for every graphics API, with the passes, render states and constants of FleX's D3D11
   renderer and FleX's HLSL ported to shader model 6.0 (DXIL, SPIR-V; DXBC at 5.0) by `ethereal_compile_shaders`, every
   change to a source recorded in `shaders/shaders.cfg`. The demo's matrix conventions are kept as they are
   (`flex_math.h`: FleX's column-major `Matrix44`, `ProjectionMatrix`, `LookAtMatrix`, `ConvertToD3DProjection`); the
   multisampled scene target is resolved into a frame texture that is blitted to the swap chain and is what a
   screenshot reads (without the UI).
3. **Particles are drawn from the GPU, never through host memory.** The position, phase, normal and active-index
   arrays of the domain are the renderer's vertex and index buffers (`setArray`) on D3D12 and Vulkan, and are copied
   into them (`setRenderArray`, `updateRenderArrays`) on D3D11. The smoothed positions, the anisotropy and the
   densities are copied into vertex buffers by `IPhysReadback` when the draw options need them, as FleX's interop path
   did with `NvFlexGet*` into registered buffers; the diffuse particles likewise, with their count copied into the
   arguments of an indirect draw. Cloth is drawn from the position and normal arrays with an index buffer of the
   triangles.
4. **Host copies on demand, as FleX's map.** The particle arrays of `SimBuffers` are host mirrors, valid after the
   upload and stale after a step. A scene that reads or writes them calls `DemoState::MapBuffers()` (FleX's `Update`
   always ran on mapped buffers), as do picking, emitting, mesh skinning and the debug lines: a download recorded on
   the engine device's own list and waited for. A frame that mapped them sends positions, velocities, phases and the
   active list back before it steps, as FleX's frame did.
5. **Physics through the scene service.** A scene's `SimBuffers` becomes one domain under a `PhysDomainInstance` leaf,
   and its shapes `PhysShapeLeaf` nodes of the same `PhysSceneGraph` (the framework creates the `IConvexAsset` of a
   convex from its planes); a `PhysSceneService` creates the solver when the domain joins the graph and runs its three
   passes every frame on the frame's graphics command list. The framework keeps FleX's frame order: update on the
   host, render the state the previous step left, then step, all on one command list; the time step is fixed
   (`DemoState::dt`), never the wall clock.
6. **FleX's globals become one `DemoState`.** Scenes are FleX's scene classes with `g_x` replaced by `g.x` (the
   `DemoState` their hooks get) and the helpers' implicit `g_buffers` passed explicitly; the framework resets
   `DemoState` before every `Initialize` exactly as FleX's `Init` did (and leaves what FleX left, e.g. the wind
   frequency a scene may scale).
7. **Validation fails the run.** A `ValidationMonitor` counts nvrhi's messages, Donut's log warnings and errors (the
   Vulkan layer reports there), the D3D11 / D3D12 info queues (read after every frame, breaks switched off) and the
   engine's messages; an unattended run exits non-zero when any was reported or the frame is one colour.

## Consequences

### Positive

- One renderer for three graphics APIs instead of FleX's two backends, and the scenes look like FleX's: the same
  frames captured from FleX's shipped demo and from `flex_demo` are, scene by scene, the same picture up to the
  simulation's own divergence (the step 10a note of the plan).
- No particle data crosses to the host to be drawn; on D3D12 and Vulkan nothing is copied at all.
- The physics of every sample exercises the scene service, the render arrays and the readback object of the engine,
  under the validation layers, in ctest.
- A ported scene stays line-for-line comparable with FleX's; new samples need no CMake edit.

### Negative

- A scene that skins meshes or draws host debug lines waits for the GPU every frame (FleX did as well).
- The framework carries the demo's conventions (column-major matrices, an OpenGL projection, geometry shaders) rather
  than Donut's; Donut's scene renderer and materials are not used for drawing.
- D3D11 pays a device-to-device copy of four particle arrays per frame.

### Risks

- nvrhi on D3D11 terminates the process (`std::out_of_range` in a `noexcept` function) when a vertex buffer is bound to
  a slot the input layout does not declare; the validation layer does not catch it. The framework binds exactly the
  streams of each layout.
- nvrhi's D3D11 `pollEventQuery` takes "not yet" for done (`scene_service.cpp` has the same note); the framework reads
  its diffuse count through a ring of readback buffers older than the frames in flight instead of event queries.

## Alternatives considered

- **Donut's scene renderers (forward or deferred shading of mesh instances).** They do not draw sprites, ellipsoid
  depth or FleX's fluid composite, and would not look like FleX's demo, which the step asks for.
- **Vertex pulling from structured buffers (`SV_VertexID`) instead of vertex buffers.** It would avoid the D3D11 copies,
  but changes every particle shader and the engine's render-array contract was designed for vertex buffers.
- **Host copies of the render data every frame (FleX's non-interop path).** A readback and an upload of every array
  per frame and a wait for the GPU, for nothing the GPU path lacks.
- **GPU skinning of the rigid meshes.** Removes the per-frame wait of the skinned scenes, but is a new shader FleX has
  not; left for when a scene needs it.
- **One executable per scene.** FleX switches scenes inside one process, which is also what exercises resource
  release; the registry and its keys are kept.

## References

- Code: `samples/framework` (`demo_app.cpp`, `demo_context.cpp`, `physics_scene.cpp`, `validation.cpp`, `shaders/`),
  `samples/helpers`, `samples/common/src/voxelize.cpp`, `sdf.cpp`, `samples/flex_demo`, `samples/CMakeLists.txt`,
  `tests/helpers_test.cpp`.
- Plan: `../plans/2026-10-04-ethereal-phys.md`, "Step 10a as built".
- ADRs 0001 (the DLL and the render arrays), 0002 (the scene service), 0003 (the utility library).
