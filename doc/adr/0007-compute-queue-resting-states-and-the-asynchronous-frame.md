# 0007. The engine on a compute queue: resting states a compute command list can hold, and the asynchronous frame's hand-off

- Status: Accepted
- Date: 2026-10-05

## Context

`PhysDeviceDesc::preferComputeQueue` has existed since step 2, but nothing had ever stepped a solver on a compute queue
(the plan's step 7 note and "Known issues"). With `ethereal_phys_solver_test <d3d12|vulkan> compute`:

- D3D12 removed the device in the first step of the first scene. The debug layer said, before the removal,
  `ID3D12CommandList::ResourceBarrier: D3D12_RESOURCE_STATES has invalid flags for compute command list.` (ten times),
  then `ExecuteCommandLists: Command lists must be successfully closed before execution` and
  `RemoveDevice ... DXGI_ERROR_INVALID_CALL`.
- Vulkan reported `VUID-vkCmdPipelineBarrier2-srcStageMask-09675` (and `-dstStageMask-`):
  `VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT is not compatible with the queue family properties
  (VK_QUEUE_COMPUTE_BIT|VK_QUEUE_TRANSFER_BIT|...)`.

The plan took the Vulkan half to be nvrhi's state-to-stage translation. A temporary print of every barrier nvrhi's D3D12
backend recorded on a compute list (the state bits of each buffer and texture) showed the same cause for both: the
engine created three resources that rest, between command lists, in `ResourceStates::ShaderResource`, which is
`PixelShaderResource | NonPixelShaderResource`. nvrhi puts every buffer or texture with `keepInitialState` back into its
initial state when a command list closes, so the compute list had to transition to the pixel-shader state; D3D12 does not
allow `PIXEL_SHADER_RESOURCE` on a compute list, and in Vulkan that state names the fragment-shader stage. The resources
were the kernel library's dummy buffers for unused read-only slots, its dummy 3D texture, and the textures of signed
distance fields (`ISdfAsset`). No kernel reads any of them in a pixel shader.

The kernel library's binding layouts already have `visibility = Compute`, for which nvrhi picks `NonPixelShaderResource`
when it binds them; only the resting states were wrong.

Separately, the scene service refused to step on the device's own command list when that was on a compute queue, and the
sample framework recorded the whole frame, physics included, on one graphics command list.

## Decision

1. **Resources the engine owns rest in states every queue can hold.** The dummy read-only buffers, the dummy volume and
   the distance field textures rest in `NonPixelShaderResource` instead of `ShaderResource`. Nothing else in the engine
   changes: its other buffers rest in `UnorderedAccess`, `CopySource`, `CopyDest`, `ConstantBuffer` and
   `IndirectArgument`, which a compute command list accepts on D3D12 (checked on two GPUs) and which map to stages a
   compute family has on Vulkan.
2. **nvrhi is not changed.** A state a queue cannot hold is the caller's error on D3D12 as well, where the debug layer
   rejects it; making the Vulkan backend silently drop stages would make the two backends disagree about what is legal.
   A change that masked the stages by queue type (it was written and removed: it also made Vulkan clean) would not have
   fixed D3D12, and was not needed once the engine's states were right.
3. **A buffer shared with the renderer rests in a state both queues can hold, and the renderer's list moves it.** This is
   the rule `IPhysDevice::adoptBuffer` already documents for the engine on a compute queue: render-shared arrays rest in
   `UnorderedAccess` (the engine creates them so), and the renderer's graphics command list transitions them to
   `VertexBuffer` or `IndexBuffer` for the draw and back at the end of the list. A buffer the engine only copies into and
   the renderer reads (`updateRenderArrays` targets, the solver output copies, the cloth indices) rests in `CopyDest` when
   the engine is on a compute queue, not in `VertexBuffer` or `IndexBuffer`.
4. **The scene service steps on whatever queue the device submits to.** The refusal is gone. With a null command list the
   passes record on the device's own list, `SyncFromPhysics` flushes it and returns the instance, and the application
   orders the queues with `queueWaitForCommandList`; with an application list the application's list is on the
   application's queue, as before.
5. **The sample frame alternates between the queues.** With `--async-compute` (default: the graphics queue) the frame's
   command list holds the rendering (and the copies of solver outputs the draw options need) and is executed first;
   the compute queue then waits for it (`queueWaitForCommandList(compute, graphics, renderInstance)`), the physics of
   the frame records on the device's own list and is flushed, and the graphics queue's next submission waits for the
   flush's instance. The step never overlaps the drawing that reads what it writes, so no double buffering is needed
   and the hand-off is exact; the cost is that the two queues do not run at the same time.

## Consequences

- The solver, the scene service, the readback and the timers work on a compute queue on D3D12 and Vulkan. The
  `_compute` test runs cover every solver scene, the XPBD scenes, the asset builders, the container, the readback and the
  timers with active particles, and a scene-service test steps a domain with a shape leaf on the compute queue for a
  renderer on the graphics queue and compares every frame with a synchronous run.
- The ordering is by queue waits, not by CPU waits; the D3D12 debug layer reports a missing wait as a cross-queue hazard.
  Vulkan has no check for it outside synchronisation validation.
- A render array shared with the engine still needs the engine's resting state to be one the engine's queue can hold; an
  application that adopts a vertex buffer resting in `VertexBuffer` and uses it from the compute queue still gets the
  warning of `adoptBuffer`.
- Overlapping a step with the frame that draws the previous one needs the render arrays twice. That is a follow-up, and
  does not change the rules above.

## Alternatives considered

- **Mask the stages by queue type in nvrhi's Vulkan state translation.** It removes the Vulkan validation error but not
  D3D12's, hides a wrong state instead of reporting it, and edits a dependency for a defect that is not there.
- **Insert a CPU wait (`waitForIdle`) between the queues.** Correct, and simpler, but it tests nothing about the
  hand-off and defeats the purpose of a second queue.
- **Double-buffer the render arrays now.** Real overlap, but the renderer would have to track which copy is current, and
  the solver outputs the renderer copies each frame would need the same treatment. Not needed to make the queues work.

## References

- `engine/src/kernel_library.cpp` (the dummy buffers and volume), `engine/src/geometry_library.cpp` (distance fields)
- `engine/scene/include/ethereal/phys_scene/scene_service.h` ("Compute queue")
- `samples/framework/src/physics_scene.cpp` (`SimulateAsync`), `samples/framework/src/demo_app.cpp` (`Render`)
- `tests/scene_test.cpp` (`testAsyncRender`), `tests/solver_support.h` (`Env::submitEngineWork`, `executeOnGraphics`)
- `doc/plans/2026-10-04-ethereal-phys.md`, "Compute queue fix as built."
