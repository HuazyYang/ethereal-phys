# 0004. Extension containers as clients of a domain and a solver, force fields as a stage callback, a host-side moving frame

- Status: Accepted
- Date: 2026-10-05

## Context

Step 9b of the migration plan (`../plans/2026-10-04-ethereal-phys.md`) ports the rest of FleX's extension library:
the container with its particle allocator, asset instances and soft joints (`reference/flex/extensions/
flexExtContainer.cpp`), the moving frame (`flexExtMovingFrame.cpp`) and the radial force fields
(`extensions/dx/flexExt.cpp`, `flexExt.hlsl`), and gives the scene graph the two leaf kinds step 8 deferred
(`PhysInstanceLeaf`, `PhysForceFieldLeaf`). FleX's demo calls none of the container code. Facts that constrain
the design:

- FleX's container wrapped an `NvFlexSolver`, which was data and operations in one object. Here the data is a
  domain (`IPhysDomain`) and the operations a solver (`IPhysSolver`), and the rule of the engine is that whatever
  drives them does so through their public interfaces (anything that owns a buffer is data, anything that
  dispatches a kernel is operations).
- The container kept host buffers (the "mirrors") that the application mapped, and `NvFlexExtMap*Data` returned
  structs of pointers into them by value, which a virtual function of this DLL may not do
  (`donut/nvrhi/doc/com-interface-abi.md`, R1); memory one module allocates must not be freed by another.
- FleX's push uploaded every mirror in full; its pull recorded GPU copies into the mirrors, and its map waited for
  them. Push, step, push without a pull in between therefore uploads mirrors older than the solver's state.
- Instances and joints were structs the application received as pointers and destroyed through the container;
  the compaction walks them in creation order, and that order fixes every index of the compacted constraints.
- FleX's force field callback was a function registered at `eNvFlexStageUpdateEnd` that launched a kernel on the
  solver's device pointers; `NvFlexRegisterSolverCallback` replaced whatever was registered. Under nvrhi a stage
  callback records on the step's open command list (`SolverStageContext`), and the solver keeps a reference to a
  registered callback.
- FleX applied the moving frame to host arrays (`demo/scenes/localspacecloth.h`, `localspacefluid.h`), the copies
  the demo maps every frame.
- The scene service owns one solver per domain (ADR 0002). A second solver over a domain would advance it twice.

## Decision

1. **A container is a client of one solver and its domain.** `IPhysDevice::createContainer(solver, maxParticles)`
   creates it; it keeps the solver alive and uses only `IPhysDevice::writeBuffer` / `copyBuffer`, the particle
   state's arrays, `setActiveCount` and `download`, the constraint sets' `upload` / `download` and `IPhysSolver::step`.
   While it lives it owns the domain's particle arrays (its first push writes all of its mirrors), the active list
   and the four constraint sets, as FleX's container owned its solver's; the collision shapes and the parameters
   stay the application's. The domain gains no interface for it.
2. **The mirrors are the container's, in nvrhi's process-wide allocator.** The map functions take the struct by
   reference and fill it (the WIDL form): `ContainerParticleData`, `ContainerTriangleData` (with a count FleX did
   not have), `ContainerShapeData` (with the count FleX left unset). The application reads and writes through the
   pointers and never frees them; the header says so in capitals.
3. **Push uploads what the host wrote; a pull is resolved when the host reads.** The container tracks the particles
   written on the host since their last push (all of them while and after the particle data is mapped, those of a
   new instance, those a joint moved) and uploads those runs. A pull records copies into Readback buffers; the first
   host read after it (a map, `createInstance`, `updateInstances`, a joint's `setTransform`, a compaction) waits for
   the device (`waitForIdle`) and copies them into the mirrors. With FleX's usage (map, write, unmap, tick) this
   uploads what FleX uploaded; push, step, push no longer rewinds the solver; a tick never waits.
4. **Instances and joints are objects.** `IPhysInstance` and `IPhysSoftJoint`, created by the container; `Release`
   replaces `NvFlexExtDestroyInstance` / `NvFlexExtDestroySoftJoint`. Each keeps its container (and an instance its
   asset) alive; the container keeps raw pointers to them in creation order, removed by their destructors.
   `createInstance` takes the mapped view, as FleX did, or null for the container's own mirrors.
5. **FleX's arithmetic, FleX's order.** The allocator is FleX's free list (a stack), the compaction and the spawn
   are FleX's statements over FleX's matrix and quaternion types (`engine/src/ext/flex_math.h`), so the indices
   handed out, the compacted arrays and the spawned particles are FleX's to the bit.
6. **Force fields are a stage callback the set owns.** `IPhysDevice::createForceFieldSet(solver)` creates an
   `IForceFieldSet` and registers, at `SolverStage::UpdateEnd`, a small forwarding callback that points back at the
   set without a reference (the set keeps the solver, the solver keeps the callback). The callback writes the
   fields when they changed and dispatches `UpdateForceFields` on the step's command list, then calls the callback
   that was registered before it (FleX replaced it). Releasing the set gives the stage back, or leaves the forwarder
   in a chain registered after it. `ForceField` is FleX's 28-byte layout with `ForceMode : int32_t` (the kernel
   reads an int) and explicit zero padding after the bool.
7. **The moving frame is host math.** `IMovingFrame::apply` works on host arrays, as FleX's did: the container has
   those arrays, the arithmetic is FleX's (bit for bit, compared), and a kernel would add a constant block and an
   index range for the same work.
8. **The scene service owns the containers and the force field sets, one of each per domain that needs them.** A
   container is created over the service's solver of the domain when its first `PhysInstanceLeaf` spawns, with the
   domain's whole capacity; a force field set when a `PhysForceFieldLeaf` first acts in the domain. An instance
   leaf names its domain as a shape does, or takes the graph's only domain; it spawns once, at its node's world
   transform in the domain's simulation space (the primary instance's, ADR 0002), which must be rigid there, and
   holds its instance while it is in a graph. Unregistering the leaf, removing the domain or destroying the service
   releases the instance. `Step` pushes a domain's container before stepping and never pulls. The graph tracks the
   two leaf kinds in lists; the `ResourceTracker<IPhysContainer>` step 8's note anticipated is not needed, because
   no leaf owns a container.
9. **Where FleX had no defined behaviour, the engine defines one.** Bad frees (out of range, free already, given
   twice) are refused (FleX asserted in debug builds); `updateInstances` skips a joint not yet compacted (FleX read
   shape 0 or past the end) and reads the mirrors without requiring a map (FleX read an unmapped buffer); an
   asset whose indices exceed its instance's particles is left out of the compaction with a message; the triangle
   normals are pulled (FleX never filled `NvFlexExtTriangleData::normals`); `IPhysInstance::getInflatableIndex` is
   set (FleX never set it); a force mode that is not one is refused.

## Consequences

### Positive

- The data/ops split holds: a container is to a domain and a solver what an application is, and nothing in the
  domain or the solver knows containers exist.
- Against FleX's shipped extension library on FleX's shipped solver, one container script (allocations, four
  instances, a destroyed one, two force fields, a soft joint created and moved, eleven ticks, a respawn) gives the
  same allocator indices and active lists, bit-identical spawns and moving frame, and positions within 5.1e-6
  (D3D12, Vulkan) or bit-identical (D3D11) after eleven ticks (`tests/container_parity_test.cpp`).
- The scene service steps instance particles without a host round trip: no pull, no wait.

### Negative

- A domain with a container holds container particles only: an application that uploads particles into it
  directly has them overwritten at the next push of what the container wrote.
- The first host read after a pull waits for the whole device, not for the pull's copies alone (as FleX's map
  waited). A pull recorded on an application command list must have been executed before that read; the
  container cannot tell.
- An instance's particle count is fixed at creation: a tearing cloth instanced in a container cannot grow (FleX
  let the application write into the instance's index array; the engine's is read-only). Its constraints are left
  out once they refer to particles beyond the count.
- The scene service never pulls, so the mirrors of a service-owned container hold the spawn values; an application
  that wants the particles on the host pulls first.

### Risks

- Callers that relied on FleX's full push to overwrite device state with stale mirrors (a rewind) get no rewind.
- An application that registers its own `UpdateEnd` callback after creating a force field set must call the
  callback it replaced (`ppPrevious`), or the fields stop acting; nothing reports it.

## Alternatives considered

- **The container as part of the domain (a domain that allocates).** Puts operations (stepping, readback copies)
  into the data half and makes every domain pay for an allocator; rejected by the data/ops rule.
- **FleX's full push.** Simple and faithful, but push, step, push rewinds the simulation, and the scene service
  would have to pull and wait every frame to avoid it.
- **Waiting at pull time.** A stall per tick where FleX had none.
- **Instances as indices or structs in the container.** No lifetime for the asset, and `NvFlexExtInstance` exposes
  mutable arrays the application could corrupt; objects match `IPhysAsset`.
- **The set registering itself as the callback.** A reference cycle (set keeps solver keeps set): the set would never
  be destroyed by its last application reference.
- **The moving frame as a kernel.** Its result would depend on the GPU's rounding, and it needs a particle range the
  application already has on the host.
- **A container per instance leaf, tracked with a `ResourceTracker<IPhysContainer>`.** Two containers over one
  domain would each own the domain's arrays; one per domain, owned with the solver, is what the tracker of domains
  already gives.

## References

- `ethereal-phys/engine/include/ethereal/phys/ext/container.h`, `ext/force_field.h`, `ext/moving_frame.h`, `device.h`
  (`createContainer`, `createForceFieldSet`, `createMovingFrame`), `params.h` (`c_HeaderVersion` 8), `kernels.h`
  (`KernelId::UpdateForceFields`)
- `ethereal-phys/engine/src/ext/container.cpp`, `force_field_set.cpp`, `moving_frame.cpp`, `flex_math.h`;
  `engine/shaders/ext/UpdateForceFields.hlsl`, `engine/shaders/PORTING.md`
- `ethereal-phys/engine/scene/include/ethereal/phys_scene/scene_graph.h`, `scene_service.h`
- `ethereal-phys/tests/container_test.cpp`, `container_parity_test.cpp`, `scene_test.cpp`
- `../plans/2026-10-04-ethereal-phys.md`, "Step 9b as built"; ADR 0002 (the service's ownership and spaces), ADR 0003
  (assets as host objects)
