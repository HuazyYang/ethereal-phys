# 0002. Physics scene service: solver ownership, shape-to-domain association and simulation space

- Status: Accepted
- Date: 2026-10-05

## Context

Step 8 of the migration plan (`../plans/2026-10-04-ethereal-phys.md`) puts the physics on Donut's own scene
graph: `PhysDomainInstance` and `PhysShapeLeaf` are `donut::engine::SceneGraphLeaf` subclasses, `PhysSceneGraph`
tracks them, and `PhysSceneService` runs `SyncToPhysics`, `Step` and `SyncFromPhysics` beside
`SceneGraph::Refresh`. The plan pins the transform authority per leaf kind (shapes: Donut to physics every
frame, current and previous transform; domain instances: a static placement, rigid with a uniform scale) but
leaves four questions open, each with more than one reasonable answer:

1. Who owns the solver that steps a domain: the leaf, or the service?
2. Which domain does a shape collide with?
3. In which space are a shape's transforms given to a domain, when the domain's instance is placed in the
   scene by a transform of its own, and several instances may share one domain?
4. How do the particle bounds reach `GetLocalBoundingBox`, which Donut calls synchronously during `Refresh`,
   without a GPU wait?

Facts that constrain the answers:

- `MeshInstance::Clone` shares the `MeshInfo`; the GVDB and SPH precedents share their data the same way.
  `Clone` is also what `SceneGraph::Attach` calls when a subgraph already in a graph is attached again.
- FleX simulates in one world space: gravity and the collision planes are in it, and a solver holds no
  per-domain transform. A domain stepped twice in a frame is a domain simulated at twice the rate.
- `ResourceTracker<T>` with `SceneResourceCallback<T>` fires its add callback when the first holder of a
  resource registers and its remove callback when the last one leaves.
- `IPhysDevice::createSolver` records clears on the device's own command list; on D3D11 the device's own list
  and an application's list are both the immediate context, and only one may be open at a time.
- Donut recomputes a node's global bounding box only for structure or transform changes; `InvalidateContent`
  sets neither.

## Decision

1. **The service owns the solvers, one per distinct domain.** It creates a domain's solver when the domain joins
   the graph (`PhysSceneGraph::OnDomainAdded`, or the service constructor for domains already there), with the
   `SolverDesc` of the first instance that holds the domain, and releases it when the last instance leaves
   (`OnDomainRemoved`). The leaf holds its domain and a `SolverDesc`, not a solver. Creating at add time (and
   not inside a per-frame pass) keeps solver creation off an open application command list.
2. **A shape names its domain, or collides with all of them.** `PhysShapeLeaf::SetTargetDomain` (a domain) or
   `SetTargetDomainPath` (a node path, resolved by the service every frame; the JSON `"domain"` property); with
   neither, the shape is given to every domain of the graph. A path that resolves to nothing leaves the shape
   out with one message.
3. **Shapes are expressed in the primary instance's space.** The primary instance of a domain is the first
   instance, in registration order, whose node transform is valid (rigid, uniform positive scale; any other is
   rejected with one message and ignored). A shape whose node has world transform `W` is given to the domain
   as `W * inverse(P)`, `P` the primary instance's current node-to-world transform, for both the current and
   the previous shape transform (the placement is static within a step). The scale left in `W * inverse(P)` is
   baked into the geometry (a sphere radius and an SDF width by a uniform scale only, a capsule radius by its
   equal y / z scale, box extents and mesh scales per axis). Other instances of a shared domain are render-only
   copies. A shape not synchronised in the previous frame sweeps from its current transform.
4. **Bounds are copied, fenced and polled, never awaited.** `SyncFromPhysics` records a copy of each stepped
   domain's `IParticleState::getBounds` into one of a few Readback buffers per domain, fenced after submission
   (an nvrhi event query; a native `ID3D11Query` on D3D11, because nvrhi's D3D11 `pollEventQuery` takes
   `S_FALSE` for done). The next `SyncFromPhysics` polls and maps only finished buffers and gives every instance
   of the domain the newest box; when the box changes it re-sets the node's local transform to its own value so
   that the next `Refresh` recomputes the global box. With all buffers in flight a frame records no copy; a
   later frame records the newest step's instead.

## Consequences

### Positive

- A shared domain is stepped exactly once per frame however many instances render it, and `Clone` is cheap
  and correct.
- Animated collision geometry needs nothing beyond Donut's animation: the previous transform is Donut's.
- `GetLocalBoundingBox` never blocks; the lag is the GPU's (measured: 1 to 2 frames on D3D12 and Vulkan).

### Negative

- A shape that should collide with one domain only must say so; the default (all domains) costs an upload
  per domain.
- Moving a domain's primary instance carries the whole simulation with it: the particles render at the new
  placement, and every shape is re-expressed in the moved space without a sweep (the current and the previous
  shape transform both use the current placement), so a shape can land inside the particles without
  continuous collision. That is the plan's "static placement": a domain node is placed, not animated.
- The service owns the shape set of every domain in its graph: shapes uploaded by other code are replaced.

### Risks

- The solver desc of a shared domain is whichever instance registered first; instances that disagree are not
  reported.
- The bounds refresh relies on Donut treating a re-set of the same local transform as a transform change. A
  Donut change that skips unchanged transforms would leave global boxes stale.

## Alternatives considered

- **Solver per leaf.** Steps a shared domain once per instance (wrong), or needs an election among the leaves
  that share a domain, which is what the tracker already provides.
- **Solver per domain created lazily in `SyncToPhysics`.** Was the first version; on D3D11 with an application
  command list it opened the device's own list beside the application's (an nvrhi error). Kept only as a
  fallback for graphs whose callbacks the application replaced, and not run on D3D11 with an application list.
- **A shape collides with its nearest ancestor domain instance.** Forces collision geometry under the domain's
  node, while animated colliders (a character's hand) live elsewhere in the graph.
- **Shapes in scene space, domains in scene space.** Would require the domain placement to be the identity, or
  a per-domain transform in the solver, which FleX does not have.
- **A synchronous readback in `GetLocalBoundingBox`, or a fixed-lag map without a fence.** The first stalls the
  frame; the second maps memory the GPU may still be writing.

## References

- `ethereal-phys/engine/scene/include/ethereal/phys_scene/scene_graph.h`, `scene_service.h`,
  `engine/scene/src/scene_service.cpp`; tests in `ethereal-phys/tests/scene_test.cpp`.
- Precedents: `ethereal-samples/src/gvdb_samples/gvdb/GVDBScene.{h,cpp}`,
  `ethereal-samples/src/gvdb_samples/sph_fluid_pool/SPHFluidScene.{h,cpp}`; Donut's
  `donut/include/donut/engine/SceneGraph.h`.
- Plan: `../plans/2026-10-04-ethereal-phys.md`, Step 8 and its "as built" note.
