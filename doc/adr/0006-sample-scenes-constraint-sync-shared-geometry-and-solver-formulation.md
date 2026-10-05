# 0006. Sample scenes: constraint updates as frame requests, shared collision geometry, force fields as scene-graph leaves, the solver formulation chosen by restarting a scene

- Status: Accepted
- Date: 2026-10-05

## Context

Step 10b of the migration plan (`../plans/2026-10-04-ethereal-phys.md`) ports the rest of FleX's demo families onto the
sample framework of step 10a (ADR 0005): tearing cloth, soft and plastic bodies, inflatables, triangle mesh and
distance field collision, force fields, ropes, moving frames, rigid-fluid coupling and the four XPBD reference scenes.
These scenes use four things of FleX's demo the framework did not have, each of which FleX did in a way the engine
does not allow as it was:

- **A scene changing its constraints while it runs.** FleX's `Scene::Sync` called `NvFlexSetSprings`,
  `NvFlexSetDynamicTriangles`, `NvFlexSetRestParticles` and `NvFlexSetInflatables` on the solver itself, every frame,
  between the particle uploads and the step (the tearing cloth re-sends its springs, triangles and rest positions; the
  inflatables their pressures). The framework records all GPU work of a frame on the frame's command list, and on
  D3D11 a second open command list (the engine device's own) while the frame's is open is not allowed (ADR 0005).
- **Collision meshes shared by several shapes.** FleX's `CreateTriangleMesh` and `CreateSDF` returned an id that
  several `AddTriangleMesh` / `AddSDF` calls named; the shapes of `ethereal_phys_helpers` are plain descriptions
  (`ShapeDesc`), and the library owns no GPU object.
- **Force fields.** FleX's `NvFlexExtCreateForceFieldCallback` registered a callback on the solver, and the scene set
  its fields every frame. In the framework the scene service owns the solver (ADR 0002), and it already owns one
  `IForceFieldSet` per domain that has `PhysForceFieldLeaf`s (ADR 0004).
- **PBD or XPBD.** FleX's `g_params.solverMode` switched the formulation of a running solver, and its panel had a
  slider for it. Here the formulation is the solver's, chosen when it is created (`SolverDesc::mode`: `PhysSolverPBD`
  and `PhysSolverXPBD` are two classes over one pipeline), and the solver belongs to the scene service.

## Decision

1. **Constraint updates are requests on `DemoState`, carried out on the frame's command list.** A scene's `Sync(g)`
   (FleX's hook, called at FleX's point of the frame: after the particles were sent, before the step) calls
   `g.SetSprings()`, `g.SetDynamicTriangles()`, `g.SetRestParticles()` or `g.SetInflatables()`; each sets a flag, and
   `PhysicsScene::Simulate` uploads what the flags name from `g.buffers` on the frame's command list before the step,
   through upload buffers kept for four frames. The renderer's copy of the cloth's indices follows the triangles.
2. **Triangle meshes and distance fields are shared, immutable descriptions.** `CreateTriangleMesh` and `CreateSDF`
   (`ethereal_phys_helpers`) return a `std::shared_ptr` to the mesh with its bounds, or to the volume with the mesh it
   was made from (`TriangleMeshId`, `DistanceFieldId`); `AddTriangleMesh` / `AddSDF` put the pointer into a
   `ShapeDesc`. The framework creates one `ITriangleMeshAsset` or `ISdfAsset`, and one GPU mesh to draw it, per
   description, however many shapes name it, and keeps them for the scene's lifetime.
3. **Force fields are leaves of the scene graph.** A scene writes its fields to `DemoState::forceFields` (the engine's
   `ForceField`); the framework keeps one `PhysForceFieldLeaf` per field, on a node at the field's position, and
   copies the radius, strength, mode and falloff to it every frame; the scene service sends a change to its
   `IForceFieldSet`.
4. **The formulation is chosen by restarting the scene.** A scene sets `g.solverMode` and the compliance block
   `g.xpbd`; `--solver pbd|xpbd` (FleX's `--xpbd` override) and the panel's Solver choice override the mode for the
   scenes that follow, and the panel's choice restarts the current scene. Changes of `g.xpbd` (the panel's stiffness
   range, spring damping and volume compliance) reach a running XPBD solver through `IPhysSolverXPBD::setCompliance`.
5. **What scenes create themselves comes from the engine device.** `DemoState::phys` is the engine's `IPhysDevice`:
   scenes build their assets (`IPhysAssetBuilder`: soft bodies and their skinning, cloths with a pressure, tearing
   cloths) and moving frames (`IMovingFrame`) with it, as FleX's scenes called its extension library.

## Consequences

### Positive

- The ported scenes keep FleX's structure (`Update` changes the host arrays, `Sync` sends them), and every upload of a
  frame is on one command list, on every graphics API.
- A mesh used by five shapes (Triangle Collision) is one asset and one GPU mesh, as in FleX.
- Force fields go through the same scene-service path as any other sample would use, and are covered by the smoke
  tests on three APIs.
- The engine keeps one formulation per solver; nothing in it had to change for the samples.

### Negative

- Switching between PBD and XPBD restarts the scene (FleX switched in place and showed the same state stiffening or
  not). The comparison FleX's XPBD scenes describe ("the tip deflection stays put in XPBD mode and keeps shrinking in
  PBD mode") is made by running the scene once in each mode.
- A scene that syncs every frame (Tearing, the inflatables) uploads its arrays every frame, as FleX did.
- The moving-frame and tearing scenes map the particle arrays every frame (a wait for the GPU), as FleX's demo did.

### Risks

- An asset description must not change after it was added: the framework keys its assets by the description's
  address. The descriptions are `const` behind the pointer.

## Alternatives considered

- **Scenes calling the domain's constraint sets directly from `Sync`.** It needs the frame's command list in the scene
  (or a list of the device's own, which D3D11 cannot have open beside the frame's), and the framework's render copy of
  the cloth's indices would go stale.
- **Re-creating the solver in place on a mode change** (detaching and re-attaching the domain's leaf so the service
  makes a new solver over the same particles). It keeps the state, but the readback and timer objects of the old
  solver, its force field set and its warm state would all have to follow; a restart is what `--solver` does anyway
  and is exercised by the same path.
- **An `IForceFieldSet` per scene, created in `PostInitialize` on the service's solver.** It bypasses the scene graph
  that owns the solver and would compete with the service's own set for the solver's `UpdateEnd` stage.
- **Copying the mesh into every `ShapeDesc`.** One asset per shape: five bowls of Triangle Collision would be five
  hierarchies and five GPU meshes.

## References

- Code: `samples/framework/include/ethereal/phys_framework/demo_state.h` and `scene.h`, `samples/framework/src/physics_scene.cpp`,
  `demo_app.cpp`, `demo_ui.cpp`; `samples/helpers/include/ethereal/phys_helpers/sim_buffers.h` and `helpers.h`;
  `samples/flex_demo/scenes/`.
- Plan: `../plans/2026-10-04-ethereal-phys.md`, "Step 10b as built".
- ADRs 0002 (the scene service owns the solvers), 0004 (force field sets and moving frames), 0005 (the sample framework).
