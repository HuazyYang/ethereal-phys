# 0001. Add `ethereal-phys` as a physics engine submodule

- Status: Accepted
- Date: 2026-10-04
- Archived: the aggregate's ADR 0002 (`docs/adr/0002-ethereal-phys-submodule.md`), which stays the
  aggregate's record of the decision. Numbered 0001 here because each repository numbers its ADRs
  independently.

## Context

The aggregate has two submodules, `donut` (which nests `donut/nvrhi`) and `ethereal-samples`. It has no
physics engine. The two simulation codes it does have — the GVDB port and the SPH fluid pool port —
live inside `ethereal-samples/src/gvdb_samples` as sample code linked into the sample executables.

A FleX reference checkout, kept outside the aggregate, contains a completed reverse-engineering of the
FleX solver that stock FleX ships only as a closed DLL: 82 PBD compute kernels as readable HLSL, driven
to byte-identical DXBC against the shipped blob in 79 of 84 shader groups with the remaining 5 runtime
bit-identical, plus a validated XPBD mode on its `feat-xpbd` branch. That is real, verified solver
source to refactor rather than a header over a black box, and it is the basis for a physics engine in
this tree.

The forces on where and how it lands:

- **The engine is a library with consumers, not a sample.** It exposes the FleX public API and
  extension layer as interfaces, two pluggable solvers (PBD and XPBD) over one shared data model, and a
  physics scene service. Samples sit beside it and consume it.
- **nvrhi already defines an ABI-stable COM object model** and the rules for crossing a module
  boundary (`donut/nvrhi/doc/com-interface-abi.md`, nvrhi ADRs 0001 to 0007). `nvrhi_core` is itself a
  DLL built to those rules.
- **donut's scene graph cannot cross a DLL boundary.** `donut::engine::SceneGraphLeaf` derives from
  `nvrhi::WeakReferenceSourceImpl` (`donut/include/donut/engine/SceneGraph.h`), an implementation class
  that the "Module boundaries" note in `donut/nvrhi/include/nvrhi/core/foundation.h` declares
  module-private because its layout depends on the compiler and on
  `NVRHI_PACK_CONTROL_BLOCK_AND_OBJECT`. `SceneGraph` also holds `std::vector`, `std::function` and
  `std::filesystem::path`, and `donut_core` is `STATIC` (`donut/donut-core.cmake`).
- **The FleX solver runs on D3D compute through ~5,150 LOC of hand-rolled D3D11/D3D12 abstraction**,
  with a second device shared with the renderer by registration. nvrhi already provides compute
  pipelines, binding layouts wide enough for the solver's 32 SRV + 8 UAV + 2 CBV dispatch, and
  cross-queue ordering.
- **The gvdb and sph ports set a precedent that does not survive a module boundary.** Their interfaces
  return aggregates by value, reuse virtual names, are `noexcept` nowhere and pass `std::vector` across
  the interface. That is tolerable only because they are in-EXE sample code.
- **The FleX tree carries material that must not enter this repository:** 300MB of prebuilt binaries,
  the shipped DXBC blobs and a 108,877-line decompiler dump, and a license position that is unresolved.

## Decision

Add `ethereal-phys` as a third submodule of the aggregate: a physics engine migrated from the
reverse-engineered FleX solver, with its samples tree beside the engine.

1. **Repository.** `ethereal-phys` is its own repository, added to `.gitmodules` at path
   `ethereal-phys` with no `branch` key, matching `ethereal-samples`. Its history starts from a fresh
   import of the FleX `feat-xpbd` tree as a single initial commit. `bin/`, `lib/`, `external/`,
   `src/dxbc/` and `src/_pseudo/` are not imported. The FleX reference checkout stays outside the
   aggregate; where a build or test needs it, its location comes from a CMake cache variable and is
   never hardcoded.
2. **Two modules.**

   | Module | Kind | Public surface |
   | --- | --- | --- |
   | `ethereal_phys` | `SHARED` | COM interfaces and `etherealPhys*` C exports. Mentions nvrhi and `donut::math` only |
   | `ethereal_phys_scene` | `STATIC` | donut `SceneGraphLeaf` subclasses, `PhysSceneGraph`, `PhysSceneService`. Links `donut_engine` |

   `ethereal_phys` is a DLL and `donut/nvrhi/doc/com-interface-abi.md` applies to its public headers in
   full, with no relaxation: `NVRHI_IID`, single inheritance from `nvrhi::IObject`, every virtual
   `noexcept`, no return-by-value aggregates, no `std::` types across the boundary, no two virtuals
   sharing a name. The donut scene-graph integration is forced out of the DLL into
   `ethereal_phys_scene`, because a `SceneGraphLeaf` subclass exposes the module-private
   `nvrhi::WeakReferenceSourceImpl` layout. `donut::math` types cross the DLL boundary: they have plain
   members and no base classes.
3. **Compute backend.** The solver dispatches its kernels through nvrhi compute on the same
   `nvrhi::IDevice` the renderer uses. No CUDA and no `gp` abstraction.
4. **Scene integration.** Physics objects are leaves on donut's scene graph, under one content flag
   (`0x400`). There is no separate physics scene graph.
5. **Aggregate build.** The root `CMakeLists.txt` gains `option(ETHEREAL_BUILD_PHYS ... ON)`, an
   `EXISTS` guard that fails with the `git submodule update --init --recursive` instruction, and
   `add_subdirectory(ethereal-phys)` placed **after** `add_subdirectory(donut)`.
   `ethereal-phys/CMakeLists.txt` opens with the `if (NOT TARGET donut_app)` guard that
   `ethereal-samples/CMakeLists.txt` uses. `build.ps1` lists `ethereal-phys\CMakeLists.txt` in its
   submodule-presence check.
6. **Enforcement lands with the first public header**, not after: the ABI lint, header hygiene and
   compiles-alone checks pointed at the `ethereal-phys` public include directory, a header-version
   check inside `etherealPhysCreateDevice`, and an export-table test that rejects any C++-mangled name.
7. **Records.** `ethereal-phys` keeps its own ADRs and plans in `ethereal-phys/doc/adr` and
   `ethereal-phys/doc/plans`, numbered from `0001` independently. Its directories are `snake_case` per
   ADR 0001.

## Consequences

### Positive

- The aggregate gains a physics engine with readable, verified solver source, PBD and XPBD together.
- Simulation and rendering share one device. The solver's final kernel writes positions straight into
  the vertex buffer the renderer binds: one state transition per frame, no copies, no registration
  step. The ~5,150 LOC of D3D abstraction and ~85 files of demo renderer backend are deleted, not
  ported.
- The kernels move to SM 6.0 DXIL and SPIRV, so the engine runs on D3D12 and Vulkan, and one wave
  intrinsic variant replaces both vendor-specific paths: the manifest falls from 82 entries to roughly
  58 plus a few wave variants, with no NVAPI or AGS dependency.
- The ABI is a checked contract. The engine can be rebuilt or replaced without relinking its consumers,
  and a stale header fails at device creation instead of corrupting memory.
- The public headers of `ethereal_phys` compile alone, because they do not include donut's scene graph.
- `donut` and `donut/nvrhi` are unchanged. The work touches two repositories: `ethereal-phys` and the
  aggregate.
- No NVIDIA binaries, bytecode or decompiler output enter the new repository.

### Negative

- Two modules where a reader expects one. The scene integration is a separate static library that every
  consumer wanting scene-graph leaves must also link.
- The full ABI rules force three FleX API shapes into less convenient forms: the detail timers become a
  count plus a per-index getter, the three by-value mapped-data returns become views over
  allocator-owned mirrors, and the 20-pointer asset struct becomes an interface released by `Release`.
- Solver stage callbacks change behaviour. A callback no longer receives raw device pointers to launch
  kernels on; it receives `nvrhi::IBuffer*` views and an open command list to record a dispatch on. Any
  out-of-tree callback must be rewritten.
- FleX's byte-exact DXBC acceptance test does not port. It depends on a vendored, pinned compiler and
  keeps running in the FleX reference checkout; `ethereal-phys` inherits the verified sources, not the
  test.
- The new repository has no history. Provenance of each kernel is carried by a source-commit comment,
  and blame stops at the import.
- D3D11 cannot share vertex buffers with the solver through nvrhi and takes a device-to-device copy
  fallback.
- A third submodule lengthens the commit order for cross-repository work and adds one more gitlink to
  bump.

### Risks

- **License.** The FleX `LICENSE.txt` grants redistribution with notices retained unmodified, while
  nearly every source file carries a banner prohibiting distribution without an express license. The
  notices cannot be removed, so they travel with any redistribution while denying it. Dropping the
  blobs and the decompiler dump removes the sharpest exposure, but the recovered HLSL was derived from
  them. Keeping `ethereal-phys` private avoids the question; publishing it does not. Unresolved, and a
  question for counsel.
- **Silent empty shader target.** `ethereal_compile_shaders` creates an empty target when no backend
  survives its gates. A kernel left at DXBC would build cleanly under a D3D12 or Vulkan configuration
  and fail at runtime. Mitigated by moving every kernel to SM 6.0 and by a verification step that
  asserts the shader output directories are non-empty per backend.
- **ABI drift.** A changed `PhysParams` layout against a stale DLL is silent memory corruption.
  Mitigated by the header-version check and `static_assert`s on the layout, landing in the same commit
  as the first header.
- **Copying the wrong precedent.** The gvdb and sph interfaces are the nearest examples in the tree and
  violate the rules this module must follow. Mitigated by stating in the public headers that only their
  data/operations split and contracts are precedent, not their signatures.
- **Behavioural divergence from FleX.** With the byte-exact test gone, correctness rests on behavioural
  parity runs against the reference build and on the published XPBD validation numbers.

## Alternatives considered

- **Static library with relaxed rules, as gvdb and sph did.** Rejected. The relaxations in those ports
  — aggregates returned by value, duplicate virtual names, no `noexcept`, `std::vector` across the
  interface — are safe only inside one executable. A physics engine is consumed by more than one
  program and is exactly the case the nvrhi ABI rules exist for; `nvrhi_core` is already a DLL under
  them. A static engine would also have allowed the scene integration to sit in the same module, which
  hides the fact that donut's scene graph is not ABI-stable instead of confining it.
- **Put the engine inside `ethereal-samples`.** Rejected. That is where gvdb and sph live, and it is
  why they are sample code: no ABI boundary, no independent versioning, and consumers outside the
  samples tree cannot depend on them without depending on every sample. The engine has its own samples,
  which would then be samples of a sample.
- **Keep the solver on CUDA through the `gp` abstraction.** Rejected. The verified kernels are HLSL, so
  a CUDA backend means rewriting them and discarding the verification. It also restores two devices:
  the SPH port goes through a host round trip to reach the renderer only because `gp` and nvrhi are
  separate devices. On nvrhi compute, simulation and rendering share a device and the solver writes the
  render buffer in place.
- **Preserve the FleX git history.** Rejected. The history carries the prebuilt binaries, the shipped
  DXBC blobs and the decompiler dump in every clone, which is both several hundred megabytes and the
  sharpest part of the license exposure. A fresh import with the reference checkout kept on disk loses
  blame and keeps the provenance through per-shader source-commit comments.
- **Keep a parallel physics scene graph.** Rejected. FleX has no articulation hierarchy, so every
  physics object needing a transform is a leaf, and a graph of leaves is a list donut already keeps via
  `RegisterLeaf`. donut already propagates dirty and content flags upward, runs the transform pass and
  keeps the previous frame's transform — which the collision shapes need for continuous collision. A
  parallel graph would reimplement all three and then need reconciling: a second source of truth for
  transforms. Both in-repo precedents, `GVDBVolumeInstance` and `SPHFluidInstance`, took leaves.

## References

- Plan: [`../plans/2026-10-04-ethereal-phys.md`](../plans/2026-10-04-ethereal-phys.md).
- Files: `.gitmodules`, `CMakeLists.txt`, `build.ps1`, `README.md`, `cmake/EtherealShaders.cmake`,
  `donut/include/donut/engine/SceneGraph.h`, `donut/donut-core.cmake`,
  `donut/nvrhi/include/nvrhi/core/foundation.h`.
- ABI rules: `donut/nvrhi/doc/com-interface-abi.md`; nvrhi ADRs 0001 to 0007 in `donut/nvrhi/doc/adr`,
  in particular `0006-no-rtti-queryinterface-casts.md`.
- Related: [0001](../../../docs/adr/0001-snake-case-directory-names.md) (directory names in the new repository).
- Precedents: `ethereal-samples/src/gvdb_samples/gvdb`,
  `ethereal-samples/src/gvdb_samples/sph_fluid_pool`.
