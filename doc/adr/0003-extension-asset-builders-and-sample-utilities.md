# 0003. Extension asset builders as host objects, and a sample-side utility library

- Status: Accepted
- Date: 2026-10-05

## Context

Step 9a of the migration plan (`../plans/2026-10-04-ethereal-phys.md`) ports the asset half of FleX's extension
library (`reference/flex/extensions/flexExtCloth.cpp`, `flexExtRigid.cpp`, `flexExtSoft.cpp`) and the part of
`reference/flex/core/` it and the samples need. Facts that constrain the design:

- `NvFlexExtAsset` is a struct of 20 raw pointers that the extension library allocates with `new[]` and
  `NvFlexExtDestroyAsset` frees: memory of one module's heap freed through another, which
  `donut/nvrhi/doc/com-interface-abi.md` (section 6) forbids for `ethereal_phys.dll`. The application also wrote
  into it: the tearing demo copies the simulated positions into `asset->particles` before every tear.
- The builders are free C functions. A DLL of this engine exports two `extern "C"` functions and nothing else
  (ADR 0001); everything else is reached through an interface.
- FleX's extension module keeps one random generator in two globals (`core/maths.cpp`: `seed1`, `seed2`), zero
  until `NvFlexExtCreateSoftFromMesh` calls `RandInit` (its surface sampling draws from it), and a generator at
  zero stays at zero. `NvFlexExtTearClothMesh` draws from the same generator to choose which particle of a strained
  spring splits. The builders' outputs therefore depend on the order of the calls made in the process.
- The builders need FleX's bounding volume hierarchy, voxeliser, fast-marching distance field and half-edge cloth
  topology, and its vector arithmetic: a division by a scalar is a multiplication by the reciprocal, a length of
  zero is 0 without a square root. Donut's math differs in those details, and the DLL's public surface mentions
  nvrhi only (step 4 as built: no `donut::math` in the DLL).
- The samples need what the FleX demo takes from `core/` besides that: OBJ and PLY readers and primitives, the PFM
  format of the distance field assets, rope tubes, the hull of a plane set, Perlin noise. The engine has no use for
  any of it.

## Decision

1. **Assets are host objects.** `IPhysAsset` (`ext/asset.h`) holds the arrays of an `NvFlexExtAsset` in the
   library's own storage and hands out `const` pointers with counts, one accessor per field; `Release` replaces
   `NvFlexExtDestroyAsset` and `NvFlexExtDestroyTearingCloth`. The arrays keep FleX's layouts and element types
   (the shape offsets without the leading 0 of `IShapeMatchSet`). An asset owns no GPU buffer and records nothing.
2. **The builder is an object of the device.** `IPhysDevice::createAssetBuilder` creates an `IPhysAssetBuilder`
   (`ext/asset_builder.h`, `c_HeaderVersion` 7) with one method per FleX function, under FleX's names
   (`createWeldedMeshIndices`, `createClothFromMesh`, `createTearingClothFromMesh`, `createRigidFromMesh`,
   `createSoftFromMesh`, `createSoftMeshSkinning`); the tuning arguments of the cloth and soft body builders are
   structs with FleX's argument names. It is a factory, not an accessor: every builder is independent. The builder
   keeps the device alive, for its message callback.
3. **Tearing is a tearing-capable asset.** `IPhysTearingCloth : IPhysAsset` (the plan's `IPhysTearingOps` as
   methods on the asset it operates on) adds `setParticles`, which replaces the application's write into
   `NvFlexExtAsset::particles`, and `tearClothMesh` (`NvFlexExtTearClothMesh`, with `TearingParticleClone` and
   `TearingMeshEdit` and caller-provided arrays with capacities). Its spring arrays are those of its topology, as
   in FleX.
4. **A builder is FleX's extension module.** It owns the random generator FleX kept per module, starting at zero;
   `createSoftFromMesh` seeds it as FleX did, and the tearing cloths the builder created draw from it (they share
   it). Two builders never share one. Given the same calls in the same order, a builder reproduces FleX's
   sequence.
5. **FleX's algorithms, to the bit.** The builders evaluate in FleX's order with FleX's types
   (`engine/src/ext/flex_core.h`), and FleX's behaviour is kept where it is defined, quirks included (a vertex
   within the threshold of two unique vertices is reassigned to the later one; edits beyond `maxEdits` are applied
   to the topology but not reported; a rigid mesh that occupies no voxel gives an asset without particles). The
   builders differ only where FleX had no defined behaviour: null arrays, indices out of range, a triangle index
   count not a multiple of 3, non-finite input, more than 64 voxels per side and a sampling without particles are
   refused with `FE_INVALID_ARGS` and one message (FleX crashed, read out of bounds or returned `NULL`), and the
   storage that FleX left uninitialised is zero.
6. **`core/` is split three ways.** Engine-private (`engine/src/ext/`): what the builders call, nothing more:
   FleX's vector arithmetic, random generator and two ray tests (`flex_core.h`), `AABBTree`, `Voxelize`, the 3D
   `MakeSDF` and `ClothMesh`. Samples (`samples/common/`, the static library `ethereal_phys_utils`, over Donut's
   math and `std::`): the mesh readers and primitives, PFM, `Extrude`, `ConvexMeshBuilder`, Perlin noise; its
   headers are not DLL boundary headers and the ABI checks do not read them. Dropped: what Donut, nvrhi, std or stb
   provide (`png`, `tga`, `matnn`, `mat22`, `free_ptr`, most of `platform`, the generic vector, matrix,
   quaternion and colour math) and what nothing calls (the 2D `MakeSDF`, `ClothMesh::IsSingularVertex`,
   `AABBTree::TraceRaySlow` and its debug drawing).
7. **The source is the reference.** Parity is with what FleX's source computes. The extension library NVIDIA
   shipped normalises the skinning weights with `rcpps` and a Newton-Raphson step (an optimisation of its build,
   whose bits depend on the CPU) where the source divides; the engine divides, as a build of the FleX checkout does,
   and the parity test explains each such difference exactly instead of accepting a tolerance.

## Consequences

### Positive

- No memory crosses the boundary to be freed, and the application cannot corrupt an asset by writing into it.
- The extension layer adds one factory to `IPhysDevice` and no export.
- `tests/ext_parity_test.cpp` compares every builder and tearing with FleX bit for bit: identical on every array
  against a build of the checkout, and against the shipped DLL except for the skinning normalisation above.
- `ethereal_phys_utils` gives the samples of step 10 FleX's names (`Mesh::m_positions`, `ImportMesh`,
  `ConvexMeshBuilder`, ...), so the demo's scene code ports with few edits.

### Negative

- An application that wrote `NvFlexExtAsset` fields directly now calls `setParticles` (tearing) or copies the
  arrays.
- FleX's tearing reads, within one call, the positions of particles it cloned earlier in that call, which nothing
  has written: uninitialised memory in FleX, zero here. A split that measures such a particle measures it at the
  origin. Kept for parity; with `maxSplits` 1 (FleX's water balloon demo) the case does not arise.
- `ethereal_phys_utils` and `engine/src/ext` hold two ports of overlapping FleX code (vector math, `SafeNormalize`),
  one over FleX's types for bit parity and one over Donut's.

### Risks

- A builder's results depend on the order of its calls, as FleX's did: an application that needs reproducible
  tearing creates its soft bodies and tearing cloths in a fixed order, or uses one builder per purpose.
- FleX's quirks kept for parity (the tearing read above, the reassignment in welding, the `%` of the periodic Perlin
  noise for negative coordinates) are behaviour samples may come to rely on.

## Alternatives considered

- **A single view struct (`getData()` returning a `const` struct of pointers and counts).** One vtable slot instead
  of twenty-three, but a struct whose pointers a tearing call invalidates, and a layout to freeze; per-field
  accessors follow the rest of the engine.
- **`IPhysDevice::getAssetBuilder()`, one builder per device.** The random generator would then be shared by
  everything built on the device, and every test and sample would see the generator state of all the others.
- **A generator per tearing cloth.** Not FleX's sequence: FleX's tearing draws from the generator the soft body
  builder seeded, and parity with FleX would be lost after the first soft body.
- **Copying a clone's position into its slot during the tear.** What FleX probably meant, and harmless for the
  demo, but it changes the result of every call with `maxSplits` above 1 that touches a fresh clone, and no
  reference could check it. Left open for after the samples show whether it matters.
- **The core tooling over Donut's math inside the DLL.** The DLL does not link `donut_core`, and Donut's operators
  are not FleX's (a vector division by a scalar divides each component): the outputs would differ in the last bits.
- **The utilities in `engine/utils/`.** Next to the engine, but they are not the engine's and must never be linked
  into the DLL; `samples/common/` puts them where their users are, beside the helpers library of step 10.

## References

- `ethereal-phys/engine/include/ethereal/phys/ext/asset.h`, `ext/asset_builder.h`, `device.h`
  (`createAssetBuilder`), `params.h` (`c_HeaderVersion` 7)
- `ethereal-phys/engine/src/ext/`, `ethereal-phys/samples/common/`
- `ethereal-phys/tests/ext_parity_test.cpp`, `ext_test.cpp`, `utils_test.cpp`
- `../plans/2026-10-04-ethereal-phys.md`, "Steps 9 and 10" and "Step 9a as built"
- ADR 0001 (the DLL link model)
