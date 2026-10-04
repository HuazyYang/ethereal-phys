# Migrate FleX into `ethereal-phys`

- Status: Approved
- Date: 2026-10-04
- Archived: approved in the aggregate as `docs/plans/2026-10-04-ethereal-phys.md`, and maintained here
  from step 5 on. The ADR it implements is this repository's
  [0001](../adr/0001-ethereal-phys-submodule.md), archived from the aggregate's ADR 0002.

## Context

The FleX reference checkout is not stock NVIDIA FleX. Stock FleX ships its solver only as a closed
`NvFlexReleaseD3D_x64.dll`; the reference checkout contains a completed reverse-engineering of it —
target `NvFlexRev`, whose 82 PBD compute kernels exist as readable HLSL and were driven to
byte-identical DXBC against the shipped blob in 79 of 84 shader groups, the remaining 5 runtime
bit-identical. Branch `feat-xpbd` adds a validated XPBD mode on top (commit `2eed78e`, with an 813-line
design document).

That is what makes this migration worth doing: there is real, verified solver source to refactor, not
just a header over a black box. The work turns it into `ethereal-phys`, a physics engine submodule of
the `ethereal` aggregate that takes rendering and scene management from donut and GPU compute from
nvrhi — deleting ~5,150 LOC of hand-rolled D3D abstraction (`src/ContextD3D11.cpp`,
`src/ContextD3D12.cpp` in the FleX tree) and ~85 files of demo renderer backend.

Intended outcome: PBD and XPBD as two pluggable solvers over one shared particle/constraint data
model; a physics scene service whose leaves attach to donut `SceneGraphNode`s; the FleX public API as
nvrhi-COM interfaces; the FleX extension layer as engine extension interfaces; a samples tree beside
the engine.

The FleX reference checkout is kept outside the aggregate and is never a submodule of it. Wherever the
build or a test needs it — the behavioural parity runs in Verification — its location is supplied by a
CMake cache variable and never hardcoded.

### ADRs

- [`../adr/0001-ethereal-phys-submodule.md`](../adr/0001-ethereal-phys-submodule.md) — the addition of
  `ethereal-phys` as a third submodule, the DLL link model, the two-module split, the nvrhi compute
  backend and the aggregate build wiring.
- [`../../../docs/adr/0001-snake-case-directory-names.md`](../../../docs/adr/0001-snake-case-directory-names.md) — directory
  names in the new repository are `snake_case`.
- The nvrhi ADRs 0001 to 0007, indexed at
  [`../../../donut/nvrhi/doc/adr/README.md`](../../../donut/nvrhi/doc/adr/README.md) — the COM object model,
  IIDs, `QueryInterface` chains, `AutoPtr` and the no-RTTI rule that the `ethereal_phys` public surface
  is written to. The rules themselves are in
  [`../../../donut/nvrhi/doc/com-interface-abi.md`](../../../donut/nvrhi/doc/com-interface-abi.md).

### Decisions taken before step 1

| Question | Decision |
| --- | --- |
| Source branch | `feat-xpbd` (= `dev` + `2eed78e`), so PBD and XPBD arrive together |
| New repo history | Fresh import, single initial commit; the FleX reference checkout stays on disk as reference |
| Prebuilt binaries | Drop `bin/` (300MB), `lib/` (19MB), `external/` (60MB) |
| RE artifacts | Drop `src/dxbc/` (1.9MB blobs) and `src/_pseudo/` (108,877-line IDA dump) |
| Engineering log | Keep `src/shaders/AGENTS.md` (46.5KB) as engine documentation |
| Parity testing | Behavioural, against the reference FleX build — no NVIDIA binaries or bytecode in `ethereal-phys` |
| Link model | **DLL**, with [`../../../donut/nvrhi/doc/com-interface-abi.md`](../../../donut/nvrhi/doc/com-interface-abi.md) applying in full, no relaxation |
| Compute backend | nvrhi compute. Not CUDA, not the `gp` abstraction |
| Samples scope | Representative subset first (~8–12 scenes spanning each feature family) |
| Isolation | Second full clone of the aggregate off `main`, feature branch there |

### Why a second clone and not a worktree

A git worktree cannot be used for this tree. The submodule gitdirs live under the shared common
directory and each pins a single `core.worktree` — verified: `.git/modules/donut` → `../../../donut`,
`.git/modules/donut/modules/nvrhi` → `../../../../../donut/nvrhi`, `.git/modules/ethereal-samples` →
`../../../ethereal-samples`. A second worktree plus the `git submodule update --init --recursive` that
`build.ps1` runs rewrites those paths and breaks the submodules in the original checkout. The
FrameGraph plan hit this and recorded it in the Branching section of
[2026-10-04-framegraph.md](../../../docs/plans/2026-10-04-framegraph.md#branching).

A second clone of the aggregate has its own `.git/modules`, so donut, nvrhi and ethereal-samples are
independent there and the original checkout, which holds the `framegraph` branch, is untouched.

### The DLL decision forces two modules, not one

`donut::engine::SceneGraphLeaf` derives from `nvrhi::WeakReferenceSourceImpl`
([SceneGraph.h:52](../../../donut/include/donut/engine/SceneGraph.h#L52)) — an *implementation* class that
the "Module boundaries" note in
[`foundation.h`](../../../donut/nvrhi/include/nvrhi/core/foundation.h#L31) declares module-private,
because its layout depends on the compiler and on `NVRHI_PACK_CONTROL_BLOCK_AND_OBJECT`. `SceneGraph`
also holds `std::vector`, `std::function` and `std::filesystem::path`, and `donut_core` is `STATIC`
([donut-core.cmake:36](../../../donut/donut-core.cmake#L36)).

The scene-graph leaves and the physics scene service therefore cannot live inside `ethereal_phys.dll`.
The split is forced:

| Module | Kind | Public surface |
| --- | --- | --- |
| `ethereal_phys` | **SHARED** | COM interfaces + `etherealPhys*` C exports. Mentions nvrhi and `donut::math` only |
| `ethereal_phys_scene` | **STATIC** | donut `SceneGraphLeaf` subclasses, `PhysSceneGraph`, `PhysSceneService`. Links `donut_engine` |

This is not a relaxation of the ABI rules — it is the correct way to satisfy them. `donut::math`
*does* cross the boundary legally: `dm::box3`/`dm::affine3`/`dm::float3` have plain members and no
base classes, and `math.h` includes nothing on the banned list.

## Steps

Only two repositories are touched: the new `ethereal-phys` and the aggregate. `donut` and
`donut/nvrhi` are left alone — nvrhi's binding limits already accommodate the solver's layout (step 3).

| Step | Lands | Repository | Delivers |
| --- | --- | --- | --- |
| 0 | Clone, plan, ADR | aggregate | Second clone of the aggregate, branched `ethereal-phys` off `main`; this plan and root ADR 0002 written there |
| 1 | Repo + build wiring | ethereal-phys, aggregate | Empty `ethereal-phys` builds from the aggregate |
| 2 | ABI foundation | ethereal-phys | `params.h`, `buffer.h`, `device.h`, ABI lint, header version |
| 3 | Shader port | ethereal-phys | 82 kernels at SM 6.0 DXIL+SPIRV, one binding layout, `IPhysKernelLibrary` |
| 4 | Data model | ethereal-phys | `IPhysDomain`, `IParticleState`, 4 constraint sets, shapes, geometry assets |
| 5 | PBD solver | ethereal-phys | `IPhysSolver` end to end |
| 6 | XPBD solver | ethereal-phys | `IPhysSolverXPBD`, compliance CB, multiplier buffers |
| 7 | Readback | ethereal-phys | `IPhysReadback`, `IPhysTimers` |
| 8 | Scene service | ethereal-phys | `ethereal_phys_scene`, content flag `0x400` |
| 9 | Extensions | ethereal-phys | Asset builders, tearing, container/instancing, joints, frame, force fields |
| 10 | Samples | ethereal-phys | ~8–12 scenes beside the engine |

### Step detail

**Step 0** writes this plan and [root ADR 0002](../adr/0001-ethereal-phys-submodule.md) in a second
clone of the aggregate, and branches that clone `ethereal-phys` off `main`. Everything below happens in
that clone.

**Step 1** creates the `HuazyYang/ethereal-phys` remote, imports the `feat-xpbd` tree minus the dropped
directories as a single initial commit, and adds it as a submodule of the clone. Then, mirroring how
`ethereal-samples` is wired:

- `.gitmodules` — add `[submodule "ethereal-phys"]` with `path` and `url`. Match `ethereal-samples`
  and omit the `branch` key.
- `build.ps1` — add `'ethereal-phys\CMakeLists.txt'` to the submodule-presence array. This literal
  list is the one place that needs it.
- Root `CMakeLists.txt` — `option(ETHEREAL_BUILD_PHYS ... ON)`, an `EXISTS` guard with the
  `FATAL_ERROR` telling the user to run `git submodule update --init --recursive`, then
  `add_subdirectory(ethereal-phys)` **after** `add_subdirectory(donut)`.
- Root `README.md` — new rows in the directory and CMake-option tables.
- `ethereal-phys/CMakeLists.txt` — the `if (NOT TARGET donut_app) message(FATAL_ERROR ...)` guard
  that `ethereal-samples/CMakeLists.txt` opens with, an exported assets dir, then `engine/` and
  `samples/`.
- `ethereal-phys/doc/adr/README.md` and `ethereal-phys/doc/plans/README.md`, adapted from the
  `ethereal-samples` copies, plus the new repo's row in the aggregate's
  [`../../../docs/adr/README.md`](../../../docs/adr/README.md) table.

Directories are `snake_case` per [root ADR 0001](../../../docs/adr/0001-snake-case-directory-names.md); headers
are PascalCase with `UPPER_SNAKE_CASE` include guards closed `#endif /* NAME */`.

**Step 2** is the commit that fixes the ABI, and the enforcement must land with it, not after.

Public headers go under `ethereal-phys/engine/include/ethereal/phys/`, namespace `ephys`. They are
written to the strict rules: `NVRHI_IID` + `NVRHI_DECLARE_UUID_TRAITS`, single inheritance from
`nvrhi::IObject`, every virtual `noexcept`, no return-by-value aggregates, no `std::` types across the
boundary, no two virtuals sharing a name, no data members, no destructor.

*`NvFlexParams` crossing the boundary.* ~60 flat fields, ~380 bytes. Return-by-value is illegal
(MSVC passes the hidden return pointer after `this`, Itanium ABIs before it); ~60 setter/getter pairs
would freeze a 120-entry vtable forever. Use three methods:

```cpp
virtual void setParams(const PhysParams& params) noexcept = 0;
[[nodiscard]] virtual const PhysParams& getParams() const noexcept = 0;   // does not AddRef
[[nodiscard]] virtual uint64_t getParamsRevision() const noexcept = 0;
```

The const-reference return is the sanctioned "non-owning view of an object the callee keeps". Reading
a field is zero-copy; a caller wanting a mutable copy writes `PhysParams p = solver->getParams();`,
copy-constructed in the caller's module over a layout the header fixes and `static_assert`s guard. Do
not use the WIDL `T& get(T& retVal)` form here — it costs a 400-byte memcpy per call and buys nothing.
Keep WIDL for small derived aggregates like `DetailTimer`, where there is no stable storage to view.

Three changes to the struct itself, and they are the invasive part:

1. `solverMode` **leaves** → `SolverDesc::mode`, consumed by `createSolver`. The two-pluggable-solvers
   outcome made structural: no `if` on a param field anywhere, in C++ or HLSL.
2. The four XPBD fields **leave** → `XpbdComplianceParams`, reachable only via `IPhysSolverXPBD`.
3. `relaxationMode` gets a fixed underlying type (`enum class RelaxationMode : uint8_t`) with explicit
   padding; `planes[8][4]` moves last so one `offsetof` assert fronts the padded region.

`NvFlexSetSolverDescDefaults()` is deleted — a DLL exports no C++ function for this; default member
initialisers do it in the header.

*`NvFlexBuffer` → nvrhi.* `IPhysBuffer` survives as a revision-tracked wrapper carrying
`elementCount` + `elementStride` beside the `nvrhi::IBuffer*`, because the kernels index ~70 buffers by
stride. `NvFlexCopyDesc` survives verbatim as `CopyRegion`. Map/Unmap survives only on CPU-visible
buffers (`cpuAccess = Write`/`Read` via `nvrhi::IDevice::mapBuffer`). Deleted: `NvFlexBufferType`
(every buffer is a `BufferDesc`), `NvFlexMap` on a device buffer (it returned a handle masquerading as
memory), the OGL register/unregister pair (no OpenGL backend), and the whole
`GetDeviceAndContext`/`AcquireContext`/`RestoreContext`/`ComputeWaitForGraphics` family — there is one
`nvrhi::IDevice`, and cross-queue ordering is `queueWaitForCommandList`.

*Registration becomes adoption.* `NvFlexRegisterD3DBuffer` disappears. The application creates a
vertex buffer with `canHaveUAVs`, calls `adoptBuffer`, and `setArray(ParticleArray::Position, ...)`.
The `Finalize` kernel then writes final positions straight into the vertex buffer the renderer binds —
one state transition per frame, no copies, no registration step. This is the largest single
simplification nvrhi buys, and it exists only because simulation and rendering now share a device.
Keep the SPH port's documented fallback contract (`areRenderArraysShared()` /
`updateRenderArrays(commandList)`) for D3D11, which cannot create shareable vertex buffers through
nvrhi — but make the fallback a device-to-device `copyBuffer`, not SPH's host round trip. SPH went
through the host only because `gp` (CUDA) and nvrhi were two devices. Say that in the header so nobody
copies SPH's shape by reflex.

*Enforcement, in this step.* `nvrhi_abi_lint`
([`../../../donut/nvrhi/tests/abi_lint.py`](../../../donut/nvrhi/tests/abi_lint.py)) hard-codes its header list and
will not scan `ethereal-phys` as written — parameterise or fork it, point it at the public include dir,
register it as a ctest. Add the header-hygiene and compiles-alone equivalents, a check that no IID
begins with the placeholder sentinel, `c_HeaderVersion` + `etherealPhysGetHeaderVersion()` verified
inside `etherealPhysCreateDevice`, and a `dumpbin /exports` smoke test rejecting any C++-mangled name.
Without the header-version check a stale `PhysParams` layout is silent memory corruption rather than an
error.

*Regenerate every GUID with `uuidgen` before the first commit.* The design sketch uses placeholders;
the rules say generate fresh, never copy.

**Step 3** carries the sharpest edge in the migration and one large unlocked win.

*The edge.* FleX compiles at `cs_5_0`/DXBC, which is D3D11-only — D3D12 cannot consume DXBC at all,
and `ethereal_compile_shaders` gates DXBC on `DONUT_WITH_DX11`
([EtherealShaders.cmake:67](../../../cmake/EtherealShaders.cmake#L67)), then silently creates an empty
`add_custom_target` when no backend survives. A DX12/Vulkan configuration would produce no shader
artifacts with no build error and fail at runtime. So the kernels must move to **SM 6.0, DXIL +
SPIRV** (6.0, not the helper's 6_5 default — nothing here needs 6.1+ and 6.0 is the widest support).
The `.cfg` manifests port verbatim; only the CMake call changes.

*The win.* At SM 6.0, `WaveActiveSum`/`Min`/`Max` and `WavePrefixSum` are core HLSL on both vendors
and on Vulkan. FleX's ~24 vendor-duplicate kernels exist only to reach these via NVAPI
(`-DUSE_NV_SHADER_EXT=1 -DNV_SHADER_EXTN_SLOT=u7`) and AGS. nvrhi supports the NVAPI path
(`ShaderDesc::hlslExtensionsUAV`, `Feature::HlslExtensionUAV`) but has **no AGS support on any
backend**, and AGS intrinsics are DXBC-only regardless. So write **one** SM6 wave variant replacing
both vendor paths, gated on `Feature::WaveLaneCountMinMax`, keeping the portable groupshared path as
fallback. That collapses the manifest from 82 entries to roughly 58 plus a few wave variants, drops
the NVAPI dependency and the `u7` slot reservation, and is portable to Vulkan.

*Binding layouts — no nvrhi change needed.* `NvFlexDispatchParams` is 32 SRVs + 8 UAVs + 2 CBVs,
which maps to one fixed `nvrhi::BindingLayout` (`t0..t31`, `u0..u7`, `b0`, `b1`) serving all kernels — a
mechanical port. nvrhi's limits are already sufficient: `BindingLayoutDesc::bindings` is an unbounded
`vector<BindingLayoutItem>` ([nvrhi.h:2303](../../../donut/nvrhi/include/nvrhi/nvrhi.h#L2303)), and
`c_MaxBindingLayouts = 8` and `c_MaxVolatileConstantBuffersPerLayout = 6` are both ample.

FleX leaves most slots null, and the null-binding rule runs the opposite way from the obvious guess:
[validation-device.cpp:1880](../../../donut/nvrhi/src/validation/validation-device.cpp#L1880) fires only
when `getGraphicsAPI() != VULKAN`, so null buffer bindings are **permitted on Vulkan** and **rejected
on D3D11/D3D12** for structured, raw and constant views. Bind one shared 16-byte dummy structured
buffer and one dummy raw buffer to the unused slots on the DX backends; Vulkan takes nulls natively.
The alternative — 82 per-kernel layouts — is what FleX's per-dispatch slot table already is, and it is
more code for no gain.

*`KernelParams`.* 544 bytes / 34 registers on `feat-xpbd`, with hand-placed padding and `offsetof`
asserts. Moving the XPBD fields to the second constant buffer (`NvFlexDispatchParams` already has
`secondConstantBuffer`, bound only by the XPBD solver) returns the shared block to **512 bytes / 32
registers** — FleX's shipped layout. `kSolverMode` and its runtime branch disappear; `#if NVFLEX_XPBD`,
already the axis on `feat-xpbd`, stays the only switch. Make all padding explicit with a
`static_assert(offsetof(...))` per padded region; `alignas` and `#pragma pack` are banned.

*What does not port.* FleX's byte-exact acceptance test. It vendors `tools/fxc63/` with a pinned
`d3dcompiler_47.dll`, while ShaderTool's DXBC backend calls `D3DCompile` from whatever DLL the loader
finds. FleX's own `AGENTS.md` records 6.3 and 10.1 as byte-identical for every shader in the tree, so
functional equivalence through ShaderTool is safe — but the SHEX comparison keeps running in the FleX
reference checkout with its vendored compiler. `ethereal-phys` inherits the verified *sources*, not the
acceptance test. Record each shader's verified source commit in a comment so provenance stays
traceable.

*Step 3 as built.* Four things came out differently from the paragraphs above; the record is
`ethereal-phys/engine/shaders/PORTING.md`.

- **No single binding layout.** One register is a structured buffer in one kernel and a raw buffer, a
  typed buffer (`R32_UINT` or `RGBA32_UINT`: the radix sort reads one buffer through both) or a 3D texture
  (the sixteen SDF volumes of `CollideShapes`, `t15..t30`) in another, and an nvrhi layout fixes the type
  of each slot. Each kernel gets the layout its source declares, from a table generated from the manifest
  and the sources (62 layouts for 66 kernels). The caller still fills 32 + 8 + 2 slots, as with
  `NvFlexDispatchParams`, plus a parallel array for the texture slots; the layouts are not visible to it.
- **Dummies on every backend, and more than two.** Slots left null are bound to dummy resources on Vulkan
  as well (one code path). D3D11 compares the stride of a structured view with the shader and refuses one
  buffer in two UAV slots, so there is a dummy per view kind, per element size and per UAV slot.
- **The manifests did not port verbatim.** 88 FleX rows became 62: variants are permutation axes of one
  row (`EPHYS_WAVE`, `NVFLEX_XPBD`), because ShaderTool names a blob by source and entry point. 66 kernel
  ids, 77 shaders per format. The wave permutation assumes no lane count and no thread-to-wave mapping.
- **D3D11 kept its kernels.** The sources still compile at `cs_5_0`, so DXBC is built when the aggregate
  has the D3D11 backend, and the kernel tests run there too.

Steps 4 to 6 should know: the sort buffers are typed buffers (a `BufferUse` for them is still to be added
to `buffer.h`); a structured buffer must have exactly the element size the kernel declares; constant
blocks are passed as bytes and pooled by content, which requires that command lists are executed in
recording order; and the inflatable volume kernel sums a subset of the triangles when the last block of
512 is not a power of two (shipped FleX behaviour, reproduced by both permutations).

**Steps 4 to 7** deliver the data model and the solvers: 32 interfaces, namespace `ephys`. The
data/ops line, stated once: **anything that owns a buffer is data; anything that dispatches a kernel is
operations.** `IPhysDomain` owns buffers and dispatches nothing; `IPhysSolver` owns scratch nobody else
may see and dispatches everything.

| Group | Interfaces |
| --- | --- |
| Device | `IPhysDevice`, `IPhysKernelLibrary`, `IPhysBufferPool`, `IPhysBuffer`, `IPhysMessageCallback` |
| Data | `IPhysDomain`, `IParticleState`, `IDiffuseParticleState`, `ISpringSet`, `IShapeMatchSet`, `ITriangleSet`, `IInflatableSet`, `ICollisionShapeSet` |
| Geometry | `ITriangleMeshAsset`, `ISdfAsset`, `IConvexAsset`, `IGeometryLibrary` |
| Solvers | `IPhysSolver`, `IPhysSolverXPBD`, `IPhysSolverStageCallback`, `IPhysSolverFactory` |
| Readback | `IPhysReadback`, `IPhysTimers` |
| Extensions | `IPhysAsset`, `IPhysAssetBuilder`, `IPhysTearingOps`, `IPhysContainer`, `IPhysInstance`, `ISoftJointSet`, `IMovingFrame`, `IForceFieldSet` |

`IGeometryLibrary` is a real need, not bureaucracy: FleX's three id types are library-scoped indices
into a GPU-side geometry table the kernels read, so something must assign and pack them.

*PBD vs XPBD as two solvers.* Shared and not duplicated: the domain, the particle state, all four
constraint sets, the shape set, `PhysParams`. Critically the spring **stiffness array stays one
array** — PBD reads `k ∈ [-1,1]` as a per-iteration multiplier, XPBD maps it geometrically onto
`[stiffnessMin, stiffnessMax]`, and negative `k` is a tether under both. That is already true on
`feat-xpbd`; do not fork the array.

XPBD-only state lives in exactly three places, none of them `IPhysDomain`: selection at creation via
`SolverDesc::mode`; `XpbdComplianceParams` reachable only through `IPhysSolverXPBD`; and the
`mHalfSpringLambdas` / `mInflatableLambdas` multiplier buffers as private members of the XPBD
implementation class, exposed read-only for tools. A PBD solver cannot be handed compliance params,
because it does not answer `IID_IPhysSolverXPBD`. Callers discover which solver they hold by
`QueryInterface` — not an enum test and not `dynamic_cast`, which
[nvrhi ADR 0006](../../../donut/nvrhi/doc/adr/0006-no-rtti-queryinterface-casts.md) forbids:

```cpp
nvrhi::AutoPtr<IPhysSolverXPBD> xpbd;
if (NVRHI_SUCCEEDED(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd))))
    xpbd->setCompliance(compliance);
```

Both implementation classes derive from a shared non-interface mixin holding the common pipeline
scratch, which is the sanctioned "inherit helper classes directly" shape:

```cpp
class SolverPipeline { /* sorted arrays, grid, deltas, contacts, neighbours, radix sort */ };

NVRHI_CLASS_CLSID(PhysSolverXPBD, "<fresh guid>")
class PhysSolverXPBD : public SolverPipeline, public nvrhi::ObjectImpl<ephys::IPhysSolverXPBD> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysSolverXPBD)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolverXPBD)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolver)
    NVRHI_IMPLEMENTS_CLASS(PhysSolverXPBD)
    NVRHI_END_INTERFACE_TABLE()
};
```

Implementation classes stay file-local in anonymous namespaces in the `.cpp`, created with
`MAKE_RC_OBJ` / `MAKE_RC_OBJ_PTR`, reached through `FRESULT createXxx(..., IXxx** out)` factories —
the pattern in [gvdb/GVDB.h:408](../../../ethereal-samples/src/gvdb_samples/gvdb/GVDB.h#L408).

*Three API shapes the rules force into uglier forms.* Say so in the headers rather than quietly
relaxing: `NvFlexGetDetailTimers` returning library-owned, next-call-invalidated memory becomes a
count plus a WIDL per-index getter; the three `NvFlexExtMap*Data` by-value struct returns become WIDL
views whose host mirrors must be allocated through `nvrhi::allocator` so callers may read but never
free them; and `NvFlexExtAsset`'s 20 raw pointers freed by `NvFlexExtDestroyAsset` — freeing another
module's `malloc` — become `IPhysAsset` with `Release` replacing the destroy call.

*One real behavioural change.* `NvFlexSolverCallbackParams` hands out raw device pointers into
solver-internal sorted arrays and promises you can launch kernels on them. There is no such thing
under nvrhi. `SolverStageContext` carries `nvrhi::IBuffer*` views plus an open `nvrhi::ICommandList*`,
so a callback goes from "launch a CUDA kernel on these pointers" to "record a dispatch on this command
list". Any out-of-tree callback must be rewritten. `IForceFieldSet` implements
`IPhysSolverStageCallback` internally, which is how it replaces `NvFlexExtForceFieldCallback`.

*Do not pattern-match on gvdb/sph signatures.* `IGVDBVolume` violates the no-duplicate-name rule
twice, returns aggregates by value repeatedly, is `noexcept` nowhere, and passes `std::vector` across
its interface; `ISPHParticles` likewise returns `dm::box3` by value with no `noexcept`. Both got away
with it as in-EXE sample code. `ethereal-phys` is a DLL and gets no relaxation. Take exactly two
things from them — the **data/ops split** and the **contracts** (revision counters,
`areRenderBuffersShared`, `updateRenderBuffers`, "a solver is created for one particle set and uses
only its public interface") — and nothing from their method signatures. Put that sentence in
`domain.h`'s header comment so the next reader does not copy the wrong thing.

*Step 4 as built.* The data model is twelve interfaces in `particles.h`, `constraints.h`, `collision.h` and
`domain.h` of `ethereal-phys/engine/include/ethereal/phys/`, with `IPhysDevice::createDomain` and
`getGeometry`; `c_HeaderVersion` is 3. What came out differently from the paragraphs above, or was left
open by them:

- **The domain owns `PhysParams`.** `setParams` / `getParams` / `getParamsRevision` (the three methods of
  Step 2) are on `IPhysDomain`, not on the solver: a solver has no parameters of its own and reads its
  domain's. The XPBD compliance block stays with `IPhysSolverXPBD`. `SolverDesc` lost `maxParticles` and
  `maxDiffuseParticles`; they are `DomainDesc`'s, since the domain owns the particles.
- **No `donut::math` in the DLL.** The particle bounds are computed on the GPU, so
  `IParticleState::getBounds` is the two-element buffer a solver writes (FleX's `NvFlexGetBounds`), not a
  `dm::box3` that would cost a readback per step. Asset bounds are a `Bounds3` POD of six floats. `math.h`
  is not header-only as a whole (`matrix.cpp`, `vector.cpp`, `frustum.cpp` in `donut_core`), and with these
  two decisions the DLL needs none of it: its public surface mentions nvrhi only.
- **Three uploads take host memory, not `IPhysBuffer`s.** Particles, diffuse particles, springs, rigids,
  triangles and the mesh and convex assets transfer by GPU copies from Upload and to Readback buffers, as
  planned. The shape set, the inflatable set and the distance field volume take host pointers: the shape
  set must read flags and geometry to find the assets its shapes name (an Upload buffer is write-only
  memory; on D3D11 mapping it discards it), the inflatables are stored in the packed form the kernels read,
  which is computed (FleX maps its five buffers on the host for the same reason), and nvrhi fills a texture
  from host memory only.
- **The D3D11 fallback has a method of its own.** `setArray` is zero-copy and `FE_UNSUPPORTED` where
  `supportsRenderSharedBuffers` is false; `setRenderArray` names the application's vertex buffer and
  `updateRenderArrays` copies into it, device to device, on every backend.
- **Capacities.** The two particle capacities are fixed at creation, as in FleX. Springs, rigids, triangles,
  inflatables and shapes grow on upload with FleX's slack (n + n/2); `DomainDesc::reserve*` only allocates
  up front. A set without capacity returns null buffers.
- **Geometry.** Asset ids are FleX's (lowest free slot for meshes, last-freed-first for convexes and
  distance fields; 4096 convexes of 64 planes, 16 distance fields). A shape set holds a reference to every
  asset its shapes name. The triangle mesh hierarchy is built in `ITriangleMeshAsset::update`, where FleX
  builds it (`NvFlexUpdateTriangleMesh`), by `engine/src/bvh_builder.cpp`, a port of
  `LinearBVHBuilderGPU` on the kernel library. Flattening the meshes into the arrays the kernels bind is
  `IGeometryLibrary::commit`: buffer copies only, so it stays on the data side; a solver calls it at the
  start of a step. The bounds of the shapes and the hierarchy over them are per step and per solver, and
  are not in the data model.
- **`BufferUse::Typed`** exists now, and the radix sort driver creates its buffers through it.

Step 5 should know: every buffer accessor of a growing set may return null and may return another object
after an upload (compare `getLayoutRevision`); a solver that writes a data buffer (positions, velocities,
normals, the rigid transforms and rest positions, triangle normals, the diffuse arrays and count, the
particle bounds) says so with `markContentChanged`, and must leave its results in the domain's buffers,
not in a ping-pong twin; the half-springs are rebuilt when `ISpringSet::getContentRevision` moves;
`IInflatableSet::getInflatables` is already the kernels' `FlexInflatable`; the densities, the anisotropy
vectors and the smoothed positions are solver outputs and were left out of `ParticleArray` on purpose;
`BvhBuilder::build` with null bounds is the path for the shape hierarchy and is tested.

*Step 5 as built.* The PBD solver is `IPhysSolver` in `ethereal-phys/engine/include/ethereal/phys/solver.h`,
created by `IPhysDevice::createSolver(domain, desc, &solver)`; `c_HeaderVersion` is 4. What came out differently
from the paragraphs above, or was left open by them:

- **One operation, no stage methods.** `step(commandList, dt, substeps)` records the whole step; the granular
  stages of the sketch (`predict()`, `buildGrid()`, ...) are not public, because each reads the scratch the one
  before it wrote and none leaves the domain in a usable state. Acting inside a step is a stage callback
  (`IPhysSolverStageCallback`, `SolverStage` = `NvFlexSolverCallbackStage`, `SolverStageContext` with views of the
  sorted arrays, both index maps and the open command list). There is no debug switch: `NvFlexSetDebug` does
  nothing in FleX. `setTimersEnabled` brackets every stage with a command-list marker of FleX's timer name;
  the GPU times are step 7's.
- **Submission.** `step` takes an optional command list, as every recording method of Steps 2 to 4 does:
  the application's open list, or null for the device's own (executed by `flush()`). One list per step, all
  substeps on it, in recording order. Nothing is read back during a step: FleX's `Solver::Update` reads no
  device data either (its maps are writes of upload/constant buffers; the diffuse count, contact counts and
  neighbour counts stay on the GPU and are clamped in-kernel), so no synchronisation point was needed.
- **Derived data is rebuilt at step time.** The half-springs (FleX: `SetSprings`) when
  `ISpringSet::getContentRevision` moves; the shape bounds and the shape hierarchy (FleX:
  `ShapeData::RebuildShapeData`) when `ICollisionShapeSet::getContentRevision` moves; the inflatable pressures
  when the inflatable capacity grows; the fluid rest density (FleX: `SetParams`) when
  `IPhysDomain::getParamsRevision` moves. `IGeometryLibrary::commit` runs first. The domain's buffers are looked
  up again at the start of every step, which covers replaced and substituted buffers without comparing
  layout revisions.
- **Results stay in the domain.** Positions, velocities, normals, the particle bounds, the rigid transforms
  (and rest positions), the triangle normals and the diffuse arrays and count; each gets `markContentChanged`.
  The diffuse ping-pong of FleX (`swap` after `CompactDiffuse`) became a copy back, so the domain's buffers keep
  their identity. Solver outputs (smoothed positions, densities, anisotropy, contacts, neighbours, the two
  index maps) are `getOutput(SolverOutput)`; Step 7 wraps readback around them.
- **`SolverDesc::baselineReductions`** (new): dispatch the summing kernels in their baseline (groupshared,
  FleX-order) variant on wave-capable devices. The parity tests use it; the default is the wave variant.
- **Faithful quirks kept:** the contact solve always uses `SolveContactsSequential` (FleX loads the averaged and
  accumulate kernels and never dispatches them; the relaxation mode acts in `ApplyDeltas`); the inflatable
  partial-block reduction; `kNumParticlesAligned == kNumParticles`; FleX's process-wide random seeds
  (`sphCalculateRestDensity` resets them, the 256-entry random table of a solver depends on what ran before).
  `SolveShapes{32,128}` group-size tuning of the vendor path does not exist (PORTING.md).
- **Grid and sorts belong to the solver** (FleX keeps the cell buckets in the library and reuses the particle
  sort for the springs).

Parity, measured: against the shipped `NvFlexReleaseD3D_x64.dll` and against `NvFlexRev.dll` of the FleX
checkout, on an AMD Radeon 780M and an NVIDIA RTX 4050, with the engine on D3D12, Vulkan and D3D11, all nine
scenes agree after one step to at most 1.2e-7 in a position (bit-identical for most scenes on D3D11; the
residual on D3D11 is in the distance-field contact only, measured by leaving that shape out). The tests are
`ethereal_phys_parity_<api>`, registered when `ETHEREAL_PHYS_FLEX_REFERENCE_DIR` names a checkout.

Step 6 should know: XPBD is a second class over `SolverPipeline` (`engine/src/solver/solver_pipeline.h`), with
`NVRHI_IMPLEMENTS_INTERFACE(IPhysSolverXPBD)` and `IPhysSolver`. The seams are the virtual hooks
`beginConstraintIterations` (clear the multipliers per substep), `configureSolveSprings`,
`configureInflatableVolume`, `configureSolveShapes` (switch the kernel id, add the multiplier slots and the
compliance block at b1) and `onHalfSpringCapacity` / `onInflatableCapacity` (size the multiplier buffers);
`createSolver` routes `SolverMode::XPBD` there (today `FE_NOT_IMPLEMENT`). Step 7 should know: the outputs are
`getOutput`, the stage markers are `SolverPipeline::StageScope`, where timer queries attach.

*Step 6 as built.* The XPBD solver is `IPhysSolverXPBD : IPhysSolver` in
`ethereal-phys/engine/include/ethereal/phys/solver.h`, implemented by `PhysSolverXPBD` over the same
`SolverPipeline` as the PBD solver (`engine/src/solver/solver_xpbd.cpp`); `c_HeaderVersion` is 5. Commits in
`ethereal-phys`: `9980d4f`, `ab2f84f`. What came out differently from the paragraphs above, or was left open
by them:

- **The interface.** `setCompliance` stores the values unclamped and FleX's clamps apply when a step reads
  them, as FleX's `InitParams` does; `getCompliance` returns a view. `getSpringMultipliers` and
  `getInflatableMultipliers` are non-owning `nvrhi::IBuffer*` views, null while the matching set is empty and
  until the first step after it was filled; spring multipliers are per half-spring, in the solver's internal
  order (sorted by particle). The half owned by a pinned particle is never solved and stays 0.
  `resetMultipliers` takes a command list, null meaning the device's own, the convention of `step`; the
  sketch above had no argument.
- **Selection.** `createSolver` validates the description, then picks the class by `SolverDesc::mode`. That
  is the only mode test in the code. A PBD solver answers `FE_NOINTERFACE` to `IID_IPhysSolverXPBD`.
- **The seams.** All six are overridden. `beginConstraintIterations` derives `KernelParamsXPBD` from the
  compliance block with FleX's clamps and clears the whole capacity of each multiplier buffer, only when the
  step has springs or inflatables, as FleX's `Solver.cpp` does. `configureSolveSprings` switches to
  `SolveSpringsXPBD` (multipliers at u1, the sorted start-of-substep positions at t8, the compliance block at
  b1); `configureInflatableVolume` to `CalculateInflatableVolumeXPBD`; `configureSolveShapes` to the plain or
  plastic `SolveShapes…XPBD` kernel. `onHalfSpringCapacity` and `onInflatableCapacity` create zero-filled
  multiplier buffers. The compliance block is rebuilt every substep rather than once per step; the values are
  the same.
- **The reference's documented limitations are kept, not fixed:** density and contact constraints stay PBD,
  multiplier bookkeeping is approximate under `RelaxationMode::Local`, damping is coupled to compliance,
  shape-matching compliance is per particle-to-goal and mass-free, and inflatables lose the PBD `k³`
  re-inflation boost.
- **Applications call `runGarbageCollection()` every frame.** Without it the XPBD test exhausted the D3D12
  shader-visible descriptor heap, because nothing retired the binding sets of finished command lists. The
  engine never calls it; it is the application's job, and the test harness's `stepScene` does it.

Parity, measured: the reference is `feat-xpbd`'s `NvFlexRev` built from `git archive` outside the FleX
reference checkout, with ShaderTool at commit `4989128` (the one FleX's gitlink names) and the Windows SDK's
FXC. The opt-in cache variable `ETHEREAL_PHYS_FLEX_XPBD_REFERENCE_DLL` registers
`ethereal_phys_parity_xpbd_<api>`. After one step, the four reference scenes agree to at most 3.8e-6 in a
position on D3D12 (6.2e-6 on Vulkan, cloth). On D3D11, chain and beam are bit-identical to the reference for
120 steps. After 120 steps the centre of mass agrees within 9.4e-5 and the bounds within 2.6e-4 on every scene
and API.

Substep convergence, the decisive check: a 64×64 cloth with its whole top row pinned, at rest after 8 s,
drops 3.3637 / 3.3627 / 3.3622 m at 8 / 16 / 32 substeps under XPBD, a 0.046% spread, held to 0.1%. PBD on the
same scene gives 3.2071 / 3.1664 / 3.1539 m, a 1.674% spread, stiffening with every substep; that contrast is
the evidence that `alpha/dt²` is right. The two-corner hanging cloth settles into folds that vary with the
substep count and is held to 0.5% only.

*Step 7 as built.* Readback and timers are `IPhysReadback` and `IPhysTimers` in
`ethereal-phys/engine/include/ethereal/phys/readback.h`; `c_HeaderVersion` is 6. Commit in `ethereal-phys`:
`1fab6d4`. What came out differently from the paragraphs above, or was left open by them:

- **The factories are on the device.** `IPhysDevice::createReadback(solver, …)` and `createTimers(solver, …)`
  take the solver. Appending them to `IPhysSolver` would have shifted every vtable slot of `IPhysSolverXPBD`,
  which derives from it, and a readback object needs only the solver's public interface. Both refuse a null
  solver or one from another device; `createTimers` also refuses an `IPhysSolver` this library did not create
  (found by class-ID `QueryInterface`). Both keep the solver alive; several may exist per solver.
- **Readback records copies and nothing else.** Every method records GPU copies on the given command list
  (null meaning the device's own), never waits and never maps. Every getter takes an optional `CopyRegion` in
  particles, scaled by the per-particle contact count for contact planes and velocities and applied per
  column for neighbour lists. All copies of a call are checked before any is recorded; a wrong stride, an
  Upload destination, an out-of-range region or no destination gives `FE_INVALID_ARGS` and one message.
- **Ordering is FleX's, with no remap kernel.** API order: densities, anisotropy, smoothed positions, contact
  indices (FleX's reverse lookup, i.e. apiToInternal) and apiToInternal. Internal order: contact planes,
  velocities and counts, neighbour lists and counts, internalToApi. Before the first step, smoothed positions
  returns the plain particle positions, as in FleX.
- **Timers never block.** Each stage gets a timer query beside its marker and the step gets one more (the
  latency); with timers off nothing is recorded. A step is read only once all its queries have finished, and
  results change only in `poll()` or at the start of a timed step. A step that cannot get as many queries as
  the previous step needed is skipped rather than half measured; one that alone exceeds the budget is measured
  up to the point the queries ran out and flagged incomplete. `PhysTimers` holds FleX's 24 members, each the
  sum FleX's table assigns it; `total` is their sum, and the detail timers are the stage names that ran,
  sorted, then "Total" (the latency).
- **Support:** D3D11 on the graphics queue; D3D12 on graphics and compute; Vulkan on graphics and compute
  where every queue family of that kind reports timestamps; never on a copy queue. An unsupported queue records
  markers only, counts the step as skipped, and the step still succeeds.
- **Divergences from FleX:** regions on every getter; with no region a destination gets as much as it holds,
  up to what the source has (FleX copied fixed counts); `GetNeighbors` copies the whole neighbour lists (FleX
  copied maxParticles × maxContacts elements, which looks like a slip); too-small destinations are refused
  rather than overrun; `NvFlexGetDeviceLatency`'s clock stamps and frequency are not available, because nvrhi
  timer queries give durations only.

Verified on the fluid scene, where sorting moves 3454 of 3456 particles off their API index: FleX's documented
loop over the neighbour readback matches a brute-force search exactly (0 mismatches over about 113,000
neighbours); densities match a host SPH sum to a worst relative error of 4.4e-7; all 968 particles touching a
plane have matching contacts. Against FleX after one step, densities agree to 1.2e-6 on D3D12 and Vulkan and
are bit-identical on D3D11; contact and neighbour counts are bit-identical everywhere.

**Known issue, found here and older than this step:** the solver's particle stages do not run on a compute
queue. D3D12 removes the device in the first step of a scene; Vulkan reports
`VUID-vkCmdPipelineBarrier2-srcStageMask-09675` (nvrhi records fragment-shader stages in barriers on the
compute-only queue family). Nothing had run the solver on a compute queue before. The Vulkan half looks like
nvrhi's state translation and is outside this repository; the D3D12 cause is not known. Until it is fixed the
solver runs on the graphics queue, and the `_compute` readback tests cover only steps without active particles.

**Step 8** adds the physics scene service, with leaves on donut's graph, not a parallel graph. FleX
has no articulation hierarchy — no joints with a parent/child transform chain — so every physics thing
needing a transform is a *leaf*, and a graph whose nodes are all leaves is a list that donut already
maintains via `RegisterLeaf` plus a vector. Donut also already propagates
`DirtyFlags::SubgraphTransforms` and content flags upward, runs the transform pass in
`Refresh(frameIndex)`, and keeps the previous frame's transform. A parallel graph would reimplement all
three and then need reconciliation — a second source of truth for transforms. Both in-repo precedents
(`GVDBVolumeInstance`, `SPHFluidInstance`) took leaves.

Content flag `0x400` — donut holds `0x01..0x20`, gvdb took `0x100`, sph took `0x200`. One bit for all
physics leaf kinds; nothing dispatches on "cloth vs soft body", so a second bit would be dead weight.

In `ethereal_phys_scene` (STATIC, where `std::` is fine): `PhysDomainInstance`, `PhysInstanceLeaf`,
`PhysShapeLeaf`, `PhysForceFieldLeaf` as `SceneGraphLeaf` subclasses with
`NVRHI_INHERIT_INTERFACE_TABLE()`, overriding `Clone` / `GetContentFlags` / `GetLocalBoundingBox`;
`PhysSceneGraph` overriding `RegisterLeaf`/`UnregisterLeaf` (base call first, `dynamic_cast`,
`ResourceTracker<T>`, `SceneResourceCallback<T>`) structurally identical to `GVDBSceneGraph`; and
`PhysSceneService` as the physics scene service — three passes per frame beside `SceneGraph::Refresh`:
`SyncToPhysics()`, `Step(dt, substeps)`, `SyncFromPhysics()`.

Transform authority is pinned per leaf kind, because this is where a feedback loop is easy to build:

| Leaf | Direction | Contract |
| --- | --- | --- |
| `PhysShapeLeaf` | donut → physics, every frame | Node's current and previous world transforms become the shape set's pos/rot and prevPos/prevRot. The best fit in the design: `NvFlexSetShapes` wants exactly both for continuous collision, and donut already keeps both — animated collision geometry comes free from donut's animation system |
| `PhysForceFieldLeaf` | donut → physics, every frame | Node translation becomes the field position |
| `PhysDomainInstance` | physics-space → scene-space, static | FleX simulates in one world space (`gravity` and `planes` are world-space; the solver holds no per-domain transform). The node transform is a domain-to-scene placement, and the service **asserts rigid plus uniform scale** — non-uniform scale would make the particle radius anisotropic and the physics silently wrong |
| `PhysInstanceLeaf` | donut → physics once, then physics is authoritative | The spawn transform. It goes stale once the solver runs; do not write particle positions back into the node, as cloth has no rigid transform to write |

**Steps 9 and 10** port the extensions and the samples. The extension layer must be ported
deliberately, including what the demo never calls. The whole container/instancing/soft-joint subsystem
— `flexExtContainer.cpp`, the single largest extension file at 30.7KB — has **zero references from
`demo/`**, along with 34 other public symbols. Two pairs are especially easy to lose:
`NvFlexExtCreateClothFromMesh` (unused, yet the only path exposing
`tetherStiffness`/`tetherGive`/`pressure`) and `NvFlexExtCreateWeldedMeshIndices` (unused, but the
documented required pre-pass for it — dropping one silently breaks the other).

From `core/`, port what has no modern equivalent: `cloth.h` (half-edge topology and tearing — real
simulation logic, not a utility), `voxelize.*`, `sdf.*`, `convex.h`, `extrude.*`, `aabbtree.*`,
`pfm.*` (the SDF asset format that all of `data/` depends on), `perlin.*`, the OBJ+PLY readers in
`mesh.*`, and the geometric-query and sampling subsets of `maths.h` (`ClosestPointOnTriangle`,
`IntersectRay*`, `PoissonSample3D`, `UniformSample*`, `Hermite*`, `SolveQuadratic`). Drop what donut,
nvrhi, std or stb already provide: all vector/matrix/quat/colour math, `png.*`, `tga.*`, `matnn.h`,
`mat22.h`, `free_ptr`, and most of `platform.h`.

`demo/helpers.h` (51.3KB, 39 scene-authoring functions) holds most of the practical FleX usage
knowledge behind no public header. It becomes a real library in the samples tree, not a header.
`demo/main.cpp` at 117.6KB — app loop, scene registry, UI, camera, picking, benchmark and screenshot
in one file — gets decomposed; `demo/demoContext.h` is the seam, with everything below it
(`d3d/`, `d3d11/`, `d3d12/`, ~85 files) deleted in favour of donut.

Samples follow the `gvdb_add_sample()` model: an `ethereal_phys_add_sample()` helper plus glob-based
discovery so new samples need no CMake edit. The initial subset spans one scene per feature family —
fluid, granular, cloth, tearing, soft body, rigid, inflatable, SDF collision, force field — plus the
four XPBD reference scenes (`xpbdcloth`, `xpbdballoon`, `xpbdbeam`, `xpbdchain`), which have published
quantitative targets.

## Verification

### ABI, no device

ctest runs the ABI lint over the public headers, the header-hygiene check, a compiles-alone test per
public header (achievable only because the DLL headers do not include donut's scene graph), the
placeholder-GUID check, and `dumpbin /exports ethereal_phys.dll | findstr /v etherealPhys` — any
`?`-mangled name is a failure. A header-version mismatch must fail `etherealPhysCreateDevice`, tested
explicitly.

### Shaders

Configure the aggregate for DX12-only and Vulkan-only and assert
`build/bin/shaders/ethereal_phys/{dxil,spirv}` is non-empty — this is the regression that catches the
silent-empty-target failure mode. Dispatch one kernel through the single binding layout on **D3D12**
with `--debug` before step 5 begins, since D3D12 is the backend that rejects null structured and raw
bindings and therefore the one that exercises the dummy-buffer path.

### Solver correctness

Behavioural parity against the reference FleX build, taken from the FleX reference checkout whose
location the CMake cache variable supplies, run on the same scene with the same seed and substep
count: compare particle position buffers after N steps within tolerance, and compare screenshots. The
XPBD scenes have published numbers from the `feat-xpbd` validation — pin separation 366.2px, checker
slope 0.21522/m, dip/D 0.277, H/D 1.175, W/D 0.903 — so assert against those directly. The substep
convergence check is the one that proves `alpha/dt²` is right: 8, 16 and 32 substeps must agree within
0.1% on a 64-wide sheet, where PBD would stiffen with every substep.

### Samples

Every GUI sample takes `--screenshot <png> [--frames N]`, as GVDBApp does, so each runs unattended in
CI and the PNG is read back to judge it. Run the set on D3D12 and Vulkan in Debug and Release; note
that nvrhi in this fork is stricter than upstream (volatile CBs need `cpuAccess None`, no null
binding-layout slots, binding sets must cover their layout, push constants after every `set*State`,
buffers need tracked state). Use `--debug` to surface validation errors.

### Build

```powershell
.\build.ps1 -CMakeArgs '-DETHEREAL_BUILD_PHYS=ON'
```

Run it from the root of the second clone. When `build.ps1` hides errors — PowerShell 5.1 wraps native
stderr — rerun via
`cmd /c "VsDevCmd.bat -arch=x64 && cmake --build build --config Release --target X > log 2>&1"` and
read the log as UTF-8.

### Commit order

Commit order is forced by the submodule nesting, and the submodule must be committed before its parent
can record the new SHA:

```
ethereal-phys  ->  aggregate (bumps ethereal-phys)
```

Messages follow the existing history: imperative sentence case, with aggregate commits as
`bump ethereal-phys (<what>)`.

## Open risk

The license position needs resolving before anything here is published. `LICENSE.txt` in the FleX
tree is the NVIDIA Source Code License granting redistribution under the same terms with notices
retained unmodified, while nearly every `.h`/`.cpp` in `demo/`, `include/` and `core/` carries a banner
asserting the opposite — "NVIDIA Confidential Information… distribution of this software… without an
express license agreement from NVIDIA Corporation is strictly prohibited". The license forbids
modifying those notices, so they travel with any redistribution while simultaneously denying it.
Dropping `src/dxbc/`, `src/_pseudo/` and `bin/` removes the reverse-engineering artifacts, which is the
sharpest part of the exposure, but the recovered HLSL in `src/shaders/` was derived from them and the
provenance chain remains. Keeping `ethereal-phys` private avoids the question; publishing it does not.
This is a question for counsel reading the actual text, not the license's title.
