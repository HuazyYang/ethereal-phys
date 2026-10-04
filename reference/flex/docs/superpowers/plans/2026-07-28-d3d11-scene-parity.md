# D3D11 Scene Parity Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make all 67 FleX demo scenes match the public D3D11 backend over playback frames `[0,200)` when run through the reconstructed backend.

**Architecture:** Repair the reconstructed solver from the outside inward. First correct C++ dispatch and state-management differences proven against the recovered public pseudocode, then localize remaining numerical divergence to shader families and recover exact shader semantics from authoritative DXBC assembly. Every correction is gated by a failing playback reproduction, affected-scene verification, and passing controls.

**Tech Stack:** C++17, HLSL Shader Model 5, CMake/Ninja Multi-Config, clang-cl cross-compilation, Wine, DXVK/D3D11, IDA 9.3 pseudocode, DXBC assembly.

## Global Constraints

- Acceptance is D3D11 only; D3D12 parity is outside scope.
- All 67 scenes must match every particle component within absolute tolerance `1e-3` for frames `[0,200)`.
- Preserve all existing user modifications in the dirty worktree.
- Do not modify public vendor DLLs or generated binaries under `bin/` and `lib/`.
- Keep generated output under `/tmp/nvflex-wine-clang` and `/tmp/nvflex-all-scenes-parity`.
- Treat `src/_pseudo/NvFlexDebugD3D_x64.cpp` as authoritative for C++ orchestration.
- Treat matching `src/dxbc/*.asm` files as authoritative for recovered shader behavior.
- Make one demonstrated production correction per red-green cycle.
- Do not loosen comparison tolerance, suppress mismatch output, or add scene-specific compensation.
- Do not create commits because the shared worktree contains unrelated user changes and `.git` is read-only in this environment.

---

### Task 1: Correct spring dispatch coverage

**Files:**
- Modify: `src/Solver.cpp:1430-1450`
- Test: `test/TestAllScenesPlaybackParity.sh`
- Reference: `src/_pseudo/NvFlexDebugD3D_x64.cpp:43710-43760`

**Interfaces:**
- Consumes: `Solver::mMaxParticles`, `Solver::mNumParticles`, and the existing `SolveSprings` compute dispatch.
- Produces: a spring dispatch covering the same maximum-particle domain as the public D3D11 solver.

- [ ] **Step 1: Verify the red playback baseline**

Run the existing reconstructed backend against fresh public oracles for scene 1
through frame 200 and scene 3 through frame 200:

```bash
DXVK_LOG_LEVEL=none NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
  ./scripts/wine/run-demo.sh --config Debug --rhi d3d11 --dev 1 \
  --smoke-seconds 4 -- --scene=1 --playback-mode=read
DXVK_LOG_LEVEL=none NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
  ./scripts/wine/run-demo.sh --config Debug --rhi d3d11 --dev 1 \
  --smoke-seconds 4 -- --scene=3 --playback-mode=read
```

Expected: Soft Octopus reports its first mismatch at frame 38 and Soft Rope
reports its first mismatch at frame 169.

- [ ] **Step 2: Confirm the public dispatch operand**

Read the complete recovered `SolveSprings` function and verify its block count
uses `8 * s->mMaxParticles`:

```bash
sed -n '43680,43780p' src/_pseudo/NvFlexDebugD3D_x64.cpp
```

Expected: `kNumParticleBlocks = (8 * s->mMaxParticles + 511) / 512`.

- [ ] **Step 3: Implement the minimal correction**

Change only the dispatch-domain operand:

```cpp
const NvFlexUint kNumParticleBlocks = divCeil<512>(8 * mMaxParticles);
```

- [ ] **Step 4: Build the reconstructed runtime**

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexRev
```

Expected: the target links successfully.

- [ ] **Step 5: Verify affected scenes and controls**

Replay scenes 1 and 3, followed by passing soft-body controls 2 and 4:

```bash
for scene in 1 3 2 4; do
  DXVK_LOG_LEVEL=none NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
    ./scripts/wine/run-demo.sh --config Debug --rhi d3d11 --dev 1 \
    --smoke-seconds 4 -- --scene="$scene" --playback-mode=read
done
```

Expected: each run reaches `[Playback] completed frame range [0,200)`; record
whether scenes 1 and 3 become green and require scenes 2 and 4 to remain green.

---

### Task 2: Correct rigid shader capability selection

**Files:**
- Modify: `src/Solver.cpp:1480-1530`
- Test: `test/TestAllScenesPlaybackParity.sh`
- Reference: `src/_pseudo/NvFlexDebugD3D_x64.cpp:43890-44020`

**Interfaces:**
- Consumes: `Library::mIsSHFLSupported`, `Library::mIsFP32ATOMICSupported`, and rigid workload size.
- Produces: public-compatible selection among `SolveShapes`, `SolveShapes32`, and `SolveShapes128`.

- [ ] **Step 1: Verify the red playback baseline**

Replay rigid-sensitive failing scenes 31, 54, and 55 and passing controls 35 and
56 from their existing complete oracles:

```bash
for scene in 31 54 55 35 56; do
  DXVK_LOG_LEVEL=none NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
    ./scripts/wine/run-demo.sh --config Debug --rhi d3d11 --dev 1 \
    --smoke-seconds 4 -- --scene="$scene" --playback-mode=read
done
```

Expected: scenes 31, 54, and 55 report mismatches; scenes 35 and 56 complete
without mismatches.

- [ ] **Step 2: Confirm the public branch predicate**

```bash
sed -n '43890,44020p' src/_pseudo/NvFlexDebugD3D_x64.cpp
```

Expected: both plastic and non-plastic optimized branches require
`mIsSHFLSupported && mIsFP32ATOMICSupported`.

- [ ] **Step 3: Implement the minimal correction**

Replace the duplicated atomic test in the non-plastic branch:

```cpp
} else if (mLib->mIsSHFLSupported && mLib->mIsFP32ATOMICSupported) {
```

- [ ] **Step 4: Build and verify**

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexRev
```

Replay scenes `31 54 55 35 56` using the command from Step 1. Expected: every
run completes; record the changed failure set and require controls 35 and 56 to
remain green.

---

### Task 3: Complete the C++ orchestration audit

**Files:**
- Modify: `src/Solver.cpp`
- Test: `test/TestAllScenesPlaybackParity.sh`
- Reference: `src/_pseudo/NvFlexDebugD3D_x64.cpp:43120-44680`

**Interfaces:**
- Consumes: every dispatch and persistent state transition called by `Solver::UpdateSubstep`.
- Produces: C++ update ordering, bindings, dispatch dimensions, shader gates, and buffer state matching the public solver.

- [ ] **Step 1: Generate an execution-order audit ledger**

Compare each function from `Predict` through `LazyClearGrid` in source and
pseudocode. Record only demonstrated differences in
`/tmp/nvflex-solver-orchestration-audit.tsv` with columns:

```text
stage	source_line	reference_line	category	source_value	reference_value
```

The categories are restricted to `order`, `condition`, `gridDim`,
`readOnly`, `readWrite`, `constantBuffer`, `shader`, and `state`.

- [ ] **Step 2: Verify each ledger entry with an affected red scene**

For each entry, select the lowest-index failing scene exercising that stage and
replay only through its first failing frame by generating a fresh public oracle
with `--playback-range=0,end`, where `end` is one greater than that first
failing frame.

Expected: the selected scene reproduces a mismatch before any source change.

- [ ] **Step 3: Correct one ledger entry**

Apply only the source/reference difference shown by the current ledger row.
For the already confirmed diffuse state error, the exact public-compatible
transition is:

```cpp
swap(mDiffuseVelocities, mDiffuseVelocitiesNew);
```

Do not change the diffuse swap unless a particle-position test exercising
diffuse state demonstrates an externally visible failure; primary particle
parity remains the acceptance boundary.

- [ ] **Step 4: Build and run the current red-green pair**

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexRev
```

Replay the affected scene and a passing scene exercising the same stage.
Expected: the affected mismatch is eliminated or moves strictly later without
regressing the control. If it does neither, revert that single correction and
return the ledger entry to investigation.

- [ ] **Step 5: Repeat until the ledger is exhausted**

Repeat Steps 2-4 for every demonstrated orchestration difference. Run the full
67-scene sweep afterward:

```bash
NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
NVFLEX_PLAYBACK_FRAME_END=200 \
NVFLEX_PLAYBACK_SMOKE_SECONDS=4 \
NVFLEX_PLAYBACK_RETRY_SECONDS=12 \
./test/TestAllScenesPlaybackParity.sh
```

Expected: a fresh results table classifying the failures remaining after C++
orchestration parity.

---

### Task 4: Repair shape-contact shader semantics

**Files:**
- Modify when assembly proves a difference: `src/shaders/CollideShapes.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveContactsSequential.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/TransformShapeBounds.hlsl_rev`
- Reference: `src/dxbc/g_Flex_CollideShapes.asm`
- Reference: `src/dxbc/g_Flex_SolveContactsSequential.asm`
- Reference: `src/dxbc/g_Flex_TransformShapeBounds.asm`
- Test: `test/TestAllScenesPlaybackParity.sh`

**Interfaces:**
- Consumes: shape BVH data, shape transforms, particle positions/phases, and contact buffers.
- Produces: shape contact planes, velocities, counts, and corrected particle positions matching public D3D11.

- [ ] **Step 1: Establish targeted red tests**

Generate fresh `[0,80)` public oracles and replay scenes 14, 15, 16, and 17.
Expected: each scene reproduces its baseline mismatch by frame 70 or earlier.
Replay passing controls 13, 18, and 19 and require them to remain green.

- [ ] **Step 2: Audit shader declarations**

For each source/assembly pair, compare resource slots, resource kinds, cbuffer
offsets, `numthreads`, UAV masks, and atomic operations. Write demonstrated
differences to `/tmp/nvflex-shape-shader-audit.tsv`.

- [ ] **Step 3: Audit instruction semantics**

Walk every assembly instruction affecting contact distance, normal, shape
velocity, and contact count. Compare branch predicates, swizzles, saturates,
reciprocal/square-root operations, and arithmetic order with recovered HLSL.
Add each demonstrated semantic difference to the audit table.

- [ ] **Step 4: Correct one demonstrated shader difference**

Apply a single audit row to the matching HLSL source. Preserve all binding
registers and entry-point names.

- [ ] **Step 5: Compile and verify**

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexShaders NvFlexRev
```

Replay scenes `14 15 16 17 13 18 19`. Expected: the affected failure is
eliminated or moves later and all controls remain green. Repeat Steps 4-5 until
the shape audit is exhausted.

---

### Task 5: Repair density and velocity shader semantics

**Files:**
- Modify when assembly proves a difference: `src/shaders/CalculateDensity.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/CalculateDensitySurfaceTension.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveDensities.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveDensitiesSurfaceTension.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveVelocities.hlsl_rev`
- Reference: matching `src/dxbc/g_Flex_*.asm` files
- Test: `test/TestAllScenesPlaybackParity.sh`

**Interfaces:**
- Consumes: sorted particle neighborhoods, phases, positions, densities, normals, contacts, and fluid constants.
- Produces: density deltas, surface-tension corrections, viscosity/vorticity velocity corrections, and final fluid positions.

- [ ] **Step 1: Establish targeted red tests and controls**

Generate fresh `[0,20)` public oracles for failing scenes 20, 22, 36, 37, 38,
41, 43, 44, 45, 46, 47, 48, 49, 50, 52, 63, and 66. Expected: every scene
whose baseline first mismatch is before frame 20 reproduces. Use scenes 39, 40,
62, 64, and 65 as passing controls.

- [ ] **Step 2: Audit resources and constants**

Compare every declared resource and every used `cb0` offset in each implicated
assembly file to `KernelParams.hlsli` and the recovered HLSL. Record
demonstrated differences in `/tmp/nvflex-fluid-shader-audit.tsv`.

- [ ] **Step 3: Audit arithmetic and control flow**

Trace all instructions contributing to density, lambda, cohesion, surface
tension, viscosity, buoyancy, and final velocity. Preserve assembly arithmetic
order, comparison polarity, swizzles, write masks, and precise reciprocal or
square-root behavior.

- [ ] **Step 4: Correct and test one semantic difference**

Apply one audit row, then build:

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexShaders NvFlexRev
```

Replay the affected scene plus at least one control using its fresh oracle.
Expected: the mismatch is eliminated or moves later; controls remain green.
Repeat until the fluid audit is exhausted.

---

### Task 6: Repair deformable and rigid shader semantics

**Files:**
- Modify when assembly proves a difference: `src/shaders/SolveSprings.hlsl`
- Modify when assembly proves a difference: `src/shaders/SolveInflatableVolume.hlsl`
- Modify when assembly proves a difference: `src/shaders/SolveShapes.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveShapes32NV.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/SolveShapes128NV.hlsl_rev`
- Modify when assembly proves a difference: `src/shaders/UpdateTriangles.hlsl`
- Reference: matching `src/dxbc/g_Flex_*.asm` files
- Test: `test/TestAllScenesPlaybackParity.sh`

**Interfaces:**
- Consumes: spring adjacency, rigid clusters, inflatable triangles, particle mappings, and constraint deltas.
- Produces: public-compatible soft-body, cloth, inflatable, and rigid corrections.

- [ ] **Step 1: Establish targeted red tests and controls**

Use remaining failures after Tasks 1-5 from these families: scenes 1, 3,
23-28, 31-34, 42, 54, 55, 60, and 61. Generate an oracle ending one frame
after each current first mismatch. Use passing scenes 2, 4-11, 21, 29, 30, 35,
56-59 as controls selected by the same constraint family.

- [ ] **Step 2: Audit shader variants**

Compare the selected generic/NV variant against its exact DXBC assembly,
including `numthreads`, capability-dependent atomics, group reductions, lane
shuffles, rigid workload thresholds, triangle atomics, and delta accumulation.
Record demonstrated differences in `/tmp/nvflex-deformable-shader-audit.tsv`.

- [ ] **Step 3: Correct and test one semantic difference**

Apply one audit row and build:

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexShaders NvFlexRev
```

Replay its affected scene and paired control. Expected: the affected mismatch
is eliminated or moves later while the control remains green. Repeat until no
targeted family failure remains.

---

### Task 7: Full D3D11 regression closure

**Files:**
- Verify: `src/Solver.cpp`
- Verify: affected files under `src/shaders/`
- Test: `test/TestAllScenesPlaybackParity.sh`
- Output: `/tmp/nvflex-all-scenes-parity/results.tsv`

**Interfaces:**
- Consumes: all corrections from Tasks 1-6.
- Produces: final evidence that all registered D3D11 scenes match the public backend.

- [ ] **Step 1: Build all affected Debug targets**

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug \
  --target NvFlexShaders NvFlexRev DemoAppD3D
```

Expected: successful shader compilation, runtime link, and demo link.

- [ ] **Step 2: Run source checks**

```bash
bash -n test/TestAllScenesPlaybackParity.sh
git diff --check -- src/Solver.cpp src/shaders test/TestAllScenesPlaybackParity.sh
```

Expected: the script parses successfully and changed lines introduce no
whitespace errors.

- [ ] **Step 3: Run the complete fresh sweep**

```bash
NVFLEX_BUILD_ROOT=/tmp/nvflex-wine-clang \
NVFLEX_PLAYBACK_FRAME_END=200 \
NVFLEX_PLAYBACK_SMOKE_SECONDS=4 \
NVFLEX_PLAYBACK_RETRY_SECONDS=12 \
NVFLEX_PLAYBACK_LOG_ROOT=/tmp/nvflex-all-scenes-parity-final \
./test/TestAllScenesPlaybackParity.sh
```

Expected summary:

```text
PASS 67
```

The result table must contain 67 unique scene indices, zero `FAIL` rows, zero
invalid oracles, and zero incomplete reads or writes.

- [ ] **Step 4: Repeat representative scenes**

Regenerate and replay scenes 0, 1, 16, 27, 31, 41, 50, 60, 63, and 66.
Expected: all ten complete `[0,200)` with zero mismatches on the independent
repeat.

