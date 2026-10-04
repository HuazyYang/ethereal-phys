# D3D11 Scene Parity Repair Design

## Goal

Make the reconstructed D3D11 backend selected by `--dev=1` reproduce the public
D3D11 backend selected by `--dev=0` for all 67 registered demo scenes.

Acceptance requires every particle position component to stay within the
existing absolute tolerance of `1e-3` for every frame in the physical playback
range `[0,200)`. Every vendor oracle and reconstructed replay must complete the
entire range. D3D12 parity is outside this repair's acceptance scope.

## Current Baseline

The complete D3D11 playback sweep currently reports 29 passing scenes and 38
failing scenes. Failures span several feature families:

- springs and soft bodies;
- shape collision and moving shapes;
- cloth and inflatables;
- rigid constraints;
- density, viscosity, surface tension, and other fluid behavior.

The first mismatch ranges from frame 0 to frame 197 depending on the scene.
This distribution indicates multiple shared solver-stage differences rather
than a single scene initialization defect.

Two C++ orchestration differences are already confirmed against
`src/_pseudo/NvFlexDebugD3D_x64.cpp`:

- `Solver::SolveSprings()` calculates its dispatch size from `mNumParticles`,
  while the public implementation uses `mMaxParticles`.
- The non-plastic branch of `Solver::SolveShapes()` tests FP32 atomic support
  twice, while the public implementation requires shuffle support and FP32
  atomic support before choosing optimized rigid shaders.

These findings are evidence for the staged audit; they are not permission to
make unverified adjacent changes.

## Repair Strategy

### Stage 1: C++ solver orchestration

Audit `src/Solver.cpp` against the recovered public implementation in execution
order. Compare:

- stage ordering and conditional execution;
- dispatch dimensions;
- root and secondary constant buffers;
- read-only and read-write resource slots;
- shader-selection capability gates;
- buffer swaps and persistent state updates.

Change one confirmed discrepancy at a time. Each correction must be tested
against the smallest scene that exposes it and at least one passing control
scene before proceeding.

### Stage 2: Shader semantics

Classify failures remaining after the orchestration corrections by the first
solver stage capable of producing the observed difference. For each implicated
shader, treat the matching `src/dxbc/*.asm` file as authoritative and compare:

- resource declarations and slots;
- constant-buffer offsets;
- thread-group dimensions;
- branch predicates;
- swizzles and write masks;
- arithmetic operation and evaluation order;
- atomics and vendor-extension behavior.

Modify recovered HLSL only when an assembly-level difference is demonstrated.
Compile the affected shader target before running playback tests.

### Stage 3: Regression closure

After each feature family becomes green, retain its targeted scene in the
regression set. When targeted failures are exhausted, rebuild the Debug D3D11
demo and reconstructed runtime and rerun the complete 67-scene sweep over
`[0,200)`.

## Verification

The vendor backend writes one oracle per scene:

```text
--scene=N --dev=0 --playback-mode=write --playback-range=0,200
```

The reconstructed backend reads and compares it:

```text
--scene=N --dev=1 --playback-mode=read
```

`test/TestAllScenesPlaybackParity.sh` performs the complete sweep. It validates
each oracle's range, particle counts, frame payloads, and absence of trailing
data before replay. A scene passes only if playback prints its completion
marker and no particle-number or particle-buffer mismatch is reported.

Targeted tests use the same oracle mechanism with the shortest frame range
that still reproduces the relevant first mismatch. Passing controls are
selected from the same feature family where possible.

## Safety and Scope

- Preserve user modifications already present in the dirty worktree.
- Do not modify the public vendor DLL or generated binaries under `bin/` and
  `lib/`.
- Keep generated build products outside the source tree.
- Do not loosen the playback tolerance or mask mismatches in the demo.
- Do not add scene-specific numerical compensations.
- Do not claim a feature family fixed until its targeted failures and controls
  pass from fresh executions.
- Do not claim completion until all 67 D3D11 scenes pass the full range.
