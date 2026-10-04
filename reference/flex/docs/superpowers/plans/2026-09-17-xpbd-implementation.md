> **Status (2026-09-17):** approved, not started. Archived for later scheduling.
> Implementation must begin on a new branch `feat-xpbd` cut from `dev` (see Step 0).
> Source paper: Macklin, Müller, Chentanez, "XPBD: Position-Based Simulation of Compliant Constrained Dynamics", MIG 2016, https://matthias-research.github.io/pages/publications/XPBD.pdf

# XPBD for NvFlexRev (D3D11/D3D12)

## Context

FleX applies a per-iteration stiffness multiplier `k ∈ [0,1]` to every constraint
correction, so effective stiffness depends on `numIterations`, `dt` and the substep
count (the manual even says "substepping will also make constraints appear
stiffer"). Macklin, Müller & Chentanez, *XPBD: Position-Based Simulation of
Compliant Constrained Dynamics* (MIG 2016) fixes this with a per-constraint
compliance `α` (inverse stiffness) and an accumulated Lagrange multiplier `λ`:

```
α̃ = α / Δt²                                                     (paper §4)
Δλ_j = (−C_j − α̃_j λ_j) / (∇C_j M⁻¹ ∇C_jᵀ + α̃_j)                   (Eq. 18)
Δx   = M⁻¹ ∇C_jᵀ Δλ_j                                            (Eq. 17)
λ_j ← λ_j + Δλ_j ; x ← x + Δx ; λ ← 0 at the start of every step (Alg. 1)
Rayleigh damping β:  β̃ = Δt² β,  γ_j = α̃_j β̃_j / Δt  (= α β / Δt)
Δλ_j = (−C_j − α̃_j λ_j − γ_j ∇C_j (x_i − x_n)) / ((1+γ_j) ∇C_j M⁻¹ ∇C_jᵀ + α̃_j)   (Eq. 26)
```
With `α = 0` this is exactly PBD with `k = 1`. Contacts stay at zero compliance and
need no λ (paper §6). The paper's 3D results use a GPU Jacobi solver, which is
FleX's layout.

Goal: an opt-in XPBD solver mode covering **springs (incl. tethers and cloth
stretch/bend/shear), inflatable volume, and rigid shape matching**, with the
paper's damping term for springs, while leaving the default PBD path bit-for-bit
unchanged. Decisions already taken with the user:

- Scope: springs + inflatables + shape matching. Density/contacts stay PBD.
- Compliance is **derived from the existing stiffness `k`** via a global scale
  (no new per-constraint buffers in the public API).
- Include the Eq. 26 damping term as a global spring damping parameter.
- No public λ read-back API; λ stays on the GPU.

## Non-negotiable repository constraint

Every `src/shaders/*.hlsl` is hand-recovered to **byte-identical DXBC** against the
shipped blobs and `tools/dxbccmp.sh <Entry>` is the acceptance test
([src/shaders/AGENTS.md](../../../src/shaders/AGENTS.md)). XPBD is therefore added as
**new shader entries in a separate manifest** and must not change any existing
entry's SHEX:

- New behaviour lives in a new file (`SolveSpringsXPBD.hlsl`) or in
  `#if NVFLEX_XPBD` blocks compiled as a second entry
  (`-DNVFLEX_XPBD=1 -D<Entry>=<Entry>XPBD`). The non-XPBD compile is textually
  identical after preprocessing, so its bytecode cannot move.
- FXC declares `cb0[N]` only up to the highest register a shader reads
  (`g_Flex_SolveSprings.asm` declares `CB0[26]`, `Predict` `CB0[25]`, against a
  32-register struct). `KernelParams` fills registers 0..31 exactly, so appending
  four scalars puts them in register 32.
  **Implementation note (measured):** this is *not* bytecode-neutral for the two
  shaders that index `cb0` dynamically (`UpdateDiffuseParticles`,
  `CollideShapes` walk `kPlanes[]` with a runtime index and therefore declare the
  full `cb0[33]`). The HLSL append is consequently wrapped in `#if NVFLEX_XPBD`
  in `KernelParams.hlsli`, so the non-XPBD preprocessed text is identical to
  before; the C++ struct is 528 bytes in every mode and non-XPBD shaders simply
  declare fewer registers than are bound. Verified with `tools/bytecmp.py`
  (unchanged before/after) and `tools/dxbccmp.sh` on the affected entries.
- `tools/bytecmp.py` and `tools/dxbccmp.sh` iterate `Shaders.cfg` and treat an
  entry without a shipped blob as a failure. The XPBD entries therefore go in a
  **new manifest `src/shaders/ShadersXPBD.cfg`** built by a second
  `shadertool_add_shader_objects(TARGET NvFlexShadersXPBD ...)` call in
  [src/CMakeLists.txt](../../../src/CMakeLists.txt) (same `OUTPUT_DIRECTORY`, add its
  `OBJECT_HEADER_OUTPUT_DIR` to `NvFlexRev`'s include path and an
  `add_dependencies`). The existing tools stay untouched and green.
- The runtime loads the shipped blobs from embedded `src/dxbc/*.txt`
  ([Library.cpp:168-191](../../../src/Library.cpp#L168-L191)); the compiled-header path is
  under `#if 0` and stays that way. XPBD shaders are created from the generated
  `<Entry>.hlsl.h` headers (symbol `g_<Entry>`, FXC default with no `-Vn`; header
  dir already on the include path) through a second `createShader` lambda of the
  shape at lines 169-177.

## Step 0: isolate the work on a branch

Before touching any file, branch off the current `dev` head so the DXBC-recovery
thread on `dev` stays untouched:

```
git switch -c feat-xpbd dev
```

`dev` currently has one uncommitted change (`src/private/Random.h`); it is
unrelated to XPBD and will simply carry over into the new working tree. Leave it
as is and do not include it in any XPBD commit. All XPBD work happens on
`feat-xpbd`; nothing is committed unless explicitly asked for.

## Public API ([include/NvFlex.h](../../../include/NvFlex.h))

Add an enum next to `NvFlexRelaxationMode` (line 91) and append four fields to
the **end** of `NvFlexParams` (after `relaxationFactor`, line 159):

```c
enum NvFlexSolverMode { eNvFlexSolverPBD = 0, eNvFlexSolverXPBD = 1 };

NvFlexSolverMode solverMode;   //!< PBD (default) or XPBD (compliant constraints, Macklin et al. 2016): stiffness independent of dt, substeps and iterations
float compliance;              //!< XPBD: global compliance unit alpha0 (m/N). Per-constraint alpha = alpha0 * (1-|k|)/|k| from the stiffness k; k = 1 is a hard constraint, k <= 0 disables the constraint. Default 0
float springDamping;           //!< XPBD: Rayleigh constraint damping beta for springs (paper Eq. 26); only acts on compliant springs. Default 0
float volumeCompliance;        //!< XPBD: compliance alpha of inflatable volume constraints (inflatables carry no k). Default 0
```

Mapping rationale: `k=1 → α=0` (== PBD `k=1`, the paper's limit), `k→0 → α→∞`;
`k ≤ 0` (after taking the tether sign off) skips the constraint, as PBD's zero
delta does. The tether sign keeps its unilateral meaning. Document that the
mapping is a convenience, not a physical calibration. Defaults go in the
`Solver::Init` defaults block ([Solver.cpp:935-939](../../../src/Solver.cpp#L935-L939)).

## Kernel parameters

Append to **both** `KernelParams` structs
([src/private/Solver.h:18-106](../../../src/private/Solver.h#L18-L106),
[KernelParams.hlsli:4-92](../../../src/shaders/KernelParams.hlsli#L4-L92)) after
`kMaxContactsPerParticle`, and add `static_assert(sizeof(KernelParams) == 528)`:

```
int   kSolverMode;
float kCompliance;
float kSpringDamping;
float kVolumeCompliance;
```
Filled in `Solver::InitParams` ([Solver.cpp:976-1063](../../../src/Solver.cpp#L976-L1063)).
`kDt`/`kInvDt` already hold the substep dt, so `α̃ = α·kInvDt²` and
`γ = α·β·kInvDt` are computed in-shader.

## GPU state: two λ buffers ([src/private/Solver.h](../../../src/private/Solver.h))

| buffer | size | allocated in | zeroed |
|---|---|---|---|
| `HStructuredBuffer<float> mHalfSpringLambdas` | `mMaxHalfSprings` | `Solver::SetSprings`, inside the `2*numSprings > mMaxHalfSprings` block ([Solver.cpp:174-197](../../../src/Solver.cpp#L174-L197)) | once per substep |
| `HStructuredBuffer<float> mInflatableLambdas` | `mMaxInflatables` | `Solver::SetInflatables`, inside the `n > mMaxInflatables` block ([Solver.cpp:435-443](../../../src/Solver.cpp#L435-L443)) | once per substep |

Add both to the constructor init list (`Solver.cpp` ~661ff). Use `CreateWithZero`;
zero with `mLib->ClearBufferInt(buf, bytes, 0)` (pattern at
[Solver.cpp:1074](../../../src/Solver.cpp#L1074); 0 bits == 0.0f; no-ops on a null buffer).
Clearing happens in `UpdateSubstep` after `ContinuousShockPropagation` and before
the iteration loop ([Solver.cpp:1077-1079](../../../src/Solver.cpp#L1077-L1079)), only when
`solverMode == XPBD`, so λ persists across the iterations of one substep and
resets per substep (Alg. 1 line 4). Half-spring λ is stored per endpoint on
purpose: both halves compute the bit-identical `Δλ` from symmetric inputs
(negation and `a−b = −(b−a)` are exact), so no sync or spring-index map is
needed. Do not reuse the dead `mHalfSpringIndices` buffer (it is bound as SRV t0
in the same dispatch; D3D11 forbids SRV+UAV of one resource).

Shape matching gets **no λ buffer**: its goal position is re-fitted every
iteration, so the constraint function changes between iterations and an
accumulated multiplier has no meaning. It is solved statelessly (below).

## Shaders

### 1. Springs: new `src/shaders/SolveSpringsXPBD.hlsl`

Clone of [SolveSprings.hlsl](../../../src/shaders/SolveSprings.hlsl) (same 512-thread,
8-lanes-per-particle gather, same `t1..t7`/`u0` bindings) plus:

```
RWStructuredBuffer<float> halfSpringLambdas : register(u1);
StructuredBuffer<float4>  sortedPositions   : register(t8);   // x_n, start of substep
```
Per half-spring (particle 0 is the owner, `d = pos0 − pos1`, `len = |d|`):

```
k = halfSpringStiffness[j];  tether = k < 0;  k = min(abs(k), 1)
if (k <= 0 || wsum == 0 || len == 0) continue;               // no delta, no λ update
alpha  = kCompliance * (1 - k) / k                           // k = 1 -> 0
alphaT = alpha * kInvDt * kInvDt
gamma  = alphaT * kSpringDamping * kDt                       // = alpha*beta/dt
C      = len - rest
if (tether) { if (C <= 0 && lambda <= 0) continue;  C = max(C, 0); }   // slack tether: no elastic and no damping term
n      = d / len
wsum   = pos0.w + pos1.w
dv     = (pos0.xyz - x0_n) - (pos1.xyz - x1_n)               // (x_i - x_n) projected below
dlam   = (-C - alphaT * lambda - gamma * dot(n, dv)) / ((1 + gamma) * wsum + alphaT)
lambda += dlam;  halfSpringLambdas[j] = lambda
Dx    += dlam * n                                            // owner's gradient is +n
```
Lane 0 then adds `Dx * pos0.w` and the count exactly as the PBD kernel does, so
`ApplyDeltas` and the count-based local relaxation keep working. With
`kCompliance == 0` and `beta == 0` this reduces to `Dx += (rest − len)/wsum · n`,
i.e. PBD with `k = 1`.

`ShadersXPBD.cfg`: `SolveSpringsXPBD.hlsl -T cs -E SolveSpringsXPBD` (no NV
variant: the spring kernel uses no vendor extension; the shipped NV entry only
differs by name).

### 2. Inflatables: guarded variant of `CalculateInflatableVolume.hlsl`

Add `#if NVFLEX_XPBD RWStructuredBuffer<float> volumeLambdas : register(u1); #endif`
and replace the thread-0 tail ([lines 265-274](../../../src/shaders/CalculateInflatableVolume.hlsl#L265-L274)) under the guard:

```
C      = volumeOfInflatable - mRestVolume                // mRestVolume already includes overPressure (Solver.cpp:462)
D      = 1.0 / mConstraintScale                          // = Σ|∇C_i|² at rest pose, host-precomputed (core/cloth.h:287-312)
alphaT = kVolumeCompliance * kInvDt * kInvDt
dlam   = (-C - alphaT * volumeLambdas[blockIdx]) / (D + alphaT)
volumeLambdas[blockIdx] += dlam
lambdas[blockIdx] = -dlam        // SolveInflatableVolume applies Dx = -lambda * n_tri, i.e. Δx = n * dlam
```
The PBD path's `k³` re-inflation boost is dropped under the guard (it is a
nonlinearity outside the constraint model); note in the plan verification that
collapsed balloons re-inflate more slowly in XPBD mode and the Inflatables demo
pressure may need retuning. `D` omits mass weighting, matching the existing
mass-free application in `SolveInflatableVolume`, which is reused unchanged.
`ShadersXPBD.cfg` entries: `CalculateInflatableVolumeXPBD`, `...XPBDNV`,
`...XPBDAMD` mirroring [Shaders.cfg:30-32](../../../src/shaders/Shaders.cfg#L30-L32) with
`-DNVFLEX_XPBD=1` added (the NV entry keeps `-DNV_SHADER_EXTN_SLOT=u7`; u1 does
not clash).

### 3. Shape matching: guarded variants of `SolveShapes.hlsl` and `SolveShapesPlasticDeformation.hlsl`

Only the two generic kernels get an XPBD entry (`SolveShapesXPBD`,
`SolveShapesPlasticDeformationXPBD`, no NV slot needed since `Utils.hlsli`'s CAS
`InterlockedAddFp32` is used); in XPBD mode the dispatcher always uses them
instead of the NV/32/128 specialisations. Under `#if NVFLEX_XPBD` also
`#include "KernelParams.hlsli"` (these files include only `Utils.hlsli` today;
`rootConstantBuffer` is already bound at [Solver.cpp:1503](../../../src/Solver.cpp#L1503)).

Each rigid particle is treated as a distance-to-goal constraint with the goal
fixed for the iteration, stateless, and with the **unit-mass convention the PBD
shape kernel already uses** (it never reads `w`), so `α = 0` reproduces PBD
exactly, including for pinned particles:

```
k = min(coefficient, 1);  if (k <= 0) skip
alphaT = kCompliance * (1 - k) / k * kInvDt²
C      = length(difference);  if (C > 0) { n = difference / C;
dlam   = -C / (1 + alphaT);   delta = dlam * n }                // == -difference / (1 + alphaT)
```
Base kernel: replace the body of `AccumulateDelta(sortedIndex, difference, coefficient)`
([SolveShapes.hlsl:124-133](../../../src/shaders/SolveShapes.hlsl#L124-L133)) under the
guard. Plastic kernel: it uses `coefficient` for creep too
([SolveShapesPlasticDeformation.hlsl:259,276](../../../src/shaders/SolveShapesPlasticDeformation.hlsl#L259)),
which stays untouched; only the delta passed to `AccumulateDelta(addr, delta)`
at [line 300](../../../src/shaders/SolveShapesPlasticDeformation.hlsl#L300) changes. Its
`AccumulateDelta` signature differs from the base kernel's, so the guarded block
is written per file. This is an accepted approximation (compliance per
particle-to-goal attachment, not on the body's energy).

### 4. Build plumbing

- `src/shaders/ShadersXPBD.cfg`: six lines (springs ×1, inflatable ×3, shapes
  ×2), flag column at 52, entry name == generated symbol. Add a short note to
  `src/shaders/AGENTS.md` that this manifest holds entries with no shipped blob
  and is outside the DXBC-equivalence sweep.
- [src/CMakeLists.txt](../../../src/CMakeLists.txt): second `shadertool_add_shader_objects`
  call for `NvFlexShadersXPBD`, include dir + dependency on `NvFlexRev`.
- [src/Library.cpp](../../../src/Library.cpp): `#include` the six generated headers; add
  `createShaderFromHeader(const BYTE*, uint64_t, label, slot)`; six
  `mShader*XPBD` members declared in [src/private/Library.h](../../../src/private/Library.h)
  (~lines 100-108), initialised in the ctor (~640-652) and released (~715-726).
  The guarded sources must compile under both the CMake-discovered FXC and
  `tools/fxc63`.

## Host dispatch ([src/Solver.cpp](../../../src/Solver.cpp))

- `UpdateSubstep` ([:1065](../../../src/Solver.cpp#L1065)): before the iteration loop, if
  `mParams.solverMode == eNvFlexSolverXPBD`, clear `mHalfSpringLambdas` (if
  `mNumSprings`) and `mInflatableLambdas` (if `mNumInflatables`).
- `SolveSprings` ([:1431](../../../src/Solver.cpp#L1431)): in XPBD mode use
  `mShaderSolveSpringsXPBD`, add `readWrite[1] = mHalfSpringLambdas`,
  `readOnly[8] = mSortedPositions` (readOnly maps to t0..t31, readWrite to u0..u7
  on both backends).
- `CalculateAndSolveInflatables` ([:1452](../../../src/Solver.cpp#L1452)): in XPBD mode
  use the XPBD volume shader (vendor variant chosen as today) with
  `readWrite[1] = mInflatableLambdas`; second dispatch unchanged.
- `SolveShapes` ([:1481](../../../src/Solver.cpp#L1481)): in XPBD mode short-circuit the
  variant selection to the two XPBD kernels (plastic vs plain).
- `InitParams`: fill the four new kernel params. `Solver::Init` defaults:
  `solverMode = PBD`, others 0.

Guarantee: when `solverMode == PBD` no new resource is bound and no new shader is
dispatched, so the default path is unchanged instruction-for-instruction.

## Demo ([demo/main.cpp](../../../demo/main.cpp), [demo/scenes/](../../../demo/scenes/))

- `g_params` is a global that `Init()` fills field by field
  ([main.cpp:990-1044](../../../demo/main.cpp#L990-L1044)); without defaults, XPBD settings
  would leak from one scene into the next. Set the four fields next to
  `relaxationFactor` ([:1027-1028](../../../demo/main.cpp#L1027-L1028)).
- UI: after the `SOR` slider ([:2208](../../../demo/main.cpp#L2208)) add a "Solver
  (0 PBD / 1 XPBD)" slider using the float-temp pattern of
  [:2184-2186](../../../demo/main.cpp#L2184-L2186), plus sliders for `compliance`,
  `springDamping`, `volumeCompliance`; show the mode in the stats overlay next
  to "Num Iterations" ([:2070](../../../demo/main.cpp#L2070)).
- Two validation scenes: header in `demo/scenes/`, `#include` in
  [demo/scenes.h](../../../demo/scenes.h), `g_scenes.push_back` in the cloth block
  ([main.cpp:3522-3529](../../../demo/main.cpp#L3522-L3529)):
  - `XPBD Hanging Cloth` (paper Fig. 6): clone [flag.h](../../../demo/scenes/flag.h)
    (two pinned corners, tethers), drop `Update()` wind and set `drag = 0`;
    stretch `k = 0.5`, `solverMode = XPBD`, `compliance = 1e-3`, 2 substeps.
    Cycling the iteration slider 1→20 must not visibly change the sag; the
    same scene in PBD mode stiffens.
  - `XPBD Oscillator` (paper §6.1): one fixed particle (`w = 0`) and one
    unit-mass particle joined by `CreateSpring` ([helpers.h:146](../../../demo/helpers.h#L146)),
    rest length 1, start at 1.5, `compliance = 0.001`, `k = 0.5` (so
    `α = 0.001`), gravity off, particles pushed as in
    [darts.h:19-25](../../../demo/scenes/darts.h#L19-L25). Period must be
    `2π√(m·α) ≈ 0.199 s` independent of iteration count; PBD mode is not.

## Known limitations to state in the header comment

- `ApplyDeltas` divides the summed Jacobi delta by the per-particle count (local
  relaxation) while λ accumulates the full `Δλ`, so the multiplier bookkeeping
  is approximate. **Measured (see Results):** iteration independence holds only
  once the Jacobi solver has converged, about 20 iterations per substep for
  cloth; the "substeps with one iteration" regime is exact for an isolated
  constraint but leaves cloth badly under-relaxed. Do not recommend it.
- Damping is coupled to compliance (`γ = αβ/Δt`), so `k = 1` springs receive no
  damping. Inherent to Eq. 26.
- Shape-matching compliance is per particle-to-goal attachment and mass-free
  (PBD convention), not a constraint on the body's energy.
- Inflatables lose the `k³` re-inflation boost in XPBD mode.

## Verification

1. **Bytecode regression (mandatory, first):**
   `python tools/bytecmp.py <outdir> tools/fxc63/fxc.exe` over `Shaders.cfg` —
   the 79 `Exact` and 5 `Verified` rows must not move (the appended
   `KernelParams` fields and the `#if NVFLEX_XPBD` guards must be
   bytecode-neutral). Spot-check `bash tools/dxbccmp.sh` for `SolveSprings`,
   `SolveShapes`, `SolveShapesPlasticDeformation`,
   `CalculateInflatableVolume`, `CalculateInflatableVolumeNV`,
   `CalculateInflatableVolumeAMD`, `ApplyDeltas`, `Predict`.
2. **Build:** `cmake --build build --config Debug --target NvFlexShaders NvFlexShadersXPBD`,
   then the full Debug build (`NvFlexRev`, `NvFlexExt`, `DemoAppD3D`); confirm
   the six `g_*XPBD` symbols resolve.
3. **Default-path parity:** record an oracle with the shipped DLL and replay it
   with the reconstructed solver in PBD mode
   (`--dev=0 --playback-mode=write --playback-range=0,200`, then
   `--dev=1 --playback-mode=read`; the Linux wrapper
   [test/TestSolverPlaybackParity.sh](../../../test/TestSolverPlaybackParity.sh) documents
   the flags, run the demo exe directly on Windows). The log must not contain
   `particle buffer mis-coincident`. Scenes: Pot Pourri, Flag Cloth,
   Inflatables, Rigid8, Plastic Bunnies.
4. **XPBD behaviour:** run `XPBD Oscillator` and `XPBD Hanging Cloth` with the
   iteration slider at 1, 5, 20 and substeps at 1, 2, 4; confirm iteration and
   substep independence, and that PBD mode on the same scenes is dependent.
   Run Flag Cloth, Inflatables, Soft Bunny, Rigid8, Plastic Bunnies in XPBD mode
   with `compliance = 0` and confirm they behave like PBD with `k = 1` (no NaNs,
   no explosions; `-benchmark` timing within a few percent of PBD). Then raise
   `compliance` and `springDamping` and confirm softening and damping respond
   monotonically.

## Amendment (2026-09-17): stiffness mapping and reference scenes

Two follow-ups requested after the first round:

- **Stiffness-to-compliance mapping** now follows Macklin's post *XPBD slides
  and stiffness* (blog.mmacklin.com, 2016-10-12): compliance is the reciprocal
  of an engineering stiffness, and an artist-facing `[0,1]` value should be
  mapped onto a stiffness range and inverted. `NvFlexParams::compliance` is
  replaced by `stiffnessMin` / `stiffnessMax` (N/m); a spring or rigid
  coefficient `k` maps geometrically, `stiffness = min·(max/min)^k`,
  `α = 1/stiffness`. Geometric rather than linear because the post's material
  table spans 1e3 (fat) to 2.5e10 (concrete) N/m². Kernel params carry
  `kInvStiffnessMin` and `kLogStiffnessRange` so the shaders evaluate
  `α = kInvStiffnessMin · exp2(−k · kLogStiffnessRange)`; the struct is padded
  to 544 bytes (34 registers). `k ≤ 0` still disables a constraint and the
  tether sign is unchanged. Defaults `[1e3, 1e9]`.
- **Reference scenes** matching the paper's supplementary video: `XPBD Hanging
  Cloth` is now the video's 64×64 cloth (24k constraints) hung from its two top
  corners under a bar, meant to be compared at 20/40/80/160 iterations; `XPBD
  Inflatable Balloon` is the video's row of four balloons whose surface
  stiffness rises left to right over a hard volume constraint, compared at 20
  vs 80 iterations. The oscillator pins its range to `[1000, 1000]` N/m so its
  compliance stays exactly 0.001 m/N.

**Second amendment (2026-09-17): the demo carries the video's four scenes.**
`XPBD Hanging Cloth` and `XPBD Inflatable Balloon` stay as measured below;
`XPBD Oscillator` is removed (its period check is kept in the Results section
as history and can be re-run by pinning any spring scene's range to
`[1000, 1000]`); two scenes are added: `XPBD Cantilever Beam`
([demo/scenes/xpbdbeam.h](../../../demo/scenes/xpbdbeam.h)), a 40×8 lattice of
stretch, shear and bending springs clamped to a wall, standing in for the
paper's St Venant-Kirchhoff triangular FEM beam which FleX has no constraint
type for, and `XPBD Chain` ([demo/scenes/xpbdchain.h](../../../demo/scenes/xpbdchain.h)),
the paper's 20-particle chain with α = 1e-8 and 50 iterations, released
horizontally from a fixed support.

**Amendment results** (same solver and GPU as below):

- Bytecode: `bytecmp.py` summary unchanged (1 / 70 / 11 / 0); `dxbccmp.sh`
  passes on SolveSprings, SolveShapes, SolveShapesPlasticDeformation,
  CalculateInflatableVolume, UpdateDiffuseParticles, ApplyDeltas. Three XPBD
  entries declare `cb0[33]` and the three inflatable ones `cb0[34]` (FXC sizes
  the buffer to the highest register read; the host uploads 544 bytes).
- Oscillator with the new mapping: 0.1998 s at 1 and 20 iterations (identical),
  0.1990 s at 8 substeps.
- Hanging cloth (64×64, two top corners, stretch/shear 0.44 → ~4.4e5 N/m,
  8 substeps), sag of the bottom row at rest: XPBD 0.365 / 0.367 / 0.367 /
  0.367 m at 20 / 40 / 80 / 160 iterations (40 upward identical to four
  decimals); PBD 0.111 at 20, 0.019 at 160. Two earlier configurations were
  rejected by measurement: 1e6 N/m at 2 substeps (near-rigid regime, XPBD as
  iteration-dependent as PBD up to 160 iterations) and 1e5 N/m at 4 substeps
  (converged, but the two-corner catenary tension stretched the cloth by more
  than its own height onto the floor). The rule that fell out: FleX's local
  Jacobi relaxation converges in ~20 iterations only when `k·dt²/m ≈ 2`, and at
  fixed resolution the stretch then scales with `dt²`, so a smaller substep is
  the lever that buys a modest droop.
- Inflatable balloons (four, coefficients 0.3 / 0.45 / 0.6 / 0.8 on
  [10, 1e6] N/m, hard volume, 4 substeps): centre heights 0.052 / 0.134 /
  0.229 / 0.362 at both 20 and 80 iterations (identical to three decimals),
  enclosed volume 1.000× rest throughout; PBD at 20 iterations keeps all four
  near-spherical (0.390 to 0.430). No non-finite positions in any capture.
- Cantilever beam (40×8 lattice, coefficient 0.54 → ~1.7e6 N/m, 16 substeps),
  tip deflection at rest: XPBD 0.350 / 0.349 / 0.349 / 0.349 m at 20 / 40 /
  80 / 160 iterations; PBD 0.111 at 20, 0.012 at 160. At 8 substeps and
  4.4e5 N/m the same lattice hung almost vertically (1.13 m, still
  iteration-independent), which is the `dt²` scaling again.
- Chain (20 particles, α = 1e-8, 50 iterations, 2 substeps): largest link
  error 0.8% mid-swing and 0.5% at rest, lowest point 1.3 cm below the taut
  length; at 10 iterations 5.2% / 3.0% and 8 cm.
- **Fidelity pass (2026-09-17, later):** the cloth now carries both rows of
  the video. XPBD preset: coefficients 0.54 / 0.54 / 0.4 at 16 substeps, sag
  0.124 m at 20 iterations and 0.121 m at 160 (wind off). PBD preset,
  applied by the scene when the Solver slider is flipped (springs re-uploaded,
  substeps switched): coefficient 1.0 at 2 substeps, sag 0.486 / 0.268 /
  0.089 m at 20 / 40 / 160 iterations, i.e. low at 20 and stiffer than XPBD
  at 160 as in the video. The video's own PBD value of 0.01 (and 0.06, 0.2,
  0.5) left FleX's count-averaged Jacobi cloth on the floor even at 160
  iterations. Balloon coefficients moved to 0.35 / 0.5 / 0.65 / 0.85: heights
  0.43 / 0.52 / 0.70 / 0.87 at both 20 and 80 iterations. A gusting wind (the
  Flag Cloth pattern, scaled by the Wind slider) was added to the cloth, and
  the demo gained `--wind=<strength>` so rest measurements can be scripted.
- **Video-value pass (2026-09-17, last):** at the user's request the PBD preset
  uses the video's own 0.01. Under FleX's local relaxation, and equally under
  plain Jacobi (global relaxation, factor 1), 0.01 left the 1.26 m cloth on
  the floor at every count, because one frame of gravity (2.7 mm) is 14% of a
  2 cm spring. The fix was scale, not stiffness: grid spacing 0.2 m (a 12.6 m
  cloth) makes that drop 1.4% of a spring. Final presets: PBD 0.01, one
  substep, global relaxation; XPBD 0.33 / 0.33 / 0.25 at 4 substeps, local
  relaxation. Measured sag of the bottom row (rest height 6.3 m, wind off):
  XPBD 1.96 m at 20 and at 160 iterations; PBD on the floor at 20 and 40,
  3.05 m at 160. PBD-160 was still softer than XPBD here where the video's is
  slightly stiffer.
- **Closing the gap (2026-09-17, last):** scaling further does not help, both
  sags being absolute lengths set by one frame of gravity. The PBD preset's
  substep count is the knob: with the video's 0.01 at 1 / 2 / 4 substeps the
  160-iteration sag is 3.05 / 1.46 / 0.51 m, and only at 1 and 2 does the
  20-iteration cloth still reach the floor. The preset now uses 2 substeps:
  PBD floor / 2.64 / 3.14 / 1.46 m at 20 / 40 / 80 / 160 iterations (40 and 80
  still swinging at frame 600), XPBD 1.96 m throughout, which is the video's
  ordering. The scene also frames the whole cloth face-on through
  `CenterCamera()` (the demo's default camera caps at 6 m height), and an
  explicit `--substeps` now wins over the preset so the sweep can be scripted.
- Build note: MSVC in this locale prints its include notes in Chinese and
  ninja's dependency scanner did not always recognise them, so a header-only
  edit to a scene can leave the demo unrebuilt; delete
  `build/msvc-ninja/demo/CMakeFiles/DemoAppD3D.dir/Debug/main.cpp.obj` (or
  build with `--clean-first`) after editing scene headers.

## Results (2026-09-17, branch `feat-xpbd`, uncommitted)

Everything above is implemented. Measurements, all on the D3D11 reconstructed
solver (`--dev=1`) running on the AMD Radeon 780M adapter the demo selects:

- **Bytecode regression:** `tools/bytecmp.py` reports the same 82-entry
  summary before and after (70 code-chunk identical, 1 full-file identical,
  11 differing, 0 failed; the 11 pre-date this work). `tools/dxbccmp.sh`
  passes on every entry that shares a source with an XPBD variant plus
  `SolveSprings`, `ApplyDeltas`, `Predict`, `SolveVelocities`,
  `UpdateDiffuseParticles`. The unguarded `KernelParams` append was measured
  to move `UpdateDiffuseParticles` (`cb0[33]`, dynamically indexed), hence the
  `#if NVFLEX_XPBD` guard in `KernelParams.hlsli`.
- **Default path unchanged:** a pre-change `NvFlexRev.dll` built from `dev`
  with identical flags recorded 200-frame oracles; the XPBD build in PBD mode
  replays Rigid8, Tearing and Plastic Bunnies frame-for-frame (these three are
  deterministic run to run). Flag Cloth, Sphere Cloth, Soft Bunny, Soft
  Octopus and Inflatables are not deterministic on this GPU even against
  themselves (float atomics; onset frames 55 to 92), and the XPBD build
  diverges at the same onsets (Inflatables 56/60/59 vs self 57/58/60).
- **Oscillator (paper §6.1, α = 0.001, m = 1):** analytic period 0.1987 s.
  XPBD, 4 substeps: 0.1998 s at 5 and at 20 iterations (identical
  trajectories); 1/2/8 substeps give 0.2158/0.2032/0.1990 s, converging with
  dt as expected from the implicit discretisation. PBD on the same scene:
  rigid at 5 and 20 iterations, ~0.034 s ringing at 1.
- **Hanging cloth (paper Fig. 6, curtain, α = 1e-5, 4 substeps), sag of the
  bottom row at rest:** XPBD 0.078 / 0.058 / 0.057 / 0.054 at 5 / 20 / 40 / 80
  iterations; PBD 0.075 / 0.019 / 0.009 / 0.005. XPBD plateaus at the compliant
  sag, PBD keeps stiffening. Below ~20 iterations both modes are unconverged
  under FleX's local Jacobi relaxation (XPBD at 1 iteration: 0.24 at 4
  substeps, 0.10 at 8), so the "substeps with one iteration" regime suggested
  earlier in this plan does **not** hold for cloth here; the scene defaults to
  20 iterations and the API comment was corrected accordingly.
- **Robustness:** Flag Cloth, Inflatables, Rigid8, Plastic Bunnies and Soft
  Bunny run 200 frames in XPBD mode with zero compliance without non-finite
  positions; Rigid8's extents are identical to the shipped-DLL oracle.
- **Demo test hooks added:** `--iterations=N`, `--substeps=N`, `--solver=0|1`
  override a scene's settings after `Initialize()`, so playback captures of
  one scene can be taken at several settings without rebuilding.
- **Build-tree note:** the `build/msvc-ninja` cache had been recreated with
  empty compiler flags (a configure attempted without the MSVC environment);
  it was reset by dropping the `CMAKE_*_FLAGS*` entries and reconfiguring.
  The five oracles Flag Cloth, Inflatables, Rigid8, Plastic Bunnies and Soft
  Bunny in `build/msvc-ninja` were overwritten during testing and regenerated
  with the shipped DLL at 200 frames.

## Amendment (2026-09-18): matching the hanging cloth to the video

The cloth scene was reported as "too stiff, wind too breezy". Both were real,
and neither had the cause I first assumed. Measured against
`data/xpbd_supplementary.mp4` with two scratch tools: `vmeasure.py` segments the
teal cloth in the video's four panels, `simcloth.py` computes the same
quantities from an `.fpb` capture, and `render.py` draws a capture as a shaded
mesh through the scene's own camera so the sim can be eyeballed next to the
video. Everything is normalised by the pin separation D, so the numbers do not
depend on the render.

Video targets ("Our Method", 20-iteration panel, t = 37..43 s, settled window):
dip/D 0.276 (range 0.25..0.30), H/D about 1.23, W/D about 0.93. The video is
60 fps, matching the demo's `g_dt`.

**The dip is top-row stretch, not pin slack.** An earlier round pinned the
corners closer together than the rest width to force a drape. A sweep showed
the whole sheet's weight hangs off the 64 springs of the top row and that the
dip is set almost entirely by the stretch coefficient; the pinch contributed
almost nothing (0.86 -> 0.92 moved the dip 0.449 -> 0.420) while it *floored*
both other metrics, because for a flat sheet H/D >= dip/D + 1/kPinFraction and
W/D ~= 1/kPinFraction. It was removed; the corners hang at full rest width.

**Shear stiffness makes the folds, and it was the real "too stiff".** With
shear equal to stretch the cloth rendered as a flat board (W/D 0.99, no folds
anywhere), which is what the report was describing. Cloth is stiff in stretch
but soft in shear; dropping shear to 0.15 against stretch 0.36 produced the
video's folds radiating from the pinned corners and brought W/D to 0.944. Soft
shear also carries less load, so it deepens the dip: tune shear first, then
stretch. A flat sheet in lateral compression is in unstable equilibrium and
never buckles on its own, so the particles are seeded with a few millimetres of
out-of-plane ripple (`kSeedRipple`) to choose a mode.

**The wind knob is the gust rate, not the strength.** The scene had inherited
Flag Cloth's `Perlin1D(g_windTime*0.05, ...)`, which advances half a period in
ten seconds - a slow lean, not a wind. That is why a strength sweep had found
no value giving both a plausible billow and any flutter. Fixes: run the noise
seven times faster with four octaves (`kGustRate`), make the wind horizontal
(the old direction had a large +y component, so more wind lifted the sheet over
its own bar instead of billowing it - at strength 1.6 the top edge finished
*above* the pins, dip/D -0.227), add `lift` (was 0), raise `drag` 0.06 -> 0.1
and cut `damping` 0.5 -> 0.1. Strength then only sets how far back the cloth
leans; above about 0.6 the drape washes out because the sheet is held flat.

Final constants: stretch 0.36, shear 0.15, bend 0.05, seed ripple 0.05,
gust rate 0.35, wind strength 0.6, 4 substeps; drag 0.1, lift 0.5, damping 0.1.

| metric | video | this scene (20 it) |
|---|---|---|
| dip/D | 0.276 (0.25..0.30) | 0.275 |
| H/D | ~1.23 | 1.273 |
| W/D | ~0.93 | 0.944 |
| z/D (billow) | n/a (2D projection) | 0.292 |

**Iteration independence at the final constants** (600 frames, mean of the last
120), which is the whole point of the scene:

| preset | 20 iterations | 160 iterations |
|---|---|---|
| XPBD dip/D | 0.275 | 0.269 |
| XPBD H/D | 1.273 | 1.267 |
| PBD (k = 0.01) dip/D | 0.582 | 0.174 |
| PBD (k = 0.01) H/D | 1.492 (resting on the floor, swing 0.000) | 1.056 |

That reproduces the video's comparison directly: XPBD's drape does not move
between 20 and 160 iterations, while the PBD cloth lies on the floor at 20 and
stiffens up at 160. No non-finite positions in any capture.

**Known remaining gap: withdrawn, it was a measurement error.** This section
previously claimed the cloth was about six times too large and swung twice as
slowly as the video, from an eyeball of an H/D series. Measured properly it is
wrong; see the amendment below.

**Tooling note.** `sweep.ps1` (scratchpad) edits one named constant in the
scene header, force-touches `demo/main.cpp` to defeat ninja's deps parser,
rebuilds, captures and measures, which is how every row above was produced.
Watch the float literal: `"{0}f" -f 1.0` yields `1f`, which is not a C++
literal and breaks the build.

## Amendment (2026-09-18b): aligning the cloth's dimension and timescale

Prompted by "for this scene, we can just fulfil XPBD; align scene dimension
with the video one and re-derive timescale", which was acting on the claim in
the previous amendment that the cloth was six times too big. That claim was
wrong, and the correction is the main result here.

**The timescale already matched.** `vperiod.py` takes the dominant period of a
cloth's sway by autocorrelating a detrended signal (mask centroid and hem width
on the video, particle centroid in the sim), cross-checked against the spectral
peak. On the video's panel 0 over 1080 frames at 60 fps, three independent
signals agree: T = 3.30, 3.30, 3.37 s by autocorrelation, 3.39 s by FFT, with
correlation peaks of 0.56-0.72. The 12.6 m scene gives 3.10 s free (wind off)
and 3.23 s wind-driven. That is within about 5%, not the factor of two claimed
before, which came from eyeballing an H/D series rather than measuring it.

So the scale was left at 12.6 m. It is worth being explicit about why this is
the right anchor: the sway period is the *only* observable in a 2D video that
fixes absolute scale, because every shape metric here is normalised by D. The
cloth looks like a ~2 m cloth and behaves like a 12.6 m one; the period is what
settles it, and shrinking the cloth would have broken a match that already held.

**Measurement-window error found and fixed.** The previous amendment's figures
came from the last 120 frames of a 600-frame run, i.e. a 2 s window. The gusts
are slow, so such a window samples one gust phase: re-measuring the *same*
capture over 20 s moved the dip from 0.275 to 0.249 and z/D from 0.294 to 0.470.
`simcloth.py` now reports medians with p10/p90 over a long window, matching how
the video figures are aggregated, and every number below is on that basis. The
earlier "dip 0.275 vs video 0.276" agreement was an artefact.

**Video targets, re-measured** over the 685 cleanly segmented frames of the
1080-frame shot (outliers rejected by requiring the panel's pin separation to
sit within 8% of its median): dip 0.277 (p10 0.233, p90 0.330), H 1.225,
W 0.931, T 3.33 s.

**Dimension alignment.** With the corners at full rest width the mid column
never stretches, so H/D = dip/D + height/width minus the wind lean (~0.04 D).
H was therefore pure geometry, and the grid moved from 64x64 to 64x63 with the
stretch coefficient softened 0.36 -> 0.345 to bring the dip back up.

| metric | video | this scene | error |
|---|---|---|---|
| dip/D | 0.277 | 0.270 | 2.5% |
| H/D | 1.225 | 1.215 | 0.8% |
| W/D | 0.931 | 0.909 | 2.4% |
| sway period | 3.33 s | 3.42 s | 2.7% |

**Iteration independence** on the same 20 s windows:

| preset | 20 it | 160 it |
|---|---|---|
| XPBD dip/D | 0.270 | 0.263 |
| XPBD H/D | 1.215 | 1.218 |
| PBD (k = 0.01) dip/D | 0.557 | 0.168 |
| PBD (k = 0.01) H/D | 1.468 | 0.972 |

1500 frames, 4032 particles, no non-finite positions in any capture.

**The PBD preset was then stripped at the user's request**, so the scene is XPBD
only: no solver-slider preset swap, no stiffness swap and re-upload, no `Sync()`
override, no `kPbdStiffness`/`kPbdSubsteps`. The PBD rows above are the last
measurement of it and are kept here because they are the video's actual
comparison; they are no longer reproducible from the scene. Re-measuring after
the strip gave dip 0.269, H 1.214, W 0.898 -- unchanged within the run-to-run
spread of the float atomics, as expected, since the removed branch only ever
fired when the solver mode changed.

The 0.2 m spacing now rests solely on the sway-period match. It had a second
justification while the PBD row existed (k = 0.01 can only hold the cloth up
when one frame of gravity, 2.7 mm, is a small fraction of a spring length);
that justification is gone, but the period one is the binding constraint and
is unaffected.

## Amendment (2026-09-18c): reproducing the video's hanging cloth exactly

Prompted by "reproduce the XPBD demo hanging cloth scene in
`data/xpbd_supplementary.mp4` exactly, align its geometry model, physical
constraints, wind force field, cloth stiffness, simulation visual effect".

The two previous amendments matched the cloth by comparing the simulation's
**world-space** shape ratios against **pixel** ratios read off the video. Those
are not the same quantity, and the difference is what most of the work below
undoes. Everything here is measured in pixel space on both sides, through the
same camera.

### The floor is a ruler

The shot is rendered with this demo: checkerboard plane, spotlight, capsule bar
and `DrawCloth` are all stock. `meshPS.hlsl` draws the ground through `bump()`,
which flips at every integer, so **the checker is one world metre per square**.

For a pinhole camera of pitch `t` over a plane, the image-space pitch `p` of a
1 m floor grid at image row `v` obeys exactly

```
v = v_horizon + p * h / cos(t)        ->      p = (v - v_horizon) cos(t) / h
```

so `p` is linear in `v`; the intercept is the horizon and the slope is
`cos(t)/h`, independent of focal length **and of image resolution**. Fitted over
28 frames x 4 panels of the "Our Method" shot the four panels agree to 0.5%:
slope 0.2152 /m, horizon at panel row 317.6, i.e. **camera height 4.49 m**. The
same estimator on a render of known camera height returns 9.40 m against a true
9.30 m, so it is good to about 1%.

The panel is 270 px wide with its floor ending at row 485; putting the horizon at
317.6 under the demo's 45 degree vertical fov needs a 203 px viewport, i.e. a
**4:3 render**, and a pitch of **15.0 degrees** -- which is the demo's own
default `g_camAngle`. Everything locks together.

### Scale: the cloth is 3.15 m, not 12.6 m

A silhouette plus a camera is scale-degenerate (scale the cloth and its distance
together and every pixel is unchanged), so the floor has to break it:

- the pins sit 14 panel rows **above** the horizon and the hem 110 below, so with
  h = 4.49 m the hem clears the floor only if the rest width W < 3.79 m;
- cloth width over the checker pitch just below the hem reads 5.15 m, and that
  recipe over-reads by 29% on a render of known size, correcting to about 4.0 m.

64 x 64 at the demo's usual 0.05 m spacing is W = 3.15 m. The confirmation is
that both readings can only be satisfied at once at the right size: at 3.15 m the
scene reproduces the video's checker slope to **0.08%** (0.21540 vs 0.21522 /m)
and its pin separation to **0.3%** (365.0 vs 366.2 px) in the same frame.

The previous amendment's sway-period argument is withdrawn. `T = 5.225 sqrt(L/g)`
is a free hanging chain; this cloth's top row is under 37% strain and it rings at
**1.43 s** in still air, not the 3.4 s the formula gives. The video's 3.33 s
autocorrelation peak is its gust envelope. The period could not have fixed the
scale at any size, in either direction.

### Geometry back to 64 x 64

The grid had been cut to 64x63 to close a gap in H/D. That gap was a projection
artefact: under the 15 degree pitch the hem is 14% further from the camera than
the pins, so the video's **pixel** H/D of 1.225 corresponds to a true H/D of
1.33, and 64x64 was right all along. 64 x 64 also gives 23,938 constraints, the
video's stated "24k"; 64x63 gives 23,554.

### The solver was not converged, and substeps fix it

At the 4 substeps the scene used, the drape moved between iteration counts --
which is precisely what the scene exists to disprove. Measured at the final
constants (top row rest length 3.150 m):

| substeps | iterations | top row | drop | dip/D |
|---|---|---|---|---|
| 4 | 20 | 4.396 m | 3.963 m | 0.332 |
| 4 | 160 | 4.193 m | 3.693 m | 0.268 |
| 8 | 20 | 4.197 m | 3.696 m | 0.268 |
| 16 | 20 | 4.194 m | 3.690 m | 0.268 |
| 16 | 160 | 4.194 m | 3.693 m | 0.268 |
| 32 | 20 | 4.196 m | 3.691 m | 0.271 |

A 64-wide sheet hangs its whole weight off its top row and FleX's Jacobi solver
with count-averaged relaxation moves tension about one cell per iteration, so 20
iterations cannot carry it across 64 cells within one substep. Substepping fixes
it: each substep corrects only one substep's worth of gravity and the tension
field carries over in the positions. **The scene now runs 16 substeps**, and the
full 20/40/80/160 sweep agrees to four figures.

No solver defect was found. The last rows are the positive check on the XPBD
implementation itself: alpha-tilde = alpha/dt^2 is supposed to make the result
independent of the substep count, and 8/16/32 substeps agree within 0.1%, where
PBD would stiffen with every extra substep.

### Wind

Rebuilt from measurements rather than tuned by eye:

- **The gust reverses.** The video's H/D has an rms of 0.124 about a median of
  1.175, and 1.175 is what this cloth projects to hanging dead vertical; its W/D
  runs 0.748..1.133 about a still-air 0.87. The swing is symmetric about the pin
  plane, which a wind that never reverses cannot produce -- it parks the sheet
  downwind and drags the median with it, which is what every clamped variant did.
  The `Max(0, ...)` clamp is gone.
- **The drive sits at the cloth's own period.** 1.43 s in still air, against the
  video's 1.73 s spectral peak, so `kGustRate = 0.7`. An earlier attempt at 0.30
  (the 3.33 s autocorrelation) is off resonance and suppresses the swing.
- Drag in `UpdateTriangles.hlsl` carries no triangle-area factor, so the wind
  acceleration does not depend on the cloth's size; and it drives the cloth
  toward the wind's *velocity*, so it both drives and damps. `kDamping = 0` so
  the free ring survives between gusts.

### Visual effect

- The bar is 1.456 W long (measured, consistent across frames; `AddCapsule`'s
  half-height excludes the end caps, so the call solves for the drawn length) at
  radius 0.063 m, centred one radius above the pin row.
- `DrawCloth` takes the face we see from `g_colors[3]` and the other side from
  `g_colors[4]`, both scaled by 1.5. The video's bar and floor are neutral grey,
  so the shot carries no colour grading -- and yet its cloth reads (79, 198, 164)
  where index 3 renders (0, 130, 94), and its folded-over side reads a neutral
  (225, 220, 220) rather than index 4's bright yellow. Both are now set from the
  video; `Init()` restores both so nothing leaks into the next scene.

### Result

Pixel measurements of the silhouette, one code path, 1140 video frames against a
2400-frame capture projected through the same camera:

| metric | video (p10..p90) | this scene (p10..p90) |
|---|---|---|
| pin row | 76.4 | 77.0 |
| hem row | 506.7 | 513.5 |
| D | 366.2 px | 365.0 px |
| dip/D | 0.277 (0.245..0.311) | 0.270 (0.230..0.321) |
| H/D | 1.175 (0.853..1.260) | 1.196 (1.032..1.549) |
| W/D | 0.903 (0.748..1.133) | 0.890 (0.762..1.107) |
| checker slope | 0.21522 /m | 0.21540 /m |
| cloth RGB | (79, 198, 164) | (81, 197, 164) |
| other side RGB | (225, 220, 220) | (210, 208, 206) |

**Still short, honestly.** The flutter is about a fifth too weak (detrended H/D
rms 0.102 against 0.124), and the two clothes get their variation from different
places: the video's H/D excursions go *down* (p10 0.853) because its hem lifts
during gusts, this scene's go *up* (p90 1.549) because its motion is mostly a
depth swing toward a camera 8 m away. Driving the hem up instead needs
`g_params.lift`, which acts on every triangle including the tensioned top row, so
every setting that lifted the hem enough also blew the catenary flat (dip/D
0.18..0.22 against the video's steady 0.27). `kLift` is held at 0.2, as much as
the dip tolerates. The video's cloth also curls its side edges far enough to show
its white back face; this one does not.

### Tooling notes

- `demo/main.cpp` gains `--screenshot=FRAME,PATH`: render to that frame, write a
  TGA and quit, with the UI suppressed. Used for every render comparison here,
  and it also lets a playback capture end as soon as its frame range is done
  instead of sitting at a timeout.
- **This toolchain's ninja does not rebuild on a scene-header change.** MSVC
  emits `/showIncludes` localized, ninja's `msvc_deps_prefix` does not match it,
  and the dependency is silently missed. Touch `demo/main.cpp` before every
  build; several measurements had to be redone after this bit. (The earlier
  `sweep.ps1` note about force-touching main.cpp was about this same thing.)
