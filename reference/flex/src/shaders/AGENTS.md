# NvFlex Shader Sources

## Introduction

This folder contains HLSL shader sources consumed by `NvFlexReversed` project.

The shader sources are recovered from `../dxbc` disassemblies. A Flex shader
group named `g_Flex_<Name>` usually maps to `<Name>.hlsl` or
`<Name>.hlsl_rev`; BVH shader groups named `g_bvh_<Name>` map under `bvh/`.
The `*.asm` files in `../dxbc` are disassembled DXBC 5.0 sources and are the
authority for reverse-recovery work.

## Development Guidelines

- HLSL assembly reference: [Shader Model 5 Assembly](https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/shader-model-5-assembly--directx-hlsl-).

- Read the matching `../dxbc/*.asm` before changing a shader. Use the preamble
  resource table, `dcl_*` declarations, thread-group size, and instruction body
  together; do not rely on `../dxbc/*.hlsl` except as a rough decompiler hint.

- Map `cb0[...]` fields through `KernelParams.hlsli`. Preserve resource binding
  slots, element strides, raw vs typed/structured buffer kinds, UAV write
  masks, atomics, and `numthreads` values.

- Vector swizzles in assembly are component-wise assignments. For example:
  ```
  mul r12.yxz, r10.yzyy, r17.zxzz
  ```
  means:
  ```
  r12.y = r10.y * r17.z
  r12.x = r10.z * r17.x
  r12.z = r10.y * r17.z
  ```

- To compile shader object headers, keep the source listed in `Shaders.cfg` and
  run:
  ```
  cmake --build build --config Debug --target NvFlexShaders
  ```
  `Shaders.cfg` entries use `-T cs -E <EntryPoint>` and are retargeted by CMake
  to `cs_5_0`. Example:
  ```
  CalculateBounds.hlsl                                -T cs -E CalculateBounds
  ```

- To compare generated DXBC against the original source of truth, diff the
  disassembly block enclosed in the generated `{entry}.hlsl.h` file against the
  matching `../dxbc/g_*.asm`. The generated header layout is:
  ```
  #if 0
   // enclosed disassembly DXBC section
  #endif
   // byte constant definition follows
  ```

- **Use `tools/dxbccmp.sh` for this.** It does everything described below --
  compile with the right compiler, normalise, diff -- in one step, and it is the
  acceptance test:

  ```
  bash tools/dxbccmp.sh <Entry>        # PASS/---- raw= rn= firstdiff= instr=
  bash tools/dxbccmp.sh <Entry> -v     # plus both diffs; '<' is shipped, '>' is ours
  ```

  It reads the recipe out of `Shaders.cfg`, resolves the shipped blob under the
  three naming conventions (`g_Flex_`, `g_bvh_`, `g_` + capitalised entry), and
  exits 0 only when the SHEX chunk is byte-identical. `raw` counts differing
  disassembly lines, `rn` counts them with register names normalised away,
  `firstdiff` is the index of the first differing instruction, and `instr` is
  shipped/ours. `tools/bytecmp.py <outdir> <fxc>` runs the same comparison across
  all 82 manifest entries at once.

  **Compile with FXC 6.3.9600.16384, vendored at `tools/fxc63/`** (gitignored;
  see the provenance note below for how to re-fetch it). Its `d3dcompiler_47.dll`
  must sit beside `fxc.exe` -- the copy in `C:\Windows\System32` is a far later
  build. That said, 6.3 and 10.1 were measured to emit byte-identical code for
  every shader in this tree, so a 10.x SDK compiler is an acceptable substitute;
  6.3 is preferred only because it is what built the shipped blobs.

  The manual recipe below is kept because it documents what the tool does and is
  what you need if you are comparing something outside `Shaders.cfg`.

- The round trip through FXC is the only way to judge bytecode equivalence. On
  Windows the SDK compiler is used directly:
  ```
  fxc=$(find "/c/Program Files (x86)/Windows Kits/10/bin" -path '*/x64/fxc.exe' \
      | sort -Vr | head -1)
  "$fxc" -nologo -T cs_5_0 -E <Entry> -Iexternal/nvapi/include -Fh out.h \
      src/shaders/<Name>.hlsl_rev
  ```
  Under Git Bash, export `MSYS2_ARG_CONV_EXCL='*'` and `MSYS_NO_PATHCONV=1`
  first: MSYS otherwise rewrites `/nologo` into a path and FXC reports
  `Too many files specified`. With path conversion off, give `-Fh` a Windows
  path (forward slashes are fine) and keep the source path relative to the
  repository root.

  On Linux the same command runs under Wine:
  ```
  fxc=$(find "${WINEPREFIX:-$HOME/.wineprefixes/dev-x64}/drive_c/Program Files (x86)/Windows Kits/10/bin" \
      -path '*/x64/fxc.exe' | sort -Vr | head -1)
  WINEDEBUG=-all wine "$fxc" /nologo -T cs_5_0 -E <Entry> -Fh out.h src/shaders/<Name>.hlsl_rev
  ```
  Keep the input and output paths relative to the repository root; FXC runs
  under Wine and cannot open absolute Linux paths.

  Extract the block from `cs_5_0` to `#endif` in `out.h` (it is CRLF), strip the
  `dcl_constantbuffer` / `dcl_resource_*` / `dcl_uav_*` / `dcl_globalFlags` /
  `dcl_indexableTemp` lines and the trailing `// Approximately N instruction
  slots used` comment, then diff against the same range of `../dxbc/g_*.asm`.
  The shipped `.asm` files were disassembled by 3Dmigoto, which prints float
  literals with eight decimals where FXC prints six, so collapse them first
  (`sed -E 's/([0-9]+[.][0-9]{6})[0-9]+/\1/g'`) or `1.0e-9` reads as a
  difference. A second pass with register names normalised away
  (`sed -E 's/\br[0-9]+\.[xyzw]+/R/g; s/\br[0-9]+\b/R/g'`) separates real
  differences from register-allocation noise; a third pass that also drops
  `mov` lines isolates control-flow and arithmetic differences from FXC 6.3's
  copy noise, which dominates the large shaders.

- Vendor extension variants use the real vendor header, never a local
  transcription: `#include <nvHLSLExtns.h>` from `external/nvapi/include`, which
  `src/CMakeLists.txt` already puts on the shader include path, plus
  `-DNV_SHADER_EXTN_SLOT=u7` in `Shaders.cfg`. Read the opcode the DXBC stores at
  byte 0 of the `u7` record against `nvShaderExtnEnums.h` before naming the
  intrinsic: opcode 3 is `NV_EXTN_OP_SHFL_DOWN`, and `NV_EXTN_OP_SHFL_XOR` is 4.
  Note `grep` can silently fail to match inside these headers because they are
  ISO-8859 rather than UTF-8; pipe through `tr -d '\r'` and use `LC_ALL=C`.

- Several apparent "compiler version" differences are in fact source-controlled.
  **Assume a difference is reachable from source until a measurement says
  otherwise** — every group that was ever written off as a compiler artifact in
  this document turned out to be reachable. The levers below, plus the fuller
  set under [The FXC 6.3 codegen levers, as
  measured](#the-fxc-63-codegen-levers-as-measured), took every remaining group
  from "explained away" to byte-identical:

  - **Commutative operand order.** FXC 10.1 emits the operands of `mul` in the
    reverse of their source order, so `a * b` disassembles as `mul b, a`; write
    the HLSL the other way round to match the DXBC. `mad`, `add`, `min` and
    `and` behave the same way in most, though not all, positions, so confirm
    each swap with a round trip rather than assuming.
  - **Scheduling by hoisting.** FXC 6.3 emits loop-invariant work in source
    order ahead of the loop; 10.1 sinks it to first use. Binding the expression
    to a named local before the loop (`int lastContact = count - 1;`,
    `bool adhesionEnabled = gParams.kAdhesion != 0.0;`) restores the shipped
    order. The same trick fixes load placement inside a loop body.
  - **Vectorised selects.** Where the DXBC selects several values in one wide
    `movc`, write the selects as a single `float4`/`float2` ternary rather than
    as separate scalar ones.
  - **Basis-axis `cross()`.** 10.1 will not fold `cross(q.xyz, float3(1,0,0))`
    down to one masked `mul`. Supply the folded cross and dot to a `RotateBasis`
    helper (see `TransformShapeBounds.hlsl`, `SolveShapes.hlsl`) and the
    single instruction comes back.
  - **Gate shape.** An `a && b && c` chain compiles to `and`s feeding one
    `if_nz`; the DXBC's nested `if_nz` gates need nested `if` statements in the
    source. Likewise, where the DXBC performs a store inside a helper rather
    than gating on its return value, the helper has to do the store.
  - **Groupshared stores.** Clearing a `float3x3` by rows rather than columns
    reproduces the shipped three 12-byte `store_raw`s instead of 16/16/4.
  - **Signed versus unsigned shifts.** `threadIdx >> 5` on a `uint` emits
    `ushr`; the DXBC's `ishr` needs `int(threadIdx) >> 5`.

- The shipped `.asm` files were produced by FXC 6.3.9600 (Windows 8.1 SDK); the
  SDK available here is FXC 10.1. Even for a perfect source recovery the two
  differ in ways that carry no semantic weight, and these are expected:
  register allocation and the `mov` shuffles it implies, instruction scheduling
  within a basic block, operand order of commutative `add`/`mul`/`and`/`or`,
  destination write masks and the source swizzles that follow from them, and
  constant folding of `cross()` against a literal basis vector. Chase a
  difference only when it changes the opcode sequence, the control flow, or an
  operand's meaning.

- Local DXC checks are useful syntax guards for recovered sources, but FXC/DXBC
  output remains authoritative for bytecode matching.

## Conventions

- Keep hand-recovered shader source close to the DXBC body. Prefer clear,
  direct HLSL that preserves control-flow gates, arithmetic order, and output
  semantics over high-level rewrites.

- Use `.hlsl_rev` while a hand-recovered source is still being reconciled with
  its DXBC. Once its group reaches `Exact` or `Verified` in the status table
  below, rename it to `.hlsl` and update its `Shaders.cfg` line (keep the flag
  column at 52), its `src/Library.cpp` generated-header include, any `#include`
  of it from a wrapper variant, and its table row in the same change; the suffix
  marks work in progress, not provenance.

  **No `.hlsl_rev` files remain.** Every hand-recovered source in this folder has
  been reconciled. Reintroduce the suffix only for genuinely new recovery work.

- The acceptance bar is **DXBC equivalence against the shipped blob**, measured
  by `tools/dxbccmp.sh <Entry>`, which compiles with the vendored FXC
  6.3.9600.16384 and compares. It is fast, deterministic, needs no GPU, and it is
  decisive when it reaches zero. Prefer it to any behavioural argument: a
  difference that is bit-neutral at runtime can still be wrong in the source, and
  most of the defects found in this tree were exactly that.

- Entry-point names must match the shader group name without the `g_Flex_` or
  `g_bvh_` prefix. Wrapper variants such as `SolveShapes32NV.hlsl` may
  include the composed NV body source and only change `numthreads`/entry point
  when that matches the corresponding DXBC.

- Vendor variants with `NV` or `AMD` suffixes may share a composed source file
  only when the composed source covers the matching DXBC resources and extension
  slots. Keep required `-D...` flags in `Shaders.cfg`.

- Do not add `-Vn` to new `Shaders.cfg` entries unless the C++ side explicitly
  needs a non-entry symbol name. Existing legacy `-Vn` lines should be left
  alone unless their C++ lookup convention is changed deliberately.

- Remove decompiler artifacts before marking a shader complete. No source in
  this directory should contain `Needs manual fix`, `unknown dcl_`, `Known bad
  code`, `3Dmigoto`, or namespace-style decompiler identifiers.

- Keep comments sparse and technical. Do not leave uncertainty notes such as
  `TODO` in recovered shaders unless the work item is still intentionally open
  and reflected in the status table.

- When updating `Shaders.cfg`, verify that every recovered source is listed
  exactly once, the entry point matches the file stem, and the line does not
  introduce duplicate output names unintentionally.

- `ShadersXPBD.cfg` holds the XPBD entries, which have no shipped blob. They are
  built by the separate `NvFlexShadersXPBD` target and sit outside the
  DXBC-equivalence sweep (`tools/bytecmp.py`, `tools/dxbccmp.sh`).

## Current Work Status

Status is tracked per unique DXBC shader group. The authoritative recovery
source is the corresponding `.asm` disassembly in `../dxbc`; DXBC-side `.hlsl`
artifacts are only rough decompiler hints.

There are two statuses, and both are measured, never asserted:

- **`Exact`** — the recovered source, compiled with FXC 6.3.9600.16384, produces
  a SHEX chunk **byte-identical** to the shipped blob. Not "equivalent", not
  "accounted for": the same program. `tools/dxbccmp.sh <Entry>` exits 0.
- **`Verified`** — not byte-identical, but a runtime differential test showed
  bit-identical output against the shipped blob on scenes whose run-to-run noise
  floor is exactly zero. This is a weaker claim and is always provisional: a
  `Verified` row means the entry has been attacked and its residual bounded, not
  that it is finished. All five remaining are diagnosed in "The five that did not
  close" below.

Summary of 84 DXBC shader groups:

| Status | Count |
| --- | --- |
| `Exact` — byte-identical bytecode | **79** |
| `Verified` — runtime bit-identical | **5** |

(84 groups against 82 `Shaders.cfg` entries: the three
`ComputeTotalBoundsFinalize` variants ship as one identical blob and are built
from a single manifest entry, so two rows are aliases of a third.)

Nothing sits below those two. Earlier revisions of this document also used
`Equivalent` ("every difference individually accounted for as a compiler
artifact"), `Partial`, `Divergent` and `Complete`. They are gone, and the reason
is worth keeping: **every group that ever carried `Equivalent` was later driven
to byte-identical DXBC.** Each "individually accounted for" explanation turned
out to be a source difference nobody had found yet — including three this
document had explicitly recorded as closed and unreachable. If a new recovery
cannot reach `Exact`, treat it as unfinished rather than inventing a status for
it.

The table is checked against a live sweep, not maintained by hand. Re-run
`tools/bytecmp.py` and cross-reference before trusting a row; the last audit
found eleven rows understating what had been achieved and none overstating.

**`Verified` cannot be promoted to `Exact` by argument.** The question has been
asked and answered by measurement: of the twenty entries carrying `Verified`
before the last campaign, two were computing genuinely different numbers behind
structurally clean disassembly — `SolveVelocities` summed a `dp3` over the wrong
lane packing, and `UpdateDiffuseParticles` tested plane collisions against the
running velocity where the shipped blob uses the entry velocity. The second sat
at `rn = 0`, meaning register-normalised disassembly that was *textually
identical*. Instruction-level equivalence is therefore not sufficient evidence of
semantic equivalence, and `SolveSprings` before it made the same point: `rn = 0`
while writing every spring delta to the wrong particle. Fifteen of those twenty
were promoted the only way that works — by finding the source difference and
reaching byte-identity.

### Reading the measurement

```
bash tools/dxbccmp.sh <Entry>        # PASS/---- raw= rn= firstdiff= instr=
bash tools/dxbccmp.sh <Entry> -v     # plus both diffs; '<' is shipped, '>' is ours
python tools/bytecmp.py <outdir> <fxc-path>   # the same check across all 82 entries
```

- `rn` — differing disassembly lines with register names normalised away. This is
  *structure*. Drive it to zero first.
- `raw` — differing lines including register names. Drive it to zero second.
- `firstdiff` — index of the first differing instruction, and the single most
  informative number: everything after a divergence is misaligned, so a large
  `raw` with a late `firstdiff` is nearly done.
- `instr` — shipped/ours instruction counts. Parity here is a strong signal;
  losing it is a red flag even when another number improves.
- **A raw byte count is not a gradient.** An early version of the tool compared
  bytes positionally, so once instruction counts shifted it saturated and once
  scored a structurally *worse* state as better. Bytes are meaningful only as a
  pass/fail at zero.

## Reproducing the shipped bytecode

### FXC 6.3.9600.16384, fetched locally

The shipped blobs were built with the Windows 8.1 SDK compiler. It now lives at
`tools/fxc63/` (gitignored -- Microsoft binaries, fetched, never committed):

| file | version | sha256 |
| --- | --- | --- |
| `fxc.exe` | 6.3.9600.16384 (winblue_rtm.130821-1623) | `1a7fb5609b54f046835e55fe572b712d06d627750f2cffa60b5c4e4292e56420` |
| `d3dcompiler_47.dll` | 6.3.9600.16384 (winblue_rtm.130821-1623) | `0bea1620dd82f5e6be1650fbe84ce819ba9f32f4148308cf1e68c4d825d74937` |

No SDK was installed. Provenance, reproducible: the 8.1 SDK web installer
(`https://go.microsoft.com/fwlink/p/?LinkId=323507`) is a WiX bundle whose
embedded `BurnManifest` lists the `Installers/` payloads. `fxc.exe` is not in
the main SDK MSI -- it is in `Windows Software Development Kit for Windows Store
Apps-x86_en-us.msi`. Querying that MSI's `File`/`Media` tables through
`WindowsInstaller.Installer` (read-only, no install) puts the x64 `fxc.exe` at
sequence 74 and its `d3dcompiler_47.dll` at sequence 32, both inside
`69661e20556b3ca9456b946c2c881ddd.cab`. Downloading that one 9 MB cab and
extracting the two members by their File keys is the whole operation.

Both must sit in the same directory. `fxc.exe` resolves `d3dcompiler_47.dll` by
the normal search order, and `C:\Windows\System32` holds 10.0.19041.7663, which
would silently give 10.x codegen. The check that this is working: any `-Fc`
listing must carry `Generated by Microsoft (R) HLSL Shader Compiler
6.3.9600.16384`, exactly as the shipped `.asm` files do.

### The FXC 6.3 codegen levers, as measured

These were established empirically against the shipped blobs and are the
transferable result of the campaign. FXC's *shape* is far more sensitive to
source spelling than its *arithmetic* is.

- **Operand order.** `mul` emits its operands in **reverse** source order; `mad`
  keeps source order. For `P + Q`, the **second** addend becomes the base `mul`.
  Commutative `add` operands also come out reversed, which is why `NormalizeQuat`
  must be written `sq.x + sq.y` to produce `add r3.w, r4.y, r4.x`.
- **Parenthesisation decides mad-contraction.** `a*b + c*d + e*f` flat is a
  different chain -- and different rounding -- from `a*b + (c*d + e*f)`.
- **`cross()` intrinsic vs component-wise.** The intrinsic packs three-wide;
  component-wise yields a two-wide pair plus a scalar. This matters *semantically*
  when the result feeds a `dot`, because the lane packing sets the `dp3`
  summation order -- that is the `CalculateVorticity` defect. Note the causation:
  the *packing* selects the summation order; the `dot`'s own argument order does
  not (see "What is not source-controlled" below).
- **Hoisting the *last* product of a chain into a named temp changes lane
  assignment ~30 instructions upstream.** `float zz = a.z * b.z;` in `QuatMul` is
  what moves `ExtractRotation`'s 4-wide cross from lanes `y,z,w` to the shipped
  `x,z,w`. Hoisting any *earlier* product fails, or lets FXC fold the hoisted mul
  into the leading three-wide `mul` and lose an instruction. Only the last term
  is safe.
- **Concatenated versus interleaved operand layout.** Two two-wide products
  written as separate terms are laid out concatenated -- group 1 at lanes (x,y),
  group 2 at (z,w). Materialising them as one four-wide multiply read back
  stride-two states the interleaved layout the shipped blob uses:

  ```hlsl
  float4 t = a.yzzx * b.zyxz;
  float2 xy = v.xy + t.xz - t.yw;
  ```

  FXC re-fuses `t` into the two mads, so the instruction count is unchanged and
  the arithmetic is bit-identical; only the operand lanes move. This was the
  final two-instruction difference on all eight shaders.
- **`inout` on a parameter forces a copy that LICM hoists.** Declaring
  `ReduceSum`'s thread index `inout uint` produces the shipped
  `mov rN, vThreadIDInGroup.x` in each loop preheader, which in turn keeps
  `and l(31)` / `ishr l(5)` inside the loop body instead of hoisted to the
  prologue. FXC evaluates expressions over *input registers* once in the
  prologue but leaves expressions over a *temp* where they are written.
- **Statement placement controls load order and scheduling.** Binding
  `rotations[rigid]`, `float(count)`, `2*w*w-1` or `addr = idx << 4` before the
  expression that consumes them moves the corresponding load or `ishl` to the
  shipped position.
- **Defeating load CSE.** Two loads of the same index survive only if the two
  uses are spelled differently -- `localNormal.xyz` for one term and
  `localNormals[entry].xyz` for the others. Routed through one function parameter
  FXC always merges them.
- **Integer comparisons reverse their operands** the same way `mul` and `add` do.
  `prevCellId != cellId` emits `ine cellId, prevCellId`; swapping the source
  operands was the sole change that closed `CreateGrid`. `mad` and `dp3` keep
  source order.
- **`x == false` versus `!x` decides whether a mask test is materialised.** With
  `bool b = (phase & MASK) != 0;`, spelling the branch `if (!b)` lets FXC fold the
  test into the branch (`and` + `if_z`); `if (b == false)` keeps an explicit
  `ine rN, rN, l(0)`. Each occurrence is +1 instruction. The `!= 0` in the
  *initialiser* does not produce the `ine` -- only the compare in the branch
  condition does. Two occurrences took `CalculateVorticity` from 55 to the
  shipped 57.
- **Ternary arm order picks `ult` versus `uge`.** `x < n ? keep : sentinel` emits
  `ult` with `movc` sources in (keep, sentinel) order; `x >= n ? sentinel : keep`
  emits `uge` with them reversed. Match the shipped opcode by choosing the arm
  order, not by negating operands.
- **Splitting a wide op into a narrower one plus a scalar restores a missing
  instruction.** This is the counterpart of the interleaved-layout lever above,
  and it fixes *counts*, not just swizzles. A wide vector op followed by a
  reduction, or a per-component `cond ? a : b` over a vector, folds into fewer
  instructions than a shipped blob that interleaves them; writing the last lane
  as a separate scalar placed *between* the other operations in source order
  restores parity. `cross(a,b)` compiles to two three-wide instructions where the
  shipped blob has four. This closed all three radix sort entries,
  `CalculateMortonCodes`, and `SolveVelocities` (139 to 142 instructions and raw
  12 to 0 in one edit). **The statement order inside such a helper is
  load-bearing**: in `TransformShapeBounds` the scalar `tw = a.y*b.x` must sit
  *after* the two-wide mad, not before it, or the count comes out one short.
- **A duplicated lane in a shipped wide op means the source spells that value
  twice, from different expressions.** FXC CSEs identical spellings, including
  through `float4` constructors and element-wise constructors; only an
  independently derived expression survives -- `(v1+v2+v3).z` alongside
  `(v1.zyx+v2.zyx+v3.zyx).x`. Probe: `UpdateTriangles`, raw 72 to 30; the same
  spelling in any of four forms always dedups back to 72.
- **`mad` inherits the operand order of a CSE'd sibling `mul`.** Given
  `float w = q2 * k;` and a later `q2 * k + acc`, the `mad` takes the
  *initialiser's* order regardless of how the `mad` line is written. Fix the
  initialiser.
- **Binding a product to a local before accumulating it fixes operand-pack
  scheduling.** `acc += w * p;` lets FXC interleave the next pack with the
  current `mad`; `float4 t = w * p; acc += t;` forces every pack to be
  materialised before the first `mad`. FXC re-fuses to the identical `mad`, so it
  is free. Seven other spellings failed on `CalculateAnisotropy`; only splitting
  the multiply from the add worked.
- **Naming an intrinsic used in one arm of a ternary hoists it above the
  compare.** FXC sinks an intrinsic that is live in only one arm to *after* the
  compare; binding it first emits it before. One `float v1InvNorm =
  rsqrt(v1Norm);` closed `Predict`, fixing six raw lines and every downstream
  lane choice. This is the statement-placement lever extended across a `?:`
  boundary into conditionally live code.
- **A function-parameter boundary blocks re-fusion where nothing else will.** In
  `CollideTriangles`, `SegmentIntersectsTriangle` recomputed `end - start`
  internally, so FXC hoisted its own copy to the wrong place; passing the delta
  in as a parameter dropped raw from 354 to 54, the single largest raw win in the
  project. Where a helper recomputes a value the caller already has, pass it.
- **Where a function's body is emitted is itself a lever.** The shipped
  `CollideTriangles` does its per-shape transform setup in the caller's loop, not
  in the traversal function; the `inout int stack[40]` copy-in is 78 `mov`s
  emitted at the call site, and its position gives the function boundary away.
  Splitting the traversal out so the setup precedes it moved raw 580 to 398.
- **Declaration order breaks coalescing tie-breaks.** With `result` declared
  before `sp`, FXC emits `mad sp, ...` / `mov result, sp`; declaring `sp` first
  emits the shipped `mad result, ...` / `mov sp, result`. Nothing else moved it.
- **The lane order of a packed comparison follows the *declaration* order of
  named `bool`s**, not the order the masks were extracted or first used. Hoisting
  `bool ea = a != 0; bool eb = b != 0;` in the desired order before the loop pins
  the pack, at zero instruction cost.
- **A flat left-fold reproduces a shipped or-chain.** `any(A) || any(B)` written
  as a six-term flat chain -- `a.x||a.y||a.z` bound, then `that||b.x||b.y||b.z` --
  produces the shipped fold and keeps the two `lt`s apart.

### What is *not* source-controlled

Equally important, and each disproven by measurement rather than assumed. These
bound the search space: when a residual is one of these, no spelling will reach
it and the effort belongs elsewhere.

- **`dp3`/`dp4` summation order.** FXC canonicalises the reduction to ascending
  lane order of whatever packing it chose, so `dot()`'s argument order does not
  select it. It is reachable only through a permute *before* the reduction. This
  corrects a reading of the operand-order lever above: `dp3` keeping source order
  is about which register is the first *operand*, not about the order the lanes
  are summed. The `SolveVelocities` defect had to be fixed with a `float4`
  repack (`vorticityGradient.yzzx`) precisely because reordering the `dot` could
  not reach it.
- **A pure swizzle rewrite.** Passing `geometry[idx].yzxw` and rewriting every
  case body through rotated accessors -- a literal statement of the shipped lane
  layout -- compiled byte-identical to the unrotated source. FXC normalises
  source lane spelling away. Lanes move only when *what is live* or *what wide op
  is materialised* changes, which is why the `SolveShapes` cross permutation
  yielded to a four-wide multiply and not to any respelling.
- **Integer loop-carried register packing.** Rewriting loop state as
  `int3 it = int3(...)` with every `continue` rewriting all components -- exactly
  what the shipped `CollideParticles` does -- was fully scalarised, instruction
  for instruction identical to scalar source. Packing is decided after
  scalarisation.
- **Whether an integer cbuffer read is hoisted.** Substituting three *different*
  cbuffer fields into the same compare produced identical prologue `mov`s and
  identical instruction counts; meanwhile the same field read inline three lines
  below in the same basic block is *not* hoisted. The hoist is positional and
  field-independent, so the CSE-defeat rule cannot reach it -- there is only one
  use to spell.
- **Operand order of a vector-times-broadcast-scalar `mul` whose operands share a
  register.** Swapping `v * s` to `s * v` there is byte-identical; the printed
  order is a register-allocation consequence. Do not chase those.
- **`precise`.** Tried as a lane-placement lever on a duplicated scalar; it
  perturbs the whole shader (raw 30 to 59, and +1 instruction) rather than
  pinning the pack. It is not a codegen lever.

### Dead ends, recorded so they are not re-run

Roughly sixty rounding-neutral variants were measured and rejected across the
campaign. The ones worth naming: binding `a.x*b.x` to a temp or putting the `w`
chain before `z` (FXC folds the mul into the leading four-wide `mul` and *loses*
an instruction -- a better `rn` with worse structure); a fully vectorised
`QuatMul` (seven instructions where the shipped blob has ten); component-wise or
swizzled spellings of `ExtractRotation`'s numerator cross; deriving a warp index
from `index & (BLOCK-1)` or `index % BLOCK` (FXC rewrites to `ubfe`/`ishr` and
inserts groupshared bounds clamps absent from the original -- this one reached the
exact shipped instruction count while corrupting structure, and is the clearest
false optimum in the project).

### Method notes

- Drive `rn` (register-normalised distance) to zero first, then `raw`, then
  bytes. `firstdiff` -- the index of the first differing instruction -- is the most
  informative single number, because everything after a divergence is misaligned.
- A raw byte count is **not** a gradient. An early version of the tool compared
  bytes positionally, so once instruction counts shifted it saturated; it once
  scored a structurally worse state as better. It is meaningful only as a
  pass/fail at zero.
- Prefer lower `raw` with exact instruction parity over marginally lower `rn`: a
  single displaced instruction is counted twice in `rn`.
- Hypotheses cost nothing when tested on throwaway copies compiled with the same
  FXC invocation; only verified-better edits need touch the real file.

## The hardest case: the cross lane permutation

Only one episode from the DXBC-equivalence work is worth keeping in full, because
it is the one that generalises. The eight `SolveShapes` /
`SolveShapesPlasticDeformation` variants had resisted for a long time, and after
the obvious levers they all converged on a single residual — two instructions,
byte-for-byte the same in every one of the eight:

```
<  mad r5.xz, r4.xxzx, r2.zzxz, r5.xxzx      shipped reads r4.z then r4.y
>  mad r5.xz, r4.xxyx, r2.zzxz, r5.xxzx      ours reads r4.y then r4.z
```

In `ExtractRotation` the numerator is `cross(r0,c0) + cross(r1,c1) +
cross(r2,c2)`. FXC emits it as one 4-wide `mul`/`mad -r` pair with a duplicated
lane, and the two builds packed it differently: shipped put the float3 in lanes
`x,z,w` holding components `(y,z,x)`; ours in `y,z,w` holding `(x,y,z)`. That one
rotation dictated the `dp3` swizzle, how much of `QuatMul` could be vectorised,
and the operand order in `NormalizeQuat`.

**Three wrong diagnoses were reached before the right one**, and each looked
convincing:

1. *"FXC chooses the packing from the consumer side."* Reached after twelve
   producer-side spellings of the cross sum failed to move it. It sent the next
   attempt at `omega`, `angle`, `axis` and the loop tail, which also failed.
2. *"It is a post-vectorisation tie-break between two lanes holding the same SSA
   value."* This one is nearly true and is the most dangerous kind of wrong: the
   4-wide cross really does write component Z into both lane `y` and lane `z`, so
   `a.z` genuinely has two valid homes. The conclusion drawn — that the choice is
   made after vectorisation and so is unreachable from HLSL — was wrong.
3. *"Not reachable from HLSL."* Recorded as a conclusion with roughly sixty
   measured variants behind it. It survived one further round of searching before
   being disproved.

The actual mechanism is a **layout** decision, made from the source:

> The two two-wide cross terms form a four-element operand set from `a`. Written
> as separate terms, FXC lays them out **concatenated** — group 1 at lanes (x,y),
> group 2 at (z,w). The shipped blob lays them **interleaved** — group 1 at (x,z),
> group 2 at (y,w). Both hold the same four values, which is exactly why `rn` was
> already 0 and the producing cross was already byte-identical; the only visible
> trace was two read swizzles.

Materialising the products as one four-wide multiply read back stride-two states
the interleaved layout directly:

```hlsl
float4 t = a.yzzx * b.zyxz;
float2 xy = v.xy + t.xz - t.yw;
```

FXC re-fuses `t` into the two mads, so the instruction count does not change and
the arithmetic is bit-for-bit identical; only the operand lanes move. That closed
all eight shaders.

Two lessons worth more than the fix:

- **A difference that leaves `rn` at zero can still be source-controlled.**
  Register-normalised distance deliberately erases swizzles, so a pure layout
  difference is invisible to it. When `rn` is 0 and `raw` is not, read the raw
  diff rather than assuming register allocation.
- **"Not reachable from HLSL" is a claim that needs the same evidence as any
  other.** Sixty failed variants are evidence about the variants tried, not about
  the search space. Every such conclusion in this document's history has been
  overturned.

## The five that did not close, and exactly why

Five entries remain `Verified`. Each was driven as far as measurement allowed and
is recorded here so the same ground is not covered twice. All five have **exact
instruction parity**; none has a known arithmetic difference.

| Entry | raw | rn | residual |
| --- | --- | --- | --- |
| `CollideShapes` | 440 | 2 | one fused multiply; 440 lines are its register renumbering |
| `UpdateTriangles` | 30 | 2 | lane permutation of one register |
| `UpdateTrianglesNV` | 30 | 2 | same register, same cause |
| `TransformShapeBounds` | 28 | 2 | free global lane rotation on the geometry load |
| `CollideParticles` | 117 | 11 | one hoisted int cbuffer read, plus five `mov`s it forces |

### `CollideShapes` -- FXC has no gradient between two stable shapes

Only two codegen shapes are reachable from HLSL at the convex-plane normalise:

- **Natural load** -- FXC always splits into `mul r28.y` (the scalar `wRaw`) plus
  a three-wide `mul r29.xyz` in a fresh register, with no gather. Eleven distinct
  spellings all produce byte-identical output here, including four scalars in
  shipped lane order, component-wise assignment into a declared `float4` or
  `float3`, writing `.w` before `.xyz`, both operand orders, and two helper forms.
- **Any spelling that states the permutation** -- FXC produces the fused 4-wide
  `mul`, but *sinks the permutation into the `ld` swizzle* and the `dp3`, drops
  the gather `mov`, and needs an extra `mov` to rebuild the pair. Strictly worse
  on every number (raw 449, rn 3, 1046 instructions).

The shipped blob is a **third** combination: fused 4-wide `mul` with a *natural*
load plus a gather `mov`. Every spelling that asks for the fusion also licenses
the lane renumbering, and every spelling that denies the renumbering also loses
the fusion. Both reachable shapes cost two instructions, so FXC has no gradient
to follow between them.

Pinning the other operand -- the usual escape -- is disproven at the mechanism
level rather than merely untried: `invScale` already occupies identical lanes in
both builds (`div r21.yzw, l(1,1,1,1), r18.xxyz`, with `r21.x` holding an
integer), and a swizzled read costs nothing, so the permutation sinks regardless.
The only untried direction is changing what the `dp4` reads, and every relaxation
of its contiguity demand changes the reduction and is not bit-neutral.

### The other four

- **`UpdateTriangles`** -- ours packs `[w.z, w.x, wz, w.y]`, shipped
  `[wz, w.z, w.y, w.x]`. Eleven spellings measured across two rounds, with only
  two outcomes: this packing, or a regression. Unlike `SolveVelocities`, the
  consumer cannot be used as the lever -- every consumer reads the register
  through a *free swizzle*, and the only lane-pinned consumers (three
  `InterlockedAddFp32` calls) constrain it merely up to a permutation, which both
  builds satisfy. There is no demand to move.
- **`TransformShapeBounds`** -- the shipped blob loads geometry as `t5.yzxw` and
  stores `upper` rotated; `lower`, `edges`, `center` and every store are identity
  in both. Proven to be a free tie-break: `upper`'s layout equals `geom`'s in
  every switch case and both layouts cost the same, the `mov l(0)` mask lane
  simply moving from `y` to `z`. Fourteen variants failed -- nine respellings and
  five structural materialisations, of which two were exactly inert and three cost
  an instruction.
- **`CollideParticles`** -- the surplus is one prologue `mov` of an int cbuffer
  read plus five `mov`s it forces: FXC pairs the hoisted invariant with
  `nextNeighborIdx`, evicting `numNeighbors` into a third loop-carried chain,
  where the shipped blob carries one pair through all three grid loops. The hoist
  is positional and field-independent (see the negatives above), and survives
  every change to the compare, to the other operand, to surrounding liveness, and
  to wrapping the block in an `inout` helper. The remaining hypothesis is that the
  **loop nest itself** differs from the original -- everything inside it has been
  ruled out.

None of these is recorded as unreachable. This document's history is emphatically
against that claim: `RotateBasis`, the +/-1.0 literal, the cross lane permutation
and the inflatable family were each called closed or unreachable and each later
yielded. They are recorded as *unreached, with the search space bounded*.


## Shader inventory

Behaviour worth knowing, preserved because the DXBC is authoritative:

- `g_Flex_SolveVelocities` ends the sleep clamp with `mov r3.xyz, -r1.xxxx`,
  i.e. the delta is `-velocity.x` broadcast to all three components rather than
  `-velocity`. `SolveVelocities.hlsl` reproduces this with `-velocity.xxx`.
- `g_Flex_SolveVelocities` scales the diffuse potential by
  `1 - 2 * kInvRestDensity * density` (emitted as a `dp2` of two equal pairs),
  not `1 - kInvRestDensity * density`. Both `SolveVelocities` quirks carry an
  in-source comment naming the DXBC instruction they come from, so they are not
  "simplified" away by a later reader.
- `g_Flex_ClampDiffuseParticleCount` clamps through `float` (`itof` / `min` /
  `ftoi`) rather than an integer `imin`.
- `g_Flex_CollideShapes` orders the shape dispatch box, convex mesh, sphere,
  capsule, SDF, and calls the contact store inside each branch.
- `g_Flex_ClearInt` and `g_Flex_ClearFloat4` compare the thread index against
  the raw `gLength` constant; neither scales it. `Library::ClearBufferFloat4`
  passes an element count, which matches. `Library::ClearBufferInt` passes
  `sizeInBytes` while dispatching only `sizeInBytes / 4` threads, so its bound is
  four times larger than the buffer and never binds — harmless, but the C++ field
  name `lengthInWords` does not describe what either shader reads.
- `g_Flex_UpdateTriangles` averages the three vertex velocities with a literal
  `0.3333`, not `1.0 / 3.0`.

| Family | DXBC shader group | `src/shaders` source | Status |
| --- | --- | --- | --- |
| Flex | `g_Flex_ApplyDeltas` | `ApplyDeltas.hlsl` | Exact |
| Flex | `g_Flex_CalculateAnisotropy` | `CalculateAnisotropy.hlsl` | Exact |
| Flex | `g_Flex_CalculateBounds` | `CalculateBounds.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsAMD` | `CalculateBounds.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsFinalize` | `CalculateBoundsFinalize.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsFinalizeAMD` | `CalculateBoundsFinalize.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsFinalizeNV` | `CalculateBoundsFinalize.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsGroup` | `CalculateBoundsGroup.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsGroupAMD` | `CalculateBoundsGroup.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsGroupNV` | `CalculateBoundsGroup.hlsl` | Exact |
| Flex | `g_Flex_CalculateBoundsNV` | `CalculateBounds.hlsl` | Exact |
| Flex | `g_Flex_CalculateDensity` | `CalculateDensity.hlsl` | Exact |
| Flex | `g_Flex_CalculateDensitySurfaceTension` | `CalculateDensitySurfaceTension.hlsl` | Exact |
| Flex | `g_Flex_CalculateInflatableVolume` | `CalculateInflatableVolume.hlsl` | Exact |
| Flex | `g_Flex_CalculateInflatableVolumeAMD` | `CalculateInflatableVolume.hlsl` | Exact |
| Flex | `g_Flex_CalculateInflatableVolumeNV` | `CalculateInflatableVolume.hlsl` | Exact |
| Flex | `g_Flex_CalculateParticleHash` | `CalculateParticleHash.hlsl` | Exact |
| Flex | `g_Flex_CalculateVorticity` | `CalculateVorticity.hlsl` | Exact |
| Flex | `g_Flex_ClampDiffuseParticleCount` | `ClampDiffuseParticleCount.hlsl` | Exact |
| Flex | `g_Flex_ClearCellBuckets` | `ClearCellBuckets.hlsl` | Exact |
| Flex | `g_Flex_ClearFloat4` | `ClearFloat4.hlsl` | Exact |
| Flex | `g_Flex_ClearInt` | `ClearInt.hlsl` | Exact |
| Flex | `g_Flex_CollideParticles` | `CollideParticles.hlsl` | Verified |
| Flex | `g_Flex_CollideShapes` | `CollideShapes.hlsl` | Verified |
| Flex | `g_Flex_CollideTriangles` | `CollideTriangles.hlsl` | Exact |
| Flex | `g_Flex_CompactDiffuseParticles` | `CompactDiffuseParticles.hlsl` | Exact |
| Flex | `g_Flex_ComputeTriangleBounds` | `ComputeTriangleBounds.hlsl` | Exact |
| Flex | `g_Flex_ContinuousShockPropagation` | `ContinuousShockPropagation.hlsl` | Exact |
| Flex | `g_Flex_CreateDiffuseParticles` | `CreateDiffuseParticles.hlsl` | Exact |
| Flex | `g_Flex_CreateGrid` | `CreateGrid.hlsl` | Exact |
| Flex | `g_Flex_Finalize` | `Finalize.hlsl` | Exact |
| Flex | `g_Flex_NormalizeVertexNormals` | `NormalizeVertexNormals.hlsl` | Exact |
| Flex | `g_Flex_Predict` | `Predict.hlsl` | Exact |
| Flex | `g_Flex_ReorderParticles` | `ReorderParticles.hlsl` | Exact |
| Flex | `g_Flex_SmoothPositions` | `SmoothPositions.hlsl` | Exact |
| Flex | `g_Flex_SolveContactsAccumulate` | `SolveContactsAccumulate.hlsl` | Exact |
| Flex | `g_Flex_SolveContactsAveraged` | `SolveContactsAveraged.hlsl` | Exact |
| Flex | `g_Flex_SolveContactsSequential` | `SolveContactsSequential.hlsl` | Exact |
| Flex | `g_Flex_SolveDensities` | `SolveDensities.hlsl` | Exact |
| Flex | `g_Flex_SolveDensitiesNonFluid` | `SolveDensitiesNonFluid.hlsl` | Exact |
| Flex | `g_Flex_SolveDensitiesSurfaceTension` | `SolveDensitiesSurfaceTension.hlsl` | Exact |
| Flex | `g_Flex_SolveInflatableVolume` | `SolveInflatableVolume.hlsl` | Exact |
| Flex | `g_Flex_SolveInflatableVolumeNV` | `SolveInflatableVolume.hlsl` | Exact |
| Flex | `g_Flex_SolveShapes` | `SolveShapes.hlsl` | Exact |
| Flex | `g_Flex_SolveShapes128NV` | `SolveShapes128NV.hlsl` | Exact |
| Flex | `g_Flex_SolveShapes32NV` | `SolveShapes32NV.hlsl` | Exact |
| Flex | `g_Flex_SolveShapesNV` | `SolveShapesNV.hlsl` | Exact |
| Flex | `g_Flex_SolveShapesPlasticDeformation` | `SolveShapesPlasticDeformation.hlsl` | Exact |
| Flex | `g_Flex_SolveShapesPlasticDeformation128NV` | `SolveShapesPlasticDeformation128NV.hlsl` | Exact |
| Flex | `g_Flex_SolveShapesPlasticDeformation32NV` | `SolveShapesPlasticDeformation32NV.hlsl` | Exact |
| Flex | `g_Flex_SolveShapesPlasticDeformationNV` | `SolveShapesPlasticDeformationNV.hlsl` | Exact |
| Flex | `g_Flex_SolveSprings` | `SolveSprings.hlsl` | Exact |
| Flex | `g_Flex_SolveSpringsNV` | `SolveSprings.hlsl` | Exact |
| Flex | `g_Flex_SolveVelocities` | `SolveVelocities.hlsl` | Exact |
| Flex | `g_Flex_SpringsGenerateIndices` | `SpringsGenerateIndices.hlsl` | Exact |
| Flex | `g_Flex_SpringsParticleRange` | `SpringsParticleRange.hlsl` | Exact |
| Flex | `g_Flex_SpringsReorder` | `SpringsReorder.hlsl` | Exact |
| Flex | `g_Flex_TransformShapeBounds` | `TransformShapeBounds.hlsl` | Verified |
| Flex | `g_Flex_UpdateDiffuseParticles` | `UpdateDiffuseParticles.hlsl` | Exact |
| Flex | `g_Flex_UpdateTriangles` | `UpdateTriangles.hlsl` | Verified |
| Flex | `g_Flex_UpdateTrianglesInit` | `UpdateTrianglesInit.hlsl` | Exact |
| Flex | `g_Flex_UpdateTrianglesNV` | `UpdateTriangles.hlsl` | Verified |
| Flex | `g_Flex_UpdateVelocities` | `UpdateVelocities.hlsl` | Exact |
| Flex | `g_Flex_UpdateVertexNormals` | `UpdateVertexNormals.hlsl` | Exact |
| Flex | `g_Flex_UpdateVertexNormalsInit` | `UpdateVertexNormalsInit.hlsl` | Exact |
| Flex | `g_Flex_UpdateVertexNormalsNV` | `UpdateVertexNormals.hlsl` | Exact |
| Radix sort | `g_RadixSort1CS` | `radixSort1CS.hlsl` | Exact |
| Radix sort | `g_RadixSort2CS` | `radixSort2CS.hlsl` | Exact |
| Radix sort | `g_RadixSort3CS` | `radixSort3CS.hlsl` | Exact |
| Radix sort | `g_RadixSortBlockCS` | `radixSortBlockCS.hlsl` | Exact |
| BVH | `g_bvh_BuildHierarchy` | `bvh/BuildHierarchy.hlsl` | Exact |
| BVH | `g_bvh_BuildLeaves` | `bvh/BuildLeaves.hlsl` | Exact |
| BVH | `g_bvh_CalculateKeyDeltas` | `bvh/CalculateKeyDeltas.hlsl` | Exact |
| BVH | `g_bvh_CalculateMortonCodes` | `bvh/CalculateMortonCodes.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBounds` | `bvh/ComputeTotalBounds.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsAMD` | `bvh/ComputeTotalBounds.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsFinalize` | `bvh/ComputeTotalBoundsFinalize.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsFinalizeAMD` | `bvh/ComputeTotalBoundsFinalize.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsFinalizeNV` | `bvh/ComputeTotalBoundsFinalize.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsGroup` | `bvh/ComputeTotalBoundsGroup.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsGroupAMD` | `bvh/ComputeTotalBoundsGroup.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsGroupNV` | `bvh/ComputeTotalBoundsGroup.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalBoundsNV` | `bvh/ComputeTotalBounds.hlsl` | Exact |
| BVH | `g_bvh_ComputeTotalInvEdges` | `bvh/ComputeTotalInvEdges.hlsl` | Exact |

Shared shader support files:

| File | Role | Status |
| --- | --- | --- |
| `KernelParams.hlsli` | Shared Flex kernel parameter declarations. | Support |
| `Utils.hlsli` | Shared shader helper code. | Support |
| `bvh/BVHCommon.hlsli` | Shared BVH shader helper code. | Support |
| `Shaders.cfg` | Build manifest for recovered shader compilation; every recovered source is listed here exactly once. | Support |
