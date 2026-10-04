# Porting record: FleX kernels to ethereal-phys

The kernels in this directory are the compute shaders of the FleX solver (`reference/flex/src/shaders`,
FleX branch `feat-xpbd`, commit `2eed78e`), moved from `cs_5_0`/DXBC to shader model 6.0 and compiled to
DXIL (D3D12), SPIR-V (Vulkan) and DXBC (D3D11). Every ported file starts with a comment that names its
origin. This file records what was changed. `AGENTS.md` is the FleX engineering log and explains why the
sources are spelled as they are.

A change is listed here when it is more than a copy: anything that alters what is compiled, even when the
arithmetic is the same.

## What compiled unchanged

Every baseline kernel of the FleX tree compiles under DXC at `cs_6_0`, for DXIL and for SPIR-V, without a
source change: FleX had already kept them DXC-clean as a syntax guard. No FXC idiom had to be rewritten.
The arithmetic of the kernels is untouched, with the two exceptions below that do not change results
(`CalculateParticleHash`) or change them only where a vendor variant is replaced (the wave permutations).

## Files

| | Count |
| --- | --- |
| FleX sources (67 `.hlsl`, 3 `.hlsli`) | 70 |
| ported as they are (origin header added) | 52 |
| ported with a change listed below | 12 |
| not ported (`SolveShapes{,32,128}NV`, `SolveShapesPlasticDeformation{,32,128}NV`) | 6 |
| new (`KernelParamsLayout.hlsli`, `KernelParamsFields.hlsli`, `WaveReduce.hlsli`, `ProbeKernelParams.hlsl`) | 4 |
| sources in this directory | 68 |

Changed: `KernelParams.hlsli`, `Utils.hlsli`, `CalculateBounds.hlsl`, `bvh/ComputeTotalBounds.hlsl`,
`CalculateInflatableVolume.hlsl`, `SolveShapes.hlsl`, `SolveShapesPlasticDeformation.hlsl`,
`SolveSpringsXPBD.hlsl`, `CalculateParticleHash.hlsl`, `SolveInflatableVolume.hlsl`, `UpdateTriangles.hlsl`,
`UpdateVertexNormals.hlsl`. The last three only lost an `#if USE_NV_SHADER_EXT` block that the baseline
never compiled.

## The manifest

| | FleX | ethereal-phys |
| --- | --- | --- |
| manifest rows | 82 (`Shaders.cfg`) + 6 (`ShadersXPBD.cfg`) = 88 | 62 (`ethereal_phys.cfg`) |
| logical kernels (`ephys::KernelId`) | | 66 |
| compiled shaders per format | 88 | 77 |

The 62 rows are 53 kernels without a permutation axis, 5 with the wave axis, 3 with the wave and the XPBD
axis and the diagnostic `ProbeKernelParams` with the XPBD axis: 53 + 5·2 + 3·4 + 2 = 77 shaders. The 66
kernel ids are the 62 rows plus the XPBD twins of the four rows with that axis.

Where the 88 FleX entries went:

- The 24 vendor entries (`*NV`, `*AMD`, including the two XPBD ones) are replaced by the `EPHYS_WAVE=1`
  permutations of 8 rows (11 shaders), or dropped (below).
- The 3 other XPBD entries that renamed an entry point (`-DSolveShapes=SolveShapesXPBD ...`) are the
  `NVFLEX_XPBD=1` permutations of their PBD rows; the entry point keeps its PBD name and the kernel id says
  `XPBD`. `SolveSpringsXPBD` stays a row of its own.
- `-D<Entry>=<Entry>NV` style renames are gone: ShaderTool names a blob by source and entry point and keys
  the permutations, so a variant no longer needs its own entry point name.

## Vendor variants: one wave permutation

FleX selected `*NV` kernels (NVAPI: `nvHLSLExtns.h`, a fake UAV at `u7`) or `*AMD` kernels (AGS) per GPU
vendor. Neither is available at SM 6.0 on D3D12 or on Vulkan. They did two unrelated things:

1. **Wave shuffles** (`NvShflDown`, `AmdDxExtShaderIntrinsics_SwizzleF`) to reduce a value over a thread
   group in fewer barriers: `CalculateBounds`, `CalculateBoundsGroup`, `CalculateBoundsFinalize`,
   `bvh/ComputeTotalBounds`, `bvh/ComputeTotalBoundsGroup`, `CalculateInflatableVolume` (and its XPBD
   form), `SolveShapes*NV`, `SolveShapesPlasticDeformation*NV`.
2. **A float atomic** (`NvInterlockedAddFp32`) instead of the compare-exchange loop of `Utils.hlsli`:
   `SolveInflatableVolumeNV`, `UpdateTrianglesNV`, `UpdateVertexNormalsNV`, `SolveShapes*NV`.

What replaced them:

- **Reductions**: the `EPHYS_WAVE=1` permutation, written once in `WaveReduce.hlsli` with `WaveActiveSum` /
  `WaveActiveMin` / `WaveActiveMax`, `WaveGetLaneCount` and `WaveGetLaneIndex`. The kernel library selects
  it when the device reports `nvrhi::Feature::WaveLaneCountMinMax` (`KernelVariant::Default`); `EPHYS_WAVE=0`
  is FleX's groupshared tree, unchanged, and is the fallback.

  The vendor kernels assumed 32 lanes per wave and that threads `32k..32k+31` of a group form wave `k` in
  lane order. The wave permutation assumes neither: it is correct for any lane count and any assignment of
  threads to waves (the argument is the header comment of `WaveReduce.hlsli`), and falls back inside the
  shader, to a serial fold by thread 0, when the wave of thread 0 is not full or the lane count does not
  divide the group size. So there is nothing for the host to check against the device's lane counts, and
  no lane-count-specific kernel. Tested with 32 and 64 lanes, on D3D12 and Vulkan (`tests/kernel_test.cpp`).

  Min and max are exact, so the bounds kernels give bit-identical results in both permutations. Float sums
  are added in another order than the tree, and in an order that depends on the lane count, so they differ
  from `EPHYS_WAVE=0` in the last bits. FleX's vendor kernels differed from its baseline in the same way.

- **`SolveShapes32NV` / `128NV`** (and the plastic-deformation pair) were not lane-count variants: they are
  the same kernel with a thread group of 32 or 128 instead of 64, and FleX picked one by the average
  number of particles per rigid body. That is a tuning of the vendor path only (the baseline has one group
  size, 64). The wave permutation keeps the group size of the baseline. Not ported; say so if profiling of
  shape matching ever asks for it.

- **The float atomic** has no portable equivalent. `SolveInflatableVolumeNV`, `UpdateTrianglesNV` and
  `UpdateVertexNormalsNV` differed from their baselines in nothing else, so they have no second variant:
  every kernel uses the compare-exchange loop (`InterlockedAddFp32` in `Utils.hlsli`), as FleX did on
  every GPU that was not NVIDIA.

- **`SolveSpringsNV`** was the same source as `SolveSprings` under another entry point name. Dropped.

- **`CalculateBoundsFinalize{NV,AMD}`**, `ComputeTotalBoundsFinalize{NV,AMD}`: the first is a reduce level
  of `CalculateBounds.hlsl` and has the wave permutation; the second shipped as three identical blobs and
  was one manifest entry in FleX already.

`USE_NV_SHADER_EXT`, `USE_AMD_SHADER_EXT`, `NV_SHADER_EXTN_SLOT`, the `nvHLSLExtns.h` and
`ags_shader_intrinsics_dx11.hlsl` includes and the `u7` slot no longer appear in any source.

### A FleX behaviour the wave permutation reproduces on purpose

The groupshared tree of `CalculateInflatableVolume` starts at `numTrisInBlock >> 1` and halves. For a block
whose triangle count is not a power of two it never reaches some lanes (6 triangles: strides 3 and 1 fold
lanes 0, 1, 3, 4 and leave out 2 and 5), so the centre and the volume of an inflatable are sums over a
subset of its triangles whenever its last block of 512 holds a number of triangles that is not a power of
two. That is what the shipped kernel computes (the source is byte-exact against the shipped blob). FleX's vendor kernels
left out a *different* subset (they summed the first `numTrisInBlock / 32` waves).

The wave permutation sums exactly the lanes the tree covers (`TreeReduceCovers`), so that the two
permutations of this engine differ in summation order only. `tests/kernel_test.cpp` pins both: the kernels
must match the subset sum and must not match the sum over all triangles. Whether to fix the tree is a
solver decision (it changes results against the FleX reference build); it is not made here.

## `KernelParams`

- The struct moved to `KernelParamsLayout.hlsli`, which is valid HLSL and C++ and is included by
  `engine/src/kernel_params.h`; that header asserts the size and the offset of every member that follows
  padding or a vector. `tests/kernel_test.cpp` proves the HLSL side on the GPU with `ProbeKernelParams`.
- The `#if NVFLEX_XPBD` tail of the struct (`kSolverMode`, `kInvStiffnessMin`, `kLogStiffnessRange`,
  `kSpringDamping`, `kVolumeCompliance`, padding: registers 32 and 33) is gone. `KernelParams` is 512 bytes,
  32 registers, in every kernel: the layout FleX shipped.
- The four compliance fields are `KernelParamsXPBD`, a second constant buffer at `b1` (`gParamsXPBD`),
  declared only when `NVFLEX_XPBD` is set. The XPBD kernels read `gParamsXPBD.k...` where they read
  `gParams.k...`: `SolveSpringsXPBD.hlsl`, and the `NVFLEX_XPBD` branches of `CalculateInflatableVolume.hlsl`,
  `SolveShapes.hlsl` and `SolveShapesPlasticDeformation.hlsl`. Same arithmetic.
- `kSolverMode` is removed. No kernel ever read it; the host does not branch on a mode either, it dispatches
  another kernel id.
- `b1` therefore has two meanings, never both in one kernel: `SubstepParams` (`gSubParams`, read by
  `CollideShapes` and `CollideTriangles`) and `KernelParamsXPBD`. `SubstepParams` got two explicit padding
  floats, so that its C++ mirror is the 16 bytes of the register.

## Other changes

- **`CalculateParticleHash.hlsl`**: `RWBuffer<uint4> cellHash / cellIndices` are `RWBuffer<uint>`, and the
  stores write `hash` / `index` instead of `hash.xxxx` / `index.xxxx`. The host binds `R32_UINT` views of
  the sort buffers there, so only `.x` was ever stored. D3D accepts a four-component declaration on a
  one-component view; in SPIR-V the declaration fixes the image format (`Rgba32ui`), which must equal the
  format of the view. Same result on D3D; required for Vulkan.
- **`Utils.hlsli`**: `[allow_uav_condition]` is spelled `EPHYS_ALLOW_UAV_CONDITION`, which is the attribute
  except for SPIR-V, where DXC ignores it with a warning per use.
- **The kernels with a wave permutation** include `WaveReduce.hlsli` first; their `#if USE_NV_SHADER_EXT` /
  `#elif USE_AMD_SHADER_EXT` blocks are replaced by `#if EPHYS_USE_WAVE` blocks. In `SolveShapes.hlsl` and
  `SolveShapesPlasticDeformation.hlsl` the baseline `ReduceSum` and its `gReduce` array are now inside
  `#if !EPHYS_USE_WAVE` / `#else`; the text of the baseline is unchanged.

## Build

- `ethereal_compile_shaders(... SHADER_MODEL 6_0 BACKENDS DXBC DXIL SPIRV EXTRA_FLAGS_SPIRV
  -fvk-use-dx-layout)` in `engine/CMakeLists.txt`.
- **`-fvk-use-dx-layout`** is required, not cosmetic: DXC's default SPIR-V layout gives a
  `StructuredBuffer<float3>` a 16-byte stride, D3D gives it 12, and the host fills such buffers
  (`bounds`, `velocities`, `localPositions`, `translations`, ...) with 12-byte elements on every backend.
  The Vulkan device therefore needs the `scalarBlockLayout` feature (Donut's device manager enables it).
- Register shifts for SPIR-V are ShaderTool's defaults (t +0, s +128, b +256, u +384), which are the
  defaults of `nvrhi::VulkanBindingOffsets`.
- **D3D11** works: the sources still compile under `D3DCompile` at `cs_5_0`. The only guard needed is in
  `WaveReduce.hlsli` (`EPHYS_WAVE && !TARGET_D3D11`): in the DXBC blobs both wave permutations are the
  baseline, and the library never selects the wave one on D3D11 anyway.
- Known compiler warning, left alone: `CollideShapes.hlsl`, `BoxContact`: "parameter 'planeW' is
  uninitialized when used here". The miss path deliberately leaves the out parameters untouched (see the
  comment in the source); the caller does not read them then.

## Bindings

One binding layout for all kernels is not possible, because one register is a different kind of view in
different kernels. `python gen_kernel_table.py --audit` prints the evidence:

```
t0   RawSRV x1, StructuredSRV x40, TypedSRV(R32_UINT) x5, TypedSRV(RGBA32_UINT) x4
t1   RawSRV x1, StructuredSRV x43, TypedSRV(RGBA32_UINT) x2
t2   StructuredSRV x40, TypedSRV(R32_UINT) x4
t3   RawSRV x1, StructuredSRV x32
t4   StructuredSRV x15, TypedSRV(R32_UINT) x4
t15  StructuredSRV x1, TextureSRV x1          (t16, t17 alike; t18..t30 TextureSRV only)
u0   RawUAV x14, StructuredUAV x43, TypedUAV(R32_UINT) x5, TypedUAV(RGBA32_UINT) x2
u1   RawUAV x1, StructuredUAV x33, TypedUAV(R32_UINT) x3, TypedUAV(RGBA32_UINT) x1
```

An nvrhi binding layout fixes the resource type of each slot (and on Vulkan the descriptor type: storage
buffer, texel buffer, sampled image), so each kernel gets the layout its declarations ask for. They come
from a table, `engine/src/kernel_table.inl`, generated from the manifest and the sources by
`gen_kernel_table.py` (62 distinct layouts for 66 kernels). The typed buffers cannot be turned into
structured ones to shrink the audit: the radix sort reads one buffer through `R32_UINT` and `RGBA32_UINT`
views (four keys per load).

The table also holds, per binding, what D3D11 checks against the shader and the other backends silently
get wrong: the element size of a structured buffer (the library refuses a buffer with another struct
stride) and the size of a constant buffer (the library binds a buffer of at least that size and refuses a
larger block).

A declared slot the caller leaves null gets a dummy: one buffer per view kind, per element size for
structured views and per slot for UAVs (D3D11 does not take one buffer in two UAV slots, and compares
strides), a 1x1x1 volume for the texture slots.

After changing a resource declaration or a manifest row, run `python gen_kernel_table.py`; the
`ethereal_phys_kernel_table` test fails until the generated file and the `KernelId` enum match.
