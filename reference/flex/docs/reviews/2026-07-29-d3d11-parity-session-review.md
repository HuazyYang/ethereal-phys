# D3D11 FleX Parity Session Review

Date: 2026-07-29  
Scope: D3D11, public FleX runtime (`--dev=0`) versus reconstructed runtime
(`--dev=1`)

## Review scope

This document records the modification procedure performed during this chat
session for files that already existed in the repository.

The following are intentionally excluded from the modification inventory:

- newly created test scripts, plans, specifications, and this review document;
- generated shader headers, build outputs, logs, and playback buffers;
- the user's change to `demo/scenes/potpourri.h`;
- unrelated modifications that were already present in the dirty worktree;
- the temporary `RadixSort.cpp` experiment, because it was reverted and left no
  intended code change.

## Procedure

1. Added deterministic scene selection and playback diagnostics to the existing
   demo entry point.
2. Generated particle-position oracles with the public runtime and replayed them
   with the reconstructed runtime.
3. Reproduced the default-scene collision divergence and expanded verification
   to all 67 demo scenes.
4. Compared `src/Solver.cpp` dispatch order, buffer slots, dispatch sizes,
   callback timing, and capability gates against
   `src/_pseudo/NvFlexDebugD3D_x64.cpp`.
5. Applied one recovered behavior at a time, rebuilt `NvFlexRev.dll`, and ran
   targeted playback checks before the next change.
6. Investigated the fluid shader family against the recovered DXBC assembly.
   This established that the D3D11 reconstructed runtime already loads the
   recovered `src/dxbc/*.txt` shader blobs, so the remaining discrepancies were
   redirected to CPU parameter generation and C++ orchestration.
7. Located the shared fluid error in the SPH rest-density packing procedure,
   corrected it, and verified representative fluid scenes.
8. Ran a complete short playback sweep over frames `[0,200)`.
9. Extended the verification procedure to support a nonzero start frame and ran
   a complete long sweep over frames `[1,600)`.

## Existing-file modification record

### `demo/main.cpp`

Purpose: make public/reconstructed comparisons reproducible and diagnosable.

Changes made during the session:

- added `--scene=N` parsing and rejected scene indices outside `0..66`;
- added `--playback-mode=<none|read|write>`;
- added `--playback-range=start,end`;
- stored the playback range in the oracle header and loaded it in read mode;
- changed playback to operate on physical frame indices and stop at the
  requested exclusive end frame;
- printed the first mismatching particle index and both four-component
  positions;
- printed and flushed
  `[Playback] completed frame range [start,end)` when a range completed;
- retained runtime backend selection through `--dev=<0|1>` and the unified
  runtime loader.

These changes supplied the evidence used for every parity result in this
review. They do not alter solver equations.

### `src/Solver.cpp`

Purpose: match the recovered D3D11 solver orchestration.

Changes, in the order they were established:

1. **Bounds reduction input**

   `ComputeBounds()` now binds `mBoundsLower` to the lower-bound input slot of
   the group reduction. It previously bound `mBoundsUpper` to both lower and
   upper slots.

2. **Spring dispatch extent**

   `SolveSprings()` now calculates its dispatch from `8 * mMaxParticles`, as in
   the recovered pseudocode, instead of `8 * mNumParticles`.

3. **Rigid optimized-kernel gate**

   The nonplastic optimized shape path now requires both
   `mIsSHFLSupported` and `mIsFP32ATOMICSupported`. The previous condition
   checked the FP32-atomic flag twice.

4. **Static-contact count reset**

   `UpdateSubstep()` clears `mStaticContactCounts` after the substep-begin
   callback and before triangle/shape collision. This mirrors the public
   runtime's UAV clear at that point in the update.

5. **Solver callbacks**

   Re-enabled `ExecuteCallback()`. For the D3D backend, the callback parameter
   fields carry GPU `NvFlexBuffer` handles through the legacy pointer-typed ABI;
   the obsolete disabled implementation attempted CPU download/map operations
   that are not part of the current reconstructed context API.

6. **Update-end callback time step**

   The final-substep `eNvFlexStageUpdateEnd` callback now receives the complete
   update `dt`, equivalent to the recovered `substepDt * numSubsteps`, rather
   than one substep's `dta`.

7. **Diffuse velocity ping-pong**

   Corrected the diffuse swap from
   `mDiffuseVelocities <-> mDiffusePositionsNew` to
   `mDiffuseVelocities <-> mDiffuseVelocitiesNew`.

8. **SDF texture binding**

   `CollideShapes()` now binds SDF textures as read-only shader resources in
   `readOnly[15..30]`. They were incorrectly supplied as writable resources.

Observed effects:

- the spring-dispatch correction made Soft Rope pass the short sweep;
- the callback and update-end `dt` correction made Force Field pass;
- the contact-count reset improved moving-shape collision coverage;
- several rigid, moving-shape, cloth, and game-mesh scenes still diverge and
  require further work.

### `src/SPH.cpp`

Purpose: correct CPU generation of the fluid rest-density constants.

`TightPack3D()` already computed an alternating half-separation X offset for
odd `(y + z)` rows, but discarded it when constructing the sample position.
The position now uses:

```cpp
x * separation + offset
```

This matches the recovered `TightPack3D` pseudocode. Because
`SPHCalculateRestDensity()` derives rest density, density constraint scale, and
surface constraint scale from these samples, the missing offset affected all
fluid kernels before the first simulated frame.

Observed effect:

- Rayleigh Taylor 2D, viscosity low/medium/high, adhesion, Goo Gun, buoyancy,
  surface tension low/medium/high, Fluid Block, both Fluid Cloth Coupling
  scenes, and Bunny Bath Dam passed the long `[1,600)` sweep.

### `src/Library.cpp`

Purpose: align shader selection and extension enablement with the public
runtime.

Changes recorded:

- selected the recovered collision shader variants already present in the
  reconstructed shader set for particle and shape collision;
- gated SHFL, FP32-atomic, and swizzle capability use on
  `NvFlexInitDesc::enableExtensions`;
- disabled NVIDIA extension paths when the library has no valid SM count,
  matching the recovered public initialization branch.

The capability change was included in the long verification build. It did not
eliminate the remaining rigid-family differences, so those paths must be
audited further before parity can be claimed.

### `core/` portability changes

Three existing `core/` files contain substantive changes:

- `core/core.h` removes inheritance from the C++17-removed
  `std::unary_function` base. `free_ptr` remains the same callable functor.
- `core/mat44.h` adds the explicit `XMatrix44<float>::kIdentity`
  specialization required by the migrated build/link configuration.
- `core/platform.cpp` removes `using namespace std` and explicitly qualifies
  `std::string`. This is source/toolchain hygiene and does not change file or
  path behavior.

### Demo build and runtime-loader changes

The following existing demo files were changed as part of the cross-platform
build/runtime-loader migration:

- `demo/CMakeLists.txt` includes C sources, adds the public include directory,
  builds one runtime-selectable demo instead of compile-time-selecting
  `NvFlexLib` or `NvFlexRev`, makes both runtimes build dependencies, and stages
  the runtime DLL dependency set beside the executable.
- `demo/helpers.h` includes `nvflex_runtime.h` and selects the public or
  reconstructed SDF upload path at runtime. The reconstructed backend uploads
  an `NvFlexTexture3D` and calls `NvFlexUpdateDistanceField2`; the public
  backend retains the buffer-based `NvFlexUpdateDistanceField` path.
- `demo/shaders.h` consumes `nvflex_runtime.h` instead of directly including
  the public `NvFlex.h`.
- `demo/d3d/loaderMacros.h` adds a two-stage macro expansion for parameter
  selection, allowing the loader declarations to preprocess correctly with
  the migrated compiler.

### Demo D3D type-safety changes

- `demo/d3d/renderParamsD3D.cpp` replaces reference-punning casts between
  `Matrix44`, `Vec*`, DirectXMath, and HLSL mirror types with explicit vector
  construction plus `XMLoadFloat4x4`/`XMStoreFloat4x4`.
- `demo/d3d11/demoContextD3D11.cpp` adds explicit `Vec3`, `Vec4`, `Colour`, and
  `Matrix44` conversion helpers and uses them for cloth, rope, plane, and fluid
  render parameters.
- `demo/d3d11/diffuseRenderD3D11.h` and
  `demo/d3d11/fluidRenderD3D11.h` remove redundant direct includes of the
  public-only `NvFlex.h`; the runtime facade supplies the selected API.
- `demo/d3d12/demoContextD3D12.cpp` makes the equivalent explicit colour/vector
  conversions. This is recorded as a build-port change only; the parity sweeps
  in this review were D3D11-only.

### Demo filename/link spelling changes

Case-exact spellings were applied for toolchains and filesystems that do not
accept the historical case mismatches:

- `demo/d3d11/appD3D11Ctx.cpp`: `DXGI.lib` to `dxgi.lib`;
- `demo/d3d11/meshRenderD3D11.cpp`: `appD3d11Ctx.h` to
  `appD3D11Ctx.h`;
- `demo/d3d12/NvCoDxDebugUtil.h`: `DXGIDebug.h` to `dxgidebug.h`;
- `demo/d3d12/appD3D12Ctx.cpp` and
  `demo/d3d12/demoContextD3D12.cpp`: `DXGI.lib` to `dxgi.lib`;
- `demo/d3d12/fluidEllipsoidRenderPipelineD3D12.h`,
  `demo/d3d12/fluidSmoothRenderPipelineD3D12.h`,
  `demo/d3d12/fluidThicknessRenderPipelineD3D12.cpp`, and
  `demo/d3d12/fluidThicknessRenderPipelineD3D12.h`: include names now match
  the checked-in lowercase-leading filenames.

### Core/demo audit boundary

The final audit found 36 tracked modified files under `core/` and 204 under
`demo/`. Ignoring end-of-line-only differences leaves 3 substantive `core/`
files and 19 substantive `demo/` files. One of those 19 is the user's
`demo/scenes/potpourri.h` change, so it is intentionally not attributed to this
session's modification procedure. The remaining 18 demo files are recorded
above or in the existing `demo/main.cpp` section.

The untracked `demo/cargs.c` and `demo/cargs.h` files are also intentionally
excluded because this review was requested to cover modifications to existing
files only. The other 33 `core/` and 185 `demo/` status entries are
end-of-line/whitespace-only differences and are not represented as code
modifications.

## Focused code diff views

These are review-oriented diffs of the functional changes described above.
They intentionally omit whitespace-only changes, line-ending changes, and
unrelated surrounding edits in the dirty worktree.

### `core/core.h` — C++17 functor compatibility

```diff
 template <typename T>
-class free_ptr : public std::unary_function<T*, void>
+class free_ptr
 {
```

### `core/mat44.h` — identity specialization

```diff
+template <>
+XMatrix44<float> XMatrix44<float>::kIdentity;
```

### `core/platform.cpp` — standard-library qualification

```diff
-using namespace std;
-
-string LoadFileToString(const char* filename)
+std::string LoadFileToString(const char* filename)
 ...
-string NormalizePath(const char* path)
+std::string NormalizePath(const char* path)
 {
-    string p(path);
+    std::string p(path);
```

The same `std::string` qualification was applied to `StripFilename`,
`GetExtension`, `StripExtension`, and `StripPath`.

### `demo/CMakeLists.txt` — unified runtime-loaded demo

```diff
 file(GLOB DemoAppD3D_SOURCES
-    "*.h" "*.cpp"
+    "*.h" "*.cpp" "*.c"
 ...
+    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
 )
```

```diff
-if(NVFLEX_USE_REVERSED_LIB)
-    target_link_libraries(DemoAppD3D PRIVATE
-        SDL2 SDL2main stb NvFlexRev NvFlexExt)
-    target_compile_definitions(DemoAppD3D PRIVATE
-        "NVFLEX_USE_REVERSED_LIB=1")
-else()
-    target_link_libraries(DemoAppD3D PRIVATE
-        SDL2 SDL2main stb NvFlexLib NvFlexExt)
-endif()
+target_link_libraries(DemoAppD3D PRIVATE
+    SDL2 SDL2main stb NvFlexLib NvFlexExt)
+add_dependencies(DemoAppD3D
+    NvFlexLib NvFlexRev DemoAppD3DShaders)
```

```diff
 COMMAND ${CMAKE_COMMAND} -E copy
     "$<TARGET_FILE:NvFlexLib>"
+    "$<TARGET_FILE:NvFlexRev>"
+    "$<TARGET_FILE:NvFlexExt>"
     "$<TARGET_FILE:SDL2>"
+    "$<TARGET_FILE:NVTX>"
+    "$<TARGET_FILE:AGS>"
+    "${CMAKE_SOURCE_DIR}/bin/win64/GFSDK_Aftermath_Lib.x64.dll"
     "$<TARGET_FILE_DIR:DemoAppD3D>"
```

### `demo/d3d/loaderMacros.h` — portable macro expansion

```diff
+#define LOADER_ESC_PARAM_SELECT_IMPL(numParams) LOADER_ESC_PARAM##numParams
+#define LOADER_ESC_PARAM_SELECT(numParams) \
+    LOADER_ESC_PARAM_SELECT_IMPL(numParams)
+#define LOADER_ESC_APPLY(macro, args) macro args
+
 #define LOADER_ESC(mode, numParams, params) \
-    LOADER_ESC_PARAM##numParams LOADER_ESC_MERGE(mode, LOADER_ESC_N params)
+    LOADER_ESC_APPLY(LOADER_ESC_PARAM_SELECT(numParams), \
+        LOADER_ESC_MERGE(mode, LOADER_ESC_N params))
```

### `demo/d3d/renderParamsD3D.cpp` — explicit matrix/vector conversion

```diff
+DirectX::XMMATRIX LoadMatrix(const Matrix44& matrix)
+{
+    return DirectX::XMLoadFloat4x4(
+        reinterpret_cast<const DirectX::XMFLOAT4X4*>(&matrix));
+}
+
+DirectX::XMFLOAT4X4 StoreMatrix(DirectX::FXMMATRIX matrix)
+{
+    DirectX::XMFLOAT4X4 result;
+    DirectX::XMStoreFloat4x4(&result, matrix);
+    return result;
+}
```

```diff
-constBuf.modelView =
-    (float4x4&)XMMatrixMultiply(params.model, params.view);
-constBuf.projection = (float4x4&)params.projection;
+constBuf.modelView =
+    StoreMatrix(XMMatrixMultiply(params.model, params.view));
+constBuf.projection = StoreMatrix(params.projection);
```

The same conversion pattern replaces the reference-punning casts in the mesh,
fluid, diffuse, composite, point, and shadow constant-buffer paths.

### `demo/d3d11/appD3D11Ctx.cpp` — library spelling

```diff
-#pragma comment (lib, "DXGI.lib")
+#pragma comment (lib, "dxgi.lib")
```

### `demo/d3d11/demoContextD3D11.cpp` — explicit render conversions

```diff
+float3 ToFloat3(const Vec3& value)
+{
+    return float3(value.x, value.y, value.z);
+}
+
+float4 ToFloat4(const Vec4& value)
+{
+    return float4(value.x, value.y, value.z, value.w);
+}
+
+float4 ToFloat4(const Colour& value)
+{
+    return float4(value.r, value.g, value.b, value.a);
+}
```

```diff
-params.invTexScale =
-    (float4&)Vec2(1.0f / screenWidth, screenAspect / screenWidth);
+params.invTexScale =
+    float4(1.0f / screenWidth, screenAspect / screenWidth, 0.0f, 0.0f);
 ...
-params.lightDir = (const float3&)-Normalize(lightTarget - lightPos);
+params.lightDir = ToFloat3(-Normalize(lightTarget - lightPos));
```

### D3D11 render headers — runtime facade ownership

`demo/d3d11/diffuseRenderD3D11.h`:

```diff
-#include "../include/NvFlex.h"
 #include "../d3d/renderParamsD3D.h"
```

`demo/d3d11/fluidRenderD3D11.h`:

```diff
-#include "../include/NvFlex.h"
 #include "../d3d/renderParamsD3D.h"
```

### `demo/d3d11/meshRenderD3D11.cpp` — case-exact include

```diff
-#include "appD3d11Ctx.h"
+#include "appD3D11Ctx.h"
```

### D3D12 source/header spelling — case-exact names

`demo/d3d12/NvCoDxDebugUtil.h`:

```diff
-#include <DXGIDebug.h>
+#include <dxgidebug.h>
```

`demo/d3d12/appD3D12Ctx.cpp` and
`demo/d3d12/demoContextD3D12.cpp`:

```diff
-#pragma comment (lib, "DXGI.lib")
+#pragma comment (lib, "dxgi.lib")
```

`demo/d3d12/fluidEllipsoidRenderPipelineD3D12.h` and
`demo/d3d12/fluidThicknessRenderPipelineD3D12.h`:

```diff
-#include "RenderStateD3D12.h"
+#include "renderStateD3D12.h"
```

`demo/d3d12/fluidSmoothRenderPipelineD3D12.h`:

```diff
-#include "RenderStateD3D12.h"
-#include "MeshRenderer.h"
-#include "FluidEllipsoidRenderPipelineD3D12.h"
+#include "renderStateD3D12.h"
+#include "meshRenderer.h"
+#include "fluidEllipsoidRenderPipelineD3D12.h"
```

`demo/d3d12/fluidThicknessRenderPipelineD3D12.cpp`:

```diff
-#include "FluidThicknessRenderPipelineD3D12.h"
+#include "fluidThicknessRenderPipelineD3D12.h"
```

### `demo/d3d12/demoContextD3D12.cpp` — explicit colour conversion

```diff
+Hlsl::float4 ToFloat4(const Colour& value)
+{
+    return Hlsl::float4(value.r, value.g, value.b, value.a);
+}
+
+Hlsl::float4 ToFloat4(const FlexVec4& value)
+{
+    return Hlsl::float4(value.x, value.y, value.z, value.w);
+}
 ...
-params.color = (Hlsl::float4&)(g_colors[colorIndex + 1] * 1.5f);
+params.color = ToFloat4(g_colors[colorIndex + 1] * 1.5f);
```

### `demo/helpers.h` — runtime-selected SDF upload

```diff
-#if NVFLEX_USE_REVERSED_LIB
-#include <nvflex/NvFlexContextExt.h>
-#endif
+#include <nvflex_runtime.h>
```

```diff
-#if NVFLEX_USE_REVERSED_LIB
-    // texture upload and NvFlexUpdateDistanceField2
-#else
-    // buffer upload and NvFlexUpdateDistanceField
-#endif
+NvFlexDistanceFieldId sdf;
+if (NvFlexRuntime::Get().GetBackend() ==
+        NvFlexRuntime::eBackendReversed) {
+    // upload NvFlexTexture3D
+    sdf = NvFlexCreateDistanceField(g_flexLib);
+    NvFlexUpdateDistanceField2(g_flexLib, sdf, sdfTex);
+} else {
+    NvFlexVector<float> field(g_flexLib);
+    field.assign(pfm.m_data,
+        pfm.m_width * pfm.m_height * pfm.m_depth);
+    field.unmap();
+    sdf = NvFlexCreateDistanceField(g_flexLib);
+    NvFlexUpdateDistanceField(
+        g_flexLib, sdf, dim, dim, dim, field.buffer);
+}
```

### `demo/shaders.h` — runtime facade include

```diff
-#include "../include/NvFlex.h"
+#include <nvflex_runtime.h>
```

### `demo/main.cpp` — runtime and argument support

```diff
+#include "cargs.h"
 #include "../core/types.h"
 ...
+#include <filesystem>
+
+#define NVFLEX_REV_EXPR(expr)                                            \
+    (NvFlexRuntime::Get().GetBackend() ==                                \
+             NvFlexRuntime::eBackendReversed ? (void)(expr) : (void)0)
+
+bool g_useFlexRev = false;
```

```diff
 static const cag_option options[] = {
+    {'R', NULL, "dev", "BOOL", "Run Flex reversed backend"},
     ...
+    {'S', NULL, "scene", "N", "Initial scene index"},
+    {'Y', NULL, "playback-mode", "MODE",
+     "Playback mode: none, read, write"},
+    {'Z', NULL, "playback-range", "start,end",
+     "Playback record physical frame range"},
 };
```

```diff
+case 'R':
+    if (value)
+        g_useFlexRev = atoi(value) != 0;
+    break;
+case 'S':
+    if (value)
+        g_scene = atoi(value);
+    break;
+case 'Y':
+    if (_stricmp(value, "none") == 0)
+        g_playbackCtx.setMode(PlaybackContext::None);
+    else if (_stricmp(value, "read") == 0)
+        g_playbackCtx.setMode(PlaybackContext::Read);
+    else if (_stricmp(value, "write") == 0)
+        g_playbackCtx.setMode(PlaybackContext::Write);
+    else
+        exit(-1);
+    break;
+case 'Z': {
+    uint32_t start, end;
+    if (sscanf(value, "%u,%u", &start, &end) == 2)
+        g_playbackCtx.setFrameRange(start, end);
+    else
+        exit(-1);
+    break;
+}
```

```diff
+if (g_scene < 0 || g_scene >= int(g_scenes.size()))
+{
+    fprintf(stderr, "--scene must be in the range 0,%d\n",
+            int(g_scenes.size()) - 1);
+    exit(-1);
+}
```

```diff
+std::string flexError;
+const std::filesystem::path flexRuntimePath =
+    std::filesystem::current_path();
+if (!NvFlexRuntime::Get().Load(
+        flexRuntimePath.wstring(),
+        g_useFlexRev ? NvFlexRuntime::eBackendReversed
+                     : NvFlexRuntime::eBackendPublic,
+        flexError))
+{
+    std::cerr << flexError << std::endl;
+    exit(-1);
+}
+
 g_flexLib = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);
```

The previous compile-time reversed-runtime guards at context reset, execution,
and wait points were replaced by the runtime check:

```diff
-#if NVFLEX_USE_REVERSED_LIB
-    NvFlexResetContext(g_flexLib, false);
-#endif
+NVFLEX_REV_EXPR(NvFlexResetContext(g_flexLib, false));

-#if NVFLEX_USE_REVERSED_LIB
-    NvFlexExecuteContext(g_flexLib);
-#endif
+NVFLEX_REV_EXPR(NvFlexExecuteContext(g_flexLib));

-#if NVFLEX_USE_REVERSED_LIB
-    NvFlexWaitContext(g_flexLib);
-#endif
+NVFLEX_REV_EXPR(NvFlexWaitContext(g_flexLib));
```

### `demo/main.cpp` — playback range and mismatch reporting

```diff
 struct PlaybackContext {
     Mode writeMode;
+    uint32_t frameStart, frameEnd;
     uint32_t frameIndex;
     FILE* archiveStream;
     std::vector<Vec4> positionsBak;
+
+    void setFrameRange(uint32_t start, uint32_t end) {
+        frameStart = start;
+        frameEnd = end;
+    }
 };
```

```diff
-frameIndex = 0;
+frameIndex = -1; // physical frame initial index

 archiveStream = fopen(nameBuff, writeMode == Write ? "wb" : "rb");
 assert(archiveStream);
+if (writeMode == Read) {
+    fread(&frameStart, sizeof(uint32_t), 1, archiveStream);
+    fread(&frameEnd, sizeof(uint32_t), 1, archiveStream);
+} else {
+    fwrite(&frameStart, sizeof(uint32_t), 1, archiveStream);
+    fwrite(&frameEnd, sizeof(uint32_t), 1, archiveStream);
+}
```

```diff
-++frameIndex;
-if (!isInSelectedFrameRange()) {
+uint32_t frameIndex = this->frameIndex++;
+if (frameIndex == -1 || frameIndex < frameStart)
+    return;
+else if (frameIndex >= frameEnd) {
     terminate();
     return;
 }
```

```diff
 if (!(component-wise position comparison)) {
+    printf(
+        "[Playback][Frame %u] particle %u "
+        "current=(%.9g, %.9g, %.9g, %.9g) "
+        "expect=(%.9g, %.9g, %.9g, %.9g)\n",
+        frameIndex, i,
+        pos0.x, pos0.y, pos0.z, pos0.w,
+        pos1.x, pos1.y, pos1.z, pos1.w);
     coincident = false;
     break;
 }
```

```diff
 if (archiveStream) {
     fclose(archiveStream);
     archiveStream = nullptr;
+    printf("[Playback] completed frame range [%u,%u)\n",
+           frameStart, frameEnd);
+    fflush(stdout);
 }
```

### `src/Solver.cpp` — bounds reduction

```diff
 params.readWrite[2] = NvFlexBufferGetResourceRW(mParticleBounds);
-params.readOnly[2] = NvFlexBufferGetResource(mBoundsUpper);
+params.readOnly[2] = NvFlexBufferGetResource(mBoundsLower);
 params.readOnly[3] = NvFlexBufferGetResource(mBoundsUpper);
```

### `src/Solver.cpp` — spring dispatch extent

```diff
-const NvFlexUint kNumParticleBlocks =
-    divCeil<512>(8 * mNumParticles);
+const NvFlexUint kNumParticleBlocks =
+    divCeil<512>(8 * mMaxParticles);
```

### `src/Solver.cpp` — optimized rigid-kernel gate

```diff
-} else if (mLib->mIsFP32ATOMICSupported &&
-           mLib->mIsFP32ATOMICSupported) {
+} else if (mLib->mIsSHFLSupported &&
+           mLib->mIsFP32ATOMICSupported) {
```

### `src/Solver.cpp` — static-contact reset

```diff
 CollideParticles(state);
 ExecuteCallback(eNvFlexStageSubstepBegin, state.dta);
+mLib->ClearBufferInt(
+    mStaticContactCounts, sizeof(int) * mMaxParticles, 0);
 CollideTriangles(state);
 CollideShapes(state);
```

### `src/Solver.cpp` — callback ABI and update-end time step

```diff
 void Solver::ExecuteCallback(
     NvFlexSolverCallbackStage stage, float dt) {
-#if 0
     if (stage >= 0 && stage < eNvFlexStageCount &&
         mCallbacks[stage].function) {
-        NvFlexBufferDownload(...);
-        params.particles =
-            (float*)NvFlexBufferMapDownload(...);
-        params.velocities =
-            (float*)NvFlexBufferMapDownload(...);
-        params.phases =
-            (int*)NvFlexBufferMapDownload(...);
+        params.particles = reinterpret_cast<float*>(
+            static_cast<NvFlexBuffer*>(mSortedNewPositions));
+        params.velocities = reinterpret_cast<float*>(
+            static_cast<NvFlexBuffer*>(mSortedNewVelocities));
+        params.phases = reinterpret_cast<int*>(
+            static_cast<NvFlexBuffer*>(mSortedPhases));
         params.numActive = mNumParticles;
-        params.sortedToOriginalMap =
-            (const int*)NvFlexBufferMapDownload(...);
-        params.originalToSortedMap =
-            (const int*)NvFlexBufferMapDownload(...);
+        params.sortedToOriginalMap =
+            reinterpret_cast<const int*>(sortedCellHash.key);
+        params.originalToSortedMap =
+            reinterpret_cast<const int*>(sortedCellHash.val);
         params.userData = cb->userData;
         params.dt = dt;
         cb->function(params);
-        NvFlexBufferUnmapDownload(...);
     }
-#endif
 }
```

```diff
 if (state.substepIdx == state.numSubsteps - 1)
-    ExecuteCallback(eNvFlexStageUpdateEnd, state.dta);
+    ExecuteCallback(eNvFlexStageUpdateEnd, state.dt);
```

### `src/Solver.cpp` — diffuse buffer ping-pong

```diff
 swap(mDiffusePositions, mDiffusePositionsNew);
-swap(mDiffuseVelocities, mDiffusePositionsNew);
+swap(mDiffuseVelocities, mDiffuseVelocitiesNew);
 swap(mNumDiffuseParticles, mNumDiffuseParticlesNew);
```

### `src/Solver.cpp` — SDF texture binding

```diff
 if (mLib->mSDFData->mTextures[i]) {
-    params.readWrite[i + 15] =
-        NvFlexTexture3DGetResourceRW(
-            mLib->mSDFData->mTextures[i]);
+    params.readOnly[i + 15] =
+        NvFlexTexture3DGetResource(
+            mLib->mSDFData->mTextures[i]);
 }
```

### `src/SPH.cpp` — staggered tight packing

```diff
-auto pos = make_float3(
-    x * separation, ypos, zpos);
+auto pos = make_float3(
+    x * separation + offset, ypos, zpos);
```

### `src/Library.cpp` — recovered collision shader variants

```diff
-#include "CollideParticles.hlsl.h"
-#include "CollideShapes.hlsl.h"
+#include "CollideParticles.hlsl_rev.h"
+#include "CollideShapes.hlsl_rev.h"
```

### `src/Library.cpp` — extension capability gate

```diff
-mIsSHFLSupported =
-    devCapabilities.isSHFLSupported;
-mIsFP32ATOMICSupported =
-    devCapabilities.isFp32AtomicSupported;
-mIsSwizzleSupported =
-    devCapabilities.isSwizzleSupported;
 mSMCount = -1;
+const bool enableExtensions =
+    desc->enableExtensions &&
+    !(mGpuVendorId == VENDOR_ID_NVIDIA && mSMCount == -1);
+mIsSHFLSupported =
+    enableExtensions && devCapabilities.isSHFLSupported;
+mIsFP32ATOMICSupported =
+    enableExtensions && devCapabilities.isFp32AtomicSupported;
+mIsSwizzleSupported =
+    enableExtensions && devCapabilities.isSwizzleSupported;
```

## Verification record

### Build

The D3D11 reconstructed runtime was rebuilt with:

```bash
cmake --build /tmp/nvflex-wine-clang --config Debug --target NvFlexRev
```

The build completed successfully. Existing warnings in
`src/private/ResourceWrapper.h` remained.

### Targeted checks

- scene 50, Rayleigh Taylor 2D, matched all frames in `[0,200)` after the SPH
  correction;
- scene 45, Surface Tension High, matched all frames in `[0,200)`;
- Force Field matched `[0,200)` after callback timing was corrected.

### Complete long sweep

Configuration:

```text
backend: D3D11
oracle:  --dev=0 --playback-mode=write --playback-range=1,600
check:   --dev=1 --playback-mode=read
tolerance: 1e-3 per position component
scenes: 0..66
```

Result:

```text
PASS 36
FAIL 31
```

The result table is stored outside the repository at:

```text
/tmp/nvflex-all-scenes-parity-1-600/results.tsv
```

The long sweep is evidence that the session improved major shared paths, but it
is not evidence of complete solver parity.

### Full scene table

| Scene | Name | Status | Mismatch frames | First mismatch frame |
|---:|---|:---:|---:|---:|
| 0 | Pot Pourri | FAIL | 372 | 228 |
| 1 | Soft Octopus | FAIL | 550 | 29 |
| 2 | Soft Teapot | PASS | 0 | — |
| 3 | Soft Rope | PASS | 0 | — |
| 4 | Soft Cloth | PASS | 0 | — |
| 5 | Soft Bowl | PASS | 0 | — |
| 6 | Soft Rod | FAIL | 293 | 307 |
| 7 | Soft Armadillo | FAIL | 131 | 465 |
| 8 | Soft Bunny | FAIL | 106 | 32 |
| 9 | Plastic Bunnies | PASS | 0 | — |
| 10 | Plastic Comparison | PASS | 0 | — |
| 11 | Plastic Stack | PASS | 0 | — |
| 12 | Friction Ramp | PASS | 0 | — |
| 13 | Friction Moving Box | PASS | 0 | — |
| 14 | Friction Moving Sphere | FAIL | 556 | 41 |
| 15 | Friction Moving Capsule | FAIL | 369 | 71 |
| 16 | Friction Moving Mesh | FAIL | 599 | 1 |
| 17 | Shape Collision | FAIL | 574 | 26 |
| 18 | Shape Channels | PASS | 0 | — |
| 19 | Triangle Collision | PASS | 0 | — |
| 20 | Local Space Fluid | FAIL | 599 | 1 |
| 21 | Local Space Cloth | PASS | 0 | — |
| 22 | World Space Fluid | FAIL | 599 | 1 |
| 23 | Env Cloth Small | FAIL | 553 | 42 |
| 24 | Env Cloth Large | FAIL | 557 | 43 |
| 25 | Flag Cloth | FAIL | 546 | 54 |
| 26 | Inflatables | FAIL | 540 | 56 |
| 27 | Cloth Layers | FAIL | 599 | 1 |
| 28 | Sphere Cloth | FAIL | 553 | 45 |
| 29 | Tearing | PASS | 0 | — |
| 30 | Pasta | PASS | 0 | — |
| 31 | Game Mesh Rigid | FAIL | 599 | 1 |
| 32 | Game Mesh Particles | FAIL | 558 | 42 |
| 33 | Game Mesh Fluid | FAIL | 580 | 20 |
| 34 | Game Mesh Cloth | FAIL | 599 | 1 |
| 35 | Rigid Debris | PASS | 0 | — |
| 36 | Viscosity Low | PASS | 0 | — |
| 37 | Viscosity Med | PASS | 0 | — |
| 38 | Viscosity High | PASS | 0 | — |
| 39 | Adhesion | PASS | 0 | — |
| 40 | Goo Gun | PASS | 0 | — |
| 41 | Buoyancy | PASS | 0 | — |
| 42 | Melting | FAIL | 578 | 18 |
| 43 | Surface Tension Low | PASS | 0 | — |
| 44 | Surface Tension Med | PASS | 0 | — |
| 45 | Surface Tension High | PASS | 0 | — |
| 46 | DamBreak 5cm | FAIL | 589 | 11 |
| 47 | DamBreak 10cm | FAIL | 583 | 17 |
| 48 | DamBreak 15cm | FAIL | 581 | 19 |
| 49 | Rock Pool | FAIL | 588 | 12 |
| 50 | Rayleigh Taylor 2D | PASS | 0 | — |
| 51 | Trigger Volume | PASS | 0 | — |
| 52 | Force Field | PASS | 0 | — |
| 53 | Initial Overlap | PASS | 0 | — |
| 54 | Rigid2 | FAIL | 504 | 96 |
| 55 | Rigid4 | FAIL | 414 | 186 |
| 56 | Rigid8 | FAIL | 335 | 265 |
| 57 | Bananas | PASS | 0 | — |
| 58 | Low Dimensional Shapes | PASS | 0 | — |
| 59 | Granular Pile | PASS | 0 | — |
| 60 | Parachuting Bunnies | FAIL | 496 | 104 |
| 61 | Water Balloons | FAIL | 598 | 2 |
| 62 | Rigid Fluid Coupling | PASS | 0 | — |
| 63 | Fluid Block | PASS | 0 | — |
| 64 | Fluid Cloth Coupling Water | PASS | 0 | — |
| 65 | Fluid Cloth Coupling Goo | PASS | 0 | — |
| 66 | Bunny Bath Dam | PASS | 0 | — |

### Failing-scene logs

Each entry records the first mismatching particle emitted by the playback
checker. The referenced read logs contain every mismatching frame for that
scene.

- **Scene 0 — Pot Pourri** (372 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/00-read.log`  
  `[Playback][Frame 228] particle 864 current=(5.84160042, 0.464320183, 1.70122731, 1) expect=(5.84218884, 0.464233488, 1.70248246, 1)`
- **Scene 1 — Soft Octopus** (550 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/01-read.log`  
  `[Playback][Frame 29] particle 279 current=(1.49079716, 0.273607135, 2.17747331, 1) expect=(1.48902559, 0.270550787, 2.17699099, 1)`
- **Scene 6 — Soft Rod** (293 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/06-read.log`  
  `[Playback][Frame 307] particle 168 current=(0.56626749, 0.445557833, -0.187205732, 1) expect=(0.56620574, 0.44544819, -0.186190218, 1)`
- **Scene 7 — Soft Armadillo** (131 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/07-read.log`  
  `[Playback][Frame 465] particle 485 current=(1.26553547, 1.13384628, 1.59285378, 1) expect=(1.26655447, 1.13370287, 1.59272718, 1)`
- **Scene 8 — Soft Bunny** (106 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/08-read.log`  
  `[Playback][Frame 32] particle 622 current=(1.17328393, 0.656715214, 1.04060996, 1) expect=(1.17303622, 0.657739997, 1.04059839, 1)`
- **Scene 14 — Friction Moving Sphere** (556 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/14-read.log`  
  `[Playback][Frame 41] particle 1574 current=(-0.18332471, 0.423824281, 0.496691614, 1) expect=(-0.182945013, 0.424030155, 0.495081127, 1)`
- **Scene 15 — Friction Moving Capsule** (369 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/15-read.log`  
  `[Playback][Frame 71] particle 2779 current=(-0.0264555681, 0.378501058, 0.151720643, 1) expect=(-0.0265020598, 0.378751218, 0.150501311, 1)`
- **Scene 16 — Friction Moving Mesh** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/16-read.log`  
  `[Playback][Frame 1] particle 0 current=(3.45158121e+15, 5.99405622, 1.55930644e+13, 1) expect=(-0.800215185, 1.49322987, -0.800167918, 1)`
- **Scene 17 — Shape Collision** (574 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/17-read.log`  
  `[Playback][Frame 26] particle 28510 current=(0.949271739, 2.09450674, 0.501972139, 1) expect=(0.952004254, 2.09603691, 0.500379622, 1)`
- **Scene 20 — Local Space Fluid** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/20-read.log`  
  `[Playback][Frame 1] particle 35887 current=(-1.17123222, 1.02346861, 0.510228574, 1) expect=(-1.17257392, 1.02356231, 0.510023832, 1)`
- **Scene 22 — World Space Fluid** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/22-read.log`  
  `[Playback][Frame 1] particle 38165 current=(-1.12396479, 1.00837338, 0.511970639, 1) expect=(-1.11833239, 1.00223088, 0.511571527, 1)`
- **Scene 23 — Env Cloth Small** (553 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/23-read.log`  
  `[Playback][Frame 42] particle 20748 current=(10.3552008, 0.0500000007, 0.0363739617, 1) expect=(10.3562059, 0.0500000007, 0.0369597822, 1)`
- **Scene 24 — Env Cloth Large** (557 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/24-read.log`  
  `[Playback][Frame 43] particle 2479 current=(1.02170241, 1.20024741, 1.5669992, 1) expect=(1.0210731, 1.20132351, 1.56657946, 1)`
- **Scene 25 — Flag Cloth** (546 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/25-read.log`  
  `[Playback][Frame 54] particle 1084 current=(1.30537343, 0.0853326246, 0.0456144735, 1) expect=(1.30515933, 0.0842865333, 0.0457332768, 1)`
- **Scene 26 — Inflatables** (540 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/26-read.log`  
  `[Playback][Frame 56] particle 208 current=(1.02607751, 0.113987662, 1.98181081, 1) expect=(1.02708101, 0.113202646, 1.98219907, 1)`
- **Scene 27 — Cloth Layers** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/27-read.log`  
  `[Playback][Frame 1] particle 8529 current=(0.250034124, 2.29382133, -0.348903626, 1) expect=(0.25, 2.29382181, -0.350000024, 1)`
- **Scene 28 — Sphere Cloth** (553 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/28-read.log`  
  `[Playback][Frame 45] particle 4452 current=(0.257743746, 0.0435791053, 0.907292008, 1) expect=(0.257647604, 0.0424402058, 0.906850457, 1)`
- **Scene 31 — Game Mesh Rigid** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/31-read.log`  
  `[Playback][Frame 1] particle 1094 current=(49.253727, 14.726038, 37.2162018, 1) expect=(49.2539253, 14.7235165, 37.2161942, 1)`
- **Scene 32 — Game Mesh Particles** (558 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/32-read.log`  
  `[Playback][Frame 42] particle 10337 current=(33.2914009, 5.62838697, 46.9866371, 1) expect=(33.2932129, 5.62640095, 46.9866066, 1)`
- **Scene 33 — Game Mesh Fluid** (580 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/33-read.log`  
  `[Playback][Frame 20] particle 10296 current=(35.3601303, 5.16568327, 49.4119568, 1) expect=(35.35886, 5.1658597, 49.4111633, 1)`
- **Scene 34 — Game Mesh Cloth** (599 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/34-read.log`  
  `[Playback][Frame 1] particle 21919 current=(38.6725006, 11.6237173, 50.4700012, 1) expect=(38.6725349, 11.6237164, 50.4710884, 1)`
- **Scene 42 — Melting** (578 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/42-read.log`  
  `[Playback][Frame 18] particle 2126 current=(-1.15306878, 3.06410074, 0.431715429, 1) expect=(-1.15360677, 3.06557202, 0.431865305, 1)`
- **Scene 46 — DamBreak 5cm** (589 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/46-read.log`  
  `[Playback][Frame 11] particle 2057 current=(0.0291179102, 0.0697758868, 0.352627546, 1) expect=(0.0302613638, 0.0691992268, 0.353230447, 1)`
- **Scene 47 — DamBreak 10cm** (583 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/47-read.log`  
  `[Playback][Frame 17] particle 5647 current=(0.682438433, 0.41345647, 0.975641608, 1) expect=(0.682148874, 0.412342221, 0.974765778, 1)`
- **Scene 48 — DamBreak 15cm** (581 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/48-read.log`  
  `[Playback][Frame 19] particle 5 current=(0.0631031916, 0.0487500019, 0.39711839, 1) expect=(0.0643323958, 0.0487500019, 0.396928072, 1)`
- **Scene 49 — Rock Pool** (588 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/49-read.log`  
  `[Playback][Frame 12] particle 1056 current=(0.0286066532, 0.0300000012, -1.00498629, 1) expect=(0.0298044067, 0.0300000012, -1.00498629, 1)`
- **Scene 54 — Rigid2** (504 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/54-read.log`  
  `[Playback][Frame 96] particle 6760 current=(1.88152206, 2.74375248, 1.31737947, 1) expect=(1.88160443, 2.74355602, 1.31631064, 1)`
- **Scene 55 — Rigid4** (414 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/55-read.log`  
  `[Playback][Frame 186] particle 1094 current=(2.37063313, 1.49875915, 1.89644635, 1) expect=(2.36953545, 1.49856412, 1.89606261, 1)`
- **Scene 56 — Rigid8** (335 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/56-read.log`  
  `[Playback][Frame 265] particle 18354 current=(-1.21297681, 0.479972243, 5.02989054, 1) expect=(-1.21276593, 0.481243908, 5.02990818, 1)`
- **Scene 60 — Parachuting Bunnies** (496 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/60-read.log`  
  `[Playback][Frame 104] particle 2202 current=(1.38545799, 4.71900797, 0.40849787, 1) expect=(1.38537288, 4.71799612, 0.408570439, 1)`
- **Scene 61 — Water Balloons** (598 mismatched frames)  
  Log: `/tmp/nvflex-all-scenes-parity-1-600/61-read.log`  
  `[Playback][Frame 2] particle 10843 current=(2.26596117, 1.14914811, 1.79978764, 1) expect=(2.265167, 1.15040445, 1.80042911, 1)`

## Remaining failure groups

The first long-range mismatch data groups the remaining work into:

- rigid/soft-body accumulation: scenes 0, 1, 6, 7, 8, 54, 55, and 56;
- moving or complex shapes: scenes 14, 15, 16, and 17;
- transformed/local-space simulations: scenes 20 and 22;
- aerodynamic cloth and inflatables: scenes 23 through 28 and scene 60;
- game-mesh collision: scenes 31 through 34;
- mixed phase/state transitions: scene 42 and scene 61;
- container/mesh collision in fluid scenes: scenes 46 through 49.

Scene 16 is the highest-severity residual: it produces invalid, extremely large
particle coordinates at frame 1. That path should be investigated before
smaller long-term numerical drift.
