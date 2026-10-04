# Repository Guidelines

## Project Structure & Module Organization

This repository contains NVIDIA FleX 1.2.0 sources, demos, binaries, and assets. Core simulation utilities live in `core/`, the reversed/runtime library implementation is in `src/`, public headers are in `include/`, and extension APIs are in `extensions/`. Demo code is under `demo/`, with renderer-specific paths such as `demo/d3d11/` and `demo/d3d12/`. HLSL sources are in `src/shaders/` and generated/disassembled shader artifacts are in `src/dxbc/`. Runtime assets are in `data/`; prebuilt DLLs/libs are kept in `bin/` and `lib/`.

## Build, Test, and Development Commands

Use an x64 Visual Studio generator unless you intentionally target another supported toolchain:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
cmake --build build --config Release
```

The top-level build creates `NvFlexRev`, `NvFlexExt`, and `DemoAppD3D`, with outputs copied to `build/bin/<Config>/`. Launch packaged demos with `run_cuda.bat` or `run_dx.bat`; pass options such as `-d3d12`, `-benchmark`, or `-vsync=0` to the demo executable for targeted checks.

## Coding Style & Naming Conventions

Follow `.clang-format` for C/C++ changes: 4-space indentation, attached braces, 92-column limit, left pointer alignment, and no tab indentation. Do not sort includes unless needed by the local file style. Existing type and API names use NVIDIA-style prefixes such as `NvFlex*`, while source files generally use PascalCase for library modules and lowercase names in legacy `core/`. Keep shader entry names and generated object names aligned with the relevant `Shaders.cfg`.

## Testing Guidelines

There is no dedicated unit-test framework in this snapshot. Before submitting changes, build at least `Debug` for compile coverage and run the D3D demo path affected by the change. For simulation, shader, or renderer edits, use a small scene smoke test and, when useful, `-benchmark` to check for crashes and obvious performance regressions.

## Commit & Pull Request Guidelines

Git history uses short, imperative or release-focused subjects such as `Update README.md` and `CUDA 9.2.148 upgrade`. Keep commit subjects concise and specific. Pull requests should describe the changed subsystem, list build/demo validation performed, call out driver/toolchain requirements, and include screenshots or captures for visual demo or rendering changes.

## Agent-Specific Instructions

Avoid touching shipped binaries in `bin/` and `lib/` unless the task explicitly requires a binary refresh. Keep generated build output under `build/` out of source edits. When changing shaders, update source HLSL and any checked-in generated/disassembly artifacts together if the repository expects them to stay synchronized.
