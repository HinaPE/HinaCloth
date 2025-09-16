# HinaCloth ![Windows CI](https://github.com/imeho/HinaCloth/actions/workflows/windows.yml/badge.svg)

HinaCloth is an XPBD (eXtended Position-Based Dynamics) cloth simulation benchmark. It ships a high-performance C++ solver, PyBind11 bridge, a Blender 4.5 extension, and tooling to measure runtime and constraint quality across backends.

## Features
- **XPBD solver** with native, Intel TBB, and AVX2 paths sharing the same numerical scheme.
- **PyBind11 module (`xpbd_core`)** automatically built against Blender’s Python, copied into the extension package with its runtime dependencies.
- **Blender extension** laid out per the 4.5 extension standard (ops/props/solver/ui), supporting modal playback, timeline bake, and backend switching.
- **Example scenes** in `example/`; CMake runs Blender in background mode after each build and produces matching `.blend` files alongside the scripts.
- **Benchmark harness** (`tests/test_xpbd.cpp`) reporting latency statistics *and* XPBD constraint residual metrics (mean/RMS/max, absolute and relative).

## Requirements
- CMake ≥ 3.26
- C++23 compiler (MSVC 19.44+/Clang/GCC)
- Python 3.11 (bundled with Blender 4.5)
- Intel TBB (downloaded and built by `setup_tbb.cmake`)
- Blender 4.5.3 LTS (for `.blend` generation and running the extension)

## Building
```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target xpbd_core --config Release
```
The build produces:
- `blender/modules/xpbd_core.pyd` (or `.so/.dylib`) with required DLLs
- `example/*.blend` alongside their corresponding `.py`

Override Blender detection if needed:
```powershell
cmake -S . -B build -DHINACLOTH_BLENDER_EXECUTABLE="C:/Program Files/Blender Foundation/Blender 4.5/blender.exe"
```

## Using the Blender Extension
1. Add the `blender/` folder as an extension source in Blender 4.5.
2. Enable “HinaCloth Extension” (panel under *View3D → Sidebar → HinaCloth*).
3. Pick a cloth mesh (or open the generated demo), choose the solver backend, then run *Start Modal* or *Bake Simulation*.

## Example Script
- `example/demo1.py` builds a 40×40 cloth scene, registers the extension, and saves `demo1.blend`:
  ```powershell
  "C:\Program Files\Blender Foundation\Blender 4.5\blender.exe" --background --python example/demo1.py
  ```

## Benchmarks & Tests
```powershell
cmake --build build --target test_xpbd --config Release
ctest --test-dir build
```
The test executable prints timing stats, relative speedups, and constraint residuals for each backend.

## Repository Layout
```
├─blender/            # Blender extension package (ops, props, solver, ui, manifest)
├─cmake/              # Build helpers (setup_pybind, setup_tbb, setup_blender)
├─example/            # Example Python scripts and generated .blend files
├─src/                # XPBD solver and PyBind11 bindings
├─tests/              # Benchmark / residual test program
└─README.md
```

## License
MPL-2.0 — see `LICENSE`.
