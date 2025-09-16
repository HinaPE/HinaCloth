# Repository Guidelines

## Project Structure & Module Organization
The repository currently stores IDE and formatting configuration in the root (e.g., `.clang-format`, `.idea/`). Place production C++ sources under `src/`, shared headers under `include/`, and test code under `tests/`. Keep large binary assets or reference textures in `assets/` and avoid cluttering the root. Whenever you introduce a new module directory, update the top-level `CMakeLists.txt` (or add one if absent) so CLion indexes it cleanly.

## Build, Test, and Development Commands
Use an out-of-tree CMake workflow: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug` configures dependencies, `cmake --build build` compiles all targets, and `ctest --test-dir build` exercises the registered suites. For quick iterations inside CLion, mirror these settings in your toolchains. Clean builds by removing `build/` or running `cmake --build build --target clean`.

## Coding Style & Naming Conventions
Formatting is enforced by `.clang-format` (LLVM-derived, 4-space indentation, left-aligned pointers, limited single-line functions). Run `clang-format -i path/to/file.cpp` before committing. Name classes in PascalCase, functions in camelCase, and constants or compile-time flags in UPPER_SNAKE_CASE. Group headers by module and keep include order consistent with the config's category priorities.

## Testing Guidelines
Add unit tests with GoogleTest or another CTest-compatible framework inside `tests/`, mirroring the source layout (`tests/cloth/ClothSolverTests.cpp` for `src/cloth/ClothSolver.cpp`). Each test binary should register with CTest via `add_test` so `ctest` catches regressions. Cover new public APIs plus failure paths (invalid cloth parameters, unstable integration steps) before requesting review.

## Commit & Pull Request Guidelines
Git history currently uses short, imperative subjects ("Initial commit"); keep that tone under 72 characters, appending issue IDs when relevant (`Improve stretch solver #12`). Pull requests should include a concise summary, validation evidence (command output or screenshots), and call out configuration changes (`.clang-format`, `.idea/`). Request review once local `ctest` runs succeed or CI passes.

## Tooling & Configuration Notes
Project files target CLion; review diffs under `.idea/` and avoid committing personal run/debug configurations. When editing `.clang-format`, test the new rules on a representative file to confirm no unexpected churn. Document third-party dependencies (compilers, physics libraries) in `README.md` when they change.