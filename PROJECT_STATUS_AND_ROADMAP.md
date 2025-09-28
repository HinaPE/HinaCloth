# PROJECT_STATUS_AND_ROADMAP

Authoring Date: 2025-09-28
Target Audience: A fresh AI development agent with zero prior context about this repository.
Primary Goal: Provide an exhaustive, implementation-grounded snapshot of the current codebase state (what *actually* exists vs. implied intent) and a forward roadmap with granular, actionable milestones.

---
## 0. High-Level Summary
This repository (root project name declared in CMake: `HinaPE`) is presently a **framework skeleton** for a future real-time / offline physics simulation platform with multi-domain coverage (cloth, fluid, gas, rigid) plus coupling, scheduling, telemetry, performance abstraction, and plugin extension. **Most translation units are placeholder-only** (no logic yet). The current value lies in the *planned modular architecture* placeholders, naming conventions, directory taxonomy, and preliminary build configuration (CMake with options for AVX2 and oneTBB).

The code does **not** yet implement real physics algorithms, data structures, numerical solvers, memory layouts, threading, or IO. All algorithm/domain/telemetry/coupling/perf layer files are minimal shells (often just forward-declared structs). This roadmap must therefore treat the present as a “blank structured contract space” and prioritize turning declarative intent into concrete subsystems.

---
## 1. Repository Structure (Observed)
Top-level notable items:
- `CMakeLists.txt`: Configures a static library `HinaPE` (C++23) with options:
  - `HINAPE_WARN_AS_ERROR` (default ON)
  - `HINAPE_WITH_TBB` (default ON) – integrates oneTBB via `cmake/setup_tbb.cmake`
  - `HINAPE_WITH_AVX2` (default ON for Release) – conditional compile flags + `HINAPE_HAVE_AVX2`
  - `HINAPE_BUILD_TESTS` (currently OFF)
  - `HINAPE_BUILD_EXAMPLES` (ON if `examples/CMakeLists.txt` exists)
  - Auto-recursive glob over `src/*.cpp|*.hpp|*.h` (note: pros/cons re incremental builds & implicit file adds)
- `include/` (currently only `rphys/`—not enumerated here but referenced by `api_impl.cpp` expected headers like `rphys/api_world.h`; those header files are actually **absent** in the repository as given—indicates a missing public API header set or not yet committed.)
- `src/` modular directories (detailed below)
- `examples/` (contains `main.cpp` & CMake integration; actual example logic not yet analyzed but implicitly relies on library symbols that currently do not exist in substantive form).

### 1.1 Core Source Subsystem Directories
| Directory | Purpose (Inferred) | Current Implementation State |
|-----------|--------------------|------------------------------|
| `src/core_base/` | Fundamental engine abstractions: world lifecycle, algorithm & domain bases, parameter & telemetry infra, field bus | All headers are thin forward declarations or trivial placeholders; `.cpp` files mostly empty (e.g., `world_core.cpp` has only an include + comment) |
| `src/api_layer/` | Gateway façade mapping (likely C-style or stable API layer) to internal opaque objects | Contains gateway_xxx.{hpp,cpp} placeholders; minimal or empty translation units |
| `src/domain_cloth/` | Cloth-specific pipeline contract and algorithm variants (PBD, XPBD, Stable PD, FEM) | Algorithms have empty `.cpp` with struct forward declarations in headers |
| `src/domain_fluid/` | Fluid algorithms (SPH, MPM, FLIP) | Same skeleton pattern |
| `src/domain_gas/` | Gas simulation (grid-based, LBM) | Same skeleton pattern |
| `src/domain_rigid/` | Rigid body algorithms (Impulse, Featherstone, XPBD) | Same skeleton pattern |
| `src/domain_new_template/` | (Implied pattern base for future domain) | Not enumerated in detail; presumably scaffolding |
| `src/coupling_modules/` | Cross-domain interaction descriptors (cloth-fluid drag/pressure, cloth-rigid contact/penalty/projection, multi-field mixers) | Header/CPP pairs exist; expected no logic (not inspected fully line-by-line but pattern consistent) |
| `src/perf_layers/` | Performance acceleration: SIMD abstraction, GPU backend stub, layout packing, cache, future optimization placeholder | All minimal stubs |
| `src/schedulers/` | Execution control: serial, job system stub, GPU stub, task pool | Placeholders only; no scheduling logic implemented |
| `src/telemetry_export/` | Export formats (CSV, JSON) | Stubs (no serialization pipelines implemented) |
| `src/plugins/` | Plugin registration sample | Minimal placeholder |
| `src/algo_sandbox/` | Experimentation tools (parameter scan, differential fields, run matrix) | Skeleton stubs |
| `src/api_impl.cpp` | Aggregates (includes) many `rphys/api_*.h` headers (which are missing in repo) | Empty translation unit indicates missing API surface files |
| `src/api_stubs.cpp` | (Present in listing) – not yet read but by pattern likely empty placeholder | Probably empty |

### 1.2 Missing / Not Present but Referenced
The file `api_impl.cpp` includes headers under `rphys/` that DON'T exist in `include/rphys/` (based on current listing). These missing public API header groups:
- `api_world.h`
- `api_domain.h`
- `api_algorithm.h`
- `api_scene.h`
- `api_params.h`
- `api_fields.h`
- `api_coupling.h`
- `api_events.h`
- `api_commands.h`
- `api_telemetry.h`
- `api_capability.h`
- `api_version.h`

Conclusion: A *planned stable C API / façade layer* is absent and must be authored before any external integration or example can compile meaningfully.

---
## 2. Current Implementation Reality (Granular)
Below is a breakdown of what is genuinely implemented (as of snapshot) vs. implied but not yet realized.

### 2.1 Core Objects
- `world_core.hpp` declares:
  - `struct world_config { int placeholder{}; };`
  - Opaque `world_core` forward declaration.
  - Factory/lifecycle: `create_world_core`, `destroy_world_core`, `step_world_core(world_core*, double dt)` – **No definitions** present.
- `algorithm_core`, `domain_core`, `telemetry_core` likewise only forward-declared; no virtual interface, no polymorphic base, no vtables, no type registries.
- `field_bus.hpp` defines a data record struct with raw pointers for field metadata and declares registration / lookup functions – **no storage container or lifetime semantics implemented**.
- `param_store.hpp` declares set/get for doubles – no type-erasure mechanics, no internal structure design.
- `status_codes.hpp` empty placeholder (no enum or code mapping yet).

### 2.2 Domain Layers
Each domain has a `pipeline_contract.hpp` with only opaque struct declarations; algorithms have no parameters, no solver loops, no memory layout decisions. Missing components include:
- Mesh / particle / grid representations.
- Constraint graph management (cloth XPBD/PBD/Stable PD/FEM need adjacency + constraint solving passes).
- Time integration scheme(s): semi-implicit Euler, symplectic, or specialized position-based loop.
- Spatial acceleration structures (broad-phase collision, neighbor queries for SPH/MPM/FLIP, etc.).
- Material parameter definitions.
- Data layout strategies (SoA vs AoS decisions; vectorization alignment).

### 2.3 Coupling Modules
Present file names imply planned features:
- Cloth–Fluid: drag, two-way pressure exchange.
- Cloth–Rigid: penalty, projection/contact.
- Multi-field mixing (generic field combination transform).
Actual content presumably empty; thus no coupling orchestration, scheduling order, or data exchange semantics exist.

### 2.4 Performance Layers
Files like `simd_vec.hpp`, `layout_pack.hpp`, `gpu_backend.hpp` are placeholders. Missing:
- Abstraction boundaries: macro or template policies for vector lanes.
- CPU feature detection (beyond compile-time `HINAPE_HAVE_AVX2`).
- Memory alignment & pooling.
- GPU dispatch abstraction (command queue, resource lifetime, kernel layering).

### 2.5 Scheduling
`schedulers/serial.*` and `task_pool.*` exist but are not implemented. Absent components:
- Task graph representation (DAG nodes for domain steps, coupling passes, telemetry flushes).
- Thread pool or integration with oneTBB (despite build option enabling TBB macro define).
- Frame orchestration (pre-step, domain solves, coupling passes, post-step flush, telemetry export).

### 2.6 Telemetry & Export
CSV / JSON dump placeholders exist; missing:
- Structured event / metric channel definitions.
- Buffering strategy (ring buffers vs direct streaming).
- Performance counters (timers, memory, stable instrumentation points).
- Schema versioning for serialized outputs.

### 2.7 API Layer Gateways
Gateway files (e.g., `gateway_world.hpp/cpp`) exist but not populated. Missing design choices:
- Handle vs pointer lifetime strategy.
- Error code enumeration & mapping to status codes.
- Thread-safety policy for API entrypoints.
- Version negotiation (`api_version.h`).

### 2.8 Examples
`examples/main.cpp` (not inspected in detail here) likely cannot run meaningful simulation due to unimplemented core symbols.

### 2.9 Build & Tooling
- C++23 standard set (opportunity: consider if features used justify; otherwise C++20 might widen compiler compatibility).
- Warning level high; warnings-as-errors ON – good for early discipline.
- No linting/format automation (e.g., clang-format, clang-tidy) configured.
- Tests disabled by default; `test/` folder mentioned but CMake guard only adds if `HINAPE_BUILD_TESTS=ON` and file exists (content unknown / presumably absent). No baseline unit tests.

---
## 3. Architectural Intent (Derived from Placeholders)
The architecture as implied would layer as follows:

1. Public C API (planned) – stable boundary for external engines/tools.
2. Gateway layer – marshals from C API handles to internal opaque C++ objects.
3. Core engine objects – world orchestrator, domain contexts, algorithm strategy objects.
4. Domain pipelines – each domain exposes pipeline contract binding algorithm phases.
5. Algorithms – specific numerical method implementations (PBD, FEM, SPH, MPM, LBM, XPBD, Impulse, Featherstone, etc.).
6. Coupling modules – inter-domain data exchange (drag forces, pressure feedback, contact resolution).
7. Scheduling layer – sequences per-frame passes, multi-threading, GPU offload.
8. Performance backends – SIMD/GPU feature adaptation.
9. Telemetry – metrics collection, structured events, export.
10. Plugins – runtime extension / dynamic module discovery.

---
## 4. Gap Analysis & Risk Register
| Gap | Impact | Risk Level | Mitigation Strategy |
|-----|--------|------------|---------------------|
| No concrete data structures | Blocks all algorithm work | Critical | Design canonical core data layout first (particles, constraints, grids) |
| Missing API headers | External integration impossible | High | Author `api_version.h` + minimal world bootstrap API first |
| No memory management policy | Potential fragmentation & performance loss | High | Introduce custom allocators / pools early; instrument allocations |
| Lack of test harness | Regression risk skyrockets as features land | High | Introduce Catch2/GoogleTest + snapshot tests for deterministic steps |
| Absent threading model spec | Concurrency retrofitting cost later | High | Define task graph contract & thread-safety rules now |
| Coupling order undefined | Unstable or non-physical results | Medium | Formalize per-frame phase ordering (e.g., DomainPredict -> Coupling -> Solve -> Post) |
| Telemetry design absent | Hard to debug performance | Medium | Add timing scopes & metric registry before heavy optimization |
| Glob source inclusion | Hidden build changes risk | Low | Optionally replace with explicit target sources list when codebase stabilizes |
| AVX2 conditional only | No runtime dispatch fallback design | Medium | Add CPU feature probe (cpuid) & dispatch tables later |
| Plugin system unspecified | Risk of ABI breakage | Medium | Define stable C structs + version negotiation early |

---
## 5. Foundational Design Decisions (To Be Made Early)
1. Coordinate & Units Convention: meters / seconds; confirm gravitational constant usage and scaling.
2. Precision: float vs double; potentially templated real type; consistency across domains.
3. Memory Layout: SoA for vectorizable loops; hybrid structures for constraints referencing indices.
4. Thread Safety: Read-only sharing vs fine-grained locks; prefer task graph + immutable frame staging.
5. Determinism Policy: Optional deterministic mode requiring fixed reduction orders.
6. Error Handling: Return status codes (C API) + assert/exception internal (decide policy).
7. Logging Interface: Decide on lightweight macro front-end with compile-time category filtering.
8. Telemetry Transport: In-memory ring buffers + periodic exporter pass.
9. Build Feature Flags: Introduce `HINAPE_ENABLE_ASSERTS`, `HINAPE_ENABLE_LOGGING`, `HINAPE_ENABLE_TELEMETRY`.
10. Plugin ABI Versioning: Semantic version triple + feature bitmask.

---
## 6. Proposed Data Model (Planned)
### 6.1 Cloth
- Particles: position (vec3), velocity (vec3), inverse mass, flags.
- Constraints:
  - Distance (struct { uint32 i,j; float rest; float compliance; })
  - Bend / dihedral (quad index set)
  - FEM elements (tet / triangle for thin-shell; store material params: Young's modulus, Poisson ratio)
- Acceleration structures: spatial hash or BVH for self-collision.

### 6.2 Fluid
- SPH: particle arrays (position, velocity, density, pressure), neighbor list (cell linked-list grid).
- FLIP: grid MAC velocities + particle carried velocities.
- MPM: material points + background grid nodes w/ mass, momentum.

### 6.3 Gas
- Grid (3D array) for temperature, velocity, density, pressure.
- Lattice Boltzmann distribution functions (D3Q19 layout). Separate bounce-back boundary metadata.

### 6.4 Rigid
- Body struct: pose (quat + vec3), linear/angular velocity, inertia tensor in body/local, mass, shape handle.
- Constraint solver variants: impulse sequential vs Featherstone articulated chain vs XPBD joints.

### 6.5 Coupling Meta-Structures
- Coupling registration table (vector of coupling pass descriptors):
  - Domain prerequisites
  - Execution phase slot
  - Function pointer or functor handle
  - Data handles required (field bus lookups)

### 6.6 Field Bus
- Internal map: string_view -> (pointer, count, stride, element type enum)
- Lifetime policy: fields registered at world construction or domain activation; mutable updates allowed per frame.

### 6.7 Parameter Store
- Hash map string -> variant (double, int64, uint64, bool, string, vec3) with typed accessors; on-set triggers validation hooks.

---
## 7. Execution Pipeline (Proposed Phases)
1. FrameBegin
2. ParameterSync (apply deferred changes)
3. DomainPredict (integrate velocities -> predicted positions)
4. CouplingPreSolve (force exchanges using predicted state)
5. DomainSolve (iterative constraint / pressure / diffusion / body solves)
6. CouplingPostSolve (projection, impulses that require solved states)
7. DomainFinalize (velocity update, volume corrections)
8. TelemetryCollect (snapshot metrics / counters)
9. TelemetryExport (batched writers: CSV, JSON)
10. FrameEnd (cleanup transient allocators)

Each phase is a node set in the scheduler DAG; dependencies enforced explicitly.

---
## 8. API Layer Minimal Viable Set (Initial Sprint)
| Function | Purpose | Priority |
|----------|---------|----------|
| `rphys_get_version(int* major, int* minor, int* patch)` | External version probe | P0 |
| `rphys_world_create(const rphys_world_desc*, rphys_world_handle*)` | Create world | P0 |
| `rphys_world_step(rphys_world_handle, double dt)` | Advance simulation | P0 |
| `rphys_world_destroy(rphys_world_handle)` | Release resources | P0 |
| `rphys_cloth_add(rphys_world_handle, const rphys_cloth_desc*, rphys_cloth_handle*)` | Register cloth domain instance | P1 |
| `rphys_param_set_double(rphys_world_handle, const char* key, double v)` | Runtime parameter tweak | P1 |
| `rphys_field_query(rphys_world_handle, const char* name, rphys_field_info*)` | Expose internal arrays (read-only) | P1 |
| `rphys_telemetry_flush(rphys_world_handle, const rphys_telemetry_sink_desc*)` | On-demand export | P2 |

Error handling: All return an int status code (0 = OK, non-zero enumerated in future `status_codes.hpp`).

---
## 9. Telemetry & Metrics Plan
Categories:
- Frame: dt, phase durations, active bodies/particles, memory high-water marks.
- Domain-specific: iteration counts (PBD solver), SPH neighbor avg, rigid solver contact pairs.
- Performance: SIMD path usage, thread pool occupancy, TBB task count, GPU dispatch times.
- Coupling: number of cross-domain interactions, rejected contacts, stabilized constraints.

Exporters produce rotating file sets with schema header:
```
{ "schema_version": 1, "timestamp_start": <epoch>, "fields": [ ... ] }
```
CSV fallback for large volume streaming.

---
## 10. Performance Strategy (Planned Sequence)
1. Baseline single-thread deterministic implementation (no premature optimization).
2. Introduce SoA packing & alignment (32/64-byte boundaries) -> measure.
3. Add constraint batching & parallel for (TBB) with conflict graph coloring for PBD/XPBD.
4. Introduce broad-phase spatial index (sweep-and-prune or hashed grid) shared across cloth & fluid neighbor queries where feasible.
5. SIMD pass: vectorize constraint projection kernels & SPH density/force loops.
6. Optional GPU backend for particle neighbor computation and grid transfers (FLIP / MPM).
7. Introduce frame-level adaptive iteration heuristics (target error thresholds).

Instrumentation: RAII timer macros writing into per-thread buckets aggregated end-of-frame.

---
## 11. Testing Strategy
Layers:
- Unit Tests: parameter store, field bus registration, handle lifecycle.
- Deterministic Snapshot Tests: fixed seed world results after N steps vs golden JSON (tolerances).
- Performance Regression: micro-bench harness measuring constraint solve time for fixed particle counts.
- Fuzz / Robustness: random invalid API calls (null handles, out-of-range indices) assert graceful errors.
- Memory / Leak: integration with sanitizers (Linux CI) + periodic `valgrind` / `Dr. Memory` (Windows alt) check.

Initial test infra tasks:
1. Add `HINAPE_BUILD_TESTS` default ON in CI config.
2. Introduce `third_party/catch2` or similar (prefer FetchContent in CMake for portability).
3. Provide `tests/test_world_basic.cpp` exercising create/step/destroy.

---
## 12. Plugin System Design (Future)
- Shared library discovery through configurable path list.
- Exported C function `rphys_plugin_register(rphys_plugin_registry*)` with registry callbacks for:
  - Register algorithm type
  - Register coupling pass
  - Register telemetry metric provider
- Version handshake struct:
```
struct rphys_plugin_version { int abi_major; int abi_minor; int abi_patch; uint64_t feature_bits; };
```
- Isolation: Plugins cannot directly include internal headers; only C API.

---
## 13. Security / Robustness Considerations
- Handle validation: maintain generation counters to detect stale handles.
- Bounds checking on public API always ON (even in Release) – low cost relative to physics.
- Input sanitation: parameter assignments validated with range metadata.
- Denial-of-service guard: Hard cap maximum entity counts configurable.
- Floating-point exceptions: Option to enable FE traps in debug.

---
## 14. Dependency & Toolchain Roadmap
| Dependency | Current | Planned Usage | Actions |
|------------|---------|---------------|---------|
| oneTBB | Build option integrated | Task parallelism | Add task partitioning after baseline serial correctness |
| (None others) | - | Future: fmt, spdlog (optional) | Evaluate adding minimal logging (compile-time toggle) |
| GPU (CUDA/Vulkan) | Not integrated | Potential fluid/cloth acceleration | Abstract backend before committing to specific API |
| Python Bindings | Not present | Scripting layer | Use pybind11 after core stable |

---
## 15. Milestone Plan (Detailed)
### Milestone 0 (Bootstrap – 1 week)
- Add missing public API headers with minimal world lifecycle.
- Implement `world_core` with simple internal struct storing `dt_accumulator`.
- Provide parameter store backed by `std::unordered_map<std::string, variant>` (double only initially).
- Implement field bus with vector registry.
- Add unit tests for lifecycle & param store.

### Milestone 1 (Cloth PBD Skeleton – 2 weeks)
- Implement particle array & distance constraint solver (single iteration loop).
- Add basic gravity & damping force.
- Expose particle positions via field bus.
- Add simple example populating cloth grid.

### Milestone 2 (Scheduling & Telemetry – 2 weeks)
- Introduce phase enum & serial scheduler executing ordered vector of phase callbacks.
- Add RAII timers & telemetry core collecting phase durations.
- CSV exporter writing per-frame metrics.

### Milestone 3 (Parallelization – 3 weeks)
- Integrate oneTBB for parallel for loops over constraints & particle integration.
- Introduce conflict mitigation (graph coloring) for constraint sets.
- Add tests comparing serial vs parallel results (bitwise tolerance or near-equality).

### Milestone 4 (Additional Domains Entry – 4 weeks)
- Implement SPH minimal (density + pressure + integration) with naive neighbor O(N^2) first.
- Abstract neighbor query interface; plug spatial hash in after baseline.
- Begin rigid impulse solver with simple shape (sphere) contact.

### Milestone 5 (Coupling Foundations – 3 weeks)
- Implement cloth-fluid one-way drag (sample velocities from fluid -> apply to cloth).
- Introduce coupling registration ordering API.
- Telemetry metric: coupling pass durations & count of interactions.

### Milestone 6 (Data Layout & SIMD – 3 weeks)
- Convert cloth particle storage to SoA arrays (positions xyz separate or struct-of-arrays with 16-byte alignment).
- Introduce `simd_vec` abstraction (intrinsics for AVX2 path + scalar fallback).
- Benchmark harness added.

### Milestone 7 (Stability & Advanced Solvers – 5 weeks)
- Add XPBD compliance parameters.
- Introduce bending constraints (cloth) & basic self-collision broad-phase (spatial hash + pruning).
- Start FEM prototype (triangle-based, linear elasticity).

### Milestone 8 (Telemetry Expansion & JSON Export – 2 weeks)
- Add JSON exporter with batched frame grouping.
- Add memory usage sampling (platform-specific) + solver iteration metrics.

### Milestone 9 (Plugin & Extensibility – 4 weeks)
- Define plugin ABI & registry.
- Provide sample external plugin implementing a custom constraint.

### Milestone 10 (Refinement & Docs – ongoing)
- Harden error codes, document all API functions, produce user integration guide.

---
## 16. Detailed Backlog (Actionable Items)
Below is a categorized backlog; items marked (!) are prerequisite-critical.

### Core Engine
- (!) Implement `world_core` internal struct with domain lists, params, field registry.
- (!) Implement create/destroy/step definitions.
- Add frame phase enumeration + dispatcher.
- Add per-domain registration structure (type id, pointer to vtable of callbacks).

### Parameter Store
- Double-only baseline map.
- Add multi-type variant support.
- Add validation metadata (min/max/default).
- Add deferred commit queue (apply at FrameBegin).

### Field Bus
- Vector registry container + hash map (string -> index).
- Add const + mutable registration distinction.
- Add debug validation for overlapping names.
- Implement `find_field` returning null if not found; add status enumeration.

### Domains
Cloth:
- Particle arrays + constraints container.
- Distance constraint solver (Gauss-Seidel iterations).
- XPBD compliance extension.
- Bending constraint + area preservation (optional).
Fluid:
- SPH minimal (density + pressure + external forces).
- Add spatial cell hashing.
- Add viscosity term.
Rigid:
- Body integrator (semi-implicit).
- Broad-phase (AABB sweep) placeholder.
- Contact resolution (impulse, sequential solver). 
Gas:
- Grid allocation + advection placeholder.
- Pressure projection (Poisson solver stub).

### Coupling
- Registry of coupling passes keyed by phase.
- Cloth-fluid drag force sampling.
- Cloth-rigid penalty contact (point-plane first).

### Scheduling
- Serial first; vector of `phase_callback { name, fn_ptr }`.
- Introduce job system wrapper for parallel tasks.
- Task dependency resolution (simple topological order) later.

### Performance
- Introduce aligned allocation helper.
- Add compile-time dispatch macros for AVX2 vs scalar.
- Micro-benchmark harness (compile optional target `bench`).

### Telemetry
- Metric registry (string -> numeric series accumulator).
- Timing scopes (stack-based start/stop).
- CSV writing with header.
- JSON batched export.

### API / Public Surface
- Define handles as `struct rphys_world_t {}` (opaque) + C functions.
- Implement error code enum with stable numeric values.
- Provide version macro + function.
- Add symbol visibility control macros for Windows DLL usage future.

### Plugins
- Registry struct & loader (deferred until core stable).

### Tooling & CI
- Add `.clang-format` & `.clang-tidy` with initial rules.
- GitHub (or other) CI workflow: configure, build (Debug/Release), run tests.
- Artifact upload for telemetry sample outputs.

### Documentation
- API reference (Doxygen or Sphinx cross-language if Python planned).
- Architecture primer (diagrams: world, phases, data flow between domains).
- Contribution guide (coding style, branching model, review checklist).

---
## 17. Coding Standards (Proposed)
- Naming: `snake_case` for functions in C API; `camelCase` or `snake_case` for internal functions (choose consistently—recommend `snake_case` for internal too). Types: `PascalCase`.
- Header guards: already consistent `#ifndef RPHYS_...` keep pattern.
- No raw `new/delete` outside central allocators.
- Use `span`-like views for non-owning arrays (consider C++23 `std::span`).
- Avoid exceptions in hot paths; optional use in initialization only.
- Each public API function must document thread safety and ownership semantics.

---
## 18. Error Handling & Status Codes (Planned Enumeration Draft)
| Code | Meaning |
|------|---------|
| 0 | OK |
| 1 | INVALID_ARGUMENT |
| 2 | NOT_FOUND |
| 3 | ALREADY_EXISTS |
| 4 | OUT_OF_MEMORY |
| 5 | UNINITIALIZED |
| 6 | INTERNAL_ERROR |
| 7 | UNSUPPORTED |
| 8 | VERSION_MISMATCH |
| 9 | CAPACITY_EXCEEDED |

Mapping macro suggestion:
```
#define RPHYS_TRY(expr) do { int _r = (expr); if (_r != 0) return _r; } while(0)
```

---
## 19. Memory Management Plan
- Phase 1: Use STL containers (vector, unordered_map).
- Phase 2: Introduce custom `pod_vector` with alignment and prefetch hints for particle arrays.
- Phase 3: Slab allocator for transient solver buffers (iteration scratch).
- Phase 4: Arena per-frame reset at FrameEnd for ephemeral allocations.
- Diagnostic: Optional memory tagging (enum resource class) aggregated by telemetry.

---
## 20. Determinism Strategy
- Deterministic mode flag in world config.
- Avoid parallel reduction without defined order (serialize or use stable segmented reduction).
- Store and lock step dt (no adaptive dt in deterministic mode).
- Provide state hash function for reproducibility checks (XOR of bit-cast particle positions/velocities).

---
## 21. Concurrency Model Plan
- Task categories: DomainPredict, DomainSolve, CouplingPre, CouplingPost, Telemetry, Export.
- Each category subdivides into tasks for each domain instance.
- Conflict avoidance for constraints: coloring or partitioning by disjoint index sets.
- Work-stealing via TBB or custom pool.
- Fallback serial path always available for debugging.

---
## 22. GPU Offload Future (Exploratory)
- Candidate kernels: neighbor search (hash build + sort), SPH density, constraint projection for large cloth.
- Abstraction: `gpu_backend` defines interface: allocate buffer, enqueue kernel, synchronize.
- Backends considered: CUDA (primary), HIP (portability), maybe Vulkan compute (existing `vulkan-visualizer` folder suggests adjacent interest).

---
## 23. Visualization Integration (Potential)
- Provide a lightweight frame buffer / debug draw interface (lines, points, AABBs).
- Expose pointer to simulation data for external renderer (zero-copy).
- Add optional compile-time flag `HINAPE_ENABLE_DEBUG_DRAW` gating instrumentation geometry capture.

---
## 24. Roadmap Timeline (Indicative Aggregate)
| Phase | Duration | Cumulative | Exit Criteria |
|-------|----------|------------|---------------|
| M0 | 1w | 1w | World create/step/destroy works; unit tests green |
| M1 | 2w | 3w | Cloth particles move under gravity with distance constraints |
| M2 | 2w | 5w | Phase scheduler + telemetry CSV; timing metrics visible |
| M3 | 3w | 8w | Parallel cloth solver matches serial within tolerance |
| M4 | 4w | 12w | Basic SPH & rigid body added (naive) |
| M5 | 3w | 15w | Cloth-fluid drag operational with telemetry metrics |
| M6 | 3w | 18w | SIMD cloth kernels producing measured speedup (>1.5x) |
| M7 | 5w | 23w | Advanced cloth (XPBD, bending) stable tests pass |
| M8 | 2w | 25w | JSON export + enriched metrics |
| M9 | 4w | 29w | Plugin loads external constraint successfully |
| M10 | ongoing | - | Docs, refinements, stability |

---
## 25. Immediate Next Actions (Action Queue)
1. Add missing `include/rphys/api_version.h` + `api_world.h` minimal declarations.
2. Implement `world_core.cpp` with simple struct + step accumulating dt.
3. Add unit test verifying world step increments frame counter.
4. Add param store simple map + test.
5. Populate gateway_world with thin wrappers bridging to core.

---
## 26. Verification Strategy After Each Milestone
- Build: Both Debug & Release (with AVX2 ON where supported) must compile warning-free.
- Tests: New coverage threshold increments (target gradually -> 60% of core logic by M7).
- Benchmarks: Record baseline ms/frame for canonical cloth (e.g., 10k particles) after M1, compare each performance milestone.
- Telemetry: Validate exporter schema unchanged or version incremented.
- API Stability: Generate header checksum manifest to detect accidental breaking changes.

---
## 27. Open Design Questions (To Resolve Soon)
| Question | Options | Recommendation |
|----------|---------|---------------|
| Real type precision | float / double / templated | Start float; allow compile-time switch later |
| Constraint solver iterations | Fixed vs adaptive | Start fixed; add adaptive by error metric later |
| Logging backend | Custom minimal vs spdlog | Custom minimal macro wrapper first |
| Handle representation | Opaque pointer vs index+gen | Index+generation improves validation |
| Build organization | Monolithic lib vs per-domain libs | Keep monolithic until complexity forces split |

---
## 28. Long-Term Extensions (Post-Core)
- Soft body tetrahedral FEM domain.
- Cloth tearing & remeshing.
- Fluid-structure interaction with two-way momentum exchange.
- GPU multi-queue scheduling & asynchronous data staging.
- Deterministic replay recorder (input + random seeds logging).
- Python scripting for batch simulations.

---
## 29. Quality Gates (Desired End-State Definition)
| Category | Gate |
|----------|------|
| Build | Zero warnings (MSVC/Clang/GCC), Werror enforced |
| Tests | >80% of logical branches in solver cores covered |
| Performance | Cloth 50k particles real-time (>=30 FPS) on reference CPU |
| Stability | No data races under TSAN in parallel mode |
| Determinism | Deterministic mode hash stable across 100 runs |
| Telemetry | <2% frame overhead with telemetry enabled |
| API | Semantic version increments only on additive or breaking changes documented |

---
## 30. Summary
The project is at an architectural scaffolding stage with minimal operational code. Success hinges on **quickly landing a thin vertical slice** (world + cloth minimal + telemetry + test) before expanding horizontally. This document defines the actionable path, clarifies missing assets, and enumerates all major design decisions to de-risk future implementation phases.

---
## 31. Document Maintenance Policy
- Update after every completed milestone (append CHANGELOG-like delta section below).
- Include a “Last Verified Commit” hash pointer (add once Git context integrated here).
- Re-run gap analysis quarterly or after major architectural change.

---
## 32. Pending Delta Section (Template for Future Updates)
```
### [Milestone X Completed: <Title>] - <Date>
Changes:
- ...
Metrics:
- Cloth 10k step time: old 4.2ms -> new 3.1ms
Open Issues Carried Forward:
- ...
```

---
End of file.

