# ROADMAP_XPBD

Authoring Date: 2025-09-29  
Intended Audience: Internal framework developers & autonomous build/analysis agents (zero prior context assumed).  
Scope: End-to-end execution plan for implementing a multi-layout, multi-backend XPBD (Extended Position Based Dynamics) cloth solver within the existing modular architecture, while retaining extensibility for Projective Dynamics (PD), FEM, and future domains (rigid, fluid, gas).  

---
## 0. Executive Summary
Goal: Deliver a production-grade, benchmarkable XPBD cloth module supporting:
- Multiple memory layouts (AoS, SoA, AoSoA, Hybrid) with pluggable layout policies.
- Multiple execution backends: Scalar (reference), SSE2, AVX2, TBB-parallel, CUDA, Vulkan Compute.
- Deterministic and optimized modes.
- Telemetry + benchmarking harness for quantitative comparison (accuracy, stability, performance, memory footprint).

Constraints:
- Maintain decoupled layering: Data Layout Policy -> Constraint Operators -> Backend Dispatch -> Scheduler -> Telemetry.
- Non-invasive integration path for future PD, FEM, and coupling modules (cloth↔fluid/rigid).
- Avoid premature GPU micro-optimizations until CPU parity and correctness established.

High-Level Milestones (X0–X7):
X0 Infrastructure & Scalar XPBD  
X1 CPU SIMD (SSE2/AVX2)  
X2 TBB Parallelization + Determinism Mode  
X3 CUDA Baseline  
X4 Vulkan Compute Path  
X5 Advanced Layouts & Telemetry Enhancements  
X6 Additional Constraints (Bending) + Stability Tuning  
X7 Consolidation & Benchmark Suite (Cross-Backend Regression)  

---
## 1. Architectural Alignment & Extension Points
| Layer | Existing Skeleton Anchor | XPBD Additions | Future Reuse (PD/FEM) |
|-------|--------------------------|----------------|-----------------------|
| World/Core | `world_core` | Register cloth contexts + backend/layout descriptors | PD & FEM domain contexts reuse registration pattern |
| Domain Pipeline | `domain_cloth/*` | Add cloth_instance + solver pipeline phases | PD/FEM add new solver phases plugged into same scheduler |
| Algorithm Contracts | (opaque structs) | `xpbd_cloth_algorithm` concrete struct with vtable-like callbacks | PD: new algorithm struct; FEM: energy/hessian assembly reuse pattern |
| Performance Layer | `perf_layers/` | SIMD vector wrappers, layout packing strategies | Shared by PD constraint projection & FEM element loops |
| Scheduler | `schedulers/serial.*` | Introduce phase tasks: Predict, ConstraintsIter, Finalize | Parallel DAG extended for multi-domain dependency resolution |
| Telemetry | `telemetry_export/` | Metrics injection points for XPBD phases | Uniform metric taxonomy across future algorithms |
| Backend Abstraction | (none) | New `backend_dispatch.hpp` + runtime selection API | PD/FEM get same backend enumeration |

---
## 2. XPBD Algorithm Overview (Target Implementation)
### 2.1 Core Steps per Frame
1. External Forces / Velocity Update (semi-implicit precursor if needed)  
2. Position Prediction: p_i' = x_i + dt * v_i  
3. Iterative Constraint Projection (N iterations):
   - For each constraint c: compute constraint value C(p'), gradient(s) ∇C
   - Solve Δλ using compliance α and effective mass term w
   - Update positions p' += w * ∇C * Δλ (distributed over involved particles)
4. Velocity Update: v_i = (p_i' - x_i) / dt  
5. Finalize: x_i = p_i'

### 2.2 Constraint Types (Initial & Extended)
| Constraint | Phase | Initial Milestone | Data Fields | Notes |
|------------|-------|------------------|-------------|-------|
| Stretch (Edge) | Iteration | X0 | indices(i,j), restLength, compliance, λ | Performance-critical path |
| Bending (Dihedral) | Iteration | X6 | indices(i,j,k,l), restAngle, compliance, λ | Add after stretch stabilized |
| (Future) Area / Volume | Iteration | Later | polygon indices, rest area, λ | For PD/FEM bridging |

### 2.3 Compliance & Stability
Use XPBD compliance α to unify constraint stiffness across varying time steps:  
Δλ = -(C + α * λ) / (∑ w_i |∇C_i|^2 + α)  
Stability Enhancers (X6): adaptive iteration count based on residual norms, staged compliance ramping.

---
## 3. Data Layout Strategy
### 3.1 Layout Policies
Interface concept (pseudo C++ for clarity):
```
struct layout_policy_base {
  static constexpr layout_kind kind; // aos, soa, aosoa, hybrid
  // Allocation & views
  particle_access positions();   // returns structure with load/store ops
  particle_access velocities();
  particle_access inverse_masses();
  // (Optional) interleaved SoA block handles for AoSoA
};
```

### 3.2 Supported Layouts
| Layout | Pros | Cons | Use Case |
|--------|------|------|---------|
| AoS | Simple indexing, minimal code gen complexity | Poor SIMD locality; cache striding | Debug, reference correctness |
| SoA | Optimal for wide SIMD and coalesced GPU loads | Less intuitive per-particle ops | Default backend baseline |
| AoSoA (block size B) | Balances cache locality + SIMD alignment | Complexity in tail handling & gather overhead | High-performance CPU path |
| Hybrid (struct of critical SoA + auxiliary AoS) | Reduces pointer chasing for hot fields | More maintenance | Future complex constraints (FEM) |

### 3.3 Transcoding Strategy
- Primary (default) storage: SoA.
- On selecting AoS / AoSoA: either (a) transcode eagerly or (b) on-demand transcode with dirty flag.
- Provide `transcode_layout(cloth_instance, target_layout)`; no automatic bi-directional sync mid-frame.

### 3.4 Memory Alignment & SIMD Assumptions
- Align base arrays to 64 bytes (AVX2 line-friendly).
- For AoSoA: choose block size B = 8 or 16 (evaluate; 8 reduces tail overhead for small meshes, 16 better throughput on large).
- Provide compile-time trait `simd_lane_count<backend>`.

---
## 4. Backend Abstraction
### 4.1 Enumeration
```
enum class backend_kind : uint8_t { scalar, sse2, avx2, tbb_parallel, cuda, vk_compute };
```

### 4.2 Dispatch Table
```
struct constraint_kernel_suite {
  void (*predict_positions)(cloth_ctx&, float dt);
  void (*solve_stretch_batch)(cloth_ctx&, const stretch_batch&, int it, float dt);
  void (*finalize)(cloth_ctx&, float dt);
};
constraint_kernel_suite resolve_suite(backend_kind, layout_kind);
```

### 4.3 CPU SIMD Path
- Implement templated inner loops: `template<int L> solve_stretch_simd(...)` where L ∈ {1,4,8} mapping to Scalar/SSE2/AVX2.
- Fallback to scalar for misaligned tail (masked operations optional later).

### 4.4 Parallel (TBB) Path
- Partition constraints into color groups (no shared particle indices within a color) OR micro-batches hashed by min(index_i,index_j) % K.
- Per iteration: parallel_for over batches → barrier → next batch.
- Deterministic mode: disable dynamic scheduling, use fixed ordering vector.

### 4.5 CUDA Path (X3)
- Data: mirrored/owned device buffers (positions, velocities, invMass, constraints, λ, rest data).
- Kernels: predict, stretch_solve (one thread per constraint), finalize.
- Memory: global loads first; shared memory staging for AoSoA-like micro-tiles (defer optimization until X5+).
- Synchronization: sequential kernel launches per iteration; later optimize with persistent kernel or graph.

### 4.6 Vulkan Compute Path (X4)
- Descriptor sets: binding 0 (positions), 1 (velocities), 2 (invMass), 3 (constraints), 4 (λ), 5 (params).
- Push constants: iteration index, dt, compliance.
- Pipeline specialization: separate pipelines for predict/stretch/finalize for modular rebuild.

### 4.7 Backend Capability Query
Expose: `bool query_backend_support(backend_kind);` factoring build flags + runtime feature detection (CPUID, CUDA presence, Vulkan instance). 

---
## 5. Scheduling & Phasing
Proposed cloth phase enumeration (extensible):
```
enum class cloth_phase : uint8_t { Predict, ConstraintsIter, Finalize };
```
Integration with global world step: world scheduler inserts cloth phases appropriately. Later multi-domain interplay coordinates cloth predict before fluid coupling. 

Parallelization detail (ConstraintsIter):
- Outer loop (iteration count) serial.
- Inner per-color / per-batch loops parallel.
- Telemetry hook before & after each iteration (collect elapsed, residual).

---
## 6. Telemetry & Benchmarking
### 6.1 Metrics
| Metric | Description | Collection Phase |
|--------|-------------|------------------|
| ms_predict | Time for position prediction | Predict |
| ms_constraints_total | Sum of all constraint iterations | ConstraintsIter |
| ms_iteration[i] | Per-iteration constraint solve time | Each iteration |
| constraint_residual[i] | Max |C| or L2 norm after iteration i | Iteration end |
| sim_backend | Enum numeric code | Frame meta |
| layout_kind | Layout enum | Frame meta |
| particles | Particle count | Frame meta |
| constraints | Stretch constraint count | Frame meta |
| mem_bytes | Total allocated bytes for cloth buffers | Frame end |
| simd_lane_util | Active lanes / lane capacity (CPU SIMD) | ConstraintsIter |
| warp_occupancy | CUDA warp occupancy (if available) | CUDA path |

### 6.2 Benchmark Harness (X7)
Command-line tool target: `bench_xpbd` supporting arguments:
```
--backend=scalar|sse2|avx2|tbb|cuda|vk \
--layout=soa|aos|aosoa16 \
--particles=N --grid=WxH \
--iters=I --frames=F \
--output=csv|json --out-file=...
```
Outputs summary + per-frame metrics. 

### 6.3 Residual Measurement
Residual for stretch: | |p_i - p_j| - restLength |; aggregated max or RMS per iteration. Used to adapt iteration count (future optimization). 

---
## 7. Testing & Validation Strategy
### 7.1 Layered Tests
| Test Type | Scope | Tools |
|-----------|-------|-------|
| Unit | Single constraint solve scalar vs expected | Catch2 |
| Layout Consistency | AoS vs SoA vs AoSoA equality within tolerance | Catch2 parametrized |
| SIMD Parity | SSE2/AVX2 vs Scalar | Relative error ≤ 1e-6 positions |
| Determinism | Same seed & operations produce identical hash in deterministic mode | Hash aggregator |
| Backend Parity | CUDA/Vulkan vs scalar | Error threshold (1e-5) due to FP differences |
| Performance Regression | Time of standard scenes (small/medium/large) | Benchmark harness + threshold files |
| Memory Footprint | Byte accounting vs expected structural formula | Internal instrumentation |

### 7.2 Golden Scenes
1. Patch_16x16 (256 particles) – dev correctness quick run.
2. Patch_64x64 (4096 particles) – SIMD & parallel scaling.
3. Patch_128x128 (16384 particles) – GPU stress.
4. Curtain_Width256_Height512 (custom indexing) – constraint non-uniformity.

### 7.3 Hashing for Determinism
Define: `uint64_t state_hash = hash( round_to_float32(position[i]) for all i )` using FNV-1a incremental. 

---
## 8. Milestone Breakdown & Deliverables
### X0 (Infrastructure & Scalar XPBD) – Duration: 1–1.5 weeks
Deliverables:
- `layout_policy.hpp` (AoS, SoA minimal) + unit tests.
- `backend_dispatch.hpp` with scalar suite only.
- `cloth_instance` struct: particle arrays, constraint arrays, λ buffer.
- Scalar stretch constraint solver (no bending).
- Deterministic reference path.
- Basic telemetry: ms_predict, ms_constraints_total.
Exit Criteria: Patch_16x16 runs; residual decreases each iteration; tests pass.

### X1 (SIMD SSE2 / AVX2) – Duration: 1 week
Deliverables:
- SIMD load/store helpers (aligned/unaligned) in `simd_vec.hpp`.
- Vectorized stretch solve (batch edges in groups of L).
- Tail handling correctness tests.
- Telemetry: simd_lane_util metric.
Exit Criteria: AVX2 speedup ≥1.5x vs scalar on Patch_64x64.

### X2 (TBB Parallel Iterations) – Duration: 1–1.5 weeks
Deliverables:
- Constraint coloring or hashed batching.
- Parallel for each batch (SSE/AVX2 inside batches allowed).
- Deterministic flag toggles serial ordering.
- Tests for determinism hash equality.
Exit Criteria: Parallel speedup ≥ (logical_cores * 0.6) efficiency on Patch_128x128.

### X3 (CUDA Baseline) – Duration: 2 weeks
Deliverables:
- Device buffer management API.
- Kernels for predict, stretch solve, finalize.
- Host-device synchronization path.
- Parity tests vs scalar (error ≤1e-5).
Exit Criteria: CUDA speedup ≥2x vs AVX2+TBB on large scene (where GPU memory fits).

### X4 (Vulkan Compute) – Duration: 2 weeks
Deliverables:
- Minimal Vulkan context bootstrap (reuse existing `vulkan-visualizer` infra if feasible).
- Compute pipelines + descriptor sets.
- Cross-platform fallback (if Vulkan unavailable -> skip).
Exit Criteria: Vulkan parity (±10% performance vs CUDA baseline acceptable early).

### X5 (Advanced Layouts & Telemetry Enhancements) – Duration: 1 week
Deliverables:
- AoSoA layout (block size config via CMake or runtime param).
- Memory footprint telemetry + constraint throughput metric.
- Optional small shared memory staged kernel path for CUDA (if performance gap justifies).
Exit Criteria: AoSoA gives ≥10% speedup vs SoA for AVX2/TBB on Patch_128x128.

### X6 (Bending Constraints & Stability) – Duration: 1–1.5 weeks
Deliverables:
- Bending constraint data & solver integration.
- Adaptive iteration prototype (based on residual threshold). 
- Compliance tuning guidelines (doc addendum).
Exit Criteria: Bending constraints stable under moderate stiffness without explosive divergence.

### X7 (Consolidation & Bench Harness) – Duration: 1 week
Deliverables:
- `bench_xpbd` tool.
- Baseline result manifest (CSV/JSON) committed.
- Regression script verifying new runs within tolerance.
Exit Criteria: All backends produce results within documented error bounds; performance baselines recorded.

---
## 9. Risk Analysis & Mitigation
| Risk | Phase | Impact | Mitigation |
|------|-------|--------|-----------|
| SIMD path drift from scalar correctness | X1+ | Subtle instability | Continuous parity test vs scalar after each change |
| Parallel race conditions (λ / positions) | X2 | Non-deterministic artifacts | Color constraints (no shared particles per batch) or atomics-free partition |
| CUDA memory over-subscription on large meshes | X3 | OOM / perf collapse | Implement capacity check + chunked solve (future) |
| Vulkan driver variability | X4 | Unstable performance | Optional backend; mark experimental until stable metrics |
| Layout transcode cost | X5 | Frame hitching | Lazy transcode once at load time; disallow mid-frame switches |
| Bending constraint instability | X6 | Explosions | Introduce per-constraint compliance scaling & clamp Δλ |
| Benchmark flakiness | X7 | Regressions masked | Multiple-run median + warmup frames |

---
## 10. Implementation Details (Selected)
### 10.1 Constraint Storage (Stretch)
```
struct stretch_constraint {
  uint32_t i, j;  // particle indices
  float    rest_length;
  float    compliance; // α
  float    lambda;     // accumulated
};
```
For SoA: separate arrays for each field except (i,j) combined for coherence (`uint64 packed = (uint64)i << 32 | j`).

AoSoA block structure example (block size B):
```
struct stretch_block {
  uint32_t ij[ B ][2];
  float rest_length[ B ];
  float compliance[ B ];
  float lambda[ B ];
};
```

### 10.2 SIMD Edge Solve (Pseudocode)
```
for batch of L constraints:
  load i_idx, j_idx -> gather positions xi, xj
  diff = xi - xj
  dist = length(diff)
  C = dist - rest
  invMassTerm = w_i + w_j
  denom = invMassTerm + compliance / (dt*dt)
  dlambda = (-C - compliance*lambda) / denom
  corr = (dlambda / dist) * diff
  xi += corr * w_i
  xj -= corr * w_j
  lambda += dlambda
  store back positions & lambda
```

### 10.3 Determinism Hash
```
uint64_t hash_positions(const float* x, size_t n) {
  uint64_t h = 1469598103934665603ULL; // FNV-1a offset
  for (size_t k=0; k<n*3; ++k) {
     uint32_t bits = float_to_u32( round_to_fp32(x[k]) );
     h ^= bits; h *= 1099511628211ULL;
  }
  return h;
}
```
Used only in deterministic testing mode.

### 10.4 Runtime Backend Selection
Provide API:
```
int cloth_set_backend(cloth_id, backend_kind);
int cloth_set_layout(cloth_id, layout_kind);
```
Return status codes; validate support before switching.

---
## 11. Documentation & Developer Experience
Planned docs (incrementally appended to main status doc & a future `docs/` directory):
- `XPBD_OVERVIEW.md`: math derivations & constraint formulas.
- `BACKENDS.md`: explanation of dispatch layering & adding new backends.
- `LAYOUTS.md`: memory diagram, alignment and cache rationale.
- `BENCH_GUIDE.md`: how to reproduce baseline metrics.

---
## 12. Acceptance Criteria Summary per Milestone
| Milestone | Correctness | Performance | Instrumentation | Docs |
|-----------|-------------|-------------|-----------------|------|
| X0 | Residual ↓ monotonic | N/A baseline | Basic phase timers | Setup notes |
| X1 | SIMD == scalar (1e-6) | ≥1.5x scalar | Lane util metric | SIMD notes |
| X2 | Deterministic hash stable | ≥(0.6 * cores) scaling | Iteration timing | Parallel section |
| X3 | CUDA parity (1e-5) | ≥2x AVX2+TBB | Kernel timings | GPU quickstart |
| X4 | Vulkan parity (lenient) | Competitive ±10% | Descriptor stats | Vulkan notes |
| X5 | AoSoA parity | ≥10% gain vs SoA | Memory footprints | Layout doc |
| X6 | Bending stable | N/A | Residual per-type | Bending math |
| X7 | All parity tests pass | Baselines locked | Full telemetry | Bench guide |

---
## 13. Future-Proofing for PD & FEM
| Aspect | XPBD Design Choice | PD/FEM Benefit |
|--------|--------------------|----------------|
| Layout abstraction | Policy-based | FEM can adopt element-wise SoA/AoSoA with reuse |
| Backend dispatch | Suite table | PD energy/gradient kernels register similarly |
| Telemetry | Generic phase + metric IDs | PD residual & energy metrics integrate seamlessly |
| Determinism hash | Reusable state hash infra | PD reproducibility tests reuse same hashing |
| Constraint batching | Coloring pipeline | PD global system assembly partitioning reuse |

---
## 14. Tooling Enhancements (Optional Stretch)
- Preprocessor feature detection macro normalization: `HINAPE_CPU_AVX2`, `HINAPE_CPU_SSE2`.
- Build options: `HINAPE_WITH_CUDA`, `HINAPE_WITH_VULKAN` gating backend compilation.
- CI matrix (later): scalar-only / SIMD / CUDA-on / Vulkan-on variants.

---
## 15. Immediate Action Items (Pre-X0 Kickoff)
1. Add enums: `layout_kind`, `backend_kind` (header stub).
2. Implement SoA & AoS minimal allocation + tests (positions/velocities/invMass counts match; zero-init).
3. Implement scalar stretch constraint solver with single iteration & unit test verifying simple two-particle spring returns toward rest.
4. Integrate residual computation + telemetry hook placeholder.
5. Document initial dev run procedure (README snippet).

---
## 16. Open Questions (To Resolve During X0)
| Question | Options | Provisional Choice |
|----------|---------|-------------------|
| Real precision for cloth | float / double / templated | float (perf) |
| Constraint batching method | Coloring vs hash bucket | Start hash (simpler), evaluate coloring overhead later |
| AoSoA block size | 8 / 16 / configurable | Start 8 (less padding), measure 16 in X5 |
| GPU memory ownership | Mirror full vs partial streaming | Mirror full for simplicity |
| Vulkan pipeline reuse | Single pipeline multi-spec or per phase | Per phase initial simplicity |

---
## 17. Glossary
- XPBD: Extended Position Based Dynamics – adds compliance (softness) with time-step independent stability.
- Compliance: Parameter controlling constraint softness; inverse stiffness scaled by dt.
- AoSoA: Array-of-Struct-of-Arrays – blocks combine benefits of SoA & contiguous cache groups.
- Residual: Constraint violation magnitude after iteration.

---
## 18. Summary
This roadmap decomposes XPBD implementation into controlled, test-backed increments, ensuring correctness first (scalar), then horizontal scaling (SIMD / parallel), then vertical acceleration (GPU), followed by layout sophistication and stability features. Telemetry and parity gating at each stage reduce regression risk and prepare a robust substrate for upcoming Projective Dynamics and FEM expansions.

---
End of file.

