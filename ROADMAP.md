# ROADMAP – Research-Oriented Highly Decoupled Physics Framework

> Scope: From zero (skeleton world + minimal cloth PBD) to multi‑domain, multi‑algorithm, coupled, deterministic, extensible research pipeline.
> Versioning: Semantic (MAJOR.MINOR.PATCH). MAJOR only when public API headers change semantics.

---
## Phase Overview Table
| Phase | Codename | Focus | Key Domains | Coupling | Performance | Determinism | Deliverable / Exit Criteria |
|-------|----------|-------|-------------|----------|-------------|------------|-----------------------------|
| 0 | Seed | Skeleton world + cloth PBD | Cloth (PBD) | None | Serial baseline | L0 | create/step works; export positions |
| 1 | Twin | Add XPBD cloth; A/B compare | Cloth (PBD/XPBD) | None | Serial baseline | L0 | run_matrix diff works; telemetry basic |
| 2 | Bus | field_bus + param_store maturity | Cloth | None | Serial baseline | L0 | Dynamic param changes; field snapshots generic |
| 3 | Flow | Introduce fluid SPH domain | Cloth + Fluid (SPH) | None | Serial baseline | L0 | Two independent domains step together |
| 4 | Bridge | Basic cloth–fluid coupling | Cloth + Fluid | Drag / Pressure | Serial baseline | L0 | Coupling modules selectable & detachable |
| 5 | Lift | Scheduling + layout pack perf layer | Cloth + Fluid | Drag / Pressure | Task pool + layout_pack | L1 | >25% speedup vs serial baseline |
| 6 | Expand | Add FEM cloth, MPM fluid, Rigid domain | Cloth/Fluid/Rigid | Existing | Task pool | L1 | All domains coexist; sandbox matrix includes new algos |
| 7 | Precision | Determinism L2 + extended telemetry | All current | Existing | Task + deterministic reduction | L2 | Bitwise stable replays, residual curves recorded |
| 8 | Plugin | External plugin API + docs | All | Existing | Configurable | L2 | Example plugin algorithm loads & runs |
| 9 | Fusion | Gas domain + cloth‑rigid contact coupling | Add Gas | + Contact | Task + layout_pack | L2 | Additional couplings toggle cleanly |
| 10 | Audit | Energy / mass audit + reproducibility harness | All | All | Perf opt optional | L2 | Audit pass thresholds < tolerances |
| 11 | Vector | SIMD/GPU prototype backend (optional) | Hot loops | All | SIMD / GPU stub | L2 | Flag enables vector path, fallback intact |
| 12 | Polish | Docs, change log, regression suite | All | All | Mix | L2 | CI green; baseline perf locked |

---
## Detailed Phases
### Phase 0 – Seed
Goals:
- Public API headers (subset): api_world, api_domain, api_algorithm, api_params, api_fields, api_telemetry, api_status, api_ids, forward.
- core_base: world_core, domain_core, algo_core, param_store (minimal), field_bus (read-only export), telemetry_core (frame_ms only).
- domain_cloth: pipeline_contract + algorithms/pbd_cloth (positions + simple Verlet update or naive PBD distance projection).
- Sandbox: run_matrix stub (just runs one algorithm once).
Exit Criteria:
- create world → add cloth domain (pbd) → step N frames → export cloth.position snapshot.
- Build < 5s; no external dependencies.

### Phase 1 – Twin
Add xpbd_cloth variant.
- Implement constraint lambdas & compliance parameter.
- Telemetry: per‑algorithm timing (prepare/solve/finalize).
- Sandbox: run_matrix executes (algo × iterations) grid; diff_fields compares final positions (L2 norm) to baseline.
Metrics Acceptance:
- XPBD stable with large stiffness where PBD elongates > threshold.

### Phase 2 – Bus
Harden field_bus + param_store.
- field_bus: typed meta (component_count, stride_bytes), support write‑back for param_scan modifications.
- param_store: variant scalar/vec/mat; reflection enumeration.
- Add param_scan_tool script interface (Python placeholder).
Acceptance:
- Dynamic iteration / substep changes mid-run take effect next frame.
- Parameter sweep logs matrix (csv/json).

### Phase 3 – Flow
Introduce fluid domain (SPH minimal): positions, velocities, density update (simplified kernel).
- fluid_sph algorithm, no pressure projection yet (just advection + neighbor density estimate).
- Distinct param namespace (fluid.*) and field names.
Acceptance:
- Cloth and fluid advance independently; no cross modification.

### Phase 4 – Bridge
Cloth–fluid drag & simple pressure coupling.
- coupling_modules/cloth_fluid_exchange: contract lists required inputs (cloth.position, fluid.velocity, fluid.density).
- simple_drag: cloth receives velocity damping from fluid; pressure: fluid receives surface impulse.
- Failure isolation: if coupling init fails → both domains continue.
Acceptance:
- Enable/disable coupling changes acceleration norms; turning off coupling reverts to baseline.

### Phase 5 – Lift
Introduce performance layers.
- schedulers/task_pool (fixed worker pool, deterministic partition for L1).
- perf_layers/layout_pack (SoA packer + optional strided to AoSoA convert for cloth & fluid arrays).
- Telemetry extended: subphase timings (prepare/solve/finalize) per domain.
Target:
- ≥25% wall‑time improvement on benchmark scene vs Phase 4 (document scenario).

### Phase 6 – Expand
Add further algorithms & domain.
- cloth_fem (simple linear triangle strain), fluid_mpm (particle→grid→particle, simplified), rigid_impulse.
- Domain registration enumeration via api_capability.
Acceptance:
- Scenes can mix cloth (pbd/xpbd/fem) + fluid (sph/mpm) + rigid without compile‑time selection.

### Phase 7 – Precision
Determinism Level 2.
- Introduce deterministic reduction tree (pairwise fixed order), stable sorting of constraints.
- RNG counter-based seed ( (world_id, frame, domain_id, alg_variant) ).
- Telemetry: residual curve per solver iteration; serialization of run metadata.
Acceptance:
- Re-running the same command/event sequence yields bitwise identical final buffers (same machine + build flags).

### Phase 8 – Plugin
External plugin support.
- Formal plugin registration (function pointer table injection into algo_core registry).
- Sample: plugin adds new cloth algorithm that can be run without rebuilding core.
- CHANGELOG + MIGRATION doc templates.
Acceptance:
- dlopen (if supported) or static registration path loads plugin algorithm selectable via API.

### Phase 9 – Fusion
Add gas domain + cloth_rigid_contact coupling.
- lattice_boltzmann prototype; contact projection & penalty strategies.
- Coupling selection list enumerated from registry.
Acceptance:
- Scenes enabling contact & drag & pressure remain stable (< energy drift threshold).

### Phase 10 – Audit
Energy / mass / volume invariants.
- Telemetry: energy_before, energy_after, drift metrics.
- Alarm thresholds configurable (fail if > tolerance).
Acceptance:
- Coupled scenario stays within configured divergence bounds.

### Phase 11 – Vector
SIMD / GPU prototypes.
- perf_simd_vec: vector width detection; re-implement cloth PBD inner loop with SIMD.
- perf_gpu_backend: stub kernel launch path; fallback verified.
Acceptance:
- Speedup on vector‑capable machine; fallback path regression tests pass.

### Phase 12 – Polish
Full documentation, automated regression harness.
- Scripts: validate_deps.py (forbidden includes), replay_runner.py (deterministic replay), perf_baseline.json.
- Freeze public API headers; mark stable version 1.0.0.
Acceptance:
- CI pipeline: build + unit tests + deterministic replay + perf baseline variance < threshold.

---
## Risk & Mitigation Matrix
| Risk | Phase Hit | Impact | Mitigation |
|------|-----------|--------|------------|
| Over‑complex coupling early | 4 | Delays perf work | Keep first two strategies trivial |
| Determinism regressions | 7+ | Hard to debug | Log canonical ordered hashes per phase |
| Plugin ABI churn | 8 | Break external algos | Provide adapter shim & version negotiation |
| SIMD path divergence | 11 | Validation cost | Always run scalar shadow path in debug |
| Perf goals missed | 5/11 | Reduced value | Telemetry hotspots & micro-bench loops |

---
## Metrics & Benchmarks
| Metric | Collection | Target Baseline |
|--------|-----------|-----------------|
| frame_ms | telemetry_core | Document per phase |
| solver_residual | per iteration array | Monotonic decrease (XPBD) |
| energy_drift | energy audit | < 1% over 500 frames |
| speedup_task_pool | compare serial | ≥1.25x Phase 4 scene |
| speedup_simd | compare scalar | ≥1.5x PBD inner loop |
| deterministic_failures | replay harness | 0 |

---
## Non-Goals (Persist)
| Non Goal | Reason | Optional Future |
|----------|--------|-----------------|
| Universal ECS integration | Adds complexity | External integration layer |
| Auto codegen kernels | Premature | Could add JIT plugin |
| Implicit global coupling | Opaque debugging | Explicit coupling modules |
| Heavy GUI embedding | Distracts research | External lightweight viewer |

---
## Implementation Checklist Snapshot
(Keep updated in issues / project board.)
- [ ] Phase 0 skeleton
- [ ] field_bus read-only export (positions)
- [ ] PBD constraints stable for small time steps
- [ ] XPBD integrated (lambda buffers, compliance param)
- [ ] Telemetry basic counters (frame_ms)
- [ ] param_store dynamic insertion / retrieval
- [ ] SPH neighbor search baseline
- [ ] Coupling drag strategy
- [ ] Task pool scheduler fixed partition
- [ ] Deterministic reduction tree
- [ ] Plugin sample registration
- [ ] Energy audit instrumentation
- [ ] SIMD PBD inner loop
- [ ] GPU stub fallback
- [ ] Replay harness & perf baseline

---
## Updating This Roadmap
- Add new phases only at end unless re-baselining pre‑1.0.
- Mark completed phases with date & short retro summary.

(End of ROADMAP)

