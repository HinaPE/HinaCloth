# Research-Oriented Highly Decoupled Physics Framework

> Version: Initial Design Report (Fresh Architecture – legacy ignored)
> Audience: Researchers / Students / Rapid Prototypers of Physical Simulation Algorithms (Cloth, Fluid, Gas, Rigid, etc.)
> Goal: A minimal, stable, scriptable public API with maximally decoupled internal domains & algorithms enabling fast A/B experimentation, algorithm proliferation, and isolated coupling modules.

---
## 1. Design Vision
| Aspect | Intent |
|--------|--------|
| Audience | Rapid algorithm research & teaching (not a monolithic production engine) |
| Stability Surface | Very small: world / domain / algorithm / params / fields / coupling / telemetry APIs |
| Isolation | Domains (cloth, fluid, gas, rigid, …) never include each other; algorithms inside a domain are side-by-side & disposable |
| Coupling | Always external modules; failure isolated; explicit field contract (pull/push) |
| Extensibility | Add algorithm / domain / scheduler / perf plugin by dropping a folder + registration code |
| Data Sharing | Minimal: field_bus (named spans), param_store (heterogeneous key/value) |
| Performance Layers | Optional (layout packing, SIMD, GPU, cache acceleration) – never required for correctness |
| Determinism | Dialable (fast → bit-stable); no hidden global states |
| Experimentation | Sandbox utilities for param sweeps, diffing field histories, algorithm comparison |

---
## 2. Core Principles
1. **Separation by Domain**: Each physical domain owns its lifecycle & algorithms; no implicit cross-domain state.
2. **Algorithm Flatness**: New algorithms are peers; no inheritance webs; share only through a local `shared/` subfolder (opt-in extraction).
3. **Contracts over Giant Models**: Domain pipeline contract = standardized init/step callbacks; coupling contract = declared field IO.
4. **Late Binding**: Choose algorithms & schedulers at runtime via API (string / id) – minimal compile wall.
5. **Fail Softly**: Failing coupling or perf plugin must not corrupt or stall core domain stepping.
6. **Pluggable Performance**: Baseline correctness path never depends on vectorization / GPU / caching modules.
7. **Observability First**: Telemetry aggregated centrally; explicit pull patterns; reproducible experiment matrices.
8. **Strict Allowed Dependencies**: Enforced by script – minimal surface prevents structural rot.

---
## 3. High-Level Module Overview
```mermaid
flowchart LR
    classDef api fill:#0d47a1,stroke:#062b5f,color:#ffffff,font-weight:bold;
    classDef core fill:#263238,stroke:#455a64,color:#fafafa,font-weight:bold;
    classDef domain fill:#283593,stroke:#1a237e,color:#ffffff;font-weight:bold;
    classDef algo fill:#2e7d32,stroke:#1b5e20,color:#e8f5e9;
    classDef couple fill:#6a1b9a,stroke:#4a148c,color:#f3e5f5;
    classDef sched fill:#00796b,stroke:#004d40,color:#ffffff;
    classDef perf fill:#546e7a,stroke:#37474f,color:#eceff1;
    classDef sand fill:#5d4037,stroke:#3e2723,color:#ffffff;
    API[public_api]:::api --> GW[api_gateway]:::core
    GW --> CORE[core_base]:::core
    CORE --> CLOTH[domain_cloth]:::domain
    CORE --> FLUID[domain_fluid]:::domain
    CORE --> GAS[domain_gas]:::domain
    CORE --> RIGID[domain_rigid]:::domain
    CLOTH --> C_ALGOS[cloth_algorithms]:::algo
    FLUID --> F_ALGOS[fluid_algorithms]:::algo
    GAS --> G_ALGOS[gas_algorithms]:::algo
    RIGID --> R_ALGOS[rigid_algorithms]:::algo
    GW --> SCHED[schedulers]:::sched
    GW --> PERF[perf_layers]:::perf
    GW --> COUP[coupling_modules]:::couple
    API --> SANDBOX[sandbox_tools]:::sand
```

---
## 4. Directory Layout (Target)
```
/include/rphys/      # Stable public API (only surface that external code touches)
  api_world.h api_domain.h api_algorithm.h api_scene.h
  api_params.h api_fields.h api_coupling.h api_events.h api_commands.h
  api_telemetry.h api_capability.h api_status.h api_version.h api_ids.h forward.h
/src/
  core_base/         # Stable minimal kernel: world, domain, algorithm, field_bus, param_store, telemetry
  api_layer/         # Gateway translating public handles to core refs
  domain_cloth/      # Independent domain (pipeline_contract + algorithms + shared)
  domain_fluid/
  domain_gas/
  domain_rigid/
  domain_new_template/  # Onboarding scaffold for new domain
  coupling_modules/     # External interaction strategies (cloth_fluid_*, cloth_rigid_*, generic mixes)
  schedulers/           # serial / task_pool / job_system_stub / gpu_stub / scheduler_*
  perf_layers/          # layout_pack / simd_vec / gpu_backend / cache_accel / future_opt_*
  algo_sandbox/         # run_matrix / diff_fields / param_scan_tool
  telemetry_export/     # json_dump / csv_dump
  plugins/              # sample_plugin_register / plugin_*
```

---
## 5. Public API Surface (Minimal & Stable)
| Header | Purpose | Stability |
|--------|---------|-----------|
| api_world.h | create/destroy/step worlds | Stable |
| api_domain.h | add/remove domain instances | Stable |
| api_algorithm.h | register/list/select algorithms per domain | Stable |
| api_scene.h | build minimal primitives (mesh grid, particle box) | Stable |
| api_params.h | set/get typed or generic parameters | Stable |
| api_fields.h | pull/push field snapshots by name | Stable |
| api_coupling.h | add/remove coupling modules | Stable |
| api_events.h | schedule structural/runtime events | Stable |
| api_commands.h | frame-level small param override queue | Stable |
| api_telemetry.h | per-frame & per-algorithm timing / counters | Stable |
| api_status.h | status enumeration | Stable |
| api_capability.h | introspect registered algorithms/schedulers/perf features | Stable |
| api_version.h | version & schema metadata | Stable |
| api_ids.h | opaque id types (WorldId, DomainId, AlgorithmId, CouplingId) | Stable |

---
## 6. Core Internal Contracts
### Domain Pipeline Contract (Essential Hooks)
```cpp
struct DomainPipelineContract {
  bool (*init)(DomainContext&, const ParamStore&);
  bool (*build_static)(DomainContext&, const ScenePrimitiveList&);
  bool (*on_algorithm_bind)(DomainContext&, const AlgorithmDescriptor&);
  bool (*step_prepare)(DomainContext&, StepContext&);
  bool (*step_solve)(DomainContext&, StepContext&);
  bool (*step_finalize)(DomainContext&, StepContext&);
  bool (*export_fields)(DomainContext&, FieldBus&);
  bool (*apply_event)(DomainContext&, const Event&);
  bool (*shutdown)(DomainContext&);
};
```
### Coupling Contract
```cpp
struct CouplingContract {
  const char* const* required_inputs; size_t required_input_count;
  const char* const* produced_outputs; size_t produced_output_count;
  bool (*initialize)(CouplingContext&, const ParamStore&);
  bool (*exchange)(CouplingContext&, FieldBus&, StepContext&);
  bool (*shutdown)(CouplingContext&);
};
```

---
## 7. Field & Parameter Abstractions
| Layer | Role | Notes |
|-------|------|-------|
| field_bus | Named ephemeral views (name + ptr + count + stride) | No ownership, read/write negotiated |
| param_store | Heterogeneous typed + string map | Introspectable for GUI / scripts |
| exported fields | Must be stable over one frame | Coupling reads after `step_finalize` |
| algorithm local storage | Free-form | NOT exposed outside domain |

Field naming conventions (suggested):
```
cloth.position, cloth.velocity, cloth.mass, cloth.lambda_xx
fluid.position, fluid.velocity, fluid.density, fluid.pressure
rigid.transform, rigid.linear_velocity, rigid.angular_velocity
```

---
## 8. Dependency Rules (Enforced)
Allowed (examples):
```
api_layer -> core_base
core_base -> (none upward)
domain_X/algorithms/* -> domain_X/pipeline_contract, param_store, field_bus, domain_X/shared/*
coupling_modules/* -> field_bus (never algorithms)
schedulers/* -> domain_core (never algorithms)
perf_layers/* -> field_bus (optional)
plugins/* -> algo_core (registration only)
```
Forbidden:
- domain_A including domain_B
- algorithm_A referencing algorithm_B
- coupling module including algorithm headers
- perf_layers forcing mandatory include in core path

---
## 9. Extension Points
| Category | Pattern | Plug-In Mechanism |
|----------|---------|-------------------|
| Algorithm | `domain_<d>/algorithms/<algo_name>` | Register via `api_algorithm` or plugin module |
| New Domain | copy `domain_new_template` | Provide pipeline_contract implementation |
| Coupling Strategy | `coupling_modules/<a>_<b>_*` | Register via `api_coupling` |
| Scheduler | `schedulers/scheduler_*` | Register at world init / capability query |
| Perf Optimization | `perf_layers/future_opt_*` | Capability gated (can be off) |
| External Plugin | `plugins/plugin_*` | Dynamic or static registration |

---
## 10. Determinism Levels
| Level | Guarantees | Techniques |
|-------|------------|------------|
| 0 Fast | No ordering guarantee | Direct iteration |
| 1 TaskStable | Stable phase & batch order | Deterministic partition & fixed queue |
| 2 BitStable | Bitwise across runs (same HW) | Fixed reduction tree, consistent FP flags, deterministic RNG seeds |
| 3 CrossHW | Bitwise across HW families | Emulated precision paths, disable FMA fusion, canonical rounding |

---
## 11. Telemetry & Experimentation
| Channel | Data | Use |
|---------|------|-----|
| Frame stats | frame_ms, per-domain_ms, per-algo_ms | Quick perf view |
| Iteration stats | substeps, iterations, solver residual | Convergence tuning |
| Coupling stats | coupling_exchange_ms, data_volume | Diagnose exchange cost |
| Sandbox outputs | matrix run table (algo × param sets) | Automated A/B sweeps |

Sandbox utilities:
- `run_matrix`: batch test matrix (algorithm × iterations × dt)
- `diff_fields`: produce norm / element diff between algorithm outputs
- `param_scan_tool`: automatic sweep for stability & performance maps

---
## 12. Error Handling & Robustness
| Source | Strategy |
|--------|----------|
| Invalid parameter | Return Status::InvalidArgs, no partial mutate |
| Failed build_static | Domain enters inert state; removable |
| Coupling failure | Log + detach coupling; domains continue |
| Plugin registration conflict | Reject new id; keep first registered |
| Telemetry overflow | Drop oldest samples (ring buffer) |

---
## 13. Security / Safety (Minimal)
- No hidden global mutable state (except controlled registry).
- All exported pointers const or size-bounded.
- No cross-domain memory writes outside coupling sanctioned outputs.

---
## 14. Minimal Implementation Phases (Summary)
See `ROADMAP.md` for deep detail.
| Phase | Focus | Deliverable |
|-------|-------|-------------|
| 0 | Skeleton (world + cloth PBD) | Step loop + field export |
| 1 | Algorithm proliferation (cloth XPBD) | A/B compare & telemetry |
| 2 | field_bus + param_store maturity | Sandbox scans |
| 3 | Fluid SPH domain | Parallel but uncoupled |
| 4 | Cloth–Fluid coupling strategies | Drag + pressure |
| 5 | Perf layer (layout_pack + task_pool) | Speedup baseline |
| 6 | Additional algorithms (FEM, MPM, Rigid) | Multi-domain runs |
| 7 | Determinism L2 + extended telemetry | Reproducibility harness |
| 8 | Plugin & extension docs | External contributions |

---
## 15. Full Dependency (Per-File Logical) Diagram
_For the authoritative, fine-grained dependency map (header+cpp merged) refer to `proposed_final_deps.md`. Below is a condensed version._
```mermaid
flowchart LR
    classDef a fill:#0d47a1,stroke:#062b5f,color:#fff,font-weight:bold;
    classDef b fill:#263238,stroke:#455a64,color:#fafafa,font-weight:bold;
    classDef c fill:#283593,stroke:#1a237e,color:#fff;font-weight:bold;
    classDef d fill:#2e7d32,stroke:#1b5e20,color:#e8f5e9;
    classDef e fill:#6a1b9a,stroke:#4a148c,color:#f3e5f5;
    classDef f fill:#00796b,stroke:#004d40,color:#fff;
    classDef g fill:#546e7a,stroke:#37474f,color:#eceff1;
    classDef h fill:#5d4037,stroke:#3e2723,color:#fff;
    API_MIN[api]:::a --> GATE[gateway]:::b --> CORE_MIN[core_base]:::b
    CORE_MIN --> CLOTH_MIN[cloth]:::c
    CORE_MIN --> FLUID_MIN[fluid]:::c
    CORE_MIN --> RIGID_MIN[rigid]:::c
    CORE_MIN --> GAS_MIN[gas]:::c
    CLOTH_MIN --> C_ALGS[cloth_algos]:::d
    FLUID_MIN --> F_ALGS[fluid_algos]:::d
    RIGID_MIN --> R_ALGS[rigid_algos]:::d
    GAS_MIN --> G_ALGS[gas_algos]:::d
    GATE --> COUP_MIN[coupling]:::e
    GATE --> SCHED_MIN[schedulers]:::f
    GATE --> PERF_MIN[perf]:::g
    API_MIN --> SANDBOX_MIN[sandbox]:::h
```

---
## 16. Contribution Guide (Brief)
1. **Add Algorithm**: Copy template folder → implement contract hooks → register via `api_algorithm`.  
2. **Add Domain**: Copy `domain_new_template`; supply pipeline_contract + at least one algorithm.  
3. **Add Coupling**: New folder in `coupling_modules/`; implement contract; register by name.  
4. **Add Scheduler**: New folder in `schedulers/`; implement `invoke(domain_core, phase_batches)`; register capability.  
5. **Perf Plugin**: New folder in `perf_layers/`; expose `apply(field_bus_view)`; guard behind capability flag.  
6. **Telemetry Field**: Extend telemetry_core schema + bump schema version (document in CHANGELOG).  
7. **Propose Change**: Open design note in `/docs/` with rationale & migration snippet.  

Coding guidelines (essentials):
- No cross-domain includes.  
- No direct algorithm → coupling module includes.  
- All new public API functions require version minor bump & README diff.  

---
## 17. Glossary
| Term | Definition |
|------|------------|
| Domain | Self-contained physical system (cloth, fluid…) |
| Algorithm Variant | One implementation strategy inside a domain |
| Coupling Module | External mediator exchanging fields between domains |
| field_bus | Runtime registry of transient field views |
| param_store | Reflective parameter container |
| Sandbox | Tools for automated comparative experiments |
| Determinism Level | Configured reproducibility tier |

---
## 18. FAQ (Selected)
| Q | A |
|---|---|
| Why not an ECS? | Increases abstraction overhead; local contexts are enough. |
| Why no global solver? | Encourages modular A/B and per-domain reasoning. |
| How to add GPU? | Implement perf layer (gpu_backend) + optional scheduler_gpu_stub. |
| How to extend telemetry? | Add counter/timer in telemetry_core + expose via api_telemetry. |
| How to compare algorithms? | Use sandbox `run_matrix` + `diff_fields` + param scan. |

---
## 19. Related Files
| File | Purpose |
|------|--------|
| `proposed_final.md` | Narrative architecture & contracts (full) |
| `proposed_final_deps.md` | Fine-grained dependency Mermaid graph |
| `ROADMAP.md` | Detailed phased implementation plan |

---
## 20. License / Attribution
(Choose license before publishing – MIT / Apache-2 recommended for research frameworks.)

---
**End of Design Report**

