#include "solver_internal.h"

namespace sim {
    bool shell_validate(const BuildDesc& d);
    void shell_translate(BuildDesc& d);
    void shell_pack(BuildDesc& d);
    void shell_cache_track_begin(const BuildDesc& d);
    void shell_cache_track_end();

    static Chosen choose_from_policy(const BuildDesc& d) {
        Chosen c;
        c.layout  = d.policy.exec.layout == DataLayout::Auto ? DataLayout::SoA : d.policy.exec.layout;
        c.backend = d.policy.exec.backend == Backend::Auto ? Backend::Native : d.policy.exec.backend;
        c.threads = d.policy.exec.threads < 0 ? 1 : d.policy.exec.threads;
        return c;
    }

    Result<Solver*> create(const BuildDesc& desc) {
        BuildDesc cfg = desc;
        if (!shell_validate(cfg)) {
            Result<Solver*> r;
            r.status = Status::ValidationFailed;
            r.value  = nullptr;
            return r;
        }
        shell_cache_track_begin(cfg);
        shell_translate(cfg);
        shell_pack(cfg);
        Solver* s = new(std::nothrow) Solver();
        if (!s) {
            shell_cache_track_end();
            Result<Solver*> r;
            r.status = Status::OOM;
            r.value  = nullptr;
            return r;
        }
        s->e = engine_create(cfg);
        shell_cache_track_end();
        if (!s->e) {
            delete s;
            Result<Solver*> r;
            r.status = Status::NoBackend;
            r.value  = nullptr;
            return r;
        }
        s->tf.step_ms             = 0.0;
        s->tf.residual_avg        = 0.0;
        s->tf.last_rebuild_ms     = 0.0;
        s->tf.avg_rebuild_ms      = 0.0;
        s->tf.commands_applied    = 0;
        s->tf.structural_rebuilds = 0;
        s->tf.solve_substeps      = cfg.policy.solve.substeps > 0 ? cfg.policy.solve.substeps : 1;
        s->tf.solve_iterations    = cfg.policy.solve.iterations > 0 ? cfg.policy.solve.iterations : 8;
        Chosen actual{};
        if (engine_query_chosen(s->e, actual)) s->chosen = actual;
        else s->chosen = choose_from_policy(cfg);
        s->applied                = 0;
        s->rebuilds               = 0;
        Result<Solver*> r;
        r.status = Status::Ok;
        r.value  = s;
        return r;
    }

    void destroy(Solver* s) {
        if (!s) return;
        engine_destroy(s->e);
        delete s;
    }

    Result<Chosen> query_chosen(Solver* s) {
        Result<Chosen> r;
        if (!s) {
            r.status = Status::InvalidArgs;
            r.value  = Chosen{DataLayout::SoA, Backend::Native, 1};
            return r;
        }
        r.status = Status::Ok;
        r.value  = s->chosen;
        return r;
    }

    Status telemetry_query_frame(Solver* s, TelemetryFrame* out) {
        if (!s || !out) return Status::InvalidArgs;
        *out = s->tf;
        return Status::Ok;
    }
}
