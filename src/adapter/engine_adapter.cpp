#include "engine_adapter.h"
#include "api/sim.h"
#include <new>
#include <cstddef>

namespace sim {
    struct Model;
    struct Data;
    struct RemapPlan;
    struct Solver;
    bool cooking_build_model(const BuildDesc& in, Model*& out);
    bool cooking_rebuild_model_from_commands(const Model& cur, const Command* cmds, size_t count, Model*& out, RemapPlan*& plan);
    bool core_data_create_from_state(const BuildDesc& in, const Model& m, Data*& out);
    bool core_data_apply_overrides(Data& d, const Command* cmds, size_t count);
    bool core_data_apply_remap(const Data& oldd, const RemapPlan& plan, Data*& newd);
    void core_model_destroy(Model* m);
    void core_data_destroy(Data* d);
    void core_remapplan_destroy(RemapPlan* p);
    bool backends_choose(const Model& m, const PolicyExec& exec, struct Chosen& out);
    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out);

    struct EngineHandle {
        Model* model;
        Data* data;
        Chosen chosen;
        int threads;
        unsigned long long applied;
        unsigned long long rebuilds;
    };

    static Status fail(Status s) {
        return s;
    }

    EngineHandle* engine_create(const BuildDesc& desc) {
        EngineHandle* e = new(std::nothrow) EngineHandle();
        if (!e) return nullptr;
        e->model    = nullptr;
        e->data     = nullptr;
        e->threads  = desc.policy.exec.threads < 0 ? 1 : desc.policy.exec.threads;
        e->applied  = 0;
        e->rebuilds = 0;
        Model* m    = nullptr;
        if (!cooking_build_model(desc, m)) {
            delete e;
            return nullptr;
        }
        Data* d = nullptr;
        if (!core_data_create_from_state(desc, *m, d)) {
            core_model_destroy(m);
            delete e;
            return nullptr;
        }
        e->model = m;
        e->data  = d;
        Chosen ch{};
        if (!backends_choose(*m, desc.policy.exec, ch)) {
            core_data_destroy(d);
            core_model_destroy(m);
            delete e;
            return nullptr;
        }
        e->chosen = ch;
        return e;
    }

    void engine_destroy(EngineHandle* e) {
        if (!e) return;
        if (e->data) core_data_destroy(e->data);
        if (e->model) core_model_destroy(e->model);
        delete e;
    }

    Status engine_apply_small_params(EngineHandle* e, const Command* cmds, size_t count) {
        if (!e || !e->data) return Status::InvalidArgs;
        if (count == 0) return Status::Ok;
        if (!core_data_apply_overrides(*e->data, cmds, count)) return Status::ValidationFailed;
        e->applied += (unsigned long long) count;
        return Status::Ok;
    }

    Status engine_apply_structural_changes(EngineHandle* e, const Command* cmds, size_t count) {
        if (!e || !e->data || !e->model) return Status::InvalidArgs;
        Model* nm       = nullptr;
        RemapPlan* plan = nullptr;
        if (!cooking_rebuild_model_from_commands(*e->model, cmds, count, nm, plan)) return Status::ValidationFailed;
        Data* nd = nullptr;
        bool ok  = core_data_apply_remap(*e->data, *plan, nd);
        if (!ok) {
            core_model_destroy(nm);
            core_remapplan_destroy(plan);
            return Status::ValidationFailed;
        }
        core_data_destroy(e->data);
        core_model_destroy(e->model);
        core_remapplan_destroy(plan);
        e->model = nm;
        e->data  = nd;
        e->rebuilds += 1ull;
        e->applied += (unsigned long long) count;
        return Status::Ok;
    }

    Status engine_step(EngineHandle* e, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        if (!e || !e->data || !e->model) return Status::InvalidArgs;
        return runtime_step(*e->model, *e->data, dt, ovr, out);
    }
}
