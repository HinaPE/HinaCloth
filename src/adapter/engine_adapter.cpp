#include "engine_adapter.h"

#include "api/sim.h"
#include "core/data/data.h"
#include "core/model/model.h"
#include <cstddef>
#include <new>

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
        // Propagate chosen backend/layout into Data
        if (d) {
            d->exec_use_avx2       = (ch.backend == Backend::AVX2);
            d->exec_use_tbb        = (ch.backend == Backend::TBB);
            d->exec_threads        = (ch.threads <= 0) ? -1 : ch.threads;
            d->exec_layout_blocked = (ch.layout == DataLayout::Blocked);
            // Ensure AoSoA buffer sized if needed
            if (d->exec_layout_blocked) {
                unsigned int blk = d->layout_block_size > 0 ? d->layout_block_size : (m->layout_block_size > 0 ? m->layout_block_size : 8u);
                d->layout_block_size = blk;
                std::size_t n = d->px.size();
                std::size_t nb = (n + (std::size_t)blk - 1) / (std::size_t)blk;
                d->pos_aosoa.assign(3u * (std::size_t)blk * nb, 0.0f);
            }
        }
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
        // Stage 4: remap robustness - resize constraint state to new edge count and clear invalid lambdas
        if (e->data) {
            std::size_t ecount = e->model ? (e->model->edges.size() / 2) : 0u;
            e->data->lambda_edge.assign(ecount, 0.0f);
            // Ensure AoSoA buffer consistent with layout
            if (e->data->exec_layout_blocked) {
                unsigned int blk = e->data->layout_block_size > 0 ? e->data->layout_block_size : (e->model && e->model->layout_block_size ? e->model->layout_block_size : 8u);
                e->data->layout_block_size = blk;
                std::size_t n = e->data->px.size();
                std::size_t nb = (n + (std::size_t)blk - 1) / (std::size_t)blk;
                e->data->pos_aosoa.assign(3u * (std::size_t)blk * nb, 0.0f);
            }
        }
        e->rebuilds += 1ull;
        e->applied += (unsigned long long) count;
        return Status::Ok;
    }

    Status engine_step(EngineHandle* e, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        if (!e || !e->data || !e->model) return Status::InvalidArgs;
        return runtime_step(*e->model, *e->data, dt, ovr, out);
    }

    bool engine_query_chosen(EngineHandle* e, Chosen& out) {
        if (!e) return false;
        out = e->chosen;
        return true;
    }
}
