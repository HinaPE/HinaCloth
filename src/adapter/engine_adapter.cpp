#include "engine_adapter.h"

#include "api/chosen.h"
#include "core/common/types.h"
#include "core/data/data.h"
#include "core/model/model.h"
#include "cooking/cooking.h"
#include "backend/registry/registry.h"
#include "runtime/step_eng.h"
#include <cstddef>
#include <new>
#include <cstdint>
#include <vector>
#include <memory>

namespace sim {
    [[nodiscard]] bool shell_cache_query(uint64_t& key_out);
    [[nodiscard]] bool shell_cache_load(uint64_t key, Model*& out);
    void shell_cache_store(uint64_t key, const Model& m);
}

namespace sim {
    using namespace eng;

    static inline ::sim::Status to_api_status(eng::Status s) {
        switch (s) {
            case eng::Status::Ok:              return ::sim::Status::Ok;
            case eng::Status::InvalidArgs:     return ::sim::Status::InvalidArgs;
            case eng::Status::ValidationFailed:return ::sim::Status::ValidationFailed;
            case eng::Status::NoBackend:       return ::sim::Status::NoBackend;
            case eng::Status::Unsupported:     return ::sim::Status::Unsupported;
            case eng::Status::OOM:             return ::sim::Status::OOM;
            case eng::Status::NotReady:        return ::sim::Status::NotReady;
            case eng::Status::Busy:            return ::sim::Status::Busy;
            default:                           return ::sim::Status::Unsupported;
        }
    }

    static inline eng::PolicyExec map_policy_exec(const ::sim::PolicyExec& a) {
        eng::PolicyExec e{}; e.layout = (eng::DataLayout)a.layout; e.backend = (eng::Backend)a.backend; e.threads = a.threads; e.deterministic = a.deterministic; e.telemetry = a.telemetry; return e;
    }
    static inline eng::PolicySolve map_policy_solve(const ::sim::PolicySolve& a) {
        eng::PolicySolve e{}; e.substeps = a.substeps; e.iterations = a.iterations; e.damping = a.damping; e.stepper = (eng::TimeStepper)a.stepper; return e;
    }
    static inline eng::BuildDesc map_build_desc(const ::sim::BuildDesc& a) {
        eng::BuildDesc b{};
        b.state.fields = reinterpret_cast<const eng::FieldView*>(a.state.fields); b.state.field_count = a.state.field_count;
        b.params.items = reinterpret_cast<const eng::Param*>(a.params.items); b.params.count = a.params.count;
        b.topo.node_count = a.topo.node_count; b.topo.relations = reinterpret_cast<const eng::RelationView*>(a.topo.relations); b.topo.relation_count = a.topo.relation_count;
        b.policy.exec = map_policy_exec(a.policy.exec);
        b.policy.solve = map_policy_solve(a.policy.solve);
        b.space = {}; b.ops = {}; b.events = {};
        b.validate = (eng::ValidateLevel)a.validate;
        b.pack.lazy_pack = a.pack.lazy_pack; b.pack.block_size = a.pack.block_size;
        return b;
    }
    static inline void map_chosen_to_api(const eng::Chosen& e, ::sim::Chosen& o) {
        o.layout = (::sim::DataLayout) e.layout; o.backend = (::sim::Backend) e.backend; o.threads = e.threads;
    }
    static inline void map_commands_to_eng(const ::sim::Command* in, size_t count, std::vector<eng::Command>& out) {
        out.resize(count);
        for (size_t i = 0; i < count; ++i) { out[i].tag = (eng::CommandTag) in[i].tag; out[i].data = in[i].data; out[i].bytes = in[i].bytes; }
    }

    struct EngineHandle {
        Model* model{};
        Data* data{};
        eng::Chosen chosen{};
        int threads{};
        unsigned long long applied{};
        unsigned long long rebuilds{};
    };

    EngineHandle* engine_create(const ::sim::BuildDesc& desc) {
        std::unique_ptr<EngineHandle> e{new(std::nothrow) EngineHandle()};
        if (!e) return nullptr;
        e->model    = nullptr;
        e->data     = nullptr;
        e->threads  = desc.policy.exec.threads < 0 ? 1 : desc.policy.exec.threads;
        e->applied  = 0;
        e->rebuilds = 0;

        Model* m_raw = nullptr;
        uint64_t key = 0;
        bool have_key = shell_cache_query(key);
        if (have_key) {
            if (!shell_cache_load(key, m_raw)) {
                eng::BuildDesc bd = map_build_desc(desc);
                if (!cooking_build_model(bd, m_raw)) {
                    return nullptr;
                }
                // m_raw is valid; store into cache below
                shell_cache_store(key, *m_raw);
            }
        } else {
            eng::BuildDesc bd = map_build_desc(desc);
            if (!cooking_build_model(bd, m_raw)) {
                return nullptr;
            }
        }
        std::unique_ptr<Model, void(*)(Model*)> m{m_raw, core_model_destroy};

        Data* d_raw = nullptr;
        eng::BuildDesc bd = map_build_desc(desc);
        if (!core_data_create_from_state(bd, *m, d_raw)) {
            return nullptr;
        }
        std::unique_ptr<Data, void(*)(Data*)> d{d_raw, core_data_destroy};

        eng::Chosen ch{};
        if (!eng::backends_choose(*m, bd.policy.exec, ch)) {
            return nullptr;
        }

        e->model = m.release();
        e->data  = d.release();
        e->chosen = ch;

        if (e->data) {
            e->data->exec_use_avx2       = (ch.backend == eng::Backend::AVX2);
            e->data->exec_use_tbb        = (ch.backend == eng::Backend::TBB);
            e->data->exec_threads        = (ch.threads <= 0) ? -1 : ch.threads;
            e->data->exec_layout_blocked = (ch.layout == eng::DataLayout::Blocked);
            e->data->exec_layout_aos     = (ch.layout == eng::DataLayout::AoS);
            if (e->data->exec_layout_blocked) {
                unsigned int blk = e->data->layout_block_size > 0 ? e->data->layout_block_size : (e->model->layout_block_size > 0 ? e->model->layout_block_size : 8u);
                e->data->layout_block_size = blk;
                std::size_t n = e->data->px.size();
                std::size_t nb = (n + (std::size_t)blk - 1) / (std::size_t)blk;
                e->data->pos_aosoa.assign(3u * (std::size_t)blk * nb, 0.0f);
            }
            if (e->data->exec_layout_aos) {
                e->data->layout_aos_stride = 3u;
                std::size_t n = e->data->px.size();
                e->data->pos_aos.assign((std::size_t)e->data->layout_aos_stride * n, 0.0f);
            }
        }
        return e.release();
    }

    void engine_destroy(EngineHandle* e) noexcept {
        if (!e) return;
        if (e->data) core_data_destroy(e->data);
        if (e->model) core_model_destroy(e->model);
        delete e;
    }

    ::sim::Status engine_apply_small_params(EngineHandle* e, const ::sim::Command* cmds, size_t count) {
        if (!e || !e->data) return ::sim::Status::InvalidArgs;
        if (count == 0) return ::sim::Status::Ok;
        std::vector<eng::Command> buf; map_commands_to_eng(cmds, count, buf);
        if (!core_data_apply_overrides(*e->data, buf.data(), buf.size())) return ::sim::Status::ValidationFailed;
        e->applied += (unsigned long long) count;
        return ::sim::Status::Ok;
    }

    ::sim::Status engine_apply_structural_changes(EngineHandle* e, const ::sim::Command* cmds, size_t count) {
        if (!e || !e->data || !e->model) return ::sim::Status::InvalidArgs;
        Model* nm_raw       = nullptr;
        RemapPlan* plan_raw = nullptr;
        std::vector<eng::Command> buf; map_commands_to_eng(cmds, count, buf);
        if (!cooking_rebuild_model_from_commands(*e->model, buf.data(), buf.size(), nm_raw, plan_raw)) return ::sim::Status::ValidationFailed;
        std::unique_ptr<Model, void(*)(Model*)> nm{nm_raw, core_model_destroy};
        std::unique_ptr<RemapPlan, void(*)(RemapPlan*)> plan{plan_raw, core_remapplan_destroy};

        Data* nd_raw = nullptr;
        bool ok  = core_data_apply_remap(*e->data, *plan, nd_raw);
        if (!ok) {
            return ::sim::Status::ValidationFailed;
        }
        std::unique_ptr<Data, void(*)(Data*)> nd{nd_raw, core_data_destroy};

        // Replace
        if (e->data) core_data_destroy(e->data);
        if (e->model) core_model_destroy(e->model);
        e->model = nm.release();
        e->data  = nd.release();
        // plan is not needed beyond this point

        if (e->data) {
            std::size_t ecount = e->model ? (e->model->edges.size() / 2) : 0u;
            e->data->lambda_edge.assign(ecount, 0.0f);
            e->data->distance_alpha_edge.assign(ecount, 0.0f);
            e->data->distance_compliance_edge.assign(ecount, 0.0f);
            if (e->data->exec_layout_blocked) {
                unsigned int blk = e->data->layout_block_size > 0 ? e->data->layout_block_size : (e->model && e->model->layout_block_size ? e->model->layout_block_size : 8u);
                e->data->layout_block_size = blk;
                std::size_t n = e->data->px.size();
                std::size_t nb = (n + (std::size_t)blk - 1) / (std::size_t)blk;
                e->data->pos_aosoa.assign(3u * (std::size_t)blk * nb, 0.0f);
            }
            if (e->data->exec_layout_aos) {
                e->data->layout_aos_stride = 3u;
                std::size_t n = e->data->px.size();
                e->data->pos_aos.assign((std::size_t)e->data->layout_aos_stride * n, 0.0f);
            }
        }
        e->rebuilds += 1ull;
        e->applied += (unsigned long long) count;
        return ::sim::Status::Ok;
    }

    ::sim::Status engine_step(EngineHandle* e, float dt, const ::sim::SolveOverrides* ovr_api, ::sim::TelemetryFrame* out_api) {
        if (!e || !e->data || !e->model) return ::sim::Status::InvalidArgs;
        eng::SolveOverrides ovr{}; if (ovr_api) { ovr.substeps_override = ovr_api->substeps_override; ovr.iterations_override = ovr_api->iterations_override; }
        eng::TelemetryFrame out{}; eng::TelemetryFrame* pout = out_api ? &out : nullptr;
        auto st = eng::runtime_step(*e->model, *e->data, dt, ovr_api ? &ovr : nullptr, pout);
        if (out_api && pout) {
            out_api->step_ms = out.step_ms; out_api->residual_avg = out.residual_avg; out_api->last_rebuild_ms = out.last_rebuild_ms; out_api->avg_rebuild_ms = out.avg_rebuild_ms;
            out_api->commands_applied = out.commands_applied; out_api->structural_rebuilds = out.structural_rebuilds; out_api->solve_substeps = out.solve_substeps; out_api->solve_iterations = out.solve_iterations;
        }
        return to_api_status(st);
    }

    bool engine_query_chosen(EngineHandle* e, ::sim::Chosen& out) {
        if (!e) return false;
        map_chosen_to_api(e->chosen, out);
        return true;
    }

    ::sim::Status engine_copy_positions(EngineHandle* e, float* dst, size_t maxCount, size_t* outCount) {
        if (!e || !e->data || !dst) return ::sim::Status::InvalidArgs;
        const Data& d = *e->data;
        size_t n = d.x.size();
        size_t cnt = (maxCount == 0 || maxCount > n) ? n : maxCount;
        for (size_t i = 0; i < cnt; ++i) {
            dst[3*i + 0] = d.x[i];
            dst[3*i + 1] = d.y[i];
            dst[3*i + 2] = d.z[i];
        }
        if (outCount) *outCount = cnt;
        return ::sim::Status::Ok;
    }
}
