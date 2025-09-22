#include "step.h"
#include "runtime/step_eng.h"
#include "core/model/model.h"
#include "core/data/data.h"
#include "backend/storage/soa.h"
#include "backend/storage/aosoa.h"
#include "backend/scheduler/seq.h"
#include "backend/kernel/constraints/distance.h"
#include "backend/kernel/constraints/distance_avx2.h"
#include "backend/kernel/constraints/distance_aosoa.h"
#include "backend/kernel/constraints/attachment.h"
#include "backend/kernel/constraints/bending.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#ifdef HINACLOTH_HAVE_TBB
#include <tbb/parallel_for.h>
#include <tbb/global_control.h>
#endif

namespace sim { namespace eng {
    using ::sim::Model; using ::sim::Data;

    static void integrate_pred(Data& d, float dt) {
        std::size_t n = d.x.size();
        for (std::size_t i = 0; i < n; i++) {
            if (d.inv_mass.empty() || d.inv_mass[i] > 0.0f) {
                d.vx[i] += dt * d.gx;
                d.vy[i] += dt * d.gy;
                d.vz[i] += dt * d.gz;
                d.px[i] = d.x[i] + dt * d.vx[i];
                d.py[i] = d.y[i] + dt * d.vy[i];
                d.pz[i] = d.z[i] + dt * d.vz[i];
            } else {
                d.px[i] = d.x[i];
                d.py[i] = d.y[i];
                d.pz[i] = d.z[i];
                d.vx[i] = 0.0f; d.vy[i] = 0.0f; d.vz[i] = 0.0f;
            }
        }
    }

    static void presolve_apply_attachment(Data& d) {
        if (!d.op_enable_attachment) return;
        SoAView3 pos{}; storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        kernel_attachment_apply(pos,
                                d.attach_w.empty() ? nullptr : d.attach_w.data(),
                                d.attach_tx.empty() ? nullptr : d.attach_tx.data(),
                                d.attach_ty.empty() ? nullptr : d.attach_ty.data(),
                                d.attach_tz.empty() ? nullptr : d.attach_tz.data(),
                                d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                                d.px.size());
    }

    static void prepare_alpha_edge(const Model& m, Data& d, float dt_sub) {
        const std::size_t ecount = m.rest.size();
        if (ecount == 0) return;
        d.distance_alpha_edge.resize(ecount);
        const bool have_edge_comp = (d.distance_compliance_edge.size() == ecount);
        float inv_dt2 = 1.0f / (dt_sub * dt_sub);
        for (std::size_t e = 0; e < ecount; ++e) {
            float comp = have_edge_comp ? d.distance_compliance_edge[e] : d.distance_compliance;
            if (comp < 0.0f) comp = 0.0f;
            d.distance_alpha_edge[e] = comp * inv_dt2;
        }
    }

    static void project_distance_islands_soa(const Model& m, Data& d, float dt, int iterations) {
        SoAView3 pos{};
        storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        const float* alpha_edge = d.distance_alpha_edge.empty() ? nullptr : d.distance_alpha_edge.data();
        float alpha_scalar = std::max(0.0f, d.distance_compliance) / (dt * dt);
        const std::size_t island_count = m.island_offsets.empty() ? 1u : (std::size_t) m.island_offsets.size() - 1u;
        auto do_island = [&](std::size_t i) {
            std::size_t base = m.island_offsets.empty() ? 0u : (std::size_t) m.island_offsets[i];
            std::size_t cnt  = m.island_offsets.empty() ? (m.rest.size()) : (std::size_t) (m.island_offsets[i + 1] - m.island_offsets[i]);
            if (cnt == 0) return;
            const uint32_t* ebeg = m.edges.data() + 2 * base;
            const float*    rbeg = m.rest.data() + base;
            float*          lbeg = d.lambda_edge.empty() ? nullptr : (d.lambda_edge.data() + base);
        #if defined(HINACLOTH_HAVE_AVX2)
            if (d.exec_use_avx2) {
                kernel_distance_project_avx2(ebeg, cnt, pos, rbeg,
                                             d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                                             lbeg,
                                             alpha_edge ? (alpha_edge + base) : nullptr,
                                             iterations, alpha_scalar, dt);
                return;
            }
        #endif
            kernel_distance_project(ebeg, cnt, pos, rbeg,
                                    d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                                    lbeg,
                                    alpha_edge ? (alpha_edge + base) : nullptr,
                                    iterations, alpha_scalar, dt);
        };
        if (d.exec_use_tbb) {
        #ifdef HINACLOTH_HAVE_TBB
            if (d.exec_threads > 0) {
                tbb::global_control ctrl(tbb::global_control::max_allowed_parallelism, (std::size_t)d.exec_threads);
                tbb::parallel_for(std::size_t(0), island_count, [&](std::size_t i){ do_island(i); });
            } else {
                tbb::parallel_for(std::size_t(0), island_count, [&](std::size_t i){ do_island(i); });
            }
        #else
            for (std::size_t i = 0; i < island_count; ++i) do_island(i);
        #endif
        } else {
            for (std::size_t i = 0; i < island_count; ++i) do_island(i);
        }
    }

    static void project_distance_islands_aosoa(const Model& m, Data& d, float dt, int iterations) {
        AoSoAView3 posb{};
        storage_bind_aosoa(posb, d.pos_aosoa.data(), d.px.size(), (std::size_t) d.layout_block_size);
        const float* alpha_edge = d.distance_alpha_edge.empty() ? nullptr : d.distance_alpha_edge.data();
        float alpha_scalar = std::max(0.0f, d.distance_compliance) / (dt * dt);
        const std::size_t island_count = m.island_offsets.empty() ? 1u : (std::size_t) m.island_offsets.size() - 1u;
        auto do_island = [&](std::size_t i) {
            std::size_t base = m.island_offsets.empty() ? 0u : (std::size_t) m.island_offsets[i];
            std::size_t cnt  = m.island_offsets.empty() ? (m.rest.size()) : (std::size_t) (m.island_offsets[i + 1] - m.island_offsets[i]);
            if (cnt == 0) return;
            const uint32_t* ebeg = m.edges.data() + 2 * base;
            const float*    rbeg = m.rest.data() + base;
            float*          lbeg = d.lambda_edge.empty() ? nullptr : (d.lambda_edge.data() + base);
            kernel_distance_project_aosoa(ebeg, cnt, posb, rbeg,
                                          d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                                          lbeg,
                                          alpha_edge ? (alpha_edge + base) : nullptr,
                                          iterations, alpha_scalar, dt);
        };
        if (d.exec_use_tbb) {
        #ifdef HINACLOTH_HAVE_TBB
            if (d.exec_threads > 0) {
                tbb::global_control ctrl(tbb::global_control::max_allowed_parallelism, (std::size_t)d.exec_threads);
                tbb::parallel_for(std::size_t(0), island_count, [&](std::size_t i){ do_island(i); });
            } else {
                tbb::parallel_for(std::size_t(0), island_count, [&](std::size_t i){ do_island(i); });
            }
        #else
            for (std::size_t i = 0; i < island_count; ++i) do_island(i);
        #endif
        } else {
            for (std::size_t i = 0; i < island_count; ++i) do_island(i);
        }
    }

    static void bending_pass(const Model& m, Data& d, float dt, int iterations) {
        if (!d.op_enable_bending) return;
        if (m.bend_pairs.empty() || m.bend_rest_angle.empty()) return;
        SoAView3 pos{}; storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        const unsigned int* quads = reinterpret_cast<const unsigned int*>(m.bend_pairs.data());
        kernel_bending_project(quads, m.bend_rest_angle.size(), pos, m.bend_rest_angle.data(), d.inv_mass.empty() ? nullptr : d.inv_mass.data(), iterations, /*alpha*/0.0f, dt);
    }

    static void finalize(Data& d, float dt, float damping) {
        damping = std::clamp(damping, 0.0f, 1.0f);
        float vel_mul = (1.0f - damping);
        std::size_t n = d.x.size();
        for (std::size_t i = 0; i < n; i++) {
            if (d.inv_mass.empty() || d.inv_mass[i] > 0.0f) {
                float nx = d.px[i], ny = d.py[i], nz = d.pz[i];
                d.vx[i]  = (nx - d.x[i]) / dt * vel_mul;
                d.vy[i]  = (ny - d.y[i]) / dt * vel_mul;
                d.vz[i]  = (nz - d.z[i]) / dt * vel_mul;
                d.x[i]   = nx; d.y[i] = ny; d.z[i] = nz;
            } else {
                d.px[i] = d.x[i]; d.py[i] = d.y[i]; d.pz[i] = d.z[i];
                d.vx[i] = 0.0f; d.vy[i] = 0.0f; d.vz[i] = 0.0f;
            }
        }
    }

    static double compute_distance_residual(const Model& m, const Data& d) {
        const std::size_t ecount = m.rest.size();
        if (ecount == 0) return 0.0;
        double acc = 0.0;
        std::size_t valid = 0;
        for (std::size_t e = 0; e < ecount; ++e) {
            uint32_t a = m.edges[2*e+0];
            uint32_t b = m.edges[2*e+1];
            if (a >= d.px.size() || b >= d.px.size()) continue;
            float ax = d.px[a], ay = d.py[a], az = d.pz[a];
            float bx = d.px[b], by = d.py[b], bz = d.pz[b];
            float r  = m.rest[e];
            if (!(std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az) &&
                  std::isfinite(bx) && std::isfinite(by) && std::isfinite(bz) &&
                  std::isfinite(r))) continue;
            float dx = bx - ax;
            float dy = by - ay;
            float dz = bz - az;
            float len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (!std::isfinite(len)) continue;
            acc += std::fabs((double)len - (double)r);
            ++valid;
        }
        return valid ? (acc / (double)valid) : 0.0;
    }

    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        auto t0 = std::chrono::high_resolution_clock::now();
        int substeps   = d.solve_substeps > 0 ? d.solve_substeps : 1;
        int iterations = d.solve_iterations > 0 ? d.solve_iterations : 8;
        float damping  = d.solve_damping;
        if (ovr) {
            if (ovr->substeps_override > 0)   substeps   = ovr->substeps_override;
            if (ovr->iterations_override > 0) iterations = ovr->iterations_override;
        }
        substeps = substeps < 1 ? 1 : substeps;
        float dt_sub = dt / (float) substeps;
        for (int s = 0; s < substeps; ++s) {
            integrate_pred(d, dt_sub);
            presolve_apply_attachment(d);
            prepare_alpha_edge(m, d, dt_sub);
            if (d.exec_layout_blocked) {
                if (d.pos_aosoa.empty()) {
                    std::size_t n = d.px.size();
                    std::size_t nb = (n + (std::size_t)d.layout_block_size - 1) / (std::size_t)d.layout_block_size;
                    d.pos_aosoa.assign(3u * (std::size_t)d.layout_block_size * nb, 0.0f);
                }
                storage_pack_soa_to_aosoa(d.px.data(), d.py.data(), d.pz.data(), d.px.size(), (std::size_t) d.layout_block_size, d.pos_aosoa.data());
                project_distance_islands_aosoa(m, d, dt_sub, iterations);
                storage_unpack_aosoa_to_soa(d.pos_aosoa.data(), d.px.size(), (std::size_t) d.layout_block_size, d.px.data(), d.py.data(), d.pz.data());
            } else {
                project_distance_islands_soa(m, d, dt_sub, iterations);
            }
            bending_pass(m, d, dt_sub, iterations);
            finalize(d, dt_sub, damping);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        if (out) {
            out->step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            out->residual_avg = compute_distance_residual(m, d);
            out->solve_substeps = substeps;
            out->solve_iterations = iterations;
        }
        return Status::Ok;
    }
}}

namespace sim {
    static inline ::sim::Status to_api_status(eng::Status s) {
        using ES = eng::Status; using AS = ::sim::Status;
        switch (s) {
            case ES::Ok: return AS::Ok;
            case ES::InvalidArgs: return AS::InvalidArgs;
            case ES::ValidationFailed: return AS::ValidationFailed;
            case ES::NoBackend: return AS::NoBackend;
            case ES::Unsupported: return AS::Unsupported;
            case ES::OOM: return AS::OOM;
            case ES::NotReady: return AS::NotReady;
            case ES::Busy: return AS::Busy;
            default: return AS::Unsupported;
        }
    }

    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        auto st = eng::runtime_step(m, d, dt,
                                    reinterpret_cast<const eng::SolveOverrides*>(ovr),
                                    reinterpret_cast<eng::TelemetryFrame*>(out));
        return to_api_status(st);
    }
}
