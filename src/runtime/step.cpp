#include "step.h"
#include "core/model/model.h"
#include "core/data/data.h"
#include "backend/storage/soa.h"
#include "backend/storage/aosoa.h"
#include "backend/scheduler/seq.h"
#include "backend/kernel/constraints/distance.h"
#include "backend/kernel/constraints/distance_avx2.h"
#include "backend/kernel/constraints/distance_aosoa.h"
#include "adapter/engine_adapter.h"
#include <algorithm>
#include <chrono>
#ifdef HINACLOTH_HAVE_TBB
#include <tbb/parallel_for.h>
#include <tbb/global_control.h>
#endif

namespace sim {
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

    static void project_distance_islands_soa(const Model& m, Data& d, float dt, int iterations) {
        SoAView3 pos{};
        storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        float alpha = std::max(0.0f, d.distance_compliance) / (dt * dt);
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
                                             lbeg, iterations, alpha, dt);
                return;
            }
        #endif
            kernel_distance_project(ebeg, cnt, pos, rbeg,
                                    d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                                    lbeg, iterations, alpha, dt);
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
        // Pack SoA -> AoSoA once per substep before calling this function
        AoSoAView3 posb{};
        storage_bind_aosoa(posb, d.pos_aosoa.data(), d.px.size(), (std::size_t) d.layout_block_size);
        float alpha = std::max(0.0f, d.distance_compliance) / (dt * dt);
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
                                          lbeg, iterations, alpha, dt);
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
            if (d.exec_layout_blocked) {
                // Pack predicted positions into AoSoA blocks
                if (d.pos_aosoa.empty()) {
                    std::size_t n = d.px.size();
                    std::size_t nb = (n + (std::size_t)d.layout_block_size - 1) / (std::size_t)d.layout_block_size;
                    d.pos_aosoa.assign(3u * (std::size_t)d.layout_block_size * nb, 0.0f);
                }
                storage_pack_soa_to_aosoa(d.px.data(), d.py.data(), d.pz.data(), d.px.size(), (std::size_t) d.layout_block_size, d.pos_aosoa.data());
                project_distance_islands_aosoa(m, d, dt_sub, iterations);
                // Unpack back to SoA for finalize phase
                storage_unpack_aosoa_to_soa(d.pos_aosoa.data(), d.px.size(), (std::size_t) d.layout_block_size, d.px.data(), d.py.data(), d.pz.data());
            } else {
                project_distance_islands_soa(m, d, dt_sub, iterations);
            }
            finalize(d, dt_sub, damping);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        if (out) out->step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return Status::Ok;
    }
}
