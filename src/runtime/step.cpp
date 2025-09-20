#include "step.h"
#include "core/model/model.h"
#include "core/data/data.h"
#include "backend/storage/soa.h"
#include "backend/scheduler/seq.h"
#include "adapter/engine_adapter.h"
#include <algorithm>
#include <chrono>

namespace sim {
    static void integrate_pred(Data& d, float dt) {
        size_t n = d.x.size();
        for (size_t i = 0; i < n; i++) {
            if (d.inv_mass.empty() || d.inv_mass[i] > 0.0f) {
                d.vx[i] += dt * d.gx;
                d.vy[i] += dt * d.gy;
                d.vz[i] += dt * d.gz;
                d.px[i] = d.x[i] + dt * d.vx[i];
                d.py[i] = d.y[i] + dt * d.vy[i];
                d.pz[i] = d.z[i] + dt * d.vz[i];
            } else {
                // pinned: keep position/velocity fixed
                d.px[i] = d.x[i];
                d.py[i] = d.y[i];
                d.pz[i] = d.z[i];
                d.vx[i] = 0.0f;
                d.vy[i] = 0.0f;
                d.vz[i] = 0.0f;
            }
        }
    }

    static void project_distance(const Model& m, Data& d, float dt, int iterations) {
        SoAView3 pos{};
        storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        size_t mcnt = m.edges.size() / 2;
        float alpha = std::max(0.0f, d.distance_compliance) / (dt * dt);
        scheduler_seq_distance(m.edges.data(), mcnt, pos, m.rest.data(),
                               d.inv_mass.empty() ? nullptr : d.inv_mass.data(),
                               d.lambda_edge.empty() ? nullptr : d.lambda_edge.data(),
                               iterations, alpha, dt);
    }

    static void finalize(Data& d, float dt, float damping) {
        damping = std::clamp(damping, 0.0f, 1.0f);
        float vel_mul = (1.0f - damping);
        size_t n = d.x.size();
        for (size_t i = 0; i < n; i++) {
            if (d.inv_mass.empty() || d.inv_mass[i] > 0.0f) {
                float nx = d.px[i];
                float ny = d.py[i];
                float nz = d.pz[i];
                d.vx[i]  = (nx - d.x[i]) / dt * vel_mul;
                d.vy[i]  = (ny - d.y[i]) / dt * vel_mul;
                d.vz[i]  = (nz - d.z[i]) / dt * vel_mul;
                d.x[i]   = nx;
                d.y[i]   = ny;
                d.z[i]   = nz;
            } else {
                // pinned stays unchanged
                d.px[i] = d.x[i]; d.py[i] = d.y[i]; d.pz[i] = d.z[i];
                d.vx[i] = 0.0f; d.vy[i] = 0.0f; d.vz[i] = 0.0f;
            }
        }
    }

    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        auto t0 = std::chrono::high_resolution_clock::now();
        // Effective solve parameters (from Data, can be overridden)
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
            project_distance(m, d, dt_sub, iterations);
            finalize(d, dt_sub, damping);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        if (out) {
            out->step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
        return Status::Ok;
    }
}
