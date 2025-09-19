#include "step.h"
#include "core/model/model.h"
#include "core/data/data.h"
#include "backend/storage/soa.h"
#include "backend/scheduler/seq.h"

namespace sim {
    static void integrate_pred(Data& d, float dt) {
        size_t n = d.x.size();
        for (size_t i = 0; i < n; i++) {
            d.vx[i] += dt * d.gx;
            d.vy[i] += dt * d.gy;
            d.vz[i] += dt * d.gz;
            d.px[i] = d.x[i] + dt * d.vx[i];
            d.py[i] = d.y[i] + dt * d.vy[i];
            d.pz[i] = d.z[i] + dt * d.vz[i];
        }
    }

    static void project_distance(const Model& m, Data& d, float dt, int iterations) {
        SoAView3 pos{};
        storage_bind_soa(pos, d.px.data(), d.py.data(), d.pz.data(), d.px.size());
        float alpha = 0.0f;
        size_t mcnt = m.edges.size() / 2;
        scheduler_seq_distance(m.edges.data(), mcnt, pos, m.rest.data(), iterations, alpha, dt);
    }

    static void finalize(Data& d, float dt) {
        size_t n = d.x.size();
        for (size_t i = 0; i < n; i++) {
            float nx = d.px[i];
            float ny = d.py[i];
            float nz = d.pz[i];
            d.vx[i]  = (nx - d.x[i]) / dt;
            d.vy[i]  = (ny - d.y[i]) / dt;
            d.vz[i]  = (nz - d.z[i]) / dt;
            d.x[i]   = nx;
            d.y[i]   = ny;
            d.z[i]   = nz;
        }
    }

    Status runtime_step(const Model& m, Data& d, float dt, const SolveOverrides* ovr, TelemetryFrame* out) {
        int iterations = 8;
        integrate_pred(d, dt);
        project_distance(m, d, dt, iterations);
        finalize(d, dt);
        if (out) {
            out->step_ms = 0.0;
        }
        return Status::Ok;
    }
}
