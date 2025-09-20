#include "seq.h"
#include "backend/kernel/constraints/distance.h"

namespace sim {
    void scheduler_seq_distance(const uint32_t* edges, size_t m,
                                SoAView3& pos,
                                const float* rest,
                                const float* inv_mass,
                                float* lambda_edge,
                                int iterations,
                                float alpha,
                                float dt) {
        kernel_distance_project(edges, m, pos, rest, inv_mass, lambda_edge, /*alpha_edge*/nullptr, iterations, alpha, dt);
    }
}
