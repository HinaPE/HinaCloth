#include "tbb.h"

namespace sim {
    void scheduler_tbb_distance(const uint32_t* edges, size_t m,
                                SoAView3& pos,
                                const float* rest,
                                const float* inv_mass,
                                float* lambda_edge,
                                int iterations,
                                float alpha,
                                float dt) {
        (void)edges; (void)m; (void)pos; (void)rest; (void)inv_mass; (void)lambda_edge; (void)iterations; (void)alpha; (void)dt;
    }
}
