#ifndef HINACLOTH_TBB_H
#define HINACLOTH_TBB_H
#include <cstdint>
#include <cstddef>
#include "backend/storage/soa.h"

namespace sim {
    void scheduler_tbb_distance(const uint32_t* edges, size_t m,
                                SoAView3& pos,
                                const float* rest,
                                const float* inv_mass,
                                float* lambda_edge,
                                int iterations,
                                float alpha,
                                float dt);
}
#endif //HINACLOTH_TBB_H
