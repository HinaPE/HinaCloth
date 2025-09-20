#ifndef HINACLOTH_DISTANCE_AVX2_H
#define HINACLOTH_DISTANCE_AVX2_H
#include <cstdint>
#include <cstddef>
#include "backend/storage/soa.h"

namespace sim {
    // AVX2-optimized distance projection over m constraints using gathers.
    void kernel_distance_project_avx2(const uint32_t* edges, std::size_t m,
                                      SoAView3& pos,
                                      const float* rest,
                                      const float* inv_mass,
                                      float* lambda_edge,
                                      int iterations,
                                      float alpha,
                                      float dt);
}

#endif //HINACLOTH_DISTANCE_AVX2_H

