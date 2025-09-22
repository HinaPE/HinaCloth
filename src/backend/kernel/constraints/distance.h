/*
 * File: distance.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_DISTANCE_H
#define HINACLOTH_DISTANCE_H
#include <cstdint>
#include <cstddef>
#include "backend/storage/soa.h"

namespace sim {
    void kernel_distance_project(const uint32_t* edges, size_t m,
                                 SoAView3& pos,
                                 const float* rest,
                                 const float* inv_mass,
                                 float* lambda_edge,
                                 const float* alpha_edge,
                                 int iterations,
                                 float alpha,
                                 float dt);
}
#endif
