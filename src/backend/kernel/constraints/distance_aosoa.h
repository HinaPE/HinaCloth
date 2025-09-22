/*
 * File: distance_aosoa.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_DISTANCE_AOSOA_H
#define HINACLOTH_DISTANCE_AOSOA_H
#include <cstddef>
#include <cstdint>
#include "backend/storage/aosoa.h"

namespace sim {
    void kernel_distance_project_aosoa(const uint32_t* edges, std::size_t m,
                                       AoSoAView3& pos_blk,
                                       const float* rest,
                                       const float* inv_mass,
                                       float* lambda_edge,
                                       const float* alpha_edge,
                                       int iterations,
                                       float alpha,
                                       float dt);
}

#endif
