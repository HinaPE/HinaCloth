/*
 * File: distance_aos.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_DISTANCE_AOS_H
#define HINACLOTH_DISTANCE_AOS_H
#include <cstddef>
#include <cstdint>
#include "backend/storage/aos.h"

namespace sim {
    void kernel_distance_project_aos(const uint32_t* edges, std::size_t m,
                                     AoSView3& pos,
                                     const float* rest,
                                     const float* inv_mass,
                                     float* lambda_edge,
                                     const float* alpha_edge,
                                     int iterations,
                                     float alpha,
                                     float dt);
}

#endif

