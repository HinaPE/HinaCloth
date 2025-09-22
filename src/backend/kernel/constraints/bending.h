/*
 * File: bending.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_BENDING_H
#define HINACLOTH_BENDING_H
#include <cstddef>

namespace sim {
    struct SoAView3;
    void kernel_bending_project(const unsigned int* quads, size_t m, SoAView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt);
}
#endif
