/*
 * File: bending.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_BENDING_H
#define HINACLOTH_BENDING_H
#include <cstddef>

namespace sim {
    struct SoAView3;
    struct AoSoAView3;
    struct AoSView3;
    void kernel_bending_project(const unsigned int* quads, size_t m, SoAView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt);
    void kernel_bending_project_aosoa(const unsigned int* quads, size_t m, AoSoAView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt);
    void kernel_bending_project_aos(const unsigned int* quads, size_t m, AoSView3& pos, const float* target, const float* inv_mass, int iterations, float alpha, float dt);
}
#endif
