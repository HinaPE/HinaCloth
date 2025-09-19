#ifndef HINACLOTH_DISTANCE_H
#define HINACLOTH_DISTANCE_H
#include <cstdint>
#include "backend/storage/soa.h"

namespace sim {
    void kernel_distance_project(const uint32_t* edges, size_t m, SoAView3& pos, const float* rest, int iterations, float alpha, float dt);
}
#endif //HINACLOTH_DISTANCE_H
