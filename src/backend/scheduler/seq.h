#ifndef HINACLOTH_SEQ_H
#define HINACLOTH_SEQ_H
#include <cstdint>
#include "backend/storage/soa.h"
namespace sim {
    void scheduler_seq_distance(const uint32_t* edges, size_t m, SoAView3& pos, const float* rest, int iterations, float alpha, float dt);
}
#endif //HINACLOTH_SEQ_H