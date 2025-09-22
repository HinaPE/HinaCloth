/*
 * File: soa.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_SOA_H
#define HINACLOTH_SOA_H
#include <cstddef>

namespace sim {
    struct SoAView3 {
        float* x;
        float* y;
        float* z;
        std::size_t n;
    };

    inline void storage_bind_soa(SoAView3& v, float* px, float* py, float* pz, std::size_t n) noexcept {
        v.x = px;
        v.y = py;
        v.z = pz;
        v.n = n;
    }
}

#endif
