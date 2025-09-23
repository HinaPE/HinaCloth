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

    inline void storage_soa_read3(const SoAView3& v, std::size_t i, float& x, float& y, float& z) noexcept {
        x = v.x[i]; y = v.y[i]; z = v.z[i];
    }
    inline void storage_soa_write3(const SoAView3& v, std::size_t i, float x, float y, float z) noexcept {
        v.x[i] = x; v.y[i] = y; v.z[i] = z;
    }
    inline void storage_soa_axpy3(const SoAView3& v, std::size_t i, float ax, float ay, float az) noexcept {
        v.x[i] += ax; v.y[i] += ay; v.z[i] += az;
    }
}

#endif
