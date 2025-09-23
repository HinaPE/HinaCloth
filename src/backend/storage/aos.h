/*
 * File: aos.h
 * Description: HinaCloth AoS storage helpers.
 */
#ifndef HINACLOTH_AOS_H
#define HINACLOTH_AOS_H
#include <cstddef>
#include <cstdint>

namespace sim {
    struct AoSView3 {
        float* base;     // interleaved [x0,y0,z0, x1,y1,z1, ...]
        std::size_t n;   // number of items
        std::size_t stride; // stride in floats between consecutive items (default 3)
    };

    inline void storage_bind_aos(AoSView3& v, float* base, std::size_t n, std::size_t stride = 3) noexcept {
        v.base = base; v.n = n; v.stride = stride;
    }

    inline void storage_aos_read3(const AoSView3& v, std::size_t i, float& x, float& y, float& z) noexcept {
        const float* p = v.base + i * v.stride;
        x = p[0]; y = p[1]; z = p[2];
    }
    inline void storage_aos_write3(const AoSView3& v, std::size_t i, float x, float y, float z) noexcept {
        float* p = v.base + i * v.stride; p[0] = x; p[1] = y; p[2] = z;
    }
    inline void storage_aos_axpy3(const AoSView3& v, std::size_t i, float ax, float ay, float az) noexcept {
        float* p = v.base + i * v.stride; p[0] += ax; p[1] += ay; p[2] += az;
    }

    // Pack/Unpack between SoA arrays and AoS interleaved buffer
    void storage_pack_soa_to_aos(const float* x, const float* y, const float* z, std::size_t n, float* out_base, std::size_t stride = 3) noexcept;
    void storage_unpack_aos_to_soa(const float* base, std::size_t n, float* x, float* y, float* z, std::size_t stride = 3) noexcept;
}

#endif

