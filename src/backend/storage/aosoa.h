/*
 * File: aosoa.h
 * Description: HinaCloth header.
 */
#ifndef HINACLOTH_AOSOA_H
#define HINACLOTH_AOSOA_H
#include <cstdint>
#include <cstddef>

namespace sim {
    struct AoSoAView3 {
        float* base;
        std::size_t n;
        std::size_t block;
        std::size_t stride;
    };

    inline void storage_bind_aosoa(AoSoAView3& v, float* base, std::size_t n, std::size_t block) noexcept {
        v.base = base; v.n = n; v.block = block; v.stride = 3 * block;
    }

    inline float* storage_aosoa_block_ptr(AoSoAView3& v, std::size_t bi) noexcept {
        return v.base + bi * v.stride;
    }

    inline void storage_aosoa_read3(const AoSoAView3& v, std::size_t i, float& x, float& y, float& z) noexcept {
        std::size_t bi = i / v.block; std::size_t li = i % v.block;
        float* base = v.base + bi * v.stride;
        x = base[0 * v.block + li];
        y = base[1 * v.block + li];
        z = base[2 * v.block + li];
    }
    inline void storage_aosoa_write3(const AoSoAView3& v, std::size_t i, float x, float y, float z) noexcept {
        std::size_t bi = i / v.block; std::size_t li = i % v.block;
        float* base = v.base + bi * v.stride;
        base[0 * v.block + li] = x;
        base[1 * v.block + li] = y;
        base[2 * v.block + li] = z;
    }
    inline void storage_aosoa_axpy3(const AoSoAView3& v, std::size_t i, float ax, float ay, float az) noexcept {
        std::size_t bi = i / v.block; std::size_t li = i % v.block;
        float* base = v.base + bi * v.stride;
        base[0 * v.block + li] += ax;
        base[1 * v.block + li] += ay;
        base[2 * v.block + li] += az;
    }

    void storage_pack_soa_to_aosoa(const float* x, const float* y, const float* z,
                                   std::size_t n, std::size_t block, float* out_base) noexcept;

    void storage_unpack_aosoa_to_soa(const float* base, std::size_t n, std::size_t block,
                                     float* x, float* y, float* z) noexcept;
}
#endif
