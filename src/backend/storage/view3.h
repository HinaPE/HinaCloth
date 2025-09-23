/*
 * File: view3.h
 * Description: Layout-agnostic Vec3 view helpers (SoA/AoSoA/AoS)
 */
#ifndef HINACLOTH_VIEW3_H
#define HINACLOTH_VIEW3_H
#include <cstddef>
#include "backend/storage/soa.h"
#include "backend/storage/aosoa.h"
#include "backend/storage/aos.h"

namespace sim {
    inline std::size_t view_size(const SoAView3& v) noexcept { return v.n; }
    inline std::size_t view_size(const AoSoAView3& v) noexcept { return v.n; }
    inline std::size_t view_size(const AoSView3& v) noexcept { return v.n; }

    inline void view_read3(const SoAView3& v, std::size_t i, float& x, float& y, float& z) noexcept { storage_soa_read3(v, i, x,y,z); }
    inline void view_read3(const AoSoAView3& v, std::size_t i, float& x, float& y, float& z) noexcept { storage_aosoa_read3(v, i, x,y,z); }
    inline void view_read3(const AoSView3& v, std::size_t i, float& x, float& y, float& z) noexcept { storage_aos_read3(v, i, x,y,z); }

    inline void view_write3(const SoAView3& v, std::size_t i, float x, float y, float z) noexcept { storage_soa_write3(v, i, x,y,z); }
    inline void view_write3(const AoSoAView3& v, std::size_t i, float x, float y, float z) noexcept { storage_aosoa_write3(v, i, x,y,z); }
    inline void view_write3(const AoSView3& v, std::size_t i, float x, float y, float z) noexcept { storage_aos_write3(v, i, x,y,z); }

    inline void view_axpy3(const SoAView3& v, std::size_t i, float ax, float ay, float az) noexcept { storage_soa_axpy3(v, i, ax,ay,az); }
    inline void view_axpy3(const AoSoAView3& v, std::size_t i, float ax, float ay, float az) noexcept { storage_aosoa_axpy3(v, i, ax,ay,az); }
    inline void view_axpy3(const AoSView3& v, std::size_t i, float ax, float ay, float az) noexcept { storage_aos_axpy3(v, i, ax,ay,az); }
}

#endif

