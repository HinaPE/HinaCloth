#ifndef HINACLOTH_UTILS_H
#define HINACLOTH_UTILS_H
#include <cstring>
#include <cmath>

namespace sim::util {
    // Field name alias matcher: supports common aliases without mutating source names
    inline bool name_matches(const char* want, const char* got) noexcept {
        if (!want || !got) return false;
        if (std::strcmp(want, got) == 0) return true;
        if (std::strcmp(want, "position") == 0) return std::strcmp(got, "pos") == 0 || std::strcmp(got, "positions") == 0;
        if (std::strcmp(want, "velocity") == 0) return std::strcmp(got, "vel") == 0 || std::strcmp(got, "velocities") == 0;
        return false;
    }

    // Small 3D helpers kept inline
    inline void cross3(float ax, float ay, float az, float bx, float by, float bz, float& rx, float& ry, float& rz) noexcept {
        rx = ay*bz - az*by; ry = az*bx - ax*bz; rz = ax*by - ay*bx;
    }
    inline float dot3(float ax, float ay, float az, float bx, float by, float bz) noexcept { return ax*bx + ay*by + az*bz; }
    inline float len3(float x, float y, float z) noexcept { return std::sqrt(x*x + y*y + z*z); }
}

#endif // HINACLOTH_UTILS_H

