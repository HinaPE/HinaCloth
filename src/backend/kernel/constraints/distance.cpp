#include "distance.h"
#include <cstdint>
#include <cmath>

namespace sim {
    void kernel_distance_project(const uint32_t* edges, size_t m, SoAView3& pos, const float* rest, int iterations, float alpha, float dt) {
        (void) alpha;
        (void) dt;
        for (int it = 0; it < iterations; ++it) {
            for (size_t e = 0; e < m; ++e) {
                uint32_t a = edges[2 * e + 0], b = edges[2 * e + 1];
                if (a >= pos.n || b >= pos.n) continue;
                float dx  = pos.x[b] - pos.x[a];
                float dy  = pos.y[b] - pos.y[a];
                float dz  = pos.z[b] - pos.z[a];
                float len = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (len == 0.0f) continue;
                float C   = len - rest[e];
                float inv = 1.0f / (2.0f);
                float s   = C / (len) * (inv);
                float sx  = s * dx, sy = s * dy, sz = s * dz;
                pos.x[a] += sx;
                pos.y[a] += sy;
                pos.z[a] += sz;
                pos.x[b] -= sx;
                pos.y[b] -= sy;
                pos.z[b] -= sz;
            }
        }
    }
}
