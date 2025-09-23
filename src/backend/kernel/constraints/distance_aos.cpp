#include "distance_aos.h"
#include <cmath>

namespace sim {
    void kernel_distance_project_aos(const uint32_t* edges, std::size_t m,
                                     AoSView3& pos,
                                     const float* rest,
                                     const float* inv_mass,
                                     float* lambda_edge,
                                     const float* alpha_edge,
                                     int iterations,
                                     float alpha,
                                     float dt) {
        (void)dt;
        for (int it = 0; it < iterations; ++it) {
            for (std::size_t e = 0; e < m; ++e) {
                uint32_t a = edges[2 * e + 0], b = edges[2 * e + 1];
                if (a >= pos.n || b >= pos.n) continue;
                float ax,ay,az,bx,by,bz; storage_aos_read3(pos, a, ax,ay,az); storage_aos_read3(pos, b, bx,by,bz);
                float dx = bx - ax, dy = by - ay, dz = bz - az;
                float len = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (len <= 1e-8f) continue;
                float C = len - rest[e];
                float wi = inv_mass ? inv_mass[a] : 1.0f;
                float wj = inv_mass ? inv_mass[b] : 1.0f;
                float a_e = alpha_edge ? alpha_edge[e] : alpha;
                float denom = wi + wj + a_e; if (denom <= 0.0f) continue;
                float lambda_prev = lambda_edge ? lambda_edge[e] : 0.0f;
                float dlambda = -(C + a_e * lambda_prev) / denom;
                float s = dlambda / len;
                float cx = s * dx, cy = s * dy, cz = s * dz;
                if (wi > 0.0f) storage_aos_axpy3(pos, a, -wi * cx, -wi * cy, -wi * cz);
                if (wj > 0.0f) storage_aos_axpy3(pos, b, +wj * cx, +wj * cy, +wj * cz);
                if (lambda_edge) lambda_edge[e] = lambda_prev + dlambda;
            }
        }
    }
}

