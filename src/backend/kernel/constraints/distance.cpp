#include "distance.h"
#include <cstdint>
#include <cmath>

namespace sim {
    void kernel_distance_project(const uint32_t* edges, size_t m,
                                 SoAView3& pos,
                                 const float* rest,
                                 const float* inv_mass,
                                 float* lambda_edge,
                                 int iterations,
                                 float alpha,
                                 float /*dt*/) {
        for (int it = 0; it < iterations; ++it) {
            for (size_t e = 0; e < m; ++e) {
                uint32_t a = edges[2 * e + 0], b = edges[2 * e + 1];
                if (a >= pos.n || b >= pos.n) continue;
                float ax = pos.x[a], ay = pos.y[a], az = pos.z[a];
                float bx = pos.x[b], by = pos.y[b], bz = pos.z[b];
                float dx = bx - ax, dy = by - ay, dz = bz - az;
                float len = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (len <= 1e-8f) continue;
                float C = len - rest[e];
                float wi = inv_mass ? inv_mass[a] : 1.0f;
                float wj = inv_mass ? inv_mass[b] : 1.0f;
                float denom = wi + wj + alpha;
                if (denom <= 0.0f) continue;
                float lambda_prev = lambda_edge ? lambda_edge[e] : 0.0f;
                float dlambda = -(C + alpha * lambda_prev) / denom;
                float s = dlambda / len;
                float cx = s * dx, cy = s * dy, cz = s * dz;
                // Apply weighted corrections
                if (wi > 0.0f) {
                    pos.x[a] += wi * cx;
                    pos.y[a] += wi * cy;
                    pos.z[a] += wi * cz;
                }
                if (wj > 0.0f) {
                    pos.x[b] -= wj * cx;
                    pos.y[b] -= wj * cy;
                    pos.z[b] -= wj * cz;
                }
                if (lambda_edge) lambda_edge[e] = lambda_prev + dlambda;
            }
        }
    }
}
