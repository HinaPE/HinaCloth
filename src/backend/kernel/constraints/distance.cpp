#include "distance.h"
#include <cstdint>
#include <cmath>
#include <span>

namespace sim {
    void kernel_distance_project(const uint32_t* edges, size_t m,
                                 SoAView3& pos,
                                 const float* rest,
                                 const float* inv_mass,
                                 float* lambda_edge,
                                 const float* alpha_edge,
                                 int iterations,
                                 float alpha,
                                 float dt) {
        (void)dt;
        std::span<const float> R{rest, m};
        std::span<const float> W{}; if (inv_mass) W = std::span<const float>(inv_mass, pos.n);
        std::span<float> L{}; if (lambda_edge) L = std::span<float>(lambda_edge, m);
        for (int it = 0; it < iterations; ++it) {
            for (size_t e = 0; e < m; ++e) {
                uint32_t a = edges[2 * e + 0], b = edges[2 * e + 1];
                if (a >= pos.n || b >= pos.n) continue;
                float ax = pos.x[a], ay = pos.y[a], az = pos.z[a];
                float bx = pos.x[b], by = pos.y[b], bz = pos.z[b];
                float dx = bx - ax, dy = by - ay, dz = bz - az;
                float len = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (len <= 1e-8f) continue;
                float C = len - R[e];
                float wi = inv_mass ? W[a] : 1.0f;
                float wj = inv_mass ? W[b] : 1.0f;
                float a_e = alpha_edge ? alpha_edge[e] : alpha;
                float denom = wi + wj + a_e;
                if (denom <= 0.0f) continue;
                float lambda_prev = lambda_edge ? L[e] : 0.0f;
                float dlambda = -(C + a_e * lambda_prev) / denom;
                float s = dlambda / len;
                float cx = s * dx, cy = s * dy, cz = s * dz;
                if (wi > 0.0f) {
                    pos.x[a] -= wi * cx;
                    pos.y[a] -= wi * cy;
                    pos.z[a] -= wi * cz;
                }
                if (wj > 0.0f) {
                    pos.x[b] += wj * cx;
                    pos.y[b] += wj * cy;
                    pos.z[b] += wj * cz;
                }
                if (lambda_edge) L[e] = lambda_prev + dlambda;
            }
        }
    }
}
