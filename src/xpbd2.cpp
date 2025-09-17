#include "xpbd.h"
#include "cloth_data_2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace HinaPE {

void xpbd_step_native2(ClothData2& cloth, const XPBDParams& params) {
    const std::size_t n = cloth.numParticles();
    const std::size_t m = cloth.numEdges();
    if (n == 0) return;

    const bool use_dist = params.enable_distance_constraints && (m > 0);

    const int substeps = std::max(1, params.substeps);
    const int iters    = std::max(0, params.solver_iterations);
    const float dt     = params.time_step;
    if (!(dt > 0.0f)) return;
    const float inv_sub = 1.0f / static_cast<float>(substeps);
    const float dt_sub  = dt * inv_sub;
    const float dt_sub2 = dt_sub * dt_sub;

    const float gx = params.gravity_x;
    const float gy = params.gravity_y;
    const float gz = params.gravity_z;
    const float vel_damp = std::clamp(params.velocity_damping, 0.0f, 1.0f);

    // Temporaries for previous positions (to update velocity)
    std::vector<float> px0(n), py0(n), pz0(n);

    // Reset lambdas for hard constraints at the beginning of the time step (unless per-substep)
    if (use_dist && !params.reset_hard_lambda_each_substep) {
        for (std::size_t c = 0; c < m; ++c) {
            if (!(cloth.compliance[c] > 0.0f)) {
                cloth.lambda[c] = 0.0f;
            }
        }
    }

    for (int s = 0; s < substeps; ++s) {
        // Optionally reset hard constraint lambdas at substep granularity
        if (use_dist && params.reset_hard_lambda_each_substep) {
            for (std::size_t c = 0; c < m; ++c) {
                if (!(cloth.compliance[c] > 0.0f)) {
                    cloth.lambda[c] = 0.0f;
                }
            }
        }

        // Save previous positions; integrate external acceleration and predict positions
        for (std::size_t i = 0; i < n; ++i) {
            px0[i] = cloth.px[i];
            py0[i] = cloth.py[i];
            pz0[i] = cloth.pz[i];

            const bool pinned = cloth.pinned[i] != 0;
            const float w = cloth.inv_mass[i];
            if (!pinned && w > 0.0f) {
                cloth.vx[i] += gx * dt_sub;
                cloth.vy[i] += gy * dt_sub;
                cloth.vz[i] += gz * dt_sub;
                cloth.px[i] += cloth.vx[i] * dt_sub;
                cloth.py[i] += cloth.vy[i] * dt_sub;
                cloth.pz[i] += cloth.vz[i] * dt_sub;
            }
        }

        // XPBD solve for distance constraints
        if (use_dist && iters > 0) {
            // Build color buckets if requested
            std::array<std::vector<std::size_t>, 256> buckets;
            if (params.use_color_ordering) {
                for (std::size_t c = 0; c < m; ++c) {
                    const u8 col = (c < cloth.color.size() ? cloth.color[c] : u8{0});
                    buckets[col].push_back(c);
                }
            }

            auto solve_edge = [&](std::size_t c) {
                const u32 i = cloth.edge_i[c];
                const u32 j = cloth.edge_j[c];
                float& xi = cloth.px[i]; float& yi = cloth.py[i]; float& zi = cloth.pz[i];
                float& xj = cloth.px[j]; float& yj = cloth.py[j]; float& zj = cloth.pz[j];
                const float wi = (cloth.pinned[i] != 0) ? 0.0f : cloth.inv_mass[i];
                const float wj = (cloth.pinned[j] != 0) ? 0.0f : cloth.inv_mass[j];
                const float dx = xi - xj;
                const float dy = yi - yj;
                const float dz = zi - zj;
                const float l2 = dx*dx + dy*dy + dz*dz;
                if (l2 < 1e-12f) return; // avoid div by zero
                const float l = std::sqrt(l2);
                const float C = l - cloth.rest[c];

                const float alpha_tilde = cloth.compliance[c] / dt_sub2;
                cloth.alpha[c] = alpha_tilde;
                float& lambda_c = cloth.lambda[c];
                const float denom = wi + wj + alpha_tilde;
                if (denom <= 0.0f) return;
                const float dl = (-C - alpha_tilde * lambda_c) / denom;
                lambda_c += dl;

                const float nx = dx / l;
                const float ny = dy / l;
                const float nz = dz / l;
                const float si = wi * dl;
                const float sj = wj * dl;
                xi += si * nx; yi += si * ny; zi += si * nz;
                xj -= sj * nx; yj -= sj * ny; zj -= sj * nz;
            };

            for (int it = 0; it < iters; ++it) {
                if (params.use_color_ordering) {
                    for (int col = 0; col < 256; ++col) {
                        const auto& list = buckets[(std::size_t)col];
                        for (std::size_t k = 0; k < list.size(); ++k) solve_edge(list[k]);
                    }
                } else {
                    for (std::size_t c = 0; c < m; ++c) solve_edge(c);
                }
            }
        }

        // Update velocities from position delta and apply damping
        for (std::size_t i = 0; i < n; ++i) {
            const bool pinned = cloth.pinned[i] != 0;
            const float w = cloth.inv_mass[i];
            const float vnx = (cloth.px[i] - px0[i]) / dt_sub;
            const float vny = (cloth.py[i] - py0[i]) / dt_sub;
            const float vnz = (cloth.pz[i] - pz0[i]) / dt_sub;
            if (!pinned && w > 0.0f) {
                cloth.vx[i] = vnx * (1.0f - vel_damp);
                cloth.vy[i] = vny * (1.0f - vel_damp);
                cloth.vz[i] = vnz * (1.0f - vel_damp);
            } else {
                cloth.vx[i] = 0.0f; cloth.vy[i] = 0.0f; cloth.vz[i] = 0.0f;
            }
        }
    }
}

} // namespace HinaPE

