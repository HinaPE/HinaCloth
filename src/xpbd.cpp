#include "xpbd.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace HinaPE {

static inline float clamp01(float x) {
    return std::clamp(x, 0.0f, 1.0f);
}

void xpbd_step_native(ClothData& cloth, const XPBDParams& params) {
    using std::size_t;

    auto P = cloth.particles();
    const size_t n = P.n;
    if (n == 0) {
        return;
    }

    const bool use_dist = params.enable_distance_constraints && cloth.num_edges() > 0;
    auto D = use_dist ? cloth.distance() : DistanceView{};

    // Precompute strides (in elements) for tight inner loops
    const size_t spx = std::max<size_t>(1, P.px.stride_bytes / sizeof(float));
    const size_t spy = std::max<size_t>(1, P.py.stride_bytes / sizeof(float));
    const size_t spz = std::max<size_t>(1, P.pz.stride_bytes / sizeof(float));
    const size_t svx = std::max<size_t>(1, P.vx.stride_bytes / sizeof(float));
    const size_t svy = std::max<size_t>(1, P.vy.stride_bytes / sizeof(float));
    const size_t svz = std::max<size_t>(1, P.vz.stride_bytes / sizeof(float));
    const size_t sim = std::max<size_t>(1, P.inv_mass.stride_bytes / sizeof(float));
    const size_t spn = std::max<size_t>(1, P.pinned.stride_bytes / sizeof(u8));

    const float gx = params.gravity_x;
    const float gy = params.gravity_y;
    const float gz = params.gravity_z;

    const int substeps = std::max(1, params.substeps);
    const int iters    = std::max(0, params.solver_iterations);
    const float dt     = params.time_step;
    if (!(dt > 0.0f)) {
        return; // nothing to do with non-positive timestep
    }
    const float inv_sub = 1.0f / static_cast<float>(substeps);
    const float dt_sub  = dt * inv_sub;
    const float dt_sub2 = dt_sub * dt_sub;

    // Temporaries for velocity update (previous positions at substep start)
    std::vector<float> px0(n), py0(n), pz0(n);

    // Strides for distance structures
    const size_t m = use_dist ? D.m : size_t{0};
    const size_t sii = use_dist ? std::max<size_t>(1, D.i.stride_bytes / sizeof(u32)) : size_t{1};
    const size_t sij = use_dist ? std::max<size_t>(1, D.j.stride_bytes / sizeof(u32)) : size_t{1};
    const size_t srs = use_dist ? std::max<size_t>(1, D.rest.stride_bytes / sizeof(float)) : size_t{1};
    const size_t scp = use_dist ? std::max<size_t>(1, D.compliance.stride_bytes / sizeof(float)) : size_t{1};
    const size_t slm = use_dist ? std::max<size_t>(1, D.lambda.stride_bytes / sizeof(float)) : size_t{1};
    const size_t sal = use_dist ? std::max<size_t>(1, D.alpha.stride_bytes / sizeof(float)) : size_t{1};
    const size_t sco = use_dist ? std::max<size_t>(1, D.color.stride_bytes / sizeof(u8)) : size_t{1};

    const float vel_damp = clamp01(params.velocity_damping);

    // Reset lambdas for hard constraints at the beginning of the time step (unless per-substep)
    if (use_dist && m > 0 && !params.reset_hard_lambda_each_substep) {
        for (size_t c = 0; c < m; ++c) {
            const float comp = D.compliance.data[c * scp];
            if (!(comp > 0.0f)) {
                D.lambda.data[c * slm] = 0.0f;
            }
        }
    }

    for (int s = 0; s < substeps; ++s) {
        // Optionally reset hard constraint lambdas at substep granularity
        if (use_dist && m > 0 && params.reset_hard_lambda_each_substep) {
            for (size_t c = 0; c < m; ++c) {
                const float comp = D.compliance.data[c * scp];
                if (!(comp > 0.0f)) {
                    D.lambda.data[c * slm] = 0.0f;
                }
            }
        }

        // Save previous positions for velocity update; integrate external acceleration and predict positions
        for (size_t i = 0; i < n; ++i) {
            const size_t ipx = i * spx, ipy = i * spy, ipz = i * spz;
            const size_t ivx = i * svx, ivy = i * svy, ivz = i * svz;
            const size_t iim = i * sim, ipn = i * spn;

            float& px = P.px.data[ipx];
            float& py = P.py.data[ipy];
            float& pz = P.pz.data[ipz];
            float& vx = P.vx.data[ivx];
            float& vy = P.vy.data[ivy];
            float& vz = P.vz.data[ivz];
            const float w = P.inv_mass.data[iim];
            const bool pinned = P.pinned.data[ipn] != 0;

            // Store originals
            px0[i] = px; py0[i] = py; pz0[i] = pz;

            if (!pinned && w > 0.0f) {
                // Semi-implicit Euler: v += a*dt, x += v*dt
                vx += gx * dt_sub;
                vy += gy * dt_sub;
                vz += gz * dt_sub;
                px += vx * dt_sub;
                py += vy * dt_sub;
                pz += vz * dt_sub;
            }
        }

        // Constraint solve (XPBD) — distance constraints only for now
        if (use_dist && iters > 0 && m > 0) {
            // Prepare optional color buckets (0..255)
            std::array<std::vector<size_t>, 256> buckets;
            if (params.use_color_ordering) {
                for (size_t c = 0; c < m; ++c) {
                    const u8 col = D.color.data[c * sco];
                    buckets[col].push_back(c);
                }
            }

            auto solve_constraint = [&](size_t c){
                    const size_t ci = c * sii;
                    const size_t cj = c * sij;
                    const size_t cr = c * srs;
                    const size_t cc = c * scp;
                    const size_t cl = c * slm;
                    const size_t ca = c * sal;

                    const u32 i = D.i.data[ci];
                    const u32 j = D.j.data[cj];
                    const float rest = D.rest.data[cr];
                    const float compliance = D.compliance.data[cc];

                    const size_t ipx = static_cast<size_t>(i) * spx; const size_t jpx = static_cast<size_t>(j) * spx;
                    const size_t ipy = static_cast<size_t>(i) * spy; const size_t jpy = static_cast<size_t>(j) * spy;
                    const size_t ipz = static_cast<size_t>(i) * spz; const size_t jpz = static_cast<size_t>(j) * spz;
                    const size_t iim = static_cast<size_t>(i) * sim; const size_t jim = static_cast<size_t>(j) * sim;
                    const size_t ipn = static_cast<size_t>(i) * spn; const size_t jpn = static_cast<size_t>(j) * spn;

                    float& xi = P.px.data[ipx];
                    float& yi = P.py.data[ipy];
                    float& zi = P.pz.data[ipz];
                    float& xj = P.px.data[jpx];
                    float& yj = P.py.data[jpy];
                    float& zj = P.pz.data[jpz];

                    const float wi = (P.pinned.data[ipn] != 0) ? 0.0f : P.inv_mass.data[iim];
                    const float wj = (P.pinned.data[jpn] != 0) ? 0.0f : P.inv_mass.data[jim];

                    const float dx = xi - xj;
                    const float dy = yi - yj;
                    const float dz = zi - zj;
                    const float len2 = dx * dx + dy * dy + dz * dz;
                    const float eps = 1e-12f;
                    if (len2 < eps) {
                        return; // avoid division by zero
                    }
                    const float len = std::sqrt(len2);
                    const float C = len - rest;

                    // XPBD: alpha_tilde = compliance / dt^2
                    const float alpha_tilde = compliance / dt_sub2;
                    // Expose computed alpha_tilde for debugging/inspection
                    D.alpha.data[ca] = alpha_tilde;

                    float& lambda_c = D.lambda.data[cl];
                    const float denom = wi + wj + alpha_tilde;
                    if (denom <= 0.0f) {
                        return;
                    }
                    const float dlambda = (-C - alpha_tilde * lambda_c) / denom;
                    lambda_c += dlambda;

                    const float nx = dx / len;
                    const float ny = dy / len;
                    const float nz = dz / len;

                    const float s_i = wi * dlambda;
                    const float s_j = wj * dlambda;

                    xi += s_i * nx;
                    yi += s_i * ny;
                    zi += s_i * nz;
                    xj -= s_j * nx;
                    yj -= s_j * ny;
                    zj -= s_j * nz;
                };

            for (int iter = 0; iter < iters; ++iter) {
                if (params.use_color_ordering) {
                    for (int col = 0; col < 256; ++col) {
                        const auto& list = buckets[(size_t)col];
                        for (size_t idx = 0; idx < list.size(); ++idx) {
                            solve_constraint(list[idx]);
                        }
                    }
                } else {
                    for (size_t c = 0; c < m; ++c) {
                        solve_constraint(c);
                    }
                }
            }
        }

        // Update velocities from positions and apply damping
        for (size_t i = 0; i < n; ++i) {
            const size_t ipx = i * spx, ipy = i * spy, ipz = i * spz;
            const size_t ivx = i * svx, ivy = i * svy, ivz = i * svz;
            const size_t iim = i * sim, ipn = i * spn;

            float& px = P.px.data[ipx];
            float& py = P.py.data[ipy];
            float& pz = P.pz.data[ipz];
            float& vx = P.vx.data[ivx];
            float& vy = P.vy.data[ivy];
            float& vz = P.vz.data[ivz];
            const float w = P.inv_mass.data[iim];
            const bool pinned = P.pinned.data[ipn] != 0;

            const float vnx = (px - px0[i]) / dt_sub;
            const float vny = (py - py0[i]) / dt_sub;
            const float vnz = (pz - pz0[i]) / dt_sub;

            if (!pinned && w > 0.0f) {
                vx = vnx * (1.0f - vel_damp);
                vy = vny * (1.0f - vel_damp);
                vz = vnz * (1.0f - vel_damp);
            } else {
                vx = 0.0f; vy = 0.0f; vz = 0.0f;
            }
        }
    }
}

} // namespace HinaPE
