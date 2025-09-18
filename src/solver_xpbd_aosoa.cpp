#include "aosoa/solver_xpbd_aosoa.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif

namespace HinaPE {
namespace {

constexpr float kConstraintEpsilon = 1e-8f;
constexpr float kConstraintEpsilonSq = kConstraintEpsilon * kConstraintEpsilon;

struct BlockIndex {
    int block{0};
    int lane{0};
};

[[nodiscard]] inline auto toBlockIndex(int idx) noexcept -> BlockIndex {
    return {.block = idx / AOSOA_BLOCK, .lane = idx % AOSOA_BLOCK};
}

template <typename BlockLoop>
void xpbd_step_aosoa_common(ClothAoSoA& cloth,
                            const XPBDSolverSettings& settings,
                            const XPBDParams& params,
                            BlockLoop&& block_loop) {
    if (cloth.count == 0) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    if (!settings.warmstart) {
        for (auto& block : cloth.cblocks) {
            for (int lane = 0; lane < AOSOA_BLOCK; ++lane) {
                block.lambda[lane] = 0.0f;
            }
        }
    } else {
        for (auto& block : cloth.cblocks) {
            for (int lane = 0; lane < AOSOA_BLOCK; ++lane) {
                block.lambda[lane] *= settings.lambda_decay;
            }
        }
    }

    const float step_dt = settings.step_dt;
    if (step_dt <= 0.0f) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    const float ax_dt = params.ax * step_dt;
    const float ay_dt = params.ay * step_dt;
    const float az_dt = params.az * step_dt;
    const float inv_h = settings.inv_step_dt;
    const float alpha_dt = settings.alpha_dt;
    const int iterations = settings.iterations;
    const bool limit_correction = settings.max_correction > 0.0f;
    const float max_correction_sq = settings.max_correction * settings.max_correction;

    const int particle_blocks = static_cast<int>(cloth.pblocks.size());
    const int constraint_blocks = static_cast<int>(cloth.cblocks.size());

    auto predict = [&](int block) {
        const int base = block * AOSOA_BLOCK;
        auto& pb = cloth.pblocks[block];
        for (int lane = 0; lane < AOSOA_BLOCK; ++lane) {
            const int idx = base + lane;
            if (idx >= cloth.count) {
                break;
            }
            pb.corr_x[lane] = pb.corr_y[lane] = pb.corr_z[lane] = 0.0f;
            if (pb.inv_mass[lane] == 0.0f) {
                pb.vx[lane] = pb.vy[lane] = pb.vz[lane] = 0.0f;
                pb.px[lane] = pb.x[lane];
                pb.py[lane] = pb.y[lane];
                pb.pz[lane] = pb.z[lane];
                continue;
            }
            pb.vx[lane] += ax_dt;
            pb.vy[lane] += ay_dt;
            pb.vz[lane] += az_dt;
            pb.px[lane] = pb.x[lane];
            pb.py[lane] = pb.y[lane];
            pb.pz[lane] = pb.z[lane];
            pb.x[lane] += pb.vx[lane] * step_dt;
            pb.y[lane] += pb.vy[lane] * step_dt;
            pb.z[lane] += pb.vz[lane] * step_dt;
        }
    };

    auto finalize_velocity = [&](int block) {
        const int base = block * AOSOA_BLOCK;
        auto& pb = cloth.pblocks[block];
        for (int lane = 0; lane < AOSOA_BLOCK; ++lane) {
            const int idx = base + lane;
            if (idx >= cloth.count) {
                break;
            }
            pb.vx[lane] = (pb.x[lane] - pb.px[lane]) * inv_h;
            pb.vy[lane] = (pb.y[lane] - pb.py[lane]) * inv_h;
            pb.vz[lane] = (pb.z[lane] - pb.pz[lane]) * inv_h;
            if (settings.velocity_scale < 1.0f) {
                pb.vx[lane] *= settings.velocity_scale;
                pb.vy[lane] *= settings.velocity_scale;
                pb.vz[lane] *= settings.velocity_scale;
            }
        }
    };

    for (int substep = 0; substep < settings.substeps; ++substep) {
        block_loop(particle_blocks, predict);

        for (int iter = 0; iter < iterations; ++iter) {
            for (int block = 0; block < constraint_blocks; ++block) {
                auto& cb = cloth.cblocks[block];
                const int base = block * AOSOA_BLOCK;
                for (int lane = 0; lane < AOSOA_BLOCK; ++lane) {
                    const int constraint_idx = base + lane;
                    if (constraint_idx >= cloth.cons_count) {
                        break;
                    }
                    const int i_idx = cb.i[lane];
                    const int j_idx = cb.j[lane];
                    const auto [bi, li] = toBlockIndex(i_idx);
                    const auto [bj, lj] = toBlockIndex(j_idx);
                    auto& pi = cloth.pblocks[bi];
                    auto& pj = cloth.pblocks[bj];

                    const float dx = pi.x[li] - pj.x[lj];
                    const float dy = pi.y[li] - pj.y[lj];
                    const float dz = pi.z[li] - pj.z[lj];
                    const float dist_sq = dx * dx + dy * dy + dz * dz;
                    if (dist_sq < kConstraintEpsilonSq) {
                        if (settings.write_debug) {
                            cb.last_c[lane] = 0.0f;
                            cb.last_dlambda[lane] = 0.0f;
                            cb.last_nx[lane] = cb.last_ny[lane] = cb.last_nz[lane] = 0.0f;
                        }
                        continue;
                    }

                    const float dist = std::sqrt(dist_sq);
                    const float nx = dx / dist;
                    const float ny = dy / dist;
                    const float nz = dz / dist;
                    const float C = dist - cb.rest_length[lane];

                    const float scale = complianceScale(settings, cb.type[lane]);
                    const float alpha_tilde = (cb.compliance[lane] * scale) * alpha_dt;
                    const float wsum = pi.inv_mass[li] + pj.inv_mass[lj];
                    const float denom = wsum + alpha_tilde;
                    if (denom <= 0.0f) {
                        if (settings.write_debug) {
                            cb.last_c[lane] = C;
                            cb.last_dlambda[lane] = 0.0f;
                            cb.last_nx[lane] = nx;
                            cb.last_ny[lane] = ny;
                            cb.last_nz[lane] = nz;
                        }
                        continue;
                    }

                    const float dlambda = (-C - alpha_tilde * cb.lambda[lane]) / denom;
                    cb.lambda[lane] += dlambda;

                    float sx = dlambda * nx;
                    float sy = dlambda * ny;
                    float sz = dlambda * nz;

                    if (limit_correction) {
                        const float mag_sq = sx * sx + sy * sy + sz * sz;
                        if (mag_sq > max_correction_sq && mag_sq > 0.0f) {
                            const float inv_mag = settings.max_correction / std::sqrt(mag_sq);
                            sx *= inv_mag;
                            sy *= inv_mag;
                            sz *= inv_mag;
                        }
                    }

                    if (pi.inv_mass[li] > 0.0f) {
                        const float scale_i = pi.inv_mass[li];
                        pi.x[li] += scale_i * sx;
                        pi.y[li] += scale_i * sy;
                        pi.z[li] += scale_i * sz;
                        pi.corr_x[li] += scale_i * sx;
                        pi.corr_y[li] += scale_i * sy;
                        pi.corr_z[li] += scale_i * sz;
                    }
                    if (pj.inv_mass[lj] > 0.0f) {
                        const float scale_j = pj.inv_mass[lj];
                        pj.x[lj] -= scale_j * sx;
                        pj.y[lj] -= scale_j * sy;
                        pj.z[lj] -= scale_j * sz;
                        pj.corr_x[lj] -= scale_j * sx;
                        pj.corr_y[lj] -= scale_j * sy;
                        pj.corr_z[lj] -= scale_j * sz;
                    }

                    if (settings.write_debug) {
                        cb.last_c[lane] = C;
                        cb.last_dlambda[lane] = dlambda;
                        cb.last_nx[lane] = nx;
                        cb.last_ny[lane] = ny;
                        cb.last_nz[lane] = nz;
                    }
                }
            }
        }

        block_loop(particle_blocks, finalize_velocity);
    }

    cloth.last_dt = settings.clamped_dt;
    cloth.last_iterations = params.iterations;
}

} // namespace

void xpbd_step_native_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
    const auto settings = makeSolverSettings(dt, params);
    auto sequential_loop = [](int blocks, auto&& body) {
        for (int b = 0; b < blocks; ++b) {
            body(b);
        }
    };
    xpbd_step_aosoa_common(cloth, settings, params, sequential_loop);
}

void xpbd_step_tbb_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    const auto settings = makeSolverSettings(dt, params);
    auto parallel_loop = [](int blocks, auto&& body) {
        tbb::parallel_for(0, blocks, [&](int b) {
            body(b);
        });
    };
    xpbd_step_aosoa_common(cloth, settings, params, parallel_loop);
#else
    xpbd_step_native_aosoa(cloth, dt, params);
#endif
}

void xpbd_step_avx2_aosoa(ClothAoSoA& cloth, float dt, const XPBDParams& params) {
    // AoSoA layout is already cache-friendly; reuse the native implementation.
    xpbd_step_native_aosoa(cloth, dt, params);
}

} // namespace HinaPE

