#include "aos/solver_xpbd_aos.h"

#include <algorithm>
#include <cstddef>
#include <cmath>

#if defined(HINACLOTH_HAVE_TBB)
#include <tbb/parallel_for.h>
#endif

#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace HinaPE {
namespace {

constexpr float kConstraintEpsilon = 1e-8f;
constexpr float kConstraintEpsilonSq = kConstraintEpsilon * kConstraintEpsilon;

template <typename ParticleLoop>
void xpbd_step_aos_common(ClothAOS& cloth,
                          const XPBDSolverSettings& settings,
                          const XPBDParams& params,
                          ParticleLoop&& loop) {
    auto& particles = cloth.particles;
    auto& constraints = cloth.constraints;

    if (particles.empty()) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    if (!settings.warmstart) {
        for (auto& c : constraints) {
            c.lambda = 0.0f;
        }
    } else {
        for (auto& c : constraints) {
            c.lambda *= settings.lambda_decay;
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

    auto predict = [&](ParticleAOS& p) {
        p.corr_x = 0.0f;
        p.corr_y = 0.0f;
        p.corr_z = 0.0f;
        if (p.inv_mass == 0.0f) {
            p.vx = p.vy = p.vz = 0.0f;
            p.px = p.x;
            p.py = p.y;
            p.pz = p.z;
            return;
        }
        p.vx += ax_dt;
        p.vy += ay_dt;
        p.vz += az_dt;
        p.px = p.x;
        p.py = p.y;
        p.pz = p.z;
        p.x += p.vx * step_dt;
        p.y += p.vy * step_dt;
        p.z += p.vz * step_dt;
    };

    auto finalize_velocity = [&](ParticleAOS& p) {
        p.vx = (p.x - p.px) * inv_h;
        p.vy = (p.y - p.py) * inv_h;
        p.vz = (p.z - p.pz) * inv_h;
        if (settings.velocity_scale < 1.0f) {
            p.vx *= settings.velocity_scale;
            p.vy *= settings.velocity_scale;
            p.vz *= settings.velocity_scale;
        }
    };

    for (int substep = 0; substep < settings.substeps; ++substep) {
        loop(particles, predict);

        for (int iter = 0; iter < iterations; ++iter) {
            for (auto& constraint : constraints) {
                auto& pi = particles[constraint.i];
                auto& pj = particles[constraint.j];

                const float dx = pi.x - pj.x;
                const float dy = pi.y - pj.y;
                const float dz = pi.z - pj.z;
                const float dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq < kConstraintEpsilonSq) {
                    if (settings.write_debug) {
                        constraint.last_c = 0.0f;
                        constraint.last_dlambda = 0.0f;
                        constraint.last_nx = constraint.last_ny = constraint.last_nz = 0.0f;
                    }
                    continue;
                }

                const float dist = std::sqrt(dist_sq);
                const float C = dist - constraint.rest_length;
                const float nx = dx / dist;
                const float ny = dy / dist;
                const float nz = dz / dist;

                const float scale = complianceScale(settings, constraint.type);
                const float alpha_tilde = (constraint.compliance * scale) * alpha_dt;
                const float wsum = pi.inv_mass + pj.inv_mass;
                const float denom = wsum + alpha_tilde;

                if (denom <= 0.0f) {
                    if (settings.write_debug) {
                        constraint.last_c = C;
                        constraint.last_dlambda = 0.0f;
                        constraint.last_nx = nx;
                        constraint.last_ny = ny;
                        constraint.last_nz = nz;
                    }
                    continue;
                }

                const float dlambda = (-C - alpha_tilde * constraint.lambda) / denom;
                constraint.lambda += dlambda;

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

                if (pi.inv_mass > 0.0f) {
                    const float scale_i = pi.inv_mass;
                    pi.x += scale_i * sx;
                    pi.y += scale_i * sy;
                    pi.z += scale_i * sz;
                    pi.corr_x += scale_i * sx;
                    pi.corr_y += scale_i * sy;
                    pi.corr_z += scale_i * sz;
                }
                if (pj.inv_mass > 0.0f) {
                    const float scale_j = pj.inv_mass;
                    pj.x -= scale_j * sx;
                    pj.y -= scale_j * sy;
                    pj.z -= scale_j * sz;
                    pj.corr_x -= scale_j * sx;
                    pj.corr_y -= scale_j * sy;
                    pj.corr_z -= scale_j * sz;
                }

                if (settings.write_debug) {
                    constraint.last_c = C;
                    constraint.last_dlambda = dlambda;
                    constraint.last_nx = nx;
                    constraint.last_ny = ny;
                    constraint.last_nz = nz;
                }
            }
        }

        loop(particles, finalize_velocity);
    }

    cloth.last_dt = settings.clamped_dt;
    cloth.last_iterations = params.iterations;
}

} // namespace

void xpbd_step_native_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
    const auto settings = makeSolverSettings(dt, params);
    auto sequential_loop = [](auto& data, auto&& body) {
        for (auto& element : data) {
            body(element);
        }
    };
    xpbd_step_aos_common(cloth, settings, params, sequential_loop);
}

void xpbd_step_tbb_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    const auto settings = makeSolverSettings(dt, params);
    auto parallel_loop = [](auto& data, auto&& body) {
        tbb::parallel_for(std::size_t{0}, data.size(), [&](std::size_t idx) {
            body(data[idx]);
        });
    };
    xpbd_step_aos_common(cloth, settings, params, parallel_loop);
#else
    xpbd_step_native_aos(cloth, dt, params);
#endif
}

void xpbd_step_avx2_aos(ClothAOS& cloth, float dt, const XPBDParams& params) {
#if defined(__AVX2__)
    const auto settings = makeSolverSettings(dt, params);
    auto& particles = cloth.particles;
    auto& constraints = cloth.constraints;

    if (particles.empty()) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    if (!settings.warmstart) {
        for (auto& c : constraints) {
            c.lambda = 0.0f;
        }
    } else {
        for (auto& c : constraints) {
            c.lambda *= settings.lambda_decay;
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
    const __m256 vax = _mm256_set1_ps(ax_dt);
    const __m256 vay = _mm256_set1_ps(ay_dt);
    const __m256 vaz = _mm256_set1_ps(az_dt);
    const __m256 vdt = _mm256_set1_ps(step_dt);
    const __m256 inv_h = _mm256_set1_ps(settings.inv_step_dt);
    const int iterations = settings.iterations;
    const float alpha_dt = settings.alpha_dt;
    const bool limit_correction = settings.max_correction > 0.0f;
    const float max_correction_sq = settings.max_correction * settings.max_correction;

    for (int s = 0; s < settings.substeps; ++s) {
        const std::size_t count = particles.size();
        std::size_t i = 0;
        for (; i + 8 <= count; i += 8) {
            float inv_mass[8];
            float x[8], y[8], z[8];
            float vx[8], vy[8], vz[8];
            for (int lane = 0; lane < 8; ++lane) {
                auto& p = particles[i + lane];
                inv_mass[lane] = p.inv_mass;
                x[lane] = p.x;
                y[lane] = p.y;
                z[lane] = p.z;
                vx[lane] = p.vx;
                vy[lane] = p.vy;
                vz[lane] = p.vz;
                p.corr_x = p.corr_y = p.corr_z = 0.0f;
            }
            const __m256 m_inv = _mm256_loadu_ps(inv_mass);
            const __m256 zero = _mm256_setzero_ps();
            const __m256 mask = _mm256_cmp_ps(m_inv, zero, _CMP_NEQ_OQ);

            __m256 mvx = _mm256_loadu_ps(vx);
            __m256 mvy = _mm256_loadu_ps(vy);
            __m256 mvz = _mm256_loadu_ps(vz);
            mvx = _mm256_blendv_ps(mvx, _mm256_add_ps(mvx, vax), mask);
            mvy = _mm256_blendv_ps(mvy, _mm256_add_ps(mvy, vay), mask);
            mvz = _mm256_blendv_ps(mvz, _mm256_add_ps(mvz, vaz), mask);

            __m256 mx = _mm256_loadu_ps(x);
            __m256 my = _mm256_loadu_ps(y);
            __m256 mz = _mm256_loadu_ps(z);

            for (int lane = 0; lane < 8; ++lane) {
                auto& p = particles[i + lane];
                p.px = p.x;
                p.py = p.y;
                p.pz = p.z;
            }

            mx = _mm256_add_ps(mx, _mm256_mul_ps(mvx, vdt));
            my = _mm256_add_ps(my, _mm256_mul_ps(mvy, vdt));
            mz = _mm256_add_ps(mz, _mm256_mul_ps(mvz, vdt));

            _mm256_storeu_ps(vx, mvx);
            _mm256_storeu_ps(vy, mvy);
            _mm256_storeu_ps(vz, mvz);
            _mm256_storeu_ps(x, mx);
            _mm256_storeu_ps(y, my);
            _mm256_storeu_ps(z, mz);

            for (int lane = 0; lane < 8; ++lane) {
                auto& p = particles[i + lane];
                if (p.inv_mass == 0.0f) {
                    p.vx = p.vy = p.vz = 0.0f;
                    p.x = p.px;
                    p.y = p.py;
                    p.z = p.pz;
                    continue;
                }
                p.vx = vx[lane];
                p.vy = vy[lane];
                p.vz = vz[lane];
                p.x = x[lane];
                p.y = y[lane];
                p.z = z[lane];
            }
        }
        for (; i < count; ++i) {
            auto& p = particles[i];
            p.corr_x = p.corr_y = p.corr_z = 0.0f;
            if (p.inv_mass == 0.0f) {
                p.vx = p.vy = p.vz = 0.0f;
                p.px = p.x;
                p.py = p.y;
                p.pz = p.z;
                continue;
            }
            p.vx += ax_dt;
            p.vy += ay_dt;
            p.vz += az_dt;
            p.px = p.x;
            p.py = p.y;
            p.pz = p.z;
            p.x += p.vx * step_dt;
            p.y += p.vy * step_dt;
            p.z += p.vz * step_dt;
        }

        const float alpha_dt_local = alpha_dt;
        for (int iter = 0; iter < iterations; ++iter) {
            for (auto& constraint : constraints) {
                auto& pi = particles[constraint.i];
                auto& pj = particles[constraint.j];
                const float dx = pi.x - pj.x;
                const float dy = pi.y - pj.y;
                const float dz = pi.z - pj.z;
                const float dist_sq = dx * dx + dy * dy + dz * dz;
                if (dist_sq < kConstraintEpsilonSq) {
                    if (settings.write_debug) {
                        constraint.last_c = 0.0f;
                        constraint.last_dlambda = 0.0f;
                        constraint.last_nx = constraint.last_ny = constraint.last_nz = 0.0f;
                    }
                    continue;
                }
                const float dist = std::sqrt(dist_sq);
                const float nx = dx / dist;
                const float ny = dy / dist;
                const float nz = dz / dist;
                const float C = dist - constraint.rest_length;
                const float scale = complianceScale(settings, constraint.type);
                const float alpha_tilde = (constraint.compliance * scale) * alpha_dt_local;
                const float wsum = pi.inv_mass + pj.inv_mass;
                const float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) {
                    if (settings.write_debug) {
                        constraint.last_c = C;
                        constraint.last_dlambda = 0.0f;
                        constraint.last_nx = nx;
                        constraint.last_ny = ny;
                        constraint.last_nz = nz;
                    }
                    continue;
                }
                const float dlambda = (-C - alpha_tilde * constraint.lambda) / denom;
                constraint.lambda += dlambda;
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
                if (pi.inv_mass > 0.0f) {
                    const float scale_i = pi.inv_mass;
                    pi.x += scale_i * sx;
                    pi.y += scale_i * sy;
                    pi.z += scale_i * sz;
                }
                if (pj.inv_mass > 0.0f) {
                    const float scale_j = pj.inv_mass;
                    pj.x -= scale_j * sx;
                    pj.y -= scale_j * sy;
                    pj.z -= scale_j * sz;
                }
                if (settings.write_debug) {
                    constraint.last_c = C;
                    constraint.last_dlambda = dlambda;
                    constraint.last_nx = nx;
                    constraint.last_ny = ny;
                    constraint.last_nz = nz;
                }
            }
        }

        i = 0;
        for (; i + 8 <= count; i += 8) {
            float x[8], y[8], z[8];
            float px[8], py[8], pz[8];
            for (int lane = 0; lane < 8; ++lane) {
                const auto& p = particles[i + lane];
                x[lane] = p.x;
                y[lane] = p.y;
                z[lane] = p.z;
                px[lane] = p.px;
                py[lane] = p.py;
                pz[lane] = p.pz;
            }
            __m256 mx = _mm256_loadu_ps(x);
            __m256 my = _mm256_loadu_ps(y);
            __m256 mz = _mm256_loadu_ps(z);
            __m256 mpx = _mm256_loadu_ps(px);
            __m256 mpy = _mm256_loadu_ps(py);
            __m256 mpz = _mm256_loadu_ps(pz);
            __m256 mvx = _mm256_mul_ps(_mm256_sub_ps(mx, mpx), inv_h);
            __m256 mvy = _mm256_mul_ps(_mm256_sub_ps(my, mpy), inv_h);
            __m256 mvz = _mm256_mul_ps(_mm256_sub_ps(mz, mpz), inv_h);
            _mm256_storeu_ps(x, mvx);
            _mm256_storeu_ps(y, mvy);
            _mm256_storeu_ps(z, mvz);
            for (int lane = 0; lane < 8; ++lane) {
                auto& p = particles[i + lane];
                p.vx = x[lane];
                p.vy = y[lane];
                p.vz = z[lane];
                if (settings.velocity_scale < 1.0f) {
                    p.vx *= settings.velocity_scale;
                    p.vy *= settings.velocity_scale;
                    p.vz *= settings.velocity_scale;
                }
            }
        }
        for (; i < count; ++i) {
            auto& p = particles[i];
            p.vx = (p.x - p.px) * settings.inv_step_dt;
            p.vy = (p.y - p.py) * settings.inv_step_dt;
            p.vz = (p.z - p.pz) * settings.inv_step_dt;
            if (settings.velocity_scale < 1.0f) {
                p.vx *= settings.velocity_scale;
                p.vy *= settings.velocity_scale;
                p.vz *= settings.velocity_scale;
            }
        }
    }

    cloth.last_dt = settings.clamped_dt;
    cloth.last_iterations = params.iterations;
#else
    xpbd_step_native_aos(cloth, dt, params);
#endif
}

} // namespace HinaPE




