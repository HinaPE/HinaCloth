#include "soa/solver_xpbd_soa.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

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

template <typename IndexLoop>
void xpbd_step_soa_common(ClothSOA& cloth,
                          const XPBDSolverSettings& settings,
                          const XPBDParams& params,
                          IndexLoop&& loop) {
    const int particle_count = static_cast<int>(cloth.x.size());
    if (particle_count == 0) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    auto& lambda = cloth.lambda;
    if (!settings.warmstart) {
        std::fill(lambda.begin(), lambda.end(), 0.0f);
    } else {
        for (auto& l : lambda) {
            l *= settings.lambda_decay;
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

    auto predict = [&](int idx) {
        cloth.corr_x[idx] = 0.0f;
        cloth.corr_y[idx] = 0.0f;
        cloth.corr_z[idx] = 0.0f;
        if (cloth.inv_mass[idx] == 0.0f) {
            cloth.vx[idx] = cloth.vy[idx] = cloth.vz[idx] = 0.0f;
            cloth.px[idx] = cloth.x[idx];
            cloth.py[idx] = cloth.y[idx];
            cloth.pz[idx] = cloth.z[idx];
            return;
        }
        cloth.vx[idx] += ax_dt;
        cloth.vy[idx] += ay_dt;
        cloth.vz[idx] += az_dt;
        cloth.px[idx] = cloth.x[idx];
        cloth.py[idx] = cloth.y[idx];
        cloth.pz[idx] = cloth.z[idx];
        cloth.x[idx] += cloth.vx[idx] * step_dt;
        cloth.y[idx] += cloth.vy[idx] * step_dt;
        cloth.z[idx] += cloth.vz[idx] * step_dt;
    };

    auto finalize_velocity = [&](int idx) {
        cloth.vx[idx] = (cloth.x[idx] - cloth.px[idx]) * inv_h;
        cloth.vy[idx] = (cloth.y[idx] - cloth.py[idx]) * inv_h;
        cloth.vz[idx] = (cloth.z[idx] - cloth.pz[idx]) * inv_h;
        if (settings.velocity_scale < 1.0f) {
            cloth.vx[idx] *= settings.velocity_scale;
            cloth.vy[idx] *= settings.velocity_scale;
            cloth.vz[idx] *= settings.velocity_scale;
        }
    };

    const int constraint_count = static_cast<int>(cloth.ci.size());

    for (int substep = 0; substep < settings.substeps; ++substep) {
        loop(particle_count, predict);

        for (int iter = 0; iter < iterations; ++iter) {
            for (int k = 0; k < constraint_count; ++k) {
                const int i = cloth.ci[k];
                const int j = cloth.cj[k];
                const float dx = cloth.x[i] - cloth.x[j];
                const float dy = cloth.y[i] - cloth.y[j];
                const float dz = cloth.z[i] - cloth.z[j];
                const float dist_sq = dx * dx + dy * dy + dz * dz;
                if (dist_sq < kConstraintEpsilonSq) {
                    if (settings.write_debug) {
                        cloth.last_c[k] = 0.0f;
                        cloth.last_dlambda[k] = 0.0f;
                        cloth.last_nx[k] = cloth.last_ny[k] = cloth.last_nz[k] = 0.0f;
                    }
                    continue;
                }

                const float dist = std::sqrt(dist_sq);
                const float nx = dx / dist;
                const float ny = dy / dist;
                const float nz = dz / dist;
                const float C = dist - cloth.rest_length[k];

                const float scale = complianceScale(settings, cloth.type[k]);
                const float alpha_tilde = (cloth.compliance[k] * scale) * alpha_dt;
                const float wsum = cloth.inv_mass[i] + cloth.inv_mass[j];
                const float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) {
                    if (settings.write_debug) {
                        cloth.last_c[k] = C;
                        cloth.last_dlambda[k] = 0.0f;
                        cloth.last_nx[k] = nx;
                        cloth.last_ny[k] = ny;
                        cloth.last_nz[k] = nz;
                    }
                    continue;
                }

                const float dlambda = (-C - alpha_tilde * cloth.lambda[k]) / denom;
                cloth.lambda[k] += dlambda;

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

                if (cloth.inv_mass[i] > 0.0f) {
                    const float scale_i = cloth.inv_mass[i];
                    cloth.x[i] += scale_i * sx;
                    cloth.y[i] += scale_i * sy;
                    cloth.z[i] += scale_i * sz;
                    cloth.corr_x[i] += scale_i * sx;
                    cloth.corr_y[i] += scale_i * sy;
                    cloth.corr_z[i] += scale_i * sz;
                }
                if (cloth.inv_mass[j] > 0.0f) {
                    const float scale_j = cloth.inv_mass[j];
                    cloth.x[j] -= scale_j * sx;
                    cloth.y[j] -= scale_j * sy;
                    cloth.z[j] -= scale_j * sz;
                    cloth.corr_x[j] -= scale_j * sx;
                    cloth.corr_y[j] -= scale_j * sy;
                    cloth.corr_z[j] -= scale_j * sz;
                }

                if (settings.write_debug) {
                    cloth.last_c[k] = C;
                    cloth.last_dlambda[k] = dlambda;
                    cloth.last_nx[k] = nx;
                    cloth.last_ny[k] = ny;
                    cloth.last_nz[k] = nz;
                }
            }
        }

        loop(particle_count, finalize_velocity);
    }

    cloth.last_dt = settings.clamped_dt;
    cloth.last_iterations = params.iterations;
}

} // namespace

void xpbd_step_native_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
    const auto settings = makeSolverSettings(dt, params);
    auto sequential_loop = [](int count, auto&& body) {
        for (int idx = 0; idx < count; ++idx) {
            body(idx);
        }
    };
    xpbd_step_soa_common(cloth, settings, params, sequential_loop);
}

void xpbd_step_tbb_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
#if defined(HINACLOTH_HAVE_TBB)
    const auto settings = makeSolverSettings(dt, params);
    auto parallel_loop = [](int count, auto&& body) {
        tbb::parallel_for(0, count, [&](int idx) {
            body(idx);
        });
    };
    xpbd_step_soa_common(cloth, settings, params, parallel_loop);
#else
    xpbd_step_native_soa(cloth, dt, params);
#endif
}

void xpbd_step_avx2_soa(ClothSOA& cloth, float dt, const XPBDParams& params) {
#if defined(__AVX2__)
    const auto settings = makeSolverSettings(dt, params);
    const int n = static_cast<int>(cloth.x.size());
    if (n == 0) {
        cloth.last_dt = settings.clamped_dt;
        cloth.last_iterations = params.iterations;
        return;
    }

    auto& lambda = cloth.lambda;
    if (!settings.warmstart) {
        std::fill(lambda.begin(), lambda.end(), 0.0f);
    } else {
        for (auto& l : lambda) {
            l *= settings.lambda_decay;
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

    for (int substep = 0; substep < settings.substeps; ++substep) {
        int i = 0;
        for (; i + 8 <= n; i += 8) {
            float inv_mass[8];
            float vx[8], vy[8], vz[8];
            float x[8], y[8], z[8];
            for (int lane = 0; lane < 8; ++lane) {
                const int idx = i + lane;
                inv_mass[lane] = cloth.inv_mass[idx];
                vx[lane] = cloth.vx[idx];
                vy[lane] = cloth.vy[idx];
               vz[lane] = cloth.vz[idx];
                x[lane] = cloth.x[idx];
                y[lane] = cloth.y[idx];
                z[lane] = cloth.z[idx];
                cloth.corr_x[idx] = cloth.corr_y[idx] = cloth.corr_z[idx] = 0.0f;
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
                const int idx = i + lane;
                cloth.px[idx] = cloth.x[idx];
                cloth.py[idx] = cloth.y[idx];
                cloth.pz[idx] = cloth.z[idx];
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
                const int idx = i + lane;
                if (cloth.inv_mass[idx] == 0.0f) {
                    cloth.vx[idx] = cloth.vy[idx] = cloth.vz[idx] = 0.0f;
                    cloth.x[idx] = cloth.px[idx];
                    cloth.y[idx] = cloth.py[idx];
                    cloth.z[idx] = cloth.pz[idx];
                    continue;
                }
                cloth.vx[idx] = vx[lane];
                cloth.vy[idx] = vy[lane];
                cloth.vz[idx] = vz[lane];
                cloth.x[idx] = x[lane];
                cloth.y[idx] = y[lane];
                cloth.z[idx] = z[lane];
            }
        }
        for (; i < n; ++i) {
            cloth.corr_x[i] = cloth.corr_y[i] = cloth.corr_z[i] = 0.0f;
            if (cloth.inv_mass[i] == 0.0f) {
                cloth.vx[i] = cloth.vy[i] = cloth.vz[i] = 0.0f;
                cloth.px[i] = cloth.x[i];
                cloth.py[i] = cloth.y[i];
                cloth.pz[i] = cloth.z[i];
                continue;
            }
            cloth.vx[i] += ax_dt;
            cloth.vy[i] += ay_dt;
            cloth.vz[i] += az_dt;
            cloth.px[i] = cloth.x[i];
            cloth.py[i] = cloth.y[i];
            cloth.pz[i] = cloth.z[i];
            cloth.x[i] += cloth.vx[i] * step_dt;
            cloth.y[i] += cloth.vy[i] * step_dt;
            cloth.z[i] += cloth.vz[i] * step_dt;
        }

        const int m = static_cast<int>(cloth.ci.size());
        for (int iter = 0; iter < iterations; ++iter) {
            for (int k = 0; k < m; ++k) {
                const int a = cloth.ci[k];
                const int b = cloth.cj[k];
                const float dx = cloth.x[a] - cloth.x[b];
                const float dy = cloth.y[a] - cloth.y[b];
                const float dz = cloth.z[a] - cloth.z[b];
                const float dist_sq = dx * dx + dy * dy + dz * dz;
                if (dist_sq < kConstraintEpsilonSq) {
                    if (settings.write_debug) {
                        cloth.last_c[k] = 0.0f;
                        cloth.last_dlambda[k] = 0.0f;
                        cloth.last_nx[k] = cloth.last_ny[k] = cloth.last_nz[k] = 0.0f;
                    }
                    continue;
                }
                const float dist = std::sqrt(dist_sq);
                const float nx = dx / dist;
                const float ny = dy / dist;
                const float nz = dz / dist;
                const float C = dist - cloth.rest_length[k];
                const float scale = complianceScale(settings, cloth.type[k]);
                const float alpha_tilde = (cloth.compliance[k] * scale) * alpha_dt;
                const float wsum = cloth.inv_mass[a] + cloth.inv_mass[b];
                const float denom = wsum + alpha_tilde;
                if (denom <= 0.0f) {
                    if (settings.write_debug) {
                        cloth.last_c[k] = C;
                        cloth.last_dlambda[k] = 0.0f;
                        cloth.last_nx[k] = nx;
                        cloth.last_ny[k] = ny;
                        cloth.last_nz[k] = nz;
                    }
                    continue;
                }
                const float dlambda = (-C - alpha_tilde * cloth.lambda[k]) / denom;
                cloth.lambda[k] += dlambda;
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
                if (cloth.inv_mass[a] > 0.0f) {
                    const float scale_a = cloth.inv_mass[a];
                    cloth.x[a] += scale_a * sx;
                    cloth.y[a] += scale_a * sy;
                    cloth.z[a] += scale_a * sz;
                }
                if (cloth.inv_mass[b] > 0.0f) {
                    const float scale_b = cloth.inv_mass[b];
                    cloth.x[b] -= scale_b * sx;
                    cloth.y[b] -= scale_b * sy;
                    cloth.z[b] -= scale_b * sz;
                }
                if (settings.write_debug) {
                    cloth.last_c[k] = C;
                    cloth.last_dlambda[k] = dlambda;
                    cloth.last_nx[k] = nx;
                    cloth.last_ny[k] = ny;
                    cloth.last_nz[k] = nz;
                }
            }
        }

        i = 0;
        for (; i + 8 <= n; i += 8) {
            float x[8], y[8], z[8];
            float px[8], py[8], pz[8];
            for (int lane = 0; lane < 8; ++lane) {
                const int idx = i + lane;
                x[lane] = cloth.x[idx];
                y[lane] = cloth.y[idx];
                z[lane] = cloth.z[idx];
                px[lane] = cloth.px[idx];
                py[lane] = cloth.py[idx];
                pz[lane] = cloth.pz[idx];
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
                const int idx = i + lane;
                cloth.vx[idx] = x[lane];
                cloth.vy[idx] = y[lane];
                cloth.vz[idx] = z[lane];
                if (settings.velocity_scale < 1.0f) {
                    cloth.vx[idx] *= settings.velocity_scale;
                    cloth.vy[idx] *= settings.velocity_scale;
                    cloth.vz[idx] *= settings.velocity_scale;
                }
            }
        }
        for (; i < n; ++i) {
            cloth.vx[i] = (cloth.x[i] - cloth.px[i]) * settings.inv_step_dt;
            cloth.vy[i] = (cloth.y[i] - cloth.py[i]) * settings.inv_step_dt;
            cloth.vz[i] = (cloth.z[i] - cloth.pz[i]) * settings.inv_step_dt;
            if (settings.velocity_scale < 1.0f) {
                cloth.vx[i] *= settings.velocity_scale;
                cloth.vy[i] *= settings.velocity_scale;
                cloth.vz[i] *= settings.velocity_scale;
            }
        }
    }

    cloth.last_dt = settings.clamped_dt;
    cloth.last_iterations = params.iterations;
#else
    xpbd_step_native_soa(cloth, dt, params);
#endif
}

} // namespace HinaPE

