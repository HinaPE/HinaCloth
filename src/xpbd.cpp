#include "xpbd.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#if defined(__AVX2__) && (defined(__x86_64__) || defined(_M_X64) || defined(__i386) || defined(_M_IX86))
#  include <immintrin.h>
#endif

namespace HinaPE {
namespace {

struct IntegrationScratch {
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> pz;
};

inline std::uint8_t max_color(const ColumnView<u8>& colors) {
    if (colors.count == 0) {
        return 0;
    }
    const auto span = colors.span();
    return *std::max_element(span.begin(), span.end());
}

inline void apply_delta(std::span<float>& px, std::span<float>& py, std::span<float>& pz, size_t idx, float dx, float dy, float dz, float weight) {
    px[idx] += dx * weight;
    py[idx] += dy * weight;
    pz[idx] += dz * weight;
}

inline bool should_process_distance(const XPBDParams& params, const DistanceView& dist_view, size_t num_edges) {
    return params.enable_distance_constraints && num_edges > 0 && dist_view.m > 0;
}

inline void solve_distance_constraints_serial(const DistanceView& dist_view,
                                              std::span<float> px,
                                              std::span<float> py,
                                              std::span<float> pz,
                                              std::span<float> inv_mass,
                                              float inv_dt_sq) {
    auto idx_i      = dist_view.i.span();
    auto idx_j      = dist_view.j.span();
    auto rest       = dist_view.rest.span();
    auto compliance = dist_view.compliance.span();
    auto lambda     = dist_view.lambda.span();
    auto alpha      = dist_view.alpha.span();
    auto color      = dist_view.color.span();

    const std::uint8_t color_max = max_color(dist_view.color);

    for (std::uint8_t color_id = 0; color_id <= color_max; ++color_id) {
        for (size_t c = 0; c < dist_view.m; ++c) {
            if (color[c] != color_id) {
                continue;
            }

            const std::uint32_t i = idx_i[c];
            const std::uint32_t j = idx_j[c];
            const float wi        = inv_mass[i];
            const float wj        = inv_mass[j];
            const float wsum      = wi + wj;
            if (wsum <= 0.0f) {
                lambda[c] = 0.0f;
                continue;
            }

            const float diff_x = px[i] - px[j];
            const float diff_y = py[i] - py[j];
            const float diff_z = pz[i] - pz[j];
            const float len_sq = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
            if (len_sq <= std::numeric_limits<float>::epsilon()) {
                continue;
            }
            const float len         = std::sqrt(len_sq);
            const float constraint  = len - rest[c];
            const float alpha_tilde = compliance[c] * inv_dt_sq;
            const float denom       = wsum + alpha_tilde;
            if (denom <= 0.0f) {
                continue;
            }

            const float lambda_prev  = lambda[c];
            const float delta_lambda = (-constraint - alpha_tilde * lambda_prev) / denom;
            const float grad_scale   = delta_lambda / len;
            const float grad_x       = diff_x * grad_scale;
            const float grad_y       = diff_y * grad_scale;
            const float grad_z       = diff_z * grad_scale;

            lambda[c] = lambda_prev + delta_lambda;
            alpha[c]  = alpha_tilde;

            if (wi > 0.0f) {
                apply_delta(px, py, pz, i, grad_x, grad_y, grad_z, wi);
            }
            if (wj > 0.0f) {
                apply_delta(px, py, pz, j, grad_x, grad_y, grad_z, -wj);
            }
        }
    }
}

inline void solve_distance_constraints_tbb(const DistanceView& dist_view,
                                           std::span<float> px,
                                           std::span<float> py,
                                           std::span<float> pz,
                                           std::span<float> inv_mass,
                                           float inv_dt_sq) {
    auto idx_i      = dist_view.i.span();
    auto idx_j      = dist_view.j.span();
    auto rest       = dist_view.rest.span();
    auto compliance = dist_view.compliance.span();
    auto lambda     = dist_view.lambda.span();
    auto alpha      = dist_view.alpha.span();
    auto color      = dist_view.color.span();

    const std::uint8_t color_max = max_color(dist_view.color);

    for (std::uint8_t color_id = 0; color_id <= color_max; ++color_id) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, dist_view.m, 512), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t c = r.begin(); c != r.end(); ++c) {
                if (color[c] != color_id) {
                    continue;
                }

                const std::uint32_t i = idx_i[c];
                const std::uint32_t j = idx_j[c];
                const float wi        = inv_mass[i];
                const float wj        = inv_mass[j];
                const float wsum      = wi + wj;
                if (wsum <= 0.0f) {
                    lambda[c] = 0.0f;
                    continue;
                }

                const float diff_x = px[i] - px[j];
                const float diff_y = py[i] - py[j];
                const float diff_z = pz[i] - pz[j];
                const float len_sq = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
                if (len_sq <= std::numeric_limits<float>::epsilon()) {
                    continue;
                }
                const float len         = std::sqrt(len_sq);
                const float constraint  = len - rest[c];
                const float alpha_tilde = compliance[c] * inv_dt_sq;
                const float denom       = wsum + alpha_tilde;
                if (denom <= 0.0f) {
                    continue;
                }

                const float lambda_prev  = lambda[c];
                const float delta_lambda = (-constraint - alpha_tilde * lambda_prev) / denom;
                const float grad_scale   = delta_lambda / len;
                const float grad_x       = diff_x * grad_scale;
                const float grad_y       = diff_y * grad_scale;
                const float grad_z       = diff_z * grad_scale;

                lambda[c] = lambda_prev + delta_lambda;
                alpha[c]  = alpha_tilde;

                if (wi > 0.0f) {
                    apply_delta(px, py, pz, i, grad_x, grad_y, grad_z, wi);
                }
                if (wj > 0.0f) {
                    apply_delta(px, py, pz, j, grad_x, grad_y, grad_z, -wj);
                }
            }
        });
    }
}

} // namespace

void xpbd_step_native(ClothData& cloth, const XPBDParams& params) {
    if (params.substeps <= 0 || params.time_step <= 0.0f) {
        return;
    }

    const float dt             = params.time_step / static_cast<float>(params.substeps);
    const float inv_dt         = 1.0f / dt;
    const float inv_dt_sq      = inv_dt * inv_dt;
    const bool use_damping     = params.velocity_damping > 0.0f;
    const float damping_factor = use_damping ? std::clamp(1.0f - params.velocity_damping, 0.0f, 1.0f) : 1.0f;

    auto particles = cloth.particles();
    DistanceView dist_view{};
    const size_t edge_count = cloth.num_edges();
    if (params.enable_distance_constraints && edge_count > 0) {
        dist_view = cloth.distance();
    }

    std::span<float> px       = particles.px.span();
    std::span<float> py       = particles.py.span();
    std::span<float> pz       = particles.pz.span();
    std::span<float> vx       = particles.vx.span();
    std::span<float> vy       = particles.vy.span();
    std::span<float> vz       = particles.vz.span();
    std::span<float> inv_mass = particles.inv_mass.span();
    std::span<u8> pinned       = particles.pinned.span();

    IntegrationScratch scratch;
    scratch.px.resize(px.size());
    scratch.py.resize(py.size());
    scratch.pz.resize(pz.size());

    const float gx = params.gravity[0];
    const float gy = params.gravity[1];
    const float gz = params.gravity[2];

    for (int step_index = 0; step_index < params.substeps; ++step_index) {
        for (size_t i = 0; i < px.size(); ++i) {
            scratch.px[i] = px[i];
            scratch.py[i] = py[i];
            scratch.pz[i] = pz[i];

            if (pinned[i]) {
                vx[i] = vy[i] = vz[i] = 0.0f;
                continue;
            }

            vx[i] += gx * dt;
            vy[i] += gy * dt;
            vz[i] += gz * dt;

            px[i] += vx[i] * dt;
            py[i] += vy[i] * dt;
            pz[i] += vz[i] * dt;
        }

        if (should_process_distance(params, dist_view, edge_count)) {
            solve_distance_constraints_serial(dist_view, px, py, pz, inv_mass, inv_dt_sq);
        }

        for (size_t i = 0; i < px.size(); ++i) {
            if (pinned[i]) {
                px[i] = scratch.px[i];
                py[i] = scratch.py[i];
                pz[i] = scratch.pz[i];
                vx[i] = vy[i] = vz[i] = 0.0f;
                continue;
            }

            vx[i] = (px[i] - scratch.px[i]) * inv_dt;
            vy[i] = (py[i] - scratch.py[i]) * inv_dt;
            vz[i] = (pz[i] - scratch.pz[i]) * inv_dt;

            if (use_damping && damping_factor < 1.0f) {
                vx[i] *= damping_factor;
                vy[i] *= damping_factor;
                vz[i] *= damping_factor;
            }
        }
    }
}

void xpbd_step_tbb(ClothData& cloth, const XPBDParams& params) {
    if (params.substeps <= 0 || params.time_step <= 0.0f) {
        return;
    }

    const float dt             = params.time_step / static_cast<float>(params.substeps);
    const float inv_dt         = 1.0f / dt;
    const float inv_dt_sq      = inv_dt * inv_dt;
    const bool use_damping     = params.velocity_damping > 0.0f;
    const float damping_factor = use_damping ? std::clamp(1.0f - params.velocity_damping, 0.0f, 1.0f) : 1.0f;

    auto particles = cloth.particles();
    DistanceView dist_view{};
    const size_t edge_count = cloth.num_edges();
    if (params.enable_distance_constraints && edge_count > 0) {
        dist_view = cloth.distance();
    }

    std::span<float> px       = particles.px.span();
    std::span<float> py       = particles.py.span();
    std::span<float> pz       = particles.pz.span();
    std::span<float> vx       = particles.vx.span();
    std::span<float> vy       = particles.vy.span();
    std::span<float> vz       = particles.vz.span();
    std::span<float> inv_mass = particles.inv_mass.span();
    std::span<u8> pinned       = particles.pinned.span();

    IntegrationScratch scratch;
    scratch.px.resize(px.size());
    scratch.py.resize(py.size());
    scratch.pz.resize(pz.size());

    const float gx = params.gravity[0];
    const float gy = params.gravity[1];
    const float gz = params.gravity[2];

    for (int step_index = 0; step_index < params.substeps; ++step_index) {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, px.size(), 256), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                scratch.px[i] = px[i];
                scratch.py[i] = py[i];
                scratch.pz[i] = pz[i];

                if (pinned[i]) {
                    vx[i] = vy[i] = vz[i] = 0.0f;
                    continue;
                }

                vx[i] += gx * dt;
                vy[i] += gy * dt;
                vz[i] += gz * dt;

                px[i] += vx[i] * dt;
                py[i] += vy[i] * dt;
                pz[i] += vz[i] * dt;
            }
        });

        if (should_process_distance(params, dist_view, edge_count)) {
            solve_distance_constraints_tbb(dist_view, px, py, pz, inv_mass, inv_dt_sq);
        }

        tbb::parallel_for(tbb::blocked_range<size_t>(0, px.size(), 256), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                if (pinned[i]) {
                    px[i] = scratch.px[i];
                    py[i] = scratch.py[i];
                    pz[i] = scratch.pz[i];
                    vx[i] = vy[i] = vz[i] = 0.0f;
                    continue;
                }

                vx[i] = (px[i] - scratch.px[i]) * inv_dt;
                vy[i] = (py[i] - scratch.py[i]) * inv_dt;
                vz[i] = (pz[i] - scratch.pz[i]) * inv_dt;

                if (use_damping && damping_factor < 1.0f) {
                    vx[i] *= damping_factor;
                    vy[i] *= damping_factor;
                    vz[i] *= damping_factor;
                }
            }
        });
    }
}

#ifdef __AVX2__
namespace {
inline __m256i load_pinned_mask8(const u8* base) {
    __m128i bytes = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(base));
    __m128i lo    = _mm_cvtepu8_epi32(bytes);
    __m128i hi    = _mm_cvtepu8_epi32(_mm_srli_si128(bytes, 4));
    __m256i pins  = _mm256_insertf128_si256(_mm256_castsi128_si256(lo), hi, 1);
    return pins;
}
} // namespace
#endif

void xpbd_step_avx2(ClothData& cloth, const XPBDParams& params) {
#ifndef __AVX2__
    xpbd_step_native(cloth, params);
#else
    if (params.substeps <= 0 || params.time_step <= 0.0f) {
        return;
    }

    const float dt             = params.time_step / static_cast<float>(params.substeps);
    const float inv_dt         = 1.0f / dt;
    const float inv_dt_sq      = inv_dt * inv_dt;
    const bool use_damping     = params.velocity_damping > 0.0f;
    const float damping_factor = use_damping ? std::clamp(1.0f - params.velocity_damping, 0.0f, 1.0f) : 1.0f;

    auto particles = cloth.particles();
    DistanceView dist_view{};
    const size_t edge_count = cloth.num_edges();
    if (params.enable_distance_constraints && edge_count > 0) {
        dist_view = cloth.distance();
    }

    std::span<float> px       = particles.px.span();
    std::span<float> py       = particles.py.span();
    std::span<float> pz       = particles.pz.span();
    std::span<float> vx       = particles.vx.span();
    std::span<float> vy       = particles.vy.span();
    std::span<float> vz       = particles.vz.span();
    std::span<float> inv_mass = particles.inv_mass.span();
    std::span<u8> pinned      = particles.pinned.span();

    IntegrationScratch scratch;
    scratch.px.resize(px.size());
    scratch.py.resize(py.size());
    scratch.pz.resize(pz.size());

    const float gx = params.gravity[0];
    const float gy = params.gravity[1];
    const float gz = params.gravity[2];

    const __m256 dt_vec      = _mm256_set1_ps(dt);
    const __m256 gx_dt_vec   = _mm256_set1_ps(gx * dt);
    const __m256 gy_dt_vec   = _mm256_set1_ps(gy * dt);
    const __m256 gz_dt_vec   = _mm256_set1_ps(gz * dt);
    const __m256 zero        = _mm256_setzero_ps();
    const __m256 inv_dt_vec  = _mm256_set1_ps(inv_dt);
    const __m256 damping_vec = _mm256_set1_ps(damping_factor);
    const __m256i zero_i     = _mm256_setzero_si256();

    for (int step_index = 0; step_index < params.substeps; ++step_index) {
        size_t i = 0;
        for (; i + 8 <= px.size(); i += 8) {
            __m256 px_old = _mm256_loadu_ps(px.data() + i);
            __m256 py_old = _mm256_loadu_ps(py.data() + i);
            __m256 pz_old = _mm256_loadu_ps(pz.data() + i);
            _mm256_storeu_ps(scratch.px.data() + i, px_old);
            _mm256_storeu_ps(scratch.py.data() + i, py_old);
            _mm256_storeu_ps(scratch.pz.data() + i, pz_old);

            __m256 vx_val = _mm256_loadu_ps(vx.data() + i);
            __m256 vy_val = _mm256_loadu_ps(vy.data() + i);
            __m256 vz_val = _mm256_loadu_ps(vz.data() + i);

            vx_val = _mm256_add_ps(vx_val, gx_dt_vec);
            vy_val = _mm256_add_ps(vy_val, gy_dt_vec);
            vz_val = _mm256_add_ps(vz_val, gz_dt_vec);

            __m256 px_new = _mm256_add_ps(_mm256_mul_ps(vx_val, dt_vec), px_old);
            __m256 py_new = _mm256_add_ps(_mm256_mul_ps(vy_val, dt_vec), py_old);
            __m256 pz_new = _mm256_add_ps(_mm256_mul_ps(vz_val, dt_vec), pz_old);

            __m256i pins = load_pinned_mask8(pinned.data() + i);
            __m256 mask  = _mm256_castsi256_ps(_mm256_cmpgt_epi32(pins, zero_i));

            vx_val = _mm256_blendv_ps(vx_val, zero, mask);
            vy_val = _mm256_blendv_ps(vy_val, zero, mask);
            vz_val = _mm256_blendv_ps(vz_val, zero, mask);

            px_new = _mm256_blendv_ps(px_new, px_old, mask);
            py_new = _mm256_blendv_ps(py_new, py_old, mask);
            pz_new = _mm256_blendv_ps(pz_new, pz_old, mask);

            _mm256_storeu_ps(px.data() + i, px_new);
            _mm256_storeu_ps(py.data() + i, py_new);
            _mm256_storeu_ps(pz.data() + i, pz_new);
            _mm256_storeu_ps(vx.data() + i, vx_val);
            _mm256_storeu_ps(vy.data() + i, vy_val);
            _mm256_storeu_ps(vz.data() + i, vz_val);
        }
        for (; i < px.size(); ++i) {
            scratch.px[i] = px[i];
            scratch.py[i] = py[i];
            scratch.pz[i] = pz[i];

            if (pinned[i]) {
                vx[i] = vy[i] = vz[i] = 0.0f;
                continue;
            }

            vx[i] += gx * dt;
            vy[i] += gy * dt;
            vz[i] += gz * dt;

            px[i] += vx[i] * dt;
            py[i] += vy[i] * dt;
            pz[i] += vz[i] * dt;
        }

        if (should_process_distance(params, dist_view, edge_count)) {
            solve_distance_constraints_serial(dist_view, px, py, pz, inv_mass, inv_dt_sq);
        }

        i = 0;
        for (; i + 8 <= px.size(); i += 8) {
            __m256 px_curr = _mm256_loadu_ps(px.data() + i);
            __m256 py_curr = _mm256_loadu_ps(py.data() + i);
            __m256 pz_curr = _mm256_loadu_ps(pz.data() + i);

            __m256 px_prev = _mm256_loadu_ps(scratch.px.data() + i);
            __m256 py_prev = _mm256_loadu_ps(scratch.py.data() + i);
            __m256 pz_prev = _mm256_loadu_ps(scratch.pz.data() + i);

            __m256 vx_new = _mm256_mul_ps(_mm256_sub_ps(px_curr, px_prev), inv_dt_vec);
            __m256 vy_new = _mm256_mul_ps(_mm256_sub_ps(py_curr, py_prev), inv_dt_vec);
            __m256 vz_new = _mm256_mul_ps(_mm256_sub_ps(pz_curr, pz_prev), inv_dt_vec);

            if (use_damping && damping_factor < 1.0f) {
                vx_new = _mm256_mul_ps(vx_new, damping_vec);
                vy_new = _mm256_mul_ps(vy_new, damping_vec);
                vz_new = _mm256_mul_ps(vz_new, damping_vec);
            }

            __m256i pins = load_pinned_mask8(pinned.data() + i);
            __m256 mask  = _mm256_castsi256_ps(_mm256_cmpgt_epi32(pins, zero_i));

            vx_new = _mm256_blendv_ps(vx_new, zero, mask);
            vy_new = _mm256_blendv_ps(vy_new, zero, mask);
            vz_new = _mm256_blendv_ps(vz_new, zero, mask);

            px_curr = _mm256_blendv_ps(px_curr, px_prev, mask);
            py_curr = _mm256_blendv_ps(py_curr, py_prev, mask);
            pz_curr = _mm256_blendv_ps(pz_curr, pz_prev, mask);

            _mm256_storeu_ps(vx.data() + i, vx_new);
            _mm256_storeu_ps(vy.data() + i, vy_new);
            _mm256_storeu_ps(vz.data() + i, vz_new);
            _mm256_storeu_ps(px.data() + i, px_curr);
            _mm256_storeu_ps(py.data() + i, py_curr);
            _mm256_storeu_ps(pz.data() + i, pz_curr);
        }
        for (; i < px.size(); ++i) {
            if (pinned[i]) {
                px[i] = scratch.px[i];
                py[i] = scratch.py[i];
                pz[i] = scratch.pz[i];
                vx[i] = vy[i] = vz[i] = 0.0f;
                continue;
            }

            vx[i] = (px[i] - scratch.px[i]) * inv_dt;
            vy[i] = (py[i] - scratch.py[i]) * inv_dt;
            vz[i] = (pz[i] - scratch.pz[i]) * inv_dt;

            if (use_damping && damping_factor < 1.0f) {
                vx[i] *= damping_factor;
                vy[i] *= damping_factor;
                vz[i] *= damping_factor;
            }
        }
    }
#endif
}

} // namespace HinaPE

