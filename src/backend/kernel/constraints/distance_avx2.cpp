#include "distance_avx2.h"
#include "distance.h"
#include <cmath>
#if defined(HINACLOTH_HAVE_AVX2)
  #include <immintrin.h>
#endif

namespace sim {
#if defined(HINACLOTH_HAVE_AVX2)
    static inline __m256 safe_rsqrt(__m256 x) {
        // Avoid div-by-zero: rsqrt approximation refined by one Newton-Raphson step
        __m256 rs = _mm256_rsqrt_ps(x);
        // refine: rs = rs * (1.5 - 0.5*x*rs*rs)
        const __m256 three_halfs = _mm256_set1_ps(1.5f);
        const __m256 half        = _mm256_set1_ps(0.5f);
        __m256 t = _mm256_mul_ps(half, _mm256_mul_ps(x, _mm256_mul_ps(rs, rs)));
        rs = _mm256_mul_ps(rs, _mm256_sub_ps(three_halfs, t));
        return rs;
    }

    void kernel_distance_project_avx2(const uint32_t* edges, std::size_t m,
                                      SoAView3& pos,
                                      const float* rest,
                                      const float* inv_mass,
                                      float* lambda_edge,
                                      const float* alpha_edge,
                                      int iterations,
                                      float alpha,
                                      float dt) {
        (void) dt;
        const int W = 8; // AVX2 width
        for (int it = 0; it < iterations; ++it) {
            std::size_t e = 0;
            for (; e + W <= m; e += W) {
                alignas(32) uint32_t a_idx[W], b_idx[W];
                for (int k = 0; k < W; ++k) {
                    a_idx[k] = edges[2*(e + k) + 0];
                    b_idx[k] = edges[2*(e + k) + 1];
                }
                __m256i ia = _mm256_load_si256((const __m256i*) a_idx);
                __m256i ib = _mm256_load_si256((const __m256i*) b_idx);
                // gather positions
                __m256 ax = _mm256_i32gather_ps(pos.x, ia, 4);
                __m256 ay = _mm256_i32gather_ps(pos.y, ia, 4);
                __m256 az = _mm256_i32gather_ps(pos.z, ia, 4);
                __m256 bx = _mm256_i32gather_ps(pos.x, ib, 4);
                __m256 by = _mm256_i32gather_ps(pos.y, ib, 4);
                __m256 bz = _mm256_i32gather_ps(pos.z, ib, 4);
                __m256 dx = _mm256_sub_ps(bx, ax);
                __m256 dy = _mm256_sub_ps(by, ay);
                __m256 dz = _mm256_sub_ps(bz, az);
                __m256 len2 = _mm256_fmadd_ps(dz, dz, _mm256_fmadd_ps(dy, dy, _mm256_mul_ps(dx, dx)));
                __m256 inv_len = safe_rsqrt(_mm256_max_ps(len2, _mm256_set1_ps(1e-16f)));
                __m256 len     = _mm256_mul_ps(len2, inv_len); // len = len2 * rsqrt(len2)
                __m256 C       = _mm256_sub_ps(len, _mm256_loadu_ps(rest + e));
                __m256 wi = inv_mass ? _mm256_i32gather_ps(inv_mass, ia, 4) : _mm256_set1_ps(1.0f);
                __m256 wj = inv_mass ? _mm256_i32gather_ps(inv_mass, ib, 4) : _mm256_set1_ps(1.0f);
                __m256 a_e = alpha_edge ? _mm256_loadu_ps(alpha_edge + e) : _mm256_set1_ps(alpha);
                __m256 denom = _mm256_add_ps(_mm256_add_ps(wi, wj), a_e);
                __m256 lambda_prev = lambda_edge ? _mm256_loadu_ps(lambda_edge + e) : _mm256_set1_ps(0.0f);
                __m256 dlambda = _mm256_div_ps(_mm256_sub_ps(_mm256_set1_ps(0.0f), _mm256_add_ps(C, _mm256_mul_ps(a_e, lambda_prev))), denom);
                __m256 s = _mm256_mul_ps(dlambda, inv_len);
                __m256 cx = _mm256_mul_ps(s, dx);
                __m256 cy = _mm256_mul_ps(s, dy);
                __m256 cz = _mm256_mul_ps(s, dz);
                // write-back per-lane (no scatter in AVX2)
                alignas(32) float cx_s[W], cy_s[W], cz_s[W], wi_s[W], wj_s[W], lnew[W], lprev[W];
                _mm256_store_ps(cx_s, cx); _mm256_store_ps(cy_s, cy); _mm256_store_ps(cz_s, cz);
                _mm256_store_ps(wi_s, wi); _mm256_store_ps(wj_s, wj);
                if (lambda_edge) { _mm256_store_ps(lprev, lambda_prev); _mm256_store_ps(lnew, _mm256_add_ps(lambda_prev, dlambda)); }
                for (int k = 0; k < W; ++k) {
                    uint32_t a = a_idx[k], b = b_idx[k];
                    float wi_k = wi_s[k], wj_k = wj_s[k];
                    float cxk = cx_s[k], cyk = cy_s[k], czk = cz_s[k];
                    if (a < pos.n && wi_k > 0.0f) { pos.x[a] -= wi_k * cxk; pos.y[a] -= wi_k * cyk; pos.z[a] -= wi_k * czk; }
                    if (b < pos.n && wj_k > 0.0f) { pos.x[b] += wj_k * cxk; pos.y[b] += wj_k * cyk; pos.z[b] += wj_k * czk; }
                    if (lambda_edge) lambda_edge[e + k] = lnew[k];
                }
            }
            // tail
            if (e < m) {
                SoAView3 v = pos;
                kernel_distance_project(edges + 2*e, m - e, v, rest + e, inv_mass, lambda_edge ? (lambda_edge + e) : nullptr, alpha_edge ? (alpha_edge + e) : nullptr, 1, alpha, dt);
            }
        }
    }
#else
    void kernel_distance_project_avx2(const uint32_t* edges, std::size_t m,
                                      SoAView3& pos,
                                      const float* rest,
                                      const float* inv_mass,
                                      float* lambda_edge,
                                      const float* alpha_edge,
                                      int iterations,
                                      float alpha,
                                      float dt) {
        // Fallback to scalar when AVX2 not available at compile-time
        kernel_distance_project(edges, m, pos, rest, inv_mass, lambda_edge, alpha_edge, iterations, alpha, dt);
    }
#endif
}
