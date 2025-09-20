#include "distance_aosoa.h"
#include <cmath>
#if defined(HINACLOTH_HAVE_AVX2)
  #include <immintrin.h>
#endif

namespace sim {
    static inline void project_edge_scalar_aosoa(uint32_t ai, uint32_t bi,
                                                  AoSoAView3& pos,
                                                  float rest,
                                                  const float* inv_mass,
                                                  float* lambda,
                                                  float alpha) {
        if (ai >= pos.n || bi >= pos.n) return;
        float ax, ay, az, bx, by, bz;
        storage_aosoa_read3(pos, ai, ax, ay, az);
        storage_aosoa_read3(pos, bi, bx, by, bz);
        float dx = bx - ax, dy = by - ay, dz = bz - az;
        float len = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (len <= 1e-8f) return;
        float C = len - rest;
        float wi = inv_mass ? inv_mass[ai] : 1.0f;
        float wj = inv_mass ? inv_mass[bi] : 1.0f;
        float denom = wi + wj + alpha;
        if (denom <= 0.0f) return;
        float lambda_prev = lambda ? *lambda : 0.0f;
        float dlambda = -(C + alpha * lambda_prev) / denom;
        float s = dlambda / len;
        float cx = s * dx, cy = s * dy, cz = s * dz;
        if (wi > 0.0f) storage_aosoa_axpy3(pos, ai, wi * cx, wi * cy, wi * cz);
        if (wj > 0.0f) storage_aosoa_axpy3(pos, bi, -wj * cx, -wj * cy, -wj * cz);
        if (lambda) *lambda = lambda_prev + dlambda;
    }

    void kernel_distance_project_aosoa(const uint32_t* edges, std::size_t m,
                                       AoSoAView3& pos_blk,
                                       const float* rest,
                                       const float* inv_mass,
                                       float* lambda_edge,
                                       const float* alpha_edge,
                                       int iterations,
                                       float alpha,
                                       float /*dt*/) {
        for (int it = 0; it < iterations; ++it) {
        #if defined(HINACLOTH_HAVE_AVX2)
            const int W = 8;
            std::size_t e = 0;
            for (; e + W <= m; e += W) {
                alignas(32) uint32_t a_idx[W], b_idx[W];
                for (int k = 0; k < W; ++k) { a_idx[k] = edges[2*(e+k) + 0]; b_idx[k] = edges[2*(e+k) + 1]; }
                // Compute AoSoA offsets per-lane for x/y/z
                alignas(32) int ax_off[W], ay_off[W], az_off[W];
                alignas(32) int bx_off[W], by_off[W], bz_off[W];
                for (int k = 0; k < W; ++k) {
                    uint32_t ai = a_idx[k]; uint32_t bi = b_idx[k];
                    std::size_t abi = ai / pos_blk.block; std::size_t ali = ai % pos_blk.block;
                    std::size_t bbi = bi / pos_blk.block; std::size_t bli = bi % pos_blk.block;
                    std::size_t stride = pos_blk.stride;
                    ax_off[k] = (int)(abi * stride + 0 * pos_blk.block + ali);
                    ay_off[k] = (int)(abi * stride + 1 * pos_blk.block + ali);
                    az_off[k] = (int)(abi * stride + 2 * pos_blk.block + ali);
                    bx_off[k] = (int)(bbi * stride + 0 * pos_blk.block + bli);
                    by_off[k] = (int)(bbi * stride + 1 * pos_blk.block + bli);
                    bz_off[k] = (int)(bbi * stride + 2 * pos_blk.block + bli);
                }
                __m256 ax = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)ax_off), 4);
                __m256 ay = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)ay_off), 4);
                __m256 az = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)az_off), 4);
                __m256 bx = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)bx_off), 4);
                __m256 by = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)by_off), 4);
                __m256 bz = _mm256_i32gather_ps(pos_blk.base, _mm256_load_si256((const __m256i*)bz_off), 4);
                __m256 dx = _mm256_sub_ps(bx, ax);
                __m256 dy = _mm256_sub_ps(by, ay);
                __m256 dz = _mm256_sub_ps(bz, az);
                __m256 len2 = _mm256_fmadd_ps(dz, dz, _mm256_fmadd_ps(dy, dy, _mm256_mul_ps(dx, dx)));
                // Avoid div by zero
                __m256 inv_len = _mm256_rsqrt_ps(_mm256_max_ps(len2, _mm256_set1_ps(1e-16f)));
                // refine once
                const __m256 three_halfs = _mm256_set1_ps(1.5f);
                const __m256 half        = _mm256_set1_ps(0.5f);
                __m256 t = _mm256_mul_ps(half, _mm256_mul_ps(len2, _mm256_mul_ps(inv_len, inv_len)));
                inv_len = _mm256_mul_ps(inv_len, _mm256_sub_ps(three_halfs, t));
                __m256 len     = _mm256_mul_ps(len2, inv_len);
                __m256 C       = _mm256_sub_ps(len, _mm256_loadu_ps(rest + e));
                __m256 wi = inv_mass ? _mm256_i32gather_ps(inv_mass, _mm256_load_si256((const __m256i*)a_idx), 4) : _mm256_set1_ps(1.0f);
                __m256 wj = inv_mass ? _mm256_i32gather_ps(inv_mass, _mm256_load_si256((const __m256i*)b_idx), 4) : _mm256_set1_ps(1.0f);
                __m256 a_e = alpha_edge ? _mm256_loadu_ps(alpha_edge + e) : _mm256_set1_ps(alpha);
                __m256 denom = _mm256_add_ps(_mm256_add_ps(wi, wj), a_e);
                __m256 lambda_prev = lambda_edge ? _mm256_loadu_ps(lambda_edge + e) : _mm256_set1_ps(0.0f);
                __m256 dlambda = _mm256_div_ps(_mm256_sub_ps(_mm256_set1_ps(0.0f), _mm256_add_ps(C, _mm256_mul_ps(a_e, lambda_prev))), denom);
                __m256 s = _mm256_mul_ps(dlambda, inv_len);
                __m256 cx = _mm256_mul_ps(s, dx);
                __m256 cy = _mm256_mul_ps(s, dy);
                __m256 cz = _mm256_mul_ps(s, dz);
                // Scatter by lanes
                alignas(32) float ax_s[W], ay_s[W], az_s[W], bx_s[W], by_s[W], bz_s[W];
                alignas(32) float cx_s[W], cy_s[W], cz_s[W], wi_s[W], wj_s[W], lnew[W], lprev[W];
                _mm256_store_ps(ax_s, ax); _mm256_store_ps(ay_s, ay); _mm256_store_ps(az_s, az);
                _mm256_store_ps(bx_s, bx); _mm256_store_ps(by_s, by); _mm256_store_ps(bz_s, bz);
                _mm256_store_ps(cx_s, cx); _mm256_store_ps(cy_s, cy); _mm256_store_ps(cz_s, cz);
                _mm256_store_ps(wi_s, wi); _mm256_store_ps(wj_s, wj);
                if (lambda_edge) { _mm256_store_ps(lprev, lambda_prev); _mm256_store_ps(lnew, _mm256_add_ps(lambda_prev, dlambda)); }
                for (int k = 0; k < W; ++k) {
                    uint32_t a = a_idx[k], b = b_idx[k];
                    float wi_k = wi_s[k], wj_k = wj_s[k];
                    float cxk = cx_s[k], cyk = cy_s[k], czk = cz_s[k];
                    if (a < pos_blk.n && wi_k > 0.0f) storage_aosoa_axpy3(pos_blk, a, wi_k * cxk, wi_k * cyk, wi_k * czk);
                    if (b < pos_blk.n && wj_k > 0.0f) storage_aosoa_axpy3(pos_blk, b, -wj_k * cxk, -wj_k * cyk, -wj_k * czk);
                    if (lambda_edge) lambda_edge[e + k] = lnew[k];
                }
            }
            // tail scalar
            for (; e < m; ++e) {
                uint32_t a = edges[2*e+0], b = edges[2*e+1];
                float a_e = alpha_edge ? alpha_edge[e] : alpha;
                project_edge_scalar_aosoa(a, b, pos_blk, rest[e], inv_mass, lambda_edge ? (lambda_edge + e) : nullptr, a_e);
            }
        #else
            for (int it2 = 0; it2 < 1; ++it2) (void)it2; // keep structure similar
            for (std::size_t e = 0; e < m; ++e) {
                uint32_t a = edges[2*e+0], b = edges[2*e+1];
                float a_e = alpha_edge ? alpha_edge[e] : alpha;
                project_edge_scalar_aosoa(a, b, pos_blk, rest[e], inv_mass, lambda_edge ? (lambda_edge + e) : nullptr, a_e);
            }
        #endif
        }
    }
}
