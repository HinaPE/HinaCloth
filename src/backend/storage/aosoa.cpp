#include "aosoa.h"
#include <algorithm>

namespace sim {
    void storage_pack_soa_to_aosoa(const float* x, const float* y, const float* z,
                                   std::size_t n, std::size_t block, float* out_base) noexcept {
        if (!out_base || !x || !y || !z || block == 0) return;
        std::size_t nb = (n + block - 1) / block;
        for (std::size_t bi = 0; bi < nb; ++bi) {
            float* base = out_base + bi * (3 * block);
            float* xb = base + 0;
            float* yb = base + block;
            float* zb = base + 2 * block;
            std::size_t start = bi * block;
            std::size_t end   = std::min(start + block, n);
            std::size_t len   = end - start;
            for (std::size_t k = 0; k < len; ++k) {
                xb[k] = x[start + k];
                yb[k] = y[start + k];
                zb[k] = z[start + k];
            }
            for (std::size_t k = len; k < block; ++k) {
                xb[k] = 0.0f; yb[k] = 0.0f; zb[k] = 0.0f;
            }
        }
    }

    void storage_unpack_aosoa_to_soa(const float* base, std::size_t n, std::size_t block,
                                     float* x, float* y, float* z) noexcept {
        if (!base || !x || !y || !z || block == 0) return;
        std::size_t nb = (n + block - 1) / block;
        for (std::size_t bi = 0; bi < nb; ++bi) {
            const float* b = base + bi * (3 * block);
            const float* xb = b + 0;
            const float* yb = b + block;
            const float* zb = b + 2 * block;
            std::size_t start = bi * block;
            std::size_t end   = std::min(start + block, n);
            for (std::size_t k = 0; k < end - start; ++k) {
                x[start + k] = xb[k];
                y[start + k] = yb[k];
                z[start + k] = zb[k];
            }
        }
    }
}
