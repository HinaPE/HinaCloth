#include "soa.h"
#include "aos.h"
#include <algorithm>

namespace sim {
    void storage_pack_soa_to_aos(const float* x, const float* y, const float* z, std::size_t n, float* out_base, std::size_t stride) noexcept {
        if (!out_base || !x || !y || !z || stride < 3) return;
        for (std::size_t i = 0; i < n; ++i) {
            float* p = out_base + i * stride;
            p[0] = x[i]; p[1] = y[i]; p[2] = z[i];
        }
    }

    void storage_unpack_aos_to_soa(const float* base, std::size_t n, float* x, float* y, float* z, std::size_t stride) noexcept {
        if (!base || !x || !y || !z || stride < 3) return;
        for (std::size_t i = 0; i < n; ++i) {
            const float* p = base + i * stride;
            x[i] = p[0]; y[i] = p[1]; z[i] = p[2];
        }
    }
}
