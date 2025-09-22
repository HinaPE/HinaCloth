#include "attachment.h"
#include <algorithm>

namespace sim {
    void kernel_attachment_apply(SoAView3& pos,
                                 const float* w,
                                 const float* tx,
                                 const float* ty,
                                 const float* tz,
                                 const float* inv_mass,
                                 std::size_t n) {
        if (!w || !tx || !ty || !tz) return;
        for (std::size_t i = 0; i < n; ++i) {
            float wi = w[i];
            if (wi <= 0.0f) continue;
            if (inv_mass && inv_mass[i] == 0.0f) continue;
            wi = std::clamp(wi, 0.0f, 1.0f);
            float ax = tx[i] - pos.x[i];
            float ay = ty[i] - pos.y[i];
            float az = tz[i] - pos.z[i];
            pos.x[i] += wi * ax;
            pos.y[i] += wi * ay;
            pos.z[i] += wi * az;
        }
    }
}
