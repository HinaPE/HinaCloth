#include "attachment.h"
#include <algorithm>
#include "backend/storage/view3.h"

namespace sim {
    template <class View3>
    static void attachment_apply_generic(View3& pos,
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
            float px,py,pz; view_read3(pos, i, px,py,pz);
            float ax = tx[i] - px;
            float ay = ty[i] - py;
            float az = tz[i] - pz;
            view_axpy3(pos, i, wi*ax, wi*ay, wi*az);
        }
    }

    void kernel_attachment_apply(SoAView3& pos,
                                 const float* w,
                                 const float* tx,
                                 const float* ty,
                                 const float* tz,
                                 const float* inv_mass,
                                 std::size_t n) {
        attachment_apply_generic(pos, w, tx, ty, tz, inv_mass, n);
    }

    void kernel_attachment_apply_aosoa(AoSoAView3& pos,
                                       const float* w,
                                       const float* tx,
                                       const float* ty,
                                       const float* tz,
                                       const float* inv_mass,
                                       std::size_t n) {
        attachment_apply_generic(pos, w, tx, ty, tz, inv_mass, n);
    }

    void kernel_attachment_apply_aos(AoSView3& pos,
                                     const float* w,
                                     const float* tx,
                                     const float* ty,
                                     const float* tz,
                                     const float* inv_mass,
                                     std::size_t n) {
        attachment_apply_generic(pos, w, tx, ty, tz, inv_mass, n);
    }
}
