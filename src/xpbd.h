#ifndef HINACLOTH_XPBD_H
#define HINACLOTH_XPBD_H

#include "cloth_data.h"

#include <array>

namespace HinaPE {

    struct XPBDParams {
        std::array<float, 3> gravity{0.0f, -9.81f, 0.0f};
        float time_step{1.0f / 60.0f};
        int substeps{1};
        int solver_iterations{8};
        bool enable_distance_constraints{true};
        bool enable_bending_constraints{false};
        float velocity_damping{0.0f};
    };

    void xpbd_step_tbb(ClothData& cloth, const XPBDParams& params);
    void xpbd_step_native(ClothData& cloth, const XPBDParams& params);
    void xpbd_step_avx2(ClothData& cloth, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_XPBD_H