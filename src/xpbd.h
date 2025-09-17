#ifndef HINACLOTH_XPBD_H
#define HINACLOTH_XPBD_H

#include "cloth_data.h"

namespace HinaPE {

    struct XPBDParams {
        // Global simulation parameters
        float time_step{1.0f / 60.0f};
        int   substeps{1};
        int   solver_iterations{8};

        // External acceleration (e.g., gravity)
        float gravity_x{0.0f};
        float gravity_y{-9.81f};
        float gravity_z{0.0f};

        // Simple velocity damping factor in [0, 1)
        float velocity_damping{0.0f};

        // Constraint toggles
        bool enable_distance_constraints{true};
        bool enable_bending_constraints{false};
        bool enable_triangle_elasticity{false};

        // XPBD controls
        // If true, reset lambdas for hard constraints (compliance==0) at EACH substep.
        // If false, reset once per time-step (recommended and default).
        bool reset_hard_lambda_each_substep{false};

        // If true, process distance constraints grouped by the provided color value (u8),
        // executing colors in ascending order for improved convergence without threading.
        bool use_color_ordering{true};
    };

    // Native XPBD step advancing all enabled constraints once per call.
    // This function updates positions and velocities in-place.
    void xpbd_step_native(ClothData& cloth, const XPBDParams& params);

} // namespace HinaPE

#endif // HINACLOTH_XPBD_H
