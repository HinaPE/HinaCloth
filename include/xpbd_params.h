// XPBD parameters (shared across layouts)
#pragma once

namespace HinaPE {

struct XPBDParams {
    float ax{0.0f};
    float ay{-9.81f};
    float az{0.0f};

    int iterations{10};
    int substeps{1};
    float min_dt{1.0f / 400.0f};
    float max_dt{1.0f / 30.0f};

    float velocity_damping{0.0f};

    bool warmstart{false};
    float lambda_decay{1.0f};

    float compliance_scale_all{1.0f};
    float compliance_scale_structural{1.0f};
    float compliance_scale_shear{1.0f};
    float compliance_scale_bending{1.0f};

    float max_correction{0.0f};

    int write_debug_fields{0};
};

} // namespace HinaPE
