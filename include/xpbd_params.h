#ifndef HINACLOTH_INCLUDE_XPBD_PARAMS_H
#define HINACLOTH_INCLUDE_XPBD_PARAMS_H

#include "cloth_types.h"

#include <algorithm>
#include <array>
#include <utility>

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

struct XPBDSolverSettings {
    float clamped_dt{0.0f};
    int substeps{1};
    float step_dt{0.0f};
    float inv_step_dt{0.0f};
    float alpha_dt{0.0f};
    int iterations{1};
    float velocity_scale{1.0f};
    float max_correction{0.0f};
    float lambda_decay{1.0f};
    bool warmstart{false};
    bool write_debug{false};
    std::array<float, 3> compliance_scale{1.0f, 1.0f, 1.0f};
};

[[nodiscard]] inline XPBDSolverSettings makeSolverSettings(float dt, const XPBDParams& params) noexcept {
    XPBDSolverSettings settings{};
    settings.clamped_dt = std::clamp(dt, params.min_dt, params.max_dt);
    settings.substeps = std::max(1, params.substeps);
    settings.step_dt = settings.substeps > 0 ? settings.clamped_dt / static_cast<float>(settings.substeps) : 0.0f;
    if (settings.step_dt > 0.0f) {
        settings.inv_step_dt = 1.0f / settings.step_dt;
        settings.alpha_dt = settings.inv_step_dt * settings.inv_step_dt;
    }
    settings.iterations = std::max(1, params.iterations);
    settings.velocity_scale = params.velocity_damping > 0.0f
        ? std::max(0.0f, 1.0f - params.velocity_damping)
        : 1.0f;
    settings.max_correction = params.max_correction;
    settings.lambda_decay = params.lambda_decay;
    settings.warmstart = params.warmstart;
    settings.write_debug = params.write_debug_fields != 0;
    const float base = params.compliance_scale_all;
    settings.compliance_scale = {
        base * params.compliance_scale_structural,
        base * params.compliance_scale_shear,
        base * params.compliance_scale_bending
    };
    return settings;
}

[[nodiscard]] inline float complianceScale(const XPBDSolverSettings& settings, ConstraintType type) noexcept {
    const auto index = static_cast<std::size_t>(std::to_underlying(type));
    return index < settings.compliance_scale.size() ? settings.compliance_scale[index] : settings.compliance_scale.front();
}

} // namespace HinaPE

#endif  // HINACLOTH_INCLUDE_XPBD_PARAMS_H
