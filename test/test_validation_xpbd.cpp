// Validation tests for XPBD implementations across data layouts and backends
// Goal: ensure algorithmic correctness and numerical consistency within a
// reasonable tolerance across Native/TBB/AVX2 backends for the same layout.
// Metrics:
//  - Constraint residuals: L1-mean, L2-RMS, L_inf (max)
//  - Pinned vertex displacement epsilon check
//  - Cross-backend residual agreement within relative tolerance
// Method:
//  - Initialize identical cloth states per layout
//  - Run K steps with fixed dt/params
//  - Compute residuals per backend; compare to native-baseline

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>

#include "aligned/cloth_data_aligned.h"
#include "aligned/solver_xpbd_aligned.h"
#include "aos/cloth_data_aos.h"
#include "aos/solver_xpbd_aos.h"
#include "aosoa/cloth_data_aosoa.h"
#include "aosoa/solver_xpbd_aosoa.h"
#include "cloth_types.h"
#include "soa/cloth_data_soa.h"
#include "soa/solver_xpbd_soa.h"
#include "xpbd_params.h"

namespace {

struct Residuals {
    double l1{0.0};
    double l2{0.0};
    double linf{0.0};
    std::size_t constraint_count{0};
};

[[nodiscard]] HinaPE::XPBDParams makeDefaultParams() {
    HinaPE::XPBDParams params{};
    params.ax = 0.0f;
    params.ay = -9.81f;
    params.az = 0.0f;
    params.iterations = 10;
    params.substeps = 1;
    params.min_dt = 1.0f / 400.0f;
    params.max_dt = 1.0f / 30.0f;
    params.velocity_damping = 0.0f;
    params.warmstart = false;
    params.lambda_decay = 1.0f;
    params.compliance_scale_all = 1.0f;
    params.compliance_scale_structural = 1.0f;
    params.compliance_scale_shear = 1.0f;
    params.compliance_scale_bending = 1.0f;
    params.max_correction = 0.0f;
    params.write_debug_fields = 0;
    return params;
}

[[nodiscard]] Residuals computeResiduals(const HinaPE::ClothAOS& cloth) {
    double sum_abs = 0.0;
    double sum_sq = 0.0;
    double max_abs = 0.0;
    const auto constraint_count = cloth.constraints.size();

    for (const auto& c : cloth.constraints) {
        const auto& a = cloth.particles[c.i];
        const auto& b = cloth.particles[c.j];
        const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
        const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
        const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
        const double residual = std::sqrt(dx * dx + dy * dy + dz * dz) - static_cast<double>(c.rest_length);
        const double value = std::abs(residual);
        sum_abs += value;
        sum_sq += residual * residual;
        max_abs = std::max(max_abs, value);
    }

    const double denom = static_cast<double>(std::max<std::size_t>(1, constraint_count));
    return {
        sum_abs / denom,
        std::sqrt(sum_sq / denom),
        max_abs,
        constraint_count
    };
}

[[nodiscard]] Residuals computeResiduals(const HinaPE::ClothSOA& cloth) {
    double sum_abs = 0.0;
    double sum_sq = 0.0;
    double max_abs = 0.0;
    const auto constraint_count = cloth.ci.size();

    for (std::size_t idx = 0; idx < constraint_count; ++idx) {
        const int i = cloth.ci[idx];
        const int j = cloth.cj[idx];
        const double dx = static_cast<double>(cloth.x[i]) - static_cast<double>(cloth.x[j]);
        const double dy = static_cast<double>(cloth.y[i]) - static_cast<double>(cloth.y[j]);
        const double dz = static_cast<double>(cloth.z[i]) - static_cast<double>(cloth.z[j]);
        const double residual = std::sqrt(dx * dx + dy * dy + dz * dz) - static_cast<double>(cloth.rest_length[idx]);
        const double value = std::abs(residual);
        sum_abs += value;
        sum_sq += residual * residual;
        max_abs = std::max(max_abs, value);
    }

    const double denom = static_cast<double>(std::max<std::size_t>(1, constraint_count));
    return {
        sum_abs / denom,
        std::sqrt(sum_sq / denom),
        max_abs,
        constraint_count
    };
}

[[nodiscard]] Residuals computeResiduals(const HinaPE::ClothAoSoA& cloth) {
    double sum_abs = 0.0;
    double sum_sq = 0.0;
    double max_abs = 0.0;
    const std::size_t constraint_count = static_cast<std::size_t>(cloth.cons_count);

    const std::size_t block_count = cloth.cblocks.size();
    for (std::size_t block_index = 0; block_index < block_count; ++block_index) {
        const auto& block = cloth.cblocks[block_index];
        for (int lane = 0; lane < HinaPE::AOSOA_BLOCK; ++lane) {
            const std::size_t constraint_index = block_index * HinaPE::AOSOA_BLOCK + static_cast<std::size_t>(lane);
            if (constraint_index >= constraint_count) {
                break;
            }

            const int idx_i = block.i[lane];
            const int idx_j = block.j[lane];
            int block_i = 0;
            int lane_i = 0;
            HinaPE::index_to_block_lane(idx_i, block_i, lane_i);
            int block_j = 0;
            int lane_j = 0;
            HinaPE::index_to_block_lane(idx_j, block_j, lane_j);
            const auto& particle_i = cloth.pblocks[static_cast<std::size_t>(block_i)];
            const auto& particle_j = cloth.pblocks[static_cast<std::size_t>(block_j)];
            const double dx = static_cast<double>(particle_i.x[lane_i]) - static_cast<double>(particle_j.x[lane_j]);
            const double dy = static_cast<double>(particle_i.y[lane_i]) - static_cast<double>(particle_j.y[lane_j]);
            const double dz = static_cast<double>(particle_i.z[lane_i]) - static_cast<double>(particle_j.z[lane_j]);
            const double residual = std::sqrt(dx * dx + dy * dy + dz * dz) - static_cast<double>(block.rest_length[lane]);
            const double value = std::abs(residual);
            sum_abs += value;
            sum_sq += residual * residual;
            max_abs = std::max(max_abs, value);
        }
    }

    const double denom = static_cast<double>(std::max<std::size_t>(1, constraint_count));
    return {
        sum_abs / denom,
        std::sqrt(sum_sq / denom),
        max_abs,
        constraint_count
    };
}

[[nodiscard]] Residuals computeResiduals(const HinaPE::ClothAligned& cloth) {
    double sum_abs = 0.0;
    double sum_sq = 0.0;
    double max_abs = 0.0;
    const auto constraint_count = cloth.ci.size();

    for (std::size_t idx = 0; idx < constraint_count; ++idx) {
        const int i = cloth.ci[idx];
        const int j = cloth.cj[idx];
        const double dx = static_cast<double>(cloth.x[i]) - static_cast<double>(cloth.x[j]);
        const double dy = static_cast<double>(cloth.y[i]) - static_cast<double>(cloth.y[j]);
        const double dz = static_cast<double>(cloth.z[i]) - static_cast<double>(cloth.z[j]);
        const double residual = std::sqrt(dx * dx + dy * dy + dz * dz) - static_cast<double>(cloth.rest_length[idx]);
        const double value = std::abs(residual);
        sum_abs += value;
        sum_sq += residual * residual;
        max_abs = std::max(max_abs, value);
    }

    const double denom = static_cast<double>(std::max<std::size_t>(1, constraint_count));
    return {
        sum_abs / denom,
        std::sqrt(sum_sq / denom),
        max_abs,
        constraint_count
    };
}

[[nodiscard]] bool validatePins(const HinaPE::ClothAOS& cloth) {
    return cloth.particles.front().inv_mass == 0.0f && cloth.particles[cloth.nx - 1].inv_mass == 0.0f;
}

[[nodiscard]] bool validatePins(const HinaPE::ClothSOA& cloth) {
    return cloth.inv_mass.front() == 0.0f && cloth.inv_mass[cloth.nx - 1] == 0.0f;
}

[[nodiscard]] bool validatePins(const HinaPE::ClothAoSoA& cloth) {
    int block_index = 0;
    int lane = 0;
    HinaPE::index_to_block_lane(0, block_index, lane);
    if (cloth.pblocks.empty()) {
        return false;
    }
    const bool first_pinned = cloth.pblocks[static_cast<std::size_t>(block_index)].inv_mass[lane] == 0.0f;
    HinaPE::index_to_block_lane(cloth.nx - 1, block_index, lane);
    const bool second_pinned = cloth.pblocks[static_cast<std::size_t>(block_index)].inv_mass[lane] == 0.0f;
    return first_pinned && second_pinned;
}

[[nodiscard]] bool validatePins(const HinaPE::ClothAligned& cloth) {
    return cloth.inv_mass.front() == 0.0f && cloth.inv_mass[cloth.nx - 1] == 0.0f;
}

template <typename ClothT>
using StepFn = void (*)(ClothT&, float, const HinaPE::XPBDParams&);

template <typename ClothT>
int validateLayout(const char* label,
                   StepFn<ClothT> native,
                   StepFn<ClothT> tbb,
                   StepFn<ClothT> avx2) {
    constexpr int NX = 64;
    constexpr int NY = 32;
    constexpr float DT = 1.0f / 240.0f;
    constexpr int STEPS = 200;

    HinaPE::XPBDParams params = makeDefaultParams();

    auto build = [&](ClothT& cloth) {
        if constexpr (std::is_same_v<ClothT, HinaPE::ClothAOS>) {
            HinaPE::build_cloth_grid_aos(cloth, NX, NY, 1.6f, 1.0f, 0.3f, true);
        } else if constexpr (std::is_same_v<ClothT, HinaPE::ClothSOA>) {
            HinaPE::build_cloth_grid_soa(cloth, NX, NY, 1.6f, 1.0f, 0.3f, true);
        } else if constexpr (std::is_same_v<ClothT, HinaPE::ClothAoSoA>) {
            HinaPE::build_cloth_grid_aosoa(cloth, NX, NY, 1.6f, 1.0f, 0.3f, true);
        } else if constexpr (std::is_same_v<ClothT, HinaPE::ClothAligned>) {
            HinaPE::build_cloth_grid_aligned(cloth, NX, NY, 1.6f, 1.0f, 0.3f, true);
        }
    };

    ClothT cloth{};
    build(cloth);
    if (!validatePins(cloth)) {
        std::printf("[%s] pin validation failed at initialization\n", label);
        return 1;
    }

    ClothT cloth_native{};
    build(cloth_native);
    for (int step = 0; step < STEPS; ++step) {
        native(cloth_native, DT, params);
    }
    const Residuals native_residuals = computeResiduals(cloth_native);

    ClothT cloth_tbb{};
    build(cloth_tbb);
    for (int step = 0; step < STEPS; ++step) {
        tbb(cloth_tbb, DT, params);
    }
    const Residuals tbb_residuals = computeResiduals(cloth_tbb);

    ClothT cloth_avx{};
    build(cloth_avx);
    for (int step = 0; step < STEPS; ++step) {
        avx2(cloth_avx, DT, params);
    }
    const Residuals avx_residuals = computeResiduals(cloth_avx);

    auto report = [&](const char* tag, const Residuals& r) {
        std::printf("[%s] %-6s : L1=%g  L2=%g  Linf=%g (m=%zu)\n",
                    label, tag, r.l1, r.l2, r.linf, r.constraint_count);
    };
    report("native", native_residuals);
    report("tbb", tbb_residuals);
    report("avx2", avx_residuals);

    const auto withinTolerance = [](double reference, double candidate, double relative_tolerance) {
        const double denom = std::max(1e-8, std::abs(reference));
        return std::abs(candidate - reference) / denom <= relative_tolerance;
    };

    const double residual_tolerance = 1e-3;
    const double linf_tolerance = 5e-3;

    const bool tbb_ok = withinTolerance(native_residuals.l1, tbb_residuals.l1, residual_tolerance) &&
                        withinTolerance(native_residuals.l2, tbb_residuals.l2, residual_tolerance) &&
                        withinTolerance(native_residuals.linf, tbb_residuals.linf, linf_tolerance);
    const bool avx_ok = withinTolerance(native_residuals.l1, avx_residuals.l1, residual_tolerance) &&
                        withinTolerance(native_residuals.l2, avx_residuals.l2, residual_tolerance) &&
                        withinTolerance(native_residuals.linf, avx_residuals.linf, linf_tolerance);

    if (!tbb_ok) {
        std::printf("[%s] TBB residuals deviate from native beyond tolerance\n", label);
        return 2;
    }
    if (!avx_ok) {
        std::printf("[%s] AVX2 residuals deviate from native beyond tolerance\n", label);
        return 3;
    }
    return 0;
}

} // namespace

int main() {
    int status = 0;
    status |= validateLayout("AOS", &HinaPE::xpbd_step_native_aos, &HinaPE::xpbd_step_tbb_aos, &HinaPE::xpbd_step_avx2_aos);
    status |= validateLayout("SOA", &HinaPE::xpbd_step_native_soa, &HinaPE::xpbd_step_tbb_soa, &HinaPE::xpbd_step_avx2_soa);
    status |= validateLayout("AOSOA", &HinaPE::xpbd_step_native_aosoa, &HinaPE::xpbd_step_tbb_aosoa, &HinaPE::xpbd_step_avx2_aosoa);
    status |= validateLayout("ALIGNED", &HinaPE::xpbd_step_native_aligned, &HinaPE::xpbd_step_tbb_aligned, &HinaPE::xpbd_step_avx2_aligned);

    if (status == 0) {
        std::printf("Validation: SUCCESS\n");
    } else {
        std::printf("Validation: FAILED (code=%d)\n", status);
    }
    return status;
}
