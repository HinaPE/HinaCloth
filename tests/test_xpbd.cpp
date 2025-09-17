#include "xpbd.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

using namespace HinaPE;

namespace {

struct ClothBlueprint {
    std::size_t width{};
    std::size_t height{};
    float spacing{0.02f};
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> pz;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vz;
    std::vector<float> inv_mass;
    std::vector<std::uint8_t> pinned;
    std::vector<std::uint32_t> edge_i;
    std::vector<std::uint32_t> edge_j;
    std::vector<float> rest;
    std::vector<float> compliance;
    std::vector<float> lambda;
    std::vector<float> alpha;
    std::vector<std::uint8_t> color;
};

ClothBlueprint makeGridBlueprint(std::size_t width, std::size_t height, float spacing) {
    ClothBlueprint bp{};
    bp.width   = width;
    bp.height  = height;
    bp.spacing = spacing;

    const std::size_t particle_count = width * height;
    const std::size_t horizontal     = (width - 1) * height;
    const std::size_t vertical       = width * (height - 1);
    const std::size_t edge_count     = horizontal + vertical;

    bp.px.resize(particle_count);
    bp.py.resize(particle_count);
    bp.pz.resize(particle_count, 0.0f);
    bp.vx.resize(particle_count, 0.0f);
    bp.vy.resize(particle_count, 0.0f);
    bp.vz.resize(particle_count, 0.0f);
    bp.inv_mass.resize(particle_count, 1.0f);
    bp.pinned.resize(particle_count, 0);

    bp.edge_i.reserve(edge_count);
    bp.edge_j.reserve(edge_count);
    bp.rest.reserve(edge_count);
    bp.compliance.reserve(edge_count);
    bp.lambda.reserve(edge_count);
    bp.alpha.reserve(edge_count);
    bp.color.reserve(edge_count);

    const float diag_offset = spacing * 0.5f;

    for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
            const std::size_t idx = y * width + x;
            bp.px[idx]            = static_cast<float>(x) * spacing;
            bp.py[idx]            = static_cast<float>(height - 1 - y) * spacing + (y % 2 ? diag_offset : 0.0f);

            if (y == 0) {
                bp.pinned[idx]   = 1;
                bp.inv_mass[idx] = 0.0f;
            }
        }
    }

    auto append_edge = [&](std::uint32_t i, std::uint32_t j, std::uint8_t color) {
        bp.edge_i.push_back(i);
        bp.edge_j.push_back(j);
        bp.rest.push_back(spacing);
        bp.compliance.push_back(0.0f);
        bp.lambda.push_back(0.0f);
        bp.alpha.push_back(0.0f);
        bp.color.push_back(color);
    };

    for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t x = 0; x + 1 < width; ++x) {
            const std::uint32_t idx   = static_cast<std::uint32_t>(y * width + x);
            const std::uint32_t jdx   = static_cast<std::uint32_t>(y * width + x + 1);
            const std::uint8_t color  = static_cast<std::uint8_t>(x % 2);
            append_edge(idx, jdx, color);
        }
    }
    for (std::size_t y = 0; y + 1 < height; ++y) {
        for (std::size_t x = 0; x < width; ++x) {
            const std::uint32_t idx   = static_cast<std::uint32_t>(y * width + x);
            const std::uint32_t jdx   = static_cast<std::uint32_t>((y + 1) * width + x);
            const std::uint8_t color  = static_cast<std::uint8_t>(2 + (y % 2));
            append_edge(idx, jdx, color);
        }
    }

    return bp;
}

void loadClothState(ClothData& cloth, const ClothBlueprint& bp) {
    cloth.allocate_particles(bp.px.size());
    cloth.allocate_distance(bp.edge_i.size());
    cloth.allocate_triangles(0);
    cloth.allocate_bending(0);
    cloth.allocate_tri_elastic(0);

    auto particles = cloth.particles();
    std::copy(bp.px.begin(), bp.px.end(), particles.px.span().begin());
    std::copy(bp.py.begin(), bp.py.end(), particles.py.span().begin());
    std::copy(bp.pz.begin(), bp.pz.end(), particles.pz.span().begin());
    std::copy(bp.vx.begin(), bp.vx.end(), particles.vx.span().begin());
    std::copy(bp.vy.begin(), bp.vy.end(), particles.vy.span().begin());
    std::copy(bp.vz.begin(), bp.vz.end(), particles.vz.span().begin());
    std::copy(bp.inv_mass.begin(), bp.inv_mass.end(), particles.inv_mass.span().begin());
    std::copy(bp.pinned.begin(), bp.pinned.end(), particles.pinned.span().begin());

    if (bp.edge_i.empty()) {
        return;
    }

    auto dist = cloth.distance();
    std::copy(bp.edge_i.begin(), bp.edge_i.end(), dist.i.span().begin());
    std::copy(bp.edge_j.begin(), bp.edge_j.end(), dist.j.span().begin());
    std::copy(bp.rest.begin(), bp.rest.end(), dist.rest.span().begin());
    std::copy(bp.compliance.begin(), bp.compliance.end(), dist.compliance.span().begin());
    std::copy(bp.lambda.begin(), bp.lambda.end(), dist.lambda.span().begin());
    std::copy(bp.alpha.begin(), bp.alpha.end(), dist.alpha.span().begin());
    std::copy(bp.color.begin(), bp.color.end(), dist.color.span().begin());
}

struct Stats {
    double mean_ms{0.0};
    double median_ms{0.0};
    double stddev_ms{0.0};
    double min_ms{0.0};
    double max_ms{0.0};
};

Stats computeStats(const std::vector<double>& samples_ms) {
    Stats s{};
    if (samples_ms.empty()) {
        return s;
    }

    s.min_ms = *std::min_element(samples_ms.begin(), samples_ms.end());
    s.max_ms = *std::max_element(samples_ms.begin(), samples_ms.end());
    s.mean_ms = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) / static_cast<double>(samples_ms.size());

    std::vector<double> sorted = samples_ms;
    std::sort(sorted.begin(), sorted.end());
    if (sorted.size() % 2 == 0) {
        const std::size_t mid = sorted.size() / 2;
        s.median_ms            = 0.5 * (sorted[mid - 1] + sorted[mid]);
    } else {
        s.median_ms = sorted[sorted.size() / 2];
    }

    double variance = 0.0;
    for (const double value : samples_ms) {
        const double diff = value - s.mean_ms;
        variance += diff * diff;
    }
    variance /= static_cast<double>(samples_ms.size());
    s.stddev_ms = std::sqrt(variance);

    return s;
}

struct BenchmarkConfig {
    std::size_t width;
    std::size_t height;
    int warmup_runs;
    int trial_runs;
    int steps_per_trial;
};

struct ConstraintResiduals {
    double mean_abs{0.0};
    double rms{0.0};
    double max_abs{0.0};
    double relative_rms{0.0};
    double relative_max{0.0};
};

struct BenchmarkResult {
    Stats stats;
    double per_step_us{0.0};
    ConstraintResiduals residuals{};
};

using StepFunction = void (*)(ClothData&, const XPBDParams&);

ConstraintResiduals computeConstraintResiduals(const ClothData& cloth, const ClothBlueprint& bp) {
    ConstraintResiduals metrics{};
    const auto particles = cloth.particles();

    const std::size_t constraint_count = bp.edge_i.size();
    if (constraint_count == 0) {
        return metrics;
    }

    double sum_abs       = 0.0;
    double sum_sq        = 0.0;
    double sum_rel_sq    = 0.0;
    double max_abs       = 0.0;
    double max_rel       = 0.0;

    const auto px = particles.px.span();
    const auto py = particles.py.span();
    const auto pz = particles.pz.span();

    for (std::size_t c = 0; c < constraint_count; ++c) {
        const std::uint32_t i = bp.edge_i[c];
        const std::uint32_t j = bp.edge_j[c];
        const double rest     = static_cast<double>(bp.rest[c]);

        const double dx = static_cast<double>(px[i]) - static_cast<double>(px[j]);
        const double dy = static_cast<double>(py[i]) - static_cast<double>(py[j]);
        const double dz = static_cast<double>(pz[i]) - static_cast<double>(pz[j]);
        const double length    = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double residual  = length - rest;
        const double abs_value = std::abs(residual);

        sum_abs += abs_value;
        sum_sq += residual * residual;
        max_abs = std::max(max_abs, abs_value);

        if (rest > std::numeric_limits<double>::epsilon()) {
            const double rel = residual / rest;
            sum_rel_sq += rel * rel;
            max_rel = std::max(max_rel, std::abs(rel));
        }
    }

    const double inv_count = 1.0 / static_cast<double>(constraint_count);
    metrics.mean_abs       = sum_abs * inv_count;
    metrics.rms            = std::sqrt(sum_sq * inv_count);
    metrics.max_abs        = max_abs;
    metrics.relative_rms   = std::sqrt(sum_rel_sq * inv_count);
    metrics.relative_max   = max_rel;
    return metrics;
}

ConstraintResiduals evaluateResiduals(const ClothBlueprint& bp, const XPBDParams& params, const BenchmarkConfig& cfg, StepFunction fn) {
    ClothData cloth;
    for (int warmup = 0; warmup < cfg.warmup_runs; ++warmup) {
        loadClothState(cloth, bp);
        for (int step = 0; step < cfg.steps_per_trial; ++step) {
            fn(cloth, params);
        }
    }

    loadClothState(cloth, bp);
    for (int step = 0; step < cfg.steps_per_trial; ++step) {
        fn(cloth, params);
    }
    return computeConstraintResiduals(cloth, bp);
}

BenchmarkResult runBenchmark(const ClothBlueprint& bp, const XPBDParams& params, const BenchmarkConfig& cfg, StepFunction fn) {
    ClothData cloth;
    std::vector<double> samples_ms;
    samples_ms.reserve(cfg.trial_runs);

    for (int i = 0; i < cfg.warmup_runs; ++i) {
        loadClothState(cloth, bp);
        for (int step = 0; step < cfg.steps_per_trial; ++step) {
            fn(cloth, params);
        }
    }

    for (int trial = 0; trial < cfg.trial_runs; ++trial) {
        loadClothState(cloth, bp);
        const auto begin = std::chrono::steady_clock::now();
        for (int step = 0; step < cfg.steps_per_trial; ++step) {
            fn(cloth, params);
        }
        const auto end   = std::chrono::steady_clock::now();
        const auto delta = std::chrono::duration<double, std::milli>(end - begin);
        samples_ms.push_back(delta.count());
    }

    BenchmarkResult result{};
    result.stats      = computeStats(samples_ms);
    const double steps = static_cast<double>(cfg.steps_per_trial);
    result.per_step_us = (result.stats.mean_ms * 1000.0) / steps;
    result.residuals   = evaluateResiduals(bp, params, cfg, fn);
    return result;
}

struct Implementation {
    std::string name;
    StepFunction fn;
};

void printHeader(std::size_t width, std::size_t height, std::size_t particles, std::size_t edges) {
    std::cout << "\nScenario: " << width << " x " << height << " cloth (" << particles << " particles, " << edges << " constraints)" << '\n';
    std::cout << "--------------------------------------------------------------------------------" << '\n';
    std::cout << std::left << std::setw(14) << "Implementation"
              << std::right << std::setw(12) << "Mean [ms]"
              << std::setw(12) << "Median"
              << std::setw(12) << "StdDev"
              << std::setw(12) << "Min"
              << std::setw(12) << "Max"
              << std::setw(12) << "Step [us]" << '\n';
    std::cout << "--------------------------------------------------------------------------------" << '\n';
}

void printResidualHeader() {
    std::cout << std::left << std::setw(14) << "Implementation"
              << std::right << std::setw(14) << "Mean |C|"
              << std::setw(14) << "RMS |C|"
              << std::setw(14) << "Max |C|"
              << std::setw(14) << "RMS rel"
              << std::setw(14) << "Max rel" << '\n';
    std::cout << "--------------------------------------------------------------------------------" << '\n';
}

void printRow(const Implementation& impl, const BenchmarkResult& result) {
    const Stats& s = result.stats;
    std::cout << std::left << std::setw(14) << impl.name
              << std::right << std::setw(12) << std::fixed << std::setprecision(4) << s.mean_ms
              << std::setw(12) << s.median_ms
              << std::setw(12) << s.stddev_ms
              << std::setw(12) << s.min_ms
              << std::setw(12) << s.max_ms
              << std::setw(12) << result.per_step_us << '\n';
}

void printResidualRow(const Implementation& impl, const ConstraintResiduals& residuals) {
    std::cout << std::left << std::setw(14) << impl.name
              << std::right << std::setw(14) << std::scientific << std::setprecision(3) << residuals.mean_abs
              << std::setw(14) << residuals.rms
              << std::setw(14) << residuals.max_abs
              << std::setw(14) << residuals.relative_rms
              << std::setw(14) << residuals.relative_max << '\n';
}

void printSpeedup(const BenchmarkResult& baseline, const BenchmarkResult& challenger, const std::string& name) {
    const double speedup = baseline.stats.mean_ms / challenger.stats.mean_ms;
    const double percent = (1.0 - challenger.stats.mean_ms / baseline.stats.mean_ms) * 100.0;
    std::cout << "  -> " << name << " speedup vs native: " << std::fixed << std::setprecision(2) << speedup << "x";
    std::cout << " (" << percent << "% faster)" << '\n';
}

} // namespace

int main() {
    const std::array<BenchmarkConfig, 7> configs{{
        BenchmarkConfig{32, 32, 5, 20, 32},
        BenchmarkConfig{64, 64, 5, 16, 24},
        BenchmarkConfig{96, 96, 3, 12, 16},
        BenchmarkConfig{128, 128, 3, 10, 12},
        BenchmarkConfig{192, 192, 2, 8, 8},
        BenchmarkConfig{256, 256, 2, 6, 6},
        BenchmarkConfig{512, 512, 5, 6, 6},
    }};

    XPBDParams params{};
    params.time_step                = 1.0f / 120.0f;
    params.substeps                 = 8;
    params.solver_iterations        = 10;
    params.enable_distance_constraints = true;
    params.enable_bending_constraints  = false;
    params.velocity_damping         = 0.02f;

#if defined(__AVX2__)
    constexpr bool has_avx2 = true;
#else
    constexpr bool has_avx2 = false;
#endif

    const std::array<Implementation, 3> implementations{{
        Implementation{"native", xpbd_step_native},
        Implementation{"tbb", xpbd_step_tbb},
        Implementation{"avx2", xpbd_step_avx2},
    }};

    std::cout << "XPBD microbenchmark (native vs TBB vs AVX2)" << '\n';
    std::cout << "Build features: AVX2 " << (has_avx2 ? "enabled" : "fallback to native") << '\n';
    std::cout << "Warm-up runs are excluded from the statistics." << '\n';

    for (const auto& cfg : configs) {
        const ClothBlueprint blueprint = makeGridBlueprint(cfg.width, cfg.height, 0.025f);
        const std::size_t particles     = blueprint.px.size();
        const std::size_t edges         = blueprint.edge_i.size();

        printHeader(cfg.width, cfg.height, particles, edges);

        std::optional<BenchmarkResult> native_result;
        std::vector<std::optional<BenchmarkResult>> results;
        results.reserve(implementations.size());

        for (const auto& impl : implementations) {
            if (impl.name == "avx2" && !has_avx2) {
                std::cout << std::left << std::setw(14) << impl.name << std::right << std::setw(12) << "n/a"
                          << std::setw(12) << "n/a" << std::setw(12) << "n/a" << std::setw(12) << "n/a"
                          << std::setw(12) << "n/a" << std::setw(12) << "n/a" << '\n';
                results.emplace_back(std::nullopt);
                continue;
            }

            const BenchmarkResult result = runBenchmark(blueprint, params, cfg, impl.fn);
            printRow(impl, result);
            results.emplace_back(result);

            if (impl.name == "native") {
                native_result = result;
            } else if (native_result.has_value()) {
                printSpeedup(*native_result, result, impl.name);
            }
        }

        std::cout << "Constraint residuals (|C(x)| metrics)" << '\n';
        printResidualHeader();
        for (std::size_t idx = 0; idx < implementations.size(); ++idx) {
            const auto& impl = implementations[idx];
            if (!results[idx].has_value()) {
                std::cout << std::left << std::setw(14) << impl.name << std::right << std::setw(14) << "n/a"
                          << std::setw(14) << "n/a" << std::setw(14) << "n/a" << std::setw(14) << "n/a"
                          << std::setw(14) << "n/a" << '\n';
                continue;
            }
            printResidualRow(impl, results[idx]->residuals);
        }
    }

    return 0;
}
