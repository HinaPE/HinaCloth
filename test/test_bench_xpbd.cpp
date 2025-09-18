// Benchmark XPBD implementations across data layouts and backends
// Methodology (expanded, rigorous):
// - Identical initial conditions per scenario with explicit parameter sweeps
// - Warmup iterations excluded from timing; wall clock measured with steady clock
// - Multiple suites cover resolution scaling, stiffness extremes, dynamic loads, and boundary variations
// - All layouts {AOS, SOA, AOSOA, ALIGNED} exercised with {Native, TBB, AVX2} backends
// - Results recorded per run and summarized in a consolidated table for publication-grade reporting

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>

#include "xpbd_params.h"
#include "cloth_types.h"

#include "aos/cloth_data_aos.h"
#include "aos/solver_xpbd_aos.h"

#include "soa/cloth_data_soa.h"
#include "soa/solver_xpbd_soa.h"

#include "aosoa/cloth_data_aosoa.h"
#include "aosoa/solver_xpbd_aosoa.h"

#include "aligned/cloth_data_aligned.h"
#include "aligned/solver_xpbd_aligned.h"
#include "cloth_grid_utils.h"

using clock_type = std::chrono::steady_clock;

struct Scenario {
    std::string suite;
    std::string name;
    int nx{0};
    int ny{0};
    int warmup{0};
    int steps{0};
    float width{0.0f};
    float height{0.0f};
    float startY{0.0f};
    bool pinTopCorners{true};
    float dt{0.0f};
    float compStruct{0.0f};
    float compShear{0.0f};
    float compBend{0.0f};
    HinaPE::XPBDParams params{};
};

struct RunResult {
    std::string suite;
    std::string scenario;
    std::string layout;
    std::string backend;
    int nx{0};
    int ny{0};
    int warmup{0};
    int steps{0};
    int iterations{0};
    int substeps{0};
    float dt{0.0f};
    float compStruct{0.0f};
    float compShear{0.0f};
    float compBend{0.0f};
    bool pinTopCorners{true};
    std::size_t particles{0};
    std::size_t constraints{0};
    double constraintsPerParticle{0.0};
    double totalMs{0.0};
    double perStepMs{0.0};
};

static HinaPE::XPBDParams defaultParams() {
    HinaPE::XPBDParams p{};
    p.ax = 0.0f;
    p.ay = -9.81f;
    p.az = 0.0f;
    p.iterations = 10;
    p.substeps = 1;
    p.min_dt = 1.0f / 400.0f;
    p.max_dt = 1.0f / 30.0f;
    p.velocity_damping = 0.005f;
    p.warmstart = false;
    p.lambda_decay = 1.0f;
    p.compliance_scale_all = 1.0f;
    p.compliance_scale_structural = 1.0f;
    p.compliance_scale_shear = 1.0f;
    p.compliance_scale_bending = 1.0f;
    p.max_correction = 0.0f;
    p.write_debug_fields = 0;
    return p;
}

static Scenario makeBaseScenario(const std::string& suite,
                                 const std::string& name,
                                 int nx,
                                 int ny,
                                 int warmup,
                                 int steps,
                                 float dt) {
    Scenario scenario{};
    scenario.suite = suite;
    scenario.name = name;
    scenario.nx = nx;
    scenario.ny = ny;
    scenario.warmup = warmup;
    scenario.steps = steps;
    scenario.width = 1.6f;
    scenario.height = 1.0f;
    scenario.startY = 0.3f;
    scenario.pinTopCorners = true;
    scenario.dt = dt;
    scenario.compStruct = 1e-6f;
    scenario.compShear = 1e-5f;
    scenario.compBend = 1e-4f;
    scenario.params = defaultParams();
    return scenario;
}

static std::vector<Scenario> academicScenarios() {
    std::vector<Scenario> scenarios;

    Scenario s32 = makeBaseScenario("Baseline", "Resolution32x32", 32, 32, 60, 900, 1.0f / 240.0f);
    scenarios.push_back(s32);

    Scenario s64 = makeBaseScenario("Baseline", "Resolution64x64", 64, 64, 60, 900, 1.0f / 240.0f);
    scenarios.push_back(s64);

    Scenario s96 = makeBaseScenario("Baseline", "Resolution96x96", 96, 96, 50, 720, 1.0f / 240.0f);
    scenarios.push_back(s96);

    Scenario s128 = makeBaseScenario("Baseline", "Resolution128x128", 128, 128, 40, 600, 1.0f / 240.0f);
    scenarios.push_back(s128);

    Scenario rigid = makeBaseScenario("StiffnessSweep", "Rigid_64x64", 64, 64, 40, 720, 1.0f / 360.0f);
    rigid.params.iterations = 24;
    rigid.params.substeps = 2;
    rigid.params.lambda_decay = 0.95f;
    rigid.params.velocity_damping = 0.002f;
    rigid.compStruct = 1e-8f;
    rigid.compShear = 5e-8f;
    rigid.compBend = 2e-6f;
    scenarios.push_back(rigid);

    Scenario soft = makeBaseScenario("StiffnessSweep", "Soft_64x64", 64, 64, 40, 720, 1.0f / 240.0f);
    soft.params.iterations = 8;
    soft.params.substeps = 1;
    soft.params.velocity_damping = 0.015f;
    soft.compStruct = 5e-6f;
    soft.compShear = 2e-5f;
    soft.compBend = 6e-5f;
    scenarios.push_back(soft);

    Scenario multi = makeBaseScenario("StabilityStudies", "Substeps_80x80", 80, 80, 60, 600, 1.0f / 120.0f);
    multi.params.substeps = 4;
    multi.params.iterations = 16;
    multi.params.warmstart = true;
    multi.params.velocity_damping = 0.003f;
    scenarios.push_back(multi);

    Scenario largeDt = makeBaseScenario("StabilityStudies", "LargeDt_80x80", 80, 80, 50, 540, 1.0f / 60.0f);
    largeDt.params.substeps = 2;
    largeDt.params.iterations = 28;
    largeDt.params.velocity_damping = 0.012f;
    largeDt.params.lambda_decay = 0.90f;
    scenarios.push_back(largeDt);

    Scenario wind = makeBaseScenario("DynamicLoading", "WindImpulse_96x48", 96, 48, 60, 720, 1.0f / 120.0f);
    wind.width = 2.4f;
    wind.height = 1.2f;
    wind.startY = 0.5f;
    wind.params.ax = 3.5f;
    wind.params.az = 1.5f;
    wind.params.iterations = 18;
    wind.params.substeps = 3;
    wind.params.velocity_damping = 0.008f;
    wind.compStruct = 9e-7f;
    wind.compShear = 9e-6f;
    wind.compBend = 4e-5f;
    scenarios.push_back(wind);

    Scenario free = makeBaseScenario("BoundaryConditions", "FreeEdge_72x72", 72, 72, 50, 600, 1.0f / 200.0f);
    free.pinTopCorners = false;
    free.params.iterations = 14;
    free.params.velocity_damping = 0.004f;
    free.compStruct = 1.2e-6f;
    free.compShear = 8e-6f;
    free.compBend = 3e-5f;
    scenarios.push_back(free);

    Scenario wide = makeBaseScenario("ExtremeScale", "WideStrip_256x64", 256, 64, 30, 360, 1.0f / 240.0f);
    wide.width = 6.0f;
    wide.height = 1.5f;
    wide.startY = 1.2f;
    wide.params.iterations = 18;
    wide.params.substeps = 2;
    wide.params.velocity_damping = 0.006f;
    wide.compStruct = 8e-7f;
    wide.compShear = 4e-6f;
    wide.compBend = 2e-5f;
    scenarios.push_back(wide);

    Scenario mega = makeBaseScenario("ExtremeScale", "Mega_256x256", 256, 256, 20, 240, 1.0f / 180.0f);
    mega.width = 5.0f;
    mega.height = 5.0f;
    mega.startY = 2.0f;
    mega.params.iterations = 20;
    mega.params.substeps = 3;
    mega.params.velocity_damping = 0.010f;
    mega.params.lambda_decay = 0.92f;
    mega.compStruct = 5e-7f;
    mega.compShear = 2e-6f;
    mega.compBend = 1e-5f;
    scenarios.push_back(mega);

    return scenarios;
}

static void printScenarioHeader(const Scenario& scenario) {
    std::printf("\nSuite %-12s | Scenario %-18s | Grid=%dx%d | warmup=%d | steps=%d | dt=%.6f s | iter=%d | substeps=%d\n",
                scenario.suite.c_str(),
                scenario.name.c_str(),
                scenario.nx,
                scenario.ny,
                scenario.warmup,
                scenario.steps,
                scenario.dt,
                scenario.params.iterations,
                scenario.params.substeps);
    std::printf("   Extent: width=%.2f m | height=%.2f m | start_y=%.2f m | pin_top_corners=%s\n",
                scenario.width,
                scenario.height,
                scenario.startY,
                scenario.pinTopCorners ? "true" : "false");
    std::printf("   Compliance: structural=%.2e | shear=%.2e | bending=%.2e\n",
                scenario.compStruct,
                scenario.compShear,
                scenario.compBend);
}







template <typename ClothT,
          typename BuildFn,
          typename ParticleCountFn,
          typename ConstraintCountFn,
          typename StepNativeFn,
          typename StepTbbFn,
          typename StepAvxFn>
static void benchLayout(const Scenario& scenario,
                        const char* layoutName,
                        BuildFn&& build,
                        ParticleCountFn&& particleCount,
                        ConstraintCountFn&& constraintCount,
                        StepNativeFn&& stepNative,
                        StepTbbFn&& stepTbb,
                        StepAvxFn&& stepAvx2,
                        std::vector<RunResult>& results) {
    ClothT cloth{};
    auto rebuild = [&]() {
        build(cloth, scenario);
    };

    rebuild();
    const std::size_t particles = particleCount(cloth);
    const std::size_t constraints = constraintCount(cloth);
    const double constraintRatio = particles > 0
        ? static_cast<double>(constraints) / static_cast<double>(particles)
        : 0.0;

    // FIX: replaced raw newline in string literal with escaped \n
    std::printf("   Layout %-6s : %7zu particles | %9zu constraints | %.3f cons/pt\n",
                layoutName,
                particles,
                constraints,
                constraintRatio);

    auto runBackend = [&](const char* backend, auto stepFn) {
        rebuild();
        for (int i = 0; i < scenario.warmup; ++i) {
            stepFn(cloth, scenario.dt, scenario.params);
        }
        const auto t0 = clock_type::now();
        for (int i = 0; i < scenario.steps; ++i) {
            stepFn(cloth, scenario.dt, scenario.params);
        }
        const auto t1 = clock_type::now();
        const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double perStep = scenario.steps > 0
            ? totalMs / static_cast<double>(scenario.steps)
            : 0.0;

        // FIX: replaced raw newline in string literal with escaped \n
        std::printf("      %-7s -> total %10.3f ms | %8.3f ms/step\n",
                    backend,
                    totalMs,
                    perStep);

        results.push_back(RunResult{
            scenario.suite,
            scenario.name,
            layoutName,
            backend,
            scenario.nx,
            scenario.ny,
            scenario.warmup,
            scenario.steps,
            scenario.params.iterations,
            scenario.params.substeps,
            scenario.dt,
            scenario.compStruct,
            scenario.compShear,
            scenario.compBend,
            scenario.pinTopCorners,
            particles,
            constraints,
            constraintRatio,
            totalMs,
            perStep
        });
    };

    runBackend("native", stepNative);
    runBackend("tbb", stepTbb);
    runBackend("avx2", stepAvx2);
}

static void printReportTable(const std::vector<RunResult>& results) {
    if (results.empty()) {
        std::printf("No benchmark results collected.\n");
        return;
    }
    std::printf("==== Detailed Stress Test Report ====\n");
    std::printf("%-12s %-20s %-7s %-8s %-11s %7s %7s %7s %9s %10s %11s %12s %10s %11s %11s\n",
                "Suite",
                "Scenario",
                "Layout",
                "Backend",
                "Grid",
                "Warm",
                "Steps",
                "Iter",
                "Substeps",
                "dt (ms)",
                "Particles",
                "Constraints",
                "Cons/Pt",
                "Total (ms)",
                "Avg (ms)");
    for (const auto& r : results) {
        char grid[24];
        std::snprintf(grid, sizeof(grid), "%dx%d", r.nx, r.ny);
        const double dtMs = static_cast<double>(r.dt) * 1000.0;
        std::printf("%-12s %-20s %-7s %-8s %-11s %7d %7d %7d %9.3f %10.3f %11zu %12zu %10.3f %11.3f %11.3f\n",
                    r.suite.c_str(),
                    r.scenario.c_str(),
                    r.layout.c_str(),
                    r.backend.c_str(),
                    grid,
                    r.warmup,
                    r.steps,
                    r.iterations,
                    r.substeps,
                    dtMs,
                    r.particles,
                    r.constraints,
                    r.constraintsPerParticle,
                    r.totalMs,
                    r.perStepMs);
    }
}







int main(int argc, char** argv) {
    std::vector<Scenario> scenarios;
    scenarios.reserve(16);

    if (argc >= 5) {
        const int nx = std::max(1, std::atoi(argv[1]));
        const int ny = std::max(1, std::atoi(argv[2]));
        const int warm = std::max(0, std::atoi(argv[3]));
        const int steps = std::max(1, std::atoi(argv[4]));
        Scenario cli = makeBaseScenario("CLI", "UserDefined", nx, ny, warm, steps, 1.0f / 240.0f);
        if (argc >= 6) {
            cli.params.iterations = std::max(1, std::atoi(argv[5]));
        }
        if (argc >= 7) {
            cli.params.substeps = std::max(1, std::atoi(argv[6]));
        }
        if (argc >= 8) {
            const float parsedDt = static_cast<float>(std::atof(argv[7]));
            cli.dt = parsedDt > 0.0f ? parsedDt : cli.dt;
        }
        scenarios.push_back(cli);
    } else {
        scenarios = academicScenarios();
    }

    std::vector<RunResult> results;
    results.reserve(scenarios.size() * 12);

    std::printf("XPBD Stress Benchmark (academic suite)\n\n");
    for (const auto& scenario : scenarios) {
        printScenarioHeader(scenario);

        benchLayout<HinaPE::ClothAOS>(
            scenario,
            "AOS",
            [](HinaPE::ClothAOS& cloth, const Scenario& sc) {
                HinaPE::build_cloth_grid_aos(cloth,
                                              sc.nx,
                                              sc.ny,
                                              sc.width,
                                              sc.height,
                                              sc.startY,
                                              sc.pinTopCorners,
                                              sc.compStruct,
                                              sc.compShear,
                                              sc.compBend);
            },
            [](const HinaPE::ClothAOS& cloth) { return cloth.particles.size(); },
            [](const HinaPE::ClothAOS& cloth) { return cloth.constraints.size(); },
            &HinaPE::xpbd_step_native_aos,
            &HinaPE::xpbd_step_tbb_aos,
            &HinaPE::xpbd_step_avx2_aos,
            results);

        benchLayout<HinaPE::ClothSOA>(
            scenario,
            "SOA",
            [](HinaPE::ClothSOA& cloth, const Scenario& sc) {
                HinaPE::build_cloth_grid_soa(cloth,
                                             sc.nx,
                                             sc.ny,
                                             sc.width,
                                             sc.height,
                                             sc.startY,
                                             sc.pinTopCorners,
                                             sc.compStruct,
                                             sc.compShear,
                                             sc.compBend);
            },
            [](const HinaPE::ClothSOA& cloth) { return cloth.x.size(); },
            [](const HinaPE::ClothSOA& cloth) { return cloth.ci.size(); },
            &HinaPE::xpbd_step_native_soa,
            &HinaPE::xpbd_step_tbb_soa,
            &HinaPE::xpbd_step_avx2_soa,
            results);

        benchLayout<HinaPE::ClothAoSoA>(
            scenario,
            "AOSOA",
            [](HinaPE::ClothAoSoA& cloth, const Scenario& sc) {
                HinaPE::build_cloth_grid_aosoa(cloth,
                                               sc.nx,
                                               sc.ny,
                                               sc.width,
                                               sc.height,
                                               sc.startY,
                                               sc.pinTopCorners,
                                               sc.compStruct,
                                               sc.compShear,
                                               sc.compBend);
            },
            [](const HinaPE::ClothAoSoA& cloth) { return static_cast<std::size_t>(cloth.count); },
            [](const HinaPE::ClothAoSoA& cloth) { return static_cast<std::size_t>(cloth.cons_count); },
            &HinaPE::xpbd_step_native_aosoa,
            &HinaPE::xpbd_step_tbb_aosoa,
            &HinaPE::xpbd_step_avx2_aosoa,
            results);

        benchLayout<HinaPE::ClothAligned>(
            scenario,
            "ALGN",
            [](HinaPE::ClothAligned& cloth, const Scenario& sc) {
                HinaPE::build_cloth_grid_aligned(cloth,
                                                 sc.nx,
                                                 sc.ny,
                                                 sc.width,
                                                 sc.height,
                                                 sc.startY,
                                                 sc.pinTopCorners,
                                                 sc.compStruct,
                                                 sc.compShear,
                                                 sc.compBend);
            },
            [](const HinaPE::ClothAligned& cloth) { return cloth.x.size(); },
            [](const HinaPE::ClothAligned& cloth) { return cloth.ci.size(); },
            &HinaPE::xpbd_step_native_aligned,
            &HinaPE::xpbd_step_tbb_aligned,
            &HinaPE::xpbd_step_avx2_aligned,
            results);
    }

    printReportTable(results);
    return 0;
}
