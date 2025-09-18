// Benchmark XPBD implementations across data layouts and backends
// Methodology (concise, rigorous):
// - Identical initial conditions across runs (grid, params, pinning)
// - Warmup steps excluded from timing to amortize one-time effects
// - Fixed number of timed steps; measure wall-clock with steady clock
// - Report: total ms, avg ms/step, particles, constraints
// - Runs each combination: {AoS, SoA, AoSoA, AlignedSoA} x {Native, TBB, AVX2}
// - AVX2 path auto-falls-back to native if not compiled with AVX2

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

using clock_type = std::chrono::steady_clock;

struct bench_case { int nx, ny; int warmup; int steps; const char* name; };

static HinaPE::XPBDParams default_params() {
    HinaPE::XPBDParams p{};
    p.ax = 0.f; p.ay = -9.81f; p.az = 0.f;
    p.iterations = 10; p.substeps = 1;
    p.min_dt = 1.f/400.f; p.max_dt = 1.f/30.f;
    p.velocity_damping = 0.005f;
    p.warmstart = false; p.lambda_decay = 1.f;
    p.compliance_scale_all = 1.f;
    p.compliance_scale_structural = 1.f;
    p.compliance_scale_shear = 1.f;
    p.compliance_scale_bending = 1.f;
    p.max_correction = 0.0f;
    p.write_debug_fields = 0;
    return p;
}

static void bench_aos(int nx, int ny, int warmup, int steps) {
    HinaPE::ClothAOS cloth; HinaPE::build_cloth_grid_aos(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
    const size_t particles = cloth.particles.size();
    const size_t constraints = cloth.constraints.size();
    HinaPE::XPBDParams params = default_params();
    const float dt = 1.f/240.f;

    auto run = [&](const char* tag, void(*fn)(HinaPE::ClothAOS&, float, const HinaPE::XPBDParams&)){
        // rebuild to reset initial conditions
        HinaPE::build_cloth_grid_aos(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
        for (int i=0;i<warmup;++i) fn(cloth, dt, params);
        auto t0 = clock_type::now();
        for (int i=0;i<steps;++i) fn(cloth, dt, params);
        auto t1 = clock_type::now();
        double ms = std::chrono::duration<double, std::milli>(t1-t0).count();
        std::printf("AOS | %-7s | %4dx%-4d | %7zu pts | %7zu cons | %8.3f ms total | %8.3f ms/step\n",
                    tag, nx, ny, particles, constraints, ms, ms/steps);
    };

    run("native", &HinaPE::xpbd_step_native_aos);
    run("tbb",    &HinaPE::xpbd_step_tbb_aos);
    run("avx2",   &HinaPE::xpbd_step_avx2_aos);
}

static void bench_soa(int nx, int ny, int warmup, int steps) {
    HinaPE::ClothSOA cloth; HinaPE::build_cloth_grid_soa(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
    const size_t particles = cloth.x.size();
    const size_t constraints = cloth.ci.size();
    HinaPE::XPBDParams params = default_params();
    const float dt = 1.f/240.f;
    auto run = [&](const char* tag, void(*fn)(HinaPE::ClothSOA&, float, const HinaPE::XPBDParams&)){
        HinaPE::build_cloth_grid_soa(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
        for (int i=0;i<warmup;++i) fn(cloth, dt, params);
        auto t0 = clock_type::now();
        for (int i=0;i<steps;++i) fn(cloth, dt, params);
        auto t1 = clock_type::now();
        double ms = std::chrono::duration<double, std::milli>(t1-t0).count();
        std::printf("SOA | %-7s | %4dx%-4d | %7zu pts | %7zu cons | %8.3f ms total | %8.3f ms/step\n",
                    tag, nx, ny, particles, constraints, ms, ms/steps);
    };
    run("native", &HinaPE::xpbd_step_native_soa);
    run("tbb",    &HinaPE::xpbd_step_tbb_soa);
    run("avx2",   &HinaPE::xpbd_step_avx2_soa);
}

static void bench_aosoa(int nx, int ny, int warmup, int steps) {
    HinaPE::ClothAoSoA cloth; HinaPE::build_cloth_grid_aosoa(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
    const size_t particles = cloth.count;
    const size_t constraints = cloth.cons_count;
    HinaPE::XPBDParams params = default_params();
    const float dt = 1.f/240.f;
    auto run = [&](const char* tag, void(*fn)(HinaPE::ClothAoSoA&, float, const HinaPE::XPBDParams&)){
        HinaPE::build_cloth_grid_aosoa(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
        for (int i=0;i<warmup;++i) fn(cloth, dt, params);
        auto t0 = clock_type::now();
        for (int i=0;i<steps;++i) fn(cloth, dt, params);
        auto t1 = clock_type::now();
        double ms = std::chrono::duration<double, std::milli>(t1-t0).count();
        std::printf("AOSOA | %-5s | %4dx%-4d | %7zu pts | %7zu cons | %8.3f ms total | %8.3f ms/step\n",
                    tag, nx, ny, particles, constraints, ms, ms/steps);
    };
    run("native", &HinaPE::xpbd_step_native_aosoa);
    run("tbb",    &HinaPE::xpbd_step_tbb_aosoa);
    run("avx2",   &HinaPE::xpbd_step_avx2_aosoa);
}

static void bench_aligned(int nx, int ny, int warmup, int steps) {
    HinaPE::ClothAligned cloth; HinaPE::build_cloth_grid_aligned(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
    const size_t particles = cloth.x.size;
    const size_t constraints = cloth.ci.size;
    HinaPE::XPBDParams params = default_params();
    const float dt = 1.f/240.f;
    auto run = [&](const char* tag, void(*fn)(HinaPE::ClothAligned&, float, const HinaPE::XPBDParams&)){
        HinaPE::build_cloth_grid_aligned(cloth, nx, ny, 1.6f, 1.0f, 0.3f, true);
        for (int i=0;i<warmup;++i) fn(cloth, dt, params);
        auto t0 = clock_type::now();
        for (int i=0;i<steps;++i) fn(cloth, dt, params);
        auto t1 = clock_type::now();
        double ms = std::chrono::duration<double, std::milli>(t1-t0).count();
        std::printf("ALGN | %-5s | %4dx%-4d | %7zu pts | %7zu cons | %8.3f ms total | %8.3f ms/step\n",
                    tag, nx, ny, particles, constraints, ms, ms/steps);
    };
    run("native", &HinaPE::xpbd_step_native_aligned);
    run("tbb",    &HinaPE::xpbd_step_tbb_aligned);
    run("avx2",   &HinaPE::xpbd_step_avx2_aligned);
}

int main(int argc, char** argv) {
    std::vector<bench_case> cases{
        {32, 32, 20, 200, "S"},
        {64, 32, 20, 200, "M"},
        {64, 64, 20, 200, "L"}
    };
    if (argc >= 5) {
        cases.clear();
        int nx = std::atoi(argv[1]);
        int ny = std::atoi(argv[2]);
        int warm = std::atoi(argv[3]);
        int steps = std::atoi(argv[4]);
        cases.push_back({nx, ny, warm, steps, "CLI"});
    }

    std::printf("XPBD Benchmark (all layouts x backends)\n");
    for (const auto& c : cases) {
        std::printf("\nCase %s: %dx%d, warmup=%d, steps=%d\n", c.name, c.nx, c.ny, c.warmup, c.steps);
        bench_aos(c.nx, c.ny, c.warmup, c.steps);
        bench_soa(c.nx, c.ny, c.warmup, c.steps);
        bench_aosoa(c.nx, c.ny, c.warmup, c.steps);
        bench_aligned(c.nx, c.ny, c.warmup, c.steps);
    }
    return 0;
}

