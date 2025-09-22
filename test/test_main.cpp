#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <limits>
#include <algorithm>
#include <array>

#include "core/model/model.h"
#include "core/data/data.h"
#include "runtime/step.h"
#include "backend/kernel/constraints/distance.h"
#include "backend/storage/soa.h"

using sim::Model;
using sim::Data;
using sim::eng::Status;
using sim::eng::TelemetryFrame;

struct TestCase { std::string name; std::function<void(void)> fn; };
static std::vector<TestCase> g_tests;
static int g_failures = 0;

#define ADD_TEST(fn) g_tests.push_back(TestCase{#fn, [](){ fn(); }});
#define REQUIRE(cond, msg) do { if (!(cond)) { std::fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); g_failures++; return; } } while(0)
#define REQUIRE_NEAR(a,b,eps,msg) do { if (std::fabs((a)-(b)) > (eps)) { std::fprintf(stderr, "[FAIL] %s:%d: %s (got=%g, want=%g, eps=%g)\n", __FILE__, __LINE__, msg, (double)(a), (double)(b), (double)(eps)); g_failures++; return; } } while(0)
#define REQUIRE_FINITE(x,msg) do { if (!std::isfinite((double)(x))) { std::fprintf(stderr, "[FAIL] %s:%d: %s (got=%g)\n", __FILE__, __LINE__, msg, (double)(x)); g_failures++; return; } } while(0)

static float len3(float x,float y,float z){ return std::sqrt(x*x+y*y+z*z); }

static void init_data(Data& d, size_t n) {
    d.x.assign(n, 0.0f); d.y.assign(n, 0.0f); d.z.assign(n, 0.0f);
    d.vx.assign(n, 0.0f); d.vy.assign(n, 0.0f); d.vz.assign(n, 0.0f);
    d.px.assign(n, 0.0f); d.py.assign(n, 0.0f); d.pz.assign(n, 0.0f);
    d.inv_mass.assign(n, 1.0f);
    d.lambda_edge.clear();
    d.distance_compliance = 0.0f;
    d.distance_compliance_edge.clear();
    d.distance_alpha_edge.clear();
    d.gx = 0.0f; d.gy = 0.0f; d.gz = 0.0f;
    d.solve_substeps = 1;
    d.solve_iterations = 20;
    d.solve_damping = 0.0f;
    d.exec_use_tbb = false; d.exec_threads = -1; d.exec_use_avx2 = false;
    d.exec_layout_blocked = false; d.layout_block_size = 8u; d.pos_aosoa.clear();
    d.op_enable_attachment = false; d.op_enable_bending = false;
    d.attach_w.assign(n, 0.0f); d.attach_tx.assign(n, 0.0f); d.attach_ty.assign(n, 0.0f); d.attach_tz.assign(n, 0.0f);
}

static void init_model_for_edges(Model& m, uint32_t node_count, const std::vector<uint32_t>& edges, const std::vector<float>& rest) {
    m.node_count = node_count;
    m.edges = edges;
    m.rest = rest;
    m.island_count = 1;
    m.island_offsets.clear();
    m.node_remap.clear();
    m.layout_block_size = 8u;
    m.bend_pairs.clear();
    m.bend_rest_angle.clear();
}

static void test_distance_basic_pbd() {
    Model m{}; Data d{};
    // 2 nodes, one edge with rest=1 from (0,0,0) to (2,0,0)
    init_model_for_edges(m, 2, {0,1}, {1.0f});
    init_data(d, 2);
    d.x[0]=0; d.y[0]=0; d.z[0]=0;
    d.x[1]=2; d.y[1]=0; d.z[1]=0;
    d.solve_iterations = 30;
    d.distance_compliance = 0.0f; // PBD
    d.lambda_edge.assign(1, 0.0f);
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed");
    float dx = d.x[1]-d.x[0], dy = d.y[1]-d.y[0], dz = d.z[1]-d.z[0];
    float L = len3(dx,dy,dz);
    REQUIRE_NEAR(L, 1.0f, 1e-4f, "Distance constraint should converge to rest length (PBD)");
    REQUIRE_FINITE(t.residual_avg, "Residual must be finite");
}

static void test_distance_basic_xpbd() {
    Model m{}; Data d{};
    init_model_for_edges(m, 2, {0,1}, {1.0f});
    init_data(d, 2);
    d.x[0]=0; d.x[1]=2; // along x
    d.solve_iterations = 10; // iterations don't affect steady-state in 2-body case
    d.distance_compliance = 1e-6f;
    d.lambda_edge.assign(1, 0.0f);
    const float dt = 0.016f;
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, dt, nullptr, &t) == Status::Ok, "runtime_step failed");
    float L = len3(d.x[1]-d.x[0], d.y[1]-d.y[0], d.z[1]-d.z[0]);
    // XPBD steady-state for two nodes (wi=wj=1): C* = alpha/(wi+wj+alpha)
    float alpha = std::max(0.0f, d.distance_compliance) / (dt*dt);
    float expected = 1.0f + alpha / (2.0f + alpha);
    REQUIRE_NEAR(L, expected, 5e-5f, "XPBD should converge to analytical steady-state length with compliance");
}

static void test_distance_pinned_endpoint() {
    Model m{}; Data d{};
    init_model_for_edges(m, 2, {0,1}, {1.0f});
    init_data(d, 2);
    d.x[0]=0; d.x[1]=2;
    d.inv_mass[0] = 0.0f; // pin first vertex
    d.lambda_edge.assign(1, 0.0f);
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed");
    REQUIRE_NEAR(d.x[0], 0.0f, 1e-6f, "Pinned vertex should not move");
    float L = len3(d.x[1]-d.x[0], d.y[1]-d.y[0], d.z[1]-d.z[0]);
    REQUIRE_NEAR(L, 1.0f, 1e-4f, "Distance constraint with one pinned endpoint should converge");
}

static void test_distance_per_edge_compliance() {
    // Chain of 3 nodes: edges (0-1) stiff, (1-2) compliant; initial stretched
    Model m{}; Data d{};
    init_model_for_edges(m, 3, {0,1, 1,2}, {1.0f, 1.0f});
    init_data(d, 3);
    d.x[0]=0; d.x[1]=2; d.x[2]=4; // each segment length 2, rest 1
    d.distance_compliance = 0.0f; // global
    d.distance_compliance_edge.assign(2, 0.0f);
    d.distance_compliance_edge[0] = 1e-7f; // stiff
    d.distance_compliance_edge[1] = 1e-3f; // more compliant
    d.lambda_edge.assign(2, 0.0f);
    d.solve_iterations = 25;
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed");
    float L0 = std::fabs((d.x[1]-d.x[0]));
    float L1 = std::fabs((d.x[2]-d.x[1]));
    REQUIRE(L0 <= L1 + 1e-3f, "Stiffer edge should end closer to rest than compliant edge");
}

static void test_distance_blocked_layout() {
    // Same as basic, but enable AoSoA blocked layout path
    Model m{}; Data d{};
    init_model_for_edges(m, 2, {0,1}, {1.0f});
    init_data(d, 2);
    d.x[0]=0; d.x[1]=2;
    d.lambda_edge.assign(1, 0.0f);
    d.exec_layout_blocked = true;
    d.layout_block_size = 8u;
    d.pos_aosoa.clear();
    d.solve_iterations = 30;
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed (blocked)");
    float L = std::fabs(d.x[1]-d.x[0]);
    REQUIRE_NEAR(L, 1.0f, 1e-4f, "Blocked layout solver should converge like SoA");
}

static void test_attachment_operator() {
    // Single node attached to target with w=1 => should snap to target in presolve
    Model m{}; Data d{};
    init_model_for_edges(m, 1, {}, {});
    init_data(d, 1);
    d.x[0] = 0.0f;
    d.attach_w[0] = 1.0f;
    d.attach_tx[0] = 1.0f; d.attach_ty[0] = 2.0f; d.attach_tz[0] = 3.0f;
    d.op_enable_attachment = true;
    TelemetryFrame t{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed (attachment)");
    REQUIRE_NEAR(d.x[0], 1.0f, 1e-6f, "Attachment x should match target when w=1");
    REQUIRE_NEAR(d.y[0], 2.0f, 1e-6f, "Attachment y should match target when w=1");
    REQUIRE_NEAR(d.z[0], 3.0f, 1e-6f, "Attachment z should match target when w=1");
}

static float dihedral_angle_simple(const std::array<float,12>& p) {
    // points: i0(0..2), i1(3..5), i2(6..8), i3(9..11)
    auto dot=[&](float ax,float ay,float az,float bx,float by,float bz){return ax*bx+ay*by+az*bz;};
    auto cross=[&](float ax,float ay,float az,float bx,float by,float bz, float& rx,float& ry,float& rz){rx=ay*bz-az*by; ry=az*bx-ax*bz; rz=ax*by-ay*bx;};
    auto len=[&](float ax,float ay,float az){return std::sqrt(ax*ax+ay*ay+az*az);} ;
    float e0x=p[3]-p[0], e0y=p[4]-p[1], e0z=p[5]-p[2];
    float e1x=p[6]-p[0], e1y=p[7]-p[1], e1z=p[8]-p[2];
    float e2x=p[9]-p[0], e2y=p[10]-p[1], e2z=p[11]-p[2];
    float n1x,n1y,n1z,n2x,n2y,n2z; cross(e0x,e0y,e0z,e1x,e1y,e1z,n1x,n1y,n1z); cross(e0x,e0y,e0z,e2x,e2y,e2z,n2x,n2y,n2z);
    float n1l=len(n1x,n1y,n1z), n2l=len(n2x,n2y,n2z);
    if (n1l<=1e-8f || n2l<=1e-8f) return 0.0f;
    float c = dot(n1x,n1y,n1z,n2x,n2y,n2z)/(n1l*n2l);
    c = std::clamp(c,-1.0f,1.0f);
    return std::acos(c);
}

static void test_bending_convergence() {
    // Four points forming two triangles sharing edge (i0-i1)
    // Initial: angle ~ 60deg, target: 0deg => expect angle reduces toward 0
    Model m{}; Data d{};
    init_model_for_edges(m, 4, {}, {});
    m.bend_pairs = {0,1,2,3};
    m.bend_rest_angle = {0.0f};
    init_data(d, 4);
    // positions
    d.x[0]=0; d.y[0]=0; d.z[0]=0;   // i0
    d.x[1]=1; d.y[1]=0; d.z[1]=0;   // i1
    d.x[2]=0; d.y[2]=1; d.z[2]=1;   // i2
    d.x[3]=1; d.y[3]=1; d.z[3]=-1;  // i3
    d.op_enable_bending = true;
    d.solve_iterations = 40;
    TelemetryFrame t{};
    float before = dihedral_angle_simple(std::array<float,12>{d.x[0],d.y[0],d.z[0], d.x[1],d.y[1],d.z[1], d.x[2],d.y[2],d.z[2], d.x[3],d.y[3],d.z[3]});
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t) == Status::Ok, "runtime_step failed (bending)");
    float after = dihedral_angle_simple(std::array<float,12>{d.x[0],d.y[0],d.z[0], d.x[1],d.y[1],d.z[1], d.x[2],d.y[2],d.z[2], d.x[3],d.y[3],d.z[3]});
    REQUIRE(after <= before + 1e-4f, "Bending pass should not increase angle away from target");
}

static void test_small_grid_stability() {
    // 3x3 grid, structural edges only, no gravity, check residual decreases over several frames
    const int W=3, H=3; const float h=1.0f; const int N=W*H;
    Model m{}; Data d{};
    std::vector<uint32_t> edges; std::vector<float> rest;
    auto idx=[&](int x,int y){return (uint32_t)(y*W+x);};
    for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
        if (x+1<W){ edges.push_back(idx(x,y)); edges.push_back(idx(x+1,y)); rest.push_back(h);}
        if (y+1<H){ edges.push_back(idx(x,y)); edges.push_back(idx(x,y+1)); rest.push_back(h);}
    }
    init_model_for_edges(m, (uint32_t)N, edges, rest);
    init_data(d, N);
    for (int y=0;y<H;y++) for (int x=0;x<W;x++) { int i=idx(x,y); d.x[i]=x*h; d.y[i]=y*h; d.z[i]=0.0f; }
    // perturb positions
    for (int i=0;i<N;i++){ d.x[i] += (i%2?0.1f:-0.05f); d.y[i] += (i%3?0.05f:-0.02f); }
    d.solve_iterations = 25; d.solve_substeps = 2; d.distance_compliance = 1e-7f;
    d.lambda_edge.assign(rest.size(), 0.0f);
    TelemetryFrame t0{}, t1{}, t2{};
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t0) == Status::Ok, "step0 failed");
    double r0 = t0.residual_avg; REQUIRE_FINITE(r0, "r0 finite");
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t1) == Status::Ok, "step1 failed");
    double r1 = t1.residual_avg; REQUIRE_FINITE(r1, "r1 finite");
    REQUIRE(sim::eng::runtime_step(m, d, 0.016f, nullptr, &t2) == Status::Ok, "step2 failed");
    double r2 = t2.residual_avg; REQUIRE_FINITE(r2, "r2 finite");
    REQUIRE(r2 <= r0 + 1e-6, "Residual should not explode over a few steps");
}

static void test_kernel_distance_direct() {
    using namespace sim;
    float px[2] = {0.0f, 2.0f};
    float py[2] = {0.0f, 0.0f};
    float pz[2] = {0.0f, 0.0f};
    SoAView3 pos{}; storage_bind_soa(pos, px, py, pz, 2);
    uint32_t edges[2] = {0u,1u};
    float rest = 1.0f;
    float inv_mass[2] = {1.0f,1.0f};
    float lambda[1] = {0.0f};
    float alpha_edge = 0.0f; // PBD
    kernel_distance_project(edges, /*m*/1, pos, &rest, inv_mass, lambda, &alpha_edge, /*iterations*/20, /*alpha*/0.0f, /*dt*/0.016f);
    float L = std::fabs(px[1]-px[0]);
    REQUIRE_NEAR(L, 1.0f, 1e-4f, "Direct kernel projection should converge to rest length");
}

int main() {
    // Register tests
    ADD_TEST(test_distance_basic_pbd);
    ADD_TEST(test_distance_basic_xpbd);
    ADD_TEST(test_distance_pinned_endpoint);
    ADD_TEST(test_distance_per_edge_compliance);
    ADD_TEST(test_distance_blocked_layout);
    ADD_TEST(test_attachment_operator);
    ADD_TEST(test_bending_convergence);
    ADD_TEST(test_small_grid_stability);
    ADD_TEST(test_kernel_distance_direct);

    int passed = 0;
    for (auto& t : g_tests) {
        int before = g_failures;
        t.fn();
        if (g_failures == before) {
            std::printf("[PASS] %s\n", t.name.c_str());
            passed++;
        } else {
            std::printf("[FAIL] %s\n", t.name.c_str());
        }
    }
    std::printf("\nTests passed: %d/%zu\n", passed, g_tests.size());
    return g_failures == 0 ? 0 : 1;
}
