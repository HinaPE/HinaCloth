#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <functional>
#include <memory>

#include "api/sim.h"
#include "api/build.h"
#include "api/status.h"
#include "api/telemetry.h"

struct Vec3 { float x,y,z; };

struct TestContext {
    // Keep payload buffers alive and stable until flush
    std::vector<std::vector<unsigned char>> payloads; // each inner vector's data pointer remains valid after outer moves
    // Keep stable pointers to const char* values for operator enable/disable commands
    std::vector<std::unique_ptr<const char*>> char_ptrs;
};

static void push_set_param(TestContext& ctx, sim::Solver* s, const char* name, float value) {
    struct Payload { const char* name; float v; } p{ name, value };
    ctx.payloads.emplace_back(sizeof(Payload));
    std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
    sim::Command cmd{ sim::CommandTag::SetParam, ctx.payloads.back().data(), ctx.payloads.back().size() };
    (void)sim::push_command(s, cmd);
}

static void push_enable_operator(TestContext& ctx, sim::Solver* s, const char* op_name) {
    // payload must be a pointer to a const char*
    ctx.char_ptrs.emplace_back(std::make_unique<const char*>(op_name));
    const char** ptr = const_cast<const char**>(ctx.char_ptrs.back().get());
    sim::Command cmd{ sim::CommandTag::EnableOperator, ptr, sizeof(const char*) };
    (void)sim::push_command(s, cmd);
}

// static void push_disable_operator(TestContext& ctx, sim::Solver* s, const char* op_name) {
//     ctx.char_ptrs.emplace_back(std::make_unique<const char*>(op_name));
//     const char** ptr = const_cast<const char**>(ctx.char_ptrs.back().get());
//     sim::Command cmd{ sim::CommandTag::DisableOperator, ptr, sizeof(const char*) };
//     (void)sim::push_command(s, cmd);
// }

static void push_set_field_region_scalar(TestContext& ctx, sim::Solver* s, const char* field, uint32_t start, uint32_t count, float scalar) {
    struct Payload { const char* field; uint32_t start; uint32_t count; float v[3]; } p{};
    p.field = field; p.start = start; p.count = count; p.v[0] = scalar; p.v[1] = scalar; p.v[2] = scalar;
    ctx.payloads.emplace_back(sizeof(Payload));
    std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
    sim::Command cmd{ sim::CommandTag::SetFieldRegion, ctx.payloads.back().data(), ctx.payloads.back().size() };
    (void)sim::push_command(s, cmd);
}

static void push_set_field_region_vec3(TestContext& ctx, sim::Solver* s, const char* field, uint32_t start, uint32_t count, float x, float y, float z) {
    struct Payload { const char* field; uint32_t start; uint32_t count; float v[3]; } p{};
    p.field = field; p.start = start; p.count = count; p.v[0] = x; p.v[1] = y; p.v[2] = z;
    ctx.payloads.emplace_back(sizeof(Payload));
    std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
    sim::Command cmd{ sim::CommandTag::SetFieldRegion, ctx.payloads.back().data(), ctx.payloads.back().size() };
    (void)sim::push_command(s, cmd);
}

static bool nearly_equal(float a, float b, float eps) { return std::fabs(a-b) <= eps; }

struct BuildHolder {
    sim::BuildDesc desc{};
    std::vector<sim::FieldView> fields;
    std::vector<sim::RelationView> rels;
    std::vector<sim::Param> params;
};

static void setup_build_basic_with_layout(BuildHolder& h,
                              const std::vector<Vec3>& pos,
                              const std::vector<Vec3>* vel,
                              const std::vector<uint32_t>& edges,
                              const std::vector<uint32_t>* bend_pairs,
                              int substeps,
                              int iterations,
                              float damping,
                              sim::DataLayout layout) {
    using namespace sim;
    h = BuildHolder{};
    h.fields.clear();
    sim::FieldView fpos{}; fpos.name = "position"; fpos.type = sim::FieldType::F32; fpos.data = pos.data(); fpos.count = pos.size(); fpos.components = 3; fpos.stride_bytes = sizeof(Vec3);
    h.fields.push_back(fpos);
    if (vel) {
        sim::FieldView fvel{}; fvel.name = "velocity"; fvel.type = sim::FieldType::F32; fvel.data = vel->data(); fvel.count = vel->size(); fvel.components = 3; fvel.stride_bytes = sizeof(Vec3);
        h.fields.push_back(fvel);
    }
    h.rels.clear();
    if (!edges.empty()) { sim::RelationView r{}; r.indices = edges.data(); r.arity = 2; r.count = edges.size()/2; r.tag = "edges"; h.rels.push_back(r); }
    if (bend_pairs && !bend_pairs->empty()) { sim::RelationView r{}; r.indices = bend_pairs->data(); r.arity = 4; r.count = bend_pairs->size()/4; r.tag = "bend_pairs"; h.rels.push_back(r); }

    sim::Policy pol{};
    pol.exec.layout = layout;
    pol.exec.backend = sim::Backend::Native;
    pol.exec.threads = 1;
    pol.exec.deterministic = true;
    pol.exec.telemetry = true;
    pol.solve.substeps = substeps;
    pol.solve.iterations = iterations;
    pol.solve.damping = damping;
    pol.solve.stepper = sim::TimeStepper::Symplectic;

    h.params.clear();
    sim::Param px{ "gravity_x", sim::ParamType::F32, {.f32 = 0.0f} };
    sim::Param py{ "gravity_y", sim::ParamType::F32, {.f32 = 0.0f} };
    sim::Param pz{ "gravity_z", sim::ParamType::F32, {.f32 = 0.0f} };
    h.params.push_back(px); h.params.push_back(py); h.params.push_back(pz);

    h.desc = sim::BuildDesc{};
    h.desc.state = sim::StateInit{ h.fields.data(), h.fields.size() };
    h.desc.params = sim::Parameters{ h.params.data(), h.params.size() };
    h.desc.topo = sim::TopologyIn{ (uint32_t)pos.size(), h.rels.data(), h.rels.size() };
    h.desc.policy = pol;
    h.desc.space = sim::SpaceDesc{ sim::SpaceType::Lagrangian, 1, 0 };
    h.desc.ops = sim::OperatorsDecl{ nullptr, 0 };
    h.desc.events = sim::EventsScript{ nullptr, 0 };
    h.desc.validate = sim::ValidateLevel::Strict;
    h.desc.pack = sim::PackOptions{ false, 8 };
}

static void setup_build_basic(BuildHolder& h,
                              const std::vector<Vec3>& pos,
                              const std::vector<Vec3>* vel,
                              const std::vector<uint32_t>& edges,
                              const std::vector<uint32_t>* bend_pairs,
                              int substeps,
                              int iterations,
                              float damping,
                              bool blocked_layout=false) {
    setup_build_basic_with_layout(h, pos, vel, edges, bend_pairs, substeps, iterations, damping, blocked_layout ? sim::DataLayout::Blocked : sim::DataLayout::SoA);
}

static int test_distance_convergence() {
    using namespace sim;
    // 2-node spring, initial velocity stretches the spring; expect residual -> small
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0} };
    std::vector<Vec3> vel = { {0,0,0}, {0.5f,0,0} };
    std::vector<uint32_t> edges = { 0,1 };
    BuildHolder bh{}; setup_build_basic(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/16, /*damping*/0.0f);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value;
    // step a few frames
    for (int i=0;i<5;i++) {
        if (sim::step(s, 0.016f) != Status::Ok) { sim::destroy(s); return 2; }
        sim::TelemetryFrame tf{}; (void) sim::telemetry_query_frame(s, &tf);
        // residual should be small after some iterations
        if (i==4 && !(tf.residual_avg < 1e-4)) { sim::destroy(s); return 3; }
    }
    sim::destroy(s);
    return 0;
}

static int test_distance_convergence_aos() {
    using namespace sim;
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0} };
    std::vector<Vec3> vel = { {0,0,0}, {0.5f,0,0} };
    std::vector<uint32_t> edges = { 0,1 };
    BuildHolder bh{}; setup_build_basic_with_layout(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/16, /*damping*/0.0f, sim::DataLayout::AoS);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value;
    for (int i=0;i<5;i++) {
        if (sim::step(s, 0.016f) != Status::Ok) { sim::destroy(s); return 2; }
        sim::TelemetryFrame tf{}; (void) sim::telemetry_query_frame(s, &tf);
        if (i==4 && !(tf.residual_avg < 1e-4)) { sim::destroy(s); return 3; }
    }
    sim::destroy(s);
    return 0;
}

static int test_distance_convergence_blocked() {
    using namespace sim;
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0} };
    std::vector<Vec3> vel = { {0,0,0}, {0.5f,0,0} };
    std::vector<uint32_t> edges = { 0,1 };
    BuildHolder bh{}; setup_build_basic_with_layout(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/16, /*damping*/0.0f, sim::DataLayout::Blocked);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value;
    for (int i=0;i<5;i++) {
        if (sim::step(s, 0.016f) != Status::Ok) { sim::destroy(s); return 2; }
        sim::TelemetryFrame tf{}; (void) sim::telemetry_query_frame(s, &tf);
        if (i==4 && !(tf.residual_avg < 1e-4)) { sim::destroy(s); return 3; }
    }
    sim::destroy(s);
    return 0;
}

static int test_attachment_operator() {
    using namespace sim;
    std::vector<Vec3> pos = { {0,0,0} };
    std::vector<Vec3> vel = { {0,0,0} };
    std::vector<uint32_t> edges; // none
    BuildHolder bh{}; setup_build_basic(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/8, /*damping*/0.0f);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value; TestContext ctx{};
    // Enable attachment and set weight and target
    push_enable_operator(ctx, s, "attachment");
    push_set_field_region_scalar(ctx, s, "attach_w", 0, 1, 1.0f);
    push_set_field_region_vec3(ctx, s, "attach_target", 0, 1, 2.0f, 3.0f, 4.0f);
    sim::flush_commands(s, ApplyPhase::BeforeFrame);
    if (sim::step(s, 0.01f) != Status::Ok) { sim::destroy(s); return 2; }
    // Read back positions
    float buf[3]{}; size_t outN=0; sim::copy_positions(s, buf, 0, &outN);
    if (outN != 1) { sim::destroy(s); return 3; }
    if (!(nearly_equal(buf[0],2.0f,1e-5f) && nearly_equal(buf[1],3.0f,1e-5f) && nearly_equal(buf[2],4.0f,1e-5f))) { sim::destroy(s); return 4; }
    sim::destroy(s);
    return 0;
}

static float dihedral_angle(const std::vector<Vec3>& p, uint32_t i0, uint32_t i1, uint32_t i2, uint32_t i3) {
    auto sub = [](const Vec3& a, const Vec3& b){ return Vec3{a.x-b.x, a.y-b.y, a.z-b.z}; };
    auto cross = [](const Vec3& a, const Vec3& b){ return Vec3{ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x }; };
    auto dot = [](const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; };
    auto len = [&](const Vec3& a){ return std::sqrt(dot(a,a)); };
    Vec3 e0 = sub(p[i1], p[i0]);
    Vec3 e1 = sub(p[i2], p[i0]);
    Vec3 e2 = sub(p[i3], p[i0]);
    Vec3 n1 = cross(e0,e1);
    Vec3 n2 = cross(e0,e2);
    float l1 = len(n1), l2 = len(n2);
    if (l1 <= 1e-12f || l2 <= 1e-12f) return 0.0f;
    float c = dot(n1,n2) / (l1*l2);
    c = std::max(-1.0f, std::min(1.0f, c));
    return std::acos(c);
}

static int test_bending_convergence() {
    using namespace sim;
    // Two triangles sharing edge (0-1). Initial plane z=0, rest dihedral=0.
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0} };
    std::vector<Vec3> vel = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };
    // Apply out-of-plane initial velocity to node 2 to bend
    vel[2].z = 1.0f;
    std::vector<uint32_t> edges = { 0,1, 1,2, 2,0, 0,3, 1,3 };
    std::vector<uint32_t> bends = { 0,1,2,3 };
    BuildHolder bh{}; setup_build_basic(bh, pos, &vel, edges, &bends, /*substeps*/1, /*iterations*/20, /*damping*/0.0f);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value; TestContext ctx{};
    // Enable bending
    push_enable_operator(ctx, s, "bending");
    sim::flush_commands(s, ApplyPhase::BeforeFrame);
    // Step several frames
    for (int i=0;i<10;i++) if (sim::step(s, 0.01f) != Status::Ok) { sim::destroy(s); return 2; }
    // Copy positions and compute dihedral angle (should be close to 0)
    std::vector<float> buf(12,0.0f); size_t outN=0; sim::copy_positions(s, buf.data(), 0, &outN);
    if (outN != 4) { sim::destroy(s); return 3; }
    std::vector<Vec3> p(4);
    for (size_t i=0;i<4;i++) p[i] = Vec3{buf[3*i+0], buf[3*i+1], buf[3*i+2]};
    float ang = dihedral_angle(p, 0,1,2,3);
    if (!(std::fabs(ang) < 0.05f)) { sim::destroy(s); return 4; }
    sim::destroy(s);
    return 0;
}

static int test_pinned_node() {
    using namespace sim;
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0} };
    std::vector<Vec3> vel = { {0,0,0}, {0,0,0} };
    std::vector<uint32_t> edges = { 0,1 };
    BuildHolder bh{}; setup_build_basic(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/8, /*damping*/0.0f);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value; TestContext ctx{};
    // Apply gravity in -y to move node 1; pin node 0 (inv_mass=0)
    push_set_param(ctx, s, "gravity_y", -9.8f);
    push_set_field_region_scalar(ctx, s, "inv_mass", 0, 1, 0.0f);
    sim::flush_commands(s, ApplyPhase::BeforeFrame);
    // Step
    for (int i=0;i<10;i++) if (sim::step(s, 0.016f) != Status::Ok) { sim::destroy(s); return 2; }
    float buf[6]{}; size_t outN=0; sim::copy_positions(s, buf, 0, &outN);
    if (outN != 2) { sim::destroy(s); return 3; }
    // Node 0 unchanged
    if (!(nearly_equal(buf[0],0.0f,1e-6f) && nearly_equal(buf[1],0.0f,1e-6f) && nearly_equal(buf[2],0.0f,1e-6f))) { sim::destroy(s); return 4; }
    // Node 1 moved in -y
    if (!(buf[4] < -0.01f)) { sim::destroy(s); return 5; }
    sim::destroy(s);
    return 0;
}

static int test_per_edge_compliance() {
    using namespace sim;
    std::vector<Vec3> pos = { {0,0,0}, {1,0,0} };
    std::vector<Vec3> vel = { {0,0,0}, {1.0f,0,0} };
    std::vector<uint32_t> edges = { 0,1 };
    // Baseline solver (stiff)
    BuildHolder ah{}; setup_build_basic(ah, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/10, /*damping*/0.0f);
    auto resA = sim::create(ah.desc); if (resA.status != Status::Ok || !resA.value) return 1;
    Solver* sA = resA.value; TestContext ctxA{};
    // Compliant solver with high per-edge compliance
    BuildHolder bh{}; setup_build_basic(bh, pos, &vel, edges, nullptr, /*substeps*/1, /*iterations*/10, /*damping*/0.0f);
    auto resB = sim::create(bh.desc); if (resB.status != Status::Ok || !resB.value) { sim::destroy(sA); return 2; }
    Solver* sB = resB.value; TestContext ctxB{};
    // Set per-edge compliance for the only edge
    push_set_field_region_scalar(ctxB, sB, "distance_compliance_edge", 0, 1, 1e-2f);
    sim::flush_commands(sA, ApplyPhase::BeforeFrame);
    sim::flush_commands(sB, ApplyPhase::BeforeFrame);
    // Step both once
    if (sim::step(sA, 0.01f) != Status::Ok) { sim::destroy(sA); sim::destroy(sB); return 3; }
    if (sim::step(sB, 0.01f) != Status::Ok) { sim::destroy(sA); sim::destroy(sB); return 4; }
    sim::TelemetryFrame tfA{}, tfB{};
    (void) sim::telemetry_query_frame(sA, &tfA);
    (void) sim::telemetry_query_frame(sB, &tfB);
    // Expect B to have larger residual due to compliance
    if (!(tfB.residual_avg > tfA.residual_avg + 1e-6)) { sim::destroy(sA); sim::destroy(sB); return 5; }
    sim::destroy(sA); sim::destroy(sB);
    return 0;
}

static int test_copy_positions() {
    using namespace sim;
    std::vector<Vec3> pos = { {10,20,30}, {40,50,60}, {70,80,90} };
    std::vector<Vec3> vel; // none
    std::vector<uint32_t> edges; // none
    BuildHolder bh{}; setup_build_basic(bh, pos, nullptr, edges, nullptr, /*substeps*/1, /*iterations*/1, /*damping*/0.0f);
    auto res = sim::create(bh.desc); if (res.status != Status::Ok || !res.value) return 1;
    Solver* s = res.value;
    std::vector<float> buf(9, 0.0f); size_t outN=0;
    if (sim::copy_positions(s, buf.data(), 0, &outN) != Status::Ok) { sim::destroy(s); return 2; }
    if (outN != 3) { sim::destroy(s); return 3; }
    for (size_t i=0;i<3;i++) {
        if (!nearly_equal(buf[3*i+0], pos[i].x, 1e-6f) || !nearly_equal(buf[3*i+1], pos[i].y, 1e-6f) || !nearly_equal(buf[3*i+2], pos[i].z, 1e-6f)) { sim::destroy(s); return 4; }
    }
    sim::destroy(s);
    return 0;
}

struct NamedTest { const char* name; std::function<int()> fn; };

int main(int argc, char** argv) {
    std::string filter;
    for (int i=1;i<argc;i++) {
        if (std::strcmp(argv[i], "--filter") == 0 && i+1<argc) { filter = argv[i+1]; i++; }
    }
    std::vector<NamedTest> tests = {
        { "distance", test_distance_convergence },
        { "distance_aos", test_distance_convergence_aos },
        { "distance_blocked", test_distance_convergence_blocked },
        { "attachment", test_attachment_operator },
        { "bending", test_bending_convergence },
        { "pinned", test_pinned_node },
        { "per_edge", test_per_edge_compliance },
        { "copypos", test_copy_positions },
    };
    int failed = 0, ran = 0;
    for (auto& t : tests) {
        if (!filter.empty() && t.name != filter) continue;
        ran++;
        int rc = t.fn();
        if (rc != 0) {
            std::cerr << "[FAIL] " << t.name << " rc=" << rc << "\n";
            failed++;
        } else {
            std::cout << "[PASS] " << t.name << "\n";
        }
    }
    if (filter.empty() && ran == 0) { std::cout << "No tests ran" << std::endl; }
    return failed == 0 ? 0 : 1;
}
