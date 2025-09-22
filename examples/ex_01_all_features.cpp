#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <memory>
#include <cmath>
#include <cstdint>

#include "api/sim.h"
#include "api/build.h"
#include "api/status.h"
#include "api/telemetry.h"

struct Vec3 { float x,y,z; };

struct PayloadKeeper {
    std::vector<std::vector<unsigned char>> payloads;
    std::vector<std::unique_ptr<const char*>> char_ptrs;
};

// static void push_set_param(PayloadKeeper& ctx, sim::Solver* s, const char* name, float value) {
//     struct Payload { const char* name; float v; } p{ name, value };
//     ctx.payloads.emplace_back(sizeof(Payload));
//     std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
//     sim::Command cmd{ sim::CommandTag::SetParam, ctx.payloads.back().data(), ctx.payloads.back().size() };
//     (void)sim::push_command(s, cmd);
// }

static void push_enable_operator(PayloadKeeper& ctx, sim::Solver* s, const char* op_name) {
    ctx.char_ptrs.emplace_back(std::make_unique<const char*>(op_name));
    const char** ptr = const_cast<const char**>(ctx.char_ptrs.back().get());
    sim::Command cmd{ sim::CommandTag::EnableOperator, ptr, sizeof(const char*) };
    (void)sim::push_command(s, cmd);
}

static void push_set_field_region_scalar(PayloadKeeper& ctx, sim::Solver* s, const char* field, uint32_t start, uint32_t count, float scalar) {
    struct Payload { const char* field; uint32_t start; uint32_t count; float v[3]; } p{};
    p.field = field; p.start = start; p.count = count; p.v[0] = scalar; p.v[1] = scalar; p.v[2] = scalar;
    ctx.payloads.emplace_back(sizeof(Payload));
    std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
    sim::Command cmd{ sim::CommandTag::SetFieldRegion, ctx.payloads.back().data(), ctx.payloads.back().size() };
    (void)sim::push_command(s, cmd);
}

static void push_set_field_region_vec3(PayloadKeeper& ctx, sim::Solver* s, const char* field, uint32_t start, uint32_t count, float x, float y, float z) {
    struct Payload { const char* field; uint32_t start; uint32_t count; float v[3]; } p{};
    p.field = field; p.start = start; p.count = count; p.v[0] = x; p.v[1] = y; p.v[2] = z;
    ctx.payloads.emplace_back(sizeof(Payload));
    std::memcpy(ctx.payloads.back().data(), &p, sizeof(Payload));
    sim::Command cmd{ sim::CommandTag::SetFieldRegion, ctx.payloads.back().data(), ctx.payloads.back().size() };
    (void)sim::push_command(s, cmd);
}

struct GridDesc {
    int nx{16};
    int ny{16};
    float spacing{0.1f};
};

static void build_grid(const GridDesc& g, std::vector<Vec3>& out_pos, std::vector<uint32_t>& out_edges, std::vector<uint32_t>& out_bends) {
    const int nx = g.nx, ny = g.ny; const float dx = g.spacing, dy = g.spacing;
    out_pos.resize((size_t)nx * (size_t)ny);
    for (int j=0;j<ny;j++) {
        for (int i=0;i<nx;i++) {
            size_t idx = (size_t)j * (size_t)nx + (size_t)i;
            out_pos[idx] = Vec3{ i*dx, 0.0f, j*dy };
        }
    }
    // structural edges (grid 4-neighbors)
    out_edges.clear(); out_edges.reserve((size_t)nx*(size_t)ny*2);
    auto id = [nx](int i, int j){ return (uint32_t)(j*nx + i); };
    for (int j=0;j<ny;j++) {
        for (int i=0;i<nx;i++) {
            if (i+1 < nx) { out_edges.push_back(id(i,j)); out_edges.push_back(id(i+1,j)); }
            if (j+1 < ny) { out_edges.push_back(id(i,j)); out_edges.push_back(id(i,j+1)); }
        }
    }
    // simple bending pairs for each quad: shared edge is (i,j)-(i+1,j) with triangles (i,j,i,j+1) and (i,j,i+1,j+1)
    out_bends.clear();
    for (int j=0;j<ny-1;j++) {
        for (int i=0;i<nx-1;i++) {
            uint32_t a = id(i,j);
            uint32_t b = id(i+1,j);
            uint32_t c = id(i,j+1);
            uint32_t d = id(i+1,j+1);
            out_bends.push_back(a); out_bends.push_back(b); out_bends.push_back(c); out_bends.push_back(d);
            // also vertical shared edge (i,j)-(i,j+1) with triangles (i,j,i+1,j) and (i,j,i+1,j+1)
            out_bends.push_back(a); out_bends.push_back(c); out_bends.push_back(b); out_bends.push_back(d);
        }
    }
}

static void print_usage(const char* exe) {
    std::cout << "Usage: " << exe << " [--nx N] [--ny N] [--spacing S] [--frames F] [--dt T]" << std::endl;
    std::cout << "       " << " [--layout soa|blocked] [--backend native|tbb|avx2] [--threads K]" << std::endl;
    std::cout << "       " << " [--substeps S] [--iters I] [--damping D] [--compliance C]" << std::endl;
}

int main(int argc, char** argv) {
    // Defaults
    GridDesc g{}; int frames = 120; float dt = 1.0f/60.0f;
    sim::DataLayout layout = sim::DataLayout::SoA; sim::Backend backend = sim::Backend::Native; int threads = 1;
    int substeps = 1, iters = 16; float damping = 0.02f; float compliance = 0.0f; bool enable_bending = true; bool enable_attachment = true;

    for (int i=1;i<argc;i++) {
        if (std::strcmp(argv[i], "--nx") == 0 && i+1<argc) { g.nx = std::max(2, std::atoi(argv[++i])); }
        else if (std::strcmp(argv[i], "--ny") == 0 && i+1<argc) { g.ny = std::max(2, std::atoi(argv[++i])); }
        else if (std::strcmp(argv[i], "--spacing") == 0 && i+1<argc) { g.spacing = std::max(1e-4f, (float)std::atof(argv[++i])); }
        else if (std::strcmp(argv[i], "--frames") == 0 && i+1<argc) { frames = std::max(1, std::atoi(argv[++i])); }
        else if (std::strcmp(argv[i], "--dt") == 0 && i+1<argc) { dt = std::max(1e-5f, (float)std::atof(argv[++i])); }
        else if (std::strcmp(argv[i], "--layout") == 0 && i+1<argc) {
            const char* v = argv[++i];
            if (std::strcmp(v, "soa") == 0) layout = sim::DataLayout::SoA;
            else if (std::strcmp(v, "blocked") == 0) layout = sim::DataLayout::Blocked;
        } else if (std::strcmp(argv[i], "--backend") == 0 && i+1<argc) {
            const char* v = argv[++i];
            if (std::strcmp(v, "native") == 0) backend = sim::Backend::Native;
            else if (std::strcmp(v, "tbb") == 0) backend = sim::Backend::TBB;
            else if (std::strcmp(v, "avx2") == 0) backend = sim::Backend::AVX2;
        } else if (std::strcmp(argv[i], "--threads") == 0 && i+1<argc) { threads = std::atoi(argv[++i]); }
        else if (std::strcmp(argv[i], "--substeps") == 0 && i+1<argc) { substeps = std::max(1, std::atoi(argv[++i])); }
        else if (std::strcmp(argv[i], "--iters") == 0 && i+1<argc) { iters = std::max(1, std::atoi(argv[++i])); }
        else if (std::strcmp(argv[i], "--damping") == 0 && i+1<argc) { damping = (float)std::atof(argv[++i]); }
        else if (std::strcmp(argv[i], "--compliance") == 0 && i+1<argc) { compliance = (float)std::atof(argv[++i]); }
        else if (std::strcmp(argv[i], "--no-bending") == 0) { enable_bending = false; }
        else if (std::strcmp(argv[i], "--no-attach") == 0) { enable_attachment = false; }
        else if (std::strcmp(argv[i], "--help") == 0) { print_usage(argv[0]); return 0; }
    }

    // Build grid cloth
    std::vector<Vec3> pos; std::vector<uint32_t> edges; std::vector<uint32_t> bends;
    build_grid(g, pos, edges, bends);

    // Set up build description (similar to tests)
    std::vector<sim::FieldView> fields; fields.reserve(2);
    sim::FieldView fpos{}; fpos.name = "position"; fpos.type = sim::FieldType::F32; fpos.data = pos.data(); fpos.count = pos.size(); fpos.components = 3; fpos.stride_bytes = sizeof(Vec3);
    fields.push_back(fpos);

    std::vector<sim::RelationView> rels; rels.reserve(2);
    if (!edges.empty()) { sim::RelationView r{}; r.indices = edges.data(); r.arity = 2; r.count = edges.size()/2; r.tag = "edges"; rels.push_back(r); }
    if (enable_bending && !bends.empty()) { sim::RelationView r{}; r.indices = bends.data(); r.arity = 4; r.count = bends.size()/4; r.tag = "bend_pairs"; rels.push_back(r); }

    sim::Policy pol{};
    pol.exec.layout = layout; pol.exec.backend = backend; pol.exec.threads = threads; pol.exec.deterministic = true; pol.exec.telemetry = true;
    pol.solve.substeps = substeps; pol.solve.iterations = iters; pol.solve.damping = damping; pol.solve.stepper = sim::TimeStepper::Symplectic;

    std::vector<sim::Param> params;
    params.push_back(sim::Param{ "gravity_x", sim::ParamType::F32, {.f32 = 0.0f} });
    params.push_back(sim::Param{ "gravity_y", sim::ParamType::F32, {.f32 = -9.8f} });
    params.push_back(sim::Param{ "gravity_z", sim::ParamType::F32, {.f32 = 0.0f} });
    params.push_back(sim::Param{ "distance_compliance", sim::ParamType::F32, {.f32 = compliance} });

    sim::BuildDesc desc{};
    desc.state = sim::StateInit{ fields.data(), fields.size() };
    desc.params = sim::Parameters{ params.data(), params.size() };
    desc.topo = sim::TopologyIn{ (uint32_t)pos.size(), rels.data(), rels.size() };
    desc.policy = pol;
    desc.space = sim::SpaceDesc{ sim::SpaceType::Lagrangian, 1, 0 };
    desc.ops = sim::OperatorsDecl{ nullptr, 0 };
    desc.events = sim::EventsScript{ nullptr, 0 };
    desc.validate = sim::ValidateLevel::Strict;
    desc.pack = sim::PackOptions{ false, 8 };

    auto created = sim::create(desc);
    if (created.status != sim::Status::Ok || !created.value) {
        std::cerr << "Failed to create solver (status=" << (int)created.status << ")\n";
        return 1;
    }
    sim::Solver* s = created.value;

    // Print chosen backend/layout
    auto ch = sim::query_chosen(s);
    if (ch.status == sim::Status::Ok) {
        std::cout << "Chosen: layout=" << (int)ch.value.layout << " backend=" << (int)ch.value.backend << " threads=" << ch.value.threads << "\n";
    }

    // Prepare runtime commands: enable operators, pin corners via inv_mass=0, also attachment targets for top edge
    PayloadKeeper ctx{};
    if (enable_bending) push_enable_operator(ctx, s, "bending");
    if (enable_attachment) push_enable_operator(ctx, s, "attachment");

    // Pin four corners using inv_mass = 0
    // uint32_t n = (uint32_t)pos.size();
    auto idx = [nx=g.nx](int i, int j){ return (uint32_t)(j*nx + i); };
    uint32_t i_tl = idx(0,0), i_tr = idx(g.nx-1,0), i_bl = idx(0,g.ny-1), i_br = idx(g.nx-1,g.ny-1);
    push_set_field_region_scalar(ctx, s, "inv_mass", i_tl, 1, 0.0f);
    push_set_field_region_scalar(ctx, s, "inv_mass", i_tr, 1, 0.0f);
    push_set_field_region_scalar(ctx, s, "inv_mass", i_bl, 1, 0.0f);
    push_set_field_region_scalar(ctx, s, "inv_mass", i_br, 1, 0.0f);

    // Also demonstrate attachment: make the entire top row attached to initial positions with weight 1 (a fixed bar)
    if (enable_attachment) {
        for (int i=0;i<g.nx;i++) {
            uint32_t id = idx(i,0);
            push_set_field_region_scalar(ctx, s, "attach_w", id, 1, 1.0f);
            const Vec3& p = pos[id];
            push_set_field_region_vec3(ctx, s, "attach_target", id, 1, p.x, p.y, p.z);
        }
    }

    // Demonstrate per-edge compliance: set uniform per-edge compliance if requested
    if (!edges.empty() && compliance > 0.0f) {
        uint32_t ecount = (uint32_t)(edges.size()/2);
        push_set_field_region_scalar(ctx, s, "distance_compliance_edge", 0, ecount, compliance);
    }

    // Apply commands
    (void) sim::flush_commands(s, sim::ApplyPhase::BeforeFrame);

    // Step simulation
    for (int f=0; f<frames; ++f) {
        if (sim::step(s, dt) != sim::Status::Ok) {
            std::cerr << "Step failed at frame " << f << "\n";
            sim::destroy(s);
            return 2;
        }
        sim::TelemetryFrame tf{}; (void) sim::telemetry_query_frame(s, &tf);
        if (f % 30 == 0 || f == frames-1) {
            std::cout << "Frame " << f << ": dt_ms=" << tf.step_ms << " residual=" << tf.residual_avg
                      << " substeps=" << tf.solve_substeps << " iters=" << tf.solve_iterations << "\n";
        }
    }

    // Read back a few points
    std::vector<float> buf; buf.resize((size_t)pos.size()*3);
    size_t outN=0; if (sim::copy_positions(s, buf.data(), 0, &outN) == sim::Status::Ok) {
        std::cout << "Read back " << outN << " positions. First 5:\n";
        for (size_t i=0;i<std::min<size_t>(outN,5);++i) {
            std::cout << "  p[" << i << "] = (" << buf[3*i+0] << ", " << buf[3*i+1] << ", " << buf[3*i+2] << ")\n";
        }
    }

    sim::destroy(s);
    return 0;
}

