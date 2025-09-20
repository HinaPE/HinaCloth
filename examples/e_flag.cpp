#include "api/sim.h"
#include <vector>
#include <cstdint>
#include <cstdio>

using namespace sim;

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx) { return j * nx + i; }

static void make_grid(uint32_t nx, uint32_t ny, float dx,
                      std::vector<float>& pos,
                      std::vector<float>& vel,
                      std::vector<uint32_t>& edges,
                      std::vector<uint32_t>& bend_pairs) {
    uint32_t n = nx * ny;
    pos.resize(3u * n);
    vel.assign(3u * n, 0.0f);
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t id = vid(i,j,nx);
            pos[3u*id+0] = (float) i * dx;
            pos[3u*id+1] = 0.5f;
            pos[3u*id+2] = (float) j * dx;
        }
    }
    // structural edges (grid 4-neighborhood)
    edges.clear();
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = vid(i,j,nx), b = vid(i+1,j,nx);
            edges.push_back(a); edges.push_back(b);
        }
    }
    for (uint32_t j = 0; j + 1 < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t a = vid(i,j,nx), b = vid(i,j+1,nx);
            edges.push_back(a); edges.push_back(b);
        }
    }
    // bending quads per cell: triangles (A,B,C) and (D,C,B) share edge (B,C)
    bend_pairs.clear();
    for (uint32_t j = 0; j + 1 < ny; ++j) {
        for (uint32_t i = 0; i + 1 < nx; ++i) {
            uint32_t A = vid(i,  j,  nx);
            uint32_t B = vid(i+1,j,  nx);
            uint32_t C = vid(i,  j+1,nx);
            uint32_t D = vid(i+1,j+1,nx);
            // shared edge (C,B): i0=C, i1=B, i2=A, i3=D
            bend_pairs.push_back(C);
            bend_pairs.push_back(B);
            bend_pairs.push_back(A);
            bend_pairs.push_back(D);
        }
    }
}

int main() {
    uint32_t nx = 24, ny = 16; float dx = 0.05f;
    std::vector<float> pos, vel; std::vector<uint32_t> edges, bend_pairs;
    make_grid(nx, ny, dx, pos, vel, edges, bend_pairs);

    FieldView fpos{"position", FieldType::F32, pos.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx*ny, 3, sizeof(float)*3};
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};

    RelationView rel_edges{edges.data(), 2, edges.size()/2, "edges"};
    RelationView rel_bend{bend_pairs.data(), 4, bend_pairs.size()/4, "bend_pairs"};
    RelationView rels[2] = {rel_edges, rel_bend};
    TopologyIn topo{nx*ny, rels, 2};

    Policy pol{{DataLayout::Auto, Backend::Native, 1, true, false}, {2, 10, 0.02f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};

    // Operators declaration (for documentation only in this MVP)
    const char* tags_dist[] = {"edges"};
    FieldUse uses_pos[] = {{"position", true}};
    OperatorDecl opd{"distance", tags_dist, 1, uses_pos, 1, OpStage::Solve, true};
    const char* tags_bend[] = {"bend_pairs"};
    OperatorDecl opb{"bending", tags_bend, 1, uses_pos, 1, OpStage::Solve, true};
    OperatorDecl ops_arr[2] = {opd, opb};
    OperatorsDecl ops{ops_arr, 2};

    EventsScript ev{nullptr, 0};
    Parameters params{nullptr, 0};

    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
    auto r = create(bd);
    if (r.status != Status::Ok) { std::printf("e_flag: create failed\n"); return 1; }
    Solver* s = r.value;

    // Enable bending and attachment operators via commands
    const char* bend_id = "bending";
    Command cmd_enable_bend{CommandTag::EnableOperator, &bend_id, sizeof(bend_id)};
    push_command(s, cmd_enable_bend);
    const char* att_id = "attachment";
    Command cmd_enable_att{CommandTag::EnableOperator, &att_id, sizeof(att_id)};
    push_command(s, cmd_enable_att);

    // Pin the top edge: inv_mass=0 for first row [0..nx-1]
    struct RegionWrite { const char* field; uint32_t start; uint32_t count; float v[3]; };
    RegionWrite pinTop{"inv_mass", 0u, nx, {0.0f,0.0f,0.0f}};
    Command cmd_pin_top{CommandTag::SetFieldRegion, &pinTop, sizeof(pinTop)};
    push_command(s, cmd_pin_top);

    // Attachment: pull the middle column slightly in +X direction
    uint32_t mid = nx / 2;
    RegionWrite attW{"attach_w", mid, ny, {0.5f,0,0}}; // set weight for nodes: but our API expects [start,count] in linear indices
    // Here we instead write a contiguous block over [mid..mid+ny) which is not the right linearization for column; for demo we'll write full range
    RegionWrite attWAll{"attach_w", 0u, nx*ny, {0.0f,0,0}};
    Command cmd_attw_all{CommandTag::SetFieldRegion, &attWAll, sizeof(attWAll)};
    push_command(s, cmd_attw_all);
    // set center line by small loop of commands (fine for demo)
    for (uint32_t j=0; j<ny; ++j) {
        uint32_t id = vid(mid,j,nx);
        RegionWrite attW1{"attach_w", id, 1u, {0.5f,0,0}};
        Command c{CommandTag::SetFieldRegion, &attW1, sizeof(attW1)};
        push_command(s, c);
        float tx = pos[3u*id+0] + 0.2f; float ty = pos[3u*id+1]; float tz = pos[3u*id+2];
        RegionWrite attT{"attach_target", id, 1u, {tx,ty,tz}};
        Command c2{CommandTag::SetFieldRegion, &attT, sizeof(attT)};
        push_command(s, c2);
    }

    flush_commands(s, ApplyPhase::BeforeFrame);

    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 120; ++f) {
        step(s, dt);
        // Optionally flush AfterSolve queued commands
        flush_commands(s, ApplyPhase::AfterSolve);
        if (f % 30 == 0) {
            TelemetryFrame tf{}; Status rc = telemetry_query_frame(s, &tf); (void)rc;
            std::printf("[flag] frame=%d step_ms=%.3f cmds=%llu rebuilds=%llu\n", f, tf.step_ms, (unsigned long long)tf.commands_applied, (unsigned long long)tf.structural_rebuilds);
        }
    }

    destroy(s);
    return 0;
}
