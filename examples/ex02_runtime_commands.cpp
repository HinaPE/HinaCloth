#include "api/sim.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace sim;

static inline uint32_t vid(uint32_t i, uint32_t j, uint32_t nx) {
    return j * nx + i;
}
static void make_grid(uint32_t nx, uint32_t ny, float dx, std::vector<float>& pos, std::vector<float>& vel, std::vector<uint32_t>& edges) {
    uint32_t n = nx * ny;
    pos.resize(3u * n);
    vel.assign(3u * n, 0.0f);
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t id      = vid(i, j, nx);
            pos[3u * id + 0] = (float) i * dx;
            pos[3u * id + 1] = 0.5f;
            pos[3u * id + 2] = (float) j * dx;
        }
    }
    edges.clear();
    for (uint32_t j = 0; j < ny; ++j) {
        for (uint32_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = vid(i, j, nx), b = vid(i + 1, j, nx);
            edges.push_back(a);
            edges.push_back(b);
        }
    }
    for (uint32_t j = 0; j + 1 < ny; ++j) {
        for (uint32_t i = 0; i < nx; ++i) {
            uint32_t a = vid(i, j, nx), b = vid(i, j + 1, nx);
            edges.push_back(a);
            edges.push_back(b);
        }
    }
}

struct ParamPayload {
    const char* name;
    float value;
};
struct RegionPayload {
    const char* name;
    uint32_t start;
    uint32_t count;
    float v[3];
};

int main() {
    uint32_t nx = 20, ny = 14;
    float dx = 0.05f;
    std::vector<float> pos, vel;
    std::vector<uint32_t> edges;
    make_grid(nx, ny, dx, pos, vel, edges);
    FieldView fpos{"position", FieldType::F32, pos.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};
    RelationView rel{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{nx * ny, &rel, 1};
    const char* tags[] = {"edges"};
    FieldUse uses[]    = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true};
    OperatorsDecl ops{&op, 1};
    Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {1, 8, 0.01f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};
    Parameters params{nullptr, 0};
    EventsScript ev{nullptr, 0};
    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
    auto r = create(bd);
    if (r.status != Status::Ok) {
        std::printf("ex02: create failed\n");
        return 1;
    }
    Solver* s = r.value;

    // Queue runtime commands: increase iterations, set substeps, adjust gravity, and pin left edge (inv_mass=0)
    ParamPayload itp{"iterations", 16.0f};
    Command c_it{CommandTag::SetParam, &itp, sizeof(itp)};
    push_command(s, c_it);
    ParamPayload sbp{"substeps", 2.0f};
    Command c_sb{CommandTag::SetParam, &sbp, sizeof(sbp)};
    push_command(s, c_sb);
    ParamPayload ddp{"damping", 0.02f};
    Command c_dp{CommandTag::SetParam, &ddp, sizeof(ddp)};
    push_command(s, c_dp);
    ParamPayload gyp{"gravity_y", -12.0f};
    Command c_gy{CommandTag::SetParam, &gyp, sizeof(gyp)};
    push_command(s, c_gy);
    // Pin leftmost column: indices j*nx + 0 => not contiguous; update per-row
    for (uint32_t j = 0; j < ny; ++j) {
        RegionPayload pin{"inv_mass", vid(0, j, nx), 1u, {0.0f, 0.0f, 0.0f}};
        Command c_pin{CommandTag::SetFieldRegion, &pin, sizeof(pin)};
        push_command(s, c_pin);
    }

    flush_commands(s, ApplyPhase::BeforeFrame);

    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 120; ++f) {
        step(s, dt);
    }
    TelemetryFrame tf{};
    Status st_tf = telemetry_query_frame(s, &tf);
    (void) st_tf;
    std::printf("ex02: step_ms=%.3f residual=%.6f sub=%d it=%d cmds=%llu\n", tf.step_ms, tf.residual_avg, tf.solve_substeps, tf.solve_iterations, (unsigned long long) tf.commands_applied);
    destroy(s);
    return 0;
}
