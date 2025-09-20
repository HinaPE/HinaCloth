#include "api/sim.h"
#include <cstdint>
#include <cstdio>
#include <vector>

static void make_grid(size_t nx, size_t ny, float dx,
    std::vector<float>& pos,
    std::vector<float>& vel,
    std::vector<uint32_t>& edges) {
    size_t n = nx * ny;
    pos.resize(3 * n);
    vel.assign(3 * n, 0.0f);
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            size_t id       = j * nx + i;
            pos[3 * id + 0] = float(i) * dx;
            pos[3 * id + 1] = 0.5f;
            pos[3 * id + 2] = float(j) * dx;
        }
    }
    edges.clear();
    edges.reserve((nx - 1) * ny + (ny - 1) * nx);
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + 1;
            edges.push_back(a); edges.push_back(b);
        }
    }
    for (size_t j = 0; j + 1 < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + (uint32_t)nx;
            edges.push_back(a); edges.push_back(b);
        }
    }
}

int main() {
    using namespace sim;
    size_t nx = 20, ny = 20; float dx = 0.05f;
    std::vector<float> pos, vel; std::vector<uint32_t> edges;
    make_grid(nx, ny, dx, pos, vel, edges);

    FieldView fpos{"position", FieldType::F32, pos.data(), nx * ny, 3, (unsigned) (sizeof(float) * 3)};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx * ny, 3, (unsigned) (sizeof(float) * 3)};
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};

    RelationView rel{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{(uint32_t)(nx * ny), &rel, 1};

    Param pgrav; pgrav.name = "gravity_y"; pgrav.type = ParamType::F32; pgrav.value.f32 = -9.8f;
    Parameters params{&pgrav, 1};

    Policy pol{{DataLayout::Auto, Backend::Native, 1, true, false}, {1, 10, 0.01f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};

    const char* tags[] = {"edges"};
    FieldUse uses[]    = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true};
    OperatorsDecl ops{&op, 1};

    EventsScript ev{nullptr, 0};

    BuildDesc bd{ st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 64} };

    auto r = create(bd);
    if (r.status != Status::Ok) { std::printf("create failed\n"); return 1; }
    Solver* s = r.value;

    // Pin top row by setting inv_mass=0 over the region
    struct RegionPayload { const char* name; uint32_t start; uint32_t count; float v[3]; };
    RegionPayload pin{}; pin.name = "inv_mass"; pin.start = (uint32_t)((ny - 1) * nx); pin.count = (uint32_t) nx; pin.v[0] = 0.0f; pin.v[1] = 0.0f; pin.v[2] = 0.0f;
    Command cmd_pin{CommandTag::SetFieldRegion, &pin, sizeof(pin)};
    Status st_pin = push_command(s, cmd_pin); (void)st_pin;

    // Increase iterations at runtime via SetParam
    struct ParamPayload { const char* name; float value; };
    ParamPayload itp{ "iterations", 15.0f };
    Command cmd_iter{CommandTag::SetParam, &itp, sizeof(itp)};
    Status st_iter = push_command(s, cmd_iter); (void)st_iter;

    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 120; ++f) {
        Status st_flush = flush_commands(s, ApplyPhase::BeforeFrame); (void)st_flush;
        Status st_step = step(s, dt); (void)st_step;
    }

    TelemetryFrame tf{}; Status st_tf = telemetry_query_frame(s, &tf); (void)st_tf;
    std::printf("telemetry: step_ms=%.3f residual=%.6f sub=%d it=%d\n", tf.step_ms, tf.residual_avg, tf.solve_substeps, tf.solve_iterations);

    destroy(s);
    return 0;
}
