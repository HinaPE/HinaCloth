#include "api/sim.h"
#include <cstdint>
#include <cstdio>
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

int main() {
    // Build a small 16x16 cloth with distance constraints only
    uint32_t nx = 16, ny = 16;
    float dx = 0.05f;
    std::vector<float> pos, vel;
    std::vector<uint32_t> edges;
    make_grid(nx, ny, dx, pos, vel, edges);
    FieldView fpos{"position", FieldType::F32, pos.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView flds[2] = {fpos, fvel};
    StateInit st{flds, 2};
    RelationView rel{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{nx * ny, &rel, 1};
    // Operators: distance writes position
    const char* tags[] = {"edges"};
    FieldUse uses[]    = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true};
    OperatorsDecl ops{&op, 1};
    // Defaults: gravity via params; simple solve policy
    Param gpy{"gravity_y", ParamType::F32, {.f32 = -9.8f}};
    Parameters params{&gpy, 1};
    Policy pol{{DataLayout::Auto, Backend::Auto, -1, true, true}, {1, 8, 0.0f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};
    EventsScript ev{nullptr, 0};
    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 8}};
    auto r = create(bd);
    if (r.status != Status::Ok) {
        std::printf("ex00: create failed\n");
        return 1;
    }
    Solver* s      = r.value;
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 60; ++f) {
        step(s, dt);
    }
    TelemetryFrame tf{};
    Status st_tf = telemetry_query_frame(s, &tf);
    (void) st_tf;
    std::printf("ex00: step_ms=%.3f residual=%.6f sub=%d it=%d\n", tf.step_ms, tf.residual_avg, tf.solve_substeps, tf.solve_iterations);
    destroy(s);
    return 0;
}
