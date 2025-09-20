// examples/e_smoke.cpp
// Stage 0: Minimal smoke test. Verifies create -> flush(BeforeFrame) -> step -> telemetry -> destroy.

#include "api/sim.h"
#include <cstdio>
#include <vector>

using namespace sim;

static void make_small_grid(size_t nx, size_t ny, float dx,
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
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i + 1 < nx; ++i) { uint32_t a = (uint32_t)(j * nx + i); edges.push_back(a); edges.push_back(a + 1); }
    }
    for (size_t j = 0; j + 1 < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) { uint32_t a = (uint32_t)(j * nx + i); edges.push_back(a); edges.push_back(a + (uint32_t)nx); }
    }
}

int main() {
    std::vector<float> pos, vel; std::vector<uint32_t> edges;
    make_small_grid(4, 4, 0.1f, pos, vel, edges);

    FieldView fpos; fpos.name = "position"; fpos.type = FieldType::F32; fpos.data = pos.data(); fpos.count = 16; fpos.components = 3; fpos.stride_bytes = sizeof(float) * 3;
    FieldView fvel; fvel.name = "velocity"; fvel.type = FieldType::F32; fvel.data = vel.data(); fvel.count = 16; fvel.components = 3; fvel.stride_bytes = sizeof(float) * 3;
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};

    RelationView rel{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{(uint32_t)16, &rel, 1};

    Param gy{}; gy.name = "gravity_y"; gy.type = ParamType::F32; gy.value.f32 = -9.8f;
    Parameters params{&gy, 1};

    Policy pol{{DataLayout::Auto, Backend::Native, 1, true, false}, {1, 4, 0.0f, TimeStepper::Symplectic}};
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};

    const char* tags[] = {"edges"};
    FieldUse uses[] = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true};
    OperatorsDecl ops{&op, 1};

    EventsScript ev{nullptr, 0};

    BuildDesc bd{st, params, topo, pol, sp, ops, ev, ValidateLevel::Strict, {true, 64}};

    auto r = create(bd);
    if (r.status != Status::Ok) { std::printf("[smoke] create failed\n"); return 1; }
    Solver* s = r.value;

    const float dt = 1.0f / 60.0f;
    flush_commands(s, ApplyPhase::BeforeFrame);
    for (int i = 0; i < 3; ++i) {
        if (step(s, dt) != Status::Ok) { std::printf("[smoke] step failed\n"); destroy(s); return 2; }
    }

    TelemetryFrame tf{}; Status rc = telemetry_query_frame(s, &tf); (void)rc;
    std::printf("[smoke] ok: step_ms=%.3f cmds=%llu rebuilds=%llu\n", tf.step_ms, (unsigned long long)tf.commands_applied, (unsigned long long)tf.structural_rebuilds);

    destroy(s);
    return 0;
}
