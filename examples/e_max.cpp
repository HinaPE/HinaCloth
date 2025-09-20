// examples/e_max.cpp
// 目标：尽可能覆盖当前 src 中可访问/可调用的 feature 路径，作为“全量冒烟”示例。
// 注意：部分功能当前为占位实现（如 bending/tbb/重建策略等），本示例主要验证调用链不崩溃、
// API 形态与数据路径保持稳定，便于后续替换实现。

#include "api/sim.h"
#include "api/capability.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace sim;

static void make_grid(size_t nx, size_t ny, float dx,
                      std::vector<float>& pos, // AoS xyzxyz...
                      std::vector<float>& vel, // AoS xyzxyz...
                      std::vector<uint32_t>& edges) // 2-ary indices
{
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
    // horizontal
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + 1;
            edges.push_back(a);
            edges.push_back(b);
        }
    }
    // vertical
    for (size_t j = 0; j + 1 < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + uint32_t(nx);
            edges.push_back(a);
            edges.push_back(b);
        }
    }
}

int main()
{
    // 0) 能力枚举（仅打印，不影响 create）
    {
        size_t n = enumerate_capabilities(nullptr, 0);
        std::vector<Capability> caps(n);
        enumerate_capabilities(caps.data(), caps.size());
        std::printf("capabilities (%zu):\n", n);
        for (auto& c : caps) {
            const char* b = c.backend == Backend::Native ? "Native" : (c.backend == Backend::AVX2 ? "AVX2" : (c.backend == Backend::TBB ? "TBB" : (c.backend == Backend::GPU ? "GPU" : "Auto")));
            const char* l = c.layout == DataLayout::SoA ? "SoA" : (c.layout == DataLayout::AoS ? "AoS" : (c.layout == DataLayout::Blocked ? "Blocked" : "Auto"));
            std::printf("  - %s / %s : %s\n", b, l, c.name ? c.name : "(noname)");
        }
    }

    // 1) 组装 State/Topology/Parameters/Policy + Space/Operators/Events
    size_t nx = 24, ny = 16;
    float dx  = 0.05f;
    std::vector<float> pos, vel;
    std::vector<uint32_t> edges;
    make_grid(nx, ny, dx, pos, vel, edges);

    FieldView fpos; fpos.name = "position"; fpos.type = FieldType::F32; fpos.data = pos.data(); fpos.count = nx * ny; fpos.components = 3; fpos.stride_bytes = sizeof(float) * 3;
    FieldView fvel; fvel.name = "velocity"; fvel.type = FieldType::F32; fvel.data = vel.data(); fvel.count = nx * ny; fvel.components = 3; fvel.stride_bytes = sizeof(float) * 3;
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};

    RelationView rel_edges{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{(uint32_t) (nx * ny), &rel_edges, 1};

    Param pgx{}; pgx.name = "gravity_x"; pgx.type = ParamType::F32; pgx.value.f32 = 0.0f;
    Param pgy{}; pgy.name = "gravity_y"; pgy.type = ParamType::F32; pgy.value.f32 = -9.8f;
    Param pgz{}; pgz.name = "gravity_z"; pgz.type = ParamType::F32; pgz.value.f32 = 0.0f;
    Param pcomp{}; pcomp.name = "distance_compliance"; pcomp.type = ParamType::F32; pcomp.value.f32 = 0.0f; // PBD-like
    Param params_arr[4] = {pgx, pgy, pgz, pcomp};
    Parameters params{params_arr, 4};

    // Exec：尝试 TBB + Auto 布局；Solve：子步/迭代/阻尼
    Policy pol{{DataLayout::Auto, Backend::TBB, 4, true, true}, {2, 10, 0.02f, TimeStepper::Symplectic}};

    // Space：任意指定，不影响本例运行路径
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};

    // Operators：声明 distance + bending（bending 当前为占位，实现后可直接纳入）
    const char* tags_distance[] = {"edges"};
    FieldUse uses_pos_rw[]      = {{"position", true}};
    OperatorDecl op_distance{"distance", tags_distance, 1, uses_pos_rw, 1, OpStage::Solve, true};

    // bending 仅为声明演示（未真正参与 runtime）
    const char* tags_bending[] = {"bend_pairs"};
    OperatorDecl op_bending{"bending", tags_bending, 1, uses_pos_rw, 1, OpStage::Solve, false};

    OperatorDecl ops_arr[2] = {op_distance, op_bending};
    OperatorsDecl ops{ops_arr, 2};

    // Events：构造少量占位事件，不会被当前 runtime 消费
    EventRecord ers[2]{{0.0, EventKind::SetParam, nullptr, 0}, {0.5, EventKind::ActivateOperator, nullptr, 0}};
    EventsScript ev{ers, 2};

    BuildDesc bd{
        st, params, topo, pol, sp, ops, ev,
        ValidateLevel::Strict,
        {true, 64}
    };

    auto r = create(bd);
    if (r.status != Status::Ok) {
        std::printf("create failed\n");
        return 1;
    }
    Solver* s = r.value;

    // 2) 查询底层选择
    {
        auto qc = query_chosen(s);
        if (qc.status == Status::Ok) {
            Chosen ch     = qc.value;
            const char* b = ch.backend == Backend::Native ? "Native" : (ch.backend == Backend::AVX2 ? "AVX2" : (ch.backend == Backend::TBB ? "TBB" : (ch.backend == Backend::GPU ? "GPU" : "Auto")));
            const char* l = ch.layout == DataLayout::SoA ? "SoA" : (ch.layout == DataLayout::AoS ? "AoS" : (ch.layout == DataLayout::Blocked ? "Blocked" : "Auto"));
            std::printf("chosen backend=%s layout=%s threads=%d\n", b, l, ch.threads);
        }
    }

    // 3) 命令与相位：小命令 + 结构命令 + AfterSolve 空刷
    struct PayloadF32 { const char* name; float value; };

    // 小命令：调整重力（会生效）
    PayloadF32 gpy1{"gravity_y", -12.0f};
    Command cmd_set_g1{CommandTag::SetParam, &gpy1, sizeof(gpy1)};
    push_command(s, cmd_set_g1);

    // 小命令：尝试调整 iterations/substeps/damping（当前实现不消费，但验证调用链）
    PayloadF32 iters{"iterations", 16.0f};
    PayloadF32 substeps{"substeps", 3.0f};
    PayloadF32 damping{"damping", 0.03f};
    Command cmd_set_iters{CommandTag::SetParam, &iters, sizeof(iters)};
    Command cmd_set_substeps{CommandTag::SetParam, &substeps, sizeof(substeps)};
    Command cmd_set_damping{CommandTag::SetParam, &damping, sizeof(damping)};
    push_command(s, cmd_set_iters);
    push_command(s, cmd_set_substeps);
    push_command(s, cmd_set_damping);

    // 小命令：启停算子（当前未消费）
    const char* op_id_bending = "bending";
    Command cmd_enable_bending{CommandTag::EnableOperator, &op_id_bending, sizeof(op_id_bending)};
    Command cmd_disable_bending{CommandTag::DisableOperator, &op_id_bending, sizeof(op_id_bending)};
    push_command(s, cmd_enable_bending);
    push_command(s, cmd_disable_bending);

    // 小命令：SetFieldRegion 演示 pin 顶边（将 inv_mass 置 0）
    struct RegionWrite { const char* field; uint32_t start; uint32_t count; float v[3]; };
    // 顶边共有 nx 个顶点，从 0 到 nx-1
    RegionWrite pinTop{"inv_mass", 0u, (uint32_t)nx, {0.0f, 0.0f, 0.0f}};
    Command cmd_pin_top{CommandTag::SetFieldRegion, &pinTop, sizeof(pinTop)};
    push_command(s, cmd_pin_top);

    // 自定义命令（未消费）
    int custom_payload = 42;
    Command cmd_custom{CommandTag::Custom, &custom_payload, sizeof(custom_payload)};
    push_command(s, cmd_custom);

    // 刷新（BeforeFrame 相位会下发小命令与可能的结构命令）
    flush_commands(s, ApplyPhase::BeforeFrame);

    // 结构命令：模拟“加边/删边/加点/删点”（payload 可为空，当前 rebuild 为占位）
    Command add_nodes{CommandTag::AddNodes, nullptr, 0};
    Command add_edges{CommandTag::AddRelations, nullptr, 0};
    Command rem_nodes{CommandTag::RemoveNodes, nullptr, 0};
    Command rem_edges{CommandTag::RemoveRelations, nullptr, 0};
    push_command(s, add_nodes);
    push_command(s, add_edges);

    // 帧循环
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 90; ++f) {
        if (f == 10) {
            // 在第 10 帧前触发一次结构变更（会走 rebuild + remap 占位路径）
            flush_commands(s, ApplyPhase::BeforeFrame);
        }
        step(s, dt);
        // AfterSolve 相位：当前 public API 没有把命令放到 after 队列的能力，这里空刷以覆盖调用
        flush_commands(s, ApplyPhase::AfterSolve);
        if (f % 30 == 0) {
            TelemetryFrame tf{};
            Status rc = telemetry_query_frame(s, &tf); (void)rc;
            std::printf("frame=%d telemetry: step_ms=%.3f cmds=%llu rebuilds=%llu\n",
                        f, tf.step_ms, (unsigned long long) tf.commands_applied, (unsigned long long) tf.structural_rebuilds);
        }
    }

    // 再触发一次结构变更命令并刷新
    push_command(s, rem_edges);
    push_command(s, rem_nodes);
    flush_commands(s, ApplyPhase::BeforeFrame);

    TelemetryFrame tf{};
    Status rc = telemetry_query_frame(s, &tf); (void)rc;
    std::printf("final telemetry: step_ms=%.3f cmds=%llu rebuilds=%llu\n",
                tf.step_ms, (unsigned long long) tf.commands_applied, (unsigned long long) tf.structural_rebuilds);

    destroy(s);

    std::printf("e_max done.\n");
    return 0;
}
