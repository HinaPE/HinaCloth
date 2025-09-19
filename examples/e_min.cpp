// examples/e_grid_cloth_annotated.cpp
// 本示例用尽可能直白的方式，展示 4+3 抽象如何被“壳层 API”消费：
// 4 = State / Parameters / Topology / Policy
// 3 = Space / Operators / Events
// 并通过 create/step/commands 这组统一入口交给引擎执行。

#include "api/sim.h"
#include <cstdint>
#include <cstdio>
#include <cstring> // 仅用于演示 strcmp 之类的检查
#include <vector>

// 生成一个 nx×ny 的长方形网格：
// - 顶点 positions: (x,z) 为网格坐标，y=0.5 作为初始高度
// - 速度 velocities: 初始全部为 0
// - 拓扑 edges: 横向+纵向的一阶距离约束边
static void make_grid(size_t nx, size_t ny, float dx,
    std::vector<float>& pos, // AoS: xyzxyz...
    std::vector<float>& vel, // AoS: xyzxyz...
    std::vector<uint32_t>& edges // 2-ary 索引对
) {
    size_t n = nx * ny;
    pos.resize(3 * n);
    vel.assign(3 * n, 0.0f);

    // 逐点写入 AoS 位置
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            size_t id       = j * nx + i;
            pos[3 * id + 0] = float(i) * dx; // x
            pos[3 * id + 1] = 0.5f; // y
            pos[3 * id + 2] = float(j) * dx; // z
        }
    }

    // 一阶网格边：横向
    edges.clear();
    edges.reserve((nx - 1) * ny + (ny - 1) * nx);
    for (size_t j = 0; j < ny; ++j) {
        for (size_t i = 0; i + 1 < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + 1;
            edges.push_back(a);
            edges.push_back(b);
        }
    }
    // 一阶网格边：纵向
    for (size_t j = 0; j + 1 < ny; ++j) {
        for (size_t i = 0; i < nx; ++i) {
            uint32_t a = uint32_t(j * nx + i);
            uint32_t b = a + uint32_t(nx);
            edges.push_back(a);
            edges.push_back(b);
        }
    }
}

int main() {
    using namespace sim;

    // 1) 组装 4 个核心输入维度中的 State/Topology/Parameters/Policy
    // State: 用最朴素的 AoS 向量传入，壳层会在 create() 中做 pack/翻译
    size_t nx = 16, ny = 16;
    float dx = 0.05f;
    std::vector<float> pos, vel;
    std::vector<uint32_t> edges;
    make_grid(nx, ny, dx, pos, vel, edges);

    // FieldView 用于把“外部 AoS 缓冲”描述给壳层：
    // name: 字段名，位置必须叫 "position"，速度叫 "velocity"
    // type: 本例全是 F32
    // data: 指向 AoS 起始
    // count: 顶点数
    // components: 3 表示 Vec3
    // stride_bytes: 每个顶点的步长（AoS 下为 3*sizeof(float)）
    FieldView fpos{"position", FieldType::F32, pos.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView fvel{"velocity", FieldType::F32, vel.data(), nx * ny, 3, sizeof(float) * 3};
    FieldView fields[2] = {fpos, fvel};
    StateInit st{fields, 2};

    // Topology: 只传入一类关系“edges”，arity=2，count 为边数，indices 为 2*count 的扁平索引
    RelationView rel{edges.data(), 2, edges.size() / 2, "edges"};
    TopologyIn topo{(uint32_t) (nx * ny), &rel, 1};

    // Parameters: 这里演示一个运行前参数 gravity_y。你也可以为空
    Param pg;
    pg.name      = "gravity_y";
    pg.type      = ParamType::F32;
    pg.value.f32 = -9.8f;
    Parameters params{&pg, 1};

    // Policy: 求解与执行策略
    // exec: 决定布局/后端/线程/确定性/遥测开关
    // solve: 子步数、迭代数、阻尼、时间积分器
    Policy pol{{DataLayout::Auto, Backend::Native, 1, true, false}, {1, 8, 0.0f, TimeStepper::Symplectic}};

    // 2) 3 个附加维度 Space/Operators/Events
    // Space: 拉格朗日空间
    SpaceDesc sp{SpaceType::Lagrangian, 1, 0};

    // Operators: 声明要参与的算子。本例只有 distance（基于 "edges" 关系）
    // fields: 声明该算子读写哪些字段（这里写 position）
    const char* tags[] = {"edges"};
    FieldUse uses[]    = {{"position", true}};
    OperatorDecl op{"distance", tags, 1, uses, 1, OpStage::Solve, true};
    OperatorsDecl ops{&op, 1};

    // Events: 初始帧事件脚本，本例为空
    EventsScript ev{nullptr, 0};

    // BuildDesc: 把 4+3 的输入打包交给 create
    BuildDesc bd{
        st, params, topo, pol, sp, ops, ev,
        ValidateLevel::Strict, // 严格校验
        {true, 64} // pack 选项：懒打包、块大小等（这里只是示例值）
    };

    // 3) 创建 Solver
    auto r = create(bd);
    if (r.status != Status::Ok) {
        std::printf("create failed\n");
        return 1;
    }
    Solver* s = r.value;

    // 4) 可选：查询底层真实选型（Chosen）
    {
        auto qc = query_chosen(s);
        if (qc.status == Status::Ok) {
            Chosen ch     = qc.value;
            const char* b = ch.backend == Backend::Native ? "Native" : (ch.backend == Backend::AVX2 ? "AVX2" : (ch.backend == Backend::TBB ? "TBB" : "GPU"));
            const char* l = ch.layout == DataLayout::SoA ? "SoA" : (ch.layout == DataLayout::AoS ? "AoS" : (ch.layout == DataLayout::Blocked ? "Blocked" : "Auto"));
            std::printf("chosen backend=%s layout=%s threads=%d\n", b, l, ch.threads);
        }
    }

    // 5) 演示一个运行期小命令：把重力从 -9.8 改为 -15.0
    // 说明：我们当前适配器的示例实现中，SetParam 命令的 payload 形式是：
    // struct Payload { const char* name; float value; };
    // Command.data 指向该结构，Command.bytes=sizeof(Payload)
    struct Payload {
        const char* name;
        float value;
    };
    Payload gpay;
    gpay.name  = "gravity_y";
    gpay.value = -15.0f;
    Command cmd_set_g{CommandTag::SetParam, &gpay, sizeof(gpay)};
    push_command(s, cmd_set_g);

    // 6) 帧循环
    // flush_commands(BeforeFrame) 会把小命令下发到引擎（修改 Overrides/Data），
    // 结构事件则在这里触发 rebuild+remap；然后 step(dt) 推进一帧
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 120; ++f) {
        flush_commands(s, ApplyPhase::BeforeFrame);
        step(s, dt);
        // AfterSolve 阶段也可以 flush，演示略
    }

    // 7) 遥测查询
    TelemetryFrame tf{};
    telemetry_query_frame(s, &tf);
    std::printf("telemetry: step_ms=%.3f cmds=%llu rebuilds=%llu\n", tf.step_ms, (unsigned long long) tf.commands_applied, (unsigned long long) tf.structural_rebuilds);

    // 8) 销毁 Solver
    destroy(s);

    // 9) 打印下 pos 的第一顶点，便于肉眼回归（注意：pos 是“外部 AoS 初始缓冲”，
    // 运行期引擎使用内部布局，不会回写到这个外部缓冲；这里仅演示接口可用）
    std::printf("example done. initial p0 = (%.3f, %.3f, %.3f)\n", pos[0], pos[1], pos[2]);
    return 0;
}
