# HinaCloth 开发者快速上手（仅覆盖 src/ 下实现）

本教程面向刚接手本项目的开发者，按“自顶向下、循序渐进”的方式，带你从最小可跑通的框架开始，一步步启用全部已经实现的功能（仅涉及 src/ 目录内的代码与接口）。

你将学会：如何用对外 API 组装 BuildDesc，创建求解器，注入命令，选择后端与布局，启用/配置算子，拉取遥测，拷贝位置做渲染，并了解各模块的职责与未完成项的路线图。

---

## 目录
- 1. 顶层模块导览（src/ 结构与职责）
- 2. 最小可运行框架：Hello, Solver
- 3. 命令系统与相位：BeforeFrame/AfterSolve
- 4. 固定点（Pin）与区域写入：inv_mass / SetFieldRegion
- 5. Attachment（目标拉拽）算子
- 6. XPBD Distance：全局/逐边 Compliance
- 7. Bending（可选）算子与 bend_pairs
- 8. 后端/布局/线程：Native/AVX2/TBB × SoA/Blocked
- 9. 遥测 Telemetry 与位置拷贝用于渲染
- 10. 能力枚举 enumerate_capabilities
- 11. 结构性重建（占位实现）与缓存
- 12. 常见问题与调试窍门
- 13. 尚未完成的功能与路线图

---

## 1. 顶层模块导览（src/ 结构与职责）

- api/
  - sim.h：对外 API。create/destroy/step/push_command/flush_commands/query_chosen/telemetry_query_frame/copy_positions
  - build.h：BuildDesc 聚合输入。含 state_in.h、parameters_in.h、topology_in.h、policy_in.h、space_in.h、operators_in.h、events_in.h
  - commands.h：命令枚举与载荷壳（不定形 payload）
  - capability.h：能力枚举（backend × layout 组合）
  - status.h / telemetry.h / version.h / ids.h：通用类型
- shell/
  - validators.cpp：输入校验（节点数、NaN、边越界、重复边等）
  - translators.cpp / packers.cpp：单位/字段归一化与打包（当前基本空实现）
  - cache_tracker.cpp：内容哈希与内存缓存（Model 级）
  - sim_create.cpp / sim_step.cpp / sim_commands.cpp / sim_readback.cpp：API 薄封装与命令相位应用
- adapter/
  - engine_adapter.*：把 shell 输入对接 engine（cooking → model，core/data → data，runtime → step），并封装 rebuild+remap
- cooking/
  - cooking.*：从 BuildDesc 生成只读 Model：
    - 取第一个二元关系为 edges；计算 rest-length
    - 可选识别 tag="bend_pairs" 的四元关系并计算弯曲目标角
    - BFS islanding，按岛重排 edges/rest 并记录 offsets
- core/
  - model/：Model 只读结构
  - data/：Data 可变状态与 overrides（inv_mass、lambda、attachment、solve 参数等）；remap
  - common/utils.h：向量工具
- backend/
  - storage/：SoA 视图与 AoSoA（blocked）打包/解包
  - kernel/constraints/：distance（标量/AVX2/AoSoA）、attachment、bending
  - registry/：后端选择与能力枚举；CPU AVX2 检测
  - scheduler/：seq 已内联在 runtime 使用；tbb.cpp 为 TODO（并行在 runtime 中直接用 tbb::parallel_for）
- runtime/
  - step.*：帧编排（子步→预测→PreSolve 附着→距离投影→可选弯曲→更新速度），采样 Telemetry
  - phases.* / apply_events.*：占位

---

## 2. 最小可运行框架：Hello, Solver

目标：以最少输入跑通一帧。仅需 position（Vec3，AoS 格式）、edges（二元关系）。

契约小抄：
- 必填 StateInit.field: name="position"（或别名："pos"/"positions"），components=3，stride_bytes=sizeof(float)*3 或更大
- 可选 StateInit.field: name="velocity"（Vec3）
- 拓扑 TopologyIn：第一个 RelationView 作为边列表（arity=2），tag 建议 "edges"
- 参数 Parameters：可为空
- Policy.solve：substeps>=1，iterations>=0；Policy.exec 可 Auto

示例片段（C++，仅引用 src/api/*）：

```cpp
#include "src/api/sim.h"
#include "src/api/build.h"
#include <vector>
using namespace sim;

struct Float3 { float x,y,z; };

int main(){
    // 2 nodes, one edge (0-1)
    std::vector<Float3> positions = { {0,0,0}, {1,0,0} };
    std::vector<uint32_t> edges   = { 0,1 };

    FieldView fields[1] = {
        { "position", FieldType::F32, positions.data(), positions.size(), 3, sizeof(Float3) }
    };
    StateInit state{ fields, 1 };

    RelationView relations[1] = { { edges.data(), 2, 1, "edges" } };
    TopologyIn topo{ (uint32_t)positions.size(), relations, 1 };

    // 默认策略（Auto）
    Policy policy{}; policy.exec.layout = DataLayout::Auto; policy.exec.backend = Backend::Auto;
    policy.exec.threads = -1; policy.exec.deterministic = false; policy.exec.telemetry = true;
    policy.solve.substeps = 1; policy.solve.iterations = 8; policy.solve.damping = 0.0f; policy.solve.stepper = TimeStepper::Symplectic;

    BuildDesc desc{}; desc.state = state; desc.topo = topo; desc.policy = policy; desc.validate = ValidateLevel::Strict;

    auto r = create(desc);
    if (r.status != Status::Ok) return -1;
    Solver* s = r.value;

    // 运行一步
    step(s, 1.0f/60.0f);

    // 查询遥测
    TelemetryFrame tf{}; telemetry_query_frame(s, &tf);

    // 提取位置（interleaved float3: x,y,z）
    std::vector<float> out(positions.size()*3);
    size_t count = 0; copy_positions(s, out.data(), 0, &count);

    destroy(s);
    return 0;
}
```

要点：
- 校验在 shell/validators.cpp 完成，Strict 模式会检查 NaN、边越界与重复边（重复边在严格模式下视为错误）。
- cooking 会计算每条边的 rest-length，并按连通分量重排（island_offsets）。

---

## 3. 命令系统与相位：BeforeFrame/AfterSolve

命令通过 push_command 排队，再用 flush_commands 按相位应用：
- BeforeFrame：在本帧 simulation 之前应用。会把“结构性”命令拆出来触发 rebuild（当前为占位实现，详见第 11 章）。
- AfterSolve：在本帧求解后应用。

已实现的“小参数/覆盖”命令（engine_apply_small_params → core_data_apply_overrides）：
- SetParam: name/value 二元载荷（gravity_x/y/z、distance_compliance、iterations、substeps、damping）
- EnableOperator / DisableOperator: op = "attachment" / "bending"
- SetFieldRegion: 对区间写入 inv_mass、attach_w、attach_target、distance_compliance_edge

辅助函数（构造命令载荷）：

```cpp
static Command make_set_param(const char* name, float v){
    struct Payload { const char* n; float v; } p{ name, v };
    return Command{ CommandTag::SetParam, &p, sizeof(p) };
}
static Command make_enable(const char* op){ return Command{ CommandTag::EnableOperator, &op, sizeof(op) }; }
static Command make_disable(const char* op){ return Command{ CommandTag::DisableOperator, &op, sizeof(op) }; }
static Command make_set_region(const char* field, uint32_t start, uint32_t count, float x, float y=0, float z=0){
    struct Payload { const char* f; uint32_t s; uint32_t c; float v[3]; } p{ field, start, count, {x,y,z} };
    return Command{ CommandTag::SetFieldRegion, &p, sizeof(p) };
}
```

示例：把重力改为 -20 m/s^2 并将迭代数设置为 12：

```cpp
push_command(s, make_set_param("gravity_y", -20.0f));
push_command(s, make_set_param("iterations", 12));
flush_commands(s, ApplyPhase::BeforeFrame);
step(s, 1.0f/60.0f);
```

---

## 4. 固定点（Pin）与区域写入：inv_mass / SetFieldRegion

Data.inv_mass[i] == 0 表示该节点被固定（不可动）。可通过 SetFieldRegion 批量写入：

```cpp
// 固定前 10 个点
push_command(s, make_set_region("inv_mass", 0, 10, 0.0f));
flush_commands(s, ApplyPhase::BeforeFrame);
step(s, dt);
```

提示：inv_mass 非 0 的点会被积分与投影更新；固定点在 integrate 与 finalize 阶段都会维持位置与零速度。

---

## 5. Attachment（目标拉拽）算子

Attachment 在 PreSolve 阶段把预测位置往目标位置（tx,ty,tz）拉，权重 attach_w ∈ [0,1]：

启用与配置：

```cpp
push_command(s, make_enable("attachment"));
// 全节点应用，权重 0.2，目标为抬高 0.5m
push_command(s, make_set_region("attach_w", 0, node_count, 0.2f));
push_command(s, make_set_region("attach_target", 0, node_count, 0.0f, 0.5f, 0.0f));
flush_commands(s, ApplyPhase::BeforeFrame);
step(s, dt);
```

实现对应：backend/kernel/constraints/attachment.*，在 runtime/step.cpp 的 presolve_apply_attachment 中调用。

---

## 6. XPBD Distance：全局/逐边 Compliance

Distance 约束实现为 XPBD 形式（带 λ 累积与 α=compliance/dt^2）：
- 全局 compliance：Parameters 或 SetParam("distance_compliance", value)
- 逐边 compliance：SetFieldRegion("distance_compliance_edge", start, count, value)

示例：设置全局柔度 1e-6，并对前 100 条边更软：

```cpp
push_command(s, make_set_param("distance_compliance", 1e-6f));
push_command(s, make_set_region("distance_compliance_edge", 0, 100, 5e-6f));
flush_commands(s, ApplyPhase::BeforeFrame);
step(s, dt);
```

实现对应：
- backend/kernel/constraints/distance.*（标量/AVX2）
- backend/kernel/constraints/distance_aosoa.*（Blocked 布局）
- runtime/step.cpp 中 prepare_alpha_edge 负责 per-edge α 计算；迭代过程中使用 λ 缓冲（Data.lambda_edge）。

---

## 7. Bending（可选）算子与 bend_pairs

如果拓扑传入 tag="bend_pairs" 的四元关系（每条 [i0,i1,i2,i3]），cooking 会预计算弯曲目标角；在运行时启用 "bending" 算子即可调用标量核（简化版）：

```cpp
// 构造 bend_pairs（例如一个四元组构成两三角共享边 i0-i1 的铰链）
std::vector<uint32_t> bend_pairs = { i0, i1, i2, i3 };
RelationView rels[2] = {
  { edges.data(), 2, edge_count, "edges" },
  { bend_pairs.data(), 4, 1, "bend_pairs" }
};
TopologyIn topo{ node_count, rels, 2 };

// 创建 solver 后启用 bending
push_command(s, make_enable("bending"));
flush_commands(s, ApplyPhase::BeforeFrame);
step(s, dt);
```

实现对应：cooking/cooking.cpp 预计算 bend_rest_angle；runtime/step.cpp 的 bending_pass 调用 backend/kernel/constraints/bending.*（简化的迭代投影）。

---

## 8. 后端/布局/线程：Native/AVX2/TBB × SoA/Blocked

后端选择在创建时完成，实际选择结果可 query：
- Backend::Auto：若编译期具备 AVX2（HINACLOTH_HAVE_AVX2），优先选择 AVX2；否则 Native
- DataLayout::Auto：若后端为 AVX2，默认 Blocked（AoSoA）；否则 SoA
- TBB（如果编译为 HINACLOTH_HAVE_TBB 且 Backend::TBB）：runtime 在按 island 并行，使用 tbb::parallel_for（并非 scheduler/tbb.cpp）

设置策略并查询：

```cpp
Policy p{}; p.exec.backend = Backend::Auto; p.exec.layout = DataLayout::Auto; p.exec.threads = -1; // -1 表示自动
BuildDesc d{}; d.policy = p; // 省略其余赋值
auto r = create(d); Solver* s = r.value;
auto chosen = query_chosen(s);
// chosen.value.backend/layout/threads 给出实际选择
```

提示：
- Blocked 布局的块大小来源：BuildDesc.pack.block_size 或 Model.layout_block_size（默认 8）。
- AoSoA 会在子步前后进行 SoA<->AoSoA 打包/解包（backend/storage/aosoa.*）。

---

## 9. 遥测 Telemetry 与位置拷贝用于渲染

每帧 step 后可读取 TelemetryFrame：
- step_ms：总帧耗时
- residual_avg：距离约束的平均绝对残差 |len - rest|
- last_rebuild_ms / avg_rebuild_ms：最近/平均重建耗时（通过 flush 的结构性命令触发）
- commands_applied / structural_rebuilds：累计计数
- solve_substeps / solve_iterations：实际使用的子步/迭代次数

位置拷贝：copy_positions 会把当前世界空间 positions（x,y,z interleaved）拷到外部缓冲（可直接喂渲染）。

---

## 10. 能力枚举 enumerate_capabilities

在创建前可枚举构建时支持的组合（不依赖具体 Model）：

```cpp
#include "src/api/capability.h"
std::vector<Capability> caps(16);
size_t n = enumerate_capabilities(caps.data(), caps.size());
for(size_t i=0;i<n;++i){ /* caps[i].backend, caps[i].layout, caps[i].name */ }
```

---

## 11. 结构性重建（占位实现）与缓存

- 结构性命令（AddNodes/RemoveNodes/AddRelations/RemoveRelations）会被识别为“结构性”，在 flush 时触发 rebuild+remap 流程：
  - shell/sim_commands.cpp 拆分 small vs structural，并测量 rebuild 耗时写入 Telemetry
  - adapter/engine_adapter.cpp 调用 cooking_rebuild_model_from_commands（当前未真正应用载荷，仅复制当前 Model）→ core_data_apply_remap（恒等 remap，但会重置/调整部分缓存，如 λ 大小）
- 缓存：shell/cache_tracker.cpp 根据 BuildDesc 内容计算 hash，并用内存 map 复用已烹制的 Model（同一进程内有效）。

因此，目前你可以用“空载荷”的结构性命令触发重建路径与遥测统计，但不会更改拓扑：

```cpp
Command dummy{ CommandTag::AddRelations, nullptr, 0 };
push_command(s, dummy);
flush_commands(s, ApplyPhase::BeforeFrame);
TelemetryFrame tf{}; telemetry_query_frame(s, &tf); // tf.last_rebuild_ms > 0
```

---

## 12. 常见问题与调试窍门

- 校验失败（create 返回 ValidationFailed）：
  - 检查 FieldView：name、components=3、stride_bytes 是否正确；position.count 必须等于 node_count
  - 边越界或重复边（Strict 模式下）：修正 relation 数据或使用 ValidateLevel::Tolerant 观察效果
- NaN 传播：validators 对 position/velocity 做 NaN 检测；运行时仍应避免在命令中写入 NaN 值
- TBB 并行无效：需确认编译时定义 HINACLOTH_HAVE_TBB 且 Backend::TBB 或 Policy.exec.backend==TBB；runtime/step.cpp 才会走 tbb::parallel_for
- AVX2 未启用：需在编译器/平台支持 AVX2 并定义 HINACLOTH_HAVE_AVX2
- 位置读回：copy_positions 会截断到 maxCount（传 0 表示全部）；目标缓冲大小需 ≥ 3*outCount 浮点

---

## 13. 尚未完成的功能与路线图

已实现并在本教程覆盖：
- Shell：严格校验、基本翻译/打包占位、内容哈希与内存缓存
- Cooking：rest-length、bend_pairs 目标角、islanding 与 edges/rest 重排、layout_block_size
- Core Data：positions/velocities/predicted、inv_mass、λ 缓冲、全局/逐边 distance compliance、attachment 数据、solve/exec 选项、AoSoA 缓冲
- Backend Storage：SoA 视图、AoSoA 打包/解包
- Kernels：distance（标量/AVX2/AoSoA）、attachment（PreSolve）、bending（简化）
- Runtime：子步、预测、attachment→distance→bending→finalize、SoA/AoSoA 分支、AVX2 路径、TBB island 并行（在 runtime 直接 parallel_for）、Telemetry 采样
- Registry：Auto 选择（AVX2/Native，布局 SoA/Blocked），capability 枚举
- API：create/destroy/step/push_command/flush/query_chosen/telemetry_query_frame/copy_positions

尚未完成/差距（与 README 蓝图比）：
- PhaseGraph/颜色/批次：依据 Operators 的读写集构建阶段与批；当前 runtime 直接串接固定顺序
- 真正的结构性事件应用：Add/RemoveNodes/Relations 等未在 cooking_rebuild_model_from_commands 中生效
- Remap 强化：节点增删/边变化时的状态迁移策略（λ 清理、速度/附件状态迁移）
- Scheduler/tbb：backend/scheduler/tbb.cpp 仍为 TODO；并行逻辑暂在 runtime 内部直接使用 tbb::parallel_for
- 确定性等级：固定任务划分/稳定归约树/跨线程位级一致的策略未落地（flag 仅占位）
- 更完整的 validators/translators/packers：单位/坐标归一、别名策略、AoS→中立输入的强健打包
- LayoutPlan/AoSoA 进阶：重排索引、字段顺序、对齐与预取策略
- Telemetry 细化：分相位/分批耗时、残差曲线、岛屿规模统计
- 能力/注册表扩展：结合硬件能力与模型规模更智能地选 backend×layout 组合，GPU 后端
- 更多算子：Area/Shear/SelfCollision、Attachment/Bending 的 XPBD 合规实现（含 per-constraint λ/α）
- 持久缓存：Model/Precompute 的磁盘缓存与版本化

建议路线图（迭代里程碑）：
1) 正确性优先（XPBD 完整化）：
   - 强化 distance/bending/attachment 为一致的 XPBD 形式（α/λ 一致），完善 per-constraint 状态
   - 残差/收敛统计加入 Telemetry
2) 调度与并行：
   - PhaseGraph/颜色/批次与确定性分块
   - 抽离 runtime 并行到 scheduler/，实现 tbb.cpp，支持 island→batch 二级调度
3) 布局与载入：
   - LayoutPlan（SoA/AoSoA 重排索引、块大小、对齐），Registry 根据 Policy 与 CPU 特性选择
4) 结构事件：
   - cooking_rebuild_model_from_commands 真正应用拓扑变更；Remap 迁移矩阵完善
5) 可观测与缓存：
   - Telemetry 分相位采样；Model/Precompute 的磁盘缓存与版本策略
6) 扩展算子与 GPU：
   - Area/Shear/SelfCollision；探索 GPU 后端与 AoSoA/GPU 友好打包

---

附：构建与运行（可选）

如需本地快速尝试（Windows cmd）：

```cmd
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

本教程中的代码示例仅依赖 src/api/* 头文件，便于嵌入你的宿主应用；如需更完整示例，可参考仓库 examples/（不在本教程覆盖范围）。

