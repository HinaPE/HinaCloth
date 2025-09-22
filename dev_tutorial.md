# HinaCloth 开发者指南（基于当前源码的实证版）

本指南面向需要二次开发或集成引擎的工程师，基于对当前源代码的完整走查，给出“能跑、可调、可扩”的最短路径与注意事项。文中所有 API/字段/行为均以仓库现状为准。

---

## 0. 一句话概览（你将得到什么）
- 对外极简 API：create/destroy/step/push_command/flush/query_chosen/telemetry_query_frame/copy_positions。
- MVP 的 XPBD 布料求解：Distance 约束（支持 λ 累积与 α=compliance/dt²）、可选 Attachment 与简化 Bending。
- 多执行路径：Native 标量、AVX2（gather）、Blocked AoSoA（可与 AVX2 配合），TBB 按岛并行。
- 模型烹制（Cooking）：从 edges 计算 rest-length，按连通分量重排为 islands；bend_pairs 可选。
- 运行期命令：全局与分区覆写（gravity/iterations/substeps/damping、inv_mass、per-edge compliance、attachment 目标）。
- 遥测：每帧 step_ms、平均距离残差、命令/重建计数与 solve 配置回报。

你暂时还得不到：PhaseGraph/颜色/批次的完整并行规划、真正的结构性事件应用（rebuild 现为占位逻辑）、严格确定性调度与细粒度遥测分项。

---

## 1. 目录和职责（src/）
- api/：对外头文件（sim.h、build.h、commands.h、policy_in.h、telemetry.h、capability.h、chosen.h、…）。
- shell/：
  - validators.cpp：输入校验（position/velocity 结构、NaN 检查；edges 越界/重复检查）。
  - translators.cpp / packers.cpp：轻量归一化（线程数）与占位打包。
  - cache_tracker.cpp：内容哈希与进程内 Model 缓存（命中则跳过烹制）。
  - sim_*.cpp：API 薄封装、命令分流（BeforeFrame/AfterSolve）、遥测聚合。
- cooking/：cooking.cpp/h
  - 从 State/Topology 生成 Model：edges、rest-length、islanding 与重排、可选 bend_pairs 与弯曲目标角。
  - rebuild 占位：复制现有 Model + 恒等 RemapPlan。
- core/
  - model/：只读 Model（node_count、edges、rest、island_offsets、node_remap、layout_block_size、bend_pairs）。
  - data/：时变 Data（x/y/z、v、p、inv_mass、λ_edge、global/per-edge compliance、AoSoA 缓冲、attachment 数据、solve/exec 选项）；remap。
- backend/
  - storage/：SoA 视图、AoSoA 打包/解包与块访问工具。
  - kernel/constraints/：distance（标量/AVX2/AoSoA）、attachment、bending（简化）。
  - registry/：根据 Policy.exec 与 CPU 能力选择 backend/layout/threads。
- runtime/
  - step.*：帧编排，集成/TBB 并行/约束投影/阻尼/遥测回报。

---

## 2. 快速上手（最小可运行）
输入契约（必须项）：
- StateInit：position（或别名 pos/positions），Vec3 AoS（components=3），count==node_count。
- TopologyIn：第一个 RelationView 作为边列表（arity=2，tag 建议为 "edges"）。

可选：
- velocity（Vec3）；bend_pairs（arity=4，tag="bend_pairs"）。
- Policy/Parameters 可使用默认。

示例（仅依赖 src/api/*）：

```cpp
#include "src/api/sim.h"
#include "src/api/build.h"
#include <vector>
using namespace sim;
struct Float3 { float x,y,z; };
int main(){
  std::vector<Float3> pos = {{0,0,0},{1,0,0}};
  std::vector<uint32_t> edges = {0,1};
  FieldView fields[] = {{"position", FieldType::F32, pos.data(), pos.size(), 3, sizeof(Float3)}};
  StateInit state{fields, 1};
  RelationView rels[] = {{edges.data(), 2, 1, "edges"}};
  TopologyIn topo{(uint32_t)pos.size(), rels, 1};
  Policy policy{}; // 默认：Auto 选择 backend/layout，solve 迭代/子步可为 8/1
  BuildDesc desc{}; desc.state=state; desc.topo=topo; desc.policy=policy; desc.validate=ValidateLevel::Strict;
  auto r = create(desc); if(r.status!=Status::Ok) return -1; Solver* s=r.value;
  step(s, 1.0f/60.0f);
  TelemetryFrame tf{}; telemetry_query_frame(s, &tf);
  std::vector<float> out(pos.size()*3); size_t n=0; copy_positions(s, out.data(), 0, &n);
  destroy(s); return 0; }
```

验证器会检查：
- position/velocity 的 components、count、stride（Strict 模式也会拒绝 strides 小于 components*sizeof(float)）。
- position/velocity 含 NaN → 拒绝。
- edges 越界与重复（Strict 模式重复视为错误）。

---

## 3. 运行时命令与相位（BeforeFrame/AfterSolve）
通过 push_command → flush_commands 应用。BeforeFrame 会分流：
- 小参数覆写（立即生效，不触发重建）：
  - SetParam：gravity_x/y/z、distance_compliance、iterations、substeps、damping。
  - EnableOperator/DisableOperator："attachment"、"bending"。
  - SetFieldRegion：
    - "inv_mass"（节点范围写入，0 表示固定点）
    - "attach_w"、"attach_target"（目标拉拽）
    - "distance_compliance_edge"（逐边柔度）
- 结构性命令（当前触发重建但不改变拓扑，见第 9 章）：Add/RemoveNodes、Add/RemoveRelations。

辅助构造函数（示例写法）：
```cpp
static Command set_param(const char* name, float v){ struct P{const char* n; float v;} p{name,v}; return {CommandTag::SetParam,&p,sizeof(p)}; }
static Command enable(const char* op){ return {CommandTag::EnableOperator,&op,sizeof(op)}; }
static Command set_region(const char* field,uint32_t s,uint32_t c,float x,float y=0,float z=0){ struct P{const char* f;uint32_t s,c;float v[3];} p{field,s,c,{x,y,z}}; return {CommandTag::SetFieldRegion,&p,sizeof(p)}; }
```

---

## 4. XPBD Distance（带 λ 累积与 α=compliance/dt²）
- 全局柔度：SetParam("distance_compliance", value)。
- 逐边柔度：SetFieldRegion("distance_compliance_edge", start, count, value)。
- 引擎在每个子步前计算 per-edge α（α=edge_or_global_compliance / dt_sub²）。
- 约束核：
  - 标量 SoA：`kernel_distance_project`（支持 inv_mass、λ_edge、alpha_edge）。
  - AVX2 SoA gather：`kernel_distance_project_avx2`（编译期 HINACLOTH_HAVE_AVX2 + 运行期 CPU 支持）。
  - AoSoA Blocked：`kernel_distance_project_aosoa`（块内线性、跨块 gather）。
- 岛并行：按 `Model.island_offsets` 将边分块；若开启 TBB，则对岛粒度 `parallel_for`。

---

## 5. Attachment（目标拉拽）与固定点（inv_mass=0）
- EnableOperator("attachment") 后生效。
- SetFieldRegion("attach_w") 写入权重 [0,1]；SetFieldRegion("attach_target") 写入目标位置。
- 核：`kernel_attachment_apply` 在预估位（px,py,pz）上直接插值；inv_mass==0 的点跳过。
- 固定点：SetFieldRegion("inv_mass", i, count, 0.0f)。固定点在积分与 finalize 均保持位置、速度置 0。

---

## 6. Bending（简化版）
- 可选输入 bend_pairs（arity=4，tag="bend_pairs"）。Cooking 计算 `bend_rest_angle`。
- EnableOperator("bending") 后，`kernel_bending_project` 以简单刚度 k=0.1 迭代拉回目标角（非完整 XPBD 形式）。

---

## 7. 执行策略：后端 × 布局 × 线程
- 选择逻辑（backend/registry）：
  - Backend::Auto → 若编译期支持 AVX2 且 CPU 检测到 AVX2，则选 AVX2，否则 Native。
  - DataLayout::Auto → 若 Backend==AVX2，默认 Blocked（AoSoA），否则 SoA。
  - 线程：threads==0 视为自动（-1）。
- 运行期：
  - `Data.exec_use_avx2 / exec_layout_blocked / exec_use_tbb` 由选择结果驱动。
  - Blocked 路径在每个子步进行 SoA ↔ AoSoA 打包/解包（storage/aosoa.*）。
  - TBB 并行当前在 runtime/step.cpp 内部按 island 粒度调用 `tbb::parallel_for`（无独立 scheduler/tbb.cpp）。

编译开关：
- AVX2：定义 HINACLOTH_HAVE_AVX2 并使用支持 AVX2 的编译器/平台。
- TBB：定义 HINACLOTH_HAVE_TBB 并正确链接 oneTBB。

---

## 8. 帧编排与遥测
- 每帧：
  1) integrate_pred：重力积分到预测位置（px,py,pz）。
  2) presolve：若启用 Attachment，按目标拉拽预测位置。
  3) prepare_alpha_edge：按 dt_sub 计算 α。
  4) project distance（SoA/AVX2/AoSoA）+ 可选 bending。
  5) finalize：根据 (p - x)/dt 更新速度，应用阻尼（solve.damping∈[0,1]），提交位置。
- 子步/迭代：由 `Policy.solve` 或运行期覆写决定（SolveOverrides）。
- TelemetryFrame：`step_ms`、`residual_avg`（平均 |len-rest|）、`solve_substeps`、`solve_iterations`，以及 `commands_applied/structural_rebuilds/last_rebuild_ms/avg_rebuild_ms`。
- 读回位置：`copy_positions`（interleaved float3）。

---

## 9. Cooking 与结构性重建（占位实现）
- 初次构建：
  - 从 edges 计算 rest-length，BFS 求连通分量；按岛重排 edges/rest，输出 `island_offsets`。
  - 可选 bend_pairs → 计算 `bend_rest_angle`。
  - `node_remap` 当前为恒等映射；`layout_block_size` 默认 8 或由 BuildDesc.pack.block_size 指定。
- 结构命令触发 rebuild：
  - 当前 cooking_rebuild_model_from_commands 仅复制 Model，不实际应用拓扑变更；RemapPlan 也为恒等。
  - Remap 后会重置/调整 Data 中与拓扑相关的缓存（λ_edge、per-edge compliance/alpha 等）。
  - 遥测记录本次重建耗时（`last_rebuild_ms`），并更新平均值。

---

## 10. 能力枚举与实际选择
- `enumerate_capabilities`（api/capability.h）：枚举编译期支持的 backend×layout 组合与名字。
- `query_chosen`：返回实际运行时选择的 backend/layout/threads。

---

## 11. 校验与容错要点
- Strict 模式：
  - position/velocity 分量与数量必须匹配 node_count；stride 不得小于分量 * sizeof(float)。
  - edges 越界或重复视为错误；字段含 NaN 视为错误。
- Tolerant 模式：
  - 部分异常（如 stride 偏小、重复边）放宽；未知关系 tag 被忽略。

---

## 12. 常见问题与排查
- 编译无 AVX2 路径：未定义 HINACLOTH_HAVE_AVX2 或 CPU 不支持 AVX2 → 回退到标量路径。
- TBB 未生效：需定义 HINACLOTH_HAVE_TBB 并正确链接 oneTBB；Policy.exec.backend 设置为 TBB 或 Auto 且 Data.exec_use_tbb==true 才会在 runtime 中并行。
- 位置读回长度：`copy_positions` 的 maxCount==0 表示输出全部；目标缓冲长度需 ≥ 3*outCount。
- 残差异常大：检查 rest-length 是否来自初始化位置；确认 dt/substeps/compliance 配置；避免写入 NaN。

---

## 13. 现状与路线（简述）
- 已完成：
  - 验证器（含 NaN/越界/重复边检测）、模型烹制（islanding + rest）、XPBD Distance（λ/α/逐边柔度）、AoSoA/AVX2、Attachment、简化 Bending、TBB 岛并行、进程内 Model 缓存、基本遥测与读回。
- 待完善：
  - PhaseGraph/颜色/批次与确定性分块；scheduler/tbb 抽离；精细遥测；真正的结构事件与 Remap 策略；更强壮的打包/翻译；bending 的 XPBD 化；持久化缓存与版本化。

---

## 14. 构建与运行（Windows cmd）
以下命令基于当前仓库的 CMake 配置；若本地已有 `cmake-build-release` 等目录，可直接用 IDE 运行内置示例。

```cmd
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

可执行产物见构建目录（例如 cmake-build-release/）中的示例与测试二进制。

---

## 15. 附：数据/结构速查（面向开发者）
- Model：
  - node_count、edges(2*m)、rest(m)、island_offsets(island_count+1)、bend_pairs(4*k)、bend_rest_angle(k)、layout_block_size。
- Data（每帧变动）：
  - x/y/z、vx/vy/vz、px/py/pz、inv_mass(n)、lambda_edge(m)、distance_compliance（全局）/distance_compliance_edge(m)/distance_alpha_edge(m)、solve_substeps/iterations/damping、exec_use_avx2/exec_use_tbb/exec_layout_blocked、layout_block_size、pos_aosoa、attachment（w/tx/ty/tz）。

以上即为以源码为依据的“可落地”开发指南。建议从最小案例开始，逐步引入命令与算子，再切换后端/布局以对比性能与行为。
