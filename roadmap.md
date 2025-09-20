# HinaCloth Roadmap (2025-09-20, fully rewritten)

本路线图完全对齐 README 的“4+3 + 两圈外环”设计思想，并据当前 src 代码实现现状确定近期与中期落地优先级。同时，我们提供全新的渐进式示例套件（examples/ex00..ex06）作为伴随式教学，用例覆盖从最小可用到布局/后端/并行/遥测与基准。

—

## 渐进式 Examples 课程（新）

说明：所有示例均通过对外稳定 API（include/sim/*）构建 4+3 输入后调用 create/step/commands/telemetry。每个示例互相独立、几十行起步；编译目标名与文件对照如下：

- ex00 Minimal: example_ex00_minimal
  - 目标：最小网格 + 距离约束，展示 4+3 的最小组合：State/Topology/Parameters/Policy + Space/Operators。
  - 要点：AoS 外部缓冲通过 FieldView 传入；只用 distance 约束；查询 Telemetry。
- ex01 Policy & Chosen: example_ex01_policy
  - 目标：演示 Policy.exec/solve 的配置与 query_chosen，理解“Auto/Blocked/AVX2/TBB”等选择。
- ex02 Runtime Commands: example_ex02_commands
  - 目标：运行期小命令队列（BeforeFrame）——SetParam 覆盖求解器参数，SetFieldRegion 进行 pin（inv_mass=0）。
- ex03 Attachment: example_ex03_attachment
  - 目标：启用 attachment 算子（EnableOperator），设置 attach_w/attach_target 拉拽一列顶点。
- ex04 Bending (实验性): example_ex04_bending
  - 目标：提供 bend_pairs 关系与启用 bending（当前实现为 demo 级 PBD 投影）。
- ex05 Blocked/AVX2: example_ex05_blocked_avx2
  - 目标：强制 Blocked（AoSoA）布局，演示 pack.block_size 与后端选择，观察遥测表现。
- ex06 Bench CLI: example_ex06_bench
  - 目标：命令行性能基准（CSV 输出），可扫不同 backend/layout/尺寸组合（结合 capability 枚举）。

以上示例已替换旧的 e_* 示例，目录中仅保留 ex00..ex06 源文件（legacy e_* 文件已移除）。

—

## 现状快照（MVP 已具备）

- API/Shell：create/destroy/step/push_command/flush/query_chosen/telemetry 全链路可用；validators/packers/translator 提供基础能力；内存级模型缓存。
- Cooking/Model：edges→rest 预计算；连通岛屿划分与按岛重排；bend_pairs 支持；layout_block_size 占位。
- Data：SoA x/y/z, v/px；inv_mass；per-edge lambda；solve/exec 覆盖；AoSoA 缓冲；attachment/bending 使能与数据列。
- Backends：distance（SoA 标量/AVX2、AoSoA）、attachment、bending(demo)；seq 调度，按岛并行（TBB 宏可用时启用 tbb::parallel_for）。
- Runtime：积分→attachment 预解→distance（可 AVX2/AoSoA）→bending→速度/阻尼；Telemetry 输出 step_ms/residual 等。

—

## 缺口（与 README 蓝图相比）

- PhaseGraph/批次/确定性顺序未落地；Scheduler/TBB 缺批内固定分块与 barrier 管理。
- Rebuild/Events 仅占位；Remap 策略简化；LayoutPlan 与 AoSoA 常驻路径缺失。
- Validators/Translators/Packers 需补齐严格模式与单位/别名归一化；模型缓存缺落盘与版本。
- Telemetry 缺分相位/分批耗时与岛屿统计；确定性档位（L1/L2）待实现。

—

## 分阶段路线图（执行计划）

A. XPBD 正确性与可调参（近期）
- 完整 per-edge compliance 覆盖（已部分实现：distance_alpha_edge 按子步计算）。
- Bending 收敛为稳健 PBD 版本（标注实验性）。
- Validators 增强（NaN/越界/重复边）与宽松/严格两档；Telemetry 完整记录 iterations/substeps。

B. PhaseGraph + 批次 + 确定性顺序（中期）
- 按 Operators 读/写集与关系染色生成相位与颜色批，形成稳定排序键与固定批粒度（默认 64）。
- Runtime 采用 island→phase→batch 的统一执行计划（先 seq 基线）。

C. TBB 调度与确定性 Level 1（中期）
- 两级并行（island 粗粒度 + batch 细粒度）；固定任务划分与稳定分配；结果逐位或阈值内一致。
- Telemetry 增 per-phase 耗时与岛屿规模直方图。

D. LayoutPlan + AoSoA 常驻 + AVX2 强化（中期/性能）
- Cooking 生成 node_remap 与 block 对齐计划；Data/Storage 提供 AoSoA 常驻视图，消除子步 pack/unpack 往返；Kernel AVX2 修整。

E. 事件重建与 Remap 稳健化（中期/工程）
- 支持 Add/RemoveNodes/Relations、启停算子、布局策略变更；Remap 状态迁移矩阵；Telemetry 记录重建耗时与次数。

F. 工程化（缓存落盘/版本/能力/测试）（后续）
- 模型缓存落盘 + 版本；capability 报告向量宽度/对齐/线程上限；单测/基准与本地脚本。

—

## 里程碑与验收

- 基线（本周）：ex00..ex05 可构建运行；ex06 输出 CSV；Release + Debug 构建通过；AVX2/TBB 宏开关下均可编译。
- 正确性（阶段 A）：残差随迭代下降；pin/attachment 行为正确；Telemetry 字段正确。
- 并行（阶段 C）：TBB 多线程下速度提升且保持确定性 Level 1（固定分块/排序/归约）。
- 性能（阶段 D）：AoSoA+AVX2 对中等规模（>50k 约束）较 SoA 标量达到 >2×。

—

## 开发任务 → 代码映射

- PhaseGraph/批次：src/runtime/phases.*、src/cooking/*（图/颜色/批次→Model）。
- Scheduler.TBB：src/backend/scheduler/tbb.*；Runtime 整合：src/runtime/step.cpp。
- LayoutPlan/AoSoA：src/cooking/cooking.*、src/backend/storage/*、src/backend/kernel/*。
- 事件/Remap：src/runtime/apply_events.*、src/cooking/cooking.*、src/core/data/*。
- 校验/打包/缓存：src/shell/*（validators/translators/packers/cache_tracker）。
- 能力/注册：src/backend/registry/*。
- 遥测：src/api/telemetry.h + src/runtime/step.cpp。

—

## 构建与运行（PowerShell，目录 build-copilot）

- 配置与构建（Release）：

```powershell
cmake -S . -B build-copilot -DCMAKE_BUILD_TYPE=Release -DHINACLOTH_BUILD_EXAMPLES=ON
cmake --build build-copilot --config Release --target example_ex00_minimal
```

- 运行示例：

```powershell
# ex00 最小示例
./build-copilot/Release/example_ex00_minimal.exe

# ex01 policy 查询
cmake --build build-copilot --config Release --target example_ex01_policy
./build-copilot/Release/example_ex01_policy.exe

# ex06 基准（可选参数：--backend=avx2|native|tbb --layout=blocked|soa --nx=64 --ny=64 --subs=2 --iters=10 --sweep）
cmake --build build-copilot --config Release --target example_ex06_bench
./build-copilot/Release/example_ex06_bench.exe --backend=auto --layout=auto --nx=64 --ny=64 --frames=120
```

注意：使用 Visual Studio 生成器时，二进制位于 build-copilot/Release；若使用 Ninja，多数情况下位于 build-copilot/ 目录。

—

## 备注

- 示例仅依赖公开 API，不触达内部模块，保证后续内部重构的兼容性。
- 随阶段推进，示例将逐步迁移至 PhaseGraph/批次驱动的路径，并补充 per-phase 遥测展示。
