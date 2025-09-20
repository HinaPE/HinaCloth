# HinaCloth Roadmap（重写版，2025-09-20）

本路线图基于对 README 设计理念与 src 代码现状的全面走查（runtime、backend、cooking、core、shell、adapter、api）。目标是在不破坏对外 API 的前提下，按“4+3 + 两圈外环”的蓝图，循序推进正确性、并行性与性能。

—

## 现状快照（MVP 已具备的能力）
- API/Shell
  - `create/step/push_command/flush/telemetry/query_chosen` 全链可用；按命令分相位（BeforeFrame/AfterSolve）。
  - 轻量校验（字段尺寸/别名）、翻译/打包占位；模型缓存支持“内存级命中”（无落盘）。
- Cooking/Model
  - 由 `edges` 关系生成 rest-length；基于连通性做岛屿划分，并将 edges/rest 以岛为粒度重排；bend_pairs 支持与弯曲目标角预计算。
  - `Model` 含节点数、edges/rest、island_offsets、node_remap(恒等)、layout_block_size。
- Data
  - SoA：x/y/z，vx/vy/vz，预测位 px/py/pz。
  - XPBD 关键：inv_mass（pin=0）、per-edge λ（lambda_edge）。
  - Solve/Exec：substeps、iterations、damping；后端/布局（SoA/AoSoA）、AVX2/TBB 开关；AoSoA 缓冲与 block_size。
  - Operator 数据：Attachment 权重与目标、使能位；Bending 使能位。
- Backends
  - Storage：SoA 视图、AoSoA 打包/解包与读写/Axpy。
  - Kernels：
    - Distance：标量 SoA（XPBD 公式含 α 与 λ）、AVX2 SoA、AoSoA（含 AVX2 gather 与尾部标量）。
    - Attachment：预测位预拉向目标（权重、pin 跳过）。
    - Bending：简化对偶投影（非 XPBD，demo 级）。
  - Scheduler：`seq` 基线；`tbb` 文件存在但未实现细粒度调度（runtime 中提供按岛并行的粗粒度路径）。
  - Registry：结合 Policy 与 CPUID 选择 Backend/Layout，并对 AVX2 默认偏好 Blocked。
- Runtime
  - 完整子步循环：积分 → Attachment 预解 → Distance 投影（SoA/AoSoA，岛级并行、可 AVX2）→ 可选 Bending → 速度更新与阻尼。
  - Telemetry：step_ms、average distance residual。
- 其他
  - Capability 枚举按编译宏导出组合；示例与可执行可构建（Debug/Release）。

—

## 与蓝图的差距（未完成/待完善清单）
- PhaseGraph/批次/确定性
  - 未按 Operators 读/写集构建 PhaseGraph、颜色/批次与稳定排序键（`src/runtime/phases.cpp` 空）。
  - 未实现“每相位 barrier + 固定分块 + 确定性分割”的统一执行计划。
- Scheduler/并行
  - `backend/scheduler/tbb.cpp` 尚未实现；当前并行仅限 runtime 内“按岛 tbb::parallel_for”，无批内分块与 barrier 管理。
  - 缺固定任务粒度与稳定分配策略（确定性 Level 1/2）。
- 事件与结构变更
  - `src/runtime/apply_events.cpp` 空；结构命令在 `engine_adapter` 触发 rebuild，但 `cooking_rebuild_model_from_commands` 仅复制旧模型（未处理 Add/RemoveNodes/Relations、算子启停、布局策略变更）。
  - Remap 仅支持恒等映射与数据搬迁的基础情形；未处理节点增删、边变更后的 λ 清理/保留策略矩阵（当前统一清零边 λ）。
- Cooking/布局/预计算
  - 无 PhaseGraph、无 LayoutPlan（重排索引/块对齐/字段顺序策略）；`node_remap` 恒等。
  - 预计算缺权重表/邻接/每约束 α 常量包；未落盘缓存（仅内存命中）。
- 存储/布局执行
  - AoSoA 仅在每子步 pack→solve→unpack，缺“常驻 blocked 存储视图 + 局部更新”以消除往返拷贝。
  - `layout_block_size` 选择缺启发式/硬件对齐策略与能力感知。
- 算子族
  - Bending 非 XPBD（无 α/λ/inv_mass 权重）；无 Area/Shear；无碰撞/自碰约束族。
- 校验/翻译/打包
  - validators 缺 NaN/越界/重复关系检测与严格/宽松两档覆盖；
  - translators/packers 基本占位，无单位/坐标/别名归一化与 AoS 变体的健壮打包与 stride 保护。
- Telemetry/可观测
  - 缺 per-phase/per-batch 耗时与残差曲线、岛屿规模分布、重建次数的细粒度统计与开关。
- 确定性/数值模式
  - 未提供稳定规约树、固定分块大小、禁 FMA 等确定性档位切换（README 的 L1/L2 未落地）。
- 缓存/版本/能力
  - 模型缓存仅内存 Map；缺版本号与落盘；能力枚举未包含具体向量宽度/对齐/线程上限。
- 示例/测试/CI
  - 示例未覆盖 pin/bending/并行/确定性演示；缺最小单元测试与性能基准输出；CI 未配置。

—

## 新路线图（阶段化交付与验收）

说明：优先保证“XPBD 正确性 → 并行与确定性 → 布局与性能”，每阶段给出交付物与验收标准；默认维持现有对外 API 不变。

### 阶段 A：XPBD 基线完善（正确性/可调参）
- 目标：Distance XPBD 完整、运行期可控；Attachment 稳定；最小 Telemetry 完善。
- 任务
  - Distance：补充 per-edge α 注入路径（Parameters: distance_compliance 支持全局+覆盖）。
  - Bending：先收敛为稳定 PBD 版本（带 inv_mass 权重），标注为“实验性”。
  - Runtime：覆盖 substeps/iterations/damping 的 SetParam 覆盖优先级；残差采样入 Telemetry。
  - Validators：添加 NaN/越界/重复 edge 检测（宽松→警告，严格→报错）。
- 交付/验收
  - e_min 自由下垂/迭代收敛可控；残差随迭代下降；Attachment 在 pin 与非 pin 上行为正确。

### 阶段 B：PhaseGraph + 确定性顺序（可并行的计划）
- 目标：固化“算子 → 阶段/批次”的执行计划，形成确定性顺序和批粒度。
- 任务
  - PhaseGraph：按 OperatorsDecl 推导读/写集，岛内对每关系独立染色；稳定排序键。
  - 批次：固定分块（默认 64 约束/任务）并持久化到 Model；
  - Runtime：按 island→phase→batch 的顺序执行（先 seq 基线）。
- 交付/验收
  - 在仅 Distance 情形下，PhaseGraph 生成一致；结果与旧实现 bitwise 保持或误差阈内一致。

### 阶段 C：TBB 调度与确定性（L1）
- 目标：实现可复现的并行调度（Level 1）。
- 任务
  - Scheduler.TBB：按 island 粗粒度 + batch 细粒度并行；相位 barrier；固定任务划分与稳定分配。
  - Runtime：可切换 Backend=Seq/TBB，结果逐位一致（或限定阈内）。
  - Telemetry：记录 per-phase 耗时与岛屿规模直方图（粗略）。
- 交付/验收
  - e_pin 在多线程下结果与单线程一致；TBB 相对 seq 可获得 >1.5× 加速（依规模）。

### 阶段 D：LayoutPlan + AoSoA 常驻 + AVX2 强化（性能）
- 目标：布局驱动性能，消除子步的 pack/unpack 往返。
- 任务
  - Cooking.LayoutPlan：根据硬件与策略选择 block_size，生成 node_remap 与对齐分配计划。
  - Data/Storage：引入“常驻 blocked 视图”，仅在必要时与 SoA 互转；约束核支持 AoSoA 直接运行。
  - Kernels：审视 AVX2 路径的 gather/scatter 与越界保护；统一签名与回退路径。
  - Registry：结合 CPUID/Policy 选择 SoA/AoSoA × Native/AVX2。
- 交付/验收
  - 中等规模（~50k 约束）下 AVX2+AoSoA 较 SoA 标量达到 >2× 加速；无未对齐访问错误。

### 阶段 E：事件重建 + Remap 稳健化
- 目标：结构变更可用，状态迁移安全可控。
- 任务
  - Cooking.Rebuild：支持 Add/RemoveNodes/Relations/启停算子/布局策略变更；
  - Remap：节点向量映射、边 λ 清零/保留策略矩阵；AoSoA 缓冲随布局重配；
  - Runtime.apply_events：在帧边界应用事件并重建+迁移；
  - Telemetry：记录重建次数与耗时。
- 交付/验收
  - 示例 e_flag：运行中启停 bending/attachment、剪切边/加边后不爆 NaN，状态连续。

### 阶段 F：工程化（Cache/Version/Capability/验证）
- 目标：提升冷启动与鲁棒性，完善输入面与能力查询。
- 任务
  - Cache：内容哈希 + 版本号 + 落盘（安全降级策略）。
  - Validators/Translators/Packers：严格/宽松两档、单位/坐标归一化、AoS/SoA 变体打包与 stride 检查。
  - Capability：输出 backend×layout×向量宽度×对齐×线程上限。
  - Tests/Bench：最小单元测试与 bench CSV 输出；本地脚本做冒烟/回归。
- 交付/验收
  - 二次 create 命中缓存提速明显；Fuzz 输入在严格模式下稳定报错，宽松模式给出合理默认继续运行。

### 阶段 G（可选）：自碰与 BVH 骨架
- 目标：支撑常见布料自碰；保留轻量性能目标。
- 任务
  - AccelSkeleton：静态 BVH 拓扑；Data 帧内 refit 叶子 AABB。
  - SelfCollision：点-三角/边-边 代理约束；进入 PhaseGraph 与并行体系；
- 交付/验收
  - e_selfcol 示例可复现基础自碰；穿透明显减少，帧时可控。

—

## 近期两周落地清单（Quick Wins）
- validators：补 NaN/越界/重复 edge 的检查与统计（宽松→警告）。
- Telemetry：记录 iterations/substeps、per-step 残差；导出 chosen 后端与布局。
- 示例：新增 e_pin 展示 inv_mass=0 与 attachment；bench 输出 CSV。
- TBB：在现有 runtime 岛级并行基础上加 threads 上限控制与稳定任务粒度。

—

## 任务→代码模块映射（执行参考）
- PhaseGraph/批次：`src/runtime/phases.*`，`src/cooking/*`（生成图/颜色/批次并序列化至 Model）。
- Scheduler（TBB）：`src/backend/scheduler/tbb.*`；Runtime 调度整合：`src/runtime/step.cpp`。
- LayoutPlan/AoSoA：`src/cooking/cooking.*`（重排索引/块计划）、`src/backend/storage/*`、`src/backend/kernel/*`（AoSoA 直接核）。
- 事件/Remap：`src/runtime/apply_events.*`、`src/cooking/cooking.*`、`src/core/data/*`。
- 校验/打包/缓存：`src/shell/*`（validators/translators/packers/cache_tracker）。
- 能力/注册表：`src/backend/registry/*`。
- Telemetry：`src/api/telemetry.h` + `src/runtime/step.cpp` 采样与聚合。

—

## 验收基线（绿灯标准）
- 构建：Windows Release/Debug 可编译；开启/关闭 AVX2、TBB 宏均能通过。
- 功能：e_min/e_pin/e_flag 跑通；命令/事件在帧边界生效；无 NaN。
- 确定性：Seq 与 TBB 在 Level 1 下数值逐位或阈值内一致（记录阈值）。
- 性能：AoSoA+AVX2 在中等规模下较 SoA 标量加速比达标；并行扩展性合理。

—

若需，我可以按此路线图先落“阶段 A + 近期 Quick Wins”，并补上 e_pin 示例与 bench 脚本。
