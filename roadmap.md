# XPBD Cloth Engine Roadmap

目标：在现有 MVP 骨架上，分阶段落地“4+3 + 两圈外环”设计，达成正确性、性能、确定性与可扩展性的平衡。每个阶段给出交付物与验收标准，便于回归与推进。

注意：
- 默认优先 CPU/SoA → AoSoA/AVX2 → TBB 并行；GPU 留作后续可选路线。
- 每阶段结束需跑最小基准与确定性回归；CI 收敛后再进入下一阶段。


阶段 0：基线稳固与最小可用
- 目的：保证当前 MVP 可持续演进，基础设施就绪。
- 任务：
  - 构建脚本最小整理：Debug/Release 配置与 warnings 设置；打开常见告警；禁用 MSVC 不必要 CRT 安全告警。
  - examples 拆分：保留 e_min，并新增 e_smoke（仅 create/step/destroy），作为冒烟测试。
  - 引入最小单元测试骨架（可选 doctest/catch2，或自写轻量测试宏）。
- 交付物：
  - Windows 下 Release/Debug 可编译，examples 运行通过。
  - 一条 CI 脚本雏形（本地脚本），包含构建+冒烟+示例 run。
- 验收：构建时间 < 5 分钟；e_min 输出 telemetry 不崩溃。


阶段 1：XPBD 正确性基线（Distance + Overrides）
- 目的：从“朴素投影”升级到“真正的 XPBD 距离约束”，并完善运行期覆盖策略。
- 任务：
  - Data：
    - 新增 inv_mass（默认为 1，支持通过 Parameters 注入质量/固定点：pin→inv_mass=0）。
    - 新增 λ 缓冲（每条边 1 个），帧内累积；新增 per-constraint 有效标志（可选）。
  - Cooking：
    - Precompute：为每条边计算权重 w_i、w_j 与 alpha = compliance/(dt^2) 的常量位（compliance 可作为参数，默认 0）。
    - 校验：检测 edges 越界、重复边（可选哈希/排序去重，先告警）。
  - Kernels（distance）：
    - 实现 XPBD 公式：Δx = - (w_i ∇C_i + w_j ∇C_j) * (C + alpha*λ_prev)/(Σw|∇C|^2 + alpha)，更新 λ。
    - 支持 inv_mass=0 的 pin；支持 iterations/alpha；允许 dt 传入。
  - Runtime：
    - 实现 substeps × iterations；添加 velocity damping（Policy.solve.damping）。
    - SolveOverrides：可覆盖 substeps 与 iterations（优先级高于 Policy）。
    - Telemetry：记录 step_ms（粗略）、迭代次数；预留残差（可先返回 0）。
  - Shell：扩展小命令 SetParam 支持 iterations/substeps/damping；支持 pin/unpin 单点（通过 SetFieldRegion 或专用命令）。
- 交付物：
  - e_min 下布料自由下垂正确（顶边若未 pin 会整体下降）。
  - 新增 e_pin：固定顶边两列点，观测稳定下垂。
- 验收：
  - 位置变化随迭代数增加而收敛；inv_mass=0 的点位置保持；修改 gravity/iterations/substeps 能即时生效。


阶段 2：PhaseGraph + Islanding + 并行调度雏形
- 目的：将“算子图”固化为可并行计划，引入确定性并行的基础设施。
- 任务：
  - Cooking：
    - PhaseGraph：解析 OperatorsDecl，识别读/写字段与依赖关系；对 edges 约束图进行颜色分组（无写冲突批次）。
    - Islanding：基于拓扑连通性与 pin 划分岛；产出 island 索引与批次表。
  - Scheduler：
    - seq：按 island→phase→batch 顺序调度，作为确定性基线。
    - tbb：并行执行不同 island；batch 内可按固定分块大小并行；加入 barrier 以维持阶段顺序。
    - 确定性：固定任务切分大小与排序键，避免竞态。
  - Runtime：
    - 跑 PhaseGraph：多算子（先仅 distance），留接口给 bending/attachment。
  - Telemetry：
    - 记录 per-phase 耗时与岛屿规模分布（粗略统计）。
- 交付物：
  - PhaseGraph/Islanding 数据结构（model 内可序列化）。
  - tbb 路径对 e_pin 加速明显（>1.5×，取决于核数与规模）。
- 验收：
  - 开关 Backend=Seq/TBB 结果 bitwise 一致（同编译/ISA 下），或至少在 Level 1 近似一致并稳定（见 README“确定性分级”）。


阶段 3：LayoutPlan + AoSoA + AVX2（性能里程碑）
- 目的：布局驱动性能，完成 SoA→AoSoA 的数据重排与 AVX2 向量核。
- 任务：
  - Cooking：
    - LayoutPlan：选择块大小（如 8/16），产出从作者编号到内部编号的重排索引；预分配对齐缓冲。
  - Storage：
    - AoSoA 视图与绑定函数；提供顺序遍历与块内向量化友好接口。
  - Kernels：
    - distance AVX2 实现（加载/归一化/更新），与 SoA/AoSoA 两套路径统一签名；回退到标量。
  - Remap：
    - rebuild 时支持从旧布局到新布局的数据迁移（positions/velocities/λ 等）。
  - Registry：
    - 结合 Policy 与硬件能力（CPUID）选择 SoA/AoSoA × Native/AVX2。
- 交付物：
  - 中等规模网格（~50k 约束）在 Release 下获得显著加速（>2× 对比 SoA 标量）。
- 验收：
  - 性能基准曲线：迭代固定下，帧时随约束数线性增长；AVX2 路径对齐正常（无越界/未对齐访存报错）。


阶段 4：Bending/Attachment + 相位化命令与事件 + 稳健 Remap
- 目的：丰富算子与命令/事件通道，完成结构变更的安全重建与状态迁移。
- 任务：
  - Cooking：
    - 识别 bending/attachment 的关系输入（bend_pairs/ pins），生成相位与批次；预计算 bending 目标角/权重。
  - Kernels：
    - 实现 attachment（朝目标投影、支持不同权重），实现 bending（基线对偶形式）。
  - Runtime：
    - BeforeFrame/AfterSolve 两阶段 flush 完整落地；小命令直达 Overrides/Data，结构事件触发 rebuild+remap。
  - Remap：
    - 支持节点增删、边修改后的数据迁移；对失效 λ 清零；对新增约束初始化 λ=0。
  - Telemetry：
    - 残差采样（每迭代或每批次）；记录重建次数与平均耗时。
- 交付物：
  - 示例 e_flag：带 bending 与 attachment 的旗帜（顶边固定，中部目标点拉扯）。
- 验收：
  - 在结构事件（增删边/启停算子）后，状态连续，无爆 NaN；残差曲线稳定。


阶段 5：Cache/Version/Capability + 校验/打包完善
- 目的：工程化完善，提升冷启动与鲁棒性。
- 任务：
  - Shell/cache_tracker：
    - 内容哈希（拓扑+Operators+结构参数+布局策略+Space）；命中即从缓存加载 Model；加版本号与兼容策略。
  - validators/translators/packers：
    - Tolerant/Strict 两档；字段名别名归一化；单位规范化；AoS/SoA 异构输入安全打包（含 stride 检查）。
  - Capability：
    - enumerate_capabilities 输出支持的组合（layout×backend×向量宽度×对齐）。
- 交付物：
  - 首次 create 会缓存 model；二次 create 命中后加速（以 2× 为目标，依赖 IO）。
- 验收：
  - Fuzz 输入（随机尺寸/stride/缺字段）在 Strict 模式下能准确报错；Tolerant 模式能给出合理默认并继续运行。


阶段 6（可选）：自碰撞与 BVH 骨架
- 目的：完善布料常见必需特性。
- 任务：
  - AccelSkeleton：构建静态 BVH 拓扑；Data 帧内 refit 叶子 AABB。
  - SelfCollision：点-三角/边-边 代理约束，引入相位并行与 λ 缓冲；支持简单 culling。
- 交付物：
  - 示例 e_selfcol：简单布料自碰样例。
- 验收：
  - 自碰激活后，穿透大幅减少；帧时回归在可控范围内（可通过算子开关对比）。


阶段 7：文档/示例/基准/CI 收尾
- 目的：对外可用与可持续。
- 任务：
  - README：补充 PhaseGraph 颜色规则规范、AoSoA/AVX2 约定小白皮书、事件重建与 remap 状态迁移矩阵。
  - examples：加入更大规模 cloth、参数扫频脚本；并行/确定性开关演示。
  - 基准：不同规模/迭代/后端对比；导出 CSV。
  - CI：不同线程数/后端/构建型的回归矩阵；确定性检查（bitwise 或阈值内一致）。
- 交付物：
  - 文档与示例齐备；一键构建与跑基准；CI 绿。
- 验收：
  - PR 模板 + 回归报告样例；跨版本缓存兼容验证通过。


附录：任务到代码模块的粗映射
- Shell：`src/shell/*.cpp`（validators/translators/packers/cache_tracker/sim_*）
- Cooking：`src/cooking/*`, `src/core/model/*`
- Data/Remap：`src/core/data/*`
- Backends：`src/backend/storage/*`, `src/backend/kernel/*`, `src/backend/scheduler/*`, `src/backend/registry/*`
- Runtime：`src/runtime/*`
- Adapter：`src/adapter/*`
- API：`src/api/*`
- 示例与测试：`examples/*`, `tests/*`（可新增）

风险与缓解
- 浮点确定性：采用固定任务切分、稳定排序、禁用 FMA（可选级别），收敛到“Level 1/2”。
- 性能回退：每阶段保留 seq+SoA 路径为金标准，对比确保回退时仍可用。
- 重构风险：优先扩展不破坏 API 的内部模块；公共结构（BuildDesc/Policy 等）谨慎演进，必要时通过版本化与兼容层处理。

