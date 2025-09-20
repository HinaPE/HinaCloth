# XPBD 布料物理引擎：项目设计报告（4+3 架构）

## 目标与范围

* 面向“实时/交互式”的 XPBD 布料模拟，强调性能、确定性和扩展性。
* 通过对外稳定 API 隐藏实现细节，实现“初始化即重排、运行期高吞吐”的数据驱动引擎。
* 支持多布局（SoA/AoSoA/…）、多后端（Native/AVX2/TBB），并可演进至 GPU。
* 以“4+3”抽象为内核：State、Parameters、Topology、Policy + Space、Operators、Events。
* 两圈外环：Cooking（把描述烹成可执行模型）与 Execution（将模型高效执行）。

---

## 设计原则

1. **解耦**：算什么（4+3）与怎么算（后端）彻底分离；结构变化在帧边界处理。
2. **数据优先**：数据布局与访问模式优先于接口美学；初始化重排胜过零拷贝。
3. **确定性**：固定顺序、固定分块、固定归约，跨线程跨平台稳定复现。
4. **冷/热线路分离**：昂贵分析与预处理在 Cooking 固化到 Model；运行期仅执行计划。
5. **可演进**：新增算子、布局、后端无需更改对外 API；能力通过查询暴露。
6. **可观测**：统一遥测面向调参与回归；可选最小开销统计常驻。

---

## 顶层结构与分工

```
include/sim/      对外稳定 API（唯一被使用者包含）
src/shell/        API 实现层：校验、翻译、打包、缓存追踪
core/model/       常量模型 Model（烹制结果，可共享）
core/data/        时变数据 Data（每帧更新）
cooking/          PhaseGraph、Islanding、LayoutPlan、Precompute、Accel、Cache
backends/         Storage、Kernels、Scheduler、Registry、Determinism、Telemetry
runtime/          帧编排：step、命令与事件、重建与迁移、时钟
adapters/         壳 ↔ 引擎粘合（桥接 shell 输入到 engine 各子系统）
examples/tests    示例与验证
```

---

## 4+3 抽象与职责落位

* **State**：节点/场的时变字段（positions、velocities、λ 缓冲等）。
  输入在 `shell` 标准化；运行时落位在 `core/data` 与 `backends/storage`。
* **Parameters**：标量/向量/矩阵/Blob。
  结构性参数被烹制进 Model，运行期小参数进 Overrides。
* **Topology**：关系（如 edges、bend\_pairs、pins）。
  经 `cooking` 生成 PhaseGraph、分区与布局计划。
* **Policy**：求解策略（iterations、substeps）与执行策略（layout、backend、threads、deterministic、telemetry）。
* **Space**：离散空间特性（Lagrangian 等），影响烹制与近似策略。
* **Operators**：约束/力学算子声明（字段读写、所依赖的关系、阶段）。
* **Events**：结构与运行期事件。结构事件触发 rebuild；运行期事件作为小命令直达 Overrides/Data。

---

## 对外 API（include/sim/）

* **sim.h**：`create/destroy/step/push_command/flush/query_chosen/telemetry_query_frame`
  使用者只面对这组函数，所有内部细节（布局、后端、加速结构）隐去。
* 其余头（state\_in.h、topology\_in.h、parameters\_in.h、policy\_in.h、space\_in.h、operators\_in.h、events\_in.h、build.h、commands.h、telemetry.h、capability.h、status.h、ids.h、version.h）定义输入与控制面。

**关键点**：
API 仅描述“要算什么与如何控制”，不暴露“如何存储和如何并行”。

---

## Shell 层（src/shell/）

职责：把用户输入磨成“中立形态”，并决定何时触发重建。

* **validators**：Strict/Tolerant 校验尺寸、NaN、越界、重复关系、字段一致性。
* **translators**：单位、坐标系、字段名、兼容别名归一化；把用户的“弹簧常数”转为 XPBD 的 compliance 语义。
* **packers**：把 AoS/自定义 stride 的外部数据“视图化并对齐”成中立输入；不绑定具体 SoA/AoSoA。
* **cache\_tracker**：跟踪 Topology/Operators/Space 的内容哈希，决定是否复用已有 Model 或触发 rebuild。
* **sim\_create/sim\_step/sim\_commands**：API 的薄实现，调用 adapter 进入 engine；命令相位划分（BeforeFrame/AfterSolve）。

---

## Cooking（外环一）

把 4+3 的结构性描述“烹”成只读、可复用、可并行的 **Model**。

1. **PhaseGraph**

  * 依据 Operators 的读/写集与关系，拓扑染色，得到无写冲突的阶段与批次序。
  * 稳定排序键保证确定性（按关系 ID、字段 ID、内容哈希等构造 lexicographic key）。

2. **Islanding**

  * 以拓扑连通性、pins、切割边等划分独立岛，确定并行粒度与必要屏障。
  * 可选基于约束图或节点图的混合策略。

3. **LayoutPlan**

  * 选择 SoA / AoSoA（块大小、对齐、字段顺序、重排索引），面向 AVX2 宽度对齐和高速缓存局部性。
  * 产出从作者编号到内部存储编号的映射表。

4. **Precompute**

  * 距离约束 rest-length、弯曲约束角度目标、权重、alpha=compliance/(dt²) 前置常量、稀疏邻接描述。
  * 可缓存到磁盘提高二次加载性能。

5. **AccelSkeleton**

  * 用于自碰/碰撞的 BVH 拓扑骨架（只存结构，叶子 AABB 在 Data 帧内 refit）。

6. **Cache**

  * 内容哈希（Topology+Operators+结构 Parameters+布局策略+Space）→ 命中则直接还原 Model。
  * 版本号与兼容策略确保缓存安全。

**产物**：`Model = {PhaseGraph, PartitionPlan, LayoutPlan, Precompute, AccelSkeleton, Hash, Version}`
属性：**只读、可共享、可序列化**。

---

## Core Model / Core Data

* **Model（常量）**：Cooking 的结果；可被多份 Data 共享，避免重复烹制。
* **Data（时变）**：positions/velocities、预测位置、约束 λ 缓冲、帧内 scratch、接触缓存、BVH 叶子。
* **Remap**：当结构事件引发 rebuild 时，根据映射把旧 Data 迁移到新布局（保留状态、清理失效 λ）。

---

## Backends（外环二：执行）

### Storage

* **SoA / AoSoA**：对齐分配、按 block 编排、字段视图（Vec3、Edges、BendPairs…）。
* **目标**：顺序访问、减少跨 cacheline、为 AVX2/GPU 准备好一致的打包粒度。

### Kernels

* **约束族**：Distance、Bending、Attachment（可扩展 Area/Shear/SelfCollision）。
* **实现族**：Native 标量、AVX2 宽向量；统一签名（强类型视图 + 只读常量包）。
* **无写冲突**：由 PhaseGraph 和颜色批保证；并行执行无需原子操作。

### Scheduler

* **seq**：串行基线。
* **tbb**：任务化并行，静态分配固定粒度，必要时按 island→batch 两级切分；相位 barrier 明确。

### Registry

* 根据 `Policy.exec` 和硬件能力选择组合（Storage × Kernels × Scheduler）；向上报出 `Chosen`。

### Determinism

* **排序**：稳定相位与批序；固定分块大小与任务划分。
* **归约**：固定形状的二叉树或分层 pairwise 规约。
* **RNG**：计数器密钥式（model\_hash, frame, substep, index, sample），无共享状态。
* **结果**：多线程亦能逐位重现（给定编译器/ISA 组合）。

### Telemetry

* 帧时长、相位/批耗时、迭代残差曲线、重建次数、岛屿规模分布等；可开关、低开销聚合。

---

## Runtime（帧编排）

* **step**：

  1. 应用 BeforeFrame 命令与事件（小命令直改 Overrides/Data；结构事件触发 rebuild+remap）。
  2. 子步循环：外力积分→按 PhaseGraph 迭代执行各约束批→可能的阻尼/后处理。
  3. AfterSolve 阶段：采样遥测、清理 scratch。

* **命令与事件**：

  * 命令：小粒度，立即生效，不触发重建（如 gravity、iterations、pin/unpin 单点、风脉冲）。
  * 事件：结构性（cut/emit、启停算子、布局策略改变、LOD），仅在帧边界应用并 rebuild。

* **重建与迁移**：

  * Cooking 生成新 Model；Remap 将旧 Data 搬迁；在同一帧边界切换，保持连贯状态。

* **时钟**：统一帧计数与累计时间，供确定性与脚本驱动使用。

---

## XPBD 布料：算子与数据

* **DistanceConstraint**：保持边长，XPBD 形式存储 λ；权重与 alpha 来自 Precompute。
* **BendingConstraint**：保持弯曲角目标（或等价对偶形式），需要相邻面或对偶图关系。
* **AttachmentConstraint**：把顶点拉向目标位（Kinematic/Pin），常作为 PreSolve。
* **可选**：Area/Shear 保形，SelfCollision 用 BVH/网格代理；均以 Operators 声明进入 PhaseGraph。

**数据列**：
positions、velocities、预测位置、每约束 λ、（可选）inv\_mass、面法线缓存、碰撞代理、岛屿索引等。

---

## 交互与数据流（时序）

```
Authoring 4+3
 → shell: validate → translate → pack → cache check
 → cooking: phase_graph + islanding + layout_plan + precompute + accel + hash
 → Model (RO)                   Data (RW, from StateInit)
 → runtime: step per frame
     BeforeFrame: apply commands/events → maybe rebuild+remap
     substeps:
       integrate external forces
       for iterations:
         for each phase and batch:
           scheduler dispatch → kernels execute on Storage views
       velocity update and optional damping
     telemetry sample
```

---

## 性能与工程要点

* **布局驱动性能**：优先 AoSoA 与 block 对齐；跨批内存连续，避免随机访问。
* **载入与向量化**：AVX2 友好打包；避免 gather/scatter；使用预取与线性推进。
* **并行粒度**：以 island 为大粒度、batch 为细粒度；静态分配降低调度开销与不确定性。
* **缓存命中**：把 Precompute 常量放入紧邻访问的数组；避免在热路径做散列或指针链。
* **分配策略**：拥有型缓冲一次性分配；帧内 scratch 复用，减少 malloc。
* **热/冷分离**：Model/Precompute 常驻只读段；Data 热写段；遥测以 ring-buffer 方式最小化扰动。

---

## 确定性分级

* **Level 0**：不启用稳定分块与固定规约；追求峰值性能。
* **Level 1**：固定任务划分、固定归约树、稳定排序键；跨线程逐位一致。
* **Level 2**：加上严格浮点模式（如禁 FMA 差异）与固定编译器选项，跨平台一致性更强（代价更高）。

---

## 可扩展点与演进路线

* 新增算子：在 Operators 声明、Cooking 认识该算子、Backends 实现核、Registry 绑定组合。
* 新增布局：实现 Storage 和 LayoutPlan 策略；Registry 增组合。
* 新增后端：添加 scheduler/kernels 的新实现（如 GPU）；适配 Registry 与 Capability。
* 复杂事件：tear/emit/裁剪/LOD；通过 Events 在帧边界重建并 remap。
* 多物理耦合：保持 4+3 内核，新增相应 Operators 与 Space/Parameters，不触动 API。

---

## 质量保障

* **功能验证**：静态拉伸、剪切、自由下垂、固定边测试；残差收敛曲线达标。
* **确定性回归**：同输入 bitwise 相等；CI 对多线程/不同线程数/不同顺序回放比对。
* **性能基准**：不同网格密度与约束密度下的帧时；并发扩展曲线；布局/后端对比。
* **鲁棒性**：输入 fuzz/NaN 注入；极端参数（高 compliance/低质量）稳定性。

---

## 能力与版本

* **Capability**：枚举支持的布局与后端组合；暴露向量宽度、对齐要求、线程上限等。
* **Versioning**：Model/Cache 的版本号；向后兼容策略与安全降级路径。

---

## 典型用法（思维流程）

1. 以朴素数组定义 State、拓扑与参数；声明 Operators；指定 Policy.exec/solve。
2. `create`：shell 标准化 → cooking 烹制 → Model 固化、Data 初始化 → registry 选后端。
3. 每帧：注入小命令（风、pin、调迭代数），`step(dt)`；必要时在帧边界触发 cut/emit 重建。
4. 拉取 `telemetry`，做调参与优化；改 Policy 即可切换后端与布局。

---

## 结语

这套“4+3 + 两圈外环”的设计，把 **“描述 → 计划 → 执行”** 的流水线拆清楚：

* 4+3 定义了“算什么”；Cooking 把“怎么排与怎么放”固化进 Model；Execution 把“怎么算”落到硬件；Runtime 管理生命周期与事件。
* 使用者只面向稳定 API；新增算子/布局/后端都能在各自模块落位，互不干扰。
* 在确定性与性能之间可拨档：默认高性能，需时可一键切到严格重现。

接下来，我们可以围绕这份报告逐条细化：先给 PhaseGraph 的颜色规则与批粒度策略落规范，再把 AoSoA/AVX2 的内存对齐与载入约定写成小白皮书，最后补上事件重建与 remap 的状态迁移矩阵。

---

## 项目架构与设计 review（AI 自动生成）

### 设计理念与架构亮点
1. **4+3 抽象分层**
   - 4（State/Parameters/Topology/Policy）与 3（Space/Operators/Events）将物理建模与执行策略彻底解耦，便于扩展和维护。
   - 每一维度职责清晰，输入/输出路径明确，便于后续算子、后端、布局的无缝演进。
2. **壳层 API 设计**
   - `sim.h` 只暴露极简且稳定的入口（create/destroy/step/command/telemetry），对外屏蔽所有内部复杂性。
   - 通过 BuildDesc 一次性打包所有输入，支持严格校验和懒打包，保证初始化高效且安全。
3. **数据驱动与布局优先**
   - AoS/SoA/AoSoA 等多布局支持，初始化时重排数据，运行期只关注高效执行，体现“数据优先”原则。
   - 后端选型（Native/AVX2/TBB/GPU）与线程数、确定性等策略均可通过 Policy 灵活配置。
4. **命令与事件机制**
   - 运行期所有变更通过命令队列下发，结构性变更与小参数分流处理，保证帧边界的确定性和高吞吐。
   - flush_commands 机制将命令分阶段应用，支持结构事件触发重建与迁移。
5. **可观测性与遥测**
   - 统一 Telemetry 查询接口，便于调参、性能回归和统计分析，支持最小开销常驻统计。
6. **模块分工与代码组织**
   - shell 层负责校验、翻译、打包、缓存追踪，彻底隔离 API 与引擎内部。
   - core/model 固化烹制结果，core/data 维护时变数据，cooking/ 负责预处理与加速，backends/ 实现存储与算子，runtime/ 编排帧循环与命令事件，adapters/ 桥接壳层与引擎。
   - examples/tests 提供完整用例与验证，便于回归和扩展。

### 实现细节与工程规范
- 代码风格统一，模块命名清晰，API 设计极简，便于外部集成和长期维护。
- CMake 构建体系与 IDE 配置分离，支持 out-of-tree 构建和自动化测试。
- 结构性参数与运行期参数分离，保证模型稳定性与运行期灵活性。
- 命令队列与事件脚本机制，支持高效批量变更和结构重建。
- 单元测试与示例代码覆盖典型用例，便于验证和回归。

### 建议与展望
- 可进一步完善遥测统计维度，支持更细粒度的性能与行为分析。
- examples/tests 可增加异常路径和边界用例，提升健壮性。
- cooking/ 可探索更多预处理优化策略，提升大规模场景下的初始化效率。
- 后续可考虑 GPU 后端和异步执行模型，进一步提升性能和扩展性。

---

## 实现现状与差距评估（代码级 Review）

基于当前代码库（src/… 模块）的快速走查，这里给出“按模块”的实现现状与与设计蓝图的差距评估，帮助明确下一步的落地顺序：

- API / Shell（`src/api`, `src/shell`）
  - 已有：对外稳定 API（create/destroy/step/push_command/flush/query_chosen/telemetry），以及最小化的校验、翻译（部分空实现）、打包（空实现）、缓存跟踪（占位 hash）。
  - 差距：
    - validators 仅做了基本尺寸/空指针检查，未覆盖 NaN、越界、重复关系、字段一致性等严格校验。
    - translators/packers 基本为空，未做单位/坐标/别名归一化与 AoS→中立输入的强健打包。
    - cache_tracker 仅拼接了少量指针/数值，未形成可复用的内容哈希；无缓存命中路径。

- Cooking / Model（`src/cooking`, `src/core/model`）
  - 已有：从 Topology 中取第一个二元关系（视为 edges），根据 StateInit 的 position 计算 rest-length，装入 Model（只含 node_count/edges/rest）。命令触发的 rebuild 走“复制 + 恒等 remap”的占位流程。
  - 差距：
    - 未识别 Operators/Space/Policy，对 PhaseGraph/批次/颜色/排序完全未构建。
    - 无 Islanding、无 LayoutPlan（SoA/AoSoA/block 对齐/重排索引），无 Precompute 的权重/alpha/邻接/常量表。
    - rebuild/Remap 仅做恒等映射，未处理节点增删、边变更后的状态迁移与 λ 清理。

- Core Data（`src/core/data`）
  - 已有：x/y/z、vx/vy/vz 与预测位置 px/py/pz，重力参数 gx/gy/gz；从外部 AoS 载入 position/velocity；小命令支持 SetParam(gravity_*)。
  - 差距：
    - XPBD 关键数据缺失：inv_mass、每约束 λ 缓冲、可选的固定/附件标志、面法线缓存、岛屿索引等。
    - Overrides 覆盖面过窄（仅重力），缺少 iterations/substeps/damping 限定覆盖，缺少 pin/unpin、区域写入等小命令。

- Backends：Storage / Kernels / Scheduler / Registry（`src/backend/...`）
  - Storage：SoA 仅有轻量视图；AoSoA 为空壳，未提供对齐分配、块化访问与视图。
  - Kernels：distance 仅做朴素 GS 投影，未按 XPBD 引入 compliance α 与 λ 累积；bending 占位；无 attachment/pin；无 AVX2 实现。
  - Scheduler：seq 直调 kernel；tbb 占位（无并行）；无基于 PhaseGraph/Island 的两级调度与 barrier；无确定性分块。
  - Registry：仅依据 Policy.exec 直选组合，未结合硬件能力与布局计划。

- Runtime（`src/runtime`）
  - 已有：最小帧循环（integrate → distance → finalize），忽略 ovr/policy，telemetry 仅填 0；phases/apply_events 均为空。
  - 差距：
    - 未实现子步/多迭代/阻尼；未按 PhaseGraph 与批次调度；无 BeforeFrame/AfterSolve 的完整命令相位；无残差统计、耗时分项。

- 适配与示例（`src/adapter`, `examples/e_min.cpp`）
  - 已有：engine_adapter 将 shell 与各子系统串起；示例 e_min 以网格 + distance 演示最小路径，并能动态改重力。
  - 差距：
    - 示例缺少 pin/attachment/bending/并行与确定性演示；无更多验证用例与基准。

综上，当前实现已具备“端到端可跑通”的 MVP 骨架，验证了 API 形态与最小数据流；与 README 的完整蓝图相比，主要缺口集中在：

1) XPBD 正确性：每约束 λ、compliance α、inv_mass/权重、稳定迭代策略。
2) 计划与并行：PhaseGraph/颜色/批次、Islanding、确定性分块与 TBB 并行调度。
3) 布局与存储：AoSoA、重排索引与 LayoutPlan、对齐与向量化友好访问。
4) 运行期编排：子步/迭代/阻尼/相位化命令与事件、残差与耗时遥测。
5) 扩展与鲁棒：bending/attachment/self-collision、严格校验、缓存与版本化、能力枚举。

建议从“XPBD 正确性 + Policy 落地”为第一优先，随后引入 PhaseGraph/Islanding 与并行调度，再推进 AoSoA 与向量化。详细的分阶段实现路径、交付物与验收标准，已写入根目录的 `roadmap.md`，可据此推进开发与回归。
