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
