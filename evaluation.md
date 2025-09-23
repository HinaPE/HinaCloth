# HinaCloth 引擎设计目标达成度评估（2025-09-24）

本评估基于当前仓库代码，对“数据与存储层支持 SoA/AoS/AoSoA 等内存布局进行计算”的设计目标进行核对与分析，并给出证据、差距与改进建议。

## 摘要结论
- SoA（Structure of Arrays）
  - 达成：已作为核心运行时数据布局与主要计算路径（位置、速度、惯量等字段以分量数组形式存储）。
- AoSoA（Blocked，结构化块化 SoA）
  - 部分达成：已实现 AoSoA 视图与打包/解包，在“距离约束”距离投影内核中提供了 AoSoA 版本；但附件（attachment）与弯曲（bending）仍为 SoA 路径。
- AoS（Array of Structures）
  - 未达成：目前没有 AoS 视图/内核；仅在数据导入阶段支持从 AoS 源数据（含 stride）加载到内部 SoA（见 load_vec3_aos）。运行时与内核未实现 AoS 计算分支。

因此，“可支持 SoA/AoS/AoSoA 等实际内存布局进行计算”的原始目标目前为“SoA 完整 + AoSoA 局部 + AoS 未实现”。

## 证据与代码定位

- 布局与后端枚举
  - `src/core/common/types.h` 中定义 `enum class DataLayout { Auto, SoA, AoS, Blocked }` 与后端枚举。尽管存在 AoS 枚举值，但下文可见尚未被实际执行路径采用。

- 数据层（核心状态以 SoA 为主）
  - `src/core/data/data.h` 定义 `Data` 结构：`x/y/z, vx/vy/vz, px/py/pz, inv_mass, ...` 均为 `std::vector<float>`（SoA）。
  - 同文件中还有 `pos_aosoa`（`std::vector<float>`）作为 AoSoA 的位置缓冲；以及运行时控制开关：`exec_layout_blocked`、`layout_block_size`。
  - `src/core/data/data.cpp`
    - `core_data_create_from_state`：读取初始状态字段，使用 `load_vec3_aos` 将 AoS 格式外部输入拆分到内部 SoA 数组。
    - 当 `exec_layout_blocked` 为真时创建 AoSoA 缓冲 `pos_aosoa`（大小 `3 * block * nb`）。
    - `core_data_apply_remap`：在重建/重映射后，如果仍为 blocked 模式，同样会重置 `pos_aosoa`。

- 存储层（Storage Views）
  - SoA：`src/backend/storage/soa.h` 定义 `SoAView3` 与 `storage_bind_soa`（简单指针视图）。
  - AoSoA：`src/backend/storage/aosoa.h/.cpp`
    - 定义 `AoSoAView3`、块参数（block、stride）、逐元素读写与 axpy 操作。
    - 提供 `storage_pack_soa_to_aosoa` 与 `storage_unpack_aosoa_to_soa` 两个打包/解包函数，用于 SoA <-> AoSoA 的数据搬运（尾块补零）。
  - AoS：未找到 AoS 对应的 `View`/工具函数/内核。

- 运行时调度与内核选路
  - `src/runtime/step.cpp`
    - 主循环将位置预测（integrate_pred）写入 `px/py/pz`（SoA）。
    - Attachment 与 Bending 始终绑定 SoA 视图：
      - `presolve_apply_attachment` 调用 `kernel_attachment_apply(SoAView3, ...)`；
      - `bending_pass` 调用 `kernel_bending_project(SoAView3, ...)`。
    - Distance 约束：
      - 如果 `exec_layout_blocked = true`，则执行：
        1) `storage_pack_soa_to_aosoa`（SoA→AoSoA）
        2) `project_distance_islands_aosoa`（AoSoA 版本的距离投影）
        3) `storage_unpack_aosoa_to_soa`（AoSoA→SoA），为后续 bending/finalize 所用。
      - 否则（SoA 路径），调用 `project_distance_islands_soa`：
        - 内部可根据 `exec_use_avx2` 选择 `kernel_distance_project_avx2(SoA)` 或标量 `kernel_distance_project(SoA)`。
  - 结果：AoSoA 目前仅用于距离约束段，且通过显式 pack/unpack 与 SoA 互转，整体运行时“以 SoA 为主、AoSoA 为加速中间缓冲”。

- 距离约束内核
  - SoA 标量：`src/backend/kernel/constraints/distance.{h,cpp}`
  - SoA AVX2：`src/backend/kernel/constraints/distance_avx2.{h,cpp}`
  - AoSoA：`src/backend/kernel/constraints/distance_aosoa.{h,cpp}`（含 AVX2 gather 路径与标量回退）。

- 其它约束内核
  - Attachment（SoA）：`src/backend/kernel/constraints/attachment.{h,cpp}`
  - Bending（SoA）：`src/backend/kernel/constraints/bending.{h,cpp}`
  - 结论：目前未提供 AoSoA/AoS 版本的附件或弯曲内核。

- 后端选择逻辑
  - `src/backend/registry/registry.cpp`：当 `exec.layout == Auto` 时，如果后端为 AVX2 则默认 `Blocked`（即 AoSoA），否则 `SoA`。但即使选择了 `Blocked`，最终只有距离约束走 AoSoA 分支，其它仍为 SoA。
  - `src/adapter/engine_adapter.cpp`：将选择结果写回 `Data`，并据此分配 `pos_aosoa`。若 `exec_layout_blocked=false` 则不会为 AoSoA 分配缓冲。

## 行为细节与影响
- SoA 为“事实上的主存储”，所有字段常驻 SoA。AoSoA 仅作为距离约束的临时中间表示（pos），每次子步中需要 pack → compute → unpack，存在额外内存搬运开销。
- `storage_pack_soa_to_aosoa` 会对尾块进行 0 填充，不影响有效范围的约束计算；unpack 仅写回有效元素。
- AVX2 支持：
  - SoA 路径下提供 `distance_avx2`；
  - AoSoA 路径下 `distance_aosoa` 亦有 AVX2 gather 实现；
  - Attachment/Bending 目前无 AVX2/AoSoA 版本。
- 并行化：TBB 并行按岛（island）维度分发，但同样仅影响距离约束路径的并行执行方式；数据布局不变。

## 与设计目标的差距
1. AoS 计算路径缺失
   - 没有 `AoSView`/`storage_bind_aos`/AoS 约束内核，也没有在 `runtime/step.cpp` 中针对 `DataLayout::AoS` 的分支。
   - 目前 AoS 仅体现在数据导入阶段（`load_vec3_aos`）。

2. AoSoA 覆盖不完整
   - 仅“距离约束”提供 AoSoA 计算；附件与弯曲仍走 SoA。
   - 这使得 Blocked 模式下仍需频繁 SoA<->AoSoA 的 pack/unpack，削弱 AoSoA 带来的缓存/向量化收益。

3. 选择与一致性
   - API 层面存在 `DataLayout::AoS` 选项，但运行时并不会真正采用该布局（会回落到 SoA 路径）。这可能与用户期望不一致。

4. 存储一致性与生命周期
   - 仅位置字段在 AoSoA 中暂存，速度/历史位置等仍为 SoA；这限制了在 Blocked 模式下跨算子维持 AoSoA 的可能性。

## 改进建议与路线图
优先级从高到低：

1) 明确 AoS 运行时支持（达成“可用”）
- 定义 AoS 视图与访问器，例如：
  - `struct AoSView3 { float* base; size_t n; size_t stride; }` + 读写/axpy 辅助函数。
- 在 `runtime/step.cpp` 中为 `DataLayout::AoS` 增加分支：
  - 绑定 AoS 视图，调用 AoS 版本的约束内核；或通过统一适配层复用 SoA 内核（见建议3）。
- 为 Attachment/Bending/Distance 提供 AoS 版本内核（可先做标量版，性能优化后续再做）。

2) 扩展 AoSoA 覆盖（达成“全算子 AoSoA”）
- 为 Attachment 与 Bending 提供 AoSoA 版本内核，避免在 Blocked 模式下频繁 SoA<->AoSoA 搬运。
- 评估是否在 Blocked 模式下让位置/速度/历史位置都常驻 AoSoA，或至少在子步（substeps）内维持 AoSoA，减少 pack/unpack 频次。典型策略：
  - 每帧开始 SoA→AoSoA；子步内全 AoSoA；帧末一次性 AoSoA→SoA。

3) 引入布局无关的访问抽象（达成“统一内核”）
- 通过轻量接口或模板参数将内核对存储访问解耦：
  - 约束内核模板：`template<class PosView3> void distance(PosView3&, ...)`，要求 `PosView3` 提供 `size()`, `read(i)`, `write(i, ...)` 或 `axpy(i, ...)` 等统一 API。
- 借此避免为每种布局复制内核逻辑，降低维护成本；同时可在编译期静态展开，保持性能。

4) 选择逻辑与文档一致性
- 当用户显式选择 `AoS` 时，应真正走 AoS 分支；否则在文档与 API 说明中明确当前仅支持 SoA/Blocked，AoS 将回退到 SoA。

5) 测试与基准
- 单元测试：同一初始状态、同一参数，在 SoA/AoSoA/AoS（或回退策略）下，结果一致（误差阈值内）。
- 微基准：对比 SoA vs AoSoA（不同 `block_size`），评估 pack/unpack 成本与纯内核收益，指导是否持久化 AoSoA。

6) 细节优化与长期方向
- AVX2/AVX-512：根据平台提供更宽 SIMD；对 AoSoA gather/scatter 做更优地址算术与对齐策略。
- 尾块与掩码：为避免尾零对统计/残差的潜在影响，继续确保只对有效索引 accumulate。
- 并行：在 AoSoA 路径下也可按块并行，减少 false sharing。

## 结论
- 设计目标达成度：
  - SoA：已达成（主路径）。
  - AoSoA：部分达成（仅距离约束；需要扩展到其他算子，并减少搬运）。
  - AoS：未达成（需新增视图与内核，或通过统一访问抽象适配）。
- 建议优先补齐 AoS 的最小可用支持，并将 AoSoA 覆盖扩展至附件/弯曲；随后引入布局无关抽象以统一维护成本与接口一致性。

