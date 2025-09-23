# HinaCloth 引擎设计目标达成度评估（2025-09-24，更新）

本评估基于当前仓库代码，对“数据与存储层支持 SoA/AoS/AoSoA 等内存布局进行计算”的设计目标进行核对与分析，并给出证据、差距与后续改进建议。该版本在原文基础上，结合最新实现状态（已补齐 AoS 运行时与 AoSoA 对 Attachment/Bending 的覆盖）进行更新。

## 摘要结论
- SoA（Structure of Arrays）
  - 达成：作为核心运行时数据布局与主要计算路径（位置、速度、惯量等字段以分量数组形式存储）。
- AoSoA（Blocked，结构化块化 SoA）
  - 达成：已实现 AoSoA 视图与打包/解包；距离、附件（attachment）、弯曲（bending）均提供 AoSoA 版本内核；运行时在 Blocked 模式下完整走 AoSoA 路径（帧内 pack→compute→unpack）。
- AoS（Array of Structures）
  - 达成（标量实现）：提供 AoS 视图/打包工具与距离/附件/弯曲 AoS 版本内核；运行时可显式选择 AoS 布局执行完整解算流程。

因此，“可支持 SoA/AoS/AoSoA 等实际内存布局进行计算”的目标现已达成（SoA 完整 + AoSoA 全算子 + AoS 最小可用路径）。

## 证据与代码定位（更新）

- 布局与后端枚举
  - `src/core/common/types.h` 中定义 `enum class DataLayout { Auto, SoA, AoS, Blocked }`。

- 数据层（核心状态）
  - `src/core/data/data.h/.cpp`
    - SoA：`x/y/z, vx/vy/vz, px/py/pz, inv_mass, ...` 为 `std::vector<float>`。
    - AoSoA：`pos_aosoa`（`std::vector<float>`）与 `layout_block_size`，旗标 `exec_layout_blocked`。
    - AoS：`pos_aos`（`std::vector<float>`）与 `layout_aos_stride`，旗标 `exec_layout_aos`。

- 存储层（Storage Views）
  - SoA：`src/backend/storage/soa.h/.cpp`（读写/axpy）。
  - AoSoA：`src/backend/storage/aosoa.h/.cpp`（视图 + SoA↔AoSoA pack/unpack，尾块填充）。
  - AoS：`src/backend/storage/aos.h` + `src/backend/storage/soa.cpp` 中的 SoA↔AoS pack/unpack。

- 运行时调度与内核选路
  - `src/runtime/step.cpp`
    - SoA 路径：`presolve_apply_attachment_soa` → `project_distance_islands_soa`（可选 AVX2）→ `bending_pass_soa`。
    - Blocked（AoSoA）路径：`storage_pack_soa_to_aosoa` → `presolve_apply_attachment_aosoa` → `project_distance_islands_aosoa` → `bending_pass_aosoa` → `storage_unpack_aosoa_to_soa`。
    - AoS 路径：`storage_pack_soa_to_aos` → `presolve_apply_attachment_aos` → `project_distance_islands_aos` → `bending_pass_aos` → `storage_unpack_aos_to_soa`。

- 距离约束内核
  - SoA 标量：`src/backend/kernel/constraints/distance.{h,cpp}`；SoA AVX2：`distance_avx2.{h,cpp}`。
  - AoSoA：`distance_aosoa.{h,cpp}`（含 AVX2 路径）。
  - AoS：`distance_aos.{h,cpp}`（标量）。

- 附件与弯曲内核（新增 AoSoA/AoS 完整覆盖）
  - Attachment：`attachment.{h,cpp}` 提供 SoA/AoSoA/AoS 三种实现。
  - Bending（实验特性）：`bending.{h,cpp}` 提供 SoA/AoSoA/AoS 三种实现。

- 后端选择逻辑
  - `src/backend/registry/registry.cpp`：`Auto` 下依据硬件选择 Backend，并为 AVX2 默认 Blocked 布局（可被显式布局覆盖）。
  - `src/adapter/engine_adapter.cpp`：将选择结果写回 `Data`，并据此分配 AoSoA/AoS 缓冲。

- 测试与示例（覆盖多布局路径）
  - `test/test_main.cpp` 含 SoA/AoS/Blocked 路径的收敛性与功能测试；全部通过。
  - `cmake-build[-release/-debug]/example_ex0*` 示例可运行，演示策略选择与布局效果。

## 行为与影响
- AoSoA：在 Blocked 模式下，Attachment/Distance/Bending 均走 AoSoA 版内核；仍采用帧内子步级别的 SoA↔AoSoA 往返（后续可评估“子步内常驻”以减少搬运）。
- AoS：提供最小可用标量实现，适合作为与外部 AoS 数据交互的参考路径与一致性基线。
- AVX2：SoA 与 AoSoA 路径具备 AVX2 支持；AoS 目前为标量实现（符合“最小可用”目标）。

## 与设计目标的差距（已收敛与仍可优化）
1. 布局统一的访问抽象（待优化）
   - 当前仍以 SoA/AoSoA/AoS 三套内核分别实现；可进一步通过模板/概念抽象统一视图接口，减少重复逻辑。
2. AoSoA 常驻策略（待评估）
   - 现为子步内 pack→compute→unpack；可探索“帧内 AoSoA 常驻”或多字段 AoSoA 化，进一步降低搬运开销。
3. 遥测与确定性增强（按路线图推进）
   - 增加 per-phase/批耗时、岛屿统计；并行调度的 Level 1 确定性与固定分块策略。

## 改进建议（下一步）
- 引入轻量的布局无关访问抽象（C++23 concepts 或 traits），在不牺牲性能的前提下降重。
- 评估 AoSoA 常驻的收益与实现复杂度，提供策略开关与 Telemetry 对比。
- 持续完善 AVX2 路径（如掩码尾块处理）与并行任务划分，配合遥测可观测性。

## 结论（更新）
- SoA：达成。
- AoSoA：达成（三算子均覆盖），后续可优化常驻策略。
- AoS：达成（标量路径），作为一致性与互操作支持。

上述状态已通过本仓库测试用例验证（Release，VS2022 生成器）。
