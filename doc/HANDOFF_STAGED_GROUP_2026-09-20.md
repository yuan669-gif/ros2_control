# FineMote × ROS 2 Humble：阶段化执行组交接文档（2026-09-20）

本文是 `HANDOFF_CURRENT_2026-09-20.md` 的**同日增补**，报告该文档“阶段 A/B”计划的实施结果。
旧文档继续作为历史记录保留；两者冲突时，阶段化执行组相关内容以本文和
`doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md` 为准。

## 一、本次推进的关键变化

`HANDOFF_CURRENT_2026-09-20.md` 的下一阶段计划是“阶段 A：ROS 适配接口”和
“阶段 B：ControllerManager 执行组接入”。本次已经实现并验证到：

- 新增 `controller_interface::StagedControllerInterface`：opt-in 双阶段接口，
  带每周期数据契约（值、周期、最旧采样时间、有效性、故障码）；
- 新增 `controller_manager::StagedExecutionGroup`：配置期解析拓扑和分配存储，
  运行期状态后序 + 命令前序 + 整组提交；
- `ControllerManager` 新增 opt-in API，并在 `update()` 中先运行执行组、再让执行组
  成员跳过原生 `update()`；
- 新增 `controller_manager/test/test_staged_execution_group.cpp`（6 个 gtest）
  与测试控制器 `test/test_staged_controller/`。

详细设计、边界和验证记录见 **`doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md`**。
本文只写状态、剩余缺口和下一步。

## 二、当前代码状态（本次新增/修改文件）

新增：

- **独立包 `hierarchical_control/`**（header-only，namespace `hierarchical_control`）：
  `package.xml`、`CMakeLists.txt`、`README.md`、
  `include/hierarchical_control/{hierarchy,staged_controller_interface,staged_execution_group}.hpp`、
  `test/test_execution_group.cpp`（9 个不依赖 `controller_manager` 的内核单测）
- `controller_manager/test/test_staged_controller/test_staged_controller.{hpp,cpp}`
- `controller_manager/test/test_staged_execution_group.cpp`
- `controller_manager/test/test_composite_controller/test_composite_controller.{hpp,cpp}`
- `controller_manager/test/test_composite_library/generic_composite_controller.{hpp,cpp}`
- `controller_manager/test/test_hierarchy_comparison.cpp`
- `doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md`
- `doc/HIERARCHY_FAIR_COMPARISON.md`
- `doc/WIRING_COST_ANALYSIS.md`
- `doc/PROJECT_REPORT_2026-09-21.md`（项目全貌报告，适合作为新读者/导师的入口）
- 本文

修改：

- `controller_manager/include/controller_manager/controller_manager.hpp`
  （引入执行组头文件、3 个公开 API、`staged_group_` 成员）
- `controller_manager/src/controller_manager.cpp`
  （实现 3 个 API；`update()` 运行执行组并跳过其成员；`manage_switch()` 之后刷新执行组的
  缓存活跃标志）
- `controller_manager/CMakeLists.txt` / `package.xml`
  （新增 `test_staged_controller`、`test_composite_controller`、`test_composite_library`、
  两个测试；依赖 `hierarchical_control`）
- `controller_manager/include/controller_manager/hierarchy.hpp`
  （改为兼容 shim，转发到 `hierarchical_control/hierarchy.hpp`）
- `controller_manager/include/controller_manager/staged_execution_group.hpp`
  （改为兼容 shim，using 转发到库）
- `controller_manager/include/controller_manager/hierarchical_controller_executor.hpp`
  （补充“已过时且不可用于硬件”的警告注释，未改行为）
- `research/cycle_tree/CMakeLists.txt`（补库 include 路径）

删除：

- `controller_interface/include/controller_interface/staged_controller_interface.hpp`
  （移入库；不留 shim，避免 `controller_interface` 与库之间的包依赖环）

未改动：`cycle_tree.hpp` 及其独立实验、原生 chaining 逻辑、`ControllerSpec`、
URDF 相关文件。

## 三、验证状态

本机（Ubuntu 22.04 / GCC 11.4.0 / ROS 2 Humble / colcon overlay）实际构建并运行：

- `colcon build --packages-up-to controller_manager hardware_interface_testing` 成功；
- `test_staged_execution_group` 6/6 通过（ctest 1/1）；
- `test_hierarchy_comparison` 3/3 通过（ctest 1/1）：同算法输出逐位一致、兄弟部分提交对照、
  分配测量、加叶配置化验证；
- `test_controllers_chaining_with_controller_manager` 通过（直接运行 5/5，全量 ctest 通过；
  重建后曾出现一次既有间歇失败，见实验文档第九节）；
- `test_cycle_tree_contract` 通过；
- 全量 `controller_manager` ctest 22 项中 19 项通过；3 项失败均为慢速虚机/服务发现问题，
  未走执行组分支，详见实验文档第九节。

内核抽取之后的复验（同一台机器）：

- `colcon build --packages-up-to controller_manager` 成功（7 个包）；
- `hierarchical_control` 自身的 9 个内核单测全部通过，**不创建 `ControllerManager`**：
  1000 次同周期与手写 baseline 逐项相等、相位顺序与单次调用、状态失败不进命令阶段、
  命令失败/NaN/sink 失败都不提交、复合节点不能重盖采样时间、采样年龄上限、
  库模式无需生命周期刷新、配置期/结构错误被拒；
- 6 个关键 ctest 全过：`test_cycle_tree_contract`、`test_controllers_chaining_with_controller_manager`、
  `test_hierarchy`、`test_urdf_hierarchy`、`test_staged_execution_group`、`test_hierarchy_comparison`；
- 非 ROS 的 standalone `research/cycle_tree` 构建 + CTest 1/1 通过（include 路径已补库目录）。

独立交付形态：`hierarchical_control` 可单独构建/测试（`colcon build/test --packages-select
hierarchical_control`），README 记录了契约、两种宿主、校验规则与非目标。

## 三之二、阶段 D 初步对照结果（详见 `doc/HIERARCHY_FAIR_COMPARISON.md`）

同一合成算法下比较原生 chaining、手写 composite、阶段化执行组：

- 三种实现每个周期输出**逐位相等**，且等于解析值；
- 两叶 fork 故障对照：原生 chaining 在 `leaf_a` 失败后仍更新 `leaf_b`，出现“一新一旧”；
  阶段化执行组让两个叶都保持上一周期值；
- 执行组自身 `run()` 在配置后 **100 次调用 0 次动态分配**（已断言）；
  但 `ControllerManager::update()` 整体仍有分配，来源是 Humble 既有的
  `ControllerSpec` 按值复制与 `get_current_state()`（`rclcpp_lifecycle::MutexMap` 内分配）；
- 非实时虚机上的 `update()` 中位耗时：手写 composite < 原生 chaining ≤ 阶段化执行组，
  执行组约为 composite 的 2 倍。**它不是更快的方案**。

接线成本（`doc/WIRING_COST_ANALYSIS.md`）：对已存在的 fork 拓扑，加一个叶在
staged/chained 下是 **1 行配置、0 行控制器代码**，并复用同一个叶控制器类；
现状 composite 插件必须改控制器代码。

**Gate B 已可下结论**：随后实现的通用 composite 库
（`test_composite_library/generic_composite_controller`）用**同一份内核**
（`StagedExecutionGroup::create_library`）在**单个普通插件**里托管整棵树，
不导出 chainable 接口、不改 manager，2/3 叶输出与执行组逐位相同，
惰性建内核后 `update()` 100 次 0 分配，加叶同样只需一个节点条目。

因此：

- “减少手写集成代码”**不是** manager 方案独有；
- 性能上 manager 方案也不占优（约 2 倍耗时）；
- 剩余真实差别是**组件可复用性/生命周期**（manager 宿主可让每个节点独立加载激活，
  库宿主把整棵树焊进一个插件）和**与原生 interface 的兼容性**（manager 宿主可与非 staged
  的普通 chainable 控制器混用），而不是接线成本。

按阶段 E 停止门槛，当前证据**不支持**继续扩大 `ControllerManager` 修改；
建议内核独立成库交付，manager 宿主作为“需要与原生 chaining 混用或节点独立生命周期”时的
可选集成方式保留。

不要把上面的结果表述为“优于原生 chaining”或“硬实时可用”。

## 四、与旧研究契约的关系

仍然成立（研究问题不变）：

> 在保留 Humble 原生 reference 依赖排序的基础上，引入显式状态阶段、周期有效性和命令组提交，
> 能否让可复用的复合控制组件完成同周期双向数据传播，并以可接受的接入和运行时成本避免陈旧或部分命令？

本次把这条问题从“独立标量内核”推进到“真实 `ControllerManager` + `ResourceManager`”。
仍然没有回答“成本是否可接受”和“是否优于 composite 基线”。

新增的明确边界（不要越界声称）：

- 依赖边来源于父控制器 claim 的子控制器 reference 接口前缀；URDF **没有**参与依赖推导；
- 每个节点必须至少导出一个 reference 接口，因为 Humble `configure_controller`
  要求 chainable 控制器导出非空 reference 接口；
- 执行组运行时要求所有成员同时 ACTIVE，且切换进行中不执行；
- 执行组是单线程读边界上的一致软件提交，不是总线原子同步，也不回滚已提交的兄弟叶节点；
- MRU/多频/异步/动态拓扑/生命周期顺序回滚均未实现。

## 五、下一步（按优先级）

### 阶段 C：依赖解析与 URDF 校验

- 把“从 `command_interface_configuration()` 前缀匹配成员名”的解析抽成独立函数并加测试，
  覆盖分支链、共享父节点、非成员前缀（普通硬件接口）；
- 显式 `state_children`/`parent` 元数据作为可选补充，并检查其与端口图一致；
- URDF 只校验 joint/link 存在性、接口存在性与机械锚点，不推断控制语义。

### 阶段 D：公平对照（已开始，未完成）

已完成：原生 chaining、手写 composite、阶段化执行组三种实现，同一算法、同一快照、
同一故障语义，输出一致性 + 兄弟部分提交 + 分配 + 耗时（见 `HIERARCHY_FAIR_COMPARISON.md`）。

仍未完成（优先级已因 Gate B 结论调整）：

1. **内核独立成库**：把 `StagedExecutionGroup` + `StagedControllerInterface` 从
   `controller_manager` 抽成独立包的公共接口，manager 宿主改为可选的薄适配层；
2. **更多拓扑变化**：插入中间层、叶换成子树、共享模块；
3. **方案 2（估计/命令显式拆分）** 实现，补齐第四个对照；
4. 在更稳定的环境重复耗时实验，报告 p50/p95/p99 与 deadline miss；
5. 消融：分别关闭“帧校验”和“整组提交”，测量各自耗时占比；
6. 库宿主的惰性建内核目前发生在第一次 `update()`，应改为 `on_activate` 后显式触发。

**Gate B 结论**：不继续扩大 `ControllerManager` 修改。如果要继续论文工作，
方向应是“内核库 + 两种宿主”的对比与工程化，而不是给 manager 增加更多层次化特性。

### 阶段 E：论文停止门槛

只有相对强基线显示出可重复的组合/诊断/失败一致性/接线成本优势时，才继续扩大
`ControllerManager` 修改；否则转为复合控制器库与静态绑定工具。

### 工程清理（可并行）

- 决定 `cycle_tree.hpp`（独立标量内核）与新执行组是否合并为单一内核，
  或在文档中明确二者是“契约验证内核”和“ROS 接入内核”；
- 处理 `hierarchical_controller_interface.hpp`、`hierarchy.hpp`、
  `controller_hierarchy_builder.hpp`、`urdf_hierarchy.hpp` 的定位，
  避免同一仓库里出现过时接口无人认领。

## 六、操作提示

- 本机工作区没有 `.git`。正式开发仓库仍在 Windows `D:/2027-1/FineMote/ros2_control`
  （origin 为用户 fork）。同步时按“本次新增/修改文件”列表拷贝，不要整体覆盖。
- 受限环境运行 gtest 需要可写日志目录：`export ROS_LOG_DIR="$PWD/log/ros"`。
- 源码放在工作区内部时，`build/`、`install/`、`log/` 都在工作区内部，便于复用增量编译。
