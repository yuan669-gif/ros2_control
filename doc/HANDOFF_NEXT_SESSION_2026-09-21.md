# 下一会话交接：课题重定位与决定性实验

> ⚠ **本文已被 `doc/HANDOFF_MANUAL.md` 取代（2026-09-21）。请读那份，不要读这份。**
> 本文保留仅作历史追溯：其中第 5 节的「下一步」、第 6 节的「待写 WHY_NO_SPEEDUP」等
> 多数条目已经完成或状态已变。

日期：2026-09-21
状态：阶段 A/B/D 与 Gate B 已完成并验证；**课题叙事已重新定位**；决定性实验待做。
读本文即可接手，不需要读完整对话历史。

---

## 0. 一页速览

**课题**：把 FineMote 的双向两阶段调度思想用到 ROS 2 `ros2_control` 的控制器层次上。

**已交付**（可运行、有测试）：

- 独立库包 `hierarchical_control`（header-only，宿主无关的两阶段内核 + 数据契约）；
- `controller_manager` 薄适配层（3 个 API + `update()` 跳过成员 + 切换后刷新活跃标志 + 兼容 shim）；
- 库宿主基线 `test_composite_library/generic_composite_controller`（单插件托管整棵树，零 manager 改动）；
- 对照实验 `test_hierarchy_comparison`（输出一致性、故障一致性、分配、耗时、接线成本、库 vs 组）；
- 文档 6 份（见第 6 节）。

**已确立的诚实结论**：

| 结论 | 证据 |
|---|---|
| 三种实现输出逐位一致 | `test_hierarchy_comparison` |
| 原生 chaining 在某叶失败后**仍更新健康兄弟**（一新一旧）；本方案不会 | 两叶 fork 故障测试 |
| 执行组运行路径**零分配**（配置后，两种宿主均验证） | 全局 `operator new` 计数，100 次调用 = 0 |
| **不更快**：中位耗时约为手写 composite 的 2 倍 | 200 周期 ×3 次 |
| **省集成代码不是 manager 独有**：库宿主同样做到"加一个节点 = 一条配置" | `library_host_matches_staged_group` |
| 因此 **Gate B 不成立，不应继续扩大 `ControllerManager` 修改** | `doc/WIRING_COST_ANALYSIS.md` |

**关键风险（必须处理）**：上游 `ros2_control` 已经有 state chaining 与 state 感知排序。
我们的 HUMBLE 实现本质上是**回移**，不是新机制。详见第 2 节。

**重新定位后的研究问题**（第 3 节）：

> 在 `ros2_control` 的单入口控制器模型下，同周期双向数据流（父读子状态、子用父参考）
> 是否可满足？若不可满足，显式两阶段契约的运行时与集成代价是多少，换来哪些可测量的正确性收益？

**决定性实验已完成**（第 5 节）：双向边在上游 master `b0c14b5` 下**不报错**，
而是按"先插入者胜"静默降级为**载荷顺序相关的一周期延迟**（哪条边陈旧取决于加载顺序），
三级链上出现逐边混合的陈旧模式。主命题"单入口下双向同周期不可满足"成立。
**尚需在 Jazzy/Rolling 上跑真实代码验证**（第 5 节末），在此之前不能说"上游有 bug"。

**当前剩余风险**：若上游文档禁止"同一控制器对同时有 reference 与 state 边"，
命题需改成"该组合被禁止，从而限制级联表达力"——仍有效，但叙事要调整。

**第 1 条已基本闭环（无需配环境）**：Jazzy 的 `controller_manager.cpp` 里
`controller_sorting` 出现 **0 次**，`build_controllers_topology_info` /
`update_list_with_controller_chain` 出现 **6 次** ⇒ Jazzy 已换成 master 那套递归插入启发式；
逐字节比较（去注释/空白）两个函数 **jazzy_len == master_len，identical=True**。
所以 `BIDIRECTIONAL_EDGE_ANALYSIS.md` 的移植结论**直接适用于 Jazzy**。
仍需（非必需）的只有"数据通路级"复现；本机 Ubuntu 22.04 且无 Docker，装不了 Jazzy/Rolling。

**第 2 条已完成**（`CONTROL_COST_OF_LAG.md` 第 8 节）：摩擦与量化**不改变**深度代价
（kd_max 差异 <0.1%）；实测 `kd_max(0):(1):(2):(3) = 1:0.502:0.311:0.225` ≈ 理论 `1:1/2:1/3:1/4`；
**执行器饱和会把发散变成有界极限环**（kd=3000、lag=0 线性发散但饱和下有界）——
"不炸了 ≠ 能工作"，在饱和系统上做实验必须同时报告跟踪误差。

**第 3 条（Gazebo 案例研究）已完成第一轮**（`doc/GAZEBO_CASE_STUDY.md`，代码在 `case_study/`）：
两轮差速底盘 + `gazebo_ros2_control` + 三个自研控制器（叶发布不可重推导的轮行程估计，
根做航位推算并回写轮速参考），管理器 YAML 参数 `two_phase_execution` 切换模式。
**核心结果**：真实仿真闭环里测得的**状态边陈旧量 单趟=1 周期、两趟=0 周期**（两趟 2/2 复现），
与定理 1/推论 1/定理 2 一致。
两个方法论坑已记录：① 叶状态必须真正**不可被父重推导**（第一版用 `∫v·r dt` 可被
`position×r` 重建，等于选错了被控对象）；② PI 需积分/输出限幅，否则 Gazebo 求解器发散。
**明确不声称**轨迹误差收益：真值指标尚不可靠，且该底盘回路带宽仅 ~0.5 Hz，
按相位裕度定律一个周期只损失 ~3.6°，预期效应本就很小。Gazebo 在本 VM 上约 1/3 启动崩溃。

**第 3 条（Gazebo）可行性已确认**：本机已装 Gazebo Classic + `gazebo_ros2_control`
（`/usr/bin/gazebo`、`ros2 pkg list` 有 `gazebo_ros2_control`），下一步可做。

**Humble 真实代码验证已完成**（`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` 第 9 节）：
新增 `controller_manager/test/test_upstream_ordering.cpp`（2 用例通过）证明双向对在 Humble
真实比较器下**总是 reference-first、与注册顺序无关** ⇒ state 边系统性陈旧（推论 1 的真实代码验证，
非移植）。额外发现：`configure_controller` **完全不校验**声明的 state 边，因此冲突完全静默。
注意 Humble 与 master 的差异：Humble 是确定的 reference-first，master 会随加载顺序翻转。

**控制代价已量化**（`doc/CONTROL_COST_OF_LAG.md`）：滞后 `L` 周期 = 纯传输延迟 `L·Δt`，
相位裕度损失 `ω_c·L·Δt`；`Δt=1 ms`、95 Hz 穿越频率下 PM 从 90°→55.6°(L=1)→21.2°(L=2)→
−13.1°(L=3，时间域确认发散)；低带宽（9.5 Hz）下 L=3 仍有 79.5°，几乎无影响。
工程结论：**30° 裕度预算下，深度 2 把闭环带宽压到 42 Hz、深度 4 压到 21 Hz**。
方法学教训：RMS 追踪误差在该场景**不敏感**（比值 0.97–1.00），必须用相位裕度这类能暴露
纯延迟的指标。

**形式化已完成**（`doc/FORMAL_MODEL.md`）：模型 + 定理 1（单趟不可满足）+ 推论 1 +
定理 2（滞后 = 深度 D，有下界与构造）+ 定理 3（两趟充分性，一个线性化、O(|V|) 存储）+
推论 2（配置期无环检查）。定理 1 已由**穷举全部 6 种执行顺序**的测试验证（无一种两向都新鲜）。
反例防护核验：**未发现**上游禁止"同一控制器对同时有 reference 与 state 边"——
应表述为"该组合未被覆盖"，不是"上游禁止"（issue #2189 只澄清术语）。

**管理器级两趟已实现并验证**（第 4.6 节）：`set_two_phase_execution(true)` 让
`ControllerManager::update()` 反向跑 `update_phase()`、正向跑 `handle_phase()`。
真实管理器实测：单趟下 root 的信息陈旧 2 个周期（=深度），两趟下全 0。
**过程中发现一条硬约束**：`switch_controller()` 持锁等待实时线程，实时路径**绝不能**获取
`controllers_lock_`，否则死锁（详见第 4.6 节与陷阱 10）。

**两趟 vs 单趟已定量证明**（`doc/TWO_PASS_VS_SINGLE_PASS.md`）：把 FineMote 的
"一个线性顺序、正向 Update / 反向 Handle" 实现在内核里（命令阶段改为反向遍历 `postorder`，
不再需要 `preorder` 数组），并用阶跃响应实验证明：

| 调度器 | 上行阶跃 lag | 下行阶跃 lag |
|---|---|---|
| 单趟、父先（ros2_control 现状方向） | **depth − i**（根最陈旧） | 0 |
| 单趟、子先 | 0 | **i**（叶最陈旧） |
| **两趟、同一顺序** | **0** | **0** |

即：**单趟必然有一个方向陈旧且正比于深度；两趟两个方向都同周期，且复用同一份线性化
（不需要第二份拓扑顺序，存储量级不变）。**
内核单测 10/10 通过。

---

## 1. 当前代码状态

### 1.1 包结构

```text
hierarchical_control/                      # 独立 header-only 包，namespace hierarchical_control
  package.xml / CMakeLists.txt / README.md
  include/hierarchical_control/
    hierarchy.hpp                          # 计划生成 + 校验（纯 C++，可脱离 ROS）
    staged_controller_interface.hpp        # 双阶段接口 + StagedFrame 数据契约
    staged_execution_group.hpp             # 内核：两阶段 + 帧校验 + 整组提交
                                           #   create()         = manager 宿主
                                           #   create_library() = 库宿主
  test/test_execution_group.cpp            # 9 个内核单测，不创建 ControllerManager

controller_manager/                        # 薄适配层
  include/controller_manager/controller_manager.hpp   # 3 个公开 API + staged_group_
  src/controller_manager.cpp                          # 实现 + update() 接入（唯一改动的核心函数）
  include/controller_manager/hierarchy.hpp            # 兼容 shim（using 转发到库）
  include/controller_manager/staged_execution_group.hpp  # 兼容 shim
  test/test_staged_controller/             # 应用控制器（staged 模式 + native 单相模式）
  test/test_composite_controller/          # 基线：手写 composite
  test/test_composite_library/             # 基线：通用 composite 库宿主
  test/test_staged_execution_group.cpp     # 6 个 manager 级验收测试
  test/test_hierarchy_comparison.cpp       # 5 个对照测试

research/cycle_tree/                       # 前序：不依赖 ROS 的独立标量内核构建入口
doc/                                       # 见第 6 节
```

依赖方向：`controller_manager → hierarchical_control → controller_interface`（无环）。

### 1.2 代码量（非空非注释行）

| 项 | code lines |
|---|---|
| 内核：接口 125 + 执行组 447 + hierarchy 121 | 693 |
| 库单测 | 386（9 tests） |
| 应用控制器（两宿主共用） | 443 |
| 基线：手写 composite | 181 |
| 基线：库宿主 | 371 |
| manager 测试 / 对照测试 | 249（6）/ 713（5） |

### 1.3 验证命令（全部实际跑通过）

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"          # 必需：受限环境下 ~/.ros 不可写会导致 gtest 崩溃

colcon test --packages-select hierarchical_control --output-on-failure
ctest --test-dir build/controller_manager \
  -R "test_staged_execution_group|test_hierarchy_comparison|test_controllers_chaining_with_controller_manager|test_cycle_tree_contract|test_hierarchy$|test_urdf_hierarchy" \
  --output-on-failure

# 非 ROS 的独立内核
cmake -S research/cycle_tree -B build_standalone -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_standalone -j4 && ctest --test-dir build_standalone --output-on-failure
```

最近一次结果：库 ctest 1/1（9 gtest）；controller_manager 关键 6/6；standalone 1/1。
全量 controller_manager ctest 22 项中 19 项通过；3 项失败均为慢速虚机/服务发现问题
（`test_controller_manager_srvs` 因 `list_large_number_of_controllers_with_chains` 单例 128s 超时、
`test_spawner_unspawner` 超时、`test_hardware_spawner` flaky），均不走执行组分支。

---

## 2. 必须知道的上游事实（2026-09-21 核实）

这些直接决定课题定位，**写论文前必须读完并正面引用**：

1. **上游已导出 state interface**。Jazzy 的 `ChainableControllerInterface` 有
   `export_state_interfaces()`（返回 `std::vector<StateInterface::ConstSharedPtr>`）、
   `on_export_state_interfaces()`、`on_export_state_interfaces_list()`。
   链接：<https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ChainableControllerInterface.html>
2. **上游排序已考虑 state 链**。issue #1123 “Deal with the state interface chaining in the sorting
   algorithm” 由 saikishor 于 2023-10-01 提出，**2026-04-11 以 completed 关闭**；
   更早的 commit `69b3225` “update controller_sorting logic to deal with chainable controller that
   export states” 已经把 `get_following_controller_names` / `get_preceding_controller_names` /
   `controller_sorting` 改成考虑 `state_interface_configuration().names`。
   链接：<https://github.com/ros-controls/ros2_control/issues/1123>、
   <https://github.com/ros-controls/ros2_control/commit/69b322528be7b290e80908518cabebaad018d3a3>
3. **上游已缓存 lifecycle id**。Jazzy `ControllerInterfaceBase::get_lifecycle_id()` 文档写明
   “cached internally to avoid calls to `get_lifecycle_state()` in the real-time control loop”。
   我们“`get_current_state()` 会分配”的发现只对 **Humble** 成立，不是新发现。

**结论**：不能声称“我们首次用接口连接生成状态依赖/层次排序”。Humble 版本是回移。

---

## 3. 重新定位后的研究问题

**旧叙事（已放弃）**：给 `ros2_control` 引入层次化调度与接口依赖图。
→ 上游已做，不成立。

**新叙事（建议采用）**：

> 在 `ros2_control` 的单入口控制器模型（每个控制器一个 `update()`）下，
> **同周期双向数据流不可满足**：同一对父子若要同时满足
> “父在本周期读到子刚算出的状态”（⇒ 子必须先跑）
> 与“子在本周期用到父刚产生的参考”（⇒ 父必须先跑），
> 则要求矛盾。上游的 state chaining 与 reference chaining 各自只解决**单向**场景
> （估计器→控制器；控制器→执行器），**双向场景未被覆盖，且在单入口下无法覆盖**。
> 本文给出显式两阶段契约作为修复，量化其运行时/集成代价，
> 并给出上游不具备的可靠性契约（每周期有效性、最旧采样、整组提交不部分提交）。

**支持新叙事的本地证据**（已有）：

- Humble `controller_sorting` 末尾已有“若 ctrl_a 的 state 接口来自 ctrl_b，则 ctrl_b 排在 ctrl_a 前”
  的分支，旁边留着 `TODO(saikishor): deal with the state interface chaining in the sorting algorithm`
  （`controller_manager/src/controller_manager.cpp` 约 2677 行）。当同一对控制器同时满足
  reference 边 a→b 与 state 边 b→a 时，比较器收到**互相冲突**的约束。
- 两叶 fork 故障测试证明：原生 chaining 在某个叶失败后仍更新健康兄弟，产生“一新一旧”。
  上游 state chaining 不改变这个行为。

**代价一侧的证据（负结果，不要隐藏）**：不更快（≈2× composite），
省集成代码不独有（库宿主同价），管理整体 `update()` 仍分配（Humble 的 `ControllerSpec` 按值复制）。

---

## 4. 该保留 / 该清理

### 保留

- `hierarchical_control/` 整个包（主交付物）；
- `controller_manager` 的三个 API + `update()` 接入 + 两个 shim；
- `test_staged_controller`、`test_composite_controller`、`test_composite_library`、
  `test_staged_execution_group.cpp`、`test_hierarchy_comparison.cpp`；
- `test_hierarchy.cpp`（现在测的是库的 `build_controller_hierarchy`，仍有效）；
- `cycle_tree.hpp` + `test_cycle_tree_standalone.cpp` + `research/cycle_tree`（前序标量内核证据）；
- `controller_manager/hierarchy.hpp` shim（`cycle_tree.hpp` 依赖）；
- 文档第 6 节列出的全部。

### 已完成的清理（2026-09-21）

以下 v1 文件已删除，`controller_manager/CMakeLists.txt` 与 `controller_interface/CMakeLists.txt`
中对应测试目标已移除，并已重新构建验证：

- `hierarchical_controller_executor.hpp` + `test_hierarchical_controller_executor.cpp`（已知 bug，不可用于硬件）
- `hierarchical_controller_interface.hpp` + `test_hierarchical_controller_interface.cpp`（v1 接口，无数据契约）
- `controller_hierarchy_builder.hpp` + `test_controller_hierarchy_builder.cpp`（URDF 推断控制依赖，已否决方向）
- `urdf_hierarchy.hpp` + `test_urdf_hierarchy.cpp`（同上）

保留：`controller_manager/hierarchy.hpp` shim（`cycle_tree.hpp` 依赖）、
`test_hierarchy.cpp`（现在测库的 `build_controller_hierarchy`）。

### 原清理清单（供核对）

| 文件 | 为什么清 |
|---|---|
| `controller_manager/include/controller_manager/hierarchical_controller_executor.hpp` | 已知 bug：状态阶段失败后仍继续命令阶段；已被 `StagedExecutionGroup` 取代。**不可用于硬件**。 |
| `controller_manager/test/test_hierarchical_controller_executor.cpp` | 同上，只测这个坏实现 |
| `controller_interface/include/controller_interface/hierarchical_controller_interface.hpp` | v1 接口，无数据契约，已被 `hierarchical_control::StagedControllerInterface` 取代 |
| `controller_interface/test/test_hierarchical_controller_interface.cpp` | 同上 |
| `controller_manager/include/controller_manager/controller_hierarchy_builder.hpp` | 用 URDF 关节祖先**推断控制依赖**——这是研究已明确否决的方向（URDF 不能推断控制语义） |
| `controller_manager/test/test_controller_hierarchy_builder.cpp` | 同上 |
| `controller_manager/include/controller_manager/urdf_hierarchy.hpp` | 同上（URDF 计划生成） |
| `controller_manager/test/test_urdf_hierarchy.cpp` | 同上 |

清理时同步：`controller_manager/CMakeLists.txt`、`controller_interface/CMakeLists.txt` 中对应的
`ament_add_gmock` / `target_link_libraries`。删除后必须重跑第 1.3 节的构建与测试。

> 注意：`cycle_tree.hpp` 通过 `controller_manager/hierarchy.hpp` shim 使用库的
> `build_controller_hierarchy`；不要删 `hierarchy.hpp` shim。

---

## 4.5 内核改动：单一线性顺序 + 两趟（2026-09-21 已完成并验证）

`hierarchical_control/include/hierarchical_control/staged_execution_group.hpp`：

- 状态阶段：`plan_.postorder` **正向**（子先于父）；
- 命令阶段：**同一顺序反向**（父先于子），不再使用 `plan_.preorder`；
- 依据：后序的反转是合法的"父先于后代"顺序；
- `hierarchy.hpp` 仍保留 `preorder` 字段（`cycle_tree` / v1 兼容），但两阶段内核不再使用它。

新增内核单测 `one_pass_can_keep_only_one_direction_same_cycle`（见 `doc/TWO_PASS_VS_SINGLE_PASS.md`）。

> **重要**：改的是库头文件，因此 `controller_manager` 全量重编（约 16 分钟）。
> 改动后必须重跑第 1.3 节的全部测试。

## 4.6 管理器级两趟（2026-09-21 已完成并验证）

新增：

- `hierarchical_control/include/hierarchical_control/two_phase_controller_interface.hpp`
  —— 轻量 mixin：`update_phase()` / `handle_phase()`，**没有**组/端口/帧/组提交；
- `ControllerManager` 公开 API：`set_two_phase_execution(bool)` / `two_phase_execution()`；
- 测试控制器 `TestStagedController` 实现该 mixin（一阶滤波级联，估计不可被父重推导）；
- 测试 `controller_manager/test/test_two_phase_execution.cpp`（2 用例）。

`ControllerManager::update()` 的形态：

```text
Pass 1 (Update)  : 反向遍历控制器列表 → update_phase()   （子先于父）
legacy 循环      : 跳过实现该 mixin 的控制器
Pass 2 (Handle)  : 正向遍历同一列表   → handle_phase()   （父先于子）
切换挂起时两趟都不跑
```

实测（真实 `ControllerManager`，同一份控制器代码）：

| 模式 | leaf lag | mid lag | root lag |
|---|---|---|---|
| 单趟（原生 `update()`） | 0 | **1** | **2** |
| **两趟（opt-in）** | **0** | **0** | **0** |

### ⚠ 发现的硬约束（必须遵守，否则死锁）

`ControllerManager::switch_controller()` 在**持有 `controllers_lock_`** 的同时等待实时线程应用
切换（"lock controllers" 之后的 `lock_guard` 一直存活到 `switch_params_.cv.wait_for(...)`）。
因此**实时路径上任何获取 `controllers_lock_` 的代码都会死锁**：
实时线程阻塞在锁上 → 不推进 `used_by_realtime_controllers_index_` →
等待线程在 `wait_until_rt_not_using()` 里永久自旋。

正确做法（已实现）：

- 非实时线程（`set_two_phase_execution`）可以加锁刷新；
- 实时路径只用**脏标志**：切换后置 `two_phase_entries_dirty_ = true`，
  在**下一个周期开头**用本线程已持有的 `rt_controller_list` 重建（O(n) `dynamic_cast`，
  每个切换最多一次），**全程不加锁**。

这也解释了为什么 staged group 的 `refresh_member_active_state()` 特意不加锁。
**任何接入 `ControllerManager` 实时循环的新机制都必须遵守。**

## 5. 决定性实验：**已完成**（结论见 `doc/BIDIRECTIONAL_EDGE_ANALYSIS.md`）

上游版本锁定：`ros-controls/ros2_control` master HEAD `b0c14b5`（2026-09-17）。

**结论**：上游 `build_controllers_topology_info()` 把 reference 边和 state 边分别写入
`following_controllers` / `preceding_controllers`。对同一对 (A,B) 同时存在两种边时，
`A.following={B}, A.preceding={B}, B.following={A}, B.preceding={A}`，顺序图出现 2-环。
而顺序构造 `update_list_with_controller_chain()` 的第一行是
`if (already in ordered) return;`——**先插入者胜，冲突被静默丢弃**。

移植验证（`research/topology_analysis/upstream_order_sim.py`）：

- 加载顺序 `[A,B]` → 运行顺序 `[A,B]` → reference 新鲜、**state 落后一周期**；
- 加载顺序 `[B,A]` → 运行顺序 `[B,A]` → state 新鲜、**reference 落后一周期**；
- 三级链上出现**逐边混合**的陈旧模式，而不是可预测的全局延迟；
- **全程无 warning、无 error。**

即：双向边不是"冲突报错"，而是**载荷顺序相关、无诊断的一周期延迟降级**。
这与"单入口 `update()` 下双向同周期不可满足"的论证一致，主命题成立。

### 下一个窗口（管理器级两趟已完成，列后续）

1. **反例防护（最高优先）**：确认上游文档/PR 是否禁止"同一控制器对同时有 reference 与 state 边"。
   若禁止，命题改为"该组合被禁止，从而限制级联表达力"。
2. ~~Humble 真实代码验证~~ **已完成**（`test_upstream_ordering.cpp`）。
   仍待做：**Jazzy/Rolling** 上的数据通路级复现（Humble 不导出 state interface，
   无法验证"父读到陈旧值"）。
3. **把 lag 换算成控制性能**：目前是周期数（离散、确定性）。要在带真实动力学的控制器上
   测相位滞后 / 跟踪误差，才算性能论证。
4. **显式父子字段接到配置**：内核 `Spec::parents` 已有，但管理器侧仍从 reference 边推导。
   若要支持"接口无法表达依赖"的组合或诊断，需要 YAML/参数入口。
   注意：两趟之后该字段**不是必需**（reference 边的一个线性化就够）。
5. **实测两趟的额外开销**：多一次列表遍历 + 每控制器一次虚调用，应与单趟对比并如实报告。
6. 单独验证两趟模式下的**故障一致性**（目前只有 staged group 有该证据）。

### 仍需在 Jazzy/Rolling 上做

移植是逻辑级证据，不是运行上游代码。必须补：

1. 在 Jazzy/Rolling 写最小双向控制器对（A claim `B/ref` 与 `B/state`，B 导出两者），
   用两种加载顺序启动，读实际执行顺序（`list_controllers` 的 chain 信息 / CM 日志），
   确认顺序随加载顺序翻转且无告警；
2. 让 B 的 state 依赖其**内部状态**（一阶滤波/积分），测 A 看到的 `B/state` 滞后周期数，
   以及由此产生的**跟踪误差 / 相位滞后**；
3. 对照组用本项目两阶段执行组跑同一算法，证明两条边同周期、误差为 0；
4. **反例防护**：确认上游是否允许同一控制器对内同时存在 reference 与 state 边。
   若文档禁止该组合，命题要改为"该组合被禁止，从而限制了级联表达力"——仍是有效结论，
   但叙事必须相应调整。

**这一步做完之前，不要把"上游有 bug"写进论文**；目前只能说"上游在双向场景下静默降级"。

---

## 6. 文档索引与各自作用

| 文档 | 作用 |
|---|---|
| **`doc/GAZEBO_CASE_STUDY.md`** | **Gazebo 案例研究**：真实仿真闭环中的状态边陈旧（单趟 1 周期 vs 两趟 0 周期）、两个方法论坑、明确不声称项、复现命令 |
| **`doc/CONTROL_COST_OF_LAG.md`** | **工程意义**：滞后→传输延迟→相位裕度损失→带宽上限；含闭式验证、时间域确认、限制 |
| **`doc/FORMAL_MODEL.md`** | **形式化核心**：模型、定理 1–3、推论 1–2、与实现/实验/上游的对应、threats to validity、可声称/不可声称 |
| **`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md`** | **决定性实验**：上游双向边行为、移植验证、方法学限制、下一步实验设计 |
| **`doc/TWO_PASS_VS_SINGLE_PASS.md`** | **两趟 vs 单趟**：FineMote 机制的内核实现 + 阶跃响应定量证明 |
| `doc/HANDOFF_NEXT_SESSION_2026-09-21.md` | **本文**，下一会话入口 |
| `doc/PROJECT_REPORT_2026-09-21.md` | 项目全貌报告（给导师/新读者）。**注意：第 3 节“接口连接是依赖来源”的叙事已过时，需按本文第 2/3 节修订** |
| `doc/WHY_NO_SPEEDUP.md` | （待写）解释负结果与 FineMote 主张不矛盾：测的是 CPU 时间，FineMote 主张的是控制延迟；且我们的算例状态可被父重推导 |
| `doc/WIRING_COST_ANALYSIS.md` | Gate B 接线成本 + 通用 composite 库基线 + 最终结论（含包结构） |
| `doc/HIERARCHY_FAIR_COMPARISON.md` | 同算法对照（输出/故障/分配/耗时）与诚实结论 |
| `doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md` | 阶段 A/B 设计与 manager 级验证记录 |
| `doc/HANDOFF_STAGED_GROUP_2026-09-20.md` | 上一轮交接（文件清单、验证状态） |
| `doc/HANDOFF_CURRENT_2026-09-20.md`、`doc/HANDOFF_2026-09-19.md`、`doc/hierarchical_research.md` | 更早的研究契约（历史） |
| `doc/CYCLE_TREE_EXPERIMENT.md` | 前序独立标量内核实验 |
| `hierarchical_control/README.md` | 库的对外文档（契约、两种宿主、校验规则、非目标） |
| **本文** | 下一会话入口 |

---

## 7. 已知陷阱与注意事项

1. **`ROS_LOG_DIR`**：受限环境下 `~/.ros` 不可写，gtest 会在 `SetUpTestSuite` 抛异常并在退出时
   崩溃（现象是 `rclcpp::shutdown()` 段错误）。必须 `export ROS_LOG_DIR="$PWD/log/ros"`。
2. **`/tmp` 在部分执行环境中按调用隔离**，构建产物要放在工作区内（用默认 `build/`、`install/`）。
3. **全量重编 `controller_manager` 约 16 分钟**；只改测试文件时增量约 1 分钟。
4. **工作区没有 `.git`**。正式仓库在 Windows `D:/2027-1/FineMote/ros2_control`
   （origin 为用户 fork）。同步按文件清单拷贝，不要整体覆盖。
5. **`members_active_` 缓存**：只在 `manage_switch()` 之后刷新。控制器不经 switch 自行改变
   生命周期状态会让缓存过期（已写入限制）。
6. **库宿主在第一次 `update()` 惰性建内核**（一次性分配）；应改为 `on_activate` 后显式触发。
7. **`test_controllers_chaining_with_controller_manager` 是既有 flaky**：负载下偶发失败
   （计数偶发偏差），隔离重跑通过。清理后一次 ctest 中它失败，随后单独重跑 3/3 通过；
   本次改动没有触碰原生 chaining 逻辑。做公平对照前应先定位它。
8. **内核命令阶段已改为反向遍历 `postorder`**：若看到 `plan_.preorder` 仍被赋值，属正常
   （`hierarchy.hpp` 兼容保留），但两阶段内核不再使用它。
9. **实时路径禁止获取 `controllers_lock_`**（见第 4.6 节）：`switch_controller()` 持锁等待实时
    线程，从实时线程抢锁会死锁，且表现为 `wait_until_rt_not_using()` 永久自旋。用脏标志 +
    下个周期开头用 `rt_controller_list` 重建。调试提示：rclcpp 装了 SIGTERM 处理器，
    挂死时 `timeout` 默认杀不掉，要用 `timeout -s KILL`。
10. **`controller_interface/staged_controller_interface.hpp` 已删除**（移入库，不留 shim，
   否则形成包依赖环）。实现该接口的插件现在依赖 `hierarchical_control`。

---

## 8. 明确的"不能声称"清单（论文写作时逐条核对）

- 首次提出层次化控制 / 树遍历 / 两阶段调度；
- 首次用接口连接生成状态依赖（上游已做：PR #1021、issue #1123、commit 69b3225）；
- 比原生 chaining 或手写 composite 更快；
- 接入后 `ControllerManager::update()` 零分配；
- 已验证硬实时 WCET / deadline miss；
- 仅凭 URDF 推断控制语义；
- 已支持多频、异步、动态拓扑、生命周期回滚。
