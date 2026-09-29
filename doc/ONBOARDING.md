# 从零了解这份工作：入口文档

> 这份文档写给**完全不了解背景的人**。目标是你读完它知道：这个仓库里的 `humble-work` 分支在做什么、
> 三个核心概念（双向调度、阶段化执行、编译期元编程）到底解决什么问题、哪些结论是实测的、哪些**不能**说，
> 以及接下来该按什么顺序读代码和文档。
>
> 阅读顺序建议：本文件 → `CODEBASE_TOUR.md`（读源码）→ `TESTING_GUIDE.md`（怎么跑、怎么复现）→
> `DEVELOPMENT_HISTORY.md`（经过与踩坑）。深入某个主题时再跳到 `doc/` 下的专题文档。
>
> 仓库：分支 `humble-work`，fork `https://github.com/yuan669-gif/ros2_control`（上游
> `ros-controls/ros2_control` 的 Humble 副本）。所有本项目的改动都在这个分支上，
> `git diff --stat 469f3055..HEAD` 可以看到全貌（`469f3055` 是 Humble 基线，见 `GIT_WORKFLOW.md`）。

---

## 1. 三句话版

1. **问题**：在 ros2_control 里做级联控制（chassis → wheel → tire）时，父子之间存在**双向依赖**——
   父要在同一周期用子的**新状态**，子要在同一周期用父的**新参考**。上游的单趟 `update()` 每次只沿一个
   方向遍历，所以**必然有一个方向拿到上一周期的数据**（滞后 = 级联深度）。
2. **做法**：在**同一份线性化顺序**上跑**两趟**（反向 `update_phase` + 正向 `handle_phase`），两向依赖同时
   满足；把"什么叫新鲜、失败怎么算、半棵树不允许提交"写成显式契约；再用**元编程**把拓扑、端口、量纲和硬件
   接口需求搬到编译期，让不合法配置**编译不过**而不是运行期才报。
3. **诚实定位**：这是把 FineMote（arXiv:2608.04600v1，SJTU，2026-08-05，见 `RELATED_WORK.md` §1.1）的
   两阶段机制思想**做成可验证工程**并落到
   ros2_control Humble 上的一套实现；**不**声称更快（两趟 CPU 更贵）、**不**声称论文级新理论
   （两阶段机制本身不新；本工作的价值在形式化命题 + 代价量化 + 可靠性契约 + 工程落地）。

---

## 2. 十分钟版：到底解决什么问题

### 2.1 双向依赖

级联控制里每个节点同时是"消费者"和"生产者"：

```
chassis ──(reference)──▶ wheel ──(reference)──▶ tire
   ▲                                               │
   └────────────────(state)────────────────────────┘
```

- **自顶向下**：chassis 每周期算出的参考速度要立刻给 wheel 用；
- **自底向上**：tire 本周期的实际行程要立刻给 wheel、再给 chassis 用。

上游 ros2_control 的执行是"按控制器列表顺序遍历一次，每个控制器 `update()` 一次"。一次遍历里，
如果父先于子执行，父读到的是子**上一周期**的状态；如果子先于父，子读到的是父**上一周期**的参考。
无论怎么排序，**总有一向陈旧**，陈旧量 = 级联深度 `L`（`doc/FORMAL_MODEL.md` 定理 1/2）。

代价不是"看起来不优雅"，而是控制意义上的：滞后等价于**传输延迟**，直接吃掉相位裕度

```
ΔPM = 360 · f_c · L · Δt   [deg]        （doc/CONTROL_COST_OF_LAG.md）
```

`f_c` 是回路穿越频率。低带宽外环（`f_c·Δt` 很小）几乎无感，高带宽内环（`f_c` 接近 `1/Δt`）就是稳定性问题。

### 2.2 两趟执行

在两趟方案里，同一个顺序被走两遍：

```
update_phase : 反向（叶 → 根）   ⇒ 每个父拿到本周期的子状态
handle_phase : 正向（根 → 叶）   ⇒ 每个子拿到本周期的父参考
```

关键洞察：**因为两趟方向相反，一份线性化顺序就够**，不需要给每个节点单独排序，也不需要时基/双缓冲。
`doc/PASS_LOWER_BOUND.md` 用"阶段顶点图"证明：对"每级都双向"的级联，**两趟是下界且与深度无关**。

同一份证据也给出了边界：
- 规范两趟调度满足阶段图**所有**边：`n ≤ 4` 的全部有根树 × 全部边标注 = **465/465**；
- 单趟可行 ⇔ 合并要求图 `G` 无环（465/465 一致，其中可行 145、不可行 320）；
- **撤回的结论**：曾经写过的"`G` 有环 ⇒ 不可能"只有**充分性**，反过来不成立；
  也曾写过"判定 `κ(D)` 是 NP-hard"，后来实测 `κ(D) ∈ {0,1,2}`，该说法**撤回**。
  详见 `doc/PASS_LOWER_BOUND.md`、`doc/HANDOFF_NEXT_SESSION_2026-09-21.md` 的更正记录。

### 2.3 可靠性与"整棵树"语义

调度对了还不够，运行期还得回答：**这帧数据是哪一周期的？坏了怎么办？**

- **新鲜度契约**：每个节点发布的状态/参考都带 `StagedFrame{cycle, sample_ns, valid, fault_code}`；
  `sample_ns` 由执行组用**最旧**的子样本时间覆盖，所以复合节点无法把自己的派生数据"重新盖章"成更新；
- **整组提交**：任一阶段失败时 `run()` **一个 sink 都不调用**（没有"父写了一半"的状态）；
  精确边界：sink 的 `commit()` 本身就是进程级副作用，**如果某个 sink 在 commit 中失败，之前的 sink 已经生效**，
  那属于"软件契约之外的硬件故障"，不能声称可回滚（见 `staged_execution_group.hpp` 的 Failure semantics 注释）；
- **失败下兄弟一致**：一个分支失败时，兄弟节点看到的是同一套输入（`doc/HIERARCHY_FAIR_COMPARISON.md` §3）；
- **部分激活不执行半棵树**：执行组成员不全为 ACTIVE 时整组**惰性**（`StagedStatus::inactive`），
  并会被点名告警；安装执行组**要求**打开 all-or-nothing 激活。

### 2.4 元编程：把错误提前到编译期

四个层次，越往下越"类型化"：

| 层 | 文件 | 编译期保证 |
|---|---|---|
| 拓扑 | `static_topology.hpp` | 层级是**类型链**；任何环/自环**写不出来**（"cannot be written"，而非"运行期拒绝"） |
| 量纲 | `dimensional_interfaces.hpp` | 接口带物理量纲；父子边量纲不匹配编译失败 |
| 契约 | `topology_contract.hpp` | 一个 binding 同时驱动**检查**和运行期 `Spec` 行；父子 reference/state 边逐条对齐 |
| 端口 | `typed_ports.hpp` | 控制器**只声明一次**端口类型（6 张表），字符串表与 `Contract` 由它**生成**，不可能互相矛盾 |
| 绑定 | `topology_binding.hpp` | `BoundNode` 变参分叉；`compose` 生成可运行的执行组，指针类型安全（无 `void*` 往返） |
| 描述 | `static_manifest.hpp` | 从 binding 的**类型**得到 `static constexpr` manifest（节点/端口/命令/状态接口 + 父链），可在 `static_assert` 里查 |
| 注册 | `static_controller_registry.hpp` | 编译进二进制的控制器用类型字符串走**同一条** `load/configure/activate` 路径，不是 singleton |

具体在"读代码"里怎么看，见 `CODEBASE_TOUR.md` §4。

---

## 3. 这份工作**不**声称什么（先看这段，省得误解）

| 不能声称 | 原因 / 证据 |
|---|---|
| 更快（CPU 时间） | 两趟多一遍遍历，**结构性**更慢；`doc/HIERARCHY_FAIR_COMPARISON.md`、`doc/WHY_NO_SPEEDUP.md` |
| 跟踪误差一定改善 | 低带宽回路本来不敏感（`ΔPM ∝ f_c·Δt`）；`WHY_NO_SPEEDUP.md` §3 |
| `update()` 变成零分配 | 执行组**自身** 0 分配，但 Humble 的 `ControllerManager::update()` 整体仍分配（来源是上游 lifecycle/`MutexMap`） |
| 完整静态初始化 ROS controller | 只做到"编译期描述 + 可按类型字符串加载"；node、参数、`configure/activate`、loan 仍在运行期 |
| 任意时刻都能安全重配 | 模式/成员/计划是一个 generation，但**控制器列表**仍是上游双缓冲；安装执行路径在周期在飞时被拒 |
| 事务性激活（无条件） | 上游是 best-effort；all-or-nothing 是**可选**开关，且只覆盖"本次 switch 触碰到的"控制器 |
| 物理总线原子提交 | `prepare/perform_command_mode_switch` 在 Humble 本身就是两段式，没有事务语义 |
| 依赖库内部无 TSan 问题 | 只给 `controller_manager` 插桩；rclcpp/lifecycle/FastRTPS 未插桩 |

这些不是"还没做"，而是"**目前证据只支持到这里**"。每条都有对应文档和实测记录。

---

## 4. 三条阅读路径

### 路径 A：30 分钟（只想懂概念）

1. 本文件 §2、§3（15 分钟）
2. `doc/TWO_PASS_VS_SINGLE_PASS.md` §1–§3（机制 + 定量实验，8 分钟）
3. `doc/PASS_LOWER_BOUND.md` §0 + §3 的表（"两趟是最优下界"的证据，5 分钟）

读完你能回答：*为什么单趟必然陈旧、为什么两趟够、两趟是不是最优*。

### 路径 B：半天（要读代码、能跑测试）

1. `CODEBASE_TOUR.md` 全文（≈1 小时，含跟着 trace 一个周期）
2. `TESTING_GUIDE.md` §2–§4（构建与跑测试，≈1 小时）
3. 动手：编译 `hierarchical_control`，跑 `test_execution_group` 与 `test_staged_execution_group`，
   改一个断言看它怎么失败（≈1 小时）
4. `doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md`（接口与运行语义，≈40 分钟）

读完你能回答：*代码分几层、每层文件是谁、一次周期里两趟在哪、测试怎么定位问题*。

### 路径 C：一周（要复现结论、能改）

按 `DEVELOPMENT_HISTORY.md` 的阶段顺序读，每个阶段对应的专题文档一起读，并复现其中的测量：

- 形式化：`FORMAL_MODEL.md`、`PASS_LOWER_BOUND.md`、`CONTROL_COST_OF_LAG.md`
- 工程接入：`STAGED_EXECUTION_GROUP_EXPERIMENT.md`、`WIRING_COST_ANALYSIS.md`、`HIERARCHY_FAIR_COMPARISON.md`
- 元编程：`METAPROGRAMMING_CONTRACT.md`、`TOPOLOGY_CONTRACT_JOIN.md`、`PORT_DIMENSIONS.md`、`COMPILE_COST.md`
- 真实闭环：`GAZEBO_CASE_STUDY.md`
- 评审与回应：`REVIEW_*.md`（这是"哪些话被质疑过、怎么回应"的原始记录）
- 当前状态：`IMPLEMENTATION_GUIDE.md`（§12 是"还没做/已妥协"清单）、`HANDOFF_MANUAL.md`

---

## 5. 代码地图（哪个文件是什么）

仓库根目录是 ros2_control 的 Humble 源码树；本项目**新增**了一个包、改了 `controller_manager`，
并加了案例与脚本。判断"哪些是上游原有、哪些是本项目改动"的最快方法：

```bash
git log --oneline                        # 28 个提交，全部是本项目的工作
git diff --stat 469f3055..HEAD           # 469f3055 是 Humble 基线（GIT_WORKFLOW.md）
```

### 新包 `hierarchical_control/`（内核与类型层，与 ROS 解耦）

| 文件 | 作用 |
|---|---|
| `include/hierarchical_control/staged_execution_group.hpp` | **管理器模式内核**：拓扑解析、两阶段执行、帧/故障/整组提交、成员激活缓存 |
| `include/hierarchical_control/staged_controller_interface.hpp` | 控制器与内核之间的**数据契约**（`StagedFrame`、`StagedContext`、`*View/Writer`、sink/source） |
| `include/hierarchical_control/two_phase_controller_interface.hpp` | 只需"两趟 pass"的轻量接口（无组、无端口声明）；用来单独验证调度 |
| `include/hierarchical_control/static_topology.hpp` | 编译期树 + 无环保证（`Root`/`Descendant`/`NameOf`） |
| `include/hierarchical_control/dimensional_interfaces.hpp` | 物理量纲类型与检查 |
| `include/hierarchical_control/topology_contract.hpp` | `Port`/`PortList`/`Contract`、`BoundNode`、`compose`、实例唯一性、父子边谓词 |
| `include/hierarchical_control/typed_ports.hpp` | `TypedPorts<State,Reference,Actuators,ForChildren,ChildState,HardwareState>` + mixin（生成字符串表与 `Contract`） |
| `include/hierarchical_control/topology_binding.hpp` | 把"已检查的类型 binding"接到可运行执行组（`create_library_group`，指针安全） |
| `include/hierarchical_control/static_manifest.hpp` | 编译期 manifest（`manifest_of_v` / `manifest_problem` / `declaration_matches_manifest`） |
| `include/hierarchical_control/hierarchy.hpp` | 计划结构（`ControllerHierarchyPlan`） |

### `controller_manager/`（管理器接入，改动集中在这里）

| 文件 | 作用 |
|---|---|
| `include/controller_manager/controller_manager.hpp` | `ExecutionGeneration`、`ActivationOutcome`、atomic activation、registry、staged group 的公开 API |
| `src/controller_manager.cpp` | `update()` 单入口（含两趟/执行组/切换）、`switch_controller`、`activate_controllers*`、回滚、按类型加载 |
| `include/controller_manager/static_controller_registry.hpp` + `src/…cpp` | 编译进二进制的控制器：工厂注册 + 封印 + manifest 描述 |
| `include/controller_manager/staged_execution_group.hpp`、`hierarchy.hpp` | **兼容转发头**（真正的实现在 `hierarchical_control`） |
| `include/controller_manager/cycle_tree.hpp` | 库模式内核的公开入口 |

### 测试与复现

| 位置 | 内容 |
|---|---|
| `hierarchical_control/test/*.cpp` | 内核/类型的 gtest（12 个程序、81 个用例） |
| `hierarchical_control/test/static_topology_negative/` | **必须编译失败**的语料（15 个源文件）+ `test_static_topology_negative.py` 驱动 |
| `hierarchical_control/test/measure_*.py` | 编译成本测量（`measure_compile_cost.py`、`measure_binding_cost.py`） |
| `hierarchical_control/test/run_tsan_publish_protocol.sh` | 发布会话协议的 TSan |
| `controller_manager/test/` | 管理器级 gtest（18 个程序、187 个用例）+ 启动/pytest 用例 |
| `controller_manager/test/test_composite_library/` | 通用 composite 宿主 + typed 分叉树插件（同一内核的第二种宿主） |
| `controller_manager/test/run_tsan_real_manager.sh` | 给真实 `ControllerManager` 插桩的 TSan |
| `case_study/` | Gazebo 闭环案例（控制器 + 配置 + 日志） |
| `research/` | 形式化模型的小工具（阶段图检查、最少趟数搜索、上游排序模拟、被否决的拓扑变体） |

---

## 6. 术语表

| 术语 | 含义 |
|---|---|
| **双向依赖 / bidirectional edge** | 父子间同时存在自顶向下（reference）和自底向上（state）的**同周期**需求 |
| **两趟 / two-pass** | 反向状态趟 + 正向命令趟，走同一份顺序；`update_phase` / `handle_phase` |
| **阶段顶点模型 / stage-vertex model** | 把每个节点拆成状态顶点 `S_v` 与命令顶点 `C_v`，把"同一周期"写成顶点图的边，用来判定最少趟数 |
| **陈旧量 L / lag** | 单趟执行下父子数据相差的周期数，等于级联深度 |
| **相位裕度损失 ΔPM** | `360·f_c·L·Δt`，把"周期数"换成控制意义的代价 |
| **`κ(D)`** | 最少趟数指标；实测只取 `{0,1,2}`（曾经的 NP-hard 说法已撤回） |
| **执行组 / staged execution group** | 管理器里的一组控制器，按阶段契约整体执行、整组提交 |
| **帧 / `StagedFrame`** | 一帧数据的溯源元数据：`cycle`、`sample_ns`、`valid`、`fault_code` |
| **惰性组** | 成员不全为 ACTIVE 时执行组**什么都不跑**（`StagedStatus::inactive`） |
| **generation（执行代）** | 模式 + 成员表 + 执行组的**不可变快照**，一次原子发布；`update()` 每周期一次读取 |
| **atomic activation** | 可选开关：一次 switch 失败时撤销**本次**已激活的控制器（含硬件模式、reference、chained mode） |
| **manifest** | 从 binding 类型派生出的编译期描述（节点/端口/接口需求），可 `static_assert` |
| **compiled-in registry** | 编译进二进制的控制器按类型字符串加载，与 pluginlib 走同一生命周期 |
| **library mode / manager mode** | 同一内核的两种宿主：单个 composite 插件内（库模式），或 `ControllerManager` 里（管理器模式） |

---

## 7. 文档地图

**入门（先读）**：`ONBOARDING.md`（本文件）、`CODEBASE_TOUR.md`、`TESTING_GUIDE.md`、`DEVELOPMENT_HISTORY.md`

**理论**：`FORMAL_MODEL.md`（模型与定理）、`PASS_LOWER_BOUND.md`（最少趟数）、`CONTROL_COST_OF_LAG.md`（代价）、
`BIDIRECTIONAL_EDGE_ANALYSIS.md`（上游行为实证）、`TWO_PASS_VS_SINGLE_PASS.md`（定量对照）

**工程**：`STAGED_EXECUTION_GROUP_EXPERIMENT.md`（接口/语义/验收）、`IMPLEMENTATION_GUIDE.md`（权威实现状态 + 未做清单）、
`HANDOFF_MANUAL.md`（当前状态与"不能声称"清单）、`WIRING_COST_ANALYSIS.md`（接线成本）、
`HIERARCHY_FAIR_COMPARISON.md`（三方案公平对照）、`WHY_NO_SPEEDUP.md`（负结果正面回答）

**元编程**：`METAPROGRAMMING_CONTRACT.md`、`TOPOLOGY_CONTRACT_JOIN.md`、`PORT_DIMENSIONS.md`、`COMPILE_COST.md`

**案例**：`GAZEBO_CASE_STUDY.md`（真实仿真闭环）

**与论文的对照**：`PAPER_ALIGNMENT_2026-09-28.md`（逐条对照 FineMote 的思想/机制/定理，判定一致·更窄·缺失·更强）

**评审史（原始记录）**：`REVIEW_HUMBLE_WORK_2026-09-23.md`、`REVIEW_RESPONSE_2026-09-23.md`、
`REVIEW_REQUIREMENTS_2026-09-24.md`、`REVIEW_REQUIREMENTS_RESPONSE_2026-09-24.md`、
`COMPILETIME_CONTROLLER_DESIGN_2026-09-26.md`、`COMPILETIME_CONTROLLER_RESPONSE_2026-09-26.md`、
`REVIEW_COMPILETIME_LATEST_2026-09-26.md`、`REVIEW_COMPILETIME_LATEST_RESPONSE_2026-09-26.md`

**论文草稿**：`PAPER.md`、`PAPER_SKELETON.md`、`RELATED_WORK.md`
（注意：`PAPER.md` 里若有与本文件 §3 冲突的表述，以 §3 和 `IMPLEMENTATION_GUIDE.md` §12 为准——
那是经过实测修正后的口径。）
