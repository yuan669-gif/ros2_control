# FineMote × ROS 2 Humble：层次化控制器项目报告

报告日期：2026-09-21
代码基线：本仓库（ROS 2 Humble overlay，upstream baseline `469f3055`）
适用读者：导师、新接手同学、需要了解本课题全貌的reviewer

---

## 0. 一句话概述

把 **FineMote 的"双向两阶段设备调度"** 思想移植到 **ROS 2 Humble 的 `controller_manager`** 上：
为控制器层次树引入**显式的状态上行阶段、命令下行阶段、每周期有效性元数据和整组命令提交**；
把这套机制做成**独立、宿主无关的库** `hierarchical_control`，并提供两种宿主
（`ControllerManager` 集成 / 单个插件内部托管）。

**重要结论先说**：这套机制在功能与故障一致性上成立，但**性能上不占优**，且**"减少集成代码"
也不是它独有的能力**（通用 composite 库用同一份内核可以做到同样的事）。
因此本项目的最终建议形态是"内核库"，而不是"改造 `controller_manager`"。

> **叙事已于 2026-09-21 修订（写论文以此为准）**
>
> 上游 `ros2_control` **已经**有 state chaining 与 state 感知排序（PR #1021、issue #1123、
> commit 69b3225），所以本文**不能**声称"用接口连接生成状态依赖/层次排序"。
> 修订后的主贡献见 `doc/FORMAL_MODEL.md`：
> **在单入口控制器模型下，同周期双向数据流不可满足（定理 1）；两趟执行是它的充分修复
> （定理 3），代价是一个线性化顺序与 `O(|V|)` 存储；单趟的滞后等于级联深度（定理 2）；
> 拓扑无环性可在配置期判定（推论 2）。**
> 本文第 3 节把"接口连接是依赖来源"当作贡献的写法**已过时**，改为"顺序只用于定向，
> 状态方向由反向遍历提供"。性能与集成成本上的负结果（第 6 节）在新叙事下是**代价一侧的
> 诚实数据**，不是失败。

---

## 1. 背景

### 1.1 FineMote 是什么

FineMote（本仓库 `FineMote-main/`，论文 `FineMote论文_文本.txt`）是一套面向复杂机电系统的
**实时嵌入式中间件合成框架**。它的核心思想是：不手写每个设备的控制逻辑，而是由用户声明功能，
框架生成"静态操作系统"，把外设协议、确定性数据交换和实时调度封装起来。

与本课题直接相关的两个机制：

1. **双向两阶段调度**。`Devices/DeviceBase.hpp` 中：

简化示意（实际实现带分频计数与 `HAL_GetTick` 统计）：

```cpp
// 正向遍历：Update()
for (auto it = getDeviceList().begin(); it != getDeviceList().end(); ++it)
    if (++cnt >= divisionFactor) { (*it)->Update(); updated = 1; }
// 反向遍历：Handle()
for (auto rit = getDeviceList().rbegin(); rit != getDeviceList().rend(); ++rit)
    if (updated) { (*it)->Handle(); updated = 0; }
```

设备在构造时注册，注册顺序由构造顺序决定。在典型用法（电机先构造、底盘后构造）中：
`Update()` 正向遍历是**叶→根**（电机解码反馈），`Handle()` 反向遍历是**根→叶**
（底盘算逆运动学、电机发命令）。这正是"状态上行 + 命令下行"。

2. **依赖由对象构造/依赖注入表达**，配合模板参数与 `static_assert` 做编译期维度检查
（`Algorithms/Control/ImplementControlBase.hpp`），不是由 URDF 自动推导。

FineMote 自身的限制（同样适用于本课题的边界讨论）：
跨翻译单元静态构造顺序仍有风险；`divisionFactor` 不是完整的周期有效性协议；
逐设备发送不等于总线原子同步；POV 底盘的部分派生计算仍留在 `Handle()` 里。

### 1.2 ROS 2 Humble 的现状

`ros2_control` 的主循环是 `read -> update -> write`：

```cpp
// controller_manager/src/ros2_control_node.cpp
cm->read(cm->now(), measured_period);
cm->update(cm->now(), measured_period);
cm->write(cm->now(), measured_period);
```

`ControllerManager::update()` 遍历控制器列表，对每个活跃控制器调用一次 `update()`。
**它已经有一套基于 reference interface 的依赖排序**（`controller_sorting`、`get_following_controller_names`、
`get_preceding_controller_names`，支持分支链），这部分是我们要**保留**的既有能力，不是要重造的东西。

Humble 这一版的 `ChainableControllerInterface` 导出的是 **reference interface**（命令链），
**没有**新版本的 state-export API（代码注释里也写着 "TODO(saikishor): deal with the state interface
chaining in the sorting algorithm"）。

### 1.3 两者之间的差距（本课题的切入点）

| | FineMote | Humble `ros2_control` |
|---|---|---|
| 一个周期内的数据方向 | 双向：`Update` 上行 + `Handle` 下行 | 单向：每个控制器一个 `update()` |
| 派生状态给父节点用 | 原生支持（子先 `Update`） | 父节点只能自己直接读硬件状态 |
| 周期有效性 | `divisionFactor`（不完整） | 无统一协议 |
| 输出提交 | 逐设备发送 | 每个控制器各自写自己的命令接口 |
| 失败一致性 | 逐设备，非事务 | 某控制器 `update()` 返回错误后，**后续控制器仍继续执行** |

关键洞察（决定了整个技术方案）：

> **一个不可拆分的 `update()` 无法同时满足两个相反方向的依赖。**
> 若父节点需要子节点算出的估计值，则子必须先跑；若父节点产生子节点的目标，则父必须先跑。
> 只有一个函数入口时，这两者矛盾。把它们拆成两个显式阶段即可解决——这不是发明新的控制律，
> 而是把调度契约显式化。

---

## 2. 研究问题

本课题的精确表述（来自 `doc/handoff` 系列与 `hierarchical_research.md`）：

> 在**保留 Humble 原生 reference 依赖排序**的基础上，引入**显式状态阶段、周期有效性和命令组提交**，
> 能否让可复用的复合控制组件完成**同周期双向数据传播**，并以**可接受的接入和运行时成本**
> **避免陈旧或部分命令**？

这是一个**假设**，不是结论。需要与以下基线公平比较：

1. 原生 Humble chaining（父节点直接读硬件状态）；
2. 估计/命令显式拆分；
3. 手写 composite plugin（同样的内部两阶段算法和缓冲）；
4. 本方案。

并且明确**不能声称**：首次提出层次化控制、自动控制器综合、已验证的 WCET、
仅凭 URDF 推断控制语义。

---

## 3. 技术方案总览

方案分三层，边界清晰：

```text
┌─────────────────────────────────────────────────────────────┐
│ 契约层  StagedControllerInterface                            │
│   控制器实现的两个 noexcept 阶段回调 + 端口声明 + 数据契约    │
├─────────────────────────────────────────────────────────────┤
│ 内核层  StagedExecutionGroup                                 │
│   配置期：拓扑解析、校验、存储规划                            │
│   运行期：状态后序 → root 参考 → 命令前序 → 校验 → 整组提交    │
├─────────────────────────────────────────────────────────────┤
│ 宿主层  ① manager 宿主：N 个独立控制器插件 + manager 调度      │
│         ② 库宿主：1 个普通插件内部托管整棵树                  │
└─────────────────────────────────────────────────────────────┘
```

### 3.1 数据契约

每个节点发布状态、消费一个 reference，两者都带来源信息：

```cpp
struct StagedFrame {
  std::uint64_t cycle;      // 产生该值的执行组周期
  std::int64_t  sample_ns;  // 所用输入中最旧的采样时间
  bool          valid;      // 仅在本周期对应阶段成功后为真
  std::uint32_t fault_code; // 控制器自定义故障码，0 = 无
};
```

四条**由内核强制**（而非靠约定）的规则：

1. **叶节点**上报硬件采样时间（默认本周期），可以更旧，**不能是未来**；
2. **复合节点的 `sample_ns` 被内核用"最旧子节点采样时间"覆盖**——父节点写什么都不生效，
   因此**无法重新盖章伪装新鲜**；
3. 所有状态 / reference / actuator 值必须是有限值；
4. **失败的周期不调用任何命令 sink**，因此硬件命令缓冲不会出现"一部分新、一部分旧"。

### 3.2 周期顺序

```text
state 阶段   : 后序（子 → 父），每个节点最多一次 update_state_stage
root 输入    : 本周期取一次外部 reference 快照（StagedReferenceSource）
command 阶段 : 前序（父 → 子），每个节点最多一次 update_command_stage
校验         : 周期一致性、采样年龄、有限性
提交         : 叶节点的 scratch 复制进控制器持有的 StagedCommandSink
```

### 3.3 依赖边从哪里来

**manager 宿主**：命令边来自 Humble 原生机制——子控制器通过 `export_reference_interfaces()`
导出 `<child>/<port>`，父控制器的 `command_interface_configuration()` 里 claim 它。
内核在配置期按 `/` 前缀匹配成员名，解析出父子关系。

- 交换控制器声明顺序不改变计划；
- 一个 reference 接口有两个写者在配置期直接报错；
- 多根、成环、未知父、自父都在配置期报错。

**库宿主**：用显式的 `Spec{names, instances, parents}` 声明，同样在配置期校验。

**当前的简化**（已与作者确认接受，论文中需写明为限制）：
**状态边 = 命令边的反向**。即"父节点 claim 了子节点的 reference，就默认也消费它的状态"。
更精确的模型是让父节点显式声明消费哪些子状态端口（更接近 FineMote 的依赖注入）。

**URDF 当前不参与依赖推导**，只用于 `ResourceManager` 提供 joint 资源。

### 3.4 失败语义与提交边界（必须准确表述）

- 状态阶段任一步失败 → **命令阶段完全不进入**；
- 命令阶段失败 / 写出非有限值 → **任何 sink 都不被调用**；
- sink 在 `commit()` 中返回 false → 这是**硬件故障域**，会报告但**不承诺回滚**已提交的兄弟叶节点；
- **保留上一周期命令不是安全策略**：应用必须检查结果并执行自己定义的故障动作；
- "原子性"指的是**软件命令缓冲的一致提交**，不是 CAN/总线原子同步，也不是物理动作同时发生。

### 3.5 配置期校验清单

`create()` / `create_library()` 在进入实时循环前拒绝：

- 空/重复节点名、Spec 尺寸不一致；
- 未知父节点、多根、环、自父；
- 声明了 actuator 端口但没有命令 sink；
- 根声明了 reference 端口但没有 reference source；
- （manager 宿主）reference 接口多写者。

---

## 4. 工程结构（仓库实际状态）

### 4.1 独立库包

```text
hierarchical_control/                        # header-only，namespace hierarchical_control
├── package.xml                              # 依赖 controller_interface / lifecycle_msgs / rclcpp
├── CMakeLists.txt                           # 导出 include；BUILD_TESTING 下编译内核单测
├── README.md                                # 契约、两种宿主、校验规则、非目标
├── include/hierarchical_control/
│   ├── hierarchy.hpp                        # 计划生成 + 校验（纯 C++，可脱离 ROS 编译）
│   ├── staged_controller_interface.hpp      # 双阶段接口 + 数据契约
│   └── staged_execution_group.hpp           # 内核（含 create() 与 create_library()）
└── test/test_execution_group.cpp            # 9 个内核单测，不创建 ControllerManager
```

### 4.2 `controller_manager` 薄适配层

```text
controller_manager/
├── include/controller_manager/controller_manager.hpp   # 3 个公开 API + staged_group_ 成员
├── src/controller_manager.cpp                          # 实现 + update() 接入
└── include/controller_manager/
    ├── hierarchy.hpp                    # 兼容 shim（using 转发到库）
    └── staged_execution_group.hpp       # 兼容 shim（using 转发到库）
```

依赖方向单向：`controller_manager → hierarchical_control → controller_interface`，**无包依赖环**。

> 为什么 `controller_interface/staged_controller_interface.hpp` 被删除而不是留 shim：
> 若在 `controller_interface` 留转发 shim，就形成 `controller_interface ↔ hierarchical_control` 环。
> 代价是：实现双阶段接口的插件现在依赖 `hierarchical_control`。

### 4.3 实验与测试代码

```text
controller_manager/test/
├── test_staged_controller/            # 应用侧控制器（同时支持 staged 模式与 native 单相模式）
├── test_composite_controller/         # 基线：手写 composite 插件（相同算法）
├── test_composite_library/            # 基线：通用 composite 库宿主（复用同一内核）
├── test_staged_execution_group.cpp    # 6 个 manager 级验收测试
├── test_hierarchy_comparison.cpp      # 5 个公平对照测试
├── test_cycle_tree_standalone.cpp     # 独立标量内核实验（前一阶段）
├── test_hierarchy.cpp / test_urdf_hierarchy.cpp   # v1 历史文件测试
└── test_controller_hierarchy_builder.cpp
research/cycle_tree/CMakeLists.txt     # 不依赖 ROS 的独立构建入口
```

### 4.4 代码量（非空非注释行，脚本统计）

| 项目 | code lines |
|---|---|
| 库：`hierarchy.hpp` | 121 |
| 库：`staged_controller_interface.hpp` | 125 |
| 库：`staged_execution_group.hpp` | 447 |
| **库合计（内核 + 契约）** | **693** |
| 库单测 `test_execution_group.cpp` | 386（9 个测试） |
| 应用侧控制器（两种宿主共用） | 443 |
| 基线：手写 composite 插件 | 181 |
| 基线：通用 composite 库宿主 | 371 |
| manager 测试 `test_staged_execution_group.cpp` | 249（6 个测试） |
| 对照测试 `test_hierarchy_comparison.cpp` | 713（5 个测试） |

### 4.5 文档索引

| 文档 | 内容 |
|---|---|
| `doc/PROJECT_REPORT_2026-09-21.md` | **本文**，项目全貌 |
| `doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md` | 阶段 A/B：接口与内核的设计、验收用例、验证记录 |
| `doc/HIERARCHY_FAIR_COMPARISON.md` | 同算法对照：输出/故障/分配/耗时，含诚实结论 |
| `doc/WIRING_COST_ANALYSIS.md` | Gate B 接线成本量化与最终结论 |
| `doc/HANDOFF_STAGED_GROUP_2026-09-20.md` | 本轮交接：文件清单、验证状态、下一步 |
| `doc/HANDOFF_CURRENT_2026-09-20.md` | 前一阶段交接（研究契约、阶段计划） |
| `doc/HANDOFF_2026-09-19.md`、`doc/hierarchical_research.md` | 更早的研究契约与修正 |
| `doc/CYCLE_TREE_EXPERIMENT.md` | 前序独立标量内核实验 |

---

## 5. 两种宿主

### 5.1 manager 宿主

节点是**独立的控制器插件**（可单独加载、可单独激活），manager 负责调度：

```cpp
// 3 个公开 API
controller_interface::return_type set_staged_execution_group(
  const std::vector<std::string> & controller_names, std::int64_t max_age_ns = 0);
void clear_staged_execution_group();
std::shared_ptr<StagedExecutionGroup> staged_execution_group() const;
```

`ControllerManager::update()` 的改动（**这是唯一被修改的核心函数**）：

```cpp
if (staged_group_ && !switch_params_.do_switch)
{
  const auto staged_result = staged_group_->run(time, period);
  // committed / inactive → OK；其余 → 记录 ERROR
}
for (auto loaded_controller : rt_controller_list)
{
  if (is_controller_active(*loaded_controller.c))
  {
    if (staged_group_ && staged_group_->owns(loaded_controller.c.get())) continue;  // 跳过成员
    ...
  }
}
if (switch_params_.do_switch)
{
  manage_switch();
  if (staged_group_) staged_group_->refresh_member_active_state();  // 只在切换后刷新
}
```

`owns()` 只做**已排序指针数组的二分查找**，不做字符串比较。

**为什么活跃状态要缓存**：`get_state()` 最终调用
`rclcpp_lifecycle::LifecycleNode::get_current_state()`，它会构造 `State` 并在
`rclcpp_lifecycle::MutexMap::add()` 中**动态分配**。若每周期对每个成员调用，执行组就不可能零分配。
现在只在切换真正发生后刷新一次。

> 代价（已写入限制）：如果控制器**不经 switch** 自行改变状态，缓存会过期。

### 5.2 库宿主

一个普通 `ControllerInterface` 插件内部托管整棵树：

```cpp
std::vector<CompositeNodeSpec> specs;            // 数据驱动：name/parent/接口/factor/offset
hierarchical_control::StagedExecutionGroup::Spec spec{names, instances, parents};
auto kernel = hierarchical_control::StagedExecutionGroup::create_library(spec, max_age_ns);
// update() 里：kernel->run(time, period);
```

不导出 chainable 接口、不 claim 子控制器接口、**不改 `controller_manager`**。

### 5.3 宿主对比

| 维度 | manager 宿主 | 库宿主 |
|---|---|---|
| 组件复用/生命周期 | 每个节点是独立可加载、可单独激活的控制器 | 整棵树焊死在一个插件里 |
| 与原生接口兼容性 | 用原生 reference interface，可与普通 chainable 控制器混用 | 只 claim 硬件接口 |
| 部署成本 | 需要维护 `controller_manager` 改动 | 零 manager 改动 |
| 加一个节点 | 1 条配置 | 1 条配置 |
| 内核代码 | **同一份**（572 行：接口 125 + 执行组 447） | 同一份 |

---

## 6. 验证与实验结果

全部数据来自本机（Ubuntu 22.04 / GCC 11.4.0 / ROS 2 Humble / colcon overlay，
`RelWithDebInfo`，非实时虚拟机）。

### 6.1 测试矩阵

| 测试 | 数量 | 关注点 |
|---|---|---|
| `hierarchical_control::test_execution_group` | 9 | 内核契约，**不依赖 controller_manager** |
| `test_staged_execution_group` | 6 | manager 集成、相位顺序、失败不部分提交、mock hardware |
| `test_hierarchy_comparison` | 5 | 同算法对照、故障一致性、分配、耗时、接线成本、库 vs 组 |
| `test_cycle_tree_contract` | 1（独立程序） | 前序标量内核，与全部无关 |

内核单测覆盖：1000 次与手写 baseline 逐项相等、相位顺序与单次调用、状态失败不进命令阶段、
命令失败/NaN/sink 失败都不提交、**复合节点不能重盖采样时间**、采样年龄上限、
库模式无需生命周期刷新、配置期错误、结构错误。

### 6.2 输出一致性

三种实现（原生 chaining、手写 composite、阶段化执行组）在相同控制律、相同状态快照下：

- 每个周期的叶命令**逐位相等**；
- 且都等于解析值（线性链 `R - 7P`，两叶 fork `R - 2(o_a+o_b) - o_x`）。

**结论**："能否算出正确的同周期值"**不是**本方案的区分点。

### 6.3 故障一致性（真正的区分点）

两叶 fork（`root → {leaf_a, leaf_b}`），第 5 周期注入 `leaf_a` 命令阶段失败：

| 观测 | 原生 chaining | 阶段化执行组 / 库宿主 |
|---|---|---|
| `update()` 返回 | ERROR | ERROR |
| `joint2`（失败叶） | 保持旧值 | 保持旧值 |
| `joint3`（健康兄弟） | **写入新值** | 保持旧值 |
| 两兄弟是否一致 | **否（一新一旧）** | **是** |
| 下一周期 | 恢复 | 恢复 |

这正是 POV 底盘必须避免的"部分新、部分旧"。

### 6.4 动态分配

计数方式：测试可执行文件中替换全局 `operator new/delete`，只在测量窗口内计数；
control 窗口（同长度空转）为 0，排除后台噪声。

| 测量 | chained | composite | staged |
|---|---|---|---|
| 每次 `ControllerManager::update()` 的分配次数（200 周期均值） | 10 | 3 | 10 |
| `StagedExecutionGroup::run()` **自身**每 100 次调用 | — | — | **0** |
| 库宿主 `update()` 惰性建内核之后每 100 次调用 | — | — | **0** |

`update()` 整体的分配来自 **Humble 既有代码**，三种方案都逃不掉：

1. legacy 循环 `for (auto loaded_controller : rt_controller_list)` **按值复制 `ControllerSpec`**
   （含字符串和 vector）；
2. `is_controller_active()` / `get_state()` 的 `get_current_state()` 分配（见 5.1）。

因此正确表述是：**执行组没有新增实时路径分配**，而不是"接入后 `update()` 变成零分配"。

### 6.5 耗时（非实时虚拟机，仅供参考）

同进程、200 周期、连续 3 次运行，`update()` 的 min/median/max（微秒）：

| 运行 | 原生 chaining | 手写 composite | 阶段化执行组 |
|---|---|---|---|
| 1 | 1.42 / 1.87 / 39.5 | 0.40 / 0.65 / 8.5 | 3.21 / 4.28 / 114.2 |
| 2 | 2.49 / 4.28 / 299.6 | 1.06 / 1.51 / 12.1 | 3.37 / 4.29 / 20.6 |
| 3 | 2.47 / 2.50 / 12.5 | 1.00 / 2.04 / 36.9 | 3.03 / 4.29 / 40.7 |

- 手写 composite 最便宜；**阶段化执行组中位耗时约为 composite 的 2 倍**；
- max 波动 8–300 µs 是调度噪声，**不能**当作 deadline miss 统计；
- 原因（假设，待消融验证）：执行组每周期 3 次状态回调 + 3 次命令回调，每次都带帧校验与有限性检查。

### 6.6 接线成本（Gate B）

场景：给已存在的 fork 拓扑**加一个叶**。

| 方案 | 改动 |
|---|---|
| 阶段化执行组 | **1 个 `LeafSpec`/`CompositeNodeSpec` 条目**，0 行控制器代码，0 行调度/缓冲代码 |
| 原生 chaining | **1 个条目**（claim 列表与 bias 由循环/求和生成） |
| 手写 composite（固定拓扑） | 插件必须改内部算法与缓冲（现状插件甚至无法表达 fork） |

两种宿主都**复用同一个叶控制器类**。测试 `adding_a_leaf_is_configuration_only` 对
`{2 叶, 3 叶} × {staged, chained}` 四种组合验证输出正确。

### 6.7 Gate B 的决定性结果

随后实现的**通用 composite 库宿主**（`generic_composite_controller`，371 行）用
**同一份内核**（`create_library()`）在**单个普通插件**里托管整棵树：

- 2 叶 / 3 叶 fork 输出与执行组**逐位相同**；
- 惰性建内核后 `update()` **100 次 0 分配**；
- **不改 `ControllerManager`**、不导出 chainable 接口；
- 加一个叶同样只需一个节点条目。

**因此："减少手写集成代码"不是 manager 方案独有的能力。** 结合 6.5 的性能劣势：

> **Gate B（相对手写 composite 的集成优势）不成立；按论文停止门槛，
> 不应继续扩大 `ControllerManager` 修改。**

### 6.8 过程中发现的两个 Humble 事实（对社区也有用）

1. `rclcpp_lifecycle::LifecycleNode::get_current_state()` 会动态分配
   （`rclcpp_lifecycle::MutexMap::add()`）。任何想做到零分配的实时路径都不能每周期调用
   `get_state()`。
2. `ControllerManager::update()` 的 legacy 循环**按值复制 `ControllerSpec`**，
   每控制器每周期都会分配。这意味着"让 `update()` 零分配"在这版 Humble 上做不到，
   除非同时改掉这个循环。

---

## 7. 结论：能声称什么 / 不能声称什么

### 已经验证（可以写进论文）

1. 存在一个**宿主无关**的两阶段执行内核，可在真实 `ControllerManager` +
   `ResourceManager` 之上运行，也可在单个普通插件内运行，两者输出逐位一致；
2. 它保证**同周期双向传播**：状态严格后序、命令严格前序、所有状态回调早于所有命令回调；
3. 它保证**复合状态采样时间等于最旧输入**，父节点无法重盖；
4. 它保证**失败周期不产生部分命令**：状态失败不进命令阶段；命令失败/非有限/sink 拒绝都不提交；
5. 原生 chaining 在某个叶失败后**仍会更新健康兄弟**，产生"一新一旧"；本方案不会；
6. 内核运行路径在配置完成后**零动态分配**（manager 模式与库模式均已断言）；
7. 配置期拓扑/结构错误被拒绝；
8. **它不比手写 composite 快**（中位约 2 倍），且**"省集成代码"不是它独有**。

### 尚未验证（不能写进结论）

- 真实硬件、真实总线原子性、物理动作同步；
- 硬实时 WCET、deadline miss 分布（当前只有非实时虚拟机的粗测）；
- 仅凭 URDF / 接口自动推断任意控制器语义依赖；
- 生命周期部分失败回滚、多频率、异步数据源、多执行组；
- 只测了一种拓扑变化（加叶）；插入中间层、子树替换、共享模块尚未测；
- 方案 2（估计/命令显式拆分）尚未实现，对照矩阵还差一格。

### 明确不能声称

- "首次提出层次化控制"；
- "自动控制器综合"；
- "比原生 chaining 更快 / 更省"；
- "已验证 WCET"；
- "接入后 `update()` 零分配"。

---

## 8. 如何构建与复现

### 8.1 完整 overlay

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

### 8.2 只构建/测试独立库

```bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select hierarchical_control --output-on-failure
```

### 8.3 manager 级与对照测试

```bash
cd build/controller_manager
ctest -R "test_staged_execution_group|test_hierarchy_comparison" --output-on-failure
ctest -R "test_cycle_tree_contract|test_controllers_chaining_with_controller_manager|test_hierarchy$|test_urdf_hierarchy"
```

### 8.4 不依赖 ROS 的独立内核实验

```bash
cmake -S research/cycle_tree -B build_standalone -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_standalone -j4
ctest --test-dir build_standalone --output-on-failure
```

### 8.5 受限环境注意

若 `~/.ros` 不可写（沙箱/只读 home），gtest 会在 `SetUpTestSuite` 抛异常并在退出时崩溃。
设置可写日志目录即可：

```bash
export ROS_LOG_DIR="$PWD/log/ros"
```

### 8.6 已验证的构建/测试结果

| 检查 | 结果 |
|---|---|
| `colcon build --packages-up-to controller_manager` | 7 个包成功，0 warning/0 error |
| `hierarchical_control` ctest | 1/1（含 9 个 gtest） |
| controller_manager 关键 ctest | 6/6 |
| 非 ROS standalone `research/cycle_tree` | 构建 + CTest 1/1 |
| 全量 controller_manager ctest（抽取**之前**的一次运行） | 22 项中 19 项通过；3 项失败为慢速虚机/服务发现问题，均不走执行组分支 |

---

## 9. 与 FineMote 的逐条对应

| FineMote 机制 | 本项目的对应物 | 差异 |
|---|---|---|
| `Update()` 正向遍历（叶→根） | `state` 阶段，后序 | 显式化为独立回调 |
| `Handle()` 反向遍历（根→叶） | `command` 阶段，前序 | 显式化为独立回调 |
| 构造注册顺序决定遍历顺序 | 配置期从 reference interface / `Spec` 解析拓扑 | 不再依赖构造顺序 |
| `divisionFactor` 分频 | `max_age_ns` + 周期一致性校验 | 更严格的周期有效性协议 |
| 对象构造/依赖注入表达依赖 | 原生 reference interface（manager）/ 显式 `Spec`（库） | 当前状态边 = 命令边反向（简化） |
| 逐设备发送命令 | 叶 scratch → 整组提交到 sink | 软件提交一致，**非总线原子** |
| 模板 + `static_assert` 编译期检查 | **尚未实现**（属阶段 D 的 TMP 工作） | 明确的后续工作 |
| POV 底盘派生计算在 `Handle()` | 复合节点必须在 `update_state_stage` 里做派生 | 要求组件显式拆分阶段 |

---

## 10. 后续工作

按优先级：

1. **更多拓扑变化**：插入中间层、叶换成子树、共享模块（两个父消费同一子状态），
   用来确认"配置化加节点"不是只对 fork 成立；
2. **方案 2（估计/命令显式拆分）**：补齐第四个对照，让对照矩阵完整；
3. **稳定环境下的性能测量**：绑核、关后台负载、报告 p50/p95/p99 与 deadline miss；
4. **消融**：分别关闭"帧校验"和"整组提交"，测量各自耗时占比；
5. **编译期维度/单位检查**（FineMote 的 TMP 思路）：静态组件用模板与 `static_assert`，
   动态插件用配置期检查；
6. 库宿主目前**在第一次 `update()` 惰性建内核**（一次性实时分配），应改为 `on_activate` 后显式触发；
7. 状态边模型：如需表达"只取 reference 不取状态"，再引入父节点显式状态输入声明。

---

## 附录 A：术语表

| 术语 | 含义 |
|---|---|
| 状态阶段 / state phase | 后序执行，子节点把状态发布给父节点 |
| 命令阶段 / command phase | 前序执行，父节点把 reference 发给子节点 |
| 叶 / leaf | 没有子节点、直接写硬件命令的节点 |
| 复合 / composite | 有子节点、由子状态派生的节点 |
| sink | 控制器持有的、整组提交时才被写入的真实命令接口 |
| source | 根节点本周期取一次外部 reference 快照的来源 |
| 同周期 | 周期开始纳入快照的数据在本周期完成软件计算与提交；不代表总线/物理同时发生 |
| 宿主 | 驱动内核的角色：manager 或单个插件 |

## 附录 B：常见误解与澄清

1. **"树遍历本身是创新"** —— 不是。Humble 已有基于 reference 的链式排序（含分支）。
   本项目的创新点在**显式两阶段 + 周期有效性 + 组提交**这三条契约。
2. **"parent 字段能表达层次"** —— 不能覆盖接口依赖。显式 parent 只能作为兼容/组合归属信息。
3. **"两次回调天然更快"** —— 相反，实测约为单插件 composite 的 2 倍。
4. **"URDF 能推出控制依赖"** —— 不能。URDF 只能校验 link/joint、资源与机械锚点。
5. **"软件提交等于物理同步"** —— 不等于。
6. **"库模式就不需要宿主"** —— 它只是把宿主从 manager 换成了插件自身，契约不变。

## 附录 C：历史脉络

```text
2026-09-19  HANDOFF_2026-09-19 / hierarchical_research / CYCLE_TREE_EXPERIMENT
            → 研究契约修正；独立标量内核 cycle_tree 实现与验证
2026-09-20  HANDOFF_CURRENT_2026-09-20
            → 确定阶段 A–E 计划、公平对照与停止门槛
2026-09-20/21  STAGED_EXECUTION_GROUP_EXPERIMENT
            → 阶段 A/B：StagedControllerInterface + StagedExecutionGroup + manager 接入
               HIERARCHY_FAIR_COMPARISON → 阶段 D：同算法对照
               WIRING_COST_ANALYSIS → Gate B：接线成本 + 通用 composite 库基线
            内核抽取为独立包 hierarchical_control（本报告）
```

`cycle_tree.hpp` 及其独立实验保留为**前序证据**：它验证了标量双阶段与提交契约在
无 ROS 环境下成立。当前内核 `staged_execution_group.hpp` 是它的 ROS 泛化版本，
两者共用 `hierarchy.hpp` 的计划生成与校验。
