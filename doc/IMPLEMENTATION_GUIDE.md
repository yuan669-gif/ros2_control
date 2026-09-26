# 实现说明：把 FineMote 的两阶段机制引入 ros2_control

日期：2026-09-22　工作区：`/home/mamingyuan/Desktop/ros2_control-humble`
读者：项目作者本人（代码层面）
上游基线：`ros-controls/ros2_control` Humble 副本，无 `.git`，改动全部在本地

> 本文只讲**做了什么、怎么做的、哪里还没做**，不谈创新性论证。
> 创新性/声称边界另见 `doc/PAPER.md` 与 `doc/HANDOFF_MANUAL.md` §11。

---

## 目录

1. [改动清单（精确到文件与行号）](#1-改动清单)
2. [依赖分层与包结构](#2-依赖分层与包结构)
3. [核心机制 A：库内核 `StagedExecutionGroup`](#3-核心机制-a库内核)
4. [核心机制 B：管理器 opt-in 两趟](#4-核心机制-b管理器-opt-in-两趟)
5. [三套数据通路的对照](#5-三套数据通路的对照)
6. [数据契约 `StagedFrame` 与校验规则](#6-数据契约)
7. [提交与失败语义](#7-提交与失败语义)
8. [宿主与生命周期（含 `members_active_` 缓存）](#8-宿主与生命周期)
9. [实时路径约束与死锁修复](#9-实时路径约束与死锁修复)
10. [FineMote 机制 → 代码的逐条映射](#10-finemote-机制--代码映射)
11. [阶段 B：编译期契约（另一条线）](#11-阶段-b编译期契约)
12. [**还没有做的工作**](#12-还没有做的工作)
13. [构建、测试与复现](#13-构建测试与复现)
14. [代码导航](#14-代码导航)

---

## 1. 改动清单

### 1.1 新增文件

| 文件 | 行数 | 作用 |
|---|---|---|
| `hierarchical_control/include/hierarchical_control/staged_controller_interface.hpp` | 235 | 控制器侧双阶段接口 + 数据契约（`StagedFrame`、视图/写入器、sink/source） |
| `hierarchical_control/include/hierarchical_control/staged_execution_group.hpp` | 550 | **内核**：拓扑解析、两阶段调度、帧校验、整组提交、库宿主入口 |
| `hierarchical_control/include/hierarchical_control/hierarchy.hpp` | 152 | 计划生成与校验（单根、无环、可达、唯一父），纯 C++ 可脱离 ROS |
| `hierarchical_control/include/hierarchical_control/two_phase_controller_interface.hpp` | 49 | **轻量 mixin**：`update_phase()` / `handle_phase()`，供管理器两趟使用 |
| `hierarchical_control/include/hierarchical_control/static_topology.hpp` | 255 | 编译期拓扑无环 |
| `hierarchical_control/include/hierarchical_control/dimensional_interfaces.hpp` | 296 | 编译期量纲代数与连接规则 |
| `hierarchical_control/include/hierarchical_control/topology_contract.hpp` | 428 | 编译期端口所有权 + 运行期计划生成 |
| `hierarchical_control/include/hierarchical_control/topology_binding.hpp` | 140 | 绑定 → `Spec` 适配 → 真实执行组 |
| `hierarchical_control/include/hierarchical_control/typed_ports.hpp` | 300 | 端口单一声明（类型）→ 生成内核字符串与 `Contract` |

`hierarchical_control` 是 **header-only 独立包**（`package.xml` + `CMakeLists.txt` + `README.md`）。

### 1.2 修改的上游既有文件（**只有 3 个**）

| 文件 | 改动 |
|---|---|
| `controller_manager/include/controller_manager/controller_manager.hpp` | +33 行 include；新增 5 个公开 API（行 135–163；`set_two_phase_execution` 自 2026-09-23 起返回 `controller_interface::return_type`）；私有成员 `staged_group_`(536，`mutable` + 原子发布)、`no_two_phase`(541)、`TwoPhaseEntry`(542–546)、`TwoPhaseAdmission`(548–557)、`two_phase_enabled_`、`two_phase_entries_`(559，`shared_ptr<const vector>` + 原子发布)、`two_phase_rejected_`；私有辅助 `two_phase_admission_reason`/`two_phase_admission`/`two_phase_entries()`/`rebuild_two_phase_entries`/`refresh_two_phase_controllers`/`two_phase_index`(563–573) |
| `controller_manager/src/controller_manager.cpp` | 两个构造函数读 `two_phase_execution` 参数(289–295, 336–342)；新增 `two_phase_admission_reason`(2207)、`two_phase_admission`(2223)、`two_phase_entries()`(2240)、`rebuild_two_phase_entries`(2248)、`refresh_two_phase_controllers`(2307)、`two_phase_index`(2315)、`set_two_phase_execution`(2327)、`two_phase_execution`(2370)、`set_staged_execution_group`(2372)、`clear_staged_execution_group`(2464)、`staged_execution_group`(2472)、`update`(2477)；另有 6 个**非实时**重建点：`unload_controller`、`reorder_controllers`、`switch_controller`、`add_controller_impl`、`set_staged_execution_group`、`clear_staged_execution_group` |
| `controller_manager/test/*` | 新增 4 个测试文件（见 1.3）；`test_two_phase_execution.cpp` 增加 `TestExecutionPathAdmission` 夹具（6 个 R7 准入用例）；既有 2 个滞后用例**未改语义** |

### 1.3 新增兼容 shim（2 个，各 ~20 行）

```
controller_manager/include/controller_manager/hierarchy.hpp            → using 转发到库
controller_manager/include/controller_manager/staged_execution_group.hpp → using 转发到库
```
保留它们是为了让已有的 `controller_manager` 测试与外部引用不破坏。

### 1.4 新增测试

| 位置 | 文件 | 用例数 |
|---|---|---|
| 库 | `test_execution_group.cpp` | 11 |
| 库 | `test_stale_state_cost.cpp` | 7 |
| 库 | `test_scheduling_performance.cpp` | 4 |
| 库 | `test_pass_lower_bound.cpp` | 4 |
| 库 | `test_static_topology.cpp` | 5 |
| 库 | `test_dimensional_interfaces.cpp` | 6 |
| 库 | `test_topology_contract.cpp` | 6 |
| 库 | `test_topology_binding.cpp` | 5 |
| 库 | `test_typed_ports.cpp` | 5 |
| 库 | `test_static_topology_negative.py` | 8 个编译语料 |
| 管理器 | `test_staged_execution_group.cpp` | 6 |
| 管理器 | `test_hierarchy_comparison.cpp` | 5 |
| 管理器 | `test_two_phase_execution.cpp` | 2 |
| 管理器 | `test_upstream_ordering.cpp` | 2 |

**库 10/10 程序（54 gtest）、管理器关键 6/6、standalone 1/1。**

---

## 2. 依赖分层与包结构

```
controller_manager  ──依赖──▶  hierarchical_control  ──依赖──▶  controller_interface
      │                                                              │
      └──────────────────────────────────────────────────────────────┘
```

**无环**。关键决策：

- `hierarchical_control` **不依赖** `controller_manager`（否则无法被"库宿主"使用）；
- `controller_interface/staged_controller_interface.hpp` **已删除**（移入库），且**不留 shim**，
  否则 `controller_interface` 与库之间会形成包依赖环；
- 实现 `StagedControllerInterface` 的插件因此依赖 `hierarchical_control`。

`hierarchical_control` 内部也分层（低→高）：

```
hierarchy.hpp ─────────────┐
staged_controller_interface.hpp
staged_execution_group.hpp ─┴─▶ 阶段 A（运行期双阶段）
two_phase_controller_interface.hpp

static_topology.hpp ─▶ dimensional_interfaces.hpp ─▶ topology_contract.hpp
                                                        ├─▶ topology_binding.hpp
                                                        └─▶ typed_ports.hpp
                                                    阶段 B（编译期契约）
```

---

## 3. 核心机制 A：库内核

### 3.1 控制器侧接口（`staged_controller_interface.hpp`）

控制器可选声明三类端口（**名字列表**）：

```cpp
virtual std::vector<std::string> staged_state_ports() const = 0;      // 本节点发布的状态
virtual std::vector<std::string> staged_reference_ports() const {return {};}  // 从父消费的参考
virtual std::vector<std::string> staged_actuator_ports() const {return {};}   // 写入的硬件命令
virtual StagedCommandSink * staged_command_sink() noexcept {return nullptr;}      // 叶需要
virtual StagedReferenceSource * staged_reference_source() noexcept {return nullptr;}// 根需要
```

两个阶段（**都 `noexcept`**）：

```cpp
// 自底向上：消费子状态，发布本节点状态
virtual return_type update_state_stage(
  const rclcpp::Time &, const rclcpp::Duration &, const StagedContext &,
  const StagedInputView & children, StagedValueWriter state) noexcept = 0;

// 自顶向下：消费父参考，发布子参考 + 执行器命令
virtual return_type update_command_stage(
  const rclcpp::Time &, const rclcpp::Duration &, const StagedContext &,
  const StagedValueView & state, const StagedValueView & reference,
  const StagedReferenceWriter & children, StagedValueWriter actuators) noexcept = 0;
```

**每个阶段每周期最多被调用一次。**

### 3.2 数据不直接传递，只经"值槽"

**控制器之间从不互相持有指针。** 配置期按端口数量一次性分配组拥有的数组：

```cpp
state_values_[i].assign(staged_[i]->staged_state_ports().size(), 0.0);
reference_values_[i].assign(staged_[i]->staged_reference_ports().size(), 0.0);
actuator_scratch_[i].assign(staged_[i]->staged_actuator_ports().size(), 0.0);
actuator_committed_[i].assign(actuator_scratch_[i].size(), 0.0);
```

控制器拿到的是 `StagedValueWriter` / `StagedValueView`（裸指针 + size + 帧指针），
**不是**硬件句柄。

### 3.3 运行路径 `run_ns(now_ns, period_ns)`

完整顺序（`staged_execution_group.hpp` 行 219–402）：

```
0) 前置检查：configured / members_active_ / cycle 未溢出 / now_ns>=0 / period_ns>0
1) 清空三组 frame（state/reference/actuator）
2) 记录根参考来源来源（见 3.5）
3) state 阶段：遍历 plan_.postorder 正向（子先于父）
4) root reference：一次外部快照
5) command 阶段：遍历同一 postorder 反向（父先于子）
6) commit：仅叶节点，全部成功才写硬件
```

#### 3.3.1 state 阶段（子先于父）

```cpp
for (const auto node : plan_.postorder)
{
  const auto & children = children_[node];
  const bool leaf = children.empty();
  std::int64_t oldest = now_ns;
  if (!leaf)
  {
    for (slot...) {
      const auto child = children[slot];
      const auto & child_frame = state_frames_[child];
      // 校验子状态：有效 / 属于本周期 / 采样时间合法 / 未超龄
      if (!child_frame.valid || child_frame.cycle != cycle ||
          child_frame.sample_ns < 0 || child_frame.sample_ns > now_ns ||
          now_ns - child_frame.sample_ns > max_age_ns_)
        return {StagedStatus::state_failed, cycle, node, child_frame.fault_code};

      oldest = std::min(oldest, child_frame.sample_ns);
      // 父的输入视图**直接指向子的状态槽**
      input_views_[node][slot] = StagedValueView(
        state_values_[child].data(), state_values_[child].size(), child_frame);
    }
  }
  // 建本节点的输出写入器，调用控制器
  frame.sample_ns = leaf ? now_ns : oldest;
  StagedValueWriter output(state_values_[node].data(), state_values_[node].size(), &frame);
  staged_[node]->update_state_stage(time, period, context,
      StagedInputView(input_views_[node].data(), children.size()), output);
  // 复合节点不得重新盖章：最旧子采样时间胜出
  if (!leaf) frame.sample_ns = oldest;
  // 有限性检查
  if (!all_finite(state_values_[node])) return {StagedStatus::state_failed, ...};
  frame.valid = true;
}
```

**上行通路要点**：`postorder` 保证子先写、父后读，父在**同一周期**看到 `state_values_[child]`。

#### 3.3.2 root reference（一次快照）

```cpp
auto & values = reference_values_[root];
if (!values.empty()) {
  if (sources_[root] == nullptr ||
      !sources_[root]->read(cycle, now_ns, values.data(), values.size()))
    return {StagedStatus::invalid_input, cycle, root, 0};
  if (!all_finite(values)) return {StagedStatus::invalid_input, cycle, root, 0};
}
```

#### 3.3.3 command 阶段（父先于子）

```cpp
for (std::size_t command_slot = plan_.postorder.size(); command_slot-- > 0;)
{
  const auto node = plan_.postorder[command_slot];
  // 校验本节点 state/reference 帧属于本周期
  for (slot...) {   // 父的输出写入器**直接指向子的参考槽**
    const auto child = children[slot];
    child_writers_[node][slot] = StagedValueWriter(
      reference_values_[child].data(), reference_values_[child].size(),
      &reference_frames_[child]);
  }
  StagedValueWriter actuator(actuator_scratch_[node].data(), ..., &frame);
  staged_[node]->update_command_stage(time, period, context,
      StagedValueView(state_values_[node]...),      // 本周期状态
      StagedValueView(reference_values_[node]...),  // 父给的参考
      StagedReferenceWriter(child_writers_[node].data(), children.size()),
      actuator);
  // 父写出的子参考：盖帧 + 有限性检查
  for (slot...) { child_frame.cycle = cycle; child_frame.sample_ns = now_ns;
                  if (!all_finite(reference_values_[child])) ...; child_frame.valid = true; }
  if (!all_finite(actuator_scratch_[node])) ...;
  frame.valid = true;
}
```

**下行通路要点**：`postorder` 的**反转**天然是"父先于子孙"的合法顺序，
所以**不需要第二份拓扑**。这就是 FineMote「一个注册表、两个方向」的落地。

### 3.4 计划生成（`hierarchy.hpp`）

`build_controller_hierarchy(nodes)` 在**配置期**校验并产出：

- `plan_.postorder`：子先于父的线性序（状态阶段正向、命令阶段反向都用它）
- `plan_.preorder`：**兼容保留，内核不再使用**（看到它在代码里是正常的）
- `plan_.root`、`leaves_`、`children_`、`actuator_offset_`

校验项（**配置期抛 `std::invalid_argument`**）：
单根、无环、无不可达节点、唯一父、重复名、未知父、缺 sink/source、重复参考写者。

**注意**：上游的**多写者参考接口**在配置期被拒绝（一个 reference 接口两个写者）。

### 3.5 两种宿主入口

```cpp
// 管理器宿主：从管理器管理的控制器构建（要求 INACTIVE + 实现 StagedControllerInterface）
static std::shared_ptr<StagedExecutionGroup> create(
    const std::vector<StagedGroupMember> & members, std::int64_t max_age_ns = 0);

// 库宿主：直接从已解析的实例构建，无 ControllerManager 依赖、无 lifecycle
static std::shared_ptr<StagedExecutionGroup> create_library(
    const Spec & spec, std::int64_t max_age_ns = 0);
```

`StagedGroupMember`：

```cpp
struct StagedGroupMember {
  std::string name;
  controller_interface::ControllerInterfaceBaseSharedPtr controller;
  std::vector<std::string> command_interfaces;  // 拓扑来源
};
```

**拓扑从哪来**：`command_interfaces` 里的名字若以另一个成员的名字为前缀，
则该名字是"本成员产生的 reference 接口，被那个成员消费"。因此：

- **交换声明顺序不改变计划**；
- 不需要新的 parent 字段。

---

## 4. 核心机制 B：管理器 opt-in 两趟

### 4.1 为什么要有第二套

库内核要求控制器实现较重的 `StagedControllerInterface`（端口声明 + 帧 + 组提交）。
管理器两趟用的是**轻量 mixin**，控制器改动最小：

```cpp
class TwoPhaseControllerInterface {
public:
  virtual ~TwoPhaseControllerInterface() = default;
  virtual return_type update_phase(const rclcpp::Time &, const rclcpp::Duration &) noexcept = 0;
  virtual return_type handle_phase(const rclcpp::Time &, const rclcpp::Duration &) noexcept = 0;
};
```

**没有端口声明、没有帧、没有组提交。**

### 4.2 开关（2026-09-23 起会**拒绝**而不是静默降级）

```cpp
// 参数（两个构造函数都读，行 289 与 336）
two_phase_execution: bool   // 默认 false
// 或运行时
controller_interface::return_type r = cm->set_two_phase_execution(true);
//   OK    : 已启用（成员集已重建）
//   ERROR : 请求被拒，two_phase_enabled_ 保持原值，原因已记日志
cm->two_phase_execution();   // 查询
```

`set_two_phase_execution(true)` 会**整体拒绝**该请求（而不是排除个别控制器）当且仅当存在
某个实现 `TwoPhaseControllerInterface` 的控制器满足下列任一条：

| 拒绝原因 | 判定 | 为什么必须拒绝 |
|---|---|---|
| `already_staged` | 它是**当前已安装 staged group 的成员** | staged group 与两趟遍历在同一周期各自执行一次 ⇒ 该控制器每周期跑两遍 |
| `unsupported_update_rate` | `get_update_rate() != 0 && != 管理器频率` | 两条新路径**没有**原生循环的 `update_loop_counter_ % controller_update_factor` 门控（`controller_manager.cpp:2400–2416`），会把它按管理器频率调用，静默改变离散化 |

注意：判定依据是**成员身份**而不是"是否实现 `StagedControllerInterface`"。
一个控制器同时支持两种执行模式是合理设计（`TestStagedController` 就是），
真正的危险是**被两条路径同时实际执行**。反方向的镜像校验在 `set_staged_execution_group()` 里：
若 `two_phase_enabled_` 且该成员正在两趟成员集里，则拒绝入组；
组内成员也被施加同一条频率规则（组同样是每周期一次、传管理器周期）。

### 4.3 成员索引：非实时构建、原子发布、实时只读

```cpp
struct TwoPhaseEntry {   // controller_manager.hpp:542-546
  const controller_interface::ControllerInterfaceBase * base;
  hierarchical_control::TwoPhaseControllerInterface * instance;
};
static constexpr std::size_t no_two_phase = SIZE_MAX;   // 行 541
// 不可变快照：构建者先造好新 vector，再一次性发布
std::shared_ptr<const std::vector<TwoPhaseEntry>> two_phase_entries_;  // 行 559

void rebuild_two_phase_entries(const std::vector<ControllerSpec> &);           // 行 2248
void refresh_two_phase_controllers();                                          // 行 2307
std::size_t two_phase_index(const std::vector<TwoPhaseEntry> &, const ... *) const noexcept; // 行 2315
```

- `rebuild_two_phase_entries()` 做 `dynamic_cast<TwoPhaseControllerInterface*>`
  + 准入判定，`std::sort` 后 `std::atomic_store` 发布新快照；**只在非实时线程调用**；
- `two_phase_index()` 在**排好序**的快照上做 `std::lower_bound`（指针比较，不比较字符串），
  `O(log n)`；快照不变，所以实时侧无需任何同步原语；
- `update()` 每周期**只做一次** `std::atomic_load`，把结果放进局部 `shared_ptr`；
  该局部量使旧快照在整个周期内存活，即使另一线程已发布新快照。
  这是本项目的 `O(1)`、**不分配**的发布协议。

### 4.4 `update()` 的实际形态（行 2477 起）

```cpp
// (0) 只读发布一次：一次原子加载 + 局部 shared_ptr 保活（不分配、不加锁）
const auto staged = std::atomic_load(&staged_group_);
const auto two_phase_entries_holder = std::atomic_load(&two_phase_entries_);
const auto & entries = two_phase_entries_holder ? *two_phase_entries_holder : no_entries;

// (1) 若装了 staged group 且无切换挂起，先跑执行组
if (staged && !switch_params_.do_switch) {
  const auto staged_result = staged->run(time, period);
  ...
}

// (2) Pass 1 "Update"：反向遍历 → update_phase（子先于父）
const bool run_two_phase = two_phase_enabled_ && !entries.empty() && !switch_params_.do_switch;
if (run_two_phase) {
  for (std::size_t slot = rt_controller_list.size(); slot-- > 0;) {
    auto & spec = rt_controller_list[slot];
    const auto index = two_phase_index(entries, spec.c.get());
    if (index == no_two_phase || !is_controller_active(*spec.c)) continue;
    if (entries[index].instance->update_phase(time, period) != OK)
      { RCLCPP_ERROR(...); ret = ERROR; }
  }
}

// (3) legacy 单相循环：跳过 staged 成员，也跳过 two-phase 成员
for (auto loaded_controller : rt_controller_list) {
  if (staged && staged->owns(loaded_controller.c.get())) continue;
  if (run_two_phase && two_phase_index(entries, loaded_controller.c.get()) != no_two_phase) continue;
  ... 原生 update()（含 2400–2416 的逐控制器降频门控）...
}

// (4) Pass 2 "Handle"：正向遍历同一列表 → handle_phase（父先于子）
if (run_two_phase) {
  for (auto & spec : rt_controller_list) {
    const auto index = two_phase_index(entries, spec.c.get());
    if (index == no_two_phase || !is_controller_active(*spec.c)) continue;
    if (entries[index].instance->handle_phase(time, period) != OK) ...
  }
}

// (5) 切换真正发生后：刷新缓存 + 置脏标志（行 2455–2461）
```

**要点**：

- 两趟**跳过**切换挂起时的执行（成员集合可能正在变化）；
- legacy 与 two-phase 控制器**可以混用**，但**相对顺序无保证**（已写入 API 注释）；
- **准入**（2026-09-23）：实现 `TwoPhaseControllerInterface` 的控制器若已在 staged group 里、
  或声明了不等于管理器频率的 `update_rate`，`set_two_phase_execution(true)` 返回 `ERROR`
  且**不改变**任何状态；反方向由 `set_staged_execution_group()` 镜像拒绝。
  这保证了一个控制器**每周期最多被一条路径执行一次**；
- **故障包含**（2026-09-24）：**任一** `update_phase` 失败 ⇒ 该周期**整趟命令阶段都不跑**，
  命令接口保持上一周期值，管理器返回 `ERROR`。理由：控制器刚否掉了自己的状态，
  就不该用该状态算命令，其上游也不可信。**这不是**整组原子提交（命令阶段已写出的值不会回滚，
  实测见下）；
- **切换期间**（2026-09-24）：两趟被暂停时，成员**不会**被原生单趟循环接管，
  因此"一个控制器只有一条执行路径"在**任何时刻**都成立（staged 成员本来就是这样）。
  旧实现把这句跳过判断绑在"两趟正在跑"上，切换的若干周期会把成员悄悄退回融合单趟语义；
- 控制器侧的 `update_and_write_commands()` 在两阶段模式下**什么都不做**：
  ```cpp
  if (legacy_) { update_phase(...); return handle_phase(...); }  // 单入口模式自己跑两阶段
  return OK;   // 两阶段模式：由管理器的两趟执行
  ```

---

## 5. 三套数据通路的对照

| | 原生单入口（上游默认） | 管理器两趟（opt-in） | 库宿主执行组 |
|---|---|---|---|
| 控制器入口 | 一个 `update()` | `update_phase` + `handle_phase` | `update_state_stage` + `update_command_stage` |
| 调度者 | `ControllerManager::update()` | 同一个 `update()`，分两趟 | `StagedExecutionGroup::run()` |
| 上行载体 | 控制器自己读硬件/接口 | 控制器自己（Humble：同进程注册表） | 组拥有的 `state_values_` 槽 |
| 下行载体 | 原生 reference interface | 原生 reference interface | 组拥有的 `reference_values_` 槽 |
| 帧校验 | 无 | 无 | **有**（周期号/采样年龄/有限性） |
| 整组提交 | 无 | 无 | **有** |
| 需要 `ControllerManager` 改动 | — | 是（薄适配） | **否** |
| 零分配运行路径 | 否 | 否 | **是**（`run()`） |

---

## 6. 数据契约

```cpp
struct StagedFrame {
  std::uint64_t cycle = 0;      // 生产该值的执行组周期
  std::int64_t  sample_ns = 0;  // 最旧输入采样时间
  bool          valid = false;  // 仅在本周期对应阶段成功后为真
  std::uint32_t fault_code = 0; // 控制器自定义故障码，0 表示未上报
};
```

**由执行组强制（不是靠约定）的规则**：

| 规则 | 实现位置 |
|---|---|
| 叶状态默认采样时间 = 本周期；可用 `set_source_sample_ns()` 上报更旧的真实时间 | `StagedValueWriter::set_source_sample_ns` |
| **复合节点的采样时间被强制覆盖为"最旧子采样时间"** | state 阶段 `if (!leaf) frame.sample_ns = oldest;` |
| 采样时间不得为未来、不得超龄 | state 阶段校验 |
| reference 的 `cycle` 由执行组填写 | command 阶段 `child_frame.cycle = cycle;` |
| 状态/reference/actuator 必须有限，否则该周期失败 | `all_finite(...)` |

**关键约束**：复合节点**无法重新盖章伪装新鲜**——它写什么都会被覆盖。

---

## 7. 提交与失败语义

### 7.1 提交

```cpp
for (slot...) {
  const auto leaf = leaves_[slot];
  auto & scratch = actuator_scratch_[leaf];
  if (scratch.empty()) continue;
  if (sinks_[leaf] == nullptr) return {StagedStatus::command_failed, cycle, leaf, 0};
  if (!sinks_[leaf]->commit(scratch.data(), scratch.size()))
    return {StagedStatus::command_failed, cycle, leaf, actuator_frames_[leaf].fault_code};
  std::copy(scratch.begin(), scratch.end(), actuator_committed_[leaf].begin());
  std::copy(scratch.begin(), scratch.end(), committed_.begin() + actuator_offset_[leaf]);
}
committed_cycle_ = cycle;
return {StagedStatus::committed, cycle, no_node, 0};
```

### 7.2 状态码

| `StagedStatus` | 含义 |
|---|---|
| `committed` | 全部阶段成功且命令已提交 |
| `inactive` | 至少一个成员不 active，**什么都没执行** |
| `invalid_input` | 根参考快照/采样年龄/配置前置条件失败 |
| `state_failed` | 状态阶段失败，或子状态无效/超龄 |
| `command_failed` | 命令阶段失败、值非有限、或提交失败 |
| `exhausted` | 周期计数溢出 |
| `not_configured` | 安装有效计划前调用了 `run()` |

### 7.3 三条必须记住的语义

1. **状态阶段失败 ⇒ 命令阶段完全不进入**（`state_failed` 直接返回）；
2. **命令阶段失败 ⇒ 任何 sink 都不被调用** ⇒ 硬件命令缓冲不会"部分新、部分旧"；
3. **sink 在 `commit()` 中返回 false 属硬件写入故障域**：报告 `command_failed`，
   **但不承诺回滚**已提交的其它叶节点。软件提交 ≠ 总线原子同步 ≠ 物理动作原子。

> **保留上一周期命令不是安全策略。** 调用方必须在看到非 `committed` 结果后
> 执行应用定义的故障动作（案例研究里是 PI 限幅 + 输出限幅）。

---

## 8. 宿主与生命周期

### 8.1 `members_active_` 缓存

```cpp
bool members_active() const noexcept {return members_active_;}
void refresh_member_active_state() noexcept;   // 只在 manage_switch() 之后调用
```

**为什么缓存**：Humble 的 `get_current_state()` 会在
`rclcpp_lifecycle::MutexMap::add()` 里**分配内存**，不能放在每周期路径上。

**代价（已知限制）**：若控制器**不经过 switch** 就自行改变生命周期状态，缓存会过期。
原型阶段接受该限制。

### 8.2 库宿主的惰性绑定

库模式在**第一次 `update()`** 惰性绑定硬件接口并建内核（一次性、非实时）。
**这是已知的待改点**（见 §12）。

### 8.3 安装/清除（管理器宿主）

```cpp
set_staged_execution_group(names, max_age_ns);  // 行 2372
clear_staged_execution_group();                 // 行 2464
staged_execution_group();                       // 行 2472
```

`set_staged_execution_group` 的拒绝条件（配置期，返回 `ERROR` 并记录日志）：

- 未知控制器名；
- 成员已是 **ACTIVE**（必须 INACTIVE 才能加入）；
- 成员未实现 `StagedControllerInterface`（`dynamic_cast` 失败）；
- **（2026-09-23）** 两趟执行已启用且该成员**正在两趟成员集里**——两条路径会同周期执行它；
- **（2026-09-23）** 成员声明了 `!= 0 && != 管理器频率` 的 `update_rate`——组没有原生循环那种
  逐控制器降频门控；
- `StagedExecutionGroup::create` 抛 `std::invalid_argument`（拓扑/端口错误）。

**组的成员必须已经加载、configure 完成且 INACTIVE。**

`clear_staged_execution_group()` 必须在任何成员被卸载或转为 ACTIVE **之前**调用；
它会原子释放组，并**重新发布**两趟成员集（原先被"已入组"排除的控制器因此重新合格）。

安装与清除都用 `std::atomic_store(&staged_group_, ...)` 发布，`update()` 用
`std::atomic_load` 读取一次并复用该局部量。

---

## 9. 实时路径约束与死锁修复

### 9.1 症状

第一版两趟实现在 `update()` 的切换路径里直接调用了**加锁**的成员刷新，结果**死锁**。

### 9.2 根因

> `ControllerManager::switch_controller()` 在**持有 `controllers_lock_`** 的同时，
> 通过条件变量等待实时线程应用切换（`lock controllers` 之后的 `guard`
> 一直存活到 `switch_params_.cv.wait_for(...)`）。
> 因此**实时路径上任何获取该锁的代码都会死锁**：
> 实时线程阻塞在锁上 → 不再推进 `used_by_realtime_controllers_index_` →
> 等待切换的线程在 `wait_until_rt_not_using()` 里**永久自旋**。

### 9.3 修复（含 2026-09-23 的发布协议重做）

| 位置 | 做法 |
|---|---|
| `set_two_phase_execution()`（非实时线程） | 加锁重建成员集并原子发布 |
| **成员集重建的全部调用点** | 全部在**非实时**线程：`unload_controller`、`reorder_controllers`、`switch_controller`（列表切换后）、`add_controller_impl`、`set_staged_execution_group`、`clear_staged_execution_group` |
| `update()`（实时路径） | **只读**：每周期一次 `std::atomic_load(&two_phase_entries_)` 和一次 `std::atomic_load(&staged_group_)`，局部 `shared_ptr` 保活；**不重建、不加锁、不分配** |
| `refresh_member_active_state()` | 特意**不加锁**，同一原因；对 `update()` 取到的局部组对象调用 |

**为什么不再用"脏标志 + 实时重建"**：初版用 `two_phase_entries_dirty_` 让实时线程在下一周期开头
用自己已持有的 `rt_controller_list` 重建。这解决了死锁，却引入两个新问题：
(1) `rebuild` 里有 `reserve/push_back/dynamic_cast/sort` 和旧 vector 析构——**实时路径在分配**；
(2) 非实时线程（`set_two_phase_execution`）也可能写同一个 `two_phase_entries_`，
与实时线程构成**数据竞争**。改为"非实时构建 + 原子发布不可变快照"后两个问题一起消失。

**代价与边界（不要夸大）**：`std::atomic_load/atomic_store(shared_ptr)` 是 C++17 设施
（本仓库 `cxx_std_17`），**不保证无锁**（libstdc++ 用自旋锁池）。实时路径上是一次原子引用计数操作，
**不是零开销**。另外 **TSan 未运行**，本节的正确性是设计论证 + 代码审查，不是并发实测。

**规则**：任何接入 `ControllerManager` 实时循环的新机制都必须遵守——
"非实时线程构建不可变快照，实时线程一次原子加载只读"。

### 9.4 调试提示

rclcpp 装了 SIGTERM 处理器，**挂死时 `timeout` 默认杀不掉**，要用 `timeout -s KILL`。

---

## 10. FineMote 机制 → 代码映射

| FineMote 概念 | 本项目对应 | 位置 |
|---|---|---|
| 线性注册表（构造顺序） | 上游排序后的控制器列表的一个线性化 | `plan_.postorder` |
| `Update()` 正向遍历 | 状态阶段：`postorder` **正向** | `run_ns` state 阶段 |
| `Handle()` 反向遍历 | 命令阶段：同一 `postorder` **反向** | `run_ns` command 阶段 |
| 一个顺序 | `plan_.postorder` 一份，两方向复用 | `hierarchy.hpp` |
| 设备内部状态 | `StagedFrame` + 控制器自有 `update_state_stage` | `staged_controller_interface.hpp` |
| 摄取 vs 产生的分离 | `update_state_stage` / `update_command_stage` | 同上 |
| （FineMote 无） | 帧校验、最旧采样、整组提交 | `staged_execution_group.hpp` |
| （FineMote 无） | 管理器 opt-in 两趟（轻量版） | `controller_manager.cpp` 2477–2610 |

**实现对照**：`for (const auto node : plan_.postorder)` 与
`for (command_slot = plan_.postorder.size(); command_slot-- > 0;)`
——同一数组，两个方向，这就是 FineMote 的机制。

---

## 11. 阶段 B：编译期契约

这条线与 FineMote 的"类型承载顺序"对应，是**附加**工作，不影响阶段 A 的运行路径。

| 头文件 | 提供 | 关键 API |
|---|---|---|
| `static_topology.hpp` | 拓扑无环（类型链） | `Root<N>`、`Descendant<N,P>`、`node_count<>()` |
| `dimensional_interfaces.hpp` | 量纲代数 | `Dimension<L,M,T,A>`、`multiply_t`、`divide_t`、`same_dimension_v` |
| `topology_contract.hpp` | 端口所有权 + 计划 | `Port<N,D>`、`PortList<...>`、`Contract<P,C>`、`BoundNode`、`compose`、`make_leaf`、`build_spec_rows`、`require_ports_are_owned` |
| `topology_binding.hpp` | 绑定 → 执行组 | `to_library_spec(binding)`、`create_library_group(binding, max_age)` |
| `typed_ports.hpp` | 端口单一声明 | `TypedPorts<State,Reference,Actuators,ForChildren,ChildState,HardwareState>`（后三者可选，默认空）、`TypedPortsMixin<C,P>`、`contract_of_t`、`declarations_are_compatible`、`verify_ports_match_interface`、`verify_ports_match_contract` |

**边界（务必记住）**：

- 只覆盖**静态声明**的拓扑；**YAML / `pluginlib` 不覆盖**；
- 所有权检查只验证 owner **是否存在**，**不验证** owner 是否**真的导出**该端口；
- **状态端口的语义区分未下探到类型层**；
- 不检查**单位**（米/毫米量纲相同）。

定位：**编译期能力 + 运行期回退**，不是取代运行期校验。

---

## 12. 还没有做的工作

### 12.1 明确未实现的功能

| # | 未做项 | 说明 | 相关位置 |
|---|---|---|---|
| 1 | **多频 / 异步回调** | 模型与实现均未支持；`run_ns` 假定单频同步 | `FORMAL_MODEL.md` §8 威胁 3 |
| 2 | **动态拓扑** | 计划在配置期固定；运行期不能增删成员 | `hierarchy.hpp` |
| 3 | **生命周期回滚顺序** | 部分失败的成员回滚未实现 | — |
| 4 | ~~两趟模式的故障一致性验证~~ **已于 2026-09-24 完成（结论是"没有该保证"）** | 新增 `two_phase_mode_has_no_group_commit`：命令趟父先子后、`handle_phase` 直写 command handle、管理器无缓冲，所以后段失败时**前段写入已经生效**。实测 `mid` 的已认领接口在 leaf 命令阶段失败的那一周期从 `-1.875` 变成 `-5.5625`。**两趟模式不得声称"整组提交不部分提交"**；该保证只有 staged group 有（`test_staged_execution_group.failure_never_partially_commits`） | `test_two_phase_execution.cpp` |
| 5 | ~~实测两趟的额外开销~~ **已于 2026-09-24 完成** | 同一套三控制器级联、同一管理器，翻转 `two_phase_execution` 与 `two_phase_legacy` 后各测 200 个周期（`read+update+write` 中位数）：**单趟 5.377 µs / 两趟 8.794 µs，比值 1.64×**。两趟更慢，唯一收益是消掉那一周期滞后 | `test_two_phase_execution.two_pass_costs_one_extra_traversal` |
| 6 | **多执行组** | 同时只支持一个 `staged_group_`（评审也建议暂不扩展，见 `REVIEW_HUMBLE_WORK_2026-09-23.md` §4.4） | `controller_manager.cpp` |
| 7 | **URDF 参与拓扑校验** | URDF 只用于 `ResourceManager` 提供资源 | — |
| 8 | **真实硬件故障动作** | 只有 mock hardware 验证；真实总线的故障语义未定义 | — |
| 9 | **硬实时 WCET / deadline miss** | **只测了分配次数与中位耗时**，非实时虚机 | `HIERARCHY_FAIR_COMPARISON.md` §5 |
| 10 | **`Spec::parents` 接到配置（YAML/参数）** | 内核支持该字段，但**没有**配置入口。注意：两趟之后该字段**不是必需的** | `staged_execution_group.hpp` |
| 11 | ~~库宿主的显式激活期建内核~~ **已于 2026-09-24 修复** | 内核改在 `on_activate()` 建（管理器先 `assign_interfaces()` 再 `activate()`，所以借用的接口此时已就绪），`on_deactivate()` 释放以免跨激活边界复用；`update()` 不再有任何建内核分支，缺内核直接返回 `ERROR`。`test_hierarchy_comparison` 新增断言：**激活后第一个 `update()` 分配数 = 0**（旧实现在这一周期分配） | `test_composite_library/generic_composite_controller.cpp` |
| 12 | **`members_active_` 的非 switch 状态变化** | 缓存会过期，仅记为限制 | §8.1 |
| 13 | ~~阶段 B 接到执行组端口声明~~ **已于 2026-09-24 打通** | `TypedPorts<State, Reference, Actuators, ForChildren>` 四组端口；mixin 从类型**生成**内核要的三张字符串表；新增 `verify_ports_match_contract` 与 `topology_binding::verify_binding_ports`（沿绑定**树**逐节点把**运行期端口**与**已检查的 `Contract`** 对齐；C 项后由 `create_library_group` 自动调用）。同时修掉一个真实缺陷：旧的三组端口模型把"自身状态/接收 reference/写进子节点的 reference"混在一起，状态槽**多算**（1 个真实状态得到 3 个槽），而内核会给状态槽预填 NaN 并要求全部写出——只写真实状态的控制器会**每周期 `state_failed`**；`declarations_are_compatible` 也把正确的 wheel/tire 判为不兼容 | `typed_ports.hpp`、`topology_binding.hpp`、`PORT_DIMENSIONS.md` §1.1 |
| 14 | ~~父子状态边的静态建模~~ **已于 2026-09-24 补齐** | 端口声明加 `ChildState`（父声明从子读哪些状态），并拆成按边的谓词：`reference_declarations_agree`（`ForChildren` vs 子 `Reference`）、`state_declarations_agree`（`ChildState` vs 子 `State`）、`declarations_are_compatible`（两者）。两条边可**分别**断言，失败时能指出是哪条边；测试含"父声明错误量纲的子状态"负向用例（且 reference 边仍通过，证明两个检查独立、都不空洞）。`ChildState` 不进内核，只是父对输入的期望。**2026-09-24 评审 B 后已推广到任意分叉树**：父侧 `ForChildren`/`ChildState` 是一张扁平表，要求等于"各孩子声明**按孩子顺序的拼接**"，检查是逐段的（见 §12.4 #8） | `typed_ports.hpp`、`PORT_DIMENSIONS.md` §1.1/§3、`test_typed_tree.cpp` |
| 15 | ~~深链编译成本优化~~ **已调查并否决（2026-09-24）** | "沿父链查询把 O(d²) 降到 O(d)" 已实现并实测：**慢 3.24 倍**（深度 64 链，template instantiation 0.17 s → 0.55 s，GGC 13 MB → 20 MB）。抽象步数少不等于实例化代价低——递归函数模板每层祖先都是一次独立实例化。**故意保留按值复制**；被否决的实现与复现脚本留在 `research/static_topology_variants/` | `COMPILE_COST.md` §3 |
| 16 | ~~诊断携带可读节点名~~ **已于 2026-09-24 完成** | `StagedExecutionGroup::node_name(index)` 返回组内字符串（越界返回 `"<none>"`，不抛异常、不分配）；管理器失败日志现在是 `at node 2 ('tp_module') (status 4, fault 0x52)`。`StagedResult` 仍只带索引，所以 `run_ns` 保持零分配 | `staged_execution_group.hpp`、`controller_manager.cpp` |

### 12.2 环境阻塞（不是"没做"，是"做不了"）

| 项 | 阻塞原因 |
|---|---|
| **Jazzy/Rolling 数据通路复现** | 本机 Ubuntu 22.04、**无 Docker**；无法原生安装 Jazzy。需 Docker 镜像或 24.04 环境 |
| 上游"有 bug"的任何主张 | 取决于上一项。**在此之前只能说"未被覆盖"** |

### 12.3 方法学上尚未解决的问题

| # | 问题 |
|---|---|
| 1 | **Gazebo 真值指标不可靠**：`/model_states` 真值位姿沿对角线以约 2 倍于航位推算的速度移动，说明底盘真实运动与轮子转动不一致（打滑/被抬起/轴向），**不是测量代码问题**。已修正 spawn 高度，但**尚未**核对两轮关节轴方向与真值 z/四元数 |
| 2 | **耗时不稳**：非实时虚机，`max` 波动 8–300 µs；未报 p95/p99；未做 CPU 绑定；未做消融（帧校验 / 整组提交各占多少） |
| 3 | **接线成本只测了一种拓扑变化（加叶）**：未测插入中间层、叶换子树、共享模块 |
| 4 | **代码量统计只算行数**：没有区分难度或出错概率 |
| 5 | ~~库宿主惰性建内核~~ **已修复**（见 12.1 #11）；仍需注意 `update()` 里的 `std::atomic_load(shared_ptr)` 不是无锁操作 |

### 12.4 已知的未修缺陷 / 妥协

| # | 妥协 |
|---|---|
| 1 | `refresh_member_active_state()` 不加锁 ⇒ 非 switch 的状态变化会让缓存过期 |
| 2 | ~~库宿主第一次 `update()` 惰性建内核（一次性分配）~~ **已修复**（见 12.1 #11） |
| 3 | legacy 与 two-phase 混用时**相对顺序无保证**（已写入 API 注释）。**已收紧（评审 E）**：一条参考边两端恰有一端是两趟成员时**拒绝启用/配置/激活**，所以"混用导致某条边用旧状态"不再可能——混用只剩下**没有任何参考边相连**的成员，它们之间本来就没有顺序要求 |
| 7 | **模式/成员/计划的发布已是一个 generation**（2026-09-26，评审 D 下半）：三者合并成不可变的 `ExecutionGeneration`，`update()` 每周期**一次 `atomic_load`**，每次配置变更**一次 `atomic_store`**（启用/禁用/安装/清除 staged group 各自一次替换，不再有先发成员再置位的顺序窗口），被拒请求不发布；新增 `execution_generation()` 单调 id 作为可观测契约。**列表本身**仍是上游双缓冲，故停止期配置的约束仍有效。**另**：激活整组回滚 `set_atomic_activation(true)`（默认关闭，见 §12.4 #15） |
| 8 | ~~静态 binding 只表达**链**~~ **已修复（评审 B）**：`BoundNode` 是变参（`std::tuple<Children...>`），`compose`/`make_leaf` 支持任意分叉；父侧 `ForChildren`/`ChildState` 与"各孩子声明按孩子顺序的拼接"逐段比较；`compose` 在 typed 参与者之间**自动**执行该检查；七节点验收树见 `hierarchical_control/test/test_typed_tree.cpp` |
| 9 | ~~两趟与 legacy 的**跨模式依赖**未拒绝~~ **已修复（评审 E）**：跨模式参考边在准入层被拒绝（`two_phase_rejections`），启用期间晚配置的不合格成员使 `configure_controller` 返回 ERROR，被拒成员由 `two_phase_rejected_controllers()` 暴露 |
| 10 | 上游 `check_preceeding_controllers_for_deactivate()` **拒绝**在父节点仍激活时停用链式子节点 ⇒ 评审判据里的"组内成员停用后父节点继续运行"在 manager API 层**不可达**（已用 `a_chained_child_cannot_be_deactivated_while_its_parent_runs` 固定这一事实）；内核要保证的是"两趟 pass 跳过非激活成员"，用两个无参考边的成员验证（`a_deactivated_member_stops_running_while_other_members_continue`） |
| 15 | **激活整组回滚（可选）**：`set_atomic_activation(true)`（参数 `atomic_activation`，**默认 false**）。上游把一次 switch 的 activate 集合尽力而为地逐个激活，失败者跳过、成功者保持激活——上游自己的 spawner 测试依赖这一点，所以默认不动。开启后，本次 switch 激活成功的控制器在任一成员失败时被 `deactivate()` + `release_interfaces()` 撤销，调用方看到的是"什么都没激活"而不是半棵树。作用域仅限**本次 switch 激活的**控制器；此前已激活的不动。见 `test_atomic_activation.cpp`（5 用例）与 `REVIEW_REQUIREMENTS_RESPONSE_2026-09-24.md` §D.3 |
| 14 | **编译进二进制的控制器可经 `load_controller(type)` 加载**（2026-09-26）：新增 `StaticControllerRegistry`（工厂注册，不是实例表）+ `ControllerManager::set_static_controller_registry`；`load_controller` 先查注册表，命中后走**同一个** `add_controller_impl()`，因此生命周期/接口 claim/全部准入检查与 pluginlib 控制器完全共用。类型声明 `ControllerT::manifest` 时，注册表还能**不构造对象**就枚举其编译期拓扑与接口需求 |
| 13 | **真实 manager 的 TSan（评审 D 第二半）已补**：只给 `controller_manager` 包插桩（`controller_manager/test/run_tsan_real_manager.sh`），跑真实 `update()` vs `switch_controller()`/`set_two_phase_execution()`。**先测出 4 条数据竞争**，全部在上游握手字段：`switch_params_.do_switch`（`update()` 读 vs `switch_controller()` 写）、`switch_params_.activate_asap`（`manage_switch()` 读 vs 写）、`RTControllerListWrapper::used_by_realtime_controllers_index_`（等待循环读 vs 实时线程写）、`updated_controllers_index_`（实时线程读 vs `switch_updated_list()` 写）。**改成 `std::atomic` + 发布/观察加 release/acquire 后归零**。**边界**：依赖库未插桩，其内部竞争看不见；剩余 lock-order 报告全部来自它们 |
| 11 | **manager 列表顺序**是两趟 pass 的前置条件，而它由上游 `controller_sorting()` 决定：**认领 0 个 command interface 的 chainable 控制器会被排在它的父节点之前**，于是 pass 1 反向、pass 2 正向都对那条边**同向走错**（实测：父节点读到上一周期的子状态，而"每周期一次"的计数完全正常）。已把"父下标 < 子下标"变成准入条件（`unschedulable_order`），启用/配置/激活会失败并点名两端；`staged` 组不受影响（顺序由声明的边推导） |
| 12 | **同一个控制器对象被登记为两个节点**（上游 `add_controller()` 只查名字）会让两趟 pass 每阶段调用同一个对象两次（实测 3 周期 6 次 `update_phase`）。库/`staged` 路径由**内核**的实例唯一性检查拦住；两趟路径**不经过内核**，因此在准入层新增 `duplicate_instance`（两个名字都记为拒绝）。编译期绑定相应由 `rows_are_well_formed` 拒绝（"same controller instance"） |
| 6 | 两趟模式的**非实时重构点**（switch/load/unload）在 `two_phase_enabled_ == false` 时**直接返回**：否则那些无谓的 `dynamic_cast`/`make_shared`/`sort` 会拉长切换，改变实时循环在切换期间完成的周期数——`test_controllers_chaining_with_controller_manager` 的计数器断言会因此失败（**这是实测到的回归**，已修） |
| 4 | `plan_.preorder` 仍被赋值但内核不再使用（兼容保留） |
| 5 | `test_controllers_chaining_with_controller_manager` 是**既有 flaky 测试**：它断言精确的 `internal_counter`，而计数由 `ControllerManagerFixture::startCmUpdater` 的 10 ms 睡线程 tick 次数决定（一次切换需要 2 个 tick，偶尔变成 3 个）。**实测：空闲时 0/4 运行全绿，加 2 个 CPU 忙循环后 4/4 全绿**——与调度实现无关。详见 `doc/CODE_AUDIT_SCHEDULING_METAPROGRAMMING.md` §3.1。本次改动未触碰原生 chaining 逻辑；要根治需改该 fixture 的驱动方式（会波及所有用它的上游用例） |

---

## 13. 构建、测试与复现

### 13.1 构建

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager \
             --packages-select hierarchical_control_case_study \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"        # 必需：受限环境下 ~/.ros 不可写，
                                          # 否则 gtest 在 SetUpTestSuite 抛异常并在退出时段错误
```

**编译时间**：全量重编 `controller_manager` ≈ **15–16 分钟**；
只改测试文件增量 ≈ **1 分钟**；改库头文件会触发全量重编（内核 header-only）。
（实测本机一次重编约 3–4 分钟，取决于负载。）

### 13.2 测试

```bash
# 库（10 个程序 / 54 gtest）
ctest --test-dir build/hierarchical_control --output-on-failure

# 管理器关键 6 个
ctest --test-dir build/controller_manager \
  -R "test_staged_execution_group|test_hierarchy_comparison|test_two_phase_execution|test_upstream_ordering|test_cycle_tree_contract|test_hierarchy$" \
  --output-on-failure

# 非 ROS 独立标量内核
cmake -S research/cycle_tree -B build_standalone -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_standalone -j4 && ctest --test-dir build_standalone --output-on-failure

# 编译期负例语料（8 条，含必须编译通过的对照）
python3 hierarchical_control/test/test_static_topology_negative.py
```

### 13.3 案例研究

```bash
bash case_study/scripts/phase_a.sh                       # 仅原生 chaining（对照组）
bash case_study/scripts/phase_b.sh 50 true  40           # 两趟
bash case_study/scripts/phase_b.sh 50 false 40           # 单趟
python3 case_study/scripts/measure_tracking.py <duration> <rate> [model] [mode]
```

---

## 14. 代码导航

### 14.1 想改/看懂"两阶段调度"

| 目标 | 文件:行 |
|---|---|
| 运行路径全貌 | `staged_execution_group.hpp:219`（`run_ns`） |
| 状态阶段 | `staged_execution_group.hpp:243–296`（根参考快照在 298–316） |
| 命令阶段 | `staged_execution_group.hpp:318–385` |
| 提交 | `staged_execution_group.hpp:386–409` |
| 计划生成/校验 | `hierarchy.hpp`（`build_controller_hierarchy`） |
| 缓冲区分配 | `staged_execution_group.hpp:463–491`（`assign` 在 477–480） |

### 14.2 想改/看懂"管理器接入"

| 目标 | 文件:行 |
|---|---|
| 公开 API | `controller_manager.hpp:135–154` |
| 私有成员 | `controller_manager.hpp:525–543` |
| 参数读取 | `controller_manager.cpp:289–295`、`336–342` |
| 条目重建 | `controller_manager.cpp:2200–2219` |
| 索引查找 | `controller_manager.cpp:2229–2238` |
| 开关 | `controller_manager.cpp:2240–2249` |
| 执行组安装 | `controller_manager.cpp:2251–2321` |
| **`update()` 全部改动** | `controller_manager.cpp:2323–2465` |

### 14.3 想改/看懂"编译期契约"

从 `PAPER.md` 附录 E 或下列文档入手，再读对应头文件：

| 文档 | 头文件 |
|---|---|
| `doc/METAPROGRAMMING_CONTRACT.md` | `static_topology.hpp` |
| `doc/DIMENSIONAL_INTERFACES.md` | `dimensional_interfaces.hpp` |
| `doc/TOPOLOGY_CONTRACT_JOIN.md` | `topology_contract.hpp`、`topology_binding.hpp` |
| `doc/PORT_DIMENSIONS.md` | `typed_ports.hpp` |
| `doc/COMPILE_COST.md` | `test/measure_compile_cost.py` |

### 14.4 数据来源文档

| 主题 | 文档 |
|---|---|
| 上游行为实证 | `doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` |
| 两趟 vs 单趟定量 | `doc/TWO_PASS_VS_SINGLE_PASS.md` |
| 同算法公平对照（耗时/分配/故障） | `doc/HIERARCHY_FAIR_COMPARISON.md` |
| 接线成本与库宿主 | `doc/WIRING_COST_ANALYSIS.md` |
| 滞后代价 | `doc/CONTROL_COST_OF_LAG.md`、`doc/SCHEDULING_PERFORMANCE_COST.md` |
| Gazebo 案例 | `doc/GAZEBO_CASE_STUDY.md` |
| 最少趟数理论 | `doc/PASS_LOWER_BOUND.md` |
| 为什么没有性能收益 | `doc/WHY_NO_SPEEDUP.md` |
