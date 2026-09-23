# 拓扑契约接合：一份定义同时驱动编译期检查与运行期计划

日期：2026-09-22
状态：**原型已完成并测试**
代码：`hierarchical_control/include/hierarchical_control/topology_contract.hpp`
测试：`hierarchical_control/test/test_topology_contract.cpp`（6 用例）+
语料 `static_topology_negative/compile_fail_owner_unknown.cpp`、
`compile_fail_port_unqualified.cpp`
上位动机：`HANDOFF_MANUAL.md` P2.5 第 1 项；`FORMAL_MODEL.md` 推论 2

---

## 0. 解决了什么

之前三件元编程设施各自独立：

| 设施 | 回答的问题 |
|---|---|
| `static_topology.hpp` | 层级**无环**吗？（构造图） |
| `dimensional_interfaces.hpp` | 接口**量纲一致**吗？ |
| —— | 运行期 `Spec`（names / instances / parents）**由调用者手写** |

第三条是缺口：**编译期被检查过的拓扑，与真正交给内核的计划，可能不一致**
（手写三个 vector 时漏一个、顺序错、父名写错）。

本原型让**一份 `BoundNode` 链同时产出**：

1. 编译期检查（无环沿自 `Node` 类型；量纲在 `Port` 类型里；**所有权**在本文件）；
2. 运行期 `SpecRows`（`build_spec_rows`），**顺序即父先于子**，与内核计划所需一致。

---

## 1. 所有权不变式（本文件的核心）

`ros2_control` 的接口名形如 `<owner>/<local>`。这条命名约定正是让**本项目实测的缺陷**
在编译期可判定的关键：

> `controller_manager/test/test_upstream_ordering.cpp` 在 `"ord_child/state"` 上声明状态边，
> 而**没有任何控制器导出它**，`configure_controller` **静默接受**
> （`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` §9）。

本文件强制：

> **(i)** 每个被引用的端口名必须是 `<owner>/<local>` 形式；
> **(ii)** 每个 `<owner>` 必须是**该拓扑中存在的控制器**。

于是 `"ord_child/state"` 只在 `ord_child` **确实是本组内控制器**时才被接受；
owner 拼错、或引用了组外的控制器，**编译失败**。

### 1.1 必须说清的边界

**它不验证 owner 自己是否真的导出了该端口。** 那取决于控制器运行期的
`export_reference_interfaces()`，在 `pluginlib` 动态加载下**不是编译期属性**。
因此这是**必要条件而非充分条件**，运行期校验**必须保留**。

这一点很重要：我在上一轮的 `DIMENSIONAL_INTERFACES.md` §4 末尾指出
"本项目实测的缺陷是**声明了没人导出的接口**"，而量纲层解决不了它。
**本文件把该缺陷推进了一步**——从"完全不检查"到"**检查 owner 是否存在**"——
但**仍不是完整解**，因为"owner 是否真的导出"仍要运行期确认。

---

## 2. 结构

```cpp
// 拓扑（类型链，无环性由 Node 类型保证）
using chassis = st::Root<chassis_n>;
using wheel   = st::Descendant<wheel_n, chassis>;

// 端口（名 + 量纲）
using wheel_target = tc::Port<wheel_target_n, dm::LinearVelocity>;  // "wheel/target"
using wheel_travel = tc::Port<wheel_travel_n, dm::Position>;        // "wheel/travel"

// 契约（各自视角的 produced / consumed）
using chassis_contract = tc::Contract<tc::PortList<wheel_target>, tc::PortList<wheel_travel>>;
using wheel_contract   = tc::Contract<tc::PortList<wheel_travel>, tc::PortList<wheel_target>>;

// 绑定（叶 → 逐层向上组合）
constexpr auto leaf = tc::make_leaf<wheel, wheel_contract>(&wheel_inst);
constexpr auto root = tc::compose<chassis, chassis_contract>(&chassis_inst, leaf);

// 编译期检查 + 运行期计划，来自同一份定义
tc::require_ports_are_owned<decltype(root)>();
auto rows = tc::build_spec_rows(root);   // names / instances / parents，父先于子
```

**为什么 `compose` 是自底向上**：`BoundNode<Node, Contract, Next>` 需要已知 `Next` 才能做检查，
所以先建叶、再逐层向上组合。这样每一层的检查都对着**真实的**子绑定做，而不是悬空类型。

---

## 3. 运行期计划

`build_spec_rows` 产出三个并行 vector：

| 数组 | 内容 |
|---|---|
| `names` | 节点名，**根在前**（父先于子） |
| `instances` | 与 `names` 同序的控制器实例指针（本文件只持有 `void*`，不依赖 `controller_interface`） |
| `parents` | 父名；根为空串——与 `StagedExecutionGroup::Spec` 的约定一致 |

`rows_are_well_formed` 在**不调用内核**的前提下复核内核会强制的那些不变式：
长度一致、名字非空且唯一、**恰好一个根**、父必须存在、**节点不能是自己的父**。

**为什么值得复核**：这样"绑定产生的计划"与"内核接受的计划"之间的差异会在
**进入内核之前**暴露，而不是靠内核抛异常。

---

## 4. 已验证的行为

正例（`test_topology_contract.cpp`，6 用例，全部通过）：

- 所有权在编译期通过；
- `binding_depth` / `has_child` / `name()` 都是编译期常量；
- `build_spec_rows` 产出的名字、父名、实例指针**与拓扑一致**（含根为空父）；
- 产出的 rows 满足 `rows_are_well_formed`；
- **`rows_are_well_formed` 非空洞**：逐一构造 7 种畸形 rows
  （长度不等、无根、双根、重名、自父、父不存在、空）**全部被拒**；
- 单节点退化拓扑合法；
- `owner_of` 的边界（无斜杠、空 owner、多个斜杠取第一个）。

负例（编译语料，2 个新文件）：

| 文件 | 期望 | 实际 |
|---|---|---|
| `compile_fail_owner_unknown.cpp` | 失败 | ✅ 失败（OWNERSHIP VIOLATION） |
| `compile_fail_port_unqualified.cpp` | 失败 | ✅ 失败（OWNERSHIP VIOLATION） |

---

## 5. 与内核的接合（**已完成**）

最后一步由 `topology_binding.hpp` 完成：

```cpp
auto group = hierarchical_control::topology_binding::create_library_group(binding);
```

它把 `SpecRows.instances`（`void*`）适配成 `StagedControllerInterface*`，调用
`StagedExecutionGroup::create_library`，返回**真正可运行的执行组**。

**为什么单独放一个头文件**：`topology_contract.hpp` 刻意**不**依赖
`controller_interface` 与 rclcpp，以便在轻量翻译单元里使用；
与内核的耦合只存在于 `topology_binding.hpp`，且只有真正要跑执行组的调用者才包含它。

### 5.1 各层检查的分工（不重复、不遗漏）

| 层 | 检查内容 |
|---|---|
| **编译期**（`Binding` 类型） | 无环（`Node`）、量纲与**端口所有权**（`Contract`/`Port`） |
| **适配层**（`topology_binding`） | 实例非空；`rows_are_well_formed`（长度/唯一性/单根/父存在/非自父），错误信息**指明问题** |
| **内核**（`create_library`） | 未知父、恰好一个根、无环、无不可达节点、控制器接口契约（sink/source） |

**内核自身的校验没有被绕过**——`create_library` 仍然照常校验；
适配层只是把"绑定自己就能看出"的问题**更早、更清楚地**报出来。

### 5.2 端到端验证

`test/test_topology_binding.cpp`（5 用例，全部通过）刻意**真的构造执行组**，
而不只是比较 vector——因为值得抓的失效模式是
"**适配后的 Spec 通过了我自己的检查、却被内核拒绝**"，只有真实构造才能暴露它：

- 适配后的 Spec 的 names/parents/instances 与绑定一致（根在前）；
- **内核接受该 Spec**：`create_library_group` 返回非空、`members_active()` 为真、`size()==3`；
- **构造出的组真的会跑**：`run_ns(0, 1e6)` 返回 `StagedStatus::committed`，
  且三个成员的 state/command 阶段各被调用**恰好一次**；
- 畸形计划被拒并给出原因（空、null 实例、双根、重名、自父）；
- 单节点绑定合法。

### 5.3 顺带发现（本轮）

单节点用例最初复用了三节点绑定的 `root_contract`（它消费 `mid/state`），
于是"只含 root 的绑定却声明了 mid 的端口"**被所有权检查在编译期拒绝**。
这是**检查按预期工作**的例证：它抓到的正是"声明了组内不存在的依赖"这一类错误。
该用例已改为使用自己的无端口契约，并在注释里记录了这个过程。

## 6. 复现

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ctest --test-dir build/hierarchical_control --output-on-failure     # 8/8
python3 hierarchical_control/test/test_static_topology_negative.py  # 编译语料 8 条
```

---

## 7. 不能声称 / 可以声称

**不能声称**：

- ❌ "解决了上游静默接受未导出接口的缺陷"：本文件只检查 **owner 是否存在**，
  **不检查 owner 是否真的导出**（§1.1）；
- ❌ "执行组已由编译期拓扑驱动"：缺"计划 → 内核"的适配（§5）；
- ❌ "覆盖 YAML / `pluginlib` 动态拓扑"：不覆盖。

**可以声称**：

- ✅ 一份编译期拓扑定义**同时**驱动编译期检查与运行期计划，二者不会漂移；
- ✅ 端口 **owner 必须存在于拓扑中**（含 `<owner>/` 前缀格式检查），
  这是对上游静默行为的一步收敛；
- ✅ 计划的不变式可在**进入内核之前**复核，且该复核**非空洞**（7 种畸形输入全被拒）；
- ✅ 纯新增，不影响现有路径。
