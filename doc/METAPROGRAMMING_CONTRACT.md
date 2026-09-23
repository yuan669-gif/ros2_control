# 元编程契约：把拓扑校验前移到编译期（原型）

日期：2026-09-22
状态：**原型已完成并测试**（`hierarchical_control/static_topology.hpp`）
测试：`hierarchical_control/test/test_static_topology.cpp`（5 用例）+
`hierarchical_control/test/test_static_topology_negative.py`（编译语料，4 个文件）
上位动机：`HANDOFF_MANUAL.md` §P2.5；`FORMAL_MODEL.md` 推论 2；FineMote 的类型承载顺序

---

## 0. 一句话结论

> 对**静态声明**的拓扑，`Descendant<Name, Parent>` 使"含环的层级"**在编译期被拒绝**：
> 自环与任意长度的祖先复用都触发 `static_assert`，
> 且**批注了明确的诊断**。这把推论 2 从"配置期检查后拒绝"强化为"**写不出来**"。

**但覆盖面有硬边界**（§4）：它只管**构造（树/参考）图**；
"非祖先的状态边"（共享子节点被另一个父消费）在这个表示里**根本无法表达**，
因此**仍必须**由运行期校验负责。定位是"**编译期能力 + 运行期回退**"，**不是取代**。

---

## 1. 表示

```cpp
struct chassis_name { static constexpr auto value = st::NameOf("chassis"); };
struct wheel_name   { static constexpr auto value = st::NameOf("wheel");   };

using chassis = st::Root<chassis_name>;
using wheel   = st::Descendant<wheel_name, chassis>;
```

每个节点类型在编译期暴露：

| 成员 | 含义 |
|---|---|
| `name_type` / `parent_type` | 自身名字类型 / 父节点类型（根为 `void`） |
| `depth` | 根到自身的节点数（根为 1） |
| `ancestry` | `std::array<std::string_view, depth>`，根在前、自身在末 |

`ancestry` 由父的 `ancestry` 加自身名字构成，是**类型级**属性；
因此所有检查都是模板实参检查，在**使用点**报错。

---

## 2. 保证与证明

> **命题 M1.** 若 `Descendant<NameT, Parent>` 通过编译，
> 则 `NameT` 的名字**不出现**在 `Parent` 的 `ancestry` 中。

**实现方式.** `Descendant` 内部：

```cpp
static_assert(!ancestry_contains<Parent, NameT>(),
  "static_topology: CYCLE -- this node's name already occurs in its parent's ancestry, ...");
```

> **推论 M1（无环性）.** 通过编译的层级，其构造图**无环**。

**证明.** 设构造图含环 `v_0 → v_1 → … → v_k → v_0`（`→` 表示"是…的父"）。
由构造方式，`v_{i+1}` 的 `ancestry` 包含 `v_i` 的名字，
故 `v_0` 的 `ancestry` 包含 `v_1, …, v_k` 中每一个的名字（沿链传递）。
又 `v_0 → v_1` 的存在意味着 `v_1` 由 `v_0` 构造，于是 M1 要求 `v_1` 的名字不在 `v_0` 的
`ancestry` 中；但环给出 `v_0 → … → v_k → v_0`，即 `v_0` 是自身的祖先，
故 `v_0` 的名字在 `v_0` 的 `ancestry` 中——矛盾。
更直接地：**自环**（`v_1` 的名字 = `v_0` 的名字）与**祖先复用**（`v_1` 的名字是 `v_0` 的某个祖先）
都恰好是 `ancestry_contains` 判定为真的两种情形。∎

**注意这不是"不可表达"的最强形式**。我在待办里最初设想的是"循环在类型上根本写不出"；
实际上 2-环**是可以写出的类型表达式**（`Descendant<A, Descendant<B, Root<A>>>`），
只是被 `static_assert` **拒绝**。两者结论相同（含环拓扑无法通过编译），
但**论证方式不同**，文档与论文必须按实际实现写：**"编译期拒绝"，而不是"不可表达"**。
（这也是我在 `PAPER_SKELETON.md` 里标记"待严格论证"的那一点，现已澄清。）

---

## 3. 测试语料（负例必须真编译）

正例在 `test_static_topology.cpp`（5 个 gtest）。
负例**不能**放进同一个翻译单元（它们是硬编译错误），因此放在
`test/static_topology_negative/`，由 `test_static_topology_negative.py` 编译并核对结果：

| 文件 | 期望 | 实际 |
|---|---|---|
| `must_compile_control.cpp` | **编译通过** | ✅ 通过 |
| `compile_fail_self_loop.cpp` | 失败 | ✅ 失败（诊断为 CYCLE） |
| `compile_fail_two_cycle.cpp` | 失败 | ✅ 失败（诊断为 CYCLE） |
| `compile_fail_three_cycle.cpp` | 失败 | ✅ 失败（诊断为 CYCLE） |

**`must_compile_control.cpp` 是必需的**：没有它，一个"把所有文件都判为失败"的检查器
会伪装成通过。脚本还额外核对失败**确实来自 CYCLE 那条 `static_assert`**，
而不是别的编译错误。

实际诊断（节选）：

```text
error: static assertion failed: static_topology: CYCLE -- this node's name already occurs in its
parent's ancestry, so the topology would contain a cycle and is rejected at compile time
```

---

## 4. 覆盖面边界（**最重要的一节**）

| 情形 | 本头文件 | 说明 |
|---|---|---|
| 树/参考方向的环 | **编译期拒绝** | 命题 M1 + 语料验证 |
| 自环 | **编译期拒绝** | 同上 |
| 分支（一个父多个子） | 允许，且各分支 `ancestry` 独立 | 对应 POV 底盘形状 |
| **非祖先的状态边**（共享子被另一个父消费） | **无法表达** | 该边在此表示中**不存在**，既不许可也不拒绝 |
| YAML / `pluginlib` 动态拓扑 | **不覆盖** | 仍需运行期 `build_controller_hierarchy` |

**为什么第 4 行是硬边界**：本表示把"依赖"编码为**祖先关系**。
但状态边可以指向**非祖先**——例如 `parent_a` 与 `parent_b` 都消费 `shared` 的状态，
而 `shared` 只可能挂在其中一个下面。这类边在类型层没有对应物，
**因此编译期对它无话可说**。它正是 `FORMAL_MODEL.md` §8 威胁 1 所指的那类边，
也是运行期校验必须继续存在的原因。

> **论文口径**：这条强化是**对树/参考方向**的，不是对**任意 `G`** 的。
> 把它写成"编译期保证无环"而不加限定，是**过度声称**。

---

## 5. 编译期成本（必须报告）

`hierarchical_control` 是 **header-only**，模板实例化深度会直接影响下游编译时间，
因此这里给出实测（g++ 11.4.0，`-fsyntax-only`，取 3 次最优）：

| 翻译单元 | 时间 |
|---|---|
| 基线（仅 `#include <array>`，空 `main`） | **0.16 s** |
| 实例化 **64 节点**线性拓扑（含 `static_assert(node_count == 64)`） | **0.73 s** |

**增量 ≈ +0.57 s / 64 节点 ≈ 9 ms 每节点**；`-ftemplate-depth=64` 足够，未触及默认上限。

**限制**：只测了**线性链**这一种形状，未测宽扇出、多拓扑共处一个 TU、
或与其他模板库（`rclcpp`、`pluginlib`）叠加后的相互放大。
真实控制器 TU 的增量**可能显著高于**此值。**在测量多种形状之前，
不要把 9 ms/节点 当作通用常数。**

---

## 6. 与运行期校验的分工（诚实评估）

| 维度 | 编译期（本原型） | 运行期（`build_controller_hierarchy`） |
|---|---|---|
| 覆盖的拓扑来源 | 仅静态声明 | 任意（含 YAML） |
| 覆盖的边类型 | 仅树/参考（祖先关系） | 任意，含非祖先状态边 |
| 报错时机 | **编译** | 配置期（`create()` 抛异常） |
| 报错质量 | 模板实例化栈 + `static_assert` 文本 | 带节点名的运行期消息 |
| 运行期开销 | 0（计划已是类型信息） | 每次配置 O(\|V\|+\|E\|) |
| 编译期开销 | +9 ms/节点（实测，单一形状） | 0 |

**结论**：二者是**互补**关系。编译期方案的价值集中在
①静态链接/代码内构造的拓扑可以**根本进不了坏状态**；
②其后的运行期校验可专注于编译期覆盖不到的边（非祖先状态边）。

**未做**：把编译期计划**喂给**运行期执行组（即让 `StagedExecutionGroup` 消费
`static_topology` 的 `ancestry` 作为计划），从而省掉运行期构图。
这是本原型与现有内核的**接合点**，目前尚未连起来——**属于未完成项**。

---

## 7. 明确不能声称

- ❌ **"循环不可表达"**：实际是**编译期拒绝**（§2 的说明）。类型表达式可以写出 2-环。
- ❌ **"编译期保证任意 `G` 无环"**：只覆盖**树/参考方向**；非祖先状态边不在内（§4）。
- ❌ **"取代运行期校验"**：YAML/`pluginlib` 拓扑仍需运行期校验（§4、§6）。
- ❌ **"零成本"**：编译期成本实测 +0.57 s/64 节点，且只测了线性链（§5）。
- ❌ **"已与执行组集成"**：编译期计划尚未喂给 `StagedExecutionGroup`（§6 末尾）。

## 8. 可以声称

- ✅ 对静态声明的树/参考方向拓扑，含环层级**在编译期被拒绝**（命题 M1，含证明）；
- ✅ 自环与任意长度祖先复用都被覆盖，且有**带诊断**的最小反例语料（§3）；
- ✅ 参照边可在编译期从 `ancestry` 直接判定（`has_reference_edge`），无需运行期扫描；
- ✅ 编译期成本已量化（§5，附限制）；
- ✅ 覆盖边界已明确并写入测试（§4，`non_ancestor_state_edges_are_outside_the_static_guarantee`）。

---

## 9. 复现

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ctest --test-dir build/hierarchical_control -R test_static_topology --output-on-failure

# 编译语料单独运行（也可直接跑）
python3 hierarchical_control/test/test_static_topology_negative.py
```

---

## 10. 下一步（已完成 / 未完成）

**已完成（2026-09-22 追加）**：
- **拓扑契约接合**：见 `doc/TOPOLOGY_CONTRACT_JOIN.md`。
  `topology_contract.hpp` + `test_topology_contract.cpp`（6 用例）+ 2 个所有权编译负例。
  一份编译期定义**同时**驱动编译期检查与运行期 `SpecRows`（names/instances/parents），
  并强制**端口 owner 必须存在于拓扑中**。
  **边界**：只检查 owner 是否存在，**不检查 owner 是否真的导出**；
  **尚未**接到 `StagedExecutionGroup::create_library`。
- **维度/单位检查**：见 `doc/DIMENSIONAL_INTERFACES.md`。
  `dimensional_interfaces.hpp` + `test_dimensional_interfaces.cpp`（6 用例）
  + 2 个编译负例（由 `test_static_topology_negative.py` 核对），
  由同一个语料检查器验证，含必须编译通过的对照。
  **注意其边界**：它检查的是"**声明之间的量纲一致性**"，
  **不是**本项目实测的"声明了没人导出的接口"（后者仍需运行期），也**不检查单位（米/毫米）**。

**已完成（同日再追加）：接合到内核**
`topology_binding.hpp` + `test_topology_binding.cpp`（5 用例）。
`create_library_group(binding)` 直接返回**可运行的** `StagedExecutionGroup`：
一份编译期定义 → 检查 → 计划 → 适配 → 真实执行组，**全链路打通**。
测试刻意真实构造执行组并跑一个周期，以覆盖"适配后的 Spec 被内核拒绝"这一失效模式。

**已完成（同日再追加）：量纲接入端口声明**
`typed_ports.hpp` + `test_typed_ports.cpp`（5 用例）+ `doc/PORT_DIMENSIONS.md`（详见该文）。
端口**只声明一次**（类型），内核所需字符串与 `Contract` 均由它生成 ⇒ 不会漂移；
父子端口的**名字 + 顺序 + 量纲**可编译期检查（量纲检查是内核做不到的）；
mixin 控制器**真的能在执行组里运行**（端到端 `committed`）。

**未完成**：
1. **状态端口的语义区分**未下探到类型层——`staged_state_ports()` 同时承载
   "发布给父的状态"与"从子消费的状态"，内核按拓扑而非按名字区分；
   在类型层建模会与内核规则重复（见 `PORT_DIMENSIONS.md` §3 末行）。
2. **深链成本的优化**：`ancestry` 目前按值复制父数组，
   可改为"继承 + 沿父链查询"把每节点成本从随深度增长降为常数。**未做**
   （当前深度下不值得；见 `doc/COMPILE_COST.md` §3）。
3. **诊断可读性（已评估，主动降级）**：试过用"未定义模板特化"技巧把节点名带进诊断，
   结果只显示出类型名（`a_n`）而非可读字符串（`"chassis"`），需 C++20 才能在编译期
   拼出可读字符串。**结论：收益有限、机制脆弱，不做**——现有诊断已含
   清晰的文字说明与 file:line，足以定位。
