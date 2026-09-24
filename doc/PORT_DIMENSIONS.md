# 量纲接入执行组的端口声明（原型）

日期：2026-09-22
状态：**原型已完成并测试**
代码：`hierarchical_control/include/hierarchical_control/typed_ports.hpp`
测试：`hierarchical_control/test/test_typed_ports.cpp`（5 用例，全部通过）
上位动机：`HANDOFF_MANUAL.md` P2.5 最后一项；`DIMENSIONAL_INTERFACES.md` §5

---

## 0. 补上了什么

此前的缺口很具体：

| 层 | 端口的表达方式 |
|---|---|
| `dimensional_interfaces.hpp` / `topology_contract.hpp` | **类型**（`Port<Name, Dim>`） |
| `StagedControllerInterface`（内核） | **运行期字符串**（`std::vector<std::string>`） |

于是控制器必须**把同一件事说两遍**：一遍用类型（给检查用），一遍用字符串（给内核用）。
**两遍可以不一致，而两层都不会发现。** 这正是本项目反复出现的那类缺陷
（"同一事实的两处声明可以漂移"）。

本原型把**声明点收敛为一处**：

```cpp
using wheel_ports = TypedPorts<
  PortList<wheel_travel>,    // 本节点自己的状态端口（父的状态阶段读它）
  PortList<wheel_target>,    // 本节点接收的 reference 端口（父的命令阶段写它）
  PortList<wheel_torque>,    // 写入的执行器端口
  PortList<tire_target>>;    // （可选）本节点写进子节点的 reference 端口，仅用于父子静态检查

class WheelController : public TypedPortsMixin<WheelController, wheel_ports> { ... };
```

由这一处声明**生成**：

- 内核需要的字符串列表（`staged_reference_ports()` / `staged_actuator_ports()` /
  `staged_state_ports()`）；
- `topology_contract` 检查所用的 `Contract`（`contract_of_t<wheel_ports>`）。

**字符串是从类型生成的，所以不可能与量纲不一致。**

---

## 1. 设计要点

### 1.1 四组端口与内核语义的对应

| 声明中的位置 | 含义 | 映射到内核的 |
|---|---|---|
| `State` | 本节点**发布**的状态端口（父的状态阶段直接读本节点的槽） | `staged_state_ports()`，**名字原样、不加后缀** |
| `Reference` | 本节点**接收**的 reference（父的命令阶段写它） | `staged_reference_ports()` |
| `Actuators` | 写入的硬件命令端口 | `staged_actuator_ports()` |
| `ForChildren`（可选） | 本节点**写进子节点**的 reference | 不进内核；只给父子静态检查用 |

**内核到底用哪些信息**：`StagedExecutionGroup` 只用这三张表的 **长度** 来分配缓冲
（`state_values_[i].assign(...size(), 0.0)` 等），**名字完全不参与**运行期逻辑——
名字是给控制器自己的阶段做文档的。所以"长度正确"是硬性要求，"名字正确"由一个显式的
自检函数（`verify_ports_match_interface`）负责。

**2026-09-24 修正（这是一个真实缺陷，值得记录）**：更早的版本只有三组端口，并把
`staged_state_ports()` 定义为"`Exported` 与 `Consumed` 各加 `/state` 后缀后拼接"。
这把三件不同的事混在了一起并**多算了槽位**：一个只有 1 个自身状态、1 个接收 reference 的
节点会拿到 **3 个** 状态槽，若它还往子节点写 reference 就更多。由于内核会给状态槽预填 NaN
并要求"声明的端口都必须写"（评审 R3 的完整性检查），一个只写自己真实状态的控制器会**每个
周期都以 `state_failed` 失败**；它的父节点拿到的"子状态视图"里还混着根本不是状态的槽位。

**同一处修正**：`declarations_are_compatible<Parent, Child>` 原来比较"父的整个 `Exported`
列表"与"子的 `Consumed` 列表"，于是会把一对**正确**的 wheel/tire 判为不兼容——一个假阳性，
而 `test_typed_ports` 当时把这个假阳性写成了期望行为。现在它比较"父声明写进子节点的
reference"与"子声明接收的 reference"（`ForChildren` vs `Reference`），量纲检查仍然生效。

### 1.2 为什么用 mixin

`TypedPortsMixin<ControllerT, Ports>` 继承 `ControllerT`（CRTP），
把三个端口访问器实现为 `final`。它**必须是最派生类**，否则 `final` 覆盖不生效。
具体控制器仍需自行实现 `update_state_stage` / `update_command_stage`。

### 1.3 量纲比较（本次真正的检查）

```cpp
template <typename ParentPorts, typename ChildPorts>
constexpr bool declarations_are_compatible() noexcept;
```

逐个比较父的 `Exported` 与子的 `Consumed`：**名字相同 且 量纲相同**。

- 内核只比字符串，所以它会接受"位置端口接到速度端口"；
- 本检查用 `same_port_v<A, B>`（`topology_contract.hpp`，比较名字 + `same_dimension_v`）拒绝它。

---

## 2. 已验证的行为（`test_typed_ports.cpp`，5 用例）

1. **生成的字符串与声明一致**（含顺序）：`wheel/target`、`wheel/torque`、
   以及 `tire/target/state, wheel/travel/state, wheel/target/state`；
2. **运行期交叉校验**：`verify_ports_match_interface` 对 mixin 控制器返回真，
   失败时给出原因字符串；
3. **派生的 `Contract` 与声明一致**（`produced_count` / `consumed_count`）；
4. **父子声明检查**：
   - `wheel` 导出 `{tire/target, wheel/travel}` 而 `tire` 消费 `{tire/target}` ⇒ **不兼容**（正确拒绝）；
   - 完全一致的父子对 ⇒ 通过；
   - **同名但量纲不同**（`LinearVelocity` vs `Position`）⇒ **拒绝**——这是字符串名世界做不到的检查；
5. **端到端**：用 mixin 构造的控制器**真的跑进执行组**，
   `run_ns` 返回 `StagedStatus::committed`，两阶段各调用一次。

---

## 3. 边界（写清楚）

| 情形 | 覆盖 |
|---|---|
| 静态声明的端口、经 mixin 生成字符串 | ✅ |
| 父子端口的**名字 + 顺序 + 量纲**一致性 | ✅ |
| 控制器自报端口与声明不一致（绕过 mixin） | ✅ `verify_ports_match_interface`（三张表全查，含状态端口）、`verify_ports_match_contract`（与绑定的 `Contract` 对齐）、`topology_binding::verify_binding_ports`（沿类型链整棵树一次查完）。都是**运行期**调用，不是编译期（构造控制器不是常量表达式） |
| **执行器端口**与 `Contract` 的一致性 | ❌ **查不了**：`Contract` 有意不含硬件执行器端口，没有可比对象。用 `verify_ports_match_interface` 才能查它 |
| 单位（米/毫米） | ❌ 量纲相同，不检查 |
| 输出缩放、坐标系、符号 | ❌ 语义问题 |
| YAML / `pluginlib` 动态拓扑 | ❌ 不覆盖 |
| **状态端口的量纲** | ⚠ 部分建模：`State` 现在是**独立的一组端口**（不再与 reference 混在一起），所以它的名字与量纲进入了 `Contract::produced` 并参与所有权检查；但"父读子状态"这条边本身**没有**静态成对检查——父拿到的子状态视图来自内核按 `parents` 建的拓扑，父并不声明它要读子节点的哪些状态。要静态检查这条边需要父声明子状态端口，目前**没做** |

**需要明说**：现在 `State` / `Reference` / `Actuators` 三组都打通到了类型层，
**父子之间的状态边**（父读子状态）还没有静态成对检查，`ForChildren` 只覆盖 reference 方向。

---

## 4. 本轮踩到的坑（值得记录）

1. **`port_key` 其实不存在**——我在重写 `topology_contract.hpp` 时把它换成了别的机制，
   而新写的 `typed_ports.hpp` 却引用了它。**编译报错才发现自己对自家 API 的记忆是错的。**
   已补上真正需要的 `same_port_v`（名字 + 量纲）。
2. **函数模板不能偏特化**：`port_lists_agree` 起初写成函数模板并偏特化，C++ 不允许，
   改成类模板。
3. **`static_assert(f<T, U>())` 漏写括号**会被解析成函数地址，于是断言**恒假**，
   并伴随 `-Waddress` 警告（"will never be NULL"）。这个警告是唯一线索——
   若忽略它就会去错误的方向排查。
4. **C++17 下 `constexpr` 不能分配**：最初想用 `std::vector<std::string>` 做编译期比较，
   不可行；改为 `constexpr std::array<std::string_view, N>`，
   顺带**避免了堆分配**（与项目 `run()` 零分配的目标一致）。
5. **运行期依赖是真实的、不是仪式**：构造执行组时先后被
   "缺 commit sink"、"缺 reference source" 拒绝，补上后才 `committed`。
   这说明端口声明与内核期望确实接上了。

---

## 5. 复现

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ctest --test-dir build/hierarchical_control --output-on-failure    # 10/10
```

---

## 6. 不能声称 / 可以声称

**不能声称**：

- ❌ "状态端口的量纲已检查"（§3 最后一行；未建模）；
- ❌ "绕过 mixin 的控制器会在编译期被发现"（是运行期校验）；
- ❌ "检查了单位/缩放/坐标系"；
- ❌ "覆盖 YAML/动态拓扑"。

**可以声称**：

- ✅ 端口**只声明一次**（类型），字符串与 `Contract` 均由它**生成**，因此不会漂移；
- ✅ 父子端口的**名字、顺序、量纲**一致性可**编译期**检查，
  其中量纲检查是**内核（只比字符串）做不到的**；
- ✅ mixin 控制器**真的能在执行组里运行**（端到端，`committed`）；
- ✅ 纯新增，不影响既有路径（库 10/10、管理器 6/6、standalone 1/1）。
