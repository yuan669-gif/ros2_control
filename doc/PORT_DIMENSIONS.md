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
  PortList<tire_target, wheel_travel>,   // 向父导出的 reference 端口
  PortList<wheel_target>,                // 从父消费的 reference 端口
  PortList<wheel_torque>>;               // 写入的执行器端口

class WheelController : public TypedPortsMixin<WheelController, wheel_ports> { ... };
```

由这一处声明**生成**：

- 内核需要的字符串列表（`staged_reference_ports()` / `staged_actuator_ports()` /
  `staged_state_ports()`）；
- `topology_contract` 检查所用的 `Contract`（`contract_of_t<wheel_ports>`）。

**字符串是从类型生成的，所以不可能与量纲不一致。**

---

## 1. 设计要点

### 1.1 三组端口与内核语义的对应

| 声明中的位置 | 含义 | 映射到内核的 |
|---|---|---|
| `Exported` | 本控制器**向父导出**的 reference 端口 | `staged_reference_ports()` **不**含它；它出现在 `staged_state_ports()` |
| `Consumed` | 本控制器**从父消费**的 reference 端口 | `staged_reference_ports()` |
| `Actuators` | 写入的硬件命令端口 | `staged_actuator_ports()` |

`staged_state_ports()` 生成为每个导出/消费端口的 `"<name>/state"`。

**为什么这样映射**：内核靠**名字**区分"拓扑 reference 边"与"内部 state 边"
（`create_library` 的检查会分别报
`root node ... declares reference ports but no reference source` 与
`... declares actuator ports but no commit sink`）。测试实跑时先后触发了这两条，
说明映射与内核期望一致。

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
| 控制器自报端口与声明不一致（绕过 mixin） | ⚠ 由 `verify_ports_match_interface` **运行期**发现，非编译期（构造控制器不是常量表达式） |
| 单位（米/毫米） | ❌ 量纲相同，不检查 |
| 输出缩放、坐标系、符号 | ❌ 语义问题 |
| YAML / `pluginlib` 动态拓扑 | ❌ 不覆盖 |
| **状态端口的量纲** | ❌ **未建模**——`staged_state_ports()` 同时承载"发布给父的状态"与"从子消费的状态"，内核按拓扑而非按名字区分；在类型层建模会与内核规则重复。`Contract` 里仍带状态端口供**所有权与量纲**检查用 |

**最后一行需要明说**：本原型把**reference / actuator** 端口打通到类型层，
**状态端口的语义区分没有**下探到类型层。

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
