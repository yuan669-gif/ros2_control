# 维度/单位检查：把物理量纲放进接口类型（原型）

日期：2026-09-22
状态：**原型已完成并测试**
代码：`hierarchical_control/include/hierarchical_control/dimensional_interfaces.hpp`
测试：`hierarchical_control/test/test_dimensional_interfaces.cpp`（6 用例）+
`hierarchical_control/test/test_static_topology_negative.cpp` 语料中的 2 个维度负例
上位动机：`HANDOFF_MANUAL.md` P2.5 第 2 项；`doc/METAPROGRAMMING_CONTRACT.md` §10

---

## 0. 为什么做这个（不是"再写一个单位库"）

`ros2_control` 用**字符串名**标识接口。父控制器声明消费 `<child>/state`，
子控制器声明导出 `<child>/state`，**类型系统从不比较这两个声明**。

本项目**实测过**这个缺口的后果：`controller_manager/test/test_upstream_ordering.cpp`
声明了一条**没有任何控制器导出**的状态边，`configure_controller` **照单全收、无声通过**
（`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` §9）。声明与实现不一致，
**在运行期完全不可见**，直到它悄悄改变了调度行为。

**量纲错接是同一类缺陷**：把轮子的**角度**状态当成**线位移**消费、
把**位置**命令接到**速度**参考上——字符串名世界一概接受，
而控制上会表现为难以归因的振荡或稳态偏差。

本原型把物理量纲放进**类型**，使这类错接成为**编译错误**。

---

## 1. 量纲代数

量纲是四个 SI 基量的指数向量：

```cpp
template <typename Length, typename Mass, typename Time, typename Angle>
struct Dimension { ... };
```

指数用 `std::ratio`，所以整数幂与有理幂都能表达；运行期只暴露 `length_exp` 等 `constexpr int`。

| 运算 | 语义 | 别名 |
|---|---|---|
| `multiply_t<D1,D2>` | 指数相加 | —— |
| `divide_t<D1,D2>` | 指数相减 | —— |
| `power_t<D,n>` | 指数乘 `n` | —— |
| `same_dimension_v<D1,D2>` | 四个指数全等 | —— |

**关键：代数必须真的归约**，否则就只是"扁平标签"。已断言：

```text
Position / Time            == LinearVelocity
LinearVelocity / Time      == LinearAcceleration
LinearAcceleration / Time  == LinearJerk
Angle / Time               == AngularVelocity
AngularVelocity / Time     == AngularAcceleration
Mass * LinearAcceleration  == Force
LinearVelocity * Time      == Position          （积分方向也成立）
Torque * Angle             == Energy
Energy / Time              == Power
```

### 1.1 两个"必须不同"的量（否则检查形同虚设）

| 量对 | 为什么必须区分 |
|---|---|
| `Torque` vs `Energy` | 做功是 `F·d·θ` ⇒ **力矩是能量的每弧度**。若不带角度指数，二者量纲相同，**力矩命令就能接到能量参考上**。本原型把角度指数放进力矩（`[N·m/rad]`），二者区分开，同时断言 `Torque * Angle == Energy` 说明它们的关系仍被正确刻画。 |
| `Angle` vs `Position` | 轮半径混淆的经典来源：`[rad]` 与 `[m]` 必须不同维度。 |
| `Force` vs `Torque` | 平动与转动的驱动力必须区分。 |

---

## 2. 接口与连接规则

```cpp
enum class Role { command, reference, state };

template <Role R, typename Dim>
struct Interface { static constexpr Role role = R; using dimension = Dim; ... };
```

两条规则：

1. **参考边**：父写的命令接口与子导出的参考接口**量纲必须相同**；
2. **状态边**：子导出的状态接口与父消费的状态接口**量纲必须相同**。

对应的**判定谓词**（可用于分支）：

```cpp
reference_edge_is_legal_v<Command, Reference>
state_edge_is_legal_v<ExportedState, ConsumedState>
```

以及**硬要求**（在使用点触发 `static_assert`）：

```cpp
require_reference_edge<Command, Reference>();
require_state_edge<ExportedState, ConsumedState>();
```

诊断文本：

```text
dimensional_interfaces: DIMENSION MISMATCH on a reference edge -- the command a parent writes and
the reference a child exports must have the same physical dimension
```

`Role` 也参与校验：**状态接口不能当参考接口用**，反之亦然。

---

## 3. 已验证的行为

### 3.1 正例（`test_dimensional_interfaces.cpp`，6 用例全通过）

- 代数归约（§1 的 9 条等式）；
- 必须区分的量对（力矩/能量、角度/位置、角速度/线速度、位置/速度…）；
- 参考边与状态边的合法/非法判定，**包括无假阳性**（合法接线一律接受）；
- `require_*` 在接受路径上可用；
- **POV 底盘完整示例**：底盘写轮速参考（`[m/s]`）、消费轮行程状态（`[m]`）——
  正确接线被接受，而"把轮子的**角度**状态当**线位移**消费"被拒绝。

### 3.2 负例（编译语料，2 个文件）

| 文件 | 期望 | 实际 |
|---|---|---|
| `compile_fail_dimension_reference.cpp` | 失败 | ✅ 失败（诊断为 DIMENSION MISMATCH） |
| `compile_fail_dimension_state.cpp` | 失败 | ✅ 失败（诊断为 DIMENSION MISMATCH） |

由 `test_static_topology_negative.py` 编译并核对，**同时核对失败来自预期的那条 `static_assert`**，
并保留 `must_compile_control.cpp` 作为**必须编译通过**的对照，使整套检查**可证伪**。

---

## 4. 明确覆盖不到的（写清楚，避免过度声称）

| 情形 | 是否覆盖 | 说明 |
|---|---|---|
| 静态声明的接口量纲错接 | ✅ 编译期拒绝 | 本文 |
| YAML / `pluginlib` 运行期声明的接口 | ❌ **不覆盖** | 仍需字符串名 + 运行期校验 |
| **单位（比例）**：米 vs 毫米 | ❌ **不覆盖** | 量纲相同；需要尺度因子，**故意不做** |
| **输出缩放**：轮半径把 `[rad]` 与 `[m]` 关联 | ❌ **不覆盖** | 这是语义/参数问题，不是量纲问题 |
| 坐标系约定、符号方向 | ❌ **不覆盖** | 同上 |
| 未导出 vs 已声明（本项目实测的那个缺陷） | ⚠ **部分** | 本原型保证"声明的量纲一致"，但**不检查名字是否真的被导出**；后者仍需运行期 |

> **最大的一条限制**：上表最后一行。本项目实测的缺陷是"**声明了没人导出的接口**"，
> 而本原型解决的是"**声明了量纲不匹配的接口**"。二者相关但**不同**：
> 前者需要知道"谁导出了什么"，这在纯类型层拿不到（子控制器的导出是运行期行为）。
> **因此这不是那个缺陷的完整解**，只能说把"声明之间的不一致"这一类收进了类型层。

---

## 5. 与既有代码的关系

- **纯新增**：本头文件不依赖也不修改 `staged_execution_group.hpp` / `hierarchy.hpp`，
  不影响现有运行期路径；现有 31+6 个测试全部保持通过（见 §6）。
- **未接合**：尚未把量纲接到 `StagedControllerInterface` 的端口声明，
  也未与 `static_topology.hpp` 的类型链组合（例如"沿 `ancestry` 传递量纲"）。
  这是下一步，**目前不能声称"执行组已做量纲检查"**。

---

## 6. 复现

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ctest --test-dir build/hierarchical_control --output-on-failure      # 7/7

python3 hierarchical_control/test/test_static_topology_negative.py    # 编译语料
```

---

## 7. 不能声称 / 可以声称

**不能声称**：

- ❌ "解决了上游静默接受未导出接口的缺陷"（只解决了量纲不一致，见 §4 末行）；
- ❌ "覆盖 YAML/动态声明的接口"；
- ❌ "检查了单位（米/毫米）"——只检查量纲，**单位相同量纲不同则不检查**；
- ❌ "已接入执行组"（§5 未接合）。

**可以声称**：

- ✅ 把物理量纲放进接口类型后，参考边/状态边的**量纲错接在编译期被拒绝**（含负例语料）；
- ✅ 量纲是**真代数**（归约、积分、力矩/能量关系均被断言），
  而非扁平枚举——因此能区分"力矩 vs 能量""角度 vs 位置"这类**物理上合理但错误**的接线；
- ✅ 负例检查**可证伪**（含必须编译通过的对照，且核对诊断来源）；
- ✅ 相对上游的**新增能力**：上游接口是字符串名，**没有**这层保护；
- ✅ 纯新增、不影响现有路径（现有测试全绿）。
