# 源码导读：从零读这份工作

> 配套：`ONBOARDING.md`（先读那个）。**只想把功能用起来、不想读源码 → `USER_GUIDE.md`。**
> 本文假设你熟悉 C++17 和 ROS 2 的基本用法，
> **不**假设你熟悉 ros2_control 内部，也**不**假设你看过 FineMote。
>
> 读法建议：§1–§3 通读（约 20 分钟），然后从 §4 挑一层深入，边读边打开对应头文件。
> 每小节都给了"这个文件存在的理由"和"验证它的测试"。

---

## 1. 前置：ros2_control Humble 的五个概念

读本项目的代码只需要这五个上游概念：

| 概念 | 一句话 | 关键类型 |
|---|---|---|
| **ControllerInterface** | 控制器插件基类，生命周期 `init → configure → activate → deactivate → cleanup` | `controller_interface::ControllerInterface` |
| **接口（interface）** | 硬件暴露的 `joint/interface` 名字，分 **command**（写）和 **state**（读）两个命名空间 | `hardware_interface::LoanedCommandInterface` / `LoanedStateInterface` |
| **ResourceManager** | 接口的拥有者：`claim_*_interface()` 借出、`command_interface_is_claimed()` 判占用 | `hardware_interface::ResourceManager` |
| **ControllerManager 循环** | 外部节点按周期调用 `cm->read()` → `cm->update()` → `cm->write()`；`update()` 里按控制器列表逐个调 `controller->update()` | `ControllerManager::update()` |
| **chaining（级联）** | 控制器可以导出 **reference interface**（名前缀是自己的控制器名），前面的控制器写它、后面的控制器当 command 读；`set_chained_mode()` 只允许控制器**非 ACTIVE** 时调用 | `ChainableControllerInterface` |

两个关键限制，后面会反复出现：

1. **上游 `update()` 一次遍历**：一个控制器每周期只被调用一次，所以父子的同周期双向需求无法同时满足；
2. **`set_chained_mode()` 不能在 ACTIVE 时调用**：这就是"为了切 chained mode 会把已激活控制器先停后启"的根源
   （也是最新一轮修复的那个坑，见 `DEVELOPMENT_HISTORY.md` §6）。

---

## 2. 一个周期里发生了什么（trace）

以管理器模式为例（`ControllerManager::update()`，`controller_manager/src/controller_manager.cpp`）：

```
read()                                  ← 硬件 read（上游）
  │
update():
  1. CycleGuard                        ← 标记"周期在飞"，用于拒绝运行期安装执行路径
  2. current_generation()              ← 每周期【一次】原子读，取出 模式+成员表+执行组 的不可变快照
  3. staged group?  → group->run()      ← 阶段化执行：状态趟（后序）+ 命令趟（先序）+ 整组提交
  4. two_phase_enabled? → 反向遍历      ← update_phase：叶 → 根（子先于父）
  5. 原生循环：对每个非两趟成员调 update()  ← 上游行为，未被两种新路径拥有的控制器照旧
  6. two_phase_enabled? → 正向遍历      ← handle_phase：根 → 叶（父先于子）
  7. do_switch? → manage_switch()       ← 应用一次切换（可含 atomic 回滚）
write()                                 ← 硬件 write（上游）
```

要点：

- **第 3 步与第 4–6 步互斥**：执行组成员被原生循环跳过，也不会进两趟成员表（准入时保证）；
- **第 4 步和第 6 步走同一份顺序**（`rt_controller_list`），只是方向相反——这就是"一份线性化满足两向"；
- **切换期间（第 7 步待处理时）两趟与执行组都暂停**，因为成员集合可能正在变；
- 第 2 步的 generation 是**一个** `shared_ptr`，所以一个周期不会出现"新模式 + 旧成员表"。

读代码时对着这七步看，就不会迷路。

---

## 3. 哪一行是本项目改的

```bash
git log --oneline                     # 全部是本项目的工作（根提交 b1bf616 即"上游 Humble 源码 + 第一版内核"）
git show --stat <commit>              # 单个提交改了什么、为什么（message 里带实测数字）
# 本仓库没有上游历史，`git diff 469f3055..HEAD` 会报 bad revision（469f3055 只是上游提交号）
git log --oneline                     # 28 个提交，每个提交的 message 都写了"为什么"
```

改动集中在三处：

| 位置 | 性质 |
|---|---|
| `hierarchical_control/` | **新包**：内核 + 类型层 + 描述层，不依赖 ROS 运行期（可单独编译成测试） |
| `controller_manager/` | **接入**：`update()`/`switch_controller()`/加载路径 + 三个新 API（generation、atomic activation、registry） |
| `case_study/`、`research/` | 案例与形式化小工具，不是产品代码 |

`controller_manager/include/controller_manager/{staged_execution_group,hierarchy}.hpp` 是**转发头**
（`using` 到 `hierarchical_control` 里的真实现），保留历史 include 路径。

---

## 4. 四层阅读顺序

### L0 内核层（先读这个，因为它定义"数据长什么样"）

**4.1 `hierarchical_control/include/hierarchical_control/staged_controller_interface.hpp`**

内核与控制器之间的契约。重点看四个东西：

- `StagedFrame{cycle, sample_ns, valid, fault_code}`：一帧数据的**溯源**。
  `cycle`/`sample_ns`/`valid` **由执行组写**，控制器只能报 `fault_code`；
  复合节点的 `sample_ns` 会被执行组覆盖为**最旧**的子样本时间——所以"把派生数据重新盖章成更新的"
  在类型/语义上就不成立。这是"新鲜度"这套说法的落点。
- `StagedContext{cycle, now_ns, period_ns}`：一个周期内所有节点共享的整数上下文（不传浮点时间）。
- `StagedInputView` / `StagedValueView` / `StagedValueWriter` / `StagedReferenceWriter`：
  阶段的**只读/只写视图**，控制器拿不到别人的 command handle。
- `StagedCommandSink` / `StagedReferenceSource`：叶节点与硬件之间的唯一出口/入口。

**4.2 `hierarchical_control/include/hierarchical_control/staged_execution_group.hpp`**（内核主体）

按这个顺序读：

1. `enum class StagedStatus`：`committed / inactive / invalid_input / state_failed / command_failed / exhausted / not_configured`。
   **每个状态都对应一个可测的失败面**，没有"其他错误"。
2. `StagedGroupMember`：成员是"名字 + 控制器指针 + 它 claim 的 command interfaces"。
   依赖边**不是配置项**：谁来填 `member.command_interfaces` 决定了边的来源——管理器模式填
   `controller->command_interface_configuration().names`（`set_staged_execution_group()`），
   库模式填端口声明生成的列表（`create_library_group()`）；内核把"名字前缀是另一个成员"的端口解释成
   那条成员之间的 reference 边。
3. `create(members, max_age_ns)`：配置期做拓扑解析、字符串处理、`dynamic_cast`、存储分配——**都在非实时路径**。
   这里能学到三个设计取舍：同实例两名字被拒、自环被拒、一个端口只能有一个写者。
4. `run_ns(now_ns, period_ns)`：实时路径。**先检查 `members_active_` 缓存**（不查 lifecycle，因为 Humble 的
   `get_current_state()` 会分配），然后状态趟（后序）→ 命令趟（先序）→ **整组提交**。
   `run()` 在配置完成后**不分配**（有断言）。
5. `refresh_member_active_state()`：唯一会查 lifecycle 的地方，由管理器在**切换之后**调用
   （`update()` 里 `manage_switch()` 之后那一小段）。

> 想验证这一层：`hierarchical_control/test/test_execution_group.cpp`（11 例）、
> `test_stale_state_cost.cpp`（陈旧/新鲜度，7 例）、
> `controller_manager/test/test_staged_execution_group.cpp`（管理器里的端到端，7 例）。

**4.3 `hierarchical_control/include/hierarchical_control/two_phase_controller_interface.hpp`**

**更轻**的一条路径：只有两趟 pass，没有组、没有端口声明、没有帧、没有整组提交。
它存在的意义是"把调度单独拿出来验证"——如果连这个都做不到同周期，那执行组的复杂度就无从谈起。
实现它的控制器由 `update_phase()`/`handle_phase()` 驱动，并且**不会被原生循环再调一次**。

> 验证：`controller_manager/test/test_two_phase_execution.cpp`（27 例，含"单趟滞后 = 深度"、
> "两趟两向都新鲜"、跨模式边拒绝、列表顺序导致边反向时拒绝等）。

**4.4 `controller_manager/include/controller_manager/cycle_tree.hpp`（库模式内核）**

同一思想的最早实现：同步标量端口、类型化 `Node`/`Context`/`Value`，完全不依赖 ROS 与 hardware handle。
它现在是"第二种宿主"（单个 composite 插件内部）的参考实现，也是形式化结论最早的可运行验证。

> 验证：`controller_manager/test/test_cycle_tree_standalone.cpp`（ctest 名 `test_cycle_tree_contract`，
> 非 gtest：失败即抛异常/非零退出）。

**4.5 两种宿主的关系**（重要，容易绕晕）

```
                      ┌── 库模式：一个普通 controller 插件内部跑整棵树  （test_composite_library/）
同一个 StagedExecutionGroup ─┤
                      └── 管理器模式：成员是管理器里的控制器，管理器执行组（controller_manager/）
```

两种宿主共用内核与契约；`doc/WIRING_COST_ANALYSIS.md`、`doc/HIERARCHY_FAIR_COMPARISON.md` 比较了它们的
接线成本、耗时与分配，结论是**通用 composite 宿主更快、接线更少**，管理器接入的独立价值在
"逐控制器生命周期、局部激活、原生接口参与、工具可见性、故障隔离"。

### L1 类型/元编程层（"让错误编译不过"）

按依赖顺序读，每层只解决一个问题：

**4.6 `static_topology.hpp` —— 树是类型，环写不出来**

```cpp
struct chassis_name { static constexpr auto value = NameOf("chassis"); };
using chassis = Root<chassis_name>;
using wheel   = Descendant<wheel_name, chassis>;   // 若 wheel_name 已出现在父的 ancestry 里 → 编译错误
```

`Descendant<Name, Parent>` 要求 `Name` 不在 `Parent` 的 ancestry 里。因为节点自己的名字也在自己的 ancestry 里，
这一条同时拒绝了**自环**和**任意长度的环**（环必然重访路径上的节点）。这把 `FORMAL_MODEL.md` 的推论 2
从"配置期拒绝"强化成"**写不出来**"。

**4.7 `dimensional_interfaces.hpp` —— 接口有量纲**

`Position` / `LinearVelocity` / `Torque` … 是类型，不是字符串。父子边量纲不匹配 → 编译失败。

**4.8 `topology_contract.hpp` —— 一个 binding 同时驱动检查与运行**

- `Port<Name, Dimension>`、`PortList<...>`：端口的**类型**表示；
- `Contract<produced, consumed>`：一个节点"产出什么/消费什么"的静态摘要；
- `BoundNode<ControllerT, Node, ContractT, Children...>`：**变参**子节点（`std::tuple`），所以分叉不用改类型；
- `compose<RootNode, RootContract>(root_ptr, children...)`：把实例与类型绑在一起，**编译期**检查
  每个孩子的声明与父为该孩子声明的边一致（reference 边、state 边分别检查，可分别断言）；
- 运行期部分：`build_spec_rows()` 生成内核要的 `Spec` 行；`rows_are_well_formed()` 检查唯一名、单根、
  父存在、非自父、**实例唯一**（同一个对象挂两个名字会被拒）。
- `compose` 还拒绝"孩子声明的父不是我"（`child_declares_this_parent`）。

> 负向证据在 `hierarchical_control/test/static_topology_negative/`：15 个源文件，**必须编译失败**的
> 12 个各要求诊断里出现特定子串（`static_topology: CYCLE`、`DIMENSION MISMATCH`、`OWNERSHIP VIOLATION`、
> `TOPOLOGY MISMATCH`、`REFERENCE EDGE MISMATCH`、`STATE EDGE MISMATCH`），另有 2 个必须编译通过。
> 驱动脚本 `test_static_topology_negative.py` 用与构建相同的编译器（`CXX` 由 CMake 传入）。

**4.9 `typed_ports.hpp` —— 端口只声明一次（本项目最实用的一层）**

问题：内核要的是**字符串**（`staged_state_ports()` …），而检查要的是**类型**（`Contract`）。
两边各写一遍就会不一致，而且没人发现。

解决：控制器只写一份

```cpp
using ports = tp::TypedPorts<
  tc::PortList<wheel_travel>,      // State      ：自己发布的状态
  tc::PortList<wheel_target>,      // Reference  ：自己接收的参考
  tc::PortList<wheel_torque>,      // Actuators  ：写进的硬件 command
  tc::PortList<tire_target>,       // ForChildren：写进子节点的参考（仅静态检查）
  tc::PortList<tire_travel>,       // ChildState ：从子节点读的状态（仅静态检查）
  tc::PortList<tire_position>>;    // HardwareState：读的硬件 state（不是拓扑边）
class WheelController : public tp::TypedPortsMixin<WheelController, ports> { ... };
```

`TypedPortsMixin` 由此**生成** `staged_*_ports()` 字符串表与 `contract_of_t<ports>`；
`HardwareState` 列表还能**生成** `command_interface_configuration()`/`state_interface_configuration()`
（见 `typed_fork_composite_controller`），于是"控制器声明"与"运行期声明"不可能互相矛盾。

> 验证：`test_typed_ports.cpp`（8 例）、`test_typed_tree.cpp`（3 例，分叉树）、`test_contract_regression.cpp`（12 例）。

**4.10 `topology_binding.hpp` —— 让静态描述真的跑起来**

- `create_library_group(binding)`：从 binding 直接建组，并**自动**对每个节点做运行期端口↔`Contract` 校验
  （`verify_binding_ports`，失败时打印"实际 vs 声明"）；
- `create_library_group_unchecked`：跳过校验（只有明确知道在做什么时才用）；
- **指针安全**：binding 存 `ControllerInterfaceBase*` + 一个做 `dynamic_cast<StagedControllerInterface*>`
  的访问器。早期版本用 `void*` 往返，当控制器同时继承两个基类时地址调整会丢——这是**实测出来的真 bug**，
  不是理论担忧。

**4.11 `static_manifest.hpp` —— 编译期描述对象**

```cpp
using binding_type = decltype(tc::compose<root_node, tp::contract_of_t<root_ports>>(&root, a_leaf, b_leaf));
static constexpr auto manifest = sm::manifest_of_v<binding_type>;      // 全是 static constexpr std::array
static_assert(sm::manifest_problem(manifest).empty());                 // 可编译期查
static_assert(manifest.node_count == 3);
```

`manifest_problem()` 检查：至少一个节点、名字唯一、恰好一个根、父存在、非自父、**父链不含环**
（纯环被"恰好一个根"拦住；"链进入环"由有界父链上溯拦住）、非硬件端口属于某个节点、每角色端口唯一。
`declaration_matches_manifest()` 在 `configure()` 里把 manifest 与控制器**手写**的接口列表按**名字**比对
（顺序无关），不一致就报"要求了但没有声明 / 声明了但没要求"。

### L2 管理器接入层

**4.12 `controller_manager/src/controller_manager.cpp` 的入口清单**

| 你要找的东西 | 函数 |
|---|---|
| 一个周期怎么跑 | `update()`（§2 的七步） |
| 切换怎么做 | `switch_controller()`（非实时，构造请求）→ `manage_switch()`（实时，应用） |
| 激活与回滚 | `activate_controllers()` → `activate_controllers_for(names)`；`rollback_activated_controllers()` |
| 加载（pluginlib 与编译内置） | `load_controller()`（先查 registry 再 pluginlib）→ `add_controller_impl()` |
| 配置期检查 | `configure_controller()`（含 manifest 一致性、两趟准入、执行组准入） |
| 执行状态发布 | `current_generation()` / `publish_generation()` / `execution_generation()` |
| 运行期配置约束 | `control_loop_busy()`、`set_two_phase_execution()`、`set_staged_execution_group()` |

**4.13 generation：为什么是"一个 shared_ptr"**

`ExecutionGeneration{two_phase_enabled, two_phase_entries, staged_group, id}` 由**一次** `atomic_store`
发布，`update()` 每周期**一次** `atomic_load`。这样"启用两趟但成员表是旧的"这种中间态在读侧不存在。
`execution_generation()` 是单调 id，测试用它断言"被拒的请求没有发布任何东西"。

**注意**：控制器**列表**不在这份 generation 里（那是上游双缓冲 + 自己的握手），所以
"安装执行路径"在周期在飞时会被**拒绝**（`control_loop_busy()`），而"移除"始终允许。

**4.14 准入（admission）：哪些配置会被拒绝**

`two_phase_admission()` 与 `two_phase_rejections()` 给出带原因的拒绝：`already_staged`（执行组成员不能被两趟
再拥有一次）、`unsupported_update_rate`（速率不能整除管理器频率，做不出整数周期桶）、
`cross_rate_dependency`（一条参考边的两端在不同周期桶；FineMote 用 Thm 3 给界，我们保证两向 0 滞后所以拒绝）、
`cross_mode_dependency`（一条参考边两端
一个是两趟成员、一个是 legacy）、`unschedulable_order`（上游 `controller_sorting()` 可能把"不 claim 任何
command interface 的 chainable 控制器"排在父节点之前，使两趟对那条边同向走错）、`duplicate_instance`
（同一个对象挂两个名字会被每阶段调用两次——实测 3 周期 6 次调用）。这些检查**在请求发出前**跑一次（对
"即将成为"的激活集合），切换后再跑一次（覆盖"只有 ACTIVE 后才看得见的 claim"）。

**4.15 atomic activation：一次 switch 的完整撤销**

三个概念要分清：

- **什么时候触发**：`set_atomic_activation(true)` 且本次 switch 有激活失败（默认关闭以保持上游 best-effort 语义）；
- **撤销什么**：① 本次从 INACTIVE 激活的控制器 → 逆序 `deactivate()+release`、reference 撤回、
  硬件 `prepare/perform_command_mode_switch({}, ifaces)` 换回；② **为切 chained mode 而被重启的既有控制器**
  → 恢复到 ACTIVE + 切换前 chained mode（判据是 `switch_controller()` 在改写请求列表前抓的 `pre_switch_state_`）；
  ③ 本次做过的所有 chained-mode 切换回退；
- **怎么知道成功**：`ActivationOutcome{activated[], rollback_performed, rollback_failed}`，
  失败**单独**上报（回滚失败 ≠ 激活失败）。

> 验证：`controller_manager/test/test_atomic_activation.cpp`（7 例，含两个"探针实测"）
> 与 `REVIEW_COMPILETIME_LATEST_RESPONSE_2026-09-26.md` §1。

**4.16 registry：编译进二进制的控制器**

`StaticControllerRegistry` 是**工厂表**（不是实例表）：`add<ControllerT>(type)` 或
`add_factory(type, factory[, descriptor])`；`load_controller()` 先查它，命中就用工厂创建**新实例**，
然后走**同一个** `add_controller_impl()`。`ControllerT::manifest` 存在时，注册表不必构造对象就能枚举其
编译期接口需求。封印规则：第一次 `load_controller()`/`add_controller()` 起类型集合**只读**，
`add()` 抛 `std::logic_error`，替换 registry 被拒。

> 验证：`controller_manager/test/test_static_controller_registry.cpp`（9 例）。

---

## 5. 动手：最小可跑的树

最快的上手方式是复制现有例子，而不是从空文件开始：

1. **只想要类型层**：抄 `hierarchical_control/test/test_static_manifest.cpp`（约 300 行，含正向与负向用例）；
2. **想要一个能跑的树**：抄 `controller_manager/test/test_composite_library/typed_fork_declaration.hpp`
   （声明：节点、端口、量纲、binding 类型、manifest）+ `typed_fork_composite_controller.cpp` 的
   `build_kernel()`（`compose` → `create_library_group` → `on_activate()` 里解析槽位）；
3. **想要管理器里的一组控制器**：抄 `controller_manager/test/test_staged_controller/`（测试替身）与
   `test_staged_execution_group.cpp` 的 fixture（配置顺序、`set_chained_mode`、安装组、逐个激活）。

写自己的树时要记住的三条硬性约束（都是实测撞出来的）：

- **控制器对象不能移动**：`ControllerInterfaceBase` 不可移动，所以树节点要用
  `std::optional<Node>`/`unique_ptr` **就地构造**，不能用 `std::tuple<Node...>` 值语义搬来搬去；
- `TypedPorts` 至少要 3 个模板参数（缺省只对后 3 张表生效，前 3 张必填）；
- **配置期建内核**：`build_kernel()` 只能在 `on_configure()`/`on_activate()`（非实时）里调用。
  库模式下管理器先 `assign_interfaces()` 再 `activate()`，所以槽位解析放 `on_activate()`；
  `update()` 里**没有**惰性建内核的分支（缺内核直接返回 `ERROR`）。

---

## 6. 调试手册

| 想确认 | 怎么做 |
|---|---|
| 执行组真的在跑哪条路径 | 成员控制器的 `legacy_update_calls` 必须为 0；`state_calls`/`command_calls` 每周期各 +1 |
| 两个方向是否同周期 | 每个节点记录 `last_state_cycle` / `last_command_cycle`，断言彼此相等 |
| 顺序对不对 | 记录全局 `sequence`：状态趟是**后序**（叶序号 < 父序号），命令趟是**先序** |
| 组是否惰性 | `cm_->staged_execution_group()->members_active()`；`run()` 返回 `StagedStatus::inactive` |
| 有没有新增实时分配 | 见 `test_scheduling_performance.cpp` 的做法：包住 `operator new` 计数，比较 N 与 N+100 次调用的差 |
| 某个控制器为什么没被两趟接管 | 看日志里的准入拒绝原因（函数名 + 控制器名），或 `two_phase_rejected_controllers()` |
| 一次切换到底做了什么 | 打开 `RCLCPP_DEBUG`/`INFO`：`Could not activate controller : 'x'`、执行组告警、回滚日志都带名字 |
| 只想跑一个用例 | `./build/controller_manager/test_two_phase_execution --gtest_filter='*same_cycle*'` |

构建与运行环境（**必须**，否则会链接到系统安装的上游库而不是你刚编译的）见 `TESTING_GUIDE.md` §2。

---

## 7. 陷阱清单（都是真实踩过的）

1. **`void*` 往返丢地址调整**：两个基类时 `static_cast` 回来的指针是错的 → 现在全程保留类型化指针。
2. **同一个控制器对象挂两个名字**：上游只查名字，于是同一周期被调用两次（实测 3 周期 6 次 `update_phase`）
   → 编译期 `rows_are_well_formed` + 运行期准入 `duplicate_instance` 双保险。
3. **"不 claim 任何 command interface 的 chainable 控制器"**会被上游排到父节点之前，使两趟对那条边**同向**
   走错（症状：计数完全正常但数据慢一周期）→ 现在是准入拒绝 `unschedulable_order`。
4. **显式欧拉伪影**：早期把 `Δt·kd = 2` 的数值边界误当成"增益上限"，后来改成精确的逐步更新。
5. **`members_active_` 缓存会过期**：非 switch 的状态变化不会被自动察觉（记为已知妥协，见
   `IMPLEMENTATION_GUIDE.md` §12.4 #1/#12）。
6. **零分配的说法要限定范围**：执行组自身 0 分配 ≠ `update()` 0 分配（上游 lifecycle 会分配）。
7. **测量口径**：`measure_compile_cost.py` 与 `measure_binding_cost.py` 的数字**不可相减**
   （不同形状、不同固定成本），见 `COMPILE_COST.md` §2/§3.1。
8. **测试宿主负载**：本机 2 核、load 经常 >3，`test_controllers_chaining_with_controller_manager` 与
   `test_spawner_unspawner` 是**既有 flaky**，判定方法见 `TESTING_GUIDE.md` §6。
