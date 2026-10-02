# 使用指南：把改造后的 ros2_control 接进你的 ROS 2 系统

> 面向**使用者**（不是本仓库的开发者）。读完你应该能：
> 把这份改造接到自己的 ROS 2 Humble 系统里、按需开启**树状分层结构**、在**库模式**和**管理器模式**
> 之间做出选择并跑通第一个例子。
>
> 想先懂原理再动手 → `ONBOARDING.md`；想读源码 → `CODEBASE_TOUR.md`；想知道每句话的证据与边界 →
> `IMPLEMENTATION_GUIDE.md` / `HANDOFF_MANUAL.md` §11；与 FineMote 论文的异同 → `PAPER_ALIGNMENT_2026-09-28.md`。

---

## 0. 30 秒选型

这套改造提供**三种用法**，按"要不要替换系统 `controller_manager`"和"要不要写 C++ 装配代码"区分：

| | A. 库模式（推荐起步） | B. 管理器模式 + 两趟执行 | C. 管理器模式 + 阶段化执行组 |
|---|---|---|---|
| 形态 | **一个**普通控制器插件，内部托管整棵树 | 若干 chainable 控制器，由管理器按两趟调度 | 若干控制器组成一个整组 |
| 要替换系统 `controller_manager`？ | **不需要**（只要 `hierarchical_control` 包） | **需要**（本仓库的 fork） | **需要**（本仓库的 fork） |
| 怎么开启 | YAML 里 `type: 你的包/你的Composite` | YAML 里 `two_phase_execution: true` | 只有 C++ API（**没有服务/参数入口**） |
| 能直接用 `ros2_control_node` + `spawner`？ | ✅ | ✅ | ❌ 需要你自己在进程里持有 `ControllerManager` |
| 双向同周期（0 周期滞后） | ✅ | ✅（同一速率桶内） | ✅ |
| 整组提交 + 帧/故障语义 | ✅ | ❌（每控制器独立提交） | ✅ |
| 逐控制器生命周期 / 局部启停 | ❌（整棵树是一个控制器） | ✅ | ✅ |
| 接线成本（代码量） | 中（写声明 + 节点） | 低（写普通 chainable 控制器） | 中 |
| 实时耗时/分配 | 最优 | 优 | 优（组自身零分配） |

**选择建议**

- 只想要"父子的双向数据同周期"，并且希望用标准 `ros2_control_node` + `spawner` 部署 → **B**（最省事）。
- 想要"整棵树要么全跑、要么全不跑"、每帧数据带周期/新鲜度/故障码、且不介意自己写一个托管进程 → **C**。
- 想完全不碰管理器、不改系统安装、可测试性最好（编译期就能查错） → **A**。
- 三者可以混用；`test_hierarchy_comparison.cpp` 里就是同一算法在三种宿主下的公平对照（输出一致、分配与耗时已量化）。

---

## 1. 装到 ROS 里

### 1.1 前置

- ROS 2 **Humble**（本改造只在 Humble 上验证过；文件里的 API 与生命周期语义按 Humble 写）
- `colcon`、`rosdep`、C++17 编译器
- 本仓库（分支 `humble-work`）

### 1.2 只做库模式：最小安装

库模式**只依赖 `hierarchical_control`**（它的依赖是 `controller_interface`、`rclcpp`、`lifecycle_msgs`，
都是 Humble 自带），所以你可以完全不碰系统的 `controller_manager`：

```bash
cd <这份改造的仓库根目录>
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control
source install/setup.bash
ros2 pkg prefix hierarchical_control        # 必须指向 <仓库>/install/hierarchical_control
```

然后在**你自己的包**里依赖它（§3.4 给了 CMake/package.xml 模板），写一个 composite 控制器插件即可。
系统 `controller_manager` 照常运行，它只会把你这一个插件当成**普通控制器**加载。

### 1.3 用管理器的新能力（B/C）：构建覆盖层

```bash
cd <这份改造的仓库根目录>
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -y        # 首次
colcon build --packages-up-to controller_manager hierarchical_control \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

验证覆盖层真的生效（**最容易踩的坑**：忘了 source，于是跑的还是系统库）：

```bash
ros2 pkg prefix controller_manager          # 必须指向 <仓库>/install/controller_manager
ros2 pkg prefix hierarchical_control
python3 -c "import subprocess"              # 无关，只是别把 source 放进子 shell
```

运行标准节点与启动器（不需要改命令行，覆盖层会优先生效）：

```bash
ros2 run controller_manager ros2_control_node --ros-args --params-file my_controllers.yaml
ros2 run controller_manager spawner my_controller
ros2 control list_controllers -v
```

### 1.4 ⚠️ ABI 提醒（用 B/C 必读）

本 fork 的 `ControllerManager` **新增了数据成员**（执行代快照、注册表、周期在飞计数、切换前快照……），
所以它的 **C++ ABI 与系统安装的 `controller_manager` 不同**。后果：

- 任何**自己实例化 `controller_manager::ControllerManager`** 的包，必须与覆盖层**一起重编**。
  典型例子是 `gazebo_ros2_control`（它在 gzserver 插件里 `make_shared<ControllerManager>`）：
  ✅ 正确做法：把 `gazebo_ros2_control` 源码放进同一个工作空间一起 `colcon build`；
  ❌ 错误做法：源码树用覆盖层编译、而 Gazebo 仍加载 apt 安装的 `gazebo_ros2_control.so`（对象大小不一致）。
- 只用 `ros2_control_node` + `spawner`/`ros2 control` CLI 的场合没有这个问题（它们与覆盖层同源）。
- 库模式（§2）没有这个问题：它不链接我们的 `controller_manager`。
- 本仓库里的 `case_study/`（Gazebo 闭环）用的是 apt 版 `gazebo_ros2_control`，属于**研究用脚手架**，
  不要照抄成部署模板。

---

## 2. 用法 A：库模式（一个控制器托管整棵树）

**最适合**：树是固定的、你想让"整棵树"成为一个可加载单元；或者你想在编译期就把拓扑/量纲/接口需求查死。

### 2.1 步骤总览

```
① 声明（编译期）：节点类型 + 端口类型 + 量纲 + binding 类型 + manifest
② 节点实现：每个节点写 update_state_stage / update_command_stage + sink/source
③ 装配：on_configure 构造节点；on_activate 解析槽位 + compose + create_library_group
④ 接口声明：command/state_interface_configuration() 由 manifest 生成（可选但强烈推荐）
⑤ 配置：YAML 里 type: 你的包/你的Composite；用 spawner 加载
```

**直接照抄的模板**（这两份就是一条可运行的完整路径）：

| 文件 | 内容 |
|---|---|
| `controller_manager/test/test_composite_library/typed_fork_declaration.hpp` | ① 一份类型声明：`typed_root -> {typed_a, typed_b}`、端口、量纲、binding 类型 |
| `controller_manager/test/test_composite_library/typed_fork_composite_controller.{hpp,cpp}` | ②③④ 节点实现、`build_kernel()`、`resolve_interface_slots()`、由 manifest 生成接口列表 |

### 2.2 ① 声明（编译期，一段就够）

```cpp
#include "hierarchical_control/static_topology.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/typed_ports.hpp"
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

struct root_n { static constexpr auto value = st::NameOf("my_root"); };
struct a_n    { static constexpr auto value = st::NameOf("my_a"); };
using my_root = st::Root<root_n>;
using my_a    = st::Descendant<a_n, my_root>;      // 环 / 复用祖先 → 编译期报错

struct a_state_n { static constexpr auto value = st::NameOf("my_a/state"); };
struct a_ref_n   { static constexpr auto value = st::NameOf("my_a/ref"); };
struct a_act_n   { static constexpr auto value = st::NameOf("joint2/velocity"); };  // 硬件 command
struct a_pos_n   { static constexpr auto value = st::NameOf("joint2/position"); };  // 硬件 state

using a_state = tc::Port<a_state_n, dm::Position>;
using a_ref   = tc::Port<a_ref_n,   dm::LinearVelocity>;
using a_act   = tc::Port<a_act_n,   dm::LinearVelocity>;
using a_pos   = tc::Port<a_pos_n,   dm::Position>;

using root_ports = tp::TypedPorts<
  tc::PortList<>/*State*/, tc::PortList<>/*Reference*/, tc::PortList<>/*Actuators*/,
  tc::PortList<a_ref>/*ForChildren*/, tc::PortList<a_state>/*ChildState*/, tc::PortList<>/*HardwareState*/>;
using a_ports = tp::TypedPorts<
  tc::PortList<a_state>, tc::PortList<a_ref>, tc::PortList<a_act>,
  tc::PortList<>, tc::PortList<>, tc::PortList<a_pos>>;
```

要点：
- `TypedPorts` 六张表依次是 **State / Reference / Actuators / ForChildren / ChildState / HardwareState**，
  **前三个必填**，后三个可省（`PORT_DIMENSIONS.md` §1.1）；
- 父侧的 `ForChildren` / `ChildState` 必须**按子节点顺序**等于各孩子声明的拼接，`compose` 会逐段检查；
- 硬件接口名只写在这里**一次**，`command/state_interface_configuration()` 由 manifest 生成，不可能写两遍不一致。

### 2.3 ② 节点实现

每个节点是一个 `TypedPortsMixin<你的节点, 它的ports>`，并实现阶段回调：

```cpp
class MyNode : public hierarchical_control::StagedControllerInterface,
               public tp::TypedPortsMixin<MyNode, a_ports>
{
  // 状态阶段（叶 → 根）：用孩子/硬件的状态算出自己的状态
  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView & children,
    hierarchical_control::StagedValueWriter state) noexcept override;

  // 命令阶段（根 → 叶）：用参考与状态算命令，写进 sink
  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView & reference,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override;

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override;      // 叶：写硬件
  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override; // 根：外部参考
};
```

契约要点（照做就不会踩）：回调**不得抛异常、不得分配、不得访问硬件 handle**；硬件只在 sink 的
`commit()` 里写；`sample_ns`/`cycle`/`valid` 由内核写，控制器只能报 `fault_code`。

### 2.4 ③ 装配（非实时）

```cpp
// 构造函数 / on_configure(): 就地构造节点（节点对象不可移动 → std::optional / unique_ptr）
//   模板里的写法：std::optional<WrappedNode<Ports>> root_node_;  root_node_.emplace(this, ...);
// on_activate():  loans 已就绪（管理器在 activate 前调用 assign_interfaces()）
if (!resolve_interface_slots()) {return CallbackReturn::FAILURE;}   // 名字 → 本次激活的槽位下标
kernel_.reset();
if (!build_kernel()) {return CallbackReturn::FAILURE;}              // compose + create_library_group
```

`build_kernel()` 的核心（`typed_fork_composite_controller.cpp:209`）：

```cpp
const auto a_leaf = tc::make_leaf<a_node, tp::contract_of_t<a_ports>>(&a_object);
const auto binding = tc::compose<root_node, tp::contract_of_t<root_ports>>(&root_object, a_leaf);
group = hierarchical_control::topology_binding::create_library_group(binding);  // 已含端口校验
```

`update()` 里只做一件事：

```cpp
controller_interface::return_type update(const rclcpp::Time & t, const rclcpp::Duration & p) override
{
  if (!kernel_) {return controller_interface::return_type::ERROR;}   // 没有惰性建内核的分支
  const auto result = kernel_->run_ns(t.nanoseconds(), p.nanoseconds());  // 每周期零分配的实时入口
  if (result.status != hierarchical_control::StagedStatus::committed)
  {
    return controller_interface::return_type::ERROR;   // 失败：本周期一个命令都没提交
  }
  // 之后就可以读本次提交的值、发布话题等（模板里把提交值抄进 committed_value_）
  return controller_interface::return_type::OK;
}
```

### 2.5 ④ 接口声明由 manifest 生成（推荐）

```cpp
static constexpr auto manifest = hierarchical_control::static_manifest::manifest_of_v<binding_type>;
static_assert(hierarchical_control::static_manifest::manifest_problem(manifest).empty());

// 小工具：把 manifest 里的 string_view 数组转成字符串表（模板见 typed_fork_composite_controller.cpp）
static std::vector<std::string> to_strings(const auto & views)
{
  std::vector<std::string> out;
  out.reserve(views.size());
  for (auto view : views) {out.emplace_back(view);}
  return out;
}

controller_interface::InterfaceConfiguration command_interface_configuration() const override
{ return make_individual_config(to_strings(manifest.command_interfaces)); }   // 生成，不手写
controller_interface::InterfaceConfiguration state_interface_configuration() const override
{ return make_individual_config(to_strings(manifest.state_interfaces)); }

// on_configure()：按名字核对"控制器声明"与"编译期描述"，不一致就 FAILURE（在任何 loan 之前）
std::string why;
if (!hierarchical_control::static_manifest::declaration_matches_manifest(
      manifest, command_interface_configuration().names,
      state_interface_configuration().names, &why))
{
  configure_error = why;              // 消息里会点名是哪个接口"要求了没声明/声明了没要求"
  return CallbackReturn::FAILURE;
}
```

### 2.6 ⑤ 配置与启动

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    my_composite:
      type: my_pkg/MyCompositeController
my_composite:
  ros__parameters:
    # 你自己定义的参数（kp、joint 名、参考值来源……）
```

```bash
ros2 run controller_manager spawner my_composite        # load → configure → activate
ros2 control list_controllers -v                        # 看它 claim 了哪些硬件接口
ros2 control list_hardware_interfaces | grep joint2
```

### 2.7 库模式常见错误

| 症状 | 原因 | 处理 |
|---|---|---|
| 编译期一串 `static_topology: CYCLE` / `DIMENSION MISMATCH` / `OWNERSHIP VIOLATION` / `EDGE MISMATCH` | 声明本身不合法 | 这是**想要的**：按诊断改声明；15 个负向样例见 `hierarchical_control/test/static_topology_negative/` |
| `TypedPorts` 报模板参数数量错 | 前 3 张表是必填的 | 补齐前三个 `PortList<...>`（空写 `tc::PortList<>`） |
| 控制器对象无法拷贝/移动 | `ControllerInterfaceBase` 不可移动 | 用 `std::unique_ptr` 或 `std::optional` **就地**持有节点 |
| `create_library_group` 抛 `std::invalid_argument` | 行的结构错（重名/多根/父不存在/自父/**同实例两名字**） | 异常消息里有原因；编译期 `rows_are_well_formed` 也会拦 |
| 激活后第一个周期分配 | 把 `build_kernel()` 放进了 `update()` | 只能在 `on_configure()`/`on_activate()` 建内核 |
| `state_failed` / `command_failed` | 某个阶段返回 ERROR、写 NaN 或漏写端口 | 日志里有 `at node i ('名字')`；检查端口数量与 `fault_code` |

---

## 3. 用法 B：管理器模式 + 两趟执行（全 YAML 可配）

**最适合**：你已经有若干 chainable 控制器（每个都是普通插件），只想让父子**双向同周期**。

### 3.1 控制器要实现什么

1. 继承 `controller_interface::ChainableControllerInterface`（上游要求；用于父子参考接口 + chained mode）；
2. **再实现** `hierarchical_control::TwoPhaseControllerInterface`：
   `update_phase(time, period)`（底向上：读孩子/硬件状态，算出自己的状态）与
   `handle_phase(time, period)`（顶向下：读参考，算命令并写出去）；
3. 命名约定：**父写子**。父的 `command_interface_configuration()` 里出现 `<子控制器名>/<端口>`
   （例如 `wheel_left/travel`），内核/管理器就把它当成"父 → 子"的参考边；
4. 建议保留一个"单趟回退"参数（本仓库案例用 `two_phase_legacy`），让同一份代码在关闭两趟时
   走原生 `update()`，便于 A/B 对照。

照抄模板：`case_study/include/case_study/{wheel,chassis}_controller.hpp`（叶/根各一个，含参考接口、
PI 计算），共约 180 行。**chained mode 由管理器负责**：父激活时它会把被写参考的子控制器切到
chained mode（若该子控制器当时已经 ACTIVE，上游会先停后启一次——这是 Humble 的限制，不是错误）。

> 注意区分：**库模式不需要 chained mode**。库模式里整棵树都在一个普通控制器内部，内部节点不注册给
> 管理器，父子之间是内核的端口，不走上游 chained-mode 机制。

### 3.2 开启

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    two_phase_execution: true       # ← 管理器级开关（构造时读取）
    atomic_activation: true         # ← 建议：切换失败时整组回滚（见 §5）
    wheel_left:
      type: case_study/WheelController
    wheel_right:
      type: case_study/WheelController
    chassis:
      type: case_study/ChassisController
```

```bash
ros2 run controller_manager ros2_control_node --ros-args --params-file controllers.yaml
for c in wheel_left wheel_right chassis; do ros2 run controller_manager spawner "$c"; done
```

两条入口的语义**不同**，按需选：

| 入口 | 语义 |
|---|---|
| 参数 `two_phase_execution: true` | 先只置位；成员集合在**首次加载/切换**时推导。不合规的成员被**排除并记日志**（尽力而为），模式保持开启 |
| C++ API `set_two_phase_execution(true)` | **全有或全无**：只要有任何成员不合规就返回 `ERROR` 且**不改变状态**（推荐在自动化部署里用这个） |

### 3.3 必须满足的四条规则（不满足会被拒绝，日志会说明原因）

| 规则 | 为什么 | 拒绝原因（日志里会出现） |
|---|---|---|
| 一条参考边的两端**都要**在树里，要么都用两趟、要么都不用 | 两端被不同调度排序 → 边会悄悄晚一周期 | `cross_mode_dependency` |
| 参考边的**父必须排在子之前**（看 `ros2 control list_controllers` 的顺序） | 两趟是"反向走状态趟 + 正向走命令趟"，靠这个顺序成立 | `unschedulable_order`（会点名两端） |
| 两端的**速率桶相同**：速率要么是 `0`/≥ 管理器频率，要么**整除**管理器频率 | 分桶后每桶各跑一次两趟 | `cross_rate_dependency` / `unsupported_update_rate` |
| 同一个控制器对象不能挂两个名字 | 否则一周期被推进两次 | `duplicate_instance` |

> 顺序最常见的坑：**chainable 子控制器如果声明 0 个 command interface，上游排序会把它排到父之前**。
> 修法：让它声明自己真正会写的硬件 command interface（案例里的 wheel 就声明了关节速度）。

### 3.4 速率桶（多速率）

声明了较低速率并**整除**管理器频率的成员会进入**自己的周期桶**：每 `factor = 管理器频率/速率`
个周期跑一次自己的两趟，两个阶段收到的是**桶自己的周期**（与原生循环对降频控制器的处理一致）。
**跨桶的参考边会被拒绝**（论文在跨周期向下方向只有有界延迟；本实现对外保证"每条被接纳的边两向
0 周期滞后"，所以宁可拒绝也不降级）。不能整除的速率一律拒绝（避免偷偷升降频）。

### 3.5 验证它真的在按两趟跑

```bash
ros2 control list_controllers -v        # 看 is_chained / claimed_interfaces
grep -E "Two-phase|excluded" ~/.ros/log/latest/*.log
```

日志里会看到：
- `Two-phase execution requested by parameter: true`（配置生效）
- `Two-phase execution excluded N controller(s) …`（有成员被排除 → 用 C++ API 时则会直接拒绝并说明原因）

运行时目测延迟可用 `case_study/scripts/measure_tracking.py`（对比 `single-pass` 与 `two-pass` 的
`sample_ns` 与命令时间差）。

关闭：参数改 `false` 重启，或在运行中 `set_two_phase_execution(false)`（**移除路径始终允许**，
即使控制周期正在跑）。

---

## 4. 用法 C：管理器模式 + 阶段化执行组

**最适合**：你要"整棵树全跑或全不跑 + 每帧带周期/新鲜度/故障码 + 整组提交"，并且能自己托管管理器。

### 4.1 控制器要实现什么

继承 `hierarchical_control::StagedControllerInterface`（端口声明 + 两个阶段 + `staged_command_sink()`；
树节点可用 `TypedPortsMixin` 生成端口字符串，也可手写字符串）。

### 4.2 ⚠️ 没有 ROS 入口（诚实说明）

执行组目前**只有 C++ API**：`ControllerManager::set_staged_execution_group(...)`。
没有服务、没有参数、CLI 也没有。原因是准入判定必须对着"控制器列表 + 执行代"的一致快照做，
把它暴露成异步服务需要额外的仲裁设计，本仓库还没做。你的选择：

- 如果你能接受"每控制器独立提交" → 用**用法 B**（全 YAML）；
- 如果你要整组语义 → 在**你自己的进程**里构造 `controller_manager::ControllerManager` 并调用 API
  （`controller_manager/test/test_staged_execution_group.cpp` 的 fixture 就是最小范例：构造管理器、
  配置控制器、调 API、跑 `read/update/write`）。

### 4.3 调用顺序（照这个顺序就不会踩）

```cpp
ControllerManager cm(std::make_unique<ResourceManager>(urdf, true, true), executor, "controller_manager");

// 1) 加载并 configure 成员（先子后父：子的参考接口要被父 claim）
cm.load_controller("leaf",  "my_pkg/Leaf");
cm.configure_controller("leaf");
cm.load_controller("root",  "my_pkg/Root");
cm.configure_controller("root");

// 2) 可选：提前把子控制器设为 chained mode。管理器本来就会在父激活时做这件事，但
//    `set_chained_mode()` 只允许非 ACTIVE 时调用，所以对已经 ACTIVE 的子控制器，上游会先停后启
//    一次（本 fork 保证切换失败时把它恢复）。提前设置可以避免这次重启。
leaf->set_chained_mode(true);

// 3) 执行组要求 all-or-nothing 激活已开启（否则 set_staged_execution_group 返回 ERROR）
cm.set_atomic_activation(true);

// 4) 安装执行组（成员）必须在 INACTIVE；成功即发布新的执行代
if (cm.set_staged_execution_group({"root", "leaf"}) != controller_interface::return_type::OK)
{ /* 日志里有原因：未知成员/已激活/未实现接口/速率不匹配/拓扑错误 */ }

// 5) 逐个激活（Humble 生命周期）；成员不全为 ACTIVE 时整组**惰性**，并会打印告警
cm.switch_controller({"leaf"}, {}, STRICT, true, rclcpp::Duration(0,0));
cm.switch_controller({"root"}, {}, STRICT, true, rclcpp::Duration(0,0));

// 6) 周期：read → update → write（组在 update() 里跑）
cm.read(t, p); cm.update(t, p); cm.write(t, p);
```

### 4.4 执行组的硬性约束

- 成员必须**已加载、已 configure、INACTIVE**；
- 成员**速率必须等于管理器频率**（组是"每周期一次、整组提交"的单位，没有桶；想多速率用**用法 B**）；
- 成员不能同时是两趟成员（两条路径会在同一周期各执行一次）；
- **卸载成员之前必须 `clear_staged_execution_group()`**（激活不受限制：成员可以逐个激活，只是成员没
  全 ACTIVE 时组是**惰性**的）；成员集合变化（加载/configure/unload/切换）会重新推导成员；
- 周期在飞时**安装/扩展**执行路径会被拒绝（`control_loop_busy()`），**移除**始终允许。

---

## 5. 可靠性开关：`atomic_activation`

```yaml
controller_manager:
  ros__parameters:
    atomic_activation: true
```

- **默认关闭**，因为上游 Humble 的语义是"尽力而为"（一个控制器激活失败，其它已激活的保持运行）；
  打开后，一次 switch 里只要有控制器失败，**本次这次 switch 激活的**控制器会被撤销回 INACTIVE
  （含硬件 command mode、chained mode、reference 接口发布的回退）；
- 使用执行组（用法 C）时**必须**打开；用法 B 建议打开（树被切成一半时调度语义不成立）；
- 回滚失败会单独记为 rollback failure（不会伪装成激活失败）；
- 作用域：不撤销**你显式请求的 deactivate**（上游顺序如此），也不提供物理总线原子性。

---

## 6. 运行期操作约束（配置期 vs 运行期）

| 操作 | 允许时机 |
|---|---|
| `set_two_phase_execution(false)` / `clear_staged_execution_group()`（**移除**路径） | **任何时刻**（在飞周期持有自己的快照） |
| `set_two_phase_execution(true)` / `set_staged_execution_group(...)`（**安装/扩展**路径） | 必须没有周期在飞（`control_loop_busy() == false`）；原因是成员判定要对着控制器列表，而列表仍是上游双缓冲发布 |
| `add_controller` / `configure_controller` / `unload_controller` | 非实时线程；会重建两趟成员集，因此**可能**返回 ERROR 并说明原因 |
| 切换（activate/deactivate） | 任何时刻；两趟与执行组在切换期间暂停，切换完成后恢复 |
| 运行中改拓扑/端口声明 | **不可能**：描述是类型（编译期） |

---

## 7. 参数与 API 速查

管理器参数（`controller_manager:` 段）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `update_rate` | 100 | 管理器周期（Hz，上游） |
| `wcet_ns`（**每个控制器**自己的参数） | 未声明 | 该控制器一周期（两个阶段合计）的最坏执行时间，纳秒。声明后管理器会给出声明口径的可调度性报告；不声明则报告为"未检查" |
| `two_phase_execution` | false | 开启两趟执行（成员在首次列表变化/切换时推导） |
| `atomic_activation` | false | 本次 switch 的全有或全无回滚 |

C++ API（需要覆盖层）：

| API | 用途 |
|---|---|
| `set_two_phase_execution(bool)` | 全有或全无地开关两趟；返回 `ERROR` 时状态不变 |
| `set_staged_execution_group(names, max_age_ns=0)` / `clear_staged_execution_group()` | 安装/清除执行组 |
| `staged_execution_group()` | 查当前组（`members_active()`、`member_names()`、`size()`） |
| `set_atomic_activation(bool)` / `atomic_activation()` | 可靠性开关（安装执行组的前置条件） |
| `execution_generation()` | 执行代单调 id（可观测"配置变更是否发布"） |
| `control_loop_busy()` | 周期是否在飞（决定能否安装执行路径） |
| `two_phase_rejected_controllers()` | 当前被两趟排除的成员及原因 |
| `two_phase_schedulability()` | 声明 WCET 口径的可调度性报告（利用率、Liu–Layland 界、是否满足、最忙的桶） |
| `set_static_controller_registry(...)` / `register_static_controller_type<T>(type)` | 编译内置控制器（工厂注册；首次加载前注册） |

---

## 8. 故障排查

| 症状 | 可能原因 | 处理 |
|---|---|---|
| 参数写了但行为没变 | 覆盖层没 source，跑的是系统 `controller_manager` | `ros2 pkg prefix controller_manager` 必须指向你的 install |
| `set_two_phase_execution` 返回 ERROR，日志 `is ordered AFTER its child` | 列表顺序不满足"父在子前" | 让 chainable 子声明它真正写的 command interface；或调整加载顺序 |
| 日志 `excluded N controller(s)` | 参数路径的尽力而为排除（有成员不合规） | 用 `two_phase_rejected_controllers()`（C++）或日志里的逐条原因定位；固定成员集合后改用 C++ 开关做全有或全无 |
| `cross_mode_dependency` | 一条参考边一端是实现两趟的控制器、一端是普通控制器 | 让两端都实现两趟，或把普通控制器移出这棵树 |
| `cross_rate_dependency` | 边两端速率不同（不同桶） | 统一速率，或把这条边拆开；可整除的**同速率**降频是允许的 |
| `unsupported_update_rate` | 速率不能整除管理器频率 | 改成 `0`、≥ 管理器频率，或能整除的值（如 100 下的 50/25/20） |
| 组安装被拒：`must be inactive` / 未知控制器 / 未实现接口 | 直接看日志 | 先 configure、保证 INACTIVE、确保实现了 `StagedControllerInterface` |
| 激活后整组不跑，日志 `Staged execution group is not fully active` | 成员没全部 ACTIVE → 组**惰性**（设计如此） | 激活剩余成员；或检查某个成员是否激活失败 |
| `Resource conflict for controller 'X'. Command interface '…' is already claimed` | 两个控制器抢同一 command interface（或父/子参考边写错人名） | 检查 `<控制器名>/<端口>` 命名与前缀 |
| 明明"跑起来了"但父读到的还是上一周期 | 该边跨模式、跨桶，或列表顺序反了 | 见上三行；两趟/执行组只在**被接纳**的边上保证 0 滞后 |
| spawner 报 `Could not contact service …/list_controllers` | 服务发现超时（宿主负载高时常见，1 s 默认超时不够） | 提高 `--controller-manager-timeout`；这与本改造无关 |

---

## 9. 性能与实时注意事项

- **两趟比单趟慢**（多走一遍列表，结构性代价），换来的是"两个方向都同周期"；控制意义上的收益见
  `CONTROL_COST_OF_LAG.md`（`ΔPM = 360·f_c·L·Δt`）。**不要**把它当成加速方案。
- **执行组自身在配置完成后零分配**（`run()` 100 次 0 次分配，有断言）；但
  `ControllerManager::update()` **整体仍会分配**，来源是上游 Humble 代码（lifecycle 状态查询、
  控制器列表拷贝）。因此"接进来就能零分配"不成立，正确说法是"我们没有新增实时路径分配"。
- **有"声明 WCET"口径的可调度性检查，但没有 WCET 分析**：给每个控制器声明 `wcet_ns` 后，管理器按
  速率桶算利用率并套用 Liu–Layland 充分条件（日志会给结论，`two_phase_schedulability()` 可读）。
  这是**你声明什么就检查什么**：不声明 → 报告"未检查"；界不满足 → 只报警告，**不拒绝**配置
  （调度仍能跑，只是没有期限保证）。也不建模通信/总线负载。真正的 WCET 与余量仍需你自己负责。
- 控制循环里不要做分配/日志风暴；成员激活缓存（`refresh_member_active_state()`）只在切换后刷新，
  **非切换导致的状态变化**可能让缓存过期（已知妥协）。

---

## 10. 接线自检清单

- [ ] `ros2 pkg prefix controller_manager` / `hierarchical_control` 指向你的 install（用了 B/C 时）
- [ ] 库模式：`static_assert(manifest_problem(manifest).empty())` 通过；`create_library_group` 不抛异常
- [ ] 库模式：`on_activate` 之后第一个周期**零分配**（打点做法见
      `test_hierarchy_comparison.reported_overhead_and_allocations`：包住 `operator new`，比较 N 与 N+100 次调用的差）
- [ ] 两趟：`ros2 control list_controllers` 顺序里父在子之前；`two_phase_rejected_controllers()` 为空
- [ ] 两趟：速率要么 0/≥管理器、要么整除管理器；同一棵树不跨桶
- [ ] 执行组：`set_atomic_activation(true)` 在前；成员都 INACTIVE；卸载前 `clear_staged_execution_group()`
- [ ] 每个阶段每周期**恰好一次**（用控制器里的计数器确认，别只看输出数值）
- [ ] 失败路径验证过：故意让某个阶段返回 `ERROR`，确认**没有任何硬件命令被提交**
- [ ] 多实例：同一份声明被两个管理器加载时，状态互不影响

---

## 11. 参考

| 想知道 | 打开 |
|---|---|
| 概念与最小背景 | `ONBOARDING.md` |
| 源码怎么读、每层文件是谁 | `CODEBASE_TOUR.md` |
| 怎么跑测试、怎么复现测量 | `TESTING_GUIDE.md` |
| 开发经过与被推翻的结论 | `DEVELOPMENT_HISTORY.md` |
| 权威实现状态 / 尚未做 | `IMPLEMENTATION_GUIDE.md`、`HANDOFF_MANUAL.md` §11 |
| 与 FineMote 论文逐条对照 | `PAPER_ALIGNMENT_2026-09-28.md` |
| 端口六张表的口径 | `PORT_DIMENSIONS.md` §1.1 |
| 三种宿主的公平对照（耗时/分配/故障） | `HIERARCHY_FAIR_COMPARISON.md` |
| 真实 Gazebo 闭环案例（含脚本） | `GAZEBO_CASE_STUDY.md`、`case_study/scripts/phase_b.sh` |
