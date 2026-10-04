# URDF 的 `ros2_control` 字段还能前移到编译期吗？

日期：2026-10-03　分支：`humble-work`　前置：`COMPILETIME_AUDIT_2026-10.md`（F9 说"URDF 显然是运行期"）

> **先给结论**：把 **URDF 文件本身拿到编译期去解析是错的方向**，而且会制造**假保证**。
> 真正值得做的是反过来——**让编译期类型成为那段 URDF 的来源**（生成），
> 以及把 URDF 里的**闭词表**（标签名、接口名后缀）变成 `constexpr`，用它去**校验证**部署数据。

---

## 0. 为什么"编译期解析 URDF"是错的方向

| 事实 | 证据 |
|---|---|
| URDF 是**运行期数据**，不是构建输入 | 它经 `robot_description` **topic/参数**到达（`controller_manager.cpp` 的 `robot_description_callback` / `init_resource_manager`），`ros2_control` 解析发生在 `component_parser.cpp:501`（"Parse a control resource from an `ros2_control` tag"） |
| 它常由 **Xacro 参数化生成**，同一份代码对应不同机器人 | 上游工作流即如此 |
| 它的**词表**是编译期已知的 | `component_parser.cpp:31-48`：`ros2_control` / `hardware` / `plugin` / `param` / `joint` / `sensor` / `gpio` / `command_interface` / `state_interface` / `name` / `type`，全部是 `constexpr const auto` 字面量 |
| 它的**内容**是纯数据、且以 `std::string`/`std::vector` 承载 | `hardware_info.hpp:105+`：`HardwareInfo{name,type,hardware_class_type,hardware_parameters,joints,sensors,gpios}`；`hardware_class_type` 还要 `dlopen` |

编译期解析只能覆盖"**构建时就固定、且不再变**"的机器人。对真实部署它给不出保证，
却会让审稿人/用户以为"URDF 已经被检查过了"——这是比不检查更糟的**假保证**。

**结论**：URDF 属于审计 §4 那条分界线的**部署侧**。正确做法不是把它搬到类型侧，
而是**缩小"部署数据"与"编译进来的契约"之间可能分叉的面积**。

---

## 1. 值得做（按价值排序）

### U1（最强）：让**编译期类型成为那段 URDF 的来源**（生成，而非解析）

研究线已经有一个"从 binding 的 **TYPE** 导出"的描述层：`static_manifest` 暴露
`command_interfaces` / `state_interfaces`（`static_manifest.hpp`），而
`TypedPortsMixin` 已经用它在**运行期生成** `command_interface_configuration()`，
从而消掉了"手写字符串"这一层（`typed_ports.hpp:305-355`）。

**同样的手法可以再往前一步**：用一个**构建期小程序**实例化 `manifest_of_v<Binding>` 并
**打印出 `<ros2_control>` 块**，由 CMake 生成 URDF 片段。于是：

* 手写 URDF 与控制器声明**不可能分叉**——因为 URDF 的那一段是**生成的**；
* 与现有哲学一致："声明一次，字符串由它生成"；
* 对论文的意义：这正好回应了调研里那个最可能的攻击点（"ROS 2 的静态校验答案是 codegen 而不是 TMP"）——
  答案是**分工**：codegen 管**值/文本**（URDF、参数），TMP 管**结构/量纲**（层次、所有权、量纲），
  两者在这里**接起来**：TMP 产出 manifest，codegen 用 manifest 产出 URDF。

代价：manifest 打印器（~100 行）+ CMake 步骤 + 一个生成文件的测试。风险低（纯新增，不改上游）。

### U2：接口名后缀 → **量纲** 的 `constexpr` 映射，用它校验 URDF 声明的接口

URDF 里接口用后缀命名：`position` / `velocity` / `acceleration` / `effort`（闭词表，见
`hardware_interface/types/hardware_interface_type_values.hpp` 的 `HW_IF_*`）。
研究线的 `dimensional_interfaces.hpp` 已经有量纲代数与 `Port<Name, Dimension>`。

**新增一小张 `constexpr` 映射**：`dimension_of_interface("velocity") -> LinearVelocity`。
于是配置期可以拿 **URDF 声明的接口**（运行期数据）去对照 **manifest 的量纲**（编译期），
把"URDF 写 `position`、控制器要 `velocity`"这类错误变成**点名 + 量纲**的错误，
而不是"数量对不上"或激活期才失败。

这是"部署数据 × 编译期词表"的最干净例子：**词表编译期化，数据仍然运行期**。

### U3：URDF 派生出的**可用接口列表**接进已有的对照函数（F9 的残余）

现状：`declaration_matches_manifest()`（`static_manifest.hpp`）只把**控制器自己的声明**与 manifest
对照，在 `on_configure` 跑；**URDF 派生的可用列表没有参与**（审计 F9）。
把它接进去（`resource_manager_->available_command_interfaces()` / `available_state_interfaces()`），
可以在 configure 期就给出"URDF 缺少 manifest 要求的接口 X"的点名错误，而不是等到激活时
由 `ResourceManager` 抛 "is already claimed"/找不到接口。

仍然是**运行期**（数据是运行期的），但**更早、更完整、可读**。代价很小。

### U4：URDF 的**结构性规则**变成 `constexpr` 谓词，供解析器与生成器共用

`ros2_control` 段有一组固定的结构约束，例如：`type` ∈ {`system`,`actuator`,`sensor`,`gpio`}；
system/actuator 必须有 `joint`；sensor 必须有 `sensor`；`hardware` 必须有 `plugin`；
`command_interface`/`state_interface` 必须有 `name`。这些现在是**散落的运行期 `throw`**
（`component_parser.cpp` 多处 `throw std::runtime_error`）。

把它们写成一张共享的 `constexpr` 规则表，价值有两层：
1. **生成器与解析器同源**——U1 生成的 URDF 一定满足解析器接受的形状，不会"生成出来却解析失败"；
2. 解析器的错误信息可以**由规则表生成**，一致性不再靠人肉维护。

价值属"内部一致性"，不改变用户可见行为，因此排在 U1/U2/U3 之后。

### U5（小）：接口名文法 `<owner>/<name>` 的编译期化

`topology_contract.hpp` 已有 `owner_of()`；把它提升为 `constexpr` 文法（含"必须以 `/` 分隔、
两段都非空"），即可让"非法的接口名"在**生成期**就被拒绝，而不是在运行期切分时才发现。
收益小，但与 U1 配套。

---

## 2. 明确不能前移（写在这里避免反复讨论）

| 内容 | 为什么 |
|---|---|
| `<ros2_control>` 的 **joint/sensor/gpio 集合** | 机器人数据；随机器人与 Xacro 参数变化 |
| `<param>` 的**值** | 运行期配置，等价于 YAML |
| `<hardware><plugin>` 的**类名字符串** | 需要运行期 `dlopen`；pluginlib 本质运行期 |
| `type` 字段的**每个实例取值** | 数据（词表可编译期化，取值不能） |
| 硬件是否真的存在/可用 | 运行期事实 |
| URDF 文件本身能否编译期解析 | 见 §0：运行期参数 + Xacro + 随机器人替换 ⇒ 只对固定机器人有效，是**假保证** |

---

## 3. 这对论文的意义

审计 §4 的那句话（"由类型唯一决定的可前移，由部署唯一决定的必须留运行期"）在 URDF 上
有一个**非平凡的推论**：

> 当某个输入位于"部署侧"时，仍有第二种前移方式——**不是把输入搬到编译期，而是把编译期类型
> 变成该输入的来源**（生成）。URDF 正是这种情况：它的**内容**不能前移，但它的**那段
> `ros2_control` 声明**可以由 manifest 生成，于是"编译期契约 vs 部署数据"的分叉面被消掉，
> 而不是被检查掉。

这条推论给"编译期能推到哪"这个枚举式结论**补上了第三种手段**（除"编译期检查"与"编译期拒绝"之外的
**编译期生成**），而且它恰好把 TMP 与 codegen 的分工说清楚了——这正是调研里预测的审稿攻击点。

---

## 4. 建议的顺序（若要实现）

1. **U3**（最小、立刻可用：把 URDF 可用列表接进对照，点名错误）；
2. **U2**（后缀→量纲映射 + 用它校验 URDF 声明的接口）；
3. **U1**（manifest 打印器 + 生成 URDF 片段 + CMake 步骤）—— 这一步价值最高但改动在构建系统，
   建议在有 CI 之后做（见 `PENDING_WORK.md` §4.2）。
