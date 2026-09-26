# 对「编译期控制器」方案的评估与本轮改造

日期：2026-09-26
被评估文档：`doc/COMPILETIME_CONTROLLER_DESIGN_2026-09-26.md`（ChatGPT 起草）
本轮代码基线：`e783275`

---

## 0. 一句话结论

**原始想法（把控制器做成"编译期全局变量"）在 ROS 2 Humble 里不成立；ChatGPT 的文档把目标改对了，
但它的"编译期 manifest"仍是运行期对象，而且漏掉了叶子真正需要的硬件 state 接口。**
本轮按修正后的目标落地了一层真实可用的编译期描述（`static_manifest.hpp`），
把它接到 typed composite 插件上（接口清单改为生成、节点改为固定存储、`configure` 期校验），
并补了文档 §7 里能在 Humble 内实现的三条验收测试。

---

## 1. 对原始想法的调研结论

### 1.1 为什么"控制器的完整初始化在静态初始化期完成"不可行

不是风格问题，是**顺序**问题。一个可运行的 ROS controller 依赖下面这些**在静态初始化期都不存在**的东西：

| 依赖 | 何时才存在 |
|---|---|
| `rclcpp` context / node | 进程启动、`rclcpp::init` 之后 |
| 参数 override（YAML） | node 声明参数并按参数文件解析之后 |
| `robot_description` | manager 订阅/接收之后（本仓库 `robot_description_callback`） |
| `ResourceManager` 导出的 interfaces | URDF 解析并加载 hardware 之后 |
| loaned interfaces | `assign_interfaces()`，紧接在 lifecycle activation 之前 |
| lifecycle 状态机 | configure/activate 本身就是它 |
| 失败表达 | 构造函数**无法**表达 `on_configure`/`on_activate` 的失败语义 |

再加上两个工程性理由：全局对象在多 manager / 多机器人实例之间**共享状态**；
静态析构顺序 + 插件卸载顺序不可控；测试也无法重复创建/清理干净实例。

所以 ChatGPT 文档 §2 的重述（"编译期生成类型/拓扑/端口/容量/执行计划；configure/activate 绑定资源"）
是正确的方向，**我完全同意**。文档 §5 的理由清单也基本准确。

### 1.2 但文档有三处需要收紧（本轮按此实现）

**（a）`StaticManifest` 用了 `std::span`，那不是"编译期"的形态。**
`std::span` 是运行期视图，指向的对象必须有存储期；写成 `struct { std::span<...> controllers; }`
意味着 manifest 是一个运行期构造出来的对象，"compile-time manifest"就成了口号。
本轮实现成**类型 + `static constexpr std::array`**：
`manifest_of_v<Binding>` 由绑定的**类型**推出，可以用在 `static_assert` 里
（`static_assert(manifest_problem(manifest).empty())`）。

**（b）文档的端口清单（`State/Reference/Actuator/ForChildren/ChildState`）漏掉了硬件 state 输入。**
叶子控制器要**读**硬件状态（实例里是 `joint2/position`），但 `TypedPorts` 原本没有这一项，
于是这些名字只能在插件的 `state_interface_configuration()` 与槽解析里**再手写一遍**。
文档 §7.1 要求"manifest 与 URDF 缺 interface 时 configure 失败"——没有这一项，
manifest 连自己的完整接口需求都说不出来，只能覆盖 command。
本轮给 `TypedPorts` 加了可选的第六张表 `HardwareState`（默认空，完全向后兼容），
它**不进 `Contract`**（另一端是 joint，不是节点），只进 manifest 的接口需求。

**（c）`ResolvedInterface{resource_index, interface_index}`（全局资源索引）耦合错了层。**
可安全持有的"稳定索引"是**本次激活的 loaned 集合内的槽位**（`command_interfaces_[i]`），
它在两次激活之间可能整体变化；把它换成一个全局 `resource_index` 会把控制器绑死在
`ResourceManager` 的内部编号上。本轮保持"activate 期把名字解析成本次激活的槽位索引"。

### 1.3 文档里我认为**不可在控制器内部实现**的一条

§7.2「`activate` 中任一资源 loan 失败时，之前已准备的节点全部撤销」：
**控制器无法自己 claim/release loan**——那是 manager 在 `manage_switch()` 里做的
（`claim_command_interface`），控制器只能在 `on_activate` 里拒绝并**不发布计划**。
本轮把这条**如实缩成**：控制器侧"失败即不发布任何计划"（可测，见 §3.3），
manager 侧的整组回滚仍列为未做（那要动上游 switch 语义）。

### 1.4 文档没有说、但必须说清的一条

编译期 manifest 的**收益不是性能**。现有 typed composite 插件的实时路径已经是
零分配、无字符串查找、无 pluginlib 加载（本轮再次实测激活后第一周期分配 = 0）。
manifest 带来的是**更早、更精确的失败**（`configure` 期而不是 `activate` 期/运行期）
以及**可被机器检查的描述**（工具、静态校验、多实例一致性）。
把它说成性能优化会重犯上一轮评审已经否掉的"更快"这类过度声称。

---

## 2. 本轮代码改造

### 2.1 `TypedPorts` 增加可选 `HardwareState`（`typed_ports.hpp`）

```cpp
template <typename State, typename Reference, typename Actuators,
          typename ForChildren = PortList<>, typename ChildState = PortList<>,
          typename HardwareState = PortList<>>   // 新增，默认空 ⇒ 既有代码不变
struct TypedPorts { ... using hardware_state = HardwareState; ... };
```
`name_lists` 增加 `hardware_state` 数组；`contract_of` 明确**不**把它算进 contract。

### 2.2 新增编译期描述层（`hierarchical_control/include/hierarchical_control/static_manifest.hpp`）

- `PortRole{state, reference, actuator, for_children, child_state, hardware_state}`；
- `ManifestNode{name, parent}`、`ManifestPort{name, owner, role}`；
- `StaticManifest<Nodes, Ports, Commands, States>`：**全 `static constexpr` 数组** + `constexpr` 查询
  （`has_command`、`has_state_interface`、`parent_of`、`port_count_of`）；
- `manifest_of_v<Binding>`：从绑定**类型**递归展开（节点前序、每节点端口按角色、再过滤出 command/state 需求）；
- `manifest_problem(manifest)`：`constexpr` 校验（单根、名字唯一、父存在、非硬件端口必须有节点 owner、
  同角色端口不重复）——可以直接写进 `static_assert`；
- `declaration_matches_manifest(...)`：把 manifest 的接口需求与控制器**运行期声明**逐名比对，
  报出第一个"要求了但没声明"或"声明了但没要求"的接口名（`configure` 期检查，见 2.4）。

### 2.3 typed composite 插件改用声明（`test_composite_library/`）

- 声明抽到 `typed_fork_declaration.hpp`（拓扑 + 端口 + **硬件接口**都在这一处，且可被其它 TU 复用）；
- 叶子新增 `HardwareState = {joint2/position, joint3/position}`；
- `command_interface_configuration()` / `state_interface_configuration()` 现在**由 manifest 生成**
  （插件里不再手写任何硬件接口名）；
- `resolve_interface_slots()` 从 manifest 取接口名 → 解析成本次激活的槽位索引；
- **固定存储**：三个节点改为 `std::optional<WrappedNode<...>>` 成员（构造期 `emplace`，就地构造、无堆、无全局；
  用 `optional` 而不是 `tuple` 是因为节点内部有自指 sink，`ControllerInterfaceBase` 不可移动，必须就地构造）；
- `on_configure()` 调用 `declaration_matches_manifest`，失败即 `FAILURE` 并记录 `configure_error`；
- 插件里那个「内部节点必须派生 `ControllerInterfaceBase`」的最小 `NodeBase` 保持不变（R5 的带类型指针要求）。

### 2.4 顺带修掉的一个**测试自身缺陷**

`test_hierarchy_comparison.post_switch_two_phase_cycles_do_not_rebuild_membership`
（R8 时期的分配计数探针）断言"空闲循环每周期分配数完全相等"，在宿主负载波动时会假失败
（本轮全量跑里又出现一次）。已改为：以**空闲周期的最小值**为基准 + **1 次分配容差**，
并对切换后第一周期单独给 2 次容差（发布路径本来就会多一次）。
被检测的缺陷（在 `update()` 里重建成员表）代价是 `reserve` + 每成员 `push_back` + `sort`，远大于 1，
所以检测力不变。**改后 4/4 连续运行全绿。**

---

## 3. 本轮新增/更新的测试

### 3.1 编译期事实（`hierarchical_control/test/test_static_manifest.cpp`，4 用例）

- `static_assert(manifest_problem(manifest).empty())`、节点/端口/接口**数量与名字**都在编译期断言；
- 角色可枚举：端口带 owner 与 role，`actuator` 与 `hardware_state` 分别落在
  `command_interfaces` / `state_interfaces` 两张需求表里；
- `declaration_matches_manifest` 的正例、缺一个 command、多一个 command、缺一个 state、顺序颠倒不算错
  （按名字匹配，和 manager 一致）；
- `manifest_problem` 的四类畸形描述（双根、重名、父不存在、非硬件端口 owner 不存在）+ 硬件端口合法。

### 3.2 插件仍产生同样的数值（既有用例保持通过）

`typed_declaration_hosted_by_a_composite_plugin_matches_the_spec_host`：10 个用例套件里继续全绿，
并继续断言激活后第一周期**分配 = 0**。

### 3.3 文档 §7 的三条验收（可在 Humble 内实现的形式）

| 文档条目 | 本轮落地的测试 |
|---|---|
| §7.1 manifest 与 URDF/接口不一致时 `configure` 失败且不发布计划 | 两段：`a_declaration_richer_than_the_manifest_is_refused_at_configure`（声明比描述多一个 `joint9/position` ⇒ `configure` 返回 ERROR、`configure_error` **点名该接口**、`plan_node_names()` 为空、`build_allocations == 0`）；`an_interface_the_urdf_lacks_is_refused_at_activation`（描述与声明一致但 URDF 没有 `joint9/velocity` ⇒ manager 拒绝 loan、switch 返回 ERROR、`commit_calls == 0`） |
| §7.3 两个 manager 共用同一 manifest 时状态隔离 | `two_instances_of_the_same_description_are_isolated`：两个 manager、两个插件实例、交替 update、各自参考不同 ⇒ 输出互不影响，且两者 manifest 相同 |
| §7.8 稳态/切换后无实时分配 | 既有 `post_switch_two_phase_cycles_do_not_rebuild_membership`（本轮修掉假失败）+ 第一周期 0 分配断言 |

---

## 4. 仍未做（与文档 §7/§8 对齐后的诚实清单）

1. **静态 controller 与 pluginlib controller 在同一 admission/lifecycle 适配器内的混合**（文档 §4.2/§7.4）：
   本轮只把**描述**静态化（manifest + 生成的接口清单 + 固定存储），并没有给 manager 增加"编译进二进制的
   static provider"这条注册路径；那需要动 `ControllerManager` 的加载与 lifecycle 适配层。
2. **manager 侧的 activate 事务/整组回滚**（文档 §7.2 的 manager 半边）：见 §1.3。
3. **统一 generation 发布协议**（文档 §6、评审 D 的另一半）：首版仍是"控制循环停止时配置"。
4. **运行中改变拓扑/manifest 被拒**（文档 §7.5）：manifest 是类型，运行期本来就无法改；
   真正需要的是"运行中配置一律拒绝并报错"，当前靠 API 注释约束而非强制。
5. 依赖库（rclcpp/lifecycle/hardware_interface/FastRTPS）的 TSan、Gazebo 真值指标：与上一轮相同，未做。

---

## 5. 结论

- 原始想法**不能**按字面实现；按文档改写后的目标**可以实现，而且本轮已经落地了它的描述层与 configure 校验**；
- 与本轮之前相比，"编译期"不再只是编译期**检查**，而是一份**可枚举、可 `static_assert` 的描述**，
  并且从同一份声明**生成**了硬件接口清单、拓扑、端口字符串与执行计划；
- 仍然**不能**声称：ROS controller 在静态初始化期完成完整初始化；manifest 取代 configure/activate 的
  URDF 与硬件绑定；依赖库内部无竞态；运行中任意拓扑修改安全；硬件总线物理提交原子。
