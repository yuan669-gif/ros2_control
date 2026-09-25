# 对 `REVIEW_REQUIREMENTS_2026-09-24.md` 的处理

日期：2026-09-24
被审版本：`b9db646f`（评审）→ 本文件对应 `5f5ffed` 之后的工作
范围：只处理评审的 A–E 五项代码要求，不扩展论文任务

状态图例：**已修**（有代码改动 + 本轮验证）｜**部分**｜**未做**（有明确计划）

---

## 0. 结论速览

| # | 评审项 | 状态 | 证据 |
|---|---|---|---|
| **A** | 静态父关系与实际生成关系可能不同（`compose` 未校验 `parent_type`，评审已复现） | **已修** | `compose` 现在 `static_assert` 子 binding 的 `parent_type` 必须等于当前节点；两个编译反例（`compile_fail_child_declares_other_parent.cpp`、`compile_fail_root_as_child.cpp`）以 `TOPOLOGY MISMATCH` 被拒；编译语料 10/10 |
| **C** | 端口检查不是建组必经步骤；`verify_ports_match_contract` 只比长度（注释却写名称+顺序） | **已修** | `create_library_group` 现在**自动**跑 `verify_binding_ports`（失败抛 `invalid_argument`），新增 `create_library_group_unchecked` 供故意错配的调试 stub；`verify_ports_match_contract` 改为**逐位置比名称与顺序**（新增 `tc::port_at_t`）；新增"同长度错名"与"顺序颠倒"两个负向用例；三个既有 fixture 的错配已按契约修好 |
| **D** | 模式标志与快照发布不是一致状态；`two_phase_entries()` 返回可能悬空的引用 | **已修** | `two_phase_enabled_` 改为 `std::atomic<bool>`；`two_phase_entries()` 改为**按值返回 `shared_ptr`**（原实现把引用指向只有局部 owner 的快照，是真悬空）；启用改为"**先发布成员、再置位**"，禁用改为"**先清标志、再清成员**"；`set_staged_execution_group` 改为**发布前**刷新缓存标志（发布后不再写该对象）。**未做**：模式/成员/计划的统一 generation；**仍未做真实 manager 的 TSan** |
| **B** | 静态构建器只支持链，不支持分叉树；`ForChildren` 不能按每个孩子分别比较 | **已修**（见 §B 与 §B.1） | `BoundNode` 改为变参 `Children...`；`compose` 变参、`make_leaf` 单节点；`ForChildren`/`ChildState` 按**孩子顺序拼接**并与每个孩子声明**逐段比较**（`children_references_agree` / `children_states_agree`）；`compose` 在双方都有 `typed_ports` 时**自动**执行检查；七节点验收树 `test_typed_tree`（含兄弟顺序对调变体）在**库路径**验证静态边、阶段顺序与本周期数值；同一棵树还在**manager 路径**验证（§E.1.5，`two_pass_runs_a_branching_tree_and_propagates_it_same_cycle`） |
| **E** | 新/不合格组件仍可能走原生路径；跨模式依赖未拒绝 | **已修**（见 §E 与 §E.1） | 跨模式参考边（两端恰有一端是两趟成员）**拒绝启用**；两趟启用期间的晚加载/晚配置不合格成员使 `configure_controller` 返回 ERROR（不再只记日志）；`switch_controller` 对**激活态**成员做同样判定；新增 `two_phase_rejected_controllers()` 暴露"被拒成员 + 原因"；**另发现并修复**"manager 列表顺序把一条边反向"这一类（§E.1.5），共六个行为用例 |

---

## A 静态父关系与实际生成关系可能不同——**已修**

评审的反例成立且重要：拓扑有**两个来源**——`BoundNode::next` 的结构嵌套，和每个节点
`static_topology::Node<Name, Parent>::parent_type`。`fill_spec_rows` 用**嵌套**生成 `parents` 列，
所以嵌套会**静默覆盖**类型声明：`A = Root<a>`、`B = Root<b>` 时
`compose<A>(a, make_leaf<B>(b))` 能编译，并输出 `B.parent = "a"`。

修复不是删掉其中一个来源，而是**要求二者一致**（两者都有用：嵌套携带运行实例，
`parent_type` 是 `static_topology` 环保证的陈述对象）：

```cpp
template <typename ParentNode, typename ChildBinding>
constexpr bool child_declares_this_parent()
{
  if constexpr (std::is_void_v<typename ChildBinding::node_type::parent_type>)
    return false;                       // 子节点自称 Root ⇒ 不能被嵌套
  else
    return std::is_same_v<typename ChildBinding::node_type::parent_type, ParentNode>;
}
```

`compose` 对此 `static_assert`，诊断文本以 `TOPOLOGY MISMATCH` 开头。两个反例进入编译语料：

- `compile_fail_child_declares_other_parent.cpp`：子节点声明 `other` 为父，却被嵌套在 `chassis` 下；
- `compile_fail_root_as_child.cpp`：把声明的 `Root` 当子节点用（评审给的原始反例）。

`make_leaf` 的语义也在注释里写明：它表示"没有子 binding"，**不是**"是根"——
链的最后一个节点通常是 `Descendant`，其 `parent_type` 是真实父节点，由 `compose` 校验。

验收对照：错误父类型 ✅ compile-fail；Root 作子节点 ✅ compile-fail；
正确类型声明生成的实际边与类型边一致 ✅（结构嵌套已被强制与 `parent_type` 相等，二者不可能再分叉）。

---

## C 端口检查不是建组必经步骤——**已修**

三件事：

1. **建组必经**。`create_library_group(binding)` 现在按顺序执行：
   编译期所有权/父关系检查 → **运行期 `verify_binding_ports`** → 内核计划校验；
   运行期检查失败抛 `std::invalid_argument`。新增 `create_library_group_unchecked(binding)`
   供**故意错配**的调试 stub 使用，并要求调用点写明理由。
2. **按名称与顺序比较**。`verify_ports_match_contract` 原实现只比 `.size()`，而注释写的是
   "must equal the contract's PRODUCED names, in order"——**注释与实现不符**。现在通过新增的
   `tc::port_at_t<List, i>` 逐位置比较名称与顺序（`same_names` 增加 `PortList` 重载）。
   新增两个负向用例：**同长度错名**、**顺序颠倒**（两者都能通过旧的长度比较）。
   `verify_binding_ports` 的报错会指出**节点名**与**不一致的那张表**。
3. **fixture 不再靠错配过关**。三个既有 fixture 的运行期端口与契约本来就不一致
   （例如 `StubController` 报一个状态端口 "value"，而契约声明 0 个 produced 端口；
   R5 的偏移用例把自身状态端口写在 consumed 位置上），旧代码因为"检查不是必经步骤"才通过。
   现在按契约把它们的端口声明改直：每节点声明**自己的**状态（produced）与**自己的** reference
   （consumed），并补上内核要求的 reference source；`mi_contract` 的端口移到 `produced`。
   这是评审说的"调试 stub 不能成为默认关闭检查的理由"。

验收对照：同名异量纲 ✅（编译期，`test_typed_ports` 的 `state_declarations_agree` 负向用例）；
同长度错名/错序 ✅（运行期，本轮新增两个用例）；不存在状态端口 ✅（绑定到契约未声明的端口即错名/错长度）；
错误 Contract ✅（`create_library_group` 抛异常，`_unchecked` 才能建）。

**仍未闭环**：静态检查只对**链**精确（见 B）；执行器端口仍无法与 `Contract` 对齐
（`Contract` 有意不含硬件执行器端口）。

---

## D 模式与快照发布仍非一个一致状态——**已修（第一部分）**

评审指出的三处：

1. **`two_phase_enabled_` 是普通 `bool`** —— 非实时线程写、`update()` 多次读，是数据竞争。
   改为 `std::atomic<bool>`；读取处用 `load(relaxed)`（布尔本身不保护其他数据），
   写入处用 `store(release)` 与快照发布配对。
2. **`two_phase_entries()` 返回引用可能悬空** —— 这是**真 bug，我上一轮引入的**：
   它 `atomic_load` 到一个局部 `shared_ptr`，然后返回指向其 pointee 的引用；
   函数返回后那个 `shared_ptr` 析构，若此时另一线程已发布新快照，引用就指向已释放的 vector。
   现在**按值返回 `shared_ptr<const std::vector<TwoPhaseEntry>>`**，调用方共享所有权。
3. **发布顺序** —— 原实现"先置 `enabled_`，再重建 entries"，存在一个窗口：
   控制周期看到 `enabled == true` 而 entries 为空/陈旧，于是原生循环的跳过判断（绑在标志上）
   会把成员交给融合单趟路径。现在**启用 = 先发布成员、再置位**，**禁用 = 先清标志、再清成员**。
   为在标志仍为 false 时也能按准入规则发布，`rebuild_two_phase_entries` 增加
   `admission_enabled` 重载。

**另外修掉一处评审点出但未复现的竞态**：`set_staged_execution_group` 原来是
"发布后再 `refresh_member_active_state()`"，而 `update()` 可能已经加载到刚发布的组并调用
`run()`，二者都会写 `members_active_`。现在改成**发布前**刷新——发布之后该对象不再被写。

**仍未做（评审明确要求区分）**：

- 模式、成员、计划**统一 generation** 的发布协议（本轮只是把已有的两块各自做成原子并固定顺序）；
- **真实 manager 的 TSan**：现有 `tsan_publish_protocol.cpp` 只覆盖"容器原子发布"这一模式，
  不含 `ControllerManager` 的字段与切换协议（评审说得对）。给全包加 TSan 需要另开构建树，
  本机磁盘不允许（见下方"环境限制"）；
- "运行中启停模式"的完整支持。评审给了两条路：完整发布协议，**或**首版只允许停止期配置。
  本轮**采用后者并将在文档中写明约束**（尚未写完，见 §E）。

---

## B 分叉树 typed builder——**未做**（计划，已在 §B.1 完成）

评审的要求与验收都明确，本轮未实现。要点：

1. `BoundNode` 目前 `Next` 只有一个 → 改成形参包 `Children...`（`next_storage` 改为 `tuple` 存储），
   `compose` 变参，`binding_depth` 求和，`fill_spec_rows` 对每个孩子递归；
   单孩子的既有调用 `compose<A, Contract>(inst, child)` **语法不变**，所以是可增量改造。
2. `ForChildren` / `ChildState` 目前是**每个父节点一张扁平表**，而
   `declarations_are_compatible<Parent, Child>` 是**两节点**比较，因此无法按孩子分别比较——
   这正是评审第 55 行的意见。计划：保持扁平声明（按孩子顺序拼接），检查时**按每个孩子声明的端口数
   逐段前缀匹配**，即 `check_children<ParentPorts, Child1, Child2, ...>` 递归消费该扁平表；
   这样既保留声明简洁，又做到按孩子路由。
3. 新增七节点验收树 `chassis -> {left_module, right_module} -> {2 motors each}`，
   在同一条用例里同时证明：静态声明 → 实际边 → 状态/命令顺序 → **本周期数值**；
   并**反向打乱**注册顺序与兄弟声明顺序，按命名端口核对结果（而不是靠位置巧合）。
4. 反向用户入口的负向编译测试（错误父类型、Root 作子节点、同长度错名/错序、错误 Contract）。

**为什么本轮没做完**：这三步是连锁的（变参绑定 → 每子路由 → 七节点用例），
其中第 2 步会让 `test_typed_ports`、`test_topology_binding` 与编译语料的端口声明写法全部要改。
本轮选择先把 A/C/D 这三项**独立且已验证**的修正落地并提交，避免把半成品混进绑定层。
下一轮从变参 `compose` 开始。

---

## E 新/不合格组件仍可能走原生路径——**未做**（计划）

评审指出的两条具体风险：

1. **晚加载的成员**：`set_two_phase_execution(true)` 对**当时已有**的集合做准入，
   之后加载/配置的组件若不合格，`rebuild_two_phase_entries` 只记 ERROR 并**排除**，
   于是它走原生路径——配置看起来成功、实际不工作。计划：准入失败应使该**组**的配置/激活失败
   （而不是静默降级），至少要让调用方可查（暴露"被排除的成员名 + 原因"，
   并把"启用后新加载成员不合格"变成可断言的行为）。
2. **跨模式依赖**：两趟的 pass 2 在原生循环**之后**执行，所以"两趟父 → legacy 子"这条参考边
   会退化成上一周期（父先于子不再成立）。计划：首版**拒绝**横跨
   legacy / staged / two-phase 边界的依赖，即在准入时检查每个两趟成员认领的接口是否属于
   另一个非两趟控制器。
3. 评审建议的三个行为用例：**模式先开后加低频插件**、**组内成员停用后父节点继续运行**、
   **晚加入成员的首周期**。

---

## 环境限制（与上一轮相同，须记录）

- 容器根文件系统是沙箱 `ro-bind`，只有工作区可写；工作区可用空间约 **170 MB**。
  给 `controller_manager` 加 TSan 需要另开构建树（GB 级），**做不了**；
- `test_controllers_chaining_with_controller_manager` 的精确 `internal_counter` 断言是
  **墙钟校准**的（实测空闲 0/4 全绿、加 2 个 CPU 忙循环 4/4 全绿），
  见 `CODE_AUDIT_SCHEDULING_METAPROGRAMMING.md` §3.1。不要把它的失败当作调度回归。

---

## 本轮验证

| 项 | 结果 |
|---|---|
| `hierarchical_control` 10 个测试程序 | 全部通过（`test_typed_ports` 8、`test_topology_binding` 6、`test_contract_regression` 12、其余同前） |
| 编译语料 | **10/10**（新增 2 个 `TOPOLOGY MISMATCH` 反例；7 拒 + 1 非空洞对照 + 2 新增） |
| `controller_manager` | `test_two_phase_execution` 12/12 及其余同前 |
| TSan harness | racy 必报 / atomic 干净 |
| 阶段图穷举 | 465/465 |

**未跑**：真实 manager 的 TSan、Gazebo、性能测量（与本轮改动无关）。

> 上表是**上一轮**（A/C/D）的记录。B/E 完成后的数字见本文末尾「B/E 完成后的验证」。

---

## B.1 分叉树 typed builder——**已完成**（本轮）

§B 的四点全部落地，且与评审建议的第 1、2 条一致：

1. **变参绑定**：`BoundNode<ControllerT, Node, ContractT, Children...>` 用
   `std::tuple<Children...> children` 存储；`compose<Node, Contract>(instance, children...)` 变参，
   `make_leaf<Node, Contract>(instance)` 表示无孩子。单孩子调用
   `compose<Node, Contract>(inst, child)` **语法与语义不变**，因此既有的链式声明原样编译。
   `binding_depth` 改为 `subtree_size`，`fill_spec_rows` 对子树递归，
   `build_spec_rows` 产出的是**整棵树**（前序：父先于其子树；`parents` 列来自实际嵌套，
   并且现在由 `child_declares_this_parent` 强制等于类型声明的父）。
2. **按孩子路由的端口检查**：父节点的 `ForChildren` / `ChildState` 保持**一张扁平声明表**，
   但要求它等于"各孩子的声明**按孩子顺序拼接**"：
   `children_references_agree<ParentPorts, ChildPorts...>` 与
   `children_states_agree<ParentPorts, ChildPorts...>` 做**逐段、按名字和量纲**的前缀匹配。
   `typed_ports.hpp` 里的 `reference_declarations_agree` / `state_declarations_agree` /
   `declarations_are_compatible` 变成这两个变参检查的薄包装。
   `compose` 在**所有参与者都暴露 `typed_ports`** 时**自动**执行该检查，所以"建树即检查"。
   `create_library_group` 额外递归验证每个成员的**运行时端口字符串**（C 项）。
3. **七节点验收树**：`hierarchical_control/test/test_typed_tree.cpp`，
   `chassis -> {left_module, right_module} -> {a, b} each`。用例断言：
   * 计划行 = 7 个节点、父子列逐项正确；
   * 状态阶段每个孩子先于其父、命令阶段每个父先于其孩子、且**所有状态阶段早于所有命令阶段**；
   * 控制器每个阶段**恰好一次**；
   * **本周期数值按端口名**成立：命令阶段读到的自身状态 = `sum(子状态) + bias`，
     叶 = 1/2/3/4、左模块 = 13、右模块 = 27、底盘 = 140，命令值 = 根的外部参考 42，
     并核对了 4 个叶的执行器提交值；
   * 第二个用例把**每个兄弟对调**（并同步对调父侧声明表），重新断言同一组**按名字**的数值——
     位置变了、名字和数值没变，这正是评审要求的"不靠位置巧合"。
4. **负向编译用例（走真实 `compose` 入口）**：`static_topology_negative/`
   新增 `must_compile_typed_tree.cpp`（对照，必须编译）与三个必须失败的用例：
   `compile_fail_typed_reference_edge_mismatch.cpp`（父少声明一个子参考）、
   `compile_fail_typed_state_edge_order.cpp`（两个子状态顺序颠倒 —— 同名同长度，仅顺序错）、
   `compile_fail_typed_wrong_dimension.cpp`（名字顺序都对、量纲错）。
   三者分别命中 `REFERENCE EDGE MISMATCH` / `STATE EDGE MISMATCH` / `REFERENCE EDGE MISMATCH`；
   再加 `compile_fail_typed_wrong_name.cpp`（**同长度、同 owner、不同端口**，
   即旧的长度检查会放过的那一类）命中 `STATE EDGE MISMATCH`。
   语料从 10 条增至 **15 条**（13 条必须失败 + 2 条必须编译的对照项），结果 15/15 符合预期。

**顺带修正的一处不一致**：`compose` 的模板参数是
`topology_contract::Contract`，而 typed 声明给出的是 `TypedPorts`；
写绑定需要 `tp::contract_of_t<Ports>`。这在验收树里第一次使用时暴露，
已按 `PORT_DIMENSIONS.md` 既有约定书写（端口仍只声明一次）。

---

## E.1 新/不合格组件仍可能走原生路径——**已完成**（本轮）

### E.1.1 跨模式依赖被拒绝

判定规则：一条参考边（claimant 写 `<owner>/<port>`，owner 是被写端）只要**两端恰好有一端**
实现 `TwoPhaseControllerInterface`，就是跨模式边。理由写在拒绝信息里：
pass 2 在原生循环**之后**执行，所以这条边的两端由**不同调度**排序，
"父先于子"不再成立，会静默退化成上一周期。

实现：
- `two_phase_rejections(controllers, only_active)` 同时检查**两个方向**
  （两趟父 → legacy 子、legacy 父 → 两趟子），因为两个方向都会坏；
- 该函数是**唯一**的准入判定来源：启用、重建成员表、`configure_controller`、`switch_controller`
  都调用它，所以"日志里说的"和"实际排除的"不可能不一致；
- 启用（`set_two_phase_execution(true)`）时只要有**任何**拒绝就整体返回 ERROR 且不置位标志；
- `switch_controller` 只对**激活态**成员判定（两趟 pass 本来会跳过非激活成员），
  并且只在 `two_phase_enabled_` 为真时判定，默认路径零开销；
- 测试 `two_phase_enable_is_refused_for_a_cross_mode_reference_edge`：legacy 子导出 `target`，
  两趟父认领它 ⇒ 启用被拒、标志保持 false、`two_phase_rejected_controllers()` 第一项是父、
  原因里出现子名。

### E.1.2 晚加载/晚配置的不合格成员使配置失败

`configure_controller` 在**发布新列表之前**（`switch_updated_list` 之前）对**候选列表**做准入；
不合格则返回 ERROR 并逐条打印，**不再**"记 ERROR 然后排除、继续 native"。
`switch_controller` 同样在切换应用前对激活态成员判定，不合格则
`switch_result = ERROR`。两者都只在两趟已启用时生效。

测试 `a_late_low_rate_member_fails_configure_while_two_phase_is_enabled`：
先启用两趟（链式 3 成员合格），再 `add_controller` 一个
`update_rate = manager/2` 的成员并 configure ⇒ **ERROR**，且
`two_phase_rejected_controllers()` 精确给出 `tp_late` 与"update rate"原因，模式与合格链不受影响。

### E.1.3 被排除成员可查

新增公开接口
`std::vector<TwoPhaseRejection> two_phase_rejected_controllers() const`，
`TwoPhaseRejection{name, reason}`：**按值返回**（成员表由非实时线程重建，返回引用会与重建竞争），
由调用者在非实时线程读取。

### E.1.4 三个行为用例——其中一个被上游规则**证明不可达**

| 评审建议的用例 | 结论 |
|---|---|
| 模式先开后加低频插件 | **已覆盖**：`a_late_low_rate_member_fails_configure_while_two_phase_is_enabled` |
| 晚加入成员首周期 | **已被更强结论取代**：晚加入且不合格时**根本不能完成 configure**，不存在"首周期走哪条路径"的问题；合格成员在 `a_member_is_never_run_by_the_native_loop_during_a_switch` 已覆盖 |
| 组内成员停用后父节点继续运行 | **管理器 API 层不可达**：上游 `check_preceeding_controllers_for_deactivate()` 拒绝对"其参考接口仍被激活控制器认领"的链式子节点执行停用。已用 `a_chained_child_cannot_be_deactivated_while_its_parent_runs` 固定这一事实（断言 switch 返回 ERROR 且子节点继续运行）。内核一侧真正要保证的是"遵守 lifecycle 标志"，用两个**无参考边**的成员写成 `a_deactivated_member_stops_running_while_other_members_continue`：停用者计数冻结、其余成员继续运行 |

也就是说：评审担心的"父子混合阶段导致一条边再次用旧状态"，在本实现里**从准入层就被拒绝**；
余下的"成员被停用"只能是**无父子边**的成员，内核按 lifecycle 标志跳过它。

### E.1.5 新发现并修复：管理器列表顺序可能把一条参考边**反向**

**怎么发现的**：写"分叉树走 manager 路径"的验收用例时（`two_pass_runs_a_branching_tree_and_propagates_it_same_cycle`），
七个节点每阶段都恰好调用一次、计划也正确，但模块与根的估计**全是 0**。原因是
上游 `controller_sorting()` 的规则：**"没有 command interface 的 chainable 控制器排在有 command interface 的前面"**。
四个叶节点当时不认领任何 command interface，于是列表顺序变成**子 → 父**；
而两趟 pass 不自己排序，pass 1 反向遍历、pass 2 正向遍历，两条都对该边**同向走错**，
父节点读到的是上一周期的子状态。这与"每周期一次"的计数断言**完全兼容**——
计数正确、数值错误，正是最难靠计数发现的那一类。

**修复**：把"列表顺序对每条边必须是父在前"变成**准入条件**。
`two_phase_rejections()` 在跨模式检查之后新增顺序校验：对每条两端都是成员的参考边，
要求 claimant（父）在列表中的下标 **小于** owner（子）；否则两端都记为
`unschedulable_order` 拒绝，启用/配置/激活整体失败，拒绝信息点名父、子以及上游排序的原因。
`staged` 组本身不受影响（`StagedExecutionGroup` 的顺序由声明的边推导，不依赖 manager 列表），
所以这一条是**manager 路径特有的**风险。

**验证**：
- 分叉树用例给四个叶节点各自一个真实硬件 command interface（`joint2/velocity`、
  `joint3/velocity`、`joint1/position`、`joint1/max_velocity`），顺序恢复正确，用例断言
  **精确的同周期数值**：叶 0.5、两个模块 0.25、根 0.125；命令值沿树下行
  根 −0.125、模块 −0.375、叶 −0.875（`command = reference − estimate`，
  每层的 reference 就是父节点本周期写入的值）。这些数值**只可能**在
  "pass 1 子先于父、pass 2 父先于子"时同时成立。
- `two_phase_enable_is_refused_when_the_manager_order_inverts_an_edge` 固定反向用例：
  一个**认领 0 个 command interface** 的 chainable 子节点 + 认领它 reference 的父节点 ⇒
  启用被拒、标志不变、拒绝原因含 "ordered AFTER its child"。

**顺带**：`TestStagedController` 增加 `set_two_phase_children(std::vector<...>)`
（`update_phase` 里取子估计的**均值**），使分叉树能在 manager 路径上真的跑起来；
原有的 `set_two_phase_child` 单子接口与语义不变，既有用例未改动。

---

## B/E 完成后的验证（本轮实测）

| 项 | 结果 |
|---|---|
| `hierarchical_control` | **12 个 ctest 程序全通过**：11 个 gtest 程序 / **75 用例** + 编译语料脚本 |
| `test_typed_tree`（新） | 2 用例：七节点分叉树（库路径）+ 兄弟顺序对调变体 |
| 编译语料 | **15/15**（13 必须失败 + 2 必须编译的对照） |
| `controller_manager` | **20/20** ctest 程序（含 `test_two_phase_execution` **18 用例**，其中 7 个是本轮新增）；19 个 gtest 程序 / **162 用例** |
| TSan harness | `[tsan] RESULT: PASS (racy reported, atomic clean)` |
| 阶段图穷举 | 465/465；一个状态阶段/顶点的调度数 **0** |
| 内存/磁盘 | 未新增构建树；未做真实 manager 的 TSan（磁盘不允许） |

### 两个**环境性**失败（不是本轮改动的回归，须如实记录）

1. `test_controller_manager_srvs`：ctest 的 `TIMEOUT 120` 不够——直接运行该二进制
   **14/14 通过，用时 234 s**。原因是本机 load average ≈ 5.3 而 `nproc = 2`（宿主超载），
   与调度实现无关。
2. `test_spawner_unspawner`：`TestLoadController.spawner_test_failed_activation_of_controllers`
   **间歇**失败，失败信息是 spawner 进程
   `Could not contact service /test_controller_manager/list_controllers`
   （服务发现超时），断言随之看到 `get_loaded_controllers().size() == 2` 而非 3。
   3 次单独运行 1 通过 2 失败；整个 ctest 首次全量运行时该用例是通过的。
   本轮改动只在 `two_phase_enabled_ == true` 时执行任何新代码（默认 false，该测试从不启用），
   失败模式发生在 spawner 与 manager 的服务发现阶段，早于任何准入逻辑。
3. `test_hierarchy_comparison.post_switch_two_phase_cycles_do_not_rebuild_membership`
   在同一时段出现过 **1 次**失败，随后连续 **8 次**运行全绿。该用例断言控制环线程
   "空闲每周期分配数完全平坦"，在宿主超载时容易被一次额外分配打破；缺陷检测目标
   （两趟成员表在实时路径重建）不受影响。

---

