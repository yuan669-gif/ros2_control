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
| **D** | 模式标志与快照发布不是一致状态；`two_phase_entries()` 返回可能悬空的引用 | **已修** | `two_phase_enabled_` 改为 `std::atomic<bool>`；`two_phase_entries()` 改为**按值返回 `shared_ptr`**（原实现把引用指向只有局部 owner 的快照，是真悬空）；启用改为"**先发布成员、再置位**"，禁用改为"**先清标志、再清成员**"；`set_staged_execution_group` 改为**发布前**刷新缓存标志（发布后不再写该对象）。**模式/成员/计划已合并为一个 generation（§D.2）**；**激活回滚见 §D.3（默认关闭）**。**真实 manager 的 TSan 已补**（见 §D.1：插桩 `controller_manager` 后测出 4 条上游握手字段的数据竞争，改成原子后归零；依赖库未插桩） |
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

### E.1.6 继续查漏补缺时修掉的三个问题（本轮第二轮）

1. **同一个控制器实例被绑成两个节点**（真实缺陷，之前无人拦）：
   `rows_are_well_formed` 只查**名字**唯一，内核 `StagedExecutionGroup` 的 spec 校验也只查名字；
   于是把**同一个对象指针**用于两个节点名（例如复制粘贴 `make_leaf` 时忘了换指针）会**通过全部检查**，
   而内核每个阶段会**调用该对象两次**、两个节点的状态槽争抢同一个对象的状态——
   这正是"每控制器每周期每阶段至多一次"这条核心保证的反例。
   修复：`topology_contract::rows_are_well_formed` **和** `StagedExecutionGroup` 构造函数
   **都**检查实例唯一性（前者给计划层好的报错，后者守住内核自己的不变式——手写 `Spec` 的用户
   不经过绑定层）。三处测试：`test_topology_contract`（错误原因含 "same controller instance"）、
   `test_execution_group`（`create_library` 抛 `invalid_argument`）、
   `test_topology_binding`（`to_library_spec` 报出具体节点名）。
2. **自查自反例**：`two_phase_rejections` 的顺序校验没有排除"控制器认领**自己**的端口"
   （`<自己的名字>/<port>`），会把这种（罕见但合法）配置判成"排在它的子节点之后"。
   已加 `child == parent` 跳过。这是写这条检查时引入的假阳性，不是既有缺陷。
3. **诊断信息不够用**：`verify_ports_match_contract` / `verify_ports_match_interface`
   原来只返回 "disagrees" 的静态字符串，且 `reason` 是 `const char **`。
   已改为 `std::string *` 并输出**实测列表 vs 声明列表**，
   例如 `staged_state_ports() is [neg_b/state_typo] but the contract it was bound with declares
   [neg_b/state]`；`verify_binding_ports` 再补上节点名。
   新增用例 `TypedTree.the_checked_build_verifies_every_child_not_only_the_first`：
   把"手写错误端口"的控制器放在**第二个孩子**位置，断言报错**点名第二个孩子**且含错误端口名——
   这条同时证明了运行时端口校验确实**递归到每一个孩子**，而不只是第一个。
4. **同一个对象被 `add_controller` 两次登记**（**实测确认的真实缺陷，属两趟路径**）：
   上游 `add_controller()` 只查**名字**重复，因此
   `add_controller(shared, "a"); add_controller(shared, "b")` 会成功，两个 spec 携带**同一个指针**
   （探针实测：`ptr=0x...240` 两次相同，configure 两次都返回 OK）。
   此时 `set_two_phase_execution(true)` 返回 **OK 且 0 条拒绝**，随后两趟 pass
   对**同一个对象**每个阶段调用两次：**3 个周期内 `update_phase_calls=6`、`handle_phase_calls=6`**
   ——而"每个名字各一次"的计数完全正常，属于最难发现的一类。
   注意：staged/库路径不受影响（内核的实例唯一性检查已经拦住），**两趟路径不经过内核**，
   所以必须在准入层拦住。修复：新增 `TwoPhaseAdmission::duplicate_instance`，
   检出"两个名字一个对象"时**两个名字都记为拒绝**（这样即使调用方选择只排除一个，
   也不会留下这条边的一半），启用/配置/激活整体失败。
   用例 `two_phase_enable_is_refused_when_one_object_has_two_names`：
   断言第二次 `add_controller` 成功（记录上游事实）、启用返回 ERROR、两条拒绝原因含
   "shares ONE controller object"。

### E.1.7 开关失败后仍发布新控制器列表——**已修**（用户指出）

**问题**：`switch_controller()` 先让实时循环执行切换（生命周期状态已改），再刷新 `claimed_interfaces`
并发布新列表。因此任何"切换后发现不合格"的判定都是**事后报告**：调用方看到 ERROR，
控制器却已经激活、新列表也已经发布——正是"看起来失败、实际生效"。

**修复**：把两趟的调度判定**前移**到请求下发之前。

1. `two_phase_rejections()` 的 `bool only_active` 改为 `const std::vector<char> * active_mask`：
   `nullptr` 表示判定整个已配置集合（启用/配置路径的语义不变），
   掩码表示"只判定这些控制器"。新增 `controller_active_mask()`（按当前激活态生成掩码）。
2. `switch_controller()` 在**请求列表定稿之后、`do_switch` 之前**用**预期激活集**
   （当前激活态 + `activate_request_` − `deactivate_request_`，同一切换里既停又启的按激活算）
   调用一次 `two_phase_rejections()`；有拒绝就 `clear_requests()` + ERROR：
   **没有生命周期切换、没有发布新列表、没有重建成员表**。
3. 切换后的判定保留为**第二道线**（覆盖"声明只在激活后才可见"的控制器），
   仍按**实际**激活态掩码判定；此时若失败，语义与上游既有的"未能激活"错误一致（已应用、报错）。

**能证明前移生效的用例**：`reactivating_a_legacy_claimant_is_refused_before_it_is_applied`。
构造（先测后写）：
- legacy 控制器（`ChainableControllerInterface`，**不**实现两趟接口）先激活，此时只持有它激活时
  认领的接口；随后一个两趟成员被配置并导入新的 reference 接口 —— 此刻**没有边**（legacy 不可能
  持有后导入的 loan），因此两趟启用被正确接受；
- 把 legacy 的声明改成包含那个 reference 接口，然后**同时请求激活 legacy 与成员**。
  这个请求是**上游本来会接受**的（链式校验要求邻居一起激活，所以两者都放进请求；
  运行日志里只有下面这一条错误，没有上游的校验告警）：
  `Refusing the switch before it is applied: controller 'tp_leaf' takes part in a reference edge
  that crosses the two-phase/legacy boundary with 'tp_root' ...`
- 断言：返回 ERROR、**legacy 与成员都仍未激活**（即"什么都没做"）。
  若只有事后判定，两者会变成 ACTIVE 并返回 ERROR——这正是本条要修的旧行为。

**顺带纠正一处设计错误（自己先改错再改回）**：为了让"派生的 ALL 集合变新"，我一度把
`claimed_command_interfaces()` 改成**永远读声明**。分析后发现那是**假阳性**：
激活态的控制器**不可能**持有"激活之后才导入"的接口的 loan，读声明会凭空造出一条边。
最终语义按生命周期区分（已写入注释）：
**ACTIVE → `claimed_interfaces` 快照（它真正持有的写接口），配置但 INACTIVE → 声明（将要认领的），
未配置 → 空**。前移的那次判定用的是"将要激活"的控制器声明 + 仍在激活的控制器快照，两者都对。

**测量到的测试环境事实（不是上游 bug，也不是本特性问题）**：
`configure_controller()` / `add_controller()` / `unload_controller()` 会替换控制器列表，而
`RTControllerListWrapper::switch_updated_list()` 在 `wait_until_rt_not_using()` 里等待**实时循环**
用完被替换的列表。真实系统里 `update()` 一直在跑，微秒级就满足；**手工泵 `update()` 的测试**
必须在这类调用期间继续泵，否则主线程会停在 `hrtimer_nanosleep`（实测：20 s 后用
`/proc/<pid>/task/*/wchan` 看到 `hrtimer_nanosleep`）。已加 `ConfigureWithPump()` 辅助函数并写明原因。

### B.2 把 typed 树放进普通 composite 插件（评审"推荐下一步"第 5 条后半句）——**已完成**

`controller_manager/test/test_composite_library/typed_fork_composite_controller.{hpp,cpp}`
是一个普通 composite 插件，内部**只声明一次**类型级拓扑：

```
fork_root -> { fork_a, fork_b }      // 端口、量纲、父子边都在这一处
```

与既有的 `GenericCompositeController` 的差别**只在拓扑来源**：那个宿主是数据驱动的
（运行期 `CompositeNodeSpec` 列表、父子用字符串、端口字符串手写、内核由 `Spec` 构建）；
这个宿主的成员端口字符串由 `TypedPortsMixin` **从类型生成**，
内核由**检查版入口** `topology_binding::create_library_group(binding)` 构建
（编译期父关系/量纲 + 整棵树的运行期端口校验 + 计划 + 执行组），
算法、接口映射、内核完全是同一套，所以两者可以在同样输入上逐周期对比。

用例 `typed_declaration_hosted_by_a_composite_plugin_matches_the_spec_host`（`test_hierarchy_comparison`）：

- 计划确实来自声明：3 个节点，`typed_root, typed_a, typed_b`（父在前）；
- **激活后第一个周期分配数 = 0**（内核整体在 `on_activate()` 里建好，控制路径不分配）；
- 6 个周期里两条叶命令**精确等于**闭式期望 `ForkExpectedA/B`，**并逐周期等于数据驱动宿主**的输出；
- 三个节点每周期各一次状态阶段、一次命令阶段，两个叶每周期各一次提交。

**一个必须记录的 API 约束（写插件时实测撞到）**：`topology_binding` 存的是**带类型的
`ControllerInterfaceBase *`**（评审 R5：换成 `void *` 会在第二个基类的地址调整上出错），
因此**要被绑定的内部节点必须派生自 `ControllerInterfaceBase`**——只实现
`StagedControllerInterface` 的节点无法通过 `compose` 绑定。插件里的 `NodeBase` 就是为满足这条而写的
最小实现（节点从不被管理器 init/configure/activate）。绕过绑定的运行期 `Spec` 路径
（`StagedExecutionGroup::create_library`）没有这个要求，这正是数据驱动宿主能只用
`StagedControllerInterface` 的原因。**若将来要走"纯 `StagedControllerInterface` 也能绑定"，
需要把绑定的实例指针类型参数化**，那会触及 R5 的不变式，本轮没做。

### D.1 真实 `ControllerManager` 的 TSan——**已做**（本轮，磁盘清理之后）

评审 D 的第二半是"现有 TSan harness 只模拟容器原子发布，不包含这些字段和实际 manager 切换协议"。
本轮把它补上了：单独一个构建树只给 **`controller_manager` 这个包**插桩
（`-fsanitize=thread`），依赖库保持正常构建，然后跑既有的 `test_two_phase_execution`
——它的模式正好是"一个线程 `update()`、另一个线程 `switch_controller()` /
`set_two_phase_execution()` / 配置",也就是要审的那条路径。

复现：`bash controller_manager/test/run_tsan_real_manager.sh [--rebuild]`
（PASS 判据 = **data race 数为 0**；脚本头写明插桩范围与局限）。

**结果（先测后改）**：

| | data race | 其中属于本仓库代码 | lock-order inversion |
|---|---|---|---|
| 修改前 | **4** | 4（全部是 manager 自己的握手字段） | 991 |
| 修改后 | **0** | 0 | ~800–2000（随运行波动） |

4 条 race 全部落在**上游的发布/握手字段**上，而没有一条落在我加的两趟协议上：

| # | 冲突 | 字段 |
|---|---|---|
| 1 | `update()` 读 vs `switch_controller()` 写 | `switch_params_.do_switch`（普通 `bool`） |
| 2 | `manage_switch()` 读 vs `switch_controller()` 写 | `switch_params_.activate_asap` |
| 3 | `wait_until_rt_not_using()` 读 vs `update_and_get_used_by_rt_list()` 写 | `used_by_realtime_controllers_index_` |
| 4 | `update_and_get_used_by_rt_list()` 读 vs `switch_updated_list()` 写 | `updated_controllers_index_` |

**修复**：把这些握手字段改成 `std::atomic`，并给发布/观察那一对加 release/acquire
（发布列表索引前写入的列表内容必须对实时线程可见）：
`switch_params_.do_switch/started/strictness/activate_asap`、
`RTControllerListWrapper::updated_controllers_index_ / used_by_realtime_controllers_index_`。
语义不变（原来的 sleep 轮询握手照旧），只是把"形式上就是数据竞争"变成真的同步。
改完后 **data race 归零**，且**没有一条报告指向 `two_phase_enabled_`、`two_phase_entries_`、
staged 快照或两趟准入/内核代码**——即 D 项里属于本特性的部分在真实 manager 上得到确认。

**仍未覆盖（如实记录）**：依赖库（rclcpp、lifecycle、`hardware_interface`、FastRTPS）**没有插桩**，
因此
- 它们内部的数据竞争在这里**看不见**；
- 剩下的 ~800–2000 条 **lock-order inversion / double lock 全部来自这些未插桩库**
  （报告里常常只有 `pthread_mutex_lock` 拦截帧，或本仓库代码只作为**调用者**出现，
  例如 fixture 构造时的 `robot_description_callback → ResourceManager::load_urdf`）。
  要处理它们得给整条依赖树插桩（GB 级、数小时），本机磁盘不允许，也不属于本特性范围。

**一个使用上的注意**：TSan 会把进程拖慢约一个数量级，套件里**断言时间**的用例
（`two_pass_costs_one_extra_traversal` 量微秒级耗时；switch-pause 用例依赖异步请求落点）
在 TSan 下可能失败——脚本把它们**列出来但不计入判据**（判据只看 race 数），
观测到的失败数在 0–9 之间波动。

### B.2 编译成本：分叉树不比同规模深链贵（新增测量）
`BoundNode` 从单槽改成变参包后，专门测了"同节点数下链 vs 树"
（`hierarchical_control/test/measure_binding_cost.py`，三次运行）：
链 **130–172 ms/节点**，树 **2–25 ms/节点**。方向稳定、倍数不稳定（宿主 load≈5/2 核），
所以只声称"**深度才是成本来源，分叉不是**"，与 §2 里 `static_topology` 的结论一致。
详见 `doc/COMPILE_COST.md` §3.1。

---

## B/E 完成后的验证（本轮实测）

| 项 | 结果 |
|---|---|
| `hierarchical_control` | **12 个 ctest 程序全通过**：11 个 gtest 程序 / **76 用例** + 编译语料脚本 |
| `test_typed_tree`（新） | **3 用例**：七节点分叉树（库路径）、兄弟顺序对调变体、运行时端口校验递归到第二个孩子 |
| `test_hierarchy_comparison` | **7 用例**（含新增的 typed 声明插件宿主用例） |
| 编译语料 | **15/15**（13 必须失败 + 2 必须编译的对照） |
| `controller_manager` | **19 个 gtest 程序 / 165 用例**，逐个直接运行全部通过：`test_two_phase_execution` **20**、`test_load_controller` 39、`test_controller_manager` 18、`test_controller_manager_srvs` 14、`test_hierarchy_comparison` **7**；另加 `test_cycle_tree_contract` 与 5 个 pytest（`ctest -R` 一并跑过，6/6） |
| TSan harness | `[tsan] RESULT: PASS (racy reported, atomic clean)` |
| 阶段图穷举 | 465/465；一个状态阶段/顶点的调度数 **0** |
| 绑定层编译成本 | 链 **130–172 ms/节点** vs 同规模分叉树 **2–25 ms/节点**（3 次运行，方向稳定、倍数不稳定）→ 见 §B.2 与 `COMPILE_COST.md` §3.1 |
| 内存/磁盘 | 未新增构建树；未做真实 manager 的 TSan（磁盘不允许） |

### 三个**环境性**失败（不是本轮改动的回归，须如实记录）

宿主为 **2 核、load average ≈ 5**，因此所有基于 launch/服务发现的用例都会间歇性超时或失败：

1. `test_controller_manager_srvs`：ctest 的 `TIMEOUT 120` 不够——直接运行该二进制
   **14/14 通过，用时 234 s**。与调度实现无关。
2. `test_spawner_unspawner`：`TestLoadController` 的 spawner 用例
   （`spawner_test_failed_activation_of_controllers`、`..._with_no_ctrl_name`）
   **随宿主负载翻转**：load≈5.1 时 22/22 通过，load≈5.6 时稳定失败 2 例，
   失败信息是 spawner 子进程
   `Could not contact service /test_controller_manager/list_controllers`（服务发现超时，
   该用例用 `--controller-manager-timeout 1.0`），断言随之看到 2 个控制器而非 3 个。
   该文件与 `test_load_controller.cpp` 里 **`two_phase` 出现 0 次**，
   本轮改动只在 `two_phase_enabled_ == true` 时执行新代码，失败发生在服务发现阶段。
3. `test_hardware_spawner`：`spawner_with_later_load_of_robot_description` 在第二轮全量运行中
   失败一次，随后 **3/3 次直接运行全部通过（8/8 用例）**；该用例本身就会断言
   "服务不可达"，在超载宿主上两边都容易翻转。
4. `test_hierarchy_comparison.post_switch_two_phase_cycles_do_not_rebuild_membership`
   在同一时段出现过 **1 次**失败，随后连续 **8 次**运行全绿。该用例断言控制环线程
   "空闲每周期分配数完全平坦"，在宿主超载时容易被一次额外分配打破；缺陷检测目标
   （两趟成员表在实时路径重建）不受影响。

**共同点**：这些断言都是时间/负载敏感的，而本轮所有改动在默认配置下
（`two_phase_enabled_ == false`）**不改变任何实时路径**，在启用时也不改变 `update()` 的代码。

---

## D.2 模式/成员/计划统一 generation——**已做（本轮）**

评审 D 的另一半是"entries、controller list、staged group 独立发布，没有统一 generation"。
本轮把**本特性发布的三样东西**合并成一个不可变快照：

```cpp
struct ExecutionGeneration {
  bool two_phase_enabled;                                        // 模式
  std::shared_ptr<const std::vector<TwoPhaseEntry>> entries;     // 两趟成员
  std::shared_ptr<StagedExecutionGroup> staged_group;            // staged 计划
  std::uint64_t id;                                              // 每次发布 +1
};
```

- `update()` 每周期**只做一次 `atomic_load`**，三个决策（跑不跑 staged、跑不跑两趟、
  原生循环跳过谁）全部来自同一个快照——不存在"标志来自一个配置态、成员表来自另一个"的混合；
- 每次配置变更用**一次 `atomic_store`** 发布完整新状态：
  启用不再是"先发成员再置位"，禁用不再是"先清标志再清成员"，而是各自一次替换
  （旧代码只能**缩小**窗口，现在窗口不存在）；
- 安装/清除 staged group 时，**group 与由它推导的两趟成员表在同一次发布里**——否则会出现
  "新 group 已生效、旧成员表还没更新"的一周期窗口，让同一控制器被两条路径同时拥有；
- 被**拒绝**的请求（启用被拒、group 被拒）**不发布**任何新状态；
- 新增 `execution_generation()`（单调 id）作为可观测契约：
  "接受的变更恰好 +1，被拒的变更 +0"。三个用例覆盖：
  `execution_state_is_published_as_one_generation`（启用/禁用各 +1、再启用 +1）、
  `a_refused_enable_and_a_group_change_publish_coherently`（被拒启用 +0）、
  `a_staged_group_change_is_one_publication`（安装 +1、被拒的两趟启用 +0、清除 +1）。

**仍未合并的一项（如实）**：**控制器列表本身**。它仍是上游的 `RTControllerListWrapper`
双缓冲 + sleep 握手；把列表也纳入 generation 等于替换上游那套机制。
`ExecutionGeneration` 里预留的 `controllers_version` 注释说明了这一点——
"模式/成员/计划一个 generation，列表沿用上游发布"。

## D.3 激活阶段的整组回滚（文档 §7.2 的 manager 半边）——**已做，但默认关闭**

**上游语义**：一次 switch 的 activate 集合是**尽力而为**的——逐个 claim、逐个激活，
失败的那个被跳过，成功的**保持激活**。上游自己的
`spawner_test_failed_activation_of_controllers` 就依赖这一点（spawner 一次起一个控制器，
后一个失败**不能**把前面正在跑的停掉）。所以**把回滚做成默认行为会破坏上游语义与它的测试**。

**本轮的实现**：`set_atomic_activation(true)`（参数 `atomic_activation`，默认 false）

- `activate_controllers()`（`activate_controllers_asap()` 转发它）现在返回
  `ActivationOutcome{any_failure, activated}`，记录**本次**激活成功的控制器；
- 任一步失败（命令接口冲突/异常、状态接口失败、`on_activate` 未到 ACTIVE）时，
  若原子激活开启，则 `rollback_activated_controllers(activated)`：
  逐个 `deactivate()` + `release_interfaces()`，并打印明确的错误；
- **作用域严格限定**：只撤销**本次 switch 激活的**控制器。此前已经激活的不动
  （撤销它们的动作远大于请求本身）；configure/unload 不受影响；
- 5 个用例（`test_atomic_activation.cpp`）：
  ① 默认（关闭时）部分激活**保持**——把上游语义也钉住，避免"悄悄改了语义"；
  ② 开启后同一次请求（3 个控制器，最后一个冲突）→ 本次激活的两个都被撤销，
     此前已激活的 blocker 仍在跑；
  ③ 回滚**释放接口**：撤销后另一个需要同一端口的控制器可以成功激活；
  ④ `on_activate` 生命周期失败（非接口冲突）走同一条回滚路径；
  ⑤ 全部成功时不受影响。

## D.4 运行中配置：从"文档约束"变成**强制拒绝**（本轮）

在 D.2 之后，模式/成员/计划已经是**一个 generation**，但**控制器列表**仍是上游独立发布的双缓冲，
而"安装执行路径"的准入判定正是对着那份列表做的。因此本轮把这条规则**强制化**：

- `update()` 进入时用 RAII 计数器 `cycles_in_flight_` 标记"一个周期正在运行"，**每个 return 路径**都会清除；
- 新增 `control_loop_busy()`（公开，可诊断）；
- **安装/扩展**路径的操作在周期在飞时**返回 ERROR 并拒绝**：
  `set_two_phase_execution(true)`、`set_staged_execution_group(...)`，
  错误信息说明原因（准入判定对着独立发布的列表）并提示"停止控制循环后重试"；
- **移除**路径的操作**始终允许**：`set_two_phase_execution(false)`、`clear_staged_execution_group()`。
  理由可验证：正在飞的周期**持有自己的 generation 快照**，会用它开始时的状态跑完；
  下一个周期看到的是更小的状态，不存在半应用；
- 拒绝时**不发布任何 generation**，因此 `execution_generation()` 的 id 不变——"被拒=没变"依旧可观测。

**用例**（`test_runtime_reconfiguration.cpp`，3 个）：用一个 `update()` 会阻塞在条件变量上的控制器
把"周期在飞"变成**确定性**状态，而不是赌竞态：

| 用例 | 断言 |
|---|---|
| `installing_a_path_while_a_cycle_is_in_flight_is_refused` | 周期在飞时：两趟启用被拒、staged group 安装被拒（**成员本身合法**，所以只能是这条规则拒绝的）、id 不变、没有任何路径被安装；释放周期后**同样的调用成功**（正向对照），且各 +1 |
| `removing_a_path_is_accepted_while_a_cycle_is_in_flight` | 周期在飞时禁用两趟 + 清除 group 均接受，id +1，在飞周期正常返回 |
| `the_busy_flag_tracks_the_cycle` | 连续 3 个周期后 busy 标志为 false——RAII 在每个 return 路径都清 |

## 仍未做的事（明确列出，避免"看起来全做完了"）

| # | 未做项 | 现状 |
|---|---|---|
| 1 | **控制器列表**纳入统一 generation（因此运行中**安装**路径仍被拒绝，见 §D.4） | 模式/成员/计划已合并（§D.2）；列表仍是上游 `RTControllerListWrapper` 双缓冲，纳进去等于替换上游机制。因此"控制循环停止时配置"这条约束**仍然有效** |
| 2 | **依赖库的 TSan**（rclcpp / lifecycle / hardware_interface / FastRTPS） | `controller_manager` 自身已插桩并跑到 0 data race（§D.1）；依赖库未插桩，其 lock-order 报告无法归属，给整条依赖树插桩需要 GB 级空间与数小时 |
| 3 | 多频 / 异步 / 动态拓扑 / 生命周期回滚 | 明确不做（评审也建议不要扩） |
| 4 | `Spec::parents` 的 YAML/参数入口 | 未做；两趟与 staged 都从 claimed interfaces 推导，该字段不是必需 |
| 5 | 状态端口的**语义**区分下探到类型层 | 未做（会与内核"按拓扑而非名字区分"的规则重复） |
| 6 | Gazebo 真值轨迹指标 | 仍不可靠（gzserver 约 1/3 启动失败，真值来源不足） |
| 7 | **绑定只接受 `ControllerInterfaceBase` 派生节点** | 见 §B.2：内部节点要经 `create_library_group` 绑定就必须满足该基类（R5 的带类型指针）。要支持"纯 `StagedControllerInterface` 节点"，需把绑定的实例指针类型参数化，触及 R5 不变式，本轮未做 |

