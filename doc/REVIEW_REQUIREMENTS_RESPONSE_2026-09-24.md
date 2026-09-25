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
| **B** | 静态构建器只支持链，不支持分叉树；`ForChildren` 不能按每个孩子分别比较 | **未做**（已给出计划，见 §B） | 需要变参 `Children...` + 每子端口路由 + 七节点验收树。本轮未实现 |
| **E** | 新/不合格组件仍可能走原生路径；跨模式依赖未拒绝 | **未做**（已给出计划，见 §E） | 需要"晚加载成员准入失败即拒绝该组"与"跨 legacy/staged/two-phase 边界依赖拒绝" |

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

## B 分叉树 typed builder——**未做**（计划）

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
