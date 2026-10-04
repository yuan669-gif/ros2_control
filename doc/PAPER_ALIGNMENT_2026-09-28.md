# 与 FineMote 论文的逐条对照（arXiv:2608.04600v1）

> ⚠️ **2026-10-03 修正（阅读前必看）**：本文件写于 `LITERATURE_SURVEY_2026-10.md` 之前，
> 其中"两趟调度/双向同周期是本文机制或贡献"的叙事**已被取代**——FineMote
> （arXiv:2608.04600）**§III-B 式 (2)** 已给出同一规则（本地 PDF 逐行核对：
> `log/finemote_paper.txt:315`），且上游 `ros2_control` 也**已经**维护一条树导出的线性化
> （issue #853 已关闭；本地源码 `controller_manager.cpp` 的 `controller_sorting()` 即此）。
> 贡献列表、摘要与相关工作请以 **`PAPER_REPOSITIONING_2026-10.md`** 为准；
> 本文件仅作为**过程记录**保留，引用其结论前必须按重定位文档改写。

日期：2026-09-28　论文：*Static Timing Orchestration for Tree-Structured Robot Control Firmware*
（Wang Xi, Feiran Wei, Mo Deng, Weiheng Lin, Pangkit Fong, Jianping He；SJTU；arXiv:2608.04600v1，2026-08-05）
本地文件：`2608.04600v1.pdf`（`pdftotext -layout` 抽取文本放在 `log/finemote_paper.txt`，不入库）

> 这份文档回答一个问题：**我们这套实现是否符合论文的思想？**
> 做法是把论文的主张（Introduction 的思想 + §II–§IV 的机制与定理）逐条对照到本仓库的代码/测试，
> 并给出四种判定：**一致**、**我们更窄（我们是它的特例）**、**缺失**、**我们更强或不同**。
> 所有"实测/测试"结论都标了文件与用例名，可以核对；没有做到的写在 §6，不掩饰。

---

## 0. 结论速览

| 论文主张 | 判定 | 一句话 |
|---|---|---|
| 树结构诱导**双向依赖**：状态自叶向根、决策自根向叶 | **一致** | `FORMAL_MODEL.md`、`BIDIRECTIONAL_EDGE_ANALYSIS.md` 用上游实证把这条钉死 |
| 把每个设备拆成 **Update / Handle 两阶段**（同周期内先后有序） | **一致** | `two_phase_controller_interface.hpp`；`update_phase` / `handle_phase` |
| 静态调度顺序（论文 Eq. 2）：桶内先全部 `+`（子→父），再全部 `−`（父→子） | **一致** | `staged_execution_group.hpp:248`（后序正向）+ `:330`（逆后序）就是同一个序列 |
| 顺序来自**编译期/初始化期**信息，运行期不再排序、开销最小 | **一致（机制不同）** | 论文靠"全局对象 + C++17 部分有序初始化"（Thm 1）；我们靠**类型声明**（`static_topology`/`topology_contract`）+ 列表序准入校验 |
| 同周期可见性条件 `I(child) < I(parent)`（同周期时） | **一致** | 我们把它变成**准入拒绝** `unschedulable_order`（上游 `controller_sorting()` 会违反它） |
| 周期分桶 + 静态优先级（RMS），支持**多周期**设备 | **本轮补齐到"每桶两趟"** | 新增 `TwoPhaseEntry::factor` 与 `two_phase_buckets`；但**跨桶依赖**我们拒绝，不做有界退化 |
| 可调度性（deadline）充分条件（Thm 2，含总线负载） | **缺失** | 我们没有 WCET/期限分析（`IMPLEMENTATION_GUIDE.md` §12.1 #9） |
| 树内决策时延**上界**（Thm 3 / Cor 1 / Cor 2） | **不同** | 我们有更强的**确定性**结论（两趟 0 周期、单趟 = 深度）与**控制代价**定律 `ΔPM = 360·f_c·L·Δt` |
| 事件触发的通信与周期控制分离（总线对象按周期抽象） | **不适用** | ros2_control 里这段边界是 `read()`/`write()` + DDS，我们不声称对它建模 |

一句话：**调度思想完全一致（同一条 Eq. 2），落地机制不同（声明序 vs 初始化序），并且在"可调度性分析"这一支
我们明确缺失；在"确定性"这一支我们给出的结论比论文更强、更窄。**

---

## 1. Introduction 的思想 → 我们的对应物

论文 Introduction 的主张逐条（原文要点 + 我们的位置）：

**(1) "robot description formats naturally represent robotic systems as hierarchical tree structures"**
→ 我们：`static_topology.hpp`（树是类型链，环写不出来）、`topology_contract.hpp`（父子边是类型断言）、
`static_manifest.hpp`（从 binding 的**类型**得到可 `static_assert` 的 manifest）。
判定：**一致**（并且把"描述"从 XML/URDF 提到类型层，见 §4 的差异说明）。

**(2) "tree-structured organization introduces structured data dependencies that affect
perception-to-decision latency"**
→ 我们：`FORMAL_MODEL.md` 定理 1/2（单趟滞后 = 级联深度）、`BIDIRECTIONAL_EDGE_ANALYSIS.md`
（上游 chaining 的实测顺序与陈旧量）、`GAZEBO_CASE_STUDY.md`（真实仿真闭环里同一现象）。
判定：**一致**，且我们有独立的上游实证而不是只做推断。

**(3) "state information is aggregated from leaves to the root, while decisions are propagated from
the root back to leaves" / "bidirectional propagation process"**
→ 我们：`two_phase_controller_interface.hpp` 的注释把两趟写成
`update phase : reverse order (children before parents) -> fresh child state` /
`handle phase : forward order (parents before children) -> fresh parent reference`。
判定：**一致**（术语一致：`Update`/`Handle` ↔ `update_phase`/`handle_phase`）。

**(4) "decomposes each device task into two stages ... These two stages must execute sequentially
within the same device period, introducing intra-task precedence constraints"**
→ 我们：同一次 `update()` 内，一个成员的状态趟必定先于它的命令趟（同一 factor 桶内，见 §3）；
`test_execution_group.phase_order_and_single_call_per_cycle` 断言每阶段每周期恰好一次且顺序正确。
判定：**一致**（论文式 (1c) 的约束）。

**(5) "exploits compile-time information to statically determine execution order with minimal
runtime overhead"**
→ 我们：`update()` 每周期**一次** `atomic_load` 取 generation，执行路径不分配
（`test_hierarchy_comparison.reported_overhead_and_allocations`：配置后 `run()` 100 次 0 分配）；
成员的顺序由内核从边推导（库模式）或由列表序 + 准入校验（管理器模式）。
判定：**一致**，但"编译期信息"的含义不同：见 §4。

**(6) "We prove that the proposed mechanism satisfies deadline and precedence constraints, and
further derive an upper bound on intra-tree decision latency"**
→ 我们：**precedence 一致**（上面 (4)）；**deadline 与上界缺失**（§6.1/§6.2）。
判定：**部分缺失**，这是本轮对照里最重要的诚实结论。

---

## 2. 机制层：论文 Eq. 2 = 我们的两趟（同一序列）

论文 §III-B 的静态策略（原文式 (2)）：对每个周期桶 `Q_w = (d_{n1}, …, d_{n|Q_w|})`（按**注册序**，
子先于父）执行

```
τ+_{n1}, τ+_{n2}, …, τ+_{n|Qw|},   τ−_{n|Qw|}, …, τ−_{n2}, τ−_{n1}
```

我们在库模式内核里就是这一个序列，只是用"后序"表达：

| 论文 | 我们（`hierarchical_control/include/hierarchical_control/staged_execution_group.hpp`） |
|---|---|
| 先全部 `+`，按子→父 | `:248` `// state stage (postorder)`，`:249` `for (const auto node : plan_.postorder)`（**正向**） |
| 再全部 `−`，按父→子 | `:330` `for (std::size_t command_slot = plan_.postorder.size(); command_slot-- > 0;)`（**逆后序**） |
| 每个 `●` 只在其 `■` 之后 | 同一次 `run()` 内两次遍历，天然成立；`test_execution_group.state_failure_aborts_before_command_phase` 与 `command_failure_and_nan_never_commit` 钉住失败语义 |

管理器模式的两趟（`controller_manager.cpp` 的 `update()`）是同一个序列作用在控制器列表上：
状态趟 `for (slot = size; slot-- > 0;)`（反向 = 子先），命令趟 `for (auto & spec : rt_controller_list)`（正向 = 父先）。
**一句话：我们实现的正是论文式 (2)，没有另创一套。**

---

## 3. 本轮补齐：周期分桶（论文 §III-B 的 period buckets）

**对照前的状态（不足）**：论文按**周期**分桶并用 RMS 跨桶定优先级；我们此前**拒绝任何**
`update_rate != 管理器速率` 的成员（准入原因 `unsupported_update_rate`，评审 R7 的产物），
等于只支持"单桶"。这是与论文思想最明显的一处偏差。

**本轮改动**（`TwoPhaseEntry::factor` + `two_phase_buckets`）：

1. 一个成员声明 `update_rate` 时，其**桶因子** `factor = update_rate_ / rate`（当 `rate < update_rate_`
   且**整除**时；`rate == 0` 或 `rate ≥ update_rate_` 时为 1，与上游原生循环的规则一致）；
2. `update()` 对每个桶：`if (update_loop_counter_ % factor != 0) continue;`，桶内状态趟（反向）+ 命令趟（正向），
   并把**桶自己的周期** `factor / update_rate_` 传给 `update_phase` / `handle_phase`——这正是原生循环对
   降频控制器所做的（`controller_manager.cpp:3508` 状态趟、`:3607` 命令趟、`:2781` `two_phase_factor`）；
3. 桶因子在**发布 generation 时**（非实时）算好，控制循环只读 `generation->two_phase_buckets`，不分配；
4. **跨桶依赖被拒绝**（新准入原因 `cross_rate_dependency`，**两端一起报**，避免只排除一端而留下半条边）。

**为什么跨桶边拒绝而不是像论文那样给界**：论文自己的调和周期推论说明，周期不同时
- **向上**（状态）：孩子周期不大于父周期 → 同 tick 可见（`U_{i,u}` 恒真）；
- **向下**（决策）：**只有周期相等**才同 tick 可见，否则要退回到响应时间上界 `R̂⁻`。

也就是说，跨周期的边在论文里本来就不是"两向同周期"，而是"一向同周期 + 一向有界延迟"。
我们对外提供的是**每一对父子、两个方向都同周期**（0 周期滞后）的保证；若接受跨桶边，就会在
"看起来启用成功"的同时把这条保证悄悄降级成有界延迟。因此我们选择**拒绝**并说明原因，
而不是复制论文的退化情形——这是"更窄但更诚实"，写进了枚举注释与日志文本。

**测试证据**（`controller_manager/test/test_two_phase_execution.cpp`，本轮 24 → 25 例全过）：

| 用例 | 钉住什么 |
|---|---|
| `declared_wcet_schedulability_is_reported_per_bucket` | 声明 WCET 后按桶算利用率与 Liu–Layland 界；不满足界时**报告 NOT MET 但不拒绝**（调度仍可运行，只是没有期限保证） |
| `an_undeclared_wcet_makes_the_check_incomplete` | 未声明 WCET ⇒ `complete=false`、`sufficient=false`（"未检查"绝不读成"通过"）；补齐后通过 |
| `a_late_lower_rate_member_joins_its_own_bucket` | 半速成员独立成桶：10 周期里桶跑 5 次、周期是 2 倍、原生循环 0 次调用；同周期全速链跑 10 次 |
| `two_phase_enable_is_refused_for_a_cross_rate_edge` | `mid` 认领 `leaf/target` 而 leaf 半速 → 启用被拒，理由点名"rate bucket"；整链回到原生循环（10 周期里 5 对，且 `legacy_update_calls > 0`） |
| `a_rate_that_does_not_divide_the_manager_rate_is_refused` | 不能整除的速率仍然拒绝（不会被悄悄降频/升频执行） |
| `test_static_controller_registry.a_non_conforming_compiled_in_controller_is_refused` | 编译内置控制器的准入判定与插件一致；**载体从"半速"改成"不能整除的速率"**，因为可整除的低速率现在合法 |

---

## 4. 顺序从哪里来：论文 Thm 1 vs 我们

论文 **Theorem 1（Initialization Order Preservation）**：要求所有设备对象**全局实例化**，并利用
**依赖注入**（子对象先于父对象构造）+ **C++17 部分有序初始化**，得到"全局初始化序与设备森林一致"
（`(d_{n1}, d_{n2}) ∈ E ⇒ I(d_{n1}) < I(d_{n2})`），从而不必在运行期维护依赖图。

我们：**同一个目的，另一个机制**。

| 维度 | 论文 | 我们 |
|---|---|---|
| 顺序载体 | 全局对象的**初始化副作用**（注册事件） | **类型声明**：`static_topology` 的 `Root`/`Descendant`、`topology_contract` 的 `BoundNode<Children...>`、`static_manifest` 的节点/父链 |
| 何时确定 | 静态初始化期（进 `main` 之前） | **编译期**（类型检查），计划在 `configure`/`create_library_group` 落地 |
| 违反顺序时 | 由 C++17 部分有序初始化规则保证（同 TU 内）或 `inline` 声明跨 TU | 编译期直接**写不出来**（父链/环/所有权/量纲），运行期另有 `unschedulable_order` 兜住"列表被上游重排" |
| 多实例/多机器人 | 全局对象 = 共享状态（论文场景是固定固件，可接受） | 声明与实例分离（`compose` 绑定实例，节点就地存储），两个 manager 可用同一 manifest 而互不影响（`test_hierarchy_comparison.two_instances_of_the_same_description_are_isolated`） |
| ROS 生命周期 | 论文对象是纯计算对象，无 lifecycle | 我们显式声明边界：**编译期描述 + configure/activate 绑定资源**（`COMPILETIME_CONTROLLER_RESPONSE_2026-09-26.md` §1.1） |

判定：**一致（目的）+ 更强（机制）**。论文靠"初始化顺序"这一**实现层约定**，我们靠**类型系统**；
后者在多实例、可测试性、失败表达（构造无法表达 `on_configure` 失败，lifecycle 可以）上更严格。
这不是偏离论文思想，而是把同一条静态性要求做得更硬。

---

## 5. 时延结论：论文 Thm 3 上界 vs 我们的确定性 + 控制代价

| 论文 | 我们 | 关系 |
|---|---|---|
| Thm 3 给 `L_m` 的**上界**（响应时间 `R̂+`,`R̂−` + 同 tick 可见性谓词 `U`,`H` 的递推） | 单趟：**滞后 = 级联深度**（`FORMAL_MODEL.md` 定理 1/2，`TWO_PASS_VS_SINGLE_PASS.md` 实测）；两趟：**每个成员的两个方向都 0 周期**（`test_two_phase_execution` / `test_hierarchy_comparison`） | 我们的是**确定性等式/界**，不是统计上界；对"同桶（同周期）"这一情形比论文的界更紧（论文 Cor 2 说 `L_m ≤ max R̂⁻_{n1}`，我们给出 `L = 0` 个释放级等待，只剩计算时间） |
| Cor 1（调和周期）：向上恒同 tick；向下退化为有界 | 我们**拒绝**跨桶边（§3），因此不进入这个情形 | 我们更窄：拒绝而不是给界 |
| 论文以 **WCET/响应时间**为输入，故能同时断言 deadline | 我们**没有** WCET，故不断言 deadline；改用控制代价：`ΔPM = 360·f_c·L·Δt`（相位裕度损失，`CONTROL_COST_OF_LAG.md`） | **互补**：论文说"能不能按时跑完"，我们说"晚一个周期值多少控制性能" |
| — | 额外：最少趟数刻画，`κ(D) ∈ {0,1,2}`、规范两趟满足阶段图 **465/465**（`PASS_LOWER_BOUND.md`）；"纯环⇒不可能"已收窄为充分条件 | **论文没有**这一支；这是我们自己的理论增量（同时撤回了早期的 NP-hard 与"有环即不可能"表述） |

---

## 6. 论文有、我们没有（明确缺口，按重要性；第 1 条已补到"声明 WCET"口径）

1. **可调度性/期限分析（Thm 2 + Lemma 1/2）**：论文把总线响应抽象成周期负载，用 Liu–Layland 给
   `Σ C/P ≤ (N_B+W)(2^{1/(N_B+W)} − 1)` 的充分条件。
   **2026-09-28 已补上论文式的充分条件检查（按"声明 WCET"口径）**：每个成员可从自己的
   `wcet_ns` 参数声明一个周期的执行时间（含两个阶段，对应论文的 `C_n`），管理器按**速率桶**聚合成
   固定优先级任务（桶周期 `T_w`），计算 `U = Σ C_w/T_w` 与 `U ≤ W(2^{1/W}−1)`，并发布
   `TwoPhaseSchedulability{complete, members, declared, buckets, utilization, bound, sufficient,
   worst_bucket_factor}`（`ControllerManager::two_phase_schedulability()`），发布时打印日志；
   未声明的成员会让结论是 **"未检查"** 而不是"通过"。用例：
   `declared_wcet_schedulability_is_reported_per_bucket`、`an_undeclared_wcet_makes_the_check_incomplete`。
   **仍然不同**：我们不测 WCET（不声称 WCET 分析），也不建模总线/通信负载（`read()`/`write()` + DDS
   是这段边界，管理器不建模），因此这仍是"论文的充分条件 + 用户声明的输入"，不是论文的完整分析。
2. **树内决策时延上界（Thm 3）**：我们没有响应时间界；跨桶边因此只能拒绝（§3）。
   → 可行的下一步：为我们承认的"调和 + 跨桶"情形推导一个**周期数**单位的上界（论文用 µs，我们用周期），
   与现有 `ΔPM` 定律衔接。
3. **RMS 优先级与抢占**：论文的桶间优先级是 RMS，且允许抢占。ros2_control 的 `update()` 在同一线程里
   顺序执行所有桶，**没有抢占**；我们只实现了"每桶各自的两趟"。桶间顺序在本实现里不可观测（因为跨桶边
   被拒），所以不影响正确性，但**不能声称实现了 RMS 调度**。
4. **事件触发通信的周期化抽象（Lemma 1）**：不适用（DDS + `read()`/`write()` 边界），也不声称。
5. **µs 级真实平台测量**：论文在 FINS-ROV（14 设备、2 层树、IMU 5 ms、其余 10 ms）上用 RTT 探针测到
   传感器事件→电机输出 **53.64 µs vs 基线 1610.00 µs**、执行器抖动 **5.34 µs vs 18.87 µs**。
   我们的平台是 Linux + ros2_control，测的是**周期数/陈旧量**（`GAZEBO_CASE_STUDY.md`）与
   **分配/中位耗时**（`HIERARCHY_FAIR_COMPARISON.md`），**量纲不同、不可比**，不能引用成"更快"。
6. **多翻译单元的初始化顺序**：论文讨论并给出实现路径（同 TU 或 `inline`）。我们的类型层不涉及
   初始化顺序，因此没有这个风险，但也没有对应的经验。

---

## 7. 我们有、论文没有（差异，写论文时要摆清）

1. **不可行性/最优性刻画**：阶段顶点模型、`κ(D) ∈ {0,1,2}`、`n ≤ 4` 全树 465/465、单趟可行 ⇔ 合并图无环；
   以及"阶段内环需要重复而非更多趟"（`PASS_LOWER_BOUND.md`、`test_pass_lower_bound`）。
2. **控制代价量化**：`ΔPM = 360·f_c·L·Δt` 与"滞后 = 传输延迟"的闭式验证（`CONTROL_COST_OF_LAG.md`、
   `test_stale_state_cost` 7 例）。
3. **可靠性契约**：`StagedFrame{cycle, sample_ns, valid, fault_code}`、复合节点不得重新盖章新鲜度、
   失败**不部分提交**、故障下兄弟一致（`staged_controller_interface.hpp`、`test_execution_group`、
   `test_hierarchy_comparison.sibling_partial_commit_…`）。论文关心时序保证，没有这一层数据级契约。
4. **上游兼容的落地语义**：与 ros2_control 的 admission/lifecycle/claim 打通，含 `atomic_activation`
   的 all-or-nothing 回滚（含硬件 command mode 与 chained-mode 重启恢复）、执行代（generation）发布、
   编译内置控制器注册（`IMPLEMENTATION_GUIDE.md` §12 相关行）。
5. **编译期描述的可用性**：manifest 可 `static_assert`、`declaration_matches_manifest` 在 `configure`
   报名字级不一致、12 个"必须编译失败"的负向语料。
6. **负结果的正面记录**：不更快（CPU 时间）、跟踪误差无改善、`update()` 整体仍分配——这些在
   `WHY_NO_SPEEDUP.md` 里逐条给出原因，避免把"我们没测到"写成"我们有收益"。

---

## 8. 判定总结

- **调度思想：完全一致。** 论文式 (2) 与我们两趟 pass 是同一个序列，双向依赖的建模、两阶段的拆分、
  子先父后的顺序要求、静态决定顺序、运行期最小开销，逐条对应。
- **顺序来源：目的一致、机制更强。** 论文用全局对象初始化序（Thm 1），我们用类型声明 + 准入校验；
  代价是不能声称"实现了 Thm 1 的初始化序"，收益是多实例隔离、可测试、编译期不可写错。
- **多周期：本轮从"拒绝"推进到"按论文 §III-B 分桶"，并只在跨桶边上保留拒绝**（因为我们保证两向 0 滞后，
  而论文在跨周期向下方向本来就只有有界延迟）。
- **最大的真实缺口是分析层**：没有 WCET/期限判定（Thm 2），没有时延上界（Thm 3）。这一条必须写进
  "尚未做到"，并且在任何论文表述里**不能**把我们的确定性结论说成"论文意义上的 deadline 保证"。
- **测量不可比**：论文的 µs 级固件指标与我们的周期/分配指标不同量纲；引用时必须标明。

---

## 9. 复现本对照

```bash
pdftotext -layout 2608.04600v1.pdf log/finemote_paper.txt      # 抽取论文文本（log/ 不入库）
grep -n "Static Device Scheduling" -A 40 log/finemote_paper.txt # Eq. (2) 与周期桶
grep -n "Corollary 1\|Corollary 2\|Theorem 1\|Theorem 3" log/finemote_paper.txt
# 我们的对应物
sed -n '240,340p' hierarchical_control/include/hierarchical_control/staged_execution_group.hpp
./build/controller_manager/test_two_phase_execution --gtest_filter='*bucket*:*cross_rate*'
./build/hierarchical_control/test_pass_lower_bound
python3 research/stage_graph/check_stage_graph.py               # 465/465
```
