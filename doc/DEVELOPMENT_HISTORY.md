# 开发经过：从 FineMote 论文到可运行实现（以及被推翻的结论）

> 这份文档是**时间线与决策记录**，不是宣传材料。它按阶段记录：当时想解决什么、做了什么、
> **实测到了什么**、哪些结论后来被自己推翻。每一段都指向原始提交或专题文档，方便核对。
>
> 仓库事实：分支 `humble-work`（fork `yuan669-gif/ros2_control`）。上游 Humble 基线为
> `469f3055`（**上游仓库**的提交号），但本仓库的 `.git` 从根提交 `b1bf616` 起，没有上游历史：
> 该根提交已包含上游源码树 + 本项目第一版内核，所以下面的阶段时间线是按**提交**组织的，
> 不是按"相对基线的 diff"。完整历史：`git log --oneline`（28 个提交，每个 message 都写了动机）。

---

## 阶段 0：起点与定位（v1 原型 → 形式化）

**起点**：仓库里已经有一个未被管理器集成的 v1 层级原型（`v1-hierarchical-prototype`，见 `GIT_WORKFLOW.md`），
以及一份研究契约（`research` 分支的 `a2ff98a2`）。

**要回答的问题**：FineMote（arXiv:2608.04600v1，SJTU，2026-08-05，见 `RELATED_WORK.md` §1.1）用
`Update`/`Handle` 两个阶段做双向数据传递。这套机制在 ros2_control 里到底能不能落地？
它解决的问题有多"真"？能不能**量化**？

**这一阶段产出的不是代码，而是"能证伪的命题"**：

- `FORMAL_MODEL.md`：把"同周期双向依赖"写成图论问题。定理 1/2：单趟执行的滞后 **L = 级联深度**；
  推论 2：同周期要求图 `G` 无环是**配置期**就可以判定的；
- `BIDIRECTIONAL_EDGE_ANALYSIS.md`：先做**上游行为实证**——不靠读源码下结论，而是用测试把
  上游 chaining 的真实顺序、陈旧量、claim 行为测出来；
- `CONTROL_COST_OF_LAG.md`：把"滞后一周期"换算成控制意义：`ΔPM = 360·f_c·L·Δt`。

**为什么先做这三份**：如果"滞后"在控制意义上不重要，后面所有工程都不值得做。这一步把问题从
"看起来优雅"变成"有代价、可量化"。

---

## 阶段 1：两趟调度内核与第一个实验（`b1bf616`）

**做法**：在同一份控制器顺序上跑两趟——反向 `update_phase` + 正向 `handle_phase`。
最轻的实现放在 `cycle_tree.hpp`（库模式、不依赖 ROS），并给出 `TwoPhaseControllerInterface`。

**当时的核心问题**："一份线性化顺序够不够？" 结论是够：两趟方向相反，所以同一份顺序同时满足两向。

**同时做了两件事，后来都变成"必须诚实对待"的部分**：

1. `PASS_LOWER_BOUND.md`：把定理 1 从"一趟不够"推广到"**最少几趟**"。
   正确的模型是**阶段顶点图**（每个节点拆成状态顶点 `S_v` 与命令顶点 `C_v`），
   `κ(D)` 是最少趟数。规范两趟调度满足阶段顶点图**全部边**：`n ≤ 4` 的全部有根树 × 全部边标注 =
   **465/465**；"单趟可行 ⇔ 合并要求图无环"也在 465 例上一致。
2. `TWO_PASS_VS_SINGLE_PASS.md`：定量实验。单趟下父读到的子状态滞后 `i` 个周期（`i` = 深度），
   两趟下两个方向都是 0。

---

## 阶段 2：四轮评审 R1–R11 与"自我推翻"（`6932750` … `1bd4c59`）

外部评审（记录在 `REVIEW_HUMBLE_WORK_2026-09-23.md`、`REVIEW_REQUIREMENTS_2026-09-24.md`）提出
R1–R11。回应在 `REVIEW_RESPONSE_2026-09-23.md`（558 行）与 `REVIEW_REQUIREMENTS_RESPONSE_2026-09-24.md`（620 行）。
这一阶段最重要的产出**不是修复，而是被推翻的结论**：

| 曾经的表述 | 实测结果 | 现状 |
|---|---|---|
| 判定 `κ(D)` 是 NP-hard | `κ(D)` 只取 `{0,1,2}` | **撤回**，不再声称复杂度结论（`PASS_LOWER_BOUND.md`） |
| "`G` 有环 ⇒ 不可能同周期" | 只有**充分性**成立，反向不成立 | **收窄**为充分条件 |
| "深链编译成本 ~9 ms/节点" | 9.63 ms 是**深度 64 的链**上的数字 | **更正**为"与形状和深度都有关"，见 `COMPILE_COST.md` §2/§3.1 |
| "沿父链查询把 `O(d²)` 降到 `O(d)`" | 实测**慢 3.24 倍**（0.17 s → 0.55 s，GGC 13 → 20 MB） | **否决**，实现留在 `research/static_topology_variants/` |
| 显式欧拉的 `Δt·kd = 2` 当作"增益上限" | 是**离散化伪影** | 改成精确的逐步更新（`CONTROL_COST_OF_LAG.md`） |
| R6 的方向（谁是 claimant） | 反了：claimant 是**父**、writer 唯一性是**按端口**、parent 唯一性是**按子** | 修正并加负向用例 |
| "接入执行组能让 `update()` 零分配" | 执行组自身 0 分配，`ControllerManager::update()` 整体仍分配（Humble 的 lifecycle/`MutexMap`） | **限定口径**，见 `HIERARCHY_FAIR_COMPARISON.md` §4 |
| "本项目比 LET 更好" | LET 用全局逻辑时刻按定义解决同一问题 | **撤回**，改为"不需要时基/双缓冲、不增加周期延迟，代价是要求可分解阶段与无环层次" |

**同阶段修掉的真缺陷**（都有测试）：

- 同一对象挂两个名字 → 每阶段被调用两次（实测 3 周期 6 次 `update_phase`）：编译期 + 运行期双重拒绝；
- 执行组 `run()` 里的局部向量导致每周期分配 → 改为两趟都遍历 `leaves_`，100 次调用 0 分配；
- `verify_ports_match_contract` 只比较 `.size()` → 改为按名字与顺序比较，并且诊断打印"实际 vs 声明"；
- `compose` 静默覆盖 `parent_type` → `child_declares_this_parent` 静态断言；
- 两个真实数据竞争（**发布会话协议**的 TSan 抓到，提交 `da1dd2a`）；阶段 4 的真实管理器 TSan 又抓到 4 条，见下。

---

## 阶段 3：编译期契约与元编程（`4b9e2d2` → `2ae9333` → `3aca6fe`）

**动机**：阶段 1–2 把**调度**做对了，但"树长什么样"仍然是运行期字符串。既然无环性、量纲、端口归属
都是**类型层就能判定**的，就应该让非法配置**写不出来**。

按依赖顺序做了六层（详见 `CODEBASE_TOUR.md` §4.6–§4.11）：

1. `static_topology.hpp`：树是类型链，环写不出来（把推论 2 从"配置期拒绝"强化成"写不出来"）；
2. `dimensional_interfaces.hpp`：量纲进类型；
3. `topology_contract.hpp`：`Port`/`PortList`/`Contract`/`BoundNode`/`compose`；
   `BoundNode` 从"单 `Next` 槽"改成**变参 `Children...`**（`std::tuple`），于是分叉不用改类型；
4. `typed_ports.hpp`：端口**只声明一次**，字符串表与 `Contract` 由类型生成；
   新增第 6 张表 `HardwareState`，并能生成运行期接口声明；
5. `topology_binding.hpp`：类型 binding → 可运行执行组，指针全程类型化；
6. `static_manifest.hpp`：从 binding **类型**得到编译期 manifest，可 `static_assert`。

**工程量最大的两个坑**（都在提交 message 里）：

- **指针**：早期用 `void*` 往返恢复 `StagedControllerInterface*`。当控制器同时继承
  `ControllerInterfaceBase` 与 `StagedControllerInterface` 时，转第二个基类需要**地址调整**，
  `void*` 往返会丢（实测：恢复出的指针与正确调整后的不同）。改为保留类型化指针 + `dynamic_cast` 访问器。
- **不可移动**：`ControllerInterfaceBase` 没有移动构造，所以树节点必须**就地构造**
  （`std::optional`/`unique_ptr`），不能用 `std::tuple<Node...>` 的值语义。

**负向证据**：`hierarchical_control/test/static_topology_negative/` 15 个源文件，
12 个必须编译失败且诊断包含特定子串，2 个必须编译通过，1 个是公共头。

---

## 阶段 4：管理器集成与可靠性（`9ffe361`、`c9e2b3d`、`408c31b`、`f242324`、`e783275`）

这一步把内核从"库模式"接到真实 `ControllerManager`：

1. **执行代（generation）**（`9ffe361`）：模式 + 两趟成员表 + 执行组合并成一个不可变快照，
   `update()` 每周期**一次** `atomic_load`，每次配置变更**一次** `atomic_store`；
   新增 `execution_generation()` 单调 id 作为可观测契约。
   同期加入 **atomic activation**（可选，默认关，见下）。
2. **编译内置控制器**（`c9e2b3d`）：`StaticControllerRegistry`（工厂表）+ `load_controller()` 先查表，
   命中后走**同一个** `add_controller_impl()`，生命周期/claim/准入完全共用。
3. **运行期重配约束写成代码**（`408c31b`）：`update()` 用 RAII 计数器标记"周期在飞"，
   周期在飞时**安装**执行路径返回 `ERROR` 且不发布任何 generation，**移除**始终允许。
4. **真实管理器 TSan**（`f242324`）：只给 `controller_manager` 插桩，跑真实 `update()` vs
   `switch_controller()`/`set_two_phase_execution()`。**先测出 4 条数据竞争**，全在上游握手字段：
   `switch_params_.do_switch`、`switch_params_.activate_asap`、
   `RTControllerListWrapper::used_by_realtime_controllers_index_`、`updated_controllers_index_`。
   改成 `std::atomic` + release/acquire 后归零。依赖库未插桩，其内部竞争**看不见**。
5. **编译期控制器设计边界**（`e783275` + `COMPILETIME_CONTROLLER_RESPONSE_2026-09-26.md`）：
   明确"编译期描述 + 运行期生命周期"这条线，不声称"静态初始化期完成 ROS 控制器初始化"。

同期（阶段 3–4 之间）还补了一个**跨模式准入**：一条参考边两端一个是两趟成员、一个是 legacy 时，
两个调度无法同时被满足 → 拒绝并点名两端。

---

## 阶段 5：最新一轮评审 P1/P2（`042c04c`）

`REVIEW_COMPILETIME_LATEST_2026-09-26.md` 提了 2 个 P1 + 4 个 P2，逐项处理（回应：`REVIEW_COMPILETIME_LATEST_RESPONSE_2026-09-26.md`）：

| 项 | 问题 | 处理 |
|---|---|---|
| P1-1 | atomic 回滚只做 lifecycle，**没有**把硬件 command mode 换回去 | 记录本次 claim 的接口，回滚时 `prepare/perform_command_mode_switch({}, ifaces)`；回滚失败单独上报。**探针实测**：mock 模式计数器 +202（关掉该步的探针 +101） |
| P1-2 | atomic 默认关闭，静态树可能部分激活 | 安装执行组**要求** atomic 打开；部分成员时组**完全惰性**并在切换线程点名告警。用例：5 周期后成员计数全不变，补齐后 3 周期恰好 +3 |
| P2-1 | `manifest_problem()` 不查手工 manifest 的环 | 加有界父链上溯。**要说清**：纯环本来就已被"恰好一个根"拦住，真正的漏洞是"链进入环"（根 + `a→b, b→c, c→b`） |
| P2-2 | registry 注册时机没有代码约束 | 第一次查表/添加前**封印**；封印后 `add()` 抛异常、替换 registry 被拒、`register_static_controller_type()` 返回 false |
| P2-3 | 参数化 factory 没有 manifest | 新增 `ManifestDescriptor` 与 `add_factory(type, factory, descriptor)`；`add_factory<ControllerT>` 从类型派生 |
| P2-4 | TSan 脚本可能把"没跑起来"当 PASS | 要求预期 suite 真的跑过（`[ RUN ]` ≥ 20 且两个 suite 都出现），否则退出码 3；race verdict 与功能 verdict 分开 |

**同一轮里被实测否定的两个自我假设**（值得单独记）：

- P1-2 的第一版用例想用"停用某个成员"构造部分状态，**被上游拒绝**：
  `Could not deactivate 'stage_leaf' because preceding controller 'stage_module' is active`
  （这条限制早在 `IMPLEMENTATION_GUIDE.md` §12.4 #10 记录过）→ 改成用可达的"逐个激活"构造；
- "部分激活只可能由失败的 switch 造成"是错的：**逐条激活**本身就会留下部分激活。
  所以"安装时要求 atomic"单独**不足以**保证整棵树，必须再加"部分成员惰性 + 告警"这一半。

---

## 阶段 6：chained-mode 重启的回滚（`6a0a7f5`）

**问题**（由使用者指出，随后实测确认）：上游为切 chained mode 会把一个**本来已 ACTIVE** 的
following controller 先停后启。它重启成功后出现在"本次激活"集合里，而旧回滚对"本次激活"一律
`deactivate()` → **一次失败的 switch 顺带停掉了一个与失败无关、本来在运行的控制器**。

**修复**：

- `switch_controller()` 在改写请求列表**之前**抓 `pre_switch_state_`（名字/是否 ACTIVE/是否 chained），
  经发布请求的同一个 release/acquire 交给实时线程；
- 回滚分三类：本次从 INACTIVE 激活的（停用并释放）／**切换前已 ACTIVE 的 restart**
  （停一次 → 恢复切换前 chained mode → 经普通激活路径重新激活）／本次做过的**所有** chained-mode 切换
  （包括没有被激活的 following controller）；
- `ActivationOutcome::activated` 改成每控制器一条记录（名字 + 本次 claim 的接口 + 是否 chainable），
  硬件换回**排除**仍然 ACTIVE 的 restart 控制器；
- restart 自己激活失败也在 `pre_switch_state_ ∩ activate_request_` 里被找回，找不回则记
  `rollback_failed` 并明确报"这些控制器切换前在运行、现在停了"。

**探针实测**：关掉 restart 恢复逻辑（只重建库）→ 新用例失败，`child` 停在
`'\x2'`(INACTIVE) 而期望 `'\x3'`(ACTIVE)；恢复后 `test_atomic_activation` 7/7。

**同轮对"既有 flaky"的前后对照**（避免把环境问题算成自己的回归）：

| 用例 | 改动前（`b079f07` 的库） | 改动后 | 失败签名 |
|---|---|---|---|
| `test_controllers_chaining_with_controller_manager` | 3/4 通过 | 2/4 通过 | 同为 `internal_counter = 15` vs 期望 `14` |
| `test_spawner_unspawner`（`failed_activation`） | 2/4 通过 | 1/4 通过 | 同为 `Could not connect to service /test_controller_manager/list_controllers` |

---

## 阶段 7：与 FineMote 原文对照，并补齐周期分桶（`2026-09-28`）

拿到论文原文（`2608.04600v1.pdf`）后逐条对照（结果见 `doc/PAPER_ALIGNMENT_2026-09-28.md`），
发现**调度思想完全一致**（论文 §III-B 的式 (2) 就是我们两趟 pass 的同一序列），但有两处实质差异：

1. **顺序来源不同**：论文用"全局对象 + 依赖注入 + C++17 部分有序初始化"得到确定的孩子先于父的注册序
   （其 Theorem 1），我们用**类型声明**（`static_topology`/`topology_contract`）+ 列表序准入校验
   （`unschedulable_order`）。目的相同，机制更强（编译期写错就不过），代价是不能声称实现了它的 Thm 1。
2. **多周期支持**：论文按周期分桶 + RMS 跨桶优先级，我们此前**拒绝任何** `update_rate != 管理器频率` 的成员。

**本轮据此放宽第二条**（评审 R7 之后第一次语义放宽）：新增 `TwoPhaseEntry::factor` 与
`generation->two_phase_buckets`，`update()` 改为**逐桶**跑"状态趟（反向）+ 命令趟（正向）"，
并把桶自己的周期传给两个阶段——这就是论文的 period bucket。**跨桶依赖仍然拒绝**（新原因
`cross_rate_dependency`，两端一起报）：论文自己的调和周期推论说明周期不同时**向下**方向只有"有界延迟"，
而我们对外保证每条被接纳的边**两向 0 周期滞后**，所以宁可拒绝也不静默降级。
不能整除管理器频率的速率也仍然拒绝（否则等于升/降频执行）。

**同时明确记录的缺口**：论文的**可调度性/期限判定**（Thm 2，含总线负载抽象）与**时延上界**（Thm 3/Cor 1/Cor 2）
我们没有；我们的互补结论是"确定性（两向 0 滞后）+ 控制代价 `ΔPM = 360·f_c·L·Δt`"，而论文的 µs 级固件指标
与我们的周期/分配指标**量纲不同、不可比**。

**测试**：`test_two_phase_execution` 24 → 25 例（新增 `a_late_lower_rate_member_joins_its_own_bucket`、
`a_rate_that_does_not_divide_the_manager_rate_is_refused`；原"速率不匹配一律拒绝"用例改名为
`two_phase_enable_is_refused_for_a_cross_rate_edge`），`test_static_controller_registry` 的准入用例改用
"不能整除的速率"作为载体；全套 ctest 21/21。

## 阶段 8：面向使用者的文档，并修掉一处错误的核对指令（`2026-09-28`）

新增 `doc/USER_GUIDE.md`：从安装（库模式只装 `hierarchical_control`；管理器模式构建覆盖层）到
**三种用法**（A 库模式 / B 管理器两趟 / C 管理器执行组）的选择与步骤、参数与 API 速查、运行期约束、
故障排查表，并明确写出两条容易被忽略的事实：

1. **执行组没有 ROS 入口**（只有 C++ API），想要"标准 node + spawner"就选 B；
2. 覆盖层的 `controller_manager` **ABI 与系统安装不同**（类新增了数据成员），
   任何自行实例化 `ControllerManager` 的包（如 `gazebo_ros2_control`）必须一起重编。

同时修掉一处**我自己写错的核对指令**：早期文档让人运行 `git diff --stat 469f3055..HEAD`，
但 `469f3055` 只是**上游仓库**的 Humble 提交号，本仓库的 `.git` 从根提交 `b1bf616` 起
（该提交已含上游源码树 + 第一版内核），本地没有该对象，命令会报 `bad revision`。
现已统一改为 `git log --oneline` + `git show --stat <commit>`，并说明"与上游对比需要另备一份干净源码树"。

## 阶段 9：补上"声明 WCET"的可调度性检查（`2026-09-28`）

对照论文时列出的最大缺口是**分析层**：论文有 Thm 2（Liu–Layland 充分条件，含总线负载抽象）与
Thm 3（树内时延上界），我们此前两者都没有。本轮把**可检查的那一半**补上，并把不能声称的部分写清：

- 新增每个控制器参数 `wcet_ns`（整数纳秒，覆盖两个阶段，对应论文的 `C_n`）；
- 管理器在**发布 generation 时**（非实时）按速率桶聚合成固定优先级任务（桶周期 `T_w`），
  计算 `U = Σ C_w/T_w` 与 `U ≤ W(2^{1/W}−1)`，发布 `TwoPhaseSchedulability`，
  并在日志里给出结论与**最忙的桶**（`ControllerManager::two_phase_schedulability()` 可读）；
- **未声明 ⇒ "未检查"**（`complete=false`, `sufficient=false`），绝不读成"通过"；
  **不满足界 ⇒ 只报告、不拒绝**（调度仍可运行，只是没有期限保证）；
- 明确**不**建模总线/通信负载（`read()`/`write()` + DDS 是这段边界），**不**做 WCET 分析——
  这两点是"论文有、我们没有"的残余，写进 `PAPER_ALIGNMENT_2026-09-28.md` §6。

用例：`declared_wcet_schedulability_is_reported_per_bucket`（舒适集 U=0.30 ≤ 0.828；
9 ms+9 ms ⇒ U=1.35 > 0.828 报告 NOT MET 且模式保持开启）、
`an_undeclared_wcet_makes_the_check_incomplete`（3 成员 1 桶、未声明 ⇒ 不完整；补齐 3×1 ms ⇒ U=0.3 通过）。
`test_two_phase_execution` 25 → 27 例。

## 决策记录（"为什么是这样"）

| 决策 | 理由 |
|---|---|
| 两趟走**同一份**顺序，而不是两份 | 反向遍历天然给出拓扑序；少一份数组就少一个不一致来源 |
| 依赖边**从 claim 推导**，不做配置项 | 配置项会与真实 claim 漂移；推导出来的边不可能不一致 |
| 帧的 `sample_ns` 由**执行组**用最旧子样本覆盖 | 否则复合节点可以把自己的派生数据"重新盖章"成更新，新鲜度契约就失效 |
| `manifest` 是**类型**（`static constexpr`），不是运行期 `span` | 只有类型才能在 `static_assert` 里用；"编译期"不能是修辞 |
| 端口声明是**类型**，字符串表**生成** | 同一事实写两遍必然会漂移，而且没人发现 |
| 硬件接口名留在 manifest，**下标**留在 activation | 下标是"某一次激活"的属性；把全局资源下标烘进编译期对象会耦合 ResourceManager 内部 |
| 执行状态是**一个** generation | 避免"新模式 + 旧成员表"的中间态；被拒请求不发布，用单调 id 可观测 |
| atomic activation **默认关** | 上游 best-effort 语义与它自己的 spawner 测试依赖这一点；改变默认值等于改变语义 |
| 安装执行路径要求 atomic 打开 | 执行组是整棵树；有且只有回滚能保证失败的 switch 不留下半棵树（我们自己的 API，无上游兼容负担） |
| 编译内置控制器用**工厂**而不是全局实例 | 两次 `load` 必须是两个独立对象（多管理器/同名类型隔离），测试有断言 |
| `void*` 往返被弃用 | 多继承下地址调整会丢（实测） |
| 深链 `O(d)` 优化被否决并保留被拒实现 | 实测慢 3 倍；保留实现与复现脚本，避免以后重复提议 |

---

## 还没做 / 明确不做

来自 `IMPLEMENTATION_GUIDE.md` §12（权威清单）与各评审的收尾项：

- **硬实时 WCET / deadline**：只测了分配次数与中位耗时，非实时虚机；
- **依赖库内部 TSan**：rclcpp/lifecycle/hardware_interface/FastRTPS 未插桩（磁盘与时间成本）；
- **控制器列表纳入 generation**：因此安装执行路径仍有"停止期配置"约束；
- **多执行组**：同时只支持一个 `staged_group`；
- **运行期并发注册**：registry 提供"封印后只读"，不是并发写安全；
- **物理总线原子提交**：`prepare/perform` 两段式的固有限制；
- **完整 Humble VM 全量构建记录**：目前是本机（2 核、高 load）记录 + 已知 flaky 的分类。

---

## 怎么核对这份历史

```bash
git log --oneline                                   # 28 个提交，按时间
git show <commit>                                   # 每个提交的 message = 动机 + 实测数字
git log --oneline                                    # 28 个提交，按时间
git show --stat <commit>                             # 单提交的改动清单（message 含实测数字）
# 想与干净的上游 Humble 源码对比：另备一份源码树，用 git diff --no-index 或 diff -ru
ls doc/REVIEW*.md doc/*RESPONSE*.md                  # 评审原文与回应（成对出现）
grep -rn "撤回\|已否决\|更正" doc/*.md               # "被推翻的结论"散落位置
```
