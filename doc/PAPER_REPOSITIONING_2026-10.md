# 论文重定位（2026-10）：在 FineMote 已发表的情况下怎么写

日期：2026-10-03　分支：`humble-work`　输入：`LITERATURE_SURVEY_2026-10.md`（调研）、
`RELATED_WORK.md`（2026-09-24 首轮查新）、`PAPER_SKELETON.md`（旧骨架）、
`COMPILETIME_AUDIT_2026-10.md`（编译期审计）、`feature/two-phase-manager`（最小可用实现）

> **这份文档的唯一目的**：把"写论文"从"提出两趟调度"改成**能过审的**叙事，并给出可直接搬用的
> 摘要/贡献列表文本。它不掩饰结论——原来的中心命题被先行工作覆盖了。

---

## 0. 新证据改变了什么（与 2026-09-24 首轮查新相比）

2026-09-24 的 `RELATED_WORK.md` 已经判定 C1/C2/C4 是教科书内容、C3 的机制来自 FineMote，
并把贡献重新表述为"应用/工程/量化"。**2026-10 的调研加了三件更硬的事实**：

| # | 新证据 | 核实方式 | 对论文的后果 |
|---|---|---|---|
| E1 | FineMote **§III-B 式 (2)** `τ⁺_n1 … τ⁺_n|Qw|, τ⁻_n|Qw| … τ⁻_n1` **就是**"同一线性化、桶内先全部 Update（子→父）、再全部 Handle（父→子）" | **我本人**对本地 `2608.04600v1.pdf` 的抽取文本逐行核对（`log/finemote_paper.txt` 第 315 行） | "两趟调度"**不能**作为机制贡献；§IV-C2 推论 2 也已给出同周期零等待 |
| E2 | 上游 `ros2_control` **已经**在配置期用 `controller_sorting()` 维护一条由树导出的线性化（issue #853 于 2023-08 关闭） | issue #853 由调研核实（抓取）；`std::stable_sort(... controller_sorting ...)` 在本地 Humble 源码中**我本人**核对 | "保持一条树导出的线性化"**不是**增量；相对上游的增量**只有第二趟** |
| E3 | 编译期元编程的每个成分都是成熟技术（typestate 1986、dimension types 1994/97、session types 1998、policy-based design、**负向编译测试** Pigweed）；未发现把"控制器拓扑"编码成类型链的先例 | 由调研核实（含 DOI/URL） | 创新点 2 是**领域空白**、不是**方法空白**；审稿人最可能问"为什么不用 codegen（`generate_parameter_library`）" |

**结论**：论文的**中心命题**必须换成"**把已知纪律移植到一个新约束集（运行期插件 + YAML 部署）上，
并量化移植的代价与边界**"，而不是"提出一种新调度"。

---

## 1. 逐条处置旧贡献（C1–C9）

| 旧编号 | 处置 | 处置后的表述 |
|---|---|---|
| C1 双向同周期不可满足 | **降级为背景** | 写成"教科书事实（引 SDF/同步语言/LET）"，**不再列为贡献**；只保留"在上游 ros2_control 上**实测**该组合未被覆盖" |
| C2 单趟滞后 = 深度 D | **降级为引理** | 作为量化上游行为的工具（引采样控制/任务链延迟分析），不作为定理卖点 |
| C3 两趟充分、不需第二份顺序 | **撤回为贡献**，改为**引用 + 移植** | "FineMote 式 (2) 的纪律；本文的工作是把它落到运行期插件集合上" |
| C4 滞后→相位裕度→增益上限 | **保留但改定位** | "经典控制结论（引 Franklin/Powell、Åström/Wittenmark）**接到具体的调度量上**"，并做闭环验证；贡献是**接通与验证**，不是定律 |
| C5 上游静默降级、顺序相关 | **保留为核心实证贡献** | 升级为主贡献之一（见 §2 的 N1） |
| C6 整组提交不部分提交 | **保留，但必须与"两阶段提交"切割** | 明确写"与事务处理的 two-phase commit 无关（引 Gray & Lamport 以区分）"，且**最小分支明确不含此保证** |
| C7 配置期可判定拓扑校验 | **并入 N3** | 作为编译期层的一部分 |
| C3′ 最少趟数/两趟最优 | **已撤回**，保持撤回 | 只在"负结果"章节出现 |
| C9 元编程前移到编译期 | **升级为核心贡献之一（N3）** | 从"未验证的设计方向"变成"已实现 + 已测量 + 有边界证明"（见 §2） |

---

## 2. 三条可辩护的贡献（替换旧贡献列表）

> **N1（实证 / 定位）**：在 `ros2_control` Humble 的真实代码上，**刻画并量化**了父子间同时存在
> state 边与 reference 边时单趟调度的**静默单向陈旧**：它总是 reference-first、与注册顺序无关；
> `configure_controller` 完全不校验声明的 state 边；并且**顺序由 `controller_sorting()` 的字符串
> 启发式决定**（"command interface 为空的 chainable 被排到父之前"是可复现的具体机制）。
> 这是**未找到他人公开分析**的行为（引上游 chaining 文档、issue #2189 说明链方向本身令人困惑）。

> **N2（系统 / 移植）**：把 FineMote 式 (2) 的两趟纪律**移植到运行期插件 + YAML 部署**的控制器集合上，
> 并证明移植的**真实成本是把"正确性"换成了"一组有限的运行期准入检查"**：本文给出这组检查
> （跨模式边、列表顺序、同实例两名、速率桶、失败包含），给出最小可用补丁
> （`feature/two-phase-manager`：**15 文件 +2862 行、无新包、默认关闭**、22 例测试），
> 并**量化**相对单趟的差异（三级链阶跃下 `root` 从 `0.0` 变为 `0.25`，叶子命令从 `-1.0` 变为 `-1.75`）。

> **N3（方法 / 枚举）**：回答"实时控制框架里**哪些东西可以前移到编译期**"，并给出**分界线**：
> 由**类型**唯一决定的那一层（层次形状、父子所有权、端口量纲、接口需求集合、每 role 端口唯一性、
> 声明式树的会员性/边/顺序）可以前移；由**部署**唯一决定的那一层（插件名、YAML 数值、URDF、
> 运行期可用接口列表、生命周期、控制器列表顺序）**必须**留在运行期，且其检查**不可删除**。
> 三处落地并测量：manifest 不变式接到入口（编译代价 **+8 ms / 0.08%**）、叶子集编译期化、
> 声明式两阶段树的编译期描述层（**核心是"编译期边集 == 运行期字符串推断边集"的等价性测试**）。

---

## 3. 可直接搬用的文本

### 3.1 摘要（草稿，英文）

> Tree-structured robot controllers have a **bidirectional** data dependency: state is aggregated
> from the leaves while decisions propagate back to them. A scheduler that runs every controller once
> per cycle cannot satisfy both directions in the same cycle, and the resulting staleness is silent.
> A recent firmware framework resolves this with a two-stage decomposition and a static schedule; we
> instead ask what that discipline costs when the controller set is **loaded at run time as plugins
> and configured by YAML**, as in ROS 2's `ros2_control`. We characterise the failure empirically on
> unmodified `ros2_control` Humble, port the two-pass discipline behind an opt-in interface with no
> new package, and show that the port's real price is not performance but a **finite set of runtime
> admission checks** — membership, ordering, rate buckets and failure containment — which we state,
> implement and test. Separately, we ask how much of a controller description can be moved to
> **compile time** in C++17, and report the boundary: everything a *type* determines uniquely can be
> rejected before the binary exists, while everything a *deployment* determines uniquely cannot, and
> its runtime checks cannot be removed. We report the negative results as well.

### 3.2 贡献列表（正文用，四条）

1. 对未修改的 `ros2_control` Humble 给出双向边**静默陈旧**的实证刻画与机制（含排序启发式的具体成因）。
2. 给出把两阶段纪律移植到**运行期插件**集合上的最小补丁与**准入条件集合**，并公开展示其边界
   （无整组提交、跨速率桶拒绝）。
3. 给出"实时控制框架的编译期边界"这一**枚举式结论**，用三处实现与编译代价测量支撑。
4. 诚实的负结果：不更快（CPU 约 2×）、跟踪误差无收益、部分早期叙事已自行撤回。

---

## 4. 相关工作必须写进去的引用（按主题）

| 主题 | 必引 | 作用 |
|---|---|---|
| 锚点（机制已存在） | FineMote, arXiv:2608.04600 | 说明本文**不主张**机制首创；对齐式 (2) 与推论 2 |
| 上游行为 | ros2_control issue #853、issue #2189、chaining 文档 | 说明上游**已**维护一条线性化，本文增量是第二趟 |
| 环路与延迟 | Lee & Messerschmitt 1987（SDF）、Lustre/Esterel（因果性） | 说明"无延迟环不可调度"是教科书结论 |
| LET | Giotto（Henzinger et al. 2001）、Kopetz & Bauer 2003 | 说明 LET 是"用延迟换确定性"，与本文方向相反 |
| 采样延迟 | Franklin/Powell/Emami-Naeini、Åström & Wittenmark | 说明相位裕度结论是经典内容 |
| 任务链延迟 | Tindell & Clark 1994、Becker et al. 2017、Dürr et al. 2019 | 支撑"滞后沿链累积"的分析框架 |
| 图算法 | Tarjan 1972、Kildall 1973 | 后序/逆后序是标准工具 |
| 与"两阶段提交"切割 | Gray & Lamport 2006 | **必须**，否则术语会被误读 |
| 编译期方法 | Strom & Yemini 1986（typestate）、Kennedy 1994/1997（dimension types）、Honda 1998（session types）、Pigweed `pw_compilation_testing` | 说明创新点 2 的成分都是已有技术 |
| 对手方法 | `generate_parameter_library` | **必须正面回应**"为什么不是 codegen" |

DOI 与 URL 清单见 `LITERATURE_SURVEY_2026-10.md` §4。

---

## 5. 每条新贡献需要什么证据（对照仓库现状）

| 贡献 | 已有证据（本仓库） | 缺口 |
|---|---|---|
| N1 | `BIDIRECTIONAL_EDGE_ANALYSIS.md`、`controller_manager/test/test_upstream_ordering.cpp`、`test_hierarchy_comparison.cpp`、`GAZEBO_CASE_STUDY.md` | Jazzy/master 只做到**逻辑层**比对，未做数据通路复现（诚实写"仅 Humble 实测"） |
| N2 | `feature/two-phase-manager`（22 例，含数值级断言的两种路径对照）；`CONTROL_COST_OF_LAG.md`、`test_stale_state_cost.cpp` | **跨速率桶**目前是"拒绝"而非"有界接纳"（`PENDING_WORK.md` §2.1 / P0-2）；无真实硬件验证 |
| N3 | `COMPILETIME_AUDIT_2026-10.md`（13 项审计 + 三处实现 + 代价测量）、`METAPROGRAMMING_CONTRACT.md`、`DIMENSIONAL_INTERFACES.md`、负向编译语料 18 项 | 编译代价只在**本机 2 核**测过；跨编译器/跨版本未测；诊断质量未量化 |
| 负结果 | `WHY_NO_SPEEDUP.md`、`SCHEDULING_PERFORMANCE_COST.md`、`PAPER_SKELETON.md` §7 | 需要在同一章与正结果并列呈现，不能藏进附录 |

**建议的最小补充实验**（按性价比排序）：
1. **P0-2 的跨桶上界**：把"拒绝跨桶边"升级为"有界接纳"并**实测滞后 ≤ 声明上界**——
   这是 N2 唯一缺的能力性证据，也是与 FineMote 差异最大处。
2. **编译代价跨编译器复测**（GCC + Clang 各一次），把 `+8 ms` 从"本机噪声内"变成"有区间的结论"。
3. **真实硬件或 `Gazebo` 的闭环复现**：把 N1 的"静默陈旧"从代码级证据提到系统级证据。

---

## 6. 审稿人会怎么打，怎么答

| 攻击 | 回答（必须写进正文，不能回避） |
|---|---|
| "这就是 FineMote 式 (2) 重实现。" | 承认机制一致并**引用**；本文的贡献是**运行期插件集合**上的移植、**准入条件集合**、以及对上游静默陈旧与编译期边界的**量化** |
| "上游已经排序控制器了，你只是加了一个循环。" | 对；增量是第二趟。用 N1 的机制说明"一条线性化**结构上**偏向 state 方向"，并给出数值证据 |
| "你的两趟不是前向/后向数据流迭代（那是迭代到不动点）。" | 明确区分：本文是**每周期各一次**的相反遍历，不是不动点迭代；引用 Kildall 仅为"术语句式相近"的说明 |
| "你放弃了整组原子提交，失败会留下半写状态。" | 承认，并把它写成**契约与边界**（并有测试）；同时说明这是最小分支的**有意取舍** |
| "为什么用 TMP 而不是 codegen？" | 正面回答：codegen 处理**值**（参数/YAML），本文处理**结构**（层次、所有权、量纲）；两者互补，且本文测量了 TMP 的编译代价与诊断质量 |
| "静态保证只覆盖编译进来的那棵树。" | 这正是 N3 的**结论**而不是缺陷：分界线本身是贡献；运行期检查保留且不可删 |

---

## 7. 投稿去向（调研结论）

| 去向 | 适配度 | 条件 |
|---|---|---|
| **JOSS** | **最现实** | 明确接受"重实现已知方案"，但**要求** state-of-the-field 与 build-vs-contribute 说明——与本文的诚实叙事完全一致；需要 CI、README、安装说明 |
| ECRTS tools track | 中 | 必须先答"相对 FineMote 新在哪"；20 页 LIPIcs、双盲、可选 artifact evaluation |
| ICRA/IROS/RA-L | 中 | 需要把 N1/N2/N3 合成一条故事线，并补真实系统验证（**页数限制未核实**） |
| T-RO / TII | 偏低 | 需要更强的系统级量化 |
| ROSCon | 补充 | 非归档、proposal 评审；适合工程叙事与社区反馈，**不能**替代同行评审 |

---

## 8. 与旧文档的关系（避免两份真相）

- `PAPER_SKELETON.md` 的贡献表**已被本文 §1 取代**；正文展开时以本文 §2 的三条为准。
- `RELATED_WORK.md`（2026-09-24）仍然有效，本文 §0 是它的**增量**。
- `LITERATURE_SURVEY_2026-10.md` 是引用与 DOI 的权威来源。
- `FINAL_REPORT_2026-09-28.md` / `PAPER_ALIGNMENT_2026-09-28.md` 里"两趟是本文机制"的措辞
  **必须**按本文 §1 改写后再用。
