# 相关工作与定位（评审 R11）

日期：2026-09-24
状态：**首次完成**。评审 R11 明确要求"在查新之前**不作**首创/无创新定论"；本文是该查新的结果。
关联：`doc/REVIEW_HUMBLE_WORK_2026-09-23.md` R11、`doc/PASS_LOWER_BOUND.md`、`doc/FORMAL_MODEL.md`、
`doc/IMPLEMENTATION_GUIDE.md` §12（拆分基线）。

> **本文的立场**：结论对本项目**不利**。四步查新显示核心命题属于已有理论的标准推论，
> 本项目的可辩护贡献不在"新原理"，而在**把该原理落到 `ros2_control` 的具体 API 上、
> 量化它的控制代价、并给出可验证的契约与诚实的负结果**。下文逐条说明依据，不做粉饰。

---

## 1. 逐条重叠分析

| 本项目命题 | 查新结论 | 最接近的已有工作 |
|---|---|---|
| **C1** 单入口模型下，父子双向同周期数据流不可满足；单趟必有一个方向陈旧 ≥ 1 周期 | **教科书级标准结果**，不是本项目发现 | 同步数据流 SDF：**含环且环上无延迟的图没有可行静态调度**（Lee & Messerschmitt 1987）；同步语言：不经过 `pre()` 的环是**因果性错误**（Lustre 1991；Esterel 1992）；Kahn 进程网络：反馈边需要初始 token（Kahn 1974）；Simulink/Modelica：直通（代数）环必须打断或联立求解；LET 的提出动机正是"读写融合 + 任意全局顺序 ⇒ 结果依赖调度顺序"（Giotto 2001） |
| **C2** 单趟陈旧量**恰好等于级联深度 D** | **初等推论**；未找到逐字发表的同一命题。由"每条层次边引入一个单位延迟"复合而来，与采样控制教科书对级联延迟的处理、以及实时任务链延迟分析同构 | Kloda/Bertout/Sorel, ETFA 2018（数据链延迟沿生产者→消费者累积）；Franklin/Powell、Åström/Wittenmark 的采样延迟章节 |
| **C3** 一份线性化、状态阶段正向、命令阶段反向；不需要第二份拓扑数组 | **机制已在 FineMote 论文里**（本项目自己就说是 FineMote 思想）；图论部分是**算法课第一周内容**（后序的反转 = 拓扑序） | **arXiv:2608.04600**（本次已核实，见 §1.1）：`Update`/`Handle` 两阶段、阶段顶点集合、屏障约束、双向数据传递关系；Tarjan 1972（DFS 后序/逆后序）；编译器教材（正向数据流按逆后序、反向按后序迭代，Muchnick 1997）；前向-后向算法（Baum–Petrie 1966；RTS 平滑 1965；Pearl 1988 的树信念传播） |
| **C4** 滞后 → 相位裕度损失 → 可用增益上限 → 带宽上限 | **经典控制理论**：一个采样周期的纯延迟降低相位裕度、限制带宽，是数字控制教科书内容 | Franklin/Powell/Emami-Naeini；Åström & Wittenmark；网络化控制稳定性（Zhang/Branicky/Phillips 2001）。本项目的增量是**把它接到具体的调度量 L 上并做闭环验证** |
| **C5** 上游对双向边静默降级（载荷顺序相关） | **实证贡献**：未找到他人对 `ros2_control` 该行为的公开分析。上游文档只说控制器"can be updated in arbitrary order"（chaining 文档），Issue #2189 显示链方向本身就让用户困惑 | ros2_control chaining 文档 / Issue #2189 / PR #1157 |
| **C6** 可靠性契约：每周期有效性、最旧采样、**整组提交不部分提交** | **未找到任何"跨控制器整组原子提交"的权威来源**。现场总线层面的类比是 EtherCAT 的 SYNC0/SYNC1 + 分布式时钟给出的**一致过程数据镜像**；CANopen 的 SYNC 只协调交换时机，**不**提供应用层事务性提交 | EtherCAT 规范/Beckhoff 文档；CiA 301；IEEE 1588。**注意**："两阶段提交"这个词在事务处理里有严格含义（Gray & Reuter），本项目的"两趟"与之无关，必须在文中区分 |
| **C7** 配置期可判定的拓扑校验 | **工程性质**，标准做法 | 拓扑排序/环检测教科书内容 |
| **C8** 最少趟数刻画 / 两趟"最优" | **平凡分类**（`κ ∈ {0,1,2}`），已撤回"NP 难"与"最优调度"叙事 | 见 `PASS_LOWER_BOUND.md` §0.1 与文首修正框 |

**结论**：C1、C2、C4 是已有理论的推论或教科书内容，C3 的机制来自 FineMote，
C6 在"整组提交"这一具体形式上找不到可直接对齐的先例。
本项目的贡献应重新表述为**应用/工程/量化**类，而不是原理类。

### 1.1 FineMote 原文核实结果（2026-09-24，本次实际抓取）

查新过程最初给出的引用不可信（只有编号没有出处），因此**逐条抓取核实**：

- **论文真实存在**：Wang Xi, Feiran Wei, Mo Deng, Weiheng Lin, Pangkit Fong, Jianping He,
  "Static Timing Orchestration for Tree-Structured Robot Control Firmware",
  **arXiv:2608.04600v1 [cs.RO]，2026-08-05**，上海交通大学自动化系；
  DOI `10.48550/arXiv.2608.04600`。本地 `FineMote论文_文本.txt` 是同一工作的投稿稿
  （"Manuscript 2234 submitted to 2026 IEEE ICRA"）。
- **已在原文中逐项核实的引用（不是转述）**：
  - 两阶段分解：`Update` 阶段 `τ_n^+`（用下层设备的数据更新自身内部状态）、
    `Handle` 阶段 `τ_n^-`（按更新后的状态产生本周期动作并把决策传播到下层）；
  - 可调度阶段集合 `T(D) = {τ_n^+, τ_n^- | d_n ∈ D}`；
  - 数据传递关系：`+` 阶段沿**子→父**传播，`−` 阶段沿**父→子**传播；
  - **相位屏障约束 (1c)：`F(τ_n^+, k) ≤ S(τ_n^-, k)`**；
  - 假设 (D, E) 是**森林**；(IV-C2) 专门分析**同周期**设备树；
  - 决策延迟 = 从叶子侧 `+` 阶段开始到同叶子侧终止 `−` 阶段完成（`F(τ_{n1}^-, ·) − S(τ_{n1}^+, ·)`）。
- **本次未能核实的只有一处**：III-B "Static Device Scheduling" 里那条把阶段排成
  "全部 `+` 按某顺序、然后全部 `−` 反向"的**具体顺序公式**（抓取在 II-D 之后被截断）。
  不过由上列已核实内容可推出该顺序：`(1c)` 要求同设备 `+` 先于 `−`，
  而 `+` 走子→父、`−` 走父→子，因此"后序正向 + 同一顺序反向"正是满足全部约束的构造。
  **如果要在正式文稿里写"FineMote 的 Eq. (2)"，需要再人工核对一次该公式。**

> **对本项目的一个直接影响（必须承认）**：`doc/PASS_LOWER_BOUND.md` §0.1 里我"自行推导"的
> **阶段顶点模型**（`S_v`/`C_v`、屏障边 `S_v → C_v`、状态边 `S_c → S_p`、参考边 `C_p → C_c`）
> **与 FineMote 论文自己的调度模型是同一个模型**。这不是抄袭（当时未读到该文），
> 但它意味着 §0.1 是**独立重导出了已有模型**：价值在于**修正了本项目原先错误的边覆盖模型**、
> 并给出了**相内环不可能**这一属于本项目的新结论；**不能**把阶段顶点建模本身当作贡献。

### 1.2 FineMote 自身的相关工作覆盖（原文第 VI 节 + 本地文本）

其 Related Works 覆盖：URDF/xacro、ros2_control、AUTOSAR、ROS-lite（2 篇）、ROSCH、micro-ROS，
以及工业界的 basic framework；参考文献里另有 Liu & Layland (1973) 与 Orocos (Bruyninckx 2003)。
**没有** LET / 同步语言 / 同步数据流 / OROCCOS-RTT 的对比 / IEC 61499 / 采样延迟的控制代价。
因此本文第 2、3 节给出的对比是**该方向上的净增量**（定位类贡献），而这也正是评审 R11 要求的。

---

## 2. 与 LET / 同步模型的定位（评审要求的重点）

### 2.1 LET 是怎么解决的

LET（Logical Execution Time，Giotto 2001 提出）给每个任务一个**逻辑执行时间**：
读取发生在区间开始、写入发生在区间结束。于是：

- 跨区间边界的所有通信**按定义**是一致的，**区间内的执行顺序对数据流结果完全无影响**；
- `父→子→父` 的环自动获得"恰好一个 LET 区间"的延迟，**不存在顺序问题**；
- 代价：**每条边**都付一个 LET 区间的延迟；需要双缓冲的 LET 变量；需要一个全局同步时基
  （时间触发架构 TTA / PTP）。

### 2.2 本项目是怎么解决的

保留现有的**异步单入口 `update()` API 不变**，只在一个物理控制周期内改变遍历顺序：
状态阶段子先于父、命令阶段父先于子（走同一线性化的反向）。这里"同周期"指**同一个物理控制周期**。

### 2.3 差别是实质性的还是术语性的

**是实质性的，但远小于措辞给人的印象。**

实质差别：

| 维度 | LET | 本项目两趟 |
|---|---|---|
| 同周期如何达成 | **按定义**（读写被时间边界分开） | **按顺序**（依赖图的顺序被固定），正确性依赖拓扑是树/DAG |
| 额外周期延迟 | **每条边一个 LET 区间** | 树边上**不加周期延迟**；代价转为结构性：控制器必须能拆成"收状态的阶段"和"出命令的阶段" |
| 额外缓冲 | 需要 LET 双缓冲变量 | 不需要双缓冲（但 staged group 仍需一组 scratch/committed 缓冲） |
| 时基 | 需要全局同步时基 | 不需要 |
| 适用图 | 任意通信图 | 无环层次（树） |

差别缩小到近乎为零的地方：**对树做两趟遍历，本质上是"用顺序实现 LET 的读写分离"而不使用 LET 变量。**
两者都施加一个全局调度，使读先于写，从而使数据流结果与线程时序无关。
因此本项目最准确的说法是：

> **"面向无环控制层次、基于顺序的轻量 LET 仿真"**（order-based emulation of LET's
> read-at-start/write-at-end discipline without LET variables and without LET-interval latency）。

**不可声称**：把两趟遍历说成新原理——LET 在 25 年前就用构造方式解决了同一个问题；
C1 也正是 Giotto 的原始动机，只是换成了 `ros2_control` 的语境。
**可以声称**：把它**打包**进一个不可修改的既有单入口 API、不需要时基与双缓冲、
不增加周期延迟，并给出代价量化与契约——这是工程适配与量化贡献。

---

## 3. 拆分基线：manager 扩展到底换来了什么（评审 R11 后半）

评审指出："估计/命令显式拆分仍是重要强基线。通用 composite 可以复用同一个内核，
说明组装便利与组提交不必要求修改 manager；Gate B 失败后应解释后来 manager 扩展服务于哪个独立需求。"

**本项目已有这个基线**，而且它**通过了**：`test_composite_library::GenericCompositeController`
是一个**普通控制器插件**（不是 chainable），用 `StagedExecutionGroup::create_library()`
在库宿主里复用**同一个内核**，不需要任何 `ControllerManager` 改动。实测（`test_hierarchy_comparison`）：

| 指标 | 原生 chaining | 通用 composite（库宿主） | 管理器 staged group |
|---|---|---|---|
| `update()` 每周期分配次数 | 10 | **3** | 10 |
| `update()` 中位耗时 | 1.65–2.46 µs | **0.78–2.03 µs** | 2.80–4.39 µs |
| `run()` 每 100 次调用分配 | — | **0** | **0** |
| 需要改 `ControllerManager` | — | **否** | 是 |

也就是说：**调度能力本身不需要 manager 扩展**。那么 manager 扩展服务于什么独立需求？
下表是评审要求的那份解释，写在这里，**结论是"调度能力不在其中"**：

| 独立需求 | 通用 composite（单插件） | 管理器集成路径（多插件） |
|---|---|---|
| **逐控制器生命周期** | 做不到：整个树是一个插件，只能整体 load/configure/activate/cleanup | **可以**：每个控制器独立 load/configure/activate/deactivate/unload |
| **局部激活/停用**（只换一个叶子的实现） | 做不到：要重建整棵树（重新构造 composite、重配参数） | **可以**：只切那一个控制器 |
| **原生接口参与** | 只认领硬件接口，树外控制器/用户无法通过 reference interface 与之交互 | **可以**：每个成员仍导出/认领原生 reference interface，可与组外控制器链式连接 |
| **标准工具可见性** | `list_controllers` 只看到一个插件，故障定位要靠自建诊断 | **可以**：`spawner`/`list_controllers` 逐控制器可见（并且现在诊断带可读节点名） |
| **故障隔离** | 一个成员构造失败 ⇒ 整棵树的 `build_kernel()` 失败 ⇒ 插件激活失败 | **可以**：单个控制器激活失败，其余成员不受影响 |
| **成员异构** | 一个 `GenericCompositeController` 类，行为由数据表驱动 | **可以**：不同成员可以是完全不同的插件类型 |
| **调度能力（同周期、契约、零分配 run）** | **有，且代价更低** | **有，但更贵**（多一次列表遍历 + 每控制器一次虚调用） |
| **CPU 代价** | 最低（0.78–2.03 µs） | 最高（2.80–4.39 µs） |

**诚实结论**：管理器集成的价值在**生命周期与接口的独立性**，
不在调度本身；而代价是可测的（更慢、`update()` 分配更多）。
如果应用接受"整棵树一个插件、整体生命周期、不参与原生 chain"，
那么**不该**改 manager——用库宿主即可。这一点必须写进论文/报告的定位章节，
不能把"必须改 manager"当作前提。

---

## 4. 建议引用（按相关度排序）

1. W. Xi, F. Wei, M. Deng, W. Lin, P. Fong, J. He, "Static Timing Orchestration for Tree-Structured
   Robot Control Firmware", **arXiv:2608.04600v1 [cs.RO], 2026-08-05**, doi:10.48550/arXiv.2608.04600
   ——两阶段 `Update`/`Handle` 机制与阶段顶点调度模型的来源（本地副本 `FineMote论文_文本.txt`
   为投稿稿）；**其 Related Works 未覆盖 LET、同步语言/数据流、Orocos/RTT、IEC 61499、控制延迟代价**。
2. Lee & Messerschmitt, "Synchronous data flow", Proc. IEEE 75(9), 1987, doi:10.1109/PROC.1987.13876
   ——C1 的正式版本：环上必须有延迟。
3. Benveniste, Caspi, Edwards, Halbwachs, Le Guernic, de Simone, "The Synchronous Languages 12 Years
   Later", Proc. IEEE 91(1), 2003, doi:10.1109/JPROC.2002.805826 ——瞬时反馈无延迟 = 因果性错误。
4. Halbwachs, Caspi, Raymond, Pilaud, "The synchronous data flow programming language LUSTRE",
   Proc. IEEE 79(9), 1991, doi:10.1109/5.97300 ——`pre()` 单位延迟算子。
5. Henzinger, Horowitz, Kirsch, "Giotto: A Time-Triggered Language for Embedded Programming",
   Proc. IEEE 91(1), 2003, doi:10.1109/JPROC.2002.805825 ——LET 起源。
6. Kirsch & Sokolova, "The Logical Execution Time Paradigm", in Advances in Real-Time Systems,
   Springer 2012, doi:10.1007/978-3-642-24349-3_5 ——LET 定义与代价模型。
7. Wang et al., "Optimizing Logical Execution Time Model for Both Determinism and Low Latency",
   RTAS 2024, arXiv:2310.19699 ——LET 的延迟代价量化（用于诚实定位）。
8. Tarjan, "Depth-First Search and Linear Graph Algorithms", SIAM J. Comput. 1(2), 1972,
   doi:10.1137/0201010 ——逆后序即拓扑序。
9. Caspi & Pouzet, "Synchronous Kahn Networks", ICFP 1996, doi:10.1145/232627.232651。
10. Kopetz & Bauer, "The Time-Triggered Architecture", Proc. IEEE 91(1), 2003,
    doi:10.1109/JPROC.2002.805821 ——LET 依赖的同步时基假设（本项目不需要）。
11. Yoong, Roop, Vyatkin, Salcic, "A Synchronous Approach for IEC 61499 Function Block
    Implementation", IEEE Trans. Computers 58(12), 2009, doi:10.1109/TC.2009.128
    ——最接近的"重定义传播顺序使同周期行为确定"的组件组合先例。
12. Kloda, Bertout, Sorel, "Latency analysis for data chains of real-time periodic tasks",
    ETFA 2018, doi:10.1109/ETFA.2018.8502498 ——最接近"延迟 = 深度"的已发表界。
13. Franklin, Powell, Emami-Naeini, *Feedback Control of Dynamic Systems*；
    Åström & Wittenmark, *Computer-Controlled Systems* ——采样+计算延迟的相位裕度损失。
14. ros2_control "Controller Chaining / Cascade Control" 文档
    （`controller_manager/doc/controller_chaining.rst`）——被适配的 API 约束
    （"controllers can be updated in arbitrary order"）。
15. Soetens, OROCOS/RTT（博士论文 2006；`https://www.orocos.org/rtt/`）
    ——机器人领域的另一条路线：端口解耦、不做全局遍历。

---

## 5. 不确定性与不可引用项

- **FineMote 的 arXiv 记录已核实**（`arXiv:2608.04600v1`，2026-08-05，6 作者，SJTU），
  本次实际抓取了 abs 与 HTML 正文并逐项核对了 §1.1 列出的内容。
  **仍未核实**的是：III-B 的具体顺序公式、正式发表venue（arXiv 是 v1 预印本，
  本地稿件写的是投稿 ICRA 2026）。引用前请再核一次这两项。
- **C2（陈旧量 = 深度 D）没有逐字来源**，不要归因给任何人；按本项目在明确假设下的推论来写，
  并写清"深度"数的是边还是节点、单位是控制周期。
- **"跨控制器整组原子提交"没有找到权威来源**（现场总线的 EtherCAT 一致过程镜像是**层面不同**的类比）。
  本项目可以声称"我们定义并验证了这个契约"，**不能**声称"这是行业标准做法"。
- **未读到原文的条目**：Simulink 代数环页面（自动抓取返回 403）、AUTOSAR Timing Extensions 的
  LET 条款（PDF 未能打开）、Orocos `updateHook`/传播语义（依据官方总览文档，未读源码）。
  这些条目在正式引用前需要一次人工核对。
- **"最少趟数"相关的新结论**（`κ ∈ {0,1,2}`、相内环不可能）见 `PASS_LOWER_BOUND.md` §0.1，
  那部分是本项目自己的推导 + 穷举，不属于查新对象。

---

## 6. 对"可以/不可以声称"清单的增量

**不可声称（新增）**：

- ❌ **首次提出**"双向同周期不可满足"或"反馈环需要单位延迟"——这是同步数据流/同步语言/Kahn 网络/
  代数环的标准结果（1987 年前后）。
- ❌ 两趟、正反遍历是**新原理**——FineMote 的 `Update`/`Handle` 就是这个机制；
  逆后序是拓扑序是教科书事实。
- ❌ 整组原子提交是**行业标准做法**——未找到来源；EtherCAT 是不同层面的类比。
- ❌ 两趟模式继承了"整组提交不部分提交"——**实测不成立**（见 `IMPLEMENTATION_GUIDE.md` §12.1 #4）。
- ❌ 本项目"比 LET 更好"——两者解决同一问题；本项目只是**不需要时基/双缓冲、不增加周期延迟**，
  代价是要求可分解的阶段与无环层次。

**可以声称（新增/强化）**：

- ✅ 把上述原理**落到不可修改的 `ros2_control` 单入口 API** 上，并证明两趟在该 API 内充分
  （`FORMAL_MODEL.md` 定理 3、`PASS_LOWER_PASS.md` §0.1 阶段顶点模型 + 465/465 穷举）。
- ✅ **代价量化**：滞后 → 相位裕度 → 增益/带宽上限（解析 + 闭环），
  以及两趟自身的 CPU 代价（**实测 1.64×**，见 `IMPLEMENTATION_GUIDE.md` §12.1 #5）。
- ✅ **契约与验证**：每周期有效性、最旧采样、整组提交不部分提交（仅 staged group），
  以及两趟模式**没有**该保证这一反向结论。
- ✅ **诚实的负结果集**：不更快、`update()` 非零分配、两趟更慢、Gazebo 无跟踪收益。
- ✅ **一份 LET/同步语言视角的定位分析**（本文），这正是评审要求"装置对比"的产物。
