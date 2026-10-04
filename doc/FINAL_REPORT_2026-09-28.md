# 最终报告：ros2_control 分层树状控制的双向同周期调度与编译期元编程

> ⚠️ **2026-10-03 修正（阅读前必看）**：本文件写于 `LITERATURE_SURVEY_2026-10.md` 之前，
> 其中"两趟调度/双向同周期是本文机制或贡献"的叙事**已被取代**——FineMote
> （arXiv:2608.04600）**§III-B 式 (2)** 已给出同一规则（本地 PDF 逐行核对：
> `log/finemote_paper.txt:315`），且上游 `ros2_control` 也**已经**维护一条树导出的线性化
> （issue #853 已关闭；本地源码 `controller_manager.cpp` 的 `controller_sorting()` 即此）。
> 贡献列表、摘要与相关工作请以 **`PAPER_REPOSITIONING_2026-10.md`** 为准；
> 本文件仅作为**过程记录**保留，引用其结论前必须按重定位文档改写。

日期：2026-09-28　分支：`humble-work`（fork `yuan669-gif/ros2_control`）　提交数：31
代码基线：上游 ROS 2 Humble（`ros2_control` 源码树；本仓库 `.git` 的根提交 `b1bf616` 已包含上游源码 + 第一版内核）

> **给师兄的阅读建议**：§0 是一页速览；想核对结论就看 §4 的证据表（每条都指向文件/测试/数字）；
> §5 是"明确不能声称"的清单，请重点看——这份工作的可信度主要来自这里；§7 是与 FineMote 论文的
> 对照现状与下一步优先级。
>
> 配套文档（都在 `doc/`）：`USER_GUIDE.md`（怎么用）、`ONBOARDING.md`（概念入门）、
> `CODEBASE_TOUR.md`（怎么读代码）、`TESTING_GUIDE.md`（怎么跑/怎么复现）、
> `DEVELOPMENT_HISTORY.md`（开发经过与被推翻的结论）、`PAPER_ALIGNMENT_2026-09-28.md`（与论文逐条对照）、
> `IMPLEMENTATION_GUIDE.md` §12（还没做的事）。

---

## §0 一页速览

### 问题

ros2_control 的级联控制（chassis → wheel → tire）里，父子之间存在**同一周期内的双向依赖**：

```
        reference（父 → 子）
chassis ───────────────▶ wheel ───────────────▶ tire
   ▲                                             │
   └────────────── state（子 → 父）──────────────┘
```

上游 `ControllerManager::update()` **一次遍历**控制器列表，每个控制器只 `update()` 一次。
因此无论列表怎么排序，**总有一个方向读到上一周期的值**，陈旧量 = **级联深度 L**（`doc/FORMAL_MODEL.md` 定理 1/2）。
这不是"不优雅"，而是控制意义上的**传输延迟**：直接损失相位裕度

```
ΔPM = 360 · f_c · L · Δt   [deg]        （doc/CONTROL_COST_OF_LAG.md）
```

### 做法（四条，都有代码与测试）

1. **两趟执行**：在**同一份**控制器顺序上跑两趟——反向走"状态趟"（子先于父），正向走"命令趟"（父先于子）。
   两向都拿到本周期的数据，且不需要时基/双缓冲/第二份拓扑。
2. **可靠性契约**：每帧数据带 `cycle / sample_ns / valid / fault_code`；复合节点不得把派生数据重新盖章成
   "更新"；**失败时整组不提交**（一个硬件命令都不写）；故障下兄弟节点看到同一套输入。
3. **编译期元编程**：拓扑是类型（环**写不出来**）、接口带物理量纲、端口只声明一次（字符串表与 `Contract`
   由类型**生成**）、从 binding 类型得到可 `static_assert` 的 manifest（节点/端口/硬件接口需求）。
4. **落到 ros2_control Humble**：两趟与执行组都是**可选**能力（不影响上游默认语义）、
   执行状态用**一个不可变"执行代"**发布、可选的 all-or-nothing 激活回滚、编译内置控制器按类型字符串
   走**同一条**加载/生命周期路径。

### 结论（实测，不是推算）

- **两趟把滞后从 1 个周期降到 0**：真实 Gazebo + 真实 `ControllerManager`，50 Hz 下单趟滞后 1 周期
  （20 ms）、两趟 0 周期；20 Hz 下单趟仍是 1 周期但墙钟代价 50 ms（`doc/GAZEBO_CASE_STUDY.md` §3.2）。
- **两趟是"最少趟数"意义下的最优**：规范两趟调度满足阶段顶点图的**全部边**，
  `n ≤ 4` 的全部有根树 × 全部边标注 = **465/465**；单趟可行 ⇔ 合并要求图无环（`doc/PASS_LOWER_BOUND.md`）。
- **滞后可以换算成控制指标**：`ΔPM = ω_c·τ = 360·f_c·L·Δt`，并有闭式验证与 7 个控制意义用例
  （`doc/CONTROL_COST_OF_LAG.md`、`hierarchical_control/test/test_stale_state_cost.cpp`）。
- **编译期保证是真的编译期**：12 个"必须编译失败"的源文件组成负向语料，诊断必须包含指定子串
  （`hierarchical_control/test/static_topology_negative/`）。
- **管理器自身并发干净**：给真实 `ControllerManager` 插桩的 TSan 先测出 **4 条数据竞争**（全在上游握手字段），
  改成原子 + release/acquire 后归零；依赖库未插桩（见 §5）。

### 明确不声称（详见 §5）

不更快（两趟 CPU 更贵）；跟踪误差不一定改善；`update()` 整体仍会分配（上游代码）；
**没有 WCET 分析**（只有"你声明 WCET，我按论文的充分条件检查"）；执行组**没有 ROS 服务/参数入口**；
覆盖层 `controller_manager` 的 **ABI 与系统安装不同**；物理总线原子提交不在保证内。

---

## §1 问题：为什么上游不够

**上游机制**（实测而非推断，`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md`）：
`controller_manager` 每个周期按列表顺序调一次 `controller->update()`；chaining 只提供"父写子的
reference 接口"，**不提供**"父读子的新状态"；`test_upstream_ordering.state_edge_cannot_be_bound_on_humble`
把这一点钉成事实。

**形式化**（`doc/FORMAL_MODEL.md`）：把"同周期双向依赖"写成有向图要求；定理 1/2：单趟执行下，
父读到的子状态滞后 `L` 个周期（`L` = 级联深度），单趟可行 ⇔ 合并要求图无环。

**代价**（`doc/CONTROL_COST_OF_LAG.md`）：滞后 = 传输延迟 `τ = L·Δt`，相位裕度损失
`ΔPM = 2π·f_c·L·Δt`（度制 `360·f_c·L·Δt`）。示例：`kp=10, kd=600` → `f_c ≈ 95.5 Hz`、`Δt = 1 ms` 时，
`L=1` 就要吃掉可观裕度；低带宽外环（`f_c ≈ 9.5 Hz`）几乎无感。

---

## §2 方法

### 2.1 两趟执行 = FineMote 的静态策略（式 (2)）

论文（arXiv:2608.04600v1）§III-B 的策略是：每个周期桶内，先按**子→父**顺序跑全部 `Update`，
再按**父→子**跑全部 `Handle`。我们的实现就是这个序列：

| 论文 | 本项目 |
|---|---|
| 全部 `τ+`（子→父） | `staged_execution_group.hpp:248` 后序**正向**遍历 |
| 全部 `τ−`（父→子） | `staged_execution_group.hpp:330` 逆后序遍历 |
| 管理器模式 | `controller_manager.cpp` `update()`：反走状态趟、正走命令趟（按速率桶各一次） |

**为什么一份顺序就够**：后序的逆序是拓扑序，两趟方向相反，所以同一份线性化同时满足两向依赖。
**为什么两趟是最少**：阶段顶点模型（每个节点拆成状态/命令两个顶点）+ 全树穷举 465/465
（`doc/PASS_LOWER_BOUND.md`、`research/stage_graph/check_stage_graph.py`）。

### 2.2 三条工程支柱

1. **数据契约**（`staged_controller_interface.hpp`）：`StagedFrame{cycle, sample_ns, valid, fault_code}`；
   `sample_ns` 由执行组用**最旧**的子样本时间覆盖 → 复合节点无法"重新盖章"；失败**不提交**任何 sink。
2. **执行路径与准入**：模式 + 成员表 + 执行组 = **一个不可变执行代**（`ExecutionGeneration`），
   `update()` 每周期**一次** `atomic_load`，每次配置变更**一次** `atomic_store`；
   准入会把"跨模式边、跨速率桶边、列表顺序反了、同一对象挂两名、非整除速率"逐项拒绝并点名。
3. **编译期描述**：`static_topology`（环写不出来）→ `dimensional_interfaces`（量纲进类型）→
   `topology_contract`（`Port`/`Contract`/`BoundNode`/`compose`）→ `typed_ports`（6 张表，只声明一次，
   字符串与 `Contract` 由类型生成）→ `topology_binding`（类型 binding → 可运行执行组）→
   `static_manifest`（可 `static_assert` 的 manifest）。

### 2.3 与 FineMote 的关系（一句话）

**调度思想一致**（同一式 (2)、同一两阶段拆分、同一"静态决定顺序"的目标）；**顺序来源不同**
（论文用全局对象初始化序 + C++17 部分有序初始化，我们用类型声明 + 列表序准入）；
**分析层不同**（论文有 WCET/期限判定与时延上界；我们补了"声明 WCET"口径的充分条件检查，
但没有 WCET 分析与时延上界）。逐条对照与判定见 `doc/PAPER_ALIGNMENT_2026-09-28.md`。

---

## §3 实现地图

新增一个包 + 改造管理器。两个口径都给：**相对本分支首个提交**（`b1bf616..HEAD`）为
123 文件/+17134/−1690；**相对上游 Humble**（`git fetch origin humble && git diff --stat 469f3055 HEAD`）为
**149 文件/+36454/−98**（后者才是"这份工作在上游之上加了什么"的完整口径）。

| 位置 | 内容 | 规模 |
|---|---|---|
| `hierarchical_control/`（**新包**，不依赖管理器） | 内核（执行组、两趟接口、计划）+ 类型层（拓扑/量纲/契约/端口/binding）+ 描述层（manifest） | 3726 行头文件 |
| `controller_manager/` | 管理器接入：两趟按桶调度、执行组安装、执行代、可选 all-or-nothing 回滚、编译内置控制器注册、按类型加载 | 20 个文件改动 |
| `hierarchical_control/test/`、`controller_manager/test/` | 单元/集成测试 + 负向编译语料 + 成本与 TSan 脚本 | 15535 行 |
| `case_study/` | 真实 Gazebo 闭环案例（wheel/chassis 控制器、配置、测量脚本、原始日志） | 1129 行 + 日志 |
| `research/` | 形式化小工具（阶段图检查、最少趟数搜索、上游排序模拟、被否决的拓扑变体） | 1029 行 |

**三种用法**（用户视角，详见 `doc/USER_GUIDE.md`）：

| | A 库模式 | B 管理器 + 两趟 | C 管理器 + 执行组 |
|---|---|---|---|
| 形态 | 一个普通插件托管整棵树 | 若干 chainable 控制器 | 若干控制器组成整组 |
| 替换系统 `controller_manager` | 不需要 | 需要 | 需要 |
| 开启方式 | YAML `type:` | YAML `two_phase_execution: true` | 仅 C++ API |
| 整组提交 + 帧语义 | ✅ | ❌ | ✅ |
| 逐控制器生命周期 | ❌ | ✅ | ✅ |

---

## §4 验证证据

### 4.1 理论/最优性

| 结论 | 证据 | 数字 |
|---|---|---|
| 单趟滞后 = 深度 | `FORMAL_MODEL.md` 定理 1/2；`test_two_phase_execution.single_pass_lags_by_depth` | 深度 1–6 全对 |
| 两趟两向都同周期 | `test_execution_group`、`test_hierarchy_comparison` | 每节点 `last_state_cycle == last_command_cycle` |
| 规范两趟满足阶段图全部边 | `test_pass_lower_bound.canonical_two_phase_satisfies_the_stage_graph` + `research/stage_graph/check_stage_graph.py` | **465/465**（n≤4 全部有根树 × 全部边标注） |
| 单趟可行 ⇔ 合并图无环 | 同上（同一脚本同时判定） | 465/465 一致；可行 145 / 不可行 320 |
| 最少趟数 | `test_pass_lower_bound` 7 例 | 双向对恰好 2 趟且与深度无关；`κ(D) ∈ {0,1,2}` |
| 停滞代价 | `test_stale_state_cost` 7 例 | 相位裕度定律二阶精度；深度 2/3 越过稳定边界；执行器饱和掩盖失稳但不修复 |

### 4.2 内核语义（失败与新鲜度）

| 结论 | 证据 |
|---|---|
| 失败绝不部分提交 | `test_execution_group.command_failure_and_nan_never_commit`、`state_failure_aborts_before_command_phase` |
| 复合节点不能重新盖章新鲜度 | 同文件 `composite_cannot_restamp_derived_sample_time` |
| 样本超龄被拒 | 同文件 `sample_age_limit_is_enforced` |
| 配置/结构错误带原因被拒 | 同文件 `configuration_errors_are_rejected`、`structural_errors_are_rejected` |
| 每阶段每周期恰好一次 | 同文件 `phase_order_and_single_call_per_cycle` |

### 4.3 管理器行为

| 结论 | 证据 | 数字 |
|---|---|---|
| 两趟开关/成员是**一个执行代** | `test_two_phase_execution.execution_state_is_published_as_one_generation` 等 3 例 | `update()` 1 次 load/周期；被拒请求不发布（单调 id 断言） |
| 运行中安装执行路径被拒 | `test_runtime_reconfiguration` 3 例 | 周期在飞 → `ERROR` 且不发布；移除始终允许 |
| 执行组部分成员**惰性** | `test_staged_execution_group.a_partial_membership_is_inert` | 部分状态下 5 个周期成员计数全不变；补齐后 3 周期 +3 |
| 执行组要求 all-or-nothing 已开启 | `test_two_phase_execution.installing_a_staged_group_requires_atomic_activation` | 关闭时安装返回 ERROR |
| **回滚把硬件模式也换回去** | `test_atomic_activation.the_rollback_switches_the_hardware_mode_back` | mock 计数器 **+202**；关掉该步的探针 **+101** |
| **为切 chained mode 被重启的控制器被恢复** | `a_restart_for_chained_mode_is_brought_back_by_the_rollback` | 关掉恢复逻辑的探针：`'\x2'`(INACTIVE) vs 期望 `'\x3'`(ACTIVE) |
| 编译内置控制器与插件同路径 | `test_static_controller_registry` 9 例 | 两次加载是两实例；准入判定一致；注册后封印（`add()` 抛异常） |
| 速率分桶（论文 §III-B） | `test_two_phase_execution` 3 例 | 半速桶 10 周期跑 5 次、周期 ×2、原生循环 0 次；跨桶边被拒 |
| 声明 WCET 的可调度性（论文 Thm 2 对应物） | 同文件 2 例 | 舒适集 U=0.30 ≤ 0.828 判 SUFFICIENT；9 ms+9 ms ⇒ U=1.35 报 NOT MET 且**不拒绝**；未声明 ⇒ "未检查"（`complete=false`） |

### 4.4 编译期保证

| 结论 | 证据 | 数字 |
|---|---|---|
| 非法拓扑/量纲/所有权/边**编译不过** | `test/static_topology_negative/` + 驱动脚本 | 12 个必须失败（诊断含指定子串）+ 2 个必须通过 |
| manifest 可 `static_assert` | `test_static_manifest` 5 例 | 含"父链进入环"的负向用例 |
| 端口只声明一次 | `test_typed_ports` 8 例 + `test_typed_tree` 3 例 | 字符串表与 `Contract` 由类型生成；顺序敏感 |
| 历史缺陷回归 | `test_contract_regression` 12 例 | 含"`void*` 往返丢地址调整"那个真 bug |

### 4.5 并发

| 结论 | 证据 | 数字 |
|---|---|---|
| 管理器自身无 data race | `controller_manager/test/run_tsan_real_manager.sh` | 先测出 **4 条**（`do_switch`、`activate_asap`、两个列表下标）→ 改原子 + release/acquire 后 **0** |
| 发布会话协议无 race | `hierarchical_control/test/run_tsan_publish_protocol.sh` | PASS |
| **边界** | 只插桩 `controller_manager` | rclcpp/lifecycle/hardware_interface/FastRTPS **未插桩**，其内部竞争看不见 |

### 4.6 真实闭环与成本

| 结论 | 证据 | 数字 |
|---|---|---|
| 真实 Gazebo 上滞后从 1 周期降到 0 | `doc/GAZEBO_CASE_STUDY.md` §3.2（`case_study/`） | 50 Hz：单趟 1 周期（20 ms）→ 两趟 0；20 Hz：单趟 1 周期（50 ms）→ 两趟 0 |
| 三方案输出一致、故障下兄弟一致 | `test_hierarchy_comparison` 10 例 | 20 周期输出逐值一致；两叶 fork 的兄弟一致性 |
| 分配 | 同上 `reported_overhead_and_allocations` | `update()` 每次分配：原生 chaining **10** / 通用 composite **3** / 执行组 **10**；**执行组自身 `run()` 100 次 = 0** |
| 编译成本 | `measure_compile_cost.py`、`measure_binding_cost.py` | 深链 3.35/4.07/5.89/9.63 ms/节点（深度 8/16/32/64）；绑定层：链 **130–172 ms/节点** vs 同规模树 **2–25 ms/节点**（方向稳定、倍数不稳） |
| 被否决的优化 | `research/static_topology_variants/measure_variants.py` | "沿父链 O(d)" 实测 **慢 3.24 倍**（0.17 s → 0.55 s，GGC 13 → 20 MB） |

### 4.7 本次最终测试记录（2026-09-28，本机 2 核）

| 套件 | 结果 |
|---|---|
| `hierarchical_control` ctest | **13/13 程序通过**（12 个 gtest = 81 例 + 15 文件负向语料） |
| `controller_manager` ctest（排除 4 个启动/超时敏感项） | **21/21 通过** |
| `test_controller_manager_srvs`（单独跑） | 14/14 |
| `test_hardware_spawner`（单独跑） | 8/8 |
| `test_spawner_unspawner`（单独跑） | 21/22（1 个**既有**服务发现抖动，见 `TESTING_GUIDE.md` §6） |
| `test_two_phase_execution` | 27/27（含本轮 2 个 WCET 用例） |

---

## §5 明确不能声称（请重点看）

| 不能声称 | 原因 / 证据 |
|---|---|
| 更快（CPU 时间） | 两趟多一遍遍历，**结构性**更慢；`doc/WHY_NO_SPEEDUP.md`、`HIERARCHY_FAIR_COMPARISON.md` |
| 跟踪误差一定改善 | 低带宽回路本来不敏感（`ΔPM ∝ f_c·Δt`）；有闭式与数值两类证据 |
| `update()` 零分配 | 执行组自身 0；整体仍有上游来源的分配（10/3/10 对比表） |
| **WCET / 期限保证** | 没有 WCET 分析。只有"成员声明 `wcet_ns` → 按桶套 Liu–Layland 充分条件"的报告；声明缺失 ⇒ "未检查"；界不满足 ⇒ 只报告不拒绝；也不建模通信/总线负载 |
| 树内决策时延**上界**（论文 Thm 3） | 没有推导；因此**跨速率桶的参考边被拒绝**，而不是给一个有界延迟 |
| 完整静态初始化 ROS controller | 只做到"编译期描述 + 按类型字符串加载"；node/参数/URDF/生命周期/loan 都在运行期（`COMPILETIME_CONTROLLER_RESPONSE_2026-09-26.md` §1.1 逐项列了依赖何时才存在） |
| 行为像 Thm 1 的初始化序保证 | 我们靠类型声明，不是靠全局对象初始化序；目的相同、机制不同 |
| 任意时刻都能安全重配 | 模式/成员/计划是一个执行代，但**控制器列表**仍是上游双缓冲；安装执行路径在周期在飞时被拒 |
| 事务性激活（无条件） | all-or-nothing 是**可选**开关；只覆盖"本次 switch 触碰到的"控制器；用户显式 deactivate 不撤销 |
| 执行组有 ROS 接口 | 只有 C++ API（没有服务/参数），要标准 `ros2_control_node` + `spawner` 部署就用两趟模式 |
| 覆盖层可以随便混装 | `ControllerManager` 新增了数据成员 ⇒ **ABI 变了**；任何自行实例化它的包（如 `gazebo_ros2_control`）必须一起重编；`case_study/` 用 apt 版属研究脚手架 |
| 依赖库内部无 TSan 问题 | 未插桩 |
| 物理总线原子提交 | `prepare/perform` 在 Humble 本就是两段式，无事务语义 |
| 与论文的 µs 数字可比 | 论文是嵌入式固件（FINS-ROV：53.64 µs vs 1610 µs、抖动 5.34 vs 18.87 µs）；我们是 Linux 周期/分配指标，**量纲不同** |

---

## §6 工程质量与复现

**构建**（详见 `doc/USER_GUIDE.md` §1、`doc/TESTING_GUIDE.md` §2）：

```bash
source /opt/ros/humble/setup.bash
# 只做库模式（不碰系统管理器）
colcon build --packages-select hierarchical_control
# 要用管理器新能力（覆盖层；注意 ABI）
colcon build --packages-up-to controller_manager hierarchical_control
source install/setup.bash
ros2 pkg prefix controller_manager        # 必须指向本工作空间的 install
export LD_LIBRARY_PATH="$PWD/build/controller_manager:$PWD/build/hierarchical_control:$LD_LIBRARY_PATH"
ctest --test-dir build/controller_manager -E "test_controller_manager_srvs|test_spawner_unspawner|test_hardware_spawner|test_ros2_control_node"
```

**复现头条数字的最短路径**：

```bash
python3 research/stage_graph/check_stage_graph.py                       # 465/465
python3 research/pass_lower_bound/search_min_passes.py                  # 最少趟数
python3 hierarchical_control/test/measure_binding_cost.py --runs 5      # 树 vs 链编译成本
./build/hierarchical_control/test_stale_state_cost                      # 相位裕度定律
./build/controller_manager/test_two_phase_execution --gtest_filter='*bucket*:*wcet*:*declared*'
bash controller_manager/test/run_tsan_real_manager.sh                   # 首次会自动建 TSan 树（约 10–15 min）
bash case_study/scripts/phase_b.sh 50 true 20                           # 真实 Gazebo 闭环（含日志）
```

**已知环境性抖动**（不是代码问题，判定方法见 `TESTING_GUIDE.md` §6）：
`test_spawner_unspawner` 的服务发现超时、`test_hardware_spawner.spawner_with_later_load_of_robot_description`
的 2.5 s wall-timer 假设、`test_controllers_chaining_with_controller_manager` 的 `internal_counter` 计时断言。
三处都做过改动前后对照，失败签名相同。

---

## §7 与论文对照的现状 + 下一步

**现状**（`doc/PAPER_ALIGNMENT_2026-09-28.md` 有逐条判定表）：

| 论文要素 | 我们的状态 |
|---|---|
| 双向依赖建模、两阶段拆分、式 (2) 顺序、静态决定顺序、最小运行期开销 | **一致** |
| 顺序来源（Thm 1 初始化序） | **目的一致、机制更强**（类型声明 + 准入校验；代价是不能声称实现了 Thm 1） |
| 周期分桶（§III-B） | **2026-09-28 已实现**（可整除的更低速率进自己的桶；跨桶边拒绝） |
| 可调度性（Thm 2） | **已补"声明 WCET"口径的充分条件检查**；仍缺 WCET 分析与通信/总线负载建模 |
| 时延上界（Thm 3 / Cor 1 / Cor 2） | **缺失**（我们给的是更强的确定性结论 + 控制代价定律） |
| 事件触发通信周期化抽象（Lemma 1） | 不适用（DDS + `read()`/`write()` 边界，不建模） |
| µs 级固件评测 | 不可比（量纲不同） |

**下一步按价值排序**：

1. **补 Thm 3 的对应物（周期数单位）**：为"调和周期 + 跨桶"情形推导一个上界，
   把跨桶参考边从"拒绝"变成"有界接纳"——这是当前唯一的**能力性**缺口。
2. **给执行组一个 ROS 入口**（服务或参数 + 仲裁）：让用法 C 也能用标准 `ros2_control_node` 部署。
   需要先解决"准入判定对着控制器列表"的一致性问题。
3. **把控制器列表纳入执行代**：去掉"安装执行路径必须停止期"这条约束（需要改上游列表发布协议）。
4. **依赖库 TSan**（rclcpp/lifecycle/hardware_interface/FastRTPS 插桩，磁盘与时间成本高）。
5. **论文写作**：把"确定性结论（两向 0 滞后、滞后=深度）+ 控制代价定律 `ΔPM`"写成主线，
   与论文的 µs 数字分开表述；`PAPER.md` 里若有与 §5 冲突的旧表述，以 §5 与
   `IMPLEMENTATION_GUIDE.md` §12 为准。

---

## §8 交付物索引

| 类型 | 位置 |
|---|---|
| 使用指南（装/开/选模式/排障） | `doc/USER_GUIDE.md` |
| 概念入门 + 阅读路径 + 术语表 | `doc/ONBOARDING.md` |
| 源码导读（一个周期的 trace、四层阅读顺序） | `doc/CODEBASE_TOUR.md` |
| 测试与复现（含已知 flaky 的判定） | `doc/TESTING_GUIDE.md` |
| 开发经过（六个+三阶段、被推翻的结论清单） | `doc/DEVELOPMENT_HISTORY.md` |
| 与 FineMote 论文逐条对照 | `doc/PAPER_ALIGNMENT_2026-09-28.md` |
| 权威实现状态 / 未做清单 | `doc/IMPLEMENTATION_GUIDE.md`、`doc/HANDOFF_MANUAL.md` §11 |
| 理论 | `doc/FORMAL_MODEL.md`、`PASS_LOWER_BOUND.md`、`CONTROL_COST_OF_LAG.md`、`BIDIRECTIONAL_EDGE_ANALYSIS.md` |
| 工程计量 | `doc/HIERARCHY_FAIR_COMPARISON.md`、`WIRING_COST_ANALYSIS.md`、`COMPILE_COST.md`、`WHY_NO_SPEEDUP.md` |
| 元编程 | `doc/METAPROGRAMMING_CONTRACT.md`、`TOPOLOGY_CONTRACT_JOIN.md`、`PORT_DIMENSIONS.md` |
| 真实案例 | `doc/GAZEBO_CASE_STUDY.md`、`case_study/` |
| 评审与回应（原始记录） | `doc/REVIEW_*.md`、`doc/*RESPONSE*.md` |
| 代码 | `hierarchical_control/`（新包）、`controller_manager/`（改造） |
| 脚本 | `hierarchical_control/test/measure_*.py`、`*run_tsan*.sh`、`research/*/*.py`、`case_study/scripts/` |
