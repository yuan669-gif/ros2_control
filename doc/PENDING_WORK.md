# 待做事项（Backlog）

日期：2026-09-28　分支：`humble-work`（31 个提交）　配套：`FINAL_REPORT_2026-09-28.md`、`PAPER_ALIGNMENT_2026-09-28.md`

> **这份文档是唯一的待办清单**，把散落在 `IMPLEMENTATION_GUIDE.md` §12、`PAPER_ALIGNMENT` §6/§7、
> `TESTING_GUIDE.md` §6、`USER_GUIDE.md`"不能声称"里的东西收拢到一起，每条都写：
> **为什么做 / 现在什么状态 / 怎么算做完 / 有什么前置**。估计工时都是**量级估计**（本机 2 核、
> 单包重编 3–17 min），不是承诺。
>
> 「已完成」的项不再列在这里，历史见 `DEVELOPMENT_HISTORY.md`；权威实现状态见 `IMPLEMENTATION_GUIDE.md`。

---

## 0. 优先级总览（推荐顺序）

| 序 | 事项 | 为什么排这个位置 | 量级 | 前置 |
|---|---|---|---|---|
| **P0-1** ✅ | **把"管理器模式 + 两趟"切成独立最小分支**（§1） | **已完成 2026-10-03**：分支 `feature/two-phase-manager`（基点上游 `469f3055`），15 文件 +2862 行，22 例测试全绿。说明见 `TWO_PHASE_BRANCH.md` | 1–2 天 | 无 |
| **P0-0** ✅ | **编译期可确定性审计 + 把 manifest 不变式接到入口**（§1.9） | **已完成 2026-10-03**：13 项审计 + 落地价值最高的 F1+F2（负向编译语料 16/16、`hierarchical_control` ctest 13/13）。见 `COMPILETIME_AUDIT_2026-10.md` | 0.5 天 | 无 |
| **P0-2** | 补论文 Thm 3 的对应物（周期数单位的时延上界）→ 把跨速率桶边从"拒绝"改成"有界接纳"（§2.1） | 目前**唯一的能力性缺口**；补上后两趟路径对多速率拓扑完整 | 3–5 天（含推导） | 需要先写清模型假设 |
| **P1-1** | 给执行组一个 ROS 入口（服务或参数 + 仲裁）（§2.2） | 决定"用法 C"能不能用标准 `ros2_control_node` 部署 | 2–4 天 | 需先定"准入判定与控制器列表一致性"协议 |
| **P1-2** | 把控制器列表纳入执行代（§2.3） | 去掉"安装执行路径必须停止期"这条约束 | 1 周（动上游列表发布协议） | 无 |
| **P1-3** | 依赖库 TSan（rclcpp/lifecycle/hardware_interface/FastRTPS 插桩）（§3.1） | 现在只能声称"管理器自身无 race" | 1–2 天（含磁盘/时间） | 需要 ≥3 GB 空间与较长构建 |
| **P1-4** | 最小 CI + 文档英文化（§4.2/§4.4） | 决定这份工作能不能被别人接手/给上游 | 1–2 天 | 无 |
| **P2\*** | 其余（`Spec::parents` 配置入口、多执行组、缓存一致性、flaky 根治、ctest 超时、ABI 版本化） | 有价值但不阻塞任何结论 | 各 0.5–2 天 | 无 |

"P0"=不做会挡住后面的事；"P1"=明显提升可用/可信度；"P2"=打磨。

---

## 1. P0-1：把"管理器模式 + 两趟"做成独立最小分支

> **状态：已完成（2026-10-03）**。分支：`feature/two-phase-manager`，基点上游 `469f3055`。
> 交付物：`controller_interface/two_phase_controller_interface.hpp`（接口头搬进上游包，**无新包**）、
> `controller_manager` 的两趟调度 + 速率分桶 + 准入（约 950 行）、
> `controller_manager/test/test_two_phase_execution.cpp`（22 例，ctest 通过）、
> `two_phase_example_controller`（示例控制器）、`two_phase_demo`（可运行 demo：URDF+YAML+launch）、
> 分支说明 `TWO_PHASE_BRANCH.md`、用户文档 `controller_manager/doc/two_phase_execution.md`。
> **验收对照见 `TWO_PHASE_BRANCH.md` §5**；下面保留当时的计划原文，作为"范围/边界"的记录。

> 用户的判断：**这种用法最接近原生 ros2_control、改动最小、最适合用户直接用**。
> 我同意，并且建议按下面的方式切（**最小补丁**，不是把 31 个提交 cherry-pick 过去）。

### 1.1 为什么值得单独开分支

1. **用户可单独采用**：它**不需要**新包、不需要 staged group、不需要类型层；用户只要 "一个打了补丁的
   `controller_manager` + 自己的 chainable 控制器 + YAML 里一行 `two_phase_execution: true`"。
2. **上游 PR 友好**：diff 只落在 `controller_interface`（一个接口头）与 `controller_manager`，
   评审面小、行为可选（默认关闭）。
3. **研究线不受影响**：`humble-work` 继续承载内核/执行组/元编程/分析层，两条线各自演进，
   但**共用同一套断言**（见 §1.7）。

### 1.2 建议的范围（包含 / 不包含）

| 包含 | 不包含（留在 `humble-work`） |
|---|---|
| `TwoPhaseControllerInterface`（两阶段接口） | `StagedExecutionGroup` 内核、`StagedControllerInterface`、帧/整组提交 |
| 管理器两趟调度：反向状态趟 + 正向命令趟、速率分桶 | 执行组安装/清除 API、部分成员惰性策略 |
| 准入与安全检查：跨模式边、列表顺序、同实例两名、速率桶、**成员绝不被原生循环接管** | 编译期类型层（拓扑/量纲/契约/typed ports/manifest） |
| `two_phase_execution` 参数 + `set_two_phase_execution()` API + `two_phase_rejected_controllers()` | 编译内置控制器注册表（`StaticControllerRegistry`） |
| 执行状态**一个不可变快照**（模式+成员表；可省掉 staged group 字段） | typed composite / 库模式宿主 |
| 测试：两趟核心断言 + 一份 runnable 示例控制器 | 论文级分析层（`ΔPM`、`κ(D)`、Gazebo 案例的测量脚手架） |
| 可选：速率分桶（建议带，见 §1.6） | 可选：`atomic_activation`、WCET 报告（见 §1.6，建议先不带） |

### 1.3 怎么做（最小补丁，不是 cherry-pick）

```bash
# 1) 从【纯上游 Humble】起一个干净分支
#    `469f3055` 是上游 Humble 的提交，它是 origin/humble 的祖先；fetch 之后即可解析。
#    注意：不要把基点选成 origin/humble —— 那条分支比上游多 4 个早期提交
#    （v1 原型 c9e6452、研究文档 a2ff98a/c74c1cd、cycle_tree 实验 ea3992e），
#    带过去会把已经废弃的原型一起扯进来。
git fetch origin humble
git switch -c feature/two-phase-manager 469f3055

# 2) 从这个分支带过来的"功能补丁"，建议用「重写」而不是「挑提交」：
#    * 接口头搬进上游包（见 1.4），因此不需要新包
#    * 管理器改动按"一个特性"整理成一个或少数几个提交
#    * 测试与示例一起提交，保证每个提交都能自证

# 3) 随时核对：这个分支相对上游应该只有"一个特性"的份量
git diff --stat 469f3055 HEAD
```

**为什么重写**：`humble-work` 的管理器改动与执行代/执行组交织（同一个 `ExecutionGeneration` 同时承载
模式、两趟成员表、执行组），cherry-pick 会把执行组一起拖过来。重写时可以只保留
`two_phase_enabled + two_phase_entries + two_phase_buckets` 三个字段。

### 1.4 关键决定：把接口头搬进 `controller_interface`（这样就没有新包）

`hierarchical_control/include/hierarchical_control/two_phase_controller_interface.hpp` 只有 66 行，
依赖只有 `controller_interface/controller_interface_base.hpp`、`rclcpp/time.hpp`、`rclcpp/duration.hpp`
（都是上游）。建议在新分支里改成：

```
controller_interface/include/controller_interface/two_phase_controller_interface.hpp
    namespace controller_interface { class TwoPhaseControllerInterface { ... }; }
```

于是新分支的 diff = **上游 `controller_interface` +1 文件**、**`controller_manager` 一个特性**，
用户不需要额外安装任何新包。

### 1.5 不能裁剪的部分（安全相关，缺一个就会"静默出错"）

| 必须保留 | 缺了会怎样 |
|---|---|
| **跨模式边拒绝**（一条参考边两端一个是两趟成员、一个是 native） | 该边被两个调度排序 → 悄悄用上一周期的值 |
| **列表顺序拒绝**（父必须在子之前） | 反向状态趟/正向命令趟对那条边同向走错 → 计数正常但数据旧 |
| **同实例两名拒绝** | 同一对象一周期被推进两次（实测 3 周期 6 次 `update_phase`） |
| **两趟成员绝不被原生循环接管**（即使切换期间也跳过） | 切换的几个周期里悄悄退回单趟语义 / 状态被推进两次 |
| **任一状态阶段失败 → 本周期不跑命令阶段** | 控制器在它刚刚拒绝的数据上继续算命令 |
| `two_phase_execution` 参数 + `set_two_phase_execution()`（全有或全无） | 部署只能靠改代码；且无法表达"要么全对、要么别开" |
| 速率规则（见 §1.6） | 降频控制器被按管理器频率调用，静默改变离散化 |

### 1.6 可选带上（建议与理由）

| 可选件 | 建议 | 理由 / 代价 |
|---|---|---|
| **速率分桶**（论文 §III-B） | **建议带** | 只有 ~120 行 + 3 个用例；不带就得保留"任何速率不等于管理器频率即拒绝"这条**假限制** |
| **声明 WCET 的可调度性报告** | 可先不带 | 纯增量（~140 行 + 2 用例），不带也不影响正确性；带了能让用户拿到"声明口径的期限检查" |
| **`atomic_activation` 回滚** | 可先不带 | ~250 行 + 5 用例；不带 = 上游默认的 best-effort（失败的 switch 可能留下半棵树，本来就是上游语义）。**但如果带**，必须连"为切 chained mode 被重启的控制器要恢复"一起带（否则一次失败 switch 会停掉无关控制器） |
| TSan 脚本 `run_tsan_real_manager.sh` | 建议带 | 它本来就只跑 `test_two_phase_execution`；是"两趟发布协议无 race"的证据 |

### 1.7 与 `humble-work` 的同步策略

- **一个真相，两个分支**：两趟核心的断言（同周期、准入拒绝、分桶、失败包含）在两条分支上
  **跑同一份测试文件**（内容可以少量裁剪，但断言名与语义保持一致）。改动核心语义时**先改测试**，
  再同步两边。
- **方向**：`humble-work` 是研究线；凡涉及两趟核心的修复/新断言，**cherry-pick 到特性分支**并补一条
  `SYNC.md` 记录；特性分支上发现的 bug 反向合回 `humble-work`（不要只在一边修）。
- **不要**把特性分支当作 `humble-work` 的父分支（避免历史倒挂）。

### 1.8 验收标准（Definition of Done）

- [ ] `git diff --stat 469f3055 feature/two-phase-manager` 只包含 `controller_interface`（1 个头）
      与 `controller_manager`（源码 + 测试 + 示例），**没有新包**；
- [ ] `colcon build --packages-up-to controller_manager` 在干净 Humble 上通过；
- [ ] 特性分支自带演示：两个 chainable 控制器（叶/根，如 `case_study` 的 wheel/chassis 精简版）
      + 一份 `controllers.yaml`，`ros2_control_node` + `spawner` 能跑，`ros2 control list_controllers -v`
      能看到 `is_chained`，并能用"计数器/时间戳"证明**父读到的是本周期子状态**；
- [ ] 两趟核心测试全绿（目标 ≥ 20 例），且**默认关闭**时上游行为不变（含上游 chaining 用例）；
- [ ] 分支根目录有一份 `TWO_PHASE_BRANCH.md`：范围、排除了什么、与 `humble-work` 的同步方式、
      已知限制（无整组提交、无帧语义、无事务回滚）；
- [ ] 风险登记：ABI 变化（`ControllerManager` 新增数据成员）写在 README/分支说明里。

**风险**：① 两条线各自演进会产生语义漂移 → 用 §1.7 的"同一套断言"约束；
② 特性分支若被用户当成"完整方案"，会缺少整组提交/帧语义 → 在分支说明里写清边界。

### 1.9 已完成：编译期可确定性审计（2026-10-03）

见 `COMPILETIME_AUDIT_2026-10.md`。要点：

- 审计了 13 项"运行期 vs 编译期"，分类 A（立即可做）/ B（需设计）/ C（不该搬）。
- **落地了价值最高的一项**：`manifest_is_well_formed<Binding>()` 早已实现并被测试，
  却没有任何入口调用；现在接进 `topology_binding::to_library_spec<Binding>()` 与
  `TypedForkCompositeController`，并补了一个负向编译用例
  （`compile_fail_manifest_claim_conflict.cpp`，两个节点认领同一硬件接口 → 编译错误）。
- 边界：**不删任何运行期检查**（pluginlib/YAML 部署看不到这些 `static_assert`）。
- **另已落地**：把"哪些节点是叶子"变成编译期事实（`static_manifest::leaf_names()` /
  `leaf_count()`），激活路径不再做 O(N²) 父名扫描，并在缺接口时点名（审计 §3.1）。
- **另已落地**：声明式两阶段树的**编译期描述层**（`static_two_phase_admission.hpp`）——
  会员性、成员先序、参考边、父先于子全部由类型导出；核心验证是**等价性**：
  编译期边集 == 运行时按 `"<owner>/"` 切分推断出的边集（审计 §3.2）。
- 剩余的 A 类（F4 里的 `node_at` switch、generic composite）与 B 类（**F5 的管理器入口**、
  F6 编译期顺序、F7 RateTag、F8 标注式链关系、F11 constexpr 容器）**未做**。
  **F5/F6 已出设计**：`STATIC_ADMISSION_DESIGN_2026-10.md`（含"六个拒绝码里三个由类型取代、
  两个必须留运行期、一个在静态树里不存在"的结论与可证伪的等价性验证计划）。

---

## 2. 功能与理论缺口

### 2.1 P0-2：论文 Thm 3 的对应物（时延上界）→ 允许跨速率桶的边

- **现状**：跨桶参考边被**拒绝**（`cross_rate_dependency`）。原因写在枚举注释里：论文自己的调和周期推论
  说明周期不同时**向下**方向只有"有界延迟"，而我们对外承诺每条被接纳的边**两向 0 周期滞后**。
- **要做的**：在**周期数**单位下推导一个上界（论文用 µs），把"拒绝"升级为"有界接纳"：
  1. 明确假设：周期调和（父周期是子周期整数倍）或更宽松的 `T_child ≤ T_parent`；
  2. 对跨桶边给出**最坏滞后 = ⌈T_parent/T_child⌉ − 1 个父周期**一类的表达式（或论文 Cor 1/Cor 2 的对应物）；
  3. 在准入报告里给出"这条边的滞后上界"，并加一个测量用例验证（把声明上界与实测滞后对比）；
  4. 决定是否仍然拒绝**非调和**（任意比例）的边。
- **为什么值**：这是两趟路径唯一"缺一块"的地方，也是与论文差异最大的一处；补上后多速率树可以完整使用。
- **DoD**：公式 + 推导说明（进 `PASS_LOWER_BOUND.md` 或新文档）+ 上界随周期比变化的测试 + 准入放宽的
  用例（跨桶边被接受且滞后 ≤ 声明上界）+ `PAPER_ALIGNMENT` §6 更新。

### 2.2 P1-1：执行组的 ROS 入口

- **现状**：`set_staged_execution_group()` 只有 C++ API（`USER_GUIDE.md` §4.2 已诚实写明）。
- **要做的**：二选一或都做——
  (a) 新服务 `~/set_staged_execution_group`（+ 清除），(b) `controller_manager` 参数
  `staged_controllers: [names]` + `staged_max_age_ns`。
- **必须先解决的一致性问题**：准入判定要对着"控制器列表 + 执行代"的一致快照，而列表是上游双缓冲发布。
  可选方案：① 服务调用时要求 `control_loop_busy() == false`（与现在 API 的约束一致，最简单）；
  ② 先在服务线程做准入，再在一个控制周期内原子落地（需要列表发布参与握手，工作量大）。
- **DoD**：服务/参数可用 + 与 C++ API 行为一致（同一准入判定）+ 失败时返回可读原因 + 测试
  （服务路径与 API 路径产生同一成员集合/同一拒绝原因）。

### 2.3 P1-2：把控制器**列表**纳入执行代

- **现状**：模式/成员/计划是一个执行代，但列表不是 → 所以"安装执行路径"必须在周期停止时做
  （`control_loop_busy()` 拒绝），"停止期配置"这条约束仍在。
- **要做的**：让列表发布与执行代绑定（同一 `atomic_store`，或为列表引入版本号 + 周期内快照校验）。
  这会动上游 `RTControllerListWrapper` 的双缓冲 + `wait_until_rt_not_using()` 握手。
- **收益**：可以在控制循环运行中安全地加载/配置/激活控制器而不需要停止期约束。
- **风险**：直接改上游核心发布协议，回归面大；建议先写一份设计（含 TSan 验证计划）再动手。

### 2.4 P2：`Spec::parents` 的配置入口

- **现状**：内核支持 `Spec::parents`（显式声明父边），但**没有** YAML/参数入口；边的来源目前是
  "claim 的接口名前缀"。两趟之后该字段**不是必需的**。
- **要做的**：`staged_group` 安装时允许用参数声明父关系（覆盖前缀推导），并给出一条"前缀推导与显式声明
  冲突时报错"的规则。
- **为什么低优先**：只在"控制器名与端口前缀不一致"的场景下需要。

### 2.5 P2：多执行组

- **现状**：同时只支持一个 `staged_group`（评审也建议暂不扩展）。
- **要做的**：执行代里把"一个组"改成"组列表"，准入时检查跨组成员的重叠与跨组边。
- **注意**：跨组边会让"整组提交"的语义变复杂（两组各自提交还是联合提交？）——先定语义再动手。

### 2.6 明确不做（写在这里避免反复讨论）

- **多频/异步回调**（不同控制率的执行组）：模型与内核都假定单频同步；不做。
- **动态拓扑**（运行期增删树成员）：描述是类型/配置期固定的；不做。
- **把控制器做成全局对象/singleton**：与多实例隔离、析构顺序、插件卸载冲突；不做（见
  `COMPILETIME_CONTROLLER_RESPONSE_2026-09-26.md` §1.1）。

---

## 3. 可靠性与并发

### 3.1 P1-3：依赖库 TSan

- **现状**：只给 `controller_manager` 插桩（`run_tsan_real_manager.sh`）；rclcpp/lifecycle/hardware_interface/
  FastRTPS 未插桩 → 只能声称"管理器自身无 data race"。
- **要做的**：为依赖树建插桩构建（或在容器/大磁盘机器上做），跑两趟 + 执行组 + 切换场景；
  把"依赖库内部 race"与"我们的 race"分开报告（脚本已支持分开报 verdict）。
- **前置**：磁盘（当前 2.6 GB 空闲；TSan 树约 150 MB + 依赖树数 GB）与时间。

### 3.2 P2：`members_active_` 缓存的非 switch 过期

- **现状**：缓存只在切换后刷新（`refresh_member_active_state()`；为了不在实时路径查 lifecycle）。
  因此**非切换**导致的生命周期变化（例如上游服务直接改状态）可能让执行组按旧缓存跑。
- **可选做法**：① 缓存里加"代 id"，任何生命周期变更路径都 bump（需要梳理上游所有路径）；
  ② 实时路径每 N 个周期校验一次（引入偶发分配/调用）；③ 明确记为限制（现状）。
- **建议**：保持现状 + 文档写明触发条件（`IMPLEMENTATION_GUIDE.md` §12.4 #1/#12）。

### 3.3 P2：registry 的"封印后只读"不是并发写安全

- **现状**：第一次加载前可注册，之后封印（`add()` 抛异常、替换被拒、`register_static_controller_type()`
  返回 false）。这是"启动期单线程配置"前提下的安全，**不是**并发写安全。
- **要做的（可选）**：若确实需要运行期注册，就上锁（`shared_mutex`）并规定 load 不能与注册并发；
  否则保持现状并把限制写在用户文档（已写）。

### 3.4 P2：回滚作用域是否需要"完整事务"

- **现状**：`atomic_activation` 只撤销**本次 switch 激活的**控制器；**用户显式请求的 deactivate 不撤销**；
  回调内部已写出的硬件副作用不可回滚（`handle_phase` 直写 command handle）。
- **要做的（若需要）**：定义"完整事务"语义（要不要恢复 deactivate？要不要为两趟引入命令缓冲？），
  并评估其对实时性的影响（缓冲 = 额外拷贝 + 一类新故障面）。
- **建议**：先不扩；在两趟路径的 API 注释里把"无整组提交"作为**契约**写清（已写）。

### 3.5 明确不可解：物理总线原子提交

`prepare/perform_command_mode_switch` 在 Humble 本身就是两段式，中间没有事务语义。只能声明边界，
不能承诺（`HANDOFF_MANUAL.md` §11 已列）。

---

## 4. 测试与工程债

### 4.1 P2：三个既有 flaky 的根治

| 用例 | 症状 | 根因 | 根治方向 |
|---|---|---|---|
| `test_controllers_chaining_with_controller_manager` | `internal_counter = 15` 期望 `14` | 计数由 10 ms 睡线程在 switch 窗口内的 tick 数决定 | 改 fixture 的驱动方式（用确定的泵循环替代睡线程）——会波及所有用它的上游用例 |
| `test_spawner_unspawner`（2 例） | `Could not contact service …` 后控制器数为 0/2 | 1.0 s 服务发现超时；且用退出码 256 同时表示"激活失败"与"联系不上服务" | 提高超时/重试，或改成"等待服务出现"而不是固定超时 |
| `test_hardware_spawner.spawner_with_later_load_of_robot_description` | 期望第一次 spawner 失败却成功（或反向） | 用例用 2.5 s wall timer 延迟发 `robot_description`，假设 1.0 s 超时的 spawner 一定先问过服务 | 用事件（服务可用性）而非固定时间假设 |

三条都已做过**改动前后对照**（见 `TESTING_GUIDE.md` §6），确认不是本项目的回归。

### 4.2 P1-4：最小 CI

- **现状**：没有 CI。所有验证都是本机手跑（记录在 `TESTING_GUIDE.md` §7）。
- **要做的**：GitHub Actions（或任意 CI）上：
  1. `colcon build --packages-up-to controller_manager hierarchical_control`；
  2. 跑 `hierarchical_control` ctest 全量 + `controller_manager` ctest（排除 §4.1 的三例）；
  3. 跑负向编译语料（`test_static_topology_negative`）；
  4. 可选：`clang-format` 检查（本仓库有 `.clang-format`，但环境里没装 `clang-format`）。
- **收益**：特性分支与主线都能"一个按钮"验证；也给上游 PR 添信。

### 4.3 P2：ctest 超时与套件时长不匹配

`test_controller_manager_srvs` 需要约 230 s，而 `CMakeLists.txt` 给它 `TIMEOUT 120` → 在 ctest 下必然失败
（单独跑 14/14 通过）。**建议**：把该属性改成 300 s（或按套件拆分）。

### 4.4 P1-4：文档英文化（如果要给上游）

现有文档以中文为主（43 份 md）。给上游 PR 至少需要：README 段、"两趟是什么/怎么用/边界"的英文说明、
接口头的 doxygen 注释（这部分已经是英文）。建议先英文化 3 份：`USER_GUIDE`（用）、
`FINAL_REPORT`（给评审）、`PAPER_ALIGNMENT` 的结论部分。

### 4.5 P2：ABI 与版本化

`ControllerManager` 新增数据成员 ⇒ ABI 与系统安装不同。建议：
① 在 `package.xml` 里对 `controller_manager` 的改动写明（版本号/描述）；
② 文档里明确"覆盖层 + 一起重编"的规则（已写 `USER_GUIDE.md` §1.4）；
③ 若将来给上游，优先用**继承 + 组合**或把新状态放进可选的 pimpl，避免 ABI 破坏（设计题）。

---

## 5. 各项的"做完"定义（模板）

每项收尾时都要满足：

- [ ] 行为有**会失败的测试**（负向与正向各至少一条）；
- [ ] 文档更新到位的三处：`IMPLEMENTATION_GUIDE.md`（实现状态）、`TESTING_GUIDE.md`（怎么验证）、
      相关专题文档（`PAPER_ALIGNMENT` / `USER_GUIDE` / `DEVELOPMENT_HISTORY` 视情况）；
- [ ] 若引入新限制或撤回旧说法，写进 `HANDOFF_MANUAL.md` §11 或 `IMPLEMENTATION_GUIDE.md` §12；
- [ ] 跑一次完整记录（ctest + 被排除套件单独跑），把结果与 flaky 分类写进 `TESTING_GUIDE.md` §7；
- [ ] 提交信息写清"为什么 + 实测数字"。

---

## 6. 这些待办在哪些文档里被提到（避免重复记账）

| 待办 | 原始出处 |
|---|---|
| 独立"管理器+两趟"分支 | 本文档 §1（用户提议）；`USER_GUIDE.md` §0/§3 |
| Thm 3 上界 / 跨桶边 | `PAPER_ALIGNMENT_2026-09-28.md` §6.2、`FINAL_REPORT_2026-09-28.md` §7 |
| 执行组 ROS 入口 | `USER_GUIDE.md` §4.2、`FINAL_REPORT_2026-09-28.md` §7 |
| 列表纳入执行代 | `IMPLEMENTATION_GUIDE.md` §12.4 #16、`HANDOFF_MANUAL.md` §11 |
| `Spec::parents` 配置入口 | `IMPLEMENTATION_GUIDE.md` §12.1 #10 |
| 多执行组 | `IMPLEMENTATION_GUIDE.md` §12.1 #6 |
| 多频/异步、动态拓扑 | `IMPLEMENTATION_GUIDE.md` §12.1 #1/#2、`FORMAL_MODEL.md` §8 |
| 依赖库 TSan | `IMPLEMENTATION_GUIDE.md` §12.4 #13、`FINAL_REPORT_2026-09-28.md` §5/§7 |
| 缓存一致性 | `IMPLEMENTATION_GUIDE.md` §12.4 #1/#12 |
| WCET / 期限 | `IMPLEMENTATION_GUIDE.md` §12.1 #9（已部分：声明口径）、`PAPER_ALIGNMENT` §6.1 |
| 真实硬件故障动作 | `IMPLEMENTATION_GUIDE.md` §12.1 #8 |
| URDF 参与拓扑校验 | `IMPLEMENTATION_GUIDE.md` §12.1 #7 |
| flaky 三例 | `TESTING_GUIDE.md` §6 |
| ctest 超时 | `TESTING_GUIDE.md` §6/§7 |
| ABI / 覆盖层规则 | `USER_GUIDE.md` §1.4、`FINAL_REPORT_2026-09-28.md` §5 |
| Gazebo 真值指标不可靠 / 耗时不稳 | `IMPLEMENTATION_GUIDE.md` §12.3 #1/#2 |
| Jazzy/Rolling 复现（无 Docker） | `IMPLEMENTATION_GUIDE.md` §12.2 |

---

## 7. 论文定位与文献（2026-10 新增）

见 `LITERATURE_SURVEY_2026-10.md`。**结论对本项目不利，但必须正视**：

1. **创新点 1（树状双向两趟调度）在机制层面不是新的**：所引的 FineMote
   （arXiv:2608.04600v1）**§III-B 式 (2)** 已经给出"同一线性化、桶内先全部 `+`（子→父）、
   再全部 `−`（父→子）"，**§IV-C2 推论 2** 已经给出同周期零等待；
   上游 `ros2_control` 也**已经**维护一条由树导出的线性化（issue #853 已修）。
   可辩护的增量是：**把它移植到插件式、运行期加载的控制器集合上**，量化上游单趟的
   **静默单向陈旧**，并把残余正确性条件**归约为一组有限的运行期准入检查**。
2. **创新点 2（编译期元编程）是成熟技术在新区间的应用**：typestate（1986）、
   dimension types（1994/1997）、session types（1998）、policy-based design、
   **负向编译测试（Pigweed `pw_compilation_testing`）** 都是已有工作；
   未发现把"控制器拓扑"编码为 C++ 类型链并 `static_assert` 的先例，但这是**领域空白**，
   不是方法空白。审稿人最可能的攻击点是 **ROS 2 自己的静态校验答案是 codegen
   （`generate_parameter_library`）而不是 TMP**——必须正面回应。
3. 因此建议的定位是**枚举式结论**：
   "在一个插件式、YAML/URDF 驱动的实时控制框架里，可前移到编译期的恰好是**由类型唯一决定**的那一层"
   （见 `COMPILETIME_AUDIT_2026-10.md` §4）。
4. 必引清单、逐条重叠分析与可辩护的贡献陈述见 `LITERATURE_SURVEY_2026-10.md` §5/§7；
   投稿去向的讨论见其 §6（JOSS 明确接受"重实现已知方案"，但**要求** state-of-the-field 对比）。
