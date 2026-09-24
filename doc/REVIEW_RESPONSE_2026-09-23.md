# 对 `REVIEW_HUMBLE_WORK_2026-09-23.md` 的逐条处理

日期：2026-09-23
基线：分支 `humble-work`
输入：`doc/REVIEW_HUMBLE_WORK_2026-09-23.md`（外部评审，R1–R11）

本文记录**每一条评审意见的实际处理**：改了哪里、怎么验证、哪些**没做**。
原则与评审一致——**没有验证过的不能说已完成**。凡本轮未复跑的实验
（ROS 包级全量测试、sanitizer/TSan、Gazebo、性能测量）都明确标注。

状态图例：**已修**（有代码/文档改动 + 本轮验证）｜**部分**（主项已修，仍有未闭合项）｜**未修**（本轮未动）

---

## 0. 结论速览

| # | 主题 | 状态 | 证据 |
|---|---|---|---|
| R1 | 最少趟数"NP 难"结论错误 | **已修** | `PASS_LOWER_BOUND.md` 文首修正框 + §4.1/§5.1 重写；`PAPER.md` §3.8 §9.2 附录 B；`PAPER_SKELETON.md`；C++ 测试注释；`search_min_passes.py` Interpretation（已重跑） |
| R2 | `G` 成环 ≠ 代数环/不可能 | **已修** | `FORMAL_MODEL.md` §6 重写为**充分条件**；`PAPER.md` §3.6；`HANDOFF_MANUAL.md` 定理表 |
| R3 | 漏写输出被重新标记为新鲜 | **已修** | `staged_execution_group.hpp` NaN 哨兵 + 5 处写入点；`test_contract_regression.cpp` 3 用例 |
| R4 | sink 中途失败仍部分提交 | **已修**（含我引入并修掉的一次分配回归） | 两遍重遍历同一 `leaves_`，不新增存储；`run()==0` 分配/100 次调用；`test_contract_regression.cpp` 1 用例 |
| R5 | `void*` 擦除破坏多继承指针 | **已修** | `BoundNode` 存类型化 `ControllerInterfaceBase*`；`test_contract_regression.cpp` 偏移用例；`TypedPortsMixin` 虚继承修菱形 |
| R6 | 多端口被误判为多个写者 | **已修**（含一次方向性返工，见 §R6） | `derive_parents_from_claimed_interfaces()`：写者按 **PORT** 唯一、父唯一性按**子节点**判定；7 个 r6_* 用例；既有 `test_staged_execution_group` / `test_hierarchy_comparison` 回归通过 |
| R7 | 两条执行路径的保证不可混用 | **已修** | 见 §R7：入口处**整体拒绝** + 频率校验 + 反方向镜像校验；6 个新用例 |
| R8 | 非实时配置与实时执行缺发布协议 | **已修**（主项 + 分配探针） | 见 §R8：成员集**原子发布**、实时路径**零重建**、`staged_group_` 原子读写；新增切换后分配探针；TSan **未跑** |
| R9 | Gazebo 周期陈旧量不是直接测量 | **已修** | 控制器打周期号 + `measure_tracking.py` 改为周期差、缺数据即失败、ms 标注为"按配置周期换算"；脚本**未在 Gazebo 上重跑** |
| R10 | 上游行为与存储表述超范围 | **部分** | 日志/PulseAudio cookie 已从 git 移除；"零额外存储"已在 7 份文档中改为"不需要第二份拓扑顺序"；上游端口**实际导出/认领**的端到端验证**仍未做** |
| R11 | 对照与新颖性需重新收敛 | **未修** | 未做文献查新；估计/命令拆分基线未补 |

**本轮唯一新增的实测数据**：
`controller_manager/test_two_phase_execution` 新增 6 个 R7 准入用例、
`test_hierarchy_comparison` 新增 1 个 R8 切换后分配探针，
以及为修掉我自己引入的两个回归而重跑的全量测试（见 §R4、§R6 的"⚠"小节）。
其余测试为**重跑既有用例**，不是新实验。

---

## R1 最少趟数"NP 难"——**已修（撤回）**

评审的反例是对的，并且比"反例"更强：对任意**无自环**有向图 `D`，任取顶点全序 `π`，
把边分成"沿 `π` 增大"与"沿 `π` 减小"两类，二者都不可能含回路。所以

```text
κ(D) = 0  (E_D = ∅) | 1  (E_D ≠ ∅ 且无环) | 2  (含环)
```

这是**完整分类**，不存在难解问题。推论 P3（"双向级联 κ = 2"）因此**不是**关于树形结构的
独立发现，而是该分类的特例：任何含环的 `D` 都是 `κ = 2`。

改动：

- `doc/PASS_LOWER_BOUND.md`
  - 标题去掉"并证明两趟最优"；文首加**修正框**（含反例证明、后果、语义近似说明）；
  - §4.1 从"两趟是最优的"改写为"两趟对含环的图都够，而且一趟不够"，并显式写明
    "把它读成'找到了树形结构的最优调度'是过度解读"；
  - §5.1 原 NP 难条目**删除线标记 + 撤回**；新增两条限制：
    "最优"只限**完整阶段遍历次数**（不含 CPU 时间/端到端延迟）、
    以及"边覆盖模型不保证同一逻辑周期，充分性应由阶段顶点 `S_i`/`C_i` 的依赖图证明（**未完成**）"。
- `doc/PAPER.md`：§1.2 第 3 点、§1.3、贡献表 C8、§3.1、§3.6、§3.8 正文与"限制"、§9.2 未完成项、
  附录 B"可以声称清单"——全部弱化或标记撤回。
- `doc/PAPER_SKELETON.md`：主线去掉"最优性"，C3′ 从"强"降为"弱（诚实的负结论）"。
- `hierarchical_control/test/test_pass_lower_bound.cpp`：文件头注释改为平凡分类。
- `research/pass_lower_bound/search_min_passes.py`：`Interpretation` 输出改为
  "验证的是平凡分类，不是最优性定理"，并明确禁止"NP-hard / 调度最优"表述。
  **已重跑**，输出与上述一致，`ast.parse` 通过。

**仍未做**：评审要求的"用实际阶段顶点 `S_i`/`C_i` 建图证明充分性"、以及四节点穷举。
这两项写在 `PASS_LOWER_BOUND.md` §5 的新限制里，**没有**声称完成。

## R2 `G` 成环 ≠ 实际代数环——**已修（撤回）**

评审给出的反例（reference 边 `A→B` 与 state 边 `A→B`，实际阶段约束可全部同周期满足）成立。

- `doc/FORMAL_MODEL.md` §6 从"配置期无环检查（含环即不可能）"改为
  **"`G` 无环是充分条件，不是必要条件"**，含反例、只有充分性方向的证明、
  以及"判断一般阶段调度是否可能必须在 `S_i`/`C_i` 图上查环——**尚未实现**"的边界说明。
- `doc/PAPER.md` §3.6 同步。
- `doc/HANDOFF_MANUAL.md` 定理表"推论 2"一行改为"`G` 无环 ⇒ 该表示可覆盖（充分条件）"。

## R3 漏写输出被重新标记为新鲜——**已修**

`staged_execution_group.hpp` 引入常量 `kUnwritten = std::numeric_limits<double>::quiet_NaN()`，
在 5 个写入位置以 NaN 作为"本周期未写"的哨兵，使"未写"不可能被下游读成"新鲜且等于 0/旧值"。
回归：`test_contract_regression.cpp` 中 3 个 r3_* 用例。

## R4 sink 中途失败仍部分提交——**已修**（含一次自己引入的性能回归）

改为两阶段提交：第一遍只调用各 leaf 的 `sink->commit()`（副作用域，不可回滚，已在注释里声明），
**全部成功之后**第二遍才把 scratch 镜像进组自己的 `actuator_committed_`/`committed_`。
于是组的内部视图永远描述"最后一次完整提交的周期"。
回归：`test_contract_regression.cpp` 的 `command_failure_and_nan_never_commit`。

> 说明：本项目**曾经**因为一次正则批量清理把 `run_ns` 里的 8 个 `return` 和 2 个守卫删掉，
> 导致该用例一度"通过"（返回 `committed` 且提交 NaN）。已整函数重建并逐一核对全部 18 个返回点；
> 该事故本身也说明"测试通过"必须配合代码审查，不能只看颜色。

### ⚠ 本次修复中我自己引入并修掉的第二个错误：内核重新开始分配

第一版 R4 实现用了一个**局部** `std::vector<std::size_t> committed_leaves` 来记住第一遍成功的
leaf，每周期 `reserve` + `push_back`，于是 `StagedExecutionGroup::run()` 从"零分配"变成
**每周期 1 次分配**——直接推翻了 `HANDOFF_MANUAL.md` §11 里"`run()` 与库宿主零分配"这条
可以声称的性质。

发现方式：`controller_manager/test/test_hierarchy_comparison` 的分配探针
（它不是我为 R4/R6 写的测试，而是**既有**的公平对照测试）：

```text
[comparison] StagedExecutionGroup::run allocations per 100 calls=100 (control=0)
[comparison] generic composite library update allocations per 100 calls=100 (leaves=2)
```

修复：两遍**重新遍历同一个 `leaves_` 数组**，不需要任何额外存储。

```cpp
for (const auto leaf : leaves_) { ...sink->commit(...)... }          // 副作用
for (const auto leaf : leaves_) { ...mirror into committed view... } // 仅在全部成功后
```

验证：修复后 `run allocations per 100 calls=0`，`staged` 每周期分配数回到与改动前一致。

> 两层教训与 R6 相同、但方向相反：R6 是"新单测跟着我的误解一起变绿"，
> 这里是"新单测不测我顺手破坏的旧性质"。**既有测试必须全跑**，尤其是测量
> 分配/耗时这类非功能性性质的测试。

## R5 `void*` 擦除破坏多继承接口指针——**已修**

`topology_contract.hpp` 的 `BoundNode` 改为保存类型化的
`controller_interface::ControllerInterfaceBase * instance`（不再经 `void*` 往返），
`SpecRows::instances` 同步类型化；`make_leaf`/`compose` 因此不再是 `constexpr`（改收 `ControllerT*`）。
另外修掉 `MinimalController` 与 `TypedPortsMixin` 的菱形继承（两处 `public virtual` 继承）。
回归：`r5_non_zero_base_offset_survives_the_binding`（若基类偏移为 0 则 `GTEST_SKIP`，
因为偏移为 0 时该用例无法区分修复前后）。

## R6 多端口被误判为多个写者——**已修**（含一次方向性返工）

`hierarchy.hpp` 新增纯函数 `derive_parents_from_claimed_interfaces(names, claimed_interfaces)`，
把原来内联在 `StagedExecutionGroup::create()` 里的推导抽成可单测的纯函数。两条**不同**的约束
必须用不同的键：

| 约束 | 键 | 含义 |
|---|---|---|
| 一个 **端口** 只能有一个写者 | 完整端口名 | 同一端口被两个 claimant 认领 ⇒ 拒绝（这是 R6 的原始问题点） |
| 一个 **节点** 只能有一个父 | 子节点 | 两个不同 claimant 认领同一 owner 的**不同**端口 ⇒ 仍拒绝（该 owner 会有两个父） |

一个 claimant 认领同一 owner 的**多个**端口是正常的二维/三维参考，**不是**冲突——
这正是 R6 报告的误判（旧代码按 owner 去重）。消费者内部重复声明单独用 `seen_in_claimant` 检出。

### ⚠ 本次修复中我自己引入并修掉的一个更严重的错误（必须记录）

我在实现该函数时把**父子方向写反了**：写成 `parents[claimant] = owner`，
而正确方向是 `parents[owner] = claimant`。
依据是内核自己的契约文档（`StagedGroupMember`，`staged_execution_group.hpp` 第 54–56 行）：

> "A name whose prefix is another member's name defines a command edge:
> **this member is the reference *producer* (parent), the prefix owner is the *consumer* (child)**."

即"认领 `owner/port` 的控制器就是写这个端口的人，因此是**父**"。`doc/hierarchical_research.md` 第 14 行
早就写了同一件事（"the child exports a reference interface and the parent claims/writes it"）。

后果：我当时的 6 个 r6_* 用例是按**错误方向**写的，所以它们全绿，掩盖了真实回归；
而**既有的** `controller_manager/test/test_staged_execution_group`（5 个用例）与
`test_hierarchy_comparison`（5 个用例）开始失败——层次被整体倒置，
状态阶段变成了"父先于子"、命令阶段变成"子先于父"，数据依赖读不到值、组提交被校验拒绝。

修复：反转函数中的赋值方向，并重写 r6_* 用例为**内核契约的方向**（7 个用例，含
"一个父带两个子"合法、"一个子有两个父"拒绝、"一个端口两个写者"拒绝）；
此前那 10 个失败的既有用例全部恢复通过。

> 教训：**新增的单元测试会继承作者对契约的误解而一起变绿。**
> 判断"修好了"必须同时跑**既有的、独立编写的**集成测试，不能只看新加的单测。
> 这也是我在 R8 里坚持保留"每周期最多执行一次"这条端到端断言的原因。

**验证（本轮实跑）**：
`hierarchical_control` 全部 10 个测试程序通过（含 `test_contract_regression` 12/12）；
`controller_manager` 的 `test_staged_execution_group` 6/6、`test_hierarchy_comparison` 5/5
——详见文末"本轮验证过的命令"。

---

## R7 两条执行路径不可混用——**已修**（本轮新增）

评审的验收条件："同时实现两个接口的插件每阶段恰好一次；低频配置明确拒绝。"

实现（`controller_manager/src/controller_manager.cpp`、`include/.../controller_manager.hpp`）：

1. **准入判定** `two_phase_admission(const ControllerSpec&)`，返回三类之一：
   - `already_staged`：该控制器是**当前已安装 staged group 的成员**；
   - `unsupported_update_rate`：`get_update_rate() != 0 && != 管理器频率`；
   - `accepted`。
2. **`set_two_phase_execution(bool)` 改为返回 `controller_interface::return_type`**：
   - 启用前先对**所有**实现 `TwoPhaseControllerInterface` 的控制器跑准入判定，
     **任一不通过就整体拒绝**（`ERROR`），且**不修改** `two_phase_enabled_`。
     理由：静默排除会让标志显示"已启用"而控制器实际还在原生路径上运行，是更坏的失败模式。
   - 禁用路径不变（清空成员集并返回 `OK`）。
3. **镜像校验**：`set_staged_execution_group()` 的成员循环里，
   若 `two_phase_enabled_` 且该成员**正在两趟成员集里**，则拒绝入组；
   另外对**组内成员**也施加同一条频率规则——staged group 同样是每周期跑一次、传管理器周期，
   没有原生循环那种逐控制器降频门控。
4. **重要修正（相对我最初的实现）**：我最初按"**是否实现** `StagedControllerInterface`"
   拒绝，这是**过严**的——一个控制器同时支持两种执行模式（本项目的
   `TestStagedController` 就是）是完全合理的设计，真正的危险是**同时被两条路径实际执行**，
   即"**成员身份重叠**"。已改为按实际成员身份判定。

关键在于"为什么频率必须拒绝"：原生循环在 `controller_manager.cpp:2400–2416` 有
`update_loop_counter_ % controller_update_factor` 门控并按需传入
`1.0/controller_update_rate` 的周期；两条新路径**没有**这个门控，一律每周期调用。
所以一个声明 50 Hz 的控制器在 100 Hz 管理器下，会被按 100 Hz 调用——
静默改变控制律的离散化，属于**正确性**问题而不是性能问题。

回归（`controller_manager/test/test_two_phase_execution.cpp`，新增 `TestExecutionPathAdmission` 夹具，
6 个用例，全部通过）：

| 用例 | 断言 |
|---|---|
| `two_phase_enable_is_refused_for_a_rate_mismatched_controller` | `ERROR` + 标志保持 `false`；随后 10 个周期内该控制器**恰好**被原生门控跑 5 次（10 个连续计数里 5 个偶数），且 `update_phase_calls == handle_phase_calls`，`legacy_update_calls > 0` |
| `staged_group_is_refused_for_a_two_phase_controller` | 两趟启用成功后，`set_staged_execution_group` 返回 `ERROR`，组仍为 `nullptr` |
| `two_phase_enable_is_refused_for_a_staged_member` | 装组成功后启用两趟返回 `ERROR`；再跑 10 周期，每个成员 `state_calls` 恰好 +10、`update_phase_calls == 0`、`legacy_update_calls == 0`（**每阶段恰好一次**的直接证据） |
| `staged_group_is_refused_for_a_rate_mismatched_member` | `ERROR`，组为 `nullptr` |
| `staged_group_accepts_a_rate_matching_member` | 频率恰等于管理器时接受（证明被拒的是**不匹配**，不是功能本身） |
| `two_phase_enable_accepts_the_default_rate` | `update_rate = 0`（跟随管理器）时接受 |

既有 2 个两趟/单趟滞后用例仍通过（8/8）。

**仍未做**：多频/异步/动态拓扑仍不支持（评审第 4.4 条也建议**先不要**扩展）。

## R8 发布协议——**已修主项**（TSan 未跑）

评审指出两个独立问题：(a) `staged_group_` / `two_phase_entries_` 的**数据竞争**；
(b) `update()` 内 `rebuild_two_phase_entries()` 含 `reserve/push_back/dynamic_cast/sort/旧 vector 析构`
——**实时路径在分配**。

改动（`controller_manager.cpp`）：

1. **成员集改为原子发布、不可变**
   - `two_phase_entries_` 从 `std::vector<TwoPhaseEntry>` 改为
     `std::shared_ptr<const std::vector<TwoPhaseEntry>>`；
   - 构建侧 `rebuild_two_phase_entries()` 先在旁边造好新 vector，再一次
     `std::atomic_store(&two_phase_entries_, new)` 发布；
   - 实时侧 `update()` 每周期**一次** `std::atomic_load(&two_phase_entries_)`，
     局部 `shared_ptr` 保证整周期存活（即使另一线程随即退休旧集合）。
2. **实时路径不再重建**
   - **删除** `two_phase_entries_dirty_` 与 `update()` 里的 rebuild 分支
     （含构造函数里对它的赋值）；
   - 成员集改在**非实时线程**、在控制器列表切换之后重建：
     `switch_controller()`（列表切换后）、`add_controller_impl()`（加载后）、
     `unload_controller()`（卸载后）、`reorder_controllers()`（重排后）、
     以及 `set_staged_execution_group()` / `clear_staged_execution_group()`（准入条件变化）。
   - 于是"成员集在 `update()` 里分配"这一条**从结构上消失**，而不是靠"通常不会触发"。
3. **`staged_group_` 改为原子读写**
   - 安装/卸载用 `std::atomic_store(&staged_group_, ...)`；`update()` 用
     `std::atomic_load(&staged_group_)` 取一次局部 `shared_ptr`，组内所有判断复用该局部量
     （`run()`、`owns()`、`refresh_member_active_state()`），不再多次读同一个 `shared_ptr` 对象。
   - 成员声明加 `mutable` 以配合 `atomic_load` 的非 const 指针参数
     （`staged_execution_group() const` 也要用）。

**运行期分配探针（本轮新增，含实测到的局限性）**

评审要求"运行期分配探针涵盖首次启用和切换后周期，而非只测稳态"。已在
`controller_manager/test/test_hierarchy_comparison.cpp` 新增
`post_switch_two_phase_cycles_do_not_rebuild_membership`：

- 分配计数钩子（`operator new` 替换）增加**线程过滤**：
  `g_count_only_current_thread` + `g_counted_thread`，只统计控制循环线程，
  于是并发的 `switch_controller()` 线程（合法地大量分配：`std::async` 启动、控制器列表拷贝、
  服务记账）既不能掩盖也不能伪造结果；`AllocationCounterGuard` 保证 RAII 复位，
  不影响同进程其它用例。
- 探针让**全部成员保持 INACTIVE**：此时每周期分配数只含固定开销，没有"按活跃控制器数"的
  分量，基线才有意义（实测：空闲稳态 18 次/周期，且逐周期完全平坦）。
- 用两次开关（先激活 leaf、再停用 leaf）把成员集"改脏"，**计数从开关被应用后的第一个周期开始**
  ——正是旧实现重建成员集的那个周期。

实测结果：

```text
[comparison] StagedExecutionGroup::run allocations per 100 calls=0 (control=0)
[comparison] generic composite library update allocations per 100 calls=0 (leaves=2)
[comparison] generic composite library update allocations per 100 calls=0 (leaves=3)
[comparison] idle baseline per cycle: first=18 steady=18
[comparison] post-switch profile: 19 18 18 18 18 18 18 18 18 18 18 18
```

断言：开关后的**稳态**周期必须**恰好**等于空闲稳态开销（`18`，捕获任何"每周期都重建"的回归）；
第一个开关后周期不得超过稳态 +1。

> ⚠ **这个探针能证明什么、不能证明什么（不要过度解读）**：
> 空闲循环每周期本来就有约 18 次分配（全在既有代码里），而重建一个 3 项的成员向量只需
> `make_shared` + `reserve` 约 2 次。**环境噪声远大于信号**，所以本探针**无法证明**
> "不再重建"——它只能(a)否掉每周期重建，(b)给第一个开关后周期一个 +1 的紧上界
> （旧实现的 `make_shared` + `reserve` ≥ 2 次，会越过该上界）。
> "不再重建"的真正保证是**结构性的**：`update()` 里已经**不存在**任何重新发布成员集的调用。
> 这一点在 `doc/IMPLEMENTATION_GUIDE.md` §9.3 里写明，也可用源码检索核对。

**仍然没做的**：

- **未运行 TSan**。并发正确性来自设计论证 + 代码审查，不是并发实测；
  上面的探针只覆盖"分配"这一侧面，**不覆盖数据竞争**。
- `std::atomic_load/atomic_store(shared_ptr)` 是 C++17 设施（本仓库
  `target_compile_features(... cxx_std_17)`），**不保证无锁**；libstdc++ 用自旋锁池实现。
  实时路径上它只做一次原子引用计数操作，代价可接受，但**不是**"零开销"。
- 成员集重建仍发生在持有 `controllers_lock_` 的非实时线程里，不阻塞实时线程
  （实时线程从不取该锁）；这与上游对控制器列表本身的做法一致。

## R9 Gazebo 陈旧量——**已修（脚本侧）**，未重跑

- 控制器侧新增周期号 `cycle()`：`travel_registry.hpp` 的 `TravelSource` 加纯虚
  `cycle()`；`wheel_controller` / `chassis_controller` 每周期自增，chassis 记录
  **它实际读到的那次写入**来自 wheel 的第几周期；诊断从 12 字段扩到 16
  （13/14/15 = chassis/left/right 周期号）。
- `case_study/scripts/measure_tracking.py` **重写**：
  - 滞后 = `chassis_cycle - wheel_cycle`（整数周期差，不再靠接收时间做最近邻匹配）；
  - 用 `DIAG_*` 常量索引诊断字段；缺数据调用 `fail()` 让实验**失败**，不再默认 `lag = 0`；
  - 毫秒标签明确写成"**按配置控制周期换算**的调度延迟"，
    并说明 `0 cycle` **不等于** 0 ms 端到端时延。
- 评审提到的"最大匹配距离/丢包/时钟偏移"在周期号方案下不再是主要误差源，
  但仍未限制丢包；`case_study/logs/` 已从版本库移除。
- **未做**：没有重新跑 Gazebo 实验。因此 `GAZEBO_CASE_STUDY.md` 里已有的数值
  **仍是旧脚本产生的**，不能当作新方法的验证。这条在文档里保持原样、不追认为已更新。

## R10 证据边界——**部分**

已做：

- `git rm -r --cached case_study/logs`（27 个文件），删除工作区里泄漏 PulseAudio cookie 的
  目录，加入 `.gitignore`（`build/ install/ log/ case_study/logs/` 等）。
- **"零额外存储"表述全部改掉**，改为"不需要第二份拓扑顺序"，并显式说明
  `HierarchyPlan` 仍保存 `preorder` 与 `postorder` 两份 `O(|V|)` 线性化、
  另有按节点/端口分配的 scratch/frame/committed 缓冲；两趟相对单趟**只是不增加量级**。
  涉及：`HANDOFF_MANUAL.md`（§1.2、§3、§10 对照表）、`PAPER.md`（§1.1、贡献表、§3.5、§3.6、§8）、
  `PAPER_SKELETON.md`、`TWO_PASS_VS_SINGLE_PASS.md`、`WHY_NO_SPEEDUP.md`、
  `HANDOFF_NEXT_SESSION_2026-09-21.md`。

**未做**：评审要求
"父声明 `ord_child/state` 的测试必须**实际导出、激活认领、消费**该状态端口"，
即证明上游原生数据通路端到端可执行，而不仅是 `configure` 接受。
`test_upstream_ordering.cpp` 目前仍只验证排序。**没有**声称已覆盖。

## R11 对照与新颖性——**未修**

未做相关工作文献查新，未补"估计/命令显式拆分"基线，未统一固定时间常数下的跨频率比较。
按评审结论，此时**不作**"首创/无创新"的定论。

---

## 附：本轮验证过的命令与结果

```bash
export ROS_LOG_DIR="$PWD/log/ros"
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select hierarchical_control controller_manager hierarchical_control_case_study
```

**`hierarchical_control`（10 个测试程序，66 个用例，全通过）**

| 程序 | 结果 |
|---|---|
| `test_contract_regression` | 12/12 |
| `test_execution_group` | 11/11 |
| `test_pass_lower_bound` | 4/4 |
| `test_stale_state_cost` | 7/7 |
| `test_dimensional_interfaces` | 6/6 |
| `test_static_topology` | 5/5 |
| `test_topology_binding` | 5/5 |
| `test_topology_contract` | 7/7 |
| `test_typed_ports` | 5/5 |
| `test_scheduling_performance` | 4/4 |

**`controller_manager`（C++ 测试，全通过）**

| 程序 | 结果 |
|---|---|
| `test_two_phase_execution` | 8/8（含 6 个新增 R7 准入用例） |
| `test_staged_execution_group` | 6/6 |
| `test_hierarchy_comparison` | 6/6（含新增 R8 分配探针） |
| `test_controller_manager` | 18/18 |
| `test_controller_manager_srvs` | 14/14 |
| `test_load_controller` | 39/39 |
| `test_controllers_chaining_with_controller_manager` | 6/6 |
| `test_hierarchy` | 4/4 |
| `test_hardware_management_srvs` | 4/4 |
| `test_controller_hierarchy_builder` | 3/3 |
| `test_controller_manager_urdf_passing` | 3/3 |
| `test_upstream_ordering` | 2/2 |
| `test_urdf_hierarchy` | 2/2 |
| `test_release_interfaces` | 2/2 |
| `test_controller_manager_with_namespace` | 2/2 |
| `test_hierarchical_controller_executor` | 1/1 |
| `test_cycle_tree_contract`（独立程序） | PASS（1000 组同周期对照 + 故障/陈旧/恢复） |
| 编译语料 `hierarchical_control/test/test_static_topology_negative.py` | 8/8（7 个必须被拒 + 1 个反空洞对照） |

### ⚠ 仍然失败的 3 个用例（**不是我改的代码路径**，但也没有做 A/B 复核）

| 测试 | 现象 |
|---|---|
| `test_spawner_unspawner.spawner_test_with_wildcard_entries_with_no_ctrl_name` | `spawner` CLI 报 `Could not contact service /test_controller_manager/list_controllers`，返回码 256；测试自己传的是 `--controller-manager-timeout 1.0` |
| `test_spawner_unspawner.spawner_test_failed_activation_of_controllers` | 同上；**重跑一次通过**，说明是时序性的 |
| `test_hardware_spawner.spawner_with_later_load_of_robot_description` | 同类 `spawner` CLI 超时 |

判断与**证据边界**：

- 这三个用例走的是 `spawner` 可执行文件 + ROS 服务发现，**不加载也不触及**
  两趟/staged 代码路径（同一测试程序里另外 20 个用例通过，其中包含成功调用 `spawner` 的用例）；
- 失败信息是 **CLI 自己的 1.0 s 服务发现超时**，且本机磁盘已 **100% 占满（可用 70 MB）**，
  2 核 VM 的 I/O 与 DDS 发现都会变慢——这与"时序性超时"的解释一致（其中一个重跑即通过）；
- **我没有把工作树回退到 HEAD 做 A/B 复核**（磁盘已满，无法再开一棵构建树），
  所以**不能**断言"与本次改动无关"；只能说**失败发生在与改动无关的代码路径上**，且表现为环境时序问题。
  这一条按"未验证"记录，不计入已修。

其余未跑：sanitizer/TSan、Gazebo 回归、Python 侧 `pytest` 用例。
`HANDOFF_MANUAL.md` §11 的"不能声称"清单继续有效。
