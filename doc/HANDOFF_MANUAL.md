# 交接手册：ros2_control 控制器层次化 / 双向同周期数据流

> **本文是唯一入口文档。** 读完本文 + 第 12 节列出的 4 份文档，即可直接接手并继续执行，
> 不需要读任何对话历史，也不需要读本目录下的其它 `HANDOFF_*.md`（那些是历史记录）。
>
> 生成日期：2026-09-21。工作区：`/home/mamingyuan/Desktop/ros2_control-humble`。
> 状态：核心实现、形式化、控制代价量化、Gazebo 案例研究、上游行为分析**均已完成并验证**；
> 当前处于「把已有结果整理成合格学术成果」阶段，主要缺口是**论文级写作与一个性能侧的正结果**。

---

## 0. 给下一个会话的 60 秒简报

**这个课题是什么**：用户（研究生）的论文课题。师兄之前做过嵌入式框架 **FineMote**（源码+论文都在
用户手里，本工作区里有其思想描述），FineMote 用「一个线性注册表、正向 `Update()`、反向 `Handle()`」
的方式管理设备层次。用户想把同样的思想用到 ROS 2 `ros2_control` 的控制器层次上。

**现在做到哪了**：机制已经实现、形式化、并在真实 `ControllerManager` + 真实 Gazebo 里测过。
核心结论是一个**不可能性结果 + 一个修复**：

> 在 `ros2_control` 的单入口控制器模型（每个控制器一个 `update()`）下，
> 同一对父子之间的**双向同周期数据流不可满足**；上游对此**静默降级**为载荷顺序相关的一周期延迟。
> 显式两阶段契约（同一线性顺序、状态阶段正向、命令阶段反向）可以修复它，且不需要第二份拓扑顺序。

**用户已做的决定（2026-09-23，不要再重复询问）**：

1. 论文**暂停**：用户认为这份工作目前发不了论文（"先不管论文的事了"）。
   全部精力转向**把项目实现打磨好**。`doc/PAPER.md` / `PAPER_SKELETON.md` 保留但**不再推进**。
2. 方案选 **A + B 合并 → 已放弃论文路线**；C（Jazzy/Rolling 数据通路复现）**仍未做**，
   本机装不了（Ubuntu 22.04、无 Docker），需要用户提供环境。
3. 已完成的整改见 **`doc/REVIEW_RESPONSE_2026-09-23.md`**（外部评审 R1–R11 逐条处理）。

**下一个会话应该做的第一件事**：

1. 读 `doc/REVIEW_RESPONSE_2026-09-23.md`（评审整改现状，含**仍未做**的清单），
   再读第 12 节列出的核心文档。
2. **先读第 11 节的「不能声称」清单**，避免把已有的诚实结论写反、或把撤回过的错误结论重新写回。
3. 仍未闭合的项（详单见评审处理文档）：R11（文献查新与拆分基线）、
   R10 的上游端口端到端验证、R9 的 Gazebo 重跑、R8 的 TSan、R1 的阶段顶点充分性证明。

**与用户沟通的注意点**：用户明确表示想要**直接、不粉饰**的评估，并且**明确重视诚实的负结果**。
不要把负结果藏起来或包装成正结果。有不确定的地方直接说不确定。

---

## 1. 课题来源与当前定位

### 1.1 原始意图（用户原话的意图）

> 「这份文件夹是我正在研究的论文课题，内容是对 ros2_control 控制器做层次化管理。
> 师兄之前做过一份嵌入式框架 FineMote，源代码和论文都在该文件夹下。
> 现在我想依照 FineMote 的层次化管理，依次调度 ros2_control 的控制器。」
>
> 用户自己提的方案：「在控制器里加入 update 和 handle 两个部分，执行完 update 再去 handle
> （这只是大概的思路），如果必要的话可以在 ros2_control 里添加字段，指定 controller 的上下级。」

注意：用户这个直觉**方向是对的**，而且后来被证明是解决不可能性的关键。这不是外行建议。

### 1.2 FineMote 的机制（精确表述，论文里要写对）

FineMote 的调度核心是：**一个线性注册表**（由依赖注入的构造顺序建立），
`Update()` 按注册顺序**正向**遍历（采集/更新内部状态），`Handle()` 按**反向**遍历（计算输出/施加动作）。
**一个顺序，两个相反方向的遍历。**

关键洞察：两趟**不需要第二份拓扑顺序**。后序（postorder）的反转天然就是「父先于后代」的合法顺序，
所以命令阶段直接反向遍历同一份 `postorder`，不必再算一个「父先」的数组。
（注意：这不是「零存储」。当前内核里 `HierarchyPlan` 仍同时保存 `preorder` 与 `postorder`
两份 `O(|V|)` 线性化，另有按节点/端口分配的 scratch/frame/committed 缓冲；
两趟相对单趟只是**不增加量级**，不是不增加存储。）

### 1.3 曾经走过的弯路（已放弃，不要再回头）

| 方向 | 为什么放弃 |
|---|---|
| 用 URDF 关节祖先关系**推断控制依赖** | URDF 描述运动学，不能推断控制语义。已删代码。 |
| 声称「首次引入层次化调度 / 首次接口依赖图」 | **上游已经做了**（见第 2 节），这个声明不成立。 |
| 用 `∫v·r dt`（轮行程积分）作为叶状态 | 它可被父用 `position × r` 重新推导出来，父根本不需要子。案例研究第一版因此看起来「机制没用」。**必须选不可被父重推导的子状态**（改成一阶滤波器内部状态后才有意义）。这是**方法论陷阱，必须写进论文**。 |
| 用 RMS 轨迹误差证明机制收益 | RMS 对该场景**不敏感**（比值 0.97–1.00），纯延迟在里面几乎看不出来。必须用相位裕度这类对纯延迟敏感的指标。 |

### 1.4 当前定位（重定位后的研究问题）

**旧叙事（已放弃）**：给 `ros2_control` 引入层次化调度与接口依赖图。→ 上游已做，不成立。

**现行叙事**：

> 在 `ros2_control` 的单入口控制器模型下，同一对父子之间的**双向同周期数据流不可满足**：
> 「父本周读到子刚算出的状态」要求子先跑，「子本周用到父刚产生的参考」要求父先跑，两者矛盾。
> 上游的 state chaining 与 reference chaining **各自只解决单向场景**
> （估计器→控制器；控制器→执行器），**双向场景未被覆盖，且在单入口模型下无法覆盖**。
> 本文用「一个线性顺序 + 两个相反遍历方向」的显式两阶段契约修复它，
> 量化其运行时与集成代价，并给出上游不具备的可靠性契约
> （每周期有效性、最旧采样、整组提交不部分提交）。

---

## 2. 必须知道的上游事实（写论文前必须正面引用）

这些事实直接决定课题定位。**不能不知道，否则会写出站不住的声明。**

1. **上游（Jazzy+）已经导出 state interface**：`ChainableControllerInterface::export_state_interfaces()`
   / `on_export_state_interfaces()` / `on_export_state_interfaces_list()`。
   → 我们「Humble 无法导出 state 接口」这一点**只对 Humble 成立**，是版本限制不是新发现。
   - <https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ChainableControllerInterface.html>
2. **上游排序已考虑 state 链**：issue #1123（saikishor, 2023-10-01 提出，2026-04-11 completed 关闭）；
   更早 commit `69b3225` 已把 `get_following_controller_names` / `get_preceding_controller_names` /
   `controller_sorting` 改为考虑 `state_interface_configuration().names`。
   - <https://github.com/ros-controls/ros2_control/issues/1123>
   - <https://github.com/ros-controls/ros2_control/commit/69b322528be7b290e80908518cabebaad018d3a3>
3. **上游已缓存 lifecycle id**：Jazzy `ControllerInterfaceBase::get_lifecycle_id()` 文档明确写
   "cached internally to avoid calls to `get_lifecycle_state()` in the real-time control loop"。
   → 我们「`get_current_state()` 会在实时路径分配」的发现**只对 Humble 成立**。
4. **上游版本与差异**（已逐字节核实）：
   - master HEAD = `b0c14b5`（2026-09-17）。
   - Jazzy 的 `controller_manager.cpp`：`controller_sorting` 出现 **0** 次，
     `build_controllers_topology_info` / `update_list_with_controller_chain` 出现 **6** 次
     ⇒ Jazzy 已换成 master 那套递归插入启发式。
   - 这两个函数在 Jazzy 与 master 之间**去注释/空白后逐字节相同**
     ⇒ `doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` 的移植结论**直接适用于 Jazzy**。
   - **Humble 与 master 的行为差异**：Humble 用 `std::stable_sort` + `controller_sorting`，
     是**确定的 reference-first**；master/Jazzy 用递归插入 + `if (already in ordered) return;`，
     是**加载顺序相关**的（先插入者胜）。
5. **反例防护结论（重要）**：**未发现**任何上游文档或 PR 禁止「同一控制器对同时声明 reference 与
   state 边」。因此表述必须是「**该组合未被覆盖**」（uncovered），**不是**「上游禁止」。
   issue #2189 只澄清了 preceding/following 的术语。

---

## 3. 已确立的结论（论文的核心材料）

### 3.1 形式化结果 —— `doc/FORMAL_MODEL.md`

设有向图 `G=(V,E)`，边分 reference 边与 state 边。单入口模型要求每个控制器每周期恰好被调用一次。

| 编号 | 内容 | 状态 |
|---|---|---|
| **定理 1** | 单趟调度**不可能**同时满足双向依赖 | 已证；并由**穷举全部 6 种执行顺序**的测试验证（无一种两向都新鲜） |
| **推论 1** | 若单趟被约束为 reference-consistent，则**每一条 state 边**都陈旧 ≥ 1 周期 | 已证 |
| **定理 2** | 链上滞后 = 深度 `D`，恰好（有下界证明 + 构造） | 已证 |
| **定理 3** | 两趟**充分**：只需一个线性化，存储 `O(\|V\|)` | 已证 |
| **推论 2** | `G` **无环 ⇒ 该表示可覆盖**（充分条件，2026-09-23 修正；`G` 有环**不能**推出不可满足） | 已证；配置期校验已实现 |

### 3.2 控制代价 —— `doc/CONTROL_COST_OF_LAG.md`

滞后 `L` 周期 = 纯传输延迟 `L·Δt`；相位裕度损失 `ω_c·L·Δt`；带宽上限 `f_c ≤ B/(2πLΔt)`。

实测（`kp=10, kd=600, Δt=1 ms`，穿越频率 ≈95.5 Hz）：

| L | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| 相位裕度 | 90.0° | 55.6° | 21.2° | **−13.1°（时间域确认发散）** |

低带宽（≈9.5 Hz）下 L=3 仍有 79.5° ⇒ 影响几乎可忽略。

**工程结论**：30° 裕度预算下，**深度 2 把闭环带宽压到 42 Hz，深度 4 压到 21 Hz**。

补充（同一文档第 8 节）：摩擦与量化**不改变**深度代价（`kd_max` 差异 <0.1%）；
实测 `kd_max(0):(1):(2):(3) = 1 : 0.502 : 0.311 : 0.225`，与理论 `1 : 1/2 : 1/3 : 1/4` 吻合。
**执行器饱和会把发散变成有界极限环**（kd=3000、lag=0 时线性发散但饱和下有界）——
「不炸了 ≠ 能工作」，在饱和系统上做实验必须同时报告跟踪误差。

> ⚠ **2026-09-22 更正**：上面这组 `kd_max` 比值（`1 : 0.502 : 0.311 : 0.225`）
> **是显式 Euler 积分的稳定性上限 `Δt·kd = 2`，不是控制回路的可用增益上限**，
> **不得再作为"与理论吻合"的证据引用**。
> 该格式中有效阻尼恰为 `kd`，故假上限天然按 `1/(L+1)` 缩放，看起来与理论一致。
> 已用精确逐步更新重测：`L=1,2,3` 的真实上限为 **1582.8 / 856.6 / 591.8**（比值 1 : 0.541 : 0.374），
> `L=0` **无上限**。摩擦结论也**只在 `f_c ≲ 2%·kp·A` 时成立**，越界后变化达 +10.9%。
> §8.3 的饱和结论**方向正确**（已用正确积分独立复现：线性发散、饱和有界，
> 且饱和后 RMS ≈ 1/√2，即输出与指令基本无关），但旧表格的数字来自错误积分。
> 详见 `doc/SCHEDULING_PERFORMANCE_COST.md` 与 `doc/CONTROL_COST_OF_LAG.md` §8（已重写）。

### 3.3 实验结论汇总（每条都有可复现证据）

| 结论 | 证据 | 文档 |
|---|---|---|
| 三种实现（staged group / 原生 chaining / 手写 composite）**输出逐位一致** | `test_hierarchy_comparison` | `HIERARCHY_FAIR_COMPARISON.md` |
| 原生 chaining 在某叶失败后**仍更新健康兄弟**（joint2 旧值、joint3 新值）；本方案整组不提交 | 两叶 fork 故障测试 | 同上 |
| 执行组运行路径**零分配**（配置后）；库宿主同样零分配 | 全局 `operator new` 计数：100 次调用 = 0 | `STAGED_EXECUTION_GROUP_EXPERIMENT.md` |
| **不更快**：中位耗时约为手写 composite 的 **2 倍** | 200 周期 ×3 次 | `HIERARCHY_FAIR_COMPARISON.md` |
| **省集成代码不是 manager 独有**：库宿主同样做到「加一个节点 = 一条配置」 | `library_host_matches_staged_group` | `WIRING_COST_ANALYSIS.md` |
| ⇒ **Gate B 不成立，不应继续扩大 `ControllerManager` 改动** | 同上 | `WIRING_COST_ANALYSIS.md` |
| 双向边在上游 master 下**不报错、不告警**，静默降级为**加载顺序相关的一周期延迟**；三级链上出现**逐边混合**的陈旧模式 | `research/topology_analysis/upstream_order_sim.py` | `BIDIRECTIONAL_EDGE_ANALYSIS.md` |
| Humble 真实比较器下双向对**总是 reference-first、与注册顺序无关** ⇒ state 边系统性陈旧 | `test_upstream_ordering.cpp`（2 用例） | 同上 §9 |
| `configure_controller` **完全不校验**声明的 state 边 ⇒ 冲突完全静默 | 同上 | 同上 |
| 真实 `ControllerManager`：单趟下 root 信息陈旧 **2 个周期**（=深度），两趟下全 **0** | `test_two_phase_execution.cpp` | `TWO_PASS_VS_SINGLE_PASS.md` |
| 真实 Gazebo 闭环：状态边陈旧 **单趟 = 1 周期、两趟 = 0 周期**（50 Hz：20 ms；20 Hz：50 ms；两趟 2/2 复现） | `case_study/` + `scripts/measure_*.py` | `GAZEBO_CASE_STUDY.md` |

### 3.4 两趟 vs 单趟的定量证明 —— `doc/TWO_PASS_VS_SINGLE_PASS.md`

| 调度器 | 上行阶跃 lag | 下行阶跃 lag |
|---|---|---|
| 单趟、父先（`ros2_control` 现状方向） | **depth − i**（根最陈旧） | 0 |
| 单趟、子先 | 0 | **i**（叶最陈旧） |
| **两趟、同一顺序** | **0** | **0** |

即：单趟必然有一个方向陈旧且正比于深度；两趟两个方向都同周期，且**复用同一份线性化**
（仍为 `O(|V|)` 存储，量级不变；**不需要**为命令阶段再维护第二份拓扑数组）。

---

## 4. 代码地图

### 4.1 包结构（准确，2026-09-21 核实）

```text
hierarchical_control/                      # 主交付物：独立 header-only 包，namespace hierarchical_control
  package.xml / CMakeLists.txt / README.md
  include/hierarchical_control/
    hierarchy.hpp                          # 计划生成 + 校验（纯 C++，可脱离 ROS）
    staged_controller_interface.hpp        # 双阶段接口 + StagedFrame 数据契约 + 端口声明
    staged_execution_group.hpp             # 内核：两阶段 + 帧校验 + 整组提交
    two_phase_controller_interface.hpp     # 轻量 mixin：update_phase() / handle_phase()
  test/
    test_execution_group.cpp               # 11 个内核单测（不创建 ControllerManager）
    test_stale_state_cost.cpp              # 6 个单测（滞后代价）

controller_manager/                        # 薄适配层（唯一改动的核心函数是 update()）
  include/controller_manager/controller_manager.hpp
  src/controller_manager.cpp
  include/controller_manager/hierarchy.hpp                 # 兼容 shim（using 转发到库）
  include/controller_manager/staged_execution_group.hpp    # 兼容 shim
  test/test_staged_controller/             # 应用控制器（staged 模式 + native 单相模式 + two-phase mixin）
  test/test_composite_controller/          # 基线：手写 composite
  test/test_composite_library/             # 基线：通用 composite 库宿主
  test/test_staged_execution_group.cpp     # 6 个 manager 级验收测试
  test/test_hierarchy_comparison.cpp       # 5 个对照测试
  test/test_two_phase_execution.cpp        # 2 个两趟测试
  test/test_upstream_ordering.cpp          # 2 个上游排序测试

case_study/                                # Gazebo 案例研究（package hierarchical_control_case_study）
  urdf/diff_drive.urdf                     # 含占位符 __CONTROLLERS_YAML__
  world/case_study.world                   # 含 libgazebo_ros_state.so（WorldPlugin）
  config/controllers_phaseA.yaml
  config/controllers_phaseB.yaml.in        # 占位符 __RATE__ __TWO_PHASE__ __LEGACY__
  include/case_study/{travel_registry,wheel_controller,chassis_controller}.hpp
  src/{travel_registry,wheel_controller,chassis_controller}.cpp
  case_study_plugins.xml
  scripts/{phase_a.sh,phase_b.sh,measure_joint_states.py,measure_tracking.py}

research/
  cycle_tree/CMakeLists.txt                # 前序：不依赖 ROS 的独立标量内核构建入口
  topology_analysis/upstream_order_sim.py  # 上游排序算法的忠实移植（决定性实验）

doc/                                       # 见第 12 节
```

依赖方向：`controller_manager → hierarchical_control → controller_interface`（**无环**）。

### 4.2 关键 API（照抄即可用）

**`hierarchical_control::TwoPhaseControllerInterface`**（`two_phase_controller_interface.hpp`）
—— 最轻量、管理器级两趟用的就是它：

```cpp
class TwoPhaseControllerInterface
{
public:
  virtual ~TwoPhaseControllerInterface() = default;
  // FineMote 的 Update：管理器 REVERSE（子先于父）遍历时调用
  virtual controller_interface::return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;
  // FineMote 的 Handle：管理器 FORWARD（父先于子）遍历时调用
  virtual controller_interface::return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;
};
```

**`hierarchical_control::StagedExecutionGroup`**（`staged_execution_group.hpp`）—— 完整契约：

```cpp
struct StagedGroupMember {
  std::string name;
  controller_interface::ControllerInterfaceBaseSharedPtr controller;
  std::vector<std::string> command_interfaces;
};

// manager 宿主：从 manager 管理的控制器构建并校验计划；配置错误抛异常
static std::shared_ptr<StagedExecutionGroup> create(
  const std::vector<StagedGroupMember> & members, std::int64_t max_age_ns = 0);

// 库宿主：直接从已解析的节点实例构建；无 ControllerManager 依赖、无 lifecycle
struct Spec {
  std::vector<std::string> names;
  std::vector<StagedControllerInterface *> instances;
  std::vector<std::string> parents;   // 根为 ""
};
static std::shared_ptr<StagedExecutionGroup> create_library(
  const Spec & spec, std::int64_t max_age_ns = 0);

// 运行路径：只做指针索引，无查找、无构造、无分配
StagedResult run(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
StagedResult run_ns(std::int64_t now_ns, std::int64_t period_ns) noexcept;

// 生命周期：只在 manage_switch() 之后刷新（缓存 members_active_，避免实时路径查 lifecycle）
void refresh_member_active_state() noexcept;
bool members_active() const noexcept;
bool owns(const controller_interface::ControllerInterfaceBase *) const noexcept;
```

周期顺序（写在头文件注释里，是论文可直接引用的规范）：

```text
update stage : postorder，FORWARD   （子先于父）
reference    : 根的一次外部快照
handle stage : 同一顺序，BACKWARD   （父先于子）
validate     : 周期标识、采样年龄、有限性
commit       : 叶 scratch 拷入控制器自有的 command sink
```

失败语义：失败周期**不调用任何 command sink**，所以提交的硬件命令不会是新旧混合。
保留上一周期命令**不是**硬件安全策略——应用必须自己定义故障动作。

**`ControllerManager` 新增 API**（`controller_manager.hpp`）：

```cpp
std::shared_ptr<StagedExecutionGroup> staged_execution_group() const;  // line 143
controller_interface::return_type set_staged_execution_group(...);      // 设置（可拒绝）
void clear_staged_execution_group();                                    // 清除
controller_interface::return_type set_two_phase_execution(bool);        // line 160（2026-09-23 起有返回值）
bool two_phase_execution() const;                                       // line 163
```

`set_two_phase_execution(true)` **会拒绝**（返回 `ERROR`，状态不变）当存在实现
`TwoPhaseControllerInterface` 的控制器①已是当前 staged group 成员，或②声明了
`!= 0 && != 管理器频率` 的 `update_rate`。`set_staged_execution_group()` 做反方向镜像拒绝。
原因：两条新路径都每周期执行一次、都传管理器周期，没有原生循环的逐控制器降频门控
（`controller_manager.cpp:2400–2416`），重叠或降频会被静默破坏。详见
`doc/REVIEW_RESPONSE_2026-09-23.md` §R7。

私有成员：`staged_group_`(536，`mutable`，原子发布)、`no_two_phase`(541)、
`struct TwoPhaseEntry`(542)、`enum class TwoPhaseAdmission`(548)、
`two_phase_enabled_`、`two_phase_entries_`(559，`shared_ptr<const vector>`，原子发布)。
成员集**只在非实时线程重建**，`update()` 每周期一次 `std::atomic_load` 只读。

### 4.3 `ControllerManager::update()` 的实际形态（`src/controller_manager.cpp` line 2323 起）

```text
line 2355  若 (two_phase_enabled_ && two_phase_entries_dirty_) 用 rt_controller_list 重建条目（不加锁！）
line 2362  Pass 1 "Update"  : 反向遍历 rt_controller_list → update_phase()   （子先于父）
               跳过条件：index == no_two_phase 或控制器不 active 或 switch 挂起
line 2386  legacy 单相循环  : 跳过 staged_group_ 的成员，也跳过 two-phase 成员
line 2430  Pass 2 "Handle"  : 正向遍历同一列表 → handle_phase()              （父先于子）
```

`two_phase_execution` 参数在 **两个构造函数**里都用 `get_parameter` 读取（line 289 / 336）。

---

## 5. 环境与复现命令（全部实际跑通过）

### 5.1 环境事实

- Ubuntu 22.04，GCC 11.4.0，**只有 ROS 2 Humble**，**没有 Docker** ⇒ Jazzy/Rolling 装不了。
- Gazebo Classic 11.10.2 + `gazebo_ros2_control` 已装。
- **工作区没有 `.git`**。正式仓库在 Windows `D:/2027-1/FineMote/ros2_control`（origin = 用户 fork）。
  同步要**按文件清单拷贝，不要整体覆盖**。
- `/tmp` 在部分执行环境中**按调用隔离**，构建产物必须放在工作区内（用默认 `build/`、`install/`）。
- 全量重编 `controller_manager` ≈ **15–16 分钟**；只改测试文件增量 ≈ **1 分钟**。

### 5.2 构建与测试

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"     # 必需！否则受限环境下 gtest 会在退出时段错误

# 库单测（2 个程序 / 17 个 gtest）
colcon test --packages-select hierarchical_control --output-on-failure

# 管理器侧关键测试（6 个）
ctest --test-dir build/controller_manager \
  -R "test_staged_execution_group|test_hierarchy_comparison|test_two_phase_execution|test_upstream_ordering|test_cycle_tree_contract|test_hierarchy$" \
  --output-on-failure

# 非 ROS 的独立标量内核
cmake -S research/cycle_tree -B build_standalone -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_standalone -j4 && ctest --test-dir build_standalone --output-on-failure
```

最近一次验证结果：库 ctest **2/2 程序通过**（11 + 6 gtest）；管理器关键 **6/6**；
standalone **1/1**。ctest 注册表里只有上述 6 个管理器测试与本项目相关
（`build/` 里残留的 `test_hierarchical_controller_executor`、`test_controller_hierarchy_builder`、
`test_urdf_hierarchy` 是**已删除源码的陈旧二进制，未注册进 ctest**，可忽略）。

### 5.3 Gazebo 案例研究

```bash
# Phase A：只跑原生 chaining（对照组）
bash case_study/scripts/phase_a.sh

# Phase B：同一批控制器，单趟 vs 两趟
# usage: phase_b.sh <update_rate> <two_phase:true|false> <duration>
bash case_study/scripts/phase_b.sh 50 true  40
bash case_study/scripts/phase_b.sh 50 false 40

# 指标脚本
python3 case_study/scripts/measure_joint_states.py <duration>
python3 case_study/scripts/measure_tracking.py <duration> <rate> [model] [mode]
```

已测矩阵：50 Hz 单趟 = 1 周期（20 ms）、50 Hz 两趟 = 0；20 Hz 单趟 = 1 周期（50 ms）、
20 Hz 两趟 = 0。两趟 **2/2 复现**。

---

## 6. 待办事项（按优先级，每项含精确入口）

### P0 —— 论文写作（真正的瓶颈）

已有材料足够支撑一篇论文：不可能性结果（定理 1）+ 充分性修复（定理 3）+
控制代价定律（相位裕度/带宽上限）+ 真实系统验证（真实 `ControllerManager` + Gazebo）。
**缺的是把它组织成学术文本**，以及把这些还没成文的点补上：

- 方法论陷阱一节（第 1.3 节那两个坑）：**必须写**，否则审稿人会问「为什么第一版看不出差别」。
- 「不更快」的解释文章 → **已完成**：`doc/WHY_NO_SPEEDUP.md`。
  要点：我们测的是 **CPU 时间**，FineMote 主张的是**控制延迟**；两者不是同一个量。

### P1 —— 性能侧正结果（**已调查，结论为负，2026-09-22**）

**结论：没有性能侧正结果，且原因已定位到结构性层面。**
完整报告见 `doc/SCHEDULING_PERFORMANCE_COST.md`。三条独立原因：

1. **低带宽回路本来不敏感**：底盘 `f_c ≈ 0.5 Hz`，一个周期只损失 ~3.6°。
2. **能暴露效应的回路，其小信号跟踪误差 RMS 不敏感**：把"可用增益上限"当作要量化的量时，
   在高带宽回路上 RMS 被参考自身主导（≈ `1/√2`），无法区分调度。
   即**能用跟踪误差看到收益的场景，与滞后造成危害的场景，不是同一个场景**。
3. **原以为可作为证据的 `kd_max` 比值是数值伪影**（见 §3.2 更正）：
   `1 : 0.502 : 0.311 : 0.225` 是显式 Euler 的 `Δt·kd = 2`，不是控制上限。

**已交付的替代正结果**（不是性能收益，是机制正确性的定量证据）：
`hierarchical_control/test/test_scheduling_performance.cpp`（4 用例）
用精确逐步更新把"调度滞后 `L`"与"可用增益上限"接起来——
`L=1,2,3` 上限 `1570.8 / 785.3 / 523.5`（比值 2.000 / 3.001），`L=0` 无上限；
且代价由**绝对延迟** `L·Δt` 决定（同样 1 周期，100 Hz 损失是 1 kHz 的 10.00 倍）。

**不要**把它包装成性能收益。若仍要追性能收益，唯一未试的方向是
**大信号/阶跃指标**（`HANDOFF_MANUAL.md` §8 第 2 条），但这需要新的实验设计。

### P2 —— Jazzy/Rolling 数据通路复现（**用户环境阻塞**）

Humble 不导出 state interface，所以无法在 Humble 上验证「父真的读到了陈旧值」。
逻辑级移植已完成（`upstream_order_sim.py` + 逐字节函数比对），但**运行真实 Jazzy 代码**做不了：
本机 Ubuntu 22.04、无 Docker。**用户已确认暂时提供不了环境**，该条挂起。
**这一步做完之前，不要把「上游有 bug」写进论文**；只能说「上游在双向场景下静默降级」。

### P2.5 —— 元编程：把契约前移到编译期（**原型已完成并测试，2026-09-22**）

**交付**：`hierarchical_control/include/hierarchical_control/static_topology.hpp` +
`hierarchical_control/test/test_static_topology.cpp`（5 用例）+
`hierarchical_control/test/test_static_topology_negative.py`（编译语料，4 文件）+
`doc/METAPROGRAMMING_CONTRACT.md`。**库 ctest 6/6 通过。**

**已实现**：层级表示为类型链 `Node<Name, Parent>`（`Parent = void` 为根），
每个节点类型在编译期暴露根到自身的 `ancestry`；
`Descendant<Name, Parent>` 在 `Name` 已出现于父的 `ancestry` 时**编译期拒绝**（命题 M1）。
覆盖**自环**与**任意长度祖先复用**（即闭任意长度 ≥ 2 的环）。
语料含一个**必须编译通过**的对照文件，使该检查**可证伪**——
否则"把所有文件都判失败"会伪装成通过。

**两个必须记住的措辞（否则就是过度声称）**：

1. 这是"**编译期拒绝**"，**不是**"不可表达"——
   2-环是**可以写出**的类型表达式（`Descendant<A, Descendant<B, Root<A>>>`），只是被拒绝。
   原待办里写的"类型上无法写出"是**错的**，已更正。
2. 只覆盖**树/参考方向**。**非祖先状态边**（共享子节点被另一个父消费）
   在此表示中**根本无法表达**，故编译期**无话可说**，**仍必须**由运行期校验负责。
   定位是"**编译期能力 + 运行期回退**"，**不是取代**。
   另有：拓扑常来自 YAML、插件经 `pluginlib` 动态加载，这类**不覆盖**。

**编译期成本（多形状已实测，见 `doc/COMPILE_COST.md`）**：
头文件固定 **0.22–0.25 s**；宽扇出 **1.28 ms/子节点（与宽度无关）**；
深链 **3.35 / 4.07 / 5.89 / 9.63 ms/节点**（深度 8/16/32/64，随深度增长）。
在真实 `controller_interface` TU（~10 s）中增量 **≲5% 且落在噪声带内**。
**先前"9 ms/节点"已更正**——那是深度 64 的数字，不是通用常数。

**追加完成（同日）：拓扑契约接合**
`hierarchical_control/include/hierarchical_control/topology_contract.hpp` +
`test_topology_contract.cpp`（6 用例）+ 2 个所有权编译负例 + `doc/TOPOLOGY_CONTRACT_JOIN.md`。
一份编译期 `BoundNode` 链**同时**产出：编译期检查（无环 / 量纲 / **端口所有权**）
与运行期 `SpecRows`（names/instances/parents，根在前）。
所有权不变式：端口名必须是 `<owner>/<local>`，且 **owner 必须是拓扑中的控制器**——
这把本项目实测的"声明了没人导出的接口"向上游静默行为**推进了一步**。
`rows_are_well_formed` 让计划在**进入内核之前**复核，且**非空洞**（7 种畸形输入全被拒）。
**边界**：只查 owner **是否存在**，**不查** owner 是否**真的导出**。

**追加完成（同日）：接合到内核**
`topology_binding.hpp` + `test_topology_binding.cpp`（5 用例）：
`create_library_group(binding)` 直接返回**可运行的** `StagedExecutionGroup`。
全链路为：**一份编译期定义 → 三层检查 → 计划 → 适配 → 真实执行组**。
测试刻意真实构造执行组并跑一个周期，覆盖"适配后的 Spec 通过自身检查却被内核拒绝"这一模式。
（顺带：单节点用例最初复用了三节点契约，**被所有权检查在编译期正确拒绝**，
已改用其自己的无端口契约——这是检查按预期工作的例证。）

**追加完成（同日）：维度/单位检查**
`hierarchical_control/include/hierarchical_control/dimensional_interfaces.hpp` +
`test_dimensional_interfaces.cpp`（6 用例）+ 2 个编译负例 + `doc/DIMENSIONAL_INTERFACES.md`。
量纲是**真代数**（指数向量，`Position/Time == LinearVelocity` 等），
故能区分**力矩 vs 能量**（力矩带角度指数 `[N·m/rad]`）、**角度 vs 位置**这类
"物理上合理但错误"的接线。**边界**：它检查的是"**声明之间的量纲一致性**"，
**不是**本项目实测的"声明了没人导出的接口"（后者仍需运行期），
**也不检查单位**（米/毫米量纲相同）。**尚未**接到执行组的端口声明。

**追加完成（同日）：量纲接入端口声明**
`typed_ports.hpp` + `test_typed_ports.cpp`（5 用例）+ `doc/PORT_DIMENSIONS.md`。
端口**只声明一次**（类型），内核所需字符串与 `Contract` 均由它生成 ⇒ 不会漂移。
父子端口的**名字 + 顺序 + 量纲**可编译期检查（量纲检查是内核做不到的）。
mixin 控制器**真的能在执行组里运行**（端到端 `committed`）。

**未完成**：
1. **状态端口的语义区分未下探到类型层**（`staged_state_ports()` 同时承载发布与消费，
   内核按拓扑而非名字区分；在类型层建模会与内核规则重复）；
3. 多形状的编译成本测量；4. 把节点名带进 `static_assert` 诊断以提升可读性。

### P3 —— 其它未做项（非必需，论文里明说为 future work）

- 「估计器/命令分离」的第三种基线（第 4 个对照）。
- 多频 / 异步执行；动态拓扑；生命周期回滚。
- 更严格的故障一致性验证（目前只有 staged group 有该证据，两趟模式没有）。
- 实测两趟的额外开销（多一次列表遍历 + 每控制器一次虚调用），要与单趟对比并如实报告。
- TMP 编译期维度/单位检查。
- `Spec::parents` 显式父子字段接到配置（YAML/参数入口）。
  注意：**两趟之后该字段不是必需的**（reference 边的一个线性化就够了）。

---

## 7. 陷阱与注意事项（踩过的坑，逐条）

1. **`ROS_LOG_DIR`**：受限环境下 `~/.ros` 不可写，gtest 会在 `SetUpTestSuite` 抛异常并在退出时
   崩溃（现象是 `rclcpp::shutdown()` 段错误）。必须 `export ROS_LOG_DIR="$PWD/log/ros"`。
2. **实时路径禁止获取 `controllers_lock_`（会导致死锁）**：
   `switch_controller()` 在**持有 `controllers_lock_`** 的同时等待实时线程应用切换，
   所以实时线程一旦抢锁 → 不推进 `used_by_realtime_controllers_index_` →
   等待线程在 `wait_until_rt_not_using()` 里**永久自旋**。
   - 正确做法（已实现）：非实时线程（`set_two_phase_execution`）加锁构建成员集并**原子发布**；
     实时路径只做一次 `std::atomic_load` 后只读，**不重建**（2026-09-23 改；旧的"脏标志 + 实时重建"
     会把 `reserve/push_back/sort/析构` 放进实时路径，且与非实时写者构成数据竞争）；
     实时路径只用**脏标志**，在下一个周期开头用本线程已持有的 `rt_controller_list` 重建
     （O(n) `dynamic_cast`，每个切换最多一次），**全程不加锁**。
   - 这也解释了为什么 `refresh_member_active_state()` 特意不加锁。
   - **任何接入 `ControllerManager` 实时循环的新机制都必须遵守。**
   - 调试提示：rclcpp 装了 SIGTERM 处理器，挂死时 `timeout` 默认杀不掉，要用 `timeout -s KILL`。
3. **全量重编 `controller_manager` ≈ 15–16 分钟**。改了 `hierarchical_control` 的头文件就会触发
   全量重编（这是预期，因为内核是 header-only）。
4. **`members_active_` 缓存**：只在 `manage_switch()` 之后刷新。控制器不经 switch 自行改变
   生命周期状态会让缓存过期（已写入限制）。
5. **库宿主在第一次 `update()` 惰性建内核**（一次性分配）；应改为 `on_activate` 后显式触发。
6. **`test_controllers_chaining_with_controller_manager` 是既有 flaky 测试**：
   负载下偶发失败（计数偶发偏差），隔离重跑 3/3 通过。本次改动没有触碰原生 chaining 逻辑。
7. **`plan_.preorder` 仍被 `hierarchy.hpp` 赋值**（兼容保留），但两阶段内核**不再使用它**
   （命令阶段改为反向遍历 `postorder`）。看到它是正常的。
8. **`controller_interface/staged_controller_interface.hpp` 已删除**（移入库，不留 shim，
   否则形成包依赖环）。实现该接口的插件现在依赖 `hierarchical_control`。
9. **运行 `update()` 时会分配内存是正常的**：Humble 的 `ControllerSpec` 按值复制。
   零分配只对 `StagedExecutionGroup::run()` 和库宿主成立，**不要**声称整个 `update()` 零分配。
10. **Gazebo 特有坑**（详见 `GAZEBO_CASE_STUDY.md`）：
    - `set -u` 会破坏 ROS 的 setup 脚本 → 用 `set +u`；
    - `~/.ros`、`~/.gazebo` 只读 → `export HOME="$LOG/home"` + `ROS_LOG_DIR`；
    - 需要 `gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so`；
    - `libgazebo_ros_state.so` 是 **WorldPlugin**，必须写在 world 文件里，
      且话题是 **`/model_states`**（不是 `/gazebo/model_states`）；
    - URDF 指向模板 YAML → 必须先生成 YAML 再替换运行路径；
    - 声明参数用控制器基类的 `auto_declare<...>(...)`（**不是** `get_node()->auto_declare`），
      否则 `ParameterAlreadyDeclaredException`；
    - PI 必须做积分限幅（±2）+ 输出限幅（±8），否则 Gazebo 求解器 NaN 发散；
    - 每次运行用**唯一 `GAZEBO_MASTER_URI` 端口**，否则偶发 `free(): invalid pointer`；
    - 底盘 spawn 高度 `-z 0.16`（轮子最低点比模型原点低 0.15 m）；
    - Gazebo 在本 VM 上约 **1/3 的启动会崩溃**，重跑即可。
11. **`two_phase_execution` 参数曾经「没效果」**，原因是**加参数后没有重新构建
   `controller_manager`**。改核心包后一定要重建。
12. **编译告警即错误**：`-Werror` 开着（`conversion` / `shadow` / `float-conversion`），
    新测试代码里要显式转型、避免变量遮蔽。
13. **覆盖全局 `operator new/delete` 会触发 `-Wmismatched-new-delete`**：
    用文件级 `#pragma GCC diagnostic ignored` 处理。

---

## 8. 构建层级时的正确做法（设计准则）

1. **子状态必须不可被父重推导**。父能用输入和已知几何重算的量，不构成依赖，
   无法体现机制价值。要么用内部滤波器状态，要么用外部测量。
2. **评估指标必须对纯延迟敏感**。RMS 误差对 1 周期延迟不敏感（实测比值 0.97–1.00）；
   相位裕度、阶跃响应的初始滞后周期数才敏感。
3. **饱和会掩盖不稳定**。在带饱和的系统上必须同时报告跟踪误差，否则「没炸」会被误读成「能用」。
4. **因果对齐要用整数周期对齐**，不要用浮点时间戳对齐（时间戳抖动会污染结果）。
   Gazebo 案例里用的是「底盘用掉的行程」vs「轮子当时拥有的行程」的整数周期交叉对齐。
5. **一个线性顺序就够**。不要维护两份拓扑数组；后序的反转就是合法的父先顺序。

---

## 9. 用户偏好与沟通方式

- 用户想要**直接、不粉饰**的评估。有坏消息就说，别包装。
- 用户**明确重视诚实的负结果**（多次强调要「合格的学术成果」，而不是好看的故事）。
- 用户希望研究重定位到「**单入口控制器模型下同周期双向流不可满足**」，
  **而不是**「我们加了层次化调度」。写作时要贴住前者。
- 用户会中文提问，文档和回复用中文。
- 用户对课题的理解是到位的（两阶段思路是用户自己提的），不要居高临下地简化。

---

## 10. 工作区状态速查

| 项 | 值 |
|---|---|
| 工作区 | `/home/mamingyuan/Desktop/ros2_control-humble` |
| 版本控制 | **无 `.git`**；正式仓库在 Windows `D:/2027-1/FineMote/ros2_control` |
| 自研包 | `hierarchical_control`（主交付物）、`case_study`（Gazebo 验证） |
| 内核代码量 | 接口 125 + 执行组 447 + hierarchy 121 ≈ 693 code lines |
| 测试 | 库 17 gtest / 2 程序；管理器 4 文件 15 gtest；关键 6/6 ctest 通过 |
| 文档 | 15 份（见第 12 节） |
| 上游对照版本 | master `b0c14b5`（2026-09-17） |
| 最近验证 | 库 ctest 2/2、管理器关键 6/6、standalone 1/1 |

---

## 11. 明确的「不能声称」清单（论文写作时逐条核对）

- ❌ 首次提出层次化控制 / 树遍历 / 两阶段调度（FineMote 更早，嵌入式领域已有）。
- ❌ 首次用接口连接生成状态依赖（**上游已做**：PR #1021、issue #1123、commit 69b3225）。
- ❌ 比原生 chaining 或手写 composite **更快**（实测约 2 倍耗时）。
- ❌ 接入后 `ControllerManager::update()` **零分配**（只有 `run()` 与库宿主零分配）。
- ❌ 已验证硬实时 **WCET / deadline miss**（只测了分配次数与耗时中位数）。
- ❌ 仅凭 **URDF 推断控制语义**（已否决的方向）。
- ❌ 已支持**多频、异步、动态拓扑、生命周期回滚**（都没做）。
- ❌ **上游有 bug**（在完成 Jazzy/Rolling 数据通路验证前，只能说「未被覆盖 / 静默降级」）。
- ❌ **Gazebo 上的轨迹误差收益**（当前真值指标尚不可靠，且带宽太低、预期效应本就很小）。
- ❌ 两阶段是「新技术」（FineMote 的机制就是这个；本文的贡献是**不可能性论证 + 代价量化 +
  可靠性契约 + 在 `ros2_control` 上的落地与验证**，不是机制本身的发明）。
- ❌ **「每周期把可用增益除以 (L+1)」**（2026-09-22 新增）：该比值是显式 Euler 的
  `Δt·kd = 2` 伪影，已撤销。见 `doc/SCHEDULING_PERFORMANCE_COST.md`。
- ❌ 把「**深度 ↔ 控制权限**」说成**性能收益**（2026-09-22 新增）：
  它是机制正确性的定量证据，不是性能收益；本文**没有**性能收益。
- ❌ **无边界地**声称"摩擦/量化不改变代价"（2026-09-22 新增）：
  摩擦只在 `f_c ≲ 2%·kp·A` 时不影响上限，越界后达 +10.9%。
- ❌ **最少趟数是 NP 难的 / 两趟是最优调度**（2026-09-23 新增，**外部评审 R1**）：
  `κ ∈ {0,1,2}` 是平凡分类（任取顶点全序按正/反向二分边即可），
  "两趟最优"只限**完整阶段遍历次数**，不含 CPU 时间、端到端延迟或一般调度最优。
  见 `doc/PASS_LOWER_BOUND.md` 文首修正框、`doc/REVIEW_RESPONSE_2026-09-23.md` §R1。
- ❌ **`G` 成环 ⇒ 任何调度都不可能**（2026-09-23 新增，**外部评审 R2**）：
  正确的只有充分性方向"`G` 无环 ⇒ 该表示可覆盖"。反例与说明见 `doc/FORMAL_MODEL.md` §6。
- ❌ **"零额外存储"**（2026-09-23 新增，**外部评审 R10**）：只能声称
  "两趟**不需要第二份拓扑顺序**"；内核仍保存 `preorder`/`postorder` 两份 `O(|V|)` 线性化
  与按端口分配的 scratch/frame/committed 缓冲，两趟相对单趟只是不增加**量级**。
- ❌ **两条 manager 执行路径可以混用同一个控制器**（2026-09-23 新增，**外部评审 R7**）：
  现在配置期**拒绝**重叠成员与频率不匹配成员（见 API 一节）。
- ✅ **发布协议的并发模式已用 TSan 验证**（2026-09-24 更新）：racy 模式必报、atomic 模式干净
  （`hierarchical_control/test/run_tsan_publish_protocol.sh`）；
  但 ❌ **不能声称"整个 `ControllerManager` 已通过 TSan"**——那需要给全包另开 TSan 构建树，
  本机磁盘不允许；且 `std::atomic_*(shared_ptr)` 也不保证无锁。
- ❌ **首次提出**"双向同周期不可满足 / 反馈环需要单位延迟"（2026-09-24 新增，**评审 R11**）：
  这是同步数据流（Lee & Messerschmitt 1987：环上必须有延迟）、同步语言（不经 `pre()` 的环是因果性错误）、
  Kahn 网络、Simulink/Modelica 代数环的标准结果。见 `doc/RELATED_WORK.md` §1。
- ❌ **两趟/正反遍历是新原理**（2026-09-24 新增）：这就是 FineMote 的 `Update`/`Handle`
  （**arXiv:2608.04600，本次已抓取核实**）；且"逆后序是拓扑序"是教科书事实（Tarjan 1972）。
- ❌ **整组原子提交是行业标准做法**（2026-09-24 新增）：**未找到任何权威来源**；
  EtherCAT 的一致过程数据镜像是**不同层面**的类比。可以声称"我们定义并验证了这个契约"。
- ❌ **两趟模式也有"整组提交不部分提交"**（2026-09-24 新增，**实测否证**）：
  命令趟父先子后且直写 command handle，后段失败时前段已生效（`two_phase_mode_has_no_group_commit`）。
- ❌ **本项目"比 LET 更好"**（2026-09-24 新增）：LET 用全局逻辑时刻按定义解决同一问题；
  本项目只是不需要时基/双缓冲、不增加周期延迟，代价是要求可分解阶段与无环层次。
- ❌ **调度能力是改 `ControllerManager` 的理由**（2026-09-24 新增）：
  通用 composite 库宿主（单插件 + `create_library()`）复用同一内核且**更快**，
  管理器集成的独立价值在**逐控制器生命周期、局部激活、原生接口参与、工具可见性、故障隔离**
  ——见 `RELATED_WORK.md` §3。

**可以声称的**（已在本文第 3 节逐条给出证据）：上述不可能性命题、
定理 1–3 与推论 1–2、控制代价定律与实测吻合、双向边在上游静默降级、
三种实现输出一致与故障一致性差异、真实管理器与真实 Gazebo 中的滞后消除、零分配运行路径、
以及（2026-09-23 新增）**一个控制器每周期最多被一条执行路径执行一次**（有 6 个用例覆盖）。

---

## 12. 文档地图（哪些读、哪些过时）

### 必读（下一个会话按此顺序读）

| 顺序 | 文档 | 作用 |
|---|---|---|
| 1 | **本文** `doc/HANDOFF_MANUAL.md` | 唯一入口，全部上下文 |
| 2 | `doc/FORMAL_MODEL.md` | 形式化核心：模型、定理 1–3、推论 1–2、threats to validity、可/不可声称 |
| 3 | `doc/BIDIRECTIONAL_EDGE_ANALYSIS.md` | 决定性实验：上游双向边行为、移植验证、Humble 真实代码验证（§9） |
| 4 | `doc/CONTROL_COST_OF_LAG.md` | 工程意义：滞后 → 传输延迟 → 相位裕度损失 → 带宽上限 |
| 5 | `doc/GAZEBO_CASE_STUDY.md` | 案例研究：真实仿真闭环中的滞后测量、两个方法论坑、明确不声称项、复现命令 |

### 按需查阅

| 文档 | 作用 |
|---|---|
| **`doc/REVIEW_RESPONSE_2026-09-23.md`** | **外部评审 R1–R11 的逐条处理记录**（改了什么、怎么验证、哪些没做）。接手时优先读它，避免重复踩已知坑 |
| `doc/RELATED_WORK.md` | **相关工作与定位（评审 R11）**：逐条重叠分析（结论对本项目不利）、LET 对比、拆分基线对照、引用清单与不确定性 |
| **`doc/CODE_AUDIT_SCHEDULING_METAPROGRAMMING.md`** | **双向调度 + 元编程的代码审查记录**（2026-09-24）：修掉的 4 个问题、仍存在的边界、以及"既有测试是时间校准的"这一实测结论 |
| **`doc/REVIEW_REQUIREMENTS_RESPONSE_2026-09-24.md`** | **第二份评审（A–E）的处理记录**：A（静态父关系 vs 实际生成关系）、C（端口检查成为建组必经步骤）、D（模式标志与快照发布的两个真 race）已修；B（分叉树 typed builder）、E（跨模式准入）有明确计划 |
| `doc/REVIEW_REQUIREMENTS_2026-09-24.md` | 第二份评审原文（A–E） |
| `doc/REVIEW_HUMBLE_WORK_2026-09-23.md` | 外部评审原文（R1–R11） |
| `doc/TWO_PASS_VS_SINGLE_PASS.md` | 两趟 vs 单趟的阶跃响应定量证明 |
| `doc/PROJECT_REPORT_2026-09-21.md` | 项目全貌报告（给导师/新读者看）。⚠ **第 3 节「接口连接是依赖来源」的叙事已过时**，需按本文第 1.4 / 2 节修订 |
| `doc/HIERARCHY_FAIR_COMPARISON.md` | 同算法对照（输出/故障/分配/耗时）与诚实结论 |
| `doc/WIRING_COST_ANALYSIS.md` | Gate B 接线成本 + 通用 composite 库基线 + 最终结论（含包结构） |
| `doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md` | 阶段 A/B 设计与 manager 级验证记录 |
| `doc/CYCLE_TREE_EXPERIMENT.md` | 前序独立标量内核实验 |
| `hierarchical_control/README.md` | 库的对外文档（契约、两种宿主、校验规则、非目标） |
| `doc/WHY_NO_SPEEDUP.md` | 为什么没有性能正结果（CPU 时间 vs 控制延迟）以及负结果清单 |

### 历史文档（**不必读**，仅供追溯）

`doc/HANDOFF_NEXT_SESSION_2026-09-21.md`（上一轮交接，其「下一步」多已在本轮完成）、
`doc/HANDOFF_STAGED_GROUP_2026-09-20.md`、`doc/HANDOFF_CURRENT_2026-09-20.md`、
`doc/HANDOFF_2026-09-19.md`、`doc/hierarchical_research.md`。

---

## 13. 与 FineMote 的对照（论文里要摆清楚）

| 维度 | FineMote | 本工作 |
|---|---|---|
| 领域 | 嵌入式设备框架 | ROS 2 `ros2_control` 控制器 |
| 顺序来源 | 依赖注入的构造顺序（一个线性注册表） | 上游排序后的控制器列表（reference/state 边的一个线性化） |
| 两阶段 | `Update()` 正向 / `Handle()` 反向 | 状态阶段正向 `postorder` / 命令阶段反向同一顺序 |
| 存储 | 注册表 | `O(\|V\|)`，两趟复用同一份线性化（不需要第二份拓扑顺序；内核另有 preorder/postorder 与按端口大小的 scratch/committed 缓冲） |
| 显式契约 | 无（隐式由注册顺序保证） | 有：每周期有效性、最旧采样、整组提交不部分提交 |
| 本文新增 | —— | **不可能性论证**（定理 1）+ 代价量化 + 在上游的真实系统验证 |
