# 两阶段执行（Two-phase execution）——管理器最小特性分支

日期：2026-10-03　分支：`feature/two-phase-manager`（基点：上游 Humble `469f3055`）

> 这份文档说明**这个分支是什么、不是什么、怎么用、怎么和 `humble-work` 同步**。
> 权威的用户说明在 `controller_manager/doc/two_phase_execution.md`（英文，给上游看）；
> 这里记录范围、边界、验收状态与风险。

---

## 0. 一句话

在**不引入任何新包**的前提下，给 `ros2_control` 的 `ControllerManager` 增加一个**默认关闭**的
执行模式：实现 `TwoPhaseControllerInterface` 的控制器，每周期按**同一条控制器列表**跑两趟——
**反向趟** `update_phase()`（子先于父，拿到本周期子状态），**正向趟** `handle_phase()`
（父先于子，拿到本周期父参考）。未开启时行为与原生完全一致。

相对 `humble-work` 的研究线，这个分支**只保留"管理器模式 + 两趟"**，因此用户只需要
"一个打了补丁的 `controller_manager` + 自己的 chainable 控制器 + YAML 里一行
`two_phase_execution: true`"。

---

## 1. 范围（包含 / 不包含）

| 包含 | 不包含（留在 `humble-work` 研究线） |
|---|---|
| `controller_interface/two_phase_controller_interface.hpp`（新接口头，在**上游包里**） | `StagedExecutionGroup` 内核、`StagedControllerInterface`、帧语义、整组提交 |
| 管理器两趟调度：反向状态趟 + 正向命令趟 | 执行组的安装/清除 API、成员惰性策略 |
| **速率分桶**：低于管理器频率的成员按自己的桶运行，桶周期正确 | 跨桶边的有界接纳（本分支仍然**拒绝**跨桶边） |
| 准入与安全检查：跨模式边、列表顺序、同实例两名、速率桶、成员绝不被原生循环接管 | 编译期类型层（拓扑/量纲/契约/typed ports/manifest） |
| `two_phase_execution` 参数 + `set_two_phase_execution()` + `two_phase_rejected_controllers()` + `control_loop_busy()` | 编译内置控制器注册表（`StaticControllerRegistry`） |
| 执行状态**一个不可变快照**（模式 + 成员表 + 桶表，一次原子发布） | 声明的 WCET 可调度性报告、`atomic_activation` 回滚 |
| 测试（29 例）+ 可运行示例控制器 + 可运行 demo（URDF/YAML/launch） | 论文级分析层（`ΔPM`、`κ(D)`、Gazebo 测量脚手架）、TSan 脚本 |

**安全相关、不可裁剪的部分**（缺一个就会"静默出错"）：

| 必须保留 | 缺了会怎样 |
|---|---|
| 跨模式边拒绝 | 该边被两个调度排序 → 悄悄用上一周期的值 |
| 列表顺序拒绝（父必须在子之前） | 反向/正向两趟对那条边同向走错 → 计数正常但数据旧 |
| 同实例两名拒绝 | 同一对象一周期被推进两次 |
| 两趟成员绝不被原生循环接管（切换期间也跳过） | 切换的几个周期里悄悄退回单趟语义 |
| 任一 `update_phase` 失败 → 本周期不跑命令阶段 | 控制器在它刚刚拒绝的数据上继续算命令 |
| 速率规则（预算内才成边，默认同桶） | 跨桶边的精确滞后超过声明预算，静默超出可接受范围 |

---

## 2. 用法

### 2.1 写一个两阶段控制器

继承 `controller_interface::ChainableControllerInterface`（或 `ControllerInterfaceBase`）**并**
实现 `controller_interface::TwoPhaseControllerInterface` 的两个入口：

```cpp
controller_interface::return_type update_phase (time, period) noexcept;  // 摄取状态（子先于父）
controller_interface::return_type handle_phase (time, period) noexcept;  // 产生命令（父先于子）
```

控制器的原生 `update()` 仍然必须可用（它是关闭该模式时的路径）。示例见
`controller_manager/test/two_phase_example_controller/`。

### 2.2 打开（**一行 YAML，不需要改 C++、不需要新包**）

YAML：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    two_phase_execution: true
```

这一行就是全部采用成本：控制器怎么写、怎么加载、怎么配都不变；不实现该接口的控制器继续走原生
单趟循环，该模式只接管**主动实现接口**的那些控制器。

或用 C++（等价，便于部署脚本/测试）：

```cpp
manager->set_two_phase_execution(true);   // 全有或全无：任一控制器不满足准入就整体拒绝
manager->two_phase_rejected_controllers(); // 先问"现在哪些控制器进不来、为什么"
```

**启用被拒怎么办**：每条拒绝都点名控制器与原因，用户文档
（`controller_manager/doc/two_phase_execution.md` §4.2）有一张"拒绝码 → 你要改什么"的对照表。

> 设计决定（2026-10-03）：**不提供** `set_two_phase_execution_static<Binding>()` 这类模板入口，
> 也**不替换** `controller_sorting()`。理由见 `humble-work` 的
> `doc/STATIC_ADMISSION_DESIGN_2026-10.md` §8：前者是 C++ API 而用户的诉求是一行配置，
> 后者是全局行为变更而两趟路径已用"拒绝"代替"静默"。

### 2.3 跑 demo

```bash
ros2 launch controller_manager two_phase_demo.launch.py
ros2 control list_controllers -v      # 可以看到 is_chained
```

---

## 3. 已实现的契约（会被测试钉住）

1. **同周期双向新鲜**：两趟下父读到的子状态、子读到的父参考都来自本周期。
   实测数值（`test_two_phase_execution.cpp`）：三级链叶子阶跃 1.0 时，
   单趟 `root=0.0`；两趟 `leaf=1.0, mid=0.5, root=0.25`，且叶子命令 `-1.75`（单趟 `-1.0`）。
2. **每阶段每周期恰好一次**（含分桶：成员只在自己的桶里跑一次）。
3. **一个控制器只有一条执行路径**：开启后成员永不被原生循环调用，**切换等待期间也不**。
4. **失败包含（最弱诚实形式）**：任一状态阶段失败 → 本周期整条命令阶段不跑，命令接口保持上一周期值。
   **没有**跨控制器回滚（`handle_phase` 直写命令接口）。
5. **准入全有或全无**：拒绝时模式标志不变、成员表不变，并逐条给出人可读原因。
6. **安装/扩展执行路径要求控制循环空闲**；**移除路径**（`set_two_phase_execution(false)`）随时允许。

---

## 4. 已知限制（诚实的"不能声称"）

1. **没有整组原子提交**：跨控制器的中间失败不会回滚已写出的命令。命令接口只保证"不被本周期
   命令阶段覆盖"。
2. **跨速率桶的边按「滞后预算」有界接纳**：准入计算每条边的**精确**最坏滞后（见
   `controller_manager/doc/cross_rate_bound.md`），与 `two_phase_max_lag_cycles` 比较。
   **默认 0 = 两向同周期 = 与「两端必须同桶」完全等价**，所以没有 opt-in 的部署行为不变。
   预算**逐边**、不构成端到端保证：链式会累积（上界 = 逐边之和；实测三级链达到逐边预算的 1.5 倍，
   见 `cross_rate_bound.md` §6）。滞后同时以**纳秒**报告，可直接喂给控制代价定律
   （`ΔPM = 360·f_c·Δt`），但管理器本身不检查相位裕度/带宽要求。
3. **模式/成员是一个执行代，控制器列表不是**：因此"安装执行路径"必须在控制循环停止时做。
4. **只有管理器模式**：没有编译期类型层、没有 typed composite、没有静态注册表。
5. 依赖库未做 TSan 插桩；本分支未附带 `run_tsan_real_manager.sh`。
6. 未验证 `ros2_control_node + spawner` 的完整 launch（demo 文件已提供，见 §5）。
7. **ABI 变化**：`ControllerManager` 新增数据成员（`cycles_in_flight_`、`generation_`、
   `publish_generation` 等），与系统安装的 `controller_manager` **ABI 不兼容**。
   采用"覆盖层 + 一起重编"规则：源码编译本分支的 `controller_interface` 与 `controller_manager`，
   不要与二进制版混用。

---

## 4b. 证据（可复跑的测量，`test_two_phase_evidence`，4 例全绿）

这些是**给论文用的数字**，不是功能测试；跑法：
`./build/controller_manager/test_two_phase_evidence`（数字同时写进 ctest XML 的 `RecordProperty`）。

| 项 | 结果 |
|---|---|
| **滞后 = 距叶子的距离**（单趟）vs **0**（两趟），深度 1..4 | 例：深度 3 时单趟 `[0,1,2]`、两趟 `[0,0,0]`。把"一个数"变成**定律** |
| **分配 / 周期**（3 成员，400 周期，预热后） | 单趟 **14**，两趟 **8**——两趟**更少**，因为它省掉了每成员的 `is_controller_active()`（Humble 上会复制含 `std::string` 的 lifecycle State） |
| **时间 / 周期**（2 核 VM，仅报告不断言） | 单趟 **8 µs**，两趟 **3 µs** |
| **真实加载路径**：pluginlib 按类型字符串加载 + 参数经**真实的 `<name>.params_file` 机制**（spawner 用的那条）来自 demo YAML，管理器用 **demo 自己的 URDF** 构建 | 三级链阶跃 `1.0 → 0.5 → 0.25` **同周期**达成，说明 demo 的 YAML/URDF/插件三者一致 |
| **上游隐患**：卸载 chainable 控制器不会移除它导出的 reference 接口 | 见 `controller_manager/doc/upstream_finding_stale_reference_interfaces.md`；测试用公开 API 钉住，**不触碰悬垂指针** |

**证据驱动的第三个改动——而且它是个功能缺口（最重要的一条）**：
链根（非 chained）的**外部参考输入在两趟模式下被冻结**。上游只有一个地方调用
`ChainableControllerInterface::update_reference_from_subscribers()`——**融合的 `update()`**——而两趟路径
**按设计绕过**它；该方法又是 `protected`，管理器自己够不到。后果：一个由话题驱动的链根会**永远**命令它
最后一次看到的参考值，**没有任何报错、没有任何拒绝**，真实机器人只是不理会输入。
所有原测试都用了 `set_external_reference()`（直接设值），所以**全都漏了**；是 Gazebo 暴露的。

修法是把上游 `update()` 的**第三步**补进契约：`refresh_reference_phase()`，管理器在命令趟对该成员
（`chainable == nullptr || !is_in_chained_mode()`）**先刷新再 `handle_phase`**。
**验证过测试不是空转**：把刷新调用临时改成 `if (false && ...)` 后，单趟测试**通过**、两趟测试**失败**（值 0），
恢复后两者都通过。

### Gazebo 级验证（`two_phase_demo_gazebo.launch.py`）

> **怎么跑：`controller_manager/doc/gazebo_demo.md`** —— 逐步操作、预期输出、以及那个"约三次一遇"的
> gzserver 崩溃与重试脚本。下面只放结论。

**真实 Gazebo 物理 + 真实 DDS** 上的端到端复现，不是 mock 硬件：

```
ros2 launch controller_manager two_phase_demo_gazebo.launch.py
python3 <share>/controller_manager/two_phase_demo/record_chain_lag.py true|false
```

| 项 | 结果 |
|---|---|
| 硬件 | `gazebo_ros2_control/GazeboSystem`（真实 Gazebo 物理），控制器由**真实 spawner 经 DDS** 按类型字符串加载 |
| 启用方式 | **一行 YAML**（`two_phase_execution: true`）；日志确认 "requested by parameter: enabled" |
| 控制周期（来自消息里的共享时间戳） | **10.00 ms**（100 Hz） |
| **两趟 ON** | leaf 0.7450 **lag 0** ｜ mid 0.3742 **lag 0** ｜ root 0.1879 **lag 0** |
| **单趟 OFF** | leaf 0.7427 lag 0 ｜ mid 0.3712 **lag 1** ｜ root 0.1864 **lag 2** |

即：**"滞后 = 距叶子的距离" 与 "两趟全为 0" 这条定律在真实仿真器上成立**，数值正是理想首周期值
（0.75 → 0.5×0.75 → 0.25×0.75）。

**Gazebo 这一步抓到的两个真问题**（都不是仿真配置问题）：

1. **chained 成员在 `on_configure` 建的发布器会消失**。父节点激活时，管理器把子节点
   **deactivate → 设 chained 模式 → 再 activate**；于是"建在 configure、销毁在 deactivate"的资源
   在 chained 成员上**静默丢失**——实测 `/tp_leaf/cycle_diagnostics` 在图上变成 Unknown，而未 chained 的
   `/tp_root/...` 正常。发布器等 ACTIVE 阶段资源必须建在 `on_activate`。**这是任何
   `TwoPhaseControllerInterface` 实现都会踩的生命周期陷阱**，已写进用户文档。
2. **测量本身曾经是错的**：`cycle_stamp_seconds_` 只在 `update_phase` 里设置，而**融合的单趟路径不走
   `update_phase`**，于是单趟运行的所有消息时间戳都是 0，按时间戳对齐的观测器把**每一级都算成 lag 0**
   ——单趟看起来"也有两趟的性质"。修好后才有上表的对比。**又一次印证：不经过真实系统验证的结论不算数。**

另外记录一个第三方限制：这个 Humble 的 `GazeboSystem` **只支持 position 命令接口**；声明 velocity 命令
接口不会干净报错，而是让 gzserver 在加载控制器时 `free(): invalid pointer` 崩溃（复现两次）。
因此 demo 用 position 设定点（`position_command: true` 让示例控制器输出设定点而不是误差律）。

**由这些证据驱动的两个改动**（都在本分支）：
1. 成员的活动状态**缓存进执行代**（`TwoPhaseEntry::active`），实时两趟不再调用
   `is_controller_active()`；这是上面"分配更少"的来源（改前是 **26 vs 14**，即两趟**更差**）。
2. demo YAML 里的 `children: []` **删掉**：**空的 YAML 数组 rclcpp 无法定型**，声明 override 时抛
   `parameter_value_from failed for parameter 'children': No parameter value set`——也就是说**原来那份
   demo 配置在真实 spawner 路径上是坏的**，是这次的"真实路径"证据抓出来的。

**踩到并记录下来的坑**（值得写进任何"手泵控制循环"的测试）：上游
`switch_updated_list()` 会**等待实时线程停止使用它要覆盖的那个列表**
（`wait_until_rt_not_using(former_index)`），而手泵测试里只有 `update()` 会推进该索引——
所以列表变更**必须在另一个线程发起、同时持续泵循环**；**事后补泵没用**（等待发生在调用内部）。
`test_two_phase_evidence.cpp` 的 `WithPump()` 就是为此而写。

---

## 5. 验收状态

| 验收项 | 状态 |
|---|---|
| 相对 `469f3055` 的 diff 只落在 `controller_interface`（1 个头）与 `controller_manager` | ✅（见 `git diff --stat 469f3055 HEAD`） |
| 没有新包 | ✅ |
| `colcon build --packages-select controller_interface controller_manager` 在干净 Humble 上通过 | ✅ |
| 两趟核心测试 ≥ 20 例全绿 | ✅ 29 例（`test_two_phase_execution`，ctest 通过），含跨速率桶滞后上界与**链式累积**的实测 |
| 默认关闭时上游行为不变 | ✅（`the_feature_is_off_by_default`、`native_single_pass_never_calls_a_two_phase_stage`；并单独跑上游 `test_controllers_chaining_with_controller_manager`） |
| 示例控制器 + YAML + launch 可跑 | ⚠️ **部分验证**：`ros2 launch controller_manager two_phase_demo.launch.py` 能启动，硬件 `TwoPhaseDemoSystem` 激活成功，日志出现 `Two-phase execution requested by parameter: enabled`；但本机 DDS 服务发现被沙箱阻断（`spawner` / `ros2 control` 都联系不上 `~/load_controller`，与上游已知 flaky 的 `test_spawner_unspawner` 同因），因此 spawner 那一段未跑通。同一控制器已被 22 例测试通过真实 `ControllerManager` 覆盖 |
| 分支说明 `TWO_PHASE_BRANCH.md` | ✅ 本文件 |
| ABI 风险登记 | ✅ §4.7 |

---

## 6. 与 `humble-work` 的同步

- **一个真相，两个分支**：两趟核心的断言（同周期、准入拒绝、分桶、失败包含）在两条分支上保持
  相同语义。改动核心语义时**先改测试**，再同步两边。
- **方向**：`humble-work` 是研究线（内核/执行组/元编程/分析）；本分支是产品化的最小补丁。
  研究线上的两趟修复/新断言应 pick 到本分支；本分支发现的 bug 反向合回研究线。
- **不要把本分支当作 `humble-work` 的父分支**（避免历史倒挂）。
- 接口头在两边的名字空间不同（本分支 `controller_interface::TwoPhaseControllerInterface`，
  研究线 `hierarchical_control::TwoPhaseControllerInterface`）；移植时注意改名。

---

## 7. 风险

1. **语义漂移**：两条线各自演进可能产生分歧 → 用 §6 的"同一套断言"约束。
2. **被误当成完整方案**：本分支没有整组提交/帧语义 → 本文 §4 已写清边界，用户文档同样声明。
3. **ABI**：见 §4.7。
