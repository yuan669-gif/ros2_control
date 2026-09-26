# 对《最新编译期控制器方案审核》（`REVIEW_COMPILETIME_LATEST_2026-09-26.md`）的回应

日期：2026-09-26
基准版本：`b079f07`（审核针对 `86ae9f64`；`b079f07` 是其后继）
范围：P1-1、P1-2、P2-1、P2-2、P2-3、P2-4 六项，按审核"推荐下一步"的顺序 1–6 逐项处理。

审核的总体结论（"编译期生成控制图和接口描述，运行时完成 ROS 生命周期与硬件资源绑定"）本轮不再改动；
下面只记录**改动内容**、**实测证据**和**仍然不能声称的东西**。

---

## 1. P1-1：atomic activation 回滚补上硬件 command mode

### 1.1 问题确认

审核的判断是对的，而且比"可能结果"更强：`rollback_activated_controllers()` 只做
`deactivate()` + `release_interfaces()`。`switch_controller()` 在此之前已经对整个 activate 集合执行过
`prepare/perform_command_mode_switch()`，失败路径只把**激活失败**的控制器接口换回去
（`failed_controllers_command_interfaces`），**本次成功激活、随后被回滚**的控制器接口没有任何人换回来。
对一个把独占模式写进硬件的总线来说，"controller 变 inactive + loan 已释放" 并不等于"硬件回到切换前"。

### 1.2 改动

| 位置 | 改动 |
|---|---|
| `ActivationOutcome` | 新增 `activated_command_interfaces`（本次成功激活者实际 claim 的 command interface **名字**集合，去重）、`activated_chainable`（本次为其发布了 reference interface 的 chainable 控制器）、`rollback_performed` / `rollback_failed` |
| `activate_controllers()` | 每次激活成功时记录该控制器 claim 的 command interface 名字，并记录 chainable 发布 |
| `rollback_activated_controllers(ActivationOutcome &)` | ① 按**激活逆序** deactivate + release（链上子节点先放，父节点不会被"已经不可用"的引用拖住）；② 对本次发布的 reference interface 调 `make_controller_reference_interfaces_unavailable()`；③ 对本次激活的接口调 `prepare_command_mode_switch({}, ifaces)` + `perform_command_mode_switch({}, ifaces)`；④ 每一步失败都置 `rollback_failed` 并单独报错 |
| `manage_switch()` | 回滚失败时输出**独立的** rollback failure 日志（与原始 activation failure 分开），并给出恢复动作 |

接口名而不是下标：控制器列表可以在一次 switch 之间被重新映射，下标描述不了它所属的那次 switch
（这也是 `PORT_DIMENSIONS.md`/manifest 层一贯的取舍）。

### 1.3 验收证据（实测，不是推理）

mock 硬件 `TestActuatorHardware`（`joint1`）在 `prepare_command_mode_switch()` 里把 `position_state_ += 1`、
在 `perform_command_mode_switch()` 里 `+= 100`，且**无视请求列表**。`joint1/position` 正好导出这个变量，
所以一个只读该 state interface 的观察控制器（`MockModeObserver`）读到的是一个精确的"模式切换计数器"：

```
delta = (#prepare 调用) + 100 * (#perform 调用)
```

用例 `TestAtomicActivation.the_rollback_switches_the_hardware_mode_back`：
blocker 先占住 `joint1/position`，然后 `Switch({"first","conflict"})`（`conflict` 必然失败）。
基线在 blocker 激活之后取。

| 版本 | 观察值 | 含义 |
|---|---|---|
| 临时关闭 ③（探针，只重建库） | 基线 **+101** | 只有失败那次 switch 自己的一对 |
| 当前实现 | 基线 **+202** | 失败 switch 一对 + 回滚一对 |

探针是真实跑出来的（改 `controller_manager.cpp` → `--cmake-target controller_manager` → 同一个测试二进制），
所以这个用例**不是空转**：它精确地测出了缺失的那次硬件回滚。

### 1.4 仍然不能声称

- **物理总线原子性**：这只保证"我们向硬件发出了回退请求并且硬件答应了"。真正的原子提交是总线/驱动器
  的事，`prepare/perform` 契约本身在 Humble 里就是两段式，中间没有事务语义。
- **chained-mode 重启无法回退**：一次 switch 可能因为 "切到 chained mode" 把一个**本来已 ACTIVE** 的
  following controller 先 deactivate 再 activate（上游 `switch_controller()` 的 restart 逻辑）。它成功后会
  出现在 `outcome.activated` 里，回滚会把它停在 INACTIVE 而不是原来的 ACTIVE。要正确处理需要区分
  "本次新激活" 与 "为切 chained mode 而重启"，属于上游结构性行为，本轮**没有**动，记为已知限制（见 §6）。

---

## 2. P1-2：静态树的部分激活变成显式策略

审核给了三个选项。"安装时要求 atomic activation" 单独用**不足**：即使打开了 atomic activation，
逐条激活（Humble 的生命周期本来就可以一个一个来）仍然会留下部分激活的组。所以本轮同时做了
**前提**和**行为**两半。

### 2.1 安装前提（可强制）

- `set_staged_execution_group()` 在 `atomic_activation_ == false` 时**拒绝安装**并返回 `ERROR`；
- `set_atomic_activation(false)` 在已安装 staged group 时**拒绝**（返回 `false`），策略不能被从背后关掉；
- 这两条只作用于**我们自己的 API**，上游 best-effort 默认语义与上游自己的测试完全不受影响。

### 2.2 部分成员的行为（可预测 + 不沉默）

部分成员是合法状态（逐个激活的中间态），此时执行组**完全惰性**：

- `run()` 返回 `StagedStatus::inactive`（内核 `members_active_` 缓存，任一成员非 ACTIVE 即整组不跑）；
- 不调用任何成员的 `update()`，不提交任何 command；
- 仍然 ACTIVE 的成员保持自己的 interface claim（不会"半个级联"写下去）；
- **新增报告**：`switch_controller()`（非实时线程）在应用 switch 后刷新并检查成员状态，不完整时打印
  `WARN` 并点名**未激活的成员**。这条放在 switch 线程而不是 `update()`，所以不破坏"控制循环零分配"。

### 2.3 验收证据

- `installing_a_staged_group_requires_atomic_activation`：关闭 atomic → 安装 `ERROR` 且 `staged_execution_group()==nullptr`；
  打开 → 安装成功；安装期间 `set_atomic_activation(false)` 被拒；`clear_staged_execution_group()` 后可以再关闭。
- `a_partial_membership_is_inert`：`SetupGroup(0.0, /*activate_root=*/false)`（leaf+module 已激活，root 未激活）→
  `members_active()==false`；`Cycle(5)` 后 leaf/module 的 `state_calls`、`commit_calls()`、`legacy_update_calls`
  **全部不变**（既没被组执行，也没被 native 循环接管）；补上 root 后组恢复执行，3 个周期恰好 +3 次。
  实测日志里能看到那条新警告两次（激活中间态各一次），点名 `stage_root, stage_module` / `stage_root`。
- 全部既有 staged 用例（`test_staged_execution_group` 7/7、`test_two_phase_execution` 24/24、
  `test_hierarchy_comparison` 10/10）通过；需要在 setup 里显式打开 atomic activation，测试里都写了原因注释。

### 2.4 一个被实测纠正的假设

最初的用例想用"停用某个成员"来构造部分状态，实测被上游拒绝：
`Could not deactivate controller 'stage_leaf' because preceding controller 'stage_module' is active`。
这正是 `IMPLEMENTATION_GUIDE.md` §12.4 #10 记录过的限制，所以部分状态只能由**逐个激活**构造，
用例已按可达路径重写。

---

## 3. P2-1：`manifest_problem()` 的父链检查

`manifest_problem()` 现在对每个节点做**有界父链上溯**（≤ N 步），走不到根即判
`"the parent chains must not contain a cycle"`。

审核举的例子需要分开说，两种形状只有一个能被"根数量"拦住：

| 形状 | 是否已被根数量拦住 | 现在由谁拒绝 |
|---|---|---|
| 纯环 `a→b→c→a` | 是（0 个根 ≠ 1） | `"a manifest must have exactly one root"` |
| 链进入环：根 `r`，`a→b, b→c, c→b` | **否**（恰好 1 个根、父节点都存在、无自父） | **新增**父链上溯 |

第二种才是真正的漏洞：`a` 永远到不了根，拓扑不是树，而所有旧检查都通过。用例
`the_constexpr_checker_rejects_parent_chains_that_enter_a_cycle` 同时钉住三种形状
（纯环、链入环、正常链），并在 `static_assert` 里使用，证明它确实是 constexpr 可用的。

---

## 4. P2-2：registry 注册时机从注释变成约束

选审核的第一个方案（"只允许未加载控制器时安装/注册"）并加了封印：

- `StaticControllerRegistry` 新增 `freeze()` / `frozen()`；`insert()` 在封印后抛 `std::logic_error`；
- `ControllerManager::load_controller()` 在**第一次查表之前**封印已安装的 registry（这样并发的 `add()`
  会抛异常，而不是在查表过程中改 `std::map`）；
- `add_controller_impl()` 也封印（直接添加控制器不经过 `load_controller()`）；
- `set_static_controller_registry()` 改为返回 `bool`：已封印（= 已尝试加载控制器）时**拒绝替换**，
  已安装的 registry 不变；
- `register_static_controller_type<T>()` 改为返回 `bool`：封印后返回 false 并记日志。重复/空类型名仍然
  抛 `std::invalid_argument`（那是启动期编程错误，不是时机问题）。

封印是**单向**的：卸载控制器不会重新打开类型集合——"已封印"正是查找无锁安全的那个状态。

用例 `the_type_set_is_sealed_once_a_controller_is_loaded`：加载前 `frozen()==false`；第一次 load 后
`frozen()==true`；替换 registry 返回 false 且 `static_controller_registry()` 未变；`add()`/`add_factory()`
抛 `std::logic_error`；`register_static_controller_type()` 返回 false；封印前注册的类型仍然可以重复加载。
`a_type_registered_through_the_manager_is_loadable` 覆盖封印前的正常路径。

**仍然不能声称**：没有为"注册 vs 查表"加锁。约束是"封印后不再写"，所以读路径无锁是安全的；
但一个在封印**之前**就从另一个线程调用 `add()` 的程序仍然是数据竞争——这属于"启动期单线程配置"的
前提，文档已写清，本轮不提供运行期并发注册。

---

## 5. P2-3：参数化 factory 的 manifest

- 新增 `StaticControllerRegistry::ManifestDescriptor{command_interfaces, state_interfaces}`；
- `add_factory(type, factory, ManifestDescriptor)`：显式描述，`has_manifest(type)==true`；
- `add_factory<ControllerT>(type, factory)`：从 `ControllerT::manifest` 取描述（`ControllerT` 无 manifest 时
  `static_assert` 失败并提示改用 descriptor 版本）；
- 原来的 `add_factory(type, factory)`（无描述）保留，`has_manifest` 仍为 false——factory 本身不携带描述，
  不替调用者编造。

用例 `a_parameterised_factory_can_carry_its_manifest` 覆盖三种情况，并确认带描述的 factory 仍然是普通
factory（可加载、可 configure）。

---

## 6. P2-4：TSan 脚本不再把"没有 race 日志"当作成功

`run_tsan_real_manager.sh` 现在区分两个 verdict：

- **功能 verdict**：要求进程有 `[==========] N tests from M test suites ran.` 行、两个预期 suite
  （`TestTwoPhaseExecution`、`TestExecutionPathAdmission`）都出现、`[ RUN ]` 数量 ≥ 20。不满足即
  `DID-NOT-RUN` → 退出码 3（"二进制没跑起来/构建复用了旧产物"这类情况再也不会 PASS）；
- **race verdict**：`data race` 计数为 0；
- 功能用例在 TSan 下的失败（脚本头部解释过：多个用例是计时敏感的）单独列出，不并入 race verdict；
- 退出码：`0` = race 干净且套件确实跑完；`1` = 有 race；`3` = 没跑完。

本轮**没有**重新跑完整 TSan（需要重建 `build_tsan`，且为腾磁盘已删除；脚本在二进制缺失时会自动重建）。
上一次运行的结论（manager 自身 0 race、23/23 用例在 TSan 下通过、依赖库未插桩）仍然有效，但**脚本逻辑
的改动本身**只做了 `bash -n` 语法检查。

---

## 7. 本轮测试统计（本机，Humble，2 核，load ≈3–5）

| 套件 | 结果 |
|---|---|
| `hierarchical_control` / `test_static_manifest` | 5/5（含新增 P2-1 用例） |
| `test_atomic_activation` | 6/6（含新增 P1-1 用例 + 探针实测） |
| `test_staged_execution_group` | 7/7（含新增 P1-2 惰性用例） |
| `test_two_phase_execution` | 24/24（含新增 P1-2 安装前提用例） |
| `test_static_controller_registry` | 9/9（含新增 P2-2/P2-3 用例） |
| `test_runtime_reconfiguration` | 3/3 |
| `test_hierarchy_comparison` | 10/10 |
| `ctest -E "srvs|spawner|hardware_spawner|ros2_control_node"` | **21/21 通过**（80.9 s） |
| `test_controller_manager_srvs`（单独跑，绕过 ctest 120 s 上限） | 14/14 通过（227.6 s；ctest 的 `TIMEOUT 120` 是既有限制，不是代码失败） |
| `test_hardware_spawner` | 8/8 通过（37.1 s） |
| `test_spawner_unspawner` | 20/22；2 个失败，见下 |

### 关于 `test_spawner_unspawner` 的两个失败（如实记录）

失败用例：`spawner_test_with_wildcard_entries_with_no_ctrl_name`、
`spawner_test_failed_activation_of_controllers`。**两者都不是激活语义失败**：

- 失败签名是 `[FATAL] [spawner_...]: Could not contact service /test_controller_manager/list_controllers`
  ——spawner 进程在 `--controller-manager-timeout 1.0` 内没发现服务，**请求根本没到 manager**；
- `spawner_test_with_wildcard_entries_with_no_ctrl_name` **单独跑通过**（3.4 s），在整套 22 个用例里
  跑则失败 ⇒ 顺序/资源相关的服务发现抖动；
- `spawner_test_failed_activation_of_controllers` 单独重复 4 次：3 次失败、1 次通过，且**通过那次同样**
  打印了 `Could not contact service`。原因在用例本身：它用退出码 256 同时表示"激活失败"与"联系不上服务"，
  而紧随其后的 `get_loaded_controllers().size() == 3` 只在"先加载成功、再激活失败"时才成立；
- 本轮改动不涉及节点创建、executor、发现或 spawner 路径；且该用例关键断言的语义
  （失败者跳过、成功者保持 ACTIVE）在 `test_atomic_activation.by_default_a_partial_activation_is_kept`
  里有**确定性**等价覆盖，且 6/6 通过。

**改动前后对照（已实测）**：把 4 个源文件 stash 回改动前、只重建库与该用例（3 min 50 s），同一宿主机上
重复 4 次：

| 版本 | 通过 | 失败 | 失败签名 |
|---|---|---|---|
| 改动前（`b079f07` 的库） | 2/4 | 2/4 | `Could not contact service` + `test_spawner_unspawner.cpp:426/435`（首个 spawner 就没连上，0 个控制器） |
| 改动后 | 1/4 | 3/4 | `Could not contact service` + `test_spawner_unspawner.cpp:473` / `:426` |

同一失败模式、同一量级的抖动，**改动前也复现** ⇒ 既有环境/用例脆弱性，与本轮改动无关。
（对照实验后已 `git stash pop` 并重建，`test_atomic_activation` 复测 6/6。）

### TSan

本轮**没有**重跑完整 TSan（`build_tsan` 已为腾磁盘删除；脚本会在二进制缺失时自动重建）。
脚本的判定逻辑改动本身只做了 `bash -n` 语法检查。上一次运行结论（manager 自身 0 条 data race、
23/23 用例在 TSan 下通过、依赖库未插桩）在本轮改动之前成立，本轮改动没有触碰发布协议
（generation 仍是**一次** `atomic_store` / 每周期**一次** `atomic_load`）。

---

## 8. 仍然不能声称（累计）

- 不能在 C++ 静态初始化阶段完成 ROS controller 的**完整初始化**；
- atomic activation 回滚恢复了本次 switch 的 lifecycle、command interface claim、reference interface
  发布和硬件 command mode，但**没有**恢复为切 chained mode 而重启的既有控制器（§1.4），也没有物理总线
  原子性；
- 依赖库内部（rclcpp / lifecycle / hardware_interface / FastRTPS）的 TSan 结论仍然未知（未插桩）；
- registry 的"运行期变更线程安全"仍然不成立：提供的是**封印后只读**，不是并发写；
- 硬件总线的物理原子提交。
