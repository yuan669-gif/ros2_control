# 测试与复现指南

> 目标：让一个新人能在**一台干净的 Humble 机器**上把测试跑起来、看懂每个套件在钉什么、
> 区分"代码坏了"和"这台机器/这个用例本来就抖"，并能复现文档里的头条数字。
>
> 环境声明（本文所有数字的来源）：Ubuntu 22.04 + ROS 2 Humble，**2 核**，
> 复现时宿主 load ≈3–5，编译与测试都是**本机实测**，不是实验室数据。

---

## 1. 测试分几层

```
编译期证据        负向语料：12 个文件必须编译失败 + 2 个必须通过（诊断子串要匹配）
   │
内核单测          hierarchical_control/：执行组语义、新鲜度、顺序、趟数、分配、成本
   │
类型/元编程单测    hierarchical_control/：拓扑、量纲、契约、typed ports、manifest
   │
管理器集成测      controller_manager/：生命周期、切换、准入、generation、atomic、registry
   │
端到端            ctest 里的 launch/pytest 用例 + case_study/（Gazebo 闭环）
   │
非功能            TSan（发布会话协议 / 真实管理器）、编译成本、运行期分配
```

**设计原则**（读测试时会反复看到）：每个"结论"都要有一个**会失败的断言**；负结果（拒绝、失败、
惰性、撤回）与正结果一样重要，通常还更重要。

---

## 2. 构建与运行（唯一正确的环境）

```bash
cd <workspace>
export ROS_LOG_DIR="$PWD/log/ros"      # 否则 ROS 日志散到 ~/.ros
export ROS_HOME="$PWD/log/ros_home"
source /opt/ros/humble/setup.bash
source install/setup.bash

# 全量（约 13–19 min；改头文件后几乎全包重编）
colcon --log-base log/build_result build --packages-select controller_manager

# 单目标（改一个测试文件时，约 1 min）
colcon --log-base log/build_result build --packages-select controller_manager \
  --cmake-target test_atomic_activation

# hierarchical_control 单目标（快，约 25 s）
colcon --log-base log/build_result build --packages-select hierarchical_control \
  --cmake-target test_static_manifest
```

**运行任何 gtest 前必须导出这个**，否则 `libcontroller_manager.so` 会解析到
`/opt/ros/humble/lib`（系统安装的上游库）而不是你刚编译的：

```bash
export LD_LIBRARY_PATH="$PWD/build/controller_manager:$PWD/build/hierarchical_control:$LD_LIBRARY_PATH"
./build/controller_manager/test_atomic_activation --gtest_filter='*rollback*'
```

`ctest` 的两种用法：

```bash
cd build/controller_manager
# 排除已知的启动/超时敏感用例（见 §5），其余全跑：21/21，约 80 s
ctest --timeout 600 -E "test_controller_manager_srvs|test_spawner_unspawner|test_hardware_spawner|test_ros2_control_node" \
      --output-on-failure
# 单独跑被排除的那些（绕过 ctest 的超时设置）
./test_controller_manager_srvs        # 约 230 s
./test_hardware_spawner               # 约 35 s
./test_spawner_unspawner              # 22 例，见 §6
```

`hierarchical_control` 的负向语料是**独立 ctest**（不是 gtest）：

```bash
cd build/hierarchical_control && ctest -R test_static_topology_negative --output-on-failure
# 或手动：CXX=$(which g++) python3 hierarchical_control/test/test_static_topology_negative.py
```

---

## 3. 套件目录（每个套件钉住什么）

### 3.1 `hierarchical_control`（内核 + 类型层，12 个 gtest 程序 / 81 例 + 1 个负向语料）

| 套件 | 例数 | 钉住的不变量 |
|---|---|---|
| `test_execution_group` | 11 | 与手写基线逐值一致；阶段顺序 + 每阶段恰好一次；**状态失败阻止命令阶段**；命令失败/NaN **绝不提交**；复合节点不能把派生数据重新盖章成更新；样本超龄被拒；库模式无需 lifecycle 刷新；配置/结构错误带原因被拒；**单趟只能保住一个方向**、任何单趟顺序都在某个方向陈旧 |
| `test_stale_state_cost` | 7 | **控制意义**：相位裕度定律 `ΔPM = 360·f_c·L·Δt`（二阶精度）；深度 2/3 越过稳定边界；滞后给出带宽上限；实测失稳上限符合定律；量化不影响上限；摩擦只在很小时无影响；**执行器饱和会掩盖失稳但不修复它** |
| `test_scheduling_performance` | 4 | 两趟保住单趟随深度丢失的控制权限；定律预测带延迟的失稳起点；被报成“增益预算”的其实是时间步进伪影；代价随**绝对延迟**而不是周期数 |
| `test_pass_lower_bound` | 7 | 单趟可行 ⇔ 要求图无环；双向对恰好需要两趟；任意深度双向级联两趟最优；小有向图穷举；规范两趟满足阶段图；双向对没有单趟调度；**阶段内环需要重复而不是更多趟** |
| `test_static_topology` | 5 | 类型层无环、自环拒绝、`ancestry` 正确 |
| `test_dimensional_interfaces` | 6 | 量纲类型与算术、量纲不匹配编译失败 |
| `test_topology_contract` | 6 | 父子边谓词（reference/state 各自独立）、实例唯一、唯一根 |
| `test_topology_binding` | 6 | 从 binding 建组、运行期端口↔`Contract` 校验、诊断内容 |
| `test_typed_ports` | 8 | 生成的字符串表与声明一致；**检查顺序而不只是成员**；拒绝状态端口不匹配；控制器可对照自己的 `Contract` 校验；派生契约与声明一致；父子声明互相检查；mixin 控制器能在真实执行组里跑 |
| `test_typed_tree` | 3 | 一份声明驱动整棵分叉树；兄弟顺序是**声明**而不是位置巧合；带检查的构建会验证**每一个**孩子（不只第一个） |
| `test_static_manifest` | 5 | 编译期 manifest 内容、`static_assert` 可用、`declaration_matches_manifest` 按名字比对、**父链环检查** |
| `test_contract_regression` | 12 | R3/R4/R5/R6 回归：缺状态写不算新鲜、部分写被检出、缺执行器写不提交、晚到的 sink 失败不破坏内部视图、**一个父可以给同一个子两个端口**、一个父两个孩子一致、一个子两个父被拒、一个端口两个写者被拒、同一 claimant 重复 claim 被拒、自属端口按硬件处理、外部 owner 被忽略、**非零基址偏移在 binding 后仍然正确**（早期 `void*` 往返丢地址调整的那个 bug） |
| `test_static_topology_negative`（脚本） | 15 文件 | 12 个**必须编译失败**且诊断含特定子串；2 个必须编译通过 |

### 3.2 `controller_manager`（管理器，18 个 gtest 程序 / 186 例 + pytest/launch）

本项目直接相关的：

| 套件 | 例数 | 钉住的不变量 |
|---|---|---|
| `test_two_phase_execution` | 24 | 单趟滞后=深度 vs 两趟 0 滞后；准入拒绝（跨模式边、降频成员、可调度顺序、同实例两名）；执行状态 generation 一次发布；被拒请求不发布；切换期间成员不被原生循环接管 |
| `test_staged_execution_group` | 7 | 管理器里的端到端阶段执行；整组提交；**部分成员惰性**（5 周期计数不变，补齐后 +3）；配置错误被拒 |
| `test_atomic_activation` | 7 | 默认=上游 best-effort；开关打开后撤销本次激活；接口确实被释放；**硬件模式换回**（计数器 +202 vs 探针 +101）；**chained-mode 重启被恢复**（`'\x2'` vs `'\x3'`） |
| `test_runtime_reconfiguration` | 3 | 周期在飞时安装执行路径被拒且不发布；移除始终允许 |
| `test_static_controller_registry` | 9 | 编译内置类型按字符串加载；两次加载是两实例；与 pluginlib 行为一致；manifest 枚举；封印后拒绝注册/替换 |
| `test_hierarchy_comparison` | 10 | 三种宿主（原生 chaining / 通用 composite / 执行组）同算法对照：输出一致、故障下兄弟一致、分配、typed 声明插件的编译期计划 |
| `test_upstream_ordering` | 4 | 上游排序行为：**reference 边**的先后与注册顺序无关（父先于子）；reference 边在同周期内传播；**Humble 上 state 边无法通过 chaining 绑定**（所以同周期子→父必须靠本项目的调度，而不是上游 chaining） |
| `test_release_interfaces` | 2 | 同一接口的切换与释放；独占接口切换失败时的行为（上游语义，不得回归） |
| `test_controller_manager` / `_with_namespace` / `_urdf_passing` / `test_load_controller` | 18/2/3/39 | 上游既有行为**不得回归**（这是"我们没改坏上游"的底线） |
| `test_controllers_chaining_with_controller_manager` | 6 | 上游 chaining 行为（含内建计时断言，见 §6） |
| `test_controller_manager_srvs` / `test_hardware_management_srvs` | 14 / 4 | 服务层行为 |
| `test_hardware_spawner` / `test_spawner_unspawner` | 8 / 22 | 启动器端到端（见 §6） |
| `test_cycle_tree_contract`（非 gtest） | — | 库模式内核：1000 组同周期比较、两层/三层、单调时钟，失败即非零退出 |

未注册的旧二进制（`test_controller_hierarchy_builder`、`test_hierarchical_controller_executor`、
`test_urdf_hierarchy`、`test_results`）是**历史构建残留**，不在 CMakeLists 里，不要把它们算进结果。

---

## 4. 复现文档里的头条数字

| 数字 | 复现命令 | 口径警告 |
|---|---|---|
| 规范两趟满足全部阶段边 **465/465** | `python3 research/stage_graph/check_stage_graph.py` | `n ≤ 4` 的全部有根树 × 全部边标注 |
| 单趟可行 ⇔ 合并图无环 | 同上（同一脚本同时打印） | 465 例里可行 145 / 不可行 320 |
| 最少趟数下界实验 | `python3 research/pass_lower_bound/search_min_passes.py` | 穷举全部有向图，会打印"少量小图存在严格间隙" |
| 上游排序行为 | `python3 research/topology_analysis/upstream_order_sim.py` | 模拟 `controller_sorting()` |
| 编译成本（形状相关） | `python3 hierarchical_control/test/measure_compile_cost.py --runs 3` | 深度 8/16/32/64 的链：每节点 3.35/4.07/5.89/9.63 ms（**不是**通用常数） |
| 树 vs 链的边际编译成本 | `python3 hierarchical_control/test/measure_binding_cost.py --runs 5` | 链 130–172 ms/节点 vs 树 2–25 ms/节点；**倍数不稳定**，只能声称方向与量级 |
| 被否决的 `O(d)` 拓扑变体 | `python3 research/static_topology_variants/measure_variants.py --runs 3 --depth 64` | 现行 0.17 s / 13 MB vs 被拒 0.55 s / 20 MB |
| 发布协议 TSan | `bash hierarchical_control/test/run_tsan_publish_protocol.sh` | 独立小程序；只证明协议本身 |
| 真实管理器 TSan | `bash controller_manager/test/run_tsan_real_manager.sh [--rebuild]` | 只插桩 `controller_manager`；race verdict 与功能 verdict 分开；二进制缺失时会自动重建 |
| Gazebo 闭环案例 | 见 `doc/GAZEBO_CASE_STUDY.md` §6（`case_study/` 下有配置与原始日志） | 日志里的计数器差值绝对值**没有意义**（见该文 §3.2 的更正）；有意义的是同一配置下两方案的**相对**关系 |

三条口径纪律（都是踩过的坑）：

1. **不同形状的数字不可相减**：`measure_compile_cost.py` 与 `measure_binding_cost.py` 的
   固定成本与形状不同，`COMPILE_COST.md` §2/§3.1 分开列；
2. **取多次的最好值**：宿主 load 高，单次测量抖动大（脚本默认 `--runs`）；
3. **区分"边际"与"绝对"**：绝对时间（11–20 s）里绝大部分是头文件固定解析成本，
   只有**边际量**（每节点/每子节点）有比较意义。

---

## 5. 写一个新测试（模板）

```cpp
// 1) 选层次：内核不变量 → hierarchical_control/test/；管理器行为 → controller_manager/test/
// 2) 断言要能失败：先让它在错误实现下失败（或临时把实现改回旧行为），再提交
TEST_F(YourFixture, the_invariant_you_are_pinning)
{
  SetupTree();                       // 配置期：add → configure（顺序：先子后父）
  SwitchNow({kLeaf}, {});             // 激活：一次一个，符合 Humble 生命周期
  SwitchNow({kRoot}, {});

  const int before = leaf_->state_calls;
  Cycle(5);
  EXPECT_EQ(5, leaf_->state_calls - before) << "失败时写出你期望的语义";

  // 负向断言同样重要：拒绝、惰性、失败不提交
  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({"unknown"}));
}
```

要点：

- **管理器用例必须 pump `update()`**：`add_controller()`/`configure_controller()`/切换都会在
  `wait_until_rt_not_using()` 上等实时循环释放列表；只调一次的测试会**永久阻塞**。
  现成写法见 `test_atomic_activation.cpp` 的 `RunWithPump` 与 `Switch`；
- **计数要在切换之后取**（切换本身会多跑若干周期），或者用"差值为 N"而不是绝对值；
- 新增 ctest 目标要同时改 `CMakeLists.txt`（`ament_add_gmock` + `target_link_libraries`），
  需要 chainable 测试替身时别忘了 `test_chainable_controller`；
- 编译期保证的新用例优先写进**负向语料**（`static_topology_negative/`），因为"编译失败"只能那样测。

---

## 6. 已知 flaky（以及怎么判定"是不是你的锅"）

本机 2 核、load 3–5，以下用例会**非确定性**失败。判定方法：**改动前后各跑 4 次对照**，比较失败签名。

| 用例 | 症状 | 根因 | 判定 |
|---|---|---|---|
| `test_controllers_chaining_with_controller_manager` | `internal_counter = 15` 期望 `14` | 计数由 10 ms 睡线程在 switch 窗口内的 tick 数决定 | 实测：改动前 3/4 通过、改动后 2/4，**同一签名**；空闲时更容易通过，加 2 个 CPU 忙循环后 4/4 通过 |
| `test_spawner_unspawner`（`failed_activation_of_controllers`、`wildcard_entries_*`） | `Could not contact service /test_controller_manager/list_controllers` 后 `loaded_controllers` 为 0/2 | 1.0 s 服务发现超时；用例还用退出码 256 同时表示"激活失败"与"联系不上服务" | 实测：改动前 2/4、改动后 1/4 通过，**同一签名**；单独跑 `wildcard` 用例可通过 |
| `test_controller_manager_srvs` | ctest `TIMEOUT 120` | 该套件需要约 230 s | 直接跑二进制 14/14 通过；是 ctest 属性问题，不是代码问题 |
| `test_hierarchy_comparison.post_switch_two_phase_cycles_do_not_rebuild_membership` | 偶发 `per_cycle[i] > steady + kStrayAllocation` | 宿主抢占让某些周期的分配数抖动 | 现在取**空闲基线的最小值**再比较（并允许少量 stray），不再拿单个基线周期做基准 |

**纪律**：把环境抖动写成"已修复"或把自己的回归归因于环境，都是错的。要下结论**必须**有改动前后的实测对照。

---

## 7. 结果记录（截至最新提交）

| 项目 | 结果 |
|---|---|
| `hierarchical_control` ctest | 13/13 程序通过（12 个 gtest = 81 例 + 负向语料 15 文件） |
| `controller_manager` ctest（排除启动/超时敏感项） | 21/21 通过（约 80 s） |
| `test_controller_manager_srvs`（单独） | 14/14（约 230 s） |
| `test_hardware_spawner`（单独） | 8/8（约 35 s） |
| `test_spawner_unspawner`（单独） | 20/22（2 个服务发现抖动，见 §6） |
| 真实管理器 TSan | 0 条 data race（manager 自身）；依赖库未插桩 |
| 发布协议 TSan | PASS |
| 执行组零分配（`test_hierarchy_comparison.reported_overhead_and_allocations`） | 配置后直接调 `run()` 100 次的分配数为 0（并带一个“空转窗口”作对照排除后台噪声）；`ControllerManager::update()` 整体仍分配（上游来源） |

更细的逐轮记录见 `REVIEW_COMPILETIME_LATEST_RESPONSE_2026-09-26.md` §7 与
`REVIEW_REQUIREMENTS_RESPONSE_2026-09-24.md`。
