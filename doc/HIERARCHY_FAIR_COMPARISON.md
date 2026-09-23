# 同算法公平对照：原生 chaining / 手写 composite / 阶段化执行组

日期：2026-09-20。范围：仅 ROS 2 Humble，本仓库。
测试：`controller_manager/test/test_hierarchy_comparison.cpp`。
前置：`doc/STAGED_EXECUTION_GROUP_EXPERIMENT.md`（阶段 A/B 的实现与边界）。

本文记录的是一次**初步**对照，不是最终性能结论。所有数字来自同一台非实时虚拟机
（Ubuntu 22.04、GCC 11.4.0、RelWithDebInfo），不构成 WCET 或硬实时保证。

## 一、公平性规则

三种实现使用**完全相同的控制律、相同的状态快照、相同频率、相同硬件接口、相同故障语义**。

合成算法（Leaf 数 `P`）：

```text
linear : S_leaf = P, S_module = 2*S_leaf, S_root = 2*S_module
         C_root = R - S_root ; C_module = C_root - S_module ; C_leaf = C_module - S_leaf
         => C_leaf = R - 7*P
fork   : S_root = 2*(S_a + S_b), C_root = R - S_root, C_x = C_root - S_x
```

实现：

| 方案 | 结构 | 状态路径 | 命令路径 | 提交 |
|---|---|---|---|---|
| 原生 chaining | 3 个 `ChainableControllerInterface` 控制器，父 claim 子 reference | 各节点自己读同一次硬件快照并本地推导 | 单相，父->子单遍 | 各控制器直接写自己的接口，无整组提交 |
| 手写 composite | 1 个普通 `ControllerInterface` 插件 | 内部两阶段（标量） | 内部两阶段 | 先写私有 scratch，校验有限性后再复制到命令接口 |
| 阶段化执行组 | 3 个 `StagedControllerInterface` 控制器 + `StagedExecutionGroup` | 后序，子->父 | 前序，父->子 | 整树成功后统一提交到各叶的 sink |

说明：原生 chaining 基线并**不是**弱基线。它可以让每个节点都直接读同周期硬件快照并本地推导，
因此对这条算法它可以产出完全相同的同周期结果。这正是交接文档要求的“父控制器直接读硬件状态”
对照。

## 二、结果一：输出一致（20 个周期）

`same_algorithm_outputs_match_across_implementations` 通过：

- 三种实现每个周期的叶命令**逐位相等**；
- 且都等于解析值 `reference(cycle) - 7 * (hardware + offset)`；
- 三种实现 0 次 update 错误。

这验证执行组的双向传播没有引入数值偏差，也说明“能否算出正确同周期值”不是本方案的区分点。

## 三、结果二：故障下的兄弟一致性（两叶 fork）

场景：`root -> {leaf_a, leaf_b}`，`leaf_a` 写 `joint2/velocity`，`leaf_b` 写 `joint3/velocity`；
在第 5 个周期注入 `leaf_a` 命令阶段失败一次，随后恢复。

| 观测 | 原生 chaining | 阶段化执行组 |
|---|---|---|
| `update()` 返回值 | 第 5 周期 ERROR | 第 5 周期 ERROR |
| `joint2/velocity`（失败叶） | 保持上一周期值 | 保持上一周期值 |
| `joint3/velocity`（健康兄弟） | **写入新值** | **保持上一周期值** |
| 两个兄弟是否一致 | 否（一新一旧） | 是（都保持旧值） |
| 下一周期恢复 | 正确 | 正确 |

这正是 POV 底盘要避免的“部分新、部分旧”情形：原生 chaining 在某个叶失败后，
`ControllerManager::update()` 只记录错误并继续执行后续控制器，因此健康兄弟仍会更新。

手写 composite 是单节点实现，其故障行为与执行组一致（校验失败则不写命令接口），
但它需要为新拓扑重新手写内部调度。

## 四、结果三：动态分配

计数方式：在测试可执行文件中替换全局 `operator new/delete`，只在测量窗口内计数。
control 窗口（100 次无执行组调用的空转）为 0，说明计数没有后台噪声。

| 测量 | chained | composite | staged |
|---|---|---|---|
| 每次 `ControllerManager::update()` 的分配次数（200 周期平均） | 10 | 3 | 10 |
| `StagedExecutionGroup::run()` **自身**每 100 次调用分配次数 | — | — | **0** |
| 控制窗口 | — | — | 0 |

结论：

- **执行组自身的运行路径在配置完成后不分配**（100 次调用 0 次分配，已断言）；
- 但 `ControllerManager::update()` 整体仍会分配，且三种方案都逃不掉，来源是 Humble 既有代码：
  1. legacy 循环里的 `for (auto loaded_controller : rt_controller_list)` **按值复制**
     `ControllerSpec`（含字符串和 vector）；
  2. `is_controller_active()` / `get_state()` 最终调用
     `rclcpp_lifecycle::LifecycleNode::get_current_state()`，它在
     `rclcpp_lifecycle::MutexMap::add()` 中分配。
- 因此“接入执行组能让 `update()` 变成零分配”是不成立的；正确表述是
  “执行组没有新增实时路径分配”。

### 顺带发现（对后续接入有用）

`get_current_state()` 会分配，所以执行组原本每个周期对每个成员调用 `get_state()` 是不合适的。
现在改为：

- `run()` 不查询生命周期；
- `ControllerManager` 只在**切换真正发生后**（`manage_switch()` 之后）以及安装执行组时调用
  `refresh_member_active_state()` 刷新缓存的 `members_active_`。

代价是：如果控制器不经过 switch 就自行改变状态，缓存会过期。原型阶段接受该限制并已写进文档。

## 五、结果四：`update()` 耗时（非实时虚机，仅供参考）

同一测试进程内、200 个周期、RelWithDebInfo、连续运行 3 次，取每次运行的 min/median/max（微秒）：

| 运行 | 方案 | min | median | max |
|---|---|---|---|---|
| 1 | 原生 chaining | 1.42 | 1.87 | 39.5 |
| 1 | 手写 composite | 0.40 | 0.65 | 8.5 |
| 1 | 阶段化执行组 | 3.21 | 4.28 | 114.2 |
| 2 | 原生 chaining | 2.49 | 4.28 | 299.6 |
| 2 | 手写 composite | 1.06 | 1.51 | 12.1 |
| 2 | 阶段化执行组 | 3.37 | 4.29 | 20.6 |
| 3 | 原生 chaining | 2.47 | 2.50 | 12.5 |
| 3 | 手写 composite | 1.00 | 2.04 | 36.9 |
| 3 | 阶段化执行组 | 3.03 | 4.29 | 40.7 |

观察（只能作为趋势，不能作为结论）：

- 手写 composite 在这条算法上最便宜；
- 阶段化执行组的中位耗时约为 composite 的 2 倍，与原生 chaining 处于同一量级或略高；
- max 值波动极大（8–300 µs），是非实时虚机调度噪声，不能当作 deadline miss 统计。

原因分析（待后续消融验证，当前只是假设）：执行组每周期有 3 次状态回调 + 3 次命令回调，
每次都是虚函数调用，并带 frame 校验和有限性检查；composite 只有 1 次 `update()`。

## 六、这对论文问题意味着什么

本轮对照支持以下**有限**结论：

1. 阶段化执行组能在真实 `ControllerManager`/`ResourceManager` 上产出与原生 chaining、
   手写 composite 完全相同的同周期结果；
2. 它在故障叶存在时**避免兄弟控制器出现部分新/部分旧命令**，而原生 chaining 会出现；
3. 它自身的实时路径在配置后不分配；
4. **它不是更快的方案**；在这条微基准上比手写 composite 慢约 2 倍。

因此阶段化执行组的候选优势只剩交接文档“Gate B”所列的：
可复用绑定校验、诊断、生命周期处理、减少手写集成代码。这些目前**尚未测量**。

按照论文停止门槛：

- 如果“每新增一个模块要手写的两阶段调度和粘合代码量”与 composite 方案相比没有明显下降，
  就应当停止扩大 `ControllerManager` 修改，转为交付可复用的复合控制器库；
- 如果确实下降（例如新增一个模块只需声明端口和连接，而 composite 需要改内部算法和缓冲），
  才值得继续。

## 七、尚未完成 / 下一步

1. **方案 2（估计/命令显式拆分）** 尚未实现；它需要两个控制器通过 Humble reference 接口
   串联，并额外声明派生状态边；
2. **接线成本量化**：统计每种方案为新增一个模块所需的控制器代码行数、配置项数量、
   改动文件数（这是 Gate B 的核心证据）；
3. 耗时分布需要在**更稳定的环境**（例如关闭后台负载、绑定 CPU、实时内核）重复，
   并报告 p50/p95/p99 与 deadline miss，而不是单机 max；
4. 消融：分别关闭“帧有效性校验”和“整组提交”，测量它们各自的耗时占比；
5. 修正项：`run()` 缓存 `members_active_` 之后，需要在文档中明确“非 switch 的状态变化会导致
   缓存过期”，并考虑在下一阶段用更可靠的通知机制替代。

## 八、复现命令

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select controller_manager --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"     # 受限环境下需要可写日志目录
./build/controller_manager/test_hierarchy_comparison
# 或
cd build/controller_manager && ctest -R test_hierarchy_comparison --output-on-failure
```
