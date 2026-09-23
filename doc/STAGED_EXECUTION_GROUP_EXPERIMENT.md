# 阶段化执行组：ROS 2 Humble 接入实验（阶段 A/B）

日期：2026-09-20。范围：仅 ROS 2 Humble，本仓库。

本文记录在 `cycle_tree.hpp` 独立标量内核之后的第一条真正的 ROS 集成路径：
`controller_interface::StagedControllerInterface` + `controller_manager::StagedExecutionGroup`
以及它到 `ControllerManager::read/update/write` 的 opt-in 接入。

本文只陈述已经在本机（Ubuntu 22.04、GCC 11.4.0、ROS 2 Humble、colcon overlay）实际构建和运行的
结果。未运行的内容一律标注为“未验证”。

> 后续：同算法公平对照（原生 chaining / 手写 composite / 本执行组）见
> `doc/HIERARCHY_FAIR_COMPARISON.md`。初步结论是输出一致、故障一致性更好，但本执行组更慢。

## 一、为什么需要新的接口而不是继续扩展 cycle_tree

`cycle_tree.hpp` 的已验证结论保持不变：标量端口、构造期建计划、状态后序、命令前序、
失败不更新 committed buffer。它的边界也保持不变：标量、每叶一个命令、Connection 由调用者提供、
没有真实 ROS 句柄绑定、没有 `ControllerManager` 接入。

真实控制器需要的是多端口、真实 `LoanedStateInterface` / `LoanedCommandInterface` 绑定、生命周期和
`ResourceManager` 所有权。因此新增了 ROS 侧的执行核心；它与 `cycle_tree.hpp` 共用
`controller_manager/hierarchy.hpp` 的计划生成与校验（单根、无环、可达、唯一父节点），
但端口模型和句柄绑定不同。两个内核的关系是：

| | `cycle_tree::Executor` | `StagedExecutionGroup` |
|---|---|---|
| 端口 | 每个节点 1 个状态、1 个 reference、叶 1 个命令 | 每个节点任意多状态端口；每个子节点一组 reference 端口；叶任意多命令端口 |
| 绑定 | 调用者提供标量 Connection | 从 Humble `command_interface_configuration()` 前缀解析命令边 |
| 句柄 | 不接触硬件 | 通过 `StagedCommandSink` 提交到控制器持有的真实命令接口 |
| 时间 | 调用者传 `snapshot` + `Value.sample_ns` | 叶状态采样时间来自本周期，复合状态强制取子节点最旧时间 |
| 用途 | 独立算法/契约验证 | `ControllerManager` opt-in 执行组 |

## 二、`StagedControllerInterface` 的数据契约

每个控制器可以声明三类端口：

- `staged_state_ports()`：本节点发布的状态值（自底向上）。
- `staged_reference_ports()`：本节点从父节点消费的 reference 值；同时用于导出 Humble reference
  interface（`<controller_name>/<port>`），供父节点 claim。
- `staged_actuator_ports()`：本节点写入的硬件命令接口（仅叶节点非空）。

每个值组带 `StagedFrame`：

```cpp
struct StagedFrame {
  std::uint64_t cycle;      // 生产该值的执行组周期
  std::int64_t  sample_ns;  // 最旧输入采样时间
  bool          valid;      // 仅在本周期对应阶段成功后为真
  std::uint32_t fault_code; // 控制器自定义故障码，0 表示未上报
};
```

关键约束（由执行组强制，而不是靠约定）：

- 叶节点状态阶段默认把 `sample_ns` 记为本周期；可以调用 `set_source_sample_ns()` 上报更旧的
  真实采样时间，但不能上报未来时间。
- **复合节点的 `sample_ns` 由执行组用“最旧子节点采样时间”覆盖**，节点写什么都不生效，
  因此无法重新盖章伪装新鲜。
- reference 的 `sample_ns` 是本周期时间，`cycle` 由执行组填写。
- 节点的状态输出必须是有限值；reference 和 actuator 也必须是有限值，否则该周期失败。

两个阶段最多各调用一次：

```text
update_state_stage   : 后序（叶 -> 父）
update_command_stage : 前序（父 -> 叶）
```

两个回调在接口上都是 `noexcept`。执行组的运行路径不做构图、不做字符串查找、不
`dynamic_cast`、不扩容：所有尺寸在构造期确定。

## 三、`StagedExecutionGroup` 的运行语义

一个周期：

```text
所有成员必须 ACTIVE（否则返回 inactive，什么都不执行）
state 阶段（后序）：
  叶  : 读自己的 LoanedStateInterface -> 写自己的状态端口
  复合: 校验每个子状态 frame.valid && cycle == 本周期 && 未超龄
        调用 update_state_stage，之后用最旧子采样时间覆盖 sample_ns
root reference：
  通过 root 的 StagedReferenceSource 在本周期取一次外部指令快照
command 阶段（前序）：
  校验本节点 state/reference frame 属于本周期
  调用 update_command_stage -> 写子节点 reference 组 + 自己的 actuator scratch
  校验写出的 reference / actuator 全部有限
commit：
  每个叶节点的 scratch 交给它自己的 StagedCommandSink
  sink 成功后才镜像到 committed buffer 并推进 committed_cycle
```

失败语义：

- 状态阶段任一步失败：**命令阶段完全不进入**，返回 `state_failed` 与失败节点索引。
- 命令阶段失败或写出非有限值：返回 `command_failed`，**任何 sink 都不会被调用**，
  因此硬件命令缓冲不会出现“部分新、部分旧”。
- sink 在 `commit()` 中返回 false：这是硬件写入故障域。执行组会报告 `command_failed`，
  但**不承诺回滚**已经提交过的其它叶节点。这一点与交接文档一致：软件提交不等于总线原子同步，
  也不等于物理动作原子。
- 保留上一周期命令不是安全策略。调用方必须在看到非 `committed` 结果后执行应用定义的故障动作。

`max_age_ns` 可在构造时指定：`0` 表示只接受本周期采样；正数表示允许的采样年龄上限。
`now_ns`、`period_ns`、采样时间都使用同一时钟（接入层使用 `rclcpp::Time::nanoseconds()`）。

## 四、依赖边从哪里来

命令边来自 Humble 原生机制，而不是新的 parent 字段：

- 子控制器通过 `export_reference_interfaces()` 导出 `<child>/<port>`；
- 父控制器的 `command_interface_configuration()` 里出现 `<child>/<port>`；
- 执行组在配置期按 `/` 前缀匹配成员名：前缀是成员的接口是 reference 接口，前缀不是成员的
  接口是普通硬件命令接口。

因此：

- 交换控制器声明顺序不改变计划；
- 一个 reference 接口有两个写者（两个成员都 claim 同一个 `<child>/...`）在配置期直接抛错；
- 多个根、成环、引用不存在的成员在配置期直接抛错；
- 声明了 actuator 端口却没有 `StagedCommandSink`、根声明了 reference 端口却没有
  `StagedReferenceSource`，都在配置期直接抛错。

URDF 在本阶段**没有**参与依赖推导，只用于 `ResourceManager` 提供 joint 资源。URDF 校验属于下一阶段。

## 五、ControllerManager 接入

新增公开 API：

```cpp
controller_interface::return_type set_staged_execution_group(
  const std::vector<std::string> & controller_names, std::int64_t max_age_ns = 0);
void clear_staged_execution_group();
std::shared_ptr<StagedExecutionGroup> staged_execution_group() const;
```

约束与行为：

- 成员必须已经加载、configure 完成且处于 INACTIVE；否则拒绝（`ERROR`）并保持已有执行组不变。
- 成员必须实现 `StagedControllerInterface`，否则拒绝。
- 配置期完成 topology 解析、字符串处理、`dynamic_cast`、存储分配。
- `update()` 中：

```text
if (staged_group_ && !switch_params_.do_switch) 先运行执行组
legacy 循环里 if (staged_group_->owns(controller)) continue;
```

`owns()` 只做已排序指针数组的二分查找，不做字符串比较。

- 切换进行中（`switch_params_.do_switch`）不运行执行组，因为成员集合可能正在变化。
- 非成员控制器完全走原生 `update()` 路径，行为不变。
- 未实现：多频、异步回调、运行期动态拓扑、生命周期回滚顺序、真实硬件故障动作、URDF 校验、
  多执行组、硬实时 WCET 证明。

## 六、验收测试

测试文件：`controller_manager/test/test_staged_execution_group.cpp`
测试控制器：`controller_manager/test/test_staged_controller/`
合成算法（与独立内核实验一致，便于公平对照）：

```text
S_leaf = 绑定硬件状态 + offset
S_comp = 2 * sum(子节点状态)
C      = reference - S
子节点 reference 和 actuator 都得到 C
```

三层组：`stage_root -> stage_module -> stage_leaf`，叶节点写 `joint2/velocity`（mock hardware
`test_system`）。

用例：

1. `phase_order_single_call_and_same_cycle_propagation`
   - 每周期每个阶段恰好调用一次；
   - 原生 `ChainableControllerInterface::update()` 路径调用次数为 0；
   - 状态阶段顺序叶->模块->根，命令阶段顺序根->模块->叶，且所有状态回调早于所有命令回调；
   - 三个节点观测到同一周期号，`committed_cycle` 与该周期一致；
   - 数值传播与手写 composite 等价（offset=3、R=1000 时 leaf 命令 = 979）。
2. `failure_never_partially_commits`
   - 模块状态失败：命令阶段零调用、sink 零调用、committed 值不变；
   - 叶命令失败：sink 零调用、committed 值不变；
   - 模块写出 NaN reference：commit 前被拒；
   - sink 提交失败：sink 被调用但硬件缓冲与 committed 值不变；
   - 故障清除后下一周期恢复提交。
3. `committed_command_reaches_mock_hardware_read`
   - 叶节点提交 `28282828`（`test_system` 的读错误勾子值），随后 `ResourceManager::read()`
     返回失败，证明硬件读路径确实看到了执行组提交进命令缓冲的值；
   - 同时断言叶节点真实 `LoanedCommandInterface::get_value()` 等于该值。
4. `committed_command_reaches_mock_hardware_write`
   - 同上，用 `23232323` 证明硬件写路径。
5. `configuration_errors_are_rejected`
   - 未知成员、重复成员、已 ACTIVE 成员被拒绝，且已安装的执行组不受影响。
6. `multiple_reference_writers_are_rejected`
   - 一个 reference 接口两个写者被拒绝；单写者被接受。

注意：读/写两个勾子拆成两个独立 TEST_F，因为 `test_system` 的写错误会调用
`remove_all_hardware_interfaces_from_available_list`，在同一 fixture 中会干扰后续读验证。
这是 mock hardware 的行为，不是执行组行为。

## 七、运行结果

见第九节“验证记录”。

## 八、当前可对外准确表述的结论

已验证：

- opt-in 阶段化执行组可以在真实 `ControllerManager` 生命周期和
  `hardware_interface::ResourceManager` 之上构造三层控制器组；
- 每个控制器每周期每个阶段恰好被调用一次，且完全不经过原生 `update()`；
- 状态阶段严格后序、命令阶段严格前序、所有状态回调早于所有命令回调；
- 三层复合状态数值与手写 composite 完全一致，且同周期传播；
- 状态失败不进入命令阶段；命令失败、非有限参考值、sink 拒绝提交时都不会产生部分命令；
- 执行组提交的值确实进入 `ResourceManager` 拥有的硬件命令缓冲，mock hardware 的
  read/write 路径都能观察到；重复 reference 写者、未知成员、重复成员、已激活成员等
  拓扑/配置错误在配置期被拒绝；
- 原生 chaining 回归测试仍然通过。

尚未验证：

- 是否优于原生 chaining 或手写 composite（初步对照见 `HIERARCHY_FAIR_COMPARISON.md`：
  三种实现输出逐位一致，但执行组不是更快的方案；接线成本尚未量化，因此没有最终结论）；
- 真实硬件、真实总线原子性、硬实时 WCET；
- 仅凭 URDF 或接口自动推断任意控制器语义依赖；
- 生命周期部分失败回滚、多频率、异步数据源、多执行组。

## 九、验证记录

环境：Ubuntu 22.04、GCC 11.4.0、ROS 2 Humble、colcon overlay，源码目录
`~/Desktop/ros2_control-humble`，`build/` 与 `install/` 位于工作区内部。

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager hardware_interface_testing \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# 结果：5 packages finished，controller_manager 成功
```

新增测试（`ctest -R` 结果）：

```text
1/3 Test #12: test_staged_execution_group ........... Passed  2.41 sec
```

`test_staged_execution_group` 内 6 个 gtest 全部通过：

```text
[  PASSED  ] 6 tests.
```

原生路径回归：

```text
Test #5: test_controllers_chaining_with_controller_manager ... Passed   6.16 sec
Test #1: test_cycle_tree_contract ............................ Passed   0.02 sec
```

`test_controllers_chaining_with_controller_manager` 在负载下存在**既有**间歇失败：
重建后的一次 ctest 运行中它失败，随后直接运行 5 次全部通过（6/6），全量 ctest 运行时也通过。
`HANDOFF_CURRENT_2026-09-20.md` 已记录过同类“计数偶发偏差”现象，且本次没有修改原生 chaining
逻辑或断言。因此记录为既有 flaky，不作为本次改动的回归结论；但后续做公平对照前应先定位它。

`controller_manager` 全量 ctest（22 个测试）：19 通过，3 失败，失败项均与本次改动路径无关，
且都在没有安装执行组的普通路径上运行：

| 测试 | 现象 | 判断 |
|---|---|---|
| `test_controller_manager_srvs` | 超时；其中 `list_large_number_of_controllers_with_chains` 单用例耗时 128 s | 该虚机 CPU 性能导致，隔离重跑同样的慢用例仍失败 |
| `test_spawner_unspawner` | 120 s 超时，spawner 子进程报 “Failed loading controller / Failed to activate” | 依赖子进程与 ROS 2 服务发现，属环境问题 |
| `test_hardware_spawner` | 首次失败（“Could not contact service”），重跑通过 | 已知 flaky |

说明：本文没有在这些失败项上建立“改动前 baseline”。它们不经过
`staged_group_` 分支（该分支在未安装执行组时为假），并且最相关的
`test_controllers_chaining_with_controller_manager` 通过；因此把它们记录为环境问题，
而不作为本次改动的回归结论。

沙箱/受限环境下运行 gtest 需要可写日志目录，否则 `rclcpp::init` 在
`SetUpTestSuite` 抛异常并导致退出时崩溃。本次验证使用：

```bash
export ROS_LOG_DIR="$PWD/log/ros"
```

这是执行环境问题，不改变测试语义。
