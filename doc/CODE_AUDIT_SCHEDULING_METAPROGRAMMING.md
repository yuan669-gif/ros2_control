# 双向调度 + 元编程 代码审查记录

日期：2026-09-24
范围：`hierarchical_control/`（调度内核 + 元编程层）与 `controller_manager/`（两趟路径）
起因：用户要求"重点把双向调度 + 元编程的代码写好"，本轮做一次查漏补缺
结论：**发现并修掉 1 个真实运行期缺陷、1 个假阳性检查、2 处语义缺口**；
其余检查项经确认无问题（列在 §4）。

---

## 1. 修掉的问题

### 1.1 【真实缺陷】`TypedPorts` 三组端口模型把三种不同的东西混在一起，状态槽多算

**症状（后果）**：一个控制器只要同时有"自身状态"和"接收的 reference"，它的
`staged_state_ports()` 就会被生成为 `Exported ∪ Consumed` 各加 `/state` 后缀，于是

| 节点 | 声明的端口 | 旧模型给出的状态槽 | 内核实际需要的 |
|---|---|---|---|
| wheel（1 个自身状态 + 1 个接收 reference + 1 个写子节点的 reference） | 见 `test_typed_ports.cpp` | **3** | **1** |

内核（`staged_execution_group.hpp:512`）只按长度分配：
`state_values_[i].assign(controller.staged_state_ports().size(), 0.0)`；
而 `run_ns` 会把状态槽**预填 NaN**，并在状态阶段后做 `all_finite()` 检查（评审 R3 的
"声明的端口必须都写"）。因此**只写自己真实状态的控制器会每个周期都以 `state_failed` 失败**；
即使它把垃圾写进多出来的槽以骗过检查，它的父节点收到的"子状态视图"里也会混进
`tire/target/state`、`wheel/target/state` 这类根本不是状态的槽位。

**为什么之前没被发现**：`test_typed_ports` 的 mixin 控制器在状态阶段里写的是
`for (i < state.size()) state[i] = 1.0`——**把所有槽都写了**，于是检查通过，
多出来的槽位被掩盖了。文档例子（`wheel_target` / `wheel_torque`）恰好**没有 `Consumed` 端口**，
也掩盖了问题。

**修复**：端口声明拆成四组，各组含义与内核用途一一对应：

```cpp
using wheel_ports = TypedPorts<
  PortList<wheel_travel>,    // State       : 本节点发布的状态（父的状态阶段直接读本节点槽）
  PortList<wheel_target>,    // Reference   : 本节点接收的 reference（父的命令阶段写）
  PortList<wheel_torque>,    // Actuators   : 硬件命令端口
  PortList<tire_target>>;    // ForChildren : 写进子节点的 reference（仅父子静态检查用）
```

`staged_state_ports()` 现在返回**恰好声明的状态端口，不加后缀**。
`kernel_state_slots / kernel_reference_slots / kernel_actuator_slots` 把"内核要用的三个长度"
显式命名，测试直接对它们断言。

**验证**：`test_typed_ports.generated_strings_match_the_declaration` 现在断言状态列表是
`{"wheel/travel"}`（1 个，不是 3 个）并 `static_assert` 三个槽位数；
`a_mixin_controller_runs_in_a_real_group` 仍在真实执行组里跑通。

### 1.2 【假阳性】`declarations_are_compatible` 把**正确**的父子对判为不兼容

旧实现比较"父的整个 `Exported` 列表"与"子的 `Consumed` 列表"。而父的 `Exported` 里混着
**自己的状态**（`wheel/travel`）和**写进子节点的 reference**（`tire/target`），
子的 `Consumed` 是它接收的 reference（`tire/target`）——于是 `{tire_target, wheel_travel}`
vs `{tire_target}` 判为不兼容。

**这个假阳性被测试当成了期望行为**：旧 `test_typed_ports` 的注释写着
"these genuinely differ, so the pair must be reported as incompatible"。也就是说，一个检查器
报错、而测试把它合理化了——这正是评审反复指出的那类失败模式。

**修复**：`declarations_are_compatible<Parent, Child>` 现在比较
`Parent::for_children` 与 `Child::reference`——父声明写进子节点的 reference
vs 子声明接收的 reference，同一方向、同一事实。量纲检查（`same_port_v`）照旧生效。
测试改为 `static_assert(tp::declarations_are_compatible<wheel_ports, tire_ports>())`，
并保留"同名不同量纲必须被拒"的用例证明检查非空洞。

### 1.3 【语义缺口】状态端口没有被任何自检覆盖，但文档声称被覆盖了

`PORT_DIMENSIONS.md` §3 写着"控制器自报端口与声明不一致 ⇒ 由
`verify_ports_match_interface` 运行期发现"，但该函数**只查 reference 和 actuator 两张表**，
状态表从未检查。文档的覆盖范围大于实现的覆盖范围。

**修复**：`verify_ports_match_interface` 现在三张表全查（含状态），并新增两个对齐器：

| 函数 | 作用 |
|---|---|
| `verify_ports_match_interface<ControllerT, Ports>` | 控制器自报字符串 vs **`TypedPorts` 声明**（三张表） |
| `verify_ports_match_contract<ContractT, ControllerT>` | 控制器自报字符串 vs **绑定时用的 `Contract`**（状态/参考两张表，**逐位置比名称与顺序**，不只是长度——评审 C） |
| `topology_binding::verify_binding_ports(binding)` | 沿绑定**树**递归查完**每一个孩子**，失败时给出"哪个节点、哪张表"以及**实测列表 vs 声明列表**（评审 B 的递归、2026-09-24 的诊断改进） |

新增 3 个负向用例（状态表不符、状态数多一个、参考数不符）证明检查非空洞，
以及 1 个正向用例证明"声明一致 ⇒ 通过"。

### 1.4 【语义缺口】两趟路径没有故障包含：状态阶段失败后仍然跑命令阶段

`update()` 的 pass 1 里，`update_phase` 返回 ERROR 时只记录日志并把返回值置 ERROR，
**pass 2 照常对所有控制器执行 `handle_phase`**。也就是说：一个控制器刚否掉了自己的状态，
管理器仍然用那个状态算命令并写进硬件命令接口；它的上游（状态依赖它的那些父节点）也不可信。

**修复**：任一 `update_phase` 失败 ⇒ **该周期的整趟命令阶段都不跑**，命令接口保持上一周期
的值，管理器返回 `ERROR`。这是"能避免拿被否数据去执行"的**最弱**规则，不假装是原子提交
（命令阶段已写出的值不会回滚，见 §3 的诚实边界）。
新增用例 `a_failed_state_stage_suppresses_the_command_stage`：leaf 的 `update_phase` 失败时，
root/mid/leaf 三者的 `handle_phase_calls` 全都不增加、执行器值不变；清除失败后恢复。

### 1.5 【语义缺口】切换期间，两趟成员会被悄悄退回融合单趟语义

原生循环跳过两趟成员的判断绑在 `run_two_phase` 上，而 `run_two_phase` 在"有切换挂起"时为
`false`。于是一个 `two_phase_legacy=false` 的成员在切换的若干周期里会被原生 `update()` 接管——
语义与稳态不同（对同时实现两个入口的控制器还会**多推进一次状态**）。
对照：staged 成员用的是无条件的 `staged->owns()` 判断，本来就不会这样。

**修复**：跳过判断改为 `two_phase_enabled_ && two_phase_index(...) != no_two_phase`，
即"只要两趟开着，成员永远不走原生路径"；两趟暂停的那些周期它**只是不执行**。
新增用例 `a_member_is_never_run_by_the_native_loop_during_a_switch`：切换窗口内
`legacy_update_calls` 严格不变（旧实现会 +1 或更多），`update_phase_calls` 少于驱动周期数
（证明窗口里确实有暂停周期，测试非空洞）。

### 1.6 【实时路径卫生】两趟关闭时仍在每周期做一次原子加载

`update()` 无条件 `std::atomic_load(&two_phase_entries_)`。默认配置（两趟关闭）下这是白付的开销。

**修复**：`two_phase_enabled_ ? std::atomic_load(...) : nullptr`，关闭时不做原子操作。

> 关联的既有教训（本文件不重复，见 `REVIEW_RESPONSE_2026-09-23.md`）：
> 配置路径上无谓的工作会改变实时循环在切换窗口里完成的周期数，计数器型断言能看见它。
> 本轮同样检查了新增的验证函数：它们都只在显式调用时执行，不在任何每周期路径上。

### 1.7 【语义缺口，第二轮补齐】父子**状态边**没有任何静态检查

`PORT_DIMENSIONS.md` §3 自己列过这条边界："父子之间的状态边（父读子状态）还没有静态成对检查"。
后果很具体：内核把**子节点的状态槽直接**交给父的状态阶段（`input_views_[node][slot] =
StagedValueView(state_values_[child]...)`），而父并不声明它要读子节点的哪些状态。父若按错误的
位置或数量去索引，表现是**父节点在状态阶段报 `state_failed`**——离"声明写错了"这个真实原因很远。

**修复**：端口声明加第 5 组 `ChildState`（父声明它从子节点读哪些状态），并让检查按边分开：

| 谓词 | 比较 |
|---|---|
| `reference_declarations_agree<Parent, Child>()` | `Parent::for_children` vs `Child::reference` |
| `state_declarations_agree<Parent, Child>()` | `Parent::child_state` vs `Child::state` |
| `declarations_are_compatible<Parent, Child>()` | 两者都要成立 |

两条边**可以分别断言**，所以静态断言失败时能直接指出是哪条边。新测试用一个"父声明了错误量纲的
子状态"的用例证明 state 检查真的会拦（而 reference 边仍然通过，说明两个检查互相独立、都不空洞）。
`ChildState` **不进内核**——它不参与任何缓冲分配，只是父对输入的期望，这一点在头文件里写明了。

**仍然只在"链"上精确**：编译期 binding（`topology_binding.hpp`）本身只能表达一条链
（每个 `BoundNode` 只有一个 `next`），树形只在运行期内核里存在。这一点也写进了文档。

### 1.8 【负结果】深链编译成本的"优化"实测是**反优化**，已否决

`COMPILE_COST.md` §3 一直写着一条"明确的优化方向"：`ancestry` 按值复制父数组，
改成沿父链查询就能把 `O(d²)` 降到 `O(d)`。本轮把它**真正实现并测量**了（深度 64 的链，
`-ftime-report` 的 template instantiation，3 次取最好）：

| 变体 | 时间 | GGC 内存 |
|---|---|---|
| **按值复制数组（现行，保留）** | **0.17 s** | **13 MB** |
| 沿父链递归（已否决） | 0.55 s | 20 MB |

**慢 3.24 倍**。原因是渐近论证用错了代价模型：复制数组是每个节点**一次** constexpr lambda 求值；
沿父链查询是**递归函数模板**，每层祖先都是一次独立实例化（各自的符号与 constexpr 求值）。
抽象步数少 ≠ 实例化代价低。

**处理方式**：现行头文件**不改**（与 HEAD 逐字节相同），被否决的实现与复现脚本留在
`research/static_topology_variants/`，`COMPILE_COST.md` §3 与 §"不能声称"都改成实测结论。
这条从"未做"变成"**已调查并基于证据否决**"。

---

## 2. 文件与测试清单

| 文件 | 改动 |
|---|---|
| `hierarchical_control/include/hierarchical_control/typed_ports.hpp` | 四组端口模型；重写头部说明（含"内核只用长度"这一事实）；`name_lists` 四表；mixin 三张表按声明原样生成；`declarations_are_compatible` 改为 `ForChildren` vs `Reference`；新增 `verify_ports_match_contract`；`verify_ports_match_interface` 三表全查 |
| `hierarchical_control/include/hierarchical_control/topology_binding.hpp` | 新增 `verify_binding_ports()`（沿类型链逐节点校验） |
| `hierarchical_control/include/hierarchical_control/two_phase_controller_interface.hpp` | 明确写出两趟的**故障语义**与**切换期间行为** |
| `controller_manager/src/controller_manager.cpp` | 故障包含（状态失败 ⇒ 跳过整趟命令阶段）；切换期间不走原生路径；两趟关闭时不做原子加载 |
| `controller_manager/test/test_staged_controller/*` | 新增 `set_fail_update` / `set_fail_handle` 两个故障注入钩子（互不影响 staged 与原生路径） |
| `hierarchical_control/test/test_typed_ports.cpp` | 适配新模型；新增 3 个负向 + 1 个正向校验用例；`wheel_ports` 示例改为五组；两条边的独立断言 |
| `hierarchical_control/test/test_topology_binding.cpp` | 新增 `binding_level_port_verification`（正向 + 用最小 stub 演示负向） |
| `controller_manager/test/test_two_phase_execution.cpp` | 新增 2 个用例（故障包含、切换期间不退化） |

---

## 3. 仍然存在、且**不打算**在本轮消除的边界（诚实清单）

| 边界 | 说明 |
|---|---|
| 两趟**没有**整组原子提交 | 命令阶段直写 command handle；后段失败时前段已生效（实测 mid 的已认领接口 −1.875 → −5.5625）。只有 staged group 有该保证 |
| 两趟是"单频、同步"的 | 多频/异步/动态拓扑未支持（评审也建议此时不要扩展） |
| 执行器端口无法与 `Contract` 对齐 | `Contract` 有意不含硬件执行器端口，没有可比对象；只有 `verify_ports_match_interface` 能查它 |
| 静态检查**只对链精确** | 两条父子边现在都检查了，但编译期 binding 只能表达一条链（`BoundNode` 只有一个 `next`），树形只在运行期内核里存在 |
| 端口**名字**不是内核契约的一部分 | 内核只用长度；名字正确性靠显式自检函数。这是设计选择，已写进 `PORT_DIMENSIONS.md` |
| 验证函数是运行期调用 | 构造控制器不是常量表达式，所以无法前移到编译期；它们也**不会**自动在 `create_library_group` 里跑（否则会破坏那些端口与契约本就不一致的最小 stub），需要调用者显式调用 |
| `plan_.preorder` 仍被赋值但内核不用 | 兼容保留（`reversed postorder` 就是父先序），见 `IMPLEMENTATION_GUIDE.md` §12.4 |
| `test_controllers_chaining_with_controller_manager` 的计数断言 | **与本次改动无关的时间校准问题**，实测证据见 §3.1 |

---

## 3.1 一个既有测试是**时间校准**的，不是本次改动造成的（必须写清楚）

`test_controllers_chaining_with_controller_manager` 有 3-5 个用例断言**精确的**
`internal_counter`（例如"3 次激活之后计数 == 3"）。这个期望等价于"每次切换恰好让更新线程
tick 2 次"，而更新线程是 `ControllerManagerFixture::startCmUpdater`
（`controller_manager_test_common.hpp:123`）：

```cpp
while (run_updater_) {
  cm_->update(...);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

一次切换需要"应用切换"和"实时线程离开旧列表"两个 tick，而墙钟上一个 tick 10 ms，
所以**只要切换线程的后处理偶尔跨过第三个 tick，计数就多 1**。

**实测（本机，无代码改动，只改负载）**：

| 条件 | 6/6 全绿的运行次数 |
|---|---|
| 空闲 | **0/4** |
| 人为加 2 个 CPU 忙循环 | **4/4** |

也就是说：这个测试的通过与否由**墙钟节奏**决定，与调度实现无关。
我在 `REVIEW_RESPONSE_2026-09-23.md` 里曾写"修复后连跑 3 次均 6/6"——那三次恰好是
**紧跟一次 14 分钟编译之后**（机器正忙），属于同一现象，**不该**当作"修好了"的证据。
我先前把"配置路径上多余的 `dynamic_cast`/`make_shared`/`sort` 拉长切换"当成唯一原因，
那个修复本身是对的（它确实减少了一次抖动），但它**不能**让这个测试变确定。

**建议的后续（本轮不做）**：把该 fixture 的 10 ms tick 调大（例如 25–50 ms），
或改成"由测试显式驱动 `update()` 次数"而不是睡线程；两者都会影响所有使用
`ControllerManagerFixture::switch_test_controllers` 的上游用例，需要逐条复核期望值，
不属于本次"调度 + 元编程代码"的范围。

**给后续会话的判据**：看到这里失败时，**先看失败用例的子集在不同运行间是否变化**、
以及机器是否空闲；不要直接当成调度回归。真正的调度回归有专门用例守着
（`test_two_phase_execution` 12 个、`test_staged_execution_group` 6 个、
`test_hierarchy_comparison` 6 个）。

---

## 4. 检查过、确认没问题的项

| 检查项 | 结论 |
|---|---|
| `run_ns` 的分配 | 状态/命令/提交三阶段均为成员缓冲区，无每周期分配（`run allocations per 100 calls = 0`，探针在 `test_hierarchy_comparison`） |
| `run_ns` 的返回路径 | 全部 18 个返回点已逐一核对（NaN 哨兵、帧校验、fault code、两遍提交）；`test_contract_regression` 12 用例覆盖 R3/R4/R5/R6 |
| `update()` 的两趟遍历顺序 | pass 1 反向（子先父）、pass 2 正向（父先子），同一份 `entries`，`two_phase_index` 为 `lower_bound`（快照已排序） |
| 成员集发布协议 | 不可变快照 + `atomic_store`/`atomic_load`；TSan：racy 必报、atomic 干净（`run_tsan_publish_protocol.sh`） |
| 准入规则 | 重叠成员与频率不匹配在**两个方向**都被拒；`test_two_phase_execution` 6 个用例 |
| `two_phase_index` 的指针比较与排序 | `rebuild_two_phase_entries` 用 `std::less<const Base*>` 排序，查询用同一比较器 ⇒ 有序性自洽 |
| 内核的树校验 | 未知父、多根、环、不可达节点均在 `create`/`create_library` 期拒绝；`test_execution_group` + `test_contract_regression` 覆盖 |
| 拓扑层所有权检查 | `require_ports_are_owned<Binding>()` 编译期拒绝"引用了不存在的 owner"；编译语料 7 个必须被拒的用例 |
| 多继承指针安全 | `BoundNode` 保存类型化基类指针，`as_staged` 用 `dynamic_cast`；非零基类偏移用例通过 |
| 菱形继承 | `MinimalController` 与 `TypedPortsMixin` 都用 `virtual` 继承 |
| 编译语料 | 8/8（7 拒 + 1 非空洞对照） |
| 深链编译成本的两个变体 | `research/static_topology_variants/measure_variants.py`：现行 0.17 s / 13 MB vs 被否决的 0.55 s / 20 MB，结论稳定（各 3 次取最好） |

---

## 5. 复现

```bash
export ROS_LOG_DIR="$PWD/log/ros"; export ROS_HOME="$PWD/log/ros_home"   # 受限环境下 ~/.ros 不可写
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/build/controller_manager:$PWD/build/hierarchical_control:$LD_LIBRARY_PATH"

# 元编程层
./build/hierarchical_control/test_typed_ports          # 7/7
./build/hierarchical_control/test_topology_binding     # 6/6
./build/hierarchical_control/test_dimensional_interfaces
./build/hierarchical_control/test_static_topology
python3 hierarchical_control/test/test_static_topology_negative.py   # 8/8

# 双向调度
./build/hierarchical_control/test_execution_group
./build/hierarchical_control/test_contract_regression  # 12/12
./build/controller_manager/test_two_phase_execution    # 12/12
./build/controller_manager/test_staged_execution_group
./build/controller_manager/test_hierarchy_comparison   # 含 run() 零分配与切换后分配探针

# 并发与理论
bash hierarchical_control/test/run_tsan_publish_protocol.sh
python3 research/stage_graph/check_stage_graph.py

# 编译成本 A/B（被否决的优化）
export TMPDIR="$PWD/log/scratch"
python3 research/static_topology_variants/measure_variants.py --runs 3 --depth 64
```
