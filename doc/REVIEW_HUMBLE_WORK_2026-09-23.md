# humble-work 科研与实现评审

评审日期：2026-09-23。审阅提交：`b1bf6166dd489f620e887118557b1cd7c68d1065`。
目的：评估研究论证、实现契约、实验可信度和后续优先级；不是投稿录用预测。

## 1. 总体意见

建议：**大修后再形成投稿结论**。工作已明显超出最初独立原型，形成了独立库、manager
适配、通用 composite 对照、静态契约工具及 Gazebo 案例。保留“不更快、尚无可靠跟踪
收益、接线便利不专属于 manager”的负结果，是有价值的研究进展。

然而，“理论与机制已完成，只差论文整理”的判断偏乐观。发现了明确的理论错误，
以及输出新鲜度、提交边界和类型安全缺口。当前不能把两阶段可行性、完整数据契约、
Gazebo 验证和零分配合并为一条无条件保证。

主线仍可保留：**限定组件模型下的同周期双向执行，以及可复用组件组合的可检查契约
与代价**。不建议继续用一般图最优调度或性能全面优越作为主要卖点。

## 2. 审阅范围与实际验证

- 读取执行内核、阶段接口、manager 改动、静态绑定、测试源码、Gazebo 控制器和测量脚本；
  对照 FORMAL_MODEL、PASS_LOWER_BOUND、PAPER、HANDOFF_MANUAL 等文档。
- Windows / GCC 8.1：`research/cycle_tree` Release 构建及 CTest 1/1 通过。
  注意这是旧标量核心，不是新 staged 内核。
- `test_static_topology_negative.py`：7 个应失败样例和 1 个应成功样例符合预期。
  这验证样例覆盖的错误，不证明全部静态契约完备。
- 原仓库 `search_min_passes.py` 运行成功；其三节点输出本身含两个“配对冲突下界不紧”的图。
- 新增评审探针 `research/review_2026_09_23/check_pass_model.py`：
  四节点全部 4096 个无自环有向图均可被一个全序及其逆序覆盖；
  最少类数分布为 0:1、1:542、2:3553；同时验证统一方向图成环的反例。
- 新增 `pointer_erasure_probe.cpp`：本机复现第二基类指针经 void* 擦除后恢复错误，输出
  `erased_pointer_differs_from_adjusted_base=1`。仅比较地址，未解引用错误指针。
- **本轮没有重跑 ROS 包级测试、sanitizer、Gazebo 或性能实验**。远端文档中的测试数量、
  耗时和仿真数值是已有报告，不能当作本轮独立复验。

严重程度：P1 表示阻断核心论文主张或有实际错误行为，P2 表示范围、方法或维护缺陷。
证据标注区分直接复现、确定代码路径和待实验验证的风险。

## 3. 主要问题

### R1 [P1，数学反例已验证] 最少趟数的复杂度结论错误

位置：`doc/PASS_LOWER_BOUND.md:145`，`doc/PAPER.md:467`、1137；定义见 PASS_LOWER_BOUND 第 1 节。

文档定义“合法 k 趟”为每条边至少在某趟中顺序正确。对任意无自环图，任选全序，
将边分为顺序向前与向后两组，二者都无环。所以该边覆盖模型中答案恒为：
无边 0；非空 DAG 1；其余 2。不存在这里声称的一般 NP 难问题，也不需要近似求解。
自环若被允许，则任何严格全序都不能满足，必须单独处理。

更关键的是，该覆盖问题不是阶段化控制执行语义：每条边曾在某趟被满足，不能保证
所有数值对应同一个逻辑周期/版本；重复融合 update 还可能重复推进积分器和估计器。

建议：撤回 NP 难和一般最优调度叙事，保留受限模型中单次融合调用的冲突证明；
用实际阶段顶点 S_i/C_i 建图证明充分性。两趟“最优”须限定为完整阶段遍历次数，
不能扩展为最少 CPU 时间、最小端到端延迟或任意调度最优。

验收：同步修正文稿、交接手册、穷举脚本的 Interpretation；加入四节点穷举与说明性反例。

### R2 [P1，反例已验证] 统一方向图成环不等于实际代数环

位置：`doc/FORMAL_MODEL.md:140`，`doc/PAPER.md:393`。

取 reference 边 A→B，state 边也 A→B。按文档定义把 state 翻转后得到 G 的 A↔B，
但实际约束是 S_A→S_B、C_A→C_B、S_A→C_A、S_B→C_B，
合法执行为 S_A,S_B,C_A,C_B。

G 无环是使用“同一线性化的正反遍历”的充分条件；G 有环说明该表示受限，
不能证明任何阶段调度都不可能。应检测实际阶段依赖图的环。
原来的双向父子树充分性不受此反例影响。

另：深度 D 的“恰好延迟 D”应说明每级成功、同频同步、无额外缓存、跟踪的是软件来源周期。
任意历史依赖函数可能包含额外算法延迟，阶跃可观测响应也未必恰好 D。

### R3 [P1，确定代码路径，待 ROS 回归用例] 漏写输出被重新标记为新鲜

位置：`hierarchical_control/include/hierarchical_control/staged_execution_group.hpp:239`、291、
369、477；writer 定义见 `staged_controller_interface.hpp`。

每周期清空 frame，但 state_values、reference_values、actuator_scratch 不清空；
返回 OK 后只验证 finite，并由内核设置 valid/current cycle。
因此成功一轮后插件下一轮漏写某元素并返回 OK，旧有限值仍可被当成当前周期值提交。
初始零值也能掩盖首次漏写。根 source 的部分写入也有相同风险。

建议：为每端口维护写入覆盖信息，或在只允许有限值的契约中每轮填 NaN 后验证；
不要让标记 current cycle 取代“确实产生了输出”的检查。
验收：状态/reference/actuator/source 分别测试全漏写、部分漏写，验证失败及不提交。

### R4 [P1，确定代码路径] sink 中途失败仍造成部分软件提交

位置：`staged_execution_group.hpp:386` 至提交循环；单测 `test_execution_group.cpp:302`。

逐叶调用可失败的 sink->commit，成功一个即复制到 actuator_committed 和 committed_。
两叶场景中第一个成功、第二个失败时，真实命令句柄和内部 committed_ 都可能部分更新，
committed_cycle 却仍是旧周期。现有单叶 fail_commit 测试没有覆盖这一情况。

代码注释明确把 sink 失败排除在契约外，应尊重这一范围；但文档中的
“sink 失败也不提交”“all-or-nothing”不能沿用为无条件结论。
sink 是软件适配器，失败不必然等价于物理硬件故障。

建议：分清计算失败与提交失败；优先设计单一软件命令向量的发布边界。
如果采用逐句柄 copy，须在写前完成所有检查，并要求最终软件写入不可失败，
另行处理 hardware.write 失败。内部 committed_ 至少应在全部 sink 成功后更新。
验收：两叶第二 sink 拒绝；对真实句柄、内部 committed_、周期号分别断言，明确异常状态。

### R5 [P1，C++机制已复现] void* 擦除破坏多继承接口指针

位置：`topology_contract.hpp:246`、256、266；`topology_binding.hpp:101`。

make_leaf/compose 接受 void*，随后 static_cast<StagedControllerInterface*>(raw)。
若插件为 ControllerInterface 与 StagedControllerInterface 多继承，Derived* 到第二基类
通常需要地址偏移；Derived*→void*→第二基类指针不会执行该调整。
现有 binding 测试只有单一 Staged 基类，未暴露此问题；任意无关对象指针也能进入入口。

建议：在知道 Derived 类型时先检查 is_base_of，并完成 Derived*→Staged* 转换，
之后才允许擦除；或一直保存 typed interface pointer。保持纯模板层不依赖 ROS 时，
应由有类型的 adapter 完成转换，而不是盲目恢复 void*。
另：to_library_spec 在检查向量长度之前访问 rows.names[i] 生成空实例错误消息，
长度不一致且含 null 的公开 SpecRows 可越界，应先做长度验证。
验收：实际多继承插件、非接口类型 compile-fail、长度不一致的 malformed rows。

### R6 [P1，确定代码路径] 多端口被误判为多个写者

位置：`staged_execution_group.hpp:148` 至 166。

父声明 child/velocity 和 child/position 时，第一个端口设置 parents[child]，
第二个端口仅检查 parent 非空，就抛 multiple writers；实际上写者是同一父节点。
二维/三维参考在控制系统中很常见，不能用节点已有 parent 判断端口冲突。

建议：相同 producer→consumer 边去重；按完整端口名验证唯一写者，再分别检查单树约束。
测试同父多端口成功、不同父冲突失败、自引用/不存在端口拒绝。
当前还只按前缀识别端点，未核对实际导出端口清单；静态 owner 存在不等于端口存在。

### R7 [P1/P2，代码路径已确认] 两条 manager 执行路径的保证不可混用

位置：`controller_manager/src/controller_manager.cpp:2336`、2364、2431；
`case_study/src/wheel_controller.cpp` 与 `chassis_controller.cpp`。

StagedExecutionGroup 有 frame/scratch；TwoPhaseControllerInterface 路径仅拆回调。
Gazebo 使用后者：状态返回 ERROR 仍跑 handle，控制器直接写 command handle，没有组提交。
所以仿真结果仅能支持阶段顺序实验，不能验证 staged 故障契约。

若一个实例实现两个接口并同时加入 staged group，两阶段循环没有 staged owns 排除，
可在同一周期重复执行它。set API 也未强制这两个模式互斥。
两条新路径还绕过 legacy controller_update_rate 门控，配置了较低频率会被按 manager 频率调用，
且未在组配置时拒绝多频。可以限制首版同频，但限制必须被代码验证。

建议：统一实现路径或明确两种实验配置；配置期拒绝重叠成员和不支持的频率。
验收：同时实现两个接口的插件每阶段恰好一次；低频配置明确拒绝；失败传播符合所选契约。

### R8 [P1/P2，静态并发风险，未运行TSan] 非实时配置与实时执行缺少发布协议

位置：`controller_manager.cpp:2200`、2240、2303、2315、2359。

set_staged_execution_group/clear 对同一 staged_group_ shared_ptr 写入或 reset，update 并发读取；
共享指针引用计数线程安全不代表同一个 shared_ptr 对象的读写安全。
成员 inactive 不代表 manager 的实时线程已停。two_phase_enabled 和 entries 也缺乏发布同步。

dirty 路径直接在 update 内 rebuild，含 reserve/push_back、dynamic_cast、sort 和旧 vector 释放。
“不持锁”不能推出“实时安全”或“配置都在非实时阶段”。

建议：离线构建不可变计划，使用已有 RT 双缓冲/受控切换发布并安排旧对象非实时回收；
或者把 API 严格限制为 manager 更新线程未运行时并落实状态检查。
验收：切换/配置并发测试、运行期分配探针涵盖首次启用和切换后周期，而非只测稳态。

### R9 [P1，测量方法核查] Gazebo 周期陈旧量不是直接测量

位置：`case_study/scripts/measure_tracking.py:47`、79、91、101。

消息无生产周期和源时间戳，使用订阅回调接收时间，按名义 1/rate 平移后最近邻匹配。
DDS/队列/不同话题偏移、丢包和仿真速度与接收时钟差异都影响结果；未限制最大匹配距离。
若无 wheel 数据，所有误差为 inf，errors.index(min(errors)) 仍得到 0，可能输出伪“零滞后”。
常量或变化很小的信号也无法可靠识别唯一 lag。

建议：在生产/消费位置直接记录 producer_cycle、consumer_cycle、源 sample_ns 和来源值；
ring buffer 收集后非实时输出。缺数据必须实验失败，不得默认为 lag=0。
将 20/50 ms 改成“按配置控制周期换算的调度延迟”，不能称为虚拟机实测墙钟代价；
0 cycle 更不等于 0 ms 端到端时延。现有相关性估计保留为辅助。

### R10 [P2，证据边界] 上游行为与存储表述超过实际验证

`test_upstream_ordering.cpp` 只添加/配置并检查排序；父声明 ord_child/state，但测试
没有实际导出、激活认领和消费该状态端口。configure 接受不等于整条原生数据路径可执行。
Humble 用注册表实现派生状态是可行实验适配，但必须与原生 state interface 区分。
Jazzy/master 算法片段等价或 Python 移植不等于完整版本上的插件运行验证。

“零额外存储”也应修改：`hierarchy.hpp:31` 仍保存 preorder 与 postorder，
虽然新内核只用后者，前者仍被生成；scratch/frame/committed 更有明确额外空间。
可说“第二遍不要求第二份顺序”，不能说系统不增加存储。两份数组同样是 O(V)，
不能将常数项变化包装为渐近复杂度提升。

### R11 [P2，方法学] 对照及新颖性需要重新收敛

“融合 update 中双向先后矛盾”在所列假设下成立，但论证基础性很强，不足以单独承载
新的调度理论论文。需要相关工作系统对照，特别是同步数据流、阶段化执行、逻辑执行时间、
控制组件组合和故障一致性；本轮未完成外部文献查新，不作“首创/无创新”的定论。

估计/命令显式拆分仍是重要强基线。通用 composite 可以复用同一个内核，说明
组装便利与组提交不必要求修改 manager；Gate B 失败后应解释后来 manager 扩展服务于
哪个独立需求，而不能仅靠可单独加载组件重新默认它必要。

叶滤波状态不能由瞬时位置重建，不代表父保存相同历史后不能计算。合理收益是避免
算法复制、保护模块封装。不要为“让机制有效”不断选择性换案例，应事先说明物理需求。
固定 alpha=0.85 在 20/50 Hz 下对应不同连续时间滤波常数；跨频率性能比较应固定时间常数。
只减少采样等待不自动带来跟踪收益；不必为投稿强造高带宽正结果。

## 4. 推荐修复与验收次序

1. **理论与表述**：修 R1/R2/R10，并全局检索 PAPER/HANDOFF 中相同结论，避免旧结论继续传播。
2. **契约可信度**：修 R3/R4/R5/R6；新增有意漏写、第二 sink 失败、多继承和多端口测试。
   这些测试须先在当前版本失败，再在修复版本通过，避免只验证实现的理想路径。
3. **执行路径和并发**：修 R7/R8；明确管理器停止期配置还是实时发布；验证模式互斥与频率限制。
4. **可审计仿真**：用周期号直接测 lag，记录缺失样本、仿真时钟、启动失败和每次运行元数据。
5. **论文贡献判断**：补拆分组件基线，统一算法/缓冲/故障策略；再评价收益和维护成本。

不建议此时继续扩展多频、异步或一般 DAG；先闭合已声明的单频单树保证。

## 5. 可保留与应暂缓的论文主张

可以保留：受限双向树中两阶段顺序的充分性；组件可复用与接口检查的工程设计；
不同宿主、性能开销和负结果；已有测试覆盖范围内的数值一致性。

需要限定：同周期仅指软件来源周期；最旧 sample 时间不包括滤波器全部历史支持；
无分配只针对被测稳态路径；提前中止计算不等于 sink 失败可回滚。

暂缓：一般图 NP 难/一般调度最优；统一方向图的环必然是代数环；无条件整组原子提交；
Gazebo 已验证全部数据契约；完整软件系统零额外存储；生产可用的实时 manager；性能优于原生。

## 6. 版本和本地工作说明

按用户要求已切换到本地 humble-work，跟踪 origin/humble-work。原 humble 分支未提交内容
保存为 stash `pre-humble-work-review-2026-09-23`（创建时 stash@{0}）。原分支及提交未改写。
本轮仅新增本文和两个评审探针，不修改被审生产代码、不提交、不推送。
需要恢复旧工作时先切回 humble，再核对 stash 列表并 apply 对应保存项，勿直接在新分支 pop。

远端分支与原 humble 当前无共同祖先，可能是重新初始化后的快照，本文不推测原因。
因此本轮按提交内容审阅，不把跨分支 diff 当作连续提交历史。后续建议保留小步提交和
实验版本标识。仿真日志目录还跟踪了 PulseAudio cookie 等环境运行产物，应清理这些
与科研复现无关的文件，保留有选择的实验输入、结果与依赖清单。

最终结论：**值得继续，但下一阶段应是“修正论证、补反例、闭合契约”，而不是继续堆功能或直接投稿。**
