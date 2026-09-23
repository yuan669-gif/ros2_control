# FineMote × ROS 2 Humble：当前研究交接文档

更新时间：2026-09-20

本文是当前阶段的工作入口。它描述研究目标、已完成工作、已验证边界和下一步实施计划。
它不替代 `HANDOFF_2026-09-19.md` 与 `hierarchical_research.md`；当三者出现细节差异时，
以本文的“当前状态”和“验收标准”为准，并保留旧文档中的历史记录。

> 同日增补：本文第五节“阶段 A/阶段 B”的落地结果见
> `HANDOFF_STAGED_GROUP_2026-09-20.md` 与 `STAGED_EXECUTION_GROUP_EXPERIMENT.md`。
> 阶段 A/B 已实现并通过 mock-hardware 测试，但尚未做性能对照；本文其余章节仍然有效。

## 一、已经确定的研究目标

研究只针对 ROS 2 Humble，目标不是把 `ControllerManager` 简单改成一棵机械树，
也不是声称 Humble 原生 chaining 没有依赖排序能力。目标是验证 FineMote 风格的双阶段
控制组件组合能否在 ROS 2 中提供以下能力：

1. 一个周期开始时读到的有效硬件反馈，可以在同一周期内沿组件依赖从叶到根传播；
2. 根组件在本周期使用这些派生状态计算目标，目标再沿依赖从根到叶传播；
3. 所有相关组件成功后，叶命令以一个软件提交组写入硬件；
4. 任一阶段失败、数据过期或周期不一致时，不提交混合的新旧命令；
5. 组合依赖可以由接口连接生成执行计划，URDF 用于资源和机械结构校验，显式 `parent`
   只作为兼容或组合归属信息，不能覆盖已解析的接口依赖；
6. 普通 Humble controller 仍可使用原生路径，不被新旧路径重复调用。

论文问题应表述为：

> 在保留 Humble 原生 reference 依赖排序的基础上，引入显式状态阶段、周期有效性和命令组提交，
> 能否让可复用的复合控制组件完成同周期双向数据传播，并以可接受的接入和运行时成本避免陈旧或部分命令？

性能优越不是预设结论。必须与正确实现的原生 chaining、手写 composite controller 和估计/命令拆分方案公平比较。

## 二、必须保持的核心语义和不变量

### 周期顺序

目标周期为：

```text
hardware.read
  -> 状态阶段（子组件到父组件，postorder）
  -> 命令阶段（父组件到子组件，preorder）
  -> 验证
  -> committed command buffer
  -> hardware.write
```

状态边表示 `S_child -> S_parent`，命令边表示 `C_parent -> C_child`。
单个不可拆分的 `update()` 不能同时满足这两类相反方向的依赖，因此新组件必须明确实现状态和命令两个阶段。

“同周期”仅表示：周期开始时纳入快照的数据，在所有生产者成功且未超过允许年龄的条件下，
本周期完成软件命令计算和提交。周期中途才到达的反馈留到下一周期；软件提交也不代表 CAN/RS485
或电机物理运动同时发生。

### 数据契约

每个状态或 reference 至少要能表达：值、生产周期、源采样时间、有效性和故障来源（后续 ROS 适配需补齐故障来源）。
复合状态的时间戳取输入中的最旧时间，不能重新盖章伪装新鲜。每个状态/reference 必须有唯一写者，消费者只读。

### 命令提交

叶命令必须先写 scratch buffer，整棵执行树成功并完成有限值/周期/接口检查后才复制到 committed buffer。
失败时 committed buffer 和 committed cycle 保持不变；实际硬件故障动作由应用和硬件后端定义，不能默认假设“置零”或“保持旧值”普遍安全。

### 拓扑与接口

接口连接是控制依赖的来源。URDF 只能校验 link/joint、ros2_control 资源及机械锚点，不能推断 PID、MPC 或状态估计的控制语义。
显式 parent 不得产生与接口图冲突的执行计划。活动期间不改变拓扑；拓扑计划在配置/切换阶段生成，实时循环只使用整数索引计划。

## 三、已完成的研究和代码工作

### FineMote 源码核对

源码位于 `FineMote仓库镜像/FineMote`。关键事实：

- `Devices/DeviceBase.hpp` 构造时注册设备；每周期先正向遍历 `Update()`，再反向遍历 `Handle()`；
- `Devices/Motors/Motor4010.hpp` 等电机类在 `Update()` 解码反馈，在 `Handle()` 计算控制器并发送命令；
- `Components/Chassis/POV_Chassis.hpp` 的 `Handle()` 执行正运动学、里程计更新和逆运动学；
- `Interface/Examples/TaskPOVChassis.cpp` 通过电机先构造、底盘后构造形成注册顺序；
- `Algorithms/Control/ImplementControlBase.hpp` 使用模板参数和 `static_assert` 检查级联维度和连接数量；
- FineMote 的顺序依赖对象构造/依赖注入，不是由 URDF 自动推导。

FineMote 的限制也必须保留：跨翻译单元静态构造顺序仍有风险，`divisionFactor` 不是完整周期有效性协议，
逐设备发送不等于物理总线原子同步，且 POV 底盘的部分派生计算仍位于 `Handle()`。

### Humble 原生行为核对

- `ControllerManager` 已根据 chainable controller 的 reference interface 关系排序，并处理分支链；
- `ChainableControllerInterface` 在 Humble 版本导出 reference interface；
- 主循环顺序是 `read -> update -> write`；
- 原生 `update()` 出错不会自动构成统一的整组命令提交契约；
- 不能把“树遍历”或“parent 字段”本身作为创新点。

### 已提交的独立研究核心

提交：`ea3992ed feat: add experimental same-cycle tree execution and commit contract`

主要文件：

版本说明：ea3992ed 已推送到用户 fork 的 humble。`VM_VALIDATION_2026-09-20.md`、本文、
后续验证记录更新和 Duration 测试修复是此提交之后的本地工作，尚未提交/推送；
不可将这些后续文档全部归为 ea3992ed 的内容。交接时用 git status / git log 复核。

- `controller_manager/include/controller_manager/cycle_tree.hpp`：独立 C++17 标量端口执行核心；
- `controller_manager/test/test_cycle_tree_standalone.cpp`：同周期、失败、时间戳、连接校验测试；
- `research/cycle_tree/CMakeLists.txt`：不依赖 ROS 的构建入口；
- `doc/CYCLE_TREE_EXPERIMENT.md`：实验边界和复现实验；
- `doc/VM_VALIDATION_2026-09-20.md`：虚拟机验证记录和下一阶段命令。

该核心在构造阶段生成树计划和存储；运行阶段不构图、不做字符串查找、不扩容；状态后序、命令前序；
失败不更新 committed buffer。它还不是 `ControllerManager` 集成，也不是硬件安全策略。

### 已完成的验证

本地 GCC 8.1、Ubuntu 22.04 VM GCC 11.4 均完成 standalone 构建。
虚拟机普通构建和 AddressSanitizer/UBSan 构建均为 CTest 1/1 通过。
测试覆盖 1000 次同周期数值对照、两/三层树、各节点状态/命令失败、过期/未来/无效数据、恢复和非法连接。

Humble overlay 后续构建结果：6 个包构建成功，`test_cycle_tree_contract` 通过；原生 chaining 首次运行出现一次计数 +2 的失败，
随后隔离用例 10/10 通过，完整 chaining 测试 5 轮、每轮 6/6 通过。该失败目前记录为可能受后台更新线程时序影响，未修改原生断言，
也未将一次复跑通过解释为形式化证明。

## 四、当前代码状态和明确缺口

已存在但仍属实验性的 v1 文件：

- `hierarchical_controller_interface.hpp`；
- `hierarchy.hpp`；
- `controller_hierarchy_builder.hpp`；
- `hierarchical_controller_executor.hpp`。

这些文件没有完成主循环接入、ROS 接口绑定、周期数据验证、整组命令提交、生命周期回滚或硬件故障处理。
特别注意：`hierarchical_controller_executor.hpp` 仍有已知问题，状态阶段失败后会继续命令阶段，不能用于硬件。

当前新增 `cycle_tree.hpp` 也有明确边界：

- 标量状态和 reference，不是通用 ROS 接口类型；
- Connection 目前由测试/调用者提供，尚未从 Humble controller interface 自动解析；
- 没有真正的 `ControllerManager::read/update/write` 接入；
- 没有 ResourceManager claim、生命周期、频率、异步、多树和动态拓扑支持；
- 没有 URDF 校验适配；
- 回调的 `noexcept`、无分配和不访问真实句柄主要是契约，当前编译器无法替插件强制证明；
- 已提交缓冲是单线程执行边界，不是多线程原子发布。

## 五、下一阶段实施计划

### 阶段 A：ROS 适配接口

新增 opt-in 的阶段化 controller 接口或适配器，将状态输入、状态输出、reference 输出和叶命令写入映射到预分配存储。
保留普通 controller 的原生 `update()` 路径。配置时拒绝同一个 controller 同时进入 legacy 和 cycle-tree 执行组。

验收：能构造一个三层 Humble 测试组件组，组件只被各阶段调用一次，执行顺序和周期编号可观测。

### 阶段 B：ControllerManager 执行组接入

在 `read -> update -> write` 周期中加入 opt-in 执行组；ResourceManager 继续拥有真实 command interface。
所有 topology、字符串绑定、dynamic_cast 和存储分配在非实时配置阶段完成。

验收：mock hardware 记录每周期 read、阶段回调、commit、write；状态失败不会进入命令阶段；命令失败不会产生部分新命令。

### 阶段 C：依赖解析和 URDF 校验

优先使用已导出的 reference interface 和新增阶段端口元数据生成依赖图；URDF 只校验 joint/link、接口存在性和锚点。
显式 parent 只作为缺少端口元数据时的兼容字段，并检查其与端口图一致。

验收：交换 controller 声明顺序不会改变计划；缺端点、重复写者、环和冲突 parent 在配置期失败。

### 阶段 D：同算法公平对照

元编程作为组合验证手段推进：静态组件用模板/固定大小存储检查端口维度、类型与单位标签，
生成绑定和阶段计划；动态 pluginlib/YAML 配置采用配置期检查。不能承诺编译器验证任意
运行时插件，也不能以“模板一定更快”为结论。当前 scalar 核心没有完成这项 TMP 工作。
性能对照需包含同样预绑定、预分配的运行时实现，避免只对比每周期字符串查找的弱基线。

使用相同状态快照、控制律、频率、故障动作和硬件接口比较：

1. 原生 Humble chaining；
2. 估计/命令拆分；
3. 手写 composite plugin；
4. 新 cycle-tree 执行组。

测量同周期输出一致性、源采样年龄、周期错配、部分提交、动态分配、耗时分布、deadline miss 和新增接线代码量。
不把 VM 测量当成硬实时 WCET，也不把两次回调天然视为更快。

### 阶段 E：论文停止门槛

只有在新执行组相对强基线显示出可重复的组合、诊断、失败一致性或接线成本优势时，才继续扩大 `ControllerManager` 修改。
如果手写 composite plugin 以相近成本提供相同保证，应停止 manager 大改，转为复合控制器库和静态绑定工具。

## 六、下一次接手者的操作顺序

唯一开发仓库：`D:/2027-1/FineMote/ros2_control`，分支 humble，当前跟踪 upstream/humble。
origin 为 `https://github.com/yuan669-gif/ros2_control.git`；upstream 为官方 ros-controls 仓库。
官方 baseline 为 `469f3055da3b0f097d0616c8b214072434529053`，标签 baseline-humble。
不要继续在 ros2_control_humble 或 ros2_control_jazzy 旧副本开发。

开发机是 Windows PowerShell，本机没有完整 ROS 编译环境。用户在 VMware Ubuntu 22.04
中代为执行实验，已有 /opt/ros/humble、colcon、rosdep、GCC 11.4；虚拟机源码目录为
`~/Desktop/ros2_control-humble`，目前是无 .git 的副本，不能直接要求用户 git pull。
通过文件同步、补丁或重新下载更新；提供具体命令、成功判据和日志回传路径。
当前 build/install 在 /tmp/finemote-humble-build 和 /tmp/finemote-humble-install，重启后可能丢失。

本地 Duration 修复为两个原型测试文件的三处默认构造改用
`rclcpp::Duration::from_seconds(0.001)`，用户已手动同步并验证包构建成功。
实验原始信息由用户保存在项目根目录 `虚拟机实验结果.md` 和 `报错信息.md`；
复跑的 10 次/5 轮结果目前来自对话中的终端摘要，不应声称已读取其完整日志。
论文带 confidential/review 标记，保留在本地，不随代码上传公开 fork。

1. 阅读本文、`doc/hierarchical_research.md`、`doc/CYCLE_TREE_EXPERIMENT.md`；
2. 确认 Humble overlay 原生回归状态，不重写原生 chaining 测试；
3. 先设计并实现 opt-in 阶段接口和最小 mock hardware 测试；
4. 不在主循环集成前宣称 ROS 级同周期保证；
5. 每完成一个阶段运行 `git diff --check`、对应 C++/colcon 测试，并提交独立 Git 记录；
6. 推送前确认目标为用户 fork 的 `origin/humble`，不要推送 `upstream`；
7. 实验报告必须区分：算法正确性、ROS 集成正确性、性能测量和硬实时保证。

## 七、当前可对外准确表述的结论

目前已经验证的是：独立的、预构建的、标量双阶段树执行核心在 Linux/Windows 上通过功能测试，
在 Linux 上通过本次 AddressSanitizer/UBSan 测试（不是对所有内存行为的证明），
并在同一周期内完成状态上行和命令下行，同时避免失败周期的部分软件命令提交。

目前尚未证明的是：它已经接入 Humble `ControllerManager`、优于原生 chaining、适用于真实 POV 底盘、满足硬实时 WCET，
或可以仅凭 URDF 自动推断控制器语义依赖。
