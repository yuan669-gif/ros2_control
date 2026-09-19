# 同周期双向树：第一阶段实验

日期：2026-09-19。仅面向本仓库 Humble。本文记录新实验，不把 v1 executor 当作已接入的实现。

## 已实现的边界

`controller_manager/include/controller_manager/cycle_tree.hpp` 是独立 C++17 研究核心。
输入是已解析的组件实例和双向端口连接，不要求用户填写 parent；内部根据连接构建树。
这里的 Connection 表示已确认的命令生产者/消费者以及反向状态连接，尚未从 ROS 接口自动提取。
首版为同频同步单树、标量状态/reference、每叶一个命令、整树一个提交组。

状态后序计算，命令前序计算；每周期失效旧状态，检查生产周期和最旧样本时间；
叶输出先写独立 scratch，整树成功后才复制到 committed buffer。
构造时分配存储和计算计划，run 内不构图、不按字符串查找、不扩容。
回调使用固定长度输出视图；回调自身必须遵守 noexcept、无分配和不访问真实硬件句柄的约定。
实例由调用者管理，必须比执行器存活更久；run 和读取提交缓冲必须在同一线程或由外部同步。
这是单线程读者边界的一致提交，不是多线程原子发布，也不是物理总线同步。

时间戳要求非负、使用同一单调时钟。复合状态保留最旧输入采样时间，不允许重新盖章伪装新鲜。
框架无法验证插件是否诚实地使用输入或驱动是否报告真实硬件时间戳。
失败返回阶段、周期和失败节点索引；已提交命令/周期不变。此行为仅便于检验“不部分提交”，
不是“继续发送旧命令”的硬件安全策略。接硬件时必须先检查结果并调用指定故障动作。
PID 积分器、估计器等内部状态不回滚，失败后恢复策略仍需应用定义。

未实现：ROS 插件适配、ControllerManager 主循环接入、资源 claim、URDF 校验、生命周期、
端口类型/单位的 TMP 检查、多频、异步、运行超时检测、硬件 write、硬件故障策略。
v1 `hierarchical_controller_executor.hpp` 尚未修复，不能用于硬件，也不是本实验的调用路径。

## 实验算法与判据

三层：root -> module -> {a,b}。每个节点 S = 2 * sum(inputs)，每个节点 C = reference - S。
叶命令为 C，内部节点把 C 作为每个孩子的 reference。
这是验证调度契约的合成算法，不是实际底盘运动学、PID 或 ROS 性能基线。

手写 composite 对照显式计算 Sa、Sb、Sm、Sr，然后计算根、模块、叶目标。
连续 1000 次改变输入，执行核心应与该对照逐项精确相等；对照同样具备同周期能力。
本实验不声称相对 composite 更快，也不模拟原生 chaining 来宣称 ROS 测量结果。

附加检查：两层树；每个节点分别状态失败和命令失败；状态失败后零命令调用；
失败后提交周期和完整输出保持不变；旧周期派生状态；未填写叶输出；过期和未来样本；
无效样本、空快照、零周期、NaN reference；恢复；改变实例/连接声明顺序；
缺失端点、重复边、环、多根、重复实例和空实例。

Windows 本地 GCC 已以 C++17、-O2、-Wall -Wextra -Wpedantic -Werror 编译运行通过。
另以 GCC 8.1.0、CMake/Ninja、Release 构建完成 CTest：1/1 通过。
这不等于 colcon 或 ROS/Gazebo 集成通过。运行日志中的 PASS 仅针对上述断言。

## Ubuntu 22.04：请协助执行

把当前修改后的整个 ros2_control 目录复制到虚拟机；以下命令在该目录执行。
无需先装 ROS，即可执行第一组实验。缺依赖时安装：

```bash
sudo apt update
sudo apt install build-essential cmake
```

编译和测试：

```bash
cmake -S research/cycle_tree -B /tmp/finemote-cycle-tree -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/finemote-cycle-tree -j2
ctest --test-dir /tmp/finemote-cycle-tree --output-on-failure -V
```

内存/未定义行为检查（独立构建目录）：

```bash
cmake -S research/cycle_tree -B /tmp/finemote-cycle-tree-asan \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
cmake --build /tmp/finemote-cycle-tree-asan -j2
ctest --test-dir /tmp/finemote-cycle-tree-asan --output-on-failure -V
```

预期：两组 CTest 各 1/1 通过；无 sanitizer 报告。
请回传两组输出，以及 `uname -a`、`g++ --version`、`ls /opt/ros` 的结果。
若 /opt/ros 不存在，也只需回传该结果，暂不要求安装完整 ROS 环境。
从干净目录复现请使用另一个新的 /tmp 构建目录，不复用其它实验的 CMake 缓存。

## 后续关卡

1. 用真实组件算法确认需要子组件派生状态的依赖，而不是为双阶段虚构需求。
2. 把 Humble reference claim 映射到命令边，显式声明/校验派生状态边；统一生成计划。
   不能仅根据机械祖先推导状态依赖，parent 不得覆盖端口约束。
3. 以 opt-in 执行组接入 manager；组内组件不得再被 legacy update 重复调用。
   ResourceManager 仍拥有硬件资源，生命周期/频率/命令隔离通过 mock hardware 验证。
4. 相同算法比较原生 chaining、估计/命令拆分、手写 composite 和新组执行器。
   虚拟机可做正确性与初步开销实验，不用于宣称硬实时保证。
5. 单独消融有效性、提交缓冲和静态绑定；只有集成成本/保证存在收益才继续扩大 manager 修改。

## FineMote 证据更新

本次用户提供的 DeviceBase.hpp 确实有正向 Update、反向 Handle；之前交接文档关于仅有 Handle
的观察已不适用于这份源码。POV_Chassis 的 ForwardKinematics/odom 仍在 Handle，不能据此
宣称任意多层派生状态已全部自底向上更新。新机制要求参与组件显式拆分相应阶段。
