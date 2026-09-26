# 编译期控制器与静态执行图方案

日期：2026-09-26

本文总结当前 `humble-work` 的最新进展，并定义下一阶段“编译期控制器初始化”的可行实现边界。

## 1. 最新代码核验

远端最新提交：`f2423243`。

本轮新增的主要证据：

- `controller_manager` 的真实 TSan 脚本：`controller_manager/test/run_tsan_real_manager.sh`；
- `RTControllerListWrapper` 的发布索引、`switch_params_` 握手字段改为原子访问；
- manager 自身 TSan data race 从修改前 4 条降为 0 条；
- lock-order 报告来自未插桩的 rclcpp/lifecycle/hardware/FastRTPS 依赖，不能归因于本项目；
- typed 分叉树已经放入普通 composite plugin，而不再只存在于库测试；
- 两阶段切换在应用前检查 prospective active set，拒绝后不激活、不发布新控制器列表。

当前仍未完成：统一 generation 发布协议、依赖库 TSan、多频/异步/动态拓扑、Gazebo 真值轨迹指标，以及“纯 StagedControllerInterface 节点不继承 ControllerInterfaceBase”的绑定形式。

## 2. 编译期目标的正确表述

不要把目标定义为：

> 在 C++ 静态初始化期间创建 ROS controller，并完成 URDF、参数和硬件接口绑定。

这个目标与 ROS 2 Humble 的生命周期和资源模型冲突。静态初始化发生时通常还没有 ROS context、controller manager node、robot description、ResourceManager、硬件导出的 interfaces、参数覆盖和 lifecycle state。

应该定义为：

> 在编译期生成控制器类型、树形拓扑、typed ports、容量和执行计划；在 `configure` 阶段解析 URDF/参数和硬件接口，在 `activate` 阶段事务化获得资源；激活后的实时循环只使用固定对象、固定索引和预构建的双向执行计划。

这保留 FineMote 的元编程优势，同时不绕过 ROS 资源所有权和生命周期。

## 3. 四层结构

### 3.1 编译期描述层

编译期固定：

- controller 节点类型；
- 节点数量和父子拓扑；
- `State`、`Reference`、`Actuator`、`ForChildren`、`ChildState` typed ports；
- 端口名称、物理量纲和连接方向；
- 单根、无环、实例唯一性；
- two-phase/legacy 边界约束；
- 固定缓冲区大小和状态/命令阶段的执行顺序。

当前 `hierarchical_control` 已支持 typed branching builder，并有七节点分叉验收树。静态检查必须继续从用户实际调用的 `compose/create_library_group` 入口触发，而不是要求调用者额外记得调用检查函数。

### 3.2 固定对象存储层

可以把控制器对象放进固定的 `tuple`、静态数组或显式拥有的 arena，避免控制周期动态分配：

```cpp
using RobotGraph = StaticComposite<Chassis, LeftModule, RightModule, MotorA, MotorB>;
```

对象应由一个明确的 owner 创建，不能依赖跨翻译单元的全局构造顺序。可以使用函数内静态对象或 manager-owned storage，但对象的 ROS 运行时状态不能在全局构造函数中访问。

固定对象存储只解决对象地址和构造顺序，不代表 ROS 生命周期已经完成。

### 3.3 configure/activate 绑定层

这些步骤必须保留在 ROS 生命周期中：

1. 创建/关联 node 和参数接口；
2. 接收并解析 `robot_description`；
3. 检查 URDF joint/link 与编译期 manifest 的一致性；
4. 解析 `<ros2_control>` 中 state/command interface；
5. 把逻辑端口名解析为 ResourceManager 的稳定索引；
6. 在 `on_activate()` 获取 loaned interfaces；
7. 检查 interface 数量、名称、类型、单位和资源所有权；
8. 全部成功后发布可供实时线程使用的不可变计划。

如果 URDF 或硬件与编译期 manifest 不一致，应该在 `configure` 失败，而不是运行时修改静态控制树。

### 3.4 实时执行层

激活后目标是：

- 无 pluginlib 加载；
- 无拓扑构建和字符串查找；
- 无动态内存分配；
- 无动态排序；
- 只使用预解析的 interface index/pointer；
- 状态后序、命令前序；
- scratch 命令全部验证后再提交；
- manager 与 staged group 使用不可变快照发布。

当前真实 manager TSan 结果证明 manager 自身的两阶段标志、成员快照和 controller-list 发布已无报告到的数据竞争；它不覆盖 rclcpp、lifecycle、hardware_interface 和 FastRTPS 内部。

## 4. 推荐 API 设计

### 4.1 静态 manifest

编译期 builder 应能导出一个 manifest：

```cpp
struct StaticManifest
{
  std::span<const ControllerDescriptor> controllers;
  std::span<const PortDescriptor> ports;
  std::span<const EdgeDescriptor> edges;
  std::span<const InterfaceRequirement> interfaces;
};
```

manifest 是编译期生成的描述，不拥有 ROS loaned interfaces，也不包含运行时 node 指针。

### 4.2 静态 controller provider

建议区分静态控制器和 pluginlib 控制器：

```cpp
template<class ControllerT>
struct StaticControllerProvider
{
  static constexpr auto descriptor = ControllerT::descriptor();
  ControllerT instance;
};
```

manager 可以同时支持：

- 现有 pluginlib 动态实例；
- 编译进二进制的静态 provider。

两者进入同一个 lifecycle adapter 和 admission checker，避免静态路径拥有一套不受检查的特殊规则。

### 4.3 资源绑定对象

编译期名称最终应在 configure 阶段变成索引：

```cpp
struct ResolvedInterface
{
  std::uint32_t resource_index;
  std::uint32_t interface_index;
};
```

实时回调只读取这些索引或稳定句柄。URDF 的作用是校验机械资源和接口存在，不是推断 controller 语义依赖。

## 5. 为什么不能把完整 controller 做成普通全局变量

不建议：

```cpp
GlobalController controller;  // constructor touches ROS, params or hardware
```

风险包括：

- ROS context 和 node 尚未初始化；
- 参数 override 尚未生效；
- robot description 尚不可用；
- ResourceManager 尚未加载或导出 interfaces；
- lifecycle configure/activate 失败无法通过构造函数表达；
- 多 manager/多机器人实例之间共享状态；
- 插件卸载和静态析构顺序不确定；
- 测试无法重复创建和清理独立实例。

可接受的做法是把**不依赖 ROS 的静态描述和固定算法对象**做成编译期/固定存储，把 node、参数、interface loan 和生命周期绑定留给 configure/activate。

## 6. 与当前双向调度结合

建议流水线为：

```text
compile-time typed graph
    -> configure: URDF + ros2_control interface resolution
    -> activate: resource transaction and immutable plan publication
    -> read snapshot
    -> state pass: children -> parents
    -> command pass: parents -> children
    -> validate and commit
    -> hardware.write
```

编译期可以检查拓扑和端口类型；configure 可以检查实际 URDF/硬件；activate 可以检查当前资源代际和 loan 成功；实时路径只执行计划。

两阶段切换建议继续使用“预检 + 提交”模型。当前代码已在应用切换前检查 prospective active set，但还没有统一 generation 覆盖 controller list、two-phase membership、staged group 和 resource interfaces。后续若支持运行中配置，应增加统一 generation；首版可以明确限制为控制循环停止时配置。

## 7. 必须补的验收测试

1. 编译期 manifest 与 URDF 缺 joint/interface 时，`configure` 失败且不发布运行计划；
2. `activate` 中任一资源 loan 失败时，之前已准备的节点全部撤销，不发布部分计划；
3. 两个 manager 实例使用同一静态 manifest 时，运行时状态和接口绑定完全隔离；
4. 动态 pluginlib controller 与静态 controller 混合时，admission、生命周期和执行顺序一致；
5. 运行中尝试改变拓扑或静态 manifest 时被拒绝；
6. 编译期七节点分叉树在普通 composite plugin 中完成同周期数值传播；
7. 使用 TSan 检查 manager 自身发布协议；当前脚本已覆盖 manager 包，依赖库仍不在插桩范围；
8. `update()` 稳态和切换后周期分别用全局 `operator new` 计数，确认没有实时分配。

## 8. 当前结论

当前代码已经能够证明：

- typed branching topology 可以在编译期描述和检查；
- 普通 composite plugin 可以承载该 typed tree；
- manager 双向执行路径支持同周期状态上行和命令下行；
- manager 自身的已测发布字段在真实 TSan 中无 data race。

当前不能声称：

- ROS controller 在 C++ 静态初始化阶段完成完整可运行初始化；
- 编译期 manifest 自动替代 configure/activate 的 URDF 和硬件绑定；
- 依赖库内部无 TSan 竞态；
- 运行中任意拓扑修改安全；
- 硬件总线物理提交原子。

下一步建议优先实现一个“静态 manifest + configure 解析 + activate 资源事务”的最小 Humble 原型，并与当前 typed composite plugin 共用 `hierarchical_control` 内核。
