# 接线成本量化（Gate B 初步）

日期：2026-09-20。范围：仅 ROS 2 Humble，本仓库。
测量代码：`controller_manager/test/test_hierarchy_comparison.cpp`
（`adding_a_leaf_is_configuration_only`）与 `test_composite_controller/`。

本文回答交接文档 Gate B 的问题：相比手写 composite，阶段化执行组是否真的减少了
“新增一个模块”时必须手写的集成/调度代码？

## 一、测量场景

拓扑：一个 root 扇出到 N 个叶（POV 底盘形状）。基线 N=2，变更 N=3。
被测变更定义为：

```cpp
// 基线
std::vector<LeafSpec> TwoLeafFork() {
  return {{kLeaf,  "joint2/position", "joint2/velocity",         kOffsetA},
          {kLeafB, "joint3/position", "joint3/velocity",         kOffsetB}};
}
// 变更：只增加一个 LeafSpec 条目
std::vector<LeafSpec> ThreeLeafFork() {
  return {{kLeaf,  "joint2/position", "joint2/velocity",         kOffsetA},
          {kLeafB, "joint3/position", "joint3/velocity",         kOffsetB},
          {kLeafC, "",               "joint2/max_acceleration",  kOffsetC}};  // <-- 唯一改动
}
```

`SetupStagedForkN()` 与 `SetupChainedForkN()` 对两种拓扑是**同一份代码**：
声明端口、claim 子 reference、加入执行组成员、激活都由 `LeafSpec` 列表循环驱动。

测试 `adding_a_leaf_is_configuration_only` 对 `{2 叶, 3 叶} × {staged, chained}` 四种组合
运行 6 个周期，断言每个叶的命令都等于解析值，四种组合全部通过。

## 二、结果一：新增一个叶的改动量

| 方案 | 新增一个叶需要改什么 | 实测改动 |
|---|---|---|
| 阶段化执行组 | 1 个 `LeafSpec` 条目；执行组成员列表由循环生成 | **1 行配置，0 行控制器代码，0 行调度/缓冲代码** |
| 原生 chaining | 1 个 `LeafSpec` 条目；父节点的 claim 列表与 bias 由循环/求和生成 | **1 行配置，0 行控制器代码** |
| 手写 composite（现状） | 该插件只实现固定的三级链，无法表达 fork；必须新写插件或重写内部调度 | 不适用（需要改控制器代码） |

说明：两种方案都**复用同一个叶控制器类**（`TestStagedController`），
这正是“复合 vs 内部调度”的差别所在。

## 三、结果二：可复用机制与固定拓扑插件的代码量

统计口径：非空、非注释行（`code lines`），由脚本统计。

| 项目 | code lines |
|---|---|
| 库 `hierarchical_control`：`staged_controller_interface.hpp`（数据契约 + 端口 + 回调） | 129 |
| 库 `hierarchical_control`：`staged_execution_group.hpp`（拓扑解析 + 阶段调度 + 校验 + 提交） | 447 |
| — 其中 `run()/run_ns()`：两阶段执行 + 帧校验 + 提交 | 176 |
| — 其中构造函数与 `Spec`：拓扑解析、唯一写者/单根/环校验、存储规划 | 约 177 |
| **内核合计（独立包，写一次，两种宿主共享）** | **576** |
| 手写 composite 插件合计（仅 1 个固定拓扑） | 181 |
| — 其中 `update()` 函数体（算法 + 本地缓冲/校验/提交） | 28 |
| 应用侧控制器（staged 与 chained 共用，可复用） | 443 |

### 包结构（已抽取）

```text
hierarchical_control/                 # 独立 header-only 包，namespace hierarchical_control
  include/hierarchical_control/
    hierarchy.hpp                     # 计划生成与校验（纯 C++）
    staged_controller_interface.hpp   # 控制器实现的可选双阶段接口 + 数据契约
    staged_execution_group.hpp        # 内核：两阶段调度 + 帧校验 + 整组提交（含库模式）
controller_manager/                   # 可选薄适配层
  include/controller_manager/hierarchy.hpp               # 兼容 shim（using 转发）
  include/controller_manager/staged_execution_group.hpp  # 兼容 shim（using 转发）
```

`controller_manager` 依赖 `hierarchical_control`；反向不依赖，因此没有包依赖环。
`controller_interface/staged_controller_interface.hpp` 已删除（移到库中，不留 shim，
否则 `controller_interface` 与库之间会形成环）。

## 四、结果三：保证覆盖对比

| 保证 | 阶段化执行组 | 现状 composite |
|---|---|---|
| 每周期来源信息（cycle / sample_ns / valid / fault） | 有 | 无 |
| 复合状态采样时间强制取最旧输入 | 有 | 无 |
| 周期一致性 + 采样年龄校验 | 有 | 无 |
| 整树命令提交（多节点） | 有 | 单节点天然成立 |
| 配置期拓扑校验（重复写者/多根/环/缺 sink） | 有 | 不适用 |
| 诊断（失败节点索引、周期、故障码） | 有 | 部分 |
| 同一控制器类可跨拓扑复用 | 有 | 无 |

## 五、结果四：通用 composite 库基线（决定性）

为了验证“这套机制能不能放进库、由单个插件托管、完全不动 `ControllerManager`”，
实现了 `test_composite_library/generic_composite_controller`：

- 一个普通 `ControllerInterface` 插件，数据驱动声明节点：

```cpp
struct CompositeNodeSpec { name, parent, state_interfaces, command_interfaces, factor, offset };
```

- 它把整棵树交给**同一个内核**的库模式入口
  `StagedExecutionGroup::create_library(spec, max_age_ns)`；
- 不导出 chainable reference 接口、不 claim 任何子控制器接口、不改 manager；
- 首次 `update()` 惰性绑定硬件接口并建内核（一次性、非实时）。

测试 `library_host_matches_staged_group`：对 2 叶和 3 叶 fork，

- 库托管与阶段化执行组**每个周期输出逐位相同**，且等于解析值；
- 惰性建好之后，`update()` 路径 **100 次调用 0 次分配**；
- 新增一个叶同样是**一个 `CompositeNodeSpec` 条目**。

代码量（非空非注释行）：

| 项目 | code lines |
|---|---|
| 库托管插件（hpp + cpp） | 371 |
| 复用的内核 + 接口（`staged_execution_group.hpp` + `staged_controller_interface.hpp`） | 576 |

也就是说：**同一份内核可以有两种宿主**——由 `ControllerManager` 驱动（N 个独立控制器插件），
或链接进一个插件内部自行驱动（库模式）。两者提供相同的阶段调度、来源校验和整组提交。

## 六、结论（Gate B 已可下结论）

1. **“新增一个模块只需配置”不是 manager 方案独有的能力**：通用 composite 库以同样的
   数据驱动方式做到了，而且复用的是同一份内核，不要求改 `ControllerManager`。
   因此交接文档 Gate B 里“减少手写集成代码”这一条**不足以支撑修改 manager**。
2. **性能上 manager 方案也不占优**（约为 composite 的 2 倍，见
   `HIERARCHY_FAIR_COMPARISON.md`）。
3. **两种宿主仍然有真实差别**，但这些差别不是“接线成本”：
   - **组件可复用性与生命周期**：manager 宿主里每个节点是独立可加载、可单独激活的控制器；
     库宿主把整棵树焊死在一个插件里，节点不能单独运行。
   - **与原生接口的兼容性**：manager 宿主使用 Humble 原生 reference interface，
     因此可以和非 staged 的普通 chainable 控制器混用；库宿主只 claim 硬件接口。
   - **部署成本**：库宿主不需要 patch `controller_manager`；manager 宿主需要维护本仓库的改动。
4. 按阶段 E 停止门槛，当前证据**不支持**继续扩大 `ControllerManager` 修改。
   已经把内核抽成独立包 `hierarchical_control`（header-only），
   `controller_manager` 侧只保留薄适配（公开 API + `update()` 跳过成员 + 切换后刷新活跃标志）
   和两个兼容 shim。这是本阶段建议的交付形态。

## 七、尚存的方法学问题（必须先解决再写进论文）

1. 只测了**一种**拓扑变化（加叶）。还应测：插入中间层、把一个叶换成一个子树、
   共享模块（两个父消费同一子状态）。
2. `code lines` 只统计行数，没有区分难度或出错概率；论文里应补充“必须手写且容易写错的
   部分”（阶段顺序、缓冲别名、提交原子性）作为定性维度。
3. 应用侧控制器（443 行）在两种方案里其实是同一份代码，不应算进框架优势；
   表格已分开列出，写作时不要混算。
4. 本测量是开发者视角的静态代码量，不是运行时成本；两者不能互相替代。
5. 库宿主的惰性建内核发生在第一次 `update()`，严格说仍有一次性实时路径分配；
   论文中应说明，或改为在 `on_activate` 后显式触发。

