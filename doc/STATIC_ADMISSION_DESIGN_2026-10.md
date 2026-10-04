# 静态准入单元与编译期调度：设计（F5 / F6）

日期：2026-10-03　分支：`humble-work`　前置：`COMPILETIME_AUDIT_2026-10.md`（审计 §3 的 F5/F6）

> **状态（2026-10-03 更新）**：§3 的描述层**已实现并验证**（`static_two_phase_admission.hpp`），
> §3 第 3 步「仍跑运行期准入、并把命中已被取代的拒绝码视为内部错误」的**可证伪内核**也已实现，
> 但形式是 **plan 等价性校验**而不是管理器入口：
> `plan_matches_description<Binding>()` 把运行期 plan 的 names 先序与 parents 边**逐项**与类型描述
> 比较（`test_static_two_phase_admission` 6/6，含两个必须失败的负向用例），并在
> `TypedForkCompositeController::build_kernel()` 里对每次激活生效（复合插件套件 9/9 + 10/10）。
> 过程中还把描述**拆成两层**：`structure_description`（结构，适用于任何编译进去的树）与
> `tree_description`（额外要求两阶段成员性）——因为把成员性要求压到结构校验上会立刻在
> **staged** 的 fork 复合控制器上编译失败。
>
> **仍未实现**：§3 的管理器入口 `set_two_phase_execution_static<Binding>()`，以及 F6 的
> 「用静态顺序替换 `controller_sorting()`」（那要动管理器排序的来源，需单独评估回归面）。
>
>
> 这份文档的其余部分是**设计**。目的：把审计里"真正的能力缺口"写清楚——
> 对**静态声明的树**，管理器的六个 `TwoPhaseAdmission` 拒绝码里哪几个可以被类型取代、
> 哪几个必须留在运行期、以及怎么**可证伪地**验证"取代"没有改变语义。

---

## 0. 一句话

> 六个拒绝码里，**三个**（会员性、边、顺序）在"整棵树编译进来"时是**类型事实**，
> 因此不是"搬到编译期检查"，而是**根本不需要检查**；
> **两个**（速率、以及指针别名导致的同实例两名）**必须留在运行期**，
> 因为它们的输入是部署（YAML 数值）与运行期指针；
> 剩下的**一个**（跨模式边）**在静态树里不存在**——整棵树要么都是两阶段成员，要么就不是一棵静态树。

---

## 1. 现状：运行期推断链

`ControllerManager::two_phase_rejections()`（`controller_manager.cpp:2919-3118`）对**任意**
控制器列表做六件事：

| # | 拒绝码 | 输入 | 判定方式 |
|---|---|---|---|
| 1 | `cross_mode_dependency` | 控制器列表 + 每个控制器的 claimed 接口 | `dynamic_cast<TwoPhaseControllerInterface*>` + 按 `'/'` 切 owner |
| 2 | `duplicate_instance` | 控制器列表 | 指针相同 |
| 3 | `unsupported_update_rate` | `update_rate` 参数 | 整数取模 |
| 4 | `cross_rate_dependency` | 参数 + claimed 接口 | 桶相等 |
| 5 | `unschedulable_order` | 控制器列表**顺序** | `parent_index < child_index` |
| 6 | （1 的镜像） | 同上 | 非成员一侧 |

**为什么必须是运行期**：输入是「插件名 + YAML + `switch_controller()` 的增删顺序」，
这三样都不是类型。**这正是审计 §4 的分界线**。

---

## 2. 静态树：把输入从"列表"换成"类型"

设整棵两阶段树**编译进一个 composite 插件**（研究线已有的 `TypedForkCompositeController` 形态），
于是"控制器列表"变成 `Binding` 的 `subtree_names`（先序），而其余输入是类型。

| # | 拒绝码 | 静态树下的地位 | 机制 |
|---|---|---|---|
| 1 | `cross_mode_dependency` | **不可表达**：`Binding` 的每个 `controller_type` 都必须是 `TwoPhaseControllerInterface` 的派生，否则 `static_assert` | `std::is_base_of_v` 递归 over `children_types` |
| 2 | `duplicate_instance` | **不可表达**：节点是互不相同的存储对象（`std::optional`/成员），同一个对象不可能在一个类型树里出现两次 | 构造即保证；不需要检查 |
| 3 | `unsupported_update_rate` | **留在运行期**（`update_rate` 是参数） | 若将来做 F7（`RateTag`）可部分前移，代价是把部署频率写进二进制 |
| 4 | `cross_rate_dependency` | **留在运行期**（同 3） | 同上 |
| 5 | `unschedulable_order` | **编译期恒真**：`compose`/`fill_spec_rows` 产生的就是先序，"父先于子"是构造性质 | `static_assert(parent_index < child_index)`，对 `compose` 树恒成立 **但对手工 `Spec` 不成立** |
| 6 | 镜像 | 同 1 | 同 1 |

**关键结论（也是要写进论文的那句）**：静态树不是"检查得更早"，而是**让四类配置错误
在类型里写不出来**；剩下两类（3、4）是部署输入，**只能**留在运行期。
因此 §2 的 F5 交付物**不是一个更快的检查器，而是一个"把运行期检查的适用范围缩小到部署相关项"的边界证明**。

---

## 3. 拟议接口（未实现）

```cpp
namespace hierarchical_control::static_two_phase
{
/// 一个静态声明、全部成员都实现两阶段接口的树。
template <typename Binding>
struct tree_description
{
  // 1) 每个节点都实现接口（编译期；取代 dynamic_cast）
  static constexpr bool every_node_is_a_member = /* 递归 over children_types */;
  static_assert(every_node_is_a_member,
    "static_two_phase: every node of a declared two-phase tree must implement "
    "TwoPhaseControllerInterface");

  // 2) 成员 = 先序节点名（编译期）
  static constexpr std::size_t member_count = Binding::subtree_size;
  static constexpr auto members = /* std::array<std::string_view, member_count> */;

  // 3) 参考边 (parent, child)（编译期；来自节点类型，不是字符串前缀）
  static constexpr std::size_t edge_count = Binding::subtree_size - 1;
  static constexpr auto edges = /* std::array<edge, edge_count> */;

  // 4) 每条边的父下标 < 子下标（编译期；`compose` 树恒真，手工 Spec 不成立）
  static constexpr bool edges_are_ordered = /* ... */;
  static_assert(edges_are_ordered,
    "static_two_phase: a declared tree must list every parent before its children");
};

/// 管理器的静态入口（新增 API；默认不存在，不改变现有行为）。
template <typename Binding>
controller_interface::return_type set_two_phase_execution_static(
  ControllerManager & manager, const Binding & binding);
}
```

`set_two_phase_execution_static` 的语义：

1. `static_assert(tree_description<Binding>::every_node_is_a_member)`；
2. 把 `members` 作为一个**有序向量**交给管理器（这同时是 F6：**不再依赖
   `controller_sorting()` 的字符串启发式**，直接给出静态顺序）；
3. 仍然调用运行期的 `two_phase_rejections()`，但**只可能命中 3/4 两类**——
   若命中 1/2/5/6 中任意一个，说明静态描述与运行期列表不一致，**这是内部错误**，
   应当 `assert`/报错而不是静默排除。

第 3 点是这套设计的**可证伪核心**：它把"静态描述 == 运行期推断"变成一个**可测的等式**，
而不是一个假设。

---

## 4. 运行期残余（必须保留，且不可被静态层替代）

| 项 | 为什么不能动 |
|---|---|
| `update_rate` / 分桶 | 值是 YAML/参数；`F7 RateTag` 只能把**部署频率**写进二进制，会破坏"同一插件多部署" |
| 指针别名 | 同一 `ControllerInterfaceBase*` 出现在两个名字下是**列表**性质，不是类型性质 |
| 控制器列表的**增删顺序** | 上游 `RTControllerListWrapper` 双缓冲；静态树只是让顺序不再需要被推断 |
| 生命周期状态 | ROS 生命周期是运行期；静态树挡不住用户单独 deactivate 一个成员 |
| `ResourceManager` 的认领检查 | pluginlib/YAML 部署唯一守卫；静态层只覆盖"编译进去的那棵树" |

---

## 5. 验证计划（可证伪）

1. **等价性测试**（最重要）：同一棵 `Binding` 树，分别用
   (a) `tree_description<Binding>::edges`（编译期）与
   (b) 管理器运行期的 `claimed_command_interfaces()` 前缀解析（运行期）
   得到边集合，断言两者**逐元素相等**；并断言静态成员集合等于管理器列表的先序子序列。
   任何一个不相等都是本设计的致命缺陷，测试必须能失败。
2. **负向编译语料**：节点类型不实现接口 → 编译失败且诊断来自本设计；
   控制用例（合法树）必须编译。
3. **回归**：`set_two_phase_execution()`（动态路径）行为不变；
   `test_two_phase_execution`（22 例）在两条路径下都通过；
   `test_hierarchy_comparison`、`test_static_controller_registry` 通过。
4. **代价**：用 `measure_binding_cost.py` + `-fsyntax-only` best-of-3 记录增量，
   与审计 §6 的 +8 ms 尺度对比。
5. **并发**：静态入口只在配置线程调用；不引入新的实时路径读取。

---

## 6. 风险与不做的部分

| 风险 | 处理 |
|---|---|
| 动管理器核心（新 API + 顺序来源） | **opt-in**：只新增 `set_two_phase_execution_static`，不改 `set_two_phase_execution` 的任何行为 |
| `controller_sorting()` 的上游怪癖是有意的 | 静态入口**不替换**它，只对静态树提供顺序；动态路径原样 |
| 静态顺序与上游顺序不一致 | §5.1 的等价性测试 + §3 的第 3 点（不一致即内部错误） |
| 编译代价 | 只新增一次 `is_base_of` 递归与两个 `constexpr` 数组；按审计 §6 尺度应在噪声内，但**必须实测** |

**不做**：多频/异步执行组；运行期动态增删树成员；把静态树做成全局单例；
整组原子提交（与 F5 正交）。

---

## 7. 与其它条目的关系

- **F1（已做）**是 F5 的前置：静态准入单元要复用的正是 manifest 不变式；
  没有 F1，`StaticTree` 的"一根/唯一名/每 role 一端口"就没有守卫。
- **F6** 是 F5 的自然产物（静态顺序），**不是**独立工作项。
- **F4（已做一部分）**把"哪些节点是叶子"变成编译期事实；F5 需要同一类"类型导出结构信息"的手法。
- **F7/F8/F11** 与 F5 正交，且 F7 有"把部署写进二进制"的副作用，**不应**与 F5 捆绑宣传。

---

## 8. 决定（2026-10-03）：§3 的管理器入口与 F6 的"替换排序" **不做**

用户的目标是「**YAML 加一行就启用双向调度**」。据此逐条判定，两条都**不做**，理由记录在此，
避免以后反复讨论。

### 决定 1：不做 `set_two_phase_execution_static<Binding>()`

| 维度 | 事实 |
|---|---|
| 它是什么 | 一个 **C++ 模板 API**，不是配置项 |
| 用它要做什么 | 把控制器编进二进制 → 用 `StaticControllerRegistry` 注册 → 把树声明成 binding 类型 → 调用模板 |
| 对比 | 现有路径是 `two_phase_execution: true` **一行 YAML**（已验证：demo 启动日志出现 `Two-phase execution requested by parameter: enabled`） |
| 它省掉了什么 | 配置期的 O(n²) 字符串比较；**不在任何每周期路径上** |
| 收益/代价 | 用数量级更高的复杂度，去省一个非实时路径上的字符串比较 |

**结论：不做。** 它服务的只是论文的"编译期边界"叙事，而那个叙事**已经**由描述层 + plan 等价性校验
+ 编译代价实测支撑（`COMPILETIME_AUDIT_2026-10.md` §3.2/§3.3），不需要再造一条用户路径。
编译期能力保留为**校验**（每次激活生效），而不是变成新的使用方式。

### 决定 2：不替换 `controller_sorting()`（F6 的"替换"部分）

| 维度 | 事实 |
|---|---|
| 生效范围 | **所有**配置，不只两趟；替换它是全局行为变更，与"不大幅改动原有逻辑"直接冲突 |
| 是否必要 | **不必要**：两趟路径**已经校验**拿到的顺序，会把某条边走反就**拒绝整个模式**（`unschedulable_order`，两端都报告）——错顺序不会被静默执行 |
| 上游怪癖 | "无命令接口的 chainable 排在父之前"是**有意的**，且历史上有实测回归记录 |
| 触发面 | 很窄：真实级联里叶子要驱动硬件，本身有命令接口，顺序正常；怪癖主要咬合成用例 |
| 静态树 | 编译进来的树的顺序**已经**是编译期事实并被 plan 校验，那条路不需要动排序 |

**结论：不做。** 替代做法是**把拒绝信息变成可执行的修复指引**：
`unschedulable_order` 现在直接说"给那个控制器一个命令接口（级联叶子通常驱动硬件，本来就有），
或者把它排在父之后加载"；用户文档新增「启用被拒怎么办」对照表（见 `feature/two-phase-manager`）。

### 这条界线对论文的意义

F5/F6 的"剩余部分"被**有意**停在"校验"而不是"替换"，这不是没做完，而是一个结论：

> 只要控制器集合是**运行期加载 + YAML 配置**的，"顺序"就不是类型事实，因此
> **不能**用静态顺序替换管理器的排序；能做的只是**校验并在不一致时拒绝**。
> 反过来，当整棵树编译进来时，"顺序"变成类型事实，plan 可以被逐项校验——
> 而那时管理器排序已经不参与其中。

这正是审计 §4 那条分界线（"由类型唯一决定"vs"由部署唯一决定"）在 F6 上的具体体现。
