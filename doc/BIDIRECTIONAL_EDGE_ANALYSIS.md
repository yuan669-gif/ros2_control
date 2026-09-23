# 双向依赖边在上游 ros2_control 排序下的行为（决定性实验）

日期：2026-09-21
上游版本：`ros-controls/ros2_control` **master HEAD `b0c14b5`**（2026-09-17，"Robustify CM tests on CI (#3542)"）
本地对照：Humble 副本（`controller_manager/src/controller_manager.cpp`）

本文回答一个问题，它决定本课题能否立住：

> 当同一对父子**同时**存在 reference 边（父产参考、子消费）和 state 边（子产状态、父消费）时，
> 上游的控制器排序会怎么做？

---

## 1. 结论

**上游不会报错，也不会检测冲突；它按"先插入者胜"的启发式给出一个顺序，
使其中一条边退化为一周期延迟——而且哪条边陈旧取决于控制器的加载顺序。**

因此：

- 单向场景（估计器→控制器，或控制器→执行器）上游处理正确；
- **双向场景未被覆盖，且在"每个控制器一个 `update()`"的模型下无法覆盖**：
  父要读子本周期状态要求"子先跑"，子要用父本周期参考要求"父先跑"，两者矛盾；
- 上游的行为不仅是一周期延迟，而且是**载荷顺序相关、无诊断**的：
  同一拓扑换个 spawn/YAML 顺序，陈旧的方向就变了。在三级链上会出现**逐边混合**的陈旧模式。

这就是本课题重新定位后的核心命题，现在有了上游代码级证据。

---

## 2. 上游代码事实

`controller_manager/src/controller_manager.cpp`（master `b0c14b5`）：

### 2.1 图的构造：`build_controllers_topology_info()`

```cpp
for (const auto & cmd_itf : cmd_itfs) {
  if (is_interface_a_chained_interface(cmd_itf, controllers, ctrl_it)) {
    add_item(controller_chain_spec_[controller.info.name].following_controllers, ctrl_it->info.name);
    add_item(controller_chain_spec_[ctrl_it->info.name].preceding_controllers, controller.info.name);
    add_item(controller_chained_reference_interfaces_cache_[ctrl_it->info.name], controller.info.name);
  }
}
// This is needed when we start exporting the state interfaces from the controllers
for (const auto & state_itf : state_itfs) {
  if (is_interface_a_chained_interface(state_itf, controllers, ctrl_it)) {
    add_item(controller_chain_spec_[controller.info.name].preceding_controllers, ctrl_it->info.name);
    add_item(controller_chain_spec_[ctrl_it->info.name].following_controllers, controller.info.name);
    add_item(controller_chained_state_interfaces_cache_[ctrl_it->info.name], controller.info.name);
  }
}
```

即：

| 边 | 效果 |
|---|---|
| 本控制器 claim `<C>/ref` | 本控制器 `following += C`；`C.preceding += 本控制器`（参考生产者先跑） |
| 本控制器 claim `<C>/state` | 本控制器 `preceding += C`；`C.following += 本控制器`（状态生产者先跑） |

**当两条边同时存在于同一对 (A,B) 时**，得到
`A.following = {B}`、`A.preceding = {B}`、`B.following = {A}`、`B.preceding = {A}`——
同一个控制器同时出现在对方的 two sides，即顺序图里出现 2-环。

### 2.2 顺序构造：`update_list_with_controller_chain()`

它不是拓扑排序，而是**递归插入启发式**，关键在第一行：

```cpp
if (new_ctrl_it != ordered_controllers_names_.end()) {
  return;                       // 已经在列表里 → 直接返回，后续约束被忽略
}
...
// 依 already-placed 的 following 取最早位置，preceding 取最晚位置，然后 insert
```

也就是说：**第一次插入决定位置，之后所有冲突约束被静默丢弃**。

---

## 3. 移植验证

`research/topology_analysis/upstream_order_sim.py` 是上述两个函数的逐条移植
（不是跑真实代码；限制见第 6 节）。运行结果：

```text
### 两个控制器，双向边，加载顺序 ['A','B']
  following_controllers : {'A': ['B'], 'B': ['A']}
  preceding_controllers : {'A': ['B'], 'B': ['A']}
  resulting run order   : ['A', 'B']
  edge A->B [reference] : same-cycle (fresh)
  edge B->A [state]     : ONE-CYCLE DELAY

### 两个控制器，双向边，加载顺序 ['B','A']
  resulting run order   : ['B', 'A']
  edge A->B [reference] : ONE-CYCLE DELAY
  edge B->A [state]     : same-cycle (fresh)

### 三级链，每级都是双向边，加载顺序 ['r','m','l']
  resulting run order   : ['r', 'm', 'l']
  edge r->m [reference] : same-cycle (fresh)
  edge m->r [state]     : ONE-CYCLE DELAY
  edge m->l [reference] : same-cycle (fresh)
  edge l->m [state]     : ONE-CYCLE DELAY

### 三级链，加载顺序 ['m','r','l']（混合）
  resulting run order   : ['m', 'r', 'l']
  edge r->m [reference] : ONE-CYCLE DELAY
  edge m->r [state]     : same-cycle (fresh)
  edge m->l [reference] : same-cycle (fresh)
  edge l->m [state]     : ONE-CYCLE DELAY
```

**三条可写进论文的结论：**

1. 双向边不会报错——它在两条矛盾约束中静默选一条；
2. 选哪一条由**控制器加载顺序**决定，不由拓扑决定；
3. 在三级链上会出现**逐边混合**（有的边新鲜、有的边陈旧），
   而不是一个可预测的全局延迟。这对"确定性"是实质性问题。

---

## 4. 为什么单入口模型下无法修复

设父子为 P、C，同一周期内：

- 状态方向：P 要读 C 本周期算出的状态 ⇒ `order(C) < order(P)`；
- 参考方向：C 要用 P 本周期产生的参考 ⇒ `order(P) < order(C)`。

两式矛盾。只要每个控制器只有一个 `update()` 入口（Humble/Jazzy 均是），
**同一周期内两条边不可能同时新鲜**。上游的 state chaining 解决的是
"估计器→控制器"这类**只有状态边**的场景；reference chaining 解决"控制器→执行器"这类
**只有参考边**的场景；双向场景没有解。

显式两阶段（本项目的 `StagedExecutionGroup`）给出解：把状态与命令拆成两个入口，
`state` 后序（子先）满足状态方向，`command` 前序（父先）满足参考方向，两者都在同一周期内完成。
这正是 `doc/PROJECT_REPORT_2026-09-21.md` 第 3 节的技术方案。

---

## 5. 对课题的意义

- **主命题成立**：不是因为"我们做了层次化调度"（上游已做），而是因为
  **"双向同周期在单入口下不可满足，且上游的实现给出载荷顺序相关的静默降级"**。
  这是一个可证伪、有上游代码证据、有可测量后果的命题。
- **代价一侧已有数据**：我们的方案不更快（≈2× 手写 composite），
  省集成代码也不是它独有（库宿主同价）。这些负结果界定了代价，是答案的一部分。
- **收益一侧**：同周期双向传播（相位滞后 = 0）+ 上游不具备的可靠性契约
  （每周期有效性、最旧采样强制、整组提交不部分提交；后者已有 fork 故障实验证据）。

---

## 6. 方法学限制（必须先解决再写进论文）

1. **这是逻辑移植，不是运行上游代码。** 必须真正在 **Jazzy/Rolling** 上运行一个双向
   控制器对，观察实际顺序与控制器内部可见的数据新鲜度。移植只用于形成假设。
2. **迭代器失效风险**：真实代码把迭代器传入递归函数，而函数内部会 `insert` 使迭代器失效。
   移植用下标建模，可能掩盖真实行为（这本身可能是上游的潜在缺陷）。需要用真实代码验证。
3. **`is_interface_a_chained_interface` 只看前缀是否匹配某个控制器名**，不校验接口是否真的被导出。
   真实场景中未导出会被 `ResourceManager` 拒绝；移植未建模这一层。
4. **上游在演进**：issue #1123 于 2026-04-11 关闭，master HEAD 为 `b0c14b5`（2026-09-17）。
   论文引用必须锁定 commit，并检查后续是否有人处理双向场景。
5. 一级延迟对控制的实际影响取决于控制周期与算法；把"一周期延迟"换算成**相位滞后/跟踪误差**
   才能成为性能论证，这需要专门实验。

---

## 7. 下一步实验设计（在新窗口中执行）

**目标**：把本文的逻辑结论变成上游真实代码上的可执行证据 + 可测控制后果。

1. 在 Jazzy/Rolling 上写两个最小 chainable 控制器 A、B：
   - A 的 `command_interface_configuration()` 含 `B/ref`；
   - A 的 `state_interface_configuration()` 含 `B/state`；
   - B 导出 `ref` reference interface 与 `state` state interface。
2. 用两种加载顺序（A 先 / B 先）启动，读取实际执行顺序（`list_controllers` 的
   `chain_connections` 或 controller_manager 日志），验证：
   顺序是否随加载顺序翻转、是否有任何 warning/error。
3. 让 B 的 `state` 依赖其内部状态（例如一阶滤波/积分），A 把 `B/state` 用于控制。
   在两种加载顺序下测量 A 看到的 `B/state` 的**周期数滞后**与由此产生的**跟踪误差/相位滞后**。
4. 对照组：用本项目的两阶段执行组跑同一算法，证明两条边都是同周期、误差为 0。
5. 反例防护：确认上游是否在**同一控制器对内**同时允许 reference 与 state 边；
   若上游文档禁止该组合，则命题要改为"该组合被禁止，从而限制了级联表达力"——
   这仍是有效结论，但叙事要改。

---

## 8. 复现

```bash
# 上游代码（用于核对行号）
curl -sL -o log/upstream/cm_master.cpp \
  https://raw.githubusercontent.com/ros-controls/ros2_control/master/controller_manager/src/controller_manager.cpp

# 移植模拟器
python3 research/topology_analysis/upstream_order_sim.py
```

移植来源：`build_controllers_topology_info()` 与 `update_list_with_controller_chain()`
（master `b0c14b5`，`controller_manager/src/controller_manager.cpp`）。

---

## 9. Humble 真实代码验证（2026-09-21 完成）

前面的第 2–3 节是对 **master 的移植**。本节是在**本机 Humble 真实代码**上的验证，
测试：`controller_manager/test/test_upstream_ordering.cpp`（2 个用例，均通过）。

构造一个双向对：

- parent：`command_interface_configuration()` = `{"ord_child/ref"}`（reference 边）、
  `state_interface_configuration()` = `{"ord_child/state"}`（state 边）；
- child：导出 `ref` reference interface，并驱动一个硬件命令接口 `joint2/velocity`。

两种注册顺序（parent 先 / child 先），配置完成后读取 `get_loaded_controllers()` 的顺序。

### 结果

| 注册顺序 | 实际执行顺序 |
|---|---|
| parent 先 | `[parent, child]` |
| child 先 | `[parent, child]` |

**两种顺序都得到"参考生产者在前"。** 因此返回顺序与加载顺序无关，
**state 边永远是陈旧的那一条**——这正是 `FORMAL_MODEL.md` 的**推论 1** 在真实代码上的验证，
不依赖任何移植。

### 机制：状态边根本没有参与排序（2026-09-22 插桩实测）

上面只给出了结果。为确认**机制**，在 `controller_sorting` 比较器入口临时插桩，
打印每次调用的 `ctrl_a`/`ctrl_b` 及其命令/状态接口：

```text
DBG cmp a=ord_parent(state=2) b=ord_child | a.cmd=[ord_child/ref,] a.state=[ord_child/state,]
```

**每个用例中比较器只被调用一次**，说明它在**第一个检查**处就返回了 `true`：
`get_following_controller_names(ord_parent)` 因 `ord_child/ref` 而包含 `ord_child`，
比较器据此把参考生产者排在前面并**提前返回**。

Humble 的比较器末尾**确实**有一段状态接口子句，且带明确的未完成标记：

```cpp
// If the ctrl_a's state interface is one exported by the ctrl_b then ctrl_b should be in front
// TODO(saikishor): deal with the state interface chaining in the sorting algorithm
```

但在**双向对**上这段代码**不可达**：参考边的提前返回把它完全遮蔽。
所以 Humble 的"确定 reference-first"**不是在两条矛盾约束中做了取舍**，
而是**状态边从未参与排序**。这与 master/Jazzy 不同——后者把 `S` 也写进
`following/preceding`，与参考边形成 2-环，再由"先插入者胜"决定。

**两者共同的结论不变**：单趟执行下双向耦合必有一条边陈旧，且**完全静默**。
而且按 `FORMAL_MODEL.md` 定理 1，这不是实现缺陷，是**单入口模型下无解**。

### 复现插桩

在 `controller_manager/src/controller_manager.cpp` 的 `controller_sorting()` 入口
（`cmd_itfs`/`state_itfs` 取到之后）临时插入：

```cpp
fprintf(stderr, "DBG cmp a=%s(state=%d) b=%s | a.cmd=[%s] a.state=[%s]\n", ...);
```

重建 `controller_manager` 后运行 `test_upstream_ordering`。
**观察要点**：每个用例只打印**一次**，即比较器在第一个检查处就返回了。
分析完成后务必还原源码并重建（本次已还原，`controller_manager` 关键测试 6/6、
库测试 3/3 复验通过）。

### 与 master 的差异（要如实写）

| | Humble（本地真实代码） | master（移植分析） |
|---|---|---|
| 排序实现 | `std::stable_sort` + `controller_sorting` 比较器 | 递归插入启发式 `update_list_with_controller_chain` |
| 双向边的结果 | **确定的** reference-first | **随加载顺序翻转** |
| state 方向 | 系统性陈旧 | 随加载顺序可能陈旧某一边 |
| 诊断 | 无告警 | 无告警 |

两者共同的结论：**单趟执行下 state 方向必然陈旧，且没有任何告警**。master 更糟一点，
因为结果还依赖加载顺序。

### 额外发现：Humble 上"声明状态边"完全不校验

`configure_controller(parent)` **成功**接受了 `ord_child/state` 这个**没有任何控制器导出的
state 接口**声明。也就是说：

- 声明状态边在配置期**可行且不被校验**（真实校验发生在激活时，claim 租借接口的阶段）；
- 该声明**不参与排序**（比较器把它当作一条与参考边冲突的约束，而参考边先被检查并返回）；
- 结果：**冲突完全静默**——没有报错、没有告警、没有日志提示。

这解释了为什么这个缺口长期没有被发现：拓扑声明层和调度层对同一对控制器给出了
互相矛盾的要求，而两层都不检查对方。

**注意**：Humble 的 chainable 控制器**不导出** state interface（这是 Jazzy+ 的能力，
PR #1021），所以上述实验只验证了**排序行为**，无法在 Humble 上验证"父读到陈旧状态值"的
数据通路。数据通路的陈旧由内核实验（`TWO_PASS_VS_SINGLE_PASS.md`）与管理器级实验
（`test_two_phase_execution.cpp`）证明。

---

## 10. Jazzy 与 master 的排序实现对比（2026-09-21 补充）

第 2–3 节的移植分析取自 master。本节回答"它是否也适用于 Jazzy"。

### 10.1 排序实现在 Humble 与 Jazzy 之间换过

| 发行版 | `controller_sorting` 出现次数 | `build_controllers_topology_info` / `update_list_with_controller_chain` 出现次数 |
|---|---|---|
| Humble | 2 | 0 |
| Jazzy | **0** | **6** |

也就是说：**Humble 用 `std::stable_sort` + 比较器；Jazzy 已经换成 master 那套递归插入启发式。**

### 10.2 Jazzy 与 master 的两个关键函数逐字节相同

把 `build_controllers_topology_info` 与 `update_list_with_controller_chain` 从两个分支提取出来、
去掉注释与空白后比较：

```text
build_controllers_topology_info          jazzy_len=2566 master_len=2566 identical=True
update_list_with_controller_chain        jazzy_len=2523 master_len=2523 identical=True
```

**因此第 2–3 节的移植结论直接适用于 Jazzy**（一个已发布的 ROS 2 LTS 发行版），
不需要在本机运行 Jazzy 就能确立**逻辑层面**的结论。

### 10.3 结论与差异（论文口径）

| | Humble（本地真实代码实测） | Jazzy / master（源码 + 移植） |
|---|---|---|
| 实现 | `std::stable_sort` + 比较器 | 递归插入启发式 |
| 双向边的结果 | **确定的** reference-first | **随加载顺序翻转** |
| state 方向 | 系统性陈旧 | 随加载顺序可能陈旧某一边 |
| 诊断 | 无 | 无 |

共同点（也是论文的主结论）：**单趟执行下双向耦合必有一条边陈旧，且完全静默。**
差异点：**从 Humble 升到 Jazzy，同一拓扑的执行顺序会从"确定"变成"依赖加载顺序"**——
这是一个值得单独指出的静默行为变化。

### 10.4 还需要什么

- **逻辑层面**：已闭环（Jazzy == master，且移植已执行验证）。
- **数据通路层面**：仍建议在 Jazzy/Rolling 上跑一次真实双向控制器对，
  证明父节点通过真实导出的 state interface 读到的是陈旧值。**这一步不是定理贡献的必需项**，
  只有在论文要主张"上游存在可复现缺陷"时才需要。
- 本机为 Ubuntu 22.04 且无 Docker，原生安装 Jazzy/Rolling 不可行；
  如需该实验，需要 Docker 镜像或 24.04 环境。
