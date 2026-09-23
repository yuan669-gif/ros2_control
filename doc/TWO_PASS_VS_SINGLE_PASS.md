# 两趟执行 vs 单趟执行：把 FineMote 的 Update/Handle 做成可验证的命题

日期：2026-09-21
关联：`doc/BIDIRECTIONAL_EDGE_ANALYSIS.md`（上游双向边的静默降级）

本文回答一个具体问题，并给出**可执行的量化证据**：

> 上游 `ros2_control` 同时用 reference 边和 state 边求**一个**执行顺序，导致双向场景下
> 静默退化为一周期延迟。FineMote 的做法是**一个线性顺序、正反两趟**。
> 两趟真的能解决吗？代价是什么？

**答案：能，而且代价是零额外存储。** 本文给出定量证明。

---

## 1. FineMote 的实际机制

论文 III-B.3 与 II-B.1：

- `Update`（■）：modifies internal state based on data received from other modules；
- `Handle`（●）：computes actions for the current cycle from the device state；
- 注册表是**构造顺序的线性数组**；每 tick **正向遍历跑 Update、反向遍历跑 Handle**。

关键性质：**同一个线性顺序，两个阶段方向相反。**
正向遍历保证"子先于父"→ 父在 Update 里读到子的**本周期**状态；
反向遍历保证"父先于子"→ 子在 Handle 里用到父的**本周期**命令。
因为两个阶段方向相反，**一个线性化顺序同时满足两个方向的依赖**，不需要 state 边参与排序。

这正好解决了上游的病根：上游要用**一个方向的一趟**同时满足两个相反约束，才不得不让
state 边与 reference 边互相冲突。

---

## 2. 内核改动（已完成）

`hierarchical_control/include/hierarchical_control/staged_execution_group.hpp`：

```diff
-    // command stage: preorder (parents before children)
-    for (const auto node : plan_.preorder)
+    // command stage: the SAME linear order iterated BACKWARD
+    for (std::size_t command_slot = plan_.postorder.size(); command_slot-- > 0;)
     {
+      const auto node = plan_.postorder[command_slot];
```

理由：**后序的反转是合法的"父先于后代"顺序**（后序保证每个后代排在其祖先之前）。
因此：

- 状态阶段 = `postorder` **正向**（子先于父）；
- 命令阶段 = `postorder` **反向**（父先于子）；
- `plan_.preorder` 不再被内核使用（`hierarchy.hpp` 仍保留该字段供 v1/`cycle_tree` 使用）。

**收益**：少一个依赖数组；内核与 FineMote 的机制一一对应（一个 registry，两个方向）。

---

## 3. 定量实验：单趟只能保一个方向

新增测试 `HierarchicalControlKernel.one_pass_can_keep_only_one_direction_same_cycle`
（`hierarchical_control/test/test_execution_group.cpp`）。

**模型**：三级级联 root → mid → leaf，每个节点有**不可被父节点重推导**的内部估计
（对自己子节点估计的一阶滤波；叶节点对硬件）。每个节点 `command = parent_reference − estimate`。
在某个周期注入一个阶跃，测量每个节点**首次响应**的周期与阶跃周期的差（lag）。

两种场景：

- **上行阶跃**（硬件阶跃，测试"父能否看到子本周期状态"）；
- **下行阶跃**（根参考阶跃，测试"子能否用到父本周期命令"）。

三个调度器：

1. 单趟、父先（**ros2_control 当前使用的方向**）：每个节点一个方法，一次调用里同时"摄取子状态"与"产生命令"；
2. 单趟、子先；
3. **两趟、同一顺序**（反向 update、正向 handle）。

### 实测结果（gtest 断言，10/10 通过）

| 调度器 | 上行阶跃 lag（每个节点） | 下行阶跃 lag（每个节点） |
|---|---|---|
| 单趟、父先 | **`depth − i`**：根最陈旧 = `depth`，叶 = 0 | 0（全部同周期） |
| 单趟、子先 | 0（全部同周期） | **`i`**：叶最陈旧 = `depth`，根 = 0 |
| **两趟** | **0** | **0** |

其中 `i` 是节点深度（根 = 0，叶 = `depth`），`depth = 节点数 − 1`。

### 结论

1. **单趟执行必然有一个方向陈旧，且陈旧量正比于级联深度。**
   换方向只是把陈旧从上行搬到下行，不能消除。
2. **两趟执行让两个方向都同周期**，且**只需一个线性顺序**（零额外存储）。
3. 这正是 FineMote 的 `Update`/`Handle` 机制，也解释了为什么它的"线性注册表 + 正反遍历"
   不是省事的技巧，而是**必要性**：单趟在双向级联上做不到。

---

## 4. 与上游的关系（论文口径）

- 上游的 state chaining（PR #1021、issue #1123、commit 69b3225）解决的是**只有状态边**的场景，
  例如估计器 → 控制器；reference chaining 解决**只有参考边**的场景，例如控制器 → 执行器。
- 上游仍是**一个 `update()` 入口**，因此**无法**同时满足两个方向；它的排序启发式
  在遇到双向边时静默降级（见 `BIDIRECTIONAL_EDGE_ANALYSIS.md`）。
- 本内核给出的修复：**把执行拆成两个方向相反的两趟**。这不需要改状态/参考接口机制，
  只需要控制器实现两个阶段，以及管理器按一个线性顺序跑两趟。

**上游需要新增的字段**：如果要在 `ControllerManager` 层面做这件事，需要一个"顺序"来源。
两种可行来源：

1. 复用现有 reference 边（一个方向就够，因为反向遍历自动给出另一个方向）——
   **不需要新字段**；
2. 显式声明父子关系（本项目内核的 `Spec::parents` 已经是这种字段），
   用于接口无法表达依赖的组合，或用于诊断。

> 也就是说：用户设想的"在 ros2_control 里加字段指定上下级"，
> 在**内核层已经存在**（`Spec::parents`）；缺的是把它接到管理器配置（YAML/参数）上。
> 但更重要的结论是：**两趟之后，字段不是必需的**——reference 边提供的顺序已经足够。

---

## 5. 尚未做（下一个窗口）

1. **管理器级两趟**：目前两趟只在执行组/库宿主内。要验证"整个 controller 列表跑两趟"
   （FineMote 的形态），需要在 `ControllerManager::update()` 里加一个 opt-in 模式：
   - 第一趟：反向遍历控制器列表，调用 `update_phase()`；
   - 第二趟：正向遍历，调用 `handle_phase()`；
   - 两阶段控制器在 legacy 循环里被跳过。
   这需要给 `ControllerInterfaceBase` 加第二个虚函数，或加一个 mixin。
2. **Humble 上的接口级双向实验**：Humble 的 `ChainableControllerInterface` 不导出 state interface，
   所以"父 claim 子状态"无法通过接口表达。要复现上游行为需要 Jazzy/Rolling，
   或用显式 `parents` 字段（本内核支持）绕过。
3. **追踪误差/相位滞后**：本文的 lag 是**周期数**（离散、确定性）。
   要写成控制性能结论，还需把它换算成相位滞后或跟踪误差（带真实控制器动力学）。
4. **存储/耗时对比**：少一个数组的收益很小，应实测并如实报告（预期在噪声内）。

---

## 6. 复现

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
./build/hierarchical_control/test_execution_group \
  --gtest_filter=*one_pass_can_keep_only_one_direction_same_cycle*
```

---

## 7. 管理器级验证（已完成）

前面的证明在内核层。本节把它搬到**真实的 `ControllerManager`**：
同一份控制器代码，用真实管理器跑两种模式，拓扑与算法完全相同。

实现：

- 新增 mixin `hierarchical_control/two_phase_controller_interface.hpp`
  （`update_phase()` / `handle_phase()`，故意比 `StagedExecutionGroup` 更轻：
  没有组、没有端口声明、没有帧、没有组提交）；
- `ControllerManager` 新增 opt-in：
  ```cpp
  void set_two_phase_execution(bool enabled);
  bool two_phase_execution() const;
  ```
  `update()` 里：
  - Pass 1：**反向**遍历控制器列表，调用 `update_phase()`；
  - legacy 循环：跳过实现了该 mixin 的控制器；
  - Pass 2：**正向**遍历，调用 `handle_phase()`；
  - 切换挂起时两趟都不跑（成员集合可能正在变化）。

测试：`controller_manager/test/test_two_phase_execution.cpp`（2 个用例）。
级联 root → mid → leaf，每级估计**不可被父重推导**（对自己子估计的一阶滤波），
第 10 周期注入叶输入阶跃，测量每级首次响应的周期滞后。

### 实测（真实 ControllerManager，两用例均通过）

| 模式 | leaf lag | mid lag | root lag |
|---|---|---|---|
| 单趟（原生 `update()`，列表正向） | 0 | **1** | **2** |
| **两趟（opt-in）** | 0 | **0** | **0** |

即：**在真实管理器里，单趟执行下根节点的信息陈旧整整一个"深度"（这里 2 个周期）；
两趟执行下所有层级都是同周期。**

注意：管理器现有排序是"父先"，所以单趟模式下**下行是新鲜的**，上行随深度变陈旧。
两趟把上行也修好。若排序方向相反（子先），单趟会反过来牺牲下行——这正是
`upstream_order_sim.py` 里观察到的"载荷顺序决定哪条边陈旧"。

### 顺带发现的 Humble 约束（重要，写进论文/文档）

第一版实现在 `update()` 的切换路径里直接调用了加锁的成员刷新，结果**死锁**。根因：

> `ControllerManager::switch_controller()` 在**持有 `controllers_lock_`** 的同时
> 通过条件变量等待实时线程应用切换（`controller_manager.cpp` 中"lock controllers"之后的
> `guard` 一直存活到 `switch_params_.cv.wait_for(...)`）。因此**实时路径上任何获取该锁的代码
> 都会死锁**：实时线程阻塞在锁上 → 不再推进 `used_by_realtime_controllers_index_` →
> 等待切换的线程在 `wait_until_rt_not_using()` 里永久自旋。

修复方式：

- `set_two_phase_execution()`（非实时线程）可以加锁刷新；
- 实时路径改为**脏标志**：切换后只置 `two_phase_entries_dirty_ = true`，
  在**下一个周期开头**用本线程已经持有的 `rt_controller_list` 重建成员（O(n) `dynamic_cast`，
  每个切换最多一次），**全程不加锁**。

这也解释了为什么先前的 staged group 刷新活跃状态时特意不加锁。任何要接入
`ControllerManager` 实时循环的新机制都必须遵守这条约束。
