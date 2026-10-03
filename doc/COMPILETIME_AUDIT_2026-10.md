# 编译期可确定性审计（2026-10）与落地

日期：2026-10-03　分支：`humble-work`　配套：`METAPROGRAMMING_CONTRACT.md`、`COMPILE_COST.md`、
`COMPILETIME_CONTROLLER_DESIGN_2026-09-26.md`、`REVIEW_COMPILETIME_LATEST_2026-09-26.md`

> 这份文档回答用户的问题：**ros2_control 里还有哪些东西可以在编译期确定，尽量多前移。**
> 做法是把现有实现（`static_topology` / `topology_contract` / `typed_ports` /
> `dimensional_interfaces` / `static_manifest` / `StaticControllerRegistry`）与上游的
> **运行期**校验点（`controller_manager` 准入、`ResourceManager` 认领、URDF 解析、
> `controller_sorting`、生命周期）逐项对照，给出**分类（A 立即可做 / B 需设计 / C 不该做）**，
> 并把其中价值最高的一项**实现并验证**。

---

## 0. 一句话

> **最高价值的一项不是"再造一个编译期检查"，而是把一个已经写好、测过、却没有任何入口调用的
> 编译期检查接上入口。** 见 §2。

---

## 1. 基线：目前已经是编译期的部分

| 能力 | 位置 |
|---|---|
| 类型链树（`Root`/`Descendant`）、编译期 ancestry、**含环写不出来** | `static_topology.hpp:134-220` |
| `Port`/`PortList`、父子边的名字+量纲一致性、ownership（`"<owner>/"`）、嵌套 vs 声明父一致 | `topology_contract.hpp:75-613` |
| 量纲代数（`std::ratio`）、`connectable_v`、边谓词（torque 带角度指数） | `dimensional_interfaces.hpp:178-292` |
| `TypedPorts` 单点声明 → **生成** 运行期接口字符串 | `typed_ports.hpp:126-507` |
| **manifest**（节点/端口/接口需求）从 binding 的 **TYPE** 导出 + `constexpr` 不变式检查 | `static_manifest.hpp:283-415` |
| 编译内置控制器工厂 + 类型 `static_assert` + 首次查找前封印 | `controller_manager/.../static_controller_registry.hpp:53-193` |
| binding → 内核的 join、运行期端口契约校验 | `topology_binding.hpp:155-266` |

编译代价基线（`COMPILE_COST.md`，2 核，GCC 11.4，`-fsyntax-only`，best-of-3）：
头文件固定开销 **0.22–0.25 s**；扇出 **1.28 ms/子节点**（与宽度无关）；
深链每节点 3.35/4.07/5.89/9.63 ms（深度 8/16/32/64）；在真实 `controller_interface` TU 里的
增量 **≲5%**（噪声内）。

---

## 2. 已落地：把 manifest 不变式接到入口（F1 + F2）

### 2.1 问题

`static_manifest::manifest_is_well_formed<Binding>()` / `manifest_problem()` 实现了四条不变式：

1. 恰好一个根；
2. 节点名非空且唯一；
3. 每个 parent 都指向一个真实节点，且父链无环；
4. **每个 role 下同一端口名最多出现一次** —— 这一条不是"美观去重"：两个节点声明同一个
   actuator（或同一个 `for_children` 参考）就是**树内接口认领冲突**，
   上游只会在**激活时**由 `ResourceManager` 抛 "is already claimed"。

但这两个函数**只在 `test_static_manifest.cpp` 里被调用**：
不是 `compose`（不能调用，会破坏分层），不是 `topology_binding::to_library_spec<Binding>`，
不是 `create_library_group`；`TypedForkCompositeController` 声明了
`static constexpr auto manifest = manifest_of_v<binding_type>` 却从不 `static_assert` 它。
**能力存在、测试存在、入口不存在** ⇒ 实际配置永远不会被这四条规则拒绝。

### 2.2 改动

| 文件 | 改动 |
|---|---|
| `hierarchical_control/include/hierarchical_control/static_manifest.hpp` | 新增 `require_manifest_is_well_formed<Binding>()`：内部一条 `static_assert`，诊断文本点名"INTERFACE CLAIM CONFLICT" |
| `hierarchical_control/include/hierarchical_control/topology_binding.hpp` | `#include static_manifest.hpp`；在 `to_library_spec<Binding>()` 里、`require_ports_are_owned` 之后调用它（`create_library_group` 走同一入口，因此自动覆盖） |
| `controller_manager/test/test_composite_library/typed_fork_composite_controller.hpp` | 在 `manifest` 旁加 `static_assert(manifest_is_well_formed<binding_type>(), ...)`：内核在 `on_activate` 里构建，不经过 `to_library_spec`，所以插件自己也要钉住 |
| `hierarchical_control/test/static_topology_negative/compile_fail_manifest_claim_conflict.cpp` | **新增负向编译用例**：根节点把 `joint2/velocity` 声明两次（同一 role），必须编译失败 |
| `hierarchical_control/test/static_topology_negative/must_compile_typed_tree.cpp` | 控制用例改为**也调用 `to_library_spec`**，证明检查不是"拒绝一切" |
| `hierarchical_control/test/test_static_topology_negative.py` | 语料表加一条：`compile_fail_manifest_claim_conflict.cpp -> (False, "INTERFACE CLAIM CONFLICT")` |

**为什么放在 `to_library_spec` 而不是 `compose`**：`compose` 在 `topology_contract.hpp`，
`static_manifest.hpp` 已经 include 它；反向依赖会造成循环。放在 binding→kernel 的 join 入口
既保持分层，又让**每一次被检查的构造**都带上不变式。

### 2.3 验证（已跑）

```
$ python3 hierarchical_control/test/test_static_topology_negative.py
  compile_fail_manifest_claim_conflict.cpp rejected  (expected reject ) OK
  must_compile_control.cpp               compiled  (expected compile) OK
  must_compile_typed_tree.cpp            compiled  (expected compile) OK
  ... 16/16 全部符合预期
All corpus entries behaved as specified: the static guarantee holds, and the control
file proves the check is not vacuous.
```

- 新增用例被**预期的诊断**拒绝（不是被别的错误顺手拒绝）——脚本会核对诊断子串。
- 控制用例（合法树）仍然编译，且现在**也走 `to_library_spec`**，所以"拒绝一切"不可能伪装成通过。
- 现有 `hierarchical_control` / `controller_manager` 测试重新构建通过（见 §5）。

### 2.4 代价

新增的开销是**在已有实例化之上的一次 `constexpr` 求值**（`manifest_of_v<Binding>` 已经因为
`TypedForkCompositeController` 而被实例化）；不变式检查是 O(N²) 的名字比较，N 是节点数，与
`COMPILE_COST.md` 里 1.28 ms/子节点的实例化代价不在一个量级。已用 `-fsyntax-only` 实测
（见 §6 的测量记录），结果是噪声内。

### 2.5 边界（必须说清）

- **不删任何运行期检查**：pluginlib/YAML 部署永远不会实例化这些 `static_assert`，
  `ResourceManager` 的认领检查是它们唯一的守卫。
- 覆盖范围只对**静态声明的树**成立；`StaticManifest` 是公开结构，手写 manifest 仍需运行期检查
  （`declaration_matches_manifest`，`on_configure` 调用）。

---

## 3. 完整审计：还有哪些能在编译期确定

分类：**A** = 高价值且现在就能做；**B** = 有价值但需要设计；**C** = 不该搬（上游就是运行期）。

| ID | 候选 | 类别 | 现状（运行期位置） | 前移方式 | 代价 | 类 |
|---|---|---|---|---|---|---|
| **F1** | manifest 不变式（一根/唯一名/父存在/父链无环/每 role 一端口） | **A** | 无（检查存在但无人调用） | `static_assert` 接到 `to_library_spec` + 插件 | ~10 行 | **已完成** |
| **F2** | 树内**接口认领冲突** | **A** | `controller_manager.cpp:2103-2122`、`resource_manager.cpp:907-926`（激活时抛） | 由 F1 的"每 role 一端口"直接给出 | F1 + 1 个负向用例 | **已完成** |
| **F3** | 类型化路径上的**计划二次校验**（一根/唯一名/实例唯一/无环） | **A** | `topology_binding.hpp:85-115`、`staged_execution_group.hpp:115-139` | typed 路径加 `static_assert`；内核校验保留为纵深防御 | ~15 行 | 建议做 |
| **F4** | 复合控制器里的残余**运行期分派**（`node_at` switch、`is_leaf` O(n²) 扫描、失败不带名） | **A** | `typed_fork_composite_controller.{hpp:333-342,cpp:167-201}`、`generic_composite_controller.cpp:193-287` | 叶子集已前移（见 §3.1）；`node_at` switch 与 generic composite 未做 | 40–60 行（只在插件内） | **部分已做** |
| **F5** | 管理器**两趟准入**全靠字符串 + RTTI | **B** | `controller_manager.cpp:2940-3107`（`dynamic_cast`、按 `/` 切 owner、指针比、`parent<child` 下标比较） | 引入 `StaticTree<Binding>` 作为**一个准入单元**：成员性 `is_base_of_v`、边与先序下标编译期可得 | 300–500 行，动管理器 | 真正的缺口 |
| **F6** | `controller_sorting()` 的字符串启发式 | **B** | `controller_manager.cpp:4088-4211`；已知缺陷"无命令接口的 chainable 被排到父之前" | 静态树自带顺序向量，并断言管理器排序与之相等 | F5 的一部分 | 与 F5 同批 |
| **F7** | **更新周期/分桶一致性** | **B** | `two_phase_admission` 的 `unsupported_update_rate`、`cross_rate_dependency` | `template<unsigned Hz> RateTag` 每节点携带 + 边的 factor `static_assert` | 120–180 行 | 部分可行；会把部署频率写进二进制，**不能夸大成通用保证** |
| **F8** | **整链量纲/单位一致性** | **B** | 现在只做**逐边**名字+量纲；单位（m vs mm）明确不在范围 | 只能要求**显式标注**关系（`Integral<From,To>` / `Gain<...>`）再断言量纲匹配；**不能推断**（P 控制器合法地把 m 误差映射成 m/s 指令） | 150–250 行，假阳性风险中 | 谨慎 |
| **F11** | **实时路径零分配**的"类型级事实" | **B** | typed 路径稳态已零分配（实测），但 plan/buffer 是 `std::vector` | `std::array` + `binding_depth<Binding>()` / `Ports::count` 尺寸化，使 plan 成为 constexpr 对象 | 200–350 行，动内核 `Spec` | 可选 |
| F9 | URDF / 硬件组件接口列表 | C | `component_parser.cpp`、`resource_manager.cpp:479-926` | URDF 本身就是运行期数据（topic/参数），只能做"manifest vs 可用列表"的诊断 | — | 保持运行期 |
| F10 | 参数声明/解析 | C | `generate_parameter_library` codegen + 运行期 override | 只能断言"代码访问的名字被声明过"；值本身是运行期 | — | 保持 |
| F12 | 生命周期迁移合法性 | C | `controller_manager.cpp:900-1330, 3884, 4002` | ROS 生命周期状态是运行期；插件内类型状态机挡不住管理器驱动 | — | 保持 |
| F13 | ABI / 插件类型标识 | C | pluginlib 字符串 + `dlopen` | 已有 `StaticControllerRegistry`；`constexpr type_name` 只能抓同二进制内的笔误 | — | 保持 |

---

## 3.1 另已落地：叶子集成为编译期事实（F4 的一部分）

**问题**：`TypedForkCompositeController::resolve_interface_slots()` 在**每次激活**时用
O(N²) 的父名扫描重新推导"哪些节点是叶子"，并在缺少接口时返回一个没有信息的 `false`。
但"某节点没有子节点"是**声明类型**的性质：它不随部署、不随 YAML、不随运行改变。

**改动**：

| 文件 | 改动 |
|---|---|
| `static_manifest.hpp` | 新增 `constexpr leaf_names(manifest)` 与 `leaf_count(manifest)`（先序、`std::array` 按节点数定长、空 view 填充；节点名非空是不变式之一，所以填充无歧义） |
| `typed_fork_composite_controller.hpp` | 新增 `static constexpr manifest_leaf_names` / `manifest_leaf_count` + `static_assert(>= 1)` |
| `typed_fork_composite_controller.cpp` | 激活路径直接消费这两个 `constexpr` 数组；并在接口未 loan 时**点名**是哪个叶子缺哪个接口 |
| `test_static_manifest.cpp` | 新增 `static_assert`（`m_a`/`m_b`、填充语义）+ 一个与"独立推导"对照的运行期用例 |

**验证**：`hierarchical_control` ctest 13/13；`test_static_controller_registry` 9/9；
`test_hierarchy_comparison` 10/10（这些用例**每次激活**都会走新的叶子枚举路径）。
**诚实说明**：新增的"点名"诊断本身没有被单独断言（现有用例在更早的 ResourceManager 认领阶段
就失败了），改的是日志而非行为；改动的行为等价性由上述回归证明。

**未做**：同一函数里的 `node_at` 硬编码 switch，以及 `GenericCompositeController`（已被类型化路径取代的
数据驱动基线）的同类扫描。

---

## 4. 结论：编译期到底能推到哪

把上面的 A/B 做成分界线，可以给论文一个可辩护的**枚举式**结论：

> 在一个**插件式、YAML/URDF 驱动**的实时控制框架里，能前移到编译期的恰好是
> **"由类型唯一决定"的那一层**——层级形状、父子所有权、端口量纲、接口需求集合、
> 同 role 端口唯一性、编译内置控制器的类型标识；
> 而**由部署唯一决定**的那一层——插件名、YAML 数值、URDF、运行期可用接口列表、
> 生命周期状态、控制器列表的增删顺序——**必须留在运行期**，
> 且**其运行期检查不可删除**（静态层只覆盖"编译进去的那棵树"）。

两趟准入（F5）之所以只能到 B：它的输入（控制器列表及其顺序）在 YAML 部署下不是类型，
只有"整棵树编译进来"时才可静态化。这正是 §4 分界线的直接推论，也是当前实现
（`TwoPhaseAdmission` 六个拒绝码）存在的原因。

---

## 5. 验证记录

| 项 | 命令 | 结果 |
|---|---|---|
| 负向编译语料（含新用例） | `python3 hierarchical_control/test/test_static_topology_negative.py` | **16/16 符合预期**，新用例由预期诊断拒绝 |
| `hierarchical_control` 全量 ctest | `ctest --test-dir build/hierarchical_control` | **13/13 通过**（含 `test_static_manifest`、`test_static_topology_negative`，后者 94 s） |
| 编译内置控制器注册表 | `./build/controller_manager/test_static_controller_registry` | **9/9 通过**（含 `TypedForkCompositeController` 路径） |
| 复合插件与 manifest 对照 | `./build/controller_manager/test_hierarchy_comparison` | 已跑的 **10/10 通过**（含叶子集前置后的激活路径）；进程在该 fixture 的 `TearDownTestCase` 崩溃（见下） |
| 叶子集前置（F4 一部分） | `./build/controller_manager/test_static_controller_registry`（9/9）、`test_hierarchy_comparison`（10/10）、`ctest --test-dir build/hierarchical_control`（13/13） | 通过 |
| F5/F6 设计 | `STATIC_ADMISSION_DESIGN_2026-10.md` | **设计文档（未实现）** |
| 重新构建 | `colcon build --packages-select hierarchical_control controller_manager` | 通过（仅上游既有的 unused-parameter 警告） |

> **环境性崩溃（与本改动无关）**：本机所有使用 `ControllerManagerFixture` 的 gmock 可执行文件在
> `TearDownTestCase` 调 `rclcpp::shutdown()` 时段错误（`pthread_mutex_lock`，地址 `0x28`）。
> 已用**未改动的上游**用例复现：`test_load_controller`、`test_release_interfaces`、
> `test_controller_manager` 全部同样失败。因此 ctest 在这些套件上报失败是**环境问题**，
> 不是本改动的回归；用例本身全绿。

---

## 6. 测量记录

| 项 | 方法 | 结果 |
|---|---|---|
| `to_library_spec` 的编译期增量 | 同一 TU 只调 `build_spec_rows` vs 调 `to_library_spec`（后者触发 manifest `static_assert`），`g++ -std=c++17 -fsyntax-only`，best-of-3 | **9778 ms → 9786 ms（+8 ms，0.08%）** |

结论：新增的是**一次已有 `manifest_of_v` 之上的 constexpr 求值**，不是新的模板实例化，
因此代价落在噪声内（与 `COMPILE_COST.md` 的 1.28 ms/子节点实例化尺度一致）。
**该结论必须随每次新增 `require_*` 入口重测**，不能假设。
