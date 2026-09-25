# 双向树调度与元编程：代码要求复审

日期：2026-09-24。版本：`b9db646f`，对比前次 `b1bf6166`。
本次不评投稿条件，只回答用户两个要求是否实现。

## 结论

**双向树调度：受限功能基本实现。元编程：有实质实现，但与整棵执行树的绑定尚未闭环。**

执行内核能对单频同步单树做子先父的状态计算和父先子的命令计算，支持多子分支；
轻量 manager 路径能在原生参考顺序有效时反向调用 update_phase、正向调用 handle_phase。
两条路径并非同一套保证：完整帧/临时缓冲属于 staged group，轻量两趟没有提交回滚。

模板层已实际生成接口列表、保存类型化实例、展开运行配置，并能检查部分拓扑/量纲错误。
但当前 BoundNode 仅有一个 Next，组合链和 Node::parent_type 可不一致；实际建组没有强制
执行父子端口兼容验证。不能据此宣布“任意分叉树都从唯一静态声明生成且自动检查”。

## 本次确认的改进

- NaN 预填使漏写状态/reference/执行器值不再静默继承上一轮有限值。
- sink 全部成功后才更新内部 committed 视图；sink 已产生的外部副作用仍不可回滚，代码已说明。
- void* 路径改成 ControllerInterfaceBase* 和 dynamic_cast，处理多继承地址调整。
- 同父多端口不再被简单误判为不同写者。
- 重叠模式、频率不一致已有准入检查；状态阶段失败抑制轻量路径的命令阶段。
- 已知轻量成员在切换暂停期间不再因 run_two_phase=false 自动进入原生 update。
- 成员容器改为原子共享快照，重建搬到非实时路径；这修复容器读写问题，但不是全部并发问题的证明。
- TypedPorts 已拆分 State、Reference、Actuators、ForChildren，比之前三组混合语义准确。

以上是源码核查，不等于本轮重新运行完整 ROS 测试。

## 仍须优先修正的项目

### A [P1] 静态父关系与实际生成关系可能不同（本机已复现）

`topology_contract.hpp:235` 的 BoundNode 只检查每个 Node 自身合法；
`compose` 没有检查 ChildBinding::node_type::parent_type 是否是当前 Node。
`fill_spec_rows` 则用嵌套结构传入 parent_name，忽略 Node 的 parent_type。

反例：A=Root<a>、B=Root<b>，make_leaf<B> 后 compose<A> 能编译；
build_spec_rows 输出 B.parent="a"。两个根的类型声明被静默改成单根链。
本机用真实 topology_contract.hpp 和仅含虚析构的 ROS 基类替身复现，输出：
`B declared as Root, emitted parent=a`。这只检验模板结构，不模拟 ROS 插件执行。

修复：消除双重拓扑来源，或在 compose 强制子节点 parent_type 与当前节点一致。
验收：错误父类型、将 Root 作为子节点、相同名字但不同父类型均 compile-fail；
正确类型声明生成的所有实际边与类型边逐条一致。

### B [P1/需求缺口] 静态构建器只支持链，不支持整棵分叉树

BoundNode<ControllerT,Node,ContractT,Next>、next_storage 和 fill_spec_rows 都只有一个 next。
运行内核的 children_ 虽然是多子数组，但从类型绑定到执行器的主入口只能产生一条链。
多根/多链手动拼接 SpecRows 并不等于统一类型树检查。

修复：采用 Children.../tuple 或一个统一静态节点表，检查唯一节点、父存在、单根及实例唯一。
ForChildren 还须按每个孩子分别定义/路由；不能把父全部子端口与单个孩子 reference 做相等比较。
首个验收树建议为 chassis -> {left_module,right_module} -> 每模块两个电机。
不必先做四舵轮或 URDF；七节点即可暴露链实现掩盖的分叉问题。

### C [P1] 端口检查不是建组必经步骤

`topology_binding.hpp:174` create_library_group 只调用 to_library_spec；后者仅强制
require_ports_are_owned。`TypedPorts::ForChildren` 不进入 contract_of_t 的 Contract<State,Reference>。
父子量纲/名称/顺序比较 require_declarations_compatible 是独立显式函数，compose/建组不会调用它。
状态方向没有“父期待子状态类型”的声明。携带 dimension 类型本身不等于比较了连接两端的量纲。

verify_binding_ports 也是可选调用，而且 verify_ports_match_contract 实际只比两个列表长度，
不是注释所写的名称和顺序检查。同长度、不同端口/顺序仍能通过。
TypedPortsMixin 可保证自己的运行列表由 Ports 生成，但不能阻止调用者另传不匹配 Contract。

修复：提供一个真正 checked 的建组入口，自动递归检查连接两端，自动完成类型与运行实例适配。
调试 stub 不能成为默认关闭检查的理由；需要绕过时使用明确的 unchecked 测试入口。
测试必须通过用户实际会调用的 compose/create 入口验证：同名异量纲、同长度错名/错序、
不存在状态端口、错误 Contract 全部被拒。不能只单独调用检查工具后宣布建组安全。

### D [P1，静态风险，未做实际manager TSan] 模式与快照发布仍非一个一致状态

`controller_manager.hpp:556` two_phase_enabled_ 仍是普通 bool；非实时 set 方法写它，
update 多次读取。快照原子化没有自动消除该 bool 的数据竞争。
先写 enabled 再发布 entries，可能在运行中短暂出现 enabled=true 但 entries 为空，
让组件走原生分支。entries、controller list、staged group 独立发布也没有统一 generation。

`two_phase_entries()` 返回引用，但持有快照的局部 shared_ptr 离开函数即释放；并发替换后
返回引用可能悬空。set_staged_execution_group 发布后再 refresh_member_active_state，
也可能与 run 对普通 members_active_ 的访问重叠。

现有 TSan harness 只模拟容器原子发布，不包含这些字段和实际 manager 切换协议。
建议先禁止运行中启停模式，给出并执行停止期配置约束；需要支持动态切换时把模式、成员和
计划代际一起发布，持有快照对象而不是返回借用引用。
atomic shared_ptr 不保证 lock-free，最后一个引用在实时线程释放仍可导致析构/释放内存。

### E [P2，调度边界] 新/不合格组件仍可能走原生路径

rebuild 对不合格成员记录 ERROR 后排除，并允许 native 执行；全局 setter 对现有集合的拒绝
不能覆盖后来加载/配置的组件。案例插件的 two_phase_legacy=false 下原生入口返回 OK 但不算命令。
结果可能是配置看似成功、某个组件实际上不工作，或父子混合阶段导致一条边再次用旧状态。

建议：运行组准入失败应使该组配置/激活失败；首版拒绝横跨 legacy/staged/two-phase 边界的依赖。
验证“模式先开后加低频插件”“组内成员停用后父节点继续运行”“晚加入成员首周期”的行为。

## 按用户两项要求的验收表

| 验收项 | 当前判断 |
|---|---|
| 一棵已正确绑定的运行树，状态叶到根、命令根到叶 | 基本具备 |
| 分叉树每节点每阶段一次、同周期来源传播 | 内核有支持；应补与静态构建器贯通的七节点用例 |
| 状态失败后不计算组命令 | 两条路径已补规则；内部状态恢复仍是应用契约 |
| 所有提交失败都没有部分硬件句柄副作用 | 不具备，也不是轻量两趟功能；必须限定承诺 |
| 模板生成端口列表与运行计划 | 具备链式示范 |
| 一个类型树声明直接生成任意分叉运行树 | 尚缺 |
| 类型父子关系与实际执行边一致 | 当前可绕过，A项已复现 |
| 两个方向的端口名称、量纲和维度自动检查 | 部分工具具备，默认入口尚未闭环 |
| 运行中启停、加载的并发和代际一致性 | 尚不能确认安全 |

## 推荐下一步（只为代码目标，不扩展论文任务）

1. 优先实现严格的、支持多孩子的 typed builder；把端口检查变成构建必经步骤。
2. 用同一七节点树同时证明：静态声明→实际边→状态/命令顺序→本周期数值。
   反向打乱注册顺序和兄弟声明顺序，按命名端口核对结果，避免靠位置巧合通过。
3. 补真实入口的负向编译测试及错误运行配置测试；保留现有计数/故障测试。
4. 明确首版只允许停止期配置，或补完整发布协议；不要把容器 TSan 通过视作 manager 全部安全。
5. 将该 typed tree 示例集成到普通 composite 插件，验证库路径；再以同算法对照 manager 路径。

当以上完成，可准确说“代码实现了元编程驱动的树状双向调度”。当前更准确的说法是：
“已实现树状双向运行内核和多项元编程工具，二者的严格整树绑定还需完善”。

## 本轮验证和版本管理

本地已快进到 b9db646f。远端已包含旧评审同名文档，本地旧评审与探针先保存为
stash `review-artifacts-before-b9db646f-2026-09-24`，未覆盖或丢弃原工作。

本机本轮：旧 standalone Release CTest 1/1；阶段图 Python 检查 465/465 规范调度成功；
上面静态父关系错配探针成功复现。新模板头已依赖 ROS，完整 ROS gtest/编译反例及 TSan/Gazebo
需在 Ubuntu 验证，本轮未重跑，不沿用前一版本的通过数量冒充本轮结果。
本轮仅增加评审文档，不修改生产代码，不提交或推送。
