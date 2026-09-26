# 最新编译期控制器方案审核

日期：2026-09-26
审核版本：`86ae9f64`
范围：`static_manifest`、编译内置 controller registry、typed composite、执行 generation、atomic activation 和 TSan 验证。

## 总体结论

本轮已经把“编译期控制器”从设计推进为可运行的分层实现：

- typed binding 生成 `constexpr` manifest；
- manifest 生成 command/state interface 声明；
- `configure` 检查运行时声明与 manifest 一致；
- `activate` 解析本次 loaned interface 的槽位；
- compiled-in controller 通过 registry 进入普通 `load/configure/activate/update` 路径；
- two-phase、成员集合和 staged group 使用统一 generation；
- manager 自身的 TSan data race 已从 4 条降为 0 条。

因此当前实现可以准确称为：

> 编译期生成控制图和接口描述，运行时完成 ROS 生命周期与硬件资源绑定。

仍不能称为“在 C++ 静态初始化阶段完成 ROS controller 的完整初始化”，也不能将真实硬件模式回滚、依赖库内部 TSan 或物理总线原子性视为已解决。

## P1 问题

### P1-1：atomic activation 回滚没有完整恢复硬件模式

位置：

- `controller_manager/src/controller_manager.cpp` 的 `manage_switch()`；
- `activate_controllers()`；
- `rollback_activated_controllers()`。

当前流程先对整个 activate 集合执行 `perform_command_mode_switch()`。如果前面的 controller 已经成功激活，后面的 controller 激活失败，atomic activation 会调用 `rollback_activated_controllers()`，但该函数只做：

```text
controller.deactivate()
controller.release_interfaces()
```

它没有对“本次已成功激活、随后被回滚”的 controller 重新执行硬件 command mode rollback。`failed_controllers_command_interfaces` 只记录激活失败的 controller，不包含已经成功后被回滚的 controller。

可能结果：

```text
controller 变为 inactive
loaned interfaces 已释放
硬件 command mode 仍处于 active
```

这使“all-or-nothing activation”只对 controller lifecycle 部分成立，对硬件模式不成立。

修复建议：为 activation outcome 记录本次所有成功激活 controller 的 command interface 集合。回滚时：

1. deactivate 所有本次激活的 controller；
2. release 它们的 interfaces；
3. 对这些成功激活接口执行对应的 `prepare_command_mode_switch` / `perform_command_mode_switch` 回退；
4. 检查 reference interface availability 和 chained mode 是否恢复；
5. 若回滚失败，记录明确的 rollback failure，而不是只报告原始 activation failure。

验收测试：两个 controller 组成激活组，第一个成功、第二个在 `on_activate` 失败；mock hardware 记录 command mode，最终必须回到切换前状态，两个 controller 都是 inactive，接口都未 claim。

### P1-2：atomic activation 默认关闭，静态树可能部分激活

位置：`ControllerManager::atomic_activation_` 及 `set_atomic_activation()`。

当前 atomic activation 是 opt-in，默认值为 false，以保持 Humble 上游行为。这对普通 controller 是合理兼容策略，但对静态 typed execution tree 不够安全：用户可以配置完整树，却只激活其中一部分节点，形成不完整的控制树。

需要明确策略：

- 安装 staged/static execution group 时要求 `atomic_activation=true`；或
- 静态树配置主动拒绝 best-effort activation；��
- 明确声明部分激活是允许的，并让执行组返回 inactive/fault，不得执行半棵树。

至少应增加一个测试，证明静态 execution group 在 atomic activation 关闭时的行为是明确且可预测的。

## P2 问题

### P2-1：`manifest_problem()` 不检查手工 manifest 的长环

位置：`hierarchical_control/include/hierarchical_control/static_manifest.hpp`。

`manifest_problem()` 检查根数量、重名、自父和父节点存在，但没有检查：

```text
a -> b
b -> c
c -> a
```

由 typed binding 生成的 manifest 通常不会出现这个问题，因为 `static_topology` 已经检查类型父链；但 `StaticManifest` 是公开结构，调用者可以手工构造。函数文档将它描述为 malformed manifest 的 constexpr 检查器，因此应完整检查环。

修复建议：加入 constexpr DFS 或父链追踪，增加手工三节点环的 `static_assert`/运行时测试。

### P2-2：compiled-in registry 的注册时机没有代码约束

位置：

- `controller_manager/include/controller_manager/static_controller_registry.hpp`；
- `ControllerManager::set_static_controller_registry()`；
- `register_static_controller_type()`。

registry 内部使用 `std::map`，`static_controller_registry_` 是普通 `shared_ptr`。文档假定注册发生在启动阶段、controller load 之前；但公开 API 没有拒绝运行期注册或替换。

如果一个线程调用 `load_controller()`，另一个线程调用 `set_static_controller_registry()` 或 registry `add()`，会产生 map/shared_ptr 并发访问风险。

建议选一个明确方案：

- 只允许 manager 未启动且没有已加载 controller 时安装/注册，违反时返回错误；或
- 构造 immutable registry snapshot，运行期只读；或
- 为注册和 lookup 增加明确的非实时锁，并规定 load 不能与注册并发。

建议测试：manager 已加载 controller 后尝试替换 registry，必须拒绝且不改变当前可加载类型集合。

### P2-3：factory registry 的 manifest 能力不覆盖参数化 factory

`add<ControllerT>()` 可以读取 `ControllerT::manifest`，但 `add_factory()` 只接受 factory，没有 manifest descriptor 参数。因此需要构造参数的 compiled-in controller 可以被加载，却无法被 registry 工具枚举其编译期 command/state 需求。

建议增加带 manifest 的 factory API，例如：

```cpp
add_factory(type, factory, StaticManifestView manifest);
```

或定义一个包含 factory 和 manifest 的 `StaticControllerDescriptor`。否则“编译内置 controller 都可被静态描述工具检查”只对默认构造类型成立。

### P2-4：TSan 脚本在测试程序启动失败时可能误报 PASS

位置：`controller_manager/test/run_tsan_real_manager.sh`。

脚本保存了测试进程退出码，但最终 verdict 只检查：

```bash
if [ "$races" = "0" ]; then
  echo "[tsan] RESULT: PASS ..."
  exit 0
fi
```

若二进制没有启动、没有输出 gtest 结果，或者构建失败后运行了旧二进制，只要没有匹配到 `data race`，脚本仍可能退出成功。

建议同时检查：

- 测试进程退出码；
- gtest 结果行是否存在；
- 是否实际运行了预期测试套件。

TSan 的 race verdict 可以与功能测试 verdict 分开报告，但不能把“没有 race 日志”当作“测试成功”。

## 已确认的正面进展

- `static_manifest.hpp` 使用 `static constexpr std::array`，不是运行时 `std::span` 假装编译期对象；
- manifest 增加 `HardwareState`，能够描述叶节点硬件 state 输入；
- typed composite 使用 manifest 生成硬件接口名，减少重复手写；
- interface 名称到本次 activation loaned slot 的解析留在 `activate`，没有把 ResourceManager 内部索引硬编码进 manifest；
- compiled-in controller 使用 factory 创建新实例，不是共享全局 singleton；
- static registry 与 pluginlib controller 走同一个 `add_controller_impl()` 生命周期路径；
- 两个 manager 实例可以使用同一 manifest，但拥有独立 controller state；
- manager 自身真实 TSan 已无 data race 报告，依赖库仍未插桩；
- runtime reconfiguration 已限制为：周期进行中拒绝安装/扩展 execution path，移除路径允许；
- generation 将 two-phase flag、成员表和 staged group 作为不可变快照一次发布。

## 推荐下一步

1. 先修 P1-1 的硬件模式回滚，并用 mock hardware 验证；
2. 决定静态树是否强制 atomic activation；
3. 增加 manifest constexpr 环检查；
4. 限制 registry 注册时机或改成 immutable snapshot；
5. 修正 TSan 脚本的进程退出码和 gtest 结果判断；
6. 再考虑参数化 factory 的 manifest descriptor；
7. 最后补完整 Humble VM 构建和测试记录，区分代码失败、宿主负载失败和未插桩依赖报告。

## 当前可准确表述的结论

当前实现已经支持：

- 编译期 typed topology 和 hardware interface manifest；
- configure 期的 manifest/interface 一致性检查；
- activate 期的实际 loaned interface 槽位解析；
- 编译内置 controller 通过普通 lifecycle 和资源认领路径加载；
- 同周期双向树执行和 manager generation 发布；
- manager 自身已测的无 data race 发布协议。

当前仍不能声称：

- 完整静态初始化 ROS controller；
- atomic activation 已经恢复所有硬件 mode side effects；
- 依赖库内部无 TSan 问题；
- 任意运行期 registry 变更是线程安全的；
- 硬件总线的物理原子提交。
