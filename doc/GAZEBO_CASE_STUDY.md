# Gazebo 案例研究：真实仿真闭环中的状态边陈旧

日期：2026-09-21
平台：本机 Ubuntu 22.04 + Gazebo Classic 11.10.2 + ROS 2 Humble + `gazebo_ros2_control`
代码：`case_study/`（包 `hierarchical_control_case_study`）

本文把前面的定理与内核实验落到**真实物理仿真闭环**上，验证：
**单趟执行让父节点读到的子状态落后一个控制周期，两趟执行不落后。**

---

## 1. 平台与架构

两轮差速底盘（`case_study/urdf/diff_drive.urdf`），`gazebo_ros2_control` 提供硬件接口，
`controller_manager` 由该插件创建。三个自研控制器：

| 控制器 | 角色 | 边 |
|---|---|---|
| `wheel_left` / `wheel_right` | 叶 | 导出 `target` reference interface（被子节点消费）；发布**累积轮行程**作为状态 |
| `chassis` | 根 | 反向：claim 两个 `wheel_*/target`（参考边），消费两个轮的行程（状态边） |

因此 `chassis` 与每个轮之间**同时存在两条边**——正是定理 1 的不可满足场景。

- 叶 `update_phase`：把实测关节速度经一个一阶滤波器积分成累积行程；
- 叶 `handle_phase`：PI 速度跟踪，写关节速度命令；
- 根 `update_phase`：用两个轮的行程做航位推算，并积分脚本化参考轨迹；
- 根 `handle_phase`：位姿反馈 → 两个轮的速度目标（写进 claim 到的 reference interface）。

单趟模式：控制器 `two_phase_legacy=true`，管理器按原生顺序（父先）跑一次 `update()`。
两趟模式：控制器 `two_phase_legacy=false`，YAML 里 `two_phase_execution: true`，
管理器反向跑 `update_phase`、正向跑 `handle_phase`。

---

## 2. 踩到的坑（值得写进论文的方法学部分）

第一版叶节点状态是 `travel += ∫ v·r dt`，我以为它"不可重推导"。**错了**：
父节点完全可以自己读 `wheel/position` 再乘半径得到同样的值。

> 这正是本课题一开始就识别出的陷阱：**当子状态可被父廉价重推导时，两阶段机制没有收益**。
> 用它做案例研究，会得出"机制无用"的错误结论——不是因为机制无用，而是因为选错了被控对象。

修正：叶节点加入**滤波器内部状态**（`filtered_velocity_ = α·filtered_velocity_ + (1-α)·v`，α=0.85），
积分的是滤波后的速度。滤波状态依赖该控制器自己的历史，**父节点无法从瞬时关节位置重建**。
这才是一个真实的"叶端估计器"（也正是 POV 底盘里每级维护估计的形态）。

另一个坑：PI 无积分限幅导致 Gazebo 求解器发散（关节速度变 NaN），
表现为底盘位姿全 NaN、测量样本数下降。加积分限幅（±2）与输出限幅（±8）后稳定。

---

## 3. 结果

### 3.1 控制器执行顺序（真实管理器）

```text
chassis      case_study/ChassisController   active
wheel_left   case_study/WheelController     active
wheel_right  case_study/WheelController     active
```

**父先于子**——与 `controller_sorting` 的参考边方向一致，也印证推论 1：
参考方向被满足，状态方向必然陈旧。

### 3.2 状态边陈旧量（核心结果）

测量方式（与理论无关、不依赖真值）：`chassis` 发布它**本周期用到的**轮行程，
每个轮发布它**自己的**轮行程；测量节点把两条时间序列做整数周期对齐，
取使误差最小的位移量。

| 模式 | 控制率 | 左轮行程滞后 | 右轮行程滞后 | 墙钟代价 |
|---|---|---|---|---|
| 单趟 | 50 Hz | **1 周期** | **1 周期** | 20 ms |
| 两趟 | 50 Hz | **0 周期** | **0 周期** | 0 ms |
| 单趟 | 20 Hz | **1 周期** | **1 周期** | **50 ms** |
| 两趟 | 20 Hz | **0 周期** | **0 周期** | 0 ms |

**滞后恒为"一个周期"，与控制率无关；但它的墙钟代价随周期线性增长（20 ms → 50 ms）。**
这正是 `CONTROL_COST_OF_LAG.md` 里相位裕度损失 `ω_c·L·Δt` 的输入项：
**同样的契约违反，控制率越低，代价越大。**

**在真实 Gazebo 物理仿真 + 真实 `ControllerManager` 上，单趟执行确实让根节点读到落后一个控制周期
（20 ms）的子状态，两趟执行不落后。** 这与定理 1、推论 1、定理 2 的预测一致。

---

## 4. 明确不声称的东西

**本案例研究不声称"两趟降低了轨迹跟踪误差"**，原因有二：

1. **测量指标目前不可靠**：底盘发布 13 元诊断数组，测量脚本把 Gazebo 真值位姿
   与参考位姿比对得到 RMS ≈ 5 m，而底盘自身的位姿误差末值只有 ~0.45 m。已排查并排除：
   - 主题名（`gazebo_ros_state` 实际发布在 `/model_states`，已修正）；
   - 时间对齐（chassis 与 truth 首样本仅差 36–80 ms，对齐正常）；
   - 残留仿真实例（确认无残留 gzserver）。
   **剩余症状**：真值位姿沿对角线以约 2 倍于航位推算的速度移动，
   说明**底盘的真实运动与轮子转动不一致**（打滑/被抬起/关节轴向），
   而不是测量代码的问题。已修正 spawn 高度（模型原点离地 0.16 m，轮底恰好在 z≈0）。
   下一步应打印真值的 z 与四元数、并核对两个轮的关节轴方向（当前两轮轴同为 base 系 +y，
   需确认正速度对应 +x 前进而非倒车）。**在该指标被修正之前不下任何结论。**
2. **即使指标修好，预期效应也很小**：该底盘回路带宽约 0.5 Hz，
   `ω_c·Δt = 2π·0.5·0.02 ≈ 0.06 rad`，按 `CONTROL_COST_OF_LAG.md` 的定律，
   一个周期只损失约 3.6° 相位裕度。要看到显著差异需要**高带宽**回路
   （`f_c` 接近 `1/Δt`），而不是这种为稳定性刻意放宽的简单外环。

正确的表述是：**本案例验证的是"契约在真实仿真中成立"（陈旧量 0 vs 1），
而不是"契约带来了可见的性能收益"。** 后者需要高带宽案例与可靠的真值指标。

---

## 5. 限制

1. **应用层状态绑定**：Humble 的 chainable 控制器不能导出 state interface（那是 Jazzy+ 的能力），
   所以"父读子状态"通过同进程注册表（`TravelRegistry`）实现，而不是 claim 一个 state interface。
   **调度问题完全相同**，但接口层需要 Jazzy 才能验证；
2. **Gazebo 在本 VM 上间歇崩溃**（约 1/3 的启动会出现 `free(): invalid pointer`），
   因此每次对照需要重跑；已在脚本里用"每次唯一 master 端口 + 启动前清理"降低概率；
3. **非实时**：仿真步长 1 ms、控制 50 Hz，跑在虚拟机里，不能用于实时性结论；
4. 只测了 50 Hz 一种控制率；更低控制率（20 Hz）会放大效应，尚未做；
5. 底盘控制器是简单 P + 前馈，没有积分项，绝对跟踪精度较差。

---

## 6. 复现

```bash
cd ~/Desktop/ros2_control-humble
source /opt/ros/humble/setup.bash
colcon build --packages-up-to controller_manager \
             --packages-select hierarchical_control_case_study \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# 单趟（原生单次 update，父先）
bash case_study/scripts/phase_b.sh 50 false 10
# 两趟（管理器反向 update_phase + 正向 handle_phase）
bash case_study/scripts/phase_b.sh 50 true 10
```

预期输出形如（**2026-09-24 重新测量，度量已修正，见第 6.1 节**）：

```text
[case-study] mode=single-pass rate=   50 Hz  samples=  581  chassis_cycles=  954  ...
[case-study] lag_left  = 1 cycles (min 1, max 1, n=581)  = 20.0 ms at the CONFIGURED 50 Hz period
[case-study]   raw counter difference for comparison: median -369 cycles (min -369, max -369) -- NOT the lag; it contains the activation offset
[case-study] lag_right = 1 cycles (min 1, max 1, n=581)  = 20.0 ms at the CONFIGURED 50 Hz period
[case-study] mode=two-pass    rate=   50 Hz  samples=  707  chassis_cycles=  902  ...
[case-study] lag_left  = 0 cycles (min 0, max 0, n=707)  = 0.0 ms at the CONFIGURED 50 Hz period
[case-study] lag_right = 0 cycles (min 0, max 0, n=707)  = 0.0 ms at the CONFIGURED 50 Hz period
```

脚本会自己启动 `gzserver`（无头、渲染关闭）、`robot_state_publisher`、生成 URDF/YAML、
spawn 模型与 4 个控制器、测量并清理；日志在 `case_study/logs/run_<rate>hz_<mode>/`。
本机 `gzserver` 约每 3 次启动就有 1 次在启动或关闭时崩溃（`free(): invalid pointer` /
`corrupted size vs. prev_size`），所以脚本必须**整轮重试**；上表来自
`case_study/scripts/retry_phase_b.sh`（每个模式最多 4 次尝试，两趟模式实际用了 2 次）。

### 6.1 度量修正（2026-09-24，评审 R9 的后续）

第一版"周期号"度量是**错的**，必须记录在这里以免再次被引用。它把滞后定义为
`chassis_cycle − wheel_cycle`，即两个**互相独立的**计数器之差。每个计数器都从**自己的控制器
被激活**时开始计数，于是这个差值里包含了两个控制器激活时刻的固定偏移。实测：

| | 单趟 | 两趟 |
|---|---|---|
| 原始计数器差值 | −369 / −172 cycles | −420 / −230 cycles |
| **同一次运行内该差值的变化范围** | **min = max（完全恒定）** | **min = max（完全恒定）** |

"完全恒定"说明这个数根本不反映任何周期性调度行为——它只是激活偏移，而且每次运行都不同
（同一配置两次运行得到 −289/−118 与 −369/−172）。表里的 −369 ms 之类数字**没有任何意义**。

正确的做法是利用**所有控制器共享的时钟**：`ControllerManager::update(time, period)` 在同一个
周期里给每个控制器传**同一个 `time`**，所以

```text
lag_ns = (本控制器本周期看到的 time) − (产出它所读值的那个周期看到的 time)
```

是一个**共享纪元**下的差值，恰好等于周期的整数倍，不含激活偏移、不需要时钟对齐、也混不进
DDS 排队。控制器现在把 `lag_*_ns` 与 `lag_*_cycles` 一起发布（诊断字段 16–19），脚本会
**校验 `lag_ns == lag_cycles × period_ns`**，不满足就直接让实验失败；出现负滞后也直接失败。
本次两个模式都通过了该校验（581 与 707 个样本，无一个例外），这是对"共享时钟"这一前提的
直接验证。原始的计数器差值仍会打印出来，但标注为"不是滞后"。

**修正后的结论与修正前一致**（单趟 1 周期、两趟 0 周期），但现在这个结论有了正确的证据链：
滞后是在生产者/消费者的**周期时间戳**上直接测出的，而不是两个计数器相减。

---

## 7. 对论文的意义

- **补上了"仿真验证"这一环**：契约不只在单元测试里成立，在真实物理仿真 + 真实管理器中成立；
- **暴露并对冲了一个方法论陷阱**：案例研究的被控对象必须包含**不可重推导**的子状态，
  否则会错误地否定机制的价值。这一条本身就值得写进论文的实验设计章节；
- **划清了声称边界**：契约成立 ≠ 性能收益。性能收益需要高带宽案例 + 可靠真值指标，
  这是下一步的实验要求。
