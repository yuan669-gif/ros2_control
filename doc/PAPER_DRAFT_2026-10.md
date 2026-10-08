# Two-Pass Execution for Runtime-Loaded Tree Controllers

### Quantifying silent staleness in ROS 2 `ros2_control` and the admission cost of porting a static discipline

*Draft, 2026-10. Branch under study: `feature/two-phase-manager` (base: upstream `ros2_control` Humble `469f3055`; 22 files, +5102 lines, of which the production change is 1280 lines in three files).*

> **中文阅读指南（给作者，不属于正文）**
> * 正文用英文写，按 (b) 只讲**创新点 1**；编译期元编程在 §9 作为 future work 一段带过。
> * 全文的"不主张什么"集中在 §1.4 与 §8 两个方框里，投稿前**不要删**——它们是这份稿子最容易被审稿人
>   攻击的地方的正面回答。
> * 所有数字都能在分支里复跑，命令见附录 A；每个数字后面标了它来自哪个测试。
> * 待你补的部分用 `TODO(author)` 标出：作者/单位、目标期刊格式、以及 §7.2 里是否补跨编译器编译代价
>   （那属于已降级为 future work 的编译期线，可只留一句）。

---

## Abstract

Tree-structured robot controllers have a **bidirectional** data dependency: state is aggregated from the
leaves while decisions propagate back to them. A scheduler that runs every controller exactly once per
cycle cannot satisfy both directions in the same cycle, and the resulting staleness is silent — every
cycle still executes every controller, so call counts and cycle times look correct. A recent firmware
framework establishes the two-stage discipline that fixes this for a statically generated schedule. We
ask what that discipline costs when the controller set is instead **loaded at run time as plugins and
configured from YAML**, as in ROS 2's `ros2_control`.

We make three contributions. First, we characterise and quantify the failure on **unmodified
`ros2_control` Humble**: the parent/child interface graph has two edge kinds that are
indistinguishable at the interface level, the manager maintains one linearisation and therefore fixes
one direction and silently ages the other, and the ordering itself is produced by a string heuristic
that can invert an edge without any diagnostic. Second, we port the two-pass discipline behind an
opt-in interface with **no new package and no change to any existing controller** (one YAML line to
enable), and we show that the port's real price is not performance but a **finite set of runtime
admission checks** — membership, ordering, rate buckets, cross-mode edges and failure containment —
which we state, implement and test. Third, we extend the port to **heterogeneous update rates**: we
derive the exact attainable age of each edge as a function of the two rate factors, show that the
previously assumed "both ends must share a bucket" rule discards capability, and expose the result as
a **per-edge lag budget** that a deployment opts into. Every claim is checked against the real
`ControllerManager`, and on Gazebo physics the same step reaches the chain root in the **same cycle**
with two passes and one cycle per level late without them.

We also report what did not work: the two-pass ordering does **not** improve tracking error or
throughput, and several early framings of this work were withdrawn after a literature check (§8.4).

---

## 1. Introduction

### 1.1 The problem

Consider a two-level cascade: a mid-level controller computes a target for a leaf controller, and the
leaf drives a joint. The controller graph has two edges between the same pair of nodes:

* a **reference edge** — the parent writes the child's target, consumed by the child in the same cycle;
* a **state edge** — the child publishes its estimate (the joint's position it just read), consumed by
  the parent to compute the next target.

Both edges exist *between the same two controllers*, and in `ros2_control` both are expressed the same
way: the parent lists the child's exported interfaces among the command interfaces it claims. Nothing
in the interface description distinguishes "I write this" from "I read this". A single-pass scheduler
that walks one linearisation of the graph visits the pair in one order, so it satisfies one edge and
ages the other.

This is a textbook causality statement (a feedback loop with no delay register cannot be scheduled; see
§2.2). The interesting engineering question is what it costs *in a specific framework*, where the
schedule is not generated at compile time but produced by the framework at configuration time from a
controller list that users build at run time.

### 1.2 Why the static answer is not directly transferable

A recent firmware framework for tree-structured control resolves the tension with a **two-stage
decomposition over one linearisation**: every node's `update` stage runs child-first, then every node's
`handle` stage runs parent-first, within the same cycle (arXiv:2608.04600, §III-B Eq. (2); its
Corollary 2 shows this costs nothing in release waiting). The mechanism is therefore *known*. What is
not known is whether it survives contact with a framework whose controller set is:

* **loaded at run time** through `pluginlib`, by type string;
* **configured from YAML**, with parents named in parameters;
* **reorderable at run time** by load/configure/unload/switch operations;
* and whose controller list order is, in the framework's own words, a user-visible concern.

We show that the discipline does survive, and that the residue is exactly a **finite, enumerable set of
admission checks** that must be evaluated at run time, because they depend on the deployment and not on
the program.

### 1.3 Contributions

1. **An empirical characterisation of silent one-directional staleness** in unmodified `ros2_control`
   Humble, including the concrete mechanism by which the framework's ordering heuristic can *invert* an
   edge while every per-cycle call count stays correct (§3). No public analysis of this behaviour was
   found.
2. **A runtime port of the two-pass discipline** with no new package and no modification to existing
   controllers — an opt-in interface plus 1280 lines across three files in the manager, off by default,
   enabled by one YAML line — together with **the finite admission-check set** that the port requires,
   stated as five rejection codes with the user action each one implies (§4, §5). The production change
   is deliberately small: 22 files and +5102 lines in total, of which 3822 lines are tests, a runnable
   demo, a Gazebo demo and documentation.
3. **Exact per-edge bounds for heterogeneous rates**, and the resulting per-edge lag budget (§6). The
   ages are *attained*, not merely upper bounds, and they are verified two ways: a closed-form/simulation
   comparison for every rate pair in 1..12, and a measurement from the real manager for representative
   pairs, including a step-chain in which per-edge budgets do **not** bound the end-to-end age — a
   negative result about the budget's interpretation that we surface rather than hide.
4. **Two deployment findings that the framework's own abstractions hid** (§7.4): a chain root's
   reference is silently frozen under the new execution path because the framework refreshes references
   only inside the fused `update()`; and unloading a chainable controller does not remove the reference
   interfaces it exported, leaving a stale interface whose value pointer references the destroyed
   instance. The second is an upstream lifetime hole that we reproduce and pin with a UB-free test.

### 1.4 What this paper does **not** claim

* We do **not** claim the two-pass mechanism. It is FineMote §III-B Eq. (2); we cite it and align to it.
* We do **not** claim that maintaining one linearisation over the tree is new. Upstream `ros2_control`
  already derives and maintains one (§2.1).
* We do **not** claim an atomic all-or-nothing commit across the chain. Our minimal port has no such
  guarantee, and §5.3 states what it guarantees instead.
* We do **not** claim performance. The two passes reorder the same work; §7.2 reports the measured
  cost and §8.4 the negative results.

---

## 2. Background and related work

### 2.1 Chaining in `ros2_control`

A *chainable* controller in `ros2_control` exports **reference interfaces** (Humble:
`export_reference_interfaces()` returns command interfaces named `<controller>/<name>`, conventionally
`<controller>/target` and `<controller>/estimate`). A parent becomes a consumer by claiming those names
in its `command_interface_configuration()`; the convention is that the parent **writes** the child's
`target` and **reads** the child's `estimate`. The controller manager

* imports the child's exported interfaces when the child is configured,
* runs `controller_sorting()` on the controller list when a controller is configured, to keep parents
  before children, and
* during `update()` walks that list once and calls each controller's `update()`, which for a chainable
  controller first refreshes its reference (only when it is *not* in chained mode) and then computes and
  writes its commands.

The order is stable, not re-derived per cycle: a single linearisation, maintained with a
`std::stable_sort` at configuration time (upstream issue #853, closed 2023-08). The two-pass port adds a
second traversal over the same linearisation; it is the *second traversal*, not the linearisation, that
is the increment.

### 2.2 Causality and sampling

That a feedback edge without a delay register is not schedulable is standard in synchronous dataflow and
synchronous languages (Lee & Messerschmitt 1987; Lustre/Esterel causality). The cost of one cycle of
staleness in a sampled loop is classical sampled-control material (Franklin et al.; Åström & Wittenmark;
see also the networked-control formulation of Zhang, Branicky & Phillips 2001). We use these as
*background*, not as contributions; the contribution here is the connection of that general fact to a
specific scheduling variable in a widely used framework, and its measurement.

*Logical Execution Time* (Giotto; Kopetz & Bauer 2003) is the opposite trade: it makes timing
deterministic by *introducing* a delay and fixing the data visibility window. Our two passes remove a
delay for a specific edge kind; we say so explicitly rather than implying that eliminating staleness is
generally desirable.

End-to-end timing of cause–effect chains is analysed by Tindell & Clark (1994), Becker et al. (2017) and
Dürr et al. (2019). Our §6.4 negative result — a per-edge budget does not bound the chain — is the same
phenomenon those papers formalise for task chains; we report it because it is an easy misinterpretation
of our own API.

### 2.3 Two-phase *what*?

The term "two-phase commit" is taken (Gray & Lamport 2006) and means an atomic distributed transaction.
Our two passes are not a commit protocol: they are two traversals of one schedule within one cycle, with
**no** atomicity across controllers. We disambiguate the term on first use and again where the failure
semantics are defined (§5.3).

### 2.4 The static/compile-time answer

The framework in the positioning of §1.2 solves the problem at *build* time, where the controller set
and the topology are known to the generator. In `ros2_control`, the same information is available much
later: plugin names, YAML values, the URDF, the set of currently available interfaces, lifecycle states
and the controller list order are all determined by the deployment. §9 records what we found when we
asked which of these can be moved to compile time; per the scope of this paper that work is reported
separately and is not a contribution here.

---

## 3. The staleness, characterised on unmodified Humble

We use an unmodified `ros2_control` Humble checkout throughout this section. Nothing is patched; the
observations are of upstream behaviour.

### 3.1 The two edges are indistinguishable

A parent that writes `<child>/target` and reads `<child>/estimate` claims both names as command
interfaces. At the level where the manager schedules, they are indistinguishable; there is no
declaration that one is an output edge and the other an input edge. Consequently the scheduler cannot
know which edge a single-pass order is satisfying, and a configuration in which both exist has no
ordering that satisfies both — a fact the framework does not report.

### 3.2 One linearisation fixes one direction

Because the maintained order is parents-first, the parent executes before the child, so:

* the **reference** direction is fresh (the parent writes the child's target before the child reads it);
* the **state** direction is one cycle old (the parent reads the child's estimate before the child
  publishes this cycle's value).

The regression is silent: every controller runs exactly once per cycle, the cycle time is unchanged, and
no warning is emitted. On Humble the outcome is deterministically reference-first, not a trade-off
between two constraints.

### 3.3 The ordering heuristic can invert an edge

The maintained order is produced by `controller_sorting()`. Its first branch orders a controller that
claims *no* command interfaces before one that does. A chainable child that **exports a reference but
claims no command interface of its own** — a "pure estimator" or a leaf whose actuator interfaces are
declared elsewhere — therefore sorts **before its parent**, i.e. the edge runs backwards. Then both
directions are stale, and nothing rejects the configuration.

This is reproducible rather than inferential: in the port's test suite,
`an_order_that_inverts_an_edge_is_refused` first asserts that the manager really produces the inverted
order `[tp_leaf, tp_root]`, and then that enabling two-pass execution is refused with a message naming
the edge. It is also the reason the port *verifies* the order it inherits instead of trusting it (§5).

Two further facts about the list that matter for any port:

* the sort happens in `configure_controller()`; `add_controller()` does not sort, so the final order
  depends on the load/configure sequence and is not a pure function of the topology;
* consequently the order is part of the *deployment*, not of the program, which is why §5's checks are
  runtime checks and why §6's budget is a runtime argument.

### 3.4 Measured effect

In-process, three levels, equal rates, a unit step on the leaf's hardware input:

| quantity at the step cycle | single pass | two passes |
|---|---|---|
| leaf estimate | 1.00 | 1.00 |
| mid estimate | 0.00 | 0.50 |
| root estimate | 0.00 | 0.25 |
| leaf command | −1.00 | −1.75 |

(`two_pass_gives_the_parent_the_same_cycle_child_state`, `single_pass_command_uses_the_previous_cycles_reference`.)
The parent sees *nothing* in the cycle the leaf reports, and the larger magnitude of the leaf command
under two passes is the same information arriving earlier, not a different control law.

On Gazebo physics, with the real `gazebo_ros2_control/GazeboSystem` hardware and controllers loaded by
the real spawner over DDS (§7.3), the same step gives

| mode | leaf | mid | root |
|---|---|---|---|
| two passes | 0.7450, lag 0 | 0.3742, lag 0 | 0.1879, lag 0 |
| single pass | 0.7427, lag 0 | 0.3712, **lag 1** | 0.1864, **lag 2** |

with the control period measured from the messages' shared timestamps as **10.00 ms**. The values are
the ideal first-cycle fractions (0.75, 0.5·0.75, 0.25·0.75).

---

## 4. Design of the port

### 4.1 An opt-in interface, not a global change

The port adds one interface, `controller_interface::TwoPhaseControllerInterface`, with three methods:

| method | called | traversal |
|---|---|---|
| `update_phase()` | state pass | the controller list walked **backward** (children before parents) |
| `handle_phase()` | command pass | the same list walked **forward** (parents before children) |
| `refresh_reference_phase()` | command pass, before `handle_phase`, only for members that are **not** in chained mode | — |

`refresh_reference_phase()` exists because of a gap found in deployment (§7.4.1): upstream reaches a
chainable controller's reference refresh only inside the fused `update()`, which the two-pass path
bypasses *by design*, and the corresponding method is `protected`, so the manager cannot call it. The
default implementation does nothing, which is correct for a controller with no input topic and for every
member whose reference its parent writes.

A controller that does not implement the interface is untouched. When two-pass execution is disabled,
the manager never calls the phases and the controller's `update()` is used exactly as before. A member
that implements both entry points is still executed by exactly one path: while the mode is enabled, a
member is **never** handed to the native single-pass loop — not even in the cycles in which a switch is
pending and the passes are paused.

### 4.2 One immutable execution generation

Mode, member table, rate-bucket table, per-member activity and each member's chainable handle are
published as a **single immutable generation** that the real-time loop loads atomically. Membership is
rebuilt on every operation that changes the controller list (add, configure, unload, switch), on the
non-real-time thread, before the passes are released. A generation therefore never changes under a
running cycle.

Installing or widening the execution path requires the control loop to be idle; **removing** it
(`set_two_phase_execution(false)`) is always accepted. This is a deliberate, documented restriction:
the controller list itself is not part of the generation (§8.2).

### 4.3 Rate buckets

A member that declares an `update_rate` below the manager's is run once every
`factor = manager_rate / controller_rate` cycles with a matching period, exactly as the native loop
rate-gates it. The bucket loop runs in ascending factor order in both passes, so a bucket with a smaller
factor runs entirely before a bucket with a larger one; within a bucket the traversal direction favours
the child in the state pass and the parent in the command pass. §6 analyses what this costs.

### 4.4 Adoption

```yaml
controller_manager:
  ros__parameters:
    two_phase_execution: true
    # optional, default 0 = require same-cycle freshness on every edge
    two_phase_max_lag_cycles: 2
```

and, for a controller author, implementing the interface. The user-facing API mirrors the YAML:
`two_phase_rejections()` (ask before enabling), `set_two_phase_execution(enabled, max_lag_cycles)`,
`two_phase_execution()`, `two_phase_max_lag_cycles()`, `two_phase_edge_lags()`.

---

## 5. The price: a finite admission-check set

This is the paper's main systems claim: porting a build-time discipline to a run-time plugin set does
not cost correctness *conditions* that can be hidden in a code generator — it costs an explicit,
enumerable set of runtime checks. We state them as the rejection codes the manager can return.

| code | what it detects | why it is a runtime property | user action |
|---|---|---|---|
| `unsupported_update_rate` | a declared rate that is neither "follow the manager" nor an exact divisor of the manager's | the rate comes from YAML | round the rate, or follow the manager |
| `cross_rate_dependency` | a reference edge whose two ends fall in different rate buckets | depends on the deployed rates | co-locate the rates, or raise `two_phase_max_lag_cycles` (§6) |
| `cross_mode_dependency` | an edge with one end in two-pass execution and the other in the native loop, so the ends would be ordered by different schedules | depends on which controllers implement the interface *and* on the mode | implement the interface on both ends, or leave the mode off |
| `unschedulable_order` | the inherited controller-list order puts an edge's ends the wrong way round | the order is built by a configuration-time heuristic from string prefixes (§3.3) | give the child a command interface, or configure it before its parent |
| `duplicate_instance` | two names in the list refer to one controller object, so a pass would advance it twice per cycle | comes from how the deployment loads controllers | give each controller one name |

Two properties of this set are worth stating.

**It is closed under the mode.** Enabling is refused as a whole if any check fails; disabling is never
refused. There is no partially enabled state, and no check is "best effort".

**The checks are not removable by moving work to compile time.** Membership, ordering, available
interfaces and rates are determined by the deployment; a statically typed description of the controller
tree can *express* the requirements but cannot evaluate them, because the values do not exist at compile
time. This is the boundary that §9's separate work enumerates in detail.

### 5.3 What the port does *not* guarantee

There is no cross-controller rollback. `handle_phase()` writes directly into command interfaces, so a
failure late in the command pass leaves the earlier writes applied. What is guaranteed is the weakest
rule that keeps a controller from acting on data it just rejected:

* if **any** `update_phase()` fails in a cycle, **no** `handle_phase()` runs in that cycle; the command
  interfaces keep their previous values and the manager reports an error.

There is also no guarantee that the chain is *complete*: a controller that does not implement the
interface is scheduled by the native loop, which is precisely what the `cross_mode_dependency` check is
for. The contract is stated as a contract, tested as one
(`a_failed_state_stage_skips_the_whole_command_stage`,
`a_two_phase_member_is_never_handed_to_the_native_loop`), and its limits are listed in §8.2 rather than
left implicit.

---

## 6. Heterogeneous rates: exact ages and a per-edge budget

### 6.1 The rule that was assumed, and what it discards

The natural rule — "both ends of an edge must have the same update rate" — was the port's first
implementation, and it is stricter than necessary. Because the bucket loop runs in **ascending** factor
order in both passes, the relative order of two buckets is fixed by their factors, not by the list. That
lets an edge be *ordered* even when the two ends run at different rates; the price is a bounded, exactly
computable age instead of zero.

### 6.2 The bound

Let `f_P` be the parent's factor, `f_C` the child's, and `g = gcd(f_P, f_C)`. Ages are in manager
cycles, and they are **attained** in every `lcm(f_P, f_C)` period, not merely upper bounds.

| edge | `f_C > f_P` | `f_C = f_P` | `f_C < f_P` |
|---|---|---|---|
| state (child publishes → parent ingests) | `f_C` | 0 | `f_C − g` |
| reference (parent writes → child consumes) | `f_P − g` | 0 | `f_P` |

Two consequences:

* a **reference** edge is same-cycle fresh **iff `f_P` divides `f_C`**;
* a **state** edge is same-cycle fresh **iff `f_C` divides `f_P`**.

Note the counter-intuitive case: a slower, harmonically related child (`f_P | f_C`) gives a zero-age
reference edge but a state that is a whole child period old.

At the interface level a parent claiming `<child>/x` may be writing a reference *into* the child or
reading a state value the child publishes — the two look identical (§3.1). The admission therefore
charges **both** directions: it compares `max(state_age, reference_age)` against the declared budget, and
`two_phase_edge_lags()` reports all three values per edge.

### 6.3 Verification, twice

A derivation that only agrees with itself is worthless, so the bound is checked two independent ways.

**Closed form vs simulation.** `research/cross_rate_bound/cross_rate_lag.py` simulates the pass order
cycle by cycle in steady state and compares the observed worst age against the table for **every** rate
pair in `1..12`; any disagreement exits non-zero.

**Declared vs measured through the real manager.** The test suite measures the age of the value each end
*actually consumed*, through the real `ControllerManager`, using a cycle-stamp instrument on the
example controller, and asserts both `measured ≤ declared` and, for harmonic cases, equality:

| configuration | declared state / reference | measured state / reference |
|---|---|---|
| `f_P = 2`, `f_C = 1` (slower parent) | 0 / 2 | **0 / 2** |
| `f_P = 1`, `f_C = 2` (slower child) | 2 / 0 | **2 / 0** |
| `f_P = 2`, `f_C = 5` (non-harmonic) | 5 / 1 | refused at budget 4, admitted at 5 |
| chain leaf(4) ← mid(2) ← root(1) | per-edge 4 and 2 | end-to-end **6** = the sum |

The declared values are asserted separately from the measured ones, so the agreement of the two columns
is a result rather than a tautology.

### 6.4 The budget is per edge, and that is a trap

The last row is a negative result about the API's interpretation: a chain of cross-rate edges
**accumulates**, and a per-edge budget does **not** bound the end-to-end age. A deployment that sets
`two_phase_max_lag_cycles: 4` on the chain above gets 6 cycles of end-to-end age. We keep the budget
per-edge (it is the only place where the required information exists locally) and state the trap in the
documentation and in a test whose name says it:
`a_chain_accumulates_per_edge_lags_so_the_budget_is_not_end_to_end`.

### 6.5 Backwards compatibility

The default budget is `0`, which requires same-cycle freshness in both directions, i.e. one bucket per
edge — exactly the behaviour of a build without the budget. A deployment that does not opt in cannot
change behaviour by upgrading.

---

## 7. Evaluation

### 7.1 Correctness: the lag law

In-process, in the real manager, for chain depths 1 to 4 (`lag_equals_distance_from_the_leaf_with_one_pass_and_zero_with_two`):

* with one pass, a level's information is **distance from the leaf** cycles old — 0, 1, 2, 3;
* with two passes, **0 at every level**.

This turns the single depth-3 observation of §3.4 into the law it is an instance of. The measurement is
repeated on Gazebo physics (§7.3) with the same result.

### 7.2 Cost

Same manager, same three active members, 400 cycles, warmed up outside the measured window, values
published into the test report rather than asserted (`the_second_traversal_adds_no_allocation`):

| per cycle | single pass | two passes |
|---|---|---|
| heap allocations | 14 | **8** |
| wall time | 8–16 µs | **3 µs** |

Two honest readings. First, the second traversal adds **no** allocation of its own — the assertion the
test enforces is `two_passes ≤ single_pass`, and the two-pass path is in fact *cheaper* here because
members bypass the native loop's per-member lifecycle query, which on Humble copies a lifecycle state
whose label is a `std::string`. That comparison is how we found the extra allocation in the first place:
before caching member activity in the generation, the two-pass path allocated **26** per cycle against
the single pass's 14.

Second, these numbers are small in absolute terms: 3 µs against a 10 ms period is ~0.03% for a
three-member chain, and we do not present the two-pass path as a throughput optimisation. On an earlier,
heavier design in the same project (a whole-group kernel with per-cycle frames) the cost was ~2× CPU,
and no tracking-error benefit was found; see §8.4.

### 7.3 Deployment: real spawner, real YAML, real physics

The demo in the branch is launched with `ros2 launch` and consists of `gzserver` with
`gazebo_ros2_control/GazeboSystem`, `robot_state_publisher`, a `spawn_entity` call, and three chained
`spawner` invocations. In one run:

* the hardware is initialised, configured and activated by the resource manager;
* the manager reports `Two-phase execution requested by parameter: enabled` — the whole opt-in is one
  YAML line;
* all three controllers are loaded **by type string through `pluginlib`**, with their parameters
  arriving through the framework's real `<controller>.params_file` mechanism, and each reports
  `Configured and activated`;
* a step published on the chain root's reference topic produces the table in §3.4, measured from
  messages whose shared cycle stamps give a 10.00 ms period.

This is the same-cycle property reproduced on a system that has real physics, real DDS and real plugin
loading, rather than in-process with test doubles.

### 7.4 Two findings that deployment surfaced

Both were found because the port was exercised through its real deployment path, and both are
consequences of the framework's own abstractions rather than of the two-pass idea.

#### 7.4.1 A chain root's reference is silently frozen

Upstream refreshes a chainable controller's reference inputs in exactly one place:
`ChainableControllerInterface::update()`, which calls `update_reference_from_subscribers()` when the
controller is **not** in chained mode. The two-pass path bypasses `update()` by design, and the method is
`protected`, so the manager cannot call it. A chain **root** — a chainable controller nobody claims —
therefore held whatever reference it last saw and commanded it forever: no error, no refusal, no log. On
a real deployment the robot simply ignores its input.

Every in-process test missed it because they all set the reference through a test hook. Gazebo found it.
The fix completes the contract rather than patching a symptom: the contract gains the third step
(`refresh_reference_phase`, §4.1), which the manager calls for members that are not in chained mode,
immediately before that member's `handle_phase`. Two tests cover it, one per mode, and we verified they
are not vacuous: with the refresh call temporarily disabled, the single-pass test still passes and the
two-phase test fails with a value of 0.

#### 7.4.2 An upstream lifetime hole: unloading does not remove exported interfaces

`unload_controller()` does not remove the reference interfaces a chainable controller exported.
`ResourceManager::remove_controller_reference_interfaces()` exists and does exactly the right thing
(erase the registration, remove the stored interfaces), but **no production path calls it** — the only
caller in the tree is a test. The unload path carries the upstream TODO. Worse,
`add_command_interfaces()` inserts with `emplace`, which does not overwrite an existing key, so when a
controller is re-created under the same name the **new** interface is silently dropped and the **old**
one — whose value pointer referenced the destroyed instance — remains registered.

Measured consequence: a parent that claims `<child>/estimate` and is configured after the child was
unloaded and re-created under the same name reads freed memory. We observed a deterministic `1.833e-317`
one level up and `6.66698e-310` two levels up instead of the child's state; the identical configuration
with fresh names is correct, which isolates the cause to the name/system lifetime and not to scheduling.
There is no crash and no exception: the value is simply wrong, and looks like a control bug.

This affects **any** chainable chain, not the two-pass path; we report it because it is the same class of
problem the paper is about — a runtime, string-keyed interface model with a lifetime hole — and because
the port made it observable. A test pins the leak through the public API without dereferencing a stale
pointer, so a future upstream fix is detected rather than silently invalidating the finding.

### 7.5 Verification methodology, and three measurement bugs we made

We list these because each produced a plausible, wrong number, and because the same traps will catch
anyone measuring a scheduling change in this framework.

1. **A per-node cycle counter is not a common time base.** A member with factor `f` only advances on the
   cycles its bucket is due. Comparing nodes by their own counters gave a lag of `4294967293`. All
   cross-node measurements must carry a stamp supplied by the observer (or by the manager).
2. **An age is only defined on the cycles the consumer actually ran.** Sampling on an idle cycle
   measures "time since it last ran", which grows with the consumer's period and is not staleness. This
   produced a lag of 14 where the bound is 2.
3. **A stamp placed only in the new path makes the new path look unnecessary.** Our cycle stamp was first
   taken in `update_phase()`, which the fused single-pass path never calls, so a single-pass run
   published stamp 0 on every message and an observer aligning by stamp computed **lag 0 for every
   level** — a single pass appearing to have the two-pass property. Only after stamping the fused path
   did the two columns of §3.4 separate.

We also had to wait for the plant to settle before stepping: a baseline taken during the start-up
transient reported every level as moving on the same cycle.

---

## 8. Limitations and negative results

### 8.1 Platform

All measurements are on `ros2_control` Humble, on a 2-core machine with modest memory, and the Gazebo
numbers come from a headless `gzserver` on the same machine. The Gazebo stack aborted at controller
loading in roughly one launch in three with `free(): invalid pointer`; measurement runs were retried.
We report this because it bounds the confidence in any single Gazebo number, not because it affects the
in-process results. Two further environment facts: declaring a `velocity` command interface on this
Humble `GazeboSystem` aborts the same way (it is position-only in practice), and FIFO real-time
scheduling is unavailable in the container we used, so no claim here depends on RT priorities.

### 8.2 Scope of the execution generation

Mode, members, buckets and membership are one immutable generation; the **controller list is not**. A
list change is applied by the manager's own double-buffered publication, and installing the execution
path is only allowed while the control loop is idle. A port that wanted to install or widen the path
without stopping the loop would have to bring the list into the generation — a change to upstream's
publication protocol, which we deliberately did not make.

### 8.3 What is not analysed

No schedulability or worst-case-execution-time analysis is performed; `update_phase` and `handle_phase`
are assumed to have the same cost as the `update()` they replace, which §7.2 supports empirically for
our example controller but does not prove in general. The heap-allocation claim is about the passes
themselves, not about the manager's pre-existing per-cycle copies, which allocate in both modes.

### 8.4 Negative results

* **No performance benefit.** The two passes reorder the same work. In an earlier, heavier design from
  the same project (a staged whole-group kernel with per-cycle frames) the cost was ≈2× CPU, and no
  tracking-error improvement was found; that design is not part of this paper. What the port buys is the
  removal of a bounded, silent staleness, and nothing else.
* **A per-edge budget does not bound the chain** (§6.4).
* **Withdrawn claims.** Three framings of this work were withdrawn after the literature check: that the
  two-pass mechanism is novel (it is FineMote Eq. (2)); that maintaining one tree-derived linearisation
  is novel (upstream does it); and that a two-pass schedule is minimal in the number of passes. We state
  this because the surviving contribution is smaller than the original claim, and no reader should have
  to rediscover that from the citations.

---

## 9. Future work: where compile time stops

The port's admission checks exist because their inputs are determined by the **deployment**: plugin
names, YAML values, the URDF, the set of currently available interfaces, lifecycle states and the
controller list order. That raises a natural question — which parts of a controller description *can*
be determined by the **program**, before the binary exists? Our separate work enumerates the boundary
and reports three lands: a manifest invariant checked at the plugin entry point (measured compile cost
+8 ms, 0.08% on one TU), a leaf-set construction with a compile-time description of the tree, and an
equivalence check between the compile-time edge set and the edge set inferred at run time from interface
name strings. The conclusion is that everything a *type* determines uniquely is movable, and everything
a *deployment* determines uniquely is not — which is the same boundary that makes §5's checks
irreducible. We report it as future work here because the type-driven authoring path asks more of a user
than the runtime framework should, and because its evidence is currently single-machine compile-cost
data.

---

## 10. Conclusion

The two-pass execution discipline is known; this paper is about what it costs to move it into a
framework that loads its controllers at run time. The answer is not performance — the work is the same
and the measured cost is negligible — but a **finite set of runtime admission checks**: membership,
ordering, rate buckets, cross-mode edges and failure containment, each of which exists because its input
is determined by the deployment rather than by the program. Porting the discipline also revealed what
the framework's abstractions had hidden: one direction of every bidirectional edge is silently aged by
default; a chain root's reference can be silently frozen; and unloading a chainable controller leaves an
interface behind whose value pointer references a destroyed object. We state the checks, implement and
test them, quantify the exact cost of heterogeneous rates, and report the negative results alongside.

---

## Appendix A. Reproduction

```bash
# build the two packages the branch touches
colcon build --packages-select controller_interface controller_manager

# 31 functional tests, including the numeric single-pass/two-pass comparison
./build/controller_manager/test_two_phase_execution

# 4 evidence tests: lag-vs-depth, cost, the real load path, and the upstream lifetime hole
./build/controller_manager/test_two_phase_evidence

# the closed-form cross-rate model, checked against simulation for every pair in 1..12
python3 research/cross_rate_bound/cross_rate_lag.py

# real system: real spawner, real YAML, Gazebo physics
ros2 launch controller_manager two_phase_demo_gazebo.launch.py
python3 <share>/controller_manager/two_phase_demo/record_chain_lag.py true|false
```

## Appendix B. Test-to-claim map

| claim | test |
|---|---|
| single pass ages state by distance from the leaf; two passes 0 | `lag_equals_distance_from_the_leaf_with_one_pass_and_zero_with_two` |
| the inherited order can invert an edge, and is then refused | `an_order_that_inverts_an_edge_is_refused` |
| a member is never executed by the native loop | `a_two_phase_member_is_never_handed_to_the_native_loop` |
| a failed state pass skips the whole command pass | `a_failed_state_stage_skips_the_whole_command_stage` |
| the second traversal adds no allocation | `the_second_traversal_adds_no_allocation` |
| the exact per-edge ages, measured | `the_measured_lag_matches_the_declared_bound_for_a_slower_parent` / `..._child` |
| a non-harmonic edge needs its exact worst age | `a_non_harmonic_edge_needs_its_exact_worst_lag` |
| the budget is per edge, not end-to-end | `a_chain_accumulates_per_edge_lags_so_the_budget_is_not_end_to_end` |
| a chain root follows its reference topic in both modes | `a_root_follows_its_reference_topic_with_native_single_pass` / `..._in_two_phase_mode` |
| unloading leaves exported reference interfaces behind | `unloading_a_chainable_controller_leaves_its_reference_interfaces_behind` |
| the demo's YAML, URDF and plugin agree through the real load path | `demo_yaml_urdf_and_plugin_agree_through_the_real_load_path` |

---

## References

`TODO(author)`: fill from `LITERATURE_SURVEY_2026-10.md` §4, which carries verified DOIs. The set this
paper actually needs:

1. Xi et al. *FineMote* (arXiv:2608.04600) — the anchor; §III-B Eq. (2), §IV-C2 Corollary 2.
2. `ros2_control` issue #853 — upstream's maintained linearisation.
3. `ros2_control` chaining documentation and issue #2189 — chain direction and the framework's own
   framing of the list order as user-visible.
4. Lee & Messerschmitt 1987, *Synchronous data flow* — causality of undelayed feedback. DOI 10.1109/TC.1987.5009446
5. Zhang, Branicky & Phillips 2001 — networked control stability. DOI 10.1109/37.898794
6. Tindell & Clark 1994 — holistic latency. DOI 10.1016/0165-6074(94)90080-9
7. Becker et al. 2017 — cause–effect chain data age. DOI 10.1016/j.sysarc.2017.09.004
8. Dürr et al. 2019 — sporadic cause–effect chains. DOI 10.1145/3358181
9. Henzinger et al. 2001 (Giotto), Kopetz & Bauer 2003 (LET) — the opposite trade.
10. Gray & Lamport 2006 — two-phase *commit*, cited only to disambiguate. DOI 10.1145/1132863.1132867
11. Franklin, Powell & Emami-Naeini; Åström & Wittenmark — sampled-control background.
12. Tarjan 1972 (DOI 10.1137/0201010) — postorder/reverse-postorder as the ordering tool.
