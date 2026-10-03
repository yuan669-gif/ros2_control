# Two-phase controller execution

This document describes the opt-in **two-phase execution** path of the `ControllerManager`. It is
**off by default**: a configuration that never sets `two_phase_execution` behaves exactly as
upstream `ros2_control` does.

## 1. The problem

Controllers form a tree through *chaining*: state information is aggregated from the leaves towards
the root, while decisions (references) are propagated from the root back to the leaves. The manager
runs every controller exactly once per cycle, in one linear order. Therefore a parent/child pair that
has **both** a state edge and a reference edge cannot have both directions fresh in the same cycle:
whichever controller runs first, the other one reads the previous cycle's value of that edge. The
staleness is silent — every counter still reports one update per controller per cycle.

## 2. The rule

A controller that implements `controller_interface::TwoPhaseControllerInterface` splits its cycle into
two explicit stages:

```cpp
controller_interface::return_type update_phase(time, period) noexcept;  // ingest state
controller_interface::return_type handle_phase(time, period) noexcept;  // produce commands
```

When `two_phase_execution` is enabled, the manager walks **one** ordered controller list twice:

| pass | direction | contract |
|---|---|---|
| `update_phase` | reverse (children before parents) | a parent ingests the child's state **published this cycle** |
| `handle_phase` | forward (parents before children) | a child consumes the parent's reference **produced this cycle** |

Because the two passes traverse the same linearization in opposite directions, one order satisfies
both dependency directions at once — no second topological order is needed. A controller that
implements the interface is **never** executed by the native single-pass loop while the mode is on,
not even in the cycles in which a switch is pending and the passes are paused.

## 3. Writing a controller

Implement both the native interface (as you already do) and the two stage entry points:

* `update_phase()`: ingest the hardware state and/or the children's published state, update internal
  state, and publish what your parent needs.
* `handle_phase()`: compute this cycle's outputs from the state that `update_phase()` just produced,
  write the hardware command interfaces and the children's references.
* the native `update()` stays the fused single pass, used when the mode is off.

A worked example is built and installed with `controller_manager`:
`controller_manager/two_phase_example_controller`
(`test/two_phase_example_controller/two_phase_example_controller.{hpp,cpp}`).

### Publishing state to a parent on Humble

Humble's `ChainableControllerInterface` can only export **command** interfaces
(`on_export_reference_interfaces`). The example therefore gives every node two exported reference
channels — `<node>/target` (what the parent writes) and `<node>/estimate` (what this node publishes)
— and the parent reads `<child>/estimate` through the loaned command interface. On Jazzy and later the
same channel is an exported **state** interface (`on_export_state_interfaces`), which is the
upstream-blessed form; the scheduling question is identical.

## 4. Enabling it

From YAML, in the `controller_manager` node section:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    two_phase_execution: true
```

Or at run time:

```cpp
manager->set_two_phase_execution(true);      // all-or-nothing
manager->two_phase_execution();              // current mode
manager->two_phase_rejected_controllers();   // who cannot join, and why (works before enabling)
manager->control_loop_busy();                // true while a cycle is in flight
```

`set_two_phase_execution(true)` is **refused** — and changes nothing — when any controller that
implements the interface cannot join the path. It is also refused while a control cycle is in flight,
because the admission decision is taken against the controller list, which is published separately
from the execution state. Removing the path (`set_two_phase_execution(false)`) is always accepted: a
cycle that is already running holds its own snapshot and finishes with it.

## 5. Admission rules

| rejection | meaning |
|---|---|
| `unsupported_update_rate` | the declared `update_rate` is neither "follow the manager" nor an exact divisor of it, so no rate bucket can run it at the rate it asked for |
| `cross_rate_dependency` | a reference edge whose two ends are in different rate buckets; the two schedules have no fixed relation, so the edge would silently age |
| `cross_mode_dependency` | a reference edge between a two-phase member and a controller that does **not** implement the interface; the two ends would be ordered by different schedules |
| `unschedulable_order` | the manager's controller list puts a parent **after** its child, so both passes would walk that edge in the wrong direction |
| `duplicate_instance` | two names in the controller list refer to one controller object; a pass would advance it twice per cycle |

Rate buckets: a controller whose `update_rate` divides the manager's runs in its own bucket, once
every `manager_rate / controller_rate` cycles, and receives the **bucket's** period. Reference edges
are only admitted **within** one bucket.

## 6. Failure semantics

There is **no** cross-controller rollback: `handle_phase()` writes straight into the controller's
command interfaces, so a failure late in the command pass leaves the earlier writes applied. What is
guaranteed is the weakest rule that keeps a controller from acting on data it just rejected:

* if **any** `update_phase()` fails in a cycle, **no** `handle_phase()` runs in that cycle; the
  command interfaces keep the previous cycle's values and `update()` returns `ERROR`.

## 7. Demo

```bash
ros2 launch controller_manager two_phase_demo.launch.py
ros2 control list_controllers -v
```

The demo brings up an in-memory two-joint system and a three-level cascade
(`tp_root -> tp_mid -> tp_leaf`) of `controller_manager/two_phase_example_controller`.

## 8. Limitations

* No whole-group atomic commit, no per-cycle frame semantics, no rollback of already-written commands.
* A reference edge across rate buckets is **refused**, not bounded.
* Mode and membership are one execution generation, but the **controller list** is not: installing an
  execution path requires the control loop to be stopped.
* This path is a manager-level feature only; it does not include the compile-time topology layer of
  the research branch.
