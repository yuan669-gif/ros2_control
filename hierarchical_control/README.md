# hierarchical_control

Standalone, host-agnostic two-phase execution kernel for ROS 2 control, inspired by the
bidirectional device scheduling of the FineMote embedded middleware.

A hierarchical controller tree needs two opposite data directions inside one control cycle:

```text
state  : child  -> parent   (a parent consumes child-derived estimates)
command: parent -> child    (a parent produces child references)
```

A single indivisible `update()` cannot satisfy both directions. This package makes the two phases
explicit, attaches per-cycle validity metadata, and commits actuator commands only after the whole
tree succeeded.

## Cycle order

```text
state phase   : postorder (children before parents)
root input    : one external reference snapshot for the root
command phase : preorder  (parents before children)
validation    : cycle identity, sample age, finiteness
commit        : leaf scratch copied into controller-owned command sinks
```

## Data contract

Each node publishes state and consumes one reference. Both carry provenance:

```cpp
struct StagedFrame {
  std::uint64_t cycle;      // kernel cycle that produced the value
  std::int64_t  sample_ns;  // oldest source sample time used
  bool          valid;      // true only after this cycle's stage succeeded
  std::uint32_t fault_code; // node-defined, 0 = none
};
```

Rules enforced by the kernel, not by convention:

- A **leaf** reports the hardware sample time (default: this cycle). It may report an older time,
  never a future one.
- A **composite** node's `sample_ns` is overwritten with the **oldest child sample time**, so a
  parent can never re-stamp derived data as fresher than its inputs.
- Values must be finite. A non-finite state, reference or actuator fails the cycle.
- A failed cycle calls **no** command sink, so actuator buffers never hold a mixture of old and new
  values.

Keeping the previous command is *not* a hardware safety policy. The application must inspect the
result and invoke its own fault action. A sink that fails during `commit()` is a hardware fault
domain and is reported, not rolled back.

## Two hosts

The kernel is independent of `controller_manager`. Configuration-time work (topology resolution,
validation, storage) and the run path are the same for both hosts.

### 1. Manager host

Every node is a normal controller plugin that implements `StagedControllerInterface`; the manager
runs the group and skips its members in the native `update()` loop:

```cpp
auto group = hierarchical_control::StagedExecutionGroup::create(members, max_age_ns);
```

Command edges are derived from the parent's claimed reference interfaces
(`<child>/<port>`), so the native Humble ordering machinery stays authoritative.

### 2. Library host

One ordinary controller plugin hosts the whole tree; no chainable interfaces, no reference
interfaces, no `controller_manager` change:

```cpp
hierarchical_control::StagedExecutionGroup::Spec spec{names, instances, parents};
auto kernel = hierarchical_control::StagedExecutionGroup::create_library(spec, max_age_ns);
// ... in update(): kernel->run(time, period);
```

Both hosts make "add one node" a configuration change, and both keep the run path allocation-free
after configuration.

## Declaring ports once (compile-time layer)

`StagedControllerInterface` declares its ports as runtime strings. `typed_ports.hpp` lets a
controller declare them **once, as types**, and generates those strings from the declaration, so the
two cannot drift:

```cpp
using wheel_ports = TypedPorts<
  PortList<wheel_travel>,    // state this node publishes (its parent's state stage reads it)
  PortList<wheel_target>,    // reference this node receives (its parent's command stage writes it)
  PortList<wheel_torque>,    // actuator ports this node writes
  PortList<tire_target>,     // reference this node writes into its children  (reference edge)
  PortList<tire_travel>>;    // state this node reads from its children       (state edge)

class WheelController : public TypedPortsMixin<WheelController, wheel_ports> { ... };
```

The kernel only uses the **lengths** of those three lists (it sizes per-node buffers from them); the
names are checked separately by `verify_ports_match_interface()` / `verify_ports_match_contract()`,
and `topology_binding::verify_binding_ports()` walks a whole checked binding at start-up.
Both edges of a parent/child pair are checked -- by name, order **and physical dimension**:

```cpp
reference_declarations_agree<Parent, Child>()   // Parent::for_children vs Child::reference
state_declarations_agree<Parent, Child>()       // Parent::child_state  vs Child::state
declarations_are_compatible<Parent, Child>()    // both, and each can be asserted separately
```

See `doc/PORT_DIMENSIONS.md` and `doc/TOPOLOGY_CONTRACT_JOIN.md`.

## Configuration-time validation

`create()` / `create_library()` reject, before entering the real-time loop:

- empty or duplicate node names, inconsistent spec sizes;
- unknown parent, multiple roots, cycles, self-parent;
- a node that declares actuator ports but has no command sink;
- a root that declares reference ports but has no reference source;
- (manager host) a reference interface with more than one writer, derived from claimed interfaces.

## Non-goals and limitations

- No multi-rate, asynchronous callbacks, dynamic topology changes or multi-tree support.
- No lifecycle rollback protocol and no hardware fault action.
- A software commit is not bus-level atomicity or simultaneous physical motion.
- **The staged group and the manager-level two-phase path do NOT have the same guarantees.** The
  staged group validates a whole cycle and commits all-or-nothing. The two-phase path
  (`TwoPhaseControllerInterface`) is deliberately lighter: it has no per-cycle frames and no group
  commit, so a failure late in its command pass leaves the earlier writes applied. Its only
  containment rule is that a failed **state** stage suppresses the command stage for that cycle. If
  you need atomic commit, use the staged group.
- Not a real-time WCET proof. Measured overhead on a non-realtime VM was roughly 2x a single
  handwritten composite plugin for the same synthetic algorithm.
- The manager host caches member activity and refreshes it only after a switch; a controller that
  changes lifecycle state outside a switch would leave the cache stale.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hierarchical_control
colcon test  --packages-select hierarchical_control --output-on-failure
```

The unit tests use plain node stubs and do not create a `ControllerManager`.

## Packages

| package | role |
|---|---|
| `hierarchical_control` | this kernel and the opt-in controller interface |
| `controller_manager` | optional thin adapter: opt-in group API, member skipping, activity refresh |
