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
