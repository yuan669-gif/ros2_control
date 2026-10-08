# Upstream finding: unloading a chainable controller leaks its reference interfaces

Date: 2026-10-03　Found on: `ros2_control` Humble, branch `feature/two-phase-manager`
(base `469f305`), while building the evidence harness
`controller_manager/test/test_two_phase_evidence.cpp`.

**This is NOT a two-phase execution defect.** It is an upstream lifetime hole in the
controller/resource-manager interface model that the two-phase work happened to run into, and it
affects **any** chainable controller chain. It is documented here because (a) it is reproducible and
pinned by a test, (b) it is memory-safety adjacent, and (c) it is exactly the class of problem a
type-checked description layer would not have.

---

## 1. Summary

`ControllerManager::unload_controller()` does **not** remove the reference interfaces a chainable
controller exported. They stay in the `ResourceManager` — together with their value pointers, which
referenced the controller instance that has just been destroyed.

## 2. Evidence in upstream code

| Fact | Location |
|---|---|
| `unload_controller()` carries the explicit upstream TODO | `controller_manager/src/controller_manager.cpp`: `// TODO(destogl): remove reference interface if chainable; i.e., add a separate method for cleaning-up controllers?` (twice in the unload path) |
| The cleanup function EXISTS | `ResourceManager::remove_controller_reference_interfaces()` (`hardware_interface/src/resource_manager.cpp`) erases the map entry and calls `remove_command_interfaces()` |
| **Nothing in production calls it** | the only caller in the tree is `hardware_interface_testing/test/test_resource_manager.cpp` |
| Deactivation only makes them UNAVAILABLE, it does not remove them | `make_controller_reference_interfaces_unavailable()` is called on deactivate (`controller_manager.cpp:998`); the map entry and the stored `CommandInterface` objects survive |
| Re-importing the same name does **not** replace the old interface | `add_command_interfaces()` uses `command_interface_map_.emplace(...)`, and `emplace` **does not overwrite** an existing key, so the NEW interface is silently dropped and the OLD one — whose value pointer dangles — stays in the map |

## 3. Consequence, measured

A parent that claims `<child>/estimate` and is configured **after** the child was unloaded and
re-created under the same name loans the **stale** interface and reads freed memory. Measured in the
depth sweep while it still reused names:

* parent one level up read `1.833e-317` instead of the child's state;
* one level further up: `6.66698e-310` — both deterministic across runs, the signature of reading
  freed heap rather than a random value;
* with a **fresh name** per iteration the identical configuration is correct (the depth test passes
  for depths 1..4), which isolates the cause to the name/system lifetime, not to the scheduling.

No crash and no exception: the value is simply wrong, and it looks like a control bug.

## 4. Why it matters

1. **It is reachable through a normal workflow.** Unloading and re-loading controllers is what
   `spawner`/`unspawner` do; a running system that reloads a chainable controller while a parent
   exists is enough.
2. **It is silent.** The consumer reads a plausible-looking number from freed memory; nothing
   reports a problem.
3. **It is not specific to the two-phase feature.** Any chainable controller pair can hit it; the
   two-phase path only made it easier to observe because it deliberately drives the same channel
   every cycle.
4. **It is a lifetime problem, not an ordering problem** — which is why the ordering discussion in
   `SCHEDULING_ORDER_ANALYSIS.md` does not cover it.

## 5. Reproduction

```bash
# after building this branch
./build/controller_manager/test_two_phase_evidence \
  --gtest_filter='*unloading_a_chainable_controller*'
```

The test is deliberately UB-free: it asserts through the public API that the resource manager still
registers the unloaded controller's reference interface names
(`get_controller_reference_interface_names()`), and never dereferences a stale pointer. If upstream
adds the missing `remove_controller_reference_interfaces()` call, **this test must start failing** —
that is the signal to delete the finding, not to weaken the test.

## 6. Suggested fix

In `unload_controller()`, after cleanup and before the controller instance is destroyed:

```cpp
if (controller.c->is_chainable())
{
  resource_manager_->remove_controller_reference_interfaces(controller_name);
}
```

(the function already does the right thing: erase the map entry and remove the stored interfaces).
`add_command_interfaces()` should probably also refuse — rather than silently drop — a duplicate
name, since today the new interface never reaches the map.

## 7. Status

Reported here for the branch; **not** fixed by this work, because it is an upstream behaviour change
that needs its own review (it touches unloading semantics for every chainable controller). The test
above pins the current behaviour so it cannot regress unnoticed and so a future fix is detected.
