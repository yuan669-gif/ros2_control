# Upstream finding: a cyclic chain overflows the stack while sorting controllers

Date: 2026-10-03　Found on: `ros2_control` Humble, branch `feature/two-phase-manager`, while writing the
test for the two-phase admission's cycle verdict.

**This is NOT a two-phase execution defect.** It is an upstream robustness hole in the same family as the
unload leak (`upstream_finding_stale_reference_interfaces.md`): the runtime, string-keyed interface model
has no structural validation, and the failure is a crash rather than a diagnosis.

---

## 1. What happens

Two chainable controllers that claim each other's reference interfaces — `A` writes `<B>/target` and
reads `<B>/estimate`, `B` writes `<A>/target` and reads `<A>/estimate` — are configured normally. The
second `configure_controller()` call then **segfaults the process with a stack overflow**.

Reproduction: add two such controllers (`children` naming each other), then configure both. The crash is
in `configure_controller()`, before anything the two-phase feature does.

## 2. Why

`configure_controller()` calls `controller_sorting()`, which orders the list by asking, for each
controller, which controllers come *after* it. `get_following_controller_names()` does that by recursing:

```cpp
following_controllers.push_back(ctrl_it->info.name);
const std::vector<std::string> ctrl_names =
  get_following_controller_names(ctrl_it->info.name, controllers);   // no visited set
```

With a cycle, that recursion never terminates. The crash was observed as an unbounded stack of identical
frames:

```
#9 ... in (anonymous namespace)::get_following_controller_names(...)
#8 ... in (anonymous namespace)::get_following_controller_names(...)
...
#3 ... in command_interface_is_reference_interface_of_controller(...)
Segmentation fault (Address not mapped to object)
```

The function does guard against *unconfigured* controllers (it returns empty unless the controller is
active or inactive), so the cycle must be configured, not merely added — which is exactly the ordinary
sequence a deployment performs.

## 3. Why it matters

1. **It is a crash, not a refusal**, in a function that runs on every configuration operation.
2. **A cyclic claim is easy to write by accident** — a naming mistake in two `children` lists is enough,
   and nothing warns earlier: each controller's own declaration is perfectly valid.
3. **It hides a second problem**: because upstream dies during configuration, no admission check that a
   framework adds afterwards can ever diagnose the cycle. The two-phase port has a `cyclic_dependency`
   verdict for exactly this graph, and it can only be exercised on a list whose cycles were never
   configured (`test_two_phase_execution.cpp`, `a_cyclic_reference_graph_is_refused`).

## 4. Suggested fix

Make the traversal total: carry a visited set (or an explicit worklist) through
`get_following_controller_names()`, and report the cycle instead of recursing into it. A cycle in the
controller graph is not schedulable in any case — the two-phase path refuses it with `cyclic_dependency`
— so detecting and reporting it at configuration time is strictly better than not finishing the call.

## 5. A cycle cannot be activated either, which is why the port's verdict is defensive

Presenting a cycle to the admission needs the graph to exist as a **declaration**: reference interfaces
become *available* only when their owner is active (`make_controller_reference_interfaces_available`, at
activation), so neither end of a cycle can ever claim the other. Activation is therefore impossible for
a cyclic chain, and configuration crashes before reaching it. Together that means upstream has no way to
present the graph to any later check, and the two-phase port's `cyclic_dependency` verdict is defensive
by construction — correct for the graph, but unreachable until the traversal is made total.

The test that covers it (`a_cyclic_reference_graph_is_refused`) therefore configures both controllers
with no children, introduces the cycle, and asks: the admission judges the declaration of a
configured-but-inactive controller, which is exactly the set the graph becomes visible in.

## 6. Status

Documented, not fixed: it is upstream traversal code, and changing it affects ordering for every
configuration, which is a decision for the upstream maintainers. The port's own verdict on the same
graph is implemented and tested.
