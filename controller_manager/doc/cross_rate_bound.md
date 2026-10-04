# Cross-rate bucket lag and the lag budget

This document answers one question: an edge whose two ends run at **different** rates cannot be
served in the same cycle by one pair of passes — but exactly **how stale** does it get, and can that
be bounded instead of refused?

Short answer: yes, the worst case is **exact** (not a loose bound), it is **attained** by the
schedule, and it is what `two_phase_max_lag_cycles` is compared against. The default budget of `0`
reproduces the strict "both ends in one bucket" rule, so nothing changes for a configuration that
does not opt in.

## 1. The schedule being analysed

`ControllerManager::update()` runs, per cycle:

```cpp
for (const auto factor : buckets) {          // ASCENDING
    if (update_loop_counter_ % factor) continue;
    // state pass: walk the controller list BACKWARD  -> within a bucket, child before parent
}
for (const auto factor : buckets) {          // ASCENDING
    if (update_loop_counter_ % factor) continue;
    // command pass: walk the controller list FORWARD -> within a bucket, parent before child
}
```

Two facts about the schedule drive everything:

* the bucket loop is **ascending in both passes**, so a bucket with a *smaller* factor runs entirely
  before a bucket with a larger one;
* within one bucket the walk direction favours the child in the state pass and the parent in the
  command pass.

Two assumptions, both made true by the implementation rather than assumed away:

| assumption | why it holds |
|---|---|
| the list is parents-first for the edge in question | the admission refuses `unschedulable_order` |
| a member with factor `f` is due exactly on the multiples of `f` | admission requires `update_rate_ % controller_rate == 0`, so `factor = update_rate_ / controller_rate` **divides** `update_rate_` and the counter's modulo wrap never perturbs the due set |

The second one matters: without it, the wrap in `(update_loop_counter_ + 1) % update_rate_` would make
a factor that does not divide the manager's rate due on two *adjacent* cycles (e.g. rate 100,
factor 3 → cycles 99 and 0), and the periodicity the derivation relies on would not hold.

## 2. The bound

Let `f_P` be the parent's factor, `f_C` the child's, and `g = gcd(f_P, f_C)`. Ages are in **manager
cycles**.

| edge direction | `f_C > f_P` | `f_C == f_P` | `f_C < f_P` |
|---|---|---|---|
| state (child publishes → parent ingests) | `f_C` | `0` | `f_C − g` |
| reference (parent writes → child consumes) | `f_P − g` | `0` | `f_P` |

Derivation:

* **state.** When `f_C < f_P` the child's bucket runs first, so a coincident cycle is same-cycle; on
  the parent's other cycles it reads the child's most recent publish, and the largest offset a parent
  tick can have below a child tick grid is `f_C − g` (0 when `f_C | f_P`). When `f_C > f_P` the
  parent's bucket always runs first, so even a coincident cycle costs the child's whole period `f_C`.
* **reference.** Mirror image, but with the opposite within-bucket direction, so "same cycle" requires
  the *parent's* bucket to be the smaller one: `f_P − g` when `f_P < f_C`, and `f_P` when `f_P > f_C`.
* **attainment.** The due sets and the traversal order repeat with period `lcm(f_P, f_C)`, so each
  worst case occurs in every period — these are exact, not upper bounds.

Two consequences worth stating on their own:

* a reference edge is same-cycle fresh **iff `f_P | f_C`**;
* a state edge is same-cycle fresh **iff `f_C | f_P`**.

So the previous rule — *both ends must share a bucket* — discarded capability: when the parent is at
least as fast and the periods are harmonic, the reference direction was already zero-lag.

Note the counter-intuitive case: `f_P | f_C` (a slower, harmonically related child) gives a **zero-lag
reference** but a state that is a whole child period old.

## 3. Why the admission charges BOTH directions

A parent claiming `<child>/x` may be writing a reference *into* the child or reading a state value the
child publishes — at the interface level the two look identical. The admission therefore compares
`max(state_lag, reference_lag)` with the budget, so neither direction can exceed what the
configuration declared acceptable. `two_phase_edge_lags()` reports all three values per edge.

## 4. How this is verified

Two independent checks, because a derivation that only agrees with itself is worthless:

1. **model vs closed form** — `research/cross_rate_bound/cross_rate_lag.py` (repository root) simulates
   the pass order above cycle by cycle in steady state and compares the observed worst age with the
   table for **every** `(f_P, f_C)` in `1..12`; any disagreement exits non-zero. The script is the
   model, and the model is the only thing it proves.
2. **manager vs declared bound** — `controller_manager/test/test_two_phase_execution.cpp` measures the
   age of the value each end **actually consumed**, through the real `ControllerManager`, using the
   example controller's cycle-stamp instrument (`set_cycle_stamp_mode`), and asserts both
   `measured ≤ declared` and, for the harmonic cases, equality with the exact value:

   | configuration | declared state / reference | measured state / reference | assertion |
   |---|---|---|---|
   | `f_P = 2`, `f_C = 1` (slower parent) | 0 / 2 | **0 / 2** | `the_measured_lag_matches_the_declared_bound_for_a_slower_parent` |
   | `f_P = 1`, `f_C = 2` (slower child) | 2 / 0 | **2 / 0** | `..._for_a_slower_child` |
   | `f_P = 2`, `f_C = 5` (non-harmonic) | 5 / 1 | refused at budget 4, admitted at 5 | `a_non_harmonic_edge_needs_its_exact_worst_lag` |

   The measured values are READ from the manager, and the declared values are asserted separately, so
   the two columns agreeing is a result rather than a tautology. Two measurement mistakes were found
   and fixed while doing this, and both are worth knowing when reading any such number:

   * a node only advances on the cycles its own bucket is due, so a per-node counter is NOT a common
     time base — the stamp has to be supplied by the observer;
   * an age is only defined on the cycles the CONSUMER actually ran; sampling on an idle cycle
     measures "time since it last ran", which grows with the consumer's period and is not staleness.
     Both mistakes produced plausible-looking but wrong numbers (a lag of 14 where the bound is 2),
     which is exactly why the derivation is not accepted until the manager confirms it.

## 5. Using it

```cpp
manager->two_phase_edge_lags();                  // exact lag of every edge, before enabling
manager->set_two_phase_execution(true, /*max_lag_cycles=*/2);
manager->two_phase_max_lag_cycles();
```

```yaml
controller_manager:
  ros__parameters:
    two_phase_execution: true
    two_phase_max_lag_cycles: 2
```

A configuration that stays at the default `0` behaves exactly as a build without the budget: the
admission requires same-cycle freshness in both directions, i.e. one bucket per edge.

## 6. What is still NOT provided

* The budget is expressed in **manager cycles**, not in time, and is not converted to a control-level
  quantity (phase margin, sampling delay in ms). For a paper that comparison has to be made
  explicitly against the manager period.
* Multi-edge interactions are not modelled: each edge is charged independently, which is correct for
  the staleness of a single value but says nothing about error accumulation along a chain of
  cross-rate edges.
* A chain of cross-rate edges can therefore accumulate `sum` of the per-edge lags; the budget is
  per-edge, **not** end-to-end.
