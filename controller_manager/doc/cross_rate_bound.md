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
   | chain leaf 4 <- mid 2 <- root 1 | per-edge 4 and 2 | end-to-end **6** (= the sum) | `a_chain_accumulates_per_edge_lags_so_the_budget_is_not_end_to_end` |

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

## 6. End-to-end: a chain accumulates, the budget does not

The budget is **per edge**. A chain of cross-rate state edges accumulates, and the per-edge budget
does not bound the result. The honest statement, and the one the tests check:

> For a state path `leaf -> ... -> root` in which every node **republishes what it ingested in the
> same state stage**, the age of the leaf's value at the root is at most the **SUM** of the per-edge
> state lags.

Why: the root reads the intermediate node's most recent publish, whose age is bounded by that edge's
lag, and the value that publish carried was itself bounded by the lag below it. The assumed
"republish in the same state stage" is what the example controller does in stamp mode; a controller
that published in a *different* stage would need its own accounting.

The sum is an upper bound because the per-edge maxima need not coincide — but they can, and in the
measured configuration they do:

| configuration | per-edge state lags | sum | measured end-to-end |
|---|---|---|---|
| leaf factor 4, mid factor 2, root factor 1 | 4 (mid←leaf) and 2 (root←mid) | 6 | **6** |

`a_chain_accumulates_per_edge_lags_so_the_budget_is_not_end_to_end` asserts `measured ≤ sum`,
`measured > the worst single edge` (so the accumulation is real, not incidental) and, for this
configuration, the exact value 6. With a per-edge budget of 4 the chain is admitted, and the value at
the root can then be **6** manager cycles old — 1.5 times the number the budget states. That gap is
the reason a per-edge budget must not be presented as a system-level guarantee.

## 7. Time, and a control quantity

The schedule's natural unit is the manager cycle, but a control argument needs time. Every reported
lag therefore carries both:

```cpp
manager->two_phase_edge_lags();   // ..._lag_cycles AND ..._lag_ns
```

`lag_ns = lag_cycles * 1e9 / update_rate`, so at a 100 Hz manager period a 2-cycle lag is 20 ms.
Feeding that into the framework's existing cost law (`doc/CONTROL_COST_OF_LAG.md` in the research
line, `ΔPM = 360 · f_c · Δt`) turns a budget into a phase-margin loss at a given crossover frequency
`f_c` — e.g. 6 cycles at 100 Hz is 60 ms.

Still **not** provided: the conversion is exposed, not applied. Nothing in the manager checks a
phase-margin or bandwidth requirement, and the cost law's applicability is a control-engineering
statement, not a scheduling one.

## 8. What is still NOT provided

* The budget is **per edge, not end-to-end** — see §6, which derives the sum bound, measures it, and
  shows the chain reaching 1.5x the admitted per-edge budget.
* The time conversion is exposed (§7) but nothing enforces a control-level requirement; the
  phase-margin link relies on the cost law of the research line and on a control-engineering
  argument rather than on anything this code checks.
* Edges are charged independently and in isolation: branch interactions (one node with several
  children at different rates) are bounded per edge, and no joint bound is derived.
