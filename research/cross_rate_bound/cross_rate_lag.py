#!/usr/bin/env python3
"""Cross-rate bucket lag: exhaustive model of the two-phase pass order.

`PENDING_WORK.md` §2.1 (P0-2) asks for the FineMote Theorem-3 analogue for the two-phase path:
today a reference edge whose two ends fall into DIFFERENT rate buckets is REFUSED
(`cross_rate_dependency`), because the implementation promises same-cycle freshness on every
admitted edge and does not have a bound for the rest.

This script makes the bound computable instead of assumed. It re-implements, exactly, the pass
order of `ControllerManager::update()` on the feature branch:

    for factor in buckets (ASCENDING):
        if cycle % factor == 0:
            state pass   : walk the controller list BACKWARD
            command pass : walk the controller list FORWARD

(`controller_manager/src/controller_manager.cpp`, the `for (const auto factor : buckets)` loops in
the two-phase section; the list is parents-first, so BACKWARD is children-first and FORWARD is
parents-first.)

For one parent P (bucket f_P) and one child C (bucket f_C) with a reference edge P -> C and a
state edge C -> P, it simulates a full period of the schedule (lcm(f_P, f_C) cycles) and reports
the WORST observed age, in manager cycles, of the value each end consumes:

    state age : cycles between the child publishing and the parent ingesting it
    ref age   : cycles between the parent writing and the child reading it

It then checks a closed form and fails loudly if the two disagree -- so the formula below is a
tested claim about the model, not prose.

Two assumptions, both of which the implementation makes TRUE rather than assumes:

* the controller list is parents-first for the edge in question. The two-phase admission refuses
  `unschedulable_order`, so this holds for every admitted edge;
* the due set of a member with factor f is exactly the multiples of f. The manager's counter is
  `update_loop_counter_ = (update_loop_counter_ + 1) % update_rate_`, which would perturb the due
  set of a factor that does not divide `update_rate_`. Admission requires
  `update_rate_ % controller_rate == 0`, so `factor = update_rate_ / controller_rate` DIVIDES
  `update_rate_`, and the wrap never perturbs the due set.

Usage:  python3 research/cross_rate_bound/cross_rate_lag.py
"""
from __future__ import annotations

import itertools
import math


def simulate(f_parent: int, f_child: int, cycles: int, measure_from: int = 0) -> tuple[int, int]:
    """Return (worst state age, worst reference age) in manager cycles.

    Ages are measured on the cycles where the CONSUMER runs, because that is the cycle in which a
    stale value becomes visible to the controller that uses it. `measure_from` skips the start-up
    transient (a producer has no stamp yet), so the caller measures a full steady-state period.
    """
    buckets = sorted({f_parent, f_child})

    child_state_stamp: int | None = None  # cycle at which C last published
    parent_ref_stamp: int | None = None  # cycle at which P last wrote
    worst_state_age = 0
    worst_ref_age = 0

    for cycle in range(1, cycles + 1):
        # ---- state pass: ascending buckets, list walked BACKWARD (child first) ----
        for factor in buckets:
            if cycle % factor != 0:
                continue
            # Within one bucket the list is walked backwards, i.e. the child is visited first.
            if factor == f_child:
                child_state_stamp = cycle
            if factor == f_parent:
                if child_state_stamp is not None and cycle > measure_from:
                    worst_state_age = max(worst_state_age, cycle - child_state_stamp)

        # ---- command pass: ascending buckets, list walked FORWARD (parent first) ----
        for factor in buckets:
            if cycle % factor != 0:
                continue
            # Within one bucket the list is walked forwards, i.e. the parent is visited first.
            if factor == f_parent:
                parent_ref_stamp = cycle
            if factor == f_child:
                if parent_ref_stamp is not None and cycle > measure_from:
                    worst_ref_age = max(worst_ref_age, cycle - parent_ref_stamp)

    return worst_state_age, worst_ref_age


def closed_form(f_parent: int, f_child: int) -> tuple[int, int]:
    """The exact worst-case age, in manager cycles, derived from the pass order.

    The bucket loop is ASCENDING in both passes, and the walk direction is fixed per pass:

        state pass   : walked BACKWARD -- within a bucket the child is visited before its parent;
                       across buckets, the SMALLER factor runs first;
        command pass : walked FORWARD  -- within a bucket the parent is visited before its child;
                       across buckets, the SMALLER factor runs first.

    A consumer therefore sees a producer's SAME-cycle value only when the producer's bucket is the
    smaller one (or equal). Otherwise it reads the producer's most recent value; over the schedule
    (period lcm(f_P, f_C)) the largest offset a consumer tick can have below a producer tick is
    exact, not merely an upper bound.

    Let g = gcd(f_P, f_C). The two directions are asymmetric:

        state edge (child -> parent):
            f_C >  f_P  -> f_C        the parent bucket always runs FIRST in that cycle, so even a
                                      coincident cycle costs the child's whole period
            f_C == f_P  -> 0
            f_C <  f_P  -> f_C - g    (0 when f_C divides f_P)

        reference edge (parent -> child):
            f_P <  f_C  -> f_P - g    (0 when f_P divides f_C)
            f_P == f_C  -> 0
            f_P >  f_C  -> f_P        the child bucket always runs first

    Because the pattern repeats with period lcm(f_P, f_C), each value is ATTAINED: it is the exact
    worst-case staleness of this schedule, not a loose bound.
    """
    g = math.gcd(f_parent, f_child)

    if f_child > f_parent:
        state_age = f_child
    elif f_child == f_parent:
        state_age = 0
    else:
        state_age = f_child - g

    if f_parent < f_child:
        ref_age = f_parent - g
    elif f_parent == f_child:
        ref_age = 0
    else:
        ref_age = f_parent

    return state_age, ref_age


def main() -> int:
    limit = 12
    header = f"{'f_P':>3} {'f_C':>3} | {'state':>5} {'ref':>4} | {'formula':>7} {'formula':>7} | verdict"
    print(header)
    print("-" * len(header))

    failures = []
    for f_parent, f_child in itertools.product(range(1, limit + 1), repeat=2):
        period = math.lcm(f_parent, f_child)
        measured = simulate(f_parent, f_child, 3 * period, measure_from=2 * period)
        claimed = closed_form(f_parent, f_child)
        ok = measured == claimed
        if not ok:
            failures.append((f_parent, f_child, measured, claimed))
        print(
            f"{f_parent:>3} {f_child:>3} | {measured[0]:>5} {measured[1]:>4} | "
            f"{claimed[0]:>7} {claimed[1]:>7} | {'OK' if ok else 'MISMATCH'}"
        )

    print()
    if failures:
        print("FORMULA DOES NOT MATCH THE MODEL:")
        for f_parent, f_child, measured, claimed in failures:
            print(f"  f_P={f_parent} f_C={f_child}: measured={measured} claimed={claimed}")
        return 1

    print(f"closed form matches the model for every pair in 1..{limit}")

    # The cases that matter for the paper.
    print()
    print("harmonic cases (one bucket divides the other):")
    for f_parent, f_child in ((1, 4), (2, 4), (4, 2), (4, 1), (1, 1)):
        period = math.lcm(f_parent, f_child)
        state_age, ref_age = simulate(f_parent, f_child, 3 * period, measure_from=2 * period)
        relation = "harmonic" if (f_parent % f_child == 0 or f_child % f_parent == 0) else "non-harmonic"
        print(
            f"  f_P={f_parent} f_C={f_child} ({relation}): "
            f"state age {state_age} manager cycles, ref age {ref_age}"
        )

    print()
    print("non-harmonic cases:")
    for f_parent, f_child in ((2, 3), (3, 2), (4, 6), (6, 4)):
        period = math.lcm(f_parent, f_child)
        state_age, ref_age = simulate(f_parent, f_child, 3 * period, measure_from=2 * period)
        print(f"  f_P={f_parent} f_C={f_child}: state age {state_age}, ref age {ref_age}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
