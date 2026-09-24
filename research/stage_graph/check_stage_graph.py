#!/usr/bin/env python3
"""Exhaustive check of the STAGE-VERTEX model for two-phase controller scheduling (review R1).

Why this model replaces the "acyclic edge cover" model
-----------------------------------------------------
The edge-cover model said: a k-pass schedule exists iff the same-cycle requirement edges can be
partitioned into k acyclic classes. That model is an approximation in two ways:

  * "every edge is satisfied in SOME pass" does not say the values a controller consumes belong to
    one logical cycle, and
  * it silently allows a controller's stateful update to run more than once per cycle, which
    advances its integrators/estimators more than once per control period.

The model that matches the implemented kernel gives each controller one vertex per PHASE:

    S_v : state stage of controller v   (update_state_stage / update_phase)
    C_v : command stage of controller v (update_command_stage / handle_phase)

with same-cycle requirements as edges between those vertices:

    state edge (child c, parent p)     : S_c -> S_p
    reference edge (parent p, child c) : C_p -> C_c
    phase barrier, every v             : S_v -> C_v

A schedule is a linear sequence of stage vertices; an edge u -> v is satisfied when some occurrence
of u precedes some occurrence of v. The canonical scheduler runs every S in postorder of the tree
and then every C over the SAME order reversed.

What this script verifies, for every rooted tree on n <= 4 nodes and every labelling of its edges as
reference-only / state-only / both:
  1. the canonical two-phase schedule satisfies every edge of the stage graph;
  2. a single-phase schedule exists iff the merged requirement graph is acyclic (i.e. no edge is
     bidirectional), and otherwise not one of the n! controller orders works;
  3. the minimum number of complete phases is therefore 1 or 2, and 2 phases are reached by the
     canonical schedule.

It also shows the real impossibility, which lives INSIDE one phase: a directed cycle among state
stages has no schedule with one stage per vertex, no matter how many phases are allowed; the only
schedules that satisfy it re-execute a stage (double-advancing that controller).

Run:  python3 research/stage_graph/check_stage_graph.py
"""

import itertools
import sys


# --------------------------------------------------------------------------------------------
# topology
# --------------------------------------------------------------------------------------------


def all_rooted_trees(n):
    """All parent arrays on nodes 0..n-1 rooted at 0 that form a tree (no cycles)."""
    if n == 1:
        yield (-1,)
        return
    for parents in itertools.product(range(-1, n), repeat=n):
        if parents[0] != -1:
            continue
        if any(parents[v] == -1 for v in range(1, n)):
            continue
        if any(parents[v] == v for v in range(n)):
            continue
        # every node must reach the root
        ok = True
        for v in range(n):
            seen = set()
            cursor = v
            while cursor != -1:
                if cursor in seen:
                    ok = False
                    break
                seen.add(cursor)
                cursor = parents[cursor]
            if not ok:
                break
        if ok:
            yield parents


def postorder(parents):
    n = len(parents)
    children = [[] for _ in range(n)]
    root = None
    for v in range(n):
        if parents[v] == -1:
            root = v
        else:
            children[parents[v]].append(v)
    order = []

    def visit(v):
        for child in children[v]:
            visit(child)
        order.append(v)

    visit(root)
    return order


# --------------------------------------------------------------------------------------------
# stage graph
# --------------------------------------------------------------------------------------------


def s_vertex(v):
    return v


def c_vertex(n, v):
    return n + v


def stage_edges(n, parents, labelling):
    """labelling[v] in {'ref', 'state', 'both'} describes the edge (v, parents[v])."""
    edges = []
    for v in range(n):
        edges.append((s_vertex(v), c_vertex(n, v)))  # phase barrier
        if parents[v] == -1:
            continue
        label = labelling[v]
        if label in ("state", "both"):
            edges.append((s_vertex(v), s_vertex(parents[v])))  # child state -> parent
        if label in ("ref", "both"):
            edges.append((c_vertex(n, parents[v]), c_vertex(n, v)))  # parent reference -> child
    return edges


def satisfied(edges, order):
    for producer, consumer in edges:
        found = False
        for i, vertex in enumerate(order):
            if vertex != producer:
                continue
            if consumer in order[i + 1 :]:
                found = True
                break
        if not found:
            return False
    return True


def canonical_schedule(n, parents):
    order = postorder(parents)
    return [s_vertex(v) for v in order] + [c_vertex(n, v) for v in reversed(order)]


def merged_requirement_edges(n, parents, labelling):
    """The single-entry model's requirement graph `G`.

    With one entry point per controller, a state edge (child v, parent p) needs the CHILD first
    (the parent reads the child's derived state), and a reference edge (parent p, child c) needs the
    PARENT first. A bidirectional edge therefore yields v -> p and p -> v, i.e. a 2-cycle.
    """
    edges = []
    for v in range(n):
        if parents[v] == -1:
            continue
        label = labelling[v]
        if label in ("state", "both"):
            edges.append((v, parents[v]))  # child -> parent
        if label in ("ref", "both"):
            edges.append((parents[v], v))  # parent -> child
    return edges


def merged_graph_acyclic(n, parents, labelling):
    edges = merged_requirement_edges(n, parents, labelling)
    adjacency = {v: [] for v in range(n)}
    for a, b in edges:
        adjacency[a].append(b)
    colour = {v: 0 for v in range(n)}

    def visit(v):
        colour[v] = 1
        for w in adjacency[v]:
            if colour[w] == 1:
                return False
            if colour[w] == 0 and not visit(w):
                return False
        colour[v] = 2
        return True

    return all(colour[v] != 0 or visit(v) for v in range(n))


def single_phase_possible(n, parents, labelling):
    """Exhaustive: does ANY total order over the controllers satisfy every requirement?"""
    edges = merged_requirement_edges(n, parents, labelling)
    for order in itertools.permutations(range(n)):
        position = {v: i for i, v in enumerate(order)}
        if all(position[a] < position[b] for a, b in edges):
            return True
    return False


def main():
    print("Stage-vertex model: canonical two-phase schedule vs single-phase orders")
    print()
    total = 0
    single_fail = 0
    single_ok = 0
    for n in range(1, 5):
        for parents in all_rooted_trees(n):
            for labelling in itertools.product(("ref", "state", "both"), repeat=n):
                if n > 1 and labelling[0] != "ref":
                    # labelling[0] is unused (the root has no parent); keep only one representative
                    continue
                total += 1
                edges = stage_edges(n, parents, labelling)
                if not satisfied(edges, canonical_schedule(n, parents)):
                    print(f"  COUNTEREXAMPLE: parents={parents} labelling={labelling}")
                    return 1
                merged_acyclic = merged_graph_acyclic(n, parents, labelling)
                possible = single_phase_possible(n, parents, labelling)
                if possible != merged_acyclic:
                    print(
                        f"  MISMATCH: parents={parents} labelling={labelling} "
                        f"merged_acyclic={merged_acyclic} single_phase={possible}"
                    )
                    return 1
                if possible:
                    single_ok += 1
                else:
                    single_fail += 1

    print(f"  trees x labellings checked                    : {total}")
    print(f"  canonical two-phase satisfies the stage graph : {total}/{total}")
    print(f"  single phase possible (no bidirectional edge) : {single_ok}")
    print(f"  single phase impossible (>=1 bidirectional)   : {single_fail}")
    print("  => the minimum number of complete phases is 1 or 2, and the canonical schedule is")
    print("     sufficient whenever the parent/child structure is a tree.")
    print()

    # --- the real impossibility: a cycle WITHIN one phase ------------------------------------
    print("Within-phase cycle (three controllers each needing another's derived state):")
    cycle = [(0, 1), (1, 2), (2, 0)]
    one_each = list(itertools.permutations(range(3)))
    ok = [order for order in one_each if satisfied(cycle, list(order))]
    print(f"  schedules with one state stage per vertex satisfying all three edges: {len(ok)}")
    repeated = [0, 1, 2, 0]
    print(f"  {repeated} satisfies all three edges: {satisfied(cycle, repeated)}")
    print("  but it runs controller 0's state stage twice, i.e. advances its state twice in one")
    print("  control period. Adding phases does not help: any linear schedule orders the two stages")
    print("  of a 2-cycle one way, so the other edge must be delayed instead.")
    print()
    print("Independent of PASS_LOWER_BOUND.md's edge-cover model, which is a coarser approximation.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
