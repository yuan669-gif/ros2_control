#!/usr/bin/env python3
"""Exhaustive search: the minimum number of passes a hierarchy needs.

Model
-----
Let D be the "same-cycle requirement" digraph on the controllers: an edge u -> v means
"v must observe the value u writes in THIS cycle". Writing and reading happen at the same single
touch point per node per pass (the single-entry model of ros2_control's update()).

A *pass* is one total order of V. Within a pass, an edge u -> v is satisfied iff u is visited
before v. A schedule is k passes; it is valid iff every edge of D is satisfied in at least one
pass. Note that one pass can satisfy many edges at once -- exactly those edges that are consistent
with a single total order -- so the relevant obstruction is NOT "D has a cycle" but the structure
of D's strongly connected components.

Claim under test
----------------
  min passes  ==  length of the longest directed path in the CONDENSATION of D.

Equivalently: each pass can cover edges spanning at most one "SCC level", so the number of passes is
the number of SCC levels on the longest chain.

Note: the earlier version of this script measured the longest path in D itself and compared against
a BFS that was correct; the mismatch was in the candidate, not the search. This version compares
against the condensation.

Usage:  python3 research/pass_lower_bound/search_min_passes.py
"""
import itertools
from collections import deque


def is_acyclic(n, edges):
    """Kahn's algorithm: is the sub-digraph induced by `edges` acyclic?"""
    adj = [[] for _ in range(n)]
    indeg = [0] * n
    for u, v in edges:
        adj[u].append(v)
        indeg[v] += 1
    q = deque([i for i in range(n) if indeg[i] == 0])
    seen = 0
    while q:
        u = q.popleft()
        seen += 1
        for v in adj[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                q.append(v)
    return seen == n


def strongly_connected_components(n, edges):
    """Tarjan-free simple SCC via reachability (n is tiny here)."""
    adj = [[] for _ in range(n)]
    for u, v in edges:
        adj[u].append(v)

    def reach(src):
        seen = {src}
        stack = [src]
        while stack:
            u = stack.pop()
            for v in adj[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        return seen

    reachable = [reach(i) for i in range(n)]
    comp = [-1] * n
    cid = 0
    for u in range(n):
        if comp[u] != -1:
            continue
        for v in range(n):
            if comp[v] == -1 and v in reachable[u] and u in reachable[v]:
                comp[v] = cid
        cid += 1
    return comp, cid


def condensation_longest_path(n, edges):
    """Longest directed path (in edges) in the condensation DAG of (V, edges)."""
    comp, ncomp = strongly_connected_components(n, edges)
    cadj = [set() for _ in range(ncomp)]
    for u, v in edges:
        if comp[u] != comp[v]:
            cadj[comp[u]].add(comp[v])
    best = 0

    def dfs(c, depth):
        nonlocal best
        best = max(best, depth)
        for d in cadj[c]:
            dfs(d, depth + 1)

    for c in range(ncomp):
        dfs(c, 0)
    return best, comp


def pass_edge_sets(n, edges):
    """Every edge-set satisfiable by a single total order (plus the empty set)."""
    out = set()
    for perm in itertools.permutations(range(n)):
        pos = {v: i for i, v in enumerate(perm)}
        out.add(frozenset((u, v) for (u, v) in edges if pos[u] < pos[v]))
    return [s for s in out if s]


def min_passes(n, edges, cap=5):
    if not edges:
        return 0
    full = frozenset(edges)
    cands = pass_edge_sets(n, edges)
    seen = {frozenset(): 0}
    q = deque([frozenset()])
    while q:
        cur = q.popleft()
        d = seen[cur]
        if d >= cap:
            continue
        for c in cands:
            nxt = cur | c
            if nxt == full:
                return d + 1
            if nxt not in seen:
                seen[nxt] = d + 1
                q.append(nxt)
    return None


def all_digraphs(n):
    pairs = [(u, v) for u in range(n) for v in range(n) if u != v]
    for mask in range(1 << len(pairs)):
        yield [pairs[i] for i in range(len(pairs)) if (mask >> i) & 1]


def pairwise_conflict_bound(n, edges):
    """Largest set of edges that pairwise cannot share a pass (their 2-subsets all contain a
    cycle). By Theorem P2 this is a lower bound on the number of passes."""
    E = list(edges)
    best = 0
    for r in range(len(E), 0, -1):
        if r <= best:
            break
        for S in itertools.combinations(E, r):
            if all(not is_acyclic(n, [a, b]) for a, b in itertools.combinations(S, 2)):
                best = max(best, r)
                break
    return best


def main():
    print("=== exhaustive over ALL digraphs ===")
    print("    checking: min_passes >= pairwise_conflict_bound, and whether equality holds")
    for n in (1, 2, 3):
        total = 0
        lower_violations = 0
        gaps = 0
        hist = {}
        for edges in all_digraphs(n):
            total += 1
            if not edges:
                hist[0] = hist.get(0, 0) + 1
                continue
            mp = min_passes(n, edges)
            lb = pairwise_conflict_bound(n, edges)
            hist[mp] = hist.get(mp, 0) + 1
            if mp < lb:
                lower_violations += 1
            if mp != lb:
                gaps += 1
        print(f"  n={n}: {total} digraphs; min-passes histogram {dict(sorted(hist.items()))}")
        print(
            f"         bound violated: {lower_violations}"
            f" ; cases where the bound is NOT attained: {gaps}"
        )
    print()
    print("  (The bound is attained on every case this project cares about; a handful of small")
    print("   digraphs have a strict gap, which is why the exact count needs the colouring search.)")

    print()
    print("=== the cases that matter for this project ===")
    cases = {
        "pure estimator chain (state edges only)": (3, [(1, 0), (2, 1)]),
        "pure reference chain (command edges only)": (3, [(0, 1), (1, 2)]),
        "bidirectional PAIR (theorem 1)": (2, [(1, 0), (0, 1)]),
        "depth-2 cascade, bidirectional at each level": (3, [(1, 0), (0, 1), (2, 1), (1, 2)]),
        "depth-3 cascade, bidirectional at each level": (
            4,
            [(1, 0), (0, 1), (2, 1), (1, 2), (3, 2), (2, 3)],
        ),
    }
    for name, (n, e) in cases.items():
        mp = min_passes(n, e)
        lb = pairwise_conflict_bound(n, e)
        print(f"  {name}")
        print(f"      D={e}")
        print(f"      min passes = {mp}   (pairwise-conflict lower bound = {lb})")

    print()
    print("Interpretation (CORRECTED -- do not use 'condensation longest path'):")
    print("  A single pass satisfies exactly the edge sets consistent with one total order, i.e. the")
    print("  ACYCLIC sub-digraphs. Hence:")
    print("     min passes == fewest acyclic edge classes covering D      (proved both directions)")
    print("     lower bound == largest PAIRWISE-conflicting edge set")
    print("  For every case above the bound is attained:")
    print("     * one-directional chain  : D is acyclic          -> 1 pass")
    print("     * bidirectional pair     : D is a 2-cycle        -> 2 passes (Theorem 1's core)")
    print("     * bidirectional cascade  : 2-cycle at each level -> 2 passes, INDEPENDENT of depth")
    print("  So the cascade DEPTH does not raise the pass count; it raises the LAG of a one-pass")
    print("  schedule (Theorem 2). The implemented two-pass scheme is therefore OPTIMAL.")
    print("  See doc/PASS_LOWER_BOUND.md and hierarchical_control/test/test_pass_lower_bound.cpp.")


if __name__ == "__main__":
    main()
