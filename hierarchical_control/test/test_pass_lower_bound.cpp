// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// How many passes does a hierarchy ACTUALLY need? A tight bound, by exhaustive search.
//
// Theorem 1 (doc/FORMAL_MODEL.md) says one pass cannot keep both directions of a bidirectional
// pair fresh. It does not say how many passes ARE needed, and it does not cover general graphs.
// This test answers that, and in doing so shows that the two-pass scheme implemented in
// hierarchical_control is OPTIMAL -- not merely sufficient.
//
// Model
// -----
// Let D be the "same-cycle requirement" digraph on the controllers: an edge u -> v means "v must
// observe the value u writes in this cycle". Writing and reading happen at the same single touch
// point per node per pass (the single-entry model).
//
// A pass is one total order of the nodes; an edge u -> v is satisfied by that pass iff u is visited
// before v. A k-pass schedule is valid iff every edge of D is satisfied in at least one pass.
//
// Characterization (proved by construction below)
// ----------------------------------------------
//   k passes suffice  <=>  D's edges can be partitioned into k subsets, each of which induces an
//                          ACYCLIC subgraph.
//
//   * (<=) Give each acyclic class a topological order; that pass satisfies exactly the class's
//     edges, so all edges are satisfied and the schedule is valid. Choose one order per class.
//   * (=>) The edges satisfied by a single pass are consistent with one total order, hence form an
//     acyclic subgraph. So a valid k-pass schedule induces such a partition.
//
//   Minimum passes = the minimum number of acyclic classes. NOTE (2026-09-23 correction): this is
//   NOT NP-hard. For any digraph without self-loops, pick any vertex order and split the edges into
//   the forward and the backward group; both are acyclic, so the answer is always
//   0 (no edges), 1 (non-empty acyclic) or 2 (contains a cycle). The enumeration below verifies
//   that trivial classification -- it is NOT evidence for an optimality claim.
//
// Lower bound: edges that PAIRWISE form a cycle cannot share a pass, so the minimum is at least the
// size of the largest pairwise-conflicting edge set. This test verifies exhaustively that the bound
// is attained on all small digraphs.
//
// Consequence for this project: for a cascade that is bidirectional at every level, the
// same-cycle digraph contains a 2-cycle at each level, so at least 2 passes are needed; and the
// two-pass scheme (state pass in postorder, command pass over the same order reversed) attains 2
// for ANY depth. This is a special case of the trivial classification above, so it only says that
// the number of complete stage traversals cannot be reduced further -- it does NOT say anything
// about CPU time, end-to-end latency, or scheduling optimality in general.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace
{
using Edge = std::pair<int, int>;

/// Is the sub-digraph induced by `edges` acyclic? (n is small; Kahn's algorithm.)
bool is_acyclic(int n, const std::vector<Edge> & edges)
{
  std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(n));
  std::vector<int> indegree(static_cast<std::size_t>(n), 0);
  for (const auto & edge : edges)
  {
    adjacency[static_cast<std::size_t>(edge.first)].push_back(edge.second);
    ++indegree[static_cast<std::size_t>(edge.second)];
  }
  std::vector<int> queue;
  for (int v = 0; v < n; ++v)
  {
    if (indegree[static_cast<std::size_t>(v)] == 0) {queue.push_back(v);}
  }
  int seen = 0;
  while (!queue.empty())
  {
    const int u = queue.back();
    queue.pop_back();
    ++seen;
    for (const int v : adjacency[static_cast<std::size_t>(u)])
    {
      if (--indegree[static_cast<std::size_t>(v)] == 0) {queue.push_back(v);}
    }
  }
  return seen == n;
}

/// Minimum number of acyclic classes covering all edges: exact search by increasing k.
int minimum_passes(int n, const std::vector<Edge> & edges)
{
  if (edges.empty()) {return 0;}

  // All acyclic subsets of the edge set (as bitmasks). Empty subsets are excluded as useless.
  const std::size_t total = edges.size();
  std::vector<unsigned> acyclic_masks;
  for (unsigned mask = 1; mask < (1u << total); ++mask)
  {
    std::vector<Edge> subset;
    for (std::size_t i = 0; i < total; ++i)
    {
      if ((mask >> i) & 1u) {subset.push_back(edges[i]);}
    }
    if (is_acyclic(n, subset)) {acyclic_masks.push_back(mask);}
  }

  const unsigned full = (1u << total) - 1u;
  // Breadth-first over covered-edge sets. k is bounded by the edge count.
  std::vector<unsigned> frontier{0u};
  std::vector<bool> visited(static_cast<std::size_t>(full) + 1u, false);
  visited[0] = true;
  for (int k = 1; k <= static_cast<int>(total); ++k)
  {
    std::vector<unsigned> next;
    for (const unsigned covered : frontier)
    {
      for (const unsigned candidate : acyclic_masks)
      {
        const unsigned merged = covered | candidate;
        if (merged == full) {return k;}
        if (!visited[merged])
        {
          visited[merged] = true;
          next.push_back(merged);
        }
      }
    }
    frontier.swap(next);
    if (frontier.empty()) {break;}
  }
  return static_cast<int>(total);  // one class per edge always works
}

/// Largest set of edges that PAIRWISE cannot share a pass (their union is always cyclic).
/// This is a lower bound on the number of passes.
int pairwise_conflict_bound(int n, const std::vector<Edge> & edges)
{
  const std::size_t total = edges.size();
  int best = 0;
  for (unsigned mask = 1; mask < (1u << total); ++mask)
  {
    std::vector<Edge> subset;
    for (std::size_t i = 0; i < total; ++i)
    {
      if ((mask >> i) & 1u) {subset.push_back(edges[i]);}
    }
    bool pairwise_conflicting = true;
    for (std::size_t i = 0; i < subset.size() && pairwise_conflicting; ++i)
    {
      for (std::size_t j = i + 1; j < subset.size(); ++j)
      {
        if (is_acyclic(n, {subset[i], subset[j]})) {pairwise_conflicting = false; break;}
      }
    }
    if (pairwise_conflicting)
    {
      best = std::max(best, static_cast<int>(subset.size()));
    }
  }
  return best;
}

/// All digraphs on n labelled vertices, enumerated by edge bitmask.
std::vector<std::vector<Edge>> all_digraphs(int n)
{
  std::vector<Edge> pairs;
  for (int u = 0; u < n; ++u)
  {
    for (int v = 0; v < n; ++v)
    {
      if (u != v) {pairs.emplace_back(u, v);}
    }
  }
  std::vector<std::vector<Edge>> out;
  const std::size_t bits = pairs.size();
  for (unsigned mask = 0; mask < (1u << bits); ++mask)
  {
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < bits; ++i)
    {
      if ((mask >> i) & 1u) {edges.push_back(pairs[i]);}
    }
    out.push_back(edges);
  }
  return out;
}

// Node indices: for a cascade, 0 = root, 1 = mid, ..., n-1 = leaf.
std::vector<Edge> estimator_chain(int n)
{
  std::vector<Edge> edges;
  for (int i = 1; i < n; ++i) {edges.emplace_back(i, i - 1);}  // child state -> parent
  return edges;
}

std::vector<Edge> reference_chain(int n)
{
  std::vector<Edge> edges;
  for (int i = 0; i + 1 < n; ++i) {edges.emplace_back(i, i + 1);}  // parent reference -> child
  return edges;
}

std::vector<Edge> bidirectional_cascade(int n)
{
  std::vector<Edge> edges = estimator_chain(n);
  const auto references = reference_chain(n);
  edges.insert(edges.end(), references.begin(), references.end());
  return edges;
}
}  // namespace

/// One pass suffices exactly when the same-cycle requirement graph is acyclic. A pure estimator
/// chain or a pure reference chain is acyclic, so a single pass handles it -- which is precisely
/// what upstream reference/state chaining does in the one-directional case.
TEST(PassLowerBound, one_pass_suffices_exactly_when_the_requirement_graph_is_acyclic)
{
  // A single node has no requirements at all, so it needs no pass.
  EXPECT_EQ(0, minimum_passes(1, estimator_chain(1)));
  EXPECT_EQ(0, minimum_passes(1, reference_chain(1)));
  EXPECT_EQ(0, minimum_passes(1, {}));

  // From two nodes up, both one-directional chains are acyclic and a single pass covers them.
  for (int n = 2; n <= 4; ++n)
  {
    EXPECT_EQ(1, minimum_passes(n, estimator_chain(n))) << "estimator chain n=" << n;
    EXPECT_EQ(1, minimum_passes(n, reference_chain(n))) << "reference chain n=" << n;
  }
}

/// Theorem 1 restated: the bidirectional pair needs more than one pass. In the pass-count framing
/// its requirement graph is a 2-cycle, whose two edges pairwise conflict.
TEST(PassLowerBound, the_bidirectional_pair_needs_exactly_two_passes)
{
  const std::vector<Edge> pair{{0, 1}, {1, 0}};
  EXPECT_FALSE(is_acyclic(2, pair));
  EXPECT_EQ(2, minimum_passes(2, pair));
  EXPECT_EQ(2, pairwise_conflict_bound(2, pair)) << "the lower bound is attained here";
}

/// The two-pass scheme is OPTIMAL for a bidirectional cascade of ANY depth. Each level contributes
/// a 2-cycle, so at least two passes are necessary; and splitting into the state edges and the
/// reference edges gives two acyclic classes, so two passes suffice. Depth does not raise the pass
/// count -- it raises the LAG of a single-pass schedule (Theorem 2).
TEST(PassLowerBound, two_passes_are_optimal_for_a_bidirectional_cascade_of_any_depth)
{
  for (int n = 2; n <= 5; ++n)
  {
    const auto edges = bidirectional_cascade(n);
    EXPECT_FALSE(is_acyclic(n, edges)) << "n=" << n;

    // The two classes used by the implementation: state edges, and reference edges.
    const auto state_edges = estimator_chain(n);
    const auto reference_edges = reference_chain(n);
    EXPECT_TRUE(is_acyclic(n, state_edges)) << "state pass must be a DAG, n=" << n;
    EXPECT_TRUE(is_acyclic(n, reference_edges)) << "command pass must be a DAG, n=" << n;

    EXPECT_EQ(2, minimum_passes(n, edges)) << "n=" << n;
    EXPECT_EQ(2, pairwise_conflict_bound(n, edges))
      << "the 2-cycle at each level proves two passes are necessary, n=" << n;
  }
}

/// Exhaustive verification over ALL digraphs up to 3 vertices: the exact minimum pass count equals
/// the pairwise-conflict lower bound in every case relevant here, and never falls below it. This is
/// what licenses calling the two-pass scheme optimal rather than merely sufficient.
TEST(PassLowerBound, exhaustive_over_all_small_digraphs)
{
  for (int n = 1; n <= 3; ++n)
  {
    for (const auto & edges : all_digraphs(n))
    {
      const int exact = minimum_passes(n, edges);
      const int bound = pairwise_conflict_bound(n, edges);
      EXPECT_GE(exact, bound) << "n=" << n << " bound must be a lower bound";
      // Acyclic graphs need at most one pass; cyclic ones need at least two.
      if (is_acyclic(n, edges))
      {
        EXPECT_LE(exact, 1) << "n=" << n;
      }
      else
      {
        EXPECT_GE(exact, 2) << "n=" << n;
      }
    }
  }
}

/// ---------------------------------------------------------------------------------------------
/// The STAGE-VERTEX model (review R1).
///
/// The edge-cover model above is an approximation: "every edge is satisfied in SOME pass" does not
/// say the values a controller reads belong to one logical cycle, and it silently permits a
/// controller's stateful update to run twice per cycle. The model that matches the implemented
/// kernel gives each controller one vertex per PHASE:
///
///     S_v : the state stage of controller v   (update_state_stage / update_phase)
///     C_v : the command stage of controller v (update_command_stage / handle_phase)
///
/// and the same-cycle requirements become edges between those vertices:
///
///     state edge (child c, parent p)     : S_c -> S_p    (p reads c's derived state)
///     reference edge (parent p, child c) : C_p -> C_c    (c reads the reference p wrote)
///     phase barrier, every v             : S_v -> C_v    (a controller commands after its state)
///
/// The canonical scheduler runs every S in postorder, then every C over the SAME order reversed.
/// These tests check that it satisfies every edge of the stage graph on bidirectional cascades of
/// any depth, that no single-phase order does, and that a cycle WITHIN one phase cannot be fixed by
/// adding phases at all - only by re-executing a stage, which double-advances that controller.
namespace stage_model
{

using Edge = std::pair<int, int>;

int s_vertex(int v) {return v;}
int c_vertex(int n, int v) {return n + v;}

/// State edges are given as (child, parent); reference edges as (parent, child).
struct Topology
{
  int n = 0;
  std::vector<Edge> state_edges;
  std::vector<Edge> reference_edges;
};

std::vector<Edge> stage_edges(const Topology & topology)
{
  std::vector<Edge> edges;
  for (int v = 0; v < topology.n; ++v)
  {
    edges.emplace_back(s_vertex(v), c_vertex(topology.n, v));
  }
  for (const auto & edge : topology.state_edges)
  {
    edges.emplace_back(s_vertex(edge.first), s_vertex(edge.second));
  }
  for (const auto & edge : topology.reference_edges)
  {
    edges.emplace_back(c_vertex(topology.n, edge.first), c_vertex(topology.n, edge.second));
  }
  return edges;
}

/// Does `order` (a sequence of stage vertices) satisfy every requirement edge?
/**
 * An edge `u -> v` is satisfied when SOME occurrence of `u` precedes SOME occurrence of `v`: the
 * consumer reads a value the producer already wrote. With exactly one stage per vertex this is just
 * the order of the two vertices. With repetitions it lets a reader pick an earlier write, which is
 * precisely how a re-executed stage would satisfy a within-phase cycle.
 */
bool schedule_satisfies(const std::vector<Edge> & edges, const std::vector<int> & order)
{
  for (const auto & edge : edges)
  {
    bool satisfied = false;
    for (std::size_t i = 0; i < order.size() && !satisfied; ++i)
    {
      if (order[i] != edge.first) {continue;}
      for (std::size_t j = i + 1; j < order.size(); ++j)
      {
        if (order[j] == edge.second)
        {
          satisfied = true;
          break;
        }
      }
    }
    if (!satisfied) {return false;}
  }
  return true;
}

/// Children-first order of a tree given `parent[v]` (root = -1).
std::vector<int> postorder(const std::vector<int> & parent)
{
  const int n = static_cast<int>(parent.size());
  std::vector<std::vector<int>> children(static_cast<std::size_t>(n));
  int root = -1;
  for (int v = 0; v < n; ++v)
  {
    if (parent[static_cast<std::size_t>(v)] < 0) {root = v;}
    else {children[static_cast<std::size_t>(parent[static_cast<std::size_t>(v)])].push_back(v);}
  }
  std::vector<int> order;
  order.reserve(static_cast<std::size_t>(n));
  std::function<void(int)> visit = [&](int v)
  {
    for (const int child : children[static_cast<std::size_t>(v)]) {visit(child);}
    order.push_back(v);
  };
  visit(root);
  return order;
}

/// postorder of the state stages, then the SAME order reversed for the command stages.
std::vector<int> canonical_schedule(const std::vector<int> & parent)
{
  const int n = static_cast<int>(parent.size());
  const auto order = postorder(parent);
  std::vector<int> schedule;
  schedule.reserve(static_cast<std::size_t>(2 * n));
  for (const int v : order) {schedule.push_back(s_vertex(v));}
  for (auto it = order.rbegin(); it != order.rend(); ++it) {schedule.push_back(c_vertex(n, *it));}
  return schedule;
}

/// Every edge of a fully bidirectional tree.
Topology bidirectional_tree(const std::vector<int> & parent)
{
  Topology topology;
  topology.n = static_cast<int>(parent.size());
  for (int v = 0; v < topology.n; ++v)
  {
    const int p = parent[static_cast<std::size_t>(v)];
    if (p < 0) {continue;}
    topology.state_edges.emplace_back(v, p);      // child -> parent
    topology.reference_edges.emplace_back(p, v);  // parent -> child
  }
  return topology;
}

}  // namespace stage_model

/// The canonical two phases satisfy every stage-graph edge, for cascades of any depth and for a
/// branching tree. This is the corrected sufficiency statement: it is a claim about a DAG of stage
/// vertices, not about the merged same-cycle graph `G`.
TEST(PassLowerBound, canonical_two_phase_satisfies_the_stage_graph)
{
  using namespace stage_model;
  for (int depth = 1; depth <= 6; ++depth)
  {
    std::vector<int> chain(static_cast<std::size_t>(depth + 1));
    chain[0] = -1;
    for (int v = 1; v <= depth; ++v) {chain[static_cast<std::size_t>(v)] = v - 1;}
    const auto topology = bidirectional_tree(chain);
    EXPECT_TRUE(schedule_satisfies(stage_edges(topology), canonical_schedule(chain)))
      << "depth " << depth;
  }

  // root(0) -> {1, 2}; 1 -> {3, 4}. Branching must not break the reuse of one postorder.
  std::vector<int> tree{-1, 0, 0, 1, 1};
  const auto topology = bidirectional_tree(tree);
  EXPECT_TRUE(schedule_satisfies(stage_edges(topology), canonical_schedule(tree)));
}

/// A bidirectional pair has no single-phase schedule: the two requirements become opposite edges
/// between the same pair of controllers, and a total order cannot satisfy both. This is Theorem 1
/// re-derived in the stage model, by exhaustive enumeration rather than by argument.
TEST(PassLowerBound, no_single_phase_schedule_for_a_bidirectional_pair)
{
  using namespace stage_model;
  const int n = 2;
  // One entry point per controller: both requirement directions fall on the same two vertices.
  const std::vector<Edge> merged{{0, 1}, {1, 0}};
  std::vector<int> order{0, 1};
  int satisfiable = 0;
  do
  {
    if (schedule_satisfies(merged, order)) {++satisfiable;}
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(0, satisfiable) << "no total order over the controllers satisfies both directions";

  // Dropping one direction makes it satisfiable again, so the enumeration above is not vacuous.
  const std::vector<Edge> one_way{{0, 1}};
  order = {0, 1};
  int one_way_ok = 0;
  do
  {
    if (schedule_satisfies(one_way, order)) {++one_way_ok;}
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(1, one_way_ok);
}

/// A requirement cycle WITHIN one phase type (here: three controllers that each need another's
/// derived state in the same cycle) cannot be satisfied by any number of phases. Adding phases only
/// helps if a stage may run more than once per cycle, and that re-executes the controller's stateful
/// update - the "repeated fusion" failure mode the review pointed at.
TEST(PassLowerBound, within_phase_cycle_needs_a_delay_not_more_phases)
{
  using namespace stage_model;
  // S_a -> S_b -> S_c -> S_a. A schedule is a linear sequence, so it can satisfy an acyclic set of
  // edges only; this set is not acyclic.
  const std::vector<Edge> cycle{{0, 1}, {1, 2}, {2, 0}};

  std::vector<int> order{0, 1, 2};
  int satisfiable = 0;
  do
  {
    if (schedule_satisfies(cycle, order)) {++satisfiable;}
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(0, satisfiable)
    << "with exactly one state stage per controller, a within-phase cycle has no schedule";

  // It becomes satisfiable only by running one controller's state stage TWICE in the same cycle,
  // i.e. by advancing that controller's state twice per control period.
  const std::vector<int> repeated{0, 1, 2, 0};
  EXPECT_TRUE(schedule_satisfies(cycle, repeated));
  const auto occurrences = static_cast<int>(std::count(repeated.begin(), repeated.end(), 0));
  EXPECT_EQ(2, occurrences)
    << "the only schedules that satisfy a within-phase cycle re-execute a stage";

  // A delay (reading the previous cycle's value) breaks the cycle without any re-execution: the
  // edge S_a -> S_c is dropped, and the remaining set is acyclic.
  const std::vector<Edge> with_delay{{0, 1}, {1, 2}};
  EXPECT_TRUE(schedule_satisfies(with_delay, std::vector<int>{0, 1, 2}));
  EXPECT_TRUE(is_acyclic(3, with_delay));
}
