// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Compile-time topology: what the static guarantee does and does not cover.
//
// doc/FORMAL_MODEL.md Corollary 2 makes acyclicity a configuration-time check. This test verifies
// the compile-time strengthening in hierarchical_control/static_topology.hpp, AND pins down its
// boundary, because the boundary is the part that is easy to overstate.
//
// The claim under test:
//   (1) A legal hierarchy typechecks and exposes its root-to-self ancestry at compile time.
//   (2) A self-loop is rejected at compile time.
//   (3) Reusing an ancestor (closing a cycle of length >= 2) is rejected at compile time.
//   (4) A bidirectional cascade -- the project's actual use case -- is acyclic in the construction
//       graph and is therefore expressible. Note carefully what this does and does not mean: the
//       COMPILE-TIME structure here is the reference/tree direction, and the fact that this
//       typechecks does NOT make the same-cycle data flow of Theorems 1-2 satisfiable in one pass.
//       The two-pass scheduling result is orthogonal to this header.
//
// Cases (2) and (3) cannot appear in this file: a negative case is a hard compile error, so it
// lives in test/static_topology_negative/ and is checked by test_static_topology_negative.py,
// which asserts that each file FAILS to compile while a control file compiles.

#include <gtest/gtest.h>

#include <cstddef>
#include <string_view>

#include "hierarchical_control/static_topology.hpp"

namespace st = hierarchical_control::static_topology;

namespace
{
struct chassis_name
{
  static constexpr auto value = st::NameOf("chassis");
};
struct wheel_left_name
{
  static constexpr auto value = st::NameOf("wheel_left");
};
struct wheel_right_name
{
  static constexpr auto value = st::NameOf("wheel_right");
};
struct tire_name
{
  static constexpr auto value = st::NameOf("tire");
};

// A four-level cascade, used by the bidirectional test below. These must be at namespace scope:
// a local class may not have a static data member, and `value` has to be one to be usable as a
// non-type template argument.
struct l0_name
{
  static constexpr auto value = st::NameOf("l0");
};
struct l1_name
{
  static constexpr auto value = st::NameOf("l1");
};
struct l2_name
{
  static constexpr auto value = st::NameOf("l2");
};
struct l3_name
{
  static constexpr auto value = st::NameOf("l3");
};

// Names for the boundary test.
struct root_name
{
  static constexpr auto value = st::NameOf("root");
};
struct parent_a_name
{
  static constexpr auto value = st::NameOf("parent_a");
};
struct parent_b_name
{
  static constexpr auto value = st::NameOf("parent_b");
};
struct shared_name
{
  static constexpr auto value = st::NameOf("shared");
};

using chassis = st::Root<chassis_name>;
using wheel_left = st::Descendant<wheel_left_name, chassis>;
using wheel_right = st::Descendant<wheel_right_name, chassis>;
using tire = st::Descendant<tire_name, wheel_left>;

using l0 = st::Root<l0_name>;
using l1 = st::Descendant<l1_name, l0>;
using l2 = st::Descendant<l2_name, l1>;
using l3 = st::Descendant<l3_name, l2>;

using root = st::Root<root_name>;
using parent_a = st::Descendant<parent_a_name, root>;
using parent_b = st::Descendant<parent_b_name, root>;
using shared = st::Descendant<shared_name, parent_a>;  // lives under parent_a
}  // namespace

/// A well-formed chain typechecks and reports its structure at compile time.
TEST(StaticTopology, legal_hierarchy_is_well_formed)
{
  static_assert(st::is_valid_node_v<chassis>);
  static_assert(st::is_valid_node_v<wheel_left>);
  static_assert(st::is_valid_node_v<tire>);

  static_assert(st::node_count<chassis>() == 1);
  static_assert(st::node_count<wheel_left>() == 2);
  static_assert(st::node_count<tire>() == 3);

  static_assert(st::is_root<chassis>());
  static_assert(!st::is_root<wheel_left>());
  static_assert(!st::is_root<tire>());

  EXPECT_EQ(std::string_view{"chassis"}, st::node_name<chassis>());
  EXPECT_EQ(std::string_view{"tire"}, st::node_name<tire>());
  EXPECT_EQ(3u, st::node_count<tire>());
}

/// Reference edges are derived from ancestry, at compile time, with no runtime topology scan.
TEST(StaticTopology, reference_edges_follow_ancestry)
{
  static_assert(st::has_reference_edge<wheel_left, chassis>());
  static_assert(st::has_reference_edge<tire, chassis>());      // transitive
  static_assert(st::has_reference_edge<tire, wheel_left>());
  static_assert(!st::has_reference_edge<chassis, wheel_left>());  // no upward edge
  static_assert(!st::has_reference_edge<wheel_left, wheel_right>());  // siblings: no edge

  SUCCEED();
}

/// A branching hierarchy (two children of one parent) is legal and each branch keeps its own
/// ancestry. This is the POV-chassis shape from the case study.
TEST(StaticTopology, branching_is_legal_and_ancestry_stays_per_branch)
{
  static_assert(st::is_valid_node_v<wheel_left>);
  static_assert(st::is_valid_node_v<wheel_right>);
  static_assert(st::node_count<wheel_left>() == 2);
  static_assert(st::node_count<wheel_right>() == 2);

  // Both share the root, but neither contains the other.
  static_assert(st::has_reference_edge<wheel_left, chassis>());
  static_assert(st::has_reference_edge<wheel_right, chassis>());
  static_assert(!st::has_reference_edge<wheel_left, wheel_right>());
  static_assert(!st::has_reference_edge<wheel_right, wheel_left>());

  SUCCEED();
}

/// A bidirectional cascade -- every level consumes its child's state and produces its child's
/// reference -- typechecks. IMPORTANT: this only says the construction graph is acyclic. The
/// same-cycle data-flow requirement of Theorems 1-2 is a separate question, answered by
/// test_pass_lower_bound.cpp (two passes are necessary and sufficient).
TEST(StaticTopology, bidirectional_cascade_typechecks_because_its_tree_is_acyclic)
{
  static_assert(st::is_valid_node_v<l3>);
  static_assert(st::node_count<l3>() == 4);
  static_assert(st::has_reference_edge<l3, l0>());

  // The static predicate agrees that no extension of this chain revisits a node.
  static_assert(st::is_acyclic_extension_v<l3_name, l2>);
  // Reusing the root would be rejected -- verified by the negative corpus, and here by the
  // predicate without provoking a hard error.
  static_assert(!st::is_acyclic_extension_v<l0_name, l3>);
  static_assert(!st::is_acyclic_extension_v<l1_name, l3>);
  static_assert(!st::is_acyclic_extension_v<l3_name, l3>);  // self-loop

  SUCCEED();
}

/// BOUNDARY OF THE GUARANTEE. The header models the construction (tree/reference) graph. A state
/// edge that points at a NON-ancestor -- a shared child consumed by a different parent -- is not
/// expressible here at all. This test documents that limitation explicitly: the type system has
/// nothing to say about such an edge, so it neither permits nor rejects it. Those edges remain the
/// runtime checker's responsibility.
///
/// Concretely: `shared` below can be a descendant of `parent_a` only; the fact that `parent_b`
/// might also consume `shared`'s state is invisible to this header.
TEST(StaticTopology, non_ancestor_state_edges_are_outside_the_static_guarantee)
{
  static_assert(st::is_valid_node_v<shared>);
  // `shared` is NOT an ancestor of parent_b, so this header has no edge to report. A real system
  // could still have parent_b consume shared's state; that is exactly the case the runtime
  // checker (build_controller_hierarchy) has to cover.
  static_assert(!st::has_reference_edge<parent_b, shared>());
  static_assert(!st::has_reference_edge<shared, parent_b>());

  SUCCEED();
}
