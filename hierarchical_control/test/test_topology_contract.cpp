// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Joining the compile-time topology to the runtime execution group.
//
// Three static facilities now exist and this test is where they meet:
//   * static_topology.hpp          -- acyclic hierarchy as a type chain
//   * dimensional_interfaces.hpp   -- physical dimensions on interfaces
//   * topology_contract.hpp        -- one binding that drives BOTH the compile-time checks and the
//                                    runtime Spec rows the kernel consumes
//
// The runtime kernel (StagedExecutionGroup::create_library) needs three parallel vectors: names,
// instances, parents. `build_spec_rows` derives them from the compile-time binding, so the plan
// cannot drift from the topology it was checked against.
//
// The check that matters most here is OWNERSHIP: every port is named "<owner>/<local>" and every
// owner must be a controller in the topology. That rejects the defect this project measured --
// a declared interface that nothing in the group exports
// (doc/BIDIRECTIONAL_EDGE_ANALYSIS.md section 9). The rejecting direction is a hard compile error,
// so it lives in test/static_topology_negative/ and is verified by
// test_static_topology_negative.py.

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>

#include "hierarchical_control/topology_contract.hpp"
#include "test_controller_stub.hpp"

namespace tc = hierarchical_control::topology_contract;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

namespace
{
struct chassis_n
{
  static constexpr auto value = st::NameOf("chassis");
};
struct wheel_n
{
  static constexpr auto value = st::NameOf("wheel");
};
struct tire_n
{
  static constexpr auto value = st::NameOf("tire");
};

using chassis = st::Root<chassis_n>;
using wheel = st::Descendant<wheel_n, chassis>;
using tire = st::Descendant<tire_n, wheel>;

// Port names follow ros2_control's "<owner>/<local>" convention.
struct wheel_target_n
{
  static constexpr auto value = st::NameOf("wheel/target");
};
struct wheel_travel_n
{
  static constexpr auto value = st::NameOf("wheel/travel");
};
struct tire_target_n
{
  static constexpr auto value = st::NameOf("tire/target");
};

using wheel_target = tc::Port<wheel_target_n, dm::LinearVelocity>;
using wheel_travel = tc::Port<wheel_travel_n, dm::Position>;
using tire_target = tc::Port<tire_target_n, dm::LinearVelocity>;

// chassis: commands wheel/target, consumes wheel/travel
using chassis_contract = tc::Contract<tc::PortList<wheel_target>, tc::PortList<wheel_travel>>;
// wheel: exports wheel/travel (state for its parent) and tire/target (reference for its child),
//        consumes wheel/target (its own reference)
using wheel_contract =
  tc::Contract<tc::PortList<wheel_travel, tire_target>, tc::PortList<wheel_target>>;
// tire: consumes tire/target
using tire_contract = tc::Contract<tc::PortList<>, tc::PortList<tire_target>>;

// Real controller objects: the binding stores a TYPED ControllerInterfaceBase*, so a placeholder
// integer no longer compiles (that is the intended tightening, see review R5).
hierarchical_control_test::MinimalController chassis_instance{"chassis"};
hierarchical_control_test::MinimalController wheel_instance{"wheel"};
hierarchical_control_test::MinimalController tire_instance{"tire"};

const auto leaf = tc::make_leaf<tire, tire_contract>(&tire_instance);
const auto mid = tc::compose<wheel, wheel_contract>(&wheel_instance, leaf);
const auto root = tc::compose<chassis, chassis_contract>(&chassis_instance, mid);
}  // namespace

/// The ownership invariant holds for a well-formed topology, and is checked at compile time.
TEST(TopologyContract, ownership_is_checked_at_compile_time)
{
  tc::require_ports_are_owned<decltype(root)>();
  SUCCEED();
}

/// The binding carries the topology shape, so depth is a compile-time constant.
TEST(TopologyContract, binding_depth_is_a_compile_time_constant)
{
  static_assert(tc::binding_depth<decltype(root)>() == 3);
  // Children are a parameter pack, so a node knows how many it has and can address each one.
  static_assert(decltype(root)::has_child);
  static_assert(decltype(root)::child_count == 1u);
  static_assert(decltype(root)::template child<0>(root).has_child);
  static_assert(!decltype(leaf)::has_child);
  static_assert(decltype(leaf)::child_count == 0u);
  static_assert(decltype(root)::name() == std::string_view{"chassis"});
  static_assert(decltype(leaf)::name() == std::string_view{"tire"});
  SUCCEED();
}

/// The runtime Spec rows are derived from the same binding, root first, with the right parents.
TEST(TopologyContract, spec_rows_match_the_compile_time_topology)
{
  const auto rows = tc::build_spec_rows(root);

  ASSERT_EQ(3u, rows.names.size());
  EXPECT_EQ("chassis", rows.names[0]);
  EXPECT_EQ("wheel", rows.names[1]);
  EXPECT_EQ("tire", rows.names[2]);

  EXPECT_EQ("", rows.parents[0]);  // root
  EXPECT_EQ("chassis", rows.parents[1]);
  EXPECT_EQ("wheel", rows.parents[2]);

  // Instances are carried through unchanged, so the kernel gets the caller's controllers.
  EXPECT_EQ(static_cast<controller_interface::ControllerInterfaceBase *>(&chassis_instance),
            rows.instances[0]);
  EXPECT_EQ(static_cast<controller_interface::ControllerInterfaceBase *>(&wheel_instance),
            rows.instances[1]);
  EXPECT_EQ(static_cast<controller_interface::ControllerInterfaceBase *>(&tire_instance),
            rows.instances[2]);
}

/// The derived rows satisfy the same invariants the kernel enforces, so a binding cannot produce a
/// plan the kernel would reject.
TEST(TopologyContract, derived_rows_satisfy_the_kernel_invariants)
{
  const auto rows = tc::build_spec_rows(root);
  std::string reason;
  EXPECT_TRUE(tc::rows_are_well_formed(rows, &reason)) << reason;
}

/// The well-formedness checker is not vacuous: it rejects each invariant it claims to enforce.
TEST(TopologyContract, well_formedness_checker_rejects_malformed_rows)
{
  const auto good = tc::build_spec_rows(root);
  std::string reason;

  {  // unequal lengths
    auto rows = good;
    rows.parents.pop_back();
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // no root
    auto rows = good;
    rows.parents[0] = "wheel";
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // two roots
    auto rows = good;
    rows.parents[1] = "";
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // duplicate names
    auto rows = good;
    rows.names[2] = "wheel";
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // a node that is its own parent
    auto rows = good;
    rows.parents[1] = "wheel";
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // parent that does not exist
    auto rows = good;
    rows.parents[2] = "ghost";
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
  {  // empty
    tc::SpecRows rows;
    EXPECT_FALSE(tc::rows_are_well_formed(rows, &reason));
  }
}

/// A leaf-only topology is a legal degenerate case: one root, no children.
TEST(TopologyContract, a_single_node_topology_is_legal)
{
  const auto only = tc::make_leaf<chassis, chassis_contract>(&chassis_instance);
  static_assert(tc::binding_depth<decltype(only)>() == 1);
  const auto rows = tc::build_spec_rows(only);
  ASSERT_EQ(1u, rows.names.size());
  EXPECT_EQ("chassis", rows.names[0]);
  EXPECT_EQ("", rows.parents[0]);
  std::string reason;
  EXPECT_TRUE(tc::rows_are_well_formed(rows, &reason)) << reason;
}

/// `owner_of` extracts the controller name the interface belongs to. This is the primitive the
/// ownership check is built on, so it is pinned directly.
TEST(TopologyContract, owner_extraction_follows_the_slash_convention)
{
  EXPECT_EQ(std::string_view{"wheel"}, tc::owner_of("wheel/target"));
  EXPECT_EQ(std::string_view{"wheel"}, tc::owner_of("wheel/a/b"));  // first separator wins
  EXPECT_EQ(std::string_view{}, tc::owner_of("target"));            // no owner
  EXPECT_EQ(std::string_view{}, tc::owner_of("/target"));           // empty owner
  SUCCEED();
}
