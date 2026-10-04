// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Tests for the compile-time description of a DECLARED two-phase tree.
//
// The point of the description is that three of the manager's runtime admission facts are
// properties of a TYPE when the whole tree is compiled in: which nodes are members, which
// reference edges exist, and that every parent precedes its children. This file pins each of
// them, and -- more importantly -- pins that the compile-time edge set AGREES with the edges the
// manager would infer at run time by splitting "<owner>/..." interface strings. That equality is
// the falsifiable core: if the two ever disagree, the static description is worthless.

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/static_two_phase_admission.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/typed_ports.hpp"
#include "test_controller_stub.hpp"

namespace
{
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;
namespace s2 = hierarchical_control::static_two_phase;
using hierarchical_control_test::MinimalController;

// ---- topology: root -> {left, right} ---------------------------------------------------------

struct root_n
{
  static constexpr auto value = st::NameOf("tp_root");
};
struct left_n
{
  static constexpr auto value = st::NameOf("tp_left");
};
struct right_n
{
  static constexpr auto value = st::NameOf("tp_right");
};

using root_node = st::Root<root_n>;
using left_node = st::Descendant<left_n, root_node>;
using right_node = st::Descendant<right_n, root_node>;

// ---- the children's declarations --------------------------------------------------------------

struct left_ref_n
{
  static constexpr auto value = st::NameOf("tp_left/ref");
};
struct left_state_n
{
  static constexpr auto value = st::NameOf("tp_left/state");
};
struct right_ref_n
{
  static constexpr auto value = st::NameOf("tp_right/ref");
};
struct right_state_n
{
  static constexpr auto value = st::NameOf("tp_right/state");
};

using left_ref = tc::Port<left_ref_n, dm::LinearVelocity>;
using left_state = tc::Port<left_state_n, dm::Position>;
using right_ref = tc::Port<right_ref_n, dm::LinearVelocity>;
using right_state = tc::Port<right_state_n, dm::Position>;

using left_ports =
  tp::TypedPorts<tc::PortList<left_state>, tc::PortList<left_ref>, tc::PortList<>>;
using right_ports =
  tp::TypedPorts<tc::PortList<right_state>, tc::PortList<right_ref>, tc::PortList<>>;

/// The root declares exactly the concatenation, in child order, of what its children declare.
using root_ports = tp::TypedPorts<
  tc::PortList<>, tc::PortList<>, tc::PortList<>, tc::PortList<left_ref, right_ref>,
  tc::PortList<left_state, right_state>>;

// ---- two stubs: one implements the two-phase interface, one does not --------------------------

/// `TypedPortsMixin` GENERATES the runtime interface strings from `Ports`; that generated set is the
/// runtime side of the equivalence test below. The two stage entry points are required by
/// `StagedControllerInterface` (the mixin's base); no cycle is run here.
template <typename Ports>
class TypedStub : public MinimalController, public tp::TypedPortsMixin<TypedStub<Ports>, Ports>
{
public:
  explicit TypedStub(std::string name) : MinimalController(std::move(name)) {}

  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return controller_interface::return_type::OK;
  }
};

template <typename Ports>
class TwoPhaseTypedStub : public TypedStub<Ports>, public hierarchical_control::TwoPhaseControllerInterface
{
public:
  explicit TwoPhaseTypedStub(std::string name) : TypedStub<Ports>(std::move(name)) {}

  controller_interface::return_type update_phase(
    const rclcpp::Time &, const rclcpp::Duration &) noexcept override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type handle_phase(
    const rclcpp::Time &, const rclcpp::Duration &) noexcept override
  {
    return controller_interface::return_type::OK;
  }
};

inline TwoPhaseTypedStub<root_ports> g_root{"tp_root"};
inline TwoPhaseTypedStub<left_ports> g_left{"tp_left"};
inline TwoPhaseTypedStub<right_ports> g_right{"tp_right"};

using binding_type = decltype(tc::compose<root_node, tp::contract_of_t<root_ports>>(
  &g_root, tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&g_left),
  tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&g_right)));

// A structurally identical tree whose nodes do NOT implement the interface.
inline TypedStub<root_ports> g_root_plain{"tp_root"};
inline TypedStub<left_ports> g_left_plain{"tp_left"};
inline TypedStub<right_ports> g_right_plain{"tp_right"};

using non_member_binding = decltype(tc::compose<root_node, tp::contract_of_t<root_ports>>(
  &g_root_plain, tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&g_left_plain),
  tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&g_right_plain)));

// ---- compile-time facts ----------------------------------------------------------------------

using description = s2::tree_description<binding_type>;

static_assert(s2::subtree_all_members<binding_type>::value, "all three nodes implement the interface");
static_assert(
  !s2::subtree_all_members<non_member_binding>::value,
  "a node without the interface must be detected -- this is the trait that replaces dynamic_cast");

static_assert(description::member_count == 3, "root plus two leaves");
static_assert(description::edge_count == 2, "one edge per child");
static_assert(description::members[0] == std::string_view("tp_root"));
static_assert(description::members[1] == std::string_view("tp_left"));
static_assert(description::members[2] == std::string_view("tp_right"));
static_assert(description::index_of("tp_left") == 1, "pre-order index");
static_assert(description::index_of("nope") == description::member_count, "absent name");
static_assert(description::edges_are_ordered(), "a compose-built tree lists parents first");
static_assert(description::edges[0].parent == std::string_view("tp_root"));
static_assert(description::edges[0].child == std::string_view("tp_left"));
static_assert(description::edges[1].parent == std::string_view("tp_root"));
static_assert(description::edges[1].child == std::string_view("tp_right"));

/// The owner prefix of a "<owner>/<local>" interface name, exactly as the manager splits it.
std::string_view owner_of(std::string_view qualified)
{
  const auto split = qualified.find_first_of('/');
  if (split == std::string_view::npos) {return {};}
  return qualified.substr(0, split);
}
}  // namespace

/// The compile-time edge set must agree with the edges the RUNTIME inference produces: each edge is
/// the pair (parent, owner of one of the child's generated reference interface names).
TEST(StaticTwoPhaseAdmission, compile_time_edges_equal_the_runtime_inference)
{
  std::vector<std::pair<std::string, std::string>> runtime_edges;
  std::vector<std::string> claimed;
  for (const auto & name : g_left.staged_reference_ports()) {claimed.push_back(name);}
  for (const auto & name : g_right.staged_reference_ports()) {claimed.push_back(name);}
  ASSERT_FALSE(claimed.empty()) << "the fixture must declare reference ports, or there is no edge";

  // The manager walks the parent's CLAIMED interfaces and takes the owner of each as a neighbour.
  for (const auto & name : claimed)
  {
    const auto owner = owner_of(name);
    ASSERT_FALSE(owner.empty()) << "an unqualified interface name is not an edge: " << name;
    runtime_edges.emplace_back("tp_root", std::string(owner));
  }

  constexpr auto edges = s2::tree_description<binding_type>::edges;
  ASSERT_EQ(runtime_edges.size(), edges.size());
  for (std::size_t i = 0; i < edges.size(); ++i)
  {
    EXPECT_EQ(runtime_edges[i].first, edges[i].parent) << "edge " << i;
    EXPECT_EQ(runtime_edges[i].second, edges[i].child) << "edge " << i;
  }
}

/// The pre-order index is what makes `parent < child` a compile-time fact rather than the manager's
/// index comparison over a list it sorted with a string heuristic.
TEST(StaticTwoPhaseAdmission, preorder_indices_order_every_edge)
{
  constexpr auto edges = s2::tree_description<binding_type>::edges;
  for (const auto & edge : edges)
  {
    EXPECT_LT(
      s2::tree_description<binding_type>::index_of(edge.parent),
      s2::tree_description<binding_type>::index_of(edge.child))
      << edge.parent << " -> " << edge.child;
  }
}

/// The entry-point form compiles for a valid tree; the negative-compile corpus pins the other side.
TEST(StaticTwoPhaseAdmission, the_entry_point_accepts_a_declared_two_phase_tree)
{
  s2::require_two_phase_tree<binding_type>();
  SUCCEED();
}

/// The plan the kernel consumes is built from the same type as the description, so the two must
/// agree: the description's pre-order IS the execution order, and its edges ARE the plan's parents.
///
/// This is the check that has to be able to fail, because a silent disagreement would make every
/// compile-time fact about the tree worthless while still compiling.
TEST(StaticTwoPhaseAdmission, the_runtime_plan_matches_the_compile_time_description)
{
  const auto tree = tc::compose<root_node, tp::contract_of_t<root_ports>>(
    &g_root, tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&g_left),
    tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&g_right));
  const auto rows = tc::build_spec_rows(tree);

  std::string reason;
  EXPECT_TRUE(s2::plan_matches_description<binding_type>(rows, &reason)) << reason;
  EXPECT_TRUE(reason.empty());

  // The plan's order is exactly the description's pre-order, and its parents are its edges.
  ASSERT_EQ(description::member_count, rows.names.size());
  for (std::size_t i = 0; i < rows.names.size(); ++i)
  {
    EXPECT_EQ(std::string(description::members[i]), rows.names[i]) << "node " << i;
  }
}

/// A PERMUTED plan must be rejected: the node set would still be right, and both passes would then
/// walk the edge the wrong way -- the runtime `unschedulable_order` case, caught at plan build time.
TEST(StaticTwoPhaseAdmission, a_permuted_plan_is_rejected_and_names_the_first_difference)
{
  const auto tree = tc::compose<root_node, tp::contract_of_t<root_ports>>(
    &g_root, tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&g_left),
    tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&g_right));
  auto rows = tc::build_spec_rows(tree);
  ASSERT_GE(rows.names.size(), 3u);
  std::swap(rows.names[1], rows.names[2]);

  std::string reason;
  EXPECT_FALSE(s2::plan_matches_description<binding_type>(rows, &reason));
  EXPECT_NE(std::string::npos, reason.find("node 1")) << "reason: " << reason;
}

/// An edge the description does not contain must be rejected as well, so the check is not only an
/// order comparison.
TEST(StaticTwoPhaseAdmission, a_plan_edge_outside_the_description_is_rejected)
{
  const auto tree = tc::compose<root_node, tp::contract_of_t<root_ports>>(
    &g_root, tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&g_left),
    tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&g_right));
  auto rows = tc::build_spec_rows(tree);
  ASSERT_GE(rows.parents.size(), 2u);
  // Point the second child at a node that is not its parent in the declaration.
  rows.parents[2] = rows.names[1];

  std::string reason;
  EXPECT_FALSE(s2::plan_matches_description<binding_type>(rows, &reason));
  EXPECT_NE(std::string::npos, reason.find("does not contain")) << "reason: " << reason;
}
