// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// ACCEPTANCE TEST: one typed declaration of a BRANCHING tree drives the whole runtime plan.
//
// This is review item B. The compile-time binding used to have a single child slot, so it could only
// express a chain even though the runtime kernel has always supported multi-child nodes; and the
// parent-side port declarations were compared against ONE child, which cannot express per-child
// routing. Here one declaration describes
//
//     chassis -> {left_module, right_module} -> {a, b} each          (7 nodes)
//
// and the test drives the SAME declaration through every layer it is supposed to cover:
//
//   * compile time : ownership, nesting vs declared parent, and both port edges per child;
//   * build time   : the checked group entry, which also verifies each node's runtime port strings;
//   * run time     : state stage children-before-parents, command stage parents-before-children,
//                    exactly one call per node per phase, and the NUMERIC value each named port
//                    carries in that same cycle.
//
// The sibling order is then reversed (children swapped AND the parent's declaration lists swapped
// with them) and the whole check is repeated, so the result is verified through port NAMES rather
// than through an accidental match of positions.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hierarchical_control/topology_binding.hpp"
#include "hierarchical_control/typed_ports.hpp"
#include "test_controller_stub.hpp"

namespace tp = hierarchical_control::typed_ports;
namespace tc = hierarchical_control::topology_contract;
namespace tb = hierarchical_control::topology_binding;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;
using Return = controller_interface::return_type;

namespace
{
// ---------------------------------------------------------------------------------------------
// Topology: chassis -> {left_module, right_module} -> {a, b}
// ---------------------------------------------------------------------------------------------

#define NAME_STRUCT(struct_name, text)      \
  struct struct_name                        \
  {                                         \
    static constexpr auto value = st::NameOf(text); \
  }

NAME_STRUCT(chassis_n, "chassis");
NAME_STRUCT(left_module_n, "left_module");
NAME_STRUCT(right_module_n, "right_module");
NAME_STRUCT(left_a_n, "left_a");
NAME_STRUCT(left_b_n, "left_b");
NAME_STRUCT(right_a_n, "right_a");
NAME_STRUCT(right_b_n, "right_b");

using chassis_node = st::Root<chassis_n>;
using left_module_node = st::Descendant<left_module_n, chassis_node>;
using right_module_node = st::Descendant<right_module_n, chassis_node>;
using left_a_node = st::Descendant<left_a_n, left_module_node>;
using left_b_node = st::Descendant<left_b_n, left_module_node>;
using right_a_node = st::Descendant<right_a_n, right_module_node>;
using right_b_node = st::Descendant<right_b_n, right_module_node>;

// ---------------------------------------------------------------------------------------------
// Ports: every name is "<owner>/<local>", owned by a node of this tree
// ---------------------------------------------------------------------------------------------

NAME_STRUCT(chassis_state_n, "chassis/state");
NAME_STRUCT(chassis_target_n, "chassis/target");
NAME_STRUCT(left_module_state_n, "left_module/state");
NAME_STRUCT(left_module_target_n, "left_module/target");
NAME_STRUCT(right_module_state_n, "right_module/state");
NAME_STRUCT(right_module_target_n, "right_module/target");
NAME_STRUCT(left_a_state_n, "left_a/state");
NAME_STRUCT(left_a_target_n, "left_a/target");
NAME_STRUCT(left_a_torque_n, "left_a/torque");
NAME_STRUCT(left_b_state_n, "left_b/state");
NAME_STRUCT(left_b_target_n, "left_b/target");
NAME_STRUCT(left_b_torque_n, "left_b/torque");
NAME_STRUCT(right_a_state_n, "right_a/state");
NAME_STRUCT(right_a_target_n, "right_a/target");
NAME_STRUCT(right_a_torque_n, "right_a/torque");
NAME_STRUCT(right_b_state_n, "right_b/state");
NAME_STRUCT(right_b_target_n, "right_b/target");
NAME_STRUCT(right_b_torque_n, "right_b/torque");

using chassis_state = tc::Port<chassis_state_n, dm::Position>;
using chassis_target = tc::Port<chassis_target_n, dm::LinearVelocity>;
using left_module_state = tc::Port<left_module_state_n, dm::Position>;
using left_module_target = tc::Port<left_module_target_n, dm::LinearVelocity>;
using right_module_state = tc::Port<right_module_state_n, dm::Position>;
using right_module_target = tc::Port<right_module_target_n, dm::LinearVelocity>;
using left_a_state = tc::Port<left_a_state_n, dm::Position>;
using left_a_target = tc::Port<left_a_target_n, dm::LinearVelocity>;
using left_a_torque = tc::Port<left_a_torque_n, dm::Torque>;
using left_b_state = tc::Port<left_b_state_n, dm::Position>;
using left_b_target = tc::Port<left_b_target_n, dm::LinearVelocity>;
using left_b_torque = tc::Port<left_b_torque_n, dm::Torque>;
using right_a_state = tc::Port<right_a_state_n, dm::Position>;
using right_a_target = tc::Port<right_a_target_n, dm::LinearVelocity>;
using right_a_torque = tc::Port<right_a_torque_n, dm::Torque>;
using right_b_state = tc::Port<right_b_state_n, dm::Position>;
using right_b_target = tc::Port<right_b_target_n, dm::LinearVelocity>;
using right_b_torque = tc::Port<right_b_torque_n, dm::Torque>;

// ---------------------------------------------------------------------------------------------
// The declarations, in CHILD ORDER. Swapping the two siblings means swapping every list below
// consistently -- which is exactly what the second half of the test does.
// ---------------------------------------------------------------------------------------------

using chassis_ports = tp::TypedPorts<
  tc::PortList<chassis_state>, tc::PortList<chassis_target>, tc::PortList<>,
  tc::PortList<left_module_target, right_module_target>,
  tc::PortList<left_module_state, right_module_state>>;

using left_module_ports = tp::TypedPorts<
  tc::PortList<left_module_state>, tc::PortList<left_module_target>, tc::PortList<>,
  tc::PortList<left_a_target, left_b_target>, tc::PortList<left_a_state, left_b_state>>;

using right_module_ports = tp::TypedPorts<
  tc::PortList<right_module_state>, tc::PortList<right_module_target>, tc::PortList<>,
  tc::PortList<right_a_target, right_b_target>, tc::PortList<right_a_state, right_b_state>>;

using left_a_ports =
  tp::TypedPorts<tc::PortList<left_a_state>, tc::PortList<left_a_target>, tc::PortList<left_a_torque>>;
using left_b_ports =
  tp::TypedPorts<tc::PortList<left_b_state>, tc::PortList<left_b_target>, tc::PortList<left_b_torque>>;
using right_a_ports = tp::TypedPorts<
  tc::PortList<right_a_state>, tc::PortList<right_a_target>, tc::PortList<right_a_torque>>;
using right_b_ports = tp::TypedPorts<
  tc::PortList<right_b_state>, tc::PortList<right_b_target>, tc::PortList<right_b_torque>>;

// `compose` / `make_leaf` take a `topology_contract::Contract`; the typed declaration implies one.
// Writing the contract as `contract_of_t<ports>` states the ports exactly ONCE (in the TypedPorts
// above) while still giving `topology_contract` the names and dimensions it checks.
using chassis_contract = tp::contract_of_t<chassis_ports>;
using left_module_contract = tp::contract_of_t<left_module_ports>;
using right_module_contract = tp::contract_of_t<right_module_ports>;
using left_a_contract = tp::contract_of_t<left_a_ports>;
using left_b_contract = tp::contract_of_t<left_b_ports>;
using right_a_contract = tp::contract_of_t<right_a_ports>;
using right_b_contract = tp::contract_of_t<right_b_ports>;

// The SAME tree with every pair of siblings declared in the opposite order. Only the parent-side
// lists change (they must follow child order); the node names, the ports and the numbers do not.
// A composite's type therefore differs, which is the point: the order is a declaration, and the
// checks bind the declaration to the tree rather than to a lucky positional match.
using chassis_ports_swapped = tp::TypedPorts<
  tc::PortList<chassis_state>, tc::PortList<chassis_target>, tc::PortList<>,
  tc::PortList<right_module_target, left_module_target>,
  tc::PortList<right_module_state, left_module_state>>;
using left_module_ports_swapped = tp::TypedPorts<
  tc::PortList<left_module_state>, tc::PortList<left_module_target>, tc::PortList<>,
  tc::PortList<left_b_target, left_a_target>, tc::PortList<left_b_state, left_a_state>>;
using right_module_ports_swapped = tp::TypedPorts<
  tc::PortList<right_module_state>, tc::PortList<right_module_target>, tc::PortList<>,
  tc::PortList<right_b_target, right_a_target>, tc::PortList<right_b_state, right_a_state>>;

using chassis_contract_swapped = tp::contract_of_t<chassis_ports_swapped>;
using left_module_contract_swapped = tp::contract_of_t<left_module_ports_swapped>;
using right_module_contract_swapped = tp::contract_of_t<right_module_ports_swapped>;

// ---------------------------------------------------------------------------------------------
// One controller class, instantiated with each node's declaration
// ---------------------------------------------------------------------------------------------

/// Shared phase counter, so the test can see the ORDER of stages across nodes.
int g_sequence = 0;

/// Minimal sink / source the kernel requires for a node with actuator ports / a root reference.
class Sink : public hierarchical_control::StagedCommandSink
{
public:
  bool commit(const double * values, std::size_t size) noexcept override
  {
    if (size > 0) {last = values[0];}
    ++commits;
    return true;
  }
  int commits = 0;
  double last = 0.0;
};

class Source : public hierarchical_control::StagedReferenceSource
{
public:
  explicit Source(double value) : value_(value) {}
  bool read(std::uint64_t, std::int64_t, double * values, std::size_t size) noexcept override
  {
    for (std::size_t i = 0; i < size; ++i) {values[i] = value_;}
    return true;
  }
  double value_ = 0.0;
};

/// `bias` distinguishes the nodes numerically. The formulas are deliberately position-independent:
/// a state is "sum of the child states I was GIVEN plus my bias", and a command writes the received
/// reference into every child writer and every actuator. A routing mistake therefore changes the
/// numbers, not just the call order.
template <typename Ports>
class TreeController : public hierarchical_control_test::MinimalController,
                       public tp::TypedPortsMixin<TreeController<Ports>, Ports>
{
public:
  TreeController(std::string name, double bias, double source = 0.0)
  : MinimalController(std::move(name)), source_value_(source), bias_(bias), source_(source)
  {
  }

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedInputView & children,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    sequence_at_state = ++g_sequence;
    double sum = 0.0;
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {sum += children[c][p];}
    }
    if (state.size() > 0) {state[0] = sum + bias_;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView & reference,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    sequence_at_command = ++g_sequence;
    const double value = reference.size() > 0 ? reference[0] : source_;
    command_value = value;
    // The command stage of the SAME cycle sees the state stage's output, so recording it here is how
    // the test observes same-cycle upward propagation by node NAME.
    command_state_value = state.size() > 0 ? state[0] : std::numeric_limits<double>::quiet_NaN();
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = value;}
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = value;}
    return Return::OK;
  }

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return &sink_;
  }

  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return &source_value_;
  }

  int state_calls = 0;
  int command_calls = 0;
  int sequence_at_state = 0;
  int sequence_at_command = 0;
  double command_value = 0.0;
  double command_state_value = 0.0;
  Sink sink_;
  Source source_value_{0.0};

private:
  double bias_ = 0.0;
  double source_ = 0.0;
};

using Chassis = TreeController<chassis_ports>;
using LeftModule = TreeController<left_module_ports>;
using RightModule = TreeController<right_module_ports>;
using LeftA = TreeController<left_a_ports>;
using LeftB = TreeController<left_b_ports>;
using RightA = TreeController<right_a_ports>;
using RightB = TreeController<right_b_ports>;

/// The seven instances and the tree built from them. Kept in one struct so the test can build it
/// with either sibling order.
struct Tree
{
  Chassis chassis{"chassis", 100.0, 42.0};
  LeftModule left_module{"left_module", 10.0};
  RightModule right_module{"right_module", 20.0};
  LeftA left_a{"left_a", 1.0};
  LeftB left_b{"left_b", 2.0};
  RightA right_a{"right_a", 3.0};
  RightB right_b{"right_b", 4.0};

  /// Declaration order: left subtree first, then right. The parent's lists follow the same order.
  /// Not `const`: the binding stores non-const `ControllerInterfaceBase *` instances.
  auto binding()
  {
    const auto left_b_leaf = tc::make_leaf<left_b_node, left_b_contract>(&left_b);
    const auto left_a_leaf = tc::make_leaf<left_a_node, left_a_contract>(&left_a);
    const auto left =
      tc::compose<left_module_node, left_module_contract>(&left_module, left_a_leaf, left_b_leaf);
    const auto right_b_leaf = tc::make_leaf<right_b_node, right_b_contract>(&right_b);
    const auto right_a_leaf = tc::make_leaf<right_a_node, right_a_contract>(&right_a);
    const auto right =
      tc::compose<right_module_node, right_module_contract>(&right_module, right_a_leaf, right_b_leaf);
    return tc::compose<chassis_node, chassis_contract>(&chassis, left, right);
  }
};

// The seven controllers have seven DISTINCT types, so a braced initializer list cannot deduce a
// common element type. A visitor keeps the type of every member and still visits all of them.
template <typename F>
void ForEachNode(Tree & tree, F && visit)
{
  visit(tree.chassis);
  visit(tree.left_module);
  visit(tree.right_module);
  visit(tree.left_a);
  visit(tree.left_b);
  visit(tree.right_a);
  visit(tree.right_b);
}

template <typename F>
void ForEachNode(const Tree & tree, F && visit)
{
  visit(tree.chassis);
  visit(tree.left_module);
  visit(tree.right_module);
  visit(tree.left_a);
  visit(tree.left_b);
  visit(tree.right_a);
  visit(tree.right_b);
}

/// Analytic expectation, derived from the same formulas, per NODE NAME:
///   state(a) = 1, state(b) = 2, state(right_a) = 3, state(right_b) = 4
///   state(left_module)  = state(left_a) + state(left_b) + 10
///   state(right_module) = state(right_a) + state(right_b) + 20
///   state(chassis)      = state(left_module) + state(right_module) + 100
///   every command value = the root's external reference (42.0), propagated unchanged
constexpr double kReference = 42.0;

void ExpectOneCyclePerNode(const Tree & tree)
{
  ForEachNode(tree, [](const auto & controller) {
    EXPECT_EQ(1, controller.state_calls);
    EXPECT_EQ(1, controller.command_calls);
  });
}

/// Run one cycle and check the plan, the phase order and the per-name numbers.
void RunAndCheck(Tree & tree)
{
  ForEachNode(tree, [](auto & controller) {
    controller.state_calls = 0;
    controller.command_calls = 0;
  });
  g_sequence = 0;

  const auto binding = tree.binding();
  const auto rows = tc::build_spec_rows(binding);

  // The static declaration produced the whole tree: seven nodes, and every parent edge names the
  // node that actually encloses the child.
  ASSERT_EQ(7u, rows.names.size());
  EXPECT_EQ("", rows.parents[0]);
  EXPECT_EQ(std::vector<std::string>({"chassis", "left_module", "left_a", "left_b", "right_module", "right_a", "right_b"}),
            rows.names);
  EXPECT_EQ(
    std::vector<std::string>({"", "chassis", "left_module", "left_module", "chassis", "right_module", "right_module"}),
    rows.parents);

  auto group = tb::create_library_group(binding, 0);
  ASSERT_NE(nullptr, group);
  ASSERT_EQ(7u, group->size());

  const auto result = group->run_ns(1000, 1000);
  ASSERT_EQ(hierarchical_control::StagedStatus::committed, result.status)
    << "status " << static_cast<int>(result.status) << " at node " << result.failed_node;

  ExpectOneCyclePerNode(tree);

  // State stage: every child's state stage runs before its parent's (postorder).
  EXPECT_LT(tree.left_a.sequence_at_state, tree.left_module.sequence_at_state);
  EXPECT_LT(tree.left_b.sequence_at_state, tree.left_module.sequence_at_state);
  EXPECT_LT(tree.right_a.sequence_at_state, tree.right_module.sequence_at_state);
  EXPECT_LT(tree.right_b.sequence_at_state, tree.right_module.sequence_at_state);
  EXPECT_LT(tree.left_module.sequence_at_state, tree.chassis.sequence_at_state);
  EXPECT_LT(tree.right_module.sequence_at_state, tree.chassis.sequence_at_state);
  // Every state stage precedes every command stage. Both extrema are taken over ALL SEVEN nodes,
  // not just the composites, so a leaf whose command ran before another node's state would fail.
  const int last_state = std::max(
    {tree.chassis.sequence_at_state, tree.left_module.sequence_at_state,
     tree.right_module.sequence_at_state, tree.left_a.sequence_at_state, tree.left_b.sequence_at_state,
     tree.right_a.sequence_at_state, tree.right_b.sequence_at_state});
  const int first_command = std::min(
    {tree.chassis.sequence_at_command, tree.left_module.sequence_at_command,
     tree.right_module.sequence_at_command, tree.left_a.sequence_at_command,
     tree.left_b.sequence_at_command, tree.right_a.sequence_at_command,
     tree.right_b.sequence_at_command});
  EXPECT_LT(last_state, first_command);

  // Command stage: every parent's command stage runs before its children's (preorder).
  EXPECT_LT(tree.chassis.sequence_at_command, tree.left_module.sequence_at_command);
  EXPECT_LT(tree.chassis.sequence_at_command, tree.right_module.sequence_at_command);
  EXPECT_LT(tree.left_module.sequence_at_command, tree.left_a.sequence_at_command);
  EXPECT_LT(tree.left_module.sequence_at_command, tree.left_b.sequence_at_command);
  EXPECT_LT(tree.right_module.sequence_at_command, tree.right_a.sequence_at_command);
  EXPECT_LT(tree.right_module.sequence_at_command, tree.right_b.sequence_at_command);

  // Same-cycle NUMERIC propagation, by node name. Each command stage sees the state stage's output
  // for the SAME cycle, so `command_state_value` is the value that node actually produced:
  //   leaf state         = its own bias                    (no children)
  //   left_module state  = 1 + 2 + 10, right_module = 3 + 4 + 20
  //   chassis state      = 13 + 27 + 100
  const double left_module_expected = 1.0 + 2.0 + 10.0;
  const double right_module_expected = 3.0 + 4.0 + 20.0;
  const double chassis_expected = left_module_expected + right_module_expected + 100.0;
  EXPECT_DOUBLE_EQ(1.0, tree.left_a.command_state_value);
  EXPECT_DOUBLE_EQ(2.0, tree.left_b.command_state_value);
  EXPECT_DOUBLE_EQ(3.0, tree.right_a.command_state_value);
  EXPECT_DOUBLE_EQ(4.0, tree.right_b.command_state_value);
  EXPECT_DOUBLE_EQ(left_module_expected, tree.left_module.command_state_value);
  EXPECT_DOUBLE_EQ(right_module_expected, tree.right_module.command_state_value);
  EXPECT_DOUBLE_EQ(chassis_expected, tree.chassis.command_state_value);

  const auto & committed = group->committed_actuators();
  // Actuator slots are ordered by leaf; the leaf order is the plan's, so assert through the sinks
  // (each leaf has exactly one actuator port) rather than by index.
  EXPECT_EQ(4, tree.left_a.sink_.commits + tree.left_b.sink_.commits +
                 tree.right_a.sink_.commits + tree.right_b.sink_.commits);
  EXPECT_EQ(kReference, tree.left_a.sink_.last);
  EXPECT_EQ(kReference, tree.left_b.sink_.last);
  EXPECT_EQ(kReference, tree.right_a.sink_.last);
  EXPECT_EQ(kReference, tree.right_b.sink_.last);
  EXPECT_EQ(kReference, tree.left_module.command_value);
  EXPECT_EQ(kReference, tree.chassis.command_value);
  EXPECT_EQ(4u, committed.size());

  // The root's external reference is unchanged by the trip down the tree, and the leaves are the
  // only nodes with actuators, so the committed hardware command is that reference.
  for (double value : committed) {EXPECT_DOUBLE_EQ(kReference, value);}
}
}  // namespace

/// One typed declaration of a branching tree produces the whole plan, and one cycle through the
/// checked group propagates state upward and commands downward in the same cycle, per NAMED port.
TEST(TypedTree, one_declaration_drives_the_whole_branching_tree)
{
  Tree tree;
  RunAndCheck(tree);
}

/// The same tree with the two sibling subtrees declared in the OPPOSITE order still builds and still
/// produces the analytic result per name. Positions change; the names and the values do not.
TEST(TypedTree, sibling_order_is_a_declaration_not_a_lucky_position)
{
  // Swapping the siblings means swapping every parent-side list consistently, which is what the
  // declaration requires: the routing follows child order, so the declaration must follow it too.
  struct SwappedTree
  {
    TreeController<chassis_ports_swapped> chassis{"chassis", 100.0, 42.0};
    TreeController<left_module_ports_swapped> left_module{"left_module", 10.0};
    TreeController<right_module_ports_swapped> right_module{"right_module", 20.0};
    LeftA left_a{"left_a", 1.0};
    LeftB left_b{"left_b", 2.0};
    RightA right_a{"right_a", 3.0};
    RightB right_b{"right_b", 4.0};

    auto binding()
    {
      const auto left_b_leaf = tc::make_leaf<left_b_node, left_b_contract>(&left_b);
      const auto left_a_leaf = tc::make_leaf<left_a_node, left_a_contract>(&left_a);
      const auto left = tc::compose<left_module_node, left_module_contract_swapped>(
        &left_module, left_b_leaf, left_a_leaf);
      const auto right_b_leaf = tc::make_leaf<right_b_node, right_b_contract>(&right_b);
      const auto right_a_leaf = tc::make_leaf<right_a_node, right_a_contract>(&right_a);
      const auto right = tc::compose<right_module_node, right_module_contract_swapped>(
        &right_module, right_b_leaf, right_a_leaf);
      return tc::compose<chassis_node, chassis_contract_swapped>(&chassis, right, left);
    }
  };

  SwappedTree tree;
  const auto binding = tree.binding();
  const auto rows = tc::build_spec_rows(binding);
  ASSERT_EQ(7u, rows.names.size());
  // The emitted order follows the new declaration, and every parent edge is still the enclosing node.
  EXPECT_EQ(
    std::vector<std::string>({"chassis", "right_module", "right_b", "right_a", "left_module", "left_b", "left_a"}),
    rows.names);
  EXPECT_EQ(
    std::vector<std::string>({"", "chassis", "right_module", "right_module", "chassis", "left_module", "left_module"}),
    rows.parents);

  auto group = tb::create_library_group(binding, 0);
  ASSERT_NE(nullptr, group);
  const auto result = group->run_ns(1000, 1000);
  ASSERT_EQ(hierarchical_control::StagedStatus::committed, result.status);

  // Values per NAME are unchanged by the reordering -- state as well as command.
  EXPECT_DOUBLE_EQ(1.0, tree.left_a.command_state_value);
  EXPECT_DOUBLE_EQ(2.0, tree.left_b.command_state_value);
  EXPECT_DOUBLE_EQ(3.0, tree.right_a.command_state_value);
  EXPECT_DOUBLE_EQ(4.0, tree.right_b.command_state_value);
  EXPECT_DOUBLE_EQ(13.0, tree.left_module.command_state_value);
  EXPECT_DOUBLE_EQ(27.0, tree.right_module.command_state_value);
  EXPECT_DOUBLE_EQ(140.0, tree.chassis.command_state_value);
  EXPECT_EQ(kReference, tree.left_a.sink_.last);
  EXPECT_EQ(kReference, tree.left_b.sink_.last);
  EXPECT_EQ(kReference, tree.right_a.sink_.last);
  EXPECT_EQ(kReference, tree.right_b.sink_.last);
}
