// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// End-to-end: a compile-time binding becomes a RUNNING StagedExecutionGroup.
//
// This is the last hop of the metaprogramming chain. topology_contract.hpp produces `SpecRows`
// carrying `void*`; the kernel wants `StagedControllerInterface*`. topology_binding.hpp performs
// that adaptation and calls `create_library`.
//
// The test deliberately builds a REAL group rather than only comparing vectors, because the
// failure mode worth catching is "the adapted Spec is accepted by my own checker but rejected by
// the kernel" -- which only a real construction can reveal.
//
// The controllers below are minimal stubs: the point is the topology plumbing, not control law.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hierarchical_control/topology_binding.hpp"
#include "test_controller_stub.hpp"

namespace tc = hierarchical_control::topology_contract;
namespace tb = hierarchical_control::topology_binding;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;
using Return = controller_interface::return_type;

namespace
{
/// A minimal staged controller: no ports, one state-stage call per cycle.
/// Minimal external reference snapshot: a node that declares reference ports needs a source, or the
/// kernel refuses to build the group. The earlier fixtures declared NO runtime reference ports (the
/// default is an empty list) while their contracts declared one, which is exactly the mismatch the
/// checked build entry now refuses.
class ZeroSource : public hierarchical_control::StagedReferenceSource
{
public:
  bool read(
    std::uint64_t /*cycle*/, std::int64_t /*now_ns*/, double * values,
    std::size_t size) noexcept override
  {
    for (std::size_t i = 0; i < size; ++i) {values[i] = 0.0;}
    return true;
  }
};

class StubController : public hierarchical_control_test::MinimalController,
                       public hierarchical_control::StagedControllerInterface
{
public:
  /// `state_ports` / `reference_ports` must be exactly what this node's Contract declares, or the
  /// checked build entry rejects the binding. Pass empty lists for a contract with no ports.
  explicit StubController(
    std::string name, std::vector<std::string> state_ports = {"value"},
    std::vector<std::string> reference_ports = {})
  : MinimalController(std::move(name)),
    state_ports_(std::move(state_ports)),
    reference_ports_(std::move(reference_ports))
  {
  }

public:
  std::vector<std::string> staged_state_ports() const override { return state_ports_; }

  std::vector<std::string> staged_reference_ports() const override { return reference_ports_; }

  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return &source_;
  }

  Return update_state_stage(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
    const hierarchical_control::StagedContext & /*context*/,
    const hierarchical_control::StagedInputView & /*children*/,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    if (state.size() > 0) {state[0] = 1.0;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
    const hierarchical_control::StagedContext & /*context*/,
    const hierarchical_control::StagedValueView & /*state*/,
    const hierarchical_control::StagedValueView & /*reference*/,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    // A node that DECLARES reference ports must write every one of them: the kernel pre-fills them
    // with NaN and reports a completeness failure otherwise. Declaring a reference list (which these
    // fixtures now do, so that they match their contracts) is what makes this necessary.
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = 1.0;}
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = 1.0;}
    return Return::OK;
  }

  int state_calls = 0;
  int command_calls = 0;

private:
  std::vector<std::string> state_ports_;
  std::vector<std::string> reference_ports_;
  ZeroSource source_;
};

// A three-level topology.
struct root_n
{
  static constexpr auto value = st::NameOf("root");
};
struct mid_n
{
  static constexpr auto value = st::NameOf("mid");
};
struct leaf_n
{
  static constexpr auto value = st::NameOf("leaf");
};
using root_t = st::Root<root_n>;
using mid_t = st::Descendant<mid_n, root_t>;
using leaf_t = st::Descendant<leaf_n, mid_t>;

// Ports, named "<owner>/<local>". Each node declares its OWN state (produced) and its OWN reference
// (consumed) -- the mapping `typed_ports::TypedPorts` uses, so a node's runtime strings can match
// its contract exactly and the checked build entry accepts the binding.
struct root_state_n
{
  static constexpr auto value = st::NameOf("root/state");
};
struct root_ref_n
{
  static constexpr auto value = st::NameOf("root/ref");
};
struct mid_state_n
{
  static constexpr auto value = st::NameOf("mid/state");
};
struct mid_ref_n
{
  static constexpr auto value = st::NameOf("mid/ref");
};
struct leaf_state_n
{
  static constexpr auto value = st::NameOf("leaf/state");
};
struct leaf_ref_n
{
  static constexpr auto value = st::NameOf("leaf/ref");
};
using root_state = tc::Port<root_state_n, dm::Position>;
using root_ref = tc::Port<root_ref_n, dm::LinearVelocity>;
using mid_state = tc::Port<mid_state_n, dm::Position>;
using mid_ref = tc::Port<mid_ref_n, dm::LinearVelocity>;
using leaf_state = tc::Port<leaf_state_n, dm::Position>;
using leaf_ref = tc::Port<leaf_ref_n, dm::LinearVelocity>;

using root_contract = tc::Contract<tc::PortList<root_state>, tc::PortList<root_ref>>;
using mid_contract = tc::Contract<tc::PortList<mid_state>, tc::PortList<mid_ref>>;
using leaf_contract = tc::Contract<tc::PortList<leaf_state>, tc::PortList<leaf_ref>>;

StubController g_root{"root", {"root/state"}, {"root/ref"}};
StubController g_mid{"mid", {"mid/state"}, {"mid/ref"}};
StubController g_leaf{"leaf", {"leaf/state"}, {"leaf/ref"}};
// A node with no ports at all, for the single-node binding below.
StubController g_lone{"lone", {}, {}};

const auto g_leaf_binding = tc::make_leaf<leaf_t, leaf_contract>(&g_leaf);
const auto g_mid_binding = tc::compose<mid_t, mid_contract>(&g_mid, g_leaf_binding);
const auto g_root_binding = tc::compose<root_t, root_contract>(&g_root, g_mid_binding);

// ---- a controller whose runtime port strings MATCH its contract ------------------------------
// `Contract::produced` is the state this node publishes and `consumed` is the reference it
// receives; `typed_ports::TypedPorts` maps State -> produced and Reference -> consumed the same way.
// The ports are owned by `leaf`, which is the node this binding contains: the ownership check
// requires every port's "<owner>/" prefix to name a controller in the topology.
using coherent_contract = tc::Contract<tc::PortList<leaf_state>, tc::PortList<leaf_ref>>;

class CoherentController : public StubController
{
public:
  CoherentController() : StubController("leaf", {"leaf/state"}, {"leaf/ref"}) {}
};

CoherentController g_coherent;
const auto g_coherent_binding = tc::make_leaf<leaf_t, coherent_contract>(&g_coherent);
}  // namespace

/// The adapted Spec preserves names, parents and instances in root-first order.
TEST(TopologyBinding, adapted_spec_matches_the_binding)
{
  const auto spec = tb::to_library_spec(g_root_binding);

  ASSERT_EQ(3u, spec.names.size());
  EXPECT_EQ("root", spec.names[0]);
  EXPECT_EQ("mid", spec.names[1]);
  EXPECT_EQ("leaf", spec.names[2]);

  EXPECT_EQ("", spec.parents[0]);
  EXPECT_EQ("root", spec.parents[1]);
  EXPECT_EQ("mid", spec.parents[2]);

  EXPECT_EQ(static_cast<hierarchical_control::StagedControllerInterface *>(&g_root),
            spec.instances[0]);
  EXPECT_EQ(static_cast<hierarchical_control::StagedControllerInterface *>(&g_mid),
            spec.instances[1]);
  EXPECT_EQ(static_cast<hierarchical_control::StagedControllerInterface *>(&g_leaf),
            spec.instances[2]);
}

/// The kernel accepts the adapted Spec: this is the failure mode that only a real construction
/// exposes.
TEST(TopologyBinding, the_kernel_accepts_the_adapted_spec)
{
  auto group = tb::create_library_group(g_root_binding);
  ASSERT_NE(nullptr, group);
  EXPECT_TRUE(group->members_active()) << "library mode reports itself active immediately";
  EXPECT_EQ(3u, group->size());
}

/// The constructed group actually runs: every member's state and command stage is called once per
/// cycle, in the two-phase order.
TEST(TopologyBinding, the_constructed_group_runs_a_cycle)
{
  auto group = tb::create_library_group(g_root_binding);
  ASSERT_NE(nullptr, group);

  g_root.state_calls = 0;
  g_mid.state_calls = 0;
  g_leaf.state_calls = 0;
  g_root.command_calls = 0;
  g_mid.command_calls = 0;
  g_leaf.command_calls = 0;

  const auto result = group->run_ns(0, 1000000);  // cycle 0, 1 ms period
  EXPECT_EQ(hierarchical_control::StagedStatus::committed, result.status);

  EXPECT_EQ(1, g_root.state_calls);
  EXPECT_EQ(1, g_mid.state_calls);
  EXPECT_EQ(1, g_leaf.state_calls);
  EXPECT_EQ(1, g_root.command_calls);
  EXPECT_EQ(1, g_mid.command_calls);
  EXPECT_EQ(1, g_leaf.command_calls);
}

/// The adapter rejects malformed plans with a diagnostic naming the problem, and does not silently
/// hand a bad Spec to the kernel.
TEST(TopologyBinding, malformed_plans_are_rejected_with_a_reason)
{
  {  // empty
    tc::SpecRows rows;
    EXPECT_THROW(tb::to_library_spec(rows), std::invalid_argument);
  }
  {  // null instance
    tc::SpecRows rows;
    rows.names = {"a"};
    rows.instances = {nullptr};
    rows.parents = {""};
    try
    {
      tb::to_library_spec(rows);
      FAIL() << "a null instance must be rejected";
    }
    catch (const std::invalid_argument & error)
    {
      EXPECT_NE(std::string::npos, std::string(error.what()).find("null controller instance"));
    }
  }
  {  // two roots
    tc::SpecRows rows;
    rows.names = {"a", "b"};
    rows.instances = {static_cast<controller_interface::ControllerInterfaceBase *>(&g_root), static_cast<controller_interface::ControllerInterfaceBase *>(&g_mid)};
    rows.parents = {"", ""};
    EXPECT_THROW(tb::to_library_spec(rows), std::invalid_argument);
  }
  {  // duplicate names
    tc::SpecRows rows;
    rows.names = {"a", "a"};
    rows.instances = {static_cast<controller_interface::ControllerInterfaceBase *>(&g_root), static_cast<controller_interface::ControllerInterfaceBase *>(&g_mid)};
    rows.parents = {"", "a"};
    EXPECT_THROW(tb::to_library_spec(rows), std::invalid_argument);
  }
  {  // self-parent
    tc::SpecRows rows;
    rows.names = {"a", "b"};
    rows.instances = {static_cast<controller_interface::ControllerInterfaceBase *>(&g_root), static_cast<controller_interface::ControllerInterfaceBase *>(&g_mid)};
    rows.parents = {"", "b"};
    EXPECT_THROW(tb::to_library_spec(rows), std::invalid_argument);
  }
}

/// A single-node binding is legal and yields a one-member group.
///
/// NOTE: it needs its own contract. The three-node `root_contract` above consumes "mid/state",
/// and a binding containing only `root` has no `mid`, so reusing it here is an ownership violation
/// -- which the checker correctly rejects at compile time. That is the intended behaviour; this
/// leaf deliberately declares no ports.
TEST(TopologyBinding, a_single_node_binding_builds_a_one_member_group)
{
  using lone = tc::Contract<tc::PortList<>, tc::PortList<>>;
  const auto only = tc::make_leaf<root_t, lone>(&g_lone);
  auto group = tb::create_library_group(only);
  ASSERT_NE(nullptr, group);
  EXPECT_EQ(1u, group->size());
}

/// The checked build entry refuses a binding whose runtime port lists disagree with the contracts
/// it was bound with; the unchecked entry still builds it, so debug stubs remain possible.
///
/// This is review item C: the check used to be an optional free function, so a stub whose ports did
/// not match its contract still produced a running group whose buffers were sized from the wrong
/// declaration.
TEST(TopologyBinding, the_checked_build_entry_enforces_the_port_contract)
{
  // The matching fixture is accepted, and the check is not vacuous.
  std::string reason;
  EXPECT_TRUE(tb::verify_binding_ports(g_coherent_binding, &reason)) << reason;
  EXPECT_TRUE(reason.empty());
  EXPECT_NE(nullptr, tb::create_library_group(g_coherent_binding));

  // A stub whose ports are OWNED by a node in the topology (so the compile-time ownership check
  // passes) but whose names do not match the contract it is bound with, and which reports no
  // reference port where the contract declares one. That is exactly the class of mistake the
  // runtime verifier exists for: it cannot be a compile error, because a controller's port strings
  // are a runtime property.
  class SloppyController : public StubController
  {
  public:
    SloppyController() : StubController("leaf", {"leaf/state_wrong"}, {}) {}
  };
  static SloppyController sloppy;
  const auto sloppy_binding = tc::make_leaf<leaf_t, coherent_contract>(&sloppy);

  EXPECT_FALSE(tb::verify_binding_ports(sloppy_binding, &reason));
  // The message names the NODE (what the plan and the operator see), not the controller class.
  EXPECT_NE(std::string::npos, reason.find("leaf")) << reason;
  EXPECT_NE(std::string::npos, reason.find("staged_state_ports")) << reason;

  // The checked entry refuses it; the explicitly unchecked entry still builds it, which is what a
  // deliberately malformed debug stub is supposed to use.
  EXPECT_THROW(tb::create_library_group(sloppy_binding), std::invalid_argument);
  EXPECT_NE(nullptr, tb::create_library_group_unchecked(sloppy_binding));
}
