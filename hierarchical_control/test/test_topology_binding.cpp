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
class StubController : public hierarchical_control_test::MinimalController,
                       public hierarchical_control::StagedControllerInterface
{
public:
  explicit StubController(std::string name) : MinimalController(std::move(name)) {}

public:
  std::vector<std::string> staged_state_ports() const override { return {"value"}; }

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
    const hierarchical_control::StagedReferenceWriter & /*children*/,
    hierarchical_control::StagedValueWriter /*actuators*/) noexcept override
  {
    ++command_calls;
    return Return::OK;
  }

  int state_calls = 0;
  int command_calls = 0;

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

// Ports, named "<owner>/<local>".
struct mid_state_n
{
  static constexpr auto value = st::NameOf("mid/state");
};
struct leaf_state_n
{
  static constexpr auto value = st::NameOf("leaf/state");
};
using mid_state = tc::Port<mid_state_n, dm::Position>;
using leaf_state = tc::Port<leaf_state_n, dm::Position>;

using root_contract = tc::Contract<tc::PortList<>, tc::PortList<mid_state>>;
using mid_contract = tc::Contract<tc::PortList<mid_state>, tc::PortList<leaf_state>>;
using leaf_contract = tc::Contract<tc::PortList<leaf_state>, tc::PortList<>>;

StubController g_root{"root"};
StubController g_mid{"mid"};
StubController g_leaf{"leaf"};

const auto g_leaf_binding = tc::make_leaf<leaf_t, leaf_contract>(&g_leaf);
const auto g_mid_binding = tc::compose<mid_t, mid_contract>(&g_mid, g_leaf_binding);
const auto g_root_binding = tc::compose<root_t, root_contract>(&g_root, g_mid_binding);

// ---- a controller whose runtime port strings MATCH its contract ------------------------------
// `Contract::produced` is the state this node publishes and `consumed` is the reference it
// receives; `typed_ports::TypedPorts` maps State -> produced and Reference -> consumed the same way.
struct c_state_n
{
  static constexpr auto value = st::NameOf("coherent/state");
};
struct c_ref_n
{
  static constexpr auto value = st::NameOf("coherent/ref");
};
using c_state = tc::Port<c_state_n, dm::Position>;
using c_ref = tc::Port<c_ref_n, dm::LinearVelocity>;
using coherent_contract = tc::Contract<tc::PortList<c_state>, tc::PortList<c_ref>>;

class CoherentController : public StubController
{
public:
  CoherentController() : StubController("coherent") {}
  std::vector<std::string> staged_state_ports() const override {return {"coherent/state"};}
  std::vector<std::string> staged_reference_ports() const override {return {"coherent/ref"};}
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
  const auto only = tc::make_leaf<root_t, lone>(&g_root);
  auto group = tb::create_library_group(only);
  ASSERT_NE(nullptr, group);
  EXPECT_EQ(1u, group->size());
}

/// The binding-level port verifier ties the CHECKED topology to the RUNTIME port lists the kernel
/// sizes its buffers from. Building a controller instance is not a constant expression, so this
/// cannot be a compile-time check; it is a start-up call.
///
/// It is also where the two conventions in this repository meet, so the test states them:
///   * `Contract::produced` = the state ports this node PUBLISHES (its parent's state stage reads
///     them);
///   * `Contract::consumed` = the reference ports this node RECEIVES (its parent's command stage
///     writes them).
/// `typed_ports::TypedPorts` uses exactly that mapping (State -> produced, Reference -> consumed).
TEST(TopologyBinding, binding_level_port_verification)
{
  std::string reason;
  EXPECT_TRUE(tb::verify_binding_ports(g_coherent_binding, &reason)) << reason;
  EXPECT_TRUE(reason.empty());

  // The minimal `StubController` used by the rest of this file reports ONE state port and no
  // reference port whatever contract it is bound with. The verifier reports it, and the message
  // names the node and the list that disagrees -- this is the check that keeps a hand-written port
  // list from silently disagreeing with the topology that was checked for it.
  EXPECT_FALSE(tb::verify_binding_ports(g_root_binding, &reason));
  EXPECT_NE(std::string::npos, reason.find("root")) << reason;
  // `root_contract` declares no produced ports, while the minimal stub always reports one state
  // port: the verifier names the node and the list that disagrees.
  EXPECT_NE(std::string::npos, reason.find("staged_state_ports")) << reason;
}
