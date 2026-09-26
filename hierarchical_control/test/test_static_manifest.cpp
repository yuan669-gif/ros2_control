// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// The compile-time description layer: a manifest derived from the TYPE of a typed binding.
//
// What this test establishes:
//   * the manifest is a genuine compile-time object -- node/port/interface counts, names, parents and
//     roles are checked with `static_assert`, not at run time;
//   * its invariants (`manifest_problem`) are constexpr, so a malformed declaration fails to compile
//     rather than failing during `configure`;
//   * the hardware requirements are separated into command and state lists, which is what lets a
//     controller GENERATE its `command_interface_configuration()` /
//     `state_interface_configuration()` from the same single declaration;
//   * a hand-written controller that disagrees with the manifest is caught by
//     `declaration_matches_manifest`, by NAME, before any resource is loaned.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "hierarchical_control/static_manifest.hpp"
#include "test_controller_stub.hpp"

namespace sm = hierarchical_control::static_manifest;
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

namespace
{
using Return = controller_interface::return_type;

#define MANIFEST_NODE_NAME(struct_name, text)  \
  struct struct_name                           \
  {                                            \
    static constexpr auto value = st::NameOf(text); \
  }

MANIFEST_NODE_NAME(root_n, "m_root");
MANIFEST_NODE_NAME(a_n, "m_a");
MANIFEST_NODE_NAME(b_n, "m_b");

using root_node = st::Root<root_n>;
using a_node = st::Descendant<a_n, root_node>;
using b_node = st::Descendant<b_n, root_node>;

MANIFEST_NODE_NAME(root_state_n, "m_root/state");
MANIFEST_NODE_NAME(root_ref_n, "m_root/ref");
MANIFEST_NODE_NAME(a_state_n, "m_a/state");
MANIFEST_NODE_NAME(a_ref_n, "m_a/ref");
MANIFEST_NODE_NAME(a_torque_n, "joint2/effort");
MANIFEST_NODE_NAME(a_position_n, "joint2/position");
MANIFEST_NODE_NAME(b_state_n, "m_b/state");
MANIFEST_NODE_NAME(b_ref_n, "m_b/ref");
MANIFEST_NODE_NAME(b_torque_n, "joint3/effort");
MANIFEST_NODE_NAME(b_position_n, "joint3/position");

using root_state = tc::Port<root_state_n, dm::Position>;
using root_ref = tc::Port<root_ref_n, dm::LinearVelocity>;
using a_state = tc::Port<a_state_n, dm::Position>;
using a_ref = tc::Port<a_ref_n, dm::LinearVelocity>;
using a_torque = tc::Port<a_torque_n, dm::Torque>;
using a_position = tc::Port<a_position_n, dm::Position>;
using b_state = tc::Port<b_state_n, dm::Position>;
using b_ref = tc::Port<b_ref_n, dm::LinearVelocity>;
using b_torque = tc::Port<b_torque_n, dm::Torque>;
using b_position = tc::Port<b_position_n, dm::Position>;

// The declaration states the topology AND the hardware interfaces (Actuators = command,
// HardwareState = state inputs).
using root_ports = tp::TypedPorts<
  tc::PortList<root_state>, tc::PortList<root_ref>, tc::PortList<>,
  tc::PortList<a_ref, b_ref>, tc::PortList<a_state, b_state>>;
using a_ports = tp::TypedPorts<
  tc::PortList<a_state>, tc::PortList<a_ref>, tc::PortList<a_torque>, tc::PortList<>,
  tc::PortList<>, tc::PortList<a_position>>;
using b_ports = tp::TypedPorts<
  tc::PortList<b_state>, tc::PortList<b_ref>, tc::PortList<b_torque>, tc::PortList<>,
  tc::PortList<>, tc::PortList<b_position>>;

/// A bare controller that only needs to exist for the binding; the manifest is derived from types.
class Stub : public hierarchical_control_test::MinimalController,
              public tp::TypedPortsMixin<Stub, root_ports>
{
public:
  Stub() : MinimalController("stub") {}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
};

template <typename Ports>
class LeafStub : public hierarchical_control_test::MinimalController,
                 public tp::TypedPortsMixin<LeafStub<Ports>, Ports>
{
public:
  explicit LeafStub(std::string name) : MinimalController(std::move(name)) {}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
};

// The binding, and the manifest derived from its type.
using root_contract = tp::contract_of_t<root_ports>;
using a_contract = tp::contract_of_t<a_ports>;
using b_contract = tp::contract_of_t<b_ports>;

inline Stub g_root;
inline LeafStub<a_ports> g_a{"m_a"};
inline LeafStub<b_ports> g_b{"m_b"};

using binding_type = decltype(tc::compose<root_node, root_contract>(
  &g_root, tc::make_leaf<a_node, a_contract>(&g_a), tc::make_leaf<b_node, b_contract>(&g_b)));

constexpr auto kManifest = sm::manifest_of_v<binding_type>;

// ---- compile-time facts ---------------------------------------------------------------------
static_assert(sm::manifest_problem(kManifest).empty(), "the manifest must be well formed");
static_assert(sm::manifest_is_well_formed<binding_type>());
static_assert(kManifest.node_count == 3, "root plus two leaves");
static_assert(kManifest.command_count == 2, "one actuator per leaf");
static_assert(kManifest.state_count == 2, "one hardware state input per leaf");
// root: state + reference + 2 for_children + 2 child_state; each leaf: state + reference +
// actuator + hardware_state
static_assert(kManifest.port_count == 6 + 4 + 4, "port count");
static_assert(kManifest.has_command("joint2/effort"));
static_assert(kManifest.has_command("joint3/effort"));
static_assert(!kManifest.has_command("joint2/position"));
static_assert(kManifest.has_state_interface("joint2/position"));
static_assert(kManifest.has_state_interface("joint3/position"));
static_assert(kManifest.parent_of("m_a") == std::string_view("m_root"));
static_assert(kManifest.parent_of("m_root").empty());
static_assert(kManifest.parent_of("m_b") == std::string_view("m_root"));
}  // namespace

/// The manifest is enumerable at run time too, in a stable order (nodes pre-order).
TEST(StaticManifest, describes_the_tree_and_its_hardware_requirements)
{
  ASSERT_EQ(3u, kManifest.nodes.size());
  EXPECT_EQ("m_root", kManifest.nodes[0].name);
  EXPECT_EQ("", kManifest.nodes[0].parent);
  EXPECT_EQ("m_a", kManifest.nodes[1].name);
  EXPECT_EQ("m_root", kManifest.nodes[1].parent);
  EXPECT_EQ("m_b", kManifest.nodes[2].name);
  EXPECT_EQ("m_root", kManifest.nodes[2].parent);

  // Every port declares its owner and the list it came from.
  EXPECT_EQ(6u, kManifest.port_count_of("m_root"));
  EXPECT_EQ(4u, kManifest.port_count_of("m_a"));
  EXPECT_EQ(4u, kManifest.port_count_of("m_b"));
  bool saw_hardware_state = false;
  bool saw_actuator = false;
  for (const auto & port : kManifest.ports)
  {
    EXPECT_FALSE(port.name.empty());
    EXPECT_FALSE(port.owner.empty());
    if (port.role == sm::PortRole::hardware_state)
    {
      saw_hardware_state = true;
      // A hardware interface's "owner" is a joint, which is exactly why it is not a topology edge.
      EXPECT_EQ(0u, port.name.rfind("joint", 0)) << "got: " << port.name;
      EXPECT_FALSE(kManifest.parent_of(port.owner).empty() || port.owner == "m_root")
        << "hardware state must not be owned by a node";
    }
    if (port.role == sm::PortRole::actuator) {saw_actuator = true;}
  }
  EXPECT_TRUE(saw_hardware_state);
  EXPECT_TRUE(saw_actuator);
}

/// The hardware requirements can be GENERATED, which is what removes the hand-written string lists.
TEST(StaticManifest, hardware_requirements_are_separated_for_generation)
{
  std::vector<std::string> commands(kManifest.command_interfaces.begin(),
                                    kManifest.command_interfaces.end());
  std::vector<std::string> states(kManifest.state_interfaces.begin(),
                                  kManifest.state_interfaces.end());
  EXPECT_EQ((std::vector<std::string>{"joint2/effort", "joint3/effort"}), commands);
  EXPECT_EQ((std::vector<std::string>{"joint2/position", "joint3/position"}), states);
}

/// A controller that hand-writes lists disagreeing with the manifest is caught BY NAME, and the
/// check runs in `configure`, i.e. before any interface is loaned.
TEST(StaticManifest, declaration_is_verified_against_the_manifest_by_name)
{
  const std::vector<std::string> good_commands = {"joint2/effort", "joint3/effort"};
  const std::vector<std::string> good_states = {"joint2/position", "joint3/position"};
  std::string reason;
  EXPECT_TRUE(
    sm::declaration_matches_manifest(kManifest, good_commands, good_states, &reason)) << reason;

  {  // a required command interface is missing
    const std::vector<std::string> missing = {"joint2/effort"};
    EXPECT_FALSE(sm::declaration_matches_manifest(kManifest, missing, good_states, &reason));
    EXPECT_NE(std::string::npos, reason.find("joint3/effort")) << reason;
    EXPECT_NE(std::string::npos, reason.find("manifest requires command interface")) << reason;
  }
  {  // an extra command interface the manifest does not know about
    const std::vector<std::string> extra = {"joint2/effort", "joint3/effort", "joint9/effort"};
    EXPECT_FALSE(sm::declaration_matches_manifest(kManifest, extra, good_states, &reason));
    EXPECT_NE(std::string::npos, reason.find("joint9/effort")) << reason;
  }
  {  // a state input switched off
    const std::vector<std::string> missing = {"joint2/position"};
    EXPECT_FALSE(sm::declaration_matches_manifest(kManifest, good_commands, missing, &reason));
    EXPECT_NE(std::string::npos, reason.find("joint3/position")) << reason;
    EXPECT_NE(std::string::npos, reason.find("manifest requires state interface")) << reason;
  }
  {  // the wrong ORDER is not a mismatch: the manager matches by name, and so must this check
    const std::vector<std::string> swapped = {"joint3/effort", "joint2/effort"};
    const std::vector<std::string> swapped_states = {"joint3/position", "joint2/position"};
    EXPECT_TRUE(
      sm::declaration_matches_manifest(kManifest, swapped, swapped_states, &reason)) << reason;
  }
}

/// A malformed manifest is a compile-time failure, so the checker has to be able to say WHY.
TEST(StaticManifest, the_constexpr_checker_rejects_malformed_descriptions)
{
  // Two roots: a real manifest cannot express that, but the checker must still diagnose it.
  constexpr std::array<sm::ManifestNode, 2> two_roots_nodes = {{{"a", ""}, {"b", ""}}};
  constexpr sm::StaticManifest<2, 0, 0, 0> two_roots{two_roots_nodes, {}, {}, {}};
  EXPECT_EQ("a manifest must have exactly one root", sm::manifest_problem(two_roots));

  constexpr std::array<sm::ManifestNode, 2> duplicate_nodes = {{{"a", ""}, {"a", "a"}}};
  constexpr sm::StaticManifest<2, 0, 0, 0> duplicate{duplicate_nodes, {}, {}, {}};
  EXPECT_EQ("node names must be unique", sm::manifest_problem(duplicate));

  constexpr std::array<sm::ManifestNode, 2> ghost_nodes = {{{"a", ""}, {"b", "ghost"}}};
  constexpr sm::StaticManifest<2, 0, 0, 0> ghost_parent{ghost_nodes, {}, {}, {}};
  EXPECT_EQ("every parent must name a node of the manifest", sm::manifest_problem(ghost_parent));

  constexpr std::array<sm::ManifestNode, 1> one_node = {{{"a", ""}}};
  constexpr std::array<sm::ManifestPort, 1> orphan_ports = {
    {{"a/state", "ghost", sm::PortRole::state}}};
  constexpr sm::StaticManifest<1, 1, 0, 0> orphan_port{one_node, orphan_ports, {}, {}};
  EXPECT_EQ(
    "every non-hardware port must be owned by a node of the manifest",
    sm::manifest_problem(orphan_port));

  constexpr std::array<sm::ManifestPort, 1> hardware_ports = {
    {{"joint2/position", "joint2", sm::PortRole::hardware_state}}};
  constexpr sm::StaticManifest<1, 1, 0, 0> hardware_port{one_node, hardware_ports, {}, {}};
  EXPECT_TRUE(sm::manifest_problem(hardware_port).empty())
    << "hardware interfaces are owned by joints, not by nodes";
}

/// P2-1: parent chains that never reach the root are rejected at compile time.
/**
 * Two distinct shapes have to be separated, because only one of them is caught by the root count:
 *
 *   * a PURE cycle (a -> b -> c -> a) has no root, so "exactly one root" already rejects it;
 *   * a chain running INTO a cycle (root r, a -> b, b -> c, c -> b) has exactly one root, unique
 *     names, existing parents and no self-parent -- and is still not a tree: `a` can never reach `r`.
 *
 * The second shape is what the bounded parent walk exists for, and it is the one that would otherwise
 * produce a manifest describing a topology no execution order can schedule.
 */
TEST(StaticManifest, the_constexpr_checker_rejects_parent_chains_that_enter_a_cycle)
{
  // Pure three-node cycle: rejected, by the root count.
  constexpr std::array<sm::ManifestNode, 3> cycle_nodes = {
    {{"a", "b"}, {"b", "c"}, {"c", "a"}}};
  constexpr sm::StaticManifest<3, 0, 0, 0> pure_cycle{cycle_nodes, {}, {}, {}};
  static_assert(
    !sm::manifest_problem(pure_cycle).empty(), "a manifest must be checkable in a static_assert");
  EXPECT_EQ("a manifest must have exactly one root", sm::manifest_problem(pure_cycle));

  // Chain into a cycle: exactly one root, every parent exists, no self-parent -- still not a tree.
  constexpr std::array<sm::ManifestNode, 4> rho_nodes = {
    {{"r", ""}, {"a", "b"}, {"b", "c"}, {"c", "b"}}};
  constexpr sm::StaticManifest<4, 0, 0, 0> rho{rho_nodes, {}, {}, {}};
  static_assert(
    !sm::manifest_problem(rho).empty(), "a chain that enters a cycle must be rejected");
  EXPECT_EQ("the parent chains must not contain a cycle", sm::manifest_problem(rho));

  // The same shape without the back edge is a tree, so the check rejects the cycle and not depth.
  constexpr std::array<sm::ManifestNode, 4> chain_nodes = {
    {{"r", ""}, {"a", "r"}, {"b", "a"}, {"c", "b"}}};
  constexpr sm::StaticManifest<4, 0, 0, 0> chain{chain_nodes, {}, {}, {}};
  static_assert(sm::manifest_problem(chain).empty(), "a chain to the root is well formed");
  EXPECT_TRUE(sm::manifest_problem(chain).empty());
}
