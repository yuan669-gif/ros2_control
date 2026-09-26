// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_COMPOSITE_LIBRARY__TYPED_FORK_DECLARATION_HPP_
#define TEST_COMPOSITE_LIBRARY__TYPED_FORK_DECLARATION_HPP_

// The compile-time DECLARATION of the tree hosted by `TypedForkCompositeController`.
//
// It lives in its own header so the description can be used without the plugin: the manifest is
// derived from the TYPE of the binding, so a translation unit that only wants to inspect or check
// the description does not have to construct a controller. Everything here is a type; nothing here
// touches ROS.

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/static_topology.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/typed_ports.hpp"

namespace test_composite_library
{
/// The declaration of the two-leaf fork hosted by the typed composite plugin.
namespace typed_fork
{
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

// THE DECLARATION: one statement of the tree, its ports, their dimensions and the hardware
// interfaces the leaves need.
// ---------------------------------------------------------------------------------------------

#define TYPED_FORK_NAME(struct_name, text)  \
  struct struct_name                        \
  {                                         \
    static constexpr auto value = st::NameOf(text); \
  }

TYPED_FORK_NAME(root_n, "typed_root");
TYPED_FORK_NAME(a_n, "typed_a");
TYPED_FORK_NAME(b_n, "typed_b");

using root_node = st::Root<root_n>;
using a_node = st::Descendant<a_n, root_node>;
using b_node = st::Descendant<b_n, root_node>;

TYPED_FORK_NAME(root_state_n, "typed_root/state");
TYPED_FORK_NAME(root_ref_n, "typed_root/ref");
TYPED_FORK_NAME(a_state_n, "typed_a/state");
TYPED_FORK_NAME(a_ref_n, "typed_a/ref");
TYPED_FORK_NAME(b_state_n, "typed_b/state");
TYPED_FORK_NAME(b_ref_n, "typed_b/ref");
/// The actuators are hardware interfaces. They are declared here only so the mixin GENERATES
/// `staged_actuator_ports()`; a `Contract` deliberately excludes them, so they take no part in the
/// topology ownership checks.
TYPED_FORK_NAME(a_actuator_n, "joint2/velocity");
TYPED_FORK_NAME(b_actuator_n, "joint3/velocity");
/// The hardware STATE interfaces the leaves read. They complete the description: without them the
/// controller would have to write these names a second time in `state_interface_configuration()`,
/// which is the duplication this whole layer exists to remove.
TYPED_FORK_NAME(a_position_n, "joint2/position");
TYPED_FORK_NAME(b_position_n, "joint3/position");

using root_state = tc::Port<root_state_n, dm::Position>;
using root_ref = tc::Port<root_ref_n, dm::LinearVelocity>;
using a_state = tc::Port<a_state_n, dm::Position>;
using a_ref = tc::Port<a_ref_n, dm::LinearVelocity>;
using b_state = tc::Port<b_state_n, dm::Position>;
using b_ref = tc::Port<b_ref_n, dm::LinearVelocity>;
using a_actuator = tc::Port<a_actuator_n, dm::LinearVelocity>;
using b_actuator = tc::Port<b_actuator_n, dm::LinearVelocity>;
using a_position = tc::Port<a_position_n, dm::Position>;
using b_position = tc::Port<b_position_n, dm::Position>;

/// The root declares the concatenation, IN CHILD ORDER, of what its two children declare: the
/// references it writes into them and the states it reads back. `compose` checks exactly this.
using root_ports = tp::TypedPorts<
  tc::PortList<root_state>, tc::PortList<root_ref>, tc::PortList<>,
  tc::PortList<a_ref, b_ref>, tc::PortList<a_state, b_state>>;
using a_ports = tp::TypedPorts<
  tc::PortList<a_state>, tc::PortList<a_ref>, tc::PortList<a_actuator>, tc::PortList<>,
  tc::PortList<>, tc::PortList<a_position>>;
using b_ports = tp::TypedPorts<
  tc::PortList<b_state>, tc::PortList<b_ref>, tc::PortList<b_actuator>, tc::PortList<>,
  tc::PortList<>, tc::PortList<b_position>>;


}  // namespace typed_fork
}  // namespace test_composite_library

#undef TYPED_FORK_NAME

#endif  // TEST_COMPOSITE_LIBRARY__TYPED_FORK_DECLARATION_HPP_
