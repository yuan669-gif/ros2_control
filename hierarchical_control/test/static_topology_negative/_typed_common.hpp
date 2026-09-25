// Shared fixtures for the TYPED branching-tree negative corpus.
//
// The other corpus (`_contract_common.hpp`) exercises topology/ownership with hand-written
// `Contract`s. This one exercises the checks that only exist once a controller declares its ports
// ONCE as types (`TypedPorts`), which is the typed builder of review item B:
//
//   * the reference edge  : a parent's ForChildren list must equal the concatenation, in child
//                           order, of the reference ports its children declare they receive;
//   * the state edge      : a parent's ChildState list must equal the concatenation, in child
//                           order, of the state ports its children declare they publish.
//
// Both checks are triggered automatically by `compose` when every participant exposes
// `typed_ports`, so a corpus file needs no extra call: declaring the tree IS the test.
#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "hierarchical_control/topology_binding.hpp"
#include "hierarchical_control/typed_ports.hpp"
#include "test_controller_stub.hpp"  // local copy, so the corpus compiles standalone

namespace negt
{
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

// ---------------------------------------------------------------------------------------------
// Topology: root -> {left, right}
// ---------------------------------------------------------------------------------------------

struct root_n { static constexpr auto value = st::NameOf("root"); };
struct left_n { static constexpr auto value = st::NameOf("left"); };
struct right_n { static constexpr auto value = st::NameOf("right"); };

using root_node = st::Root<root_n>;
using left_node = st::Descendant<left_n, root_node>;
using right_node = st::Descendant<right_n, root_node>;

// ---------------------------------------------------------------------------------------------
// The children's declarations (fixed; a negative case varies only the ROOT's declaration)
// ---------------------------------------------------------------------------------------------

struct left_state_n { static constexpr auto value = st::NameOf("left/state"); };
struct left_target_n { static constexpr auto value = st::NameOf("left/target"); };
struct left_travel_n { static constexpr auto value = st::NameOf("left/travel"); };
struct right_state_n { static constexpr auto value = st::NameOf("right/state"); };
struct right_target_n { static constexpr auto value = st::NameOf("right/target"); };

using left_state = tc::Port<left_state_n, dm::Position>;
using left_target = tc::Port<left_target_n, dm::LinearVelocity>;
using left_travel = tc::Port<left_travel_n, dm::Position>;
using right_state = tc::Port<right_state_n, dm::Position>;
using right_target = tc::Port<right_target_n, dm::LinearVelocity>;

using left_ports =
  tp::TypedPorts<tc::PortList<left_state>, tc::PortList<left_target>, tc::PortList<>>;
using right_ports =
  tp::TypedPorts<tc::PortList<right_state>, tc::PortList<right_target>, tc::PortList<>>;

// ---------------------------------------------------------------------------------------------
// A controller that declares its ports once as types
// ---------------------------------------------------------------------------------------------

/// `TypedPortsMixin` generates the port STRINGS from `Ports` and exposes `typed_ports`, which is
/// what makes `compose` run the per-child edge checks. The two stage entry points are required by
/// `StagedControllerInterface`; the corpus never runs a cycle, it only builds the binding.
template <typename Ports>
class TypedStub : public hierarchical_control_test::MinimalController,
                  public tp::TypedPortsMixin<TypedStub<Ports>, Ports>
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

/// Build `root -> {left, right}` with a caller-supplied ROOT declaration, so each negative case
/// differs from the control ONLY in that declaration.
template <typename RootPorts>
auto make_tree()
{
  static TypedStub<RootPorts> root{"root"};
  static TypedStub<left_ports> left{"left"};
  static TypedStub<right_ports> right{"right"};

  const auto left_leaf = tc::make_leaf<left_node, tp::contract_of_t<left_ports>>(&left);
  const auto right_leaf = tc::make_leaf<right_node, tp::contract_of_t<right_ports>>(&right);
  return tc::compose<root_node, tp::contract_of_t<RootPorts>>(&root, left_leaf, right_leaf);
}
}  // namespace negt
