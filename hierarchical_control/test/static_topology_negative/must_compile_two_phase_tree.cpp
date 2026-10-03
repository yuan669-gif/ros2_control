// CONTROL for the declared-two-phase-tree check: this file must COMPILE.
//
// The same tree as `compile_fail_two_phase_non_member.cpp`, but every node implements
// `TwoPhaseControllerInterface`, so `require_two_phase_tree` must accept it. Without this file a
// check that rejected every tree would look like a pass.
#include "_typed_common.hpp"

#include <type_traits>

#include "hierarchical_control/static_two_phase_admission.hpp"

namespace
{
/// The corpus stub plus the two-phase entry points, so the same topology can be used both ways.
template <typename Ports>
class TwoPhaseStub : public negt::TypedStub<Ports>,
                     public hierarchical_control::TwoPhaseControllerInterface
{
public:
  explicit TwoPhaseStub(std::string name) : negt::TypedStub<Ports>(std::move(name)) {}

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
}  // namespace

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  static TwoPhaseStub<root_ports> root{"root"};
  static TwoPhaseStub<negt::left_ports> left{"left"};
  static TwoPhaseStub<negt::right_ports> right{"right"};

  const auto tree = negt::tc::compose<negt::root_node, negt::tp::contract_of_t<root_ports>>(
    &root, negt::tc::make_leaf<negt::left_node, negt::tp::contract_of_t<negt::left_ports>>(&left),
    negt::tc::make_leaf<negt::right_node, negt::tp::contract_of_t<negt::right_ports>>(&right));

  using binding_type = std::decay_t<decltype(tree)>;
  hierarchical_control::static_two_phase::require_two_phase_tree<binding_type>();

  constexpr auto edges =
    hierarchical_control::static_two_phase::tree_description<binding_type>::edges;
  return static_cast<int>(edges.size());
}
