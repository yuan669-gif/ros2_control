// MUST FAIL: the topology is fine and every node is a valid typed controller, but none of them
// implements `TwoPhaseControllerInterface`.
//
// This is the compile-time counterpart of the manager's `cross_mode_dependency` admission: a tree
// that mixes the two execution paths cannot be ordered by one pair of passes (the state pass and the
// command pass assume the same membership), so on the runtime path the whole configuration is
// refused. Here the refusal is a `static_assert`, and the configuration is not expressible at all.
#include "_typed_common.hpp"

#include <type_traits>

#include "hierarchical_control/static_two_phase_admission.hpp"

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  hierarchical_control::static_two_phase::require_two_phase_tree<std::decay_t<decltype(tree)>>();
  return 0;
}
