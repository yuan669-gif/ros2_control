// MUST FAIL: same LENGTH as the correct declaration, but one entry is a DIFFERENT port that the same
// child owns (`left/travel` instead of `left/state`). This is the case a length-only check accepts --
// which is what `verify_ports_match_contract` used to do while its comment claimed a name check. The
// ownership check passes too (owner "left" IS in the tree, and the name is well formed), so only the
// name-by-name, in-child-order comparison can reject it.
#include "_typed_common.hpp"

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::left_travel, negt::right_state>>;  // <-- left/travel, not left/state

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  return static_cast<int>(negt::tc::build_spec_rows(tree).names.size());
}
