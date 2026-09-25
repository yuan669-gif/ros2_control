// MUST FAIL: the root declares its two child states in the WRONG ORDER (`right/state` first, while
// it encloses `left` first). Both ports exist and both are declared, so a check that only compared
// names as a SET, or only compared lengths, would accept this -- and the states would then be routed
// to the wrong children, silently. This is the regression the typed edge check exists for: it
// compares names in CHILD ORDER.
#include "_typed_common.hpp"

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::right_state, negt::left_state>>;  // <-- swapped

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  return static_cast<int>(negt::tc::build_spec_rows(tree).names.size());
}
