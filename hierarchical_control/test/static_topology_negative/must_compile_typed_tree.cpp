// CONTROL for the typed branching corpus: this file must COMPILE.
//
// The root declares exactly the concatenation, in child order, of what its two children declare:
//   ForChildren = left/target, right/target      (the references it writes into them, in order)
//   ChildState  = left/state,  right/state       (the states it reads back, in order)
//
// Without this file a checker that rejected every typed branching tree would look like a pass.
#include "_typed_common.hpp"

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  const auto rows = negt::tc::build_spec_rows(tree);
  return static_cast<int>(rows.names.size());
}
