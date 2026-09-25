// MUST FAIL: the root declares only ONE child reference (`left/target`) while it encloses TWO
// children. A hand-written-strings topology accepts this silently -- the missing reference is just
// never written -- so the child keeps whatever its own default was. The typed edge check rejects it
// at compile time: the declared list is not the concatenation of the children's declarations.
#include "_typed_common.hpp"

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<negt::left_target>,  // <-- `right/target` missing
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  return static_cast<int>(negt::tc::build_spec_rows(tree).names.size());
}
