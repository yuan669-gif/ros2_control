// MUST FAIL: name and ORDER are right, but the root declares the reference it writes into `left`
// with the WRONG PHYSICAL DIMENSION (a position where the child consumes a velocity). String-name
// topologies cannot express this at all: "left/target" matches, and the numeric mismatch only shows
// up as a wrong physical result at runtime. The typed edge check compares dimensions too.
#include "_typed_common.hpp"

using left_target_as_position = negt::tc::Port<negt::left_target_n, negt::dm::Position>;

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<left_target_as_position, negt::right_target>,  // <-- dimension
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  return static_cast<int>(negt::tc::build_spec_rows(tree).names.size());
}
