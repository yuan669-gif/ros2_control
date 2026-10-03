// MUST FAIL: the tree is structurally fine, but TWO nodes declare the SAME hardware interface for
// the same role. Here the root declares `joint2/velocity` twice in its Actuators list.
//
// This is the in-tree form of an INTERFACE CLAIM CONFLICT: at runtime the `ResourceManager` would
// only reject it when the second controller claims the interface (throw "is already claimed"), i.e.
// after activation has begun. The manifest knows the whole tree as a TYPE, so the conflict is
// decided at compile time instead.
//
// Why the duplicate is declared on the ROOT's actuators: actuator ports are hardware-facing, so the
// ownership check deliberately ignores them (their owner is a joint, not a node). That is exactly
// the case where a name collision can slip past ownership and only the manifest's per-role
// uniqueness rule catches it.
#include "_typed_common.hpp"

struct root_actuator_n
{
  static constexpr auto value = negt::st::NameOf("joint2/velocity");
};
// Same NAME, same role, different type: a second declaration of one physical interface.
struct root_actuator_dup_n
{
  static constexpr auto value = negt::st::NameOf("joint2/velocity");
};

using root_actuator = negt::tc::Port<root_actuator_n, negt::dm::LinearVelocity>;
using root_actuator_dup = negt::tc::Port<root_actuator_dup_n, negt::dm::LinearVelocity>;

using root_ports = negt::tp::TypedPorts<
  negt::tc::PortList<>, negt::tc::PortList<>,
  negt::tc::PortList<root_actuator, root_actuator_dup>,  // <-- same port, twice
  negt::tc::PortList<negt::left_target, negt::right_target>,
  negt::tc::PortList<negt::left_state, negt::right_state>>;

int main()
{
  const auto tree = negt::make_tree<root_ports>();
  // `to_library_spec` is the checked entry point: it enforces the manifest invariants.
  return static_cast<int>(hierarchical_control::topology_binding::to_library_spec(tree).names.size());
}
