// MUST FAIL: the child binding IS nested under `chassis`, but its static_topology type declares a
// DIFFERENT parent (`other`). The nesting and the type-level parent disagree, so the emitted plan
// would not describe the declared topology.
//
// This is review item A. Before `compose` checked it, the nesting silently overrode the type: the
// plan was emitted with the enclosing node's name, so a binding that declared one topology produced
// another.
#include "_contract_common.hpp"
namespace c = negc;

struct other_n { static constexpr auto value = c::st::NameOf("other"); };
using other = c::st::Root<other_n>;
// `stray` claims `other` as its parent, but it is about to be nested under `chassis`.
using stray = c::st::Descendant<c::wheel_n, other>;

using chassis_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;
using stray_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;

const auto leaf = c::tc::make_leaf<stray, stray_contract>(&c::wheel_instance);
const auto root = c::tc::compose<c::chassis, chassis_contract>(&c::chassis_instance, leaf);

int main()
{
  auto spec = c::tc::build_spec_rows(root);
  return static_cast<int>(spec.names.size());
}
