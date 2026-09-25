// MUST FAIL: a node declared as a Root is used as a CHILD binding. Two declared roots cannot be
// nested into one chain; the nesting would silently turn the second root into a child of the first
// (review item A, the exact counterexample in the review).
#include "_contract_common.hpp"
namespace c = negc;

// A second root, with its own name so the failure cannot be a duplicate-name rejection.
struct other_root_n { static constexpr auto value = c::st::NameOf("other_root"); };
using other_root = c::st::Root<other_root_n>;

using chassis_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;
using child_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;

const auto leaf = c::tc::make_leaf<other_root, child_contract>(&c::wheel_instance);
const auto root = c::tc::compose<c::chassis, chassis_contract>(&c::chassis_instance, leaf);

int main()
{
  auto spec = c::tc::build_spec_rows(root);
  return static_cast<int>(spec.names.size());
}
