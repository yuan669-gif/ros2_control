// MUST FAIL: a port name without an "<owner>/" prefix cannot be attributed to any controller.
#include "_contract_common.hpp"
namespace c = negc;
struct bare_n { static constexpr auto value = c::st::NameOf("travel"); };
using bare = c::tc::Port<bare_n, c::dm::Position>;

using chassis_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<bare>>;
using wheel_contract   = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;

const auto leaf = c::tc::make_leaf<c::wheel, wheel_contract>(&c::wheel_instance);
const auto root = c::tc::compose<c::chassis, chassis_contract>(&c::chassis_instance, leaf);

int main()
{
  c::tc::require_ports_are_owned<decltype(root)>();
  return 0;
}
