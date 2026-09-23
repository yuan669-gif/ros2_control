// MUST FAIL: the port's owner "ghost" is not a controller in this topology. This is exactly the
// defect class upstream accepts silently (a declared interface nothing in the group exports).
#include "_contract_common.hpp"
namespace c = negc;
struct ghost_travel_n { static constexpr auto value = c::st::NameOf("ghost/travel"); };
using ghost_travel = c::tc::Port<ghost_travel_n, c::dm::Position>;

using chassis_contract = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<ghost_travel>>;
using wheel_contract   = c::tc::Contract<c::tc::PortList<>, c::tc::PortList<>>;

constexpr auto leaf = c::tc::make_leaf<c::wheel, wheel_contract>(&c::wheel_instance);
constexpr auto root = c::tc::compose<c::chassis, chassis_contract>(&c::chassis_instance, leaf);

int main()
{
  c::tc::require_ports_are_owned<decltype(root)>();
  return 0;
}
