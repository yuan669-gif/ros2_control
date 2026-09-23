// MUST FAIL: an angle state is consumed as a linear position state (the wheel-radius confusion).
#include "hierarchical_control/dimensional_interfaces.hpp"
namespace d = hierarchical_control::dimensions;
using angle_state = d::Interface<d::Role::state, d::Angle>;
using pos_state = d::Interface<d::Role::state, d::Position>;
int main()
{
  d::require_state_edge<angle_state, pos_state>();  // dimension mismatch
  return 0;
}
