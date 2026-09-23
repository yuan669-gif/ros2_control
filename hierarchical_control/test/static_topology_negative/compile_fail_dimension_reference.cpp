// MUST FAIL: a position command is wired to a velocity reference. The string-name world accepts
// this silently; the dimensional check rejects it.
#include "hierarchical_control/dimensional_interfaces.hpp"
namespace d = hierarchical_control::dimensions;
using pos_cmd = d::Interface<d::Role::command, d::Position>;
using vel_ref = d::Interface<d::Role::reference, d::LinearVelocity>;
int main()
{
  d::require_reference_edge<pos_cmd, vel_ref>();  // dimension mismatch
  return 0;
}
