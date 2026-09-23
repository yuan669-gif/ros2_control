// Shared fixtures for the topology/ownership negative corpus.
#pragma once
#include "hierarchical_control/topology_contract.hpp"
namespace negc
{
namespace tc = hierarchical_control::topology_contract;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

struct chassis_n { static constexpr auto value = st::NameOf("chassis"); };
struct wheel_n   { static constexpr auto value = st::NameOf("wheel"); };
using chassis = st::Root<chassis_n>;
using wheel   = st::Descendant<wheel_n, chassis>;

struct wheel_travel_n { static constexpr auto value = st::NameOf("wheel/travel"); };
using wheel_travel = tc::Port<wheel_travel_n, dm::Position>;

inline int chassis_instance = 1;
inline int wheel_instance = 2;
}  // namespace negc
