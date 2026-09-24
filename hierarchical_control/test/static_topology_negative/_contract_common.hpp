// Shared fixtures for the topology/ownership negative corpus.
#pragma once
#include "hierarchical_control/topology_contract.hpp"
#include "test_controller_stub.hpp"  // local copy, so the corpus compiles standalone
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

// Real controller objects: the binding stores a TYPED ControllerInterfaceBase*, so an integer
// placeholder would now trip the base-class static_assert instead of the ownership check these
// corpus files are meant to exercise.
inline hierarchical_control_test::MinimalController chassis_instance{"chassis"};
inline hierarchical_control_test::MinimalController wheel_instance{"wheel"};
}  // namespace negc
