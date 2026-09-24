// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Dimensions on the execution group's port declarations.
//
// The gap being closed: StagedControllerInterface declares its ports as runtime STRINGS, while the
// dimensional and topology checks work on TYPES. Before this header a controller had to state its
// ports twice -- once as types, once as strings -- with nothing keeping the two statements equal.
//
// Now the ports are declared once as types and the strings are GENERATED, so they cannot disagree.
// This test verifies that the generated strings are exactly what the kernel expects, and that a
// controller built this way runs in a real StagedExecutionGroup.

#include <gtest/gtest.h>

#include <cstdio>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hierarchical_control/topology_binding.hpp"
#include "hierarchical_control/typed_ports.hpp"
#include "test_controller_stub.hpp"

namespace tp = hierarchical_control::typed_ports;
namespace tc = hierarchical_control::topology_contract;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;
using Return = controller_interface::return_type;

namespace
{
// ---- names ---------------------------------------------------------------------------------
struct wheel_target_n
{
  static constexpr auto value = st::NameOf("wheel/target");
};
struct wheel_torque_n
{
  static constexpr auto value = st::NameOf("wheel/torque");
};
struct wheel_travel_n
{
  static constexpr auto value = st::NameOf("wheel/travel");
};
struct tire_target_n
{
  static constexpr auto value = st::NameOf("tire/target");
};

// ---- typed ports ---------------------------------------------------------------------------
using wheel_target = tc::Port<wheel_target_n, dm::LinearVelocity>;
using wheel_torque = tc::Port<wheel_torque_n, dm::Torque>;
using wheel_travel = tc::Port<wheel_travel_n, dm::Position>;
using tire_target = tc::Port<tire_target_n, dm::LinearVelocity>;

// The wheel exports both a reference for its child (tire/target) and its own state (wheel/travel);
// it consumes the reference its parent commands (wheel/target); it writes one actuator port.
using wheel_ports = tp::TypedPorts<
  tc::PortList<tire_target, wheel_travel>, tc::PortList<wheel_target>, tc::PortList<wheel_torque>>;

// The tire consumes what the wheel exports.
using tire_ports =
  tp::TypedPorts<tc::PortList<>, tc::PortList<tire_target>, tc::PortList<>>;

// ---- controllers ---------------------------------------------------------------------------

/// Minimal commit sink: the wheel declares an actuator port, so the kernel requires a sink.
class CountingSink : public hierarchical_control::StagedCommandSink
{
public:
  bool commit(const double * values, std::size_t size) noexcept override
  {
    last_value = (size > 0) ? values[0] : 0.0;
    ++commits;
    return true;
  }

  int commits = 0;
  double last_value = 0.0;
};

/// Minimal external reference snapshot for the root.
class FixedReference : public hierarchical_control::StagedReferenceSource
{
public:
  bool read(
    std::uint64_t /*cycle*/, std::int64_t /*now_ns*/, double * values,
    std::size_t size) noexcept override
  {
    for (std::size_t i = 0; i < size; ++i) {values[i] = 0.0;}
    return true;
  }
};

class WheelController : public hierarchical_control_test::MinimalController,
                        public tp::TypedPortsMixin<WheelController, wheel_ports>
{
public:
  WheelController() : MinimalController("wheel") {}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    // A controller must write EVERY declared state port; the kernel verifies completeness, so an
    // unwritten port fails the cycle (review R3).
    for (std::size_t i = 0; i < state.size(); ++i) {state[i] = 1.0;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    // A controller must WRITE every port it declares: the kernel now verifies completeness, so a
    // declared-but-unwritten port fails the cycle instead of silently reusing the last value.
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = 1.0;}
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = 0.5;}
    return Return::OK;
  }

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return &sink;
  }

  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return &reference;
  }

  int state_calls = 0;
  int command_calls = 0;
  CountingSink sink;
  FixedReference reference;
};

class TireController : public hierarchical_control_test::MinimalController,
                       public tp::TypedPortsMixin<TireController, tire_ports>
{
public:
  TireController() : MinimalController("tire") {}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    for (std::size_t i = 0; i < state.size(); ++i) {state[i] = 2.0;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
};

// ---- fixtures for the parent/child declaration check ----------------------------------------
struct parent_out_n
{
  static constexpr auto value = st::NameOf("p/out");
};
struct child_out_n
{
  static constexpr auto value = st::NameOf("p/out");
};
struct child_out_wrong_n
{
  static constexpr auto value = st::NameOf("p/out");
};
using parent_out = tc::Port<parent_out_n, dm::LinearVelocity>;
using child_out = tc::Port<child_out_n, dm::LinearVelocity>;
using child_out_wrong = tc::Port<child_out_wrong_n, dm::Position>;

using agreeing_parent = tp::TypedPorts<tc::PortList<parent_out>, tc::PortList<>, tc::PortList<>>;
using agreeing_child = tp::TypedPorts<tc::PortList<>, tc::PortList<child_out>, tc::PortList<>>;
using wrong_dimension_child =
  tp::TypedPorts<tc::PortList<>, tc::PortList<child_out_wrong>, tc::PortList<>>;

// ---- topology ------------------------------------------------------------------------------
struct wheel_node_n
{
  static constexpr auto value = st::NameOf("wheel");
};
struct tire_node_n
{
  static constexpr auto value = st::NameOf("tire");
};
using wheel_node = st::Root<wheel_node_n>;
using tire_node = st::Descendant<tire_node_n, wheel_node>;

// The contract is DERIVED from the typed declaration rather than written out again.
using wheel_contract = tp::contract_of_t<wheel_ports>;
using tire_contract = tp::contract_of_t<tire_ports>;
}  // namespace

/// The generated strings are exactly the declared ports, in declaration order.
TEST(TypedPorts, generated_strings_match_the_declaration)
{
  WheelController controller;

  EXPECT_EQ((std::vector<std::string>{"wheel/target"}), controller.staged_reference_ports());
  EXPECT_EQ((std::vector<std::string>{"wheel/torque"}), controller.staged_actuator_ports());
  // State ports are derived as "<port>/state" for exported then consumed ports.
  EXPECT_EQ(
    (std::vector<std::string>{"tire/target/state", "wheel/travel/state", "wheel/target/state"}),
    controller.staged_state_ports());
}

/// The generated lists agree with the declaration; the checker is not vacuous, and it reports why
/// when it fails.
TEST(TypedPorts, interface_verification_succeeds_for_a_mixin_controller)
{
  WheelController controller;
  const char * reason = nullptr;
  const bool matches = tp::verify_ports_match_interface<WheelController, wheel_ports>(controller, &reason);
  EXPECT_TRUE(matches) << (reason ? reason : "");
}

/// The derived contract carries the declared ports, so the topology checks see the same facts.
TEST(TypedPorts, the_derived_contract_matches_the_declaration)
{
  static_assert(wheel_contract::produced_count == 2);
  static_assert(wheel_contract::consumed_count == 1);
  static_assert(tire_contract::produced_count == 0);
  static_assert(tire_contract::consumed_count == 1);
  SUCCEED();
}

/// A parent's exported ports must be exactly what its child consumes, in name, order and dimension.
TEST(TypedPorts, declarations_of_a_parent_and_child_are_checked)
{
  // wheel exports {tire/target, wheel/travel} while tire consumes {tire/target}: these genuinely
  // differ, so the pair must be reported as incompatible.
  static_assert(!tp::declarations_are_compatible<wheel_ports, tire_ports>());

  // A pair that does agree.
  static_assert(tp::declarations_are_compatible<agreeing_parent, agreeing_child>());

  // Same name, DIFFERENT dimension: rejected. This is the check the string-name world cannot make.
  static_assert(!tp::declarations_are_compatible<agreeing_parent, wrong_dimension_child>());
  SUCCEED();
}

/// A controller built from typed ports runs in a REAL execution group, which means its generated
/// port strings are compatible with what the kernel expects.
TEST(TypedPorts, a_mixin_controller_runs_in_a_real_group)
{
  WheelController wheel;
  TireController tire;

  const auto tire_binding = tc::make_leaf<tire_node, tire_contract>(&tire);
  const auto wheel_binding = tc::compose<wheel_node, wheel_contract>(&wheel, tire_binding);

  auto group = hierarchical_control::topology_binding::create_library_group(wheel_binding);
  ASSERT_NE(nullptr, group);
  EXPECT_EQ(2u, group->size());

  wheel.state_calls = 0;
  wheel.command_calls = 0;
  const auto result = group->run_ns(0, 1000000);
  EXPECT_EQ(hierarchical_control::StagedStatus::committed, result.status);
  EXPECT_EQ(1, wheel.state_calls);
  EXPECT_EQ(1, wheel.command_calls);
}
