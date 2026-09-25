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
struct tire_travel_n
{
  static constexpr auto value = st::NameOf("tire/travel");
};

// ---- typed ports ---------------------------------------------------------------------------
using wheel_target = tc::Port<wheel_target_n, dm::LinearVelocity>;
using wheel_torque = tc::Port<wheel_torque_n, dm::Torque>;
using wheel_travel = tc::Port<wheel_travel_n, dm::Position>;
using tire_target = tc::Port<tire_target_n, dm::LinearVelocity>;
using tire_travel = tc::Port<tire_travel_n, dm::Position>;
using tire_travel_wrong = tc::Port<tire_travel_n, dm::LinearVelocity>;

// Five separate facts. The first three are what the kernel uses; the last two exist so BOTH edges
// of the parent/child pair can be checked statically:
//   state        = the wheel's OWN state (its parent's state stage reads it out of the wheel's slots)
//   reference    = the reference the wheel RECEIVES (its parent's command stage writes it)
//   actuators    = the hardware port the wheel writes
//   for_children = the reference the wheel writes INTO its tire   (reference edge)
//   child_state  = the state the wheel READS FROM its tire        (state edge)
using wheel_ports = tp::TypedPorts<
  tc::PortList<wheel_travel>, tc::PortList<wheel_target>, tc::PortList<wheel_torque>,
  tc::PortList<tire_target>, tc::PortList<tire_travel>>;

// The tire publishes its own state and receives what the wheel writes into it.
using tire_ports = tp::TypedPorts<
  tc::PortList<tire_travel>, tc::PortList<tire_target>, tc::PortList<>, tc::PortList<>,
  tc::PortList<>>;

// The same parent, but declaring the WRONG DIMENSION for the state it reads from its child.
using wheel_ports_wrong_child_state = tp::TypedPorts<
  tc::PortList<wheel_travel>, tc::PortList<wheel_target>, tc::PortList<wheel_torque>,
  tc::PortList<tire_target>, tc::PortList<tire_travel_wrong>>;

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

  int state_calls = 0;

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

/// A controller that does NOT use the mixin: it hand-writes its port strings. That is exactly the
/// case `verify_ports_match_interface` and `verify_ports_match_contract` exist to catch, and it
/// cannot be expressed by deriving from the mixin because the mixin's accessors are `final`.
class HandWrittenController : public hierarchical_control_test::MinimalController,
                              public hierarchical_control::StagedControllerInterface
{
public:
  explicit HandWrittenController(
    std::vector<std::string> state, std::vector<std::string> reference = {"wheel/target"})
  : MinimalController("hand_written"), state_(std::move(state)), reference_(std::move(reference))
  {
  }

  std::vector<std::string> staged_state_ports() const override {return state_;}
  std::vector<std::string> staged_reference_ports() const override {return reference_;}
  std::vector<std::string> staged_actuator_ports() const override {return {"wheel/torque"};}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &, const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter) noexcept override
  {
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

private:
  std::vector<std::string> state_;
  std::vector<std::string> reference_;
};

// Two same-length ports used only to show that ORDER is compared, not just membership.
struct a_n
{
  static constexpr auto value = st::NameOf("p/a");
};
struct b_n
{
  static constexpr auto value = st::NameOf("p/b");
};
using a_port = tc::Port<a_n, dm::Position>;
using b_port = tc::Port<b_n, dm::LinearVelocity>;
using two_state = tc::Contract<tc::PortList<a_port, b_port>, tc::PortList<>>;

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

// The parent declares what it writes into its child; the child declares what it receives.
using agreeing_parent = tp::TypedPorts<
  tc::PortList<>, tc::PortList<>, tc::PortList<>, tc::PortList<parent_out>>;
using agreeing_child =
  tp::TypedPorts<tc::PortList<>, tc::PortList<child_out>, tc::PortList<>, tc::PortList<>>;
using wrong_dimension_child =
  tp::TypedPorts<tc::PortList<>, tc::PortList<child_out_wrong>, tc::PortList<>, tc::PortList<>>;

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
  // Exactly the declared state ports -- one, not three. The previous scheme appended "/state" to
  // the exported AND the consumed list, which gave the kernel three state slots for a node with one
  // real state port; since the kernel pre-fills state slots with NaN and requires all of them to be
  // written, any controller that wrote only its real state would have failed every cycle.
  EXPECT_EQ((std::vector<std::string>{"wheel/travel"}), controller.staged_state_ports());
  // The counts the kernel sizes its buffers from.
  static_assert(wheel_ports::kernel_state_slots == 1);
  static_assert(wheel_ports::kernel_reference_slots == 1);
  static_assert(wheel_ports::kernel_actuator_slots == 1);
}

/// The generated lists agree with the declaration; the checker is not vacuous, and it reports why
/// when it fails.
TEST(TypedPorts, interface_verification_succeeds_for_a_mixin_controller)
{
  WheelController controller;
  std::string reason;
  const bool matches = tp::verify_ports_match_interface<WheelController, wheel_ports>(controller, &reason);
  EXPECT_TRUE(matches) << reason;
}

/// Order matters, not just membership: a controller that reports the RIGHT ports in the WRONG order
/// must be rejected.
TEST(TypedPorts, contract_verification_checks_order_not_just_membership)
{
  HandWrittenController in_order({"p/a", "p/b"}, {});
  std::string reason;
  EXPECT_TRUE(tp::verify_ports_match_contract<two_state>(in_order, &reason))
    << reason;

  HandWrittenController swapped({"p/b", "p/a"}, {});
  std::string swapped_reason;
  EXPECT_FALSE(tp::verify_ports_match_contract<two_state>(swapped, &swapped_reason));
  ASSERT_FALSE(swapped_reason.empty());
  EXPECT_NE(std::string::npos, swapped_reason.find("staged_state_ports"))
    << "got: " << swapped_reason;
}

/// The non-vacuous counterpart: the state list IS verified now. A controller whose state ports
/// disagree with its declaration must be reported, with a reason that names the list.
TEST(TypedPorts, interface_verification_rejects_a_state_port_mismatch)
{
  HandWrittenController controller({"not/a/state"});
  std::string reason;
  const bool matches =
    tp::verify_ports_match_interface<HandWrittenController, wheel_ports>(controller, &reason);
  EXPECT_FALSE(matches);
  ASSERT_FALSE(reason.empty());
  EXPECT_NE(std::string::npos, std::string(reason).find("staged_state_ports"))
    << "the reason must name the list that disagrees, got: " << reason;
}

/// A controller can be verified against the CONTRACT it was bound with, which is what ties the
/// compile-time-checked topology to the runtime port lists the kernel sizes its buffers from.
TEST(TypedPorts, a_controller_can_be_verified_against_its_contract)
{
  // The mixin makes a mismatch impossible, so it passes; the interesting case is a controller that
  // hand-writes its strings.
  WheelController mixin_controller;
  std::string reason;
  EXPECT_TRUE(tp::verify_ports_match_contract<wheel_contract>(mixin_controller, &reason))
    << reason;

  HandWrittenController agreeing({"wheel/travel"});
  EXPECT_TRUE(tp::verify_ports_match_contract<wheel_contract>(agreeing, &reason))
    << reason;

  // The check is not vacuous: one state port more than the contract declares would make the kernel
  // allocate a state buffer the checked topology does not know about.
  HandWrittenController extra({"wheel/travel", "wheel/extra"});
  std::string extra_reason;
  EXPECT_FALSE(tp::verify_ports_match_contract<wheel_contract>(extra, &extra_reason));
  ASSERT_FALSE(extra_reason.empty());
  EXPECT_NE(std::string::npos, extra_reason.find("staged_state_ports"))
    << "got: " << extra_reason;

  // SAME LENGTH, WRONG NAME: the check must compare names, not just counts (review item C: the
  // earlier revision compared only `.size()` while its comment claimed name-and-order checking).
  HandWrittenController wrong_name({"wheel/travel_wrong"});
  std::string name_reason;
  EXPECT_FALSE(tp::verify_ports_match_contract<wheel_contract>(wrong_name, &name_reason));
  ASSERT_FALSE(name_reason.empty());
  EXPECT_NE(std::string::npos, name_reason.find("staged_state_ports"))
    << "got: " << name_reason;

  // And a wrong reference count is reported against the reference list.
  class WrongReference : public HandWrittenController
  {
  public:
    WrongReference() : HandWrittenController({"wheel/travel"}) {}
    std::vector<std::string> staged_reference_ports() const override {return {};}
  };
  WrongReference wrong_reference;
  std::string reference_reason;
  EXPECT_FALSE(tp::verify_ports_match_contract<wheel_contract>(wrong_reference, &reference_reason));
  ASSERT_FALSE(reference_reason.empty());
  EXPECT_NE(std::string::npos, reference_reason.find("staged_reference_ports"))
    << "got: " << reference_reason;
}

/// The derived contract carries the declared ports, so the topology checks see the same facts.
TEST(TypedPorts, the_derived_contract_matches_the_declaration)
{
  static_assert(wheel_contract::produced_count == 1, "the wheel's own state");
  static_assert(wheel_contract::consumed_count == 1, "the reference the wheel receives");
  static_assert(tire_contract::produced_count == 1, "the tire's own state");
  static_assert(tire_contract::consumed_count == 1, "the reference the tire receives");
  SUCCEED();
}

/// Both edges of a parent/child pair are checked: the reference the parent writes into its child
/// against what the child receives, and the child state the parent consumes against what the child
/// publishes -- each in name, order and dimension, and independently of the other.
TEST(TypedPorts, declarations_of_a_parent_and_child_are_checked)
{
  // The wheel declares that it writes `tire/target` into its child, and the tire declares that it
  // receives `tire/target`: the pair AGREES. The previous revision compared the parent's whole
  // exported list (which mixed its own state with the child-facing reference) against the child's
  // reference list, so it reported this correct pair as incompatible and this test encoded that
  // false positive as expected behaviour.
  static_assert(tp::reference_declarations_agree<wheel_ports, tire_ports>());

  // The state edge, which no earlier revision checked at all: the wheel declares it consumes
  // `tire/travel` and the tire declares it publishes `tire/travel`.
  static_assert(tp::state_declarations_agree<wheel_ports, tire_ports>());

  static_assert(tp::declarations_are_compatible<wheel_ports, tire_ports>());

  // The two edges are checked INDEPENDENTLY: a parent that declares the wrong dimension for the
  // state it reads from its child fails only the state check, while the reference edge still agrees.
  // This is the failure the runtime kernel would otherwise report as an unexplained `state_failed`
  // in the parent, far from the declaration that caused it.
  static_assert(tp::reference_declarations_agree<wheel_ports_wrong_child_state, tire_ports>());
  static_assert(!tp::state_declarations_agree<wheel_ports_wrong_child_state, tire_ports>());
  static_assert(!tp::declarations_are_compatible<wheel_ports_wrong_child_state, tire_ports>());

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
