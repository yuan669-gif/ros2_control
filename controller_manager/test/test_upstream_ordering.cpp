// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Corollary 1 (doc/FORMAL_MODEL.md) on REAL ros2_control code, not on a port.
//
// A bidirectional pair: the parent claims `child/ref` (reference edge) AND `child/state`
// (state edge). Corollary 1 predicts that any order respecting the reference edge must leave the
// state edge stale, and that the manager's single-pass ordering therefore always puts the
// reference producer first - independently of registration order.
//
// Humble cannot actually export a controller state interface (that is Jazzy+), so the state edge
// is expressed only as a claimed name: this test checks the ORDERING behaviour of the real
// `controller_sorting` comparator, which is exactly where the conflict lives.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "test_staged_controller/test_staged_controller.hpp"

namespace
{
using TestStagedController = test_staged_controller::TestStagedController;
using Return = controller_interface::return_type;

constexpr char kParent[] = "ord_parent";
constexpr char kChild[] = "ord_child";
constexpr char kType[] = "ordering_test";

controller_interface::InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

std::vector<std::string> LoadedNames(const controller_manager::ControllerManager & cm)
{
  std::vector<std::string> names;
  for (const auto & spec : cm.get_loaded_controllers()) {names.push_back(spec.info.name);}
  return names;
}

class TestUpstreamOrdering : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  /// Request a switch and drive real-time cycles until the manager applies it. Returns the
  /// switch result instead of asserting, because one of the tests below is about a switch that is
  /// EXPECTED to fail.
  Return SwitchResult(const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
      start, stop, STRICT, true, rclcpp::Duration(0, 0));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return Return::ERROR;
    }
    return future.get();
  }

  bool IsActive(const std::string & name)
  {
    for (const auto & spec : cm_->get_loaded_controllers())
    {
      if (spec.info.name == name)
      {
        return spec.c->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      }
    }
    return false;
  }

  void Build(bool parent_registered_first)
  {
    parent_ = std::make_shared<TestStagedController>();
    child_ = std::make_shared<TestStagedController>();

    // Parent: produces the child's reference (reference edge) and consumes the child's state.
    parent_->set_reference_interface_names({"src"});
    parent_->set_command_interface_configuration(individual({std::string(kChild) + "/ref"}));
    parent_->set_state_interface_configuration(individual({std::string(kChild) + "/state"}));

    // Child: exports the reference and drives one hardware command interface.
    child_->set_reference_interface_names({"ref"});
    child_->set_actuator_ports({"joint2/velocity"});
    child_->set_command_interface_configuration(individual({"joint2/velocity"}));
    child_->set_state_interface_configuration(individual({}));

    if (parent_registered_first)
    {
      cm_->add_controller(parent_, kParent, kType);
      cm_->add_controller(child_, kChild, kType);
    }
    else
    {
      cm_->add_controller(child_, kChild, kType);
      cm_->add_controller(parent_, kParent, kType);
    }

    // Configure the child first so its reference interface is exported before the parent is sorted.
    ASSERT_EQ(Return::OK, cm_->configure_controller(kChild));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kParent));
  }

  std::shared_ptr<TestStagedController> parent_, child_;
};

/// Registration order must not change the resulting order: the reference edge wins, so the state
/// edge is the one that ends up stale.
TEST_F(TestUpstreamOrdering, reference_wins_regardless_of_registration_order)
{
  Build(true);
  const auto names = LoadedNames(*cm_);
  ASSERT_EQ(2u, names.size());
  EXPECT_EQ(kParent, names[0]);
  EXPECT_EQ(kChild, names[1]);
}

TEST_F(TestUpstreamOrdering, reference_wins_even_when_child_is_registered_first)
{
  Build(false);
  const auto names = LoadedNames(*cm_);
  ASSERT_EQ(2u, names.size());
  EXPECT_EQ(kParent, names[0]) << "the parent must precede the child, so the state edge is the "
                                  "stale one (Corollary 1)";
  EXPECT_EQ(kChild, names[1]);
}

/// R10: the REFERENCE half of the bidirectional pair must be executable end to end, not merely
/// accepted by `configure`. The parent claims the child's exported reference interface, so it must
/// hold a real bound command interface, and the child must observe the value the parent wrote.
///
/// (`configure` returning OK only proves the name resolved; the reviewed revision was criticised
/// for treating that as evidence that the whole native data path works.)
TEST_F(TestUpstreamOrdering, reference_edge_is_bound_and_propagates_within_the_cycle)
{
  parent_ = std::make_shared<TestStagedController>();
  child_ = std::make_shared<TestStagedController>();

  // Child: exports `ref` and drives one hardware command interface.
  child_->set_native_mode(true, 0.0, 0.0);
  child_->set_reference_interface_names({"ref"});
  child_->set_actuator_ports({"joint2/velocity"});
  child_->set_command_interface_configuration(individual({"joint2/velocity"}));
  child_->set_state_interface_configuration(individual({}));

  // Parent: WRITES the child's reference (the claim) and takes its own reference from `src`.
  parent_->set_native_mode(true, 0.0, 0.0);
  parent_->set_reference_interface_names({"src"});
  parent_->set_command_interface_configuration(individual({std::string(kChild) + "/ref"}));
  parent_->set_state_interface_configuration(individual({}));

  cm_->add_controller(parent_, kParent, kType);
  cm_->add_controller(child_, kChild, kType);
  ASSERT_EQ(Return::OK, cm_->configure_controller(kChild));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kParent));
  child_->set_chained_mode(true);

  // Activate the reference producer last: the child's exported interface must exist to be claimed.
  ASSERT_EQ(Return::OK, SwitchResult({kChild}, {}));
  ASSERT_EQ(Return::OK, SwitchResult({kParent}, {}));
  ASSERT_TRUE(IsActive(kParent));
  ASSERT_TRUE(IsActive(kChild));

  // (1) The claim is REAL: one ResourceManager-owned command interface is bound to the parent.
  EXPECT_EQ(1u, parent_->command_interface_count())
    << "the parent must actually hold the claimed reference interface, not just name it";

  // (2) The value crosses the edge. The parent writes 7.0 into the interface it claimed, which is
  // the child's own reference storage, and the child consumes it in the same manager cycle.
  parent_->set_external_reference(7.0);
  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  cm_->write(TIME, PERIOD);

  ASSERT_FALSE(child_->last_reference.empty())
    << "the child must have consumed a reference value at all";
  EXPECT_DOUBLE_EQ(7.0, child_->last_reference[0])
    << "the reference writer runs first, so the child sees this cycle's value";
  EXPECT_DOUBLE_EQ(7.0, child_->command_interface_value())
    << "and the consumed reference reaches the hardware command interface";
}

/// R10: the STATE half of the pair cannot be expressed on Humble at all.
///
/// `configure` accepts a state interface name that no controller exports, so the ordering test
/// above can only exercise the comparator. This test establishes the real boundary: Humble's
/// `ChainableControllerInterface` has no `export_state_interfaces()`, so a parent cannot claim a
/// child's *state*. Whatever activation does with such a name, the state port is never bound, and
/// that - not a scheduling result - is what limits the Humble experiment.
TEST_F(TestUpstreamOrdering, state_edge_cannot_be_bound_on_humble)
{
  Build(true);

  // Before activation, the parent's requirement set contains the state name.
  const auto parent_cfg = parent_->state_interface_configuration();
  EXPECT_EQ(
    std::vector<std::string>({std::string(kChild) + "/state"}), parent_cfg.names)
    << "the parent declares a controller-state interface that Humble cannot export";

  const auto result = SwitchResult({kParent}, {});
  const auto bound = parent_->state_interface_count();
  std::cout << "[upstream] activating a controller that declares '" << kChild
            << "/state': switch=" << (result == Return::OK ? "OK" : "ERROR")
            << " active=" << (IsActive(kParent) ? "yes" : "no")
            << " bound_state_interfaces=" << bound << "\n";

  // MEASURED (this is what the assertions encode; an earlier draft predicted the opposite):
  //   - configure ACCEPTS the name (`Build` asserts that), because Humble resolves state interfaces
  //     at ACTIVATION, not at configure time;
  //   - activation then FAILS: the manager logs "Aborting, no controller is switched! (::STRICT
  //     switch)" and the parent stays inactive, because no controller can export a state interface
  //     for it to claim;
  //   - consequently nothing is ever bound.
  // So the reviewer's point is proven the hard way: a successful `configure` says nothing about
  // whether the native data path is executable. On Humble a parent-child STATE dependency cannot
  // be instantiated at all, which is why the bidirectional pair is only representable as an
  // ordering constraint here, and why `BIDIRECTIONAL_EDGE_ANALYSIS.md` has to lean on Jazzy for
  // the real state datapath.
  EXPECT_EQ(0u, bound)
    << "no controller can export a state interface on Humble, so nothing can be bound here";
  EXPECT_EQ(Return::ERROR, result)
    << "activation must be refused, so that a configure result cannot be mistaken for a working "
       "data path";
  EXPECT_FALSE(IsActive(kParent));
}

}  // namespace
