// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Manager-level FineMote experiment: the SAME controller code is executed by the real
// ControllerManager in two modes over the SAME ordered controller list:
//
//   single pass (legacy)  : one update() per controller, list traversed FORWARD (parents first)
//   two pass (opt-in)     : update_phase() traversed BACKWARD, then handle_phase() FORWARD
//
// The cascade root -> mid -> leaf has a deliberately non-re-derivable estimate at every level
// (a first-order filter of the child's estimate), so a parent cannot reconstruct it from raw
// hardware. A step in the leaf input then exposes how many cycles each level's information lags.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <future>
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

constexpr char kRoot[] = "tp_root";
constexpr char kMid[] = "tp_mid";
constexpr char kLeaf[] = "tp_leaf";
constexpr char kType[] = "two_phase_test";

controller_interface::InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

struct LagResult
{
  int leaf = -1;
  int mid = -1;
  int root = -1;
  int update_phase_calls = 0;
  int handle_phase_calls = 0;
};

class TestTwoPhaseExecution : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  void SwitchNow(const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
      start, stop, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, future.wait_for(std::chrono::milliseconds(50)));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    EXPECT_EQ(Return::OK, future.get());
  }

  LagResult Run(bool two_phase)
  {
    root_ = std::make_shared<TestStagedController>();
    mid_ = std::make_shared<TestStagedController>();
    leaf_ = std::make_shared<TestStagedController>();

    // Reference chain: root claims mid/target, mid claims leaf/target. The manager's sorting puts
    // the list in parents-first order, which both modes reuse.
    root_->set_reference_interface_names({"command"});
    root_->set_command_interface_configuration(individual({std::string(kMid) + "/target"}));
    root_->set_state_interface_configuration(individual({}));
    root_->set_two_phase_child(mid_.get());

    mid_->set_reference_interface_names({"target"});
    mid_->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
    mid_->set_state_interface_configuration(individual({}));
    mid_->set_two_phase_child(leaf_.get());

    leaf_->set_reference_interface_names({"target"});
    leaf_->set_actuator_ports({"joint2/velocity"});
    leaf_->set_command_interface_configuration(individual({"joint2/velocity"}));
    leaf_->set_state_interface_configuration(individual({}));
    leaf_->set_two_phase_input(0.0);

    // In single-pass mode the native update() runs both phases, which is what the manager calls.
    root_->set_two_phase_legacy(!two_phase);
    mid_->set_two_phase_legacy(!two_phase);
    leaf_->set_two_phase_legacy(!two_phase);

    cm_->add_controller(root_, kRoot, kType);
    cm_->add_controller(mid_, kMid, kType);
    cm_->add_controller(leaf_, kLeaf, kType);
    EXPECT_EQ(Return::OK, cm_->configure_controller(kLeaf));
    EXPECT_EQ(Return::OK, cm_->configure_controller(kMid));
    EXPECT_EQ(Return::OK, cm_->configure_controller(kRoot));
    leaf_->set_chained_mode(true);
    mid_->set_chained_mode(true);
    EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(two_phase));
    EXPECT_EQ(two_phase, cm_->two_phase_execution());
    SwitchNow({kLeaf}, {});
    SwitchNow({kMid}, {});
    SwitchNow({kRoot}, {});

    const int step_cycle = 10;
    const int cycles = 25;
    LagResult result;
    for (int cycle = 1; cycle <= cycles; ++cycle)
    {
      if (cycle == step_cycle) {leaf_->set_two_phase_input(1.0);}
      cm_->read(TIME, PERIOD);
      EXPECT_EQ(Return::OK, cm_->update(TIME, PERIOD));
      cm_->write(TIME, PERIOD);

      const int lag = cycle - step_cycle;
      if (lag >= 0)
      {
        if (result.leaf < 0 && leaf_->two_phase_estimate() > 0.05) {result.leaf = lag;}
        if (result.mid < 0 && mid_->two_phase_estimate() > 0.05) {result.mid = lag;}
        if (result.root < 0 && root_->two_phase_estimate() > 0.05) {result.root = lag;}
      }
    }
    result.update_phase_calls = leaf_->update_phase_calls;
    result.handle_phase_calls = leaf_->handle_phase_calls;
    if (cm_->staged_execution_group()) {cm_->clear_staged_execution_group();}
    return result;
  }

  std::shared_ptr<TestStagedController> root_, mid_, leaf_;
};

/// Three-level chain shared by the admission tests: root claims mid/target, mid claims leaf/target,
/// leaf owns the mock hardware port. Mirrors the topology used by the staged group tests.
class TestExecutionPathAdmission
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  void TearDown() override
  {
    if (cm_ && cm_->staged_execution_group()) {cm_->clear_staged_execution_group();}
    ControllerManagerFixture::TearDown();
  }

  void SwitchNow(const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
      start, stop, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, future.wait_for(std::chrono::milliseconds(50)));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    ASSERT_EQ(Return::OK, future.get());
  }

  void MakeChainController(
    const std::shared_ptr<TestStagedController> & controller, const std::string & name,
    const std::string & reference_suffix, const std::vector<std::string> & claimed_children,
    const std::vector<std::string> & actuator_ports, const std::vector<std::string> & state_ports)
  {
    controller->set_reference_interface_names({reference_suffix});
    std::vector<std::string> claimed = claimed_children;
    for (const auto & port : actuator_ports) {claimed.push_back(port);}
    controller->set_command_interface_configuration(individual(claimed));
    controller->set_state_interface_configuration(individual(state_ports));
    controller->set_actuator_ports(actuator_ports);
    cm_->add_controller(controller, name, kType);
  }

  /// Configure the chain but leave it inactive. `leaf_update_rate` of 0 means "follow the manager".
  void BuildChain(unsigned int leaf_update_rate)
  {
    root_ = std::make_shared<TestStagedController>();
    mid_ = std::make_shared<TestStagedController>();
    leaf_ = std::make_shared<TestStagedController>();

    MakeChainController(root_, kRoot, "command", {std::string(kMid) + "/target"}, {}, {});
    MakeChainController(mid_, kMid, "target", {std::string(kLeaf) + "/target"}, {}, {});
    MakeChainController(leaf_, kLeaf, "target", {}, {"joint2/velocity"}, {"joint2/position"});

    if (leaf_update_rate != 0)
    {
      // The controller reads this parameter in configure(); the fixture node allows undeclared
      // parameters, so no declaration is needed.
      leaf_->get_node()->set_parameter({"update_rate", static_cast<int>(leaf_update_rate)});
    }

    ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kMid));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
    ASSERT_EQ(leaf_update_rate, leaf_->get_update_rate());
    ASSERT_TRUE(leaf_->set_chained_mode(true));
    ASSERT_TRUE(mid_->set_chained_mode(true));
  }

  void Cycle(int count)
  {
    for (int i = 0; i < count; ++i)
    {
      cm_->read(TIME, PERIOD);
      ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
      cm_->write(TIME, PERIOD);
    }
  }

  std::shared_ptr<TestStagedController> root_, mid_, leaf_;
};

/// R7: the two-phase passes always run every cycle, so a controller whose own update rate differs
/// from the manager's would silently be called off-rate. Enabling must therefore be refused, and
/// the controller must keep running through the native, rate-gated loop.
TEST_F(TestExecutionPathAdmission, two_phase_enable_is_refused_for_a_rate_mismatched_controller)
{
  ASSERT_GE(cm_->get_update_rate(), 2u);
  const unsigned int half_rate = cm_->get_update_rate() / 2;
  BuildChain(half_rate);

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution()) << "a refused request must not flip the flag";

  // Make the native path run both halves so the counters show which path did the work.
  root_->set_two_phase_legacy(true);
  mid_->set_two_phase_legacy(true);
  leaf_->set_two_phase_legacy(true);

  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});

  const int leaf_phase0 = leaf_->update_phase_calls;
  Cycle(10);

  // 10 consecutive cycles at half the manager's rate: the native loop runs the controller on every
  // other cycle, so 5 update/handle pairs -- and none of them comes from the two-phase passes.
  EXPECT_EQ(5, leaf_->update_phase_calls - leaf_phase0);
  EXPECT_EQ(leaf_->update_phase_calls, leaf_->handle_phase_calls);
  EXPECT_GT(leaf_->legacy_update_calls, 0);
}

/// R7 mirror image: while two-phase execution owns a controller, it must not also enter the staged
/// group, because the group is executed in the same cycle.
TEST_F(TestExecutionPathAdmission, staged_group_is_refused_for_a_two_phase_controller)
{
  BuildChain(0);

  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ASSERT_TRUE(cm_->two_phase_execution());

  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({kRoot, kMid, kLeaf}));
  EXPECT_EQ(nullptr, cm_->staged_execution_group());
}

/// R7 mirror image: while a controller is a staged group member, two-phase execution must refuse
/// to enable rather than execute it twice per cycle.
TEST_F(TestExecutionPathAdmission, two_phase_enable_is_refused_for_a_staged_member)
{
  BuildChain(0);
  ASSERT_EQ(Return::OK, cm_->set_staged_execution_group({kRoot, kMid, kLeaf}));
  ASSERT_NE(nullptr, cm_->staged_execution_group());

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());

  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});

  const int root_state0 = root_->state_calls;
  const int leaf_state0 = leaf_->state_calls;
  Cycle(10);

  // The group still owns the members and runs each stage once per cycle; the two-phase passes never
  // touch them.
  EXPECT_EQ(10, root_->state_calls - root_state0);
  EXPECT_EQ(10, leaf_->state_calls - leaf_state0);
  EXPECT_EQ(0, root_->update_phase_calls);
  EXPECT_EQ(0, leaf_->update_phase_calls);
  EXPECT_EQ(0, root_->legacy_update_calls);
  EXPECT_EQ(0, leaf_->legacy_update_calls);
}

/// R7: there is no staged equivalent of the native per-controller rate gate either.
TEST_F(TestExecutionPathAdmission, staged_group_is_refused_for_a_rate_mismatched_member)
{
  ASSERT_GE(cm_->get_update_rate(), 2u);
  BuildChain(cm_->get_update_rate() / 2);

  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({kRoot, kMid, kLeaf}));
  EXPECT_EQ(nullptr, cm_->staged_execution_group());
}

/// R7: exactly the manager's rate is accepted, so the checks reject the mismatch and not the
/// feature itself.
TEST_F(TestExecutionPathAdmission, staged_group_accepts_a_rate_matching_member)
{
  BuildChain(cm_->get_update_rate());

  EXPECT_EQ(Return::OK, cm_->set_staged_execution_group({kRoot, kMid, kLeaf}));
  EXPECT_NE(nullptr, cm_->staged_execution_group());
}

/// R7: a controller that follows the manager's rate (0) is the ordinary case and stays accepted.
TEST_F(TestExecutionPathAdmission, two_phase_enable_accepts_the_default_rate)
{
  BuildChain(0);
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  EXPECT_TRUE(cm_->two_phase_execution());
}

/// Native single pass: the manager walks parents first, so every parent ingests its child's
/// estimate from the PREVIOUS cycle and the lag grows with depth.
TEST_F(TestTwoPhaseExecution, single_pass_lags_by_depth)
{
  const auto result = Run(false);
  EXPECT_EQ(0, result.leaf);
  EXPECT_EQ(1, result.mid);
  EXPECT_EQ(2, result.root);
  EXPECT_GT(result.update_phase_calls, 0);
  EXPECT_EQ(result.update_phase_calls, result.handle_phase_calls);
}

/// FineMote two pass: the same list is walked backward for update_phase and forward for
/// handle_phase, so every level sees this cycle's data and the lag is zero everywhere.
TEST_F(TestTwoPhaseExecution, two_pass_keeps_both_directions_same_cycle)
{
  const auto result = Run(true);
  EXPECT_EQ(0, result.leaf) << "leaf is the source and must react immediately";
  EXPECT_EQ(0, result.mid) << "mid must see the leaf's same-cycle estimate";
  EXPECT_EQ(0, result.root) << "root must see the mid's same-cycle estimate";
  EXPECT_GT(result.update_phase_calls, 0);
  EXPECT_EQ(result.update_phase_calls, result.handle_phase_calls);
}

}  // namespace
