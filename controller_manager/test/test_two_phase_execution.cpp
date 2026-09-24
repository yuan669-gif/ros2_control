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

#include <algorithm>
#include <chrono>
#include <iostream>
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

/// The two-phase path has NO group commit, and this test pins that down instead of leaving it as a
/// footnote. `handle_phase()` writes straight into the controller's command interfaces, the command
/// pass runs parents first, and the manager keeps no buffer, so a failure late in the pass leaves
/// the earlier writes applied. The staged group's "all-or-nothing" guarantee (`failure_never_
/// partially_commits` in `test_staged_execution_group.cpp`) does NOT carry over to this path.
TEST_F(TestTwoPhaseExecution, two_phase_mode_has_no_group_commit)
{
  auto root = std::make_shared<TestStagedController>();
  auto mid = std::make_shared<TestStagedController>();
  auto leaf = std::make_shared<TestStagedController>();

  root->set_reference_interface_names({"command"});
  root->set_command_interface_configuration(individual({std::string(kMid) + "/target"}));
  root->set_state_interface_configuration(individual({}));
  mid->set_reference_interface_names({"target"});
  mid->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  mid->set_state_interface_configuration(individual({}));
  leaf->set_reference_interface_names({"target"});
  leaf->set_actuator_ports({"joint2/velocity"});
  leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  leaf->set_state_interface_configuration(individual({}));
  // The two-phase estimate path is an explicit child link, not a reference interface.
  root->set_two_phase_child(mid.get());
  mid->set_two_phase_child(leaf.get());

  ASSERT_NE(nullptr, cm_->add_controller(root, kRoot, kType));
  ASSERT_NE(nullptr, cm_->add_controller(mid, kMid, kType));
  ASSERT_NE(nullptr, cm_->add_controller(leaf, kLeaf, kType));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kMid));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
  ASSERT_TRUE(leaf->set_chained_mode(true));
  ASSERT_TRUE(mid->set_chained_mode(true));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));

  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});

  // Drive a few clean cycles so every level has a non-zero command to overwrite.
  root->set_two_phase_input(0.0);
  for (int i = 0; i < 5; ++i)
  {
    cm_->read(TIME, PERIOD);
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    cm_->write(TIME, PERIOD);
  }
  leaf->set_two_phase_input(5.0);   // the leaf's estimate, hence mid's command, changes
  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  cm_->write(TIME, PERIOD);

  const double mid_written = mid->command_interface_value();
  const int mid_handle_calls = mid->handle_phase_calls;

  // Now the LEAF fails its command stage. The command pass is parents-first, so `mid` has already
  // written the leaf's reference by the time the leaf fails.
  leaf->set_fail_handle(true);
  leaf->set_two_phase_input(9.0);
  cm_->read(TIME, PERIOD);
  const auto failed = cm_->update(TIME, PERIOD);
  cm_->write(TIME, PERIOD);

  EXPECT_EQ(Return::ERROR, failed) << "the manager must report the failed command stage";
  EXPECT_EQ(mid_handle_calls + 1, mid->handle_phase_calls) << "mid ran before the leaf failed";

  std::cout << "[two-phase] cycle with a failing leaf command stage: mid's claimed interface went "
            << mid_written << " -> " << mid->command_interface_value() << "\n";
  EXPECT_NE(mid_written, mid->command_interface_value())
    << "mid's write survives the leaf's failure: the two-phase path commits partially";
  EXPECT_EQ(1, leaf->handle_phase_calls - (mid_handle_calls + 1))
    << "the leaf is still called after failing, so the pass is not aborted cleanly either";

  // The contrast is the point: the same topology under the staged group does commit atomically,
  // which is asserted in test_staged_execution_group.failure_never_partially_commits.
}

/// Cost of the second traversal, measured on the same three-controller cascade in both modes.
/// This asserts nothing about the numbers: the machine is a non-real-time VM and the review asked
/// for the measurement to exist, not for a particular outcome.
TEST_F(TestTwoPhaseExecution, two_pass_costs_one_extra_traversal)
{
  // ONE setup, measured in both modes by flipping `two_phase_execution` and the per-controller
  // `two_phase_legacy` switch. Both modes then execute the same three controllers over the same
  // hardware interfaces, so the comparison is apples to apples: single-pass calls the native
  // `update()` on each controller (which runs its two halves), two-pass has the manager drive the
  // two passes itself.
  auto root = std::make_shared<TestStagedController>();
  auto mid = std::make_shared<TestStagedController>();
  auto leaf = std::make_shared<TestStagedController>();
  root->set_reference_interface_names({"command"});
  root->set_command_interface_configuration(individual({std::string(kMid) + "/target"}));
  root->set_state_interface_configuration(individual({}));
  mid->set_reference_interface_names({"target"});
  mid->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  mid->set_state_interface_configuration(individual({}));
  leaf->set_reference_interface_names({"target"});
  leaf->set_actuator_ports({"joint2/velocity"});
  leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  leaf->set_state_interface_configuration(individual({}));
  root->set_two_phase_child(mid.get());
  mid->set_two_phase_child(leaf.get());

  ASSERT_NE(nullptr, cm_->add_controller(root, kRoot, kType));
  ASSERT_NE(nullptr, cm_->add_controller(mid, kMid, kType));
  ASSERT_NE(nullptr, cm_->add_controller(leaf, kLeaf, kType));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kMid));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
  ASSERT_TRUE(leaf->set_chained_mode(true));
  ASSERT_TRUE(mid->set_chained_mode(true));
  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});

  const auto measure = [this](bool two_phase)
  {
    std::vector<double> samples;
    samples.reserve(200);
    for (int i = 0; i < 200; ++i)
    {
      const auto start = std::chrono::steady_clock::now();
      cm_->read(TIME, PERIOD);
      cm_->update(TIME, PERIOD);
      cm_->write(TIME, PERIOD);
      samples.push_back(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
  };

  // Single-pass: every controller runs its own native update(), i.e. its two halves in sequence.
  for (auto * controller : {root.get(), mid.get(), leaf.get()})
  {
    controller->set_two_phase_legacy(true);
  }
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(false));
  const double single_pass_us = measure(false);

  // Two-pass: the manager drives the state stage backwards and the command stage forwards.
  for (auto * controller : {root.get(), mid.get(), leaf.get()})
  {
    controller->set_two_phase_legacy(false);
  }
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  const double two_pass_us = measure(true);
  std::cout << "[two-phase] median read+update+write over 200 cycles: single-pass "
            << single_pass_us << " us, two-pass " << two_pass_us << " us (ratio "
            << (two_pass_us / single_pass_us) << ")\n";
  EXPECT_GT(single_pass_us, 0.0);
  EXPECT_GT(two_pass_us, 0.0);
}

/// A failed STATE stage suppresses the COMMAND stage for the whole cycle.
///
/// The two-phase contract has no cross-controller rollback (see `two_phase_mode_has_no_group_commit`),
/// but it must not run a controller's command stage on the state that controller just rejected. The
/// rule implemented here is the weakest honest one: if any state stage fails, no command stage runs
/// in that cycle, so the command interfaces keep the previous cycle's values and nothing is
/// half-computed from a rejected state.
TEST_F(TestTwoPhaseExecution, a_failed_state_stage_suppresses_the_command_stage)
{
  auto root = std::make_shared<TestStagedController>();
  auto mid = std::make_shared<TestStagedController>();
  auto leaf = std::make_shared<TestStagedController>();

  root->set_reference_interface_names({"command"});
  root->set_command_interface_configuration(individual({std::string(kMid) + "/target"}));
  root->set_state_interface_configuration(individual({}));
  mid->set_reference_interface_names({"target"});
  mid->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  mid->set_state_interface_configuration(individual({}));
  leaf->set_reference_interface_names({"target"});
  leaf->set_actuator_ports({"joint2/velocity"});
  leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  leaf->set_state_interface_configuration(individual({}));
  root->set_two_phase_child(mid.get());
  mid->set_two_phase_child(leaf.get());

  ASSERT_NE(nullptr, cm_->add_controller(root, kRoot, kType));
  ASSERT_NE(nullptr, cm_->add_controller(mid, kMid, kType));
  ASSERT_NE(nullptr, cm_->add_controller(leaf, kLeaf, kType));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kMid));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
  ASSERT_TRUE(leaf->set_chained_mode(true));
  ASSERT_TRUE(mid->set_chained_mode(true));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));

  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});

  for (int cycle = 0; cycle < 3; ++cycle)
  {
    cm_->read(TIME, PERIOD);
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    cm_->write(TIME, PERIOD);
  }

  const int root_handle = root->handle_phase_calls;
  const int mid_handle = mid->handle_phase_calls;
  const int leaf_handle = leaf->handle_phase_calls;
  const double leaf_command = leaf->command_interface_value();

  // The LEAF's state stage fails. Pass 1 walks children first, so the root and mid state stages
  // already ran; the command pass must not run at all.
  leaf->set_fail_update(true);
  cm_->read(TIME, PERIOD);
  const auto failed = cm_->update(TIME, PERIOD);
  cm_->write(TIME, PERIOD);

  EXPECT_EQ(Return::ERROR, failed);
  EXPECT_EQ(root_handle, root->handle_phase_calls) << "no command stage may run this cycle";
  EXPECT_EQ(mid_handle, mid->handle_phase_calls) << "not even one that already had fresh state";
  EXPECT_EQ(leaf_handle, leaf->handle_phase_calls);
  EXPECT_DOUBLE_EQ(leaf_command, leaf->command_interface_value())
    << "the actuator keeps the previous cycle's value";

  // The containment is per cycle, not sticky: clearing the failure resumes normal operation.
  leaf->set_fail_update(false);
  cm_->read(TIME, PERIOD);
  EXPECT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  cm_->write(TIME, PERIOD);
  EXPECT_EQ(root_handle + 1, root->handle_phase_calls);
  EXPECT_EQ(leaf_handle + 1, leaf->handle_phase_calls);
}

/// A two-phase member is never executed by the native single-pass loop -- not even while a switch is
/// pending and the two passes are paused.
///
/// Gating the native-loop skip on "the passes are running" instead would silently revert such a
/// controller to the fused single-pass semantics for the few cycles a switch takes. The staged group
/// already behaves this way (`owns()` is checked unconditionally), so this makes the two opt-in
/// paths consistent and preserves the "one controller, one execution path" rule at all times.
TEST_F(TestTwoPhaseExecution, a_member_is_never_run_by_the_native_loop_during_a_switch)
{
  auto first = std::make_shared<TestStagedController>();
  auto second = std::make_shared<TestStagedController>();
  const std::string kSecond = "tp_second";
  // Distinct hardware ports: two controllers cannot claim the same command interface.
  const std::vector<std::pair<std::shared_ptr<TestStagedController>, std::pair<std::string, std::string>>>
    members{{first, {std::string(kLeaf), "joint2/velocity"}},
            {second, {kSecond, "joint3/velocity"}}};
  for (const auto & member : members)
  {
    member.first->set_reference_interface_names({"target"});
    member.first->set_actuator_ports({member.second.second});
    member.first->set_command_interface_configuration(individual({member.second.second}));
    member.first->set_state_interface_configuration(individual({}));
    member.first->set_two_phase_legacy(false);
    ASSERT_NE(nullptr, cm_->add_controller(member.first, member.second.first, kType));
    ASSERT_EQ(Return::OK, cm_->configure_controller(member.second.first));
  }
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  SwitchNow({kLeaf}, {});

  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  const int legacy_before = first->legacy_update_calls;
  const int phase_before = first->update_phase_calls;

  // Activate the second member. `switch_controller` sets `do_switch`, so the two-phase passes are
  // paused for every cycle until the switch is applied; those are exactly the cycles in which the
  // native loop could pick the first member up if the skip were gated on `run_two_phase`.
  int cycles_driven = 0;
  auto future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
    std::vector<std::string>{kSecond}, std::vector<std::string>{}, STRICT, true,
    rclcpp::Duration(0, 0));
  for (int i = 0;
       i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cm_->read(TIME, PERIOD);
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    ++cycles_driven;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(5)));
  ASSERT_EQ(Return::OK, future.get());

  EXPECT_EQ(legacy_before, first->legacy_update_calls)
    << "the native loop must never pick up a two-phase member";
  EXPECT_LT(first->update_phase_calls, phase_before + cycles_driven)
    << "at least one cycle must have been paused, otherwise this test proves nothing";

  // Normal operation resumes afterwards, through the passes only.
  const int phase_after_switch = first->update_phase_calls;
  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  EXPECT_EQ(phase_after_switch + 1, first->update_phase_calls);
  EXPECT_EQ(legacy_before, first->legacy_update_calls);
}

}  // namespace
