// Copyright 2026
// Licensed under the Apache License, Version 2.0.

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
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using Return = controller_interface::return_type;
using TestStagedController = test_staged_controller::TestStagedController;

constexpr char ROOT_NAME[] = "stage_root";
constexpr char MODULE_NAME[] = "stage_module";
constexpr char LEAF_NAME[] = "stage_leaf";
constexpr char CONTROLLER_TYPE[] = "test_staged_controller";

InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

class TestStagedExecutionGroup;

/// Exposes the protected ResourceManager so the test can observe mock hardware read/write status.
class TestableControllerManager : public controller_manager::ControllerManager
{
public:
  friend TestStagedExecutionGroup;
  using controller_manager::ControllerManager::ControllerManager;
};

/// Phase A + B acceptance: three staged Humble controllers, group commit, mock hardware.
class TestStagedExecutionGroup : public ControllerManagerFixture<TestableControllerManager>
{
public:
  void TearDown() override
  {
    if (cm_ && cm_->staged_execution_group())
    {
      cm_->clear_staged_execution_group();
    }
    ControllerManagerFixture::TearDown();
  }

  /// Request a switch and drive real-time cycles until the switch is applied.
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

  void MakeController(
    const std::shared_ptr<TestStagedController> & controller, const std::string & name,
    const std::string & reference_suffix, const std::vector<std::string> & claimed_children,
    const std::vector<std::string> & actuator_ports, const std::vector<std::string> & state_ports,
    double state_offset)
  {
    controller->sequence = &sequence_;
    controller->set_reference_interface_names({reference_suffix});
    std::vector<std::string> claimed = claimed_children;
    for (const auto & port : actuator_ports) {claimed.push_back(port);}
    controller->set_command_interface_configuration(individual(claimed));
    controller->set_state_interface_configuration(individual(state_ports));
    controller->set_actuator_ports(actuator_ports);
    controller->set_state_offset(state_offset);
    cm_->add_controller(controller, name, CONTROLLER_TYPE);
  }

  /// Build and activate root -> module -> leaf, then install the staged group.
  /**
   * `activate_root = false` stops after the two children, which is how the manager really reaches a
   * PARTIALLY active group: members are activated one at a time. Deactivating one instead is not an
   * option -- upstream refuses to deactivate a chained child while its preceding controller stays
   * active (`check_preceeding_controllers_for_deactivate`), which is recorded in
   * `IMPLEMENTATION_GUIDE.md` §12.4 #10.
   */
  void SetupGroup(double state_offset = 0.0, bool activate_root = true)
  {
    sequence_ = 0;
    root_ = std::make_shared<TestStagedController>();
    module_ = std::make_shared<TestStagedController>();
    leaf_ = std::make_shared<TestStagedController>();

    MakeController(
      root_, ROOT_NAME, "command", {std::string(MODULE_NAME) + "/target"}, {}, {}, state_offset);
    MakeController(
      module_, MODULE_NAME, "target", {std::string(LEAF_NAME) + "/target"}, {}, {}, state_offset);
    MakeController(
      leaf_, LEAF_NAME, "target", {}, {"joint2/velocity"}, {"joint2/position"}, state_offset);

    ASSERT_EQ(3u, cm_->get_loaded_controllers().size());

    // Configure following controllers first: their exported reference interfaces are claimed by
    // the preceding controller during the latter's activation.
    ASSERT_EQ(Return::OK, cm_->configure_controller(LEAF_NAME));
    ASSERT_EQ(Return::OK, cm_->configure_controller(MODULE_NAME));
    ASSERT_EQ(Return::OK, cm_->configure_controller(ROOT_NAME));

    // Native Humble chain requirement: following controllers enter chained mode while inactive.
    ASSERT_TRUE(leaf_->set_chained_mode(true));
    ASSERT_TRUE(module_->set_chained_mode(true));

    // P1-2: a staged group may only be installed together with all-or-nothing activation, so a
    // FAILED multi-controller switch can not leave the tree half-activated.
    ASSERT_TRUE(cm_->set_atomic_activation(true));
    ASSERT_EQ(Return::OK, cm_->set_staged_execution_group({ROOT_NAME, MODULE_NAME, LEAF_NAME}));
    ASSERT_NE(nullptr, cm_->staged_execution_group());
    EXPECT_EQ(3u, cm_->staged_execution_group()->size());

    SwitchNow({LEAF_NAME}, {});
    SwitchNow({MODULE_NAME}, {});
    if (activate_root) {SwitchNow({ROOT_NAME}, {});}
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

  /// Mock hardware read/write status: TestSystem reports ERROR for its documented magic commands.
  bool HardwareReadOk() {return cm_->resource_manager_->read(TIME, PERIOD).ok;}
  bool HardwareWriteOk() {return cm_->resource_manager_->write(TIME, PERIOD).ok;}

  std::shared_ptr<TestStagedController> root_, module_, leaf_;
  int sequence_ = 0;
};

TEST_F(TestStagedExecutionGroup, phase_order_single_call_and_same_cycle_propagation)
{
  SetupGroup(3.0);
  root_->set_external_reference(1000.0);

  const int root_state0 = root_->state_calls, root_cmd0 = root_->command_calls;
  const int module_state0 = module_->state_calls, module_cmd0 = module_->command_calls;
  const int leaf_state0 = leaf_->state_calls, leaf_cmd0 = leaf_->command_calls;
  const std::size_t leaf_commit0 = leaf_->commit_calls();

  const int cycles = 5;
  Cycle(cycles);

  // Each stage runs exactly once per controller per cycle.
  EXPECT_EQ(cycles, root_->state_calls - root_state0);
  EXPECT_EQ(cycles, root_->command_calls - root_cmd0);
  EXPECT_EQ(cycles, module_->state_calls - module_state0);
  EXPECT_EQ(cycles, module_->command_calls - module_cmd0);
  EXPECT_EQ(cycles, leaf_->state_calls - leaf_state0);
  EXPECT_EQ(cycles, leaf_->command_calls - leaf_cmd0);
  EXPECT_EQ(static_cast<std::size_t>(cycles), leaf_->commit_calls() - leaf_commit0);

  // The native update() path is never taken for group members.
  EXPECT_EQ(0, root_->legacy_update_calls);
  EXPECT_EQ(0, module_->legacy_update_calls);
  EXPECT_EQ(0, leaf_->legacy_update_calls);

  // State phase is postorder (leaf -> module -> root), command phase preorder (root -> module ->
  // leaf), and every state callback precedes every command callback.
  EXPECT_LT(leaf_->sequence_at_state, module_->sequence_at_state);
  EXPECT_LT(module_->sequence_at_state, root_->sequence_at_state);
  EXPECT_LT(root_->sequence_at_command, module_->sequence_at_command);
  EXPECT_LT(module_->sequence_at_command, leaf_->sequence_at_command);
  EXPECT_LT(root_->sequence_at_state, root_->sequence_at_command);

  // Every node observed the same cycle, and the commit is labelled with that cycle.
  EXPECT_EQ(root_->last_state_cycle, module_->last_state_cycle);
  EXPECT_EQ(module_->last_state_cycle, leaf_->last_state_cycle);
  EXPECT_EQ(root_->last_command_cycle, leaf_->last_command_cycle);
  EXPECT_EQ(cm_->staged_execution_group()->committed_cycle(), leaf_->last_state_cycle);

  // S_leaf = 3, S_module = 2 * 3, S_root = 2 * 6, C = 1000 - 12 - 6 - 3 = 979.
  EXPECT_DOUBLE_EQ(979.0, leaf_->committed_value());
}

TEST_F(TestStagedExecutionGroup, failure_never_partially_commits)
{
  SetupGroup(0.0);
  root_->set_external_reference(50.0);
  Cycle(1);
  EXPECT_DOUBLE_EQ(50.0, leaf_->committed_value());
  std::size_t commits = leaf_->commit_calls();
  double last_good = leaf_->committed_value();

  // 1) State failure at the module: command phase must not start and nothing is committed.
  const int leaf_cmd_before = leaf_->command_calls;
  const int root_cmd_before = root_->command_calls;
  module_->set_fail_state(true);
  root_->set_external_reference(999.0);
  EXPECT_EQ(Return::ERROR, cm_->update(TIME, PERIOD));
  EXPECT_EQ(leaf_cmd_before, leaf_->command_calls);
  EXPECT_EQ(root_cmd_before, root_->command_calls);
  EXPECT_EQ(commits, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(last_good, leaf_->committed_value());
  module_->set_fail_state(false);

  // Recovery in the next cycle.
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  EXPECT_EQ(commits + 1, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(999.0, leaf_->committed_value());
  commits = leaf_->commit_calls();
  last_good = leaf_->committed_value();

  // 2) Command failure at the leaf: the sink is not called at all.
  root_->set_external_reference(1234.0);
  leaf_->set_fail_command(true);
  EXPECT_EQ(Return::ERROR, cm_->update(TIME, PERIOD));
  EXPECT_EQ(commits, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(last_good, leaf_->committed_value());
  leaf_->set_fail_command(false);

  // 3) Non-finite derived reference from the module is rejected before any commit.
  root_->set_external_reference(7.0);
  module_->set_emit_nan(true);
  EXPECT_EQ(Return::ERROR, cm_->update(TIME, PERIOD));
  EXPECT_EQ(commits, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(last_good, leaf_->committed_value());
  module_->set_emit_nan(false);

  // 4) Commit sink failure: the sink is tried but no value reaches the hardware buffer.
  root_->set_external_reference(31.0);
  leaf_->set_fail_commit(true);
  EXPECT_EQ(Return::ERROR, cm_->update(TIME, PERIOD));
  EXPECT_EQ(commits + 1, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(last_good, leaf_->committed_value());
  leaf_->set_fail_commit(false);

  // Recovery commits again, and the committed value is the same-cycle result.
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  EXPECT_EQ(commits + 2, leaf_->commit_calls());
  EXPECT_DOUBLE_EQ(31.0, leaf_->committed_value());
}

TEST_F(TestStagedExecutionGroup, committed_command_reaches_mock_hardware_read)
{
  SetupGroup(0.0);
  ASSERT_EQ(1u, leaf_->command_interface_count());
  EXPECT_TRUE(HardwareReadOk());

  // TestSystem treats 28282828 as a read error: reaching it proves the hardware read path observed
  // the value the staged group committed into the ResourceManager-owned command buffer.
  root_->set_external_reference(28282828.0);
  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  EXPECT_DOUBLE_EQ(28282828.0, leaf_->committed_value());
  EXPECT_DOUBLE_EQ(28282828.0, leaf_->command_interface_value());
  EXPECT_FALSE(HardwareReadOk());
}

TEST_F(TestStagedExecutionGroup, committed_command_reaches_mock_hardware_write)
{
  SetupGroup(0.0);
  ASSERT_EQ(1u, leaf_->command_interface_count());
  EXPECT_TRUE(HardwareWriteOk());

  // TestSystem treats 23232323 as a write error: reaching it proves the hardware write path
  // observed the staged commit.
  root_->set_external_reference(23232323.0);
  cm_->read(TIME, PERIOD);
  ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  EXPECT_DOUBLE_EQ(23232323.0, leaf_->committed_value());
  EXPECT_DOUBLE_EQ(23232323.0, leaf_->command_interface_value());
  EXPECT_FALSE(HardwareWriteOk());
}

TEST_F(TestStagedExecutionGroup, configuration_errors_are_rejected)
{
  SetupGroup(0.0);

  // Unknown member.
  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({"does_not_exist"}));
  // Duplicate member.
  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({ROOT_NAME, ROOT_NAME}));
  // Members must be inactive: SetupGroup already activated them.
  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({ROOT_NAME, MODULE_NAME, LEAF_NAME}));
  // The previously installed group is untouched by the rejected attempts.
  ASSERT_NE(nullptr, cm_->staged_execution_group());
  EXPECT_EQ(3u, cm_->staged_execution_group()->size());
  EXPECT_EQ(Return::OK, cm_->update(TIME, PERIOD));
}

TEST_F(TestStagedExecutionGroup, multiple_reference_writers_are_rejected)
{
  sequence_ = 0;
  auto root_a = std::make_shared<TestStagedController>();
  auto root_b = std::make_shared<TestStagedController>();
  auto module = std::make_shared<TestStagedController>();
  auto leaf = std::make_shared<TestStagedController>();

  MakeController(root_a, "root_a", "command", {std::string(MODULE_NAME) + "/target"}, {}, {}, 0.0);
  MakeController(root_b, "root_b", "command", {std::string(MODULE_NAME) + "/target"}, {}, {}, 0.0);
  MakeController(module, MODULE_NAME, "target", {std::string(LEAF_NAME) + "/target"}, {}, {}, 0.0);
  MakeController(leaf, LEAF_NAME, "target", {}, {"joint2/velocity"}, {"joint2/position"}, 0.0);

  ASSERT_EQ(Return::OK, cm_->configure_controller(LEAF_NAME));
  ASSERT_EQ(Return::OK, cm_->configure_controller(MODULE_NAME));
  ASSERT_EQ(Return::OK, cm_->configure_controller("root_a"));
  ASSERT_EQ(Return::OK, cm_->configure_controller("root_b"));

  // The refusals below must be tested for THEIR reason, so the P1-2 precondition is satisfied.
  ASSERT_TRUE(cm_->set_atomic_activation(true));

  // Both roots claim module/target: two reference writers for one consumer must be rejected.
  EXPECT_EQ(
    Return::ERROR,
    cm_->set_staged_execution_group({"root_a", "root_b", MODULE_NAME, LEAF_NAME}));
  EXPECT_EQ(nullptr, cm_->staged_execution_group());

  // One writer is accepted.
  EXPECT_EQ(
    Return::OK, cm_->set_staged_execution_group({"root_a", MODULE_NAME, LEAF_NAME}));
}

/// P1-2, the behavioural half of the policy: partial membership is INERT, not half-executed.
/**
 * Installation requires atomic activation (tested in `test_two_phase_execution.cpp`), but partial
 * membership itself is legitimate: Humble activates members one at a time, and the switch that
 * activates the last one has not happened yet. What must never happen is that the group executes a
 * SUBSET -- the leaf would receive a command whose cascade stops halfway. The group therefore returns
 * `StagedStatus::inactive`, no member's update() is called and no command is committed, while the
 * members that are still active keep their claims. The switch that produced the partial state reports
 * it, so the state is explicit rather than a silent "ACTIVE but driving nothing".
 */
TEST_F(TestStagedExecutionGroup, a_partial_membership_is_inert)
{
  // Two of three members active: the reachable form of "partial" (see SetupGroup).
  SetupGroup(0.0, false);
  ASSERT_FALSE(cm_->staged_execution_group()->members_active());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, leaf_->get_state().id());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, module_->get_state().id());
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, root_->get_state().id());

  const int leaf_state0 = leaf_->state_calls, module_state0 = module_->state_calls;
  const std::size_t leaf_commit0 = leaf_->commit_calls();

  Cycle(5);

  // Nothing ran: not the active members, and not a half cascade. The native loop must not pick them
  // up either, since a group member is never executed by both paths in the same cycle.
  EXPECT_EQ(leaf_state0, leaf_->state_calls) << "no member may run while the group is incomplete";
  EXPECT_EQ(module_state0, module_->state_calls);
  EXPECT_EQ(leaf_commit0, leaf_->commit_calls()) << "no command may be committed";
  EXPECT_EQ(0, leaf_->legacy_update_calls);
  EXPECT_EQ(0, module_->legacy_update_calls);

  // Completing the group makes it run again, so "inert" is a state and not a shutdown.
  SwitchNow({ROOT_NAME}, {});
  ASSERT_TRUE(cm_->staged_execution_group()->members_active());
  // Counts are taken AFTER the switch: the pump loop that applies it may run one more cycle.
  const int root_state1 = root_->state_calls;
  const int leaf_state1 = leaf_->state_calls;
  const std::size_t leaf_commit1 = leaf_->commit_calls();
  Cycle(3);
  EXPECT_EQ(3, root_->state_calls - root_state1);
  EXPECT_EQ(3, leaf_->state_calls - leaf_state1);
  EXPECT_EQ(3u, leaf_->commit_calls() - leaf_commit1);
}

}  // namespace
