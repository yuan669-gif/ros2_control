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

/// `is_controller_active()` lives in the manager's own anonymous namespace, so the test reads the
/// lifecycle state directly, exactly as the other manager tests do.
bool ControllerIsActive(const controller_interface::ControllerInterfaceBase & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

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

  /// Configure a controller while driving the control loop.
  /**
   * `configure_controller()` (and `add_controller()`/`unload_controller()`) replaces the controller
   * list, and `RTControllerListWrapper::switch_updated_list()` waits in `wait_until_rt_not_using()`
   * for the real-time loop to stop using the list being replaced. In a running system `update()` does
   * that within microseconds; a test that pumps the loop by hand must keep pumping, exactly as
   * `SwitchNow()` does, or the call sleeps forever (measured: the main thread sits in
   * `hrtimer_nanosleep`). That is a harness property, not a feature property.
   */
  Return ConfigureWithPump(const std::string & name)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::configure_controller, cm_.get(),
      name);
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    return future.get();
  }

  /// Attempt a switch that must be REFUSED, and return the result. Used to check that a refused
  /// switch did not apply anything: the caller asserts the lifecycle states afterwards.
  Return SwitchExpectingRefusal(
    const std::vector<std::string> & start, const std::vector<std::string> & stop)
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
    EXPECT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    return future.get();
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

/// A chainable controller that does NOT implement `TwoPhaseControllerInterface`: it stands for the
/// legacy mode. It exports one reference interface so a two-phase controller can claim it, which is
/// exactly the cross-mode edge review item E refuses.
class TestLegacyChainableController : public controller_interface::ChainableControllerInterface
{
public:
  TestLegacyChainableController()
  {
    command_interface_configuration_.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interface_configuration_.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  }

  /// Change the declared command interfaces at runtime.
  /**
   * This models how a controller's claims can differ from the snapshot `switch_controller()` caches
   * for active controllers. The realistic instance is a controller declaring `ALL`: that set is
   * resolved against the resource manager each time it is asked, so it grows on its own as soon as
   * another controller imports a reference interface. (The test uses an explicit list instead of
   * `ALL` only because upstream `controller_sorting()` reads `command_interface_configuration().names`
   * alone, so an `ALL` declaration looks like "no command interfaces" to the sort.)
   */
  void set_command_ports(std::vector<std::string> names)
  {
    command_interface_configuration_.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interface_configuration_.names = std::move(names);
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return command_interface_configuration_;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return state_interface_configuration_;
  }

  CallbackReturn on_init() override
  {
    reference_interface_names_ = {"target"};
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    reference_interfaces_.assign(reference_interface_names_.size(), 0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(reference_interface_names_.size());
    for (std::size_t i = 0; i < reference_interface_names_.size(); ++i)
    {
      interfaces.emplace_back(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]);
    }
    return interfaces;
  }

  Return update_reference_from_subscribers() override {return Return::OK;}

  Return update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return Return::OK;
  }

private:
  std::vector<std::string> reference_interface_names_;
  controller_interface::InterfaceConfiguration command_interface_configuration_;
  controller_interface::InterfaceConfiguration state_interface_configuration_;
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

/// Item E, cross-mode edge: a two-phase parent claims a reference interface owned by a LEGACY
/// controller. The two ends would be ordered by different schedules (the two-phase command pass runs
/// after the native loop), so the reference the parent writes reaches the child a cycle late. The
/// enable request is refused, the flag stays false, and the offending edge is exposed by name.
TEST_F(TestExecutionPathAdmission, two_phase_enable_is_refused_for_a_cross_mode_reference_edge)
{
  auto legacy = std::make_shared<TestLegacyChainableController>();
  cm_->add_controller(legacy, kLeaf, kType);
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));

  // Two-phase parent, claiming the legacy child's exported reference interface.
  root_ = std::make_shared<TestStagedController>();
  MakeChainController(root_, kRoot, "command", {std::string(kLeaf) + "/target"}, {}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution()) << "a refused request must not flip the flag";

  const auto rejections = cm_->two_phase_rejected_controllers();
  ASSERT_FALSE(rejections.empty());
  EXPECT_EQ(kRoot, rejections.front().name);
  EXPECT_NE(std::string::npos, rejections.front().reason.find(kLeaf))
    << "the reason must name the other end of the edge: " << rejections.front().reason;
}

/// Item E, late member: the mode is enabled for a conforming chain, and a controller that implements
/// the interface but declares a lower update rate is configured afterwards. It used to be logged and
/// silently left to the native loop, so the configuration looked successful while the controller did
/// not follow the two-phase schedule. The configure now fails, and the rejected member is exposed.
TEST_F(TestExecutionPathAdmission, a_late_low_rate_member_fails_configure_while_two_phase_is_enabled)
{
  ASSERT_GE(cm_->get_update_rate(), 2u);
  BuildChain(0);
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ASSERT_TRUE(cm_->two_phase_execution());

  auto late = std::make_shared<TestStagedController>();
  MakeChainController(late, "tp_late", "target", {}, {"joint2/velocity"}, {"joint2/position"});
  // The parameter must be set AFTER add_controller(): only then does the plugin have a live node.
  late->get_node()->set_parameter({"update_rate", static_cast<int>(cm_->get_update_rate() / 2)});

  EXPECT_EQ(Return::ERROR, cm_->configure_controller("tp_late"));
  const auto rejections = cm_->two_phase_rejected_controllers();
  ASSERT_EQ(1u, rejections.size());
  EXPECT_EQ("tp_late", rejections.front().name);
  EXPECT_NE(std::string::npos, rejections.front().reason.find("update rate"))
    << rejections.front().reason;
  // The conforming chain is untouched: the mode is still on and still admits its three members.
  EXPECT_TRUE(cm_->two_phase_execution());
  EXPECT_EQ(0u, late->update_phase_calls);
}

/// Item E, member deactivated: the two-phase passes must skip an inactive member while the other
/// members keep running. Before this was stated, "member runs every cycle" could have meant "even
/// when it is not active", which would advance a controller the rest of the system considers
/// stopped.
///
/// Two INDEPENDENT members are used, not a parent and its child: upstream ros2_control refuses to
/// deactivate a chained child while an active controller still claims its reference interface (see
/// the test below), so the "deactivated child with a running parent" state cannot be produced at
/// all. What the kernel actually has to guarantee is that it obeys the lifecycle flag it is given.
TEST_F(TestExecutionPathAdmission, a_deactivated_member_stops_running_while_other_members_continue)
{
  auto alpha = std::make_shared<TestStagedController>();
  auto beta = std::make_shared<TestStagedController>();
  MakeChainController(alpha, kRoot, "command", {}, {}, {});
  MakeChainController(beta, kLeaf, "command", {}, {}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ASSERT_TRUE(cm_->two_phase_execution());

  SwitchNow({kLeaf}, {});
  SwitchNow({kRoot}, {});
  Cycle(3);
  const int root_before = alpha->update_phase_calls;
  const int root_handles_before = alpha->handle_phase_calls;
  const int leaf_before = beta->update_phase_calls;
  const int leaf_handles_before = beta->handle_phase_calls;
  ASSERT_GT(leaf_before, 0) << "the member must have run while it was active";
  ASSERT_GT(root_before, 0);

  // Deactivate one member; the other stays active.
  SwitchNow({}, {kLeaf});
  Cycle(3);

  EXPECT_GT(alpha->update_phase_calls, root_before) << "the active member must keep running";
  EXPECT_GT(alpha->handle_phase_calls, root_handles_before);
  EXPECT_EQ(leaf_before, beta->update_phase_calls) << "an inactive member must not advance";
  EXPECT_EQ(leaf_handles_before, beta->handle_phase_calls);
}

/// Item E, the case the review named cannot be reached: upstream `switch_controller()` refuses to
/// deactivate a chained child while an active controller still claims the child's reference
/// interface, so "member deactivated, parent still running" is not a state the manager can enter for
/// a real parent-child edge. Recording it as a test keeps the claim falsifiable: if upstream ever
/// starts allowing the switch, this test fails and the kernel-level skip above becomes the case that
/// matters for edges too.
TEST_F(TestExecutionPathAdmission, a_chained_child_cannot_be_deactivated_while_its_parent_runs)
{
  BuildChain(0);
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  SwitchNow({kLeaf}, {});
  SwitchNow({kMid}, {});
  SwitchNow({kRoot}, {});
  Cycle(3);

  const int leaf_before = leaf_->update_phase_calls;
  auto future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
    std::vector<std::string>{}, std::vector<std::string>{kLeaf}, STRICT, true,
    rclcpp::Duration(0, 0));
  for (int i = 0; i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;
       ++i)
  {
    cm_->update(TIME, PERIOD);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
  EXPECT_EQ(Return::ERROR, future.get()) << "upstream must refuse to orphan the mid level";

  // The child is still active, so it still runs through the passes: nothing changed.
  Cycle(3);
  EXPECT_GT(leaf_->update_phase_calls, leaf_before);
}

/// Review B at the MANAGER level: one BRANCHING tree (root -> {left, right} -> {a, b} each, seven
/// controllers) driven by the real `ControllerManager::update()` two-phase passes. The kernel always
/// supported multi-child nodes; what this pins down is that the manager's own controller ORDERING
/// (`controller_sorting`, which upstream documents as handling branched chains) is a valid
/// parents-before-children linearization, so walking it FORWARD for `handle_phase` and BACKWARD for
/// `update_phase` really does order every edge of a tree correctly.
///
/// The numeric assertion is exact and only holds if every level ingested its children's value from
/// the SAME cycle: with `estimate = 0.5*estimate + 0.5*mean(children)`, a step of 1.0 at all four
/// leaves in cycle N gives leaves 0.5, the two modules 0.25 and the root 0.125 at the END of cycle N
/// (a single parents-first pass would leave the root at 0). `two_phase_command()` is then
/// `0 - estimate`, so it also proves the command pass ran AFTER the state pass in that cycle.
TEST_F(TestExecutionPathAdmission, two_pass_runs_a_branching_tree_and_propagates_it_same_cycle)
{
  constexpr char kRootName[] = "bt_root";
  constexpr char kLeft[] = "bt_left";
  constexpr char kRight[] = "bt_right";
  constexpr char kLA[] = "bt_la";
  constexpr char kLB[] = "bt_lb";
  constexpr char kRA[] = "bt_ra";
  constexpr char kRB[] = "bt_rb";

  auto root = std::make_shared<TestStagedController>();
  auto left = std::make_shared<TestStagedController>();
  auto right = std::make_shared<TestStagedController>();
  auto la = std::make_shared<TestStagedController>();
  auto lb = std::make_shared<TestStagedController>();
  auto ra = std::make_shared<TestStagedController>();
  auto rb = std::make_shared<TestStagedController>();

  // Leaves: one exported reference so the parent can claim it, and one REAL hardware command
  // interface each. The command interface is not decoration: upstream `controller_sorting()` places
  // a chainable controller that claims NOTHING before its own parent, which would invert both passes
  // for that edge (the admission check below refuses such a configuration, and a separate test pins
  // that down).
  struct LeafSpec
  {
    std::shared_ptr<TestStagedController> controller;
    const char * name;
    const char * hardware_port;
  };
  const std::vector<LeafSpec> leaves = {
    {la, kLA, "joint2/velocity"},
    {lb, kLB, "joint3/velocity"},
    {ra, kRA, "joint1/position"},
    {rb, kRB, "joint1/max_velocity"}};
  for (const auto & leaf : leaves)
  {
    MakeChainController(leaf.controller, leaf.name, "target", {}, {leaf.hardware_port}, {});
    leaf.controller->set_two_phase_input(0.0);
  }
  // Modules: claim the two children's reference interfaces, in declaration order.
  MakeChainController(left, kLeft, "target", {std::string(kLA) + "/target", std::string(kLB) + "/target"}, {}, {});
  MakeChainController(right, kRight, "target", {std::string(kRA) + "/target", std::string(kRB) + "/target"}, {}, {});
  // Root: claims both modules.
  MakeChainController(
    root, kRootName, "command", {std::string(kLeft) + "/target", std::string(kRight) + "/target"}, {}, {});

  left->set_two_phase_children({la.get(), lb.get()});
  right->set_two_phase_children({ra.get(), rb.get()});
  root->set_two_phase_children({left.get(), right.get()});

  // Children first, so a parent's reference interfaces exist when it is configured.
  for (const auto & leaf : leaves) {ASSERT_EQ(Return::OK, cm_->configure_controller(leaf.name));}
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeft));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRight));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRootName));

  for (const auto & child : {la, lb, ra, rb, left, right}) {ASSERT_TRUE(child->set_chained_mode(true));}

  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true)) << "a fully two-phase tree must be admitted";
  ASSERT_TRUE(cm_->two_phase_execution());

  SwitchNow({kLA}, {});
  SwitchNow({kLB}, {});
  SwitchNow({kRA}, {});
  SwitchNow({kRB}, {});
  SwitchNow({kLeft}, {});
  SwitchNow({kRight}, {});
  SwitchNow({kRootName}, {});

  const std::vector<std::shared_ptr<TestStagedController>> all = {root, left, right, la, lb, ra, rb};

  // A steady cycle first, so any cycle-0 artefact is out of the way.
  Cycle(1);
  const std::vector<int> updates_before = {root->update_phase_calls, left->update_phase_calls,
                                           right->update_phase_calls, la->update_phase_calls,
                                           lb->update_phase_calls, ra->update_phase_calls,
                                           rb->update_phase_calls};
  const std::vector<int> handles_before = {root->handle_phase_calls, left->handle_phase_calls,
                                           right->handle_phase_calls, la->handle_phase_calls,
                                           lb->handle_phase_calls, ra->handle_phase_calls,
                                           rb->handle_phase_calls};

  for (const auto & leaf : leaves) {leaf.controller->set_two_phase_input(1.0);}
  Cycle(1);

  // Exactly one of each phase per node per cycle.
  for (std::size_t i = 0; i < all.size(); ++i)
  {
    EXPECT_EQ(1, all[i]->update_phase_calls - updates_before[i])
      << "node " << all[i]->get_node()->get_name() << " ran a wrong number of state stages";
    EXPECT_EQ(1, all[i]->handle_phase_calls - handles_before[i])
      << "node " << all[i]->get_node()->get_name() << " ran a wrong number of command stages";
  }

  // Same-cycle upward propagation, exact: 0.5 -> 0.25 -> 0.125.
  EXPECT_DOUBLE_EQ(0.5, la->two_phase_estimate());
  EXPECT_DOUBLE_EQ(0.5, lb->two_phase_estimate());
  EXPECT_DOUBLE_EQ(0.5, ra->two_phase_estimate());
  EXPECT_DOUBLE_EQ(0.5, rb->two_phase_estimate());
  EXPECT_DOUBLE_EQ(0.25, left->two_phase_estimate()) << "the left module must see both leaves in the same cycle";
  EXPECT_DOUBLE_EQ(0.25, right->two_phase_estimate());
  EXPECT_DOUBLE_EQ(0.125, root->two_phase_estimate()) << "the root must see both branches in the same cycle";

  // ... and the command pass propagated the root's reference down the tree in the SAME cycle:
  // command = reference - estimate, and the reference each level sees is the value its parent just
  // wrote. The root's own external reference is 0, so:
  //   root  = 0        - 0.125 = -0.125
  //   left  = -0.125   - 0.25  = -0.375        (right likewise)
  //   leaf  = -0.375   - 0.5   = -0.875
  // A single parents-first pass cannot produce these values for the deeper levels in one cycle.
  EXPECT_DOUBLE_EQ(-0.125, root->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.375, left->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.375, right->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.875, la->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.875, lb->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.875, ra->two_phase_command());
  EXPECT_DOUBLE_EQ(-0.875, rb->two_phase_command());
}

/// A switch that would put a legacy claimant next to a two-phase member is refused BEFORE it is
/// applied -- the case where the activation itself creates the cross-mode edge.
///
/// The declaration/loan split is what makes this reachable: while the legacy controller is ACTIVE it
/// only holds the interfaces it claimed at activation, so a reference interface that appeared
/// afterwards is NOT an edge (and must not be treated as one). The moment the controller is
/// REACTIVATED it claims the whole declared set, including that interface -- and then the edge is
/// real. Checking the prospective active set before the switch is the only place that can refuse it
/// while nothing has happened: the old code activated the controller, published the new list, and
/// only then reported an error.
TEST_F(TestExecutionPathAdmission, reactivating_a_legacy_claimant_is_refused_before_it_is_applied)
{
  auto member = std::make_shared<TestStagedController>();
  MakeChainController(member, kRoot, "command", {}, {"joint2/velocity"}, {});

  auto legacy = std::make_shared<TestLegacyChainableController>();
  legacy->set_command_ports({"joint2/velocity"});
  cm_->add_controller(legacy, kLeaf, kType);
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));

  // Activate the legacy controller while the member's reference interface does not exist yet: it
  // holds no loan for it, so there is no edge and the mode can be enabled.
  SwitchNow({kLeaf}, {});
  ASSERT_TRUE(ControllerIsActive(*legacy));
  ASSERT_EQ(Return::OK, ConfigureWithPump(kRoot));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true))
    << "no edge exists yet, so the mode must be admitted";

  // Deactivate while the declaration is still the old one: the resource manager refuses to release
  // an interface combination it does not accept, and a controller-reference interface in the
  // deactivate list is not one it accepts.
  SwitchNow({}, {kLeaf});
  ASSERT_FALSE(ControllerIsActive(*legacy));

  // From now on the declaration covers the member's reference interface, so a reactivation WOULD
  // claim it -- and write a reference the member consumes on a different schedule.
  legacy->set_command_ports({"joint2/velocity", std::string(kRoot) + "/command"});

  // Reactivate the legacy claimant TOGETHER with the member. Both are requested on purpose: the
  // manager's own chained-controller validation requires a chain neighbour to be activated in the
  // same switch, so this request would otherwise be applied -- which makes the refusal below
  // attributable to the two-phase pre-flight rather than to upstream's checks.
  const auto result = SwitchExpectingRefusal({kLeaf, kRoot}, {});
  EXPECT_EQ(Return::ERROR, result) << "the switch must be refused";
  EXPECT_FALSE(ControllerIsActive(*legacy))
    << "a refused switch must not activate anything, and must not publish a new list";
  EXPECT_FALSE(ControllerIsActive(*member))
    << "the member must not be activated either";

  const auto rejections = cm_->two_phase_rejected_controllers();
  bool mentions_cross_mode = false;
  for (const auto & rejection : rejections)
  {
    mentions_cross_mode |=
      rejection.reason.find("crosses the two-phase/legacy boundary") != std::string::npos;
  }
  EXPECT_TRUE(mentions_cross_mode) << "the edge must be reported as cross-mode";
}

/// The order validation above is NOT vacuous, and this is the configuration that motivated it: a
/// chainable child that keeps its exported reference but claims NO command interface at all. Upstream
/// `controller_sorting()` sorts such a controller ahead of one that has command interfaces, so the
/// manager's list holds the CHILD before its PARENT. The two-phase passes walk that list backwards
/// and forwards without re-sorting, so both would traverse the edge in the same (wrong) direction and
/// the parent would quietly compute from the previous cycle's child estimate -- exactly the lag the
/// feature exists to remove, while every counter still says "one call per stage per cycle".
///
/// The manager therefore refuses the mode instead of producing wrong numbers.
TEST_F(TestExecutionPathAdmission, two_phase_enable_is_refused_when_the_manager_order_inverts_an_edge)
{
  auto child = std::make_shared<TestStagedController>();
  // No command interface, one exported reference interface: the child is a pure source.
  MakeChainController(child, kLeaf, "target", {}, {}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));

  root_ = std::make_shared<TestStagedController>();
  MakeChainController(
    root_, kRoot, "command", {std::string(kLeaf) + "/target"}, {"joint2/velocity"}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution()) << "a refused request must not flip the flag";

  const auto rejections = cm_->two_phase_rejected_controllers();
  ASSERT_FALSE(rejections.empty());
  bool mentions_order = false;
  for (const auto & rejection : rejections)
  {
    mentions_order |= rejection.reason.find("ordered AFTER its child") != std::string::npos;
  }
  EXPECT_TRUE(mentions_order) << "expected an order rejection, got: " << rejections.front().reason;
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
  // Give the asynchronous request time to be registered BEFORE the first driven cycle. The pause
  // window exists only until the loop applies the switch, so without this head start the very first
  // `update()` can consume it and the assertion below would fail for a scheduling reason rather than
  // for the behaviour it checks. (Adding the two-phase pre-flight lengthened the request path, which
  // is what made this race show up.)
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

/// The same controller OBJECT added under two names. Measured upstream behaviour: `add_controller()`
/// rejects duplicate NAMES only, so the second add succeeds and both specs carry the same pointer; the
/// manager then runs that one object once per name, i.e. twice per stage in one cycle (measured: 6
/// `update_phase` and 6 `handle_phase` calls in 3 cycles), while every per-name counter still looks
/// correct. The staged/library path is protected by the kernel's own instance check; the two-phase
/// path does not go through the kernel, so it must refuse the mode.
TEST_F(TestExecutionPathAdmission, two_phase_enable_is_refused_when_one_object_has_two_names)
{
  auto shared = std::make_shared<TestStagedController>();
  MakeChainController(shared, "dup_a", "target", {}, {"joint2/velocity"}, {});
  ASSERT_NE(nullptr, cm_->add_controller(shared, "dup_b", "two_phase_test"))
    << "upstream accepts a second name for the same instance; the mode must catch it";

  // Both names refer to one object, so configuring either configures both.
  EXPECT_EQ(Return::OK, cm_->configure_controller("dup_a"));
  EXPECT_EQ(Return::OK, cm_->configure_controller("dup_b"));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());

  const auto rejections = cm_->two_phase_rejected_controllers();
  ASSERT_EQ(2u, rejections.size()) << "both names of the shared object must be reported";
  bool mentions_sharing = false;
  for (const auto & rejection : rejections)
  {
    mentions_sharing |= rejection.reason.find("shares ONE controller object") != std::string::npos;
  }
  EXPECT_TRUE(mentions_sharing) << "expected a duplicate-instance rejection, got: "
                                << rejections.front().reason;
}

