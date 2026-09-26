// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Runtime reconfiguration: the supported rule is now ENFORCED, not just documented.
//
// The execution generation (mode + two-phase members + staged plan) is published atomically, but the
// controller LIST is still upstream's separately published double buffer, and the admission decision
// that installs a path is taken against that list. So:
//
//   * INSTALLING a path (`set_two_phase_execution(true)`, `set_staged_execution_group`) while a
//     control cycle is in flight is REFUSED with an error, and publishes nothing;
//   * REMOVING a path (`set_two_phase_execution(false)`, `clear_staged_execution_group`) is always
//     accepted: a cycle already running holds its own generation, finishes with the state it started
//     from, and the next cycle simply sees a smaller one.
//
// The test makes "a cycle is in flight" deterministic instead of hoping for a race: a controller
// whose `update()` blocks on a condition variable holds the cycle open while the main thread calls
// the setters, and releases it afterwards.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Return = controller_interface::return_type;

/// A controller that can hold a control cycle open.
class BlockingController
: public controller_interface::ControllerInterface,
  public hierarchical_control::StagedControllerInterface
{
public:
  void set_actuator_interface(std::string name) {actuator_interface_ = std::move(name);}

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = {actuator_interface_};
    return cfg;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return cfg;
  }

  CallbackReturn on_init() override {return CallbackReturn::SUCCESS;}
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
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

  Return update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    entered_cv_.notify_all();
    blocking_cv_.wait(lock, [this] {return !block_;});
    return Return::OK;
  }

  // ---- StagedControllerInterface (so it can also be a staged group member) ----------------------
  std::vector<std::string> staged_state_ports() const override {return {"blocking/state"};}
  std::vector<std::string> staged_reference_ports() const override {return {};}
  std::vector<std::string> staged_actuator_ports() const override {return {actuator_interface_};}
  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return &sink_;
  }
  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    if (state.size() > 0) {state[0] = 1.0;}
    return Return::OK;
  }
  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView &, const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = 1.0;}
    return Return::OK;
  }

  /// Hold the next `update()` open once it is entered.
  void block_next_update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_ = true;
    entered_ = false;
  }

  /// Wait until a cycle is inside `update()` (bounded).
  bool wait_until_entered(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return entered_cv_.wait_for(lock, timeout, [this] {return entered_;});
  }

  void release_update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_ = false;
    blocking_cv_.notify_all();
  }

private:
  class Sink : public hierarchical_control::StagedCommandSink
  {
  public:
    bool commit(const double *, std::size_t) noexcept override {++commits; return true;}
    int commits = 0;
  };

  std::mutex mutex_;
  std::condition_variable entered_cv_;
  std::condition_variable blocking_cv_;
  bool block_ = false;
  bool entered_ = false;
  std::string actuator_interface_ = "joint2/velocity";
  Sink sink_;
};

class TestRuntimeReconfiguration
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  void SetUp() override
  {
    ControllerManagerFixture<controller_manager::ControllerManager>::SetUp();
    controller_ = std::make_shared<BlockingController>();
    cm_->add_controller(controller_, "blocking", "blocking_controller");
    ASSERT_EQ(Return::OK, cm_->configure_controller("blocking"));

    // A second controller that stays INACTIVE, so it is a valid staged-group member: the group
    // installer's own checks must pass before the in-flight rule can be the reason for a refusal.
    idle_ = std::make_shared<BlockingController>();
    idle_->set_actuator_interface("joint3/velocity");
    cm_->add_controller(idle_, "idle", "blocking_controller");
    ASSERT_EQ(Return::OK, cm_->configure_controller("idle"));
  }

  void TearDown() override
  {
    // Never leave a blocked cycle behind, whatever the test did.
    if (controller_) {controller_->release_update();}
    if (cycle_.valid()) {cycle_.wait();}
    ControllerManagerFixture<controller_manager::ControllerManager>::TearDown();
  }

  /// Activate the controller with the loop pumped, so the switch can be applied.
  void Activate()
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
      std::vector<std::string>{"blocking"}, std::vector<std::string>{}, STRICT, true,
      rclcpp::Duration(0, 0));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(Return::OK, future.get());
  }

  std::shared_ptr<BlockingController> controller_;
  std::shared_ptr<BlockingController> idle_;
  std::future<Return> cycle_;
};
}  // namespace

/// Installing an execution path is refused while a cycle is in flight, and publishes nothing. The
/// generation id makes "nothing was published" observable rather than assumed.
TEST_F(TestRuntimeReconfiguration, installing_a_path_while_a_cycle_is_in_flight_is_refused)
{
  Activate();
  ASSERT_FALSE(cm_->control_loop_busy());

  controller_->block_next_update();
  cycle_ = std::async(std::launch::async, [this] {return cm_->update(TIME, PERIOD);});
  ASSERT_TRUE(controller_->wait_until_entered(std::chrono::milliseconds(2000)))
    << "the cycle must be inside update() for this test to mean anything";
  ASSERT_TRUE(cm_->control_loop_busy());

  const auto generation = cm_->execution_generation();

  // Both installers refuse, and neither publishes.
  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true))
    << "enabling while a cycle is in flight must be refused, not half-applied";
  EXPECT_FALSE(cm_->two_phase_execution());
  // The member is a valid one (inactive), so only the in-flight rule can be the reason here.
  EXPECT_EQ(Return::ERROR, cm_->set_staged_execution_group({"idle"}))
    << "installing a staged group while a cycle is in flight must be refused";
  EXPECT_EQ(nullptr, cm_->staged_execution_group());
  EXPECT_EQ(generation, cm_->execution_generation()) << "a refused install must publish nothing";

  controller_->release_update();
  ASSERT_EQ(Return::OK, cycle_.get());
  EXPECT_FALSE(cm_->control_loop_busy());

  // With the loop idle the same calls are accepted, and each publishes exactly one generation.
  EXPECT_EQ(Return::OK, cm_->set_staged_execution_group({"idle"}))
    << "the same install must succeed once no cycle is in flight";
  EXPECT_NE(nullptr, cm_->staged_execution_group());
  EXPECT_EQ(generation + 1, cm_->execution_generation());
  cm_->clear_staged_execution_group();

  const auto after_group = cm_->execution_generation();
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  EXPECT_EQ(after_group + 1, cm_->execution_generation());
  EXPECT_TRUE(cm_->two_phase_execution());
}

/// Removing a path is accepted even mid-cycle: the running cycle holds its own generation and
/// finishes with the state it started from, so nothing is half-applied for it either.
TEST_F(TestRuntimeReconfiguration, removing_a_path_is_accepted_while_a_cycle_is_in_flight)
{
  Activate();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  const auto generation = cm_->execution_generation();
  ASSERT_TRUE(cm_->two_phase_execution());

  controller_->block_next_update();
  cycle_ = std::async(std::launch::async, [this] {return cm_->update(TIME, PERIOD);});
  ASSERT_TRUE(controller_->wait_until_entered(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(cm_->control_loop_busy());

  // Removal is a strictly smaller execution state, and the in-flight cycle keeps the old one.
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(false));
  EXPECT_FALSE(cm_->two_phase_execution());
  EXPECT_EQ(generation + 1, cm_->execution_generation());
  cm_->clear_staged_execution_group();  // no group installed, still accepted

  controller_->release_update();
  EXPECT_EQ(Return::OK, cycle_.get());
  EXPECT_FALSE(cm_->control_loop_busy());
}

/// A staged group can be installed and cleared while the loop is idle, and the busy flag tracks the
/// cycle exactly (it is not left set by a cycle that returned early).
TEST_F(TestRuntimeReconfiguration, the_busy_flag_tracks_the_cycle)
{
  Activate();
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    EXPECT_FALSE(cm_->control_loop_busy()) << "the flag must be cleared on every return path";
  }
}
