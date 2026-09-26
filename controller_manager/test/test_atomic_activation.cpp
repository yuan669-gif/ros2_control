// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// All-or-nothing activation (our extension) versus upstream's best-effort activation.
//
// Upstream activates a request SET controller by controller: one that cannot be activated is
// skipped, the others stay active. Its own tests rely on that (a spawner starting controllers one at
// a time must not stop the ones already running when a later one fails).
//
// A scheduled tree has a stronger requirement: a PARTIAL tree is not a tree. The parent-before-child
// order the execution plan guarantees does not hold for a subset, and the hardware sees commands
// from half a cascade. `set_atomic_activation(true)` makes a switch that activates several
// controllers undo the ones IT activated when any of them fails.
//
// These tests pin BOTH behaviours: the default is asserted to stay exactly as upstream has it, so
// enabling the extension is a visible decision and not a silent change of semantics.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Return = controller_interface::return_type;

/// A minimal controller whose claims and activation outcome the test controls.
class SwitchableController : public controller_interface::ControllerInterface
{
public:
  SwitchableController() = default;

  void set_claims(
    std::vector<std::string> command_interfaces, std::vector<std::string> state_interfaces)
  {
    command_interfaces_ = std::move(command_interfaces);
    state_interfaces_ = std::move(state_interfaces);
  }

  /// Make `on_activate` fail, i.e. a lifecycle-level failure rather than an interface conflict.
  void set_fail_activate(bool fail) {fail_activate_ = fail;}

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = command_interfaces_;
    return cfg;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = state_interfaces_;
    return cfg;
  }

  CallbackReturn on_init() override {return CallbackReturn::SUCCESS;}
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (fail_activate_) {return CallbackReturn::FAILURE;}
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  Return update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return Return::OK;
  }

private:
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;
  bool fail_activate_ = false;
};

class TestAtomicActivation : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  /// Run a call that replaces the controller list WHILE driving the control loop.
  /**
   * `add_controller()`/`configure_controller()` publish a new controller list and wait in
   * `RTControllerListWrapper::wait_until_rt_not_using()` for the real-time loop to release the list
   * being replaced. In a running system `update()` does that within microseconds; a test that pumps
   * the loop by hand must keep pumping, exactly as `Switch()` does below (found by measurement: the
   * call otherwise blocks forever, with the main thread in `hrtimer_nanosleep`).
   */
  template <typename Callable>
  auto RunWithPump(Callable && call)
  {
    auto future = std::async(std::launch::async, std::forward<Callable>(call));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    return future.get();
  }

  std::shared_ptr<SwitchableController> AddController(
    const std::string & name, std::vector<std::string> command_interfaces,
    std::vector<std::string> state_interfaces = {})
  {
    auto controller = std::make_shared<SwitchableController>();
    controller->set_claims(std::move(command_interfaces), std::move(state_interfaces));
    controllers_.push_back(controller);
    RunWithPump([&]() {cm_->add_controller(controller, name, "switchable");});
    EXPECT_EQ(Return::OK, RunWithPump([&]() {return cm_->configure_controller(name);}));
    return controller;
  }

  /// `true` when the switch was applied; the outcome is asserted by the caller.
  Return Switch(const std::vector<std::string> & start, const std::vector<std::string> & stop = {})
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

  static bool IsActive(const std::shared_ptr<SwitchableController> & controller)
  {
    return controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  std::vector<std::shared_ptr<SwitchableController>> controllers_;
};
}  // namespace

/// DEFAULT (upstream semantics): the controllers that could be activated stay active.
///
/// This is the behaviour upstream's own spawner test depends on, so it is asserted rather than
/// assumed -- the extension below must be a decision, not a change of meaning.
TEST_F(TestAtomicActivation, by_default_a_partial_activation_is_kept)
{
  EXPECT_FALSE(cm_->atomic_activation());
  auto good = AddController("good", {"joint2/velocity"});
  auto bad = AddController("bad", {"joint2/velocity"});  // same port: the second must fail

  EXPECT_EQ(Return::OK, Switch({"good"}));
  ASSERT_TRUE(IsActive(good));

  // `bad` cannot claim joint2/velocity (already claimed by the active `good`).
  EXPECT_EQ(Return::ERROR, Switch({"bad"}));
  EXPECT_TRUE(IsActive(good)) << "upstream semantics: an unrelated active controller is untouched";
  EXPECT_FALSE(IsActive(bad));
}

/// ATOMIC: the same request undoes the controllers this switch activated, so the caller sees
/// "nothing activated" instead of half a tree.
TEST_F(TestAtomicActivation, atomic_activation_undoes_what_the_failed_switch_activated)
{
  cm_->set_atomic_activation(true);
  EXPECT_TRUE(cm_->atomic_activation());

  // `first` and `second` are fine on their own, `conflict` needs a port that is already taken.
  auto first = AddController("first", {"joint2/velocity"});
  auto second = AddController("second", {"joint3/velocity"});
  auto conflict = AddController("conflict", {"joint1/position"});

  // Take joint1/position with an already-active controller, so the third member of the next switch
  // cannot be activated.
  auto blocker = AddController("blocker", {"joint1/position"});
  EXPECT_EQ(Return::OK, Switch({"blocker"}));
  ASSERT_TRUE(IsActive(blocker));

  // One switch, three controllers, the last one impossible.
  EXPECT_EQ(Return::ERROR, Switch({"first", "second", "conflict"}));

  EXPECT_FALSE(IsActive(first)) << "atomic activation must undo the controllers it activated";
  EXPECT_FALSE(IsActive(second)) << "atomic activation must undo the controllers it activated";
  EXPECT_FALSE(IsActive(conflict));
  EXPECT_TRUE(IsActive(blocker)) << "a controller that was already active keeps running";
}

/// The rollback releases the interfaces it had claimed, so the ports are usable again afterwards --
/// otherwise "undone" would only mean "deactivated but still holding the hardware".
TEST_F(TestAtomicActivation, the_rollback_releases_the_interfaces_it_claimed)
{
  cm_->set_atomic_activation(true);
  auto first = AddController("first", {"joint2/velocity"});
  auto conflict = AddController("conflict", {"joint1/position"});
  auto blocker = AddController("blocker", {"joint1/position"});
  ASSERT_EQ(Return::OK, Switch({"blocker"}));

  ASSERT_EQ(Return::ERROR, Switch({"first", "conflict"}));
  ASSERT_FALSE(IsActive(first));

  // `first`'s port must be claimable again: activate a fresh controller that needs exactly it.
  auto reuser = AddController("reuser", {"joint2/velocity"});
  EXPECT_EQ(Return::OK, Switch({"reuser"}))
    << "the rollback must have released joint2/velocity";
  EXPECT_TRUE(IsActive(reuser));
}

/// A lifecycle-level failure (not an interface conflict) takes the same path.
TEST_F(TestAtomicActivation, a_lifecycle_activation_failure_also_rolls_back)
{
  cm_->set_atomic_activation(true);
  auto first = AddController("first", {"joint2/velocity"});
  auto broken = AddController("broken", {"joint3/velocity"});
  broken->set_fail_activate(true);

  EXPECT_EQ(Return::ERROR, Switch({"first", "broken"}));
  EXPECT_FALSE(IsActive(first));
  EXPECT_FALSE(IsActive(broken));

  // The switch really did try: deactivating the failing controller's claim happened, so the port is
  // free for a later, working controller.
  broken->set_fail_activate(false);
  EXPECT_EQ(Return::OK, Switch({"broken"}));
  EXPECT_TRUE(IsActive(broken));
}

/// Atomic activation must not change the successful case.
TEST_F(TestAtomicActivation, a_successful_switch_is_unaffected)
{
  cm_->set_atomic_activation(true);
  auto first = AddController("first", {"joint2/velocity"});
  auto second = AddController("second", {"joint3/velocity"});

  EXPECT_EQ(Return::OK, Switch({"first", "second"}));
  EXPECT_TRUE(IsActive(first));
  EXPECT_TRUE(IsActive(second));
}
