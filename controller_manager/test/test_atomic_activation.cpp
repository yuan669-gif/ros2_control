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
#include "test_chainable_controller/test_chainable_controller.hpp"

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

/// An INSTRUMENT, not a subject: it only reads `joint1/position`.
/**
 * `TestActuatorHardware` -- the mock behind `joint1` in `minimal_robot_urdf` -- adds 1 to
 * `position_state_` in `prepare_command_mode_switch()` and 100 in `perform_command_mode_switch()`,
 * whatever the requested lists are. That state is exported as `joint1/position`, so the value read
 * here is an exact counter of the hardware command-mode switches the manager asked for:
 *
 *     delta = (#prepare calls) + 100 * (#perform calls)
 *
 * The hardware belongs to the manager's private ResourceManager, and this is the only channel a test
 * has to observe it -- which is why the P1-1 rollback is verified through this counter rather than
 * through an accessor that does not exist.
 */
class MockModeObserver : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::NONE;
    return cfg;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = {"joint1/position"};
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
  Return update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return Return::OK;
  }

  /// Live value of the mock's mode counter, or -1 while it is not active (no loan).
  double mode_counter() const
  {
    return state_interfaces_.empty() ? -1.0 : state_interfaces_[0].get_value();
  }
};

/// A chainable controller that can be told to fail `on_activate`.
/**
 * A chainable controller is needed for the chained-mode restart path: a preceding controller writes
 * the reference interfaces of a following one, and activating the preceding controller requires the
 * following one to be in chained mode. `set_chained_mode()` is only allowed while inactive, so
 * upstream stops and restarts an already-ACTIVE following controller for that -- which is what the
 * rollback has to undo correctly.
 */
class SwitchableChainable : public test_chainable_controller::TestChainableController
{
public:
  void set_fail_activate(bool fail) {fail_activate_ = fail;}

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    if (fail_activate_) {return CallbackReturn::FAILURE;}
    return test_chainable_controller::TestChainableController::on_activate(previous_state);
  }

private:
  bool fail_activate_ = false;
};

controller_interface::InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

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

/// P1-1: the rollback must also switch the HARDWARE command mode back.
/**
 * Deactivating a controller and releasing its loan is not the whole undo. The switch had already
 * switched the hardware INTO the interfaces of the controllers it activated; a rollback that stopped
 * at the lifecycle left the hardware configured for controllers that no longer run. The mode counter
 * of the mock behind `joint1` makes that difference measurable:
 *
 *   * one prepare (joint1/position += 1) and one perform (+= 100) per requested switch;
 *   * the failed switch itself asks for one pair (+101);
 *   * `joint1/position` is already claimed by `blocker`, so for `conflict` no pair is emitted by the
 *     failure path -- the rollback of `first` must emit the second pair (+101).
 *
 * Baseline is taken AFTER the blocker is active, so it contains everything the test set-up did.
 */
TEST_F(TestAtomicActivation, the_rollback_switches_the_hardware_mode_back)
{
  cm_->set_atomic_activation(true);

  // The instrument. It claims no command interface, so it can never be the reason a switch fails.
  auto observer = std::make_shared<MockModeObserver>();
  ASSERT_NE(
    nullptr,
    RunWithPump([&]() {return cm_->add_controller(observer, "observer", "mock_mode_observer");}));
  ASSERT_EQ(Return::OK, RunWithPump([&]() {return cm_->configure_controller("observer");}));
  ASSERT_EQ(Return::OK, Switch({"observer"}));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, observer->get_state().id());

  auto first = AddController("first", {"joint2/velocity"});
  auto conflict = AddController("conflict", {"joint1/position"});
  auto blocker = AddController("blocker", {"joint1/position"});
  ASSERT_EQ(Return::OK, Switch({"blocker"}));

  const double baseline = observer->mode_counter();
  ASSERT_GT(baseline, 0.0) << "the instrument must be reading the mock's state";

  ASSERT_EQ(Return::ERROR, Switch({"first", "conflict"}));
  ASSERT_FALSE(IsActive(first));
  ASSERT_FALSE(IsActive(conflict));

  EXPECT_EQ(baseline + 202.0, observer->mode_counter())
    << "one pair for the failed switch (101) plus the rollback's pair for first's interface (101); "
       "without the hardware part of the rollback this is only +101";
}

/// A restart for a chained-mode change must be undone as a RESTART, not as a deactivation.
/**
 * Upstream restarts an already-ACTIVE controller when a switch has to change its chained mode
 * (`set_chained_mode()` is only allowed while inactive), putting it into the deactivate AND the
 * activate request. If another controller of the same switch then fails, a rollback that simply
 * deactivates "everything this pass activated" stops a controller that was running before the switch
 * and had nothing to do with the failure -- a failed switch silently became a successful
 * deactivation. The pre-switch snapshot is what lets the rollback put it back.
 */
TEST_F(TestAtomicActivation, a_restart_for_chained_mode_is_brought_back_by_the_rollback)
{
  cm_->set_atomic_activation(true);

  // `child` writes the hardware and exports `child/target`; `parent` writes that reference interface,
  // which is what makes activating `parent` require `child` to be in chained mode.
  auto child = std::make_shared<SwitchableChainable>();
  child->set_command_interface_configuration(individual({"joint2/velocity"}));
  child->set_state_interface_configuration(individual({"joint2/position"}));
  child->set_reference_interface_names({"target"});
  auto parent = std::make_shared<SwitchableChainable>();
  parent->set_command_interface_configuration(individual({"child/target"}));
  parent->set_state_interface_configuration(individual({}));
  // A chainable controller must export at least one reference interface (the manager refuses one that
  // does not). Nothing consumes this one; only `child/target` matters for the restart below.
  parent->set_reference_interface_names({"command"});
  parent->set_fail_activate(true);

  ASSERT_NE(
    nullptr,
    RunWithPump([&]() {return cm_->add_controller(child, "child", "switchable_chainable");}));
  ASSERT_NE(
    nullptr,
    RunWithPump([&]() {return cm_->add_controller(parent, "parent", "switchable_chainable");}));
  // Following controller first: it exports the reference interface its preceding controller writes.
  ASSERT_EQ(Return::OK, RunWithPump([&]() {return cm_->configure_controller("child");}));
  ASSERT_EQ(Return::OK, RunWithPump([&]() {return cm_->configure_controller("parent");}));

  // `child` runs alone: ACTIVE and not chained to anything.
  ASSERT_EQ(Return::OK, Switch({"child"}));
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, child->get_state().id());
  ASSERT_FALSE(child->is_in_chained_mode());

  // Activating `parent` makes the manager restart `child` for the chained-mode change, and `parent`
  // then fails. The rollback must leave `child` exactly as it was.
  EXPECT_EQ(Return::ERROR, Switch({"parent"}));

  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, child->get_state().id())
    << "a controller that ran before the switch must still run after the failed switch";
  EXPECT_FALSE(child->is_in_chained_mode()) << "its pre-switch chained mode must be restored";
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, parent->get_state().id());

  // It really holds its command interface again: a fresh controller asking for the same port has to
  // be refused, so the rollback re-claimed the interface instead of only flipping the lifecycle.
  auto rival = AddController("rival", {"joint2/velocity"});
  EXPECT_EQ(Return::ERROR, Switch({"rival"}));
  EXPECT_FALSE(IsActive(rival));
  EXPECT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, child->get_state().id());
}
