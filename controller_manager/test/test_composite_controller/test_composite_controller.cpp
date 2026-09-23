// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "test_composite_controller/test_composite_controller.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace test_composite_controller
{

TestCompositeController::TestCompositeController()
{
  cmd_iface_cfg_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_iface_cfg_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
}

controller_interface::InterfaceConfiguration
TestCompositeController::command_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return cmd_iface_cfg_;
  }
  throw std::runtime_error(
    "Can not get command interface configuration until the controller is configured.");
}

controller_interface::InterfaceConfiguration
TestCompositeController::state_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return state_iface_cfg_;
  }
  throw std::runtime_error(
    "Can not get state interface configuration until the controller is configured.");
}

CallbackReturn TestCompositeController::on_init() {return CallbackReturn::SUCCESS;}

CallbackReturn TestCompositeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestCompositeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestCompositeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestCompositeController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestCompositeController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ++update_calls;
  if (fail_state_ || state_interfaces_.empty() || command_interfaces_.empty())
  {
    return controller_interface::return_type::ERROR;
  }

  // Private scratch, fixed size: same "compute everything, then commit" discipline as the group.
  std::array<double, kMaxLevels> s{};
  const double raw = state_interfaces_[0].get_value() + state_offset_;
  s[0] = raw;
  for (std::size_t i = 1; i < levels_ && i < kMaxLevels; ++i) {s[i] = growth_ * s[i - 1];}
  double command = external_reference_;
  for (std::size_t i = levels_; i > 0; --i) {command -= s[i - 1];}
  if (emit_nan_) {command = std::numeric_limits<double>::quiet_NaN();}
  last_scratch = command;

  if (fail_command_ || !std::isfinite(command))
  {
    return controller_interface::return_type::ERROR;
  }
  if (fail_commit_)
  {
    return controller_interface::return_type::ERROR;
  }
  command_interfaces_[0].set_value(command);
  ++commit_calls;
  return controller_interface::return_type::OK;
}

void TestCompositeController::set_command_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  cmd_iface_cfg_ = cfg;
}

void TestCompositeController::set_state_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  state_iface_cfg_ = cfg;
}

void TestCompositeController::set_external_reference(double value) {external_reference_ = value;}

void TestCompositeController::set_state_offset(double value) {state_offset_ = value;}

void TestCompositeController::set_fail_state(bool value) {fail_state_ = value;}

void TestCompositeController::set_fail_command(bool value) {fail_command_ = value;}

void TestCompositeController::set_fail_commit(bool value) {fail_commit_ = value;}

void TestCompositeController::set_emit_nan(bool value) {emit_nan_ = value;}

double TestCompositeController::command_interface_value() const
{
  if (command_interfaces_.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  return command_interfaces_.front().get_value();
}

}  // namespace test_composite_controller
