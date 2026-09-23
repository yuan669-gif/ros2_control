// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "case_study/wheel_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"

namespace case_study
{
WheelController::WheelController() = default;

controller_interface::CallbackReturn WheelController::on_init()
{
  joint_ = auto_declare<std::string>("joint", "left_wheel_joint");
  radius_ = auto_declare<double>("wheel_radius", 0.1);
  kp_ = auto_declare<double>("kp", 5.0);
  ki_ = auto_declare<double>("ki", 1.0);
  legacy_ = auto_declare<bool>("two_phase_legacy", true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelController::on_configure(
  const rclcpp_lifecycle::State &)
{
  reference_interfaces_.assign(1, 0.0);
  travel_ = 0.0;
  velocity_ = 0.0;
  filtered_velocity_ = 0.0;
  integral_ = 0.0;
  travel_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/travel", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelController::on_activate(
  const rclcpp_lifecycle::State &)
{
  TravelRegistry::register_source(get_node()->get_name(), this);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  TravelRegistry::unregister_source(get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  TravelRegistry::unregister_source(get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration WheelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {joint_ + "/velocity"};
  return cfg;
}

controller_interface::InterfaceConfiguration WheelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {joint_ + "/position", joint_ + "/velocity"};
  return cfg;
}

std::vector<hardware_interface::CommandInterface> WheelController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    hardware_interface::CommandInterface(get_node()->get_name(), "target", &reference_interfaces_[0]));
  return interfaces;
}

controller_interface::return_type WheelController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;  // chained: the parent writes the reference
}

controller_interface::return_type WheelController::update_phase(
  const rclcpp::Time &, const rclcpp::Duration & period) noexcept
{
  const double dt = period.seconds();
  velocity_ = state_interfaces_.size() > 1 ? state_interfaces_[1].get_value() : 0.0;
  // A slip/estimator filter at the leaf: its state depends on this controller's own history, so a
  // parent cannot reconstruct `travel_` from the instantaneous joint position (position * radius).
  constexpr double kAlpha = 0.85;
  filtered_velocity_ = kAlpha * filtered_velocity_ + (1.0 - kAlpha) * velocity_;
  travel_ += filtered_velocity_ * radius_ * dt;
  return controller_interface::return_type::OK;
}

controller_interface::return_type WheelController::handle_phase(
  const rclcpp::Time &, const rclcpp::Duration & period) noexcept
{
  const double dt = period.seconds();
  const double target = reference_interfaces_.empty() ? 0.0 : reference_interfaces_[0];
  const double error = target - velocity_;
  integral_ += error * dt;
  // Bounded integrator and bounded output: an unbounded PI makes the Gazebo solver diverge.
  integral_ = std::clamp(integral_, -2.0, 2.0);
  const double command = std::clamp(kp_ * error + ki_ * integral_, -8.0, 8.0);
  if (!command_interfaces_.empty()) {command_interfaces_[0].set_value(command);}

  std_msgs::msg::Float64 msg;
  msg.data = travel_;
  travel_publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

controller_interface::return_type WheelController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (legacy_)
  {
    if (update_phase(time, period) != controller_interface::return_type::OK)
    {
      return controller_interface::return_type::ERROR;
    }
    return handle_phase(time, period);
  }
  return controller_interface::return_type::OK;  // executed by the manager's two passes
}

}  // namespace case_study

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  case_study::WheelController, controller_interface::ChainableControllerInterface)
