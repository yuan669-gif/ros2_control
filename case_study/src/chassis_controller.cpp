// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "case_study/chassis_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace case_study
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_to_pi(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}
}  // namespace

ChassisController::ChassisController() = default;

controller_interface::CallbackReturn ChassisController::on_init()
{
  left_name_ = auto_declare<std::string>("left_wheel_controller", "wheel_left");
  right_name_ = auto_declare<std::string>("right_wheel_controller", "wheel_right");
  radius_ = auto_declare<double>("wheel_radius", 0.1);
  track_ = auto_declare<double>("track", 0.34);
  kx_ = auto_declare<double>("kx", 1.0);
  ky_ = auto_declare<double>("ky", 1.0);
  kth_ = auto_declare<double>("kth", 1.0);
  legacy_ = auto_declare<bool>("two_phase_legacy", true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_configure(
  const rclcpp_lifecycle::State &)
{
  reference_interfaces_.assign(1, 0.0);
  x_ = y_ = th_ = ref_x_ = ref_y_ = ref_th_ = t_ = 0.0;
  prev_left_ = prev_right_ = 0.0;
  used_left_ = used_right_ = 0.0;
  diagnostics_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());

  left_ = TravelRegistry::find(left_name_);
  right_ = TravelRegistry::find(right_name_);
  if (left_ == nullptr || right_ == nullptr)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Wheel controllers not found in the travel registry ('%s', '%s'). Activate the wheels first.",
      left_name_.c_str(), right_name_.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  }
  prev_left_ = left_->travel();
  prev_right_ = right_->travel();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  left_ = nullptr;
  right_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChassisController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {left_name_ + "/target", right_name_ + "/target"};
  return cfg;
}

controller_interface::InterfaceConfiguration ChassisController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = {};
  return cfg;
}

std::vector<hardware_interface::CommandInterface>
ChassisController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    hardware_interface::CommandInterface(get_node()->get_name(), "command", &reference_interfaces_[0]));
  return interfaces;
}

controller_interface::return_type ChassisController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;  // the manoeuvre is scripted internally
}

controller_interface::return_type ChassisController::update_phase(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const double dt = period.seconds();
  ++cycle_;
  if (left_ == nullptr || right_ == nullptr) {return controller_interface::return_type::ERROR;}

  // ---- state edge: consume the wheels' cumulative travel --------------------------------
  const double left_travel = left_->travel();
  const double right_travel = right_->travel();
  // Lag is measured on the SHARED MANAGER CLOCK, not by differencing two independent cycle
  // counters. Every controller in one ControllerManager::update() cycle gets the same `time`, so
  // `our time - the time of the cycle that produced the value we read` is exactly the scheduling
  // lag in manager periods. Differencing per-controller counters instead mixes in the
  // activation-time offset and produced a constant, meaningless -289 cycles in one run (review R9).
  sample_ns_ = time.nanoseconds();
  used_left_sample_ns_ = left_->sample_ns();
  used_right_sample_ns_ = right_->sample_ns();
  lag_left_ns_ = sample_ns_ - used_left_sample_ns_;
  lag_right_ns_ = sample_ns_ - used_right_sample_ns_;
  const auto period_ns = period.nanoseconds();
  if (period_ns > 0)
  {
    lag_left_cycles_ = lag_left_ns_ / period_ns;
    lag_right_cycles_ = lag_right_ns_ / period_ns;
  }
  // Kept for continuity: the raw per-controller counters, which are NOT comparable across
  // controllers because each starts when its own controller starts.
  used_left_cycle_ = left_->cycle();
  used_right_cycle_ = right_->cycle();
  used_left_ = left_travel;
  used_right_ = right_travel;
  const double delta_left = left_travel - prev_left_;
  const double delta_right = right_travel - prev_right_;
  prev_left_ = left_travel;
  prev_right_ = right_travel;

  const double distance = 0.5 * (delta_left + delta_right);
  const double dtheta = (delta_right - delta_left) / track_;
  x_ += distance * std::cos(th_ + 0.5 * dtheta);
  y_ += distance * std::sin(th_ + 0.5 * dtheta);
  th_ = wrap_to_pi(th_ + dtheta);

  // ---- scripted manoeuvre and its ideal pose -------------------------------------------
  t_ += dt;
  v_ff_ = 0.3 * std::min(1.0, t_ / 1.0);  // 1 s ramp: no step at t=0
  w_ff_ = 0.8 * std::sin(2.0 * kPi * 0.5 * t_);
  ref_x_ += v_ff_ * dt * std::cos(ref_th_);
  ref_y_ += v_ff_ * dt * std::sin(ref_th_);
  ref_th_ = wrap_to_pi(ref_th_ + w_ff_ * dt);
  return controller_interface::return_type::OK;
}

controller_interface::return_type ChassisController::handle_phase(
  const rclcpp::Time &, const rclcpp::Duration &) noexcept
{
  // ---- reference edge: pose feedback -> wheel velocity references -----------------------
  const double ex = ref_x_ - x_;
  const double ey = ref_y_ - y_;
  const double eth = wrap_to_pi(ref_th_ - th_);
  const double ex_body = std::cos(th_) * ex + std::sin(th_) * ey;
  const double ey_body = -std::sin(th_) * ex + std::cos(th_) * ey;

  const double v = std::clamp(v_ff_ + kx_ * ex_body, -1.0, 1.0);
  const double w = std::clamp(w_ff_ + kth_ * eth + ky_ * ey_body, -3.0, 3.0);
  const double left_target = (v - w * track_ * 0.5) / radius_;
  const double right_target = (v + w * track_ * 0.5) / radius_;
  if (command_interfaces_.size() >= 2)
  {
    command_interfaces_[0].set_value(left_target);
    command_interfaces_[1].set_value(right_target);
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    x_, y_, th_, ref_x_, ref_y_, ref_th_, ex, ey, eth, used_left_, used_right_, v, w,
    static_cast<double>(cycle_),
    static_cast<double>(used_left_cycle_),
    static_cast<double>(used_right_cycle_),
    static_cast<double>(lag_left_ns_),
    static_cast<double>(lag_right_ns_),
    static_cast<double>(lag_left_cycles_),
    static_cast<double>(lag_right_cycles_)};
  diagnostics_publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

controller_interface::return_type ChassisController::update_and_write_commands(
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
  case_study::ChassisController, controller_interface::ChainableControllerInterface)
