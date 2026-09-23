// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CASE_STUDY__CHASSIS_CONTROLLER_HPP_
#define CASE_STUDY__CHASSIS_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "case_study/travel_registry.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hierarchical_control/two_phase_controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace case_study
{

/// Root chassis controller: the bidirectional node of the case study.
/**
 * Update phase: dead-reckon the pose from the two wheels' cumulative travel (the state edge) and
 * integrate a scripted reference pose.
 * Handle phase: pose feedback -> wheel velocity references (the reference edge).
 *
 * Because both edges exist between the same parent/child pairs, a single-pass schedule cannot keep
 * both same-cycle (Theorem 1); the two-pass schedule can.
 */
class ChassisController : public controller_interface::ChainableControllerInterface,
                          public hierarchical_control::TwoPhaseControllerInterface
{
public:
  ChassisController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;
  controller_interface::return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string left_name_;
  std::string right_name_;
  double radius_ = 0.1;
  double track_ = 0.34;
  double kx_ = 1.0;
  double ky_ = 1.0;
  double kth_ = 1.0;
  bool legacy_ = true;

  TravelSource * left_ = nullptr;
  TravelSource * right_ = nullptr;

  double prev_left_ = 0.0;
  double prev_right_ = 0.0;
  double used_left_ = 0.0;
  double used_right_ = 0.0;

  double x_ = 0.0, y_ = 0.0, th_ = 0.0;
  double ref_x_ = 0.0, ref_y_ = 0.0, ref_th_ = 0.0;
  double t_ = 0.0;
  double v_ff_ = 0.0, w_ff_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_publisher_;
};

}  // namespace case_study

#endif  // CASE_STUDY__CHASSIS_CONTROLLER_HPP_
