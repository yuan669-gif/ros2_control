// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CASE_STUDY__WHEEL_CONTROLLER_HPP_
#define CASE_STUDY__WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "case_study/travel_registry.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hierarchical_control/two_phase_controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace case_study
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Leaf wheel controller.
/**
 * Update phase (FineMote's Update): integrate the MEASURED joint velocity into a cumulative wheel
 * travel. This is deliberately non-re-derivable: the parent cannot reconstruct it from raw joint
 * feedback, because it depends on this controller's own history.
 * Handle phase (FineMote's Handle): PI velocity tracking, writing the joint velocity command.
 */
class WheelController : public controller_interface::ChainableControllerInterface,
                        public hierarchical_control::TwoPhaseControllerInterface,
                        public TravelSource
{
public:
  WheelController();

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

  double travel() const override {return travel_;}
  std::uint64_t cycle() const override {return cycle_;}
  std::int64_t sample_ns() const override {return sample_ns_;}

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string joint_;
  double radius_ = 0.1;
  double kp_ = 5.0;
  double ki_ = 1.0;
  bool legacy_ = true;
  double travel_ = 0.0;
  std::uint64_t cycle_ = 0;  ///< increments once per update_phase, i.e. once per control cycle
  /// Manager time of the cycle that produced `travel_`; the shared epoch for the lag measurement.
  std::int64_t sample_ns_ = 0;
  double velocity_ = 0.0;
  double filtered_velocity_ = 0.0;  // estimator state: NOT re-derivable by the parent
  double integral_ = 0.0;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr travel_publisher_;
};

}  // namespace case_study

#endif  // CASE_STUDY__WHEEL_CONTROLLER_HPP_
