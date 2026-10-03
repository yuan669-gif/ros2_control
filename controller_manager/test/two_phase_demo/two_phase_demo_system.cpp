// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

namespace two_phase_demo
{
/// A tiny in-memory system so the two-phase demo runs under `ros2_control_node` with no hardware.
/**
 * Two joints, each with a position command and position+velocity state. `read()` simply reports the
 * last commanded position and a finite-difference velocity; `write()` records the command. It exists
 * only so the demo has a real `ResourceManager` behind it.
 */
class TwoPhaseDemoSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    const auto joints = info.joints.size();
    command_position_.assign(joints, 0.0);
    state_position_.assign(joints, 0.0);
    state_velocity_.assign(joints, 0.0);

    for (const auto & joint : info.joints)
    {
      bool has_position_command = false;
      bool has_position_state = false;
      bool has_velocity_state = false;
      for (const auto & interface : joint.command_interfaces)
      {
        if (interface.name == hardware_interface::HW_IF_POSITION) {has_position_command = true;}
      }
      for (const auto & interface : joint.state_interfaces)
      {
        if (interface.name == hardware_interface::HW_IF_POSITION) {has_position_state = true;}
        if (interface.name == hardware_interface::HW_IF_VELOCITY) {has_velocity_state = true;}
      }
      if (!has_position_command || !has_position_state || !has_velocity_state)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TwoPhaseDemoSystem"),
          "joint '%s' must declare a position command and position+velocity state interfaces",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
      interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
    }
    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    const double dt = period.seconds();
    for (std::size_t i = 0; i < command_position_.size(); ++i)
    {
      state_velocity_[i] = dt > 0.0 ? (command_position_[i] - state_position_[i]) / dt : 0.0;
      state_position_[i] = command_position_[i];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> command_position_;
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
};
}  // namespace two_phase_demo

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(two_phase_demo::TwoPhaseDemoSystem, hardware_interface::SystemInterface)
