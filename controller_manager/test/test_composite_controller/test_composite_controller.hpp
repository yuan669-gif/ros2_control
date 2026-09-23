// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_COMPOSITE_CONTROLLER__TEST_COMPOSITE_CONTROLLER_HPP_
#define TEST_COMPOSITE_CONTROLLER__TEST_COMPOSITE_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/visibility_control.h"

namespace test_composite_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Handwritten composite baseline: one ordinary plugin with the same internal two-phase algorithm
/// and the same scratch/commit discipline as the staged execution group.
/**
 * Synthetic algorithm (identical in every comparison implementation):
 *
 *     raw     = bound hardware state + offset
 *     s[0]    = raw ;  s[i] = growth * s[i-1]              (leaf .. root)
 *     c       = reference
 *     for i from root down to leaf: c -= s[i]
 *
 * With `levels = 3`, `growth = 2` and one hardware sample this is `c = reference - 7 * raw`.
 * The controller computes into private scratch, validates finiteness, and only then copies the
 * command into the real command interface, so its failure behaviour is comparable to the group's.
 */
class TestCompositeController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestCompositeController();

  CONTROLLER_MANAGER_PUBLIC
  ~TestCompositeController() override = default;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_init() override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  // ---- test configuration -------------------------------------------------------------------
  CONTROLLER_MANAGER_PUBLIC
  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value);

  CONTROLLER_MANAGER_PUBLIC
  void set_state_offset(double value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_state(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_command(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_commit(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_emit_nan(bool value);

  /// Value currently stored in the first real command interface (hardware-side buffer).
  CONTROLLER_MANAGER_PUBLIC
  double command_interface_value() const;

  int update_calls = 0;
  std::size_t commit_calls = 0;
  double last_scratch = 0.0;

private:
  static constexpr std::size_t kMaxLevels = 8;
  controller_interface::InterfaceConfiguration cmd_iface_cfg_;
  controller_interface::InterfaceConfiguration state_iface_cfg_;
  double external_reference_ = 0.0;
  double state_offset_ = 0.0;
  double growth_ = 2.0;
  std::size_t levels_ = 3;
  bool fail_state_ = false;
  bool fail_command_ = false;
  bool fail_commit_ = false;
  bool emit_nan_ = false;
};

}  // namespace test_composite_controller

#endif  // TEST_COMPOSITE_CONTROLLER__TEST_COMPOSITE_CONTROLLER_HPP_
