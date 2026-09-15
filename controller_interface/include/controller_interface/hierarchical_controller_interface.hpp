// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_INTERFACE__HIERARCHICAL_CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE__HIERARCHICAL_CONTROLLER_INTERFACE_HPP_

#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/visibility_control.h"

namespace controller_interface
{

/// Optional two-phase interface for controllers managed in a hierarchy.
/**
 * Humble's original ControllerInterfaceBase exposes one real-time `update()` method. This mixin
 * lets a new controller participate in FineMote-style bidirectional execution without breaking
 * existing plugins. A hierarchical manager calls `update_state()` in postorder and
 * `update_command()` in preorder. Existing controllers remain valid in the legacy flat mode.
 */
class HierarchicalControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  virtual ~HierarchicalControllerInterface() = default;

  /// Consume child state and publish this node's state/reference data.
  CONTROLLER_INTERFACE_PUBLIC
  virtual return_type update_state(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Consume parent references and publish this node's command data.
  CONTROLLER_INTERFACE_PUBLIC
  virtual return_type update_command(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__HIERARCHICAL_CONTROLLER_INTERFACE_HPP_
