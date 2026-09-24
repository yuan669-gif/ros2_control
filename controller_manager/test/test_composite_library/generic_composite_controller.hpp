// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_COMPOSITE_LIBRARY__GENERIC_COMPOSITE_CONTROLLER_HPP_
#define TEST_COMPOSITE_LIBRARY__GENERIC_COMPOSITE_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/staged_execution_group.hpp"
#include "controller_manager/visibility_control.h"

namespace test_composite_library
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Data-driven declaration of one node inside a composite controller.
/**
 * `parent` is empty only for the root. Leaves read `state_interfaces` from hardware and write
 * `command_interfaces`; composites combine their children. The generic formula is
 *
 *     state   = factor * sum(child states)          (composite)
 *     state   = offset + sum(hardware states)       (leaf)
 *     command = reference - state, propagated to every child and to the actuators
 *
 * so one controller class covers any tree of this family. Adding a leaf is one more entry.
 */
struct CompositeNodeSpec
{
  std::string name;
  std::string parent;
  std::vector<std::string> state_interfaces;
  std::vector<std::string> command_interfaces;
  double factor = 1.0;
  double offset = 0.0;
};

/// Gate B baseline: a single ordinary controller plugin that hosts the whole tree and reuses the
/// same `hierarchical_control::StagedExecutionGroup` kernel in library mode.
/**
 * Unlike the manager-integrated path this controller:
 *  - is a plain `ControllerInterface` (not chainable), so it claims only hardware interfaces;
 *  - does not use native reference interfaces and needs no `ControllerManager` change;
 *  - builds the kernel in `on_activate()` (non-real-time), so the control loop never allocates;
 *  - resets it in `on_deactivate()`, so an activation boundary can never reuse stale loans.
 */
class GenericCompositeController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  GenericCompositeController();

  CONTROLLER_MANAGER_PUBLIC
  ~GenericCompositeController() override;

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

  CONTROLLER_MANAGER_PUBLIC
  void set_nodes(std::vector<CompositeNodeSpec> nodes);

  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_state(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_command(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_emit_nan(bool value);

  /// Value currently stored in the given leaf's real command interface (hardware-side buffer).
  CONTROLLER_MANAGER_PUBLIC
  double command_interface_value(std::size_t leaf_index) const;

  int update_calls = 0;
  std::size_t commit_calls = 0;
  int build_allocations = 0;

private:
  class Node;

  bool build_kernel();
  bool is_leaf(std::size_t index) const;

  std::vector<CompositeNodeSpec> specs_;
  std::vector<std::unique_ptr<Node>> nodes_;
  std::vector<std::vector<std::size_t>> state_slots_;
  std::vector<std::vector<std::size_t>> command_slots_;
  std::shared_ptr<hierarchical_control::StagedExecutionGroup> kernel_;
  double external_reference_ = 0.0;
  bool fail_state_ = false;
  bool fail_command_ = false;
  bool emit_nan_ = false;
};

}  // namespace test_composite_library

#endif  // TEST_COMPOSITE_LIBRARY__GENERIC_COMPOSITE_CONTROLLER_HPP_
