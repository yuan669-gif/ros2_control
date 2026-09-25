// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_
#define TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "hierarchical_control/staged_execution_group.hpp"
#include "hierarchical_control/topology_binding.hpp"
#include "hierarchical_control/typed_ports.hpp"

namespace test_composite_library
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// A composite plugin that hosts a BRANCHING tree declared ONCE at compile time.
/**
 * The difference from `GenericCompositeController` is where the topology comes from. That host is
 * data-driven: a list of `CompositeNodeSpec` is read at runtime, the parent of each node is a string,
 * and the kernel's port lists are hand-written strings. Here one type-level declaration states the
 * tree, the ports and their physical dimensions:
 *
 *     fork_root -> { fork_a, fork_b }        (the same two-leaf fork the fair comparison uses)
 *
 * `typed_fork` below is that declaration. From it:
 *   * `compose` checks the parent/child edges AT COMPILE TIME -- every child declares the parent it
 *     is nested under, and the parent's per-child reference/state lists must equal the concatenation,
 *     in child order, of what its children declare (name, order and dimension);
 *   * `TypedPortsMixin` GENERATES the port strings the kernel sizes its buffers from, so the
 *     declaration and the runtime strings cannot drift;
 *   * `topology_binding::create_library_group` (the checked entry) verifies the runtime strings
 *     against the declaration, builds the plan and returns a runnable `StagedExecutionGroup`.
 *
 * Everything else is deliberately identical to the data-driven host -- same algorithm, same
 * interface mapping, same kernel -- so the two hosts can be compared on the same input.
 */
class TypedForkCompositeController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TypedForkCompositeController();

  CONTROLLER_MANAGER_PUBLIC
  ~TypedForkCompositeController() override;

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

  /// The root's external reference, i.e. what the root's reference source returns.
  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value);

  /// What the kernel actually committed to leaf `index` (0 = fork_a, 1 = fork_b).
  CONTROLLER_MANAGER_PUBLIC
  double command_interface_value(std::size_t leaf_index) const;

  /// The node names the checked plan was built from, in plan order (root first).
  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> plan_node_names() const;

  /// Per-node counters, indexed like `plan_node_names()`.
  CONTROLLER_MANAGER_PUBLIC
  int state_calls(std::size_t node) const;

  CONTROLLER_MANAGER_PUBLIC
  int command_calls(std::size_t node) const;

  int update_calls = 0;
  std::size_t commit_calls = 0;
  int build_allocations = 0;

private:
  /// Type-erased handle to one internal node, so the plugin can keep nodes of different port types.
  class Node;
  /// One typed internal node. Nested, so it may use the plugin's private interface plumbing.
  template <typename Ports>
  class TypedNode;
  template <typename Ports>
  class WrappedNode;

  bool build_kernel();
  bool resolve_interface_slots();

  std::vector<std::unique_ptr<Node>> nodes_;
  /// state/command slot indices into the loaned interfaces, per leaf (0 = a, 1 = b).
  std::vector<std::vector<std::size_t>> state_slots_;
  std::vector<std::vector<std::size_t>> command_slots_;
  std::shared_ptr<hierarchical_control::StagedExecutionGroup> kernel_;
  std::vector<std::string> plan_names_;
  std::vector<double> committed_value_;
  double external_reference_ = 0.0;
};

}  // namespace test_composite_library

#endif  // TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_
