// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__HIERARCHICAL_CONTROLLER_EXECUTOR_HPP_
#define CONTROLLER_MANAGER__HIERARCHICAL_CONTROLLER_EXECUTOR_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/hierarchical_controller_interface.hpp"
#include "controller_manager/hierarchy.hpp"

namespace controller_manager
{

/// Executes a prevalidated hierarchy of already-loaded Humble controllers.
/**
 * This is intentionally a small adapter around the existing ControllerInterfaceBase pointer. It
 * does not own lifecycle objects or hardware interfaces. The manager still owns pluginlib,
 * lifecycle and ResourceManager operations; this class only supplies the real-time dispatch order.
 * The vector position of each controller must match the node index used in the plan.
 */
class HierarchicalControllerExecutor
{
public:
  void set_controllers(
    std::vector<controller_interface::ControllerInterfaceBaseSharedPtr> controllers)
  {
    controllers_ = std::move(controllers);
  }

  void set_plan(ControllerHierarchyPlan plan) { plan_ = std::move(plan); }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    if (plan_.preorder.size() != controllers_.size() ||
        plan_.postorder.size() != controllers_.size())
    {
      return controller_interface::return_type::ERROR;
    }

    auto ret = controller_interface::return_type::OK;
    for (const auto index : plan_.postorder)
    {
      if (index >= controllers_.size() || !controllers_[index])
      {
        return controller_interface::return_type::ERROR;
      }
      auto * hierarchical =
        dynamic_cast<controller_interface::HierarchicalControllerInterface *>(
          controllers_[index].get());
      if (hierarchical)
      {
        if (hierarchical->update_state(time, period) != controller_interface::return_type::OK)
        {
          ret = controller_interface::return_type::ERROR;
        }
      }
      else
      {
        // Legacy Humble controllers have one update() entry point. Execute them once in the
        // postorder phase and do not call them again during command propagation.
        if (controllers_[index]->update(time, period) != controller_interface::return_type::OK)
        {
          ret = controller_interface::return_type::ERROR;
        }
      }
    }

    for (const auto index : plan_.preorder)
    {
      if (index >= controllers_.size() || !controllers_[index])
      {
        return controller_interface::return_type::ERROR;
      }
      auto * hierarchical =
        dynamic_cast<controller_interface::HierarchicalControllerInterface *>(
          controllers_[index].get());
      if (hierarchical &&
          hierarchical->update_command(time, period) != controller_interface::return_type::OK)
      {
        ret = controller_interface::return_type::ERROR;
      }
    }
    return ret;
  }

private:
  std::vector<controller_interface::ControllerInterfaceBaseSharedPtr> controllers_;
  ControllerHierarchyPlan plan_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__HIERARCHICAL_CONTROLLER_EXECUTOR_HPP_
