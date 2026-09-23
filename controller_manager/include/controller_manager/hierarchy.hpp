// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__HIERARCHY_HPP_
#define CONTROLLER_MANAGER__HIERARCHY_HPP_

// Compatibility shim. The canonical plan builder now lives in the standalone `hierarchical_control`
// package; this header keeps the historical include path working for the v1 helpers and cycle_tree.
#include "hierarchical_control/hierarchy.hpp"

namespace controller_manager
{
using hierarchical_control::build_controller_hierarchy;
using hierarchical_control::ControllerHierarchyNode;
using hierarchical_control::ControllerHierarchyPlan;
using hierarchical_control::execute_controller_hierarchy;
}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__HIERARCHY_HPP_
