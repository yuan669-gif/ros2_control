// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__STAGED_EXECUTION_GROUP_HPP_
#define CONTROLLER_MANAGER__STAGED_EXECUTION_GROUP_HPP_

// Compatibility shim. The canonical kernel now lives in the standalone `hierarchical_control`
// package so that it can be hosted either by the controller manager or by a single controller.
#include "hierarchical_control/staged_execution_group.hpp"

namespace controller_manager
{
using hierarchical_control::StagedExecutionGroup;
using hierarchical_control::StagedGroupMember;
using hierarchical_control::StagedResult;
using hierarchical_control::StagedStatus;
}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__STAGED_EXECUTION_GROUP_HPP_
