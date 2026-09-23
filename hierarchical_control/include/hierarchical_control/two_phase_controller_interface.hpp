// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__TWO_PHASE_CONTROLLER_INTERFACE_HPP_
#define HIERARCHICAL_CONTROL__TWO_PHASE_CONTROLLER_INTERFACE_HPP_

#include "controller_interface/controller_interface_base.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hierarchical_control
{

/// Minimal FineMote port: two explicit phases executed as two list-wide passes.
/**
 * FineMote's device registry uses ONE linear order: it is traversed forward for `Update` and
 * backward for `Handle`. Because the two phases run in opposite directions over the same
 * linearization, a single order satisfies both dependencies at once:
 *
 *     update phase : reverse order (children before parents) -> fresh child state
 *     handle phase : forward order (parents before children) -> fresh parent reference
 *
 * A controller that implements this mixin is executed by these two passes and is skipped by the
 * manager's native single-pass `update()` loop. This is deliberately *lighter* than
 * `StagedExecutionGroup`: no group membership, no port declarations, no per-cycle frames, no group
 * commit. It exists to test whether the two-pass schedule alone resolves the bidirectional
 * dependency that a single-pass schedule cannot.
 *
 * The mixin does not own execution order: the manager supplies it from its sorted controller list.
 */
class TwoPhaseControllerInterface
{
public:
  virtual ~TwoPhaseControllerInterface() = default;

  /// FineMote's `Update`: ingest data and modify internal state.
  /** Runs while iterating the manager's controller list in REVERSE (children before parents). */
  virtual controller_interface::return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;

  /// FineMote's `Handle`: compute this cycle's outputs from the updated state.
  /** Runs while iterating the manager's controller list FORWARD (parents before children). */
  virtual controller_interface::return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;
};

}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__TWO_PHASE_CONTROLLER_INTERFACE_HPP_
