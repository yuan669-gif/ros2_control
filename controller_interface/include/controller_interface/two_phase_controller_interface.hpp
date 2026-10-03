// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_INTERFACE__TWO_PHASE_CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE__TWO_PHASE_CONTROLLER_INTERFACE_HPP_

#include "controller_interface/controller_interface_base.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace controller_interface
{

/// Opt-in contract for executing one controller as two explicit phases instead of one fused `update()`.
/**
 * A hierarchical controller set has a BIDIRECTIONAL dependency: state is aggregated from the leaves
 * towards the root, while decisions are propagated from the root back to the leaves. Because the
 * `ControllerManager` runs every controller exactly once per cycle in one linear order, a
 * parent/child pair that has BOTH a state edge and a reference edge cannot have both directions
 * fresh in the same cycle: whichever controller runs first, the other one reads the previous
 * cycle's value of that edge.
 *
 * This interface splits a controller's cycle into the two phases that a single linear order CAN
 * serve simultaneously, provided the two phases are traversed in opposite directions over the SAME
 * order:
 *
 *     update phase : reverse traversal (children before parents) -> fresh child state
 *     handle phase : forward traversal (parents before children) -> fresh parent reference
 *
 * The manager derives the order from its own controller list and does not ask the controller for
 * it, so a controller implementing this interface stays an ordinary `ControllerInterfaceBase`
 * (or `ChainableControllerInterface`) and keeps its lifecycle, interface claiming and parameters.
 * When two-phase execution is DISABLED, the manager never calls either phase and the controller's
 * native `update()` is used exactly as before.
 *
 * EXECUTION PATH IS EXCLUSIVE. While two-phase execution is enabled, a controller that implements
 * this interface is called through the two passes and is SKIPPED by the manager's native single-pass
 * loop -- also in the cycles in which a controller switch is pending and the passes are paused. A
 * controller is therefore never executed by both paths in the same cycle.
 *
 * FAILURE SEMANTICS.
 * There is no cross-controller rollback: `handle_phase` writes straight into the controller's
 * command interfaces, so a failure late in the command pass leaves the earlier writes applied. What
 * is guaranteed is the weakest rule that keeps a controller from acting on data it just rejected:
 *
 *   * if ANY `update_phase` fails in a cycle, NO `handle_phase` runs in that cycle; the command
 *     interfaces keep the previous cycle's values and the manager reports an error.
 *
 * RATE BUCKETS.
 * A controller that declares an `update_rate` below the manager's is run once every
 * `manager_rate / controller_rate` cycles, with a matching period, exactly as the native loop
 * rate-gates it. An edge whose two ends fall into different buckets cannot be ordered by one pair of
 * passes and is refused by the manager's admission check rather than run with a silently varying
 * delay.
 */
class TwoPhaseControllerInterface
{
public:
  virtual ~TwoPhaseControllerInterface() = default;

  /// Ingest the previous cycle's data into this controller's internal state.
  /** Runs while the manager walks its controller list in REVERSE (children before parents). */
  virtual return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;

  /// Compute this cycle's outputs from the state `update_phase` just produced.
  /** Runs while the manager walks its controller list FORWARD (parents before children). */
  virtual return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept = 0;
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__TWO_PHASE_CONTROLLER_INTERFACE_HPP_
