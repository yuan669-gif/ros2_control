// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TWO_PHASE_EXAMPLE_CONTROLLER__TWO_PHASE_EXAMPLE_CONTROLLER_HPP_
#define TWO_PHASE_EXAMPLE_CONTROLLER__TWO_PHASE_EXAMPLE_CONTROLLER_HPP_

#include <atomic>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/two_phase_controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace two_phase_example_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// A minimal CASCADE controller that can run under either execution path of the same manager.
/**
 * The controller is deliberately as small as a useful cascade node, but it exposes the one thing the
 * two-phase contract is about: a BIDIRECTIONAL dependency between a parent and its children.
 *
 *   state direction (children -> parent)
 *     Each node publishes its estimate on its own `estimate` reference interface. A node with
 *     children consumes theirs in `update_phase`, which the manager walks BACKWARD, so a child has
 *     already published when its parent ingests. A leaf takes its estimate from a hardware state
 *     interface instead.
 *
 *   reference direction (parent -> children)
 *     Each node writes its `target` reference interface into every child's `target` command
 *     interface in `handle_phase`, which the manager walks FORWARD. A leaf turns the target it
 *     receives into a hardware command.
 *
 * The algorithm is an exact integer-friendly filter, chosen so a test can tell the two execution
 * paths apart by VALUE alone:
 *
 *     leaf   : estimate = hardware state
 *     parent : estimate = 0.5 * previous estimate + 0.5 * mean(children's estimates)
 *     command: target - estimate      (written to the hardware and to every child)
 *
 * A step of 1.0 on the leaf's hardware state therefore reaches the parent as 0.0 (legacy, single
 * pass: the parent ran before the child) or 0.125 after three levels (two-phase, same cycle).
 *
 * HUMBLE NOTE. `ChainableControllerInterface` in Humble can only export COMMAND interfaces, so the
 * child-to-parent state channel is realised by having each node write the value of its own exported
 * `estimate` reference interface and the parent READ the matching loaned command interface. On Jazzy
 * and later the same channel would be an exported STATE interface
 * (`on_export_state_interfaces`), which is the upstream-blessed form. The scheduling question -- who
 * runs first -- is identical either way.
 *
 * WIRING. `command_interfaces`, `state_interfaces` and `children` are node parameters (also settable
 * programmatically before `configure()`), so the same plugin serves YAML deployment and tests.
 */
class TwoPhaseExampleController : public controller_interface::ChainableControllerInterface,
                                  public controller_interface::TwoPhaseControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TwoPhaseExampleController();

  CONTROLLER_MANAGER_PUBLIC
  ~TwoPhaseExampleController() override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_init() override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ---- TwoPhaseControllerInterface ----------------------------------------------------------

  /// Ingest this cycle's state: the hardware state for a leaf, the children's estimates otherwise.
  /// Publishes the estimate on this node's `estimate` reference interface, so a parent that runs
  /// later in the SAME backward pass reads a value from this cycle.
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;

  /// Turn the target received from the parent (or the external reference, for the root) into a
  /// command, write it to the hardware and forward it to every child.
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;

  // ---- wiring / observability ---------------------------------------------------------------

  /// Hardware command interfaces this node writes (empty for a non-leaf in a pure cascade).
  CONTROLLER_MANAGER_PUBLIC
  void set_command_interface_names(std::vector<std::string> names);

  /// Hardware state interfaces this node reads (used only when `children` is empty).
  CONTROLLER_MANAGER_PUBLIC
  void set_state_interface_names(std::vector<std::string> names);

  /// Child controller names; each contributes `<child>/target` (write) and `<child>/estimate` (read).
  CONTROLLER_MANAGER_PUBLIC
  void set_children(std::vector<std::string> children);

  CONTROLLER_MANAGER_PUBLIC
  double estimate() const noexcept;

  CONTROLLER_MANAGER_PUBLIC
  double command() const noexcept;

  CONTROLLER_MANAGER_PUBLIC
  std::int64_t update_phase_calls() const noexcept;

  CONTROLLER_MANAGER_PUBLIC
  std::int64_t handle_phase_calls() const noexcept;

  CONTROLLER_MANAGER_PUBLIC
  std::int64_t native_update_calls() const noexcept;

  /// The period the manager passed to the last stage call. A rate bucket must receive the BUCKET's
  /// period, not the manager's.
  CONTROLLER_MANAGER_PUBLIC
  std::int64_t last_period_ns() const noexcept;

  /// The external reference of a ROOT (what a subscriber would provide in a real controller).
  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value) noexcept;

  /// Hardware stand-in for a leaf's input, so a test can drive a step without a real joint.
  CONTROLLER_MANAGER_PUBLIC
  void set_hardware_state(double value) noexcept;

  /// Make the NEXT `update_phase` fail once, to exercise the containment rule.
  CONTROLLER_MANAGER_PUBLIC
  void fail_next_update_phase() noexcept;

  /// Test hook: while held, `update_phase` spins so a test can keep the control loop inside a cycle
  /// and observe `ControllerManager::control_loop_busy()`. Never used by the demo configuration.
  CONTROLLER_MANAGER_PUBLIC
  void hold_update_phase(bool hold) noexcept;

  /// True once `update_phase` has been entered at least once (the hold hook is armed before this).
  CONTROLLER_MANAGER_PUBLIC
  bool update_phase_entered() const noexcept;

  /// Test instrument: when ON, a node writes the SUPPLIED cycle stamp instead of the control law
  /// (into its children's targets and its actuators) and publishes it on its `estimate` channel. A
  /// consumer records the producer's stamp, so a test can read the AGE of the value it consumed.
  ///
  /// The stamp is supplied by the TEST rather than derived from a per-node counter, because a node
  /// only advances on the cycles its own bucket is due: a factor-1 node and a factor-2 node would
  /// otherwise be counting different clocks and their difference would be meaningless. One global
  /// stamp is the common time base the age is measured on.
  ///
  /// That age is exactly the quantity `doc/CROSS_RATE_BOUND.md` predicts, and reading it is the only
  /// way to compare the bound against the REAL controller manager rather than against a model of it.
  /// The control law is still computed, so nothing else about the controller changes.
  CONTROLLER_MANAGER_PUBLIC
  void set_cycle_stamp_mode(bool enabled) noexcept;

  /// Set the stamp this node writes/publishes while cycle-stamp mode is on. Called by the test before
  /// each `update()`; must be the SAME value for every node of the configuration.
  CONTROLLER_MANAGER_PUBLIC
  void set_cycle_stamp(std::int64_t stamp) noexcept;

  /// The value of this node's `target` reference the last time it computed a command: in cycle-stamp
  /// mode, the stamp the parent had when it last wrote.
  CONTROLLER_MANAGER_PUBLIC
  double last_target_seen() const noexcept;

  /// The mean of the children's `estimate` channel the last time this node ingested state: in
  /// cycle-stamp mode, the stamp the child had when it last published.
  CONTROLLER_MANAGER_PUBLIC
  double last_child_estimate_seen() const noexcept;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  /// The FUSED single-pass body, used when two-phase execution is disabled. It performs exactly the
  /// two steps the two passes perform, but inside one call, so the comparison is like for like.
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Refresh the reference from our own input topic (see the interface's documentation for why this
  /// step must exist in the two-phase contract at all).
  controller_interface::return_type refresh_reference_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Ingest state and publish the estimate. Shared by `update_phase` and the fused path.
  void ingest() noexcept;
  /// Optional deployment instrument: publish `[cycle, estimate, child_published]` once per cycle.
  void publish_cycle_diagnostics() noexcept;
  /// Compute the command from the reference and the estimate, then write it. Shared likewise.
  void compute_and_write() noexcept;

  /// Index of the loaned interface with this exact full name, or `npos`.
  static constexpr std::size_t npos = static_cast<std::size_t>(-1);
  std::size_t find_command(const std::string & name) const;
  std::size_t find_state(const std::string & name) const;

  /// Resolve the configured names into indices once, at activation (never in the real-time path).
  bool resolve_interfaces(std::string * reason);

  // Reference interface slots (exported as `<node>/target` and `<node>/estimate`).
  static constexpr std::size_t k_target = 0;
  static constexpr std::size_t k_estimate = 1;
  static constexpr std::size_t k_reference_count = 2;

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::vector<std::string> children_;

  std::vector<std::size_t> actuator_index_;
  std::vector<std::size_t> state_index_;
  std::vector<std::size_t> child_target_index_;
  std::vector<std::size_t> child_estimate_index_;

  double estimate_ = 0.0;
  double command_ = 0.0;
  double hardware_state_override_ = 0.0;
  bool use_hardware_state_override_ = false;
  bool fail_next_update_phase_ = false;
  std::atomic<bool> hold_update_phase_{false};
  std::atomic<bool> update_phase_entered_{false};

  /// Measurement instrument (see set_cycle_stamp_mode).
  bool cycle_stamp_mode_ = false;
  std::int64_t cycle_ = 0;
  std::int64_t cycle_stamp_ = 0;
  double last_target_seen_ = 0.0;
  double last_child_estimate_seen_ = 0.0;

  std::int64_t update_phase_calls_ = 0;
  std::int64_t handle_phase_calls_ = 0;
  std::int64_t native_update_calls_ = 0;
  std::int64_t last_period_ns_ = 0;

  /// Deployment instrument, OFF unless the `publish_cycle_diagnostics` parameter is set. It exists
  /// so a real deployment (a simulator or hardware, not a hand-pumped test) can be measured from
  /// outside: one message per cycle, carrying the cycle index and this node's estimate, so an
  /// observer can reconstruct WHO saw WHAT on WHICH cycle after the fact.
  bool publish_diagnostics_ = false;
  /// Latest value received on `~/reference`; a chain ROOT is driven from here.
  std::atomic<double> subscribed_reference_{0.0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_publisher_;
  std_msgs::msg::Float64MultiArray diagnostics_message_;
};
}  // namespace two_phase_example_controller

#endif  // TWO_PHASE_EXAMPLE_CONTROLLER__TWO_PHASE_EXAMPLE_CONTROLLER_HPP_
