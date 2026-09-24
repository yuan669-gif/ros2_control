// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_STAGED_CONTROLLER__TEST_STAGED_CONTROLLER_HPP_
#define TEST_STAGED_CONTROLLER__TEST_STAGED_CONTROLLER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/two_phase_controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "hardware_interface/loaned_command_interface.hpp"

namespace test_staged_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Test controller implementing both the native chainable path and the opt-in staged path.
/**
 * Synthetic, deterministic algorithm shared by every comparison implementation:
 *
 *     state   : leaves publish their bound hardware state; composites publish 2 * sum(children)
 *     command : command = reference - state; child references and actuator all receive `command`
 *
 * The controller deliberately exposes counters, last observed frames and the committed command so a
 * test can prove phase ordering, single invocation per phase, cycle identity and commit atomicity.
 */
class TestStagedController : public controller_interface::ChainableControllerInterface,
                             public hierarchical_control::StagedControllerInterface,
                             public hierarchical_control::TwoPhaseControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TestStagedController();

  CONTROLLER_MANAGER_PUBLIC
  ~TestStagedController() override;

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

  CONTROLLER_MANAGER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  // ---- StagedControllerInterface ------------------------------------------------------------
  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> staged_state_ports() const override;

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> staged_reference_ports() const override;

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> staged_actuator_ports() const override;

  CONTROLLER_MANAGER_PUBLIC
  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override;

  CONTROLLER_MANAGER_PUBLIC
  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update_state_stage(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const hierarchical_control::StagedContext & context,
    const hierarchical_control::StagedInputView & children,
    hierarchical_control::StagedValueWriter state) noexcept override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update_command_stage(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const hierarchical_control::StagedContext & context,
    const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView & reference,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override;

  // ---- manager-level two-phase experiment ---------------------------------------------------
  /// Make the native `update()` entry point run `update_phase()` then `handle_phase()`, so the
  /// same controller can be compared under single-pass and two-pass manager execution.
  CONTROLLER_MANAGER_PUBLIC
  void set_two_phase_legacy(bool enabled);

  /// Child whose estimate this node consumes in the two-phase cascade experiment.
  CONTROLLER_MANAGER_PUBLIC
  void set_two_phase_child(TestStagedController * child);

  /// Hardware stand-in for a leaf's own input.
  CONTROLLER_MANAGER_PUBLIC
  void set_two_phase_input(double value);

  CONTROLLER_MANAGER_PUBLIC
  double two_phase_estimate() const;

  CONTROLLER_MANAGER_PUBLIC
  double two_phase_command() const;

  controller_interface::return_type update_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;

  controller_interface::return_type handle_phase(
    const rclcpp::Time & time, const rclcpp::Duration & period) noexcept override;

  int update_phase_calls = 0;
  int handle_phase_calls = 0;

  // ---- test configuration -------------------------------------------------------------------
  CONTROLLER_MANAGER_PUBLIC
  void set_command_interface_configuration(
    const controller_interface::InterfaceConfiguration & cfg);

  CONTROLLER_MANAGER_PUBLIC
  void set_state_interface_configuration(const controller_interface::InterfaceConfiguration & cfg);

  /// Reference interface suffixes exported by this node (a parent claims `name/suffix`).
  CONTROLLER_MANAGER_PUBLIC
  void set_reference_interface_names(const std::vector<std::string> & names);

  /// Hardware command interface names this node commits into (leaves only).
  CONTROLLER_MANAGER_PUBLIC
  void set_actuator_ports(const std::vector<std::string> & names);

  /// Value returned to the root through `staged_reference_source()`.
  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_state(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_command(bool value);

  /// Make the manager-level two-phase `handle_phase()` fail, without touching the staged contract's
  /// `update_command_stage()` or the native path. Used to measure what the two-phase path does when
  /// a command stage fails halfway through the command pass.
  CONTROLLER_MANAGER_PUBLIC
  void set_fail_handle(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_fail_commit(bool value);

  CONTROLLER_MANAGER_PUBLIC
  void set_emit_nan(bool value);

  /// Added to every leaf state port so numeric propagation can be checked with mock hardware.
  CONTROLLER_MANAGER_PUBLIC
  void set_state_offset(double value);

  /// Enable/disable per-cycle diagnostic recording. Disabled when measuring the real-time path's
  /// own allocation behaviour, because the diagnostics themselves use std::vector.
  CONTROLLER_MANAGER_PUBLIC
  void set_record_diagnostics(bool enabled);

  /// Run the native single-phase `ChainableControllerInterface::update()` path instead of the
  /// staged contract. The node derives `factor * (sum(bound hardware state) + bias)` from the raw
  /// snapshot it reads itself (leaf 1, module 2, root 4 for the three-layer comparison).
  CONTROLLER_MANAGER_PUBLIC
  void set_native_mode(bool enabled, double factor = 1.0, double bias = 0.0);

  /// Counter of committed values observed by the sink (all cycles, including failed commits).
  CONTROLLER_MANAGER_PUBLIC
  std::size_t commit_calls() const;

  CONTROLLER_MANAGER_PUBLIC
  double committed_value() const;

  /// Number of real (ResourceManager-owned) command interfaces bound to this controller.
  CONTROLLER_MANAGER_PUBLIC
  std::size_t command_interface_count() const;

  /// Number of real (ResourceManager-owned) state interfaces bound to this controller.
  CONTROLLER_MANAGER_PUBLIC
  std::size_t state_interface_count() const;

  /// Value currently stored in the first real command interface (hardware-side buffer).
  CONTROLLER_MANAGER_PUBLIC
  double command_interface_value() const;

  int state_calls = 0;
  int command_calls = 0;
  int source_calls = 0;
  /// Incremented by the native `ChainableControllerInterface::update()` path; must stay 0 when the
  /// controller is executed by the staged group, proving it is never called twice.
  int legacy_update_calls = 0;
  /// Native-path diagnostics: command writes actually performed and command-stage failures.
  int native_command_writes = 0;
  int native_command_failures = 0;
  std::uint64_t last_state_cycle = 0;
  std::uint64_t last_command_cycle = 0;
  std::uint64_t last_source_cycle = 0;
  std::vector<double> last_state;
  std::vector<double> last_reference;
  std::vector<double> last_actuator;
  /// Global sequence number shared by every test controller, written in each phase callback.
  int * sequence = nullptr;
  int sequence_at_state = 0;
  int sequence_at_command = 0;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  class Sink;
  class Source;

  controller_interface::InterfaceConfiguration cmd_iface_cfg_;
  controller_interface::InterfaceConfiguration state_iface_cfg_;
  std::vector<std::string> reference_interface_names_;
  std::vector<std::string> actuator_ports_;
  double external_reference_ = 0.0;
  bool fail_state_ = false;
  bool fail_command_ = false;
  bool fail_handle_ = false;
  bool fail_commit_ = false;
  bool emit_nan_ = false;
  double state_offset_ = 0.0;
  bool native_mode_ = false;
  double native_factor_ = 1.0;
  double native_bias_ = 0.0;
  bool record_diagnostics_ = true;
  bool two_phase_legacy_ = false;
  TestStagedController * two_phase_child_ = nullptr;
  double two_phase_input_ = 0.0;
  double two_phase_estimate_ = 0.0;
  double two_phase_command_ = 0.0;
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<Source> source_;
  std::size_t commit_calls_ = 0;
  std::vector<double> committed_;
};

}  // namespace test_staged_controller

#endif  // TEST_STAGED_CONTROLLER__TEST_STAGED_CONTROLLER_HPP_
