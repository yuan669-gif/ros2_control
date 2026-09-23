// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__STAGED_CONTROLLER_INTERFACE_HPP_
#define HIERARCHICAL_CONTROL__STAGED_CONTROLLER_INTERFACE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hierarchical_control
{

/// Provenance metadata for one node's published state or reference group in one cycle.
/**
 * This record is owned and finalised by the execution group, not by the controller:
 * `cycle`, `sample_ns` and `valid` are written by the group. A node may only report a
 * `fault_code`, and - for a *leaf* state group - an actual hardware `sample_ns` that is older than
 * the cycle. A composite node can never re-stamp derived data as fresher than its inputs, because
 * the group overwrites its `sample_ns` with the oldest child sample time.
 */
struct StagedFrame
{
  /// Execution-group cycle that produced the value. 0 means "never produced".
  std::uint64_t cycle = 0;
  /// Oldest source sample time (nanoseconds, same monotonic/ROS clock as the context).
  std::int64_t sample_ns = 0;
  /// True only after a stage completed successfully in this cycle.
  bool valid = false;
  /// Controller-defined fault identifier; 0 means "no fault reported".
  std::uint32_t fault_code = 0;
};

/// Per-cycle integer context shared by both stages.
/**
 * Mirrors the information the execution group itself uses for validity, so a controller can reason
 * about cycle identity and deadlines without touching wall clocks inside the real-time callback.
 */
struct StagedContext
{
  std::uint64_t cycle = 0;
  std::int64_t now_ns = 0;
  std::int64_t period_ns = 0;
};

/// Read-only view of one value group plus its provenance.
class StagedValueView
{
public:
  StagedValueView() = default;
  StagedValueView(const double * data, std::size_t size, const StagedFrame & frame) noexcept
  : data_(data), size_(size), frame_(&frame)
  {
  }

  std::size_t size() const noexcept {return size_;}
  const double * data() const noexcept {return data_;}
  double operator[](std::size_t index) const noexcept {return data_[index];}
  const StagedFrame & frame() const noexcept {return *frame_;}

private:
  const double * data_ = nullptr;
  std::size_t size_ = 0;
  const StagedFrame * frame_ = nullptr;
};

/// Writable view of one value group plus its provenance slot.
/**
 * The data buffer is group-owned scratch storage, never a live hardware handle. The group copies
 * scratch values into the real command interfaces only after the whole tree has succeeded.
 */
class StagedValueWriter
{
public:
  StagedValueWriter() = default;
  StagedValueWriter(double * data, std::size_t size, StagedFrame * frame) noexcept
  : data_(data), size_(size), frame_(frame)
  {
  }

  std::size_t size() const noexcept {return size_;}
  double * data() const noexcept {return data_;}
  double & operator[](std::size_t index) const noexcept {return data_[index];}

  /// Leaf state only: report the actual hardware sample time.
  /**
   * Ignored for composite nodes; their sample time is the oldest child sample time by contract.
   * Set 0 to mean "sampled at the current cycle".
   */
  void set_source_sample_ns(std::int64_t sample_ns) noexcept
  {
    if (frame_) {frame_->sample_ns = sample_ns;}
  }

  /// Report a controller-defined fault code for this cycle.
  void set_fault(std::uint32_t code) noexcept
  {
    if (frame_) {frame_->fault_code = code;}
  }

private:
  double * data_ = nullptr;
  std::size_t size_ = 0;
  StagedFrame * frame_ = nullptr;
};

/// Borrowed, executor-owned array of child state views in binding order.
/**
 * The group keeps this storage for the lifetime of the plan; building the view performs no
 * allocation and performs no lookup.
 */
class StagedInputView
{
public:
  StagedInputView() = default;
  StagedInputView(const StagedValueView * groups, std::size_t count) noexcept
  : groups_(groups), count_(count)
  {
  }

  std::size_t size() const noexcept {return count_;}
  StagedValueView operator[](std::size_t index) const noexcept {return groups_[index];}

private:
  const StagedValueView * groups_ = nullptr;
  std::size_t count_ = 0;
};

/// Borrowed, executor-owned array of child reference writers in binding order.
class StagedReferenceWriter
{
public:
  StagedReferenceWriter() = default;
  StagedReferenceWriter(StagedValueWriter * groups, std::size_t count) noexcept
  : groups_(groups), count_(count)
  {
  }

  std::size_t size() const noexcept {return count_;}
  StagedValueWriter operator[](std::size_t index) const noexcept {return groups_[index];}

private:
  StagedValueWriter * groups_ = nullptr;
  std::size_t count_ = 0;
};

/// Controller-owned destination for the group's post-validation command commit.
/**
 * The controller adapter owns the real `LoanedCommandInterface` handles (ResourceManager stays the
 * resource owner) and exposes them here. The group calls `commit()` only after every node succeeded
 * and every value passed the final checks. `commit()` must not throw, allocate or block.
 */
class StagedCommandSink
{
public:
  virtual ~StagedCommandSink() = default;
  virtual bool commit(const double * values, std::size_t size) noexcept = 0;
};

/// Controller-owned snapshot source for the root reference of one cycle.
/**
 * The external command snapshot is taken exactly once per cycle, before the command phase, so the
 * root uses the same cycle-stable input for the whole tree.
 */
class StagedReferenceSource
{
public:
  virtual ~StagedReferenceSource() = default;
  virtual bool read(
    std::uint64_t cycle, std::int64_t now_ns, double * values, std::size_t size) noexcept = 0;
};

/// Optional two-phase interface for controllers participating in an opt-in staged execution group.
/**
 * Humble's `ControllerInterfaceBase` exposes a single `update()` entry point. A controller that
 * implements this mixin declares explicit *state* ports (published bottom-up) and *reference* ports
 * (consumed top-down), and is executed by `controller_manager::StagedExecutionGroup` as:
 *
 *     state stage  : postorder, children before parents   (update_state_stage)
 *     command stage: preorder,  parents before children   (update_command_stage)
 *
 * Both stages are invoked at most once per controller per cycle. Controllers that do not implement
 * this interface keep the native `update()` path; a controller must never be executed by both
 * paths in the same cycle.
 */
class StagedControllerInterface
{
public:
  virtual ~StagedControllerInterface() = default;

  /// Names of the state values this node publishes. Order defines the state port indices.
  virtual std::vector<std::string> staged_state_ports() const = 0;

  /// Names of the reference values this node consumes from its parent, in parent-child order.
  virtual std::vector<std::string> staged_reference_ports() const {return {};}

  /// Names of the hardware command values this node writes. Empty for internal nodes.
  virtual std::vector<std::string> staged_actuator_ports() const {return {};}

  /// Destination for the group commit. Internal nodes return `nullptr`.
  virtual StagedCommandSink * staged_command_sink() noexcept {return nullptr;}

  /// External root-reference snapshot provider. Only the root is expected to return a source.
  virtual StagedReferenceSource * staged_reference_source() noexcept {return nullptr;}

  /// Bottom-up state stage: consume child states, publish this node's state.
  /**
   * `children` are the state groups of this node's children in binding order. `state` is this
   * node's own output group. Implementations must be `noexcept`, allocation-free and must not
   * touch live hardware command handles.
   */
  virtual controller_interface::return_type update_state_stage(
    const rclcpp::Time & time, const rclcpp::Duration & period, const StagedContext & context,
    const StagedInputView & children, StagedValueWriter state) noexcept = 0;

  /// Top-down command stage: consume the parent reference, publish child references and actuators.
  /**
   * `state` is this node's own state produced earlier in the same cycle, `reference` is the
   * reference produced by this node's parent (for the root, the external snapshot), `children`
   * receives one reference group per child, and `actuators` receives this node's hardware commands.
   */
  virtual controller_interface::return_type update_command_stage(
    const rclcpp::Time & time, const rclcpp::Duration & period, const StagedContext & context,
    const StagedValueView & state, const StagedValueView & reference,
    const StagedReferenceWriter & children, StagedValueWriter actuators) noexcept = 0;
};

}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__STAGED_CONTROLLER_INTERFACE_HPP_
