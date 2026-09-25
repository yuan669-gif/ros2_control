// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__STAGED_EXECUTION_GROUP_HPP_
#define HIERARCHICAL_CONTROL__STAGED_EXECUTION_GROUP_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/hierarchy.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hierarchical_control
{

/// Outcome of one staged execution group cycle.
enum class StagedStatus
{
  committed,      ///< every stage succeeded and all actuator commands were committed
  inactive,       ///< at least one member is not active; nothing was executed
  invalid_input,  ///< root reference snapshot, sample age or configuration precondition failed
  state_failed,   ///< a state stage failed or a child state was invalid/stale
  command_failed,  ///< a command stage failed, a value was non-finite, or a commit failed
  exhausted,      ///< cycle counter overflow
  not_configured  ///< run() called before a valid plan was installed
};

/// Diagnostic record for one group cycle.
struct StagedResult
{
  StagedStatus status = StagedStatus::not_configured;
  std::uint64_t cycle = 0;
  std::size_t failed_node = 0;
  std::uint32_t fault_code = 0;
};

/// One resolved group member.
/**
 * `command_interfaces` must be the names reported by the controller's
 * `command_interface_configuration()`. A name whose prefix is another member's name defines a
 * command edge: this member is the reference *producer* (parent), the prefix owner is the
 * *consumer* (child). Names whose prefix is not a group member are ordinary hardware command
 * interfaces owned by this member.
 */
struct StagedGroupMember
{
  std::string name;
  controller_interface::ControllerInterfaceBaseSharedPtr controller;
  std::vector<std::string> command_interfaces;
};

/// Opt-in execution group implementing the two-phase same-cycle contract.
/**
 * Cycle order:
 *
 *     update stage : postorder, iterated FORWARD  (children before parents)
 *     reference    : one external snapshot for the root
 *     handle stage : the SAME order iterated BACKWARD (parents before children)
 *     validate     : cycle identity, sample age, finiteness
 *     commit       : leaf scratch copied into controller-owned command sinks
 *
 * All topology resolution, string handling, `dynamic_cast` and storage allocation happen in
 * `create()`. `run()` performs pointer indexing only: no lookups, no construction, no allocation.
 *
 * Failure semantics: a failed cycle never calls any command sink, so committed hardware commands
 * are not a mixture of old and new values. Keeping the previous command is *not* a hardware safety
 * policy; the application must define and invoke its own fault action. A sink that fails during
 * `commit()` is a hardware fault outside this software contract.
 */
class StagedExecutionGroup
{
public:
  static constexpr std::size_t no_node = std::numeric_limits<std::size_t>::max();

  /// A resolved plan that is independent of ControllerManager ownership and lifecycle.
  struct Spec
  {
    std::vector<std::string> names;
    std::vector<StagedControllerInterface *> instances;
    std::vector<std::string> parents;  // empty string for the root
  };

  /// Library mode: build the same kernel directly from resolved node instances.
  /**
   * The returned group has no `ControllerManager` dependency, no native reference interfaces and no
   * lifecycle: it reports itself active immediately so a single controller plugin can host the
   * whole tree. This is the "machinery lives in a library" baseline for the Gate B comparison.
   */
  static std::shared_ptr<StagedExecutionGroup> create_library(
    const Spec & spec, std::int64_t max_age_ns = 0)
  {
    return std::shared_ptr<StagedExecutionGroup>(new StagedExecutionGroup(spec, max_age_ns, false));
  }

  /// Build and validate the plan from manager-managed controllers. Throws on configuration error.
  static std::shared_ptr<StagedExecutionGroup> create(
    const std::vector<StagedGroupMember> & members, std::int64_t max_age_ns = 0)
  {
    if (members.empty())
    {
      throw std::invalid_argument("staged execution group requires at least one member");
    }
    std::unordered_map<std::string, std::size_t> index;
    index.reserve(members.size());
    Spec spec;
    spec.names.reserve(members.size());
    spec.instances.reserve(members.size());
    std::vector<controller_interface::ControllerInterfaceBaseSharedPtr> controllers;
    controllers.reserve(members.size());
    for (std::size_t i = 0; i < members.size(); ++i)
    {
      const auto & member = members[i];
      if (member.name.empty() || !member.controller)
      {
        throw std::invalid_argument("staged group members need a non-empty name and an instance");
      }
      if (!index.emplace(member.name, i).second)
      {
        throw std::invalid_argument("duplicate staged group member: " + member.name);
      }
      auto * staged = dynamic_cast<StagedControllerInterface *>(
        member.controller.get());
      if (staged == nullptr)
      {
        throw std::invalid_argument(
          "controller '" + member.name + "' does not implement StagedControllerInterface");
      }
      spec.names.push_back(member.name);
      spec.instances.push_back(staged);
      controllers.push_back(member.controller);
    }

    // Derive command edges from claimed interfaces. Kept as a pure function so the rule
    // ("one writer per PORT, not per node") is testable on its own; see hierarchy.hpp.
    std::vector<std::vector<std::string>> claimed;
    claimed.reserve(members.size());
    for (const auto & member : members) {claimed.push_back(member.command_interfaces);}
    spec.parents = derive_parents_from_claimed_interfaces(spec.names, claimed);

    auto group = std::shared_ptr<StagedExecutionGroup>(
      new StagedExecutionGroup(spec, max_age_ns, true));
    group->controllers_ = std::move(controllers);
    group->owned_.reserve(group->controllers_.size());
    for (const auto & controller : group->controllers_)
    {
      group->owned_.push_back(controller.get());
    }
    std::sort(group->owned_.begin(), group->owned_.end(), pointer_less_);
    return group;
  }

  std::size_t size() const noexcept {return names_.size();}
  const std::vector<std::string> & member_names() const noexcept {return names_;}

  /// Readable name of a node index, for diagnostics.
  /**
   * `StagedResult::failed_node` is an index, which is not something an operator can act on.
   * Returning the group's own string keeps `run_ns` allocation-free (no name is copied into the
   * result) while the log message can still name the node. Out-of-range indices - including
   * `no_node`, which is what a successful result carries - return a literal instead of throwing.
   */
  const std::string & node_name(std::size_t index) const noexcept
  {
    static const std::string unknown = "<none>";
    return index < names_.size() ? names_[index] : unknown;
  }
  std::int64_t max_age_ns() const noexcept {return max_age_ns_;}
  std::uint64_t committed_cycle() const noexcept {return committed_cycle_;}
  const std::vector<double> & committed_actuators() const noexcept {return committed_;}

  /// True when `controller` is a member. Pointer comparison only; safe for the real-time loop.
  bool owns(const controller_interface::ControllerInterfaceBase * controller) const noexcept
  {
    return std::binary_search(owned_.begin(), owned_.end(), controller, pointer_less_);
  }

  /// Re-read the lifecycle state of every member. Must be called outside the per-cycle path
  /// (after a switch, or after installing the group); `run()` never queries the lifecycle.
  void refresh_member_active_state() noexcept
  {
    members_active_ = true;
    for (const auto & controller : controllers_)
    {
      if (controller->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        members_active_ = false;
        break;
      }
    }
  }

  bool members_active() const noexcept {return members_active_;}

  StagedResult run(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
  {
    time_ = time;
    period_ = period;
    return run_ns(time.nanoseconds(), period.nanoseconds());
  }

  /// Direct integer entry point; uses the ROS time/period of the last `run()` call (zero before).
  StagedResult run_ns(std::int64_t now_ns, std::int64_t period_ns) noexcept
  {
    const auto & time = time_;
    const auto & period = period_;
    if (!configured_) {return {StagedStatus::not_configured, cycle_, no_node};}
    // `members_active_` is refreshed only when a switch has been applied, because Humble's
    // `get_current_state()` allocates inside rclcpp_lifecycle::MutexMap. Keeping that query off
    // the per-cycle path is what makes run() allocation-free after configuration.
    if (!members_active_) {return {StagedStatus::inactive, cycle_, no_node};}
    if (cycle_ == std::numeric_limits<std::uint64_t>::max())
    {
      return {StagedStatus::exhausted, cycle_, no_node};
    }
    if (now_ns < 0 || period_ns <= 0)
    {
      return {StagedStatus::invalid_input, cycle_, no_node};
    }

    const std::uint64_t cycle = ++cycle_;
    const StagedContext context{cycle, now_ns, period_ns};

    // Write-completeness sentinel (review R3). Clearing the FRAMES is not enough: the value
    // buffers are reused across cycles, so a controller that returns OK without writing one of its
    // declared ports would leave the PREVIOUS cycle's value in place, and stamping the frame with
    // the current cycle would present stale data as fresh. Pre-filling with NaN makes every
    // unwritten element fail the finiteness checks that already guard each stage, so "produced
    // nothing" is reported instead of silently accepted.
    constexpr double kUnwritten = std::numeric_limits<double>::quiet_NaN();
    for (auto & frame : state_frames_) {frame = StagedFrame{};}
    for (auto & frame : reference_frames_) {frame = StagedFrame{};}
    for (auto & frame : actuator_frames_) {frame = StagedFrame{};}

    // ------------------------------------------------------------------ state stage (postorder)
    for (const auto node : plan_.postorder)
    {
      const auto & children = children_[node];
      const bool leaf = children.empty();
      std::int64_t oldest = now_ns;
      if (!leaf)
      {
        for (std::size_t slot = 0; slot < children.size(); ++slot)
        {
          const auto child = children[slot];
          const auto & child_frame = state_frames_[child];
          if (!child_frame.valid || child_frame.cycle != cycle ||
            child_frame.sample_ns < 0 || child_frame.sample_ns > now_ns ||
            now_ns - child_frame.sample_ns > max_age_ns_)
          {
            return {StagedStatus::state_failed, cycle, node, child_frame.fault_code};
          }
          oldest = std::min(oldest, child_frame.sample_ns);
          input_views_[node][slot] = StagedValueView(
            state_values_[child].data(), state_values_[child].size(), child_frame);
        }
      }

      auto & frame = state_frames_[node];
      frame = StagedFrame{};
      frame.cycle = cycle;
      frame.sample_ns = leaf ? now_ns : oldest;
      std::fill(state_values_[node].begin(), state_values_[node].end(), kUnwritten);
      StagedValueWriter output(
        state_values_[node].data(), state_values_[node].size(), &frame);
      const auto ret = staged_[node]->update_state_stage(
        time, period, context,
        StagedInputView(input_views_[node].data(), children.size()), output);
      if (ret != controller_interface::return_type::OK)
      {
        return {StagedStatus::state_failed, cycle, node, frame.fault_code};
      }
      if (!leaf)
      {
        // A composite may not re-stamp derived data: the oldest child sample time wins.
        frame.sample_ns = oldest;
      }
      else if (
        frame.sample_ns < 0 || frame.sample_ns > now_ns ||
        now_ns - frame.sample_ns > max_age_ns_)
      {
        return {StagedStatus::invalid_input, cycle, node, frame.fault_code};
      }
      if (!all_finite(state_values_[node]))
      {
        return {StagedStatus::state_failed, cycle, node, frame.fault_code};
      }
      frame.valid = true;
    }

    // ---------------------------------------------------------------- root reference (one shot)
    const auto root = plan_.root;
    {
      auto & frame = reference_frames_[root];
      frame = StagedFrame{};
      frame.cycle = cycle;
      frame.sample_ns = now_ns;
      auto & values = reference_values_[root];
      std::fill(values.begin(), values.end(), kUnwritten);
      if (!values.empty())
      {
        if (sources_[root] == nullptr ||
          !sources_[root]->read(cycle, now_ns, values.data(), values.size()))
        {
          return {StagedStatus::invalid_input, cycle, root, 0};
        }
        if (!all_finite(values)) {return {StagedStatus::invalid_input, cycle, root, 0};}
      }
      frame.valid = true;
    }

    // ---------------------------------------------------------------- command stage (reverse)
    // FineMote device registry: ONE linear order, traversed forward for the "update" phase and
    // backward for the "handle" phase. The reverse of a postorder is a valid
    // parents-before-descendants order, so a single linearization serves both phases. This is why
    // the state direction never needs to constrain the ordering: the reverse pass supplies it.
    for (std::size_t command_slot = plan_.postorder.size(); command_slot-- > 0;)
    {
      const auto node = plan_.postorder[command_slot];
      const auto & state_frame = state_frames_[node];
      const auto & reference_frame = reference_frames_[node];
      if (!state_frame.valid || state_frame.cycle != cycle)
      {
        return {StagedStatus::state_failed, cycle, node, state_frame.fault_code};
      }
      if (!reference_frame.valid || reference_frame.cycle != cycle)
      {
        return {StagedStatus::command_failed, cycle, node, reference_frame.fault_code};
      }

      const auto & children = children_[node];
      for (std::size_t slot = 0; slot < children.size(); ++slot)
      {
        const auto child = children[slot];
        // Sentinel first, so a parent that writes only some of a child's reference ports is
        // detected by the completeness check below (review R3).
        std::fill(
          reference_values_[child].begin(), reference_values_[child].end(), kUnwritten);
        child_writers_[node][slot] = StagedValueWriter(
          reference_values_[child].data(), reference_values_[child].size(),
          &reference_frames_[child]);
      }

      auto & frame = actuator_frames_[node];
      frame = StagedFrame{};
      frame.cycle = cycle;
      frame.sample_ns = now_ns;
      std::fill(actuator_scratch_[node].begin(), actuator_scratch_[node].end(), kUnwritten);
      StagedValueWriter actuator(
        actuator_scratch_[node].data(), actuator_scratch_[node].size(), &frame);

      const auto ret = staged_[node]->update_command_stage(
        time, period, context,
        StagedValueView(
          state_values_[node].data(), state_values_[node].size(), state_frame),
        StagedValueView(
          reference_values_[node].data(), reference_values_[node].size(), reference_frame),
        StagedReferenceWriter(child_writers_[node].data(), children.size()),
        actuator);
      if (ret != controller_interface::return_type::OK)
      {
        return {StagedStatus::command_failed, cycle, node, frame.fault_code};
      }

      for (std::size_t slot = 0; slot < children.size(); ++slot)
      {
        const auto child = children[slot];
        auto & child_frame = reference_frames_[child];
        child_frame.cycle = cycle;
        child_frame.sample_ns = now_ns;
        // Completeness: every declared reference port of the child must have been written by this
        // parent this cycle. An untouched element still holds the NaN sentinel and fails here.
        if (!all_finite(reference_values_[child]))
        {
          child_frame.valid = false;
          return {StagedStatus::command_failed, cycle, node, frame.fault_code};
        }
        child_frame.valid = true;
      }
      if (!all_finite(actuator_scratch_[node]))
      {
        return {StagedStatus::command_failed, cycle, node, frame.fault_code};
      }
      frame.valid = true;
    }

    // ---------------------------------------------------------------------------- commit
    // Two-phase commit over the leaves.
    //
    // Pass 1 performs every sink side effect; pass 2 mirrors into the group's own committed view,
    // and runs ONLY IF pass 1 completed. Before this split, a success on an early leaf updated
    // `actuator_committed_` and `committed_` even when a later leaf failed, leaving the group's
    // internal state describing a cycle that never committed (review R4). With the split, the
    // internal view always matches the last fully committed cycle.
    //
    // Both passes re-iterate the same `leaves_` array, so the split costs no storage: an earlier
    // revision collected the committed leaves in a local `std::vector`, which allocated once per
    // cycle and silently broke the kernel's zero-allocation contract (caught by
    // `test_hierarchy_comparison`). A sink is the controller's adapter to the real command handles,
    // so `sink->commit()` is where a process-global side effect can happen; that cannot be rolled
    // back, and we do not pretend otherwise (see the note below).
    //
    // CONTRACT NOTE: "all-or-nothing" holds for the group's own buffers and for anything the group
    // controls. It does NOT hold for side effects a sink's commit() already performed on
    // process-global state; those are the hardware fault domain and are reported, not undone.
    for (const auto leaf : leaves_)
    {
      auto & scratch = actuator_scratch_[leaf];
      if (scratch.empty()) {continue;}
      if (sinks_[leaf] == nullptr)
      {
        return {StagedStatus::command_failed, cycle, leaf, 0};
      }
      if (!sinks_[leaf]->commit(scratch.data(), scratch.size()))
      {
        return {StagedStatus::command_failed, cycle, leaf, actuator_frames_[leaf].fault_code};
      }
    }

    for (const auto leaf : leaves_)
    {
      const auto & scratch = actuator_scratch_[leaf];
      if (scratch.empty()) {continue;}
      std::copy(scratch.begin(), scratch.end(), actuator_committed_[leaf].begin());
      const auto destination =
        committed_.begin() + static_cast<std::ptrdiff_t>(actuator_offset_[leaf]);
      std::copy(scratch.begin(), scratch.end(), destination);
    }
    committed_cycle_ = cycle;
    return {StagedStatus::committed, cycle, no_node, 0};
  }

private:
  StagedExecutionGroup(const Spec & spec, std::int64_t max_age_ns, bool managed)
  : max_age_ns_(max_age_ns), managed_(managed)
  {
    if (
      spec.names.empty() || spec.names.size() != spec.instances.size() ||
      spec.names.size() != spec.parents.size())
    {
      throw std::invalid_argument("staged execution group spec is empty or inconsistent");
    }

    std::unordered_map<std::string, std::size_t> index;
    index.reserve(spec.names.size());
    // The kernel's central guarantee is "at most one call per stage per controller per cycle". That
    // is a statement about INSTANCES, not names: the same controller object bound under two node
    // names would be advanced twice per phase, and its two storage slots would fight over one
    // object's state. Nothing else rejects it -- the names differ, so every name-based check passes
    // -- so the kernel enforces it where the guarantee lives.
    std::unordered_map<const StagedControllerInterface *, std::string> instance_owner;
    instance_owner.reserve(spec.names.size());
    for (std::size_t i = 0; i < spec.names.size(); ++i)
    {
      if (spec.names[i].empty() || spec.instances[i] == nullptr)
      {
        throw std::invalid_argument("staged group nodes need a non-empty name and an instance");
      }
      if (!index.emplace(spec.names[i], i).second)
      {
        throw std::invalid_argument("duplicate staged group member: " + spec.names[i]);
      }
      const auto inserted = instance_owner.emplace(spec.instances[i], spec.names[i]);
      if (!inserted.second)
      {
        throw std::invalid_argument(
          "the same controller instance is bound to both '" + inserted.first->second + "' and '" +
          spec.names[i] +
          "'; one controller may appear only once in a group, otherwise it would be advanced twice "
          "per stage in the same cycle");
      }
      names_.push_back(spec.names[i]);
      staged_.push_back(spec.instances[i]);
      sinks_.push_back(spec.instances[i]->staged_command_sink());
      sources_.push_back(spec.instances[i]->staged_reference_source());
    }

    children_.resize(names_.size());
    for (std::size_t child = 0; child < names_.size(); ++child)
    {
      if (spec.parents[child].empty()) {continue;}
      const auto parent_it = index.find(spec.parents[child]);
      if (parent_it == index.end())
      {
        throw std::invalid_argument(
          "staged group node '" + names_[child] + "' has unknown parent '" + spec.parents[child] +
          "'");
      }
      children_[parent_it->second].push_back(child);
    }

    std::vector<ControllerHierarchyNode> nodes;
    nodes.reserve(names_.size());
    for (std::size_t i = 0; i < names_.size(); ++i)
    {
      nodes.push_back(ControllerHierarchyNode{names_[i], spec.parents[i]});
    }
    // build_controller_hierarchy validates: single root, no cycle, no unreachable node.
    plan_ = build_controller_hierarchy(nodes);

    // Size all phase storage once; run() only indexes it.
    state_values_.resize(names_.size());
    state_frames_.resize(names_.size());
    reference_values_.resize(names_.size());
    reference_frames_.resize(names_.size());
    actuator_scratch_.resize(names_.size());
    actuator_committed_.resize(names_.size());
    actuator_frames_.resize(names_.size());
    input_views_.resize(names_.size());
    child_writers_.resize(names_.size());
    actuator_offset_.assign(names_.size(), no_node);

    for (std::size_t i = 0; i < names_.size(); ++i)
    {
      state_values_[i].assign(staged_[i]->staged_state_ports().size(), 0.0);
      reference_values_[i].assign(staged_[i]->staged_reference_ports().size(), 0.0);
      actuator_scratch_[i].assign(staged_[i]->staged_actuator_ports().size(), 0.0);
      actuator_committed_[i].assign(actuator_scratch_[i].size(), 0.0);
      input_views_[i].resize(children_[i].size());
      child_writers_[i].resize(children_[i].size());
      if (!actuator_scratch_[i].empty() && sinks_[i] == nullptr)
      {
        throw std::invalid_argument(
          "node '" + names_[i] + "' declares actuator ports but no commit sink");
      }
      if (children_[i].empty())
      {
        actuator_offset_[i] = committed_.size();
        committed_.insert(committed_.end(), actuator_scratch_[i].size(), 0.0);
        leaves_.push_back(i);
      }
    }

    if (!reference_values_[plan_.root].empty() && sources_[plan_.root] == nullptr)
    {
      throw std::invalid_argument(
        "root node '" + names_[plan_.root] + "' declares reference ports but no reference source");
    }

    configured_ = true;
    // Library mode has no lifecycle to wait for; manager mode refreshes this after each switch.
    members_active_ = !managed_;
  }

  static bool all_finite(const std::vector<double> & values) noexcept
  {
    for (const auto value : values)
    {
      if (!std::isfinite(value)) {return false;}
    }
    return true;
  }

  static bool pointer_less_(
    const controller_interface::ControllerInterfaceBase * a,
    const controller_interface::ControllerInterfaceBase * b) noexcept
  {
    return std::less<const controller_interface::ControllerInterfaceBase *>()(a, b);
  }

  std::int64_t max_age_ns_ = 0;
  bool configured_ = false;
  bool members_active_ = false;
  bool managed_ = false;
  std::vector<std::string> names_;
  std::vector<controller_interface::ControllerInterfaceBaseSharedPtr> controllers_;
  std::vector<StagedControllerInterface *> staged_;
  std::vector<StagedCommandSink *> sinks_;
  std::vector<StagedReferenceSource *> sources_;
  std::vector<const controller_interface::ControllerInterfaceBase *> owned_;
  ControllerHierarchyPlan plan_;
  std::vector<std::vector<std::size_t>> children_;
  std::vector<std::size_t> leaves_;
  std::vector<std::size_t> actuator_offset_;
  std::vector<std::vector<double>> state_values_, reference_values_;
  std::vector<std::vector<double>> actuator_scratch_, actuator_committed_;
  std::vector<StagedFrame> state_frames_, reference_frames_, actuator_frames_;
  std::vector<std::vector<StagedValueView>> input_views_;
  std::vector<std::vector<StagedValueWriter>> child_writers_;
  std::vector<double> committed_;
  std::uint64_t cycle_ = 0, committed_cycle_ = 0;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration period_{0, 0};
};

}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__STAGED_EXECUTION_GROUP_HPP_
