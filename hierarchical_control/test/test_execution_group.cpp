// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Standalone unit tests for the hierarchical_control kernel.
// They do not include controller_manager and do not create a ControllerManager:
// the kernel is host-agnostic, so plain node stubs are enough.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdio>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hierarchical_control/hierarchy.hpp"
#include "hierarchical_control/staged_execution_group.hpp"

namespace hc = hierarchical_control;

namespace
{

/// Minimal host-agnostic node: no lifecycle, no ROS node, no hardware handle.
class StubNode : public hc::StagedControllerInterface
{
public:
  StubNode(std::string name, double factor, double leaf_value)
  : name_(std::move(name)), factor_(factor), leaf_value_(leaf_value)
  {
  }

  StubNode & as_root()
  {
    is_root_ = true;
    sink_ = std::make_unique<Sink>(this);
    source_ = std::make_unique<Source>(this);
    return *this;
  }

  StubNode & as_leaf(std::size_t actuator_ports = 1)
  {
    is_leaf_ = true;
    actuator_names_.assign(actuator_ports, "actuator");
    sink_ = std::make_unique<Sink>(this);
    return *this;
  }

  StubNode & as_composite()
  {
    is_leaf_ = false;
    return *this;
  }

  // ---- StagedControllerInterface ----
  std::vector<std::string> staged_state_ports() const override {return {"state"};}
  std::vector<std::string> staged_reference_ports() const override {return {"ref"};}
  std::vector<std::string> staged_actuator_ports() const override {return actuator_names_;}

  hc::StagedCommandSink * staged_command_sink() noexcept override
  {
    return is_leaf_ ? sink_.get() : nullptr;
  }

  hc::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return is_root_ ? source_.get() : nullptr;
  }

  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext & context,
    const hc::StagedInputView & children, hc::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    last_state_cycle = context.cycle;
    if (sequence != nullptr) {sequence_at_state = ++(*sequence);}
    if (children.size() > 0) {observed_child_sample_ns = children[0].frame().sample_ns;}
    if (fail_state)
    {
      state.set_fault(0x11u);
      return controller_interface::return_type::ERROR;
    }
    if (is_leaf_)
    {
      for (std::size_t i = 0; i < state.size(); ++i) {state[i] = leaf_value_;}
      if (reported_sample_ns >= 0) {state.set_source_sample_ns(reported_sample_ns);}
    }
    else
    {
      double sum = 0.0;
      for (std::size_t c = 0; c < children.size(); ++c)
      {
        for (std::size_t p = 0; p < children[c].size(); ++p) {sum += children[c][p];}
      }
      for (std::size_t p = 0; p < state.size(); ++p) {state[p] = factor_ * sum;}
      if (overwrite_sample_ns >= 0) {state.set_source_sample_ns(overwrite_sample_ns);}
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext & context,
    const hc::StagedValueView & state, const hc::StagedValueView & reference,
    const hc::StagedReferenceWriter & children, hc::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    last_command_cycle = context.cycle;
    if (sequence != nullptr) {sequence_at_command = ++(*sequence);}
    if (fail_command)
    {
      actuators.set_fault(0x12u);
      return controller_interface::return_type::ERROR;
    }
    const double state_value = state.size() > 0 ? state[0] : 0.0;
    const double reference_in = reference.size() > 0 ? reference[0] : 0.0;
    double command = reference_in - state_value;
    if (emit_nan) {command = std::numeric_limits<double>::quiet_NaN();}
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = command;}
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = command;}
    return controller_interface::return_type::OK;
  }

  int state_calls = 0;
  int command_calls = 0;
  int commit_calls = 0;
  int * sequence = nullptr;
  int sequence_at_state = 0;
  int sequence_at_command = 0;
  std::uint64_t last_state_cycle = 0;
  std::uint64_t last_command_cycle = 0;
  std::int64_t observed_child_sample_ns = -1;
  std::int64_t reported_sample_ns = -1;
  std::int64_t overwrite_sample_ns = -1;
  double reference_value = 0.0;
  std::vector<double> committed;
  bool fail_state = false;
  bool fail_command = false;
  bool fail_commit = false;
  bool emit_nan = false;

private:
  class Sink : public hc::StagedCommandSink
  {
  public:
    explicit Sink(StubNode * owner) : owner_(owner) {}
    bool commit(const double * values, std::size_t size) noexcept override
    {
      ++owner_->commit_calls;
      if (owner_->fail_commit) {return false;}
      owner_->committed.assign(values, values + size);
      return true;
    }

  private:
    StubNode * owner_;
  };

  class Source : public hc::StagedReferenceSource
  {
  public:
    explicit Source(StubNode * owner) : owner_(owner) {}
    bool read(std::uint64_t, std::int64_t, double * values, std::size_t size) noexcept override
    {
      for (std::size_t i = 0; i < size; ++i) {values[i] = owner_->reference_value;}
      return true;
    }

  private:
    StubNode * owner_;
  };

  std::string name_;
  double factor_;
  double leaf_value_;
  bool is_leaf_ = false;
  bool is_root_ = false;
  std::vector<std::string> actuator_names_;
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<Source> source_;
};

/// root -> module -> leaf, all in one library-mode group.
struct Chain
{
  StubNode root{"root", 2.0, 0.0};
  StubNode module{"module", 2.0, 0.0};
  StubNode leaf{"leaf", 1.0, 3.0};
  std::shared_ptr<hc::StagedExecutionGroup> group;

  hc::StagedExecutionGroup::Spec spec()
  {
    return {{"root", "module", "leaf"}, {&root, &module, &leaf}, {"", "root", "module"}};
  }
};

double HandwrittenLeafCommand(double reference)
{
  const double s_leaf = 3.0;
  const double s_module = 2.0 * s_leaf;
  const double s_root = 2.0 * s_module;
  const double c_root = reference - s_root;
  const double c_module = c_root - s_module;
  return c_module - s_leaf;  // = reference - 21
}

}  // namespace

TEST(HierarchicalControlKernel, three_layer_chain_matches_handwritten_baseline)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  for (int cycle = 1; cycle <= 1000; ++cycle)
  {
    const double reference = static_cast<double>(cycle) * 0.5 - 10.0;
    chain.root.reference_value = reference;
    const auto result = chain.group->run_ns(1000, 1);
    ASSERT_EQ(hc::StagedStatus::committed, result.status);
    EXPECT_DOUBLE_EQ(HandwrittenLeafCommand(reference), chain.leaf.committed[0]);
    EXPECT_EQ(static_cast<std::uint64_t>(cycle), chain.group->committed_cycle());
  }
}

TEST(HierarchicalControlKernel, phase_order_and_single_call_per_cycle)
{
  int sequence = 0;
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.root.sequence = chain.module.sequence = chain.leaf.sequence = &sequence;
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  const int cycles = 5;
  for (int i = 0; i < cycles; ++i)
  {
    chain.root.reference_value = 7.0;
    ASSERT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
  }

  EXPECT_EQ(cycles, chain.root.state_calls);
  EXPECT_EQ(cycles, chain.root.command_calls);
  EXPECT_EQ(cycles, chain.module.state_calls);
  EXPECT_EQ(cycles, chain.module.command_calls);
  EXPECT_EQ(cycles, chain.leaf.state_calls);
  EXPECT_EQ(cycles, chain.leaf.command_calls);
  EXPECT_EQ(static_cast<std::size_t>(cycles), static_cast<std::size_t>(chain.leaf.commit_calls));

  // state postorder: leaf -> module -> root ; command preorder: root -> module -> leaf
  EXPECT_LT(chain.leaf.sequence_at_state, chain.module.sequence_at_state);
  EXPECT_LT(chain.module.sequence_at_state, chain.root.sequence_at_state);
  EXPECT_LT(chain.root.sequence_at_command, chain.module.sequence_at_command);
  EXPECT_LT(chain.module.sequence_at_command, chain.leaf.sequence_at_command);
  EXPECT_LT(chain.root.sequence_at_state, chain.root.sequence_at_command);

  // every node observed the same cycle
  EXPECT_EQ(chain.root.last_state_cycle, chain.leaf.last_state_cycle);
  EXPECT_EQ(chain.root.last_command_cycle, chain.leaf.last_command_cycle);
  EXPECT_EQ(chain.group->committed_cycle(), chain.leaf.last_state_cycle);
}

TEST(HierarchicalControlKernel, state_failure_aborts_before_command_phase)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  chain.root.reference_value = 5.0;
  ASSERT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
  const auto previous = chain.leaf.committed;
  const auto previous_cycle = chain.group->committed_cycle();

  chain.module.fail_state = true;
  const int root_commands_before = chain.root.command_calls;
  const auto result = chain.group->run_ns(1000, 1);
  EXPECT_EQ(hc::StagedStatus::state_failed, result.status);
  EXPECT_EQ(1u, result.failed_node);
  EXPECT_EQ(0x11u, result.fault_code);
  EXPECT_EQ(root_commands_before, chain.root.command_calls);
  EXPECT_EQ(previous, chain.leaf.committed);
  EXPECT_EQ(previous_cycle, chain.group->committed_cycle());

  chain.module.fail_state = false;
  EXPECT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
}

TEST(HierarchicalControlKernel, command_failure_and_nan_never_commit)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  chain.root.reference_value = 5.0;
  ASSERT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
  const auto previous = chain.leaf.committed;
  const int commits = chain.leaf.commit_calls;

  chain.leaf.fail_command = true;
  EXPECT_EQ(hc::StagedStatus::command_failed, chain.group->run_ns(1000, 1).status);
  EXPECT_EQ(commits, chain.leaf.commit_calls);
  EXPECT_EQ(previous, chain.leaf.committed);
  chain.leaf.fail_command = false;

  chain.module.emit_nan = true;
  EXPECT_EQ(hc::StagedStatus::command_failed, chain.group->run_ns(1000, 1).status);
  EXPECT_EQ(commits, chain.leaf.commit_calls);
  EXPECT_EQ(previous, chain.leaf.committed);
  chain.module.emit_nan = false;

  chain.leaf.fail_commit = true;
  EXPECT_EQ(hc::StagedStatus::command_failed, chain.group->run_ns(1000, 1).status);
  EXPECT_EQ(commits + 1, chain.leaf.commit_calls);
  EXPECT_EQ(previous, chain.leaf.committed);
  chain.leaf.fail_commit = false;

  EXPECT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
}

TEST(HierarchicalControlKernel, composite_cannot_restamp_derived_sample_time)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.leaf.reported_sample_ns = 400;
  // The module tries to claim a fresher sample time than its input; the kernel must ignore it.
  chain.module.overwrite_sample_ns = 900;
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec(), 1000000);

  ASSERT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
  // module saw the leaf's real source time, and root saw the module's (still the leaf's) time.
  EXPECT_EQ(400, chain.module.observed_child_sample_ns);
  EXPECT_EQ(400, chain.root.observed_child_sample_ns);
}

TEST(HierarchicalControlKernel, sample_age_limit_is_enforced)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec(), 50);
  chain.leaf.reported_sample_ns = 100;

  EXPECT_EQ(hc::StagedStatus::invalid_input, chain.group->run_ns(200, 1).status);
  chain.leaf.reported_sample_ns = 190;
  EXPECT_EQ(hc::StagedStatus::committed, chain.group->run_ns(200, 1).status);

  // a leaf may not report a future sample
  chain.leaf.reported_sample_ns = 500;
  EXPECT_EQ(hc::StagedStatus::invalid_input, chain.group->run_ns(200, 1).status);
}

TEST(HierarchicalControlKernel, library_mode_runs_without_lifecycle_refresh)
{
  Chain chain;
  chain.root.as_root().as_composite();
  chain.module.as_composite();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());
  EXPECT_TRUE(chain.group->members_active());
  EXPECT_EQ(hc::StagedStatus::committed, chain.group->run_ns(1000, 1).status);
}

TEST(HierarchicalControlKernel, configuration_errors_are_rejected)
{
  StubNode a{"a", 2.0, 1.0}, b{"b", 1.0, 3.0};
  a.as_root().as_composite();
  b.as_leaf();

  auto rejects = [](std::function<void()> f)
  {
    bool rejected = false;
    try
    {
      f();
    }
    catch (const std::invalid_argument &)
    {
      rejected = true;
    }
    EXPECT_TRUE(rejected);
  };

  using Spec = hc::StagedExecutionGroup::Spec;
  // duplicate names
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "a"}, {&a, &b}, {"", "a"}});});
  // inconsistent sizes
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a"}, {&a, &b}, {""}});});
  // unknown parent
  rejects(
    [&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "b"}, {&a, &b}, {"", "missing"}});});
  // two roots
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "b"}, {&a, &b}, {"", ""}});});
  // cycle
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "b"}, {&a, &b}, {"b", "a"}});});
  // node is its own parent
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "b"}, {&a, &b}, {"", "b"}});});
  // the SAME instance bound to two node names. Nothing else rejects this (the names differ, so a
  // name check passes), yet the kernel would advance that controller twice per phase in one cycle,
  // so the kernel enforces it where the "once per stage per controller" guarantee lives.
  rejects([&] {hc::StagedExecutionGroup::create_library(Spec{{"a", "b"}, {&a, &a}, {"", "a"}});});
}

TEST(HierarchicalControlKernel, structural_errors_are_rejected)
{
  StubNode root{"root", 2.0, 0.0};
  root.as_root().as_composite();

  using Spec = hc::StagedExecutionGroup::Spec;
  // root declares reference ports but provides no reference source
  class NoSourceNode : public StubNode
  {
  public:
    NoSourceNode() : StubNode("root", 2.0, 0.0) {}
    hc::StagedReferenceSource * staged_reference_source() noexcept override {return nullptr;}
  } no_source;
  no_source.as_composite();
  bool rejected = false;
  try
  {
    hc::StagedExecutionGroup::create_library(Spec{{"root"}, {&no_source}, {""}});
  }
  catch (const std::invalid_argument &)
  {
    rejected = true;
  }
  EXPECT_TRUE(rejected);

  // actuator ports without a sink
  class NoSinkNode : public StubNode
  {
  public:
    NoSinkNode() : StubNode("naked", 1.0, 1.0) {}
    std::vector<std::string> staged_actuator_ports() const override {return {"actuator"};}
    hc::StagedCommandSink * staged_command_sink() noexcept override {return nullptr;}
  } no_sink;
  rejected = false;
  try
  {
    hc::StagedExecutionGroup::create_library(
      Spec{{"root", "naked"}, {&root, &no_sink}, {"", "root"}});
  }
  catch (const std::invalid_argument &)
  {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

// ---------------------------------------------------------------------------------------------
// Why two passes are necessary: a single pass can keep only ONE direction same-cycle.
//
// Model: a cascade root -> mid -> leaf. Each node owns a non-re-derivable internal estimate
// (a first-order filter of its child's estimate, or of hardware for the leaf), and each node's
// command is `parent_reference - estimate`. A step is injected either in hardware (upward path)
// or in the root reference (downward path); we record the first cycle at which each node responds.
//
//   scheduler                       | hardware step (upward) | reference step (downward)
//   --------------------------------|------------------------|--------------------------
//   single pass, parents first      | lag = depth            | fresh
//   single pass, children first     | fresh                  | lag = depth
//   two passes over the same order  | fresh                  | fresh
// ---------------------------------------------------------------------------------------------
namespace
{
struct CascadeNode
{
  double estimate = 0.0;
  double command = 0.0;
};

/// One method per node: each call both ingests its child and produces its command.
void run_single_pass(
  std::vector<CascadeNode> & nodes, double hardware, double root_reference, bool parents_first)
{
  const int n = static_cast<int>(nodes.size());
  for (int step = 0; step < n; ++step)
  {
    const int i = parents_first ? step : n - 1 - step;
    const auto next = static_cast<std::size_t>(i + 1);
    const auto prev = static_cast<std::size_t>(i - 1);
    const double child_estimate = (i + 1 < n) ? nodes[next].estimate : hardware;
    const double parent_reference = (i > 0) ? nodes[prev].command : root_reference;
    auto & node = nodes[static_cast<std::size_t>(i)];
    node.estimate = 0.5 * node.estimate + 0.5 * child_estimate;
    node.command = parent_reference - node.estimate;
  }
}

/// Two passes over ONE linear order: reverse for update (children first), forward for handle.
void run_two_pass(std::vector<CascadeNode> & nodes, double hardware, double root_reference)
{
  const int n = static_cast<int>(nodes.size());
  for (int i = n - 1; i >= 0; --i)  // update: children before parents
  {
    const double child_estimate =
      (i + 1 < n) ? nodes[static_cast<std::size_t>(i + 1)].estimate : hardware;
    auto & node = nodes[static_cast<std::size_t>(i)];
    node.estimate = 0.5 * node.estimate + 0.5 * child_estimate;
  }
  for (int i = 0; i < n; ++i)  // handle: parents before children (reverse of the same order)
  {
    const double parent_reference =
      (i > 0) ? nodes[static_cast<std::size_t>(i - 1)].command : root_reference;
    auto & node = nodes[static_cast<std::size_t>(i)];
    node.command = parent_reference - node.estimate;
  }
}

/// First cycle (1-based) at which node i's estimate exceeds `threshold`, in the given scenario.
int first_response_cycle(
  int nodes_count, int cycles, double step_cycle, bool hardware_step, bool two_pass,
  bool parents_first, std::size_t node_index)
{
  std::vector<CascadeNode> nodes(static_cast<std::size_t>(nodes_count));
  const double root_reference = 10.0;
  // Start from the steady state so the response detector only fires on the injected step.
  for (auto & node : nodes) {node.command = root_reference;}
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    const double hardware = (hardware_step && cycle >= static_cast<int>(step_cycle)) ? 1.0 : 0.0;
    const double reference =
      (!hardware_step && cycle >= static_cast<int>(step_cycle)) ? 20.0 : root_reference;
    if (two_pass)
    {
      run_two_pass(nodes, hardware, reference);
    }
    else
    {
      run_single_pass(nodes, hardware, reference, parents_first);
    }
    const double observed = hardware_step ? nodes[node_index].estimate
                                          : nodes[node_index].command;
    const double baseline = hardware_step ? 0.0 : root_reference;
    if (std::fabs(observed - baseline) > 0.05) {return cycle;}
  }
  return -1;
}
}  // namespace

TEST(HierarchicalControlKernel, one_pass_can_keep_only_one_direction_same_cycle)
{
  const int nodes_count = 3;
  const int step_cycle = 10;
  const int cycles = 40;
  auto lag = [&](bool hardware_step, bool two_pass, bool parents_first, int node)
  {
    return first_response_cycle(
             nodes_count, cycles, step_cycle, hardware_step, two_pass, parents_first,
             static_cast<std::size_t>(node)) -
           step_cycle;
  };

  for (int i = 0; i < nodes_count; ++i)
  {
    // one pass, parents first (the order ros2_control already uses): the root is the stalest,
    // its information is `depth` cycles old; the leaf itself is fresh.
    EXPECT_EQ(nodes_count - 1 - i, lag(true, false, true, i))
      << "1-pass parents-first, upward, depth " << i;
    EXPECT_EQ(0, lag(false, false, true, i)) << "1-pass parents-first, downward, depth " << i;
    // one pass, children first: the staleness is traded to the other direction
    EXPECT_EQ(0, lag(true, false, false, i)) << "1-pass children-first, upward, depth " << i;
    EXPECT_EQ(i, lag(false, false, false, i)) << "1-pass children-first, downward, depth " << i;
    // two passes over the SAME order: both directions same-cycle
    EXPECT_EQ(0, lag(true, true, true, i)) << "2-pass, upward, depth " << i;
    EXPECT_EQ(0, lag(false, true, true, i)) << "2-pass, downward, depth " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// Theorem 1 (single-pass impossibility), checked by exhaustion over ALL node orders.
//
// For a chain root -> mid -> leaf where every level also has a state edge back to its parent:
//   * a reference edge (p,c) requires p before c, otherwise the child uses a stale reference;
//   * the state edge (c,p) in a single pass requires c before p, otherwise the parent reads a
//     stale child state.
// Both cannot hold at once, so EVERY single-pass order is stale in at least one direction.
// ---------------------------------------------------------------------------------------------
namespace
{
/// One pass over an arbitrary node order. `order` holds node indices, root = 0, leaf = n-1.
void run_order_pass(
  std::vector<CascadeNode> & nodes, const std::vector<int> & order, double hardware,
  double root_reference)
{
  const int n = static_cast<int>(nodes.size());
  for (const int i : order)
  {
    const double child_estimate =
      (i + 1 < n) ? nodes[static_cast<std::size_t>(i + 1)].estimate : hardware;
    const double parent_reference =
      (i > 0) ? nodes[static_cast<std::size_t>(i - 1)].command : root_reference;
    auto & node = nodes[static_cast<std::size_t>(i)];
    node.estimate = 0.5 * node.estimate + 0.5 * child_estimate;
    node.command = parent_reference - node.estimate;
  }
}

struct DirectionLags
{
  int upward_root = -1;    ///< cycles before the root reacts to a leaf-side (hardware) step
  int downward_leaf = -1;  ///< cycles before the leaf reacts to a root-side (reference) step
};

DirectionLags lags_for_order(
  const std::vector<int> & order, int nodes_count, int step_cycle, int cycles)
{
  const double root_reference = 10.0;
  const int leaf = nodes_count - 1;
  DirectionLags result;

  std::vector<CascadeNode> up(static_cast<std::size_t>(nodes_count));
  for (auto & node : up) {node.command = root_reference;}
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    run_order_pass(up, order, cycle >= step_cycle ? 1.0 : 0.0, root_reference);
    const int lag = cycle - step_cycle;
    if (lag >= 0 && result.upward_root < 0 && up[0].estimate > 0.05) {result.upward_root = lag;}
  }

  std::vector<CascadeNode> down(static_cast<std::size_t>(nodes_count));
  for (auto & node : down) {node.command = root_reference;}
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    const double reference = cycle >= step_cycle ? 20.0 : root_reference;
    run_order_pass(down, order, 0.0, reference);
    const int lag = cycle - step_cycle;
    if (
      lag >= 0 && result.downward_leaf < 0 &&
      std::fabs(down[static_cast<std::size_t>(leaf)].command - root_reference) > 0.05)
    {
      result.downward_leaf = lag;
    }
  }
  return result;
}
}  // namespace

TEST(HierarchicalControlKernel, every_single_pass_order_is_stale_in_some_direction)
{
  const int nodes_count = 3;
  const int depth = nodes_count - 1;
  const int step_cycle = 10;
  const int cycles = 40;

  std::vector<int> order{0, 1, 2};
  int considered = 0;
  do
  {
    const auto lags = lags_for_order(order, nodes_count, step_cycle, cycles);
    ASSERT_GE(lags.upward_root, 0);
    ASSERT_GE(lags.downward_leaf, 0);
    // Theorem 1: never fresh in both directions, and never worse than the depth.
    EXPECT_TRUE(lags.upward_root > 0 || lags.downward_leaf > 0)
      << "order [" << order[0] << "," << order[1] << "," << order[2] << "] is fresh in both";
    EXPECT_LE(lags.upward_root, depth);
    EXPECT_LE(lags.downward_leaf, depth);
    ++considered;
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(6, considered) << "three nodes must have six distinct orders";

  // Among the orders a manager can actually use, only [0,1,2] keeps the reference (command)
  // direction consistent, and it leaves every state edge one cycle stale: upward lag = depth.
  const auto consistent = lags_for_order({0, 1, 2}, nodes_count, step_cycle, cycles);
  EXPECT_EQ(depth, consistent.upward_root);
  EXPECT_EQ(0, consistent.downward_leaf);
}
