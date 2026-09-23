// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__CYCLE_TREE_HPP_
#define CONTROLLER_MANAGER__CYCLE_TREE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager/hierarchy.hpp"

namespace controller_manager
{
namespace cycle_tree
{

// Experimental synchronous scalar-port contract, independent of ROS and hardware handles.
// Configure while inactive. Callbacks must not throw, allocate, or access live command handles.
// Slots are private to this executor; they are NOT Humble exported state interfaces.
struct Value
{
  double value = 0.0;
  std::uint64_t cycle = 0;
  std::int64_t sample_ns = 0;
  bool valid = false;
};

struct Context
{
  std::uint64_t cycle;
  std::int64_t now_ns;
  std::int64_t period_ns;
};

// Fixed-size borrowed output view: callbacks cannot resize executor-owned buffers.
class OutputView
{
public:
  OutputView(double * data, std::size_t size) : data_(data), size_(size) {}
  std::size_t size() const noexcept {return size_;}
  double & operator[](std::size_t index) const noexcept {return data_[index];}

private:
  double * data_;
  std::size_t size_;
};

class Node
{
public:
  virtual ~Node() = default;
  // Leaves consume their indexed hardware snapshot; composites consume children in binding order.
  // sample_ns is the oldest source timestamp. The executor checks timestamp propagation.
  virtual bool state(
    const Context &, const std::vector<Value> & inputs, Value & output) noexcept = 0;
  // References are scalar in this research kernel. Leaves return exactly one actuator command.
  virtual bool command(
    const Context &, const Value & state, const Value & reference,
    OutputView child_references, double & actuator_command) noexcept = 0;
};

struct Binding
{
  std::string name;
  Node * instance;  // Must outlive the executor; instances must be unique.
};

// A resolved reference-interface claim: producer writes a reference consumed by consumer.
// For the first tree-only profile, the consumer also supplies state to the producer.
// A Humble adapter must explicitly verify this reciprocal state binding, not infer it from URDF.
struct Connection
{
  std::string producer;
  std::string consumer;
};

enum class Status { committed, invalid_input, state_failed, command_failed, exhausted };

struct Result
{
  Status status;
  std::uint64_t cycle;
  std::size_t failed_node;
};

class Executor
{
public:
  static constexpr std::size_t no_node = std::numeric_limits<std::size_t>::max();

  Executor(std::vector<Binding> bindings, const std::vector<Connection> & connections)
  : bindings_(std::move(bindings))
  {
    std::vector<ControllerHierarchyNode> nodes;
    for (const auto & binding : bindings_)
    {
      if (!binding.instance) {throw std::invalid_argument("null cycle-tree instance");}
      nodes.push_back({binding.name, ""});
    }
    for (std::size_t i = 0; i < bindings_.size(); ++i)
    {
      for (std::size_t j = 0; j < i; ++j)
      {
        if (bindings_[i].instance == bindings_[j].instance)
        {
          throw std::invalid_argument("duplicate cycle-tree instance");
        }
      }
    }
    children_.resize(nodes.size());
    for (const auto & connection : connections)
    {
      const auto parent = find(connection.producer);
      const auto child = find(connection.consumer);
      if (!nodes[child].parent.empty())
      {
        throw std::invalid_argument("multiple reference writers or duplicate connection");
      }
      nodes[child].parent = nodes[parent].name;
      children_[parent].push_back(child);
    }
    plan_ = build_controller_hierarchy(nodes);
    states_.resize(nodes.size());
    references_.resize(nodes.size());
    inputs_.resize(nodes.size());
    outputs_.resize(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
      inputs_[i].resize(children_[i].empty() ? 1 : children_[i].size());
      outputs_[i].resize(children_[i].size());
      if (children_[i].empty()) {leaves_.push_back(i);}
    }
    scratch_.resize(leaves_.size());
    committed_.resize(leaves_.size());
    leaf_slot_.resize(nodes.size(), no_node);
    for (std::size_t i = 0; i < leaves_.size(); ++i) {leaf_slot_[leaves_[i]] = i;}
  }

  const std::vector<std::size_t> & leaves() const noexcept {return leaves_;}
  const std::vector<double> & committed_commands() const noexcept {return committed_;}
  std::uint64_t committed_cycle() const noexcept {return committed_cycle_;}

  // snapshot order is leaves(); snapshot.cycle is assigned here, not trusted from the caller.
  // Retaining the last committed buffer on failure is NOT a hardware safety policy.
  // Caller must inspect Result before writing hardware and apply its explicit fault action.
  Result run(
    std::int64_t now_ns, std::int64_t period_ns, std::int64_t max_age_ns,
    const std::vector<Value> & snapshot, double root_reference) noexcept
  {
    if (cycle_ == std::numeric_limits<std::uint64_t>::max())
    {
      return {Status::exhausted, cycle_, no_node};
    }
    const Context context{++cycle_, now_ns, period_ns};
    for (auto & value : states_) {value.valid = false;}
    for (auto & value : references_) {value.valid = false;}
    if (now_ns < 0 || period_ns <= 0 || max_age_ns < 0 ||
      snapshot.size() != leaves_.size() || !std::isfinite(root_reference))
    {
      return {Status::invalid_input, cycle_, no_node};
    }
    for (std::size_t slot = 0; slot < snapshot.size(); ++slot)
    {
      const auto & value = snapshot[slot];
      if (!value.valid || !std::isfinite(value.value) || value.sample_ns < 0 ||
        value.sample_ns > now_ns || now_ns - value.sample_ns > max_age_ns)
      {
        return {Status::invalid_input, cycle_, leaves_[slot]};
      }
    }
    for (const auto index : plan_.postorder)
    {
      auto & input = inputs_[index];
      const auto expected_size = children_[index].empty() ? 1 : children_[index].size();
      if (children_[index].empty())
      {
        input[0] = snapshot[leaf_slot_[index]];
        input[0].cycle = cycle_;
      }
      else
      {
        for (std::size_t j = 0; j < expected_size; ++j)
        {
          input[j] = states_[children_[index][j]];
        }
      }
      auto oldest = input[0].sample_ns;
      for (const auto & value : input) {oldest = std::min(oldest, value.sample_ns);}
      auto & output = states_[index];
      output = {};
      if (!bindings_[index].instance->state(context, input, output) || !output.valid ||
        output.cycle != cycle_ || output.sample_ns != oldest || !std::isfinite(output.value))
      {
        output.valid = false;
        return {Status::state_failed, cycle_, index};
      }
    }
    references_[plan_.root] = {root_reference, cycle_, now_ns, true};
    for (const auto index : plan_.preorder)
    {
      auto & output = outputs_[index];
      std::fill(output.begin(), output.end(), std::numeric_limits<double>::quiet_NaN());
      double actuator = std::numeric_limits<double>::quiet_NaN();
      if (!bindings_[index].instance->command(
          context, states_[index], references_[index],
          OutputView(output.data(), output.size()), actuator))
      {
        return {Status::command_failed, cycle_, index};
      }
      for (std::size_t j = 0; j < output.size(); ++j)
      {
        if (!std::isfinite(output[j])) {return {Status::command_failed, cycle_, index};}
        references_[children_[index][j]] = {output[j], cycle_, now_ns, true};
      }
      if (children_[index].empty())
      {
        if (!std::isfinite(actuator)) {return {Status::command_failed, cycle_, index};}
        scratch_[leaf_slot_[index]] = actuator;
      }
    }
    std::copy(scratch_.begin(), scratch_.end(), committed_.begin());
    committed_cycle_ = cycle_;
    return {Status::committed, cycle_, no_node};
  }

private:
  std::size_t find(const std::string & name) const
  {
    for (std::size_t i = 0; i < bindings_.size(); ++i)
    {
      if (bindings_[i].name == name) {return i;}
    }
    throw std::invalid_argument("unknown cycle-tree interface owner: " + name);
  }

  std::vector<Binding> bindings_;
  ControllerHierarchyPlan plan_;
  std::vector<std::vector<std::size_t>> children_;
  std::vector<std::size_t> leaves_, leaf_slot_;
  std::vector<Value> states_, references_;
  std::vector<std::vector<Value>> inputs_;
  std::vector<std::vector<double>> outputs_;
  std::vector<double> scratch_, committed_;
  std::uint64_t cycle_ = 0, committed_cycle_ = 0;
};

}  // namespace cycle_tree
}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CYCLE_TREE_HPP_
