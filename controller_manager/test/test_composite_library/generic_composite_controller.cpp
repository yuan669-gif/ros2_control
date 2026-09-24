// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "test_composite_library/generic_composite_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace test_composite_library
{
namespace
{
controller_interface::InterfaceConfiguration make_individual_config(
  const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}
}  // namespace

/// One node of the hosted tree. It implements the same staged interface as a manager-managed
/// controller, which is what lets the exact same kernel run in library mode.
class GenericCompositeController::Node : public hierarchical_control::StagedControllerInterface
{
public:
  Node(GenericCompositeController * owner, std::size_t index, bool leaf, bool root,
    const CompositeNodeSpec & spec)
  : owner_(owner), index_(index), is_leaf_(leaf), factor_(spec.factor), offset_(spec.offset)
  {
    state_ports_.push_back("state");
    reference_ports_.push_back("ref");
    if (leaf)
    {
      actuator_ports_ = spec.command_interfaces;
      sink_ = std::make_unique<Sink>(owner, index);
    }
    if (root) {source_ = std::make_unique<Source>(owner);}
  }

  std::vector<std::string> staged_state_ports() const override {return state_ports_;}
  std::vector<std::string> staged_reference_ports() const override {return reference_ports_;}
  std::vector<std::string> staged_actuator_ports() const override {return actuator_ports_;}

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return sink_.get();
  }

  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return source_.get();
  }

  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView & children,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    if (owner_->fail_state_)
    {
      state.set_fault(0x61u);
      return controller_interface::return_type::ERROR;
    }
    if (is_leaf_)
    {
      const auto & slots = owner_->state_slots_[index_];
      double sum = offset_;
      for (std::size_t i = 0; i < slots.size(); ++i)
      {
        sum += owner_->state_interfaces_[slots[i]].get_value();
      }
      if (state.size() > 0) {state[0] = sum;}
      for (std::size_t i = 1; i < state.size(); ++i) {state[i] = 0.0;}
    }
    else
    {
      double sum = 0.0;
      for (std::size_t c = 0; c < children.size(); ++c)
      {
        for (std::size_t p = 0; p < children[c].size(); ++p) {sum += children[c][p];}
      }
      if (state.size() > 0) {state[0] = factor_ * sum;}
      for (std::size_t i = 1; i < state.size(); ++i) {state[i] = 0.0;}
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &,
    const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView & reference,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    if (owner_->fail_command_)
    {
      actuators.set_fault(0x62u);
      return controller_interface::return_type::ERROR;
    }
    const double state_value = state.size() > 0 ? state[0] : 0.0;
    const double reference_value = reference.size() > 0 ? reference[0] : 0.0;
    double command = reference_value - state_value;
    if (owner_->emit_nan_) {command = std::numeric_limits<double>::quiet_NaN();}
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

private:
  class Sink : public hierarchical_control::StagedCommandSink
  {
  public:
    Sink(GenericCompositeController * owner, std::size_t index) : owner_(owner), index_(index) {}

    bool commit(const double * values, std::size_t size) noexcept override
    {
      ++owner_->commit_calls;
      const auto & slots = owner_->command_slots_[index_];
      const auto count = std::min(size, slots.size());
      for (std::size_t i = 0; i < count; ++i)
      {
        owner_->command_interfaces_[slots[i]].set_value(values[i]);
      }
      return true;
    }

  private:
    GenericCompositeController * owner_;
    std::size_t index_;
  };

  class Source : public hierarchical_control::StagedReferenceSource
  {
  public:
    explicit Source(GenericCompositeController * owner) : owner_(owner) {}

    bool read(
      std::uint64_t, std::int64_t, double * values, std::size_t size) noexcept override
    {
      for (std::size_t i = 0; i < size; ++i) {values[i] = owner_->external_reference_;}
      return true;
    }

  private:
    GenericCompositeController * owner_;
  };

  GenericCompositeController * owner_;
  std::size_t index_;
  bool is_leaf_;
  double factor_;
  double offset_;
  std::vector<std::string> state_ports_;
  std::vector<std::string> reference_ports_;
  std::vector<std::string> actuator_ports_;
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<Source> source_;
};

GenericCompositeController::GenericCompositeController() = default;
GenericCompositeController::~GenericCompositeController() = default;

void GenericCompositeController::set_nodes(std::vector<CompositeNodeSpec> nodes)
{
  specs_ = std::move(nodes);
}

void GenericCompositeController::set_external_reference(double value) {external_reference_ = value;}
void GenericCompositeController::set_fail_state(bool value) {fail_state_ = value;}
void GenericCompositeController::set_fail_command(bool value) {fail_command_ = value;}
void GenericCompositeController::set_emit_nan(bool value) {emit_nan_ = value;}

bool GenericCompositeController::is_leaf(std::size_t index) const
{
  for (const auto & spec : specs_)
  {
    if (spec.parent == specs_[index].name) {return false;}
  }
  return true;
}

controller_interface::InterfaceConfiguration
GenericCompositeController::command_interface_configuration() const
{
  std::vector<std::string> names;
  for (const auto & spec : specs_)
  {
    for (const auto & name : spec.command_interfaces) {names.push_back(name);}
  }
  return make_individual_config(names);
}

controller_interface::InterfaceConfiguration
GenericCompositeController::state_interface_configuration() const
{
  std::vector<std::string> names;
  for (const auto & spec : specs_)
  {
    for (const auto & name : spec.state_interfaces) {names.push_back(name);}
  }
  return make_individual_config(names);
}

CallbackReturn GenericCompositeController::on_init() {return CallbackReturn::SUCCESS;}

CallbackReturn GenericCompositeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (specs_.empty()) {return CallbackReturn::FAILURE;}
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCompositeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Build the kernel HERE, while the manager is still on a non-real-time activation path:
  // `ControllerManager::activate_controllers()` calls `assign_interfaces()` immediately before
  // `get_node()->activate()`, so the loaned interfaces are already indexed. Building lazily inside
  // `update()` instead would put a one-time allocation on the control path
  // (IMPLEMENTATION_GUIDE section 12, item 11).
  if (!build_kernel()) {return CallbackReturn::FAILURE;}
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCompositeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release the kernel so a re-activation rebuilds it: the interface loans may differ across an
  // activation boundary, so a cached kernel must never be reused.
  kernel_.reset();
  nodes_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCompositeController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  kernel_.reset();
  nodes_.clear();
  return CallbackReturn::SUCCESS;
}

bool GenericCompositeController::build_kernel()
{
  // One-time, non-real-time setup: resolve names to already-loaned interface slots.
  state_slots_.assign(specs_.size(), {});
  command_slots_.assign(specs_.size(), {});
  for (std::size_t i = 0; i < specs_.size(); ++i)
  {
    if (!is_leaf(i)) {continue;}
    for (const auto & wanted : specs_[i].state_interfaces)
    {
      const auto it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedStateInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == state_interfaces_.end()) {return false;}
      state_slots_[i].push_back(
        static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it)));
    }
    for (const auto & wanted : specs_[i].command_interfaces)
    {
      const auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedCommandInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == command_interfaces_.end()) {return false;}
      command_slots_[i].push_back(
        static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it)));
    }
  }

  nodes_.clear();
  hierarchical_control::StagedExecutionGroup::Spec spec;
  for (std::size_t i = 0; i < specs_.size(); ++i)
  {
    nodes_.push_back(
      std::make_unique<Node>(this, i, is_leaf(i), specs_[i].parent.empty(), specs_[i]));
    spec.names.push_back(specs_[i].name);
    spec.instances.push_back(nodes_.back().get());
    spec.parents.push_back(specs_[i].parent);
  }
  try
  {
    kernel_ = hierarchical_control::StagedExecutionGroup::create_library(spec);
  }
  catch (const std::invalid_argument &)
  {
    kernel_.reset();
    return false;
  }
  ++build_allocations;
  return true;
}

controller_interface::return_type GenericCompositeController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ++update_calls;
  // The kernel is built in on_activate(). Never build it here: the control loop must not allocate.
  if (!kernel_) {return controller_interface::return_type::ERROR;}
  const auto result = kernel_->run(time, period);
  return result.status == hierarchical_control::StagedStatus::committed
           ? controller_interface::return_type::OK
           : controller_interface::return_type::ERROR;
}

double GenericCompositeController::command_interface_value(std::size_t leaf_index) const
{
  std::size_t ordinal = 0;
  for (std::size_t i = 0; i < specs_.size(); ++i)
  {
    if (!is_leaf(i)) {continue;}
    if (ordinal == leaf_index)
    {
      if (command_slots_[i].empty()) {break;}
      return command_interfaces_[command_slots_[i].front()].get_value();
    }
    ++ordinal;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace test_composite_library
