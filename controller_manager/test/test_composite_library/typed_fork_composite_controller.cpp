// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "test_composite_library/typed_fork_composite_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/static_topology.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace test_composite_library
{
namespace
{
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

controller_interface::InterfaceConfiguration make_individual_config(
  const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

// ---------------------------------------------------------------------------------------------
// THE DECLARATION. This block is the whole topology statement: nodes, ports, dimensions, edges.
// ---------------------------------------------------------------------------------------------

#define TYPED_FORK_NAME(struct_name, text)  \
  struct struct_name                        \
  {                                         \
    static constexpr auto value = st::NameOf(text); \
  }

TYPED_FORK_NAME(root_n, "typed_root");
TYPED_FORK_NAME(a_n, "typed_a");
TYPED_FORK_NAME(b_n, "typed_b");

using root_node = st::Root<root_n>;
using a_node = st::Descendant<a_n, root_node>;
using b_node = st::Descendant<b_n, root_node>;

TYPED_FORK_NAME(root_state_n, "typed_root/state");
TYPED_FORK_NAME(root_ref_n, "typed_root/ref");
TYPED_FORK_NAME(a_state_n, "typed_a/state");
TYPED_FORK_NAME(a_ref_n, "typed_a/ref");
TYPED_FORK_NAME(b_state_n, "typed_b/state");
TYPED_FORK_NAME(b_ref_n, "typed_b/ref");
/// The actuators are hardware interfaces. They are declared here only so the mixin GENERATES
/// `staged_actuator_ports()`; a `Contract` deliberately excludes them, so they take no part in the
/// topology ownership checks.
TYPED_FORK_NAME(a_actuator_n, "joint2/velocity");
TYPED_FORK_NAME(b_actuator_n, "joint3/velocity");

using root_state = tc::Port<root_state_n, dm::Position>;
using root_ref = tc::Port<root_ref_n, dm::LinearVelocity>;
using a_state = tc::Port<a_state_n, dm::Position>;
using a_ref = tc::Port<a_ref_n, dm::LinearVelocity>;
using b_state = tc::Port<b_state_n, dm::Position>;
using b_ref = tc::Port<b_ref_n, dm::LinearVelocity>;
using a_actuator = tc::Port<a_actuator_n, dm::LinearVelocity>;
using b_actuator = tc::Port<b_actuator_n, dm::LinearVelocity>;

/// The root declares the concatenation, IN CHILD ORDER, of what its two children declare: the
/// references it writes into them and the states it reads back. `compose` checks exactly this.
using root_ports = tp::TypedPorts<
  tc::PortList<root_state>, tc::PortList<root_ref>, tc::PortList<>,
  tc::PortList<a_ref, b_ref>, tc::PortList<a_state, b_state>>;
using a_ports = tp::TypedPorts<tc::PortList<a_state>, tc::PortList<a_ref>, tc::PortList<a_actuator>>;
using b_ports = tp::TypedPorts<tc::PortList<b_state>, tc::PortList<b_ref>, tc::PortList<b_actuator>>;

}  // namespace

// ---------------------------------------------------------------------------------------------
// One internal node. It is NOT a manager-managed controller: it lives inside this plugin.
// ---------------------------------------------------------------------------------------------

/// The minimal `ControllerInterfaceBase` an internal node needs to be BINDABLE.
/**
 * `topology_binding` stores a TYPED `ControllerInterfaceBase *` (review R5: a `void *` round trip
 * would lose the address adjustment of a second base), so a node that is only a
 * `StagedControllerInterface` cannot be bound -- the compile-time declaration would not compile.
 * This base satisfies that requirement with the smallest possible object; the node is never
 * initialised, configured or activated by the manager, so the overrides below are never called.
 * (A node that skips the binding and builds the kernel from a runtime `Spec` does not need it, which
 * is why the data-driven host in this package works with a `StagedControllerInterface` alone.)
 */
class TypedForkCompositeController::Node
{
public:
  virtual ~Node() = default;
  virtual int state_calls() const = 0;
  virtual int command_calls() const = 0;
  virtual hierarchical_control::StagedControllerInterface * staged() = 0;
  virtual controller_interface::ControllerInterfaceBase * bindable() = 0;
};

namespace
{
class NodeBase : public virtual controller_interface::ControllerInterfaceBase
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {};
  }
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }
  bool is_chainable() const override {return false;}
  std::vector<hardware_interface::CommandInterface> export_reference_interfaces() override
  {
    return {};
  }
  bool set_chained_mode(bool /*chained_mode*/) override {return true;}
  bool is_in_chained_mode() const override {return false;}
};

}  // namespace

/// A typed node: the port strings come from `Ports`, and the algorithm is the same synthetic one the
/// data-driven host uses, so both hosts compute identical output for identical input.
/**
 *     leaf  : state = offset + sum(loaned hardware states),  command = reference - state
 *     branch: state = factor * sum(child states),            command = reference - state
 *
 * `command` is written into every child reference and into the actuators. The port lists the kernel
 * sizes its buffers from are GENERATED by `TypedPortsMixin` from `Ports`, so a hand-written string
 * cannot drift from the declaration.
 */
template <typename Ports>
class TypedForkCompositeController::TypedNode
: public NodeBase,
  public tp::TypedPortsMixin<TypedNode<Ports>, Ports>
{
public:
  TypedNode(
    TypedForkCompositeController * owner, std::size_t index, bool leaf, bool root, double factor,
    double offset)
  : owner_(owner), index_(index), is_leaf_(leaf), factor_(factor), offset_(offset)
  {
    if (leaf) {sink_ = std::make_unique<Sink>(this);}
    if (root) {source_ = std::make_unique<Source>(this);}
  }

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return sink_.get();
  }
  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return source_.get();
  }

  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView & children,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    ++state_calls_;
    double value = offset_;
    if (is_leaf_)
    {
      const auto & slots = owner_->state_slots_[index_];
      for (const auto slot : slots) {value += owner_->state_interfaces_[slot].get_value();}
    }
    else
    {
      value = 0.0;
      for (std::size_t c = 0; c < children.size(); ++c)
      {
        for (std::size_t p = 0; p < children[c].size(); ++p) {value += children[c][p];}
      }
      value *= factor_;
    }
    if (state.size() > 0) {state[0] = value;}
    for (std::size_t i = 1; i < state.size(); ++i) {state[i] = 0.0;}
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView & reference,
    const hierarchical_control::StagedReferenceWriter & children,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    ++command_calls_;
    const double command =
      (reference.size() > 0 ? reference[0] : 0.0) - (state.size() > 0 ? state[0] : 0.0);
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = command;}
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = command;}
    return controller_interface::return_type::OK;
  }

  int state_calls() const {return state_calls_;}
  int command_calls() const {return command_calls_;}

private:
  class Sink : public hierarchical_control::StagedCommandSink
  {
  public:
    explicit Sink(TypedNode * node) : node_(node) {}
    bool commit(const double * values, std::size_t size) noexcept override
    {
      ++node_->owner_->commit_calls;
      const auto & slots = node_->owner_->command_slots_[node_->index_];
      const auto count = std::min(size, slots.size());
      for (std::size_t i = 0; i < count; ++i)
      {
        node_->owner_->command_interfaces_[slots[i]].set_value(values[i]);
      }
      return true;
    }

  private:
    TypedNode * node_;
  };

  class Source : public hierarchical_control::StagedReferenceSource
  {
  public:
    explicit Source(TypedNode * node) : node_(node) {}
    bool read(std::uint64_t, std::int64_t, double * values, std::size_t size) noexcept override
    {
      for (std::size_t i = 0; i < size; ++i) {values[i] = node_->owner_->external_reference_;}
      return true;
    }

  private:
    TypedNode * node_;
  };

  TypedForkCompositeController * owner_;
  std::size_t index_;
  bool is_leaf_;
  double factor_;
  double offset_;
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<Source> source_;
  int state_calls_ = 0;
  int command_calls_ = 0;
};

/// Type-erased adapter, so the plugin can keep nodes with different port types.
template <typename Ports>
class TypedForkCompositeController::WrappedNode : public Node
{
public:
  WrappedNode(
    TypedForkCompositeController * owner, std::size_t index, bool leaf, bool root, double factor,
    double offset)
  : node_(owner, index, leaf, root, factor, offset)
  {
  }

  int state_calls() const override {return node_.state_calls();}
  int command_calls() const override {return node_.command_calls();}
  hierarchical_control::StagedControllerInterface * staged() override {return &node_;}
  controller_interface::ControllerInterfaceBase * bindable() override {return &node_;}
  TypedNode<Ports> & get() {return node_;}

private:
  TypedNode<Ports> node_;
};

TypedForkCompositeController::TypedForkCompositeController() = default;
TypedForkCompositeController::~TypedForkCompositeController() = default;

void TypedForkCompositeController::set_external_reference(double value)
{
  external_reference_ = value;
}

double TypedForkCompositeController::command_interface_value(std::size_t leaf_index) const
{
  if (leaf_index >= committed_value_.size()) {return 0.0;}
  return committed_value_[leaf_index];
}

std::vector<std::string> TypedForkCompositeController::plan_node_names() const
{
  return plan_names_;
}

int TypedForkCompositeController::state_calls(std::size_t node) const
{
  return node < nodes_.size() ? nodes_[node]->state_calls() : -1;
}

int TypedForkCompositeController::command_calls(std::size_t node) const
{
  return node < nodes_.size() ? nodes_[node]->command_calls() : -1;
}

controller_interface::InterfaceConfiguration
TypedForkCompositeController::command_interface_configuration() const
{
  // The actuator ports of the two leaves. They are declared in the typed ports above, so this list
  // and what the kernel sizes its buffers from come from the same statement.
  return make_individual_config({"joint2/velocity", "joint3/velocity"});
}

controller_interface::InterfaceConfiguration
TypedForkCompositeController::state_interface_configuration() const
{
  return make_individual_config({"joint2/position", "joint3/position"});
}

CallbackReturn TypedForkCompositeController::on_init() {return CallbackReturn::SUCCESS;}

CallbackReturn TypedForkCompositeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

bool TypedForkCompositeController::resolve_interface_slots()
{
  state_slots_.assign(2, {});
  command_slots_.assign(2, {});
  const std::vector<std::string> wanted_states = {"joint2/position", "joint3/position"};
  const std::vector<std::string> wanted_commands = {"joint2/velocity", "joint3/velocity"};
  for (std::size_t leaf = 0; leaf < 2; ++leaf)
  {
    for (const auto & wanted : {wanted_states[leaf]})
    {
      const auto it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedStateInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == state_interfaces_.end()) {return false;}
      state_slots_[leaf].push_back(static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it)));
    }
    for (const auto & wanted : {wanted_commands[leaf]})
    {
      const auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedCommandInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == command_interfaces_.end()) {return false;}
      command_slots_[leaf].push_back(
        static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it)));
    }
  }
  committed_value_.assign(2, 0.0);
  return true;
}

bool TypedForkCompositeController::build_kernel()
{
  // The nodes. `factor`/`offset` are the same numbers the data-driven host takes from its specs.
  auto root_wrapper = std::make_unique<WrappedNode<root_ports>>(this, 0, false, true, 2.0, 0.0);
  auto a_wrapper = std::make_unique<WrappedNode<a_ports>>(this, 0, true, false, 1.0, 3.0);
  auto b_wrapper = std::make_unique<WrappedNode<b_ports>>(this, 1, true, false, 1.0, 5.0);

  // The whole binding, written once.
  const auto a_leaf = tc::make_leaf<a_node, tp::contract_of_t<a_ports>>(&a_wrapper->get());
  const auto b_leaf = tc::make_leaf<b_node, tp::contract_of_t<b_ports>>(&b_wrapper->get());
  const auto binding = tc::compose<root_node, tp::contract_of_t<root_ports>>(
    &root_wrapper->get(), a_leaf, b_leaf);

  std::shared_ptr<hierarchical_control::StagedExecutionGroup> group;
  try
  {
    // The CHECKED entry: compile-time ownership/dimensions (already enforced by `compose`), the
    // runtime port lists of every node of the tree, then the plan and the group.
    group = hierarchical_control::topology_binding::create_library_group(binding);
  }
  catch (const std::invalid_argument &)
  {
    return false;
  }

  plan_names_ = tc::build_spec_rows(binding).names;
  nodes_.push_back(std::move(root_wrapper));
  nodes_.push_back(std::move(a_wrapper));
  nodes_.push_back(std::move(b_wrapper));
  kernel_ = std::move(group);
  ++build_allocations;
  return true;
}

CallbackReturn TypedForkCompositeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Non-real-time setup: the loans are already indexed (`assign_interfaces()` runs just before the
  // lifecycle activation), so the whole kernel -- and every allocation it makes -- happens here and
  // the control loop stays allocation-free.
  if (!resolve_interface_slots()) {return CallbackReturn::FAILURE;}
  nodes_.clear();
  kernel_.reset();
  if (!build_kernel()) {return CallbackReturn::FAILURE;}
  return CallbackReturn::SUCCESS;
}

CallbackReturn TypedForkCompositeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Never reuse a kernel across an activation boundary: the loans may differ.
  kernel_.reset();
  nodes_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TypedForkCompositeController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  kernel_.reset();
  nodes_.clear();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TypedForkCompositeController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ++update_calls;
  if (!kernel_) {return controller_interface::return_type::ERROR;}
  const auto result = kernel_->run_ns(time.nanoseconds(), period.nanoseconds());
  if (result.status != hierarchical_control::StagedStatus::committed)
  {
    return controller_interface::return_type::ERROR;
  }
  for (std::size_t leaf = 0; leaf < committed_value_.size(); ++leaf)
  {
    if (command_slots_[leaf].empty()) {continue;}
    committed_value_[leaf] = command_interfaces_[command_slots_[leaf][0]].get_value();
  }
  return controller_interface::return_type::OK;
}

}  // namespace test_composite_library
