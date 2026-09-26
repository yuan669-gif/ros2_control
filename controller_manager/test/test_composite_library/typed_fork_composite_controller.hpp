// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_
#define TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "hierarchical_control/staged_execution_group.hpp"
#include "hierarchical_control/static_manifest.hpp"
#include "test_composite_library/typed_fork_declaration.hpp"

namespace test_composite_library
{
namespace typed_ports = hierarchical_control::typed_ports;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// A composite plugin that hosts a BRANCHING tree declared ONCE at compile time.
/**
 * `GenericCompositeController` in this package is data-driven: a list of `CompositeNodeSpec` is read
 * at runtime, parents are strings, and the kernel's port lists are hand-written strings. Here the
 * whole description is one type-level statement (`test_composite_library::typed_fork`):
 *
 *     typed_root -> { typed_a, typed_b }
 *
 * and everything else is DERIVED from it:
 *   * `compose` checks the parent/child edges at compile time;
 *   * `TypedPortsMixin` generates the port strings the kernel sizes its buffers from;
 *   * `static_manifest::manifest_of_v<binding_type>` is the compile-time description (below);
 *   * `command_interface_configuration()` / `state_interface_configuration()` are GENERATED from
 *     that manifest, so no hardware interface name is written twice;
 *   * the internal nodes live in FIXED storage (a `std::tuple` member), constructed once per plugin
 *     instance: no heap object per node, no cross-translation-unit construction order, and two
 *     managers using the same declaration share nothing but the immutable description.
 */
class TypedForkCompositeController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_MANAGER_PUBLIC
  TypedForkCompositeController();

  CONTROLLER_MANAGER_PUBLIC
  ~TypedForkCompositeController() override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

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

  /// The root's external reference, i.e. what the root's reference source returns.
  CONTROLLER_MANAGER_PUBLIC
  void set_external_reference(double value);

  /// What the kernel actually committed to leaf `index` (0 = typed_a, 1 = typed_b).
  CONTROLLER_MANAGER_PUBLIC
  double command_interface_value(std::size_t leaf_index) const;

  /// The node names the checked plan was built from, in plan order (root first).
  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> plan_node_names() const;

  /// Per-node counters, indexed like `plan_node_names()`.
  CONTROLLER_MANAGER_PUBLIC
  int state_calls(std::size_t node) const;

  CONTROLLER_MANAGER_PUBLIC
  int command_calls(std::size_t node) const;

  /// Declare an extra state interface, so a test can show that `on_configure` REFUSES a declaration
  /// that disagrees with the compile-time description instead of letting it reach activation.
  CONTROLLER_MANAGER_PUBLIC
  void set_extra_state_interface(std::string name);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> manifest_command_interfaces() const;

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::string> manifest_state_interfaces() const;

  int update_calls = 0;
  std::size_t commit_calls = 0;
  int build_allocations = 0;
  /// Why `on_configure` failed, when it did ("" when it succeeded).
  std::string configure_error;

class Node
{
public:
  virtual ~Node() = default;
  virtual int state_calls() const = 0;
  virtual int command_calls() const = 0;
  virtual hierarchical_control::StagedControllerInterface * staged() = 0;
  virtual controller_interface::ControllerInterfaceBase * bindable() = 0;
};

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
class TypedNode
: public NodeBase,
  public typed_ports::TypedPortsMixin<TypedNode<Ports>, Ports>
{
public:
  TypedNode(
    TypedForkCompositeController * owner, std::size_t index, bool leaf, bool root, double factor,
    double offset)
  : owner_(owner), index_(index), is_leaf_(leaf), factor_(factor), offset_(offset)
  {
    (void)leaf;
    (void)root;
  }

  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return &sink_;
  }
  hierarchical_control::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return &source_;
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
  // Direct members, not heap objects: the node itself lives in fixed storage, so a sink/source
  // allocation per node would be the only remaining one outside the kernel.
  Sink sink_{this};
  Source source_{this};
  int state_calls_ = 0;
  int command_calls_ = 0;
};

/// Type-erased adapter, so the plugin can keep nodes with different port types.
template <typename Ports>
class WrappedNode : public Node
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

public:
  /// The binding TYPE of this tree: the description exists as a type, so the manifest below needs no
  /// construction, no ROS and no runtime state.
  using binding_type = decltype(hierarchical_control::topology_contract::compose<
    typed_fork::root_node, typed_ports::contract_of_t<typed_fork::root_ports>>(
    std::declval<TypedNode<typed_fork::root_ports> *>(),
    hierarchical_control::topology_contract::make_leaf<
      typed_fork::a_node, typed_ports::contract_of_t<typed_fork::a_ports>>(
      std::declval<TypedNode<typed_fork::a_ports> *>()),
    hierarchical_control::topology_contract::make_leaf<
      typed_fork::b_node, typed_ports::contract_of_t<typed_fork::b_ports>>(
      std::declval<TypedNode<typed_fork::b_ports> *>())));

  /// The compile-time description: nodes, ports and hardware interface requirements.
  static constexpr auto manifest =
    hierarchical_control::static_manifest::manifest_of_v<binding_type>;

private:
  bool build_kernel();
  bool resolve_interface_slots();
  std::vector<std::string> interfaces_of(
    std::string_view owner, hierarchical_control::static_manifest::PortRole role) const;

  /// Index the fixed node storage without recursion at the call site.
  const Node * node_at(std::size_t index) const
  {
    switch (index)
    {
      case 0: return root_node_.has_value() ? &*root_node_ : nullptr;
      case 1: return a_node_.has_value() ? &*a_node_ : nullptr;
      case 2: return b_node_.has_value() ? &*b_node_ : nullptr;
      default: return nullptr;
    }
  }

  /// FIXED storage: three in-place objects owned by this plugin instance. `std::optional` (not a
  /// `std::tuple`) because the nodes hold pointers to themselves (their sinks), so they must be
  /// constructed IN PLACE: `emplace` needs no move constructor, and `ControllerInterfaceBase` has
  /// none. No heap, no global, and nothing shared between two plugin instances.
  std::optional<WrappedNode<typed_fork::root_ports>> root_node_;
  std::optional<WrappedNode<typed_fork::a_ports>> a_node_;
  std::optional<WrappedNode<typed_fork::b_ports>> b_node_;
  /// state/command slot indices into the loaned interfaces, per leaf (0 = typed_a, 1 = typed_b).
  std::vector<std::vector<std::size_t>> state_slots_;
  std::vector<std::vector<std::size_t>> command_slots_;
  std::shared_ptr<hierarchical_control::StagedExecutionGroup> kernel_;
  std::vector<std::string> plan_names_;
  std::vector<double> committed_value_;
  std::string extra_state_interface_;
  double external_reference_ = 0.0;
};

}  // namespace test_composite_library

#endif  // TEST_COMPOSITE_LIBRARY__TYPED_FORK_COMPOSITE_CONTROLLER_HPP_
