// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "test_composite_library/typed_fork_composite_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/topology_binding.hpp"
#include "test_composite_library/typed_fork_declaration.hpp"

#include "hierarchical_control/static_manifest.hpp"
#include "hierarchical_control/static_two_phase_admission.hpp"
#include "hierarchical_control/static_topology.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace test_composite_library
{
namespace
{
namespace tf = typed_fork;
namespace tc = hierarchical_control::topology_contract;
namespace tp = hierarchical_control::typed_ports;
namespace st = hierarchical_control::static_topology;
namespace dm = hierarchical_control::dimensions;

/// `std::string` is only EXPLICITLY constructible from `std::string_view`, so the manifest's arrays
/// (string_view) cannot feed a vector's range constructor directly.
template <typename Array>
std::vector<std::string> to_strings(const Array & names)
{
  std::vector<std::string> out;
  out.reserve(names.size());
  for (const auto & name : names) {out.emplace_back(name);}
  return out;
}

controller_interface::InterfaceConfiguration make_individual_config(
  const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}
}  // namespace

// The internal nodes are defined in the header: the manifest is derived from the binding TYPE, which
// requires them to be complete types wherever the description is used.

TypedForkCompositeController::TypedForkCompositeController()
{
  // In place: the nodes are never moved, so their self-referencing sinks stay valid.
  root_node_.emplace(this, 0, /*leaf=*/false, /*root=*/true, 2.0, 0.0);
  a_node_.emplace(this, 0, /*leaf=*/true, /*root=*/false, 1.0, 3.0);
  b_node_.emplace(this, 1, /*leaf=*/true, /*root=*/false, 1.0, 5.0);
}

TypedForkCompositeController::~TypedForkCompositeController() = default;

void TypedForkCompositeController::set_external_reference(double value)
{
  external_reference_ = value;
}

void TypedForkCompositeController::set_extra_state_interface(std::string name)
{
  extra_state_interface_ = std::move(name);
}

std::vector<std::string> TypedForkCompositeController::manifest_command_interfaces() const
{
  return to_strings(manifest.command_interfaces);
}

std::vector<std::string> TypedForkCompositeController::manifest_state_interfaces() const
{
  return {manifest.state_interfaces.begin(), manifest.state_interfaces.end()};
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
  const auto * entry = node_at(node);
  return entry != nullptr ? entry->state_calls() : -1;
}

int TypedForkCompositeController::command_calls(std::size_t node) const
{
  const auto * entry = node_at(node);
  return entry != nullptr ? entry->command_calls() : -1;
}

controller_interface::InterfaceConfiguration
TypedForkCompositeController::command_interface_configuration() const
{
  // GENERATED from the declaration. There is no second, hand-written list of names here, so the
  // kernel's buffer sizes and the manager's claim can only disagree if the declaration does.
  return make_individual_config(to_strings(manifest.command_interfaces));
}

controller_interface::InterfaceConfiguration
TypedForkCompositeController::state_interface_configuration() const
{
  std::vector<std::string> names = to_strings(manifest.state_interfaces);
  // Test hook: a controller that declares MORE than the manifest requires. `on_configure` must
  // refuse it by name instead of letting it through to activation.
  if (!extra_state_interface_.empty()) {names.push_back(extra_state_interface_);}
  return make_individual_config(names);
}

CallbackReturn TypedForkCompositeController::on_init() {return CallbackReturn::SUCCESS;}

CallbackReturn TypedForkCompositeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // The configure-time half of the compile-time-description contract: the manifest is fixed at
  // compile time, the declarations are what this controller asks the manager for, and they must
  // agree BEFORE any resource is loaned. A controller whose declaration is generated from the
  // manifest cannot fail here; one that hand-writes it can, and then the failure names the
  // interface instead of surfacing later as a missing slot or a refused activation.
  std::string reason;
  if (!hierarchical_control::static_manifest::declaration_matches_manifest(
        manifest, command_interface_configuration().names,
        state_interface_configuration().names, &reason))
  {
    configure_error = reason;
    RCLCPP_ERROR(
      get_node()->get_logger(), "TypedForkCompositeController refused to configure: %s",
      reason.c_str());
    return CallbackReturn::FAILURE;
  }
  configure_error.clear();
  return CallbackReturn::SUCCESS;
}

std::vector<std::string> TypedForkCompositeController::interfaces_of(
  std::string_view owner, hierarchical_control::static_manifest::PortRole role) const
{
  std::vector<std::string> names;
  for (const auto & port : manifest.ports)
  {
    if (port.owner == owner && port.role == role) {names.emplace_back(port.name);}
  }
  return names;
}

bool TypedForkCompositeController::resolve_interface_slots()
{
  // WHICH nodes are leaves is a compile-time fact of the declaration (`manifest_leaf_names` /
  // `manifest_leaf_count`); only the name -> loaned-slot INDICES are a property of this activation,
  // so only those are resolved here. The previous shape rediscovered leafness with an O(N^2)
  // parent-name scan on every activation and returned a bare `false` when an interface was missing.
  state_slots_.assign(manifest_leaf_count, {});
  command_slots_.assign(manifest_leaf_count, {});
  for (std::size_t leaf = 0; leaf < manifest_leaf_count; ++leaf)
  {
    const std::string_view leaf_name = manifest_leaf_names[leaf];
    for (const auto & wanted :
         interfaces_of(leaf_name, hierarchical_control::static_manifest::PortRole::actuator))
    {
      const auto it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedCommandInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == command_interfaces_.end())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "leaf '%s' requires command interface '%s', which was not loaned to this controller",
          std::string(leaf_name).c_str(), wanted.c_str());
        return false;
      }
      command_slots_[leaf].push_back(
        static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it)));
    }
    for (const auto & wanted : interfaces_of(
           leaf_name, hierarchical_control::static_manifest::PortRole::hardware_state))
    {
      const auto it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&wanted](const hardware_interface::LoanedStateInterface & itf)
        {return itf.get_name() == wanted;});
      if (it == state_interfaces_.end())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "leaf '%s' requires state interface '%s', which was not loaned to this controller",
          std::string(leaf_name).c_str(), wanted.c_str());
        return false;
      }
      state_slots_[leaf].push_back(
        static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it)));
    }
  }
  committed_value_.assign(manifest_leaf_count, 0.0);
  return true;
}

bool TypedForkCompositeController::build_kernel()
{
  // The nodes are already constructed (fixed storage). The binding names them; `compose` checks the
  // parent/child edges against the declaration, and the checked entry verifies every node's runtime
  // port strings before building the plan and the group.
  auto & root_object = *root_node_;
  auto & a_object = *a_node_;
  auto & b_object = *b_node_;

  const auto a_leaf = tc::make_leaf<tf::a_node, tp::contract_of_t<tf::a_ports>>(&a_object.get());
  const auto b_leaf = tc::make_leaf<tf::b_node, tp::contract_of_t<tf::b_ports>>(&b_object.get());
  const auto binding = tc::compose<tf::root_node, tp::contract_of_t<tf::root_ports>>(
    &root_object.get(), a_leaf, b_leaf);

  // The execution ORDER is a compile-time fact of the declaration, not something discovered at
  // activation: verify that the plan the kernel is about to receive agrees with it. A disagreement
  // would make every compile-time guarantee about this tree worthless, so it is fatal and named.
  const auto rows = tc::build_spec_rows(binding);
  std::string plan_reason;
  if (!hierarchical_control::static_two_phase::plan_matches_description<binding_type>(
        rows, &plan_reason))
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "The plan disagrees with the compile-time description of this tree: %s", plan_reason.c_str());
    return false;
  }

  std::shared_ptr<hierarchical_control::StagedExecutionGroup> group;
  try
  {
    group = hierarchical_control::topology_binding::create_library_group(binding);
  }
  catch (const std::invalid_argument &)
  {
    return false;
  }

  plan_names_ = rows.names;
  kernel_ = std::move(group);
  ++build_allocations;
  return true;
}

CallbackReturn TypedForkCompositeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Non-real-time setup: the loans are already indexed (`assign_interfaces()` runs just before the
  // lifecycle activation), so every allocation the kernel makes happens here and the control loop
  // stays allocation-free. Nothing is published unless BOTH steps succeed.
  if (!resolve_interface_slots()) {return CallbackReturn::FAILURE;}
  kernel_.reset();
  if (!build_kernel()) {return CallbackReturn::FAILURE;}
  return CallbackReturn::SUCCESS;
}

CallbackReturn TypedForkCompositeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Never reuse a kernel across an activation boundary: the loans may differ. The nodes themselves
  // are fixed storage and stay alive (their counters are test instrumentation).
  kernel_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TypedForkCompositeController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  kernel_.reset();
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
