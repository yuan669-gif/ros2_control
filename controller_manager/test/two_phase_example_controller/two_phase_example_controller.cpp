// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "two_phase_example_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/parameter_value.hpp"

namespace two_phase_example_controller
{
namespace
{
/// Suffix of the reference interface a node RECEIVES its target on.
constexpr char kTargetSuffix[] = "target";
/// Suffix of the reference interface a node PUBLISHES its estimate on.
constexpr char kEstimateSuffix[] = "estimate";

std::string qualified(const std::string & owner, const char * suffix)
{
  return owner + "/" + suffix;
}
}  // namespace

TwoPhaseExampleController::TwoPhaseExampleController()
: controller_interface::ChainableControllerInterface(),
  controller_interface::TwoPhaseControllerInterface()
{
}

TwoPhaseExampleController::~TwoPhaseExampleController() = default;

controller_interface::InterfaceConfiguration
TwoPhaseExampleController::command_interface_configuration() const
{
  if (
    get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    throw std::runtime_error(
      "Can not get command interface configuration until the controller is configured.");
  }

  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = command_interface_names_;
  // The child channels are ordinary command interfaces exported by the children: `target` is what
  // we write, `estimate` is what the child publishes for us to read.
  for (const auto & child : children_)
  {
    cfg.names.push_back(qualified(child, kTargetSuffix));
    cfg.names.push_back(qualified(child, kEstimateSuffix));
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
TwoPhaseExampleController::state_interface_configuration() const
{
  if (
    get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    throw std::runtime_error(
      "Can not get state interface configuration until the controller is configured.");
  }

  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = state_interface_names_;
  return cfg;
}

CallbackReturn TwoPhaseExampleController::on_init()
{
  // `update_rate` is read by the base class during configure(); make sure it exists so a deployment
  // that does not mention it still configures (value 0 = follow the manager).
  if (!get_node()->has_parameter("update_rate"))
  {
    get_node()->declare_parameter("update_rate", rclcpp::ParameterValue(0));
  }

  // Optional parameter wiring, so the SAME plugin is deployable from YAML and from a test.
  const auto read_string_array = [this](const char * name, std::vector<std::string> & out)
  {
    if (!get_node()->has_parameter(name)) {return;}
    const auto parameter = get_node()->get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      out = parameter.as_string_array();
    }
  };
  read_string_array("command_interfaces", command_interface_names_);
  read_string_array("state_interfaces", state_interface_names_);
  read_string_array("children", children_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TwoPhaseExampleController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Two exported reference interfaces: the target we receive and the estimate we publish. The base
  // class checks this size against `on_export_reference_interfaces()`.
  reference_interfaces_.assign(k_reference_count, 0.0);
  estimate_ = 0.0;
  command_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwoPhaseExampleController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Resolve every configured name to a loaned-interface index ONCE: the real-time path must not
  // build strings. A failure here is a configuration error, not a runtime one.
  std::string reason;
  if (!resolve_interfaces(&reason))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Can not activate: %s", reason.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn TwoPhaseExampleController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  actuator_index_.clear();
  state_index_.clear();
  child_target_index_.clear();
  child_estimate_index_.clear();
  return CallbackReturn::SUCCESS;
}

std::size_t TwoPhaseExampleController::find_command(const std::string & name) const
{
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (command_interfaces_[i].get_name() == name) {return i;}
  }
  return npos;
}

std::size_t TwoPhaseExampleController::find_state(const std::string & name) const
{
  for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    if (state_interfaces_[i].get_name() == name) {return i;}
  }
  return npos;
}

bool TwoPhaseExampleController::resolve_interfaces(std::string * reason)
{
  actuator_index_.clear();
  state_index_.clear();
  child_target_index_.clear();
  child_estimate_index_.clear();

  for (const auto & name : command_interface_names_)
  {
    const auto index = find_command(name);
    if (index == npos)
    {
      if (reason != nullptr) {*reason = "command interface '" + name + "' was not loaned";}
      return false;
    }
    actuator_index_.push_back(index);
  }
  for (const auto & name : state_interface_names_)
  {
    const auto index = find_state(name);
    if (index == npos)
    {
      if (reason != nullptr) {*reason = "state interface '" + name + "' was not loaned";}
      return false;
    }
    state_index_.push_back(index);
  }
  for (const auto & child : children_)
  {
    const auto target = find_command(qualified(child, kTargetSuffix));
    const auto estimate = find_command(qualified(child, kEstimateSuffix));
    if (target == npos || estimate == npos)
    {
      if (reason != nullptr)
      {
        *reason = "child '" + child + "' does not export both 'target' and 'estimate'";
      }
      return false;
    }
    child_target_index_.push_back(target);
    child_estimate_index_.push_back(estimate);
  }
  return true;
}

void TwoPhaseExampleController::ingest() noexcept
{
  // One increment per cycle in EITHER execution path, because both call `ingest()` exactly once.
  ++cycle_;

  const double previous = estimate_;
  if (children_.empty())
  {
    if (use_hardware_state_override_)
    {
      estimate_ = hardware_state_override_;
    }
    else
    {
      double sum = 0.0;
      for (const auto index : state_index_) {sum += state_interfaces_[index].get_value();}
      estimate_ = state_index_.empty()
                    ? 0.0
                    : sum / static_cast<double>(state_index_.size());
    }
  }
  else
  {
    double sum = 0.0;
    for (const auto index : child_estimate_index_)
    {
      sum += command_interfaces_[index].get_value();
    }
    const double mean = sum / static_cast<double>(child_estimate_index_.size());
    // The measurement instrument records what the children had PUBLISHED when they were last read.
    last_child_estimate_seen_ = mean;
    estimate_ = 0.5 * previous + 0.5 * mean;
  }

  // Publish the estimate. In the two-phase path this happens in `update_phase`, i.e. BEFORE the
  // parent's own `update_phase` because the manager walks the list backward.
  reference_interfaces_[k_estimate] =
    cycle_stamp_mode_ ? static_cast<double>(cycle_stamp_) : estimate_;
}

void TwoPhaseExampleController::compute_and_write() noexcept
{
  // In the two-phase path this is `handle_phase`, i.e. the moment the node CONSUMES its reference.
  last_target_seen_ = reference_interfaces_[k_target];
  command_ = reference_interfaces_[k_target] - estimate_;
  const double written = cycle_stamp_mode_ ? static_cast<double>(cycle_stamp_) : command_;
  for (const auto index : actuator_index_) {command_interfaces_[index].set_value(written);}
  for (const auto index : child_target_index_) {command_interfaces_[index].set_value(written);}
}

controller_interface::return_type TwoPhaseExampleController::update_phase(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period) noexcept
{
  ++update_phase_calls_;
  update_phase_entered_.store(true, std::memory_order_release);
  while (hold_update_phase_.load(std::memory_order_acquire))
  {
    std::this_thread::yield();
  }
  last_period_ns_ = period.nanoseconds();
  if (fail_next_update_phase_)
  {
    fail_next_update_phase_ = false;
    return controller_interface::return_type::ERROR;
  }
  ingest();
  return controller_interface::return_type::OK;
}

controller_interface::return_type TwoPhaseExampleController::handle_phase(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period) noexcept
{
  ++handle_phase_calls_;
  last_period_ns_ = period.nanoseconds();
  compute_and_write();
  return controller_interface::return_type::OK;
}

controller_interface::return_type TwoPhaseExampleController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  ++native_update_calls_;
  last_period_ns_ = period.nanoseconds();
  ingest();
  compute_and_write();
  return controller_interface::return_type::OK;
}

controller_interface::return_type
TwoPhaseExampleController::update_reference_from_subscribers()
{
  // No subscriber in this example: the reference is either written by a parent (chained mode) or set
  // externally through `set_external_reference()`.
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
TwoPhaseExampleController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(k_reference_count);
  interfaces.emplace_back(
    get_node()->get_name(), kTargetSuffix, &reference_interfaces_[k_target]);
  interfaces.emplace_back(
    get_node()->get_name(), kEstimateSuffix, &reference_interfaces_[k_estimate]);
  return interfaces;
}

void TwoPhaseExampleController::set_command_interface_names(std::vector<std::string> names)
{
  command_interface_names_ = std::move(names);
}

void TwoPhaseExampleController::set_state_interface_names(std::vector<std::string> names)
{
  state_interface_names_ = std::move(names);
}

void TwoPhaseExampleController::set_children(std::vector<std::string> children)
{
  children_ = std::move(children);
}

double TwoPhaseExampleController::estimate() const noexcept {return estimate_;}

double TwoPhaseExampleController::command() const noexcept {return command_;}

std::int64_t TwoPhaseExampleController::update_phase_calls() const noexcept
{
  return update_phase_calls_;
}

std::int64_t TwoPhaseExampleController::handle_phase_calls() const noexcept
{
  return handle_phase_calls_;
}

std::int64_t TwoPhaseExampleController::native_update_calls() const noexcept
{
  return native_update_calls_;
}

std::int64_t TwoPhaseExampleController::last_period_ns() const noexcept {return last_period_ns_;}

void TwoPhaseExampleController::set_external_reference(double value) noexcept
{
  reference_interfaces_[k_target] = value;
}

void TwoPhaseExampleController::set_hardware_state(double value) noexcept
{
  hardware_state_override_ = value;
  use_hardware_state_override_ = true;
}

void TwoPhaseExampleController::fail_next_update_phase() noexcept {fail_next_update_phase_ = true;}

void TwoPhaseExampleController::hold_update_phase(bool hold) noexcept
{
  if (hold) {update_phase_entered_.store(false, std::memory_order_release);}
  hold_update_phase_.store(hold, std::memory_order_release);
}

bool TwoPhaseExampleController::update_phase_entered() const noexcept
{
  return update_phase_entered_.load(std::memory_order_acquire);
}

void TwoPhaseExampleController::set_cycle_stamp_mode(bool enabled) noexcept
{
  cycle_stamp_mode_ = enabled;
}

void TwoPhaseExampleController::set_cycle_stamp(std::int64_t stamp) noexcept
{
  cycle_stamp_ = stamp;
}

double TwoPhaseExampleController::last_target_seen() const noexcept {return last_target_seen_;}

double TwoPhaseExampleController::last_child_estimate_seen() const noexcept
{
  return last_child_estimate_seen_;
}

}  // namespace two_phase_example_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  two_phase_example_controller::TwoPhaseExampleController,
  controller_interface::ChainableControllerInterface)
