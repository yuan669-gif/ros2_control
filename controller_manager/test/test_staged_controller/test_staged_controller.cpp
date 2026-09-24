// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "test_staged_controller/test_staged_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace test_staged_controller
{

class TestStagedController::Sink : public hierarchical_control::StagedCommandSink
{
public:
  explicit Sink(TestStagedController * owner) : owner_(owner) {}

  bool commit(const double * values, std::size_t size) noexcept override
  {
    ++owner_->commit_calls_;
    if (owner_->fail_commit_) {return false;}
    auto & interfaces = owner_->command_interfaces_;
    const auto count = std::min(size, interfaces.size());
    for (std::size_t i = 0; i < count; ++i)
    {
      interfaces[i].set_value(values[i]);
    }
    if (owner_->record_diagnostics_) {owner_->committed_.assign(values, values + size);}
    return true;
  }

private:
  TestStagedController * owner_;
};

class TestStagedController::Source : public hierarchical_control::StagedReferenceSource
{
public:
  explicit Source(TestStagedController * owner) : owner_(owner) {}

  bool read(
    std::uint64_t cycle, std::int64_t /*now_ns*/, double * values,
    std::size_t size) noexcept override
  {
    ++owner_->source_calls;
    owner_->last_source_cycle = cycle;
    if (owner_->record_diagnostics_) {owner_->last_reference.assign(values, values + size);}
    for (std::size_t i = 0; i < size; ++i)
    {
      values[i] = owner_->external_reference_;
    }
    return true;
  }

private:
  TestStagedController * owner_;
};

TestStagedController::TestStagedController()
: sink_(std::make_unique<Sink>(this)), source_(std::make_unique<Source>(this))
{
  cmd_iface_cfg_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_iface_cfg_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
}

TestStagedController::~TestStagedController() = default;

controller_interface::InterfaceConfiguration
TestStagedController::command_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return cmd_iface_cfg_;
  }
  throw std::runtime_error(
    "Can not get command interface configuration until the controller is configured.");
}

controller_interface::InterfaceConfiguration
TestStagedController::state_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return state_iface_cfg_;
  }
  throw std::runtime_error(
    "Can not get state interface configuration until the controller is configured.");
}

CallbackReturn TestStagedController::on_init() {return CallbackReturn::SUCCESS;}

CallbackReturn TestStagedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reference_interfaces_.assign(reference_interface_names_.size(), 0.0);
  committed_.assign(actuator_ports_.size(), 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestStagedController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestStagedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestStagedController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  reference_interfaces_.clear();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
TestStagedController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interface_names_.size());
  for (std::size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }
  return interfaces;
}

controller_interface::return_type TestStagedController::update_reference_from_subscribers()
{
  ++legacy_update_calls;
  if (native_mode_ && !reference_interfaces_.empty())
  {
    // A non-chained (root) controller takes its reference from the external snapshot.
    reference_interfaces_[0] = external_reference_;
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type TestStagedController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ++legacy_update_calls;
  if (two_phase_legacy_)
  {
    if (update_phase(time, period) != controller_interface::return_type::OK)
    {
      return controller_interface::return_type::ERROR;
    }
    return handle_phase(time, period);
  }
  if (!native_mode_) {return controller_interface::return_type::OK;}

  // Native chaining baseline: every node reads the raw hardware snapshot itself and derives its
  // own local estimate; there is no bottom-up child-state path and no group commit.
  double raw = native_bias_;
  for (const auto & state : state_interfaces_) {raw += state.get_value();}
  const double reference = reference_interfaces_.empty() ? 0.0 : reference_interfaces_[0];
  double command = reference - native_factor_ * raw;
  if (fail_command_)
  {
    ++native_command_failures;
    return controller_interface::return_type::ERROR;
  }
  if (emit_nan_) {command = std::numeric_limits<double>::quiet_NaN();}
  for (auto & interface : command_interfaces_) {interface.set_value(command);}
  if (!command_interfaces_.empty()) {++native_command_writes;}
  last_reference = reference_interfaces_;
  last_actuator.assign(1, command);
  return controller_interface::return_type::OK;
}

std::vector<std::string> TestStagedController::staged_state_ports() const
{
  std::vector<std::string> names;
  const auto count = state_iface_cfg_.names.empty() ? 1u : state_iface_cfg_.names.size();
  names.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    names.push_back("state_" + std::to_string(i));
  }
  return names;
}

std::vector<std::string> TestStagedController::staged_reference_ports() const
{
  return reference_interface_names_;
}

std::vector<std::string> TestStagedController::staged_actuator_ports() const
{
  return actuator_ports_;
}

hierarchical_control::StagedCommandSink * TestStagedController::staged_command_sink() noexcept
{
  return sink_.get();
}

hierarchical_control::StagedReferenceSource *
TestStagedController::staged_reference_source() noexcept
{
  return source_.get();
}

controller_interface::return_type TestStagedController::update_state_stage(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
  const hierarchical_control::StagedContext & context,
  const hierarchical_control::StagedInputView & children,
  hierarchical_control::StagedValueWriter state) noexcept
{
  ++state_calls;
  last_state_cycle = context.cycle;
  if (sequence != nullptr) {sequence_at_state = ++(*sequence);}
  if (fail_state_)
  {
    state.set_fault(0x51u);
    return controller_interface::return_type::ERROR;
  }
  if (children.size() == 0)
  {
    const auto count = std::min(state.size(), state_interfaces_.size());
    for (std::size_t i = 0; i < count; ++i)
    {
      state[i] = state_interfaces_[i].get_value() + state_offset_;
    }
    for (std::size_t i = count; i < state.size(); ++i) {state[i] = state_offset_;}
  }
  else
  {
    double sum = 0.0;
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p) {sum += children[c][p];}
    }
    if (state.size() > 0) {state[0] = 2.0 * sum;}
    for (std::size_t i = 1; i < state.size(); ++i) {state[i] = 0.0;}
  }
  if (record_diagnostics_) {last_state.assign(state.data(), state.data() + state.size());}
  return controller_interface::return_type::OK;
}

controller_interface::return_type TestStagedController::update_command_stage(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
  const hierarchical_control::StagedContext & context,
  const hierarchical_control::StagedValueView & state,
  const hierarchical_control::StagedValueView & reference,
  const hierarchical_control::StagedReferenceWriter & children,
  hierarchical_control::StagedValueWriter actuators) noexcept
{
  ++command_calls;
  last_command_cycle = context.cycle;
  if (sequence != nullptr) {sequence_at_command = ++(*sequence);}
  if (fail_command_)
  {
    actuators.set_fault(0x52u);
    return controller_interface::return_type::ERROR;
  }
  const double state_value = state.size() > 0 ? state[0] : 0.0;
  const double reference_value = reference.size() > 0 ? reference[0] : 0.0;
  const double command = reference_value - state_value;
  if (emit_nan_)
  {
    for (std::size_t c = 0; c < children.size(); ++c)
    {
      for (std::size_t p = 0; p < children[c].size(); ++p)
      {
        children[c][p] = std::numeric_limits<double>::quiet_NaN();
      }
    }
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
      actuators[i] = std::numeric_limits<double>::quiet_NaN();
    }
    return controller_interface::return_type::OK;
  }
  for (std::size_t c = 0; c < children.size(); ++c)
  {
    for (std::size_t p = 0; p < children[c].size(); ++p) {children[c][p] = command;}
  }
  for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = command;}
  if (record_diagnostics_)
  {
    last_reference.assign(reference.data(), reference.data() + reference.size());
    last_actuator.assign(actuators.data(), actuators.data() + actuators.size());
  }
  return controller_interface::return_type::OK;
}

void TestStagedController::set_command_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  cmd_iface_cfg_ = cfg;
}

void TestStagedController::set_state_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  state_iface_cfg_ = cfg;
}

void TestStagedController::set_reference_interface_names(const std::vector<std::string> & names)
{
  reference_interface_names_ = names;
}

void TestStagedController::set_actuator_ports(const std::vector<std::string> & names)
{
  actuator_ports_ = names;
}

void TestStagedController::set_external_reference(double value) {external_reference_ = value;}

void TestStagedController::set_fail_state(bool value) {fail_state_ = value;}

void TestStagedController::set_fail_command(bool value) {fail_command_ = value;}

void TestStagedController::set_fail_handle(bool value) {fail_handle_ = value;}

void TestStagedController::set_fail_update(bool value) {fail_update_ = value;}

void TestStagedController::set_fail_commit(bool value) {fail_commit_ = value;}

void TestStagedController::set_emit_nan(bool value) {emit_nan_ = value;}

void TestStagedController::set_state_offset(double value) {state_offset_ = value;}

void TestStagedController::set_record_diagnostics(bool enabled)
{
  record_diagnostics_ = enabled;
}

controller_interface::return_type TestStagedController::update_phase(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) noexcept
{
  ++update_phase_calls;
  if (fail_update_) {return controller_interface::return_type::ERROR;}
  const double child_estimate =
    two_phase_child_ ? two_phase_child_->two_phase_estimate_ : two_phase_input_;
  // Deliberately non-re-derivable: the node's estimate depends on its own history plus the
  // child's CURRENT estimate, so a parent cannot reconstruct it from raw hardware.
  two_phase_estimate_ = 0.5 * two_phase_estimate_ + 0.5 * child_estimate;
  return controller_interface::return_type::OK;
}

controller_interface::return_type TestStagedController::handle_phase(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) noexcept
{
  ++handle_phase_calls;
  if (fail_handle_) {return controller_interface::return_type::ERROR;}
  const double reference =
    reference_interfaces_.empty() ? external_reference_ : reference_interfaces_[0];
  two_phase_command_ = reference - two_phase_estimate_;
  for (auto & interface : command_interfaces_) {interface.set_value(two_phase_command_);}
  return controller_interface::return_type::OK;
}

void TestStagedController::set_two_phase_legacy(bool enabled) {two_phase_legacy_ = enabled;}

void TestStagedController::set_two_phase_child(TestStagedController * child)
{
  two_phase_child_ = child;
}

void TestStagedController::set_two_phase_input(double value) {two_phase_input_ = value;}

double TestStagedController::two_phase_estimate() const {return two_phase_estimate_;}

double TestStagedController::two_phase_command() const {return two_phase_command_;}

void TestStagedController::set_native_mode(bool enabled, double factor, double bias)
{
  native_mode_ = enabled;
  native_factor_ = factor;
  native_bias_ = bias;
}

std::size_t TestStagedController::commit_calls() const {return commit_calls_;}

double TestStagedController::committed_value() const
{
  if (committed_.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  return committed_.front();
}

std::size_t TestStagedController::command_interface_count() const
{
  return command_interfaces_.size();
}

std::size_t TestStagedController::state_interface_count() const
{
  return state_interfaces_.size();
}

double TestStagedController::command_interface_value() const
{
  if (command_interfaces_.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  return command_interfaces_.front().get_value();
}

}  // namespace test_staged_controller
