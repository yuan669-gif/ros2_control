// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// A minimal ControllerInterfaceBase implementation for tests that need a real controller object.
//
// Why this exists: the compile-time topology binding stores a TYPED
// `ControllerInterfaceBase*`, not a `void*`, because converting an object pointer to the second of
// two base classes needs an address adjustment that a void* round trip loses (review R5). Tests
// that only exercise topology logic therefore still need an object that really derives from
// ControllerInterfaceBase -- an `int` placeholder no longer compiles, which is the intended
// tightening.
//
// Only the pure virtuals required to instantiate the type are implemented; every behaviour a
// topology test cares about is irrelevant here.

#ifndef HIERARCHICAL_CONTROL__TEST_CONTROLLER_STUB_HPP_
#define HIERARCHICAL_CONTROL__TEST_CONTROLLER_STUB_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hierarchical_control_test
{
/// The smallest object that satisfies ControllerInterfaceBase.
// VIRTUAL inheritance: a test controller may also derive from TypedPortsMixin, which itself
// derives from ControllerInterfaceBase. Without `virtual` that is a diamond and the
// StagedExecutionGroup sees two distinct base subobjects.
class MinimalController : public virtual controller_interface::ControllerInterfaceBase
{
public:
  explicit MinimalController(std::string name = "stub") : name_(std::move(name)) {}

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{};
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

private:
  std::string name_;
};
}  // namespace hierarchical_control_test

#endif  // HIERARCHICAL_CONTROL__TEST_CONTROLLER_STUB_HPP_
