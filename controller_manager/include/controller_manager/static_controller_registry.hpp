// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__STATIC_CONTROLLER_REGISTRY_HPP_
#define CONTROLLER_MANAGER__STATIC_CONTROLLER_REGISTRY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"

namespace controller_manager
{

/// Controllers COMPILED INTO THE BINARY, addressable by the same type string a pluginlib controller
/// uses.
/**
 * Why this exists. `ControllerManager::load_controller(name, type)` resolves `type` through
 * pluginlib, so a controller that is built into the executable (or into a library the executable
 * already links) cannot be named by a type string, and therefore cannot be reached by the ordinary
 * configuration path: a YAML `type:` field, the spawner, and `load_controller(name)` reading
 * `<name>.type`.
 *
 * The registry closes that gap WITHOUT creating a second code path. A registered type is instantiated
 * by a factory and then handed to the very same `add_controller_impl()`, so it goes through the same
 * controller list, the same `configure`/`activate` lifecycle, the same interface claiming and the same
 * admission checks (staged group membership, two-phase admission, cross-mode edges, duplicate
 * instances). The only difference is where the object comes from.
 *
 * What it is NOT: a way to make controllers global objects. Each `load_controller()` call creates a
 * NEW instance with its own ROS node and its own lifecycle, so two managers (or two controllers with
 * the same type) never share state -- which is what the compile-time-controller plan's isolation
 * requirement demands. A factory, not a singleton instance, is exactly what keeps that true.
 *
 * The optional manifest accessors expose the compile-time description of a registered type when the
 * type declares one (`ControllerT::manifest`, see `hierarchical_control/static_manifest.hpp`), so a
 * tool can enumerate a compiled-in controller's declared topology and hardware requirements without
 * constructing it.
 *
 * Thread-safety: registration is expected at start-up, before controllers are loaded; lookups are
 * read-only afterwards and take no lock (there is no concurrent `load_controller()` in the supported
 * configuration constraint: controllers are loaded from the non-real-time thread).
 */
class StaticControllerRegistry
{
public:
  using SharedPtr = std::shared_ptr<StaticControllerRegistry>;
  using Factory = std::function<controller_interface::ControllerInterfaceBaseSharedPtr()>;

  /// Detect a type-level `manifest` (a compile-time description) without requiring one.
  template <typename ControllerT, typename = void>
  struct manifest_traits
  {
    static constexpr bool available = false;
  };

  template <typename ControllerT>
  struct manifest_traits<ControllerT, std::void_t<decltype(ControllerT::manifest)>>
  {
    static constexpr bool available = true;
    static std::vector<std::string> command_interfaces()
    {
      std::vector<std::string> out;
      for (const auto & name : ControllerT::manifest.command_interfaces) {out.emplace_back(name);}
      return out;
    }
    static std::vector<std::string> state_interfaces()
    {
      std::vector<std::string> out;
      for (const auto & name : ControllerT::manifest.state_interfaces) {out.emplace_back(name);}
      return out;
    }
  };

  /// Register `ControllerT` under the type string `type`.
  /**
   * Throws `std::invalid_argument` on an empty type or a duplicate registration: silently replacing a
   * type would make the configuration depend on registration order.
   */
  template <typename ControllerT>
  void add(const std::string & type)
  {
    static_assert(
      std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
      "a static controller type must derive from ControllerInterfaceBase");
    static_assert(
      std::is_default_constructible_v<ControllerT>,
      "a static controller type must be default-constructible: the factory creates a new instance "
      "per load_controller() call");

    Entry entry;
    entry.create = []() -> controller_interface::ControllerInterfaceBaseSharedPtr
    {return std::make_shared<ControllerT>();};
    entry.has_manifest = manifest_traits<ControllerT>::available;
    if constexpr (manifest_traits<ControllerT>::available)
    {
      entry.command_interfaces = manifest_traits<ControllerT>::command_interfaces();
      entry.state_interfaces = manifest_traits<ControllerT>::state_interfaces();
    }
    insert(type, std::move(entry));
  }

  /// Register an explicitly supplied factory (for a type whose construction needs arguments).
  void add_factory(const std::string & type, Factory factory);

  bool has(const std::string & type) const;
  /// A NEW instance, or nullptr when `type` is not registered.
  controller_interface::ControllerInterfaceBaseSharedPtr create(const std::string & type) const;
  /// Registered type strings, ordered (deterministic diagnostics).
  std::vector<std::string> types() const;

  bool has_manifest(const std::string & type) const;
  std::vector<std::string> command_interfaces(const std::string & type) const;
  std::vector<std::string> state_interfaces(const std::string & type) const;

private:
  struct Entry
  {
    Factory create;
    bool has_manifest = false;
    std::vector<std::string> command_interfaces;
    std::vector<std::string> state_interfaces;
  };

  void insert(const std::string & type, Entry entry);

  std::map<std::string, Entry> entries_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__STATIC_CONTROLLER_REGISTRY_HPP_
