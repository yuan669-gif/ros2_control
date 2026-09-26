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
 * Thread-safety (review item P2-2, now ENFORCED rather than documented): the type set is mutable
 * only during start-up. `ControllerManager` freezes the registry it installed as soon as it has
 * loaded a controller, and `add()`/`add_factory()` throw `std::logic_error` afterwards, so a
 * concurrent `load_controller()` can not race a registration through the `std::map`. Lookups are
 * read-only and take no lock; they are safe once the registry is frozen, which is exactly the state
 * a manager that is loading controllers is in.
 */
class StaticControllerRegistry
{
public:
  using SharedPtr = std::shared_ptr<StaticControllerRegistry>;
  using Factory = std::function<controller_interface::ControllerInterfaceBaseSharedPtr()>;

  /// A compile-time interface description supplied by the CALLER, for a factory that has no type to
  /// read a manifest from (review item P2-3).
  /**
   * `add<ControllerT>()` derives this from `ControllerT::manifest`. A parameterised factory
   * (`add_factory(type, factory, descriptor)`) has no such type, so it states the description
   * instead: a compiled-in controller that a static tool cannot enumerate would only be half
   * compiled-in.
   */
  struct ManifestDescriptor
  {
    std::vector<std::string> command_interfaces;
    std::vector<std::string> state_interfaces;
  };

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
   * type would make the configuration depend on registration order. Throws `std::logic_error` when
   * the registry is frozen.
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
  /**
   * No manifest: a factory alone does not say what the type declares, so `has_manifest(type)` stays
   * false. Prefer the two overloads below, which keep the description attached (P2-3).
   */
  void add_factory(const std::string & type, Factory factory);

  /// Factory for `ControllerT` plus the manifest THAT TYPE declares.
  /**
   * Same description `add<ControllerT>()` derives, for a type that needs construction arguments --
   * for example one that reads its ports from a compile-time binding:
   *
   * \code
   * registry->add_factory<MyFork>("my_fork", [] {auto c = std::make_shared<MyFork>();
   *                                               c->use_binding(); return c;});
   * \endcode
   *
   * Fails to compile when `ControllerT` declares no `manifest`, because the point of this overload
   * is the type's own description; use the descriptor overload to state one by hand.
   */
  template <typename ControllerT>
  void add_factory(const std::string & type, Factory factory)
  {
    static_assert(
      manifest_traits<ControllerT>::available,
      "ControllerT declares no `manifest`; use add_factory(type, factory, descriptor) to supply "
      "the compile-time description explicitly");
    if constexpr (manifest_traits<ControllerT>::available)
    {
      ManifestDescriptor descriptor;
      descriptor.command_interfaces = manifest_traits<ControllerT>::command_interfaces();
      descriptor.state_interfaces = manifest_traits<ControllerT>::state_interfaces();
      add_factory(type, std::move(factory), std::move(descriptor));
    }
  }

  /// Factory plus an explicit compile-time description (hand-written or generated).
  void add_factory(const std::string & type, Factory factory, ManifestDescriptor manifest);

  /// Seal the type set. Idempotent; `add`/`add_factory` throw `std::logic_error` afterwards.
  void freeze();
  bool frozen() const noexcept {return frozen_;}

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
  bool frozen_ = false;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__STATIC_CONTROLLER_REGISTRY_HPP_
