// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__STATIC_MANIFEST_HPP_
#define HIERARCHICAL_CONTROL__STATIC_MANIFEST_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/static_topology.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/typed_ports.hpp"

namespace hierarchical_control
{
/// A COMPILE-TIME description of one controller tree: nodes, ports and the hardware interfaces the
/// tree needs.
/**
 * This is the "compile-time description layer" of the compile-time-controller plan. It is derived
 * from the TYPE of a binding (`tc::compose(...)` produces it), so it is a compile-time object: every
 * member is a `static constexpr` array, and `manifest_problem()` can be used in a `static_assert`.
 *
 * What it is NOT, deliberately:
 *
 *   * it does not contain controller instances, ROS nodes, parameters or loans. Those do not exist
 *     at compile time, and a manifest that tried to hold them could not be `constexpr`;
 *   * it does not resolve names to interface indices. The indices are a property of one ACTIVATION
 *     (the manager loans a concrete set), so they are resolved in `on_activate` and kept in the
 *     controller's own slots; baking a global index into a compile-time object would couple the
 *     controller to `ResourceManager` internals that change across activations;
 *   * `HardwareState` interfaces are the one part of a controller's interface description that is
 *     NOT a topology edge (the other end is a joint, not a node). They are declared in
 *     `TypedPorts::HardwareState` and appear here as requirements, but they never enter the
 *     `Contract` or any parent/child check.
 *
 * Usage:
 *
 *     using binding_type = decltype(tc::compose<node_root, contract>(&root, a_leaf, b_leaf));
 *     constexpr auto manifest = manifest_of_v<binding_type>;
 *     static_assert(manifest_problem(manifest).empty());
 *     for (const auto & node : manifest.nodes) { ... }
 */
namespace static_manifest
{
namespace tc = topology_contract;
namespace tp = typed_ports;
namespace st = static_topology;

/// Which port list an entry came from. Roles keep the manifest's `ports` array self-describing.
enum class PortRole
{
  state,          ///< this node publishes it (its parent's state stage reads it)
  reference,      ///< this node receives it from its parent
  actuator,       ///< hardware COMMAND interface this node writes
  for_children,   ///< reference this node writes into its children
  child_state,    ///< child state this node reads back
  hardware_state  ///< hardware STATE interface this node reads
};

constexpr std::string_view role_name(PortRole role) noexcept
{
  switch (role)
  {
    case PortRole::state: return "state";
    case PortRole::reference: return "reference";
    case PortRole::actuator: return "actuator";
    case PortRole::for_children: return "for_children";
    case PortRole::child_state: return "child_state";
    case PortRole::hardware_state: return "hardware_state";
  }
  return "?";
}

struct ManifestNode
{
  std::string_view name;
  std::string_view parent;  ///< empty for the root
};

struct ManifestPort
{
  std::string_view name;
  std::string_view owner;
  PortRole role;
};

/// The compile-time description, as plain aggregates of `static constexpr` arrays.
/**
 * `command_interfaces` and `state_interfaces` are the hardware requirements, already separated
 * because that is how a controller declares them (`command_interface_configuration()` /
 * `state_interface_configuration()`), so a controller can GENERATE those lists from here instead of
 * writing the names a second time.
 */
template <
  std::size_t NodeCount, std::size_t PortCount, std::size_t CommandCount, std::size_t StateCount>
struct StaticManifest
{
  std::array<ManifestNode, NodeCount> nodes{};
  std::array<ManifestPort, PortCount> ports{};
  std::array<std::string_view, CommandCount> command_interfaces{};
  std::array<std::string_view, StateCount> state_interfaces{};

  static constexpr std::size_t node_count = NodeCount;
  static constexpr std::size_t port_count = PortCount;
  static constexpr std::size_t command_count = CommandCount;
  static constexpr std::size_t state_count = StateCount;

  constexpr bool has_command(std::string_view name) const noexcept
  {
    for (std::size_t i = 0; i < CommandCount; ++i)
    {
      if (command_interfaces[i] == name) {return true;}
    }
    return false;
  }

  constexpr bool has_state_interface(std::string_view name) const noexcept
  {
    for (std::size_t i = 0; i < StateCount; ++i)
    {
      if (state_interfaces[i] == name) {return true;}
    }
    return false;
  }

  /// The parent name of `name`, or empty when `name` is the root or unknown.
  constexpr std::string_view parent_of(std::string_view name) const noexcept
  {
    for (std::size_t i = 0; i < NodeCount; ++i)
    {
      if (nodes[i].name == name) {return nodes[i].parent;}
    }
    return {};
  }

  /// Every port of `owner`, in declaration order, as role/name pairs (for diagnostics and tests).
  constexpr std::size_t port_count_of(std::string_view owner) const noexcept
  {
    std::size_t count = 0;
    for (std::size_t i = 0; i < PortCount; ++i)
    {
      if (ports[i].owner == owner) {++count;}
    }
    return count;
  }
};

// ---------------------------------------------------------------------------------------------
// Deriving the manifest from a binding TYPE
// ---------------------------------------------------------------------------------------------

/// The name of a parent node: `void` means "this is the root".
template <typename Parent, bool IsVoid = std::is_void_v<Parent>>
struct parent_name
{
  static constexpr std::string_view value = st::view_of(st::name_value_v<Parent>);
};

template <typename Parent>
struct parent_name<Parent, true>
{
  static constexpr std::string_view value{};
};

template <typename Name, typename Parent>
struct node_entry
{
  static constexpr std::string_view name = st::view_of(st::name_value_v<Name>);
  static constexpr std::string_view parent = parent_name<Parent>::value;
};

template <typename PortT, typename Owner, PortRole Role>
struct port_entry
{
  static constexpr std::string_view name = PortT::name();
  static constexpr std::string_view owner = st::view_of(st::name_value_v<Owner>);
  static constexpr PortRole role = Role;
};

/// One port list of one node, as `port_entry`s.
template <typename Owner, PortRole Role, typename List>
struct port_entries
{
  using type = tc::TypeList<>;
};

template <typename Owner, PortRole Role, typename... Ports>
struct port_entries<Owner, Role, tc::PortList<Ports...>>
{
  using type = tc::TypeList<port_entry<Ports, Owner, Role>...>;
};

/// The node entries of a binding subtree, pre-order (this node, then each child subtree).
template <typename Binding, typename Parent, typename ChildrenList>
struct node_entries_impl;

template <typename Binding, typename Parent, typename... Children>
struct node_entries_impl<Binding, Parent, tc::TypeList<Children...>>
{
  using type = tc::concat_t<
    tc::TypeList<node_entry<typename Binding::node_type::name_type, Parent>>,
    typename tc::concat_all<
      typename node_entries_impl<
        Children, typename Binding::node_type::name_type,
        typename Children::children_types>::type...>::type>;
};

/// The port entries of one node: empty unless the controller exposes a typed declaration.
template <typename Binding, typename = void>
struct node_port_entries
{
  using type = tc::TypeList<>;
};

template <typename Binding>
struct node_port_entries<
  Binding, std::void_t<typename Binding::controller_type::typed_ports>>
{
  using ports = typename Binding::controller_type::typed_ports;
  using owner = typename Binding::node_type::name_type;
  using type = typename tc::concat_all<
    typename port_entries<owner, PortRole::state, typename ports::state>::type,
    typename port_entries<owner, PortRole::reference, typename ports::reference>::type,
    typename port_entries<owner, PortRole::actuator, typename ports::actuators>::type,
    typename port_entries<owner, PortRole::for_children, typename ports::for_children>::type,
    typename port_entries<owner, PortRole::child_state, typename ports::child_state>::type,
    typename port_entries<owner, PortRole::hardware_state, typename ports::hardware_state>::type>::
    type;
};

/// The port entries of a binding subtree (same traversal as the node entries).
template <typename Binding, typename ChildrenList>
struct port_entries_impl;

template <typename Binding, typename... Children>
struct port_entries_impl<Binding, tc::TypeList<Children...>>
{
  using type = typename tc::concat_all<
    typename node_port_entries<Binding>::type,
    typename port_entries_impl<Children, typename Children::children_types>::type...>::type;
};

/// Keep only the entries with the given role.
template <PortRole Role, typename List>
struct filter_role
{
  using type = tc::TypeList<>;
};

template <PortRole Role, typename First, typename... Rest>
struct filter_role<Role, tc::TypeList<First, Rest...>>
{
  using tail = typename filter_role<Role, tc::TypeList<Rest...>>::type;
  using type =
    std::conditional_t<First::role == Role, tc::concat_t<tc::TypeList<First>, tail>, tail>;
};

template <typename... Entries>
constexpr std::array<ManifestNode, sizeof...(Entries)> to_node_array(tc::TypeList<Entries...>)
{
  return {ManifestNode{Entries::name, Entries::parent}...};
}

template <typename... Entries>
constexpr std::array<ManifestPort, sizeof...(Entries)> to_port_array(tc::TypeList<Entries...>)
{
  return {ManifestPort{Entries::name, Entries::owner, Entries::role}...};
}

template <typename... Entries>
constexpr std::array<std::string_view, sizeof...(Entries)> to_name_array(tc::TypeList<Entries...>)
{
  return {Entries::name...};
}

/// The manifest of a binding type.
template <typename Binding>
struct manifest_of
{
  using node_list =
    typename node_entries_impl<Binding, void, typename Binding::children_types>::type;
  using port_list =
    typename port_entries_impl<Binding, typename Binding::children_types>::type;
  using command_list = typename filter_role<PortRole::actuator, port_list>::type;
  using state_list = typename filter_role<PortRole::hardware_state, port_list>::type;

  static constexpr auto nodes = to_node_array(node_list{});
  static constexpr auto ports = to_port_array(port_list{});
  static constexpr auto command_interfaces = to_name_array(command_list{});
  static constexpr auto state_interfaces = to_name_array(state_list{});

  using type = StaticManifest<
    nodes.size(), ports.size(), command_interfaces.size(), state_interfaces.size()>;

  static constexpr type value{nodes, ports, command_interfaces, state_interfaces};
};

template <typename Binding>
inline constexpr auto manifest_of_v = manifest_of<Binding>::value;

/// The first structural problem of a manifest, or an empty view when it is well formed.
/**
 * Usable in a `static_assert`, which is the point: the description layer is only "compile time" if
 * its invariants can be checked without running anything.
 */
template <std::size_t N, std::size_t P, std::size_t C, std::size_t S>
constexpr std::string_view manifest_problem(const StaticManifest<N, P, C, S> & manifest) noexcept
{
  if (N == 0) {return "a manifest must describe at least one node";}

  std::size_t roots = 0;
  for (std::size_t i = 0; i < N; ++i)
  {
    if (manifest.nodes[i].name.empty()) {return "node names must not be empty";}
    for (std::size_t j = i + 1; j < N; ++j)
    {
      if (manifest.nodes[i].name == manifest.nodes[j].name)
      {
        return "node names must be unique";
      }
    }
    if (manifest.nodes[i].parent.empty()) {++roots;}
  }
  if (roots != 1) {return "a manifest must have exactly one root";}

  for (std::size_t i = 0; i < N; ++i)
  {
    const auto parent = manifest.nodes[i].parent;
    if (parent.empty()) {continue;}
    if (parent == manifest.nodes[i].name) {return "a node cannot be its own parent";}
    bool found = false;
    for (std::size_t j = 0; j < N; ++j)
    {
      if (manifest.nodes[j].name == parent) {found = true; break;}
    }
    if (!found) {return "every parent must name a node of the manifest";}
  }

  for (std::size_t i = 0; i < P; ++i)
  {
    if (manifest.ports[i].name.empty()) {return "port names must not be empty";}
    const auto owner = manifest.ports[i].owner;
    bool found = false;
    for (std::size_t j = 0; j < N; ++j)
    {
      if (manifest.nodes[j].name == owner) {found = true; break;}
    }
    if (!found)
    {
      // A hardware interface is owned by a joint, never by a node: those two roles are the only
      // ones allowed to point outside the tree.
      const auto role = manifest.ports[i].role;
      if (role != PortRole::actuator && role != PortRole::hardware_state)
      {
        return "every non-hardware port must be owned by a node of the manifest";
      }
    }
  }

  for (std::size_t i = 0; i < P; ++i)
  {
    for (std::size_t j = i + 1; j < P; ++j)
    {
      if (
        manifest.ports[i].role == manifest.ports[j].role &&
        manifest.ports[i].name == manifest.ports[j].name)
      {
        return "a port may appear only once per role";
      }
    }
  }
  return {};
}

/// Compile-time convenience: does this binding produce a well-formed manifest?
template <typename Binding>
constexpr bool manifest_is_well_formed() noexcept
{
  return manifest_problem(manifest_of_v<Binding>).empty();
}

/// Compare the manifest's hardware requirements against a controller's runtime declarations.
/**
 * The manifest is compiled in; `command_interface_configuration()` / `state_interface_configuration()`
 * are what the controller actually asks the manager for. A controller that GENERATES those lists from
 * the manifest cannot disagree (that is the recommended shape). A controller that hand-writes them
 * can, and this is the check to run in `on_configure`: it reports the first interface that is
 * required but not declared, or declared but not required, BY NAME, before any resource is loaned.
 */
template <std::size_t N, std::size_t P, std::size_t C, std::size_t S>
bool declaration_matches_manifest(
  const StaticManifest<N, P, C, S> & manifest, const std::vector<std::string> & command_config,
  const std::vector<std::string> & state_config, std::string * reason = nullptr)
{
  const auto fail = [reason](const std::string & message)
  {
    if (reason != nullptr) {*reason = message;}
    return false;
  };

  for (std::size_t i = 0; i < C; ++i)
  {
    const std::string_view required = manifest.command_interfaces[i];
    bool declared = false;
    for (const auto & name : command_config)
    {
      if (name == required) {declared = true; break;}
    }
    if (!declared)
    {
      return fail(
        "the manifest requires command interface '" + std::string(required) +
        "' but command_interface_configuration() does not declare it");
    }
  }
  for (const auto & name : command_config)
  {
    if (!manifest.has_command(name))
    {
      return fail(
        "command_interface_configuration() declares '" + name +
        "', which the manifest does not require");
    }
  }

  for (std::size_t i = 0; i < S; ++i)
  {
    const std::string_view required = manifest.state_interfaces[i];
    bool declared = false;
    for (const auto & name : state_config)
    {
      if (name == required) {declared = true; break;}
    }
    if (!declared)
    {
      return fail(
        "the manifest requires state interface '" + std::string(required) +
        "' but state_interface_configuration() does not declare it");
    }
  }
  for (const auto & name : state_config)
  {
    if (!manifest.has_state_interface(name))
    {
      return fail(
        "state_interface_configuration() declares '" + name +
        "', which the manifest does not require");
    }
  }
  if (reason != nullptr) {reason->clear();}
  return true;
}

}  // namespace static_manifest
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__STATIC_MANIFEST_HPP_
