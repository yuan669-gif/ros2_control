// Copyright 2026
// Licensed under the Apache License, Version 2.0.
//
// Joining a compile-time topology to the runtime execution group, with ownership checking.
//
// WHY THIS EXISTS
// ---------------
// The runtime kernel (StagedExecutionGroup::create_library) takes a Spec of three parallel vectors:
// names, instances, parents. A caller writes them by hand today, so the compile-time facts
// (static_topology.hpp, dimensional_interfaces.hpp) and the runtime plan can drift apart.
//
// This header lets ONE `BoundNode` chain carry both: the compile-time checks, and the runtime Spec
// rows (via `build_spec_rows`).
//
// THE OWNERSHIP INVARIANT
// -----------------------
// ros2_control names interfaces "<owner>/<local>". That convention is what makes the defect this
// project measured detectable at compile time:
//
//   controller_manager/test/test_upstream_ordering.cpp declares a state edge on
//   "ord_child/state" although NOTHING exports it, and configure_controller accepts it silently
//   (doc/BIDIRECTIONAL_EDGE_ANALYSIS.md section 9).
//
// The invariant enforced here is:
//
//   (i)  every referenced port name has the form "<owner>/<local>"; and
//   (ii) every <owner> names a controller that exists in the topology.
//
// Together these reject the measured defect: "ord_child/state" is acceptable only if "ord_child"
// is a controller in the topology. A typo in the owner, or a reference to a controller outside the
// group, fails to compile.
//
// Note carefully what this does NOT do: it does not verify that the OWNER itself exports the port.
// That depends on each controller's runtime `export_reference_interfaces()`, which is not a
// compile-time property when plugins are loaded dynamically. So this is a necessary, not
// sufficient, condition -- the runtime checker remains in place.
//
// The DIMENSION of each port is carried in its type (dimensional_interfaces.hpp), so a port whose
// owner exists but whose dimension is wrong is caught by the dimensional layer rather than here.
//
// SCOPE
// -----
// Statically declared topologies only. YAML / pluginlib topologies are unaffected.
//
// See doc/TOPOLOGY_CONTRACT_JOIN.md.

#ifndef HIERARCHICAL_CONTROL_TOPOLOGY_CONTRACT_HPP
#define HIERARCHICAL_CONTROL_TOPOLOGY_CONTRACT_HPP

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/static_topology.hpp"

namespace hierarchical_control
{
namespace topology_contract
{
namespace st = static_topology;
namespace dim = dimensions;

// ---------------------------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------------------------

/// A port reference: a compile-time NAME (of the form "<owner>/<local>") and a DIMENSION.
template <typename PortName, typename Dim>
struct Port
{
  using name_type = PortName;
  using dimension = Dim;

  static constexpr std::string_view name()
  {
    return st::view_of(st::name_value_v<PortName>);
  }
};

/// Do two ports have the same name and the same physical dimension?
///
/// This is the comparison that neither the kernel (strings only) nor a plain name check can make.
template <typename PortA, typename PortB>
inline constexpr bool same_port_v =
  (PortA::name() == PortB::name()) &&
  dim::same_dimension_v<typename PortA::dimension, typename PortB::dimension>;

/// The "owner" part of "<owner>/<local>", or an empty view if there is no separator.
constexpr std::string_view owner_of(std::string_view qualified) noexcept
{
  const std::size_t slash = qualified.find('/');
  if (slash == std::string_view::npos) {return std::string_view{};}
  return qualified.substr(0, slash);
}

// ---------------------------------------------------------------------------------------------
// Port lists and contracts
// ---------------------------------------------------------------------------------------------

/// A compile-time list of ports, usable as a list of name types or of `Port` types.
template <typename... Items>
struct TypeList
{
  static constexpr std::size_t count = sizeof...(Items);
};

/// A list of `Port` types, exposing their names for the ownership check.
template <typename... Ports>
struct PortList
{
  static constexpr std::size_t count = sizeof...(Ports);

  static constexpr std::array<std::string_view, sizeof...(Ports)> names = {Ports::name()...};

  static constexpr bool contains_name(std::string_view candidate) noexcept
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      if (names[i] == candidate) {return true;}
    }
    return false;
  }
};

/// A controller's static interface contract, split by direction from ITS OWN point of view:
///   Produced -- ports it writes (its actuator commands, or the reference interfaces it exports)
///   Consumed -- ports it reads  (its reference from its parent, or state from its children)
///
/// Every entry is a `Port` and therefore carries a dimension.
template <typename Produced, typename Consumed>
struct Contract
{
  static_assert(std::is_class_v<Produced>, "topology_contract: Produced must be a PortList");
  static_assert(std::is_class_v<Consumed>, "topology_contract: Consumed must be a PortList");

  using produced = Produced;
  using consumed = Consumed;

  static constexpr std::size_t produced_count = Produced::count;
  static constexpr std::size_t consumed_count = Consumed::count;
};

template <typename C, typename = void>
struct is_contract : std::false_type
{
};

template <typename P, typename C>
struct is_contract<Contract<P, C>, void> : std::true_type
{
};

template <typename C>
inline constexpr bool is_contract_v = is_contract<C>::value;

// ---------------------------------------------------------------------------------------------
// Ownership checking
// ---------------------------------------------------------------------------------------------

/// Is `PortT`'s name of the form "<owner>/..." with <owner> among `Names...`?
template <typename PortT, typename... Names>
constexpr bool port_owner_is_known() noexcept
{
  const std::string_view qualified = PortT::name();
  const std::string_view owner = owner_of(qualified);
  if (owner.empty()) {return false;}
  const std::array<std::string_view, sizeof...(Names)> owners = {
    st::view_of(st::name_value_v<Names>)...};
  for (std::size_t i = 0; i < owners.size(); ++i)
  {
    if (owners[i] == owner) {return true;}
  }
  return false;
}

template <typename PortsList, typename... Names>
struct list_owners_are_known;

template <typename... Ports, typename... Names>
struct list_owners_are_known<PortList<Ports...>, Names...>
{
  static constexpr bool value = (port_owner_is_known<Ports, Names...>() && ...);
};

/// Every port a contract mentions -- produced or consumed -- must be owned by one of `Names...`.
template <typename ContractT, typename... Names>
struct contract_owners_are_known
  : std::bool_constant<
      list_owners_are_known<typename ContractT::produced, Names...>::value &&
      list_owners_are_known<typename ContractT::consumed, Names...>::value>
{
};

// ---------------------------------------------------------------------------------------------
// Bound topology
// ---------------------------------------------------------------------------------------------

/// Safe extraction of a child binding's members; the void case must never instantiate `void::...`.
template <typename Next>
struct next_traits
{
  static constexpr bool present = true;
  using node_type = typename Next::node_type;
  using contract_type = typename Next::contract_type;
};

template <>
struct next_traits<void>
{
  static constexpr bool present = false;
  using node_type = void;
  using contract_type = void;
};

/// Optional storage for the child binding: empty when there is no child, so a leaf carries no
/// extra state (empty base optimisation).
template <typename Next>
struct next_storage
{
  Next next{};
};

template <>
struct next_storage<void>
{
};

/// A topology node bound to its port contract and to its child (if any).
template <typename ControllerT, typename Node, typename ContractT, typename Next = void>
struct BoundNode : next_storage<Next>
{
  using controller_type = ControllerT;
  using node_type = Node;
  using contract_type = ContractT;
  using next_type = Next;

  static_assert(
    std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
    "topology_contract: the bound controller type must derive from ControllerInterfaceBase");

  static_assert(
    is_contract_v<ContractT>, "topology_contract: BoundNode needs a Contract for its ports");
  static_assert(
    st::is_valid_node_v<Node>,
    "topology_contract: BoundNode needs a well-formed static_topology node (most likely a cycle)");

  // Typed, NOT erased to void*: converting to a second base needs an address adjustment that a
  // void* round trip would lose (review R5).
  controller_interface::ControllerInterfaceBase * instance = nullptr;

  static constexpr std::string_view name() { return st::node_name<Node>(); }

  static constexpr bool has_child = next_traits<Next>::present;
};

/// Compose a parent binding on top of an existing child binding.
///
/// The controller type is deduced from the argument, so the binding keeps a correctly adjusted
/// `ControllerInterfaceBase*` and never sees a `void*`.
template <typename Node, typename ContractT, typename ControllerT, typename ChildBinding>
BoundNode<ControllerT, Node, ContractT, ChildBinding> compose(
  ControllerT * parent_instance, const ChildBinding & child)
{
  static_assert(
    std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
    "topology_contract: compose needs a controller deriving from ControllerInterfaceBase");
  BoundNode<ControllerT, Node, ContractT, ChildBinding> binding;
  binding.instance = static_cast<controller_interface::ControllerInterfaceBase *>(parent_instance);
  binding.next = child;
  return binding;
}

/// Compose a leaf binding (no child).
template <typename Node, typename ContractT, typename ControllerT>
BoundNode<ControllerT, Node, ContractT, void> make_leaf(ControllerT * instance)
{
  static_assert(
    std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
    "topology_contract: make_leaf needs a controller deriving from ControllerInterfaceBase");
  BoundNode<ControllerT, Node, ContractT, void> binding;
  binding.instance = static_cast<controller_interface::ControllerInterfaceBase *>(instance);
  return binding;
}

// ---------------------------------------------------------------------------------------------
// Ownership verification over a whole binding chain
// ---------------------------------------------------------------------------------------------

/// Prepend an item to a TypeList.
template <typename Item, typename List>
struct prepend;

template <typename Item, typename... Items>
struct prepend<Item, TypeList<Items...>>
{
  using type = TypeList<Item, Items...>;
};

/// Collect the node name types of a binding chain into a `TypeList`.
template <typename Binding>
struct binding_owner_types
{
  using type = typename prepend<
    typename Binding::node_type::name_type,
    typename binding_owner_types<typename Binding::next_type>::type>::type;
};

template <>
struct binding_owner_types<void>
{
  using type = TypeList<>;
};

/// Apply the ownership check for one node against a given owner list.
template <typename Binding, typename OwnerList>
struct node_owners_ok;

template <typename Binding, typename... Names>
struct node_owners_ok<Binding, TypeList<Names...>>
  : contract_owners_are_known<typename Binding::contract_type, Names...>
{
};

/// Recurse over the chain: every node's ports must be owned by some controller in the topology.
/// The void case terminates the recursion.
template <typename Binding, typename OwnerList>
struct binding_owners_ok
  : std::bool_constant<
      node_owners_ok<Binding, OwnerList>::value &&
      binding_owners_ok<typename Binding::next_type, OwnerList>::value>
{
};

template <typename OwnerList>
struct binding_owners_ok<void, OwnerList> : std::true_type
{
};

/// The public check: fails to compile when a declared port names an owner outside the topology.
template <typename Binding>
constexpr void require_ports_are_owned()
{
  static_assert(
    binding_owners_ok<Binding, typename binding_owner_types<Binding>::type>::value,
    "topology_contract: OWNERSHIP VIOLATION -- a declared port does not belong to any controller "
    "in this topology. Port names must be \"<owner>/<local>\" where <owner> is a controller in the "
    "group. This is the defect class upstream ros2_control accepts silently (a declared interface "
    "that nothing in the group exports); here it is a compile error");
}

// ---------------------------------------------------------------------------------------------
// Runtime spec rows
// ---------------------------------------------------------------------------------------------

/// The runtime shape produced from a compile-time binding. Deliberately not
/// StagedExecutionGroup::Spec, so this header stays independent of the kernel.
struct SpecRows
{
  std::vector<std::string> names;
  /// Typed base pointers, never void*: see the pointer-safety note in topology_binding.hpp.
  std::vector<controller_interface::ControllerInterfaceBase *> instances;
  std::vector<std::string> parents;
};

template <typename Binding>
constexpr std::size_t binding_depth()
{
  if constexpr (std::is_void_v<typename Binding::next_type>) {return 1;}
  else {return 1 + binding_depth<typename Binding::next_type>();}
}

template <typename Binding>
void fill_spec_rows(const Binding & binding, const std::string & parent_name, SpecRows & rows)
{
  rows.names.emplace_back(Binding::name());
  rows.instances.push_back(binding.instance);
  rows.parents.push_back(parent_name);
  if constexpr (!std::is_void_v<typename Binding::next_type>)
  {
    fill_spec_rows(binding.next, std::string{Binding::name()}, rows);
  }
}

/// Build the three parallel Spec vectors from a compile-time binding. The chain order is root
/// first, so parents always precede their children -- the same order the kernel's plan needs.
template <typename Binding>
SpecRows build_spec_rows(const Binding & binding)
{
  SpecRows rows;
  const std::size_t depth = binding_depth<Binding>();
  rows.names.reserve(depth);
  rows.instances.reserve(depth);
  rows.parents.reserve(depth);
  fill_spec_rows(binding, std::string{}, rows);
  return rows;
}

/// The invariants the runtime kernel enforces, checkable here without invoking the kernel.
inline bool rows_are_well_formed(const SpecRows & rows, std::string * reason = nullptr)
{
  auto fail = [&](const char * why)
  {
    if (reason != nullptr) {*reason = why;}
    return false;
  };
  if (rows.names.size() != rows.instances.size() || rows.names.size() != rows.parents.size())
  {
    return fail("names/instances/parents must have equal length");
  }
  if (rows.names.empty()) {return fail("topology must have at least one node");}

  std::size_t roots = 0;
  for (std::size_t i = 0; i < rows.names.size(); ++i)
  {
    if (rows.names[i].empty()) {return fail("node names must not be empty");}
    for (std::size_t j = i + 1; j < rows.names.size(); ++j)
    {
      if (rows.names[i] == rows.names[j]) {return fail("node names must be unique");}
    }
    if (rows.parents[i].empty()) {++roots;}
  }
  if (roots != 1) {return fail("a topology must have exactly one root");}

  for (std::size_t i = 0; i < rows.parents.size(); ++i)
  {
    const std::string & parent = rows.parents[i];
    if (parent.empty()) {continue;}
    if (parent == rows.names[i]) {return fail("a node cannot be its own parent");}
    bool found = false;
    for (const std::string & candidate : rows.names)
    {
      if (candidate == parent) {found = true; break;}
    }
    if (!found) {return fail("every parent must name an existing node");}
  }
  return true;
}
}  // namespace topology_contract
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TOPOLOGY_CONTRACT_HPP
