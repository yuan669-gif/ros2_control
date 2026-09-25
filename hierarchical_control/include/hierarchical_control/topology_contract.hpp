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
#include <tuple>
#include <utility>
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

/// The `Index`-th port type of a `PortList`, so callers can compare a runtime list against a
/// compile-time list POSITION BY POSITION (name and order, not just length).
template <typename List, std::size_t Index>
struct port_at;

template <typename First, typename... Rest>
struct port_at<PortList<First, Rest...>, 0>
{
  using type = First;
};

template <typename First, typename... Rest, std::size_t Index>
struct port_at<PortList<First, Rest...>, Index>
{
  using type = typename port_at<PortList<Rest...>, Index - 1>::type;
};

template <typename List, std::size_t Index>
using port_at_t = typename port_at<List, Index>::type;

/// Two port lists are equal element-wise, comparing NAME and DIMENSION together.
template <typename ListA, typename ListB>
struct port_lists_agree : std::false_type
{
};

template <>
struct port_lists_agree<PortList<>, PortList<>> : std::true_type
{
};

template <typename A0, typename... A, typename B0, typename... B>
struct port_lists_agree<PortList<A0, A...>, PortList<B0, B...>>
  : std::bool_constant<
      // `same_port_v` folds NAME and DIMENSION together, so one comparison covers both.
      same_port_v<A0, B0> && port_lists_agree<PortList<A...>, PortList<B...>>::value>
{
};

/// Concatenate port lists, so a parent's per-child declarations can be compared against the
/// CONCATENATION of what its children declare, in child order.
template <typename... Lists>
struct concat_port_lists
{
  using type = PortList<>;
};

template <typename... As>
struct concat_port_lists<PortList<As...>>
{
  using type = PortList<As...>;
};

template <typename... As, typename... Bs, typename... Rest>
struct concat_port_lists<PortList<As...>, PortList<Bs...>, Rest...>
{
  using type = typename concat_port_lists<PortList<As..., Bs...>, Rest...>::type;
};

template <typename... Lists>
using concat_port_lists_t = typename concat_port_lists<Lists...>::type;

/// The REFERENCE edge of a parent with ANY NUMBER of children: the ports the parent declares it
/// writes into its children must be the CONCATENATION, in child order, of the reference ports those
/// children declare they receive.
/**
 * The concatenation is what makes the routing per-child rather than aggregate: port i of the
 * parent's list belongs to whichever child owns that segment, so a declaration cannot silently
 * associate a parent's ports with the wrong child (review item B).
 */
template <typename ParentPorts, typename... ChildPorts>
constexpr bool children_references_agree() noexcept
{
  return port_lists_agree<
    typename ParentPorts::for_children, concat_port_lists_t<typename ChildPorts::reference...>>::value;
}

/// The STATE edge of a parent with ANY NUMBER of children: the child state ports the parent
/// declares it consumes must be the concatenation, in child order, of the state ports those
/// children declare they publish.
template <typename ParentPorts, typename... ChildPorts>
constexpr bool children_states_agree() noexcept
{
  return port_lists_agree<
    typename ParentPorts::child_state, concat_port_lists_t<typename ChildPorts::state...>>::value;
}

/// Both edges agree for a branching parent.
template <typename ParentPorts, typename... ChildPorts>
constexpr bool children_declarations_agree() noexcept
{
  return children_references_agree<ParentPorts, ChildPorts...>() &&
         children_states_agree<ParentPorts, ChildPorts...>();
}

/// Halt compilation unless both edges of a branching parent/child group agree.
template <typename ParentPorts, typename... ChildPorts>
constexpr void require_children_declarations_compatible()
{
  static_assert(
    children_references_agree<ParentPorts, ChildPorts...>(),
    "topology_contract: REFERENCE EDGE MISMATCH -- the reference ports this parent declares it "
    "writes into its children are not the concatenation, in child order, of the reference ports its "
    "children declare they receive (name, order or physical dimension differs)");
  static_assert(
    children_states_agree<ParentPorts, ChildPorts...>(),
    "topology_contract: STATE EDGE MISMATCH -- the child state ports this parent declares it "
    "consumes are not the concatenation, in child order, of the state ports its children declare "
    "they publish (name, order or physical dimension differs)");
}

/// Detection: does a controller declare its ports as types (`TypedPortsMixin` exposes
/// `using typed_ports = Ports;`)? A controller that hand-writes its runtime strings has nothing to
/// compare here, and the runtime verifier covers it instead.
template <typename T, typename = void>
struct typed_ports_of
{
  using type = void;
};

template <typename T>
struct typed_ports_of<T, std::void_t<typename T::typed_ports>>
{
  using type = typename T::typed_ports;
};

template <typename T>
using typed_ports_of_t = typename typed_ports_of<T>::type;

template <typename T>
inline constexpr bool has_typed_ports_v = !std::is_void_v<typed_ports_of_t<T>>;

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

/// Concatenate two TypeLists.
template <typename A, typename B>
struct concat;

template <typename... As, typename... Bs>
struct concat<TypeList<As...>, TypeList<Bs...>>
{
  using type = TypeList<As..., Bs...>;
};

template <typename A, typename B>
using concat_t = typename concat<A, B>::type;

/// Fold a pack of TypeLists into one (empty pack -> empty list).
template <typename... Lists>
struct concat_all
{
  using type = TypeList<>;
};

template <typename First, typename... Rest>
struct concat_all<First, Rest...>
{
  using type = concat_t<First, typename concat_all<Rest...>::type>;
};

/// Forward declaration: `BoundNode` uses it to expose its own ownership verdict, since the verdict
/// needs the whole subtree's name list, which only the enclosing template can expand.
template <typename Binding, typename OwnerList>
struct node_owners_ok;

/// Slot access to a `std::tuple`, guarded so that an out-of-range index is a compile error with a
/// message instead of a template backtrace.
template <std::size_t Index, typename... Children>
constexpr const auto & child_at(const std::tuple<Children...> & children)
{
  static_assert(
    Index < sizeof...(Children), "topology_contract: child index out of range for this node");
  return std::get<Index>(children);
}

template <std::size_t Index, typename... Children>
constexpr auto & child_at(std::tuple<Children...> & children)
{
  static_assert(
    Index < sizeof...(Children), "topology_contract: child index out of range for this node");
  return std::get<Index>(children);
}

/// A topology node bound to its port contract and to ANY NUMBER of child bindings.
/**
/// The children are a parameter pack stored in a `std::tuple`, so one declaration describes a whole
/// branching tree, not just a chain. An earlier revision had a single `Next` slot
/// (`next_storage<Next>`) and could therefore only express a chain, even though the runtime kernel
/// has always supported multi-child nodes (review item B).
///
/// `compose(node, c1, c2, ...)` builds one; `make_leaf(node)` builds a node with no children.
/// A single-child call is unchanged, so existing chain declarations keep compiling.
*/
template <typename ControllerT, typename Node, typename ContractT, typename... Children>
struct BoundNode
{
  using controller_type = ControllerT;
  using node_type = Node;
  using contract_type = ContractT;
  using children_types = TypeList<Children...>;

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
  /// The child bindings, in DECLARATION order. Every parent-side per-child declaration list is
  /// matched against exactly this order, so a port can never be routed to the wrong child.
  std::tuple<Children...> children{};

  static constexpr std::size_t child_count = sizeof...(Children);
  static constexpr bool has_child = (sizeof...(Children) > 0);

  /// Every node name in this SUBTREE, pre-order: this node first, then each child's subtree in
  /// declaration order. Expandable here because the children pack is in scope.
  using subtree_names = concat_t<
    TypeList<typename Node::name_type>, typename concat_all<typename Children::subtree_names...>::type>;

  /// Number of nodes in this subtree (this node + all descendants).
  static constexpr std::size_t subtree_size = 1 + (0 + ... + Children::subtree_size);

  /// Does every node in this subtree declare only ports owned by `OwnerList`?
  /**
   * The owner list of a whole tree contains the names of all of its nodes, so this is the "no port
   * refers to a controller outside the tree" rule, checked per subtree.
   */
  template <typename OwnerList>
  struct owners_ok_for
    : std::bool_constant<
        node_owners_ok<BoundNode, OwnerList>::value &&
        (Children::template owners_ok_for<OwnerList>::value && ...)>
  {
  };

  static constexpr std::string_view name() { return st::node_name<Node>(); }

  template <std::size_t Index>
  static constexpr auto & child(BoundNode & self)
  {
    return child_at<Index>(self.children);
  }

  template <std::size_t Index>
  static constexpr const auto & child(const BoundNode & self)
  {
    return child_at<Index>(self.children);
  }
};

/// A binding's topology has TWO sources: the structural nesting of `BoundNode::next` and each
/// node's own `static_topology::Node<Name, Parent>::parent_type`. They must agree.
///
/// The nesting decides the `parents` column of the emitted Spec (`fill_spec_rows` passes the
/// enclosing node's name down), so before this check the nesting silently OVERRODE the type-level
/// parent: with `A = Root<a>` and `B = Root<b>`, `compose<A>(a, make_leaf<B>(b))` compiled and
/// emitted `B.parent = "a"`, turning two declared roots into a chain (reproduced by the review,
/// item A). A node that declares itself a root can therefore never be bound as a child, and a child
/// must declare exactly the node it is nested under.
///
/// Enforcing agreement -- rather than deleting one of the sources -- keeps both useful: the nesting
/// carries the runtime instances, and `parent_type` is what `static_topology`'s cycle guarantee is
/// stated over.
template <typename ParentNode, typename ChildBinding>
constexpr bool child_declares_this_parent()
{
  if constexpr (std::is_void_v<typename ChildBinding::node_type::parent_type>)
  {
    // The child declares itself a root, so it cannot be nested under anything.
    return false;
  }
  else
  {
    return std::is_same_v<typename ChildBinding::node_type::parent_type, ParentNode>;
  }
}

/// Every child of this node must declare it as its parent, and the children must be distinct.
template <typename Node, typename... ChildBindings>
constexpr bool children_declare_this_parent()
{
  return (child_declares_this_parent<Node, ChildBindings>() && ...);
}

/// No two children may carry the same node name, and none may reuse this node's name.
template <typename Node, typename... ChildBindings>
constexpr bool children_names_are_distinct()
{
  constexpr std::array<std::string_view, sizeof...(ChildBindings)> child_names{
    ChildBindings::name()...};
  for (std::size_t i = 0; i < child_names.size(); ++i)
  {
    if (child_names[i] == st::node_name<Node>()) {return false;}
    for (std::size_t j = i + 1; j < child_names.size(); ++j)
    {
      if (child_names[i] == child_names[j]) {return false;}
    }
  }
  return true;
}

/// Compose a parent binding on top of ANY NUMBER of child bindings.
///
/// The controller type and the children are deduced from the arguments, so the binding keeps
/// correctly adjusted `ControllerInterfaceBase*` pointers and never sees a `void*`. A single-child
/// call (`compose<Parent, Contract>(instance, child)`) is unchanged; passing two or more children is
/// what makes a branching tree expressible.
template <typename Node, typename ContractT, typename ControllerT, typename... ChildBindings>
BoundNode<ControllerT, Node, ContractT, ChildBindings...> compose(
  ControllerT * parent_instance, const ChildBindings &... children)
{
  static_assert(
    std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
    "topology_contract: compose needs a controller deriving from ControllerInterfaceBase");
  static_assert(
    children_declare_this_parent<Node, ChildBindings...>(),
    "topology_contract: TOPOLOGY MISMATCH -- a child binding does not declare this node as its "
    "parent. Either the child is declared a Root (a root cannot be nested under anything), or its "
    "static_topology parent_type names a different node. The structural nesting and the type-level "
    "parent must agree, otherwise the emitted plan would silently differ from the declared "
    "topology.");
  static_assert(
    children_names_are_distinct<Node, ChildBindings...>(),
    "topology_contract: a node's children must have distinct names, and none may repeat the node's "
    "own name");

  // PER-CHILD PORT ROUTING, when every participant declares its ports as types. The parent's
  // child-facing lists are compared against the CONCATENATION of its children's declarations in
  // child order, so each segment belongs to a known child and a port can never be routed to the
  // wrong one -- checking the parent's whole list against a single child (which an earlier revision
  // did) cannot express that. A controller that hand-writes its runtime strings has no type-level
  // declaration to compare; the runtime verifier covers that case instead.
  if constexpr (
    has_typed_ports_v<ControllerT> &&
    (has_typed_ports_v<typename ChildBindings::controller_type> && ...))
  {
    require_children_declarations_compatible<
      typed_ports_of_t<ControllerT>,
      typed_ports_of_t<typename ChildBindings::controller_type>...>();
  }
  BoundNode<ControllerT, Node, ContractT, ChildBindings...> binding;
  binding.instance = static_cast<controller_interface::ControllerInterfaceBase *>(parent_instance);
  binding.children = std::make_tuple(children...);
  return binding;
}

/// Compose a leaf binding (no child).
///
/// "Leaf" here means "no child BINDING", not "a root": the last node of a chain is normally a
/// `Descendant`, so its `parent_type` is its real parent. Whether that parent is the node it ends up
/// nested under is checked by `compose`.
template <typename Node, typename ContractT, typename ControllerT>
BoundNode<ControllerT, Node, ContractT> make_leaf(ControllerT * instance)
{
  static_assert(
    std::is_base_of_v<controller_interface::ControllerInterfaceBase, ControllerT>,
    "topology_contract: make_leaf needs a controller deriving from ControllerInterfaceBase");
  BoundNode<ControllerT, Node, ContractT> binding;
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

/// Apply the ownership check for one node against a given owner list.
template <typename Binding, typename OwnerList>
struct node_owners_ok;

template <typename Binding, typename... Names>
struct node_owners_ok<Binding, TypeList<Names...>>
  : contract_owners_are_known<typename Binding::contract_type, Names...>
{
};

/// The names of every node in the binding TREE (not just a chain).
template <typename Binding>
struct binding_owner_types
{
  using type = typename Binding::subtree_names;
};

/// The public check: fails to compile when a declared port names an owner outside the topology.
template <typename Binding>
constexpr void require_ports_are_owned()
{
  static_assert(
    Binding::template owners_ok_for<typename binding_owner_types<Binding>::type>::value,
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

/// Number of nodes in a binding tree.
template <typename Binding>
constexpr std::size_t binding_depth()
{
  return Binding::subtree_size;
}

template <typename Binding>
void fill_spec_rows(const Binding & binding, const std::string & parent_name, SpecRows & rows);

/// Emit every child of `binding` with `binding`'s name as their parent, in DECLARATION order.
template <typename Binding, std::size_t... Index>
void fill_child_rows(
  const Binding & binding, SpecRows & rows, std::index_sequence<Index...>)
{
  (fill_spec_rows(child_at<Index>(binding.children), std::string{Binding::name()}, rows), ...);
}

/// Emit one node, then its subtree: the order is pre-order, parents always before their children.
template <typename Binding>
void fill_spec_rows(const Binding & binding, const std::string & parent_name, SpecRows & rows)
{
  rows.names.emplace_back(Binding::name());
  rows.instances.push_back(binding.instance);
  rows.parents.push_back(parent_name);
  if constexpr (Binding::child_count > 0)
  {
    fill_child_rows(binding, rows, std::make_index_sequence<Binding::child_count>{});
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

  // Unique names are not enough: the kernel executes INSTANCES. The same controller object bound
  // under two node names would be advanced twice per stage in one cycle, which is exactly what the
  // "one call per controller per cycle" guarantee forbids, so the plan rejects it as well.
  for (std::size_t i = 0; i < rows.instances.size(); ++i)
  {
    for (std::size_t j = i + 1; j < rows.instances.size(); ++j)
    {
      if (rows.instances[i] == rows.instances[j])
      {
        const std::string why =
          "the same controller instance is bound to both node '" + rows.names[i] + "' and node '" +
          rows.names[j] + "'";
        return fail(why.c_str());
      }
    }
  }

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
