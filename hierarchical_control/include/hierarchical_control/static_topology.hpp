// Copyright 2026
// Licensed under the Apache License, Version 2.0.
//
// Compile-time controller topology with a statically-enforced acyclicity guarantee.
//
// MOTIVATION
// ----------
// doc/FORMAL_MODEL.md Corollary 2 makes acyclicity of the same-cycle requirement graph G a
// *configuration-time* check: the runtime rejects an unsatisfiable topology when the plan is
// built. This header moves that guarantee to *compile time* for statically declared topologies,
// in the spirit of FineMote, where order and hierarchy are carried by the type system rather than
// by configuration.
//
// THE REPRESENTATION
// ------------------
// A hierarchy is a chain of types:  Node<Name, Parent>, with Parent = void for a root. Each node
// type exposes its root-to-self ancestry as a compile-time array of names, so ancestry is a
// *type-level* property and every check below is a template-argument check that fires at the point
// of use.
//
// USAGE
// -----
//     struct chassis_name { static constexpr auto value = NameOf("chassis"); };
//     struct wheel_name   { static constexpr auto value = NameOf("wheel"); };
//
//     using chassis = Root<chassis_name>;
//     using wheel   = Descendant<wheel_name, chassis>;
//
// THE GUARANTEE
// -------------
// `Descendant<Name, Parent>` is rejected at compile time if Name already occurs in the parent's
// ancestry. Because a node's ancestry always contains that node's own name, this rejects both
//
//   (a) a SELF-LOOP (child name == parent name), and
//   (b) reusing ANY ancestor as a new parent, i.e. closing a cycle of any length >= 2, since a
//       cycle through the hierarchy must revisit a node already on the root-to-parent path.
//
// So every topology that typechecks is acyclic -- a genuine static guarantee about the
// tree/reference structure, and a strengthening of Corollary 2 from "rejected at configuration
// time" to "cannot be written".
//
// WHAT THIS DOES NOT COVER (stated up front, because the distinction matters)
// -------------------------------------------------------------------------
//   * The guarantee concerns the CONSTRUCTION (tree/reference) graph. A state edge may point at a
//     controller that is not its hierarchical ancestor -- a shared child consumed by a different
//     parent. Such edges are not expressible here at all, so this header neither permits nor
//     rejects them; they remain the runtime checker's job
//     (hierarchical_control::build_controller_hierarchy).
//   * Controller plugins are loaded dynamically via pluginlib and topologies normally come from
//     YAML. This header therefore covers STATICALLY DECLARED topologies only; YAML-driven
//     topologies still need the runtime check. The positioning is
//     "compile-time capability + runtime fallback", NOT a replacement.
//   * Compilation cost grows with the number of distinct topologies instantiated in a translation
//     unit. See doc/METAPROGRAMMING_CONTRACT.md.

#ifndef HIERARCHICAL_CONTROL_STATIC_TOPOLOGY_HPP
#define HIERARCHICAL_CONTROL_STATIC_TOPOLOGY_HPP

#include <array>
#include <cstddef>
#include <string_view>
#include <type_traits>

namespace hierarchical_control
{
namespace static_topology
{
/// A compile-time string.
template <std::size_t N>
struct Name
{
  std::array<char, N> chars{};
  static constexpr std::size_t length = N - 1;
};

/// Build a Name from a string literal:  static constexpr auto n = NameOf("chassis");
template <std::size_t N>
constexpr Name<N> NameOf(const char (&literal)[N])
{
  Name<N> out{};
  for (std::size_t i = 0; i < N; ++i) {out.chars[i] = literal[i];}
  return out;
}

template <std::size_t N>
constexpr std::string_view view_of(const Name<N> & name)
{
  return std::string_view{name.chars.data(), N - 1};
}

/// A name type is any type with a static member `value` convertible to a Name<N>.
template <typename NameT, typename = void>
struct name_of
{
  static constexpr bool defined = false;
};

template <typename NameT>
struct name_of<NameT, std::void_t<decltype(NameT::value)>>
{
  static constexpr bool defined = true;
  static constexpr auto value = NameT::value;
};

template <typename NameT>
inline constexpr auto name_value_v = name_of<NameT>::value;

template <typename NameT>
inline constexpr bool has_name_v = name_of<NameT>::defined;

// ---------------------------------------------------------------------------------------------
// Type-level ancestry
// ---------------------------------------------------------------------------------------------

/// Depth of a parent type, with a void specialisation so that Node<Name, void> never instantiates
/// `void::depth` (a plain ternary would, because both arms are instantiated).
template <typename Parent>
struct parent_depth
{
  static constexpr std::size_t value = Parent::depth;
};

template <>
struct parent_depth<void>
{
  static constexpr std::size_t value = 0;
};

/// Each node type exposes:
///   using name_type   -- the type carrying its own name
///   using parent_type -- the parent node type, or void for a root
///   static constexpr std::size_t depth
///   static constexpr std::array<std::string_view, depth> ancestry  -- root-first, self last
template <typename NameT, typename Parent>
struct Node
{
  using name_type = NameT;
  using parent_type = Parent;

  static constexpr std::size_t depth = parent_depth<Parent>::value + 1;

  static constexpr std::array<std::string_view, depth> ancestry = []() constexpr
  {
    std::array<std::string_view, depth> out{};
    if constexpr (!std::is_void_v<Parent>)
    {
      for (std::size_t i = 0; i < Parent::depth; ++i) {out[i] = Parent::ancestry[i];}
    }
    out[depth - 1] = view_of(name_value_v<NameT>);
    return out;
  }();
};

// ---------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------

/// Does a node type's ancestry already contain the name carried by `NameT`?
template <typename NodeT, typename NameT>
constexpr bool ancestry_contains()
{
  constexpr auto candidate = view_of(name_value_v<NameT>);
  for (std::size_t i = 0; i < NodeT::depth; ++i)
  {
    if (NodeT::ancestry[i] == candidate) {return true;}
  }
  return false;
}

/// Is a node type well-formed? A node is well-formed iff its own name does not occur in its
/// parent's ancestry. `Node<Name, void>` is always well-formed.
template <typename NodeT>
struct is_valid_node : std::false_type
{
};

template <typename NameT>
struct is_valid_node<Node<NameT, void>> : std::true_type
{
};

template <typename NameT, typename Parent>
struct is_valid_node<Node<NameT, Parent>>
  : std::bool_constant<!ancestry_contains<Parent, NameT>()>
{
};

template <typename T>
inline constexpr bool is_valid_node_v = is_valid_node<T>::value;

// ---------------------------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------------------------

/// Root of a hierarchy: no parent, so no constraint to violate.
template <typename NameT>
using Root = Node<NameT, void>;

/// Descendant of `Parent` named `NameT`: a TYPE, so topologies are written with `using`.
///
/// The static_assert is what enforces the guarantee: if `NameT` already occurs in the parent's
/// ancestry then the requested topology would contain a cycle (a self-loop when the names are
/// equal, otherwise a cycle through an ancestor).
template <typename NameT, typename Parent>
struct descendant
{
  static_assert(
    !std::is_void_v<Parent>,
    "static_topology: descendant requires a parent; use Root<Name> for a root node");
  static_assert(
    !ancestry_contains<Parent, NameT>(),
    "static_topology: CYCLE -- this node's name already occurs in its parent's ancestry, so the "
    "topology would contain a cycle and is rejected at compile time");

  using type = Node<NameT, Parent>;
};

/// Convenience alias:  using wheel = Descendant<wheel_name, chassis>;
template <typename NameT, typename Parent>
using Descendant = typename descendant<NameT, Parent>::type;

/// Compile-time predicate mirroring the static_assert, so callers and tests can assert on it
/// directly instead of provoking a hard error.
template <typename NameT, typename Parent>
inline constexpr bool is_acyclic_extension_v = is_valid_node_v<Node<NameT, Parent>>;

template <typename NodeT>
constexpr bool is_root()
{
  return std::is_void_v<typename NodeT::parent_type>;
}

template <typename NodeT>
constexpr std::size_t node_count()
{
  return NodeT::depth;
}

template <typename NodeT>
constexpr std::string_view node_name()
{
  return view_of(name_value_v<typename NodeT::name_type>);
}

/// A reference edge exists from an ancestor to a descendant iff the ancestor's name occurs in the
/// descendant's ancestry.
template <typename DescendantNode, typename AncestorNode>
constexpr bool has_reference_edge()
{
  return ancestry_contains<DescendantNode, typename AncestorNode::name_type>();
}
}  // namespace static_topology
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_STATIC_TOPOLOGY_HPP
