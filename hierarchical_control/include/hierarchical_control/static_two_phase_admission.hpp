// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__STATIC_TWO_PHASE_ADMISSION_HPP_
#define HIERARCHICAL_CONTROL__STATIC_TWO_PHASE_ADMISSION_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>

#include "hierarchical_control/static_topology.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/two_phase_controller_interface.hpp"

namespace hierarchical_control
{
/// Compile-time description of a DECLARED two-phase tree.
/**
 * The manager's runtime admission (`ControllerManager::two_phase_rejections`) has to work on an
 * arbitrary controller list loaded from plugins and configured by YAML, which is why it infers
 * everything from strings and RTTI. When the whole tree is compiled in as one composite, that
 * inference is unnecessary: several of the rejection codes become *type facts*.
 *
 *   membership  : `is_base_of_v<TwoPhaseControllerInterface, ControllerT>` -- replaces the
 *                 manager's `dynamic_cast`;
 *   the edges   : derived from the node types, not from splitting `"<owner>/..."` strings;
 *   the order   : the pre-order of `compose` lists every parent before its children, so
 *                 `parent_index < child_index` is a CONSTRUCTION property and a `static_assert`,
 *                 not a comparison over the manager's own list.
 *
 * What deliberately does NOT live here, because it is a property of the DEPLOYMENT rather than of
 * the type: the `update_rate` buckets (the rate is a YAML/parameter value) and the aliasing check
 * (two names for one controller object is a property of the controller LIST). Those remain runtime,
 * and so does the `ResourceManager` claim check, which is the only guard for plugin/YAML trees.
 *
 * This header is description only: it decides nothing and changes no behaviour. The design that
 * consumes it is in `doc/STATIC_ADMISSION_DESIGN_2026-10.md`.
 */
namespace static_two_phase
{
namespace st = static_topology;
namespace tc = topology_contract;

/// One reference edge of the declared tree: the parent's name and the child's name.
struct Edge
{
  std::string_view parent;
  std::string_view child;
};

// ---------------------------------------------------------------------------------------------
// Membership: every node must implement the two-phase interface.
// ---------------------------------------------------------------------------------------------

template <typename List>
struct all_nodes_are_members;

template <typename Binding>
struct subtree_all_members
{
  static constexpr bool value =
    std::is_base_of_v<TwoPhaseControllerInterface, typename Binding::controller_type> &&
    all_nodes_are_members<typename Binding::children_types>::value;
};

template <>
struct all_nodes_are_members<tc::TypeList<>>
{
  static constexpr bool value = true;
};

template <typename First, typename... Rest>
struct all_nodes_are_members<tc::TypeList<First, Rest...>>
{
  static constexpr bool value =
    subtree_all_members<First>::value && all_nodes_are_members<tc::TypeList<Rest...>>::value;
};

// ---------------------------------------------------------------------------------------------
// Names and edges, straight from the node types.
// ---------------------------------------------------------------------------------------------

/// The node names of a `subtree_names` list, in pre-order (the list already is pre-order).
template <typename... Names>
constexpr std::array<std::string_view, sizeof...(Names)> names_of(tc::TypeList<Names...>) noexcept
{
  return {st::view_of(st::name_value_v<Names>)...};
}

template <typename ParentName, typename Child>
struct edge_of
{
  static constexpr Edge value{
    st::view_of(st::name_value_v<ParentName>),
    st::view_of(st::name_value_v<typename Child::node_type::name_type>)};
};

template <typename ParentName, typename ChildrenList>
struct child_edges;

template <typename Binding>
struct edges_of;

template <typename ParentName, typename... Children>
struct child_edges<ParentName, tc::TypeList<Children...>>
{
  using type = typename tc::concat_all<
    typename tc::concat_t<
      tc::TypeList<edge_of<ParentName, Children>>, typename edges_of<Children>::type>...>::type;
};

template <typename Binding>
struct edges_of
{
  using type = typename child_edges<
    typename Binding::node_type::name_type, typename Binding::children_types>::type;
};

template <typename... Entries>
constexpr std::array<Edge, sizeof...(Entries)> edges_array(tc::TypeList<Entries...>) noexcept
{
  return {Entries::value...};
}

// ---------------------------------------------------------------------------------------------
// The description.
// ---------------------------------------------------------------------------------------------

/// The STRUCTURE of a declared tree: its members in pre-order, and its parent/child edges.
/**
 * Deliberately says nothing about what the nodes implement, because the structure is what a runtime
 * PLAN can be checked against, and that check applies to every compiled-in tree -- including one that
 * is executed by the staged kernel rather than by the two-phase passes.
 */
template <typename Binding>
struct structure_description
{
  static constexpr std::size_t member_count = Binding::subtree_size;
  static constexpr auto members = names_of(typename Binding::subtree_names{});

  using edge_list = typename edges_of<Binding>::type;
  static constexpr std::size_t edge_count = member_count - 1;
  static constexpr auto edges = edges_array(edge_list{});

  /// Pre-order index of a member, or `member_count` when the name is not a member.
  static constexpr std::size_t index_of(std::string_view name) noexcept
  {
    for (std::size_t i = 0; i < member_count; ++i)
    {
      if (members[i] == name) {return i;}
    }
    return member_count;
  }

  /// Every edge runs parent -> child in pre-order. For a `compose`-built binding this is a
  /// construction property; asserting it here is what makes it a guarantee for a hand-built plan.
  static constexpr bool edges_are_ordered() noexcept
  {
    for (std::size_t i = 0; i < edges.size(); ++i)
    {
      if (index_of(edges[i].parent) >= index_of(edges[i].child)) {return false;}
    }
    return true;
  }
  static_assert(
    edges_are_ordered(),
    "static_two_phase: a declared tree must list every parent BEFORE its children; otherwise both "
    "passes walk that edge in the same (wrong) direction -- the runtime admission's "
    "unschedulable_order");
};

/// A declared tree that is EXECUTED by the two-phase passes: the structure above, plus the
/// requirement that every node implements the two-phase interface.
/**
 * Membership is a `static_assert` rather than a runtime check: a tree that mixes the two execution
 * paths cannot be ordered by one pair of passes. That is the manager's runtime
 * `cross_mode_dependency`, and here the configuration is not expressible at all.
 */
template <typename Binding>
struct tree_description : structure_description<Binding>
{
  static_assert(
    subtree_all_members<Binding>::value,
    "static_two_phase: every node of a declared two-phase tree must implement "
    "TwoPhaseControllerInterface. A tree that mixes the two execution paths cannot be ordered by "
    "one pair of passes -- that is the runtime admission's cross_mode_dependency, and here it is "
    "not expressible");
};

/// Does the runtime plan of a compiled-in tree agree with the STRUCTURE of its TYPE?
/**
 * The description's `members` ARE the execution order (the pre-order the two passes rely on), and its
 * `edges` are the topology edges. The plan the kernel consumes is built from the same type, so the
 * two must agree -- and that agreement is the whole reason the compile-time facts can stand in for
 * the manager's runtime admission (membership, edges, order). If they ever disagree, the static
 * guarantee is worthless, so this function exists to be able to FAIL.
 *
 * Runtime because the plan's instances and their generated port strings only exist once a controller
 * is activated; the comparison is a handful of string compares per node and is on no per-cycle path.
 */
template <typename Binding>
bool plan_matches_description(const tc::SpecRows & rows, std::string * reason = nullptr)
{
  using description = structure_description<Binding>;
  constexpr auto members = description::members;
  constexpr auto edges = description::edges;

  if (rows.names.size() != members.size())
  {
    if (reason != nullptr)
    {
      *reason = "the plan has " + std::to_string(rows.names.size()) +
                " node(s), the description has " + std::to_string(members.size());
    }
    return false;
  }

  // ORDER: the description's pre-order is the order both passes walk, so a permutation is fatal even
  // though the node SET would still be right.
  for (std::size_t i = 0; i < members.size(); ++i)
  {
    if (rows.names[i] != members[i])
    {
      if (reason != nullptr)
      {
        *reason = "node " + std::to_string(i) + " is '" + rows.names[i] +
                  "' in the plan but '" + std::string(members[i]) + "' in the description";
      }
      return false;
    }
  }

  // EDGES: every parent the plan names must be one of the declared edges, with this node as child.
  for (std::size_t i = 0; i < rows.parents.size(); ++i)
  {
    if (rows.parents[i].empty()) {continue;}
    bool found = false;
    for (std::size_t e = 0; e < edges.size(); ++e)
    {
      if (edges[e].parent == rows.parents[i] && edges[e].child == rows.names[i])
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      if (reason != nullptr)
      {
        *reason = "the plan has the edge '" + rows.parents[i] + "' -> '" + rows.names[i] +
                  "', which the description does not contain";
      }
      return false;
    }
  }
  return true;
}

/// Entry-point form, so a call site reads as one requirement instead of two traits.
template <typename Binding>
constexpr void require_two_phase_tree() noexcept
{
  static_assert(
    subtree_all_members<Binding>::value,
    "static_two_phase: every node of a declared two-phase tree must implement "
    "TwoPhaseControllerInterface");
  static_assert(
    tree_description<Binding>::edges_are_ordered(),
    "static_two_phase: a declared tree must list every parent BEFORE its children");
}

}  // namespace static_two_phase
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__STATIC_TWO_PHASE_ADMISSION_HPP_
