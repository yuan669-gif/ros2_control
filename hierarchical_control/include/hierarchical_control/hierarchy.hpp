// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef HIERARCHICAL_CONTROL__HIERARCHY_HPP_
#define HIERARCHICAL_CONTROL__HIERARCHY_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hierarchical_control
{

/// A controller topology edge. The names are resolved before the real-time loop starts.
struct ControllerHierarchyNode
{
  std::string name;
  std::string parent;  // Empty only for the root.
};

/// Immutable-by-convention execution order generated during configure/switch time.
struct ControllerHierarchyPlan
{
  std::size_t root = 0;
  std::vector<std::size_t> preorder;
  std::vector<std::size_t> postorder;
};

/// Derive each node's parent from the reference interfaces it claims.
///
/// ros2_control names interfaces "<owner>/<local>", so an entry in `claimed_interfaces[i]` whose
/// owner is another node means node `i` is the CHILD of that owner: node `i` consumes a reference
/// the owner produces. Entries whose owner is not a node of this group are ordinary hardware
/// command interfaces and are ignored.
///
/// A node may claim SEVERAL ports of the same owner (a 2-D or 3-D reference is ordinary in
/// control). That is one writer per port, not a conflict. A conflict is one PORT with two
/// different writers, so uniqueness is checked per (owner, port) pair rather than per node.
///
/// Returns a vector parallel to `names`, with an empty string for a root. Throws
/// std::invalid_argument when a single port is claimed by two different writers, or when a node
/// claims a reference interface that would make it its own parent.
inline std::vector<std::string> derive_parents_from_claimed_interfaces(
  const std::vector<std::string> & names,
  const std::vector<std::vector<std::string>> & claimed_interfaces)
{
  if (names.size() != claimed_interfaces.size())
  {
    throw std::invalid_argument(
      "derive_parents_from_claimed_interfaces: names and claimed interfaces must be parallel");
  }

  std::unordered_map<std::string, std::size_t> index;
  index.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (names[i].empty())
    {
      throw std::invalid_argument("controller hierarchy node name must not be empty");
    }
    if (!index.emplace(names[i], i).second)
    {
      throw std::invalid_argument("duplicate controller hierarchy node: " + names[i]);
    }
  }

  std::vector<std::string> parents(names.size());
  // One entry per fully qualified port name, across ALL consumers, so that the same port claimed
  // by two different consumers is detected as two writers.
  std::unordered_map<std::string, std::string> port_writers;
  port_writers.reserve(names.size());

  for (std::size_t consumer = 0; consumer < names.size(); ++consumer)
  {
    // Ports this consumer claims more than once would be its own duplicate, not a conflict.
    std::unordered_map<std::string, bool> seen_in_consumer;

    for (const auto & interface : claimed_interfaces[consumer])
    {
      if (!seen_in_consumer.emplace(interface, true).second)
      {
        throw std::invalid_argument(
          "controller '" + names[consumer] + "' claims reference interface '" + interface +
          "' more than once");
      }
      const auto split = interface.find_first_of('/');
      if (split == std::string::npos) {continue;}
      const auto owner = interface.substr(0, split);
      if (owner == names[consumer])
      {
        // Own hardware command interface, not a reference into this group.
        continue;
      }
      const auto owner_it = index.find(owner);
      if (owner_it == index.end())
      {
        // Ordinary hardware command interface owned by a controller outside this group.
        continue;
      }
      if (owner_it->second == consumer)
      {
        throw std::invalid_argument(
          "controller hierarchy node cannot consume its own reference: " + names[consumer]);
      }

      // Uniqueness is per PORT. The same owner writing several ports of one child is fine.
      const auto inserted = port_writers.emplace(interface, owner);
      if (!inserted.second && inserted.first->second != owner)
      {
        throw std::invalid_argument(
          "reference interface '" + interface + "' has multiple writers: '" +
          inserted.first->second + "' and '" + owner + "'");
      }

      const std::string & parent = owner;
      if (parents[consumer].empty())
      {
        parents[consumer] = parent;
      }
      else if (parents[consumer] != parent)
      {
        throw std::invalid_argument(
          "controller '" + names[consumer] + "' consumes references from both '" +
          parents[consumer] + "' and '" + parent + "'; a node may have only one parent");
      }
    }
  }
  return parents;
}

/// Build and validate a controller tree in O(V + E).
/**
 * The planner is deliberately independent of URDF and ROS types. A URDF adapter can turn
 * link/joint relationships into ControllerHierarchyNode records, while a YAML/plugin adapter can
 * provide semantic controller names. This keeps topology validation reusable in controller_manager
 * and makes it possible to precompute integer plans before entering the real-time loop.
 */
inline ControllerHierarchyPlan build_controller_hierarchy(
  const std::vector<ControllerHierarchyNode> & nodes)
{
  if (nodes.empty())
  {
    throw std::invalid_argument("controller hierarchy must contain at least one node");
  }

  std::unordered_map<std::string, std::size_t> index;
  index.reserve(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i)
  {
    if (nodes[i].name.empty())
    {
      throw std::invalid_argument("controller hierarchy node name must not be empty");
    }
    if (!index.emplace(nodes[i].name, i).second)
    {
      throw std::invalid_argument("duplicate controller hierarchy node: " + nodes[i].name);
    }
  }

  std::vector<std::vector<std::size_t>> children(nodes.size());
  std::size_t root = nodes.size();
  for (std::size_t i = 0; i < nodes.size(); ++i)
  {
    if (nodes[i].parent.empty())
    {
      if (root != nodes.size())
      {
        throw std::invalid_argument("controller hierarchy must have exactly one root");
      }
      root = i;
      continue;
    }
    const auto parent_it = index.find(nodes[i].parent);
    if (parent_it == index.end())
    {
      throw std::invalid_argument(
        "controller hierarchy parent does not exist: " + nodes[i].parent);
    }
    if (parent_it->second == i)
    {
      throw std::invalid_argument("controller hierarchy node cannot be its own parent: " + nodes[i].name);
    }
    children[parent_it->second].push_back(i);
  }
  if (root == nodes.size())
  {
    throw std::invalid_argument("controller hierarchy must have exactly one root");
  }

  ControllerHierarchyPlan plan;
  plan.root = root;
  std::vector<std::uint8_t> active(nodes.size(), 0);
  std::vector<std::uint8_t> visited(nodes.size(), 0);

  std::function<void(std::size_t)> visit = [&](const std::size_t node) {
    if (active[node])
    {
      throw std::invalid_argument("cycle in controller hierarchy at: " + nodes[node].name);
    }
    if (visited[node])
    {
      return;
    }
    active[node] = 1;
    visited[node] = 1;
    plan.preorder.push_back(node);
    for (const auto child : children[node])
    {
      visit(child);
    }
    plan.postorder.push_back(node);
    active[node] = 0;
  };
  visit(root);
  for (std::size_t i = 0; i < nodes.size(); ++i)
  {
    if (!visited[i])
    {
      throw std::invalid_argument("controller hierarchy contains an unreachable node: " + nodes[i].name);
    }
  }
  return plan;
}

/// Execute a two-phase tree plan without lookups or allocations.
/**
 * `state_update` is invoked in postorder (children before parent), and `command_propagate` is
 * invoked in preorder (parent before children). The caller owns the node storage and is expected to
 * pass callbacks that dispatch to already-loaded ControllerInterface instances.
 */
template<typename StateUpdate, typename CommandPropagate>
void execute_controller_hierarchy(
  const ControllerHierarchyPlan & plan, StateUpdate state_update,
  CommandPropagate command_propagate)
{
  for (const auto node : plan.postorder)
  {
    state_update(node);
  }
  for (const auto node : plan.preorder)
  {
    command_propagate(node);
  }
}

}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL__HIERARCHY_HPP_
