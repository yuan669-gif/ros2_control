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
