// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__CONTROLLER_HIERARCHY_BUILDER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_HIERARCHY_BUILDER_HPP_

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager/hierarchy.hpp"
#include "controller_manager/urdf_hierarchy.hpp"

namespace controller_manager
{

/// A controller-to-resource association used by the URDF-driven tree builder.
/**
 * `joint_names` normally comes from a controller's command interface configuration. `anchor_link`
 * is optional and is useful for a semantic/composite controller which does not claim a hardware
 * joint directly (for example a chassis controller exporting references to wheel controllers).
 */
struct ControllerResourceBinding
{
  std::string controller_name;
  std::vector<std::string> joint_names;
  std::string anchor_link;
};

namespace detail
{
inline bool is_ancestor(
  const std::string & ancestor, const std::string & descendant,
  const std::unordered_map<std::string, std::string> & parent)
{
  auto current = descendant;
  while (true)
  {
    const auto it = parent.find(current);
    if (it == parent.end() || it->second.empty())
    {
      return false;
    }
    if (it->second == ancestor)
    {
      return true;
    }
    current = it->second;
  }
}

inline std::size_t link_depth(
  const std::string & link, const std::unordered_map<std::string, std::string> & parent)
{
  std::size_t depth = 0;
  auto current = link;
  while (true)
  {
    const auto it = parent.find(current);
    if (it == parent.end() || it->second.empty())
    {
      return depth;
    }
    ++depth;
    current = it->second;
  }
}
}  // namespace detail

/// Infer a controller tree from URDF joint resources.
/**
 * Each controller is anchored at the child link of its claimed joint(s). For every controller B,
 * the builder chooses the deepest controller A whose anchor is a strict ancestor of B's anchor and
 * records A as B's parent. This gives a deterministic tree for nested joint controllers while
 * leaving semantic controllers without direct joint claims expressible through `anchor_link`.
 *
 * The function intentionally rejects multiple roots. A caller that needs broadcasters or unrelated
 * monitoring controllers should keep them in the legacy flat group or create a separate tree.
 */
inline std::vector<ControllerHierarchyNode> build_controller_hierarchy_from_urdf(
  const std::vector<std::string> & links, const std::vector<UrdfJointEdge> & joints,
  const std::vector<ControllerResourceBinding> & bindings)
{
  if (bindings.empty())
  {
    throw std::invalid_argument("controller hierarchy requires at least one resource binding");
  }

  const auto urdf = build_hierarchy_from_urdf(links, joints);
  std::unordered_map<std::string, std::string> parent;
  parent.reserve(urdf.nodes.size());
  for (const auto & node : urdf.nodes)
  {
    parent.emplace(node.name, node.parent);
  }
  std::unordered_map<std::string, std::string> joint_child;
  joint_child.reserve(joints.size());
  for (const auto & joint : joints)
  {
    if (!joint_child.emplace(joint.name, joint.child_link).second)
    {
      throw std::invalid_argument("duplicate URDF joint: " + joint.name);
    }
  }

  std::vector<std::string> anchors;
  anchors.reserve(bindings.size());
  std::unordered_map<std::string, std::size_t> binding_index;
  binding_index.reserve(bindings.size());
  for (std::size_t i = 0; i < bindings.size(); ++i)
  {
    const auto & binding = bindings[i];
    if (binding.controller_name.empty() ||
        !binding_index.emplace(binding.controller_name, i).second)
    {
      throw std::invalid_argument("controller names must be non-empty and unique");
    }
    std::string anchor = binding.anchor_link;
    for (const auto & joint_name : binding.joint_names)
    {
      const auto joint_it = joint_child.find(joint_name);
      if (joint_it == joint_child.end())
      {
        throw std::invalid_argument(
          "controller '" + binding.controller_name + "' references unknown URDF joint '" +
          joint_name + "'");
      }
      if (anchor.empty())
      {
        anchor = joint_it->second;
      }
      else if (anchor != joint_it->second &&
               !detail::is_ancestor(anchor, joint_it->second, parent) &&
               !detail::is_ancestor(joint_it->second, anchor, parent))
      {
        throw std::invalid_argument(
          "controller '" + binding.controller_name + "' claims joints from unrelated branches");
      }
    }
    if (anchor.empty())
    {
      throw std::invalid_argument(
        "controller '" + binding.controller_name +
        "' needs a joint resource or an anchor_link for automatic placement");
    }
    if (parent.find(anchor) == parent.end())
    {
      throw std::invalid_argument(
        "controller '" + binding.controller_name + "' anchor link does not exist: " + anchor);
    }
    anchors.push_back(anchor);
  }

  std::vector<ControllerHierarchyNode> nodes;
  nodes.reserve(bindings.size());
  for (const auto & binding : bindings)
  {
    nodes.push_back(ControllerHierarchyNode{binding.controller_name, ""});
  }
  for (std::size_t child = 0; child < bindings.size(); ++child)
  {
    std::size_t best_parent = bindings.size();
    std::size_t best_depth = 0;
    for (std::size_t candidate = 0; candidate < bindings.size(); ++candidate)
    {
      if (candidate == child ||
          !detail::is_ancestor(anchors[candidate], anchors[child], parent))
      {
        continue;
      }
      const auto depth = detail::link_depth(anchors[candidate], parent);
      if (best_parent == bindings.size() || depth > best_depth)
      {
        best_parent = candidate;
        best_depth = depth;
      }
    }
    if (best_parent != bindings.size())
    {
      nodes[child].parent = bindings[best_parent].controller_name;
    }
  }
  return nodes;
}

/// Build and validate a plan directly from controller resource bindings.
inline ControllerHierarchyPlan build_controller_plan_from_urdf(
  const std::vector<std::string> & links, const std::vector<UrdfJointEdge> & joints,
  const std::vector<ControllerResourceBinding> & bindings)
{
  const auto nodes = build_controller_hierarchy_from_urdf(links, joints, bindings);
  return build_controller_hierarchy(nodes);
}

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_HIERARCHY_BUILDER_HPP_
