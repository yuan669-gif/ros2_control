// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#ifndef CONTROLLER_MANAGER__URDF_HIERARCHY_HPP_
#define CONTROLLER_MANAGER__URDF_HIERARCHY_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager/hierarchy.hpp"

namespace controller_manager
{

/// Minimal URDF link/joint record used to derive a controller hierarchy.
struct UrdfJointEdge
{
  std::string name;
  std::string parent_link;
  std::string child_link;
};

/// An adapter result: one controller node per link, with joint leaves marked as controllable.
struct UrdfHierarchyResult
{
  std::vector<ControllerHierarchyNode> nodes;
  ControllerHierarchyPlan plan;
};

/// Derive a tree-shaped controller skeleton from a URDF link/joint tree.
/**
 * This function deliberately does not infer controller algorithms. It creates one node for every
 * link and uses the URDF parent/child relation as the default hierarchy. A higher layer can map
 * ros2_control joint interfaces to leaves and replace selected link nodes with semantic composite
 * controller plugins.
 */
inline UrdfHierarchyResult build_hierarchy_from_urdf(
  const std::vector<std::string> & links, const std::vector<UrdfJointEdge> & joints)
{
  std::unordered_map<std::string, std::size_t> link_index;
  link_index.reserve(links.size());
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    if (links[i].empty() || !link_index.emplace(links[i], i).second)
    {
      throw std::invalid_argument("URDF links must be non-empty and unique");
    }
  }

  std::vector<ControllerHierarchyNode> nodes;
  nodes.reserve(links.size());
  std::vector<std::string> parent(links.size());
  for (const auto & joint : joints)
  {
    const auto parent_it = link_index.find(joint.parent_link);
    const auto child_it = link_index.find(joint.child_link);
    if (parent_it == link_index.end() || child_it == link_index.end())
    {
      throw std::invalid_argument("URDF joint references a link that does not exist");
    }
    auto & child_parent = parent[child_it->second];
    if (!child_parent.empty())
    {
      throw std::invalid_argument("URDF child link has multiple parent joints: " + joint.child_link);
    }
    child_parent = joint.parent_link;
  }
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    nodes.push_back(ControllerHierarchyNode{links[i], parent[i]});
  }
  return UrdfHierarchyResult{nodes, build_controller_hierarchy(nodes)};
}

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__URDF_HIERARCHY_HPP_
