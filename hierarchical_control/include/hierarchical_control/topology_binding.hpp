// Copyright 2026
// Licensed under the Apache License, Version 2.0.
//
// The final join: a checked compile-time topology becomes a RUNNING execution group.
//
// The chain so far
// ----------------
//   static_topology.hpp        -- acyclic hierarchy as a type chain
//   dimensional_interfaces.hpp -- physical dimensions on interfaces
//   topology_contract.hpp      -- one binding drives both the checks and the runtime Spec rows
//
// What was still missing is the last hop: `SpecRows` carries `void*` instances, while
// StagedExecutionGroup::create_library wants `StagedControllerInterface*`. This header performs
// that adaptation and nothing else.
//
// It deliberately lives in its own header. `topology_contract.hpp` stays free of
// `controller_interface` and rclcpp so it can be included in a lightweight translation unit; the
// coupling to the kernel exists only here, and only for callers that actually run a group.
//
// WHAT IS CHECKED WHERE (no check is duplicated)
// ----------------------------------------------
//   * compile time : acyclicity (Node types), ownership and port dimensions (binding),
//                    and that a binding whose own rows say "two roots" is rejected --
//                    a single binding is a single chain and cannot have two.
//   * here         : non-null instances; well-formed rows (length, unique names, one root,
//                    parents exist, no self-parent) with a diagnostic that names the problem.
//   * in the kernel: unknown parents, exactly one root, no cycle, no unreachable node, and the
//                    controller interface contract (sinks/sources).
//
// The kernel's own validation is NOT bypassed: `create_library` still runs it. This adapter adds
// earlier, better-described failures for the cases the binding can already see.
//
// See doc/TOPOLOGY_CONTRACT_JOIN.md.

#ifndef HIERARCHICAL_CONTROL_TOPOLOGY_BINDING_HPP
#define HIERARCHICAL_CONTROL_TOPOLOGY_BINDING_HPP

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/staged_execution_group.hpp"
#include "hierarchical_control/topology_contract.hpp"

namespace hierarchical_control
{
namespace topology_binding
{
namespace tc = topology_contract;

/// How many rows declare themselves a root (empty parent).
inline std::size_t root_count(const tc::SpecRows & rows)
{
  std::size_t roots = 0;
  for (const std::string & parent : rows.parents)
  {
    if (parent.empty()) {++roots;}
  }
  return roots;
}

/// Convert a binding's rows into the kernel's Spec.
///
/// Throws std::invalid_argument when the binding is empty, contains a null instance, or its rows
/// are not well formed. A single binding is a single chain, so more than one root in its rows is a
/// contradiction in the binding itself and is rejected here rather than left to the kernel.
inline StagedExecutionGroup::Spec to_library_spec(const tc::SpecRows & rows)
{
  if (rows.names.empty())
  {
    throw std::invalid_argument("topology_binding: cannot build a group from an empty binding");
  }
  for (std::size_t i = 0; i < rows.instances.size(); ++i)
  {
    if (rows.instances[i] == nullptr)
    {
      throw std::invalid_argument(
        "topology_binding: node '" + rows.names[i] + "' has a null controller instance");
    }
  }

  std::string reason;
  if (!tc::rows_are_well_formed(rows, &reason))
  {
    throw std::invalid_argument("topology_binding: malformed plan: " + reason);
  }
  if (root_count(rows) != 1)
  {
    throw std::invalid_argument(
      "topology_binding: a single binding must describe exactly one chain, so exactly one root is "
      "expected");
  }

  StagedExecutionGroup::Spec spec;
  spec.names = rows.names;
  spec.parents = rows.parents;
  spec.instances.reserve(rows.instances.size());
  for (void * raw : rows.instances)
  {
    spec.instances.push_back(static_cast<StagedControllerInterface *>(raw));
  }
  return spec;
}

/// Build the kernel's Spec from a compile-time binding, checking the binding at compile time first.
///
/// This is the intended entry point: one compile-time topology definition, one call, a running
/// group. The kernel still validates the plan it receives; in particular it enforces "exactly one
/// root", which this layer deliberately does not duplicate.
template <typename Binding>
StagedExecutionGroup::Spec to_library_spec(const Binding & binding)
{
  static_assert(
    tc::binding_depth<Binding>() >= 1, "topology_binding: the binding must have at least one node");

  // Compile-time: ownership of every declared port (and, transitively, the acyclicity of the
  // Node types and the dimensions carried by the Port types).
  tc::require_ports_are_owned<Binding>();

  // Runtime: the binding's own shape, reported with a diagnostic that names the problem.
  return to_library_spec(tc::build_spec_rows(binding));
}

/// Create a library-hosted execution group directly from a compile-time binding.
///
/// The returned group is the same kernel object `create_library` produces; this only removes the
/// hand-written Spec and the chance of it drifting from the checked topology.
template <typename Binding>
std::shared_ptr<StagedExecutionGroup> create_library_group(
  const Binding & binding, std::int64_t max_age_ns = 0)
{
  return StagedExecutionGroup::create_library(to_library_spec(binding), max_age_ns);
}
}  // namespace topology_binding
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TOPOLOGY_BINDING_HPP
