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
// What was still missing is the last hop: the binding carries controller instances while
// StagedExecutionGroup::create_library wants a typed StagedControllerInterface*. This header
// performs that adaptation.
//
// POINTER SAFETY (review R5)
// --------------------------
// An earlier revision exposed instance pointers as `void*` and recovered them with
// `static_cast<StagedControllerInterface*>(raw)`. That is wrong as soon as a controller inherits
// BOTH ControllerInterfaceBase and StagedControllerInterface: converting the object pointer to the
// SECOND base requires an address adjustment, and a void* round trip loses it (measured: the
// recovered pointer differed from the correctly adjusted one).
//
// The fix keeps the typed pointer on the path. The binding stores a `ControllerInterfaceBase*`
// together with a typed accessor that performs `dynamic_cast<StagedControllerInterface*>` on it --
// a conversion the runtime can perform correctly because the object is intact. No `void*` is
// involved anywhere in the public entry points, so no unrelated pointer can enter them either.
// bind_controller additionally rejects a controller type that is not derived from
// ControllerInterfaceBase at compile time.
//
// It deliberately lives in its own header. `topology_contract.hpp` stays free of
// `controller_interface` and rclcpp so it can be included in a lightweight translation unit; the
// coupling to the kernel exists only here, and only for callers that actually run a group.
//
// WHAT IS CHECKED WHERE (no check is duplicated)
// ----------------------------------------------
//   * compile time : acyclicity (Node types), ownership and port dimensions (binding), and the
//                    base-class relationship between the controller type and ControllerInterfaceBase.
//   * here         : row lengths before indexing; non-null instances; well-formed rows (unique
//                    names, one root, parents exist, no self-parent) with a named reason.
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
#include <utility>
#include <type_traits>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/staged_execution_group.hpp"
#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/typed_ports.hpp"

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

/// Validate a row set and report a named reason. LENGTHS ARE CHECKED BEFORE ANY INDEXING, so a
/// malformed `SpecRows` cannot cause an out-of-bounds read (review R5).
inline void require_rows_are_usable(const tc::SpecRows & rows)
{
  if (rows.names.empty())
  {
    throw std::invalid_argument("topology_binding: cannot build a group from an empty binding");
  }
  if (rows.instances.size() != rows.names.size() || rows.parents.size() != rows.names.size())
  {
    throw std::invalid_argument(
      "topology_binding: names, instances and parents must have equal length");
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
      "topology_binding: a binding must describe exactly one tree, so exactly one root is expected");
  }
}

/// Recover a `StagedControllerInterface*` from a bound instance pointer. The stored pointer is a
/// `ControllerInterfaceBase*` for an intact object, so the downcast is well defined and performs
/// any address adjustment the object layout requires.
inline StagedControllerInterface * as_staged(controller_interface::ControllerInterfaceBase * base)
{
  if (base == nullptr) {return nullptr;}
  return dynamic_cast<StagedControllerInterface *>(base);
}

/// Convert a binding's rows into the kernel's Spec.
///
/// Throws std::invalid_argument when the rows are unusable; see `require_rows_are_usable`.
inline StagedExecutionGroup::Spec to_library_spec(const tc::SpecRows & rows)
{
  require_rows_are_usable(rows);

  StagedExecutionGroup::Spec spec;
  spec.names = rows.names;
  spec.parents = rows.parents;
  spec.instances.reserve(rows.instances.size());
  for (auto * base : rows.instances)
  {
    auto * staged = as_staged(base);
    if (staged == nullptr)
    {
      throw std::invalid_argument(
        "topology_binding: a bound controller does not implement StagedControllerInterface");
    }
    spec.instances.push_back(staged);
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

template <typename Binding>
bool verify_binding_ports(const Binding & binding, std::string * reason);

/// Verify every child subtree, stopping at the first failure.
template <typename Binding, std::size_t... Index>
bool verify_children_ports(
  const Binding & binding, std::string * reason, std::index_sequence<Index...>)
{
  bool ok = true;
  // The fold short-circuits: once a child fails, later children are not visited.
  ((ok = ok && verify_binding_ports(tc::child_at<Index>(binding.children), reason)), ...);
  return ok;
}

/// Verify that every node's RUNTIME port lists agree with the contract it was bound with.
///
/// This is the last hop of the metaprogramming chain. It walks the WHOLE binding tree (every child
/// subtree, short-circuiting at the first failure), because a mismatch in the second child is just as
/// fatal as one in the first. The compile-time layer checks ownership and dimensions from the
/// `Contract`; the kernel sizes its per-node buffers from the strings the controller reports. A
/// controller that uses `TypedPortsMixin` generates those strings from its declaration, so a mismatch
/// is impossible by construction. A controller that hand-writes them can still disagree, and this
/// function is how such a controller is caught:
///
///     std::string reason;
///     if (!topology_binding::verify_binding_ports(binding, &reason)) { throw ...; }
///
/// It is a runtime call because building a controller instance is not a constant expression, and it
/// is not on any path the kernel runs per cycle. `create_library_group` runs it for you (review item
/// C); call it directly only when you build the group some other way.
///
/// Actuator ports cannot be verified here: the `Contract` deliberately excludes hardware actuators,
/// so there is nothing to compare them against. Use
/// `typed_ports::verify_ports_match_interface()` for a full three-list check when the declaration
/// type is available.
template <typename Binding>
bool verify_binding_ports(const Binding & binding, std::string * reason)
{
  const auto * staged = as_staged(binding.instance);
  if (staged == nullptr)
  {
    if (reason != nullptr)
    {
      *reason = "node '" + std::string(Binding::name()) + "' has no staged interface";
    }
    return false;
  }

  using contract_type = typename Binding::contract_type;
  if (!typed_ports::verify_ports_match_contract<contract_type>(*staged, reason))
  {
    // `verify_ports_match_contract` writes the actual and declared lists; only the node identity is
    // added here, so the caller learns which node of the tree is wrong and why.
    if (reason != nullptr)
    {
      *reason = "node '" + std::string(Binding::name()) + "': " + *reason;
    }
    return false;
  }

  if constexpr (Binding::child_count > 0)
  {
    // Every child's message already names its own node, so a failure deeper in the tree propagates.
    return verify_children_ports(
      binding, reason, std::make_index_sequence<Binding::child_count>{});
  }
  if (reason != nullptr) {reason->clear();}
  return true;
}

/// Build a library-hosted execution group from a compile-time binding, CHECKING THE PORTS.
///
/// This is the entry point to use. It runs, in order:
///
///   1. the compile-time checks (`require_ports_are_owned`, and in `compose` the nesting vs declared
///      parent agreement) -- a violation does not compile;
///   2. `verify_binding_ports` at run time: every node's OWN port strings must equal the contract it
///      was bound with, in name and order;
///   3. the kernel's own plan validation.
///
/// An earlier revision left step 2 to the caller, so the checked path was optional and a controller
/// whose runtime port lists disagreed with its contract built a group whose buffers were sized from
/// the wrong declaration (review item C: a stub that reported one state port against an empty
/// contract still built and ran). Use `create_library_group_unchecked` only for a deliberately
/// malformed debug stub, and say why at the call site.
template <typename Binding>
std::shared_ptr<StagedExecutionGroup> create_library_group(
  const Binding & binding, std::int64_t max_age_ns = 0)
{
  std::string reason;
  if (!verify_binding_ports(binding, &reason))
  {
    throw std::invalid_argument(
      "topology_binding: refusing to build a group whose runtime ports disagree with the checked "
      "topology -- " + reason);
  }
  return StagedExecutionGroup::create_library(to_library_spec(binding), max_age_ns);
}

/// Build without the runtime port check. For debug stubs that intentionally disagree.
template <typename Binding>
std::shared_ptr<StagedExecutionGroup> create_library_group_unchecked(
  const Binding & binding, std::int64_t max_age_ns = 0)
{
  return StagedExecutionGroup::create_library(to_library_spec(binding), max_age_ns);
}

}  // namespace topology_binding
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TOPOLOGY_BINDING_HPP
