// Copyright 2026
// Licensed under the Apache License, Version 2.0.
//
// Dimensions on the execution group's port declarations.
//
// THE GAP THIS CLOSES
// -------------------
// dimensional_interfaces.hpp gives interfaces a physical dimension as a type, and
// topology_contract.hpp checks dimensions between a parent's and a child's ports. But
// StagedControllerInterface declares its ports as RUNTIME STRINGS:
//
//     virtual std::vector<std::string> staged_state_ports() const;
//     virtual std::vector<std::string> staged_reference_ports() const;
//
// So until now a controller stated its ports twice: once as types (in a Contract, for the checks)
// and once as strings (in the runtime interface, for the kernel), with nothing tying them
// together. Two declarations of the same fact can disagree, and neither layer would notice.
//
// WHAT THIS PROVIDES
// ------------------
// A single declaration site. The controller declares its ports as TYPES:
//
//     using ports = TypedPorts<
//       /* state ports it publishes (its parent's state stage reads them) */ tc::PortList<wheel_travel>,
//       /* reference ports it receives (its parent's command stage writes them) */ tc::PortList<wheel_target>,
//       /* actuator ports it writes */ tc::PortList<wheel_torque>,
//       /* reference ports it writes INTO its children (static check only) */ tc::PortList<tire_target>,
//       /* state ports it READS FROM its children (static check only) */ tc::PortList<tire_travel>>;
//
//     class WheelController : public TypedPortsMixin<WheelController, ports> { ... };
//
// From that one declaration the mixin
//   * synthesizes the string lists the kernel needs (`staged_state_ports()`,
//     `staged_reference_ports()`, `staged_actuator_ports()`), and
//   * derives the `Contract` used by topology_contract's checks (via `contract_of_t`).
//
// Because the strings are GENERATED from the types, they cannot disagree with the dimensions. The
// mixin also provides non-virtual helpers that return the same lists as `string_view`, which are
// allocation-free and usable in a constant expression.
//
// The mixin must be the most-derived class so its overrides win; the accessors are `final`.
// The concrete controller must still implement `update_state_stage` / `update_command_stage`.
//
// WHAT THE KERNEL ACTUALLY USES (this is why the lists must mean what they say)
// -----------------------------------------------------------------------------
// `StagedExecutionGroup` sizes its per-node buffers from these three lists and then hands the
// buffers to the controller's stages:
//
//     state_values_[i].assign(controller.staged_state_ports().size(), 0.0);
//     reference_values_[i].assign(controller.staged_reference_ports().size(), 0.0);
//     actuator_scratch_[i].assign(controller.staged_actuator_ports().size(), 0.0);
//
// The NAMES are not used by the kernel at all -- they are documentation for the controller's own
// stages, which is why `verify_ports_match_interface()` exists as an explicit self-check. The
// COUNTS are load-bearing:
//   * `staged_state_ports()` is the node's OWN state (the parent's state stage is given a view
//     straight into the child's `state_values_`, so a node does not need slots for its children);
//   * `staged_reference_ports()` is the node's OWN reference (written by the parent's command stage).
//
// CORRECTED DECLARATION MODEL (2026-09-24)
// ----------------------------------------
// An earlier revision had only three lists and defined the state list as "exported then consumed,
// each suffixed with /state". That conflated three different things and over-counted:
// a node with one own state and one received reference was given THREE state slots, and a node that
// also wrote a reference into its child was given one more. Because the kernel pre-fills state slots
// with NaN and requires every declared port to be written (review R3), a controller that wrote only
// its real state would have failed EVERY cycle with `state_failed`; and its parent received a
// child-state view padded with slots that are not states at all.
//
// The lists are now separate and mean exactly what the kernel needs. The last two are optional and
// exist only to make the static parent/child checks meaningful, one per edge direction:
//
//     reference edge : Parent::for_children  vs  Child::reference
//     state edge     : Parent::child_state   vs  Child::state
//
// (The previous check compared the parent's whole exported list against the child's reference list,
// so it reported a CORRECT wheel/tire pair as incompatible -- a false positive that
// `test_typed_ports` had encoded as expected behaviour -- and it never checked the state edge at
// all.)
//
// See doc/PORT_DIMENSIONS.md.

#ifndef HIERARCHICAL_CONTROL_TYPED_PORTS_HPP
#define HIERARCHICAL_CONTROL_TYPED_PORTS_HPP

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "hierarchical_control/dimensional_interfaces.hpp"
#include "hierarchical_control/staged_controller_interface.hpp"
#include "hierarchical_control/topology_contract.hpp"

namespace hierarchical_control
{
namespace typed_ports
{
// NOTE: `tc` shadows the enclosing namespace name here, so anything defined directly in
// hierarchical_control::topology_contract (rather than in a nested namespace) is reached through
// `hc_topology`.
namespace hc_topology = topology_contract;
namespace tc = topology_contract;
namespace dim = dimensions;

/// The controller's single port declaration.
///
///   State        -- state ports this controller PUBLISHES; its parent's state stage reads them
///                   straight out of this node's slots.
///   Reference    -- reference ports this controller RECEIVES; its parent's command stage writes
///                   them.
///   Actuators    -- hardware command ports this controller writes (leaves only, in practice).
///   ForChildren  -- reference ports this controller WRITES INTO its children. Optional; it exists
///                   only so `declarations_are_compatible` can check the parent/child edge in name,
///                   order and physical dimension.
template <
  typename State, typename Reference, typename Actuators,
  typename ForChildren = topology_contract::PortList<>,
  typename ChildState = topology_contract::PortList<>>
struct TypedPorts
{
  static_assert(std::is_class_v<State>, "typed_ports: State must be a PortList");
  static_assert(std::is_class_v<Reference>, "typed_ports: Reference must be a PortList");
  static_assert(std::is_class_v<Actuators>, "typed_ports: Actuators must be a PortList");
  static_assert(std::is_class_v<ForChildren>, "typed_ports: ForChildren must be a PortList");
  static_assert(std::is_class_v<ChildState>, "typed_ports: ChildState must be a PortList");

  using state = State;
  using reference = Reference;
  using actuators = Actuators;
  using for_children = ForChildren;
  using child_state = ChildState;

  static constexpr std::size_t state_count = State::count;
  static constexpr std::size_t reference_count = Reference::count;
  static constexpr std::size_t actuator_count = Actuators::count;
  static constexpr std::size_t for_children_count = ForChildren::count;
  static constexpr std::size_t child_state_count = ChildState::count;

  /// How many slots the kernel will allocate for each stage. These are the numbers the kernel
  /// actually reads, so they are named once and used by the mixin, the verifier and the tests.
  static constexpr std::size_t kernel_state_slots = state_count;
  static constexpr std::size_t kernel_reference_slots = reference_count;
  static constexpr std::size_t kernel_actuator_slots = actuator_count;
};

/// The `Contract` implied by a port declaration.
///
/// `Contract<produced, consumed>` carries the node's own state as `produced` and its own reference
/// as `consumed`. Actuators are hardware rather than topology edges, and child-facing references
/// belong to the parent/child edge rather than to this node, so neither enters the contract --
/// matching how topology_contract treats them.
template <typename Ports>
struct contract_of;

template <
  typename State, typename Reference, typename Actuators, typename ForChildren, typename ChildState>
struct contract_of<TypedPorts<State, Reference, Actuators, ForChildren, ChildState>>
{
  using type = tc::Contract<State, Reference>;
};

template <typename Ports>
using contract_of_t = typename contract_of<Ports>::type;

// ---------------------------------------------------------------------------------------------
// String lists, as constexpr string_view arrays
// ---------------------------------------------------------------------------------------------

template <typename PortT>
constexpr std::string_view state_name_of()
{
  return PortT::name();
}

/// The three name lists implied by a declaration, as constexpr arrays of string_view.
/// These are allocation-free and usable in a constant expression, unlike std::vector<std::string>.
template <typename Ports>
struct name_lists;

template <
  typename... State, typename... Reference, typename... Actuators, typename... ForChildren,
  typename... ChildState>
struct name_lists<TypedPorts<
  tc::PortList<State...>, tc::PortList<Reference...>, tc::PortList<Actuators...>,
  tc::PortList<ForChildren...>, tc::PortList<ChildState...>>>
{
  static constexpr std::array<std::string_view, sizeof...(State)> state = {State::name()...};
  static constexpr std::array<std::string_view, sizeof...(Reference)> reference = {
    Reference::name()...};
  static constexpr std::array<std::string_view, sizeof...(Actuators)> actuators = {
    Actuators::name()...};
  static constexpr std::array<std::string_view, sizeof...(ForChildren)> for_children = {
    ForChildren::name()...};
  static constexpr std::array<std::string_view, sizeof...(ChildState)> child_state = {
    ChildState::name()...};
};

/// Compare a runtime list of names against a compile-time list, order-sensitively.
template <std::size_t N>
inline bool same_names(
  const std::vector<std::string> & actual, const std::array<std::string_view, N> & expected)
{
  if (actual.size() != N) {return false;}
  for (std::size_t i = 0; i < N; ++i)
  {
    if (actual[i] != expected[i]) {return false;}
  }
  return true;
}

/// Compare a runtime list against a `PortList` at compile-time indices.
template <typename List, std::size_t... Index>
bool same_names_by_index(
  const std::vector<std::string> & actual, std::index_sequence<Index...>)
{
  return ((actual[Index] == tc::port_at_t<List, Index>::name()) && ...);
}

/// Compare a runtime list of names against a `PortList`, POSITION BY POSITION.
///
/// Comparing lengths alone is not a check: the same length with different ports, or the same ports
/// in a different order, would pass. `verify_ports_match_contract` compares through this overload --
/// an earlier revision compared only `.size()` while its comment claimed name-and-order checking
/// (review item C).
template <typename... Ports>
bool same_names(const std::vector<std::string> & actual, tc::PortList<Ports...>)
{
  if (actual.size() != sizeof...(Ports)) {return false;}
  return same_names_by_index<tc::PortList<Ports...>>(
    actual, std::make_index_sequence<sizeof...(Ports)>{});
}

/// Compare a constexpr list of names against a compile-time list.
template <std::size_t N, std::size_t M>
constexpr bool same_names(
  const std::array<std::string_view, N> & actual,
  const std::array<std::string_view, M> & expected) noexcept
{
  if (N != M) {return false;}
  for (std::size_t i = 0; i < N; ++i)
  {
    if (actual[i] != expected[i]) {return false;}
  }
  return true;
}

/// Render a runtime list of port names, for a diagnostic.
inline std::string render_names(const std::vector<std::string> & names)
{
  if (names.empty()) {return "<none>";}
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (i != 0) {out += ", ";}
    out += names[i];
  }
  return out;
}

/// Render a compile-time `PortList`'s names, for a diagnostic. The pair of overloads is what makes
/// the "actual vs declared" messages below possible; a message that only says "they disagree" forces
/// the caller to go and print both lists by hand.
template <typename... Ports>
std::string render_names(tc::PortList<Ports...>)
{
  const std::array<std::string_view, sizeof...(Ports)> names = {Ports::name()...};
  if (names.empty()) {return "<none>";}
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (i != 0) {out += ", ";}
    out += names[i];
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// The mixin
// ---------------------------------------------------------------------------------------------

/// Implements the string-valued port accessors from a `TypedPorts` declaration.
///
/// `ControllerT` is the concrete controller (CRTP); it must derive from
/// `StagedControllerInterface`, and this mixin must be its MOST-DERIVED base so the `final`
/// overrides take effect.
template <typename ControllerT, typename Ports, typename Base = StagedControllerInterface>
// `virtual` so that a controller which also derives from another ControllerInterfaceBase
// implementation (a test stub, or a real plugin base) sees ONE base subobject.
class TypedPortsMixin : public virtual Base
{
public:
  std::vector<std::string> staged_reference_ports() const final
  {
    std::vector<std::string> out;
    out.reserve(Ports::reference_count);
    for (const std::string_view name : name_lists<Ports>::reference)
    {
      out.emplace_back(name);
    }
    return out;
  }

  std::vector<std::string> staged_actuator_ports() const final
  {
    std::vector<std::string> out;
    out.reserve(Ports::actuator_count);
    for (const std::string_view name : name_lists<Ports>::actuators)
    {
      out.emplace_back(name);
    }
    return out;
  }

  std::vector<std::string> staged_state_ports() const final
  {
    // Exactly the declared state ports, in declaration order. No suffix is appended: the kernel
    // never interprets these names, and the previous "<name>/state" scheme produced strings for
    // ports that are not states at all.
    std::vector<std::string> out;
    out.reserve(Ports::state_count);
    for (const std::string_view name : name_lists<Ports>::state)
    {
      out.emplace_back(name);
    }
    return out;
  }

  /// The declaration itself, so `topology_contract` can check a whole branching tree against it
  /// without knowing this mixin.
  using typed_ports = Ports;

  static constexpr std::size_t exported_port_count = Ports::for_children_count;
  static constexpr std::size_t state_port_count = Ports::state_count;
  static constexpr std::size_t reference_port_count = Ports::reference_count;
  static constexpr std::size_t actuator_port_count = Ports::actuator_count;
};

// ---------------------------------------------------------------------------------------------
// Cross-checking
// ---------------------------------------------------------------------------------------------

/// The REFERENCE edge of a parent/child pair. Thin wrapper over the variadic check in
/// `topology_contract`, which is where the branching case (several children) lives: a parent's
/// declaration is compared against the CONCATENATION of what its children declare, in child order.
template <typename ParentPorts, typename ChildPorts>
constexpr bool reference_declarations_agree() noexcept
{
  return tc::children_references_agree<ParentPorts, ChildPorts>();
}

/// The STATE edge of a parent/child pair.
template <typename ParentPorts, typename ChildPorts>
constexpr bool state_declarations_agree() noexcept
{
  return tc::children_states_agree<ParentPorts, ChildPorts>();
}

/// Both edges of a parent/child pair agree.
template <typename ParentPorts, typename ChildPorts>
constexpr bool declarations_are_compatible() noexcept
{
  return tc::children_declarations_agree<ParentPorts, ChildPorts>();
}

/// Halt compilation unless a parent/child pair's REFERENCE edge agrees.
template <typename ParentPorts, typename ChildPorts>
constexpr void require_reference_declarations_compatible()
{
  static_assert(
    reference_declarations_agree<ParentPorts, ChildPorts>(),
    "typed_ports: REFERENCE EDGE MISMATCH -- the reference ports the parent declares it writes into "
    "its child differ from the reference ports the child declares it receives, in name, order, or "
    "physical dimension");
}

/// Halt compilation unless a parent/child pair's STATE edge agrees.
template <typename ParentPorts, typename ChildPorts>
constexpr void require_state_declarations_compatible()
{
  static_assert(
    state_declarations_agree<ParentPorts, ChildPorts>(),
    "typed_ports: STATE EDGE MISMATCH -- the child state ports the parent declares it consumes "
    "differ from the state ports the child declares it publishes, in name, order, or physical "
    "dimension");
}

/// Halt compilation unless BOTH edges of a parent/child pair agree.
template <typename ParentPorts, typename ChildPorts>
constexpr void require_declarations_compatible()
{
  require_reference_declarations_compatible<ParentPorts, ChildPorts>();
  require_state_declarations_compatible<ParentPorts, ChildPorts>();
}

/// Runtime verification (returns a reason when it fails) that a controller's declared port strings
/// match its `TypedPorts`. Intended for tests and for a controller's own start-up assertions.
///
/// All THREE lists are checked, including the state ports: an earlier revision left the state list
/// unverified, so the documentation's claim that a controller bypassing the mixin is caught here was
/// only true for the reference and actuator lists.
template <typename ControllerT, typename Ports>
bool verify_ports_match_interface(const ControllerT & controller, std::string * reason = nullptr)
{
  auto mismatch = [reason](
                    const char * list_name, const std::string & actual,
                    const std::string & declared)
  {
    if (reason != nullptr)
    {
      *reason = std::string(list_name) + " is [" + actual + "] but the declaration is [" + declared +
                "]";
    }
    return false;
  };

  if (!same_names(controller.staged_reference_ports(), name_lists<Ports>::reference))
  {
    return mismatch(
      "staged_reference_ports()", render_names(controller.staged_reference_ports()),
      render_names(typename Ports::reference{}));
  }
  if (!same_names(controller.staged_state_ports(), name_lists<Ports>::state))
  {
    return mismatch(
      "staged_state_ports()", render_names(controller.staged_state_ports()),
      render_names(typename Ports::state{}));
  }
  if (!same_names(controller.staged_actuator_ports(), name_lists<Ports>::actuators))
  {
    return mismatch(
      "staged_actuator_ports()", render_names(controller.staged_actuator_ports()),
      render_names(typename Ports::actuators{}));
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// Verifying a controller against the CONTRACT it was bound with
// ---------------------------------------------------------------------------------------------

/// Runtime check that a controller's port strings agree with a `Contract` (rather than with a
/// `TypedPorts` declaration).
///
/// The topology binding checks ownership and dimensions at compile time from the `Contract`, while
/// the kernel sizes its buffers from the controller's runtime strings. This is the function that
/// closes that last gap for a controller that does NOT use the mixin (the mixin makes a mismatch
/// impossible by construction, because it generates the strings):
///
///   `staged_state_ports()`     must equal the contract's PRODUCED names, in order
///   `staged_reference_ports()` must equal the contract's CONSUMED names, in order
///
/// Actuator ports cannot be checked this way: the `Contract` deliberately excludes hardware
/// actuators, so there is nothing to compare them against.
template <typename ContractT, typename ControllerT>
bool verify_ports_match_contract(const ControllerT & controller, std::string * reason = nullptr)
{
  static_assert(tc::is_contract_v<ContractT>, "typed_ports: expected a topology_contract::Contract");
  // The message names BOTH lists. "They disagree" is not enough for a caller whose only clue is the
  // thrown string: the whole point of this check is to catch a hand-written controller, and the
  // first thing anyone needs is which port is wrong.
  auto mismatch = [reason](
                    const char * list_name, const std::string & actual,
                    const std::string & declared)
  {
    if (reason != nullptr)
    {
      *reason = std::string(list_name) + " is [" + actual +
                "] but the contract it was bound with declares [" + declared +
                "]; the kernel would size its buffers from the first list while the topology was "
                "checked against the second";
    }
    return false;
  };

  if (!same_names(controller.staged_state_ports(), typename ContractT::produced{}))
  {
    return mismatch(
      "staged_state_ports()", render_names(controller.staged_state_ports()),
      render_names(typename ContractT::produced{}));
  }
  if (!same_names(controller.staged_reference_ports(), typename ContractT::consumed{}))
  {
    return mismatch(
      "staged_reference_ports()", render_names(controller.staged_reference_ports()),
      render_names(typename ContractT::consumed{}));
  }
  return true;
}

}  // namespace typed_ports
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TYPED_PORTS_HPP
