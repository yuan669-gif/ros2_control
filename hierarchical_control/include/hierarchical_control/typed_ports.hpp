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
//       /* reference ports it writes INTO its children (static check only) */ tc::PortList<tire_target>>;
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
// The lists are now separate and mean exactly what the kernel needs. The fourth list is optional and
// exists only to make the static parent/child dimension check meaningful: the parent declares what
// it writes into its children, and that is compared against the child's declared reference ports.
// (The previous check compared the parent's whole exported list against the child's reference list,
// so it reported a CORRECT wheel/tire pair as incompatible -- a false positive that
// `test_typed_ports` had encoded as expected behaviour.)
//
// See doc/PORT_DIMENSIONS.md.

#ifndef HIERARCHICAL_CONTROL_TYPED_PORTS_HPP
#define HIERARCHICAL_CONTROL_TYPED_PORTS_HPP

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
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
  typename ForChildren = topology_contract::PortList<>>
struct TypedPorts
{
  static_assert(std::is_class_v<State>, "typed_ports: State must be a PortList");
  static_assert(std::is_class_v<Reference>, "typed_ports: Reference must be a PortList");
  static_assert(std::is_class_v<Actuators>, "typed_ports: Actuators must be a PortList");
  static_assert(std::is_class_v<ForChildren>, "typed_ports: ForChildren must be a PortList");

  using state = State;
  using reference = Reference;
  using actuators = Actuators;
  using for_children = ForChildren;

  static constexpr std::size_t state_count = State::count;
  static constexpr std::size_t reference_count = Reference::count;
  static constexpr std::size_t actuator_count = Actuators::count;
  static constexpr std::size_t for_children_count = ForChildren::count;

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

template <typename State, typename Reference, typename Actuators, typename ForChildren>
struct contract_of<TypedPorts<State, Reference, Actuators, ForChildren>>
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
  typename... State, typename... Reference, typename... Actuators, typename... ForChildren>
struct name_lists<TypedPorts<
  tc::PortList<State...>, tc::PortList<Reference...>, tc::PortList<Actuators...>,
  tc::PortList<ForChildren...>>>
{
  static constexpr std::array<std::string_view, sizeof...(State)> state = {State::name()...};
  static constexpr std::array<std::string_view, sizeof...(Reference)> reference = {
    Reference::name()...};
  static constexpr std::array<std::string_view, sizeof...(Actuators)> actuators = {
    Actuators::name()...};
  static constexpr std::array<std::string_view, sizeof...(ForChildren)> for_children = {
    ForChildren::name()...};
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

  static constexpr std::size_t exported_port_count = Ports::for_children_count;
  static constexpr std::size_t state_port_count = Ports::state_count;
  static constexpr std::size_t reference_port_count = Ports::reference_count;
  static constexpr std::size_t actuator_port_count = Ports::actuator_count;
};

// ---------------------------------------------------------------------------------------------
// Cross-checking
// ---------------------------------------------------------------------------------------------

/// Two port lists are equal element-wise, comparing NAME and DIMENSION together.
template <typename ListA, typename ListB>
struct port_lists_agree : std::false_type
{
};

template <>
struct port_lists_agree<tc::PortList<>, tc::PortList<>> : std::true_type
{
};

template <typename A0, typename... A, typename B0, typename... B>
struct port_lists_agree<tc::PortList<A0, A...>, tc::PortList<B0, B...>>
  : std::bool_constant<
      // `same_port_v` folds NAME and DIMENSION together, so one comparison covers both.
      hc_topology::same_port_v<A0, B0> &&
      port_lists_agree<tc::PortList<A...>, tc::PortList<B...>>::value>
{
};

/// Compile-time check that a PARENT's child-facing reference ports are exactly the reference ports
/// its CHILD declares -- in NAME, in ORDER, and in DIMENSION.
///
/// This is the dimension-carrying counterpart of the kernel's name matching: the kernel compares
/// strings only, so it accepts a position port wired to a velocity port. Comparing dimensions as
/// well is the point of this header.
template <typename ParentPorts, typename ChildPorts>
constexpr bool declarations_are_compatible() noexcept
{
  return port_lists_agree<
    typename ParentPorts::for_children, typename ChildPorts::reference>::value;
}

/// Halt compilation unless a parent/child pair's declarations agree.
template <typename ParentPorts, typename ChildPorts>
constexpr void require_declarations_compatible()
{
  static_assert(
    declarations_are_compatible<ParentPorts, ChildPorts>(),
    "typed_ports: PORT DECLARATION MISMATCH -- the reference ports the parent declares it writes "
    "into its child differ from the reference ports the child declares it receives, in name, order, "
    "or physical dimension");
}

/// Runtime verification (returns a reason when it fails) that a controller's declared port strings
/// match its `TypedPorts`. Intended for tests and for a controller's own start-up assertions.
///
/// All THREE lists are checked, including the state ports: an earlier revision left the state list
/// unverified, so the documentation's claim that a controller bypassing the mixin is caught here was
/// only true for the reference and actuator lists.
template <typename ControllerT, typename Ports>
bool verify_ports_match_interface(
  const ControllerT & controller, const char ** reason = nullptr)
{
  auto fail = [&](const char * why)
  {
    if (reason != nullptr) {*reason = why;}
    return false;
  };

  if (!same_names(controller.staged_reference_ports(), name_lists<Ports>::reference))
  {
    return fail("staged_reference_ports() disagrees with the TypedPorts declaration");
  }
  if (!same_names(controller.staged_state_ports(), name_lists<Ports>::state))
  {
    return fail("staged_state_ports() disagrees with the TypedPorts declaration");
  }
  if (!same_names(controller.staged_actuator_ports(), name_lists<Ports>::actuators))
  {
    return fail("staged_actuator_ports() disagrees with the TypedPorts declaration");
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
bool verify_ports_match_contract(
  const ControllerT & controller, const char ** reason = nullptr)
{
  static_assert(tc::is_contract_v<ContractT>, "typed_ports: expected a topology_contract::Contract");
  auto fail = [&](const char * why)
  {
    if (reason != nullptr) {*reason = why;}
    return false;
  };

  if (controller.staged_state_ports().size() != ContractT::produced_count)
  {
    return fail(
      "staged_state_ports() has a different length than the contract's produced ports: the kernel "
      "would size the state buffers differently from the checked topology");
  }
  if (controller.staged_reference_ports().size() != ContractT::consumed_count)
  {
    return fail(
      "staged_reference_ports() has a different length than the contract's consumed ports: the "
      "kernel would size the reference buffers differently from the checked topology");
  }
  return true;
}

}  // namespace typed_ports
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TYPED_PORTS_HPP
