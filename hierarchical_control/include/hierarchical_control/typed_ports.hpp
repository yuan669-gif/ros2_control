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
//       /* reference ports it exports for its parent */ tc::PortList<wheel_target>,
//       /* reference ports it consumes from its parent */ tc::PortList<>,
//       /* actuator ports it writes */ tc::PortList<wheel_torque>>;
//
//     class WheelController : public TypedPortsMixin<WheelController, ports> { ... };
//
// From that one declaration the mixin
//   * synthesizes the string lists the kernel needs (`staged_reference_ports()`,
//     `staged_actuator_ports()`, `staged_state_ports()`), and
//   * derives the `Contract` used by topology_contract's checks (via `contract_of_t`).
//
// Because the strings are GENERATED from the types, they cannot disagree with the dimensions. The
// mixin also provides non-virtual helpers that return the same lists as `string_view`, which are
// allocation-free and usable in a constant expression.
//
// The mixin must be the most-derived class so its overrides win; the accessors are `final`.
// The concrete controller must still implement `update_state_stage` / `update_command_stage`.
//
// STATE PORTS
// -----------
// `staged_state_ports()` is synthesized as "<name>/state" for every exported and every consumed
// port. That matches how the kernel distinguishes an internal state edge from a topology
// reference edge by NAME, and it means a controller routed through the mixin can be run by
// `create_library` without its port names colliding.
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
///   Exported  -- reference ports this controller PRODUCES for its parent
///   Consumed  -- reference ports this controller CONSUMES from its parent
///   Actuators -- hardware command ports this controller writes
template <typename Exported, typename Consumed, typename Actuators>
struct TypedPorts
{
  static_assert(std::is_class_v<Exported>, "typed_ports: Exported must be a PortList");
  static_assert(std::is_class_v<Consumed>, "typed_ports: Consumed must be a PortList");
  static_assert(std::is_class_v<Actuators>, "typed_ports: Actuators must be a PortList");

  using exported = Exported;
  using consumed = Consumed;
  using actuators = Actuators;

  static constexpr std::size_t exported_count = Exported::count;
  static constexpr std::size_t consumed_count = Consumed::count;
  static constexpr std::size_t actuator_count = Actuators::count;
};

/// The `Contract` implied by a port declaration. Actuators are hardware rather than topology
/// edges, so they do not enter the contract -- matching how topology_contract treats them.
template <typename Ports>
struct contract_of;

template <typename Exported, typename Consumed, typename Actuators>
struct contract_of<TypedPorts<Exported, Consumed, Actuators>>
{
  using type = tc::Contract<Exported, Consumed>;
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

template <typename... Exported, typename... Consumed, typename... Actuators>
struct name_lists<TypedPorts<tc::PortList<Exported...>, tc::PortList<Consumed...>,
                             tc::PortList<Actuators...>>>
{
  static constexpr std::array<std::string_view, sizeof...(Exported)> exported = {
    Exported::name()...};
  static constexpr std::array<std::string_view, sizeof...(Consumed)> consumed = {
    Consumed::name()...};
  static constexpr std::array<std::string_view, sizeof...(Actuators)> actuators = {
    Actuators::name()...};
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
    out.reserve(Ports::consumed_count);
    for (const std::string_view name : name_lists<Ports>::consumed)
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
    std::vector<std::string> out;
    out.reserve(Ports::exported_count + Ports::consumed_count);
    for (const std::string_view name : name_lists<Ports>::exported)
    {
      out.emplace_back(name);
      out.back() += "/state";
    }
    for (const std::string_view name : name_lists<Ports>::consumed)
    {
      out.emplace_back(name);
      out.back() += "/state";
    }
    return out;
  }

  static constexpr std::size_t exported_port_count = Ports::exported_count;
  static constexpr std::size_t consumed_port_count = Ports::consumed_count;
  static constexpr std::size_t actuator_port_count = Ports::actuator_count;
};

// ---------------------------------------------------------------------------------------------
// Cross-checking
// ---------------------------------------------------------------------------------------------

/// Compile-time check that a PARENT's exported reference ports are exactly the reference ports its
/// CHILD consumes -- in NAME, in ORDER, and in DIMENSION.
///
/// This is the dimension-carrying counterpart of the kernel's name matching: the kernel compares
/// strings only, so it accepts a position port wired to a velocity port. Comparing dimensions as
/// well is the point of this header.
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
      // `port_key` folds NAME and DIMENSION together, so one comparison covers both.
      hc_topology::same_port_v<A0, B0> &&
      port_lists_agree<tc::PortList<A...>, tc::PortList<B...>>::value>
{
};

/// Compile-time check that a PARENT's exported reference ports are exactly the reference ports its
/// CHILD consumes -- in NAME, in ORDER, and in DIMENSION.
///
/// This is the dimension-carrying counterpart of the kernel's name matching: the kernel compares
/// strings only, so it accepts a position port wired to a velocity port. Comparing dimensions as
/// well is the point of this header.
template <typename ParentPorts, typename ChildPorts>
constexpr bool declarations_are_compatible() noexcept
{
  return port_lists_agree<
    typename ParentPorts::exported, typename ChildPorts::consumed>::value;
}

/// Halt compilation unless a parent/child pair's declarations agree.
template <typename ParentPorts, typename ChildPorts>
constexpr void require_declarations_compatible()
{
  static_assert(
    declarations_are_compatible<ParentPorts, ChildPorts>(),
    "typed_ports: PORT DECLARATION MISMATCH -- the parent's exported reference ports differ from "
    "the reference ports its child consumes, in name, order, or physical dimension");
}

/// Runtime verification (returns a reason when it fails) that a controller's declared port strings
/// match its `TypedPorts`. Intended for tests and for a controller's own start-up assertions.
template <typename ControllerT, typename Ports>
bool verify_ports_match_interface(
  const ControllerT & controller, const char ** reason = nullptr)
{
  auto fail = [&](const char * why)
  {
    if (reason != nullptr) {*reason = why;}
    return false;
  };

  if (!same_names(controller.staged_reference_ports(), name_lists<Ports>::consumed))
  {
    return fail("staged_reference_ports() disagrees with the TypedPorts declaration");
  }
  if (!same_names(controller.staged_actuator_ports(), name_lists<Ports>::actuators))
  {
    return fail("staged_actuator_ports() disagrees with the TypedPorts declaration");
  }
  return true;
}

}  // namespace typed_ports
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_TYPED_PORTS_HPP
