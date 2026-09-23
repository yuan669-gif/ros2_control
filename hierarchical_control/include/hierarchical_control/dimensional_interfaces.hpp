// Copyright 2026
// Licensed under the Apache License, Version 2.0.
//
// Compile-time dimensional analysis for controller interfaces.
//
// WHY THIS EXISTS
// ---------------
// ros2_control identifies interfaces by STRING NAME. A parent declares that it consumes
// "<child>/state", the child declares that it exports "<child>/state", and the two declarations
// are never compared by the type system. In this project that gap was measured directly:
// controller_manager/test/test_upstream_ordering.cpp declares a state edge on a name that NOTHING
// exports, and configure_controller accepts it without complaint (doc/BIDIRECTIONAL_EDGE_ANALYSIS.md
// section 9). The mismatch is invisible until it silently changes scheduling behaviour.
//
// This header gives interfaces a physical dimension as a TYPE, so that such a mismatch is a
// compile error. Two things are checked:
//
//   1. A command interface can only be connected to a reference interface of the SAME dimension.
//      Connecting a position command to a velocity reference does not compile.
//   2. A state interface can only be consumed at the dimension the producer exports.
//
// The dimension is a vector of exponents over the SI base quantities, so it is a genuine algebra
// rather than a flat tag: Position / Time automatically reduces to Velocity, and so on.
// Dimensional reduction is what makes the check meaningful rather than cosmetic -- a flat "kind"
// enum would not know that [m] and [m/s] differ, which is the case that actually matters here.
//
// SCOPE AND LIMITS (stated up front)
// ----------------------------------
//   * This is a compile-time device for statically declared interfaces. Interfaces that arrive
//     from YAML or from a plugin's runtime declaration are NOT covered; the string-name path and
//     the runtime checks remain. Positioning is "compile-time capability + runtime fallback".
//   * Dimensions, not units: metres and millimetres have the same dimension and are accepted.
//     Catching that requires a scale factor, which is deliberately out of scope.
//   * Output scaling and frame conventions (an angular vs linear quantity of the same exponent
//     vector, e.g. a wheel radius factor) are semantic and are NOT checked.
//
// See doc/DIMENSIONAL_INTERFACES.md.

#ifndef HIERARCHICAL_CONTROL_DIMENSIONAL_INTERFACES_HPP
#define HIERARCHICAL_CONTROL_DIMENSIONAL_INTERFACES_HPP

#include <cstddef>
#include <ratio>
#include <type_traits>

namespace hierarchical_control
{
namespace dimensions
{
// ---------------------------------------------------------------------------------------------
// Dimension vector: one exponent per SI base quantity.
//
// Exponents are std::ratio so that integer and rational powers are both representable.
// ---------------------------------------------------------------------------------------------
template <typename Length, typename Mass, typename Time, typename Angle>
struct Dimension
{
  using length = Length;
  using mass = Mass;
  using time = Time;
  using angle = Angle;

  static constexpr int length_exp = Length::num / Length::den;
  static constexpr int mass_exp = Mass::num / Mass::den;
  static constexpr int time_exp = Time::num / Time::den;
  static constexpr int angle_exp = Angle::num / Angle::den;

  /// Is every exponent zero? (A dimensionless quantity.)
  static constexpr bool is_dimensionless() noexcept
  {
    return length_exp == 0 && mass_exp == 0 && time_exp == 0 && angle_exp == 0;
  }

  /// Human-readable rendering, handy in static_assert diagnostics.
  static constexpr const char * spelling() noexcept
  {
    if (length_exp == 1 && mass_exp == 0 && time_exp == 0 && angle_exp == 0) {return "length [m]";}
    if (length_exp == 0 && mass_exp == 0 && time_exp == -1 && angle_exp == 1)
    {
      return "angular velocity [rad/s]";
    }
    if (length_exp == 1 && mass_exp == 0 && time_exp == -1 && angle_exp == 0)
    {
      return "linear velocity [m/s]";
    }
    if (length_exp == 0 && mass_exp == 0 && time_exp == -2 && angle_exp == 1)
    {
      return "angular acceleration [rad/s^2]";
    }
    if (length_exp == 1 && mass_exp == 0 && time_exp == -2 && angle_exp == 0)
    {
      return "linear acceleration [m/s^2]";
    }
    if (length_exp == 1 && mass_exp == 1 && time_exp == -2 && angle_exp == 0)
    {
      return "force [N]";
    }
    if (length_exp == 2 && mass_exp == 1 && time_exp == -2 && angle_exp == 0)
    {
      return "energy [J = N*m]";
    }
    if (length_exp == 2 && mass_exp == 1 && time_exp == -2 && angle_exp == -1)
    {
      return "torque [N*m/rad]";
    }
    if (length_exp == 2 && mass_exp == 1 && time_exp == -3 && angle_exp == 0)
    {
      return "power [W]";
    }
    if (length_exp == 1 && mass_exp == 0 && time_exp == -3 && angle_exp == 0)
    {
      return "linear jerk [m/s^3]";
    }
    if (length_exp == 0 && mass_exp == 0 && time_exp == -3 && angle_exp == 1)
    {
      return "angular jerk [rad/s^3]";
    }
    if (length_exp == 0 && mass_exp == 0 && time_exp == 0 && angle_exp == 1)
    {
      return "angle [rad]";
    }
    return "dimension";
  }
};

// ---------------------------------------------------------------------------------------------
// Algebra
// ---------------------------------------------------------------------------------------------

namespace detail
{
template <typename A, typename B>
using ratio_add_t = std::ratio_add<A, B>;

template <typename A, typename B>
using ratio_sub_t = std::ratio_subtract<A, B>;
}  // namespace detail

/// Product of two dimensions: exponents add.
template <typename D1, typename D2>
using multiply_t = Dimension<
  detail::ratio_add_t<typename D1::length, typename D2::length>,
  detail::ratio_add_t<typename D1::mass, typename D2::mass>,
  detail::ratio_add_t<typename D1::time, typename D2::time>,
  detail::ratio_add_t<typename D1::angle, typename D2::angle>>;

/// Quotient of two dimensions: exponents subtract.
template <typename D1, typename D2>
using divide_t = Dimension<
  detail::ratio_sub_t<typename D1::length, typename D2::length>,
  detail::ratio_sub_t<typename D1::mass, typename D2::mass>,
  detail::ratio_sub_t<typename D1::time, typename D2::time>,
  detail::ratio_sub_t<typename D1::angle, typename D2::angle>>;

/// Integer power of a dimension.
template <typename D, int Power>
using power_t = Dimension<
  std::ratio_multiply<typename D::length, std::ratio<Power>>,
  std::ratio_multiply<typename D::mass, std::ratio<Power>>,
  std::ratio_multiply<typename D::time, std::ratio<Power>>,
  std::ratio_multiply<typename D::angle, std::ratio<Power>>>;

template <typename D1, typename D2>
struct same_dimension
  : std::bool_constant<
      D1::length_exp == D2::length_exp && D1::mass_exp == D2::mass_exp &&
      D1::time_exp == D2::time_exp && D1::angle_exp == D2::angle_exp>
{
};

template <typename D1, typename D2>
inline constexpr bool same_dimension_v = same_dimension<D1, D2>::value;

// ---------------------------------------------------------------------------------------------
// Named dimensions
// ---------------------------------------------------------------------------------------------

namespace base
{
using dimensionless = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>>;
using length = Dimension<std::ratio<1>, std::ratio<0>, std::ratio<0>, std::ratio<0>>;
using mass = Dimension<std::ratio<0>, std::ratio<1>, std::ratio<0>, std::ratio<0>>;
using time = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>;
using angle = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;
}  // namespace base

// Positions and their derivatives. Angular and linear are deliberately distinct dimensions.
using Position = base::length;                                        // [m]
using LinearVelocity = divide_t<base::length, base::time>;            // [m/s]
using LinearAcceleration = divide_t<base::length, power_t<base::time, 2>>;  // [m/s^2]
using LinearJerk = divide_t<base::length, power_t<base::time, 3>>;    // [m/s^3]
using Angle = base::angle;                                            // [rad]
using AngularVelocity = divide_t<base::angle, base::time>;            // [rad/s]
using AngularAcceleration = divide_t<base::angle, power_t<base::time, 2>>;  // [rad/s^2]
using AngularJerk = divide_t<base::angle, power_t<base::time, 3>>;    // [rad/s^3]
using Force = multiply_t<base::mass, divide_t<base::length, power_t<base::time, 2>>>;   // [N]
// Torque must carry the angle: work is F*d*theta, so torque is an ENERGY PER RADIAN. Without the
// angle exponent this would be the same dimension as energy, and a torque command could be wired
// to an energy reference.
using Torque = divide_t<
  multiply_t<Force, base::length>, base::angle>;                      // [N*m/rad]
using Energy = multiply_t<Force, base::length>;                       // [J = N*m]
using Power = divide_t<Energy, base::time>;                           // [W]
using Dimensionless = base::dimensionless;

// ---------------------------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------------------------

/// The physical role of an interface. The dimension is part of the type, so two interfaces of the
/// same role but different dimension are different types.
enum class Role
{
  command,   ///< a value this controller writes (its output, or its reference for a child)
  reference, ///< a value this controller reads from its parent
  state,     ///< a value this controller reads from a child / the hardware
};

/// An interface handle: a compile-time contract, not a runtime binding.
template <Role R, typename Dim>
struct Interface
{
  static constexpr Role role = R;
  using dimension = Dim;
  static constexpr const char * dimension_name = Dim::spelling();
};

// ---------------------------------------------------------------------------------------------
// Connection rules
// ---------------------------------------------------------------------------------------------

/// A connection is legal iff the two dimensions agree. This is the whole rule, and it is the rule
/// the string-name world cannot express.
template <typename ParentInterface, typename ChildInterface>
inline constexpr bool connectable_v = same_dimension_v<
  typename ParentInterface::dimension, typename ChildInterface::dimension>;

/// Reference edge: the parent writes a command interface of dimension D; the child exports a
/// reference interface that must have dimension D. A legal pair is `connectable`.
template <typename CommandInterface, typename ReferenceInterface>
struct reference_edge_is_legal
  : std::bool_constant<
      CommandInterface::role == Role::command && ReferenceInterface::role == Role::reference &&
      connectable_v<CommandInterface, ReferenceInterface>>
{
};

/// State edge: the child exports a state interface; the parent consumes one of the same dimension.
template <typename ExportedState, typename ConsumedState>
struct state_edge_is_legal
  : std::bool_constant<
      ExportedState::role == Role::state && ConsumedState::role == Role::state &&
      connectable_v<ExportedState, ConsumedState>>
{
};

template <typename CommandInterface, typename ReferenceInterface>
inline constexpr bool reference_edge_is_legal_v =
  reference_edge_is_legal<CommandInterface, ReferenceInterface>::value;

template <typename ExportedState, typename ConsumedState>
inline constexpr bool state_edge_is_legal_v =
  state_edge_is_legal<ExportedState, ConsumedState>::value;

/// The check a controller performs on itself: every reference it writes into a child must have the
/// dimension the child actually exports, and vice versa.
template <typename CommandInterface, typename ReferenceInterface>
constexpr void require_reference_edge()
{
  static_assert(
    CommandInterface::role == Role::command,
    "dimensional_interfaces: the first argument of a reference edge must be a command interface");
  static_assert(
    ReferenceInterface::role == Role::reference,
    "dimensional_interfaces: the second argument of a reference edge must be a reference interface");
  static_assert(
    connectable_v<CommandInterface, ReferenceInterface>,
    "dimensional_interfaces: DIMENSION MISMATCH on a reference edge -- the command a parent writes "
    "and the reference a child exports must have the same physical dimension");
}

template <typename ExportedState, typename ConsumedState>
constexpr void require_state_edge()
{
  static_assert(
    ExportedState::role == Role::state && ConsumedState::role == Role::state,
    "dimensional_interfaces: a state edge needs two state interfaces");
  static_assert(
    connectable_v<ExportedState, ConsumedState>,
    "dimensional_interfaces: DIMENSION MISMATCH on a state edge -- the state a child exports and "
    "the state a parent consumes must have the same physical dimension");
}
}  // namespace dimensions
}  // namespace hierarchical_control

#endif  // HIERARCHICAL_CONTROL_DIMENSIONAL_INTERFACES_HPP
