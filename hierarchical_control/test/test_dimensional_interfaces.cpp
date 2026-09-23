// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Dimensional analysis of controller interfaces, at compile time.
//
// ros2_control names interfaces by STRING. A parent can declare that it consumes
// "<child>/state" and nothing checks that the child exports anything of the kind, or of the right
// physical quantity. This project measured exactly that: test_upstream_ordering.cpp declares a
// state edge on a name nothing exports and configure_controller accepts it silently
// (doc/BIDIRECTIONAL_EDGE_ANALYSIS.md section 9). A dimension-wrong wiring is the same class of
// defect and would be just as invisible.
//
// This test asserts two things:
//   1. the dimensional ALGEBRA reduces correctly and, crucially, keeps quantities that a naive
//      "kind enum" would conflate distinct (torque vs energy, angular vs linear velocity);
//   2. the CONNECTION RULES reject dimension mismatches, and the rejection predicate is exact
//      (no false positives on legal wirings).
//
// The negative direction still needs to be shown as a hard compile error, which cannot live in
// this file; see test/static_topology_negative/compile_fail_dimension_*.cpp and
// test_static_topology_negative.py.

#include <gtest/gtest.h>

#include "hierarchical_control/dimensional_interfaces.hpp"

namespace d = hierarchical_control::dimensions;

namespace
{
using PositionCommand = d::Interface<d::Role::command, d::Position>;
using PositionReference = d::Interface<d::Role::reference, d::Position>;
using VelocityReference = d::Interface<d::Role::reference, d::LinearVelocity>;
using TorqueCommand = d::Interface<d::Role::command, d::Torque>;
using ForceCommand = d::Interface<d::Role::command, d::Force>;
using PositionState = d::Interface<d::Role::state, d::Position>;
using AngleState = d::Interface<d::Role::state, d::Angle>;
using VelocityState = d::Interface<d::Role::state, d::LinearVelocity>;
using AngularVelocityState = d::Interface<d::Role::state, d::AngularVelocity>;
}  // namespace

/// The algebra reduces: derivatives and products land on the named dimensions.
TEST(DimensionalInterfaces, algebra_reduces_to_the_named_dimensions)
{
  static_assert(d::same_dimension_v<d::divide_t<d::Position, d::base::time>, d::LinearVelocity>);
  static_assert(
    d::same_dimension_v<d::divide_t<d::LinearVelocity, d::base::time>, d::LinearAcceleration>);
  static_assert(
    d::same_dimension_v<d::divide_t<d::LinearAcceleration, d::base::time>, d::LinearJerk>);
  static_assert(d::same_dimension_v<d::divide_t<d::Angle, d::base::time>, d::AngularVelocity>);
  static_assert(
    d::same_dimension_v<d::divide_t<d::AngularVelocity, d::base::time>, d::AngularAcceleration>);
  static_assert(d::same_dimension_v<d::multiply_t<d::base::mass, d::LinearAcceleration>, d::Force>);
  static_assert(d::same_dimension_v<d::divide_t<d::Energy, d::base::time>, d::Power>);

  // Integrating a velocity recovers a position -- the algebra is not a one-way tag.
  static_assert(
    d::same_dimension_v<d::multiply_t<d::LinearVelocity, d::base::time>, d::Position>);

  SUCCEED();
}

/// Quantities that a flat "kind" enum would conflate must remain distinct. These are the cases
/// where a wrong wiring is physically plausible and therefore dangerous.
TEST(DimensionalInterfaces, distinct_quantities_stay_distinct)
{
  // Torque is energy per radian: WITHOUT the angle exponent these would be the same dimension and
  // a torque command could be wired to an energy reference.
  static_assert(!d::same_dimension_v<d::Torque, d::Energy>);
  static_assert(
    d::same_dimension_v<
      d::multiply_t<d::Torque, d::base::angle>, d::Energy>);  // and they are related

  // Angular vs linear: the classic wheel-radius confusion.
  static_assert(!d::same_dimension_v<d::Angle, d::Position>);
  static_assert(!d::same_dimension_v<d::AngularVelocity, d::LinearVelocity>);
  static_assert(!d::same_dimension_v<d::AngularAcceleration, d::LinearAcceleration>);

  // Position vs its derivatives.
  static_assert(!d::same_dimension_v<d::Position, d::LinearVelocity>);
  static_assert(!d::same_dimension_v<d::LinearVelocity, d::LinearAcceleration>);

  // Effort quantities.
  static_assert(!d::same_dimension_v<d::Force, d::Power>);
  static_assert(!d::same_dimension_v<d::Force, d::Torque>);

  SUCCEED();
}

/// Reference edges accept matching dimensions and reject mismatches, with no false positives.
TEST(DimensionalInterfaces, reference_edges_require_matching_dimensions)
{
  static_assert(d::reference_edge_is_legal_v<PositionCommand, PositionReference>);
  static_assert(!d::reference_edge_is_legal_v<PositionCommand, VelocityReference>);
  static_assert(!d::reference_edge_is_legal_v<TorqueCommand, PositionReference>);
  static_assert(!d::reference_edge_is_legal_v<ForceCommand, PositionReference>);

  // Role matters too: a state interface is not a reference interface.
  static_assert(!d::reference_edge_is_legal_v<PositionCommand, PositionState>);

  SUCCEED();
}

/// State edges require matching dimensions.
TEST(DimensionalInterfaces, state_edges_require_matching_dimensions)
{
  static_assert(d::state_edge_is_legal_v<PositionState, PositionState>);
  static_assert(!d::state_edge_is_legal_v<PositionState, VelocityState>);

  // The wheel case: an angle state must not be consumed as a linear position state.
  static_assert(!d::state_edge_is_legal_v<AngleState, PositionState>);
  static_assert(!d::state_edge_is_legal_v<AngularVelocityState, VelocityState>);

  // Role matters: a command interface is not a state interface.
  static_assert(!d::state_edge_is_legal_v<PositionCommand, PositionState>);

  SUCCEED();
}

/// The check is usable as a hard requirement at a point of use, and produces a message that names
/// the failure. This test only exercises the ACCEPTING path; the rejecting path is a hard compile
/// error and lives in the negative corpus.
TEST(DimensionalInterfaces, require_helpers_accept_legal_edges)
{
  d::require_reference_edge<PositionCommand, PositionReference>();
  d::require_reference_edge<TorqueCommand, d::Interface<d::Role::reference, d::Torque>>();
  d::require_state_edge<PositionState, PositionState>();
  SUCCEED();
}

/// A worked example of the defect class this prevents, mirroring the case study's POV chassis.
///
/// The chassis writes a velocity target to each wheel (reference edge, [m/s]) and consumes each
/// wheel's accumulated travel (state edge, [m]). If the travel state were declared with the wrong
/// dimension -- say the wheel exported an ANGLE and the chassis consumed it as a LENGTH -- the
/// string-name world would accept the wiring and the chassis would dead-reckon in the wrong units.
/// Here the mismatch is rejected at compile time.
TEST(DimensionalInterfaces, the_pov_chassis_wiring_is_checked)
{
  using WheelReferenceVelocity = d::Interface<d::Role::reference, d::LinearVelocity>;
  using WheelTravelState = d::Interface<d::Role::state, d::Position>;
  using WheelAngleState = d::Interface<d::Role::state, d::Angle>;

  using ChassisVelocityCommand = d::Interface<d::Role::command, d::LinearVelocity>;
  using ChassisTravelInput = d::Interface<d::Role::state, d::Position>;

  // The correct wiring is accepted.
  static_assert(d::reference_edge_is_legal_v<ChassisVelocityCommand, WheelReferenceVelocity>);
  static_assert(d::state_edge_is_legal_v<WheelTravelState, ChassisTravelInput>);

  // The plausible mistake -- consuming the wheel's ANGLE as a LENGTH -- is rejected.
  static_assert(!d::state_edge_is_legal_v<WheelAngleState, ChassisTravelInput>);

  SUCCEED();
}
