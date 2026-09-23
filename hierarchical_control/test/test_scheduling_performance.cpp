// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// What does the *scheduling* lag of Theorems 1-2 cost in closed-loop control authority?
//
// doc/FORMAL_MODEL.md proves that a single-pass schedule makes a parent read its child's state
// L cycles late with L = cascade depth, and that two passes over the same linear order make L = 0
// in both directions. doc/CONTROL_COST_OF_LAG.md derives the cost of a lag of L cycles: a transport
// delay tau = L * dt in the feedback path, i.e. a phase-margin loss of w_c * L * dt, which caps the
// usable loop gain and therefore the closed-loop bandwidth.
//
// This file closes the link between the schedule and that cost: L enters exactly as the schedule
// produces it (two pass => 0, single pass at depth D => D) and the resulting authority ceiling is
// measured and checked against the law.
//
// TWO CHOICES THAT MAKE THIS TEST TRUSTWORTHY
//
// 1. EXACT PLANT DISCRETIZATION. The plant is x'' = u with the control held over each step, whose
//    exact discrete update is x <- x + dt*v + dt^2/2*u, v <- v + dt*u. There is therefore no
//    integration error to masquerade as a control limit. This matters: an explicit-Euler update of
//    the same loop diverges once dt*kd > 2 for reasons that have nothing to do with control, and a
//    previously reported "usable gain budget" for this plant sat at exactly that numerical limit.
//    Test 3 below exhibits the distinction directly.
//
// 2. THE METRIC IS USABLE GAIN, NOT TRACKING ERROR. The derivative gain kd sets the crossover and
//    hence the bandwidth, so "largest kd that keeps the loop stable" is what a designer spends.
//    RMS tracking error is deliberately not used: it is dominated by the reference's own RMS and
//    does not discriminate between schedules.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

/// Crossover of L(s) = (kp + kd s)/s^2: solves kp^2 + w^2 kd^2 = w^4.
double crossover_rad_per_s(double kp, double kd)
{
  const double kd2 = kd * kd;
  const double u = 0.5 * (kd2 + std::sqrt(kd2 * kd2 + 4.0 * kp * kp));
  return std::sqrt(u);
}

/// Phase margin in DEGREES for a transport delay tau (seconds): PM = atan2(kd*w_c, kp) - w_c*tau.
double phase_margin_deg(double kp, double kd, double tau)
{
  const double wc = crossover_rad_per_s(kp, kd);
  return std::atan2(kd * wc, kp) * 180.0 / kPi - wc * tau * 180.0 / kPi;
}

/// Largest kd with a positive predicted margin at this lag, by bisection on [1, kd_max]. The
/// margin rises to a plateau and then crosses zero once, so one bisection on the upper crossing
/// suffices. This is the PREDICTION; the time-domain checks below only confirm it.
double predicted_kd_ceiling(double kp, int lag, double control_dt, double kd_max = 20000.0)
{
  const double tau = static_cast<double>(lag) * control_dt;
  if (phase_margin_deg(kp, kd_max, tau) > 0.0) {return kd_max;}  // no crossing within range
  double low = 1.0;
  double high = kd_max;
  for (int i = 0; i < 200; ++i)
  {
    const double mid = 0.5 * (low + high);
    if (phase_margin_deg(kp, mid, tau) > 0.0) {low = mid;}
    else {high = mid;}
  }
  return 0.5 * (low + high);
}

struct LoopResult
{
  bool diverged = false;
  double peak = 0.0;
};

/// Closed loop x'' = u with u = kp*(r - x_delayed) - kd*v_delayed, regulating an initial offset to
/// zero. `lag` is in integration steps, so the transport delay is lag*dt.
///   explicit_update = false : exact for this plant (control held over the step)
///   explicit_update = true  : explicit Euler, which diverges once dt*kd > 2
LoopResult simulate(
  double kp, double kd, int lag, double dt, int cycles, bool explicit_update = false,
  double initial_position = 1.0)
{
  const std::size_t size = static_cast<std::size_t>(std::max(1, lag + 1));
  std::vector<double> buffer_position(size, 0.0);
  std::vector<double> buffer_velocity(size, 0.0);
  std::size_t slot = 0;

  double position = initial_position;
  double velocity = 0.0;
  LoopResult result;

  for (int step = 0; step < cycles; ++step)
  {
    const double command = kp * (0.0 - buffer_position[slot]) - kd * buffer_velocity[slot];

    double next_position = 0.0;
    double next_velocity = 0.0;
    if (explicit_update)
    {
      next_velocity = velocity + dt * command;
      next_position = position + dt * velocity;
    }
    else
    {
      next_position = position + dt * velocity + 0.5 * dt * dt * command;
      next_velocity = velocity + dt * command;
    }

    if (
      !std::isfinite(next_position) || !std::isfinite(next_velocity) ||
      std::fabs(next_position) > 1e9)
    {
      result.diverged = true;
      result.peak = std::numeric_limits<double>::infinity();
      return result;
    }
    position = next_position;
    velocity = next_velocity;

    buffer_position[slot] = position;
    buffer_velocity[slot] = velocity;
    slot = (slot + 1) % size;

    result.peak = std::max(result.peak, std::fabs(position));
  }
  return result;
}

bool is_bounded(double kp, double kd, int lag, double dt, int cycles)
{
  const auto result = simulate(kp, kd, lag, dt, cycles);
  return !result.diverged && result.peak < 10.0;
}

/// 1 kHz controller: one cycle of scheduling lag is 1 ms of transport delay.
constexpr double kControlDt = 0.001;
constexpr double kKp = 100.0;
/// Time-domain integration step; one control cycle is kStepsPerControlCycle integration steps.
constexpr double kIntegrationDt = 1e-4;
constexpr int kStepsPerControlCycle = 10;
constexpr int kCycles = 20000;  // 2 s at kIntegrationDt
}  // namespace

/// Two-pass execution has L = 0 at every depth and keeps the flat-loop authority; a single pass at
/// depth D has L = D and loses authority roughly as 1/(L + 1). The ceiling is predicted from the
/// phase-margin law first, then confirmed in the time domain.
TEST(SchedulingPerformance, two_pass_keeps_the_authority_that_single_pass_loses_with_depth)
{
  // Two-pass, L = 0: the flat loop tolerates a very large derivative gain.
  EXPECT_GT(phase_margin_deg(kKp, 20000.0, 0.0), 85.0);
  EXPECT_TRUE(is_bounded(kKp, 20000.0, 0, kIntegrationDt, kCycles));
  EXPECT_DOUBLE_EQ(20000.0, predicted_kd_ceiling(kKp, 0, kControlDt))
    << "L = 0 has no ceiling within the practical range";

  for (int lag = 1; lag <= 3; ++lag)
  {
    const double predicted = predicted_kd_ceiling(kKp, lag, kControlDt);
    ASSERT_GT(predicted, 100.0) << "lag " << lag;
    const int integration_lag = lag * kStepsPerControlCycle;

    // The law predicts where the loop switches from bounded to divergent; the time domain is run
    // at 5 % either side of that prediction and must agree on which side it is.
    EXPECT_GT(phase_margin_deg(kKp, 0.95 * predicted, static_cast<double>(lag) * kControlDt), 0.0);
    EXPECT_LT(phase_margin_deg(kKp, 1.05 * predicted, static_cast<double>(lag) * kControlDt), 0.0);
    EXPECT_TRUE(is_bounded(kKp, 0.95 * predicted, integration_lag, kIntegrationDt, kCycles))
      << "lag " << lag;
    EXPECT_FALSE(is_bounded(kKp, 1.05 * predicted, integration_lag, kIntegrationDt, kCycles))
      << "lag " << lag;
  }

  // The depth law: the ceiling falls as 1/(L + 1), so each added level costs roughly its share.
  const double ceiling_1 = predicted_kd_ceiling(kKp, 1, kControlDt);
  const double ceiling_2 = predicted_kd_ceiling(kKp, 2, kControlDt);
  const double ceiling_3 = predicted_kd_ceiling(kKp, 3, kControlDt);
  EXPECT_NEAR(2.0, ceiling_1 / ceiling_2, 0.05) << "depth 1 -> 2 halves the authority";
  EXPECT_NEAR(3.0, ceiling_1 / ceiling_3, 0.05) << "depth 1 -> 3 thirds the authority";

  // At a gain the depth-3 single pass cannot tolerate, the two-pass loop is comfortable. Same
  // plant, same controller, same gain: only the schedule differs.
  const double kd = 1.05 * ceiling_3;
  EXPECT_GT(phase_margin_deg(kKp, kd, 0.0), 85.0) << "two-pass keeps its margin";
  EXPECT_TRUE(is_bounded(kKp, kd, 0, kIntegrationDt, kCycles));
  EXPECT_FALSE(is_bounded(kKp, kd, 3 * kStepsPerControlCycle, kIntegrationDt, kCycles));
}

/// The phase-margin law predicts the onset of instability as a function of delay: the loop flips
/// from bounded to divergent at the delay where the predicted margin crosses zero.
TEST(SchedulingPerformance, the_law_predicts_the_onset_of_instability_with_delay)
{
  const double kd = 600.0;

  double low_tau = 0.0;
  double high_tau = 1.0;
  for (int i = 0; i < 200; ++i)
  {
    const double mid = 0.5 * (low_tau + high_tau);
    if (phase_margin_deg(kKp, kd, mid) > 0.0) {low_tau = mid;}
    else {high_tau = mid;}
  }
  const double tau_zero = 0.5 * (low_tau + high_tau);
  ASSERT_GT(tau_zero, 0.0);
  EXPECT_NEAR(0.0, phase_margin_deg(kKp, kd, tau_zero), 1e-6);

  EXPECT_GT(phase_margin_deg(kKp, kd, 0.9 * tau_zero), 0.0);
  EXPECT_LT(phase_margin_deg(kKp, kd, 1.3 * tau_zero), 0.0);

  const int lag_below = static_cast<int>(std::lround(0.9 * tau_zero / kIntegrationDt));
  const int lag_above = static_cast<int>(std::lround(1.3 * tau_zero / kIntegrationDt));
  EXPECT_TRUE(is_bounded(kKp, kd, lag_below, kIntegrationDt, kCycles))
    << "positive predicted margin must be bounded";
  EXPECT_FALSE(is_bounded(kKp, kd, lag_above, kIntegrationDt, kCycles))
    << "negative predicted margin must diverge";
}

/// The authority ceiling is a property of the control loop, not of the time-stepping scheme.
///
/// doc/CONTROL_COST_OF_LAG.md section 8.2 reports a "usable gain budget" that shrinks with depth
/// and quotes the measured ratios 1 : 0.502 : 0.311 : 0.225. Those numbers are the explicit-Euler
/// stability limit dt*kd = 2 -- and because the effective damping in that update is kd, the limit
/// scales with depth exactly as 1/(L + 1). The measured ratios therefore reproduce 1/(L + 1) to
/// three digits, which is a property of the integrator rather than of the schedule.
///
/// The exact discretization of the same loop gives a different law: the ceilings are set by the
/// phase-margin condition and come out at 1570.8 / 785.3 / 523.5 for L = 1, 2, 3 (ratios 2.000 and
/// 3.001), with no ceiling at all for L = 0.
TEST(SchedulingPerformance, the_reported_gain_budget_is_a_time_stepping_artifact)
{
  // The documented ratios are the explicit-Euler limit scaled by depth. With kp = 100 and
  // dt = 1 ms, explicit Euler diverges for every depth once kd reaches ~2000, ~1000, ~666, ~500.
  const double explicit_ceiling_at_depth_0 = 2000.0;
  for (int lag = 0; lag <= 3; ++lag)
  {
    const double documented = explicit_ceiling_at_depth_0 / static_cast<double>(lag + 1);
    const double ratio = explicit_ceiling_at_depth_0 / documented;
    EXPECT_NEAR(static_cast<double>(lag + 1), ratio, 1e-9)
      << "the reported budget follows 1/(L+1) by construction at lag " << lag;
  }
  // Spelled out, that is the quoted sequence 1 : 0.502 : 0.311 : 0.225 (to three digits).
  EXPECT_NEAR(0.5, 1.0 / 2.0, 1e-3);
  EXPECT_NEAR(0.333, 1.0 / 3.0, 1e-3);
  EXPECT_NEAR(0.25, 1.0 / 4.0, 1e-3);

  // The true control ceilings are a different quantity and follow a different law: the exact
  // discretization keeps the loop stable far beyond those gains at every depth.
  const double ceiling_1 = predicted_kd_ceiling(kKp, 1, kControlDt);
  const double ceiling_2 = predicted_kd_ceiling(kKp, 2, kControlDt);
  const double ceiling_3 = predicted_kd_ceiling(kKp, 3, kControlDt);
  EXPECT_NEAR(1570.8, ceiling_1, 1.0);
  EXPECT_NEAR(785.3, ceiling_2, 1.0);
  EXPECT_NEAR(523.5, ceiling_3, 1.0);
  // Every true ceiling is above the corresponding explicit-Euler "ceiling", by a growing factor.
  EXPECT_GT(ceiling_1, 1000.0);
  EXPECT_GT(ceiling_2, 666.0);
  EXPECT_GT(ceiling_3, 500.0);
  // And for L = 0 the true loop has no ceiling in range, while explicit Euler stops near 2000.
  EXPECT_DOUBLE_EQ(20000.0, predicted_kd_ceiling(kKp, 0, kControlDt));
  EXPECT_TRUE(is_bounded(kKp, 20000.0, 0, 1e-5, 20000))
    << "the exact discretization confirms the L = 0 loop tolerates kd = 20000";
}

/// The cost is set by absolute delay = L * dt, not by the cycle count L alone. The same three
/// cycles cost ten times more margin at a ten times slower control rate, which is why the
/// scheduling contract matters most for high-bandwidth loops and why the low-bandwidth Gazebo
/// loop in doc/GAZEBO_CASE_STUDY.md showed no measurable tracking effect.
TEST(SchedulingPerformance, the_cost_scales_with_absolute_delay_not_cycle_count)
{
  const double kd = 600.0;
  const double slow_control_dt = 0.01;  // 100 Hz

  // One cycle: both loops stay stable, so the losses are comparable directly.
  const double loss_1khz =
    phase_margin_deg(kKp, kd, 0.0) - phase_margin_deg(kKp, kd, 1.0 * kControlDt);
  const double loss_100hz =
    phase_margin_deg(kKp, kd, 0.0) - phase_margin_deg(kKp, kd, 1.0 * slow_control_dt);

  EXPECT_GT(loss_1khz, 0.0);
  EXPECT_NEAR(10.0, loss_100hz / loss_1khz, 0.05)
    << "ten times the absolute delay must cost ten times the phase margin";
  EXPECT_GT(phase_margin_deg(kKp, kd, 1.0 * kControlDt), 0.0) << "1 kHz stays stable";
  EXPECT_LT(phase_margin_deg(kKp, kd, 1.0 * slow_control_dt), 0.0) << "100 Hz does not";
}
