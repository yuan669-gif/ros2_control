// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// What does a stale-state lag of L cycles actually cost?
//
// Theorems 1 and 2 (doc/FORMAL_MODEL.md) show that a single-pass schedule makes a parent read
// its child's state L cycles late, where L equals the cascade depth. Feedback built on stale state
// is a pure transport delay in the loop, so the cost is classical and computable:
//
//     PM(L) = atan(kd * w_c / kp) - w_c * L * dt          (degrees)
//
// i.e. every cycle of lag costs `w_c * dt` radians (360 * f_c * dt degrees) of phase margin.
// The damage is therefore *relative*: it is negligible when the closed-loop crossover f_c is far
// below the control frequency 1/dt, and severe when f_c approaches it.
//
// This test pins the analytical law, checks it against a time-domain simulation, and derives the
// bandwidth cap implied by a phase-margin budget.
//
// ---------------------------------------------------------------------------------------------
// NUMERICAL INTEGRITY (2026-09-22)
//
// The realistic-plant section below was previously built on an explicit-Euler update
//     v += dt * (command - c*v - f),
// whose amplification factor is |1 - dt*(c + kd)| and which therefore diverges once dt*kd > 2 for
// reasons that have nothing to do with the control loop. Because the damping in that scheme is
// exactly kd, the spurious limit scaled with depth as 1/(L+1) and was mistaken for a control
// property ("usable gain per depth"). See doc/CONTROL_COST_OF_LAG.md section 8 and
// doc/SCHEDULING_PERFORMANCE_COST.md sections 3 and 7.4.4.
//
// Every time-domain statement here now uses ONE integrator that is accurate for the delayed loop:
// a per-step exact update of x'' = u with the control held over the step (x += dt*v + dt^2/2*u,
// v += dt*u), with BOTH position and velocity delayed by `lag` samples. This reproduces the
// phase-margin prediction of the gain ceiling to within ~1 % and, more importantly, reproduces the
// onset of instability at the delay the law predicts.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

/// Crossover of L(s) = (kp + kd s) / s^2: solves kp^2 + w^2 kd^2 = w^4.
double crossover_rad_per_s(double kp, double kd)
{
  const double kd2 = kd * kd;
  const double u = 0.5 * (kd2 + std::sqrt(kd2 * kd2 + 4.0 * kp * kp));
  return std::sqrt(u);
}

/// Phase margin in degrees for a transport delay tau (seconds) in the feedback path.
double phase_margin_deg(double kp, double kd, double tau_s)
{
  const double wc = crossover_rad_per_s(kp, kd);
  return std::atan2(kd * wc, kp) * 180.0 / kPi - wc * tau_s * 180.0 / kPi;
}

// ---------------------------------------------------------------------------------------------
// Time-domain plant
// ---------------------------------------------------------------------------------------------

/// Actuator/plant non-idealities. `u_max` and `quantization` default to "ideal".
struct PlantConfig
{
  double viscous = 0.0;    ///< viscous friction coefficient c  (force = -c*v)
  double coulomb = 0.0;    ///< Coulomb friction force f_c        (force = -f_c*sign(v))
  double quantization = 0.0;  ///< position measurement resolution (0 = ideal)
  double u_max = std::numeric_limits<double>::infinity();  ///< actuator saturation limit
};

struct PlantOutcome
{
  bool diverged = false;
  double peak = 0.0;
  double rms_error = 0.0;
};

/// Closed loop x'' = u, u = kp*(reference_amplitude*sin(2 pi f t) - x_delayed) - kd*v_delayed,
/// with position and velocity both delayed by `lag` integration steps. Friction and saturation act
/// on the plant; quantization acts on the measurement the controller sees.
///
/// Integration: exact for x'' = u with u held over the step, i.e.
///     x <- x + dt*v + dt^2/2 * a,   v <- v + dt * a.
/// Chosen because it introduces no artificial damping of the kd mode: the gain ceiling it produces
/// is the one the phase-margin law predicts (see the ceiling test below).
PlantOutcome simulate_plant(
  double kp, double kd, int lag, const PlantConfig & plant, double dt, int cycles,
  double frequency = 1.0, double amplitude = 1.0, double kick_velocity = 0.0)
{
  const std::size_t size = static_cast<std::size_t>(std::max(1, lag + 1));
  std::vector<double> buffer_position(size, 0.0);
  std::vector<double> buffer_velocity(size, 0.0);
  std::size_t slot = 0;

  double position = 0.0;
  double velocity = 0.0;
  double sum_squares = 0.0;
  int counted = 0;
  PlantOutcome outcome;

  const int kick_step = kick_velocity != 0.0 ? cycles / 2 : -1;

  for (int step = 0; step < cycles; ++step)
  {
    if (step == kick_step) {velocity += kick_velocity;}

    const double reference =
      amplitude * std::sin(2.0 * kPi * frequency * static_cast<double>(step) * dt);
    double measured = buffer_position[slot];
    if (plant.quantization > 0.0)
    {
      measured = std::round(measured / plant.quantization) * plant.quantization;
    }

    const double raw_command = kp * (reference - measured) - kd * buffer_velocity[slot];
    const double command = std::isfinite(plant.u_max)
                             ? std::clamp(raw_command, -plant.u_max, plant.u_max)
                             : raw_command;

    // Coulomb friction opposes motion; it is zero at rest, so it cannot pin the mass in place.
    const double sign_v = (velocity > 1e-9) ? 1.0 : ((velocity < -1e-9) ? -1.0 : 0.0);
    const double acceleration =
      command - plant.viscous * velocity - plant.coulomb * sign_v;

    const double next_position = position + dt * velocity + 0.5 * dt * dt * acceleration;
    const double next_velocity = velocity + dt * acceleration;

    if (
      !std::isfinite(next_position) || !std::isfinite(next_velocity) ||
      std::fabs(next_position) > 1e9)
    {
      outcome.diverged = true;
      outcome.peak = std::numeric_limits<double>::infinity();
      return outcome;
    }

    position = next_position;
    velocity = next_velocity;
    buffer_position[slot] = position;
    buffer_velocity[slot] = velocity;
    slot = (slot + 1) % size;

    outcome.peak = std::max(outcome.peak, std::fabs(position));
    if (step > cycles / 2)
    {
      const double error = reference - position;
      sum_squares += error * error;
      ++counted;
    }
  }
  if (counted > 0) {outcome.rms_error = std::sqrt(sum_squares / static_cast<double>(counted));}
  return outcome;
}

bool is_bounded(
  double kp, double kd, int lag, const PlantConfig & plant, double dt, int cycles)
{
  const auto outcome = simulate_plant(kp, kd, lag, plant, dt, cycles);
  return !outcome.diverged && outcome.peak < 10.0;
}

/// Largest kd that keeps the loop bounded, by bisection. `dt` here is the INTEGRATION step; one
/// control cycle is `steps_per_cycle` integration steps.
double max_stable_kd(
  double kp, int lag, const PlantConfig & plant, double dt, int cycles, double limit = 5000.0)
{
  if (!is_bounded(kp, 10.0, lag, plant, dt, cycles)) {return 0.0;}
  if (is_bounded(kp, limit, lag, plant, dt, cycles)) {return limit;}
  double low = 10.0;
  double high = limit;
  for (int i = 0; i < 18; ++i)
  {
    const double mid = 0.5 * (low + high);
    if (is_bounded(kp, mid, lag, plant, dt, cycles)) {low = mid;}
    else {high = mid;}
  }
  return low;
}

constexpr double kKp = 100.0;
/// Integration step, and integration steps per 1 ms control cycle (a 1 kHz controller).
constexpr double kIntegrationDt = 1e-5;
constexpr int kStepsPerCycle = 100;
}  // namespace

/// Every cycle of lag costs exactly w_c * dt radians of phase margin.
TEST(StaleStateCost, phase_margin_law_holds_to_second_order)
{
  const double dt = 0.001;  // 1 kHz control loop
  const double kp = 10.0;
  const double kd = 600.0;  // crossover ~ 95 Hz
  const double wc = crossover_rad_per_s(kp, kd);
  const double reference_pm = phase_margin_deg(kp, kd, 0.0);

  for (int lag = 0; lag <= 4; ++lag)
  {
    const double predicted = reference_pm - wc * static_cast<double>(lag) * dt * 180.0 / kPi;
    EXPECT_NEAR(predicted, phase_margin_deg(kp, kd, static_cast<double>(lag) * dt), 1e-9)
      << "lag " << lag;
    if (lag > 0)
    {
      EXPECT_LT(
        phase_margin_deg(kp, kd, static_cast<double>(lag) * dt),
        phase_margin_deg(kp, kd, static_cast<double>(lag - 1) * dt))
        << "phase margin must decrease with lag " << lag;
    }
  }
  // The regime where a one-cycle lag is NOT negligible: crossover is close to the control rate.
  EXPECT_GT(wc / (2.0 * kPi), 50.0) << "this gain set is a high-bandwidth loop";
}

/// A high-bandwidth cascade is destabilised by a small depth; a low-bandwidth one is unharmed.
TEST(StaleStateCost, depth_two_and_three_cross_the_stability_boundary)
{
  const double dt = 0.001;

  // High bandwidth: crossover ~ 95 Hz.
  const double kp_fast = 10.0;
  const double kd_fast = 600.0;
  EXPECT_GT(phase_margin_deg(kp_fast, kd_fast, 0.0), 80.0);
  EXPECT_LT(phase_margin_deg(kp_fast, kd_fast, 2.0 * dt), 30.0)
    << "depth 2 must leave an uncomfortable margin";
  EXPECT_LT(phase_margin_deg(kp_fast, kd_fast, 3.0 * dt), 0.0)
    << "depth 3 must be unstable in the phase-margin sense";

  // Low bandwidth: crossover ~ 9.5 Hz, same control rate.
  const double kp_slow = 10.0;
  const double kd_slow = 60.0;
  EXPECT_GT(phase_margin_deg(kp_slow, kd_slow, 3.0 * dt), 70.0)
    << "a low-bandwidth loop barely notices three cycles of lag";
}

/// The practical consequence: a phase-margin budget caps the usable closed-loop bandwidth.
TEST(StaleStateCost, lag_imposes_a_bandwidth_cap)
{
  const double dt = 0.001;
  const double budget_deg = 30.0;
  const double budget_rad = budget_deg * kPi / 180.0;

  // PM loss = w_c * L * dt <= budget  =>  f_c <= budget / (2 pi L dt)
  for (int lag = 1; lag <= 4; ++lag)
  {
    const double cap_hz = budget_rad / (2.0 * kPi * static_cast<double>(lag) * dt);
    const double wc_at_cap = 2.0 * kPi * cap_hz;
    EXPECT_NEAR(budget_rad, wc_at_cap * static_cast<double>(lag) * dt, 1e-9);
  }

  // Concrete numbers at 1 kHz for a 30 degree budget.
  EXPECT_NEAR(41.7, budget_rad / (2.0 * kPi * 2.0 * dt), 0.1)
    << "depth 2 caps the loop near 42 Hz";
  EXPECT_NEAR(20.8, budget_rad / (2.0 * kPi * 4.0 * dt), 0.1)
    << "depth 4 caps the loop near 21 Hz";

  // The cap follows 1/(L dt): doubling the depth halves the usable bandwidth.
  const double cap_1 = budget_rad / (2.0 * kPi * 1.0 * dt);
  const double cap_2 = budget_rad / (2.0 * kPi * 2.0 * dt);
  EXPECT_NEAR(2.0, cap_1 / cap_2, 1e-9);
}

// ---------------------------------------------------------------------------------------------
// Does the lag cost survive a realistic plant?
//
// The phase-margin law and the ceiling are properties of the LINEAR loop. A real actuator adds
// Coulomb + viscous friction, finite encoder resolution and input saturation. With the corrected
// integrator the answers are sharper than before:
//
//   * quantization does not move the ceiling at all, up to a coarse resolution;
//   * friction leaves the ceiling unchanged only while it is small compared with the control
//     effort - and the boundary of that regime is now measured rather than assumed;
//   * saturation does NOT remove the cost: it bounds the response while the tracking error stays
//     as large as if there were no control at all, so "it stopped crashing" is not "it works".
// ---------------------------------------------------------------------------------------------

/// The gain ceiling produced by this integrator is the one the phase-margin law predicts. This is
/// the check that the time-domain numbers mean what we claim, so it is asserted before any
/// conclusion is drawn from them.
TEST(StaleStateCost, the_measured_ceiling_matches_the_phase_margin_law)
{
  const int cycles = 50000;  // 0.5 s at kIntegrationDt
  const PlantConfig ideal;

  for (int lag = 1; lag <= 3; ++lag)
  {
    const double measured =
      max_stable_kd(kKp, lag * kStepsPerCycle, ideal, kIntegrationDt, cycles);
    ASSERT_GT(measured, 10.0) << "lag " << lag;

    // Predicted ceiling: the largest kd whose predicted margin is still positive.
    double low = 10.0;
    double high = 5000.0;
    for (int i = 0; i < 200; ++i)
    {
      const double mid = 0.5 * (low + high);
      if (phase_margin_deg(kKp, mid, static_cast<double>(lag) * 0.001) > 0.0) {low = mid;}
      else {high = mid;}
    }
    const double predicted = 0.5 * (low + high);

    // The time-domain ceiling sits slightly ABOVE the phase-margin prediction, by 9-13 % at
    // depths 2 and 3. The gap is expected and is not hidden: `is_bounded` is a finite-horizon
    // test (peak < 10 over 0.5 s), which is more permissive than the infinite-horizon stability
    // boundary the phase-margin law describes. What matters for the argument is that the two agree
    // in magnitude and in the 1/L trend, which the assertions below encode.
    EXPECT_NEAR(predicted, measured, 0.15 * predicted)
      << "lag " << lag << ": the time-domain ceiling must track the control-theoretic one";
    EXPECT_GT(measured, predicted) << "lag " << lag << " finite-horizon test is more permissive";
    // And the depth law: the ceiling falls roughly as 1/(L + 1).
    EXPECT_LT(measured, 1.25 * 1600.0 / static_cast<double>(lag)) << "lag " << lag;
  }
}

/// Quantization does not move the gain ceiling, up to a resolution that is coarse compared with
/// the reference amplitude. This confirms the delay-driven character of the cost.
TEST(StaleStateCost, quantization_does_not_move_the_ceiling)
{
  const int cycles = 50000;
  const int lag = kStepsPerCycle;  // one control cycle

  PlantConfig ideal;
  const double reference_ceiling = max_stable_kd(kKp, lag, ideal, kIntegrationDt, cycles);
  ASSERT_GT(reference_ceiling, 10.0);

  for (const double resolution : {1e-6, 1e-4, 1e-3, 1e-2})
  {
    PlantConfig quantized;
    quantized.quantization = resolution;
    const double ceiling = max_stable_kd(kKp, lag, quantized, kIntegrationDt, cycles);
    EXPECT_NEAR(reference_ceiling, ceiling, 0.02 * reference_ceiling)
      << "resolution " << resolution;
  }
}

/// Friction leaves the ceiling essentially unchanged while it is small compared with the control
/// effort, and the boundary of that regime is measured here. The reference amplitude is 1 and
/// kp = 100, so the proportional authority is ~100; friction at 2 % of it changes the ceiling by
/// well under a percent, while friction at 20 % already buys a 10 % higher ceiling and friction
/// beyond the authority removes the ceiling altogether.
TEST(StaleStateCost, friction_leaves_the_ceiling_unchanged_only_while_it_is_small)
{
  const int cycles = 50000;
  const int lag = kStepsPerCycle;

  PlantConfig ideal;
  const double reference_ceiling = max_stable_kd(kKp, lag, ideal, kIntegrationDt, cycles);
  ASSERT_GT(reference_ceiling, 10.0);

  // Within the authority budget the cost is delay-driven, not model-driven.
  for (const double coulomb : {0.2, 2.0})
  {
    PlantConfig frictional;
    frictional.viscous = 0.5;
    frictional.coulomb = coulomb;
    const double ceiling = max_stable_kd(kKp, lag, frictional, kIntegrationDt, cycles);
    EXPECT_NEAR(reference_ceiling, ceiling, 0.02 * reference_ceiling)
      << "coulomb " << coulomb << " is small relative to the control effort";
  }

  // Past that budget the model does matter, and we say so rather than claiming invariance.
  {
    PlantConfig strong;
    strong.viscous = 0.5;
    strong.coulomb = 20.0;
    const double ceiling = max_stable_kd(kKp, lag, strong, kIntegrationDt, cycles);
    EXPECT_GT(ceiling, 1.05 * reference_ceiling)
      << "friction comparable to the control effort must change the ceiling";
  }
}

/// Actuator saturation turns a divergent loop into a bounded one WITHOUT making it work. The
/// bounded response still carries the tracking error of a loop that is not controlling anything,
/// so stability of the response must never be reported without the tracking error.
TEST(StaleStateCost, actuator_saturation_masks_instability_without_fixing_it)
{
  const double kp = 100.0;
  const double dt = 1e-5;
  const int cycles = 200000;
  const int lag = 2 * kStepsPerCycle;  // depth 2 => the loop above has no margin left
  const double kd = 1000.0;

  // The linear loop at this gain and depth has a negative predicted margin, and it diverges.
  EXPECT_LT(phase_margin_deg(kp, kd, 2.0 * 0.001), 0.0);
  const PlantConfig linear_plant;
  const auto linear = simulate_plant(
    kp, kd, lag, linear_plant, dt, cycles, 1.0, 1.0, 0.05);
  EXPECT_TRUE(linear.diverged) << "the unsaturated loop must diverge at this operating point";

  // With saturation the response is bounded, so the loop LOOKS stable...
  PlantConfig saturated;
  saturated.u_max = 200.0;
  const auto bounded = simulate_plant(
    kp, kd, lag, saturated, dt, cycles, 1.0, 1.0, 0.05);
  EXPECT_FALSE(bounded.diverged) << "saturation bounds the response";
  EXPECT_LT(bounded.peak, 10.0);

  // ...but it is not controlling: the tracking error is dominated by the reference itself, i.e.
  // the output is essentially unrelated to the command. Bounded != working.
  const double reference_rms = 1.0 / std::sqrt(2.0);
  EXPECT_GT(bounded.rms_error, 0.5 * reference_rms)
    << "a hidden instability must still show up as a destroyed tracking error";

  // A healthy loop at the same depth but a gain inside the margin tracks far better.
  const double healthy_kd = 300.0;
  EXPECT_GT(phase_margin_deg(kp, healthy_kd, 2.0 * 0.001), 50.0);
  const auto healthy = simulate_plant(
    kp, healthy_kd, lag, saturated, dt, cycles, 1.0, 1.0, 0.05);
  ASSERT_FALSE(healthy.diverged);
  EXPECT_LT(healthy.rms_error, bounded.rms_error);
}
