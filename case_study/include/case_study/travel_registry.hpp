// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef CASE_STUDY__TRAVEL_REGISTRY_HPP_
#define CASE_STUDY__TRAVEL_REGISTRY_HPP_

#include <cstdint>
#include <string>

namespace case_study
{

/// A child's derived state exposed to a parent in the same process.
/**
 * Humble's chainable controllers cannot export a state interface (that is Jazzy+), so a parent
 * that needs a child's derived state must read it out of band. This registry is that binding.
 * On Jazzy the same dependency would be expressed as export_state_interfaces() plus a claimed
 * state interface; the *scheduling* question is identical, which is what this case study tests.
 */
class TravelSource
{
public:
  virtual ~TravelSource() = default;
  /// Accumulated wheel travel in metres; an integral of the measured velocity, so a parent cannot
  /// reconstruct it from raw joint feedback alone.
  virtual double travel() const = 0;

  /// The control cycle in which this source last updated its travel.
  /**
   * Lag is measured by comparing cycle NUMBERS, not wall-clock timestamps. Timestamp-based nearest
   * neighbour alignment on the subscriber side mixes in DDS queueing, topic offsets and the
   * simulation clock, so it cannot distinguish a scheduling lag from transport jitter.
   */
  virtual std::uint64_t cycle() const = 0;
};

class TravelRegistry
{
public:
  static void register_source(const std::string & name, TravelSource * source);
  static void unregister_source(const std::string & name);
  static TravelSource * find(const std::string & name);
};

}  // namespace case_study

#endif  // CASE_STUDY__TRAVEL_REGISTRY_HPP_
