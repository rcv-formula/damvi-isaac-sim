#ifndef LAUNCH_RAMP_HPP
#define LAUNCH_RAMP_HPP

#include <algorithm>

namespace launch_ramp {

struct StepResult {
  double speed;
  bool active;
  bool command_valid;
  bool target_not_above_ramp;
};

inline bool target_is_above(double target_speed, double current_speed) {
  return target_speed > current_speed;
}

// Launch start is an acceleration-only feature. A trigger must never create a
// deceleration ramp when the measured wheel speed is already at or above the
// target.
inline bool should_engage(double target_speed, double wheel_speed,
                          double engage_diff) {
  const double gap = target_speed - wheel_speed;
  return target_is_above(target_speed, wheel_speed) &&
         gap >= std::max(0.0, engage_diff);
}

// Bound a delayed callback to one nominal output period. The next callback may
// arrive immediately from another source (for example odom after a wall timer),
// so catching up an entire stall in one published command creates a real step.
inline double bounded_max_step(double acceleration, double elapsed,
                               double nominal_period) {
  const double bounded_elapsed =
      std::clamp(elapsed, 0.0, std::max(0.0, nominal_period));
  return std::max(0.0, acceleration) * bounded_elapsed;
}

// Advance only toward a higher target. If the target falls to/below the
// current ramp, return no launch command so the caller can hand control back
// to its normal deceleration path immediately.
inline StepResult advance(double ramp_speed, double target_speed,
                          double max_step, double release_diff) {
  if (target_speed <= ramp_speed) {
    return {ramp_speed, false, false, true};
  }

  const double next_speed =
      ramp_speed +
      std::min(target_speed - ramp_speed, std::max(0.0, max_step));
  // The configured release window changes the state to a handoff limiter. The
  // caller must keep that limiter in control until the target is reached.
  const double handoff_tolerance = std::max(0.0, release_diff);
  const bool active = target_speed - next_speed > handoff_tolerance;
  return {next_speed, active, true, false};
}

// After the configured release window is reached, preserve the normal blend
// candidate but keep limiting its positive slope until the target is reached.
// This closes the gap between an early release and a closely-following output
// callback. Deceleration is deliberately returned to the caller immediately.
inline StepResult advance_handoff(double current_speed, double target_speed,
                                  double normal_candidate, double max_step) {
  if (target_speed <= current_speed) {
    return {current_speed, false, false, true};
  }

  const double positive_step = std::max(0.0, max_step);
  const double remaining = target_speed - current_speed;
  if (remaining <= positive_step) {
    return {target_speed, false, true, false};
  }

  // A zero normal-blend gain must not leave the safety handoff latched forever.
  // Launch already owns positive acceleration, so fall back to its bounded step
  // when the normal candidate makes no forward progress.
  const double progressing_candidate =
      normal_candidate > current_speed
          ? normal_candidate
          : current_speed + positive_step;
  const double bounded_candidate =
      std::clamp(progressing_candidate, current_speed, target_speed);
  const double next_speed =
      std::min(bounded_candidate, current_speed + positive_step);
  return {next_speed, true, true, false};
}

} // namespace launch_ramp

#endif // LAUNCH_RAMP_HPP
