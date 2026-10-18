#pragma once

// Hardware-independent arm control and readiness waits. The robot supplies
// sensor, motor and clock callbacks; host tests use a clock that does not sleep.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace skills_control {

enum class ArmOutcome { running, reached, cancelled, timed_out, sensor_error };
enum class Readiness { waiting, ready, failed };
enum class WaitOutcome { ready, failed, timed_out };

// The existing moveArm motor command, including its direction, gain, soft
// approach and power floors. Keep this independent of a movement's deadline.
inline int arm_motor_power(double error_deg) {
    int power = static_cast<int>(std::clamp(error_deg * 0.55, -110.0, 110.0));
    if (std::abs(error_deg) < 60.0) power = std::clamp(power, -35, 35);
    if (std::abs(error_deg) < 25.0) power = std::clamp(power, -20, 20);
    if (std::abs(error_deg) > 25.0) {
        if (power > 0 && power < 25) power = 25;
        else if (power < 0 && power > -25) power = -25;
    }
    return -power;
}

// IO: now(), delay(ms), position_deg(), cancelled(), command(power), brake().
// Map an invalid raw sensor reading to NaN in position_deg(). Include a
// competition-mode change in cancelled(). A zero timeout preserves the
// driver's unlimited move; skills supplies a finite limit. The worker itself
// enforces that limit, even when no other task asks whether it is done.
template <typename IO>
ArmOutcome run_arm(double target_deg, std::uint32_t timeout_ms, IO& io) {
    const std::uint32_t began = io.now();
    const auto stop = [&](ArmOutcome outcome) {
        io.brake();
        return outcome;
    };
    if (!std::isfinite(target_deg)) return stop(ArmOutcome::sensor_error);
    while (true) {
        if (io.cancelled()) return stop(ArmOutcome::cancelled);
        const double position = io.position_deg();
        const double error = target_deg - position;
        if (!std::isfinite(position) || !std::isfinite(error))
            return stop(ArmOutcome::sensor_error);
        if (std::abs(error) <= 3.0) return stop(ArmOutcome::reached);
        const std::uint32_t elapsed = io.now() - began;
        if (timeout_ms != 0 && elapsed >= timeout_ms)
            return stop(ArmOutcome::timed_out);
        io.command(arm_motor_power(error));
        io.delay(timeout_ms == 0
                     ? std::uint32_t{10}
                     : std::min<std::uint32_t>(10, timeout_ms - elapsed));
    }
}

// Poll returns Readiness. Reuse `began` for every check of the same score:
// the second check must not receive a fresh waiting budget. Readiness is
// checked first, so an already-ready mechanism never waits, including a
// recheck after driving. A zero limit permits that check but no waiting.
// `now` and `delay` are callbacks. Unsigned subtraction also handles a clock
// rollover; these short waits must not span an entire uint32_t clock period.
template <typename Poll, typename Clock, typename Delay>
WaitOutcome wait_until(std::uint32_t began, std::uint32_t limit_ms,
                       Poll poll, Clock now, Delay delay) {
    while (true) {
        const Readiness state = poll();
        if (state == Readiness::ready) return WaitOutcome::ready;
        if (state == Readiness::failed) return WaitOutcome::failed;
        const std::uint32_t elapsed = now() - began;
        if (elapsed >= limit_ms) return WaitOutcome::timed_out;
        delay(std::min<std::uint32_t>(10, limit_ms - elapsed));
    }
}

// Position checks only: callers separately reject worker failure and retain
// the lift's ownership join before starting a second lift movement. Job
// completion or timeout must never stand in for any of these measurements.
// Preserve the loader's existing one-sided limits and its own arm tolerance.
inline bool loader_position_ready(double lift, double arm, double lift_down,
                                  double arm_rest, double arm_tolerance) {
    return std::isfinite(lift) && std::isfinite(arm) &&
           std::isfinite(lift_down) && std::isfinite(arm_rest) &&
           std::isfinite(arm_tolerance) && arm_tolerance >= 0.0 &&
           lift >= lift_down && arm <= arm_rest + arm_tolerance;
}

inline bool score_position_ready(double lift, double arm, double lift_target,
                                 double arm_target, double lift_tolerance,
                                 double arm_tolerance, double lock_from) {
    return std::isfinite(lift) && std::isfinite(arm) &&
           std::isfinite(lift_target) && std::isfinite(arm_target) &&
           std::isfinite(lift_tolerance) && lift_tolerance >= 0.0 &&
           std::isfinite(arm_tolerance) && arm_tolerance >= 0.0 &&
           std::isfinite(lock_from) &&
           (lift <= lift_target + lift_tolerance || lift <= lock_from) &&
           std::abs(arm - arm_target) <= arm_tolerance;
}

// The lift may finish its -150 target just short of 60 degrees of travel
// when it started at -83. Accept its measured position inside the existing
// 8-degree target band only after success, never merely after job completion.
// Callers still handle an explicit lift failure before using this check.
inline bool pickup_clear(double lift, double start, bool raise_succeeded,
                         double target = -150.0, double clearance = 60.0,
                         double tolerance = 8.0) {
    return std::isfinite(lift) && std::isfinite(start) &&
           std::isfinite(target) && std::isfinite(clearance) &&
           clearance >= 0.0 && std::isfinite(tolerance) && tolerance >= 0.0 &&
           (lift <= start - clearance ||
            (raise_succeeded && std::abs(lift - target) <= tolerance));
}

}  // namespace skills_control
