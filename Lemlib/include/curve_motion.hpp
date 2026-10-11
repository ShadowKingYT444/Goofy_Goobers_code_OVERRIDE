#pragma once

// A curved drive for routes: arrive at (x, y) facing `heading`, e.g. square
// on to a wall pin. It is built from two LemLib motions:
//   1. moveToPose (LemLib's "boomerang" curve) to a staging point that is
//      straight_in short of the target, on the line of the final heading;
//   2. moveToPoint straight along that line onto the target.
// The curve does the turning with room to spare; the straight finish is what
// arrives, so the robot ends lined up instead of still swinging round. The
// hand-over between the two is rolling (no stop).
//
// The numbers in `tuned` come from the curve tuner (src/curve_autotune.cpp),
// which drives this exact function, so what it measures is what a route gets.

#include "lemlib/api.hpp"
#include <cmath>

extern lemlib::Chassis chassis;

namespace curve {

struct Settings {
    float lead = 0.45f;        // how wide the curve swings (0-1)
    float drift = 4.0f;        // LemLib horizontalDrift: speed held in the curve
    float straight_in = 0.0f;  // length of the straight finish; 0 = curve only
    float max_speed = 100.0f;  // LemLib 0-127 scale
};

// What routes use unless they pass their own: the curve tuner's result on
// this robot (lead 0.45, drift 4, no straight finish).
inline Settings tuned{};

// Speed kept through the hand-over from curve to straight finish, and how
// close to the staging point the curve hands over.
inline constexpr float kHandoverSpeed = 40.0f;
inline constexpr float kHandoverRangeIn = 2.0f;

// Blocking. `forwards = false` drives it claw-first: the robot still ends
// facing `heading_deg`, having backed onto the target.
inline void to(float x, float y, float heading_deg, bool forwards,
               int timeout_ms, const Settings& s = tuned) {
    if (s.straight_in <= 0.0f) {
        chassis.moveToPose(x, y, heading_deg, timeout_ms,
                           {.forwards = forwards, .horizontalDrift = s.drift,
                            .lead = s.lead, .maxSpeed = s.max_speed},
                           false);
        return;
    }
    // Direction the robot is travelling as it arrives.
    const float heading = heading_deg * static_cast<float>(M_PI) / 180.0f;
    const float sign = forwards ? 1.0f : -1.0f;
    const float travel_x = sign * std::sin(heading);
    const float travel_y = sign * std::cos(heading);
    const std::uint32_t began = pros::millis();
    chassis.moveToPose(x - travel_x * s.straight_in, y - travel_y * s.straight_in,
                       heading_deg, timeout_ms,
                       {.forwards = forwards, .horizontalDrift = s.drift,
                        .lead = s.lead, .maxSpeed = s.max_speed,
                        .minSpeed = kHandoverSpeed,
                        .earlyExitRange = kHandoverRangeIn},
                       false);
    const int used = static_cast<int>(pros::millis() - began);
    if (used >= timeout_ms) return;
    chassis.moveToPoint(x, y, timeout_ms - used,
                        {.forwards = forwards, .maxSpeed = s.max_speed}, false);
}

}  // namespace curve
