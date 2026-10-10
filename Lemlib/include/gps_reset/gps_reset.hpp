#pragma once

#include <cstdint>

#include "lemlib/api.hpp"

namespace gpsreset {

// The pipeline, start to finish:
//
//   1. Wall strips -> sensor.  The VEX GPS camera reads the checkerboard
//      field-code strips on the walls and reports where its LENS is on the
//      field (origin = field center, heading 0 = +Y, clockwise) with an RMS
//      error estimate. Readings above 3 in of error are discarded.
//   2. Sensor -> robot center, absolute.  read_pose() removes the mounting:
//      camera direction (facing_deg) and lens offset (forward/right).
//   3. Noise screen.  live_pose() runs every reading through the Estimator,
//      which blends GPS with drivetrain odometry: smooth between fixes,
//      absolute over time, and it ignores jumps the robot could not have made.
//   4. Absolute -> route coordinates.  capture_start_as(0, 0, 180) records
//      the field pose at the start; to_auton_frame() then expresses any field
//      pose in the same start-relative frame the autonomous routes use.
//
// Usage:
//   initialize():  gpsreset::init(chassis, port, forward_in, right_in, facing);
//                  gpsreset::capture_start_as(0, 0, 180);   // robot at start
//   telemetry:     gpsreset::live_pose()                     // one task only
//   in a route:    gpsreset::set_auto_reset(true)  // corrects odometry at rest
//                  gpsreset::reset()               // or one explicit correction

// Call once in initialize(), after chassis.calibrate().
//   port        = smart port the GPS sensor is plugged into
//   forward_in  = inches from robot rotation center to the GPS sensor,
//                 + = toward robot front  (0 if centered)
//   right_in    = inches from robot rotation center to the GPS sensor,
//                 + = toward robot right  (0 if centered)
//   facing_deg  = direction the GPS camera looks, clockwise from the robot's
//                 front: 0 = forward, 90 = right, 180 = rearward, 270 = left.
//                 The sensor reports the heading of its camera, so a wrong
//                 value rotates every GPS coordinate about the start by the
//                 same error (90 off swaps the X and Y axes).
// The offsets let the GPS report the robot's center instead of the sensor's
// own position. Measure them with a ruler; errors show up as the GPS position
// swinging by a few inches when the robot turns in place.
inline void init(lemlib::Chassis& chassis, std::uint8_t gps_port,
                 double forward_in = 0.0, double right_in = 0.0,
                 double facing_deg = 0.0);

// Blocking: anchor the relative (auton-frame) mode. Call once per match with
// the robot stationary at its start pose, before autonomous() runs.
// (x_in, y_in, theta_deg) declares what that physical spot means in your
// auton coordinates, e.g. (0, 0, 180) to match setPose(0,0,180) autons.
// Afterwards reset() applies readings in the declared start-relative frame,
// so routes can receive start-relative GPS position corrections.
// Returns false if a stable reading is not available within a bounded 250 ms
// window; failure invalidates the
// current run's anchor so relative reset() calls fail closed.
inline bool capture_start_as(double x_in, double y_in, double theta_deg);

using AnchorObserver = void (*)(bool valid, double field_x_in,
                                double field_y_in, double field_heading);
// Optional read-only display listener, notified when capture invalidates or
// establishes the start's GPS pose. It must not call back into GPS operations.
inline void set_anchor_observer(AnchorObserver observer);

// Automatic correction: while enabled (and the start anchor is valid), the
// telemetry task's live_pose() calls correct LemLib's X/Y from the screened
// GPS estimate whenever the robot has sat still for 0.3 s between motions.
// Corrections under 0.75 in or over 8 in are not applied; heading is never
// changed. Turn it on at the start of a route and off at the end.
inline void set_auto_reset(bool enabled);

// Route checkpoint: call right after a motion that was driving to (x, y) in
// route coordinates. If the GPS is confident, the robot is stopped, the GPS
// places it within radius_in of that waypoint and within 8 in of odometry,
// LemLib's X/Y is set from the GPS (heading untouched). Gaps under 1.5 in are
// left alone. Waits at most wait_ms for those conditions; returns true if a
// correction was applied. Needs the telemetry task to be calling live_pose().
inline bool checkpoint(double x_in, double y_in, double radius_in = 6.0,
                       std::uint32_t wait_ms = 200);

// Bounded reset for auton: requires a stable GPS sample, a valid anchor in
// relative mode, stable odometry, a stopped chassis, and a plausible correction.
// It updates X/Y while preserving LemLib heading. Returns true only if applied.
inline bool reset();

}  // namespace gpsreset

// Alias so both spellings work: gpsreset::reset() and gps_reset::reset().
namespace gps_reset = gpsreset;

#include "gps_reset_impl.hpp"
