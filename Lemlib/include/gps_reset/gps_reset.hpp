#pragma once

#include <cstdint>

#include "lemlib/api.hpp"

namespace gpsreset {

// How it works (30-second version):
// The VEX GPS has a camera that watches the black-and-white checkerboard
// "field code" strips printed on the field walls. The pattern never repeats,
// so from the chunk of strip it sees the sensor triangulates its absolute
// X/Y/heading on the field (origin = field center, meters, heading 0 = north
// going clockwise). It also reports an RMS error estimate: small error =
// high confidence. We only trust it when that error is tiny.
//
// Usage (3 lines in your existing code):
//   1. #include "gps_reset/gps_reset.hpp"
//   2. in initialize(), after chassis.calibrate():
//        gpsreset::init(chassis, /*port=*/7);
//   3a. absolute field frame (origin = field center):
//        if (gpsreset::reset())
//            pros::screen::print(pros::E_TEXT_SMALL, 8, 208, "GPS RESET OK");
//   3b. OR start-relative frame (keeps setPose(0,0,180)-style autons working):
//        call ONCE with the robot stationary at its start pose, before auton:
//        gpsreset::capture_start_as(0, 0, 180);
//        then reset()/test() apply drift-free GPS corrections in that frame.
//      or non-blocking test tick in opcontrol():
//        gpsreset::test();   // shows last reset pose at the bottom of the brain screen
//        ("GPSR" prefix = relative mode, "GPS" = absolute.)

// Call once in initialize(), after chassis.calibrate().
//   port        = smart port the GPS sensor is plugged into
//   forward_in  = inches from robot rotation center to the GPS sensor,
//                 + = toward robot front  (0 if centered)
//   right_in    = inches from robot rotation center to the GPS sensor,
//                 + = toward robot right  (0 if centered)
// The offsets let the GPS report the robot's center instead of the sensor's
// own position. Measure them with a ruler; sign errors show up as a constant
// few-inch offset in the reset pose.
inline void init(lemlib::Chassis& chassis, std::uint8_t gps_port,
                 double forward_in = 0.0, double right_in = 0.0);

// Blocking: anchor the relative (auton-frame) mode. Call once per match with
// the robot stationary at its start pose, before autonomous() runs.
// (x_in, y_in, theta_deg) declares what that physical spot means in your
// auton coordinates, e.g. (0, 0, 180) to match setPose(0,0,180) autons.
// Afterwards reset()/test() apply P_rel = P_abs - P0 + D, so start-relative
// paths get drift-free GPS corrections. Returns false if no confident GPS
// reading appears within ~2 s (mode left unchanged).
inline bool capture_start_as(double x_in, double y_in, double theta_deg);

// Blocking reset for auton: polls the GPS (up to ~1 s) and hard-resets the
// LemLib pose the moment confidence is high. Returns true only if a reset
// was actually applied. Safe to call from initialize() or autonomous().
inline bool reset();

// Non-blocking opcontrol tick: call it every loop. Whenever the GPS is
// confident it snaps the LemLib pose to the GPS reading and prints the pose
// it reset to at the bottom of the brain screen, so you can see it working live.
inline void test();

}  // namespace gpsreset

// Alias so both spellings work: gpsreset::reset() and gps_reset::reset().
namespace gps_reset = gpsreset;

#include "gps_reset_impl.hpp"
