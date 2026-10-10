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
//            pros::lcd::print(7, "GPS RESET OK");
//   3b. OR start-relative frame (keeps setPose(0,0,180)-style autons working):
//        call ONCE with the robot stationary at its start pose, before auton:
//        gpsreset::capture_start_as(0, 0, 180);
//        then reset()/test() attempt bounded position corrections in that
//        frame while retaining LemLib's heading.
//   4. Read-only GPS coordinates in opcontrol(), with the known field start:
//        gpsreset::print_position_from_start(64, 6);
//      test() is separate: it can apply corrections to LemLib pose.

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
// Afterwards reset()/test() apply readings in the declared start-relative
// frame, so routes can receive start-relative GPS position corrections.
// Returns false if a stable reading is not available within a bounded 250 ms
// window; failure invalidates the
// current run's anchor so relative reset() calls fail closed.
inline bool capture_start_as(double x_in, double y_in, double theta_deg);

// Bounded reset for auton: requires a stable GPS sample, a valid anchor in
// relative mode, stable odometry, a stopped chassis, and a plausible correction.
// It updates X/Y while preserving LemLib heading. Returns true only if applied.
inline bool reset();

struct PositionEstimate {
    bool anchor_ok = false;
    bool ok = false;
    double x_in = 0.0;
    double y_in = 0.0;
    double theta_deg = 0.0;
};

// Read-only live GPS estimate. The known robot-center location in the field
// becomes (0,0); the captured GPS heading aligns it with the declared start
// heading. This does not change LemLib pose or apply a GPS reset.
inline PositionEstimate position_from_start(double field_start_x_in,
                                            double field_start_y_in);
// Print that estimate on LCD line 7; call at 100 ms intervals in opcontrol.
inline void print_position_from_start(double field_start_x_in,
                                      double field_start_y_in);

// Non-blocking opcontrol tick: call it every loop. When the GPS is confident,
// the chassis is stopped, and the correction is plausible, it updates X/Y
// while preserving heading and prints the resulting pose.
inline void test();

}  // namespace gpsreset

// Alias so both spellings work: gpsreset::reset() and gps_reset::reset().
namespace gps_reset = gpsreset;

#include "gps_reset_impl.hpp"
