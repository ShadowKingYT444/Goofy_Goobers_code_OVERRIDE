#pragma once

#include <cstdint>
#include "lemlib/api.hpp"

namespace avreset {

// Measure offsets from the LemLib tracking/rotation center to the CAMERA LENS.
// +forward = robot front, +right = robot right.
// yaw_deg is clockwise from robot-forward: 0 = front camera, 180 = rear camera.
// Camera height is not required for this planar field reset.
struct CameraMount {
    double forward_in = 0.0;
    double right_in = 0.0;
    double yaw_deg = 180.0;

    // Defaults for the VEX AI Vision camera. Calibrate if needed.
    double fx_px = 212.34;
    double fy_px = 195.82;
    double cx_px = 160.0;
    double cy_px = 120.0;
};

// Call once after chassis.calibrate().
inline void init(lemlib::Chassis& chassis, std::uint8_t ai_vision_port,
                 CameraMount mount = {});

// Blocking auton reset: waits briefly for 3 consistent good AprilTag readings.
// Corrects LemLib X/Y/theta and returns true only when a reset is applied.
inline bool reset();

// Non-blocking opcontrol test tick. Call every loop; good readings automatically
// reset pose and log to /usd/aivision_reset.csv when an SD card is installed.
inline void test();

}  // namespace avreset

#include "aivision_reset_impl.hpp"
