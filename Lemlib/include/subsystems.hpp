#pragma once

#include "lemlib/api.hpp"

namespace lift_position {
inline constexpr float stage_0_deg = 0.0f;
inline constexpr float matchload = -250.0f;
inline constexpr float stage_1_deg = -600.7f;
inline constexpr float stage_2_deg = -1000.0f;
inline constexpr float max_height_deg = -1670.0f;
inline constexpr float target_tolerance_ratio = 0.01f;
}
extern pros::Motor side_toggle;
extern lemlib::Chassis chassis;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Imu imu;
//extern lemlib::TrackingWheel vertical_wheel;

extern pros::adi::DigitalOut claw_piston;
extern pros::Motor slider_left;
extern pros::Motor slider_right;
extern pros::Motor claw_arm;
extern pros::adi::DigitalOut clamp_piston;
extern pros::Rotation claw_sensor;
extern pros::Rotation lift_sensor;