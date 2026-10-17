#pragma once

#include <cstdint>

#include "lemlib/api.hpp"

namespace lift_position {
inline constexpr float stage_0_deg = 0.0f;
inline constexpr float matchload = -250.0f;
inline constexpr float stage_1_deg = -660.7f;
inline constexpr float stage_2_deg = -1150.0f;  // 15% above the old -1000
// First stack onto the (taller) middle goal: 90% of the second level.
inline constexpr float middle_goal_stage_1_deg = 0.9f * stage_2_deg;
inline constexpr float max_height_deg = -1670.0f;
// From 95% of max height up, the lift is held at max height.
inline constexpr float lock_from_deg = 0.95f * max_height_deg;
inline constexpr float target_tolerance_ratio = 0.01f;
}
// Arm (claw_sensor degrees). Shared by the driver macros and the routes.
namespace arm_position {
inline constexpr double rest_deg = 0.0;
inline constexpr double max_deg = 530.0;      // Y in driver control
// RIGHT in driver control: the arm comes down to here and the claw lets go
// halfway there.
inline constexpr double release_deg = 100.0;
inline constexpr std::uint32_t release_settle_ms = 100;
}
extern pros::Motor side_toggle;
extern lemlib::Chassis chassis;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Imu imu;

extern pros::adi::DigitalOut claw_piston;
extern pros::Motor slider_left;
extern pros::Motor slider_right;
extern pros::Motor claw_arm;
extern pros::adi::DigitalOut clamp_piston;
extern pros::Rotation claw_sensor;
extern pros::Rotation lift_sensor;
