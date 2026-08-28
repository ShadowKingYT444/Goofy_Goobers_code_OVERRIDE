#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;
extern pros::Distance distance_1;
extern pros::Gps gps_7;
extern pros::Rotation horizontal_odom;

inline pros::Motor slider_right(2);
// Keep in sync with current branch wiring before enabling slider_left motion.
inline pros::Motor slider_left(-9);

inline pros::Rotation slider_rotation_sensor(16);

inline pros::adi::DigitalOut clamp_piston('H');

inline pros::Motor claw_arm(4);
inline pros::Motor upper_intake(15);
inline pros::Motor counter_rollers(3);
