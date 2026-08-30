#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

#include <atomic>

extern Drive chassis;
extern pros::Distance distance_1;
extern pros::Gps gps_7;
extern pros::Rotation horizontal_odom;

inline pros::Motor slider_right(2);
// Keep in sync with current branch wiring before enabling slider_left motion.
inline pros::Motor slider_left(-9);

inline pros::Rotation slider_rotation_sensor(16);

inline pros::adi::DigitalOut clamp_piston('D');
inline pros::adi::DigitalOut claw_piston('E');
inline std::atomic<bool> claw_piston_extended{false};

inline void set_claw_piston(bool extended) {
  claw_piston.set_value(extended);
  claw_piston_extended.store(extended, std::memory_order_release);
}

// Port 4 is now a full-size 11 W V5 motor with the standard green cartridge.
inline pros::Motor claw_arm(4, pros::v5::MotorGears::green);
inline pros::Motor upper_intake(15);
// Legacy port-3 motor retained only for disabled diagnostics; the active claw
// is now the ADI-E piston above.
inline pros::Motor counter_rollers(3);
