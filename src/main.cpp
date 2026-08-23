#include "main.h"

#include <cmath>

namespace {

pros::MotorGroup left_drive({17, 18});
pros::MotorGroup right_drive({-11, -13});

constexpr double kTravelMotorDegrees = 100.0;  // About 2.4 in with 2.75 in wheels.
constexpr int kVelocityRpm = 30;
constexpr std::uint32_t kMotionTimeoutMs = 3000;

void stop_drive() {
  left_drive.brake();
  right_drive.brake();
}

bool wait_for_target(double target_degrees) {
  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < kMotionTimeoutMs) {
    const auto left_positions = left_drive.get_position_all();
    const auto right_positions = right_drive.get_position_all();
    bool settled = true;
    for (const double position : left_positions) {
      settled = settled && std::fabs(position - target_degrees) < 4.0;
    }
    for (const double position : right_positions) {
      settled = settled && std::fabs(position - target_degrees) < 4.0;
    }
    if (settled) return true;
    pros::delay(20);
  }
  return false;
}

void run_once() {
  left_drive.set_brake_mode(pros::MotorBrake::hold);
  right_drive.set_brake_mode(pros::MotorBrake::hold);
  left_drive.tare_position();
  right_drive.tare_position();

  for (int seconds = 3; seconds > 0; --seconds) {
    pros::lcd::print(2, "Moving in %d...", seconds);
    pros::delay(1000);
  }

  pros::lcd::set_text(2, "Forward slowly");
  left_drive.move_absolute(kTravelMotorDegrees, kVelocityRpm);
  right_drive.move_absolute(kTravelMotorDegrees, kVelocityRpm);
  const bool forward_ok = wait_for_target(kTravelMotorDegrees);
  stop_drive();
  pros::delay(750);

  pros::lcd::set_text(2, "Returning slowly");
  left_drive.move_absolute(0.0, kVelocityRpm);
  right_drive.move_absolute(0.0, kVelocityRpm);
  const bool return_ok = wait_for_target(0.0);
  stop_drive();

  pros::lcd::set_text(2, forward_ok && return_ok ? "DONE - reboot to repeat"
                                                 : "STOPPED - motion timeout");
}

}  // namespace

void initialize() {
  pros::lcd::initialize();
  stop_drive();
  pros::lcd::set_text(1, "SAFE DRIVE TEST");
  pros::lcd::set_text(2, "Remote run armed");
}

void disabled() { stop_drive(); }

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  pros::delay(1000);
  run_once();
  while (true) {
    stop_drive();
    pros::delay(20);
  }
}
