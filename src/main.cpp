#include "main.h"

#include <cmath>
#include <cstdio>

namespace {
pros::Gps gps(7);
constexpr double kMetersToInches = 39.37007874015748;

void report() {
  const auto raw = gps.get_position_and_orientation();
  const double error = gps.get_error();
  std::printf("GPS_NOW x_in=%.3f y_in=%.3f sensor_heading=%.3f "
              "robot_heading=%.3f rms_in=%.3f valid=%d\n",
              raw.x * kMetersToInches, raw.y * kMetersToInches, raw.yaw,
              std::remainder(raw.yaw - 90.0, 360.0),
              error * kMetersToInches,
              static_cast<int>(std::isfinite(error) && error > 0.0 &&
                               error < 0.0762));
  std::fflush(stdout);
}
}

void initialize() {
  pros::screen::set_eraser(0x00101824);
  pros::screen::erase();
  pros::Task::create([] {
    while (true) {
      report();
      pros::delay(100);
    }
  });
}
void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() { while (true) pros::delay(1000); }
