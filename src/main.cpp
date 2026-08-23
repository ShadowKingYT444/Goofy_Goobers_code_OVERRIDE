#include "main.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

pros::Gps gps(12);
pros::MotorGroup left_drive({1, 11});
pros::MotorGroup right_drive({-10, -2});

constexpr double kMetersToInches = 39.37007874015748;
constexpr double kSensorRightIn = 6.0;
constexpr double kSensorForwardIn = -6.0;
constexpr double kSensorYawDeg = 90.0;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaximumPower = 35;

struct CenterPose { double x; double y; double heading; double error; };

double wrap(double angle) {
  while (angle >= 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void stop_drive() {
  left_drive.move(0);
  right_drive.move(0);
  left_drive.brake();
  right_drive.brake();
}

CenterPose center_pose() {
  const auto raw = gps.get_position_and_orientation();
  const double heading = wrap(raw.yaw - kSensorYawDeg);
  const double radians = heading * kPi / 180.0;
  const double sensor_dx = kSensorRightIn * std::cos(radians) +
                           kSensorForwardIn * std::sin(radians);
  const double sensor_dy = -kSensorRightIn * std::sin(radians) +
                           kSensorForwardIn * std::cos(radians);
  return {raw.x * kMetersToInches - sensor_dx,
          raw.y * kMetersToInches - sensor_dy,
          heading, gps.get_error() * kMetersToInches};
}

bool pose_valid(const CenterPose& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.heading) && std::isfinite(pose.error) &&
         pose.error > 0.0 && pose.error < 3.0;
}

bool all_motors_ready() {
  const auto left = left_drive.get_position_all();
  const auto right = right_drive.get_position_all();
  return left.size() == 2 && right.size() == 2 &&
         std::isfinite(left[0]) && std::isfinite(left[1]) &&
         std::isfinite(right[0]) && std::isfinite(right[1]);
}

void report(const char* phase, const CenterPose& pose, double tx, double ty,
            int left, int right) {
  std::printf("GPS_PATH phase=%s t=%u x=%.3f y=%.3f h=%.3f err=%.3f "
              "tx=%.3f ty=%.3f cmd=%d,%d\n", phase, pros::millis(), pose.x,
              pose.y, pose.heading, pose.error, tx, ty, left, right);
  std::fflush(stdout);
}

bool go_to(double target_x, double target_y, std::uint32_t timeout_ms,
           const char* phase) {
  const std::uint32_t started = pros::millis();
  int settled = 0;
  while (pros::millis() - started < timeout_ms) {
    const CenterPose pose = center_pose();
    if (!pose_valid(pose) || !all_motors_ready()) {
      stop_drive();
      report("abort_sensor", pose, target_x, target_y, 0, 0);
      return false;
    }
    const double dx = target_x - pose.x;
    const double dy = target_y - pose.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1.25) {
      stop_drive();
      report(phase, pose, target_x, target_y, 0, 0);
      if (++settled >= 6) return true;
      pros::delay(50);
      continue;
    }
    settled = 0;
    double desired = std::atan2(dx, dy) * 180.0 / kPi;
    double heading_error = wrap(desired - pose.heading);
    bool reverse = false;
    if (std::abs(heading_error) > 90.0) {
      reverse = true;
      desired = wrap(desired + 180.0);
      heading_error = wrap(desired - pose.heading);
    }
    int drive = static_cast<int>(std::clamp(distance * 4.0, 26.0,
                                             static_cast<double>(kMaximumPower)));
    if (reverse) drive = -drive;
    if (std::abs(heading_error) > 30.0) drive = 0;
    const int turn = static_cast<int>(std::clamp(heading_error * 0.65,
                                                 -25.0, 25.0));
    const int left = std::clamp(drive + turn, -kMaximumPower, kMaximumPower);
    const int right = std::clamp(drive - turn, -kMaximumPower, kMaximumPower);
    left_drive.move(left);
    right_drive.move(right);
    report(phase, pose, target_x, target_y, left, right);
    pros::delay(50);
  }
  stop_drive();
  return false;
}

void run_once() {
  stop_drive();
  CenterPose start{};
  const std::uint32_t gps_wait_started = pros::millis();
  do {
    start = center_pose();
    pros::delay(100);
  } while (!pose_valid(start) && pros::millis() - gps_wait_started < 4000);
  if (!pose_valid(start) || !all_motors_ready()) {
    std::printf("GPS_PATH ABORT initial_sensor_check\n");
    std::fflush(stdout);
    return;
  }
  for (int seconds = 3; seconds > 0; --seconds) {
    pros::screen::set_pen(0x00F3F7FA);
    pros::screen::print(TEXT_LARGE, 80, 110, "Moving in %d...", seconds);
    pros::delay(1000);
  }
  start = center_pose();
  const double radians = start.heading * kPi / 180.0;
  const double target_x = start.x + 18.0 * std::sin(radians);
  const double target_y = start.y + 18.0 * std::cos(radians);
  report("start", start, target_x, target_y, 0, 0);
  const bool outbound_ok = go_to(target_x, target_y, 10000, "outbound");
  pros::delay(800);
  const bool return_ok = outbound_ok && go_to(start.x, start.y, 10000, "return");
  stop_drive();
  const CenterPose final = center_pose();
  report(return_ok ? "done" : "failed", final, start.x, start.y, 0, 0);
  pros::screen::set_pen(return_ok ? 0x0049E391 : 0x00FF5B68);
  pros::screen::print(TEXT_LARGE, 100, 110,
                      return_ok ? "STOPPED - DONE" : "STOPPED - FAILED");
}

}  // namespace

void initialize() {
  stop_drive();
  pros::screen::set_eraser(0x00101824);
  pros::screen::erase();
  pros::Task::create(run_once);
}
void disabled() { stop_drive(); }
void competition_initialize() {}
void autonomous() {}
void opcontrol() { while (true) pros::delay(1000); }
