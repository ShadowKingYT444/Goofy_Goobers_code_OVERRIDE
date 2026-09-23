#include "main.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
constexpr double kMmPerInch = 25.4;
constexpr double kPi = 3.14159265358979323846;
constexpr double kAiFxPx = 212.34;
constexpr double kAiImageCenterXPx = 160.0;
constexpr std::int32_t kTestPower = 25;
constexpr std::uint32_t kTelemetryPeriodMs = 100;

pros::Distance lidar_front(8);
pros::Distance lidar_rear(9);
pros::Rotation side_odom(5);
pros::Imu imu(3);
pros::Motor left_front(17);
pros::Motor left_rear(18);
pros::Motor right_front(-11);
pros::Motor right_rear(-13);

volatile int motion_phase = 0;
volatile bool telemetry_enabled = true;

struct TagSample {
  int count = 0;
  int id = -1;
  double center_x = 0.0;
  double center_y = 0.0;
  double mean_edge_px = 0.0;
  double bearing_deg = 0.0;
  int x[4] = {0, 0, 0, 0};
  int y[4] = {0, 0, 0, 0};
};

double point_distance(int x0, int y0, int x1, int y1) {
  return std::hypot(static_cast<double>(x1 - x0),
                    static_cast<double>(y1 - y0));
}

void stop_drive() {
  left_front.move(0);
  left_rear.move(0);
  right_front.move(0);
  right_rear.move(0);
}

void drive_tank(std::int32_t left, std::int32_t right) {
  left_front.move(left);
  left_rear.move(left);
  right_front.move(right);
  right_rear.move(right);
}

bool configure_ai(std::uint8_t port) {
  if (static_cast<int>(pros::c::get_plugged_type(port)) != 29) return false;
  const int reset = pros::c::aivision_reset(port);
  const int family =
      pros::c::aivision_set_tag_family_override(port, pros::TAG_CIRCLE_21H7);
  const int disable = pros::c::aivision_disable_detection_types(
      port, pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS |
                pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enable = pros::c::aivision_enable_detection_types(
      port, pros::E_AIVISION_MODE_TAGS);
  std::printf(
      "SYNC_AI_INIT port=%u reset=%d family=%d disable=%d enable=%d\n",
      static_cast<unsigned>(port), reset, family, disable, enable);
  std::fflush(stdout);
  return reset == 1 && family == 1 && disable == 1 && enable == 1;
}

TagSample read_best_tag(std::uint8_t port) {
  TagSample sample;
  const int count = pros::c::aivision_get_object_count(port);
  sample.count = std::clamp(count, 0, 24);
  double best_area = -1.0;
  pros::aivision_object_s_t best{};
  for (int i = 0; i < sample.count; ++i) {
    const auto object = pros::c::aivision_get_object(port, i);
    if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
    const auto& tag = object.object.tag;
    const double area = std::abs(
        static_cast<double>(tag.x0) * tag.y1 -
        static_cast<double>(tag.y0) * tag.x1 +
        static_cast<double>(tag.x1) * tag.y2 -
        static_cast<double>(tag.y1) * tag.x2 +
        static_cast<double>(tag.x2) * tag.y3 -
        static_cast<double>(tag.y2) * tag.x3 +
        static_cast<double>(tag.x3) * tag.y0 -
        static_cast<double>(tag.y3) * tag.x0) *
        0.5;
    if (area > best_area) {
      best_area = area;
      best = object;
    }
  }
  if (best_area < 0.0) return sample;

  const auto& tag = best.object.tag;
  sample.x[0] = tag.x0;
  sample.y[0] = tag.y0;
  sample.x[1] = tag.x1;
  sample.y[1] = tag.y1;
  sample.x[2] = tag.x2;
  sample.y[2] = tag.y2;
  sample.x[3] = tag.x3;
  sample.y[3] = tag.y3;
  sample.id = best.id;
  sample.center_x = 0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
  sample.center_y = 0.25 * (tag.y0 + tag.y1 + tag.y2 + tag.y3);
  sample.mean_edge_px =
      0.25 * (point_distance(tag.x0, tag.y0, tag.x1, tag.y1) +
              point_distance(tag.x1, tag.y1, tag.x2, tag.y2) +
              point_distance(tag.x2, tag.y2, tag.x3, tag.y3) +
              point_distance(tag.x3, tag.y3, tag.x0, tag.y0));
  sample.bearing_deg =
      std::atan((sample.center_x - kAiImageCenterXPx) / kAiFxPx) *
      180.0 / kPi;
  return sample;
}

void emit_sync_sample() {
  const std::uint32_t now = pros::millis();
  const std::int32_t lidar8_mm = lidar_front.get_distance();
  const std::int32_t lidar9_mm = lidar_rear.get_distance();
  const std::int32_t lidar8_conf = lidar_front.get_confidence();
  const std::int32_t lidar9_conf = lidar_rear.get_confidence();
  const TagSample cam19 = read_best_tag(19);
  const TagSample cam20 = read_best_tag(20);
  std::printf(
      "SYNC t=%lu phase=%d "
      "l8=%ld,%.3f,%ld l9=%ld,%.3f,%ld "
      "m17=%.3f,%.3f m18=%.3f,%.3f m11=%.3f,%.3f m13=%.3f,%.3f "
      "amps=%ld,%ld,%ld,%ld "
      "side5=%ld imu3=%.3f "
      "c19=%d,%d,%.2f,%.2f,%.2f,%.3f "
      "c20=%d,%d,%.2f,%.2f,%.2f,%.3f "
      "q19=%d,%d,%d,%d,%d,%d,%d,%d "
      "q20=%d,%d,%d,%d,%d,%d,%d,%d\n",
      static_cast<unsigned long>(now), motion_phase,
      static_cast<long>(lidar8_mm), lidar8_mm / kMmPerInch,
      static_cast<long>(lidar8_conf),
      static_cast<long>(lidar9_mm), lidar9_mm / kMmPerInch,
      static_cast<long>(lidar9_conf),
      left_front.get_position(), left_front.get_actual_velocity(),
      left_rear.get_position(), left_rear.get_actual_velocity(),
      right_front.get_position(), right_front.get_actual_velocity(),
      right_rear.get_position(), right_rear.get_actual_velocity(),
      static_cast<long>(left_front.get_current_draw()),
      static_cast<long>(left_rear.get_current_draw()),
      static_cast<long>(right_front.get_current_draw()),
      static_cast<long>(right_rear.get_current_draw()),
      static_cast<long>(side_odom.get_position()), imu.get_rotation(),
      cam19.count, cam19.id, cam19.center_x, cam19.center_y,
      cam19.mean_edge_px, cam19.bearing_deg,
      cam20.count, cam20.id, cam20.center_x, cam20.center_y,
      cam20.mean_edge_px, cam20.bearing_deg,
      cam19.x[0], cam19.y[0], cam19.x[1], cam19.y[1],
      cam19.x[2], cam19.y[2], cam19.x[3], cam19.y[3],
      cam20.x[0], cam20.y[0], cam20.x[1], cam20.y[1],
      cam20.x[2], cam20.y[2], cam20.x[3], cam20.y[3]);
  std::fflush(stdout);
}

void run_phase(int phase,
               std::int32_t left_power,
               std::int32_t right_power,
               std::uint32_t duration_ms) {
  motion_phase = phase;
  std::printf("SYNC_PHASE t=%lu phase=%d left=%ld right=%ld duration=%lu\n",
              static_cast<unsigned long>(pros::millis()), phase,
              static_cast<long>(left_power), static_cast<long>(right_power),
              static_cast<unsigned long>(duration_ms));
  std::fflush(stdout);
  drive_tank(left_power, right_power);
  pros::delay(duration_ms);
  stop_drive();
}

void run_imu_sweep(int phase,
                   std::int32_t left_power,
                   std::int32_t right_power,
                   double target_delta_deg,
                   std::uint32_t timeout_ms) {
  motion_phase = phase;
  const double start_imu_deg = imu.get_rotation();
  const std::uint32_t start_ms = pros::millis();
  std::printf(
      "SYNC_SWEEP t=%lu phase=%d start_imu=%.3f target_delta=%.3f "
      "left=%ld right=%ld timeout=%lu\n",
      static_cast<unsigned long>(start_ms), phase, start_imu_deg,
      target_delta_deg, static_cast<long>(left_power),
      static_cast<long>(right_power), static_cast<unsigned long>(timeout_ms));
  std::fflush(stdout);
  drive_tank(left_power, right_power);
  std::uint32_t stalled_since_ms = 0;
  while (pros::millis() - start_ms < timeout_ms) {
    const double delta_deg = imu.get_rotation() - start_imu_deg;
    const bool reached =
        target_delta_deg >= 0.0 ? delta_deg >= target_delta_deg
                                : delta_deg <= target_delta_deg;
    if (std::isfinite(delta_deg) && reached) break;

    const double mean_commanded_rpm =
        ((left_power != 0
              ? std::abs(left_front.get_actual_velocity()) +
                    std::abs(left_rear.get_actual_velocity())
              : 0.0) +
         (right_power != 0
              ? std::abs(right_front.get_actual_velocity()) +
                    std::abs(right_rear.get_actual_velocity())
              : 0.0)) /
        (2.0 * ((left_power != 0) + (right_power != 0)));
    if (pros::millis() - start_ms > 500 && mean_commanded_rpm < 1.0) {
      if (stalled_since_ms == 0) stalled_since_ms = pros::millis();
      if (pros::millis() - stalled_since_ms >= 500) {
        std::printf("SYNC_STALL t=%lu phase=%d imu=%.3f\n",
                    static_cast<unsigned long>(pros::millis()), phase,
                    imu.get_rotation());
        std::fflush(stdout);
        break;
      }
    } else {
      stalled_since_ms = 0;
    }
    pros::delay(10);
  }
  stop_drive();
  std::printf("SYNC_SWEEP_DONE t=%lu phase=%d final_imu=%.3f delta=%.3f\n",
              static_cast<unsigned long>(pros::millis()), phase,
              imu.get_rotation(), imu.get_rotation() - start_imu_deg);
  std::fflush(stdout);
}
}  // namespace

void initialize() {
  stop_drive();
  pros::delay(500);
  for (std::uint8_t port = 1; port <= 21; ++port) {
    const int type = static_cast<int>(pros::c::get_plugged_type(port));
    if (type != 0) {
      std::printf("SYNC_DEVICE port=%u type=%d\n",
                  static_cast<unsigned>(port), type);
    }
  }
  const bool cam19_ready = configure_ai(19);
  const bool cam20_ready = configure_ai(20);
  imu.reset(true);
  left_front.tare_position();
  left_rear.tare_position();
  right_front.tare_position();
  right_rear.tare_position();
  side_odom.reset_position();
  std::printf(
      "SYNC_READY lidar8=%d lidar9=%d side5=%d imu3=%d cam19=%d cam20=%d\n",
      static_cast<int>(lidar_front.is_installed()),
      static_cast<int>(lidar_rear.is_installed()),
      static_cast<int>(side_odom.is_installed()),
      static_cast<int>(imu.is_installed()),
      static_cast<int>(cam19_ready), static_cast<int>(cam20_ready));
  std::fflush(stdout);

  pros::Task telemetry([] {
    while (telemetry_enabled) {
      emit_sync_sample();
      pros::delay(kTelemetryPeriodMs);
    }
  });

  run_phase(0, 0, 0, 2000);
  run_phase(21, 0, 0, 10000);
  motion_phase = 9;
  stop_drive();
  emit_sync_sample();
  telemetry_enabled = false;
  std::printf("SYNC_DONE t=%lu power=0\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  while (true) pros::delay(1000);
}

void disabled() { stop_drive(); }
void competition_initialize() {}
void autonomous() {}
void opcontrol() {
  stop_drive();
  while (true) pros::delay(1000);
}
