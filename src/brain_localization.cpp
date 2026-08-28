#include "main.h"
#include "brain_localization.hpp"
#include "pros/apix.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMmPerInch = 25.4;
constexpr double kDriveWheelDiameterIn = 2.75;
constexpr double kSideWheelDiameterIn = 2.0;
constexpr double kSideRotationLeverIn = 0.90;
constexpr double kEncoderYawDegPerDifferentialDeg = 0.086;
constexpr double kTagSizeIn = 18.0 / kMmPerInch;
constexpr double kGoalFaceOffsetIn = 5.61 / 2.0;
constexpr double kCameraHalfBaselineIn = (35.0 / 2.54) / 2.0;
constexpr std::uint32_t kLoopPeriodMs = 20;
constexpr std::uint32_t kVisionPeriodMs = 100;
constexpr std::uint32_t kTelemetryPeriodMs = 100;
constexpr std::uint32_t kTagCorrectionCooldownMs = 250;
constexpr double kMaximumNormalTagInnovationIn = 12.0;
constexpr double kTagPositionGain = 0.25;
constexpr double kMaximumTagPositionStepIn = 0.75;
constexpr double kMinimumTagEdgePx = 4.5;
constexpr int kAiVisionDeviceType = 29;

pros::Motor left_front(17);
pros::Motor left_rear(18);
pros::Motor right_front(-11);
pros::Motor right_rear(-13);
pros::Rotation side_odom(5);
pros::Imu imu(3);
pros::Distance lidar_8(8);
pros::Distance lidar_9(9);
pros::Gps gps_7(7);

using Pose = brainloc::Pose;
using Waypoint = brainloc::Waypoint;

struct Goal {
  const char* name;
  int id;
  double x_in;
  double y_in;
};

constexpr std::array<Goal, 9> kGoals{{
    {"center", 0, 72.0, 72.0},
    {"upper_left_1", 1, 24.0, 96.0},
    {"lower_right_1", 1, 120.0, 48.0},
    {"lower_left_2", 2, 24.0, 48.0},
    {"upper_right_2", 2, 120.0, 96.0},
    {"bottom_3", 3, 48.0, 24.0},
    {"top_3", 3, 96.0, 120.0},
    {"top_4", 4, 48.0, 120.0},
    {"bottom_4", 4, 96.0, 24.0},
}};

struct CameraConfig {
  std::uint8_t port;
  double fx_px;
  double fy_px;
  double cx_px;
  double cy_px;
  double forward_offset_in;
  double right_offset_in;
  double yaw_deg;
  double range_scale;
};

// Intrinsics are the verified 640x480 calibration divided by two because the
// Brain API reports 320x240 coordinates. Extrinsics match the measured 35 cm
// lens baseline and opposed left/right mounting.
constexpr std::array<CameraConfig, 2> kCameras{{
    {19, 465.1726312941188 / 2.0, 465.4083323165754 / 2.0,
     348.31937245110544 / 2.0, 238.9504806160313 / 2.0,
     0.0, -kCameraHalfBaselineIn, 60.7, 1.03},
    {20, 384.3048291621567 / 2.0, 384.62417346588273 / 2.0,
     339.5266656603091 / 2.0, 222.19229544431812 / 2.0,
     0.0, kCameraHalfBaselineIn, -110.0, 1.0},
}};

struct TagObservation {
  bool valid = false;
  int id = -1;
  std::array<int, 8> corners{};
  double center_x_px = 0.0;
  double center_y_px = 0.0;
  double mean_edge_px = 0.0;
  double range_in = 0.0;
  double bearing_deg = 0.0;
  int same_id_streak = 0;
  bool geometry_fresh = false;
};

struct CameraState {
  TagObservation observation;
  int previous_id = -1;
  std::array<int, 9> previous_geometry{};
  std::array<int, 9> corrected_geometry{};
  bool have_previous_geometry = false;
  bool have_corrected_geometry = false;
  std::uint32_t last_correction_ms = 0;
};

struct TagCandidate {
  bool valid = false;
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_deg = 0.0;
  double position_innovation_in = 0.0;
  double heading_residual_deg = 0.0;
  double score = 1e9;
  const char* goal = "none";
  int face_index = -1;
};

Pose fused_pose{};
pros::Mutex pose_mutex;
pros::Mutex path_mutex;
pros::Mutex camera_mutex;
constexpr std::size_t kHistoryCapacity = 1000;
std::array<Pose, kHistoryCapacity> pose_history{};
std::size_t history_head = 0;
std::size_t history_size = 0;
std::vector<Waypoint> active_path;
std::atomic_bool initialization_started{false};
std::atomic_bool localization_started{false};
std::atomic_bool cancel_requested{false};
std::atomic_bool motion_running{false};
std::array<CameraState, 2> camera_states{};
double previous_left_deg = 0.0;
double previous_right_deg = 0.0;
double previous_side_deg = 0.0;
double previous_imu_deg = 0.0;
double accumulated_left_deg = 0.0;
double accumulated_right_deg = 0.0;
double accumulated_side_deg = 0.0;
double accumulated_imu_deg = 0.0;
int accumulated_odometry_samples = 0;
std::uint32_t last_vision_ms = 0;
std::uint32_t last_telemetry_ms = 0;
bool encoder_consistent = true;
bool lidar_candidate_valid = false;
double lidar_wall_angle_deg = 0.0;
int applied_tag_port = 0;
int applied_tag_id = -1;
const char* applied_goal = "none";
double last_tag_innovation_in = 0.0;
std::uint32_t last_tag_correction_ms = 0;

double degrees_to_radians(double degrees) { return degrees * kPi / 180.0; }
double radians_to_degrees(double radians) { return radians * 180.0 / kPi; }

double wrap_degrees(double degrees) {
  while (degrees >= 180.0) degrees -= 360.0;
  while (degrees < -180.0) degrees += 360.0;
  return degrees;
}

Pose snapshot_pose() {
  pose_mutex.take();
  Pose result = fused_pose;
  pose_mutex.give();
  return result;
}

void record_pose(std::uint32_t now) {
  pose_mutex.take();
  fused_pose.timestamp_ms = now;
  pose_history[history_head] = fused_pose;
  history_head = (history_head + 1) % kHistoryCapacity;
  history_size = std::min(history_size + 1, kHistoryCapacity);
  pose_mutex.give();
}

void set_active_path(const std::vector<Waypoint>& path) {
  path_mutex.take();
  active_path = path;
  path_mutex.give();
}

double point_distance(int x0, int y0, int x1, int y1) {
  return std::hypot(static_cast<double>(x1 - x0),
                    static_cast<double>(y1 - y0));
}

bool is_known_self_tag(const CameraConfig& camera,
                       const pros::aivision_object_s_t& object) {
  // Port 19 repeatedly sees ID 3 at the same camera-relative pose while the
  // chassis translates. That proves this particular observation is mounted on
  // (or moves with) the robot rather than being a global field landmark.
  if (camera.port != 19 || object.id != 3 ||
      object.type != pros::E_AIVISION_DETECTED_TAG) {
    return false;
  }
  const auto& tag = object.object.tag;
  const double center_x =
      0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
  const double top =
      point_distance(tag.x0, tag.y0, tag.x1, tag.y1);
  const double bottom =
      point_distance(tag.x2, tag.y2, tag.x3, tag.y3);
  const double left =
      point_distance(tag.x3, tag.y3, tag.x0, tag.y0);
  const double right =
      point_distance(tag.x1, tag.y1, tag.x2, tag.y2);
  if (std::min({top, bottom, left, right}) < 1.0) return false;
  const double width_px = 0.5 * (top + bottom);
  const double height_px = 0.5 * (left + right);
  const double z_width = camera.fx_px * kTagSizeIn / width_px;
  const double z_height = camera.fy_px * kTagSizeIn / height_px;
  const double normalized_x = (center_x - camera.cx_px) / camera.fx_px;
  const double range_in =
      0.5 * (z_width + z_height) *
      std::sqrt(1.0 + normalized_x * normalized_x);
  const double bearing_deg =
      radians_to_degrees(std::atan(normalized_x));
  return range_in >= 14.0 && range_in <= 23.0 &&
         bearing_deg >= -38.0 && bearing_deg <= -22.0;
}

void stop_drive() {
  left_front.move(0);
  left_rear.move(0);
  right_front.move(0);
  right_rear.move(0);
}

void drive_tank(int left, int right) {
  left_front.move(left);
  left_rear.move(left);
  right_front.move(right);
  right_rear.move(right);
}

bool configure_camera(std::uint8_t port) {
  if (static_cast<int>(pros::c::get_plugged_type(port)) !=
      kAiVisionDeviceType) {
    return false;
  }
  const int reset = pros::c::aivision_reset(port);
  pros::delay(200);
  const int family = pros::c::aivision_set_tag_family_override(
      port, pros::TAG_CIRCLE_21H7);
  const int disabled = pros::c::aivision_disable_detection_types(
      port, pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS |
                pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enabled = pros::c::aivision_enable_detection_types(
      port, pros::E_AIVISION_MODE_TAGS);
  std::printf("LOC_CAMERA_INIT port=%u reset=%d family=%d disable=%d enable=%d\n",
              static_cast<unsigned>(port), reset, family, disabled, enabled);
  return reset == 1 && family == 1 && disabled == 1 && enabled == 1;
}

TagObservation read_camera(std::size_t camera_index) {
  const CameraConfig& camera = kCameras[camera_index];
  CameraState& state = camera_states[camera_index];
  TagObservation result;
  const int count = pros::c::aivision_get_object_count(camera.port);
  double largest_area = -1.0;
  pros::aivision_object_s_t best{};
  for (int index = 0; index < std::clamp(count, 0, 24); ++index) {
    const auto object = pros::c::aivision_get_object(camera.port, index);
    if (object.type != pros::E_AIVISION_DETECTED_TAG ||
        object.id > 4 || is_known_self_tag(camera, object)) {
      continue;
    }
    const auto& tag = object.object.tag;
    const double area = std::abs(
        static_cast<double>(tag.x0) * tag.y1 -
        static_cast<double>(tag.y0) * tag.x1 +
        static_cast<double>(tag.x1) * tag.y2 -
        static_cast<double>(tag.y1) * tag.x2 +
        static_cast<double>(tag.x2) * tag.y3 -
        static_cast<double>(tag.y2) * tag.x3 +
        static_cast<double>(tag.x3) * tag.y0 -
        static_cast<double>(tag.y3) * tag.x0) * 0.5;
    if (area > largest_area) {
      largest_area = area;
      best = object;
    }
  }
  if (largest_area < 20.0) {
    state.previous_id = -1;
    state.observation = result;
    return result;
  }

  const auto& tag = best.object.tag;
  result.id = best.id;
  result.corners = {tag.x0, tag.y0, tag.x1, tag.y1,
                    tag.x2, tag.y2, tag.x3, tag.y3};
  const std::array<double, 4> edges{
      point_distance(tag.x0, tag.y0, tag.x1, tag.y1),
      point_distance(tag.x1, tag.y1, tag.x2, tag.y2),
      point_distance(tag.x2, tag.y2, tag.x3, tag.y3),
      point_distance(tag.x3, tag.y3, tag.x0, tag.y0),
  };
  const auto [minimum_edge, maximum_edge] =
      std::minmax_element(edges.begin(), edges.end());
  result.mean_edge_px = 0.25 * (edges[0] + edges[1] + edges[2] + edges[3]);
  if (*minimum_edge < kMinimumTagEdgePx ||
      *maximum_edge / *minimum_edge > 2.0) {
    state.previous_id = -1;
    state.observation = result;
    return result;
  }
  result.center_x_px =
      0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
  result.center_y_px =
      0.25 * (tag.y0 + tag.y1 + tag.y2 + tag.y3);
  const double width_px = 0.5 * (edges[0] + edges[2]);
  const double height_px = 0.5 * (edges[1] + edges[3]);
  const double z_width = camera.fx_px * kTagSizeIn / width_px;
  const double z_height = camera.fy_px * kTagSizeIn / height_px;
  const double optical_axis_range = 0.5 * (z_width + z_height);
  const double normalized_x =
      (result.center_x_px - camera.cx_px) / camera.fx_px;
  result.range_in =
      camera.range_scale * optical_axis_range *
      std::sqrt(1.0 + normalized_x * normalized_x);
  result.bearing_deg = radians_to_degrees(std::atan(normalized_x));
  result.same_id_streak =
      state.previous_id == result.id ? state.observation.same_id_streak + 1 : 1;
  state.previous_id = result.id;
  const std::array<int, 9> geometry{
      result.id, tag.x0, tag.y0, tag.x1, tag.y1,
      tag.x2, tag.y2, tag.x3, tag.y3};
  result.geometry_fresh =
      !state.have_previous_geometry || geometry != state.previous_geometry;
  state.previous_geometry = geometry;
  state.have_previous_geometry = true;
  result.valid = std::isfinite(result.range_in) &&
                 result.range_in >= 4.0 && result.range_in <= 96.0;
  state.observation = result;
  return result;
}

TagCandidate best_candidate(const CameraConfig& camera,
                            const TagObservation& observation) {
  TagCandidate best;
  constexpr double diagonal = 0.70710678118654752440;
  constexpr std::array<std::array<double, 2>, 4> normals{{
      {diagonal, diagonal}, {-diagonal, diagonal},
      {-diagonal, -diagonal}, {diagonal, -diagonal},
  }};
  for (const Goal& goal : kGoals) {
    if (goal.id != observation.id) continue;
    for (std::size_t face = 0; face < normals.size(); ++face) {
      const double nx = normals[face][0];
      const double ny = normals[face][1];
      const double tag_x = goal.x_in + nx * kGoalFaceOffsetIn;
      const double tag_y = goal.y_in + ny * kGoalFaceOffsetIn;
      // A single planar tag's range+bearing does not independently observe
      // robot heading. Use the IMU-propagated heading and let the tag correct
      // position only; the face normal is visibility metadata, not a ray.
      const double candidate_heading = fused_pose.heading_deg;
      const double heading_radians = degrees_to_radians(candidate_heading);
      const double ray_heading_radians = degrees_to_radians(
          candidate_heading + camera.yaw_deg + observation.bearing_deg);
      const double camera_x =
          tag_x - observation.range_in * std::cos(ray_heading_radians);
      const double camera_y =
          tag_y - observation.range_in * std::sin(ray_heading_radians);
      const double forward_x = std::cos(heading_radians);
      const double forward_y = std::sin(heading_radians);
      const double right_x = std::sin(heading_radians);
      const double right_y = -std::cos(heading_radians);
      const double robot_x = camera_x -
          camera.forward_offset_in * forward_x -
          camera.right_offset_in * right_x;
      const double robot_y = camera_y -
          camera.forward_offset_in * forward_y -
          camera.right_offset_in * right_y;
      if (robot_x < -6.0 || robot_x > 150.0 ||
          robot_y < -6.0 || robot_y > 150.0) {
        continue;
      }
      const double position_innovation =
          std::hypot(robot_x - fused_pose.x_in, robot_y - fused_pose.y_in);
      const double heading_residual =
          0.0;
      const double score = position_innovation;
      if (score < best.score) {
        best.valid = true;
        best.x_in = robot_x;
        best.y_in = robot_y;
        best.heading_deg = candidate_heading;
        best.position_innovation_in = position_innovation;
        best.heading_residual_deg = heading_residual;
        best.score = score;
        best.goal = goal.name;
        best.face_index = static_cast<int>(face);
      }
    }
  }
  return best;
}

void apply_tag_correction(std::size_t camera_index,
                          const TagObservation& observation,
                          std::uint32_t now) {
  if (!observation.valid || observation.same_id_streak < 2) return;
  CameraState& state = camera_states[camera_index];
  const std::array<int, 9> geometry{
      observation.id,
      observation.corners[0], observation.corners[1],
      observation.corners[2], observation.corners[3],
      observation.corners[4], observation.corners[5],
      observation.corners[6], observation.corners[7]};
  if ((state.have_corrected_geometry && geometry == state.corrected_geometry) ||
      now - state.last_correction_ms < kTagCorrectionCooldownMs) {
    return;
  }
  const TagCandidate candidate =
      best_candidate(kCameras[camera_index], observation);
  if (!candidate.valid ||
      candidate.position_innovation_in > kMaximumNormalTagInnovationIn) {
    return;
  }
  double dx = candidate.x_in - fused_pose.x_in;
  double dy = candidate.y_in - fused_pose.y_in;
  const double distance = std::hypot(dx, dy);
  if (distance > 1e-6) {
    const double requested_step = distance * kTagPositionGain;
    const double scale =
        std::min(requested_step, kMaximumTagPositionStepIn) / distance;
    fused_pose.x_in += dx * scale;
    fused_pose.y_in += dy * scale;
  }
  state.corrected_geometry = geometry;
  state.have_corrected_geometry = true;
  state.last_correction_ms = now;
  applied_tag_port = kCameras[camera_index].port;
  applied_tag_id = observation.id;
  applied_goal = candidate.goal;
  last_tag_innovation_in = candidate.position_innovation_in;
  last_tag_correction_ms = now;
}

void update_odometry() {
  const double left_deg =
      0.5 * (left_front.get_position() + left_rear.get_position());
  const double right_deg =
      0.5 * (right_front.get_position() + right_rear.get_position());
  const double side_deg = side_odom.get_position() / 100.0;
  const double imu_deg = imu.get_rotation();
  if (!std::isfinite(imu_deg)) return;

  const double delta_left_deg = left_deg - previous_left_deg;
  const double delta_right_deg = right_deg - previous_right_deg;
  const double delta_side_deg = side_deg - previous_side_deg;
  const double delta_imu_deg = imu_deg - previous_imu_deg;
  previous_left_deg = left_deg;
  previous_right_deg = right_deg;
  previous_side_deg = side_deg;
  previous_imu_deg = imu_deg;

  accumulated_left_deg += delta_left_deg;
  accumulated_right_deg += delta_right_deg;
  accumulated_side_deg += delta_side_deg;
  accumulated_imu_deg += delta_imu_deg;
  ++accumulated_odometry_samples;
  if (accumulated_odometry_samples < 5) return;

  const double differential_deg =
      accumulated_right_deg - accumulated_left_deg;
  const double predicted_yaw_deg =
      kEncoderYawDegPerDifferentialDeg * differential_deg;
  encoder_consistent =
      std::abs(differential_deg) < 5.0 ||
      std::abs(predicted_yaw_deg - accumulated_imu_deg) <=
          std::max(2.0, 0.65 * std::abs(predicted_yaw_deg));
  const double forward_in = encoder_consistent
      ? 0.5 * (accumulated_left_deg + accumulated_right_deg) *
          (kPi * kDriveWheelDiameterIn / 360.0)
      : 0.0;
  const double delta_heading_rad =
      degrees_to_radians(accumulated_imu_deg);
  const double raw_side_in =
      accumulated_side_deg * (kPi * kSideWheelDiameterIn / 360.0);
  double right_in =
      raw_side_in + kSideRotationLeverIn * delta_heading_rad;
  if (std::abs(right_in) > 0.75) right_in = 0.0;

  const double midpoint_heading_rad = degrees_to_radians(
      fused_pose.heading_deg + 0.5 * accumulated_imu_deg);
  fused_pose.x_in +=
      forward_in * std::cos(midpoint_heading_rad) +
      right_in * std::sin(midpoint_heading_rad);
  fused_pose.y_in +=
      forward_in * std::sin(midpoint_heading_rad) -
      right_in * std::cos(midpoint_heading_rad);
  fused_pose.heading_deg =
      wrap_degrees(fused_pose.heading_deg + accumulated_imu_deg);
  accumulated_left_deg = 0.0;
  accumulated_right_deg = 0.0;
  accumulated_side_deg = 0.0;
  accumulated_imu_deg = 0.0;
  accumulated_odometry_samples = 0;
}

void update_lidar() {
  const int range_8_mm = lidar_8.get_distance();
  const int range_9_mm = lidar_9.get_distance();
  const int confidence_8 = lidar_8.get_confidence();
  const int confidence_9 = lidar_9.get_confidence();
  lidar_candidate_valid =
      confidence_8 >= 40 && confidence_9 >= 40 &&
      range_8_mm >= 20 && range_8_mm <= 2000 &&
      range_9_mm >= 20 && range_9_mm <= 2000 &&
      std::abs(range_9_mm - range_8_mm) <= 90;
  lidar_wall_angle_deg = lidar_candidate_valid
      ? radians_to_degrees(
            std::atan2(range_9_mm - range_8_mm, 2.0 * kMmPerInch))
      : 0.0;
}

void emit_telemetry(std::uint32_t now) {
  const Pose pose = snapshot_pose();
  camera_mutex.take();
  const TagObservation camera_19 = camera_states[0].observation;
  const TagObservation camera_20 = camera_states[1].observation;
  camera_mutex.give();
  std::printf(
      "LOC t=%lu x=%.3f y=%.3f h=%.3f imu=%.3f enc_ok=%d "
      "m=%.2f,%.2f,%.2f,%.2f side=%.2f "
      "l8=%ld,%ld l9=%ld,%ld ltheta=%.3f,lvalid=%d "
      "c19=%d,%.3f,%.3f,%d c20=%d,%.3f,%.3f,%d "
      "corr=%d,%d,%s,%.3f\n",
      static_cast<unsigned long>(now),
      pose.x_in, pose.y_in, pose.heading_deg,
      imu.get_rotation(), static_cast<int>(encoder_consistent),
      left_front.get_position(), left_rear.get_position(),
      right_front.get_position(), right_rear.get_position(),
      side_odom.get_position() / 100.0,
      static_cast<long>(lidar_8.get_distance()),
      static_cast<long>(lidar_8.get_confidence()),
      static_cast<long>(lidar_9.get_distance()),
      static_cast<long>(lidar_9.get_confidence()),
      lidar_wall_angle_deg, static_cast<int>(lidar_candidate_valid),
      camera_19.id, camera_19.range_in, camera_19.bearing_deg,
      static_cast<int>(camera_19.valid),
      camera_20.id, camera_20.range_in, camera_20.bearing_deg,
      static_cast<int>(camera_20.valid),
      applied_tag_port, applied_tag_id, applied_goal,
      last_tag_innovation_in);
  std::fflush(stdout);
}

void draw_gps_dashboard(const pros::gps_status_s_t& gps_status,
                        double gps_error_m, bool gps_valid) {
  constexpr std::uint32_t kBackground = 0x00101824;
  constexpr std::uint32_t kPanel = 0x001B2A3A;
  constexpr std::uint32_t kCyan = 0x0039D5FF;
  constexpr std::uint32_t kGreen = 0x0049E391;
  constexpr std::uint32_t kWhite = 0x00F3F7FA;
  constexpr std::uint32_t kMuted = 0x008EA4B8;
  constexpr std::uint32_t kRed = 0x00FF5B68;
  constexpr double kMetersToInches = 39.37007874015748;

  static bool dashboard_initialized = false;
  if (!dashboard_initialized) {
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();
    pros::screen::set_pen(kPanel);
    pros::screen::fill_rect(0, 0, 479, 39);
    pros::screen::set_pen(kWhite);
    pros::screen::print(TEXT_MEDIUM, 34, 10, "GPS 7  LIVE POSITION");
    dashboard_initialized = true;
  }
  pros::screen::set_pen(kBackground);
  pros::screen::fill_rect(0, 40, 479, 271);
  pros::screen::set_pen(gps_valid ? kGreen : kRed);
  pros::screen::fill_circle(20, 20, 6);

  if (!gps_valid) {
    pros::screen::set_pen(kRed);
    pros::screen::print(TEXT_LARGE, 30, 105,
                        "GPS unavailable / calibrating");
    return;
  }

  constexpr int field_x = 18;
  constexpr int field_y = 48;
  constexpr int field_size = 205;
  constexpr double field_half_in = 72.0;
  pros::screen::set_pen(kPanel);
  pros::screen::fill_rect(field_x, field_y, field_x + field_size,
                          field_y + field_size);
  pros::screen::set_pen(kMuted);
  pros::screen::draw_rect(field_x, field_y, field_x + field_size,
                          field_y + field_size);
  for (int division = 1; division < 6; ++division) {
    const int offset = field_size * division / 6;
    pros::screen::draw_line(field_x + offset, field_y,
                            field_x + offset, field_y + field_size);
    pros::screen::draw_line(field_x, field_y + offset,
                            field_x + field_size, field_y + offset);
  }
  pros::screen::set_pen(kWhite);
  pros::screen::print(TEXT_SMALL, field_x + field_size / 2 - 5,
                      field_y - 15, "N");

  const double x_in = gps_status.x * kMetersToInches;
  const double y_in = gps_status.y * kMetersToInches;
  const double normalized_x = std::clamp(x_in / field_half_in, -1.0, 1.0);
  const double normalized_y = std::clamp(y_in / field_half_in, -1.0, 1.0);
  const int robot_x = field_x + field_size / 2 +
      static_cast<int>(normalized_x * (field_size / 2 - 7));
  const int robot_y = field_y + field_size / 2 -
      static_cast<int>(normalized_y * (field_size / 2 - 7));
  const double radians = gps_status.yaw * 3.14159265358979323846 / 180.0;
  const int robot_tip_x = robot_x + static_cast<int>(std::sin(radians) * 20.0);
  const int robot_tip_y = robot_y - static_cast<int>(std::cos(radians) * 20.0);
  pros::screen::set_pen(kCyan);
  pros::screen::draw_line(robot_x, robot_y, robot_tip_x, robot_tip_y);
  pros::screen::fill_circle(robot_tip_x, robot_tip_y, 3);
  pros::screen::set_pen(kGreen);
  pros::screen::fill_circle(robot_x, robot_y, 7);
  pros::screen::set_pen(kWhite);
  pros::screen::fill_circle(robot_x, robot_y, 3);

  constexpr int center_x = 356;
  constexpr int center_y = 123;
  constexpr int radius = 66;
  pros::screen::set_pen(kPanel);
  pros::screen::fill_circle(center_x, center_y, radius + 3);
  pros::screen::set_pen(kMuted);
  pros::screen::draw_circle(center_x, center_y, radius);
  pros::screen::draw_line(center_x, center_y - radius, center_x,
                          center_y - radius + 9);
  pros::screen::draw_line(center_x + radius - 9, center_y,
                          center_x + radius, center_y);
  pros::screen::draw_line(center_x, center_y + radius - 9, center_x,
                          center_y + radius);
  pros::screen::draw_line(center_x - radius, center_y,
                          center_x - radius + 9, center_y);
  pros::screen::print(TEXT_SMALL, center_x - 5, center_y - radius - 18,
                      "N");

  const int tip_x = center_x + static_cast<int>(std::sin(radians) * 51.0);
  const int tip_y = center_y - static_cast<int>(std::cos(radians) * 51.0);
  const int tail_x = center_x - static_cast<int>(std::sin(radians) * 16.0);
  const int tail_y = center_y + static_cast<int>(std::cos(radians) * 16.0);
  pros::screen::set_pen(kCyan);
  pros::screen::draw_line(tail_x, tail_y, tip_x, tip_y);
  pros::screen::fill_circle(tip_x, tip_y, 5);
  pros::screen::set_pen(kWhite);
  pros::screen::fill_circle(center_x, center_y, 4);
  pros::screen::print(TEXT_MEDIUM, 286, 204, "H %6.2f deg", gps_status.yaw);
  pros::screen::set_pen(kMuted);
  pros::screen::print(TEXT_SMALL, 260, 232, "X %.2f  Y %.2f in", x_in, y_in);
  pros::screen::set_pen(kGreen);
  pros::screen::print(TEXT_SMALL, 278, 251, "RMS +/- %.2f in",
                      gps_error_m * kMetersToInches);
}

void localization_loop() {
  while (true) {
    const std::uint32_t now = pros::millis();
    pose_mutex.take();
    update_odometry();
    pose_mutex.give();
    update_lidar();
    if (now - last_vision_ms >= kVisionPeriodMs) {
      last_vision_ms = now;
      camera_mutex.take();
      for (std::size_t index = 0; index < kCameras.size(); ++index) {
        const TagObservation observation = read_camera(index);
        pose_mutex.take();
        apply_tag_correction(index, observation, now);
        pose_mutex.give();
      }
      camera_mutex.give();
    }
    if (now - last_telemetry_ms >= kTelemetryPeriodMs) {
      last_telemetry_ms = now;
      record_pose(now);
      emit_telemetry(now);
      const auto gps_status = gps_7.get_position_and_orientation();
      const double gps_error_m = gps_7.get_error();
      const bool gps_valid =
          gps_7.is_installed() && std::isfinite(gps_status.x) &&
          std::isfinite(gps_status.y) && std::isfinite(gps_status.yaw) &&
          std::isfinite(gps_error_m) && gps_error_m >= 0.0 &&
          gps_error_m < 1.0;
      draw_gps_dashboard(gps_status, gps_error_m, gps_valid);
    }
    pros::delay(kLoopPeriodMs);
  }
}

brainloc::MotionResult go_to_impl(double target_x, double target_y,
                                  brainloc::MotionOptions options) {
  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_progress_ms = started_ms;
  double best_distance = INFINITY;
  double best_heading_error = INFINITY;
  bool drive_reversed = false;
  int settled_samples = 0;
  while (true) {
    if (cancel_requested.load()) {
      stop_drive();
      return brainloc::MotionResult::cancelled;
    }
    if (!localization_started.load() || !std::isfinite(imu.get_rotation())) {
      stop_drive();
      return brainloc::MotionResult::localization_unhealthy;
    }
    if (pros::millis() - started_ms > options.timeout_ms) {
      stop_drive();
      return brainloc::MotionResult::timed_out;
    }

    const Pose pose = snapshot_pose();
    const double dx = target_x - pose.x_in;
    const double dy = target_y - pose.y_in;
    const double distance = std::hypot(dx, dy);
    if (distance <= best_distance - 0.10) {
      best_distance = distance;
      last_progress_ms = pros::millis();
    }
    if (distance <= options.position_tolerance_in) {
      stop_drive();
      if (++settled_samples >= 8) {
        return brainloc::MotionResult::reached;
      }
      pros::delay(20);
      continue;
    }
    settled_samples = 0;

    double target_heading =
        radians_to_degrees(std::atan2(dy, dx));
    double heading_error =
        wrap_degrees(target_heading - pose.heading_deg);
    // Keep the selected direction through the ambiguous sideways region.
    // Without hysteresis, sensor noise around 90 degrees can alternate the
    // forward/reverse choice every update and reverse the turn command.
    if (drive_reversed && std::abs(heading_error) < 70.0) {
      drive_reversed = false;
      best_heading_error = INFINITY;
    } else if (!drive_reversed && std::abs(heading_error) > 100.0) {
      drive_reversed = true;
      best_heading_error = INFINITY;
    }
    if (drive_reversed) {
      target_heading = wrap_degrees(target_heading + 180.0);
      heading_error = wrap_degrees(target_heading - pose.heading_deg);
    }
    if (std::abs(heading_error) <= best_heading_error - 1.0) {
      best_heading_error = std::abs(heading_error);
      last_progress_ms = pros::millis();
    }
    double forward = 0.0;
    if (std::abs(heading_error) <= 25.0) {
      const double minimum_drive =
          std::min(35.0, static_cast<double>(options.maximum_drive));
      forward = std::clamp(distance * 4.0, minimum_drive,
                           static_cast<double>(options.maximum_drive));
      if (drive_reversed) forward = -forward;
    }
    double turn = std::clamp(
        heading_error * 0.75,
        -static_cast<double>(options.maximum_turn),
        static_cast<double>(options.maximum_turn));
    // Static-friction compensation is only for an in-place turn. Applying a
    // 22-power floor while translating creates a discontinuity near the
    // heading tolerance and can make the robot weave past a waypoint.
    if (std::abs(forward) < 1.0 &&
        std::abs(heading_error) > options.heading_tolerance_deg &&
        std::abs(turn) <
            std::min(30.0, static_cast<double>(options.maximum_turn))) {
      turn = std::copysign(
          std::min(30.0, static_cast<double>(options.maximum_turn)), turn);
    }

    const int left = static_cast<int>(std::lround(std::clamp(
        forward - turn, -127.0, 127.0)));
    const int right = static_cast<int>(std::lround(std::clamp(
        forward + turn, -127.0, 127.0)));
    if ((std::abs(left) >= 12 || std::abs(right) >= 12) &&
        pros::millis() - last_progress_ms >= 800) {
      stop_drive();
      std::printf(
          "NAV_STALL target=%.3f,%.3f pose=%.3f,%.3f,%.3f "
          "command=%d,%d current=%ld,%ld,%ld,%ld\n",
          target_x, target_y, pose.x_in, pose.y_in, pose.heading_deg,
          left, right,
          static_cast<long>(left_front.get_current_draw()),
          static_cast<long>(left_rear.get_current_draw()),
          static_cast<long>(right_front.get_current_draw()),
          static_cast<long>(right_rear.get_current_draw()));
      std::fflush(stdout);
      return brainloc::MotionResult::stalled;
    }
    drive_tank(left, right);
    std::printf(
        "NAV target=%.3f,%.3f pose=%.3f,%.3f,%.3f "
        "distance=%.3f error=%.3f command=%d,%d\n",
        target_x, target_y, pose.x_in, pose.y_in, pose.heading_deg,
        distance, heading_error, left, right);
    std::fflush(stdout);
    pros::delay(20);
  }
}

brainloc::MotionResult turn_to_impl(double target_heading,
                                    brainloc::MotionOptions options) {
  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_progress_ms = started_ms;
  double previous_imu = imu.get_rotation();
  int settled_samples = 0;
  while (true) {
    if (cancel_requested.load()) {
      stop_drive();
      return brainloc::MotionResult::cancelled;
    }
    if (!localization_started.load() || !std::isfinite(imu.get_rotation())) {
      stop_drive();
      return brainloc::MotionResult::localization_unhealthy;
    }
    if (pros::millis() - started_ms > options.timeout_ms) {
      stop_drive();
      return brainloc::MotionResult::timed_out;
    }
    const Pose pose = snapshot_pose();
    const double error = wrap_degrees(target_heading - pose.heading_deg);
    if (std::abs(error) <= options.heading_tolerance_deg) {
      stop_drive();
      if (++settled_samples >= 8) {
        return brainloc::MotionResult::reached;
      }
      pros::delay(20);
      continue;
    }
    settled_samples = 0;
    double turn = std::clamp(
        error * 0.75,
        -static_cast<double>(options.maximum_turn),
        static_cast<double>(options.maximum_turn));
    const double minimum_turn =
        std::min(30.0, static_cast<double>(options.maximum_turn));
    if (std::abs(turn) < minimum_turn) {
      turn = std::copysign(minimum_turn, turn);
    }
    const int command = static_cast<int>(std::lround(turn));
    const double current_imu = imu.get_rotation();
    if (std::abs(current_imu - previous_imu) >= 0.08) {
      last_progress_ms = pros::millis();
    } else if (pros::millis() - last_progress_ms >= 600) {
      stop_drive();
      std::printf(
          "NAV_STALL_TURN target=%.3f pose=%.3f command=%d,%d\n",
          target_heading, pose.heading_deg, -command, command);
      std::fflush(stdout);
      return brainloc::MotionResult::stalled;
    }
    previous_imu = current_imu;
    drive_tank(-command, command);
    std::printf(
        "NAV_TURN target=%.3f pose=%.3f error=%.3f command=%d,%d\n",
        target_heading, pose.heading_deg, error, -command, command);
    std::fflush(stdout);
    pros::delay(20);
  }
}

}  // namespace

namespace brainloc {

void init(Pose start) {
  if (initialization_started.exchange(true)) {
    pose_mutex.take();
    fused_pose = start;
    fused_pose.timestamp_ms = pros::millis();
    history_head = 0;
    history_size = 0;
    pose_mutex.give();
    return;
  }
  pose_mutex.take();
  fused_pose = start;
  fused_pose.timestamp_ms = pros::millis();
  history_head = 0;
  history_size = 0;
  pose_mutex.give();
  stop_drive();
  pros::screen::set_eraser(0x00101824);
  pros::screen::erase();
  left_front.set_brake_mode(pros::MotorBrake::brake);
  left_rear.set_brake_mode(pros::MotorBrake::brake);
  right_front.set_brake_mode(pros::MotorBrake::brake);
  right_rear.set_brake_mode(pros::MotorBrake::brake);
  pros::delay(500);
  for (const CameraConfig& camera : kCameras) {
    configure_camera(camera.port);
  }
  imu.reset(true);
  left_front.tare_position();
  left_rear.tare_position();
  right_front.tare_position();
  right_rear.tare_position();
  side_odom.reset_position();
  previous_left_deg = previous_right_deg = previous_side_deg = 0.0;
  previous_imu_deg = 0.0;
  std::printf("LOC_READY x=%.3f y=%.3f h=%.3f brain_only=1\n",
              start.x_in, start.y_in, start.heading_deg);
  std::fflush(stdout);
  localization_started.store(true);
  pros::Task::create(localization_loop, TASK_PRIORITY_DEFAULT,
                     TASK_STACK_DEPTH_DEFAULT, "localization");
}

Pose get_pose() { return snapshot_pose(); }

std::vector<Pose> get_history() {
  std::vector<Pose> result;
  pose_mutex.take();
  result.reserve(history_size);
  const std::size_t first =
      (history_head + kHistoryCapacity - history_size) % kHistoryCapacity;
  for (std::size_t index = 0; index < history_size; ++index) {
    result.push_back(pose_history[(first + index) % kHistoryCapacity]);
  }
  pose_mutex.give();
  return result;
}

std::vector<Waypoint> get_active_path() {
  path_mutex.take();
  const std::vector<Waypoint> result = active_path;
  path_mutex.give();
  return result;
}

std::vector<TagReading> get_visible_tags() {
  std::vector<TagReading> result;
  camera_mutex.take();
  for (std::size_t index = 0; index < camera_states.size(); ++index) {
    const TagObservation observation = camera_states[index].observation;
    if (!observation.valid) continue;
    result.push_back(TagReading{
        static_cast<int>(kCameras[index].port),
        observation.id,
        observation.range_in,
        observation.bearing_deg});
  }
  camera_mutex.give();
  return result;
}

Status get_status() {
  const Pose pose = snapshot_pose();
  const std::uint32_t now = pros::millis();
  return Status{
      localization_started.load(),
      encoder_consistent,
      lidar_candidate_valid,
      now - pose.timestamp_ms,
      last_tag_correction_ms == 0
          ? UINT32_MAX
          : now - last_tag_correction_ms,
      applied_tag_port,
      applied_tag_id,
      last_tag_innovation_in};
}

bool healthy() {
  const Pose pose = snapshot_pose();
  return localization_started.load() &&
         std::isfinite(pose.x_in) && std::isfinite(pose.y_in) &&
         std::isfinite(pose.heading_deg) &&
         std::isfinite(imu.get_rotation()) &&
         pros::millis() - pose.timestamp_ms <= 300;
}

MotionResult go_to(double x_in, double y_in, MotionOptions options) {
  cancel_requested.store(false);
  motion_running.store(true);
  set_active_path({Waypoint{x_in, y_in, options.position_tolerance_in}});
  const MotionResult result = go_to_impl(x_in, y_in, options);
  stop_drive();
  set_active_path({});
  motion_running.store(false);
  return result;
}

MotionResult drive_straight_to(double x_in, double y_in,
                               MotionOptions options) {
  return go_to(x_in, y_in, options);
}

MotionResult turn_to(double heading_deg, MotionOptions options) {
  cancel_requested.store(false);
  motion_running.store(true);
  set_active_path({});
  const MotionResult result = turn_to_impl(heading_deg, options);
  stop_drive();
  motion_running.store(false);
  return result;
}

MotionResult go_to_pose(Pose target, MotionOptions options) {
  cancel_requested.store(false);
  motion_running.store(true);
  set_active_path(
      {Waypoint{target.x_in, target.y_in, options.position_tolerance_in}});
  MotionResult result = go_to_impl(target.x_in, target.y_in, options);
  if (result == MotionResult::reached) {
    result = turn_to_impl(target.heading_deg, options);
  }
  stop_drive();
  set_active_path({});
  motion_running.store(false);
  return result;
}

MotionResult follow_path(const std::vector<Waypoint>& path,
                         MotionOptions options) {
  cancel_requested.store(false);
  motion_running.store(true);
  set_active_path(path);
  MotionResult result = MotionResult::reached;
  for (const Waypoint& waypoint : path) {
    MotionOptions waypoint_options = options;
    waypoint_options.position_tolerance_in = waypoint.tolerance_in;
    result = go_to_impl(waypoint.x_in, waypoint.y_in, waypoint_options);
    if (result != MotionResult::reached) break;
  }
  stop_drive();
  set_active_path({});
  motion_running.store(false);
  return result;
}

void cancel_motion() {
  cancel_requested.store(true);
  stop_drive();
}

bool motion_active() { return motion_running.load(); }

}  // namespace brainloc
