#pragma once

#include <cstdint>
#include <vector>

namespace brainloc {

struct Pose {
  double x_in;
  double y_in;
  double heading_deg;
  std::uint32_t timestamp_ms = 0;
};

struct Waypoint {
  double x_in;
  double y_in;
  double tolerance_in = 1.5;
};

struct MotionOptions {
  int maximum_drive = 35;
  int maximum_turn = 30;
  double position_tolerance_in = 1.5;
  double heading_tolerance_deg = 4.0;
  std::uint32_t timeout_ms = 12000;
};

struct TagReading {
  int camera_port;
  int id;
  double range_in;
  double bearing_deg;
};

struct Status {
  bool running;
  bool encoder_consistent;
  bool paired_lidar_valid;
  std::uint32_t pose_age_ms;
  std::uint32_t tag_age_ms;
  int last_tag_port;
  int last_tag_id;
  double last_tag_innovation_in;
};

enum class MotionResult {
  reached,
  timed_out,
  stalled,
  cancelled,
  localization_unhealthy,
};

// Starts continuous Brain-only localization at a known field pose.
void init(Pose start);

Pose get_pose();
std::vector<Pose> get_history();
std::vector<Waypoint> get_active_path();
std::vector<TagReading> get_visible_tags();
Status get_status();
bool healthy();

// Blocking motion helpers. They use the continuously fused pose and never
// depend on a laptop, webcam, network connection, or external processor.
MotionResult go_to(double x_in, double y_in,
                   MotionOptions options = {});
MotionResult drive_straight_to(double x_in, double y_in,
                               MotionOptions options = {});
MotionResult turn_to(double heading_deg, MotionOptions options = {});
MotionResult go_to_pose(Pose target, MotionOptions options = {});
MotionResult follow_path(const std::vector<Waypoint>& path,
                         MotionOptions options = {});
void cancel_motion();
bool motion_active();

}  // namespace brainloc
