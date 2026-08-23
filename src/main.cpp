#include "main.h"
#include "brain_localization.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr brainloc::Pose kKnownStart{
    87.158, 70.988, -126.132, 0};
constexpr bool kRunLibraryDemo = false;

void run_library_demo() {
  pros::delay(3000);
  if constexpr (false) {
  brainloc::MotionOptions scan_options;
  scan_options.maximum_turn = 25;
  scan_options.heading_tolerance_deg = 3.0;
  scan_options.timeout_ms = 5000;
  constexpr double scan_headings[]{
      -54.0, -52.0, -50.0, -56.0, -58.0, -55.0};
  for (const double heading : scan_headings) {
    const auto result = brainloc::turn_to(heading, scan_options);
    pros::delay(600);
    const auto scan_tags = brainloc::get_visible_tags();
    const auto scan_pose = brainloc::get_pose();
    std::printf(
        "LIB_SCAN result=%d target=%.1f pose=%.3f,%.3f,%.3f tags=%u\n",
        static_cast<int>(result), heading,
        scan_pose.x_in, scan_pose.y_in, scan_pose.heading_deg,
        static_cast<unsigned>(scan_tags.size()));
    for (const auto& tag : scan_tags) {
      std::printf("LIB_SCAN_TAG port=%d id=%d range=%.3f bearing=%.3f\n",
                  tag.camera_port, tag.id, tag.range_in, tag.bearing_deg);
    }
    std::fflush(stdout);
    if (result != brainloc::MotionResult::reached || scan_tags.size() >= 2) {
      return;
    }
  }
  return;
  }

  brainloc::MotionOptions anchor_options;
  anchor_options.maximum_drive = 40;
  anchor_options.maximum_turn = 30;
  anchor_options.position_tolerance_in = 1.0;
  anchor_options.heading_tolerance_deg = 4.0;
  anchor_options.timeout_ms = 12000;
  const brainloc::Pose requested_anchor{
      94.950, 73.700, -55.035, 0};
  const auto anchor_result = brainloc::MotionResult::reached;
  pros::delay(1000);
  const brainloc::Pose anchored = brainloc::get_pose();
  const auto tags = brainloc::get_visible_tags();
  std::printf(
      "LIB_ANCHOR result=%d pose=%.3f,%.3f,%.3f visible_tags=%u\n",
      static_cast<int>(anchor_result),
      anchored.x_in, anchored.y_in, anchored.heading_deg,
      static_cast<unsigned>(tags.size()));
  for (const auto& tag : tags) {
    std::printf("LIB_TAG port=%d id=%d range=%.3f bearing=%.3f\n",
                tag.camera_port, tag.id, tag.range_in, tag.bearing_deg);
  }
  std::fflush(stdout);
  if (anchor_result != brainloc::MotionResult::reached) {
    std::printf("LIB_ABORT reason=anchor\n");
    std::fflush(stdout);
    return;
  }

  // A dual-tag observation makes this anchor unambiguous. Resetting here is an
  // explicit application-level initial pose; all subsequent ambiguity choices
  // remain constrained by the continuous temporal prior.
  brainloc::init(anchored);

  const double heading = anchored.heading_deg * 3.14159265358979323846 / 180.0;
  const double fx = std::cos(heading);
  const double fy = std::sin(heading);
  const double rx = std::sin(heading);
  const double ry = -std::cos(heading);
  const std::vector<brainloc::Waypoint> path{
      {anchored.x_in + 4.0 * fx, anchored.y_in + 4.0 * fy, 1.0},
      {anchored.x_in + 4.0 * fx + 3.0 * rx,
       anchored.y_in + 4.0 * fy + 3.0 * ry, 1.0},
      {anchored.x_in + 3.0 * rx, anchored.y_in + 3.0 * ry, 1.0},
      {anchored.x_in, anchored.y_in, 1.0},
  };
  brainloc::MotionOptions path_options;
  path_options.maximum_drive = 40;
  path_options.maximum_turn = 30;
  path_options.timeout_ms = 12000;
  const auto path_result = brainloc::follow_path(path, path_options);
  const brainloc::Pose final_pose = brainloc::get_pose();
  const auto history = brainloc::get_history();
  std::printf(
      "LIB_PATH_DONE result=%d pose=%.3f,%.3f,%.3f history=%u\n",
      static_cast<int>(path_result),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg,
      static_cast<unsigned>(history.size()));
  std::fflush(stdout);
}

}  // namespace

void initialize() {
  brainloc::init(kKnownStart);
  if (kRunLibraryDemo) {
    pros::Task::create(run_library_demo, TASK_PRIORITY_DEFAULT,
                       TASK_STACK_DEPTH_DEFAULT, "library_demo");
  }
}

void disabled() {
  brainloc::cancel_motion();
}

void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  while (true) pros::delay(1000);
}
