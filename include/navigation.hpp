#pragma once

#include <cstddef>
#include <cstdint>

// Blocking, field-coordinate navigation for this differential-drive robot.
// Coordinates are inches in the same field frame used by the VEX GPS map;
// headings are mathematical degrees (0 = +X, 90 = +Y, counterclockwise).
namespace navigation {

struct Pose {
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_deg = 0.0;
  double dead_reckoning_distance_in = 0.0;
  double position_error_envelope_in = 0.0;
  std::uint32_t absolute_position_age_ms = 0;
  bool valid = false;
  // Appended after the original fields to preserve aggregate-init ordering.
  std::uint32_t estimator_age_ms = 0;
};

struct PathPoint {
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_deg = 0.0;
  double position_error_envelope_in = 0.0;
  std::uint32_t session_time_ms = 0;
  std::uint32_t absolute_position_age_ms = 0;
};

inline constexpr std::size_t kPathCapacity = 512;

struct SensorHealth {
  bool pose_valid = false;
  bool drive_encoders_valid = false;
  bool imu_valid = false;
  bool gps_installed = false;
  bool gps_fix_accepted = false;
  double gps_reported_error_in = 0.0;
  // Raw diagnostic from the inertial element inside the GPS sensor. It is not
  // fused until its sign, units, and mounting behavior pass a live sweep.
  bool gps_gyro_valid = false;
  double gps_gyro_z = 0.0;
  const char* gps_state = "not_initialized";
  bool forward_distance_installed = false;
  bool forward_distance_api_ok = false;
  bool forward_distance_valid = false;
  double forward_distance_in = 0.0;
  long forward_distance_confidence = 0;
  bool ai_vision_installed = false;
  bool ai_tag_visible = false;
  int ai_tag_id = -1;
  double ai_horizontal_range_in = 0.0;
  double ai_3d_range_in = 0.0;
  double ai_bearing_right_deg = 0.0;
  double ai_elevation_deg = 0.0;
  std::uint32_t ai_geometry_age_ms = 0;
  const char* ai_state = "not_initialized";
  bool lateral_tracker_enabled = false;
  double dead_reckoning_distance_in = 0.0;
  double position_error_envelope_in = 0.0;
  std::uint32_t absolute_position_age_ms = 0;
  std::uint32_t estimator_age_ms = 0;
};

enum class Result {
  kSuccess,
  kInvalidArgument,
  kPoseUnavailable,
  kUnsafePath,
  kTurnFailed,
  kDriveFailed,
};

// Establishes the known robot-center starting pose and resets all estimator
// baselines. The entered pose is an absolute anchor; GPS can only make bounded,
// innovation-checked corrections from it. The fourth argument is a conservative
// radial bound on physical start-placement error. The three-coordinate overload
// uses a provisional 1-inch bound rather than assuming perfect placement.
bool init(double start_x_in, double start_y_in, double start_heading_deg);
bool init(double start_x_in,
          double start_y_in,
          double start_heading_deg,
          double start_position_error_in);

// Call about every 20 ms when navigation is idle. Blocking turn/go-to calls
// refresh once before preflight and then update internally while they own the
// drivetrain. Public pose validity expires after 250 ms without an update.
void update();

// These read APIs are safe to call from a separate dashboard/logger task. They
// consume the last complete 20-ms estimator snapshot and never expose a
// partially updated pose. Pose and health expose estimator_age_ms; valid is
// false once the published estimator is older than 250 ms. Motion/init/update
// calls must still have one owner.
Pose current_pose();
SensorHealth sensor_health();

// Copies the fused trail oldest-first into caller-owned storage and returns
// the number written. The fixed internal ring retains at most 512 points and
// requires no heap allocation. A point is recorded at meaningful pose change,
// at least 100 ms apart, plus a 1-second stationary heartbeat.
std::size_t copy_path(PathPoint* output, std::size_t capacity);
std::size_t path_size();
void clear_path();

// Turns in place using IMU heading. Positive max_power is clamped to a
// conservative range; zero/negative is invalid and never moves. Returns only
// after settling or a stall/timeout. A meaningful in-place turn is rejected
// with kUnsafePath unless the robot center satisfies the provisional wall and
// mapped-Goal clearances, inflated by the official one-inch Field Element
// tolerance and current reported pose-error envelope, because P1 cannot
// protect the sides or rear.
Result turn_to(double heading_deg,
               int max_power = 35,
               std::uint32_t timeout_ms = 6000);

// Turns onto the start-to-target bearing, then follows that fixed straight
// path. It stops at the finish plane rather than orbiting a noisy endpoint.
// The forward Distance sensor aborts when an obstacle is within 8 inches.
// The endpoint must remain inside the provisional wall envelope, and the
// straight segment must clear mapped Goal centers. Both exclusions grow by
// the official one-inch Field Element tolerance and current reported pose
// error. Its initial in-place turn
// must also have provisional wall/Goal sweep clearance unless no meaningful
// turn is required. These are conservative center-point checks pending a
// measured robot footprint.
// max_power must be positive; zero/negative is invalid and never moves.
// timeout_ms is one deadline shared by the initial turn and straight leg.
Result go_straight_to(double target_x_in,
                      double target_y_in,
                      int max_power = 40,
                      std::uint32_t timeout_ms = 12000);

// Drives a signed distance along the robot's current heading without first
// turning: positive is forward and negative is reverse. It uses the same fused
// estimator, map/corridor bounds, deadline, and stall checks as go_straight_to.
// P1 protects positive travel only because the sensor faces forward; reverse
// travel has static field-map protection but no rear dynamic-obstacle sensor.
Result drive_relative(double distance_in,
                      int max_power = 40,
                      std::uint32_t timeout_ms = 12000);

// Follows one continuous curved path to a field-coordinate pose, then uses the
// tuned fused turn controller for any residual final-heading error. Forward
// and side targets do not pre-turn; a target more than 100 degrees behind uses
// a bounded map-checked pre-turn so the forward-only curve cannot loop.
// The public map check reserves a six-inch curved-path corridor around the
// centerline in addition to the projected localization-error envelope, and
// the live controller brakes with kDriveFailed if fused cross-track leaves
// that corridor.
// timeout_ms is one deadline shared by the curved drive and heading settle.
Result go_to_pose(double target_x_in,
                  double target_y_in,
                  double target_heading_deg,
                  int max_power = 60,
                  std::uint32_t timeout_ms = 12000);

// Thread-safe cancellation: latches a request that the active blocking public
// turn/drive observes within its next control iteration, and brakes now.
void stop();
const char* result_name(Result result);

}  // namespace navigation
