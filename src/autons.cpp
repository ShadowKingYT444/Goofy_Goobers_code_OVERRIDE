#include "main.h"
#include "gps_frame.hpp"
#include "localization_config.hpp"
#include "pid_autotune.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kWheelDiameterIn = localization::kDriveWheelDiameterIn;
constexpr double kWheelCircumferenceIn = kWheelDiameterIn * kPi;
constexpr double kTrackWidthIn = localization::kDriveTrackWidthIn;
constexpr double kLeftEncoderSign = 1.0;
constexpr double kRightEncoderSign = 1.0;
constexpr double kSideOdomDiameterIn = localization::kSideOdomWheelDiameterIn;
constexpr double kSideOdomCircumferenceIn = kSideOdomDiameterIn * kPi;
constexpr double kSideOdomRawSign = localization::kSideOdomRawToRobotRightSign;
constexpr double kSideOdomOffsetBackIn = localization::kSideOdomRearOffsetIn;
constexpr double kSensorSpacingIn = 2.0;
constexpr std::array<double, 4> kLidarDistanceCalibrationMm = {7.0, 0.0, -5.0, -2.0};
constexpr double kLidarForwardOffsetIn = 0.0;
constexpr double kLidarLeftOffsetIn = 5.29;
constexpr double kMaxLidarDistanceIn = 50.0;
constexpr long kConfidenceMeaningfulDistanceMm = 200;
constexpr long kMinMeaningfulLidarConfidence = 20;
constexpr long kMinCorrectionLidarConfidence = 50;
constexpr double kMaxLidarRmseIn = 0.16;
constexpr double kMaxLidarPointErrorIn = 0.55;
constexpr std::uint32_t kLidarImuCorrectionPeriodMs = 100;
constexpr double kMinWallDistanceWinnerMarginIn = 5.0;
constexpr double kMaxLidarAngularRateDegS = 80.0;
// The start pose is exact and the IMU is calibrated while stationary. A line
// implying a larger jump is far more likely to be a field object blocking the
// LiDAR bar than a real wall. Keep the absolute correction inside the user's
// requested 10 degree orientation tolerance.
constexpr double kMaxWallHeadingErrorDeg = 8.0;
constexpr double kMaxImuBiasDeg = 90.0;
constexpr int kRequiredConsistentLidarFits = 3;
constexpr double kMaxLidarCandidateChangeDeg = 4.0;
constexpr double kLidarBiasGain = 0.35;
constexpr double kMaxLidarBiasStepDeg = 3.0;
constexpr double kLidarFineBiasGain = 0.15;
constexpr double kMaxLidarFineBiasStepDeg = 0.75;
constexpr double kLidarFineCorrectionRangeDeg = 10.0;
constexpr double kLidarCandidateFilterGain = 0.35;
constexpr double kLidarHeadingScoreInPerDeg = 0.35;
constexpr double kMinLidarCandidateScoreMargin = 8.0;
// The round begins from an exact entered pose. A larger single-wall position
// residual after a pure turn is more likely a field object or transform error
// than real translation; defer it until independent evidence exists.
constexpr double kMaxLidarAxisInnovationIn = 1.5;
constexpr double kMaxLidarAxisCandidateChangeIn = 1.25;
constexpr double kLidarAxisFilterGain = 0.35;
constexpr double kLidarAxisGain = 0.25;
constexpr double kMaxLidarAxisStepIn = 2.0;
constexpr double kLidarFineAxisGain = 0.20;
constexpr double kMaxLidarFineAxisStepIn = 0.35;
constexpr double kLidarFineAxisRangeIn = 5.0;
constexpr double kLidarAxisDeadbandIn = 0.10;
constexpr double kLidarHeadingDeadbandDeg = 0.35;
// Live 20-foot coupled-drive testing peaked at 9 degrees (about 0.22 inch at
// the wheel). Reject the side well before the old 45-degree allowance can
// turn a slipping/disconnected geared motor into false odometry.
constexpr double kMaxSameSideMotorSpreadDeg = 15.0;
// A single dynamic sample reached 16 degrees on 2026-08-25 and recovered to
// one degree 35 ms later. Debounce only the spread fault; a missing/non-finite
// encoder still invalidates the pose on its first sample.
constexpr int kDriveSpreadFaultConsecutiveSamples = 3;
constexpr double kMaxSideOdomSpeedInS = 100.0;
constexpr double kSideOdomJumpAllowanceIn = 0.5;
constexpr double kFusionTestDriveToleranceIn = 0.45;
constexpr double kFusionTestTurnToleranceDeg = 1.8;
constexpr std::uint32_t kFusionTestSettleMs = 40;
constexpr std::uint32_t kFusionTestLogPeriodMs = 250;
constexpr std::uint32_t kFusionTestChainSampleMs = 40;
constexpr int kFusionTestMinForwardPower = 36;
constexpr int kFusionTestMaxHeadingCorrection = 18;
constexpr double kFusionTestTurnKp = 0.95;
constexpr double kFusionTestTurnKd = 0.08;
constexpr double kFusionTestTurnMinPower = 12.0;
constexpr double kFusionTestTurnMinPowerErrorDeg = 4.0;
constexpr double kFusionTestTurnSlewPowerPerSec = 360.0;
constexpr double kFusedDriveToleranceIn = 0.35;
constexpr double kFusedDriveSettleMs = 80;
constexpr double kFusedDriveKp = 10.5;
// Live 2026-08-25 breakaway: 18/127 moved 0.096 in in 300 ms; 24/127
// moved 0.430 in with all four motors tracking. Keep a modest margin for
// battery/carpet variation while relying on finish-window braking near target.
constexpr double kFusedDriveMinPower = 34.0;
constexpr double kFusedDriveHeadingKp = 1.15;
constexpr double kFusedDriveFinalHeadingKp = 1.0;
constexpr double kFusedDriveMaxTurnPower = 45.0;
constexpr double kFusedDriveCrossTrackLookaheadIn = 10.0;
constexpr double kFusedDriveMaxCrossTrackHeadingDeg = 15.0;
constexpr double kFusedDriveMaxFinishCrossTrackIn = 2.0;
constexpr double kFusedDriveForwardSlewPowerPerSec = 420.0;
constexpr double kFusedDriveTurnSlewPowerPerSec = 420.0;
constexpr double kFusedDriveStallProgressIn = 0.10;
constexpr std::uint32_t kFusedDriveStallTimeoutMs = 1000;
constexpr double kFusedTurnToleranceDeg = 1.5;
constexpr double kFusedTurnSettleMs = 120;
constexpr double kFusedTurnSettleRateDegPerSec = 14.0;
// Paired +/-45-degree live sweeps on this chassis selected this lightly
// damped response. The previous 0.22 derivative term spent most of each turn
// braking and was visibly slow even with a high requested power limit.
constexpr double kFusedTurnKp = 1.8;
constexpr double kFusedTurnKd = 0.058514;
// Static power is only applied when the chassis is nearly stationary. Applying
// a hard floor while it is already approaching the target prevents braking and
// was the source of the slow cross-target oscillation seen in every turn.
constexpr double kFusedTurnMinPower = 20.0;
constexpr double kFusedTurnBreakawayPower = 28.0;
constexpr std::uint32_t kFusedTurnBreakawayDelayMs = 250;
constexpr double kFusedTurnStaticRateDegPerSec = 6.0;
constexpr double kFusedTurnBrakeZoneDeg = 12.0;
constexpr double kFusedTurnBrakeZoneMaxPower = 55.0;
constexpr double kFusedTurnSlewPowerPerSec = 800.0;
constexpr std::uint32_t kFusedTurnStallTimeoutMs = 1000;
constexpr std::uint32_t kNavigationInitSettleMs = 250;
constexpr double kNavigationInitMaxWheelMotionIn = 0.10;
constexpr double kNavigationInitMaxImuMotionDeg = 1.0;
constexpr double kNavigationGoToPosePreturnThresholdDeg = 100.0;

enum class LidarFusionMode {
  kDisabled,
  kBiasOnly,
};

struct PoseEstimate {
  double x = localization::kEnteredStartPose.x_in;
  double y = localization::kEnteredStartPose.y_in;
  double heading_rad = localization::kEnteredStartPose.heading_deg * kPi / 180.0;
  double imu_heading_deg = localization::kEnteredStartPose.heading_deg;
  double imu_zero_field_heading_deg = localization::kEnteredStartPose.heading_deg;
  double imu_bias_deg = 0.0;
  std::uint32_t last_update_ms = 0;
  std::uint32_t last_lidar_bias_ms = 0;
  std::uint32_t last_lidar_sample_ms = 0;
  double left_deg = 0.0;
  double right_deg = 0.0;
  std::int32_t side_centideg = 0;
  bool ready = false;
  bool imu_ready = false;
  bool lidar_used = false;
  bool side_odom_ready = false;
  int left_motor_count = 0;
  int right_motor_count = 0;
  double left_motor_spread_deg = 0.0;
  double right_motor_spread_deg = 0.0;
  int drive_spread_fault_samples = 0;
  double pending_lidar_heading_deg = 0.0;
  double pending_lidar_axis_value_in = 0.0;
  const char* pending_lidar_wall = "none";
  int consistent_lidar_fits = 0;
  double lidar_axis_correction_in = 0.0;
  double lidar_theta_deg = NAN;
  double lidar_distance_in = NAN;
  double lidar_rmse_in = NAN;
  const char* lidar_axis = "none";
  const char* lidar_wall = "none";
  const char* lidar_reject = "none";
  const char* side_odom_reject = "none";
  int ai_tag_id = -1;
  double ai_bearing_right_deg = NAN;
  double ai_candidate_residual_deg = NAN;
  double ai_candidate_margin_deg = NAN;
  double ai_observed_range_in = NAN;
  double ai_predicted_range_in = NAN;
  double ai_range_residual_in = NAN;
  double ai_position_innovation_in = NAN;
  double ai_position_step_in = 0.0;
  double ai_heading_step_deg = 0.0;
  std::uint32_t ai_age_ms = 0;
  std::uint32_t last_ai_poll_id = 0;
  std::uint32_t last_ai_correction_ms = 0;
  double total_drive_distance_in = 0.0;
  double last_ai_geometry_drive_distance_in = 0.0;
  double last_ai_geometry_heading_deg = 0.0;
  bool have_ai_geometry_motion_anchor = false;
  int consistent_ai_observations = 0;
  const char* last_ai_goal = "none";
  const char* last_ai_face = "none";
  const char* ai_goal = "none";
  const char* ai_face = "none";
  const char* ai_reject = "not_initialized";
  bool gps_anchored = false;
  bool gps_frame_aligned = false;
  double gps_frame_rotation_deg = 0.0;
  double gps_frame_translation_x_in = 0.0;
  double gps_frame_translation_y_in = 0.0;
  int consistent_gps_observations = 0;
  std::uint32_t last_gps_sample_ms = 0;
  double pending_gps_x_in = NAN;
  double pending_gps_y_in = NAN;
  double pending_gps_heading_deg = NAN;
  double last_gps_raw_x_in = NAN;
  double last_gps_raw_y_in = NAN;
  double last_gps_raw_heading_deg = NAN;
  double last_gps_geometry_drive_distance_in = 0.0;
  double last_gps_geometry_heading_deg = 0.0;
  bool have_gps_geometry_motion_anchor = false;
  double gps_x_in = NAN;
  double gps_y_in = NAN;
  double gps_heading_deg = NAN;
  double gps_error_in = NAN;
  double gps_position_innovation_in = NAN;
  double gps_position_step_in = 0.0;
  double gps_heading_step_deg = 0.0;
  const char* gps_reject = "not_initialized";
  double dead_reckoning_distance_in = 0.0;
  double absolute_position_base_error_in = 0.0;
  double position_error_envelope_in = 0.0;
  std::uint32_t last_absolute_position_ms = 0;
};

PoseEstimate telemetry_pose;
bool telemetry_pose_initialized = false;
// Telemetry is allowed to initialize from the dashboard's entered pose, but
// the public navigation API must not treat that as permission to move. Only a
// successful navigation::init() establishes this separate motion authority.
bool navigation_api_initialized = false;
std::atomic_bool navigation_stop_requested{false};
// Internal qualification hook. It is unreachable unless its dedicated
// startup harness is explicitly compiled in for one supervised boot.
bool navigation_test_inject_imu_dropout = false;
bool navigation_test_imu_dropout_latched = false;
double navigation_test_imu_dropout_after_in = 0.0;
// Readers such as a dashboard/logger task must never observe a PoseEstimate
// while the 20-ms estimator is partway through mutating it. The estimator
// publishes a complete trivially-copyable snapshot after each update; sensor
// and motor I/O always remains outside this short critical section.
PoseEstimate telemetry_pose_snapshot;
bool telemetry_snapshot_initialized = false;
bool telemetry_snapshot_navigation_initialized = false;
pros::Mutex telemetry_snapshot_mutex;
// Linearizes public physical motor writes with navigation::stop(). Without
// this, stop could brake after the atomic check but before motor.move(), and a
// stale control write could re-energize the drivetrain afterward.
pros::Mutex drive_output_mutex;
// A fault stop must survive wrapper cleanup that also calls the normal stop
// helper. The latch is protected by drive_output_mutex and is cleared only by
// a subsequent authorized motor command or program restart.
bool drive_emergency_hold_latched = false;
std::uint32_t telemetry_last_log_ms = 0;
localization::FieldPose runtime_start_pose = localization::kEnteredStartPose;
std::array<navigation::PathPoint, navigation::kPathCapacity> navigation_path{};
std::size_t navigation_path_start = 0;
std::size_t navigation_path_count = 0;
std::uint32_t navigation_path_session_start_ms = 0;
std::uint32_t navigation_path_last_record_ms = 0;
pros::Mutex navigation_path_mutex;

void publish_telemetry_snapshot() {
  std::lock_guard<pros::Mutex> lock(telemetry_snapshot_mutex);
  telemetry_pose_snapshot = telemetry_pose;
  telemetry_snapshot_initialized = telemetry_pose_initialized;
  telemetry_snapshot_navigation_initialized = navigation_api_initialized;
}

void clear_telemetry_snapshot() {
  std::lock_guard<pros::Mutex> lock(telemetry_snapshot_mutex);
  telemetry_pose_snapshot = PoseEstimate{};
  telemetry_snapshot_initialized = false;
  telemetry_snapshot_navigation_initialized = false;
}

void publish_if_telemetry_pose(PoseEstimate& pose) {
  if (&pose == &telemetry_pose) publish_telemetry_snapshot();
}

bool copy_telemetry_snapshot(PoseEstimate& pose,
                             bool& navigation_initialized) {
  std::lock_guard<pros::Mutex> lock(telemetry_snapshot_mutex);
  if (!telemetry_snapshot_initialized) {
    navigation_initialized = false;
    return false;
  }
  pose = telemetry_pose_snapshot;
  navigation_initialized = telemetry_snapshot_navigation_initialized;
  return true;
}

struct LidarFit {
  bool ok = false;
  double theta_deg = 0.0;
  double wall_distance_in = 0.0;
  double rmse_in = 0.0;
  double max_error_in = 0.0;
  long min_confidence = 0;
};

struct Waypoint {
  double x;
  double y;
};

struct WallCandidate {
  const char* wall;
  double heading_deg;
  bool corrects_x;
  double axis_value_in;
  double heading_error_deg;
  double axis_error_in;
  double score;
};

struct AiLandmarkCandidate {
  const char* goal = "none";
  const char* face = "none";
  double predicted_bearing_right_deg = NAN;
  double residual_deg = INFINITY;
  double predicted_range_in = NAN;
  double range_residual_in = INFINITY;
  double association_score = INFINITY;
  double face_x = NAN;
  double face_y = NAN;
};

struct TestDriveBaseline {
  double left_deg = 0.0;
  double right_deg = 0.0;
};

struct MotorSideReading {
  double position_deg = NAN;
  double spread_deg = 0.0;
  int valid_count = 0;
  bool trustworthy = false;
};

struct GpsObservation {
  bool valid = false;
  double x_in = NAN;
  double y_in = NAN;
  double heading_deg = NAN;
  double error_in = NAN;
};

struct ForwardObstacleObservation {
  bool installed = false;
  bool api_ok = false;
  bool valid = false;
  long distance_mm = 9999;
  long confidence = 0;
};

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double normalize_deg(double angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

double signed_angle_diff_deg(double target, double current);
double deg_to_rad(double deg);
double rad_to_deg(double rad);
double heading_rad_from_deg(double deg);
void stop_drive_motors();
void stop_drive_motors_unlocked();
void emergency_stop_drive_motors();
double pose_heading_deg(const PoseEstimate& pose);

void update_ai_vision_shadow(PoseEstimate& pose, std::uint32_t now) {
  const auto& observation = ai_vision_shadow_snapshot();
  pose.ai_tag_id = observation.tag_id;
  pose.ai_bearing_right_deg = observation.bearing_deg;
  pose.ai_goal = "none";
  pose.ai_face = "none";
  pose.ai_candidate_residual_deg = NAN;
  pose.ai_candidate_margin_deg = NAN;
  // Field-map geometry is horizontal. Keep full 3D lens range in raw vision
  // telemetry, but never compare it with a 2D Goal distance.
  pose.ai_observed_range_in = observation.horizontal_range_in;
  pose.ai_predicted_range_in = NAN;
  pose.ai_range_residual_in = NAN;
  pose.ai_position_innovation_in = NAN;
  pose.ai_position_step_in = 0.0;
  pose.ai_heading_step_deg = 0.0;
  pose.ai_age_ms = now >= observation.brain_ms ? now - observation.brain_ms : 0;
  pose.ai_reject = observation.reason;

  // update_pose() runs faster than the P6 polling task. A single immutable
  // shadow snapshot must never count as several temporally consistent camera
  // observations merely because the estimator read it several times.
  if (observation.poll_id == 0) {
    pose.ai_reject = "no_poll";
    return;
  }
  if (observation.poll_id == pose.last_ai_poll_id) {
    // Preserve the last evaluated camera result while refusing to count this
    // immutable snapshot again.
    return;
  }
  pose.last_ai_poll_id = observation.poll_id;

  if (!observation.installed || !observation.configured ||
      !observation.tag_valid) {
    pose.consistent_ai_observations = 0;
    return;
  }
  if (pose.ai_age_ms > localization::kAiVisionPollPeriodMs * 3) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "stale";
    return;
  }

  const double current_heading_deg =
      normalize_deg(rad_to_deg(pose.heading_rad));
  if (!observation.repeated_geometry) {
    pose.last_ai_geometry_drive_distance_in = pose.total_drive_distance_in;
    pose.last_ai_geometry_heading_deg = current_heading_deg;
    pose.have_ai_geometry_motion_anchor = true;
  } else if (pose.have_ai_geometry_motion_anchor) {
    const double drive_since_geometry_in =
        pose.total_drive_distance_in -
        pose.last_ai_geometry_drive_distance_in;
    const double heading_since_geometry_deg = std::fabs(
        signed_angle_diff_deg(current_heading_deg,
                              pose.last_ai_geometry_heading_deg));
    if (drive_since_geometry_in >
            localization::kAiMaxRepeatedGeometryDriveIn ||
        heading_since_geometry_deg >
            localization::kAiMaxRepeatedGeometryHeadingDeg) {
      pose.consistent_ai_observations = 0;
      pose.ai_reject = "stale_geometry";
      return;
    }
    // poll_id identifies our Smart Port read, not a new optical exposure.
    // An unchanged quantized corner tuple may be a legitimate new frame or a
    // cache, so it can preserve a pending sequence but must not add temporal
    // evidence or apply another correction.
    pose.ai_reject = "repeat";
    return;
  } else {
    pose.ai_reject = "repeat";
    return;
  }

  const double robot_heading_deg = normalize_deg(rad_to_deg(pose.heading_rad));
  const double robot_heading_rad = deg_to_rad(robot_heading_deg);
  const double forward_x = std::cos(robot_heading_rad);
  const double forward_y = std::sin(robot_heading_rad);
  const double right_x = std::sin(robot_heading_rad);
  const double right_y = -std::cos(robot_heading_rad);
  const double camera_x =
      pose.x + localization::kAiCameraForwardOffsetIn * forward_x +
      localization::kAiCameraRightOffsetIn * right_x;
  const double camera_y =
      pose.y + localization::kAiCameraForwardOffsetIn * forward_y +
      localization::kAiCameraRightOffsetIn * right_y;
  const double camera_heading_deg = normalize_deg(
      robot_heading_deg - localization::kAiCameraYawRightDeg);

  constexpr std::array<std::array<double, 2>, 4> kFaceNormals{{
      {{1.0, 0.0}}, {{0.0, 1.0}}, {{-1.0, 0.0}}, {{0.0, -1.0}},
  }};
  constexpr std::array<const char*, 4> kFaceNames{{"+x", "+y", "-x", "-y"}};
  // IDs 1-4 each map to at most two physical Goals and each Goal has four
  // faces, so association has a hard eight-candidate bound. Keep this storage
  // on the stack; allocating a vector on every fresh camera poll adds
  // nondeterministic allocator work to the 50 Hz estimator/control loop.
  std::array<AiLandmarkCandidate, 8> candidates{};
  std::size_t candidate_count = 0;
  for (const auto& landmark : localization::kGoalTagLandmarks) {
    if (landmark.tag_id != observation.tag_id) continue;
    for (std::size_t face_index = 0; face_index < kFaceNormals.size(); ++face_index) {
      const double normal_x = kFaceNormals[face_index][0];
      const double normal_y = kFaceNormals[face_index][1];
      const double face_x =
          landmark.x_in + normal_x * localization::kAiGoalFaceOffsetIn;
      const double face_y =
          landmark.y_in + normal_y * localization::kAiGoalFaceOffsetIn;
      const double camera_from_face_x = camera_x - face_x;
      const double camera_from_face_y = camera_y - face_y;
      if (camera_from_face_x * normal_x + camera_from_face_y * normal_y <= 0.0) {
        continue;
      }
      const double global_bearing_deg = normalize_deg(
          rad_to_deg(std::atan2(face_y - camera_y, face_x - camera_x)));
      // AI Vision image X is left-negative/right-positive in the live sensor
      // frame. signed_angle_diff already matches that convention here; the
      // previous extra negation mirrored every mapped landmark bearing.
      const double predicted_bearing_right_deg =
          signed_angle_diff_deg(global_bearing_deg, camera_heading_deg);
      const double residual_deg = std::fabs(signed_angle_diff_deg(
          observation.bearing_deg, predicted_bearing_right_deg));
      const double predicted_range_in =
          std::hypot(face_x - camera_x, face_y - camera_y);
      const double range_residual_in =
          std::fabs(observation.horizontal_range_in - predicted_range_in);
      const double association_score = std::hypot(
          residual_deg,
          range_residual_in *
              localization::kAiRangeResidualScoreDegPerIn);
      if (candidate_count >= candidates.size()) {
        pose.consistent_ai_observations = 0;
        pose.ai_reject = "candidate_overflow";
        return;
      }
      candidates[candidate_count++] = AiLandmarkCandidate{
          landmark.name,
          kFaceNames[face_index],
          predicted_bearing_right_deg,
          residual_deg,
          predicted_range_in,
          range_residual_in,
          association_score,
          face_x,
          face_y,
      };
    }
  }

  if (candidate_count == 0) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "no_map_candidate";
    return;
  }
  const auto candidates_end = candidates.begin() + candidate_count;
  std::sort(candidates.begin(), candidates_end,
            [](const auto& a, const auto& b) {
              return a.association_score < b.association_score;
            });
  const auto& best = candidates[0];
  const auto different_goal = std::find_if(
      candidates.begin() + 1, candidates_end,
      [&best](const auto& candidate) {
        return std::strcmp(candidate.goal, best.goal) != 0;
      });
  const double margin = different_goal != candidates_end
                            ? different_goal->association_score -
                                  best.association_score
                            : INFINITY;
  const auto different_face = std::find_if(
      candidates.begin() + 1, candidates_end,
      [&best](const auto& candidate) {
        return std::strcmp(candidate.goal, best.goal) == 0 &&
               std::strcmp(candidate.face, best.face) != 0;
      });
  const double face_margin = different_face != candidates_end
                                 ? different_face->association_score -
                                       best.association_score
                                 : INFINITY;
  pose.ai_goal = best.goal;
  pose.ai_face = best.face;
  pose.ai_candidate_residual_deg = best.residual_deg;
  pose.ai_candidate_margin_deg = margin;
  pose.ai_predicted_range_in = best.predicted_range_in;
  pose.ai_range_residual_in = best.range_residual_in;
  if (best.residual_deg > localization::kAiMaxCandidateBearingResidualDeg) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "bearing_residual";
  } else if (!std::isfinite(observation.horizontal_range_in) ||
             observation.horizontal_range_in < localization::kAiMinUsableRangeIn ||
             observation.horizontal_range_in > localization::kAiMaxUsableRangeIn) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "range_invalid";
  } else if (best.range_residual_in >
             localization::kAiMaxCandidateRangeResidualIn) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "range_residual";
  } else if (margin < localization::kAiMinCandidateWinnerMarginDeg) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "ambiguous";
  } else if (face_margin < localization::kAiMinFaceWinnerMarginScore) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "face_ambiguous";
  } else if (!localization::kAiVisionPoseCorrectionEnabled) {
    pose.consistent_ai_observations = 0;
    pose.ai_reject = "shadow_valid";
  } else {
    if (std::strcmp(best.goal, pose.last_ai_goal) == 0 &&
        std::strcmp(best.face, pose.last_ai_face) == 0) {
      ++pose.consistent_ai_observations;
    } else {
      pose.last_ai_goal = best.goal;
      pose.last_ai_face = best.face;
      pose.consistent_ai_observations = 1;
    }
    if (pose.consistent_ai_observations <
        localization::kAiRequiredConsistentObservations) {
      pose.ai_reject = "settling";
      return;
    }

    const double observed_global_bearing_rad = deg_to_rad(normalize_deg(
        camera_heading_deg + observation.bearing_deg));
    const double measured_camera_x = best.face_x -
        observation.horizontal_range_in * std::cos(observed_global_bearing_rad);
    const double measured_camera_y = best.face_y -
        observation.horizontal_range_in * std::sin(observed_global_bearing_rad);
    const double measured_robot_x = measured_camera_x -
        localization::kAiCameraForwardOffsetIn * forward_x -
        localization::kAiCameraRightOffsetIn * right_x;
    const double measured_robot_y = measured_camera_y -
        localization::kAiCameraForwardOffsetIn * forward_y -
        localization::kAiCameraRightOffsetIn * right_y;
    const double innovation_x = measured_robot_x - pose.x;
    const double innovation_y = measured_robot_y - pose.y;
    const double innovation_in = std::hypot(innovation_x, innovation_y);
    pose.ai_position_innovation_in = innovation_in;
    const bool normal_innovation = std::isfinite(innovation_in) &&
        innovation_in <= localization::kAiMaxPositionInnovationIn;
    const bool proven_reacquisition = std::isfinite(innovation_in) &&
        innovation_in <= localization::kAiMaxReacquisitionInnovationIn &&
        pose.consistent_ai_observations >=
            localization::kAiRequiredReacquisitionObservations;
    if (!normal_innovation && !proven_reacquisition) {
      pose.ai_reject = "position_innovation";
      return;
    }
    if (now - pose.last_ai_correction_ms <
        localization::kAiCorrectionPeriodMs) {
      pose.ai_reject = "correction_wait";
      return;
    }

    const double requested_step_in = innovation_in <=
            localization::kAiPositionDeadbandIn
        ? 0.0
        : innovation_in * localization::kAiPositionGain;
    const double applied_step_in =
        std::min(requested_step_in, localization::kAiMaxPositionStepIn);
    if (innovation_in > 1e-6) {
      pose.x += innovation_x * applied_step_in / innovation_in;
      pose.y += innovation_y * applied_step_in / innovation_in;
    }
    const double signed_bearing_error_deg = signed_angle_diff_deg(
        observation.bearing_deg, best.predicted_bearing_right_deg);
    const double heading_step_deg =
        std::fabs(signed_bearing_error_deg) <=
                localization::kAiHeadingDeadbandDeg
            ? 0.0
            : clamp(-signed_bearing_error_deg * localization::kAiHeadingGain,
                    -localization::kAiMaxHeadingStepDeg,
                    localization::kAiMaxHeadingStepDeg);
    pose.imu_bias_deg = clamp(pose.imu_bias_deg + heading_step_deg,
                              -kMaxImuBiasDeg, kMaxImuBiasDeg);
    pose.heading_rad = heading_rad_from_deg(
        robot_heading_deg + heading_step_deg);
    pose.ai_position_step_in = applied_step_in;
    pose.ai_heading_step_deg = heading_step_deg;
    pose.last_ai_correction_ms = now;
    pose.ai_reject = "corrected";
  }
}

double signed_angle_diff_deg(double target, double current) {
  double diff = normalize_deg(target) - normalize_deg(current);
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

double line_angle_diff_deg(double a, double b) {
  return std::min(std::fabs(signed_angle_diff_deg(a, b)), std::fabs(signed_angle_diff_deg(a + 180.0, b)));
}

double deg_to_rad(double deg) {
  return deg * kPi / 180.0;
}

double rad_to_deg(double rad) {
  return rad * 180.0 / kPi;
}

navigation::PathPoint make_navigation_path_point(
    const PoseEstimate& pose,
    std::uint32_t now_ms) {
  return navigation::PathPoint{
      pose.x,
      pose.y,
      normalize_deg(rad_to_deg(pose.heading_rad)),
      pose.position_error_envelope_in,
      now_ms - navigation_path_session_start_ms,
      pose.last_absolute_position_ms > 0
          ? now_ms - pose.last_absolute_position_ms
          : 0,
  };
}

void reset_navigation_path_unlocked(const PoseEstimate& pose,
                                    std::uint32_t now_ms) {
  navigation_path_start = 0;
  navigation_path_count = 1;
  navigation_path_session_start_ms = now_ms;
  navigation_path_last_record_ms = now_ms;
  navigation_path[0] = make_navigation_path_point(pose, now_ms);
}

void reset_navigation_path(const PoseEstimate& pose, std::uint32_t now_ms) {
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  reset_navigation_path_unlocked(pose, now_ms);
}

void clear_navigation_path_storage(std::uint32_t now_ms) {
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  navigation_path_start = 0;
  navigation_path_count = 0;
  navigation_path_session_start_ms = now_ms;
  navigation_path_last_record_ms = now_ms;
}

void record_navigation_path(const PoseEstimate& pose, std::uint32_t now_ms) {
  if (!navigation_api_initialized || !pose.ready || !pose.imu_ready) return;
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  if (navigation_path_count == 0) {
    reset_navigation_path_unlocked(pose, now_ms);
    return;
  }
  const std::size_t newest_index =
      (navigation_path_start + navigation_path_count - 1) %
      navigation::kPathCapacity;
  const auto& newest = navigation_path[newest_index];
  const double translation_in = std::hypot(pose.x - newest.x_in,
                                            pose.y - newest.y_in);
  const double heading_change_deg = std::fabs(signed_angle_diff_deg(
      normalize_deg(rad_to_deg(pose.heading_rad)), newest.heading_deg));
  const std::uint32_t age_ms = now_ms - navigation_path_last_record_ms;
  if (age_ms < 100 ||
      (translation_in < 0.10 && heading_change_deg < 0.25 && age_ms < 1000)) {
    return;
  }

  std::size_t write_index = 0;
  if (navigation_path_count < navigation::kPathCapacity) {
    write_index =
        (navigation_path_start + navigation_path_count) %
        navigation::kPathCapacity;
    ++navigation_path_count;
  } else {
    write_index = navigation_path_start;
    navigation_path_start =
        (navigation_path_start + 1) % navigation::kPathCapacity;
  }
  navigation_path[write_index] = make_navigation_path_point(pose, now_ms);
  navigation_path_last_record_ms = now_ms;
}

double heading_rad_from_deg(double deg) {
  return deg_to_rad(normalize_deg(deg));
}

double read_field_imu_heading_deg(const PoseEstimate& pose) {
  // A finite-looking stale value is not sufficient evidence of a healthy
  // heading source. Explicitly reject a missing, calibrating, or API-error IMU
  // before reading rotation so hot-unplug/recalibration cannot pass through.
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating() ||
      chassis.imu.get_status() == pros::ImuStatus::error) {
    return NAN;
  }
  const double imu_rotation_deg = chassis.drive_imu_get();
  if (!std::isfinite(imu_rotation_deg)) return NAN;
  // PROS/EZ IMU rotation is clockwise-positive. Field math here is CCW-positive.
  return normalize_deg(pose.imu_zero_field_heading_deg - imu_rotation_deg);
}

MotorSideReading read_motor_side(const std::vector<pros::Motor>& motors) {
  double sum = 0.0;
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  int count = 0;
  for (const auto& motor : motors) {
    const double position = motor.get_position();
    if (std::isfinite(position)) {
      sum += position;
      minimum = std::min(minimum, position);
      maximum = std::max(maximum, position);
      count++;
    }
  }
  const double spread = count > 1 ? maximum - minimum : 0.0;
  const int expected_count = static_cast<int>(motors.size());
  return MotorSideReading{
      count > 0 ? sum / count : NAN,
      spread,
      count,
      expected_count > 0 && count == expected_count &&
          spread <= kMaxSameSideMotorSpreadDeg,
  };
}

double average_motor_position(const std::vector<pros::Motor>& motors) {
  const MotorSideReading reading = read_motor_side(motors);
  return reading.trustworthy ? reading.position_deg : NAN;
}

bool read_side_odom_position(std::int32_t& position_centideg) {
  if (!horizontal_odom.is_installed()) return false;
  const std::int32_t position = horizontal_odom.get_position();
  if (position == std::numeric_limits<std::int32_t>::max()) return false;
  position_centideg = position;
  return true;
}

GpsObservation read_robot_gps() {
  constexpr double kInchesPerMeter = 39.37007874015748;
  GpsObservation observation;
  if (!gps_7.is_installed()) return observation;

  const auto sensor_position = gps_7.get_position();
  const double sensor_heading_cw_deg = gps_7.get_heading();
  observation.error_in = gps_7.get_error() * kInchesPerMeter;
  if (!std::isfinite(sensor_position.x) ||
      !std::isfinite(sensor_position.y) ||
      !std::isfinite(sensor_heading_cw_deg) ||
      !std::isfinite(observation.error_in) ||
      observation.error_in < 0.0 ||
      observation.error_in > localization::kGpsMaxReportedErrorIn) {
    return observation;
  }

  const auto project_pose = localization::vex_gps_to_project_robot_pose(
      sensor_position.x, sensor_position.y, sensor_heading_cw_deg);
  observation.x_in = project_pose.x_in;
  observation.y_in = project_pose.y_in;
  observation.heading_deg = project_pose.heading_deg;
  observation.valid = true;
  return observation;
}

void update_position_error_envelope(PoseEstimate& pose) {
  const double scale_error_in =
      pose.dead_reckoning_distance_in *
      localization::kDeadReckoningScaleEnvelopeFraction;
  const double heading_cross_track_in =
      pose.dead_reckoning_distance_in * std::tan(deg_to_rad(
          localization::kDeadReckoningHeadingEnvelopeDeg));
  pose.position_error_envelope_in =
      pose.absolute_position_base_error_in +
      std::hypot(scale_error_in, heading_cross_track_in);
}

void record_absolute_position_update(PoseEstimate& pose,
                                     double measurement_error_in,
                                     double remaining_innovation_in,
                                     std::uint32_t now_ms) {
  pose.dead_reckoning_distance_in = 0.0;
  // Remaining innovation and the sensor's own RMS describe different error
  // sources, so add them for a conservative engineering envelope.
  pose.absolute_position_base_error_in =
      std::max(0.0, measurement_error_in) +
      std::max(0.0, remaining_innovation_in);
  pose.last_absolute_position_ms = now_ms;
  update_position_error_envelope(pose);
}

ForwardObstacleObservation read_forward_obstacle() {
  ForwardObstacleObservation observation;
  observation.installed = distance_1.is_installed();
  if (!observation.installed) return observation;
  observation.distance_mm = static_cast<long>(distance_1.get_distance());
  observation.confidence = static_cast<long>(distance_1.get_confidence());
  // PROS_ERR is INT32_MAX, not a negative value. Keep it distinct from the
  // documented 9999-mm healthy/no-target result so an API failure cannot be
  // mistaken for clear space.
  observation.api_ok =
      observation.distance_mm != static_cast<long>(PROS_ERR) &&
      observation.confidence != static_cast<long>(PROS_ERR) &&
      observation.confidence >= 0 && observation.confidence <= 63;
  const bool range_valid =
      observation.distance_mm >= localization::kForwardObstacleMinRangeMm &&
      observation.distance_mm <= localization::kForwardObstacleMaxRangeMm;
  // PROS documents confidence only for readings beyond 200 mm. Inside that
  // range, a finite close return is sufficient for a conservative stop.
  const bool confidence_valid =
      observation.distance_mm <=
          localization::kForwardObstacleConfidenceAvailableMm ||
      observation.confidence >= localization::kForwardObstacleMinConfidence;
  observation.valid = observation.api_ok && range_valid && confidence_valid;
  return observation;
}

bool forward_obstacle_requires_stop(const char* phase) {
  static std::uint32_t close_started_ms = 0;
  static std::uint32_t last_close_poll_ms = 0;
  static bool pending_logged = false;
  auto reset_close_confirmation = [&]() {
    close_started_ms = 0;
    last_close_poll_ms = 0;
    pending_logged = false;
  };
  const ForwardObstacleObservation obstacle = read_forward_obstacle();
  if (!obstacle.installed || !obstacle.api_ok) {
    reset_close_confirmation();
    emergency_stop_drive_motors();
    std::printf(
        "FUSE_TEST phase=%s abort=forward_sensor_%s distance_mm=%ld\n",
        phase,
        obstacle.installed ? "fault" : "missing",
        obstacle.distance_mm);
    std::fflush(stdout);
    return true;
  }
  // 9999 is the documented successful no-target result. This particular P1
  // sensor also repeatedly reports low-confidence 2.5-2.6 m returns when its
  // beam leaves the official 2.0 m calibrated range. A value above the maximum
  // cannot represent an obstacle inside our 8 in stop boundary, so ignore it
  // for collision stopping while leaving observation.valid false for health
  // and fusion diagnostics. Device/API failures still fail closed above.
  if (obstacle.distance_mm == 9999) {
    reset_close_confirmation();
    return false;
  }
  if (obstacle.distance_mm > localization::kForwardObstacleMaxRangeMm) {
    reset_close_confirmation();
    return false;
  }
  if (obstacle.distance_mm < localization::kForwardObstacleMinRangeMm) {
    reset_close_confirmation();
    emergency_stop_drive_motors();
    std::printf(
        "FUSE_TEST phase=%s abort=forward_sensor_range distance_mm=%ld\n",
        phase,
        obstacle.distance_mm);
    std::fflush(stdout);
    return true;
  }
  const double distance_in = obstacle.distance_mm / 25.4;
  if (distance_in > localization::kForwardObstacleStopIn) {
    reset_close_confirmation();
    return false;
  }
  const std::uint32_t now_ms = pros::millis();
  const bool immediate =
      distance_in <= localization::kForwardObstacleImmediateStopIn;
  if (!immediate) {
    if (close_started_ms == 0 ||
        now_ms - last_close_poll_ms >
            localization::kForwardObstacleConfirmationMs) {
      close_started_ms = now_ms;
      pending_logged = false;
    }
    last_close_poll_ms = now_ms;
    const std::uint32_t close_elapsed_ms = now_ms - close_started_ms;
    if (close_elapsed_ms < localization::kForwardObstacleConfirmationMs) {
      if (!pending_logged) {
        std::printf(
            "FUSE_TEST phase=%s event=forward_obstacle_pending "
            "distance_in=%.2f confidence=%ld confirm_ms=%lu\n",
            phase, distance_in, obstacle.confidence,
            static_cast<unsigned long>(
                localization::kForwardObstacleConfirmationMs));
        std::fflush(stdout);
        pending_logged = true;
      }
      return false;
    }
  }
  // At the emergency-stop boundary, a physical return is enough to stop even
  // when its confidence is low, but a non-immediate return must persist for a
  // fresh sensor interval so one cached/transient frame cannot stop the route.
  reset_close_confirmation();
  emergency_stop_drive_motors();
  std::printf(
      "FUSE_TEST phase=%s abort=forward_obstacle distance_in=%.2f "
      "confidence=%ld stop_in=%.2f immediate=%d\n",
      phase,
      distance_in,
      obstacle.confidence,
      localization::kForwardObstacleStopIn,
      static_cast<int>(immediate));
  std::fflush(stdout);
  return true;
}

void apply_gps_fusion(PoseEstimate& pose,
                      double linear_speed_in_s,
                      double angular_rate_deg_s,
                      std::uint32_t now_ms) {
  pose.gps_position_step_in = 0.0;
  pose.gps_heading_step_deg = 0.0;
  if (now_ms - pose.last_gps_sample_ms <
      localization::kGpsCorrectionPeriodMs) {
    // Preserve the last evaluated GPS outcome between physical sensor polls;
    // otherwise health telemetry says "wait" and accepted=false for most
    // estimator ticks even after a genuinely accepted fix.
    return;
  }
  pose.last_gps_sample_ms = now_ms;

  GpsObservation observation = read_robot_gps();
  if (observation.valid && pose.gps_frame_aligned) {
    const double rotation_rad = deg_to_rad(pose.gps_frame_rotation_deg);
    const double raw_x = observation.x_in;
    const double raw_y = observation.y_in;
    observation.x_in =
        std::cos(rotation_rad) * raw_x -
        std::sin(rotation_rad) * raw_y +
        pose.gps_frame_translation_x_in;
    observation.y_in =
        std::sin(rotation_rad) * raw_x +
        std::cos(rotation_rad) * raw_y +
        pose.gps_frame_translation_y_in;
    observation.heading_deg = normalize_deg(
        observation.heading_deg + pose.gps_frame_rotation_deg);
  }
  pose.gps_x_in = observation.x_in;
  pose.gps_y_in = observation.y_in;
  pose.gps_heading_deg = observation.heading_deg;
  pose.gps_error_in = observation.error_in;
  pose.gps_position_innovation_in = NAN;
  if (!observation.valid) {
    pose.consistent_gps_observations = 0;
    pose.gps_reject = gps_7.is_installed() ? "quality" : "missing";
    return;
  }

  const double current_heading_deg =
      normalize_deg(rad_to_deg(pose.heading_rad));
  const bool changed_observation =
      !pose.have_gps_geometry_motion_anchor ||
      std::hypot(observation.x_in - pose.last_gps_raw_x_in,
                 observation.y_in - pose.last_gps_raw_y_in) > 1e-4 ||
      std::fabs(signed_angle_diff_deg(
          observation.heading_deg, pose.last_gps_raw_heading_deg)) > 1e-3;
  if (changed_observation) {
    pose.last_gps_raw_x_in = observation.x_in;
    pose.last_gps_raw_y_in = observation.y_in;
    pose.last_gps_raw_heading_deg = observation.heading_deg;
    pose.last_gps_geometry_drive_distance_in = pose.total_drive_distance_in;
    pose.last_gps_geometry_heading_deg = current_heading_deg;
    pose.have_gps_geometry_motion_anchor = true;
  } else {
    const double drive_since_geometry_in =
        pose.total_drive_distance_in -
        pose.last_gps_geometry_drive_distance_in;
    const double heading_since_geometry_deg = std::fabs(
        signed_angle_diff_deg(current_heading_deg,
                              pose.last_gps_geometry_heading_deg));
    if (drive_since_geometry_in >
            localization::kGpsMaxRepeatedObservationDriveIn ||
        heading_since_geometry_deg >
            localization::kGpsMaxRepeatedObservationHeadingDeg) {
      pose.consistent_gps_observations = 0;
      pose.gps_reject = "stale_geometry";
      return;
    }
    // The GPS API likewise exposes no frame timestamp. Do not let a frozen
    // tuple qualify merely because the estimator polled it repeatedly while
    // stationary; wait for actual numeric change before adding evidence.
    pose.gps_reject = "repeat";
    return;
  }

  if (std::fabs(linear_speed_in_s) >
      localization::kGpsMaxCorrectionLinearSpeedInS) {
    pose.consistent_gps_observations = 0;
    pose.gps_reject = "motion";
    return;
  }

  // GPS is a visual, lower-rate absolute reference. The 2026-08-23 live
  // rotation tests showed that its apparent X/Y can move by many inches while
  // encoders and IMU return to the same pose. Never let a rotating observation
  // bend the trajectory; reset the consistency chain and let IMU + encoders
  // own the turn, then reacquire GPS only after stationary agreement.
  if (std::fabs(angular_rate_deg_s) >
      localization::kGpsMaxCorrectionAngularRateDegS) {
    pose.consistent_gps_observations = 0;
    pose.gps_reject = "spin";
    return;
  }

  // pending_gps_* is the fixed anchor of this candidate cluster, not the
  // previous sample. This rejects a slowly walking visual solution whose
  // adjacent steps are small but whose total displacement is impossible.
  const bool reset_cluster =
      pose.consistent_gps_observations == 0 ||
      !std::isfinite(pose.pending_gps_x_in) ||
      std::hypot(observation.x_in - pose.pending_gps_x_in,
                 observation.y_in - pose.pending_gps_y_in) >
          localization::kGpsMaxObservationStepIn ||
      std::fabs(signed_angle_diff_deg(
          observation.heading_deg, pose.pending_gps_heading_deg)) >
          localization::kGpsMaxObservationHeadingStepDeg;
  if (reset_cluster) {
    pose.consistent_gps_observations = 1;
    pose.pending_gps_x_in = observation.x_in;
    pose.pending_gps_y_in = observation.y_in;
    pose.pending_gps_heading_deg = observation.heading_deg;
  } else {
    ++pose.consistent_gps_observations;
  }
  if (pose.consistent_gps_observations <
      localization::kGpsRequiredConsistentObservations) {
    pose.gps_reject = "settling";
    return;
  }

  if (!pose.gps_anchored) {
    // Automatic first-fix anchoring is deliberately unsupported. Competition
    // initialization always supplies a known start coordinate, and live tests
    // proved that a visually stable first GPS fix can be wrong by 16+ inches
    // even after the VEX-native-to-project frame conversion is applied.
    pose.gps_reject = "start_anchor_required";
    return;
  }

  const double innovation_x = observation.x_in - pose.x;
  const double innovation_y = observation.y_in - pose.y;
  const double innovation_in = std::hypot(innovation_x, innovation_y);
  pose.gps_position_innovation_in = innovation_in;
  const bool normal_innovation =
      std::isfinite(innovation_in) &&
      innovation_in <= localization::kGpsMaxPositionInnovationIn;
  const bool proven_reacquisition =
      std::isfinite(innovation_in) &&
      innovation_in <= localization::kGpsMaxReacquisitionInnovationIn &&
      pose.consistent_gps_observations >=
          localization::kGpsRequiredReacquisitionObservations;
  if (!normal_innovation && !proven_reacquisition) {
    pose.gps_reject = "position_innovation";
    return;
  }

  const double requested_position_step_in =
      innovation_in <= localization::kGpsPositionDeadbandIn
          ? 0.0
          : innovation_in * localization::kGpsPositionGain;
  const double position_step_in = std::min(
      requested_position_step_in, localization::kGpsMaxPositionStepIn);
  if (innovation_in > 1e-9) {
    pose.x += innovation_x * position_step_in / innovation_in;
    pose.y += innovation_y * position_step_in / innovation_in;
  }
  pose.gps_position_step_in = position_step_in;
  record_absolute_position_update(
      pose,
      observation.error_in,
      std::max(0.0, innovation_in - position_step_in),
      now_ms);

  const double heading_error_deg = signed_angle_diff_deg(
      observation.heading_deg, normalize_deg(rad_to_deg(pose.heading_rad)));
  if constexpr (localization::kGpsHeadingGain <= 0.0 ||
                localization::kGpsMaxHeadingStepDeg <= 0.0) {
    pose.gps_heading_step_deg = 0.0;
    pose.gps_reject = "position_only";
    return;
  }
  if (std::fabs(heading_error_deg) <=
          localization::kGpsMaxHeadingInnovationDeg) {
    const double heading_step_deg =
        std::fabs(heading_error_deg) <= localization::kGpsHeadingDeadbandDeg
            ? 0.0
            : clamp(heading_error_deg * localization::kGpsHeadingGain,
                    -localization::kGpsMaxHeadingStepDeg,
                    localization::kGpsMaxHeadingStepDeg);
    pose.imu_bias_deg = clamp(pose.imu_bias_deg + heading_step_deg,
                              -kMaxImuBiasDeg, kMaxImuBiasDeg);
    pose.heading_rad = heading_rad_from_deg(
        normalize_deg(rad_to_deg(pose.heading_rad)) + heading_step_deg);
    pose.gps_heading_step_deg = heading_step_deg;
    pose.gps_reject = "corrected";
  } else {
    pose.gps_reject = "heading_innovation_position_only";
  }
}

void stop_drive() {
  chassis.drive_set(0, 0);
}

void stop_drive_motors_unlocked() {
  chassis.pid_targets_reset();
  // Cancel any prior voltage command first, then issue the API operation that
  // actually honors motor brake mode. motor.move(0) is only a zero-voltage
  // command and would leave these autonomous safety/arrival stops coasting.
  stop_drive();
  const auto stop_mode = drive_emergency_hold_latched
                             ? pros::E_MOTOR_BRAKE_HOLD
                             : pros::E_MOTOR_BRAKE_BRAKE;
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(stop_mode);
    motor.brake();
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(stop_mode);
    motor.brake();
  }
}

void stop_drive_motors() {
  std::lock_guard<pros::Mutex> lock(drive_output_mutex);
  stop_drive_motors_unlocked();
}

void emergency_stop_drive_motors() {
  std::lock_guard<pros::Mutex> lock(drive_output_mutex);
  drive_emergency_hold_latched = true;
  stop_drive_motors_unlocked();
}

bool blocking_motion_abort_requested(const char* phase) {
  // Cancellation is independent of the authorization flag. A concurrent pose
  // edit intentionally revokes navigation_api_initialized before an active
  // loop can next poll; conditioning this read on that flag would hide the
  // cancellation precisely when it is most important.
  const bool caller_cancelled =
      navigation_stop_requested.load(std::memory_order_acquire);
  const bool field_disabled = pros::competition::is_connected() &&
                              pros::competition::is_disabled();
  if (!caller_cancelled && !field_disabled) return false;
  emergency_stop_drive_motors();
  std::printf("FUSE_TEST phase=%s abort=%s\n",
              phase,
              field_disabled ? "field_disabled" : "cancelled");
  std::fflush(stdout);
  return true;
}

bool motion_pose_invalid(const PoseEstimate& pose, const char* phase) {
  if (pose.ready && pose.imu_ready) return false;
  emergency_stop_drive_motors();
  std::printf(
      "FUSE_TEST phase=%s abort=localization_sensor drive=%d imu=%d\n",
      phase,
      static_cast<int>(pose.ready),
      static_cast<int>(pose.imu_ready));
  std::fflush(stdout);
  return true;
}

void init_pose(PoseEstimate& pose, const localization::FieldPose& start_pose) {
  const MotorSideReading left = read_motor_side(chassis.left_motors);
  const MotorSideReading right = read_motor_side(chassis.right_motors);
  std::int32_t side_centideg = 0;
  const bool side_odom_ready = read_side_odom_position(side_centideg);
  pose.imu_zero_field_heading_deg = normalize_deg(start_pose.heading_deg);
  chassis.drive_imu_reset(0.0);
  pros::delay(20);
  const double imu_heading_deg = read_field_imu_heading_deg(pose);
  pose.left_deg = left.trustworthy ? left.position_deg : 0.0;
  pose.right_deg = right.trustworthy ? right.position_deg : 0.0;
  pose.side_centideg = side_odom_ready ? side_centideg : 0;
  pose.x = start_pose.x_in;
  pose.y = start_pose.y_in;
  pose.imu_heading_deg = std::isfinite(imu_heading_deg) ? imu_heading_deg : pose.imu_zero_field_heading_deg;
  pose.imu_bias_deg = 0.0;
  pose.left_motor_count = left.valid_count;
  pose.right_motor_count = right.valid_count;
  pose.left_motor_spread_deg = left.spread_deg;
  pose.right_motor_spread_deg = right.spread_deg;
  pose.side_odom_ready = side_odom_ready;
  pose.heading_rad = heading_rad_from_deg(pose.imu_heading_deg);
  pose.last_update_ms = pros::millis();
  pose.last_lidar_bias_ms = pose.last_update_ms - kLidarImuCorrectionPeriodMs;
  pose.last_lidar_sample_ms = pose.last_lidar_bias_ms;
  pose.last_gps_sample_ms = pose.last_update_ms -
                            localization::kGpsCorrectionPeriodMs;
  // init_pose() is supplied an explicit, field-frame robot pose. Treat that
  // pose as the absolute anchor immediately; otherwise a visually stable but
  // wrong GPS solution can bypass the innovation gate and teleport the
  // estimator on its first accepted cluster.
  pose.gps_anchored = true;
  const GpsObservation initial_gps = read_robot_gps();
  if (initial_gps.valid) {
    pose.gps_frame_rotation_deg = signed_angle_diff_deg(
        start_pose.heading_deg, initial_gps.heading_deg);
    const double rotation_rad = deg_to_rad(pose.gps_frame_rotation_deg);
    const double rotated_x =
        std::cos(rotation_rad) * initial_gps.x_in -
        std::sin(rotation_rad) * initial_gps.y_in;
    const double rotated_y =
        std::sin(rotation_rad) * initial_gps.x_in +
        std::cos(rotation_rad) * initial_gps.y_in;
    pose.gps_frame_translation_x_in = start_pose.x_in - rotated_x;
    pose.gps_frame_translation_y_in = start_pose.y_in - rotated_y;
    pose.gps_frame_aligned = true;
    std::printf(
        "GPS_FRAME aligned=1 rotation=%.3f tx=%.3f ty=%.3f "
        "raw_x=%.3f raw_y=%.3f raw_h=%.3f\n",
        pose.gps_frame_rotation_deg,
        pose.gps_frame_translation_x_in,
        pose.gps_frame_translation_y_in,
        initial_gps.x_in, initial_gps.y_in, initial_gps.heading_deg);
  }
  pose.consistent_gps_observations = 0;
  pose.pending_gps_x_in = NAN;
  pose.pending_gps_y_in = NAN;
  pose.pending_gps_heading_deg = NAN;
  pose.gps_reject = "start_anchor";
  pose.dead_reckoning_distance_in = 0.0;
  pose.absolute_position_base_error_in = 0.0;
  pose.position_error_envelope_in = 0.0;
  pose.last_absolute_position_ms = pose.last_update_ms;
  pose.imu_ready = std::isfinite(imu_heading_deg);
  pose.ready = left.trustworthy && right.trustworthy;

  printf("FUSE_INIT x=%.2f y=%.2f heading=%.2f raw_imu=%.2f imu_zero_field=%.2f drive=L%d/R%d spread=%.1f/%.1f side=%s\n",
         pose.x,
         pose.y,
         normalize_deg(rad_to_deg(pose.heading_rad)),
         chassis.drive_imu_get(),
         pose.imu_zero_field_heading_deg,
         pose.left_motor_count,
         pose.right_motor_count,
         pose.left_motor_spread_deg,
         pose.right_motor_spread_deg,
         pose.side_odom_ready ? "ok" : "invalid");
  fflush(stdout);
}

LidarFit read_lidar_fit() {
  // Port 9 is now the right slider motor. The calibrated wall fit requires
  // all four rigidly aligned sensors, so LiDAR correction remains fail-closed.
  return {};
}

void apply_lidar_fusion(PoseEstimate& pose,
                        const LidarFit& fit,
                        double angular_rate_deg_s,
                        double field_imu_heading_deg,
                        std::uint32_t now_ms) {
  pose.lidar_used = false;
  pose.lidar_reject = "none";
  pose.lidar_axis_correction_in = 0.0;
  if (!fit.ok) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "fit";
    return;
  }
  if (now_ms - pose.last_lidar_bias_ms < kLidarImuCorrectionPeriodMs) {
    pose.lidar_reject = "wait";
    return;
  }
  if (angular_rate_deg_s > kMaxLidarAngularRateDegS) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "spin";
    return;
  }
  if (!std::isfinite(field_imu_heading_deg)) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "imu";
    return;
  }

  const double wall = localization::kPhysicalWallHalfSpanIn;
  const double theta_deg = fit.theta_deg;
  // Live CW/CCW calibration proved ports 6->9 run opposite the originally
  // assumed bar direction: field-heading candidates advance with +theta.
  const double red_heading_deg = normalize_deg(theta_deg);
  const double audience_heading_deg = normalize_deg(90.0 + theta_deg);
  const double blue_heading_deg = normalize_deg(180.0 + theta_deg);
  const double zero_heading_deg = normalize_deg(270.0 + theta_deg);
  const auto lidar_offset_x = [](double heading_deg) {
    const double heading_rad = deg_to_rad(heading_deg);
    return kLidarForwardOffsetIn * std::cos(heading_rad) -
           kLidarLeftOffsetIn * std::sin(heading_rad);
  };
  const auto lidar_offset_y = [](double heading_deg) {
    const double heading_rad = deg_to_rad(heading_deg);
    return kLidarForwardOffsetIn * std::sin(heading_rad) +
           kLidarLeftOffsetIn * std::cos(heading_rad);
  };
  const double heading_est_deg = normalize_deg(field_imu_heading_deg + pose.imu_bias_deg);
  std::array<WallCandidate, 4> candidates = {{
      {"red",
       red_heading_deg,
       false,
       wall - fit.wall_distance_in - lidar_offset_y(red_heading_deg),
       0.0,
       0.0,
       0.0},
      {"audience",
       audience_heading_deg,
       true,
       -wall + fit.wall_distance_in - lidar_offset_x(audience_heading_deg),
       0.0,
       0.0,
       0.0},
      {"blue",
       blue_heading_deg,
       false,
       -wall + fit.wall_distance_in - lidar_offset_y(blue_heading_deg),
       0.0,
       0.0,
       0.0},
      {"zero",
       zero_heading_deg,
       true,
       wall - fit.wall_distance_in - lidar_offset_x(zero_heading_deg),
       0.0,
       0.0,
       0.0},
  }};
  for (auto& candidate : candidates) {
    candidate.heading_error_deg =
        std::fabs(signed_angle_diff_deg(candidate.heading_deg, heading_est_deg));
    const double predicted_axis = candidate.corrects_x ? pose.x : pose.y;
    candidate.axis_error_in = std::fabs(candidate.axis_value_in - predicted_axis);
    candidate.score = candidate.axis_error_in +
                      candidate.heading_error_deg * kLidarHeadingScoreInPerDeg;
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.score < b.score;
  });
  const WallCandidate& best = candidates[0];
  const WallCandidate& second = candidates[1];
  const double score_margin = second.score - best.score;
  if (score_margin < kMinLidarCandidateScoreMargin) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "ambig";
    return;
  }
  if (best.axis_error_in > kMaxLidarAxisInnovationIn ||
      best.axis_value_in < -wall || best.axis_value_in > wall) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "axis";
    return;
  }

  const double wall_heading_deg = best.heading_deg;
  const double heading_error_deg =
      signed_angle_diff_deg(wall_heading_deg, heading_est_deg);
  if (std::fabs(heading_error_deg) > kMaxWallHeadingErrorDeg) {
    pose.consistent_lidar_fits = 0;
    pose.lidar_reject = "theta";
    return;
  }

  if (pose.consistent_lidar_fits == 0 ||
      std::strcmp(best.wall, pose.pending_lidar_wall) != 0 ||
      std::fabs(signed_angle_diff_deg(wall_heading_deg,
                                      pose.pending_lidar_heading_deg)) >
          kMaxLidarCandidateChangeDeg ||
      std::fabs(best.axis_value_in - pose.pending_lidar_axis_value_in) >
          kMaxLidarAxisCandidateChangeIn) {
    pose.pending_lidar_heading_deg = wall_heading_deg;
    pose.pending_lidar_axis_value_in = best.axis_value_in;
    pose.pending_lidar_wall = best.wall;
    pose.consistent_lidar_fits = 1;
    pose.lidar_reject = "settle";
    return;
  }

  const double candidate_delta_deg =
      signed_angle_diff_deg(wall_heading_deg, pose.pending_lidar_heading_deg);
  pose.pending_lidar_heading_deg = normalize_deg(
      pose.pending_lidar_heading_deg + candidate_delta_deg * kLidarCandidateFilterGain);
  pose.pending_lidar_axis_value_in +=
      (best.axis_value_in - pose.pending_lidar_axis_value_in) *
      kLidarAxisFilterGain;
  pose.consistent_lidar_fits++;
  if (pose.consistent_lidar_fits < kRequiredConsistentLidarFits) {
    pose.lidar_reject = "settle";
    return;
  }

  const double filtered_heading_error_deg =
      signed_angle_diff_deg(pose.pending_lidar_heading_deg, heading_est_deg);
  double bias_step_deg = 0.0;
  if (std::fabs(filtered_heading_error_deg) > kLidarHeadingDeadbandDeg) {
    const bool fine_correction =
        std::fabs(filtered_heading_error_deg) <= kLidarFineCorrectionRangeDeg;
    const double gain = fine_correction ? kLidarFineBiasGain : kLidarBiasGain;
    const double maximum_step =
        fine_correction ? kMaxLidarFineBiasStepDeg : kMaxLidarBiasStepDeg;
    bias_step_deg = clamp(filtered_heading_error_deg * gain,
                          -maximum_step,
                          maximum_step);
  }
  pose.imu_bias_deg = clamp(pose.imu_bias_deg + bias_step_deg,
                            -kMaxImuBiasDeg,
                            kMaxImuBiasDeg);
  pose.last_lidar_bias_ms = now_ms;
  const double corrected_field_heading_deg = normalize_deg(field_imu_heading_deg + pose.imu_bias_deg);
  pose.imu_heading_deg = field_imu_heading_deg;
  pose.heading_rad = heading_rad_from_deg(corrected_field_heading_deg);
  double axis_error_in = pose.pending_lidar_axis_value_in -
                         (best.corrects_x ? pose.x : pose.y);
  double axis_step_in = 0.0;
  if (std::fabs(axis_error_in) > kLidarAxisDeadbandIn) {
    const bool fine_axis = std::fabs(axis_error_in) <= kLidarFineAxisRangeIn;
    const double axis_gain = fine_axis ? kLidarFineAxisGain : kLidarAxisGain;
    const double maximum_axis_step =
        fine_axis ? kMaxLidarFineAxisStepIn : kMaxLidarAxisStepIn;
    axis_step_in = clamp(axis_error_in * axis_gain,
                         -maximum_axis_step,
                         maximum_axis_step);
    if (best.corrects_x) {
      pose.x += axis_step_in;
    } else {
      pose.y += axis_step_in;
    }
  }
  pose.lidar_used = true;
  pose.lidar_wall = best.wall;
  pose.lidar_axis = best.corrects_x ? "x" : "y";
  pose.lidar_axis_correction_in = axis_step_in;

  printf("FUSE_TEST lidar_correction wall=%s theta=%.2f distance=%.2f wall_heading=%.2f filtered_heading=%.2f heading_error=%.2f heading_step=%.2f corrected_heading=%.2f bias=%.2f axis=%s axis_observed=%.2f axis_error=%.2f axis_step=%.2f score=%.2f margin=%.2f fits=%d conf=%ld rmse=%.2f\n",
         best.wall,
         fit.theta_deg,
         fit.wall_distance_in,
         wall_heading_deg,
         pose.pending_lidar_heading_deg,
         filtered_heading_error_deg,
         bias_step_deg,
         normalize_deg(rad_to_deg(pose.heading_rad)),
         pose.imu_bias_deg,
         best.corrects_x ? "x" : "y",
         pose.pending_lidar_axis_value_in,
         axis_error_in,
         axis_step_in,
         best.score,
         score_margin,
         pose.consistent_lidar_fits,
         fit.min_confidence,
         fit.rmse_in);
  fflush(stdout);
}

void update_pose(PoseEstimate& pose,
                 LidarFusionMode lidar_mode = LidarFusionMode::kBiasOnly,
                 bool allow_absolute_position_correction = true) {
  const MotorSideReading left = read_motor_side(chassis.left_motors);
  const MotorSideReading right = read_motor_side(chassis.right_motors);
  std::int32_t side_centideg = 0;
  const bool side_sample_valid =
      localization::kSideOdomEnabled && read_side_odom_position(side_centideg);
  const std::uint32_t now = pros::millis();
  pose.left_motor_count = left.valid_count;
  pose.right_motor_count = right.valid_count;
  pose.left_motor_spread_deg = left.spread_deg;
  pose.right_motor_spread_deg = right.spread_deg;
  const bool drive_encoder_missing =
      left.valid_count != static_cast<int>(chassis.left_motors.size()) ||
      right.valid_count != static_cast<int>(chassis.right_motors.size());
  const bool drive_spread_excessive =
      left.spread_deg > kMaxSameSideMotorSpreadDeg ||
      right.spread_deg > kMaxSameSideMotorSpreadDeg;
  pose.drive_spread_fault_samples = drive_spread_excessive
      ? pose.drive_spread_fault_samples + 1
      : 0;
  const bool persistent_drive_spread =
      pose.drive_spread_fault_samples >=
      kDriveSpreadFaultConsecutiveSamples;
  if (drive_encoder_missing || persistent_drive_spread) {
    // Losing either coupled encoder pair makes motion during the gap
    // unobservable. Latch the pose invalid until navigation::init() supplies a
    // fresh absolute start; silently resuming would either integrate a stale
    // encoder jump or pretend that any movement during the gap never happened.
    pose.ready = false;
    pose.last_update_ms = now;
    pose.consistent_gps_observations = 0;
    pose.gps_position_step_in = 0.0;
    pose.gps_heading_step_deg = 0.0;
    pose.gps_reject = "drive_invalid";
    pose.consistent_ai_observations = 0;
    pose.ai_position_step_in = 0.0;
    pose.ai_heading_step_deg = 0.0;
    pose.ai_reject = "drive_invalid";
    publish_if_telemetry_pose(pose);
    return;
  }
  if (!pose.ready) {
    // Continuously re-baseline healthy readings without restoring validity.
    // Only explicit navigation::init() can assert where the robot is after an
    // encoder-observation gap.
    pose.left_deg = left.position_deg;
    pose.right_deg = right.position_deg;
    pose.last_update_ms = now;
    pose.side_odom_ready = false;
    pose.gps_reject = "reinit_required";
    pose.ai_reject = "reinit_required";
    publish_if_telemetry_pose(pose);
    return;
  }

  const double dt_s = pose.last_update_ms > 0 ? static_cast<double>(now - pose.last_update_ms) / 1000.0 : 0.02;
  pose.last_update_ms = now;

  const double delta_left_in = ((left.position_deg - pose.left_deg) / 360.0) * kWheelCircumferenceIn * kLeftEncoderSign;
  const double delta_right_in = ((right.position_deg - pose.right_deg) / 360.0) * kWheelCircumferenceIn * kRightEncoderSign;
  double delta_side_wheel_in = 0.0;
  bool side_delta_accepted = false;
  pose.side_odom_reject = "none";
  if (!localization::kSideOdomEnabled) {
    pose.side_odom_ready = false;
    pose.side_odom_reject = "disabled";
  } else if (!side_sample_valid) {
    // Never reuse a stale tracking-wheel baseline after a disconnect or bad
    // sample. The first recovered sample establishes a new baseline.
    pose.side_odom_ready = false;
    pose.side_odom_reject = "invalid";
  } else if (!pose.side_odom_ready) {
    pose.side_centideg = side_centideg;
    pose.side_odom_ready = true;
    pose.side_odom_reject = "baseline";
  } else {
    const double delta_side_centideg =
        static_cast<double>(side_centideg) - static_cast<double>(pose.side_centideg);
    const double candidate_side_in =
        ((delta_side_centideg / 100.0) / 360.0) * kSideOdomCircumferenceIn *
        kSideOdomRawSign;
    const double maximum_side_delta_in = kMaxSideOdomSpeedInS * std::max(0.001, dt_s) +
                                         kSideOdomJumpAllowanceIn;
    if (std::fabs(candidate_side_in) <= maximum_side_delta_in) {
      delta_side_wheel_in = candidate_side_in;
      side_delta_accepted = true;
      pose.side_centideg = side_centideg;
    } else {
      pose.side_centideg = side_centideg;
      pose.side_odom_reject = "jump";
    }
  }
  const double drivetrain_delta_heading_rad = (delta_right_in - delta_left_in) / kTrackWidthIn;

  pose.left_deg = left.position_deg;
  pose.right_deg = right.position_deg;
  // During an in-place tank turn the two sides counter-rotate, so the true
  // center translation is zero. Averaging tiny side/gearbox asymmetries here
  // accumulated almost an inch of fictitious travel across four 15-degree
  // out/return legs. Curved forward driving is unaffected because both sides
  // retain the same sign.
  const bool in_place_counter_rotation =
      delta_left_in * delta_right_in < 0.0;
  const double delta_center_in = in_place_counter_rotation
                                     ? 0.0
                                     : (delta_left_in + delta_right_in) / 2.0;
  const double field_imu_heading_deg = read_field_imu_heading_deg(pose);
  double delta_heading_rad = drivetrain_delta_heading_rad;
  double angular_rate_deg_s = std::fabs(rad_to_deg(drivetrain_delta_heading_rad)) / std::max(0.001, dt_s);
  if (std::isfinite(field_imu_heading_deg) && pose.imu_ready) {
    const double delta_heading_deg = signed_angle_diff_deg(field_imu_heading_deg, pose.imu_heading_deg);
    delta_heading_rad = deg_to_rad(delta_heading_deg);
    angular_rate_deg_s = std::fabs(delta_heading_deg) / std::max(0.001, dt_s);
    pose.imu_heading_deg = field_imu_heading_deg;
  } else if (!std::isfinite(field_imu_heading_deg)) {
    // As with encoder loss, heading accumulated during an IMU gap cannot be
    // certified. Keep encoder-only propagation for diagnostics, but latch the
    // public pose invalid until navigation::init() re-establishes an anchor.
    pose.imu_ready = false;
  }

  // Rear-offset compensation is meaningful only when a fresh tracking-wheel
  // delta was accepted. Applying it on an absent/baseline/rejected sample
  // invents sideways movement during an otherwise pure rotation.
  const double delta_side_center_in = side_delta_accepted
                                          ? delta_side_wheel_in -
                                                kSideOdomOffsetBackIn * delta_heading_rad
                                          : 0.0;
  const double mid_heading = pose.heading_rad + delta_heading_rad / 2.0;

  pose.x += delta_center_in * std::cos(mid_heading) + delta_side_center_in * std::sin(mid_heading);
  pose.y += delta_center_in * std::sin(mid_heading) - delta_side_center_in * std::cos(mid_heading);
  pose.dead_reckoning_distance_in +=
      std::hypot(delta_center_in, delta_side_center_in);
  pose.total_drive_distance_in += std::fabs(delta_center_in);
  update_position_error_envelope(pose);
  if (std::isfinite(field_imu_heading_deg)) {
    pose.heading_rad = heading_rad_from_deg(field_imu_heading_deg + pose.imu_bias_deg);
  } else {
    pose.heading_rad += delta_heading_rad;
  }
  const double linear_speed_in_s =
      std::fabs(delta_center_in) / std::max(0.001, dt_s);
  if (allow_absolute_position_correction) {
    apply_gps_fusion(pose, linear_speed_in_s, angular_rate_deg_s, now);
    update_ai_vision_shadow(pose, now);
  } else {
    pose.consistent_gps_observations = 0;
    pose.gps_position_step_in = 0.0;
    pose.gps_heading_step_deg = 0.0;
    pose.gps_reject = "turn_suppressed";
    pose.consistent_ai_observations = 0;
    pose.ai_position_step_in = 0.0;
    pose.ai_heading_step_deg = 0.0;
    pose.ai_reject = "turn_suppressed";
  }

  if (navigation_test_inject_imu_dropout &&
      !navigation_test_imu_dropout_latched &&
      pose.dead_reckoning_distance_in >= navigation_test_imu_dropout_after_in) {
    navigation_test_imu_dropout_latched = true;
    pose.imu_ready = false;
    std::printf(
        "NAV_DROPOUT event=injected dr_travel=%.3f x=%.3f y=%.3f heading=%.3f\n",
        pose.dead_reckoning_distance_in, pose.x, pose.y,
        pose_heading_deg(pose));
    std::fflush(stdout);
  }

  if (lidar_mode == LidarFusionMode::kDisabled) {
    pose.lidar_used = false;
    pose.lidar_wall = "none";
    pose.lidar_reject = "motion";
  } else {
    if (now - pose.last_lidar_sample_ms >= kLidarImuCorrectionPeriodMs) {
      pose.last_lidar_sample_ms = now;
      const LidarFit fit = read_lidar_fit();
      pose.lidar_theta_deg = fit.ok ? fit.theta_deg : NAN;
      pose.lidar_distance_in = fit.ok ? fit.wall_distance_in : NAN;
      pose.lidar_rmse_in = fit.ok ? fit.rmse_in : NAN;
      apply_lidar_fusion(
          pose,
          fit,
          angular_rate_deg_s,
          field_imu_heading_deg,
          now);
    } else {
      pose.lidar_used = false;
      pose.lidar_reject = "wait";
      pose.lidar_axis_correction_in = 0.0;
    }
  }
  record_navigation_path(pose, now);
  publish_if_telemetry_pose(pose);
}

void log_pose(const char* phase, const PoseEstimate& pose) {
  printf(
      "FUSE_TEST phase=%s x=%.2f y=%.2f heading=%.2f estimator_valid=%d pose_valid=%d imu=%.2f bias=%.2f lidar=%s reject=%s theta=%.2f distance=%.2f rmse=%.3f lidar_axis=%s axis_step=%.2f drive=L%d/R%d spread=%.1f/%.1f side=%s track=%.4f rear=%.4f lidar_scale=%.6f gps_x=%.2f gps_y=%.2f gps_heading=%.2f gps_error=%.2f gps_innovation=%.2f gps_pos_step=%.2f gps_heading_step=%.2f gps_reject=%s ai_id=%d ai_bearing=%.2f ai_range=%.2f ai_pred_range=%.2f ai_range_residual=%.2f ai_innovation=%.2f ai_pos_step=%.2f ai_heading_step=%.2f ai_goal=%s ai_face=%s ai_residual=%.2f ai_margin=%.2f ai_age=%lu ai_reject=%s dr_travel=%.2f pos_envelope=%.2f abs_age=%lu\n",
      phase,
      pose.x,
      pose.y,
      normalize_deg(rad_to_deg(pose.heading_rad)),
      static_cast<int>(pose.ready && pose.imu_ready),
      static_cast<int>(navigation_api_initialized && pose.ready &&
                       pose.imu_ready),
      pose.imu_heading_deg,
      pose.imu_bias_deg,
      pose.lidar_wall,
      pose.lidar_reject,
      pose.lidar_theta_deg,
      pose.lidar_distance_in,
      pose.lidar_rmse_in,
      pose.lidar_axis,
      pose.lidar_axis_correction_in,
      pose.left_motor_count,
      pose.right_motor_count,
      pose.left_motor_spread_deg,
      pose.right_motor_spread_deg,
      pose.side_odom_reject,
      kTrackWidthIn,
      kSideOdomOffsetBackIn,
      localization::kLidarThetaScale,
      pose.gps_x_in,
      pose.gps_y_in,
      pose.gps_heading_deg,
      pose.gps_error_in,
      pose.gps_position_innovation_in,
      pose.gps_position_step_in,
      pose.gps_heading_step_deg,
      pose.gps_reject,
      pose.ai_tag_id,
      pose.ai_bearing_right_deg,
      pose.ai_observed_range_in,
      pose.ai_predicted_range_in,
      pose.ai_range_residual_in,
      pose.ai_position_innovation_in,
      pose.ai_position_step_in,
      pose.ai_heading_step_deg,
      pose.ai_goal,
      pose.ai_face,
      pose.ai_candidate_residual_deg,
      pose.ai_candidate_margin_deg,
      static_cast<unsigned long>(pose.ai_age_ms),
      pose.ai_reject,
      pose.dead_reckoning_distance_in,
      pose.position_error_envelope_in,
      static_cast<unsigned long>(pose.last_absolute_position_ms > 0
          ? pros::millis() - pose.last_absolute_position_ms
          : 0));
  fflush(stdout);
}

TestDriveBaseline capture_test_drive_baseline() {
  const double left_deg = average_motor_position(chassis.left_motors);
  const double right_deg = average_motor_position(chassis.right_motors);
  return TestDriveBaseline{
      std::isfinite(left_deg) ? left_deg : 0.0,
      std::isfinite(right_deg) ? right_deg : 0.0,
  };
}

int clamp_power(double value) {
  return static_cast<int>(clamp(value, -127.0, 127.0));
}

double apply_slew(double target, double current, double max_delta) {
  return clamp(target, current - max_delta, current + max_delta);
}

void log_drive_health(const char* phase) {
  const std::array<int, 4> ports{{17, 18, 11, 13}};
  const std::array<pros::Motor*, 4> motors{{
      &chassis.left_motors[0], &chassis.left_motors[1],
      &chassis.right_motors[0], &chassis.right_motors[1]}};
  std::printf(
      "DRIVE_POWER phase=%s battery_mv=%ld battery_ma=%ld capacity=%.1f\n",
      phase, static_cast<long>(pros::battery::get_voltage()),
      static_cast<long>(pros::battery::get_current()),
      pros::battery::get_capacity());
  for (std::size_t i = 0; i < motors.size(); ++i) {
    std::printf(
        "DRIVE_MOTOR phase=%s port=%d pos=%.2f velocity=%.2f current_ma=%ld "
        "command_mv=%ld temp_c=%.1f faults=%lu flags=%lu\n",
        phase, ports[i], motors[i]->get_position(),
        motors[i]->get_actual_velocity(),
        static_cast<long>(motors[i]->get_current_draw()),
        static_cast<long>(motors[i]->get_voltage()),
        motors[i]->get_temperature(),
        static_cast<unsigned long>(motors[i]->get_faults()),
        static_cast<unsigned long>(motors[i]->get_flags()));
  }
  std::fflush(stdout);
}

void set_physical_drive_power(int forward_power, int turn_power) {
  // Linearize the cancellation check and all physical writes against the
  // brake path. If stop already set the latch, refuse output; if it sets the
  // latch while this mutex is held, stop blocks briefly and brakes after this
  // complete write, so no stale write can occur after the brake.
  std::lock_guard<pros::Mutex> lock(drive_output_mutex);
  const bool caller_cancelled =
      navigation_stop_requested.load(std::memory_order_acquire);
  const bool field_disabled = pros::competition::is_connected() &&
                              pros::competition::is_disabled();
  if (caller_cancelled || field_disabled) {
    drive_emergency_hold_latched = true;
    stop_drive_motors_unlocked();
    return;
  }
  drive_emergency_hold_latched = false;
  // Positive field-math turn is CCW. On this drivetrain that requires the
  // left side forward and right side backward.
  const int left_power = clamp_power(forward_power + turn_power);
  const int right_power = clamp_power(forward_power - turn_power);
  for (auto& motor : chassis.left_motors) {
    motor.move(left_power);
  }
  for (auto& motor : chassis.right_motors) {
    motor.move(right_power);
  }
}

double forward_inches_since(const TestDriveBaseline& baseline) {
  const double left_deg = average_motor_position(chassis.left_motors);
  const double right_deg = average_motor_position(chassis.right_motors);
  if (!std::isfinite(left_deg) || !std::isfinite(right_deg)) return 0.0;
  const double left_in = ((left_deg - baseline.left_deg) / 360.0) * kWheelCircumferenceIn * kLeftEncoderSign;
  const double right_in = ((right_deg - baseline.right_deg) / 360.0) * kWheelCircumferenceIn * kRightEncoderSign;
  return (left_in + right_in) / 2.0;
}

double pose_heading_deg(const PoseEstimate& pose) {
  return normalize_deg(rad_to_deg(pose.heading_rad));
}

Waypoint project_field_point(Waypoint origin, double heading_deg, double distance_in) {
  const double heading_rad = deg_to_rad(normalize_deg(heading_deg));
  return Waypoint{
      origin.x + std::cos(heading_rad) * distance_in,
      origin.y + std::sin(heading_rad) * distance_in,
  };
}

void sample_fusion_for(PoseEstimate& pose,
                       const char* phase,
                       std::uint32_t duration_ms,
                       LidarFusionMode lidar_mode = LidarFusionMode::kBiasOnly) {
  const std::uint32_t start = pros::millis();
  do {
    update_pose(pose, lidar_mode);
    pros::delay(20);
  } while (pros::millis() - start < duration_ms);
  log_pose(phase, pose);
}

}  // namespace

void localization_telemetry_update() {
  // Zero encoder positions are a perfectly normal stationary state after a
  // route and after baselines are re-anchored. They must never erase a valid
  // LiDAR/AprilTag correction merely because the corrected field pose is more
  // than two inches from the entered start. Program startup and start-pose
  // edits already call the explicit reset path below.
  if (telemetry_pose_initialized &&
      pros::millis() < telemetry_pose.last_update_ms) {
    telemetry_pose_initialized = false;
    telemetry_last_log_ms = 0;
  }
  if (!telemetry_pose_initialized) {
    telemetry_pose = PoseEstimate{};
    init_pose(telemetry_pose, runtime_start_pose);
    telemetry_pose_initialized = true;
  }

  update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
  const std::uint32_t now = pros::millis();
  if (now - telemetry_last_log_ms >= 100) {
    log_pose("telemetry", telemetry_pose);
    telemetry_last_log_ms = now;
  }
}

void localization_telemetry_reset() {
  telemetry_pose_initialized = false;
  navigation_api_initialized = false;
  clear_telemetry_snapshot();
  clear_navigation_path_storage(pros::millis());
  telemetry_last_log_ms = 0;
}

bool localization_set_runtime_start_pose(double x_in,
                                         double y_in,
                                         double heading_deg) {
  const double wall = localization::kPhysicalWallHalfSpanIn;
  if (!std::isfinite(x_in) || !std::isfinite(y_in) ||
      !std::isfinite(heading_deg) || x_in < -wall || x_in > wall ||
      y_in < -wall || y_in > wall) {
    std::printf("POSE_ACK ok=0 reason=range x=%.3f y=%.3f heading=%.3f\n",
                x_in, y_in, heading_deg);
    std::fflush(stdout);
    return false;
  }

  // A start-pose edit changes the meaning of every in-flight field command.
  // Brake and latch cancellation before revoking/resetting its estimator so a
  // concurrent blocking command cannot continue toward an old-frame target.
  navigation_stop_requested.store(true, std::memory_order_release);
  stop_drive_motors();
  runtime_start_pose = {x_in, y_in, normalize_deg(heading_deg)};
  telemetry_pose_initialized = false;
  navigation_api_initialized = false;
  clear_telemetry_snapshot();
  clear_navigation_path_storage(pros::millis());
  telemetry_last_log_ms = 0;
  std::printf("POSE_ACK ok=1 x=%.3f y=%.3f heading=%.3f\n",
              runtime_start_pose.x_in, runtime_start_pose.y_in,
              runtime_start_pose.heading_deg);
  std::fflush(stdout);
  return true;
}

void localization_get_runtime_start_pose(double& x_in,
                                         double& y_in,
                                         double& heading_deg) {
  x_in = runtime_start_pose.x_in;
  y_in = runtime_start_pose.y_in;
  heading_deg = runtime_start_pose.heading_deg;
}

namespace {

bool fused_turn_to_heading(PoseEstimate& pose,
                           const char* phase,
                           double target_heading_deg,
                           int max_turn_power,
                           std::uint32_t timeout_ms,
                           LidarFusionMode lidar_mode =
                               LidarFusionMode::kBiasOnly);

bool fused_drive_to_point(PoseEstimate& pose,
                          const char* phase,
                          Waypoint target,
                          double final_heading_deg,
                          int max_forward_power,
                          std::uint32_t timeout_ms,
                          double max_cross_track_in =
                              std::numeric_limits<double>::infinity(),
                          int drive_direction = 1,
                          LidarFusionMode lidar_mode =
                              LidarFusionMode::kBiasOnly,
                          bool stop_for_forward_obstacle = true) {
  const std::uint32_t start = pros::millis();
  std::uint32_t last_loop_ms = start;
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;
  std::uint32_t last_progress_ms = start;
  double last_encoder_progress_in = 0.0;
  double forward_command = 0.0;
  double turn_command = 0.0;
  const double start_x = pose.x;
  const double start_y = pose.y;
  const double start_left_deg = pose.left_deg;
  const double start_right_deg = pose.right_deg;
  const double path_dx = target.x - start_x;
  const double path_dy = target.y - start_y;
  const double path_length = std::hypot(path_dx, path_dy);
  if (path_length <= kFusedDriveToleranceIn) return true;
  const double path_unit_x = path_dx / path_length;
  const double path_unit_y = path_dy / path_length;
  const double path_heading_deg = normalize_deg(
      std::atan2(path_unit_y, path_unit_x) * 180.0 / kPi);
  const double motion_sign = drive_direction < 0 ? -1.0 : 1.0;

  printf("FUSE_TEST command=%s type=fused_drive_to target_x=%.2f target_y=%.2f final_heading=%.2f max_power=%d direction=%s\n",
         phase,
         target.x,
         target.y,
         normalize_deg(final_heading_deg),
         max_forward_power,
         motion_sign > 0.0 ? "forward" : "reverse");
  fflush(stdout);

  while (pros::millis() - start < timeout_ms) {
    if (blocking_motion_abort_requested(phase)) return false;
    update_pose(pose, lidar_mode);
    if (motion_pose_invalid(pose, phase)) return false;
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);
    const double traveled_x = pose.x - start_x;
    const double traveled_y = pose.y - start_y;
    const double along_traveled =
        traveled_x * path_unit_x + traveled_y * path_unit_y;
    const double along_remaining = path_length - along_traveled;
    const double cross_track =
        path_unit_x * traveled_y - path_unit_y * traveled_x;
    if (std::isfinite(max_cross_track_in) &&
        std::fabs(cross_track) > max_cross_track_in) {
      emergency_stop_drive_motors();
      std::printf(
          "FUSE_TEST phase=%s abort=path_corridor cross=%.2f limit=%.2f\n",
          phase, cross_track, max_cross_track_in);
      std::fflush(stdout);
      return false;
    }
    const double heading_deg = pose_heading_deg(pose);
    const double cross_track_heading_deg = clamp(
        -rad_to_deg(std::atan2(cross_track,
                              kFusedDriveCrossTrackLookaheadIn)),
        -kFusedDriveMaxCrossTrackHeadingDeg,
        kFusedDriveMaxCrossTrackHeadingDeg);
    // For reverse travel, face opposite the desired travel tangent. Cross-
    // track correction is first computed in the direction of displacement,
    // then rotated 180 degrees into the chassis-heading command.
    const double bearing_deg = normalize_deg(
        path_heading_deg + cross_track_heading_deg +
        (motion_sign < 0.0 ? 180.0 : 0.0));
    const double bearing_error_deg = signed_angle_diff_deg(bearing_deg, heading_deg);
    const double final_heading_error_deg = signed_angle_diff_deg(final_heading_deg, heading_deg);

    // Use raw drive-encoder travel for the stall watchdog. Fused X/Y can move
    // due to a bounded absolute correction while the chassis is stationary;
    // that must never masquerade as mechanical progress.
    const double encoder_left_travel_in =
        ((pose.left_deg - start_left_deg) / 360.0) *
        kWheelCircumferenceIn * kLeftEncoderSign;
    const double encoder_right_travel_in =
        ((pose.right_deg - start_right_deg) / 360.0) *
        kWheelCircumferenceIn * kRightEncoderSign;
    const double encoder_forward_travel_in = motion_sign *
        0.5 * (encoder_left_travel_in + encoder_right_travel_in);

    if (encoder_forward_travel_in >=
        last_encoder_progress_in + kFusedDriveStallProgressIn) {
      last_encoder_progress_in = encoder_forward_travel_in;
      last_progress_ms = now;
    }

    const bool inside_finish_window =
        along_remaining <= kFusedDriveToleranceIn &&
        along_remaining >= -kFusedDriveToleranceIn &&
        std::fabs(cross_track) <= kFusedDriveMaxFinishCrossTrackIn;
    if (inside_finish_window) {
      // Brake as soon as the finish window is entered. The previous version
      // kept applying proportional forward power during the 80 ms settle
      // timer, which could turn a valid arrival into an avoidable overshoot.
      stop_drive_motors();
      forward_command = 0.0;
      turn_command = 0.0;
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >= static_cast<std::uint32_t>(kFusedDriveSettleMs)) {
        return true;
      }
      pros::delay(20);
      continue;
    } else {
      settled_since = 0;
    }

    // Once the robot crosses the finish plane, stop instead of turning around
    // and orbiting a noisy point estimate. A large cross-track miss is a
    // failed leg that the caller can handle safely.
    if (along_remaining < -kFusedDriveToleranceIn) {
      emergency_stop_drive_motors();
      printf("FUSE_TEST phase=%s abort=finish_plane_miss along=%.2f cross=%.2f\n",
             phase, along_remaining, cross_track);
      fflush(stdout);
      return false;
    }

    if (std::fabs(forward_command) >= kFusedDriveMinPower * 0.75 &&
        now - last_progress_ms >= kFusedDriveStallTimeoutMs) {
      log_drive_health("fused_drive_stall_commanded");
      emergency_stop_drive_motors();
      std::printf(
          "FUSE_TEST phase=%s abort=drive_stall encoder=%.2f along=%.2f "
          "command=%.1f\n",
          phase, encoder_forward_travel_in, along_traveled, forward_command);
      std::fflush(stdout);
      return false;
    }

    double target_forward_magnitude = clamp(
        std::max(0.0, along_remaining) * kFusedDriveKp,
        0.0,
        static_cast<double>(max_forward_power));
    // This chassis cannot overcome static friction below the measured 28/127
    // floor. Keep that floor through the edge of the existing arrival window;
    // otherwise a slow approach can park at 1.0-1.5 inches forever. The loop
    // brakes before commanding again as soon as it enters the 1-inch window.
    if (along_remaining > kFusedDriveToleranceIn &&
        target_forward_magnitude < kFusedDriveMinPower) {
      target_forward_magnitude = kFusedDriveMinPower;
    }
    const double heading_scale = clamp(std::cos(deg_to_rad(clamp(std::fabs(bearing_error_deg), 0.0, 80.0))),
                                       0.25,
                                       1.0);
    target_forward_magnitude *= heading_scale;
    if (std::fabs(bearing_error_deg) > 55.0) {
      target_forward_magnitude = std::min(target_forward_magnitude, 32.0);
    }
    const double target_forward = motion_sign * target_forward_magnitude;

    const double drive_heading_error_deg = bearing_error_deg;
    double target_turn =
        drive_heading_error_deg * kFusedDriveHeadingKp +
        final_heading_error_deg * kFusedDriveFinalHeadingKp;
    target_turn = clamp(target_turn, -kFusedDriveMaxTurnPower, kFusedDriveMaxTurnPower);

    forward_command = apply_slew(
        target_forward,
        forward_command,
        kFusedDriveForwardSlewPowerPerSec * dt_s);
    turn_command = apply_slew(
        target_turn,
        turn_command,
        kFusedDriveTurnSlewPowerPerSec * dt_s);
    // The robot is heavy enough that a low reverse command on the inner side
    // can hold it stationary while the outer side fights static friction. A
    // go-to-pose drive remains a forward curve: cap curvature at a stationary
    // inner wheel, then let the dedicated final turn settle exact heading.
    turn_command = clamp(turn_command, -std::fabs(forward_command),
                         std::fabs(forward_command));

    if (stop_for_forward_obstacle && motion_sign > 0.0 &&
        forward_command > 0.0 &&
        forward_obstacle_requires_stop(phase)) {
      return false;
    }

    set_physical_drive_power(clamp_power(forward_command), clamp_power(turn_command));

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf(
          "FUSE_TEST phase=%s controller=fused_drive dist=%.2f along=%.2f cross=%.2f bearing=%.2f bearing_err=%.2f final_err=%.2f fwd=%.1f turn=%.1f x=%.2f y=%.2f h=%.2f left_deg=%.2f right_deg=%.2f imu=%.2f gps_x=%.2f gps_y=%.2f gps_heading=%.2f gps_error=%.2f gps_reject=%s dr_travel=%.2f pos_envelope=%.2f lidar=%s reject=%s\n",
          phase,
          distance,
          along_remaining,
          cross_track,
          bearing_deg,
          bearing_error_deg,
          final_heading_error_deg,
          forward_command,
          turn_command,
          pose.x,
          pose.y,
          heading_deg,
          pose.left_deg,
          pose.right_deg,
          pose.imu_heading_deg,
          pose.gps_x_in,
          pose.gps_y_in,
          pose.gps_heading_deg,
          pose.gps_error_in,
          pose.gps_reject,
          pose.dead_reckoning_distance_in,
          pose.position_error_envelope_in,
          pose.lidar_wall,
          pose.lidar_reject);
      fflush(stdout);
      last_log = now;
    }

    pros::lcd::print(6, "%s d%.1f h%.0f", phase, distance, heading_deg);
    pros::delay(20);
  }

  stop_drive_motors();
  return false;
}

bool drive_forward_test_leg(PoseEstimate& pose,
                            const char* phase,
                            double target_in,
                            int max_power,
                            std::uint32_t timeout_ms);
bool turn_clockwise_test_leg(PoseEstimate& pose,
                             const char* phase,
                             double target_delta_cw_deg,
                             int turn_speed,
                             std::uint32_t timeout_ms);

void run_fused_rotation_health_test() {
  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  sample_fusion_for(pose, "fused_turn_start", 500,
                    LidarFusionMode::kDisabled);
  const double start_heading_deg = pose_heading_deg(pose);
  const double start_x = pose.x;
  const double start_y = pose.y;
  bool ok = fused_turn_to_heading(
      pose, "fused_turn_plus_45", start_heading_deg + 45.0, 70, 5000);
  sample_fusion_for(pose, "fused_turn_plus_hold", 500,
                    LidarFusionMode::kDisabled);
  if (ok) {
    ok = fused_turn_to_heading(
        pose, "fused_turn_return_1", start_heading_deg, 70, 5000);
  }
  sample_fusion_for(pose, "fused_turn_center_hold", 500,
                    LidarFusionMode::kDisabled);
  if (ok) {
    ok = fused_turn_to_heading(
        pose, "fused_turn_minus_45", start_heading_deg - 45.0, 70, 5000);
  }
  sample_fusion_for(pose, "fused_turn_minus_hold", 500,
                    LidarFusionMode::kDisabled);
  if (ok) {
    ok = fused_turn_to_heading(
        pose, "fused_turn_return_2", start_heading_deg, 70, 5000);
  }
  sample_fusion_for(pose, "fused_turn_final_hold", 1000,
                    LidarFusionMode::kDisabled);
  stop_drive_motors();

  const double position_residual_in = std::hypot(pose.x - start_x,
                                                  pose.y - start_y);
  const double heading_residual_deg = std::fabs(signed_angle_diff_deg(
      pose_heading_deg(pose), start_heading_deg));
  printf("FUSE_TEST route=fused_rotation_health_done ok=%d position_residual=%.3f heading_residual=%.3f gps_reject=%s\n",
         static_cast<int>(ok), position_residual_in, heading_residual_deg,
         pose.gps_reject);
  fflush(stdout);

  telemetry_pose = pose;
  telemetry_pose_initialized = true;
  publish_telemetry_snapshot();
  telemetry_last_log_ms = 0;
}

void run_fused_relative_motion_test() {
  constexpr double kLegDistanceIn = 12.0;
  constexpr int kDrivePower = 25;
  constexpr int kTurnPower = 25;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  // A controller-triggered test starts from the estimator that opcontrol has
  // already been updating. Do not discard valid stopped GPS/P6 corrections by
  // reinitializing from the configured fallback anchor at button press.
  PoseEstimate pose;
  if (telemetry_pose_initialized) {
    pose = telemetry_pose;
  } else {
    init_pose(pose, runtime_start_pose);
  }
  sample_fusion_for(pose, "relative_test_start_settle", 1000,
                    LidarFusionMode::kDisabled);
  const double start_x = pose.x;
  const double start_y = pose.y;
  const double start_heading_deg = pose_heading_deg(pose);

  pros::lcd::set_text(3, "FUSED TEST: 12/12/CW90");
  pros::lcd::print(4, "START X%5.1f Y%5.1f", start_x, start_y);
  pros::lcd::print(5, "START H%6.1f", start_heading_deg);
  pros::lcd::set_text(6, "Running...");

  bool ok = pose.ready && pose.imu_ready;
  std::printf(
      "FUSE_TEST route=relative_12in_90_start ok=%d x=%.2f y=%.2f "
      "heading=%.2f gps_reject=%s ai_id=%d ai_range=%.2f ai_reject=%s\n",
      static_cast<int>(ok), pose.x, pose.y, start_heading_deg,
      pose.gps_reject, pose.ai_tag_id, pose.ai_observed_range_in,
      pose.ai_reject);
  std::fflush(stdout);

  if (ok) {
    ok = drive_forward_test_leg(
        pose, "relative_reverse_12", -kLegDistanceIn, kDrivePower, 9000);
  }
  stop_drive_motors();
  sample_fusion_for(pose, "relative_reverse_settle", 1500,
                    LidarFusionMode::kDisabled);

  if (ok) {
    ok = drive_forward_test_leg(
        pose, "relative_forward_12", kLegDistanceIn, kDrivePower, 9000);
  }
  stop_drive_motors();
  sample_fusion_for(pose, "relative_forward_settle", 1500,
                    LidarFusionMode::kDisabled);

  if (ok) {
    ok = turn_clockwise_test_leg(
        pose, "relative_turn_cw_90", 90.0, kTurnPower, 9000);
  }
  stop_drive_motors();
  sample_fusion_for(pose, "relative_final_settle", 2000,
                    LidarFusionMode::kDisabled);

  const double position_delta_in = std::hypot(
      pose.x - start_x, pose.y - start_y);
  const double heading_delta_deg = signed_angle_diff_deg(
      pose_heading_deg(pose), start_heading_deg);
  pros::lcd::print(4, "S%4.0f,%4.0f H%3.0f", start_x, start_y,
                   start_heading_deg);
  pros::lcd::print(5, "E%4.0f,%4.0f H%3.0f", pose.x, pose.y,
                   pose_heading_deg(pose));
  pros::lcd::print(6, "%s d%.2f dh%.1f", ok ? "PASS" : "FAIL",
                   position_delta_in, heading_delta_deg);
  std::printf(
      "FUSE_TEST route=relative_12in_90_done ok=%d x=%.2f y=%.2f "
      "heading=%.2f position_delta=%.3f heading_delta=%.3f "
      "gps_reject=%s ai_id=%d ai_range=%.2f ai_reject=%s\n",
      static_cast<int>(ok), pose.x, pose.y, pose_heading_deg(pose),
      position_delta_in, heading_delta_deg, pose.gps_reject,
      pose.ai_tag_id, pose.ai_observed_range_in, pose.ai_reject);
  std::fflush(stdout);

  telemetry_pose = pose;
  telemetry_pose_initialized = true;
  publish_telemetry_snapshot();
  telemetry_last_log_ms = 0;
}

bool fused_turn_to_heading(PoseEstimate& pose,
                           const char* phase,
                           double target_heading_deg,
                           int max_turn_power,
                           std::uint32_t timeout_ms,
                           LidarFusionMode lidar_mode) {
  const std::uint32_t start = pros::millis();
  std::uint32_t last_loop_ms = start;
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;
  double turn_command = 0.0;
  double last_error_deg = signed_angle_diff_deg(target_heading_deg, pose_heading_deg(pose));
  double filtered_error_rate_deg_s = 0.0;
  double last_motion_heading_deg = pose_heading_deg(pose);
  std::uint32_t last_motion_ms = start;

  printf("FUSE_TEST command=%s type=fused_turn_to target_heading=%.2f max_power=%d\n",
         phase,
         normalize_deg(target_heading_deg),
         max_turn_power);
  fflush(stdout);

  while (pros::millis() - start < timeout_ms) {
    if (blocking_motion_abort_requested(phase)) return false;
    update_pose(pose, lidar_mode, false);
    if (motion_pose_invalid(pose, phase)) return false;
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;

    const double heading_deg = pose_heading_deg(pose);
    const double error_deg = signed_angle_diff_deg(target_heading_deg, heading_deg);
    const double raw_error_rate_deg_s =
        signed_angle_diff_deg(error_deg, last_error_deg) / dt_s;
    // Heading quantization makes a raw derivative noisy at the 20 ms loop
    // rate. Filtering keeps the damping term useful instead of alternating
    // between full acceleration and full braking.
    filtered_error_rate_deg_s +=
        0.30 * (raw_error_rate_deg_s - filtered_error_rate_deg_s);
    const double error_rate_deg_s = filtered_error_rate_deg_s;
    const bool crossed_target = error_deg * last_error_deg < 0.0;
    last_error_deg = error_deg;

    // Check the arrival window before reacting to a sign change. Around zero,
    // normal IMU quantization can alternate the error sign while the chassis
    // is already stopped. Treating every one of those samples as an overshoot
    // used to reset settled_since forever, causing an aligned turn to time out
    // and abort the rest of an autonomous route.
    if (std::fabs(error_deg) <= kFusedTurnToleranceDeg &&
        std::fabs(error_rate_deg_s) < kFusedTurnSettleRateDegPerSec) {
      stop_drive_motors();
      turn_command = 0.0;
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >=
                 static_cast<std::uint32_t>(kFusedTurnSettleMs)) {
        return true;
      }
      pros::delay(20);
      continue;
    }

    if (crossed_target &&
        std::fabs(error_deg) > kFusedTurnToleranceDeg) {
      // Do not slew for several cycles in the old direction after crossing.
      // Brake immediately, then let the next loop make a fresh correction.
      // Crossings wholly inside the arrival window are intentionally handled
      // above so sensor jitter cannot continuously reset the settle timer.
      stop_drive_motors();
      turn_command = 0.0;
      filtered_error_rate_deg_s = 0.0;
      settled_since = 0;
      pros::delay(20);
      continue;
    }

    if (std::fabs(signed_angle_diff_deg(heading_deg,
                                        last_motion_heading_deg)) >= 0.50) {
      last_motion_heading_deg = heading_deg;
      last_motion_ms = now;
    } else if (std::fabs(error_deg) > kFusedTurnToleranceDeg &&
               now - last_motion_ms >= kFusedTurnStallTimeoutMs) {
      log_drive_health("fused_turn_stall");
      printf("FUSE_TEST phase=%s abort=turn_stall error=%.2f heading=%.2f\n",
             phase, error_deg, heading_deg);
      fflush(stdout);
      emergency_stop_drive_motors();
      return false;
    }

    settled_since = 0;

    const double active_max_power =
        std::fabs(error_deg) <= kFusedTurnBrakeZoneDeg
            ? std::min(static_cast<double>(max_turn_power),
                       kFusedTurnBrakeZoneMaxPower)
            : static_cast<double>(max_turn_power);
    double target_turn =
        kFusedTurnKp * error_deg + kFusedTurnKd * error_rate_deg_s;
    target_turn = clamp(target_turn, -active_max_power, active_max_power);
    // Only overcome static friction when rotation has actually slowed down.
    // While approaching at speed, allowing a command below this floor gives
    // the derivative term room to brake before the target.
    if (std::fabs(error_deg) > kFusedTurnToleranceDeg &&
        std::fabs(error_rate_deg_s) < kFusedTurnStaticRateDegPerSec &&
        std::fabs(target_turn) < kFusedTurnMinPower) {
      target_turn = error_deg >= 0.0 ? kFusedTurnMinPower
                                     : -kFusedTurnMinPower;
    }
    // Surface/load orientation changes the heavy chassis breakaway threshold.
    // Preserve the lower normal floor, but step above the measured 24/127
    // breakaway point if P6 has seen no meaningful rotation for 250 ms.
    if (std::fabs(error_deg) > kFusedTurnToleranceDeg &&
        now - last_motion_ms >= kFusedTurnBreakawayDelayMs &&
        std::fabs(target_turn) < kFusedTurnBreakawayPower) {
      target_turn = error_deg >= 0.0 ? kFusedTurnBreakawayPower
                                     : -kFusedTurnBreakawayPower;
    }
    turn_command = apply_slew(target_turn, turn_command, kFusedTurnSlewPowerPerSec * dt_s);
    set_physical_drive_power(0, clamp_power(turn_command));

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf(
          "FUSE_TEST phase=%s controller=fused_turn target_heading=%.2f error=%.2f rate=%.2f turn=%.1f x=%.2f y=%.2f h=%.2f left_deg=%.2f right_deg=%.2f imu=%.2f gps_x=%.2f gps_y=%.2f gps_heading=%.2f gps_error=%.2f gps_reject=%s dr_travel=%.2f pos_envelope=%.2f lidar=%s reject=%s\n",
          phase,
          normalize_deg(target_heading_deg),
          error_deg,
          error_rate_deg_s,
          turn_command,
          pose.x,
          pose.y,
          heading_deg,
          pose.left_deg,
          pose.right_deg,
          pose.imu_heading_deg,
          pose.gps_x_in,
          pose.gps_y_in,
          pose.gps_heading_deg,
          pose.gps_error_in,
          pose.gps_reject,
          pose.dead_reckoning_distance_in,
          pose.position_error_envelope_in,
          pose.lidar_wall,
          pose.lidar_reject);
      fflush(stdout);
      last_log = now;
    }

    pros::lcd::print(6, "%s e%.1f h%.0f", phase, error_deg, heading_deg);
    pros::delay(20);
  }

  stop_drive_motors();
  return false;
}

bool drive_forward_test_leg(PoseEstimate& pose,
                            const char* phase,
                            double target_in,
                            int max_power,
                            std::uint32_t timeout_ms) {
  const TestDriveBaseline baseline = capture_test_drive_baseline();
  const double motion_sign = target_in >= 0.0 ? 1.0 : -1.0;
  const double target = std::fabs(target_in);
  const double hold_heading_deg = pose_heading_deg(pose);
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;

  printf("FUSE_TEST command=%s type=pros_encoder_bounded_drive inches=%.2f max_power=%d hold_heading=%.2f\n",
         phase,
         target,
         max_power,
         hold_heading_deg);
  fflush(stdout);

  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < timeout_ms) {
    update_pose(pose, LidarFusionMode::kDisabled);
    const double traveled_in =
        motion_sign * forward_inches_since(baseline);
    const double remaining_in = target - traveled_in;
    const std::uint32_t now = pros::millis();

    if (remaining_in <= kFusionTestDriveToleranceIn) {
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >= kFusionTestSettleMs) {
        stop_drive_motors();
        return true;
      }
    } else {
      settled_since = 0;
    }

    const double heading_error_deg = signed_angle_diff_deg(hold_heading_deg, pose_heading_deg(pose));
    const int forward_power = clamp_power(
        motion_sign * clamp(remaining_in * 8.0 + kFusionTestMinForwardPower,
                            static_cast<double>(kFusionTestMinForwardPower),
                            static_cast<double>(max_power)));
    const int turn_power =
        clamp_power(clamp(heading_error_deg * 1.4,
                          -static_cast<double>(kFusionTestMaxHeadingCorrection),
                          static_cast<double>(kFusionTestMaxHeadingCorrection)));
    if (motion_sign > 0.0 && forward_obstacle_requires_stop(phase)) {
      return false;
    }
    set_physical_drive_power(forward_power, turn_power);

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf(
          "FUSE_TEST phase=%s traveled=%.2f target=%.2f remaining=%.2f power=%d turn=%d x=%.2f y=%.2f h=%.2f imu=%.2f lidar=%s reject=%s\n",
          phase,
          traveled_in,
          target,
          remaining_in,
          forward_power,
          turn_power,
          pose.x,
          pose.y,
          pose_heading_deg(pose),
          chassis.drive_imu_get(),
          pose.lidar_wall,
          pose.lidar_reject);
      fflush(stdout);
      last_log = now;
    }

    pros::lcd::print(6, "%s %.1f/%.1f", phase, traveled_in, target);
    pros::delay(20);
  }

  stop_drive_motors();
  return false;
}

bool turn_clockwise_test_leg(PoseEstimate& pose,
                             const char* phase,
                             double target_delta_cw_deg,
                             int turn_speed,
                             std::uint32_t timeout_ms) {
  const double start_imu_cw_deg = chassis.drive_imu_get();
  const double target_imu_cw_deg = start_imu_cw_deg + target_delta_cw_deg;
  chassis.pid_targets_reset();
  const std::uint32_t start = pros::millis();
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;
  std::uint32_t last_loop_ms = start;
  double last_error_deg = target_delta_cw_deg;
  double last_command_cw = 0.0;

  while (pros::millis() - start < timeout_ms) {
    update_pose(pose, LidarFusionMode::kDisabled);
    const double imu_cw_deg = chassis.drive_imu_get();
    const double error_deg = std::isfinite(imu_cw_deg) ? target_imu_cw_deg - imu_cw_deg : 0.0;
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;
    const double error_rate_deg_s = (error_deg - last_error_deg) / dt_s;
    last_error_deg = error_deg;

    if (std::fabs(error_deg) <= kFusionTestTurnToleranceDeg) {
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >= kFusionTestSettleMs) {
        stop_drive_motors();
        return true;
      }
    } else {
      settled_since = 0;
    }

    double command_cw = kFusionTestTurnKp * error_deg + kFusionTestTurnKd * error_rate_deg_s;
    command_cw = clamp(command_cw, -static_cast<double>(turn_speed), static_cast<double>(turn_speed));
    if (std::fabs(error_deg) > kFusionTestTurnMinPowerErrorDeg &&
        std::fabs(command_cw) < kFusionTestTurnMinPower) {
      command_cw = command_cw >= 0.0 ? kFusionTestTurnMinPower : -kFusionTestTurnMinPower;
    }
    const double max_step = kFusionTestTurnSlewPowerPerSec * dt_s;
    command_cw = clamp(command_cw, last_command_cw - max_step, last_command_cw + max_step);
    last_command_cw = command_cw;

    const int signed_turn_power = clamp_power(-command_cw);
    set_physical_drive_power(0, signed_turn_power);

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf("FUSE_TEST phase=%s imu_cw=%.2f target=%.2f error=%.2f rate=%.2f command_cw=%.2f power=%d x=%.2f y=%.2f h=%.2f bias=%.2f lidar=%s reject=%s\n",
             phase,
             imu_cw_deg,
             target_imu_cw_deg,
             error_deg,
             error_rate_deg_s,
             command_cw,
             signed_turn_power,
             pose.x,
             pose.y,
             normalize_deg(rad_to_deg(pose.heading_rad)),
             pose.imu_bias_deg,
             pose.lidar_wall,
             pose.lidar_reject);
      fflush(stdout);
      last_log = now;
    }

    pros::lcd::print(6, "turn %.1f/%.1f", imu_cw_deg - start_imu_cw_deg, target_delta_cw_deg);
    pros::delay(20);
  }

  stop_drive_motors();
  return false;
}

bool drive_to_field_target(PoseEstimate& pose,
                           const char* phase,
                           Waypoint target,
                           double planned_segment_in,
                           int max_power,
                           std::uint32_t timeout_ms) {
  update_pose(pose);
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double target_distance_in = std::hypot(dx, dy);
  const double commanded_distance_in = std::min(target_distance_in, planned_segment_in);
  const double bearing_deg = normalize_deg(std::atan2(dy, dx) * 180.0 / kPi);
  const double heading_error_deg = signed_angle_diff_deg(bearing_deg, pose_heading_deg(pose));

  printf(
      "FUSE_TEST command=%s type=field_drive_to target_x=%.2f target_y=%.2f from_x=%.2f from_y=%.2f bearing=%.2f heading_error=%.2f pose_distance=%.2f commanded_inches=%.2f max_power=%d\n",
      phase,
      target.x,
      target.y,
      pose.x,
      pose.y,
      bearing_deg,
      heading_error_deg,
      target_distance_in,
      commanded_distance_in,
      max_power);
  fflush(stdout);

  if (commanded_distance_in < 0.25) return true;
  return drive_forward_test_leg(pose, phase, commanded_distance_in, max_power, timeout_ms);
}

bool turn_to_field_heading(PoseEstimate& pose,
                           const char* phase,
                           double target_field_heading_deg,
                           int turn_speed,
                           std::uint32_t timeout_ms) {
  update_pose(pose);
  const double current_field_heading_deg = pose_heading_deg(pose);
  const double field_error_ccw_deg =
      signed_angle_diff_deg(target_field_heading_deg, current_field_heading_deg);
  const double target_delta_cw_deg = -field_error_ccw_deg;

  printf(
      "FUSE_TEST command=%s type=field_turn_to target_heading=%.2f current_heading=%.2f delta_cw=%.2f speed=%d\n",
      phase,
      normalize_deg(target_field_heading_deg),
      current_field_heading_deg,
      target_delta_cw_deg,
      turn_speed);
  fflush(stdout);

  if (std::fabs(target_delta_cw_deg) < 0.5) return true;
  return turn_clockwise_test_leg(pose, phase, target_delta_cw_deg, turn_speed, timeout_ms);
}

bool drive_to_point(PoseEstimate& pose, Waypoint target, int max_speed, std::uint32_t timeout_ms) {
  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < timeout_ms) {
    update_pose(pose);
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 2.0) {
      stop_drive_motors();
      return true;
    }

    const double desired_heading_deg = std::atan2(dy, dx) * 180.0 / kPi;
    const double heading_deg = rad_to_deg(pose.heading_rad);
    const double heading_error = signed_angle_diff_deg(desired_heading_deg, heading_deg);

    double drive = clamp(distance * 4.0, -static_cast<double>(max_speed), static_cast<double>(max_speed));
    if (std::fabs(heading_error) > 45.0) drive *= 0.35;
    const double turn = clamp(-heading_error * 1.2, -60.0, 60.0);

    const int left = static_cast<int>(clamp(drive + turn, -max_speed, max_speed));
    const int right = static_cast<int>(clamp(drive - turn, -max_speed, max_speed));
    chassis.drive_set(left, right);
    pros::lcd::print(6, "x%3.0f y%3.0f h%3.0f b%+.1f", pose.x, pose.y, heading_deg, pose.imu_bias_deg);
    pros::delay(20);
  }

  stop_drive_motors();
  return false;
}
}  // namespace

void localization_fused_rotation_health_test() {
  run_fused_rotation_health_test();
}

void localization_fused_boomerang_test() {
  constexpr double kRelativeForwardIn = 12.0;
  constexpr double kRelativeLeftIn = 12.0;
  constexpr int kMaximumPower = 60;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  sample_fusion_for(pose, "boomerang_start_settle", 1000,
                    LidarFusionMode::kDisabled);
  const double start_x = pose.x;
  const double start_y = pose.y;
  const double start_heading_deg = pose_heading_deg(pose);
  const double start_heading_rad = deg_to_rad(start_heading_deg);
  const Waypoint target{
      start_x + kRelativeForwardIn * std::cos(start_heading_rad) -
          kRelativeLeftIn * std::sin(start_heading_rad),
      start_y + kRelativeForwardIn * std::sin(start_heading_rad) +
          kRelativeLeftIn * std::cos(start_heading_rad),
  };
  const double target_heading_deg = normalize_deg(start_heading_deg + 45.0);

  std::printf(
      "FUSE_TEST route=relative_boomerang_12_12_start x=%.3f y=%.3f "
      "heading=%.3f target_x=%.3f target_y=%.3f target_heading=%.3f\n",
      start_x, start_y, start_heading_deg, target.x, target.y,
      target_heading_deg);
  std::fflush(stdout);
  const std::uint32_t motion_started_ms = pros::millis();
  const bool drive_ok = fused_drive_to_point(
      pose, "relative_boomerang_12_12", target, target_heading_deg,
      kMaximumPower, 8000);
  const bool heading_ok = drive_ok && fused_turn_to_heading(
      pose, "relative_boomerang_final_heading", target_heading_deg,
      kMaximumPower, 3000);
  const bool ok = drive_ok && heading_ok;
  const std::uint32_t motion_elapsed_ms = pros::millis() - motion_started_ms;
  stop_drive_motors();

  const double endpoint_x = pose.x;
  const double endpoint_y = pose.y;
  const double endpoint_heading_deg = pose_heading_deg(pose);
  const double field_dx = endpoint_x - start_x;
  const double field_dy = endpoint_y - start_y;
  const double achieved_forward_in =
      field_dx * std::cos(start_heading_rad) +
      field_dy * std::sin(start_heading_rad);
  const double achieved_left_in =
      -field_dx * std::sin(start_heading_rad) +
      field_dy * std::cos(start_heading_rad);
  const double endpoint_error_in =
      std::hypot(endpoint_x - target.x, endpoint_y - target.y);
  const double heading_error_deg =
      signed_angle_diff_deg(target_heading_deg, endpoint_heading_deg);

  std::printf(
      "FUSE_TEST route=relative_boomerang_12_12_done ok=%d drive_ok=%d "
      "heading_ok=%d elapsed_ms=%lu "
      "forward=%.3f left=%.3f endpoint_error=%.3f heading=%.3f "
      "heading_error=%.3f x=%.3f y=%.3f\n",
      static_cast<int>(ok), static_cast<int>(drive_ok),
      static_cast<int>(heading_ok),
      static_cast<unsigned long>(motion_elapsed_ms),
      achieved_forward_in, achieved_left_in, endpoint_error_in,
      endpoint_heading_deg, heading_error_deg, endpoint_x, endpoint_y);
  std::fflush(stdout);

  sample_fusion_for(pose, "boomerang_final_settle", 1000,
                    LidarFusionMode::kDisabled);
  telemetry_pose = pose;
  telemetry_pose_initialized = true;
  publish_telemetry_snapshot();
  telemetry_last_log_ms = 0;
}

void localization_fused_relative_motion_test() {
  run_fused_relative_motion_test();
}

namespace navigation {

namespace {

double point_to_segment_distance(Waypoint point, Waypoint a, Waypoint b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-9) return std::hypot(point.x - a.x, point.y - a.y);
  const double t = clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_sq,
      0.0,
      1.0);
  return std::hypot(point.x - (a.x + t * dx),
                    point.y - (a.y + t * dy));
}

double projected_position_error_envelope_in(double current_envelope_in,
                                            double additional_travel_in) {
  if (!std::isfinite(current_envelope_in) || current_envelope_in < 0.0 ||
      !std::isfinite(additional_travel_in) || additional_travel_in < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double per_inch_growth = std::hypot(
      localization::kDeadReckoningScaleEnvelopeFraction,
      std::tan(deg_to_rad(
          localization::kDeadReckoningHeadingEnvelopeDeg)));
  return current_envelope_in + additional_travel_in * per_inch_growth;
}

bool public_straight_segment_is_safe(Waypoint start,
                                     Waypoint target,
                                     double position_error_envelope_in,
                                     double& minimum_goal_clearance_in,
                                     double& required_goal_clearance_in,
                                     const char*& reject_reason,
                                     bool allow_goal_contact = false,
                                     bool allow_wall_proximity = false) {
  if (!std::isfinite(position_error_envelope_in) ||
      position_error_envelope_in < 0.0) {
    reject_reason = "pose_uncertainty";
    return false;
  }
  const double center_limit =
      localization::kPhysicalWallHalfSpanIn -
      localization::kNavigationProvisionalWallClearanceIn -
      localization::kNavigationFieldElementToleranceIn -
      position_error_envelope_in;
  if (!allow_wall_proximity &&
      (std::fabs(target.x) > center_limit ||
       std::fabs(target.y) > center_limit)) {
    reject_reason = "wall_clearance";
    return false;
  }

  double start_goal_clearance_in =
      std::numeric_limits<double>::infinity();
  minimum_goal_clearance_in = std::numeric_limits<double>::infinity();
  for (const auto& goal : localization::kGoalTagLandmarks) {
    const Waypoint center{goal.x_in, goal.y_in};
    start_goal_clearance_in = std::min(
        start_goal_clearance_in,
        std::hypot(center.x - start.x, center.y - start.y));
    minimum_goal_clearance_in = std::min(
        minimum_goal_clearance_in,
        point_to_segment_distance(center, start, target));
  }
  // A legal start can already be closer than the normal route clearance.
  // Permit an outward escape without demanding that the start point itself
  // satisfy a larger radius, while retaining a 12-in fail-closed floor.
  const double normal_goal_clearance_in =
      localization::kNavigationProvisionalGoalClearanceIn +
      localization::kNavigationFieldElementToleranceIn +
      position_error_envelope_in;
  const double minimum_escape_clearance_in =
      localization::kNavigationMinimumGoalEscapeClearanceIn +
      localization::kNavigationFieldElementToleranceIn +
      position_error_envelope_in;
  required_goal_clearance_in = std::min(
      normal_goal_clearance_in,
      std::max(minimum_escape_clearance_in,
               start_goal_clearance_in - 0.5));
  if (!allow_goal_contact &&
      minimum_goal_clearance_in < required_goal_clearance_in) {
    reject_reason = "goal_clearance";
    return false;
  }
  reject_reason = "none";
  return true;
}

bool public_turn_center_is_safe(Waypoint center,
                                double position_error_envelope_in,
                                double& minimum_goal_clearance_in,
                                const char*& reject_reason,
                                bool allow_goal_contact = false,
                                bool allow_wall_proximity = false) {
  if (!std::isfinite(position_error_envelope_in) ||
      position_error_envelope_in < 0.0) {
    reject_reason = "turn_pose_uncertainty";
    return false;
  }
  const double center_limit =
      localization::kPhysicalWallHalfSpanIn -
      localization::kNavigationProvisionalWallClearanceIn -
      localization::kNavigationFieldElementToleranceIn -
      position_error_envelope_in;
  if (!allow_wall_proximity &&
      (std::fabs(center.x) > center_limit ||
       std::fabs(center.y) > center_limit)) {
    minimum_goal_clearance_in = std::numeric_limits<double>::infinity();
    reject_reason = "turn_wall_clearance";
    return false;
  }

  minimum_goal_clearance_in = std::numeric_limits<double>::infinity();
  for (const auto& goal : localization::kGoalTagLandmarks) {
    minimum_goal_clearance_in = std::min(
        minimum_goal_clearance_in,
        std::hypot(goal.x_in - center.x, goal.y_in - center.y));
  }
  if (!allow_goal_contact && minimum_goal_clearance_in <
      localization::kNavigationProvisionalGoalClearanceIn +
          localization::kNavigationFieldElementToleranceIn +
          position_error_envelope_in) {
    reject_reason = "turn_goal_clearance";
    return false;
  }
  reject_reason = "none";
  return true;
}

bool stationary_for_navigation_init() {
  stop_drive_motors();
  const MotorSideReading left_start = read_motor_side(chassis.left_motors);
  const MotorSideReading right_start = read_motor_side(chassis.right_motors);
  const bool imu_start_ok = chassis.imu.is_installed() &&
                            !chassis.imu.is_calibrating() &&
                            chassis.imu.get_status() != pros::ImuStatus::error;
  const double imu_start_deg = imu_start_ok ? chassis.drive_imu_get() : NAN;
  if (!left_start.trustworthy || !right_start.trustworthy ||
      !std::isfinite(imu_start_deg)) {
    std::printf("NAV_INIT reject=sensor_precheck drive=L%d/R%d imu=%d\n",
                static_cast<int>(left_start.trustworthy),
                static_cast<int>(right_start.trustworthy),
                static_cast<int>(std::isfinite(imu_start_deg)));
    std::fflush(stdout);
    return false;
  }

  pros::delay(kNavigationInitSettleMs);
  const MotorSideReading left_end = read_motor_side(chassis.left_motors);
  const MotorSideReading right_end = read_motor_side(chassis.right_motors);
  const bool imu_end_ok = chassis.imu.is_installed() &&
                          !chassis.imu.is_calibrating() &&
                          chassis.imu.get_status() != pros::ImuStatus::error;
  const double imu_end_deg = imu_end_ok ? chassis.drive_imu_get() : NAN;
  if (!left_end.trustworthy || !right_end.trustworthy ||
      !std::isfinite(imu_end_deg)) {
    std::printf("NAV_INIT reject=sensor_settle drive=L%d/R%d imu=%d\n",
                static_cast<int>(left_end.trustworthy),
                static_cast<int>(right_end.trustworthy),
                static_cast<int>(std::isfinite(imu_end_deg)));
    std::fflush(stdout);
    return false;
  }

  const double left_motion_in = std::fabs(
      (left_end.position_deg - left_start.position_deg) / 360.0 *
      kWheelCircumferenceIn);
  const double right_motion_in = std::fabs(
      (right_end.position_deg - right_start.position_deg) / 360.0 *
      kWheelCircumferenceIn);
  const double imu_motion_deg = std::fabs(
      signed_angle_diff_deg(imu_end_deg, imu_start_deg));
  const bool stationary =
      left_motion_in <= kNavigationInitMaxWheelMotionIn &&
      right_motion_in <= kNavigationInitMaxWheelMotionIn &&
      imu_motion_deg <= kNavigationInitMaxImuMotionDeg;
  if (!stationary) {
    std::printf(
        "NAV_INIT reject=moving left=%.3f right=%.3f imu=%.3f settle_ms=%lu\n",
        left_motion_in,
        right_motion_in,
        imu_motion_deg,
        static_cast<unsigned long>(kNavigationInitSettleMs));
    std::fflush(stdout);
  }
  return stationary;
}

}  // namespace

bool init(double start_x_in,
          double start_y_in,
          double start_heading_deg) {
  return init(start_x_in,
              start_y_in,
              start_heading_deg,
              localization::kNavigationDefaultStartPositionErrorIn);
}

bool init(double start_x_in,
          double start_y_in,
          double start_heading_deg,
          double start_position_error_in) {
  navigation_api_initialized = false;
  publish_telemetry_snapshot();
  const double wall = localization::kPhysicalWallHalfSpanIn;
  if (!std::isfinite(start_x_in) || !std::isfinite(start_y_in) ||
      !std::isfinite(start_heading_deg) || start_x_in < -wall ||
      start_x_in > wall || start_y_in < -wall || start_y_in > wall ||
      !std::isfinite(start_position_error_in) ||
      start_position_error_in < 0.0 || start_position_error_in > wall) {
    return false;
  }
  // Mechanism motion or a freshly released chassis can perturb one 250 ms
  // sample. Require one clean window across three bounded attempts instead of
  // silently cancelling the whole autonomous on the first transient.
  bool stationary = false;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (stationary_for_navigation_init()) {
      stationary = true;
      break;
    }
    std::printf("NAV_INIT retry=%d/3\n", attempt);
    std::fflush(stdout);
    pros::delay(100);
  }
  if (!stationary) {
    pros::lcd::set_text(6, "AUTON FAIL: NAV SENSORS");
    std::printf("NAV_INIT abort=stationary_preflight_failed\n");
    std::fflush(stdout);
    return false;
  }
  if (!localization_set_runtime_start_pose(
          start_x_in, start_y_in, start_heading_deg)) {
    pros::lcd::set_text(6, "AUTON FAIL: START POSE");
    std::printf("NAV_INIT abort=start_pose_rejected\n");
    std::fflush(stdout);
    return false;
  }
  localization_telemetry_reset();
  localization_telemetry_update();
  // The coordinate is an anchor, not perfect ground truth. Carry the caller's
  // placement bound into every public map check until a validated absolute
  // position update establishes a different conservative base error.
  telemetry_pose.absolute_position_base_error_in = start_position_error_in;
  telemetry_pose.dead_reckoning_distance_in = 0.0;
  update_position_error_envelope(telemetry_pose);
  navigation_api_initialized = telemetry_pose_initialized &&
                               telemetry_pose.ready &&
                               telemetry_pose.imu_ready;
  publish_telemetry_snapshot();
  if (navigation_api_initialized) {
    reset_navigation_path(telemetry_pose, pros::millis());
  } else {
    pros::lcd::set_text(6, "AUTON FAIL: POSE/IMU");
    std::printf("NAV_INIT abort=estimator_not_ready pose=%d imu=%d\n",
                static_cast<int>(telemetry_pose.ready),
                static_cast<int>(telemetry_pose.imu_ready));
    std::fflush(stdout);
  }
  return navigation_api_initialized;
}

void update() {
  if (!navigation_api_initialized) return;
  localization_telemetry_update();
}

Pose current_pose() {
  PoseEstimate snapshot;
  bool navigation_initialized = false;
  if (!copy_telemetry_snapshot(snapshot, navigation_initialized) ||
      !navigation_initialized) {
    return {};
  }
  const std::uint32_t estimator_age_ms =
      pros::millis() - snapshot.last_update_ms;
  return Pose{
      snapshot.x,
      snapshot.y,
      pose_heading_deg(snapshot),
      snapshot.dead_reckoning_distance_in,
      snapshot.position_error_envelope_in,
      snapshot.last_absolute_position_ms > 0
          ? pros::millis() - snapshot.last_absolute_position_ms
          : 0,
      snapshot.ready && snapshot.imu_ready &&
          estimator_age_ms <=
              localization::kNavigationSnapshotMaxAgeMs,
      estimator_age_ms,
  };
}

SensorHealth sensor_health() {
  SensorHealth health;
  const ForwardObstacleObservation forward = read_forward_obstacle();
  const auto& vision = ai_vision_shadow_snapshot();
  const MotorSideReading left = read_motor_side(chassis.left_motors);
  const MotorSideReading right = read_motor_side(chassis.right_motors);
  health.drive_encoders_valid = left.trustworthy && right.trustworthy;
  health.imu_valid = chassis.imu.is_installed() &&
                     !chassis.imu.is_calibrating() &&
                     chassis.imu.get_status() != pros::ImuStatus::error;
  health.gps_installed = gps_7.is_installed();
  const double gps_gyro_z =
      health.gps_installed ? gps_7.get_gyro_rate_z() : NAN;
  // PROS returns +infinity (PROS_ERR_F) while the GPS is absent, calibrating,
  // or otherwise unavailable. Never expose that sentinel as a real rate.
  health.gps_gyro_valid = std::isfinite(gps_gyro_z);
  health.gps_gyro_z = health.gps_gyro_valid ? gps_gyro_z : 0.0;
  health.forward_distance_installed = forward.installed;
  health.forward_distance_api_ok = forward.api_ok;
  health.forward_distance_valid = forward.valid;
  health.forward_distance_in =
      forward.valid ? static_cast<double>(forward.distance_mm) / 25.4 : 0.0;
  health.forward_distance_confidence = forward.confidence;
  health.ai_vision_installed = vision.installed;
  health.ai_tag_visible = vision.tag_valid;
  health.ai_tag_id = vision.tag_id;
  health.ai_horizontal_range_in = vision.horizontal_range_in;
  health.ai_3d_range_in = vision.range_estimate_in;
  health.ai_bearing_right_deg = vision.bearing_deg;
  health.ai_elevation_deg = vision.elevation_deg;
  health.ai_geometry_age_ms = vision.geometry_age_ms;
  health.ai_state = vision.reason;
  health.lateral_tracker_enabled = localization::kSideOdomEnabled;

  // Raw sensor preflight is intentionally available before init, but a health
  // query must not create/reset a fused estimator or grant motion authority.
  PoseEstimate snapshot;
  bool navigation_initialized = false;
  if (!copy_telemetry_snapshot(snapshot, navigation_initialized)) return health;
  const std::uint32_t estimator_age_ms =
      pros::millis() - snapshot.last_update_ms;
  const bool estimator_fresh =
      estimator_age_ms <= localization::kNavigationSnapshotMaxAgeMs;

  const bool gps_accepted =
      std::strcmp(snapshot.gps_reject, "corrected") == 0 ||
      std::strcmp(snapshot.gps_reject, "position_only") == 0 ||
      std::strcmp(snapshot.gps_reject,
                  "heading_innovation_position_only") == 0;
  health.pose_valid = navigation_initialized && estimator_fresh && snapshot.ready &&
                      snapshot.imu_ready;
  // Do not overwrite a just-observed hardware failure with an older complete
  // estimator frame. Recovery remains latched by the snapshot until init, but
  // loss is visible immediately through the conjunction.
  health.drive_encoders_valid =
      health.drive_encoders_valid && snapshot.ready;
  health.imu_valid = health.imu_valid && snapshot.imu_ready;
  health.gps_fix_accepted = estimator_fresh && gps_accepted;
  health.gps_reported_error_in = snapshot.gps_error_in;
  health.gps_state = estimator_fresh ? snapshot.gps_reject : "estimator_stale";
  if (estimator_fresh) health.ai_state = snapshot.ai_reject;
  health.dead_reckoning_distance_in =
      snapshot.dead_reckoning_distance_in;
  health.position_error_envelope_in =
      snapshot.position_error_envelope_in;
  health.absolute_position_age_ms =
      snapshot.last_absolute_position_ms > 0
          ? pros::millis() - snapshot.last_absolute_position_ms
          : 0;
  health.estimator_age_ms = estimator_age_ms;
  return health;
}

std::size_t copy_path(PathPoint* output, std::size_t capacity) {
  if (output == nullptr || capacity == 0) return 0;
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  const std::size_t write_count = std::min(capacity, navigation_path_count);
  // When the caller requests fewer points than retained, return the newest
  // suffix while preserving chronological order.
  const std::size_t skip = navigation_path_count - write_count;
  for (std::size_t i = 0; i < write_count; ++i) {
    const std::size_t index =
        (navigation_path_start + skip + i) % navigation::kPathCapacity;
    output[i] = navigation_path[index];
  }
  return write_count;
}

std::size_t path_size() {
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  return navigation_path_count;
}

void clear_path() {
  PoseEstimate snapshot;
  bool navigation_initialized = false;
  const bool have_snapshot =
      copy_telemetry_snapshot(snapshot, navigation_initialized);
  std::lock_guard<pros::Mutex> lock(navigation_path_mutex);
  navigation_path_start = 0;
  navigation_path_count = 0;
  navigation_path_session_start_ms = pros::millis();
  navigation_path_last_record_ms = navigation_path_session_start_ms;
  if (have_snapshot && navigation_initialized && snapshot.ready &&
      snapshot.imu_ready) {
    reset_navigation_path_unlocked(
        snapshot, navigation_path_session_start_ms);
  }
}

Result turn_to(double heading_deg,
               int max_power,
               std::uint32_t timeout_ms,
               bool allow_goal_contact,
               bool allow_wall_proximity) {
  if (!std::isfinite(heading_deg) || max_power <= 0 || timeout_ms < 100) {
    return Result::kInvalidArgument;
  }
  if (!navigation_api_initialized || !telemetry_pose_initialized) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }
  // This atomic store is the command's cancellation linearization point: an
  // older stop is superseded by the explicit new call, while any stop after
  // this point remains latched through preflight and motion.
  navigation_stop_requested.store(false, std::memory_order_release);
  localization_telemetry_update();
  if (!telemetry_pose.ready || !telemetry_pose.imu_ready) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }
  const double heading_error_deg = std::fabs(signed_angle_diff_deg(
      normalize_deg(heading_deg), pose_heading_deg(telemetry_pose)));
  double minimum_goal_clearance_in = 0.0;
  const char* turn_reject = "none";
  if (heading_error_deg > kFusedTurnToleranceDeg &&
      !public_turn_center_is_safe(
          Waypoint{telemetry_pose.x, telemetry_pose.y},
          telemetry_pose.position_error_envelope_in,
          minimum_goal_clearance_in,
          turn_reject,
          allow_goal_contact,
          allow_wall_proximity)) {
    stop_drive_motors();
    std::printf(
        "NAV_TURN reject=%s at=%.2f,%.2f heading_error=%.2f "
        "goal_clearance=%.2f pose_envelope=%.2f\n",
        turn_reject,
        telemetry_pose.x,
        telemetry_pose.y,
        heading_error_deg,
        minimum_goal_clearance_in,
        telemetry_pose.position_error_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }
  chassis.drive_mode_set(ez::DISABLE, true);
  const bool ok = fused_turn_to_heading(
      telemetry_pose,
      "navigation_turn",
      normalize_deg(heading_deg),
      std::clamp(max_power, 20, 127),
      timeout_ms);
  stop_drive_motors();
  telemetry_last_log_ms = 0;
  return ok ? Result::kSuccess : Result::kTurnFailed;
}

Result go_straight_to(double target_x_in,
                      double target_y_in,
                      int max_power,
                      std::uint32_t timeout_ms) {
  const double wall = localization::kPhysicalWallHalfSpanIn;
  if (!std::isfinite(target_x_in) || !std::isfinite(target_y_in) ||
      target_x_in < -wall || target_x_in > wall ||
      target_y_in < -wall || target_y_in > wall || max_power <= 0 ||
      timeout_ms < 100) {
    return Result::kInvalidArgument;
  }
  if (!navigation_api_initialized || !telemetry_pose_initialized) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }
  // Clear an older cancellation before any potentially lengthy pose/map
  // preflight. A concurrent stop after this store can no longer be overwritten.
  navigation_stop_requested.store(false, std::memory_order_release);
  localization_telemetry_update();
  if (!telemetry_pose.ready || !telemetry_pose.imu_ready) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }

  const double dx = target_x_in - telemetry_pose.x;
  const double dy = target_y_in - telemetry_pose.y;
  const double requested_path_length_in = std::hypot(dx, dy);
  if (requested_path_length_in <= kFusedDriveToleranceIn) {
    stop_drive_motors();
    return Result::kSuccess;
  }
  const double projected_path_error_envelope_in =
      projected_position_error_envelope_in(
          telemetry_pose.position_error_envelope_in,
          requested_path_length_in);
  double minimum_goal_clearance_in = 0.0;
  double required_goal_clearance_in = 0.0;
  const char* path_reject = "none";
  if (!public_straight_segment_is_safe(
          Waypoint{telemetry_pose.x, telemetry_pose.y},
          Waypoint{target_x_in, target_y_in},
          projected_path_error_envelope_in,
          minimum_goal_clearance_in,
          required_goal_clearance_in,
          path_reject)) {
    stop_drive_motors();
    std::printf(
        "NAV_PATH reject=%s from=%.2f,%.2f target=%.2f,%.2f "
        "goal_clearance=%.2f required=%.2f projected_envelope=%.2f\n",
        path_reject,
        telemetry_pose.x,
        telemetry_pose.y,
        target_x_in,
        target_y_in,
        minimum_goal_clearance_in,
        required_goal_clearance_in,
        projected_path_error_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }
  const double bearing_deg = normalize_deg(rad_to_deg(std::atan2(dy, dx)));
  const double initial_turn_error_deg = std::fabs(signed_angle_diff_deg(
      bearing_deg, pose_heading_deg(telemetry_pose)));
  double turn_goal_clearance_in = 0.0;
  const char* turn_reject = "none";
  if (initial_turn_error_deg > kFusedTurnToleranceDeg &&
      !public_turn_center_is_safe(
          Waypoint{telemetry_pose.x, telemetry_pose.y},
          telemetry_pose.position_error_envelope_in,
          turn_goal_clearance_in,
          turn_reject)) {
    stop_drive_motors();
    std::printf(
        "NAV_PATH reject=%s from=%.2f,%.2f target=%.2f,%.2f "
        "turn_error=%.2f goal_clearance=%.2f pose_envelope=%.2f\n",
        turn_reject,
        telemetry_pose.x,
        telemetry_pose.y,
        target_x_in,
        target_y_in,
        initial_turn_error_deg,
        turn_goal_clearance_in,
        telemetry_pose.position_error_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }
  const std::uint32_t command_started_ms = pros::millis();
  chassis.drive_mode_set(ez::DISABLE, true);
  const bool turned = fused_turn_to_heading(
      telemetry_pose,
      "navigation_go_to_turn",
      bearing_deg,
      std::clamp(max_power, 20, 45),
      std::min<std::uint32_t>(timeout_ms, 6000));
  if (!turned) {
    stop_drive_motors();
    telemetry_last_log_ms = 0;
    return Result::kTurnFailed;
  }

  // timeout_ms is one end-to-end command deadline, not a separate allowance
  // for both phases. The prior implementation could consume up to six extra
  // seconds turning and then grant the drive its full original timeout.
  const std::uint32_t elapsed_ms = pros::millis() - command_started_ms;
  if (elapsed_ms >= timeout_ms || timeout_ms - elapsed_ms < 100) {
    stop_drive_motors();
    telemetry_last_log_ms = 0;
    return Result::kDriveFailed;
  }
  const std::uint32_t drive_timeout_ms = timeout_ms - elapsed_ms;

  const bool arrived = fused_drive_to_point(
      telemetry_pose,
      "navigation_go_to_drive",
      Waypoint{target_x_in, target_y_in},
      bearing_deg,
      std::clamp(max_power, 20, 80),
      drive_timeout_ms);
  stop_drive_motors();
  telemetry_last_log_ms = 0;
  return arrived ? Result::kSuccess : Result::kDriveFailed;
}

Result drive_relative(double distance_in,
                      int max_power,
                      std::uint32_t timeout_ms,
                      bool stop_for_forward_obstacle,
                      bool allow_goal_contact,
                      bool allow_wall_proximity) {
  if (!std::isfinite(distance_in) || max_power <= 0 || timeout_ms < 100) {
    return Result::kInvalidArgument;
  }
  if (!navigation_api_initialized || !telemetry_pose_initialized) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }

  navigation_stop_requested.store(false, std::memory_order_release);
  localization_telemetry_update();
  if (!telemetry_pose.ready || !telemetry_pose.imu_ready) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }
  if (std::fabs(distance_in) <= kFusedDriveToleranceIn) {
    stop_drive_motors();
    return Result::kSuccess;
  }

  const double heading_deg = pose_heading_deg(telemetry_pose);
  const double heading_rad = deg_to_rad(heading_deg);
  const Waypoint start{telemetry_pose.x, telemetry_pose.y};
  const Waypoint target{
      start.x + distance_in * std::cos(heading_rad),
      start.y + distance_in * std::sin(heading_rad),
  };
  const double wall = localization::kPhysicalWallHalfSpanIn;
  if (target.x < -wall || target.x > wall ||
      target.y < -wall || target.y > wall) {
    stop_drive_motors();
    return Result::kUnsafePath;
  }

  const double projected_path_error_envelope_in =
      projected_position_error_envelope_in(
          telemetry_pose.position_error_envelope_in,
          std::fabs(distance_in));
  double minimum_goal_clearance_in = 0.0;
  double required_goal_clearance_in = 0.0;
  const char* path_reject = "none";
  if (!public_straight_segment_is_safe(
          start, target, projected_path_error_envelope_in,
          minimum_goal_clearance_in, required_goal_clearance_in,
          path_reject, allow_goal_contact, allow_wall_proximity)) {
    stop_drive_motors();
    std::printf(
        "NAV_RELATIVE reject=%s from=%.2f,%.2f target=%.2f,%.2f "
        "distance=%.2f goal_clearance=%.2f required=%.2f "
        "projected_envelope=%.2f\n",
        path_reject, start.x, start.y, target.x, target.y, distance_in,
        minimum_goal_clearance_in, required_goal_clearance_in,
        projected_path_error_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }

  chassis.drive_mode_set(ez::DISABLE, true);
  const bool arrived = fused_drive_to_point(
      telemetry_pose,
      distance_in > 0.0 ? "navigation_relative_forward"
                        : "navigation_relative_reverse",
      target,
      heading_deg,
      std::clamp(max_power, 20, 80),
      timeout_ms,
      localization::kNavigationCurvedPathCorridorIn,
      distance_in > 0.0 ? 1 : -1,
      LidarFusionMode::kBiasOnly,
      stop_for_forward_obstacle);
  stop_drive_motors();
  telemetry_last_log_ms = 0;
  return arrived ? Result::kSuccess : Result::kDriveFailed;
}

Result go_to_pose(double target_x_in,
                  double target_y_in,
                  double target_heading_deg,
                  int max_power,
                  std::uint32_t timeout_ms,
                  bool reverse,
                  bool allow_goal_contact,
                  bool allow_wall_proximity) {
  const double wall = localization::kPhysicalWallHalfSpanIn;
  if (!std::isfinite(target_x_in) || !std::isfinite(target_y_in) ||
      !std::isfinite(target_heading_deg) || target_x_in < -wall ||
      target_x_in > wall || target_y_in < -wall || target_y_in > wall ||
      max_power <= 0 || timeout_ms < 100) {
    return Result::kInvalidArgument;
  }
  if (!navigation_api_initialized || !telemetry_pose_initialized) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }

  navigation_stop_requested.store(false, std::memory_order_release);
  localization_telemetry_update();
  if (!telemetry_pose.ready || !telemetry_pose.imu_ready) {
    stop_drive_motors();
    return Result::kPoseUnavailable;
  }

  const Waypoint start{telemetry_pose.x, telemetry_pose.y};
  const Waypoint target{target_x_in, target_y_in};
  const double requested_path_length_in =
      std::hypot(target.x - start.x, target.y - start.y);
  const double path_bearing_deg = requested_path_length_in > 1e-9
      ? normalize_deg(rad_to_deg(std::atan2(target.y - start.y,
                                           target.x - start.x)))
      : pose_heading_deg(telemetry_pose);
  const double chassis_path_heading_deg = normalize_deg(
      path_bearing_deg + (reverse ? 180.0 : 0.0));
  const double initial_bearing_error_deg = std::fabs(signed_angle_diff_deg(
      chassis_path_heading_deg, pose_heading_deg(telemetry_pose)));
  const bool requires_preturn =
      requested_path_length_in > kFusedDriveToleranceIn &&
      initial_bearing_error_deg > kNavigationGoToPosePreturnThresholdDeg;
  const double projected_path_error_envelope_in =
      projected_position_error_envelope_in(
          telemetry_pose.position_error_envelope_in,
          requested_path_length_in);
  const double curved_corridor_envelope_in =
      projected_path_error_envelope_in +
      localization::kNavigationCurvedPathCorridorIn;

  double minimum_goal_clearance_in = 0.0;
  double required_goal_clearance_in = 0.0;
  const char* path_reject = "none";
  if (requested_path_length_in > kFusedDriveToleranceIn &&
      !public_straight_segment_is_safe(
          start, target, curved_corridor_envelope_in,
          minimum_goal_clearance_in, required_goal_clearance_in,
          path_reject, allow_goal_contact, allow_wall_proximity)) {
    stop_drive_motors();
    std::printf(
        "NAV_POSE reject=%s from=%.2f,%.2f target=%.2f,%.2f "
        "goal_clearance=%.2f required=%.2f corridor_envelope=%.2f\n",
        path_reject, start.x, start.y, target.x, target.y,
        minimum_goal_clearance_in, required_goal_clearance_in,
        curved_corridor_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }

  if (requires_preturn) {
    double start_turn_goal_clearance_in = 0.0;
    const char* start_turn_reject = "none";
    if (!public_turn_center_is_safe(
            start, telemetry_pose.position_error_envelope_in,
            start_turn_goal_clearance_in, start_turn_reject,
            allow_goal_contact, allow_wall_proximity)) {
      stop_drive_motors();
      std::printf(
          "NAV_POSE reject=%s preturn=1 from=%.2f,%.2f bearing_error=%.2f "
          "goal_clearance=%.2f pose_envelope=%.2f\n",
          start_turn_reject, start.x, start.y, initial_bearing_error_deg,
          start_turn_goal_clearance_in,
          telemetry_pose.position_error_envelope_in);
      std::fflush(stdout);
      return Result::kUnsafePath;
    }
  }

  double target_goal_clearance_in = 0.0;
  const char* target_turn_reject = "none";
  if (!public_turn_center_is_safe(
          target, projected_path_error_envelope_in,
          target_goal_clearance_in, target_turn_reject,
          allow_goal_contact, allow_wall_proximity)) {
    stop_drive_motors();
    std::printf(
        "NAV_POSE reject=%s target=%.2f,%.2f goal_clearance=%.2f "
        "projected_envelope=%.2f\n",
        target_turn_reject, target.x, target.y, target_goal_clearance_in,
        projected_path_error_envelope_in);
    std::fflush(stdout);
    return Result::kUnsafePath;
  }

  const std::uint32_t command_started_ms = pros::millis();
  chassis.drive_mode_set(ez::DISABLE, true);
  if (requires_preturn) {
    const bool preturned = fused_turn_to_heading(
        telemetry_pose, "navigation_go_to_pose_preturn",
        chassis_path_heading_deg,
        std::clamp(max_power, 20, 75),
        std::min<std::uint32_t>(timeout_ms, 6000));
    if (!preturned) {
      stop_drive_motors();
      telemetry_last_log_ms = 0;
      return Result::kTurnFailed;
    }
  }
  if (requested_path_length_in > kFusedDriveToleranceIn) {
    const std::uint32_t predrive_elapsed_ms =
        pros::millis() - command_started_ms;
    if (predrive_elapsed_ms >= timeout_ms ||
        timeout_ms - predrive_elapsed_ms < 100) {
      stop_drive_motors();
      telemetry_last_log_ms = 0;
      return Result::kDriveFailed;
    }
    const bool arrived = fused_drive_to_point(
        telemetry_pose, "navigation_go_to_pose_drive", target,
        normalize_deg(target_heading_deg), std::clamp(max_power, 20, 90),
        timeout_ms - predrive_elapsed_ms,
        localization::kNavigationCurvedPathCorridorIn,
        reverse ? -1 : 1);
    if (!arrived) {
      stop_drive_motors();
      telemetry_last_log_ms = 0;
      return Result::kDriveFailed;
    }
  }

  const std::uint32_t elapsed_ms = pros::millis() - command_started_ms;
  if (elapsed_ms >= timeout_ms || timeout_ms - elapsed_ms < 100) {
    stop_drive_motors();
    telemetry_last_log_ms = 0;
    return Result::kTurnFailed;
  }
  const bool heading_settled = fused_turn_to_heading(
      telemetry_pose, "navigation_go_to_pose_heading",
      normalize_deg(target_heading_deg), std::clamp(max_power, 20, 90),
      timeout_ms - elapsed_ms);
  stop_drive_motors();
  telemetry_last_log_ms = 0;
  return heading_settled ? Result::kSuccess : Result::kTurnFailed;
}

void stop() {
  navigation_stop_requested.store(true, std::memory_order_release);
  emergency_stop_drive_motors();
}

const char* result_name(Result result) {
  switch (result) {
    case Result::kSuccess: return "success";
    case Result::kInvalidArgument: return "invalid_argument";
    case Result::kPoseUnavailable: return "pose_unavailable";
    case Result::kUnsafePath: return "unsafe_path";
    case Result::kTurnFailed: return "turn_failed";
    case Result::kDriveFailed: return "drive_failed";
  }
  return "unknown";
}

}  // namespace navigation

void localization_navigation_qualification_route() {
  // This synthetic field anchor puts both mirrored 12-by-12-inch routes in a
  // region that clears every mapped Goal and wall with the public API's full
  // curved-corridor and uncertainty inflation. It does not claim that the
  // physical robot is actually at this field coordinate; the trial measures
  // relative control and estimator loop closure.
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kForwardXIn = -23.0;
  constexpr double kLeftYIn = 12.0;
  constexpr double kRightYIn = -12.0;
  constexpr int kMaxPower = 60;
  constexpr std::uint32_t kMoveTimeoutMs = 9000;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_QUAL event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) {
    stop_drive_motors();
    return;
  }

  struct Leg {
    const char* name;
    double x_in;
    double y_in;
    double heading_deg;
  };
  constexpr std::array<Leg, 4> kLegs{{
      {"left_out", kForwardXIn, kLeftYIn, 45.0},
      {"left_return", kStartXIn, kStartYIn, kStartHeadingDeg},
      {"right_out", kForwardXIn, kRightYIn, 315.0},
      {"right_return", kStartXIn, kStartYIn, kStartHeadingDeg},
  }};

  bool all_ok = true;
  for (std::size_t i = 0; i < kLegs.size(); ++i) {
    const auto& leg = kLegs[i];
    const navigation::Pose before = navigation::current_pose();
    std::printf(
        "NAV_QUAL event=leg_start index=%u name=%s x=%.3f y=%.3f "
        "heading=%.3f target_x=%.3f target_y=%.3f target_heading=%.3f\n",
        static_cast<unsigned>(i), leg.name, before.x_in, before.y_in,
        before.heading_deg, leg.x_in, leg.y_in, leg.heading_deg);
    std::fflush(stdout);

    const std::uint32_t started_ms = pros::millis();
    const navigation::Result result = navigation::go_to_pose(
        leg.x_in, leg.y_in, leg.heading_deg, kMaxPower, kMoveTimeoutMs);
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const navigation::SensorHealth health = navigation::sensor_health();
    const double position_error_in =
        std::hypot(after.x_in - leg.x_in, after.y_in - leg.y_in);
    const double heading_error_deg = std::fabs(signed_angle_diff_deg(
        leg.heading_deg, after.heading_deg));
    std::printf(
        "NAV_QUAL event=leg_done index=%u name=%s result=%s elapsed_ms=%lu "
        "x=%.3f y=%.3f heading=%.3f position_error=%.3f "
        "heading_error=%.3f envelope=%.3f absolute_age_ms=%lu "
        "gps=%s imu=%d encoders=%d distance_valid=%d distance=%.3f\n",
        static_cast<unsigned>(i), leg.name, navigation::result_name(result),
        static_cast<unsigned long>(pros::millis() - started_ms), after.x_in,
        after.y_in, after.heading_deg, position_error_in, heading_error_deg,
        after.position_error_envelope_in,
        static_cast<unsigned long>(after.absolute_position_age_ms),
        health.gps_state, static_cast<int>(health.imu_valid),
        static_cast<int>(health.drive_encoders_valid),
        static_cast<int>(health.forward_distance_valid),
        health.forward_distance_in);
    std::fflush(stdout);
    if (result != navigation::Result::kSuccess) {
      all_ok = false;
      break;
    }
    pros::delay(500);
  }

  navigation::stop();
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_QUAL event=complete ok=%d path_points=%u loop_error=%.3f "
      "heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(all_ok),
      static_cast<unsigned>(navigation::path_size()),
      std::hypot(final_pose.x_in - kStartXIn, final_pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  stop_drive_motors();
}

void localization_navigation_bidirectional_qualification() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr int kMaxPower = 50;
  constexpr std::uint32_t kMoveTimeoutMs = 6000;
  constexpr std::array<double, 4> kLegsIn{{6.0, -6.0, 10.0, -10.0}};

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_BIDIR event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  bool all_ok = true;
  for (std::size_t index = 0; index < kLegsIn.size(); ++index) {
    const navigation::Pose before = navigation::current_pose();
    const std::uint32_t started_ms = pros::millis();
    const navigation::Result result = navigation::drive_relative(
        kLegsIn[index], kMaxPower, kMoveTimeoutMs);
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const double displacement_in = std::hypot(
        after.x_in - before.x_in, after.y_in - before.y_in);
    std::printf(
        "NAV_BIDIR event=leg_done index=%u command=%.3f result=%s "
        "elapsed_ms=%lu x=%.3f y=%.3f heading=%.3f displacement=%.3f "
        "envelope=%.3f\n",
        static_cast<unsigned>(index), kLegsIn[index],
        navigation::result_name(result),
        static_cast<unsigned long>(pros::millis() - started_ms),
        after.x_in, after.y_in, after.heading_deg, displacement_in,
        after.position_error_envelope_in);
    std::fflush(stdout);
    if (result != navigation::Result::kSuccess) {
      all_ok = false;
      break;
    }
    pros::delay(500);
  }

  navigation::stop();
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_BIDIR event=complete ok=%d loop_error=%.3f heading_error=%.3f "
      "x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(all_ok),
      std::hypot(final_pose.x_in - kStartXIn, final_pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  stop_drive_motors();
}

void localization_navigation_turn_qualification() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr int kDrivePower = 50;
  constexpr int kTurnPower = 45;
  constexpr std::uint32_t kMoveTimeoutMs = 7000;
  constexpr std::array<double, 4> kHeadingsDeg{{45.0, 0.0, 315.0, 0.0}};

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_TURN_QUAL event=init ok=%d\n",
              static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  bool all_ok = true;
  navigation::Result result = navigation::drive_relative(
      12.0, kDrivePower, kMoveTimeoutMs);
  std::printf("NAV_TURN_QUAL event=position result=%s direction=forward\n",
              navigation::result_name(result));
  std::fflush(stdout);
  all_ok = result == navigation::Result::kSuccess;

  for (std::size_t index = 0; all_ok && index < kHeadingsDeg.size(); ++index) {
    const std::uint32_t started_ms = pros::millis();
    result = navigation::turn_to(
        kHeadingsDeg[index], kTurnPower, kMoveTimeoutMs);
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const double heading_error_deg = std::fabs(signed_angle_diff_deg(
        kHeadingsDeg[index], after.heading_deg));
    std::printf(
        "NAV_TURN_QUAL event=turn_done index=%u target=%.3f result=%s "
        "elapsed_ms=%lu x=%.3f y=%.3f heading=%.3f heading_error=%.3f "
        "envelope=%.3f\n",
        static_cast<unsigned>(index), kHeadingsDeg[index],
        navigation::result_name(result),
        static_cast<unsigned long>(pros::millis() - started_ms),
        after.x_in, after.y_in, after.heading_deg, heading_error_deg,
        after.position_error_envelope_in);
    std::fflush(stdout);
    all_ok = result == navigation::Result::kSuccess;
    pros::delay(350);
  }

  if (all_ok) {
    result = navigation::drive_relative(-12.0, kDrivePower, kMoveTimeoutMs);
    all_ok = result == navigation::Result::kSuccess;
  }
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_TURN_QUAL event=complete ok=%d return_result=%s "
      "loop_error=%.3f heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(all_ok), navigation::result_name(result),
      std::hypot(final_pose.x_in - kStartXIn, final_pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

void localization_navigation_turn_recovery_qualification() {
  // The preceding turn qualification reached this synthetic/physical center
  // pose before its final low-power stall. Reinitialize after the safe-image
  // deployment reset, repeat the public turns, then reverse along the verified
  // 12-inch inbound corridor to the original physical start.
  constexpr double kCenterXIn = -23.0;
  constexpr double kCenterYIn = 0.0;
  constexpr double kCenterHeadingDeg = 0.0;
  constexpr int kTurnPower = 45;
  constexpr std::uint32_t kMoveTimeoutMs = 7000;
  constexpr std::array<double, 4> kHeadingsDeg{{45.0, 0.0, 315.0, 0.0}};

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kCenterXIn, kCenterYIn, kCenterHeadingDeg, 1.0);
  std::printf("NAV_TURN_RECOVERY event=init ok=%d\n",
              static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  bool all_ok = true;
  navigation::Result result = navigation::Result::kSuccess;
  for (std::size_t index = 0; all_ok && index < kHeadingsDeg.size(); ++index) {
    const std::uint32_t started_ms = pros::millis();
    result = navigation::turn_to(
        kHeadingsDeg[index], kTurnPower, kMoveTimeoutMs);
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const double heading_error_deg = std::fabs(signed_angle_diff_deg(
        kHeadingsDeg[index], after.heading_deg));
    std::printf(
        "NAV_TURN_RECOVERY event=turn_done index=%u target=%.3f result=%s "
        "elapsed_ms=%lu x=%.3f y=%.3f heading=%.3f heading_error=%.3f "
        "envelope=%.3f\n",
        static_cast<unsigned>(index), kHeadingsDeg[index],
        navigation::result_name(result),
        static_cast<unsigned long>(pros::millis() - started_ms),
        after.x_in, after.y_in, after.heading_deg, heading_error_deg,
        after.position_error_envelope_in);
    std::fflush(stdout);
    all_ok = result == navigation::Result::kSuccess;
    pros::delay(350);
  }

  if (all_ok) {
    result = navigation::drive_relative(-12.0, 50, kMoveTimeoutMs);
    all_ok = result == navigation::Result::kSuccess;
  }
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_TURN_RECOVERY event=complete ok=%d return_result=%s "
      "return_distance_error=%.3f heading_error=%.3f x=%.3f y=%.3f "
      "heading=%.3f\n",
      static_cast<int>(all_ok), navigation::result_name(result),
      std::fabs(final_pose.x_in - (kCenterXIn - 12.0)),
      std::fabs(signed_angle_diff_deg(kCenterHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

void localization_navigation_mirrored_curve_qualification() {
  constexpr double kStartXIn = -50.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kRepositionIn = 12.0;
  constexpr double kCurveForwardIn = 10.0;
  constexpr double kCurveSideIn = 4.0;
  constexpr double kCurveLengthIn = 10.770329614269007;
  constexpr int kMaxPower = 60;
  constexpr std::uint32_t kMoveTimeoutMs = 9000;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_MIRROR event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  bool all_ok = navigation::drive_relative(
      kRepositionIn, 50, kMoveTimeoutMs) == navigation::Result::kSuccess;
  std::printf("NAV_MIRROR event=reposition ok=%d\n",
              static_cast<int>(all_ok));
  std::fflush(stdout);

  constexpr std::array<double, 2> kSideSigns{{1.0, -1.0}};
  for (std::size_t index = 0; all_ok && index < kSideSigns.size(); ++index) {
    navigation::update();
    const navigation::Pose start = navigation::current_pose();
    const double side_in = kSideSigns[index] * kCurveSideIn;
    const double target_heading_deg = normalize_deg(rad_to_deg(
        std::atan2(side_in, kCurveForwardIn)));
    const navigation::Result curve_result = navigation::go_to_pose(
        start.x_in + kCurveForwardIn,
        start.y_in + side_in,
        target_heading_deg,
        kMaxPower,
        kMoveTimeoutMs);
    navigation::update();
    const navigation::Pose curve_pose = navigation::current_pose();
    std::printf(
        "NAV_MIRROR event=curve_done index=%u side=%s result=%s "
        "x=%.3f y=%.3f heading=%.3f target_heading=%.3f envelope=%.3f\n",
        static_cast<unsigned>(index), side_in > 0.0 ? "left" : "right",
        navigation::result_name(curve_result), curve_pose.x_in,
        curve_pose.y_in, curve_pose.heading_deg, target_heading_deg,
        curve_pose.position_error_envelope_in);
    std::fflush(stdout);
    all_ok = curve_result == navigation::Result::kSuccess;
    if (!all_ok) break;

    const navigation::Result reverse_result = navigation::drive_relative(
        -kCurveLengthIn, 50, kMoveTimeoutMs);
    const navigation::Result center_result =
        reverse_result == navigation::Result::kSuccess
            ? navigation::turn_to(0.0, 45, kMoveTimeoutMs)
            : reverse_result;
    navigation::update();
    const navigation::Pose returned = navigation::current_pose();
    const double branch_loop_error_in = std::hypot(
        returned.x_in - start.x_in, returned.y_in - start.y_in);
    std::printf(
        "NAV_MIRROR event=branch_return index=%u reverse=%s center=%s "
        "loop_error=%.3f heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
        static_cast<unsigned>(index),
        navigation::result_name(reverse_result),
        navigation::result_name(center_result), branch_loop_error_in,
        std::fabs(signed_angle_diff_deg(0.0, returned.heading_deg)),
        returned.x_in, returned.y_in, returned.heading_deg);
    std::fflush(stdout);
    all_ok = reverse_result == navigation::Result::kSuccess &&
             center_result == navigation::Result::kSuccess;
    pros::delay(400);
  }

  navigation::Result home_result = navigation::Result::kDriveFailed;
  if (all_ok) {
    home_result = navigation::drive_relative(
        -kRepositionIn, 50, kMoveTimeoutMs);
    all_ok = home_result == navigation::Result::kSuccess;
  }
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_MIRROR event=complete ok=%d home=%s loop_error=%.3f "
      "heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(all_ok), navigation::result_name(home_result),
      std::hypot(final_pose.x_in - kStartXIn,
                 final_pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

void localization_navigation_obstacle_approach_qualification() {
  constexpr double kStartXIn = -55.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kMaximumApproachIn = 24.0;
  constexpr std::uint32_t kMoveTimeoutMs = 10000;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_OBS_APPROACH event=init ok=%d\n",
              static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  const navigation::Pose start = navigation::current_pose();
  const navigation::Result approach_result = navigation::drive_relative(
      kMaximumApproachIn, 45, kMoveTimeoutMs);
  navigation::update();
  const navigation::Pose stopped = navigation::current_pose();
  const navigation::SensorHealth stopped_health = navigation::sensor_health();
  const double heading_rad = deg_to_rad(kStartHeadingDeg);
  const double signed_travel_in =
      (stopped.x_in - start.x_in) * std::cos(heading_rad) +
      (stopped.y_in - start.y_in) * std::sin(heading_rad);
  const double return_distance_in = clamp(
      signed_travel_in, 0.0, kMaximumApproachIn + 2.0);
  const bool obstacle_observed =
      stopped_health.forward_distance_valid &&
      stopped_health.forward_distance_in <=
          localization::kForwardObstacleStopIn;
  std::printf(
      "NAV_OBS_APPROACH event=stopped result=%s traveled=%.3f "
      "p1_valid=%d p1_distance=%.3f p1_confidence=%ld obstacle=%d\n",
      navigation::result_name(approach_result), return_distance_in,
      static_cast<int>(stopped_health.forward_distance_valid),
      stopped_health.forward_distance_in,
      stopped_health.forward_distance_confidence,
      static_cast<int>(obstacle_observed));
  std::fflush(stdout);
  pros::delay(500);

  navigation::Result return_result = navigation::Result::kSuccess;
  if (return_distance_in > kFusedDriveToleranceIn) {
    return_result = navigation::drive_relative(
        -return_distance_in, 45, kMoveTimeoutMs);
  }
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double loop_error_in = std::hypot(
      final_pose.x_in - start.x_in, final_pose.y_in - start.y_in);
  const bool pass = approach_result == navigation::Result::kDriveFailed &&
                    obstacle_observed &&
                    return_result == navigation::Result::kSuccess &&
                    loop_error_in <= 1.5;
  std::printf(
      "NAV_OBS_APPROACH event=complete pass=%d approach=%s return=%s "
      "loop_error=%.3f heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(pass), navigation::result_name(approach_result),
      navigation::result_name(return_result), loop_error_in,
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

void localization_navigation_reverse_recovery() {
  // This is a relative, camera-verified recovery. Anchor at field center so
  // the public wall-clearance gate evaluates the 24-inch reverse corridor
  // without inventing a near-wall endpoint.
  constexpr double kStartXIn = 0.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kRecoveryDistanceIn = -24.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_REVERSE_RECOVERY event=init ok=%d\n",
              static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  const navigation::Pose start = navigation::current_pose();
  const navigation::Result result = navigation::drive_relative(
      kRecoveryDistanceIn, 45, 12000);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_REVERSE_RECOVERY event=complete result=%s traveled=%.3f "
      "lateral=%.3f heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
      navigation::result_name(result), start.x_in - final_pose.x_in,
      std::fabs(final_pose.y_in - start.y_in),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg,
                                     final_pose.heading_deg)),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

void localization_slider_slow_picture_test() {
  constexpr int kSliderPower = 25;
  constexpr std::uint32_t kDurationMs = 500;
  constexpr std::int32_t kCurrentAbortMa = 2000;

  stop_drive_motors();
  slider_right.move(0);
  slider_left.move(0);
  slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  const double right_start_deg = slider_right.get_position();
  const double left_start_deg = slider_left.get_position();
  const std::int32_t rotation_start = slider_rotation_sensor.get_position();
  std::printf(
      "SLIDER_TEST event=start power=%d duration_ms=%lu installed_right=%d "
      "installed_left=%d right_deg=%.3f left_deg=%.3f rotation=%ld\n",
      kSliderPower, static_cast<unsigned long>(kDurationMs),
      static_cast<int>(slider_right.is_installed()),
      static_cast<int>(slider_left.is_installed()), right_start_deg,
      left_start_deg, static_cast<long>(rotation_start));
  std::fflush(stdout);

  bool current_abort = false;
  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < kDurationMs) {
    stop_drive_motors();
    slider_right.move(kSliderPower);
    slider_left.move(kSliderPower);
    const std::int32_t right_current_ma = slider_right.get_current_draw();
    const std::int32_t left_current_ma = slider_left.get_current_draw();
    if (right_current_ma >= kCurrentAbortMa ||
        left_current_ma >= kCurrentAbortMa) {
      current_abort = true;
      break;
    }
    pros::delay(20);
  }
  slider_right.move(0);
  slider_left.move(0);
  stop_drive_motors();
  pros::delay(250);

  const double right_end_deg = slider_right.get_position();
  const double left_end_deg = slider_left.get_position();
  const std::int32_t rotation_end = slider_rotation_sensor.get_position();
  std::printf(
      "SLIDER_TEST event=complete current_abort=%d right_delta=%.3f "
      "left_delta=%.3f rotation_delta=%ld right_current=%ld "
      "left_current=%ld\n",
      static_cast<int>(current_abort), right_end_deg - right_start_deg,
      left_end_deg - left_start_deg,
      static_cast<long>(rotation_end - rotation_start),
      static_cast<long>(slider_right.get_current_draw()),
      static_cast<long>(slider_left.get_current_draw()));
  std::fflush(stdout);
}

void localization_all_mechanisms_test() {
  constexpr std::uint32_t kPulseMs = 350;
  constexpr std::uint32_t kSnapshotPauseMs = 3000;
  constexpr std::int32_t kCurrentAbortMa = 2000;

  const auto pulse = [&](pros::Motor& motor, const char* label, int power,
                         const char* snapshot) {
    stop_drive_motors();
    const double start_deg = motor.get_position();
    std::int32_t maximum_current_ma = 0;
    bool current_abort = false;
    const std::uint32_t started_ms = pros::millis();
    while (pros::millis() - started_ms < kPulseMs) {
      stop_drive_motors();
      motor.move(power);
      const std::int32_t current_ma = motor.get_current_draw();
      maximum_current_ma = std::max(maximum_current_ma, current_ma);
      if (current_ma >= kCurrentAbortMa) {
        current_abort = true;
        break;
      }
      pros::delay(20);
    }
    motor.move(0);
    stop_drive_motors();
    pros::delay(200);
    std::printf(
        "MECH_TEST part=%s snapshot=%s power=%d installed=%d "
        "delta_deg=%.3f max_current_ma=%ld current_abort=%d\n",
        label, snapshot, power, static_cast<int>(motor.is_installed()),
        motor.get_position() - start_deg,
        static_cast<long>(maximum_current_ma),
        static_cast<int>(current_abort));
    std::fflush(stdout);
    pros::delay(kSnapshotPauseMs);
  };

  stop_drive_motors();
  upper_intake.move(0);
  counter_rollers.move(0);
  claw_arm.move(0);
  std::printf(
      "MECH_TEST event=inventory intake_installed=%d counter_installed=%d "
      "claw_installed=%d slider_right_installed=%d slider_left_installed=%d "
      "slider_rotation_installed=%d\n",
      static_cast<int>(upper_intake.is_installed()),
      static_cast<int>(counter_rollers.is_installed()),
      static_cast<int>(claw_arm.is_installed()),
      static_cast<int>(slider_right.is_installed()),
      static_cast<int>(slider_left.is_installed()),
      static_cast<int>(slider_rotation_sensor.is_installed()));
  std::fflush(stdout);

  pulse(upper_intake, "upper_intake_p15", 25, "intake_positive");
  pulse(upper_intake, "upper_intake_p15", -25, "intake_return");
  pulse(counter_rollers, "counter_rollers_p3", 25, "counter_positive");
  pulse(counter_rollers, "counter_rollers_p3", -25, "counter_return");
  pulse(claw_arm, "claw_arm_p4", 20, "claw_positive");
  pulse(claw_arm, "claw_arm_p4", -20, "claw_return");

  stop_drive_motors();
  clamp_piston.set_value(false);
  std::printf("MECH_TEST part=clamp_adi_h snapshot=clamp_false state=0\n");
  std::fflush(stdout);
  pros::delay(kSnapshotPauseMs);
  clamp_piston.set_value(true);
  pros::delay(300);
  std::printf("MECH_TEST part=clamp_adi_h snapshot=clamp_true state=1\n");
  std::fflush(stdout);
  pros::delay(kSnapshotPauseMs);
  clamp_piston.set_value(false);
  std::printf("MECH_TEST event=complete clamp_safe_state=0\n");
  std::fflush(stdout);
  upper_intake.move(0);
  counter_rollers.move(0);
  claw_arm.move(0);
  stop_drive_motors();
}

void localization_clamp_picture_test() {
  stop_drive_motors();
  upper_intake.move(0);
  counter_rollers.move(0);
  claw_arm.move(0);
  slider_right.move(0);
  slider_left.move(0);

  clamp_piston.set_value(false);
  pros::delay(300);
  std::printf("CLAMP_TEST snapshot=clamp_false state=0\n");
  std::fflush(stdout);
  pros::delay(5000);

  clamp_piston.set_value(true);
  pros::delay(300);
  std::printf("CLAMP_TEST snapshot=clamp_true state=1\n");
  std::fflush(stdout);
  pros::delay(5000);

  clamp_piston.set_value(false);
  pros::delay(300);
  std::printf("CLAMP_TEST event=complete safe_state=0\n");
  std::fflush(stdout);
  stop_drive_motors();
}

void localization_toggle_goal_example_auton() {
  constexpr double kExpectedStartXIn = -5.8;
  constexpr double kExpectedStartYIn = 60.4;
  constexpr double kExpectedStartHeadingDeg = 89.45;
  constexpr double kStartPositionGateIn = 3.0;
  constexpr double kStartHeadingGateDeg = 4.0;
  constexpr double kGpsMaximumSampleSpreadIn = 2.0;
  constexpr double kGpsMaximumHeadingSpreadDeg = 1.5;
  constexpr int kGpsSamples = 20;
  constexpr double kToggleMinimumRangeIn = 4.0;
  constexpr double kToggleMaximumRangeIn = 8.0;
  constexpr double kMillimetersPerInch = 25.4;
  constexpr double kRamMaximumTravelIn = 2.5;
  constexpr std::uint32_t kRamMaximumMs = 700;
  constexpr std::int32_t kRamCurrentLimitMa = 2400;
  constexpr double kTurnClearanceReverseIn = 30.0;
  constexpr double kGoalCenterXIn = 24.0;
  constexpr double kGoalCenterYIn = -48.0;
  // A Goal is a physical obstacle. Stop the robot center north of it, facing
  // the Goal, rather than asking navigation to occupy the Goal center.
  constexpr double kGoalApproachXIn = 24.0;
  constexpr double kGoalApproachYIn = -24.0;
  constexpr double kGoalApproachHeadingDeg = 270.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  double gps_x_sum = 0.0;
  double gps_y_sum = 0.0;
  double gps_heading_sin = 0.0;
  double gps_heading_cos = 0.0;
  double gps_error_sum = 0.0;
  double gps_x_min = std::numeric_limits<double>::infinity();
  double gps_x_max = -std::numeric_limits<double>::infinity();
  double gps_y_min = std::numeric_limits<double>::infinity();
  double gps_y_max = -std::numeric_limits<double>::infinity();
  double gps_heading_min = std::numeric_limits<double>::infinity();
  double gps_heading_max = -std::numeric_limits<double>::infinity();
  int gps_valid_samples = 0;
  for (int index = 0; index < kGpsSamples; ++index) {
    const GpsObservation observation = read_robot_gps();
    if (observation.valid) {
      gps_x_sum += observation.x_in;
      gps_y_sum += observation.y_in;
      const double heading_rad = deg_to_rad(observation.heading_deg);
      gps_heading_sin += std::sin(heading_rad);
      gps_heading_cos += std::cos(heading_rad);
      gps_error_sum += observation.error_in;
      gps_x_min = std::min(gps_x_min, observation.x_in);
      gps_x_max = std::max(gps_x_max, observation.x_in);
      gps_y_min = std::min(gps_y_min, observation.y_in);
      gps_y_max = std::max(gps_y_max, observation.y_in);
      gps_heading_min = std::min(gps_heading_min, observation.heading_deg);
      gps_heading_max = std::max(gps_heading_max, observation.heading_deg);
      ++gps_valid_samples;
    }
    pros::delay(50);
  }
  if (gps_valid_samples < kGpsSamples * 4 / 5) {
    std::printf("TOGGLE_GOAL abort=gps_samples valid=%d required=%d\n",
                gps_valid_samples, kGpsSamples * 4 / 5);
    std::fflush(stdout);
    return;
  }

  const double start_x_in = gps_x_sum / gps_valid_samples;
  const double start_y_in = gps_y_sum / gps_valid_samples;
  const double start_heading_deg = normalize_deg(rad_to_deg(std::atan2(
      gps_heading_sin, gps_heading_cos)));
  const double gps_error_in = gps_error_sum / gps_valid_samples;
  const double gps_position_spread_in = std::hypot(
      gps_x_max - gps_x_min, gps_y_max - gps_y_min);
  const double gps_heading_spread_deg = gps_heading_max - gps_heading_min;
  const double expected_position_error_in = std::hypot(
      start_x_in - kExpectedStartXIn, start_y_in - kExpectedStartYIn);
  const double expected_heading_error_deg = std::fabs(
      signed_angle_diff_deg(kExpectedStartHeadingDeg, start_heading_deg));
  const double p1_start_in = static_cast<double>(distance_1.get_distance()) /
      kMillimetersPerInch;
  const bool start_gate_ok =
      gps_position_spread_in <= kGpsMaximumSampleSpreadIn &&
      gps_heading_spread_deg <= kGpsMaximumHeadingSpreadDeg &&
      expected_position_error_in <= kStartPositionGateIn &&
      expected_heading_error_deg <= kStartHeadingGateDeg &&
      std::isfinite(p1_start_in) &&
      p1_start_in >= kToggleMinimumRangeIn &&
      p1_start_in <= kToggleMaximumRangeIn;
  std::printf(
      "TOGGLE_GOAL event=preflight ok=%d start=%.3f,%.3f,%.3f "
      "gps_error=%.3f gps_spread=%.3f heading_spread=%.3f "
      "expected_pos_error=%.3f expected_heading_error=%.3f p1=%.3f\n",
      static_cast<int>(start_gate_ok), start_x_in, start_y_in,
      start_heading_deg, gps_error_in, gps_position_spread_in,
      gps_heading_spread_deg, expected_position_error_in,
      expected_heading_error_deg, p1_start_in);
  std::fflush(stdout);
  if (!start_gate_ok) return;

  const double start_error_envelope_in = std::max(
      1.0, gps_error_in + 0.5 * gps_position_spread_in);
  if (!navigation::init(start_x_in, start_y_in, start_heading_deg,
                        start_error_envelope_in)) {
    std::printf("TOGGLE_GOAL abort=navigation_init\n");
    std::fflush(stdout);
    return;
  }

  // init() deliberately leaves cancellation latched after re-anchoring the
  // estimator. Public navigation commands clear that older latch at their
  // command boundary; these bounded contact/clearance phases use the internal
  // controller, so give them the same linearization semantics. A stop or
  // field-disable arriving after this store remains visible to every loop.
  auto begin_internal_motion = [&](const char* phase) {
    navigation_stop_requested.store(false, std::memory_order_release);
    return !blocking_motion_abort_requested(phase);
  };

  auto ram_and_return = [&](int cycle_index) {
    if (!begin_internal_motion("toggle_ram")) return false;
    navigation::update();
    const navigation::Pose cycle_start = navigation::current_pose();
    const MotorSideReading left_start = read_motor_side(chassis.left_motors);
    const MotorSideReading right_start = read_motor_side(chassis.right_motors);
    if (!left_start.trustworthy || !right_start.trustworthy) return false;

    std::int32_t maximum_current_ma = 0;
    bool current_limit = false;
    bool travel_limit = false;
    const std::uint32_t started_ms = pros::millis();
    while (pros::millis() - started_ms < kRamMaximumMs) {
      navigation::update();
      const MotorSideReading left = read_motor_side(chassis.left_motors);
      const MotorSideReading right = read_motor_side(chassis.right_motors);
      if (!left.trustworthy || !right.trustworthy) break;
      const double left_travel_in =
          ((left.position_deg - left_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kLeftEncoderSign;
      const double right_travel_in =
          ((right.position_deg - right_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kRightEncoderSign;
      const double travel_in = 0.5 * (left_travel_in + right_travel_in);
      if (travel_in >= kRamMaximumTravelIn) {
        travel_limit = true;
        break;
      }
      for (auto& motor : chassis.left_motors) {
        maximum_current_ma = std::max(
            maximum_current_ma, motor.get_current_draw());
      }
      for (auto& motor : chassis.right_motors) {
        maximum_current_ma = std::max(
            maximum_current_ma, motor.get_current_draw());
      }
      if (maximum_current_ma >= kRamCurrentLimitMa) {
        current_limit = true;
        break;
      }
      const navigation::Pose current = navigation::current_pose();
      const double heading_error_deg = signed_angle_diff_deg(
          start_heading_deg, current.heading_deg);
      set_physical_drive_power(45, clamp_power(heading_error_deg * 1.2));
      pros::delay(20);
    }
    stop_drive_motors();
    navigation::update();
    const MotorSideReading left_stop = read_motor_side(chassis.left_motors);
    const MotorSideReading right_stop = read_motor_side(chassis.right_motors);
    if (!left_stop.trustworthy || !right_stop.trustworthy) return false;
    const double ram_travel_in = 0.5 * (
        ((left_stop.position_deg - left_start.position_deg) / 360.0) *
            kWheelCircumferenceIn * kLeftEncoderSign +
        ((right_stop.position_deg - right_start.position_deg) / 360.0) *
            kWheelCircumferenceIn * kRightEncoderSign);
    const double p1_contact_in =
        static_cast<double>(distance_1.get_distance()) / kMillimetersPerInch;
    std::printf(
        "TOGGLE_GOAL event=ram cycle=%d travel=%.3f p1=%.3f "
        "max_current=%ld current_limit=%d travel_limit=%d\n",
        cycle_index, ram_travel_in, p1_contact_in,
        static_cast<long>(maximum_current_ma),
        static_cast<int>(current_limit), static_cast<int>(travel_limit));
    std::fflush(stdout);

    // Do not count a suppressed/no-op command as a completed toggle cycle.
    // Encoder travel proves motion; the current trip proves contact when the
    // toggle prevents measurable travel.
    const bool contact_motion_proven =
        ram_travel_in >= 0.25 || current_limit;
    if (!contact_motion_proven) {
      std::printf(
          "TOGGLE_GOAL abort=ram_no_motion cycle=%d travel=%.3f "
          "max_current=%ld\n",
          cycle_index, ram_travel_in,
          static_cast<long>(maximum_current_ma));
      std::fflush(stdout);
      return false;
    }

    if (!begin_internal_motion("toggle_cycle_return")) return false;
    const bool returned = fused_drive_to_point(
        telemetry_pose, "toggle_cycle_return",
        Waypoint{cycle_start.x_in, cycle_start.y_in}, start_heading_deg,
        50, 5000, 3.0, -1);
    stop_drive_motors();
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const double loop_error_in = std::hypot(
        after.x_in - cycle_start.x_in, after.y_in - cycle_start.y_in);
    std::printf(
        "TOGGLE_GOAL event=cycle_complete cycle=%d returned=%d "
        "loop_error=%.3f heading_error=%.3f x=%.3f y=%.3f heading=%.3f\n",
        cycle_index, static_cast<int>(returned), loop_error_in,
        std::fabs(signed_angle_diff_deg(start_heading_deg,
                                       after.heading_deg)),
        after.x_in, after.y_in, after.heading_deg);
    std::fflush(stdout);
    return returned && loop_error_in <= 1.25;
  };

  bool route_ok = ram_and_return(1);
  if (route_ok) route_ok = ram_and_return(2);

  if (route_ok) {
    route_ok = begin_internal_motion("toggle_clearance_reverse");
  }
  if (route_ok) {
    navigation::update();
    const navigation::Pose pose = navigation::current_pose();
    const double heading_rad = deg_to_rad(pose.heading_deg);
    const Waypoint clearance_target{
        pose.x_in - kTurnClearanceReverseIn * std::cos(heading_rad),
        pose.y_in - kTurnClearanceReverseIn * std::sin(heading_rad)};
    route_ok = fused_drive_to_point(
        telemetry_pose, "toggle_clearance_reverse", clearance_target,
        pose.heading_deg, 55, 9000, 4.0, -1);
    stop_drive_motors();
    std::printf("TOGGLE_GOAL event=clearance ok=%d target=%.3f,%.3f\n",
                static_cast<int>(route_ok), clearance_target.x,
                clearance_target.y);
    std::fflush(stdout);
  }

  const double opposite_heading_deg = normalize_deg(start_heading_deg + 180.0);
  navigation::Result turn_result = navigation::Result::kDriveFailed;
  navigation::Result waypoint_result = navigation::Result::kDriveFailed;
  navigation::Result approach_result = navigation::Result::kDriveFailed;
  navigation::Result final_heading_result = navigation::Result::kDriveFailed;
  if (route_ok) {
    turn_result = navigation::turn_to(opposite_heading_deg, 55, 7000);
    route_ok = turn_result == navigation::Result::kSuccess;
  }
  if (route_ok) {
    waypoint_result = navigation::go_straight_to(24.0, 24.0, 60, 12000);
    route_ok = waypoint_result == navigation::Result::kSuccess;
  }
  if (route_ok) {
    approach_result = navigation::go_straight_to(
        kGoalApproachXIn, kGoalApproachYIn, 60, 15000);
    route_ok = approach_result == navigation::Result::kSuccess;
  }
  if (route_ok) {
    final_heading_result = navigation::turn_to(
        kGoalApproachHeadingDeg, 50, 7000);
    route_ok = final_heading_result == navigation::Result::kSuccess;
  }

  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "TOGGLE_GOAL event=complete ok=%d turn=%s waypoint=%s approach=%s "
      "final_heading=%s x=%.3f y=%.3f heading=%.3f "
      "approach_error=%.3f heading_error=%.3f goal_center=%.1f,%.1f "
      "goal_center_distance=%.3f\n",
      static_cast<int>(route_ok), navigation::result_name(turn_result),
      navigation::result_name(waypoint_result),
      navigation::result_name(approach_result),
      navigation::result_name(final_heading_result), final_pose.x_in,
      final_pose.y_in, final_pose.heading_deg,
      std::hypot(final_pose.x_in - kGoalApproachXIn,
                 final_pose.y_in - kGoalApproachYIn),
      std::fabs(signed_angle_diff_deg(kGoalApproachHeadingDeg,
                                     final_pose.heading_deg)),
      kGoalCenterXIn, kGoalCenterYIn,
      std::hypot(final_pose.x_in - kGoalCenterXIn,
                 final_pose.y_in - kGoalCenterYIn));
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

bool localization_path1_opening_tuning_test() {
  constexpr localization::FieldPose kExactStart{63.0, -8.0, 0.0};
  constexpr double kStartErrorEnvelopeIn = 0.5;
  constexpr double kRamTravelLimitIn = 7.0;
  constexpr std::uint32_t kRamTimeoutMs = 2000;
  constexpr std::int32_t kRamCurrentLimitMa = 2400;
  constexpr double kReverseDistanceIn = 6.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating()) {
    std::printf("PATH1_OPEN abort=imu_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  if (!navigation::init(kExactStart.x_in, kExactStart.y_in,
                        kExactStart.heading_deg,
                        kStartErrorEnvelopeIn)) {
    std::printf("PATH1_OPEN abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }
  navigation_stop_requested.store(false, std::memory_order_release);
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  navigation::update();
  const navigation::Pose anchored = navigation::current_pose();
  std::printf(
      "PATH1_OPEN event=anchor x=%.3f y=%.3f heading=%.3f valid=%d\n",
      anchored.x_in, anchored.y_in, anchored.heading_deg,
      static_cast<int>(anchored.valid));
  std::fflush(stdout);

  const MotorSideReading ram_left_start =
      read_motor_side(chassis.left_motors);
  const MotorSideReading ram_right_start =
      read_motor_side(chassis.right_motors);
  if (!ram_left_start.trustworthy || !ram_right_start.trustworthy) {
    stop_drive_motors();
    return false;
  }
  std::int32_t maximum_current_ma = 0;
  bool current_limit = false;
  bool travel_limit = false;
  std::uint32_t current_over_limit_since_ms = 0;
  const std::uint32_t ram_started_ms = pros::millis();
  while (pros::millis() - ram_started_ms < kRamTimeoutMs) {
    navigation::update();
    if (blocking_motion_abort_requested("path1_toggle_ram")) break;
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    if (!left.trustworthy || !right.trustworthy) break;
    const double left_in =
        ((left.position_deg - ram_left_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kLeftEncoderSign;
    const double right_in =
        ((right.position_deg - ram_right_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kRightEncoderSign;
    const double travel_in = 0.5 * (left_in + right_in);
    if (travel_in >= kRamTravelLimitIn) {
      travel_limit = true;
      break;
    }
    std::int32_t instantaneous_current_ma = 0;
    for (auto& motor : chassis.left_motors) {
      instantaneous_current_ma = std::max(instantaneous_current_ma,
                                          motor.get_current_draw());
    }
    for (auto& motor : chassis.right_motors) {
      instantaneous_current_ma = std::max(instantaneous_current_ma,
                                          motor.get_current_draw());
    }
    maximum_current_ma = std::max(maximum_current_ma,
                                  instantaneous_current_ma);
    if (travel_in >= 4.5 && instantaneous_current_ma >= kRamCurrentLimitMa) {
      if (current_over_limit_since_ms == 0) {
        current_over_limit_since_ms = pros::millis();
      } else if (pros::millis() - current_over_limit_since_ms >= 80) {
        current_limit = true;
        break;
      }
    } else {
      current_over_limit_since_ms = 0;
    }
    const navigation::Pose pose = navigation::current_pose();
    const double heading_error_deg = signed_angle_diff_deg(
        kExactStart.heading_deg, pose.heading_deg);
    const double remaining_in = kRamTravelLimitIn - travel_in;
    const int ram_power = remaining_in > 3.0
        ? 127
        : clamp_power(std::clamp(28.0 + remaining_in * 28.0,
                                 28.0, 100.0));
    set_physical_drive_power(
        ram_power, clamp_power(heading_error_deg * 1.2));
    pros::delay(10);
  }
  stop_drive_motors();
  pros::delay(250);
  navigation::update();
  const MotorSideReading ram_left_stop =
      read_motor_side(chassis.left_motors);
  const MotorSideReading ram_right_stop =
      read_motor_side(chassis.right_motors);
  if (!ram_left_stop.trustworthy || !ram_right_stop.trustworthy) return false;
  const double ram_travel_in = 0.5 * (
      ((ram_left_stop.position_deg - ram_left_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kLeftEncoderSign +
      ((ram_right_stop.position_deg - ram_right_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kRightEncoderSign);
  const navigation::Pose after_ram = navigation::current_pose();
  std::printf(
      "PATH1_OPEN event=ram travel=%.3f current=%ld current_limit=%d "
      "travel_limit=%d x=%.3f y=%.3f heading=%.3f\n",
      ram_travel_in, static_cast<long>(maximum_current_ma),
      static_cast<int>(current_limit), static_cast<int>(travel_limit),
      after_ram.x_in, after_ram.y_in, after_ram.heading_deg);
  std::fflush(stdout);
  if (ram_travel_in < 4.5 && !current_limit) return false;

  const double heading_rad = deg_to_rad(after_ram.heading_deg);
  const Waypoint reverse_target{
      after_ram.x_in - kReverseDistanceIn * std::cos(heading_rad),
      after_ram.y_in - kReverseDistanceIn * std::sin(heading_rad)};
  navigation_stop_requested.store(false, std::memory_order_release);
  const bool reverse_ok = fused_drive_to_point(
      telemetry_pose, "path1_reverse_6", reverse_target,
      after_ram.heading_deg, 70, 5000, 3.0, -1,
      LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  pros::delay(250);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double reverse_error_in = std::hypot(
      final_pose.x_in - reverse_target.x,
      final_pose.y_in - reverse_target.y);
  std::printf(
      "PATH1_OPEN event=complete reverse_ok=%d target=%.3f,%.3f "
      "x=%.3f y=%.3f heading=%.3f reverse_error=%.3f\n",
      static_cast<int>(reverse_ok), reverse_target.x, reverse_target.y,
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg,
      reverse_error_in);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return reverse_ok && reverse_error_in <= 1.25;
}

bool localization_path1_goal_turn_tuning_test() {
  // The first backside pivot was fail-closed when the Toggle-side appendage
  // physically blocked rotation. Reverse along the current chassis axis into
  // open floor before completing the rear-first Goal orientation.
  constexpr localization::FieldPose kMeasuredStart{
      62.660, -7.160, 41.340};
  constexpr double kClearanceReverseIn = 4.0;
  constexpr double kTargetHeadingDeg = 90.0;
  constexpr double kStartErrorEnvelopeIn = 0.75;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating()) {
    std::printf("PATH1_TURN abort=imu_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  if (!navigation::init(kMeasuredStart.x_in, kMeasuredStart.y_in,
                        kMeasuredStart.heading_deg,
                        kStartErrorEnvelopeIn)) {
    std::printf("PATH1_TURN abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }
  navigation_stop_requested.store(false, std::memory_order_release);
  navigation::update();
  const navigation::Pose anchored = navigation::current_pose();
  std::printf(
      "PATH1_TURN event=anchor x=%.3f y=%.3f heading=%.3f valid=%d\n",
      anchored.x_in, anchored.y_in, anchored.heading_deg,
      static_cast<int>(anchored.valid));
  std::fflush(stdout);

  const double start_heading_rad = deg_to_rad(kMeasuredStart.heading_deg);
  const Waypoint clearance_target{
      kMeasuredStart.x_in - kClearanceReverseIn * std::cos(start_heading_rad),
      kMeasuredStart.y_in - kClearanceReverseIn * std::sin(start_heading_rad)};
  bool clearance_ok = fused_drive_to_point(
      telemetry_pose, "path1_backside_clearance", clearance_target,
      kMeasuredStart.heading_deg, 45, 4500, 2.5, -1,
      LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  pros::delay(250);
  bool turn_ok = false;
  if (clearance_ok) {
    turn_ok = fused_turn_to_heading(
        telemetry_pose, "path1_backside_turn_retry", kTargetHeadingDeg,
        50, 6500, LidarFusionMode::kBiasOnly);
  }
  stop_drive_motors();
  pros::delay(300);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double heading_error_deg = std::fabs(signed_angle_diff_deg(
      kTargetHeadingDeg, final_pose.heading_deg));
  const double position_shift_in = std::hypot(
      final_pose.x_in - kMeasuredStart.x_in,
      final_pose.y_in - kMeasuredStart.y_in);
  std::printf(
      "PATH1_TURN event=complete clearance_ok=%d turn_ok=%d x=%.3f y=%.3f heading=%.3f "
      "heading_error=%.3f position_shift=%.3f\n",
      static_cast<int>(clearance_ok), static_cast<int>(turn_ok),
      final_pose.x_in, final_pose.y_in,
      final_pose.heading_deg, heading_error_deg, position_shift_in);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return clearance_ok && turn_ok && heading_error_deg <= 1.5;
}

bool localization_path1_goal_approach_tuning_test() {
  // Measured endpoint of the camera-verified blue->red Toggle flip and
  // six-inch backout. Goal 3 must receive the robot's rear mechanism, so turn
  // the front away from the Goal and reverse into it.
  constexpr localization::FieldPose kMeasuredStart{
      59.647, -9.767, 89.951};
  constexpr double kTargetHeadingDeg = 90.0;
  constexpr double kTravelLimitIn = 7.0;
  constexpr std::uint32_t kTimeoutMs = 2200;
  constexpr std::int32_t kContactCurrentMa = 2400;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating()) {
    std::printf("PATH1_APPROACH abort=imu_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  if (!navigation::init(kMeasuredStart.x_in, kMeasuredStart.y_in,
                        kMeasuredStart.heading_deg, 0.75)) {
    std::printf("PATH1_APPROACH abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }
  navigation_stop_requested.store(false, std::memory_order_release);
  navigation::update();
  const bool turn_ok = fused_turn_to_heading(
      telemetry_pose, "path1_backside_turn", kTargetHeadingDeg,
      60, 6500, LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  pros::delay(300);
  navigation::update();
  if (!turn_ok) {
    std::printf("PATH1_APPROACH abort=backside_turn\n");
    std::fflush(stdout);
    navigation::stop();
    return false;
  }
  const MotorSideReading left_start = read_motor_side(chassis.left_motors);
  const MotorSideReading right_start = read_motor_side(chassis.right_motors);
  if (!left_start.trustworthy || !right_start.trustworthy) {
    stop_drive_motors();
    return false;
  }

  bool current_limit = false;
  bool travel_limit = false;
  std::int32_t maximum_current_ma = 0;
  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < kTimeoutMs) {
    navigation::update();
    if (blocking_motion_abort_requested("path1_goal_approach")) break;
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    if (!left.trustworthy || !right.trustworthy) break;
    const double left_in =
        ((left.position_deg - left_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kLeftEncoderSign;
    const double right_in =
        ((right.position_deg - right_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kRightEncoderSign;
    const double travel_in = -0.5 * (left_in + right_in);
    if (travel_in >= kTravelLimitIn) {
      travel_limit = true;
      break;
    }
    for (auto& motor : chassis.left_motors) {
      maximum_current_ma = std::max(maximum_current_ma,
                                    motor.get_current_draw());
    }
    for (auto& motor : chassis.right_motors) {
      maximum_current_ma = std::max(maximum_current_ma,
                                    motor.get_current_draw());
    }
    const std::uint32_t elapsed_ms = pros::millis() - started_ms;
    if (maximum_current_ma >= kContactCurrentMa &&
        (elapsed_ms >= 150 || travel_in >= 1.0)) {
      current_limit = true;
      break;
    }
    const navigation::Pose pose = navigation::current_pose();
    const double heading_error_deg = signed_angle_diff_deg(
        kTargetHeadingDeg, pose.heading_deg);
    const double remaining_in = kTravelLimitIn - travel_in;
    const int reverse_power = remaining_in > 2.0 ? -55 : -32;
    set_physical_drive_power(reverse_power,
                             clamp_power(heading_error_deg * 1.2));
    pros::delay(10);
  }
  stop_drive_motors();
  pros::delay(300);
  navigation::update();
  const MotorSideReading left_stop = read_motor_side(chassis.left_motors);
  const MotorSideReading right_stop = read_motor_side(chassis.right_motors);
  if (!left_stop.trustworthy || !right_stop.trustworthy) return false;
  const double travel_in = -0.5 * (
      ((left_stop.position_deg - left_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kLeftEncoderSign +
      ((right_stop.position_deg - right_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kRightEncoderSign);
  const navigation::Pose final_pose = navigation::current_pose();
  const double heading_error_deg = std::fabs(signed_angle_diff_deg(
      kTargetHeadingDeg, final_pose.heading_deg));
  std::printf(
      "PATH1_APPROACH event=complete backside=1 turn_ok=%d travel=%.3f current=%ld "
      "current_limit=%d travel_limit=%d x=%.3f y=%.3f heading=%.3f "
      "heading_error=%.3f\n",
      static_cast<int>(turn_ok), travel_in,
      static_cast<long>(maximum_current_ma),
      static_cast<int>(current_limit), static_cast<int>(travel_limit),
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg,
      heading_error_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return (travel_limit || current_limit) && travel_in >= 1.0 &&
         heading_error_deg <= 2.0;
}

bool localization_path1_goal_outtake_test() {
  constexpr int kOuttakePower = 35;
  constexpr std::uint32_t kOuttakeDurationMs = 1400;
  constexpr std::int32_t kMechanismCurrentStopMa = 2300;
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  counter_rollers.move(0);
  if (!counter_rollers.is_installed()) {
    std::printf("PATH1_OUTTAKE abort=p3_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  const double start_deg = counter_rollers.get_position();
  std::int32_t peak_current_ma = 0;
  bool current_stop = false;
  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < kOuttakeDurationMs) {
    if (blocking_motion_abort_requested("path1_goal_outtake")) break;
    const std::int32_t current_ma = counter_rollers.get_current_draw();
    peak_current_ma = std::max(peak_current_ma, current_ma);
    if (current_ma >= kMechanismCurrentStopMa) {
      current_stop = true;
      break;
    }
    counter_rollers.move(kOuttakePower);  // L1/outtake direction.
    pros::delay(10);
  }
  counter_rollers.move(0);
  stop_drive_motors();
  pros::delay(300);
  const double end_deg = counter_rollers.get_position();
  const bool moved = std::isfinite(start_deg) && std::isfinite(end_deg) &&
                     std::fabs(end_deg - start_deg) >= 20.0;
  std::printf(
      "PATH1_OUTTAKE event=complete power=%d duration_ms=%lu "
      "start_deg=%.2f end_deg=%.2f delta_deg=%.2f peak_current=%ld "
      "current_stop=%d moved=%d\n",
      kOuttakePower, static_cast<unsigned long>(pros::millis() - started_ms),
      start_deg, end_deg, end_deg - start_deg,
      static_cast<long>(peak_current_ma), static_cast<int>(current_stop),
      static_cast<int>(moved));
  std::fflush(stdout);
  return moved && !current_stop;
}

bool localization_path1_return_to_exact_start_test() {
  constexpr localization::FieldPose kRejectedApproachEndpoint{
      58.959, -14.688, 254.784};
  constexpr Waypoint kAfterOpening{59.303, -8.042};
  constexpr Waypoint kExactStart{63.0, -8.0};

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating()) {
    std::printf("PATH1_RESET abort=imu_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  if (!navigation::init(kRejectedApproachEndpoint.x_in,
                        kRejectedApproachEndpoint.y_in,
                        kRejectedApproachEndpoint.heading_deg, 1.0)) {
    std::printf("PATH1_RESET abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }

  navigation_stop_requested.store(false, std::memory_order_release);
  bool ok = fused_drive_to_point(
      telemetry_pose, "path1_reset_reverse", kAfterOpening,
      270.0, 55, 5500, 3.0, -1, LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  if (ok) {
    navigation_stop_requested.store(false, std::memory_order_release);
    ok = fused_turn_to_heading(
        telemetry_pose, "path1_reset_turn_zero", 0.0,
        45, 6500, LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }
  if (ok) {
    navigation_stop_requested.store(false, std::memory_order_release);
    ok = fused_drive_to_point(
        telemetry_pose, "path1_reset_to_start", kExactStart,
        0.0, 45, 4500, 2.5, 1, LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }
  pros::delay(300);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double position_error_in = std::hypot(
      final_pose.x_in - kExactStart.x,
      final_pose.y_in - kExactStart.y);
  const double heading_error_deg = std::fabs(signed_angle_diff_deg(
      0.0, final_pose.heading_deg));
  std::printf(
      "PATH1_RESET event=complete ok=%d x=%.3f y=%.3f heading=%.3f "
      "position_error=%.3f heading_error=%.3f\n",
      static_cast<int>(ok), final_pose.x_in, final_pose.y_in,
      final_pose.heading_deg, position_error_in, heading_error_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return ok && position_error_in <= 1.0 && heading_error_deg <= 1.5;
}

bool localization_path1_opening_return_to_start_test() {
  constexpr localization::FieldPose kMeasuredStart{
      67.082, -7.945, 359.475};
  constexpr Waypoint kExactStart{63.0, -8.0};
  constexpr double kToggleFacingHeadingDeg = 180.0;
  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating()) {
    std::printf("PATH1_OPEN_RESET abort=imu_unavailable\n");
    std::fflush(stdout);
    return false;
  }
  if (!navigation::init(kMeasuredStart.x_in, kMeasuredStart.y_in,
                        kMeasuredStart.heading_deg, 0.75)) {
    std::printf("PATH1_OPEN_RESET abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }
  navigation_stop_requested.store(false, std::memory_order_release);
  bool ok = fused_drive_to_point(
      telemetry_pose, "path1_open_reset_reverse", kExactStart,
      0.0, 50, 4500, 2.5, -1, LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  if (ok) {
    navigation_stop_requested.store(false, std::memory_order_release);
    ok = fused_turn_to_heading(
        telemetry_pose, "path1_face_toggle", kToggleFacingHeadingDeg,
        50, 6500, LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }
  pros::delay(300);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double position_error_in = std::hypot(
      final_pose.x_in - kExactStart.x, final_pose.y_in - kExactStart.y);
  const double heading_error_deg = std::fabs(signed_angle_diff_deg(
      kToggleFacingHeadingDeg, final_pose.heading_deg));
  std::printf(
      "PATH1_OPEN_RESET event=complete ok=%d x=%.3f y=%.3f heading=%.3f "
      "position_error=%.3f heading_error=%.3f\n",
      static_cast<int>(ok), final_pose.x_in, final_pose.y_in,
      final_pose.heading_deg, position_error_in, heading_error_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return ok && position_error_in <= 0.8 && heading_error_deg <= 1.5;
}

bool localization_path1_toggle_finish_probe_test() {
  constexpr localization::FieldPose kLocalAnchor{63.0, -8.0, 0.0};
  constexpr double kMaximumProbeTravelIn = 14.0;
  constexpr std::int32_t kContactCurrentMa = 1500;
  constexpr double kReverseDistanceIn = 6.0;
  constexpr double kRangeStopIn = 1.0;
  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!navigation::init(kLocalAnchor.x_in, kLocalAnchor.y_in,
                        kLocalAnchor.heading_deg, 0.75)) {
    std::printf("PATH1_TOGGLE_PROBE abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  const MotorSideReading left_start = read_motor_side(chassis.left_motors);
  const MotorSideReading right_start = read_motor_side(chassis.right_motors);
  if (!left_start.trustworthy || !right_start.trustworthy) return false;
  navigation_stop_requested.store(false, std::memory_order_release);
  bool contact = false;
  bool travel_limit = false;
  bool range_stop = false;
  bool wrong_direction = false;
  std::uint32_t over_current_since_ms = 0;
  std::int32_t maximum_current_ma = 0;
  const double initial_p1_in =
      static_cast<double>(distance_1.get_distance()) / 25.4;
  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < 6500) {
    navigation::update();
    if (blocking_motion_abort_requested("path1_toggle_finish_probe")) break;
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    if (!left.trustworthy || !right.trustworthy) break;
    const double left_in =
        ((left.position_deg - left_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kLeftEncoderSign;
    const double right_in =
        ((right.position_deg - right_start.position_deg) / 360.0) *
        kWheelCircumferenceIn * kRightEncoderSign;
    const double travel_in = 0.5 * (left_in + right_in);
    if (travel_in >= kMaximumProbeTravelIn) {
      travel_limit = true;
      break;
    }
    const double p1_in =
        static_cast<double>(distance_1.get_distance()) / 25.4;
    const bool p1_valid = distance_1.is_installed() &&
        std::isfinite(p1_in) && p1_in >= 20.0 / 25.4 && p1_in <= 78.75;
    if (travel_in >= 1.0 && p1_valid &&
        p1_in > initial_p1_in + 1.0) {
      wrong_direction = true;
      break;
    }
    if (p1_valid && p1_in <= kRangeStopIn) {
      range_stop = true;
      break;
    }
    std::int32_t instantaneous_current_ma = 0;
    for (auto& motor : chassis.left_motors) {
      instantaneous_current_ma = std::max(
          instantaneous_current_ma, motor.get_current_draw());
    }
    for (auto& motor : chassis.right_motors) {
      instantaneous_current_ma = std::max(
          instantaneous_current_ma, motor.get_current_draw());
    }
    maximum_current_ma = std::max(maximum_current_ma,
                                  instantaneous_current_ma);
    if (travel_in >= 0.5 && instantaneous_current_ma >= kContactCurrentMa) {
      if (over_current_since_ms == 0) {
        over_current_since_ms = pros::millis();
      } else if (pros::millis() - over_current_since_ms >= 120) {
        contact = true;
        break;
      }
    } else {
      over_current_since_ms = 0;
    }
    const navigation::Pose pose = navigation::current_pose();
    const double heading_error_deg = signed_angle_diff_deg(
        0.0, pose.heading_deg);
    const int approach_power = p1_valid && p1_in <= 3.0 ? 30 : 55;
    set_physical_drive_power(
        approach_power, clamp_power(heading_error_deg * 1.5));
    pros::delay(10);
  }
  stop_drive_motors();
  pros::delay(200);
  navigation::update();
  const MotorSideReading left_contact = read_motor_side(chassis.left_motors);
  const MotorSideReading right_contact = read_motor_side(chassis.right_motors);
  if (!left_contact.trustworthy || !right_contact.trustworthy) return false;
  const double probe_travel_in = 0.5 * (
      ((left_contact.position_deg - left_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kLeftEncoderSign +
      ((right_contact.position_deg - right_start.position_deg) / 360.0) *
          kWheelCircumferenceIn * kRightEncoderSign);
  const navigation::Pose after_probe = navigation::current_pose();
  const double heading_rad = deg_to_rad(after_probe.heading_deg);
  const Waypoint reverse_target{
      after_probe.x_in - kReverseDistanceIn * std::cos(heading_rad),
      after_probe.y_in - kReverseDistanceIn * std::sin(heading_rad)};
  navigation_stop_requested.store(false, std::memory_order_release);
  const bool reverse_ok = fused_drive_to_point(
      telemetry_pose, "path1_toggle_probe_reverse", reverse_target,
      after_probe.heading_deg, 60, 5500, 3.0, -1,
      LidarFusionMode::kBiasOnly);
  stop_drive_motors();
  pros::delay(250);
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double reverse_error_in = std::hypot(
      final_pose.x_in - reverse_target.x,
      final_pose.y_in - reverse_target.y);
  std::printf(
      "PATH1_TOGGLE_PROBE event=complete contact=%d travel_limit=%d "
      "range_stop=%d wrong_direction=%d initial_p1=%.3f probe_travel=%.3f "
      "max_current=%ld reverse_ok=%d reverse_error=%.3f "
      "x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(contact), static_cast<int>(travel_limit),
      static_cast<int>(range_stop), static_cast<int>(wrong_direction),
      initial_p1_in, probe_travel_in,
      static_cast<long>(maximum_current_ma),
      static_cast<int>(reverse_ok), reverse_error_in,
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return !wrong_direction && reverse_ok && reverse_error_in <= 1.0 &&
         (contact || range_stop || travel_limit);
}

void localization_toggle_goal_continue_auton() {
  // Measured fused endpoint of the fail-closed first run. The robot has not
  // moved since that run; SafeTuned rebooted afterward, so its local encoder
  // and IMU zeros must be re-anchored to this saved field pose.
  constexpr double kResumeXIn = -6.711;
  constexpr double kResumeYIn = 31.985;
  constexpr double kResumeHeadingDeg = 339.726;
  constexpr double kResumeErrorEnvelopeIn = 3.0;
  constexpr double kGoalApproachXIn = 24.0;
  constexpr double kGoalApproachYIn = -24.0;
  constexpr double kGoalHeadingDeg = 270.0;
  constexpr double kMinimumOpenRangeIn = 24.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();

  const MotorSideReading left = read_motor_side(chassis.left_motors);
  const MotorSideReading right = read_motor_side(chassis.right_motors);
  const double p1_in = static_cast<double>(distance_1.get_distance()) / 25.4;
  const bool preflight_ok = left.trustworthy && right.trustworthy &&
      chassis.imu.is_installed() && !chassis.imu.is_calibrating() &&
      gps_7.is_installed() && distance_1.is_installed() &&
      std::isfinite(p1_in) && p1_in >= kMinimumOpenRangeIn;
  std::printf(
      "TOGGLE_CONTINUE event=preflight ok=%d anchor=%.3f,%.3f,%.3f "
      "p1=%.3f drive=%d/%d imu=%d gps=%d\n",
      static_cast<int>(preflight_ok), kResumeXIn, kResumeYIn,
      kResumeHeadingDeg, p1_in, static_cast<int>(left.trustworthy),
      static_cast<int>(right.trustworthy),
      static_cast<int>(chassis.imu.is_installed() &&
                       !chassis.imu.is_calibrating()),
      static_cast<int>(gps_7.is_installed()));
  std::fflush(stdout);
  if (!preflight_ok) return;

  bool route_ok = navigation::init(
      kResumeXIn, kResumeYIn, kResumeHeadingDeg,
      kResumeErrorEnvelopeIn);
  navigation::Result waypoint_result = navigation::Result::kDriveFailed;
  navigation::Result approach_result = navigation::Result::kDriveFailed;
  navigation::Result heading_result = navigation::Result::kDriveFailed;
  if (route_ok) {
    const navigation::Result reposition_result =
        navigation::drive_relative(3.0, 45, 3500);
    route_ok = reposition_result == navigation::Result::kSuccess;
    std::printf("TOGGLE_CONTINUE event=reposition result=%s\n",
                navigation::result_name(reposition_result));
    std::fflush(stdout);
  }
  if (route_ok) {
    waypoint_result = navigation::go_straight_to(24.0, 24.0, 60, 12000);
    route_ok = waypoint_result == navigation::Result::kSuccess;
  }
  if (route_ok) {
    approach_result = navigation::go_straight_to(
        kGoalApproachXIn, kGoalApproachYIn, 60, 15000);
    route_ok = approach_result == navigation::Result::kSuccess;
  }
  if (route_ok) {
    heading_result = navigation::turn_to(kGoalHeadingDeg, 50, 7000);
    route_ok = heading_result == navigation::Result::kSuccess;
  }

  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "TOGGLE_CONTINUE event=complete ok=%d waypoint=%s approach=%s "
      "heading=%s x=%.3f y=%.3f h=%.3f approach_error=%.3f "
      "heading_error=%.3f\n",
      static_cast<int>(route_ok), navigation::result_name(waypoint_result),
      navigation::result_name(approach_result),
      navigation::result_name(heading_result), final_pose.x_in,
      final_pose.y_in, final_pose.heading_deg,
      std::hypot(final_pose.x_in - kGoalApproachXIn,
                 final_pose.y_in - kGoalApproachYIn),
      std::fabs(signed_angle_diff_deg(kGoalHeadingDeg,
                                     final_pose.heading_deg)));
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
}

bool localization_far_goal_trial_recover_to_start() {
  // Measured encoder/IMU endpoint from the first field-center-side trial.
  // This one-shot development helper returns through the clear corridor
  // between the two nearby Goals. P1 remains armed, so approaching the Toggle
  // wall stops at the eight-inch boundary even if the endpoint estimate is
  // optimistic.
  constexpr localization::FieldPose kTrialEndpoint{31.688, -4.015, 204.195};
  constexpr Waypoint kToggleStart{60.5, 0.25};
  constexpr std::array<Waypoint, 1> kReturnPoints{{kToggleStart}};
  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  if (!navigation::init(kTrialEndpoint.x_in, kTrialEndpoint.y_in,
                        kTrialEndpoint.heading_deg, 2.5)) {
    return false;
  }
  navigation_stop_requested.store(false, std::memory_order_release);
  const MotorSideReading unload_left =
      read_motor_side(chassis.left_motors);
  const MotorSideReading unload_right =
      read_motor_side(chassis.right_motors);
  if (!unload_left.trustworthy || !unload_right.trustworthy) return false;
  double unload_left_in = 0.0;
  double unload_right_in = 0.0;
  const std::uint32_t unload_started_ms = pros::millis();
  while (pros::millis() - unload_started_ms < 450) {
    update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    if (!left.trustworthy || !right.trustworthy) break;
    unload_left_in = std::fabs(
        ((left.position_deg - unload_left.position_deg) / 360.0) *
        kWheelCircumferenceIn * kLeftEncoderSign);
    unload_right_in = std::fabs(
        ((right.position_deg - unload_right.position_deg) / 360.0) *
        kWheelCircumferenceIn * kRightEncoderSign);
    if (std::fabs(unload_left_in - unload_right_in) >= 0.75 ||
        0.5 * (unload_left_in + unload_right_in) >= 2.0) {
      break;
    }
    set_physical_drive_power(-45, 0);
    pros::delay(10);
  }
  stop_drive_motors();
  log_drive_health("far_goal_recovery_unload");
  const bool drivetrain_freed = unload_left_in >= 0.20 &&
      unload_right_in >= 0.20 &&
      std::fabs(unload_left_in - unload_right_in) <= 0.50;
  std::printf(
      "FAR_GOAL_RECOVERY event=unload freed=%d left=%.3f right=%.3f\n",
      static_cast<int>(drivetrain_freed), unload_left_in, unload_right_in);
  std::fflush(stdout);
  if (!drivetrain_freed) return false;
  navigation_stop_requested.store(false, std::memory_order_release);
  bool ok = true;
  navigation_stop_requested.store(false, std::memory_order_release);
  const double return_heading_deg = normalize_deg(rad_to_deg(std::atan2(
      kReturnPoints[0].y - telemetry_pose.y,
      kReturnPoints[0].x - telemetry_pose.x)));
  ok = fused_turn_to_heading(
      telemetry_pose, "far_goal_recovery_face", return_heading_deg,
      127, 6000, LidarFusionMode::kBiasOnly);
  for (std::size_t index = 0; ok && index < kReturnPoints.size(); ++index) {
    navigation_stop_requested.store(false, std::memory_order_release);
    const Waypoint target = kReturnPoints[index];
    const double leg_heading_deg = normalize_deg(rad_to_deg(std::atan2(
        target.y - telemetry_pose.y, target.x - telemetry_pose.x)));
    ok = fused_drive_to_point(
        telemetry_pose, "far_goal_recovery_drive", target,
        leg_heading_deg, 85, 7000, 4.0, 1, LidarFusionMode::kBiasOnly);
  }
  stop_drive_motors();
  navigation::update();
  const auto pose = navigation::current_pose();
  const double p1_in = static_cast<double>(distance_1.get_distance()) / 25.4;
  // A P1 emergency stop 4-8.5 inches from the Toggle is also a successful,
  // safe recovery endpoint for the next route trial.
  const bool ready_for_trial = std::isfinite(p1_in) &&
      p1_in >= 4.0 && p1_in <= 8.5;
  std::printf(
      "FAR_GOAL_RECOVERY event=complete controller_ok=%d ready=%d "
      "x=%.3f y=%.3f h=%.3f p1=%.3f\n",
      static_cast<int>(ok), static_cast<int>(ready_for_trial),
      pose.x_in, pose.y_in, pose.heading_deg, p1_in);
  std::fflush(stdout);
  navigation::stop();
  stop_drive_motors();
  return ready_for_trial;
}

bool localization_pure_pursuit_endpoint_test() {
  // The start pose is the stationary P7-derived field placement visible in
  // the preflight frame. P7 is used to establish this one-time anchor only;
  // encoder/P6 propagation owns motion and the normal temporal gates decide
  // whether later GPS/P6 fixes are trustworthy enough for bounded correction.
  constexpr localization::FieldPose kStart{29.425, -7.067, 267.074};
  constexpr Waypoint kTarget{3.55, -47.09};
  constexpr double kTargetHeadingDeg = 270.0;
  // Smooth continuation tangent to the latest fail-closed endpoint. This
  // remaining leg already clears the center Goal, so a simple cubic avoids
  // the prior S-curve's steering reversal and converges gently to -Y.
  constexpr Waypoint kControl1{28.812, -19.051};
  constexpr Waypoint kControl2{3.55, -32.09};
  constexpr std::size_t kPathPointCount = 81;
  constexpr double kLookaheadIn = 16.0;
  constexpr double kCrossTrackLimitIn = 5.0;
  constexpr int kMaximumPower = 70;
  constexpr std::uint32_t kTimeoutMs = 18000;
  constexpr double kMinimumGoalCenterClearanceIn = 17.0;
  constexpr double kMinimumWallClearanceIn = 8.25;

  struct PursuitPoint {
    double x = 0.0;
    double y = 0.0;
    double s = 0.0;
  };
  std::array<PursuitPoint, kPathPointCount> path{};
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double t = static_cast<double>(index) /
        static_cast<double>(path.size() - 1);
    const double u = 1.0 - t;
    path[index].x =
        u * u * u * kStart.x_in +
        3.0 * u * u * t * kControl1.x +
        3.0 * u * t * t * kControl2.x +
        t * t * t * kTarget.x;
    path[index].y =
        u * u * u * kStart.y_in +
        3.0 * u * u * t * kControl1.y +
        3.0 * u * t * t * kControl2.y +
        t * t * t * kTarget.y;
    if (index > 0) {
      path[index].s = path[index - 1].s + std::hypot(
          path[index].x - path[index - 1].x,
          path[index].y - path[index - 1].y);
    }
  }

  double minimum_goal_clearance_in =
      std::numeric_limits<double>::infinity();
  double minimum_wall_clearance_in =
      std::numeric_limits<double>::infinity();
  for (const auto& point : path) {
    minimum_wall_clearance_in = std::min(
        minimum_wall_clearance_in,
        localization::kPhysicalWallHalfSpanIn -
            std::max(std::fabs(point.x), std::fabs(point.y)));
    for (const auto& goal : localization::kGoalTagLandmarks) {
      minimum_goal_clearance_in = std::min(
          minimum_goal_clearance_in,
          std::hypot(point.x - goal.x_in, point.y - goal.y_in));
    }
  }
  const bool geometry_safe =
      minimum_goal_clearance_in >= kMinimumGoalCenterClearanceIn &&
      minimum_wall_clearance_in >= kMinimumWallClearanceIn;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  std::printf(
      "PP_ENDPOINT event=preflight geometry=%d start=%.2f,%.2f,%.2f "
      "target=%.2f,%.2f,%.2f path_length=%.2f goal_clear=%.2f "
      "wall_clear=%.2f\n",
      static_cast<int>(geometry_safe), kStart.x_in, kStart.y_in,
      kStart.heading_deg, kTarget.x, kTarget.y, kTargetHeadingDeg,
      path.back().s, minimum_goal_clearance_in, minimum_wall_clearance_in);
  std::fflush(stdout);
  if (!geometry_safe || !chassis.imu.is_installed() ||
      chassis.imu.is_calibrating()) {
    return false;
  }
  if (!navigation::init(kStart.x_in, kStart.y_in, kStart.heading_deg, 2.0)) {
    std::printf("PP_ENDPOINT abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }

  navigation_stop_requested.store(false, std::memory_order_release);
  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_loop_ms = started_ms;
  std::uint32_t last_progress_ms = started_ms;
  std::uint32_t last_log_ms = 0;
  std::size_t nearest_index = 0;
  double last_progress_in = 0.0;
  double forward_command = 0.0;
  double turn_command = 0.0;
  bool arrived = false;
  bool ok = true;
  while (pros::millis() - started_ms < kTimeoutMs) {
    if (blocking_motion_abort_requested("pp_endpoint")) {
      ok = false;
      break;
    }
    update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
    if (motion_pose_invalid(telemetry_pose, "pp_endpoint")) {
      ok = false;
      break;
    }
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(
        0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;

    double nearest_distance_in = std::numeric_limits<double>::infinity();
    for (std::size_t index = nearest_index; index < path.size(); ++index) {
      const double distance = std::hypot(
          telemetry_pose.x - path[index].x,
          telemetry_pose.y - path[index].y);
      if (distance < nearest_distance_in) {
        nearest_distance_in = distance;
        nearest_index = index;
      }
    }
    const double progress_in = path[nearest_index].s;
    if (progress_in >= last_progress_in + kFusedDriveStallProgressIn) {
      last_progress_in = progress_in;
      last_progress_ms = now;
    }
    if (nearest_distance_in > kCrossTrackLimitIn) {
      std::printf("PP_ENDPOINT abort=cross_track distance=%.3f limit=%.3f\n",
                  nearest_distance_in, kCrossTrackLimitIn);
      std::fflush(stdout);
      ok = false;
      break;
    }

    const double endpoint_distance_in = std::hypot(
        telemetry_pose.x - kTarget.x, telemetry_pose.y - kTarget.y);
    if (endpoint_distance_in <= 1.5 && nearest_index + 2 >= path.size()) {
      arrived = true;
      break;
    }
    std::size_t carrot_index = nearest_index;
    const double carrot_s = progress_in + kLookaheadIn;
    while (carrot_index + 1 < path.size() &&
           path[carrot_index].s < carrot_s) {
      ++carrot_index;
    }
    const double dx = path[carrot_index].x - telemetry_pose.x;
    const double dy = path[carrot_index].y - telemetry_pose.y;
    const double desired_heading_deg = normalize_deg(rad_to_deg(
        std::atan2(dy, dx)));
    const double heading_error_deg = signed_angle_diff_deg(
        desired_heading_deg, pose_heading_deg(telemetry_pose));
    const double remaining_in = std::max(0.0, path.back().s - progress_in);
    double target_forward = std::min(
        static_cast<double>(kMaximumPower),
        std::max(kFusedDriveMinPower, remaining_in * 3.0));
    target_forward *= clamp(
        std::cos(deg_to_rad(std::min(75.0, std::fabs(heading_error_deg)))),
        0.30, 1.0);
    const double lookahead_distance_in = std::max(1.0, std::hypot(dx, dy));
    const double curvature_per_in =
        2.0 * std::sin(deg_to_rad(heading_error_deg)) /
        lookahead_distance_in;
    double target_turn = target_forward * curvature_per_in *
        kTrackWidthIn / 2.0;
    target_turn = clamp(target_turn, -35.0, 35.0);
    forward_command = apply_slew(
        target_forward, forward_command,
        kFusedDriveForwardSlewPowerPerSec * dt_s);
    turn_command = apply_slew(
        target_turn, turn_command,
        kFusedDriveTurnSlewPowerPerSec * dt_s);
    // This heavy skid-steer chassis binds if pure pursuit asks the inner side
    // to sit below its measured 28/127 rolling floor while the outer side
    // pushes hard. Preserve a rolling inner wheel; use a wider arc rather than
    // reproducing trial 2's 86/17-power current-limit stall.
    const double rolling_turn_limit = std::max(
        0.0, std::fabs(forward_command) - kFusedDriveMinPower);
    turn_command = clamp(
        turn_command, -rolling_turn_limit, rolling_turn_limit);

    if (forward_obstacle_requires_stop("pp_endpoint")) {
      ok = false;
      break;
    }
    if (std::fabs(forward_command) >= kFusedDriveMinPower * 0.75 &&
        now - last_progress_ms >= kFusedDriveStallTimeoutMs) {
      log_drive_health("pp_endpoint_stall");
      std::printf("PP_ENDPOINT abort=stall progress=%.3f command=%.2f\n",
                  progress_in, forward_command);
      std::fflush(stdout);
      ok = false;
      break;
    }
    set_physical_drive_power(
        clamp_power(forward_command), clamp_power(turn_command));
    if (now - last_log_ms >= kFusionTestLogPeriodMs) {
      std::printf(
          "PP_ENDPOINT controller=pure_pursuit progress=%.2f/%.2f "
          "cross=%.2f endpoint=%.2f heading_error=%.2f forward=%.1f "
          "turn=%.1f x=%.2f y=%.2f h=%.2f gps=%s ai=%s\n",
          progress_in, path.back().s, nearest_distance_in,
          endpoint_distance_in, heading_error_deg, forward_command,
          turn_command, telemetry_pose.x, telemetry_pose.y,
          pose_heading_deg(telemetry_pose), telemetry_pose.gps_reject,
          telemetry_pose.ai_reject);
      std::fflush(stdout);
      last_log_ms = now;
    }
    pros::delay(20);
  }
  stop_drive_motors();
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double final_error_in = std::hypot(
      final_pose.x_in - kTarget.x, final_pose.y_in - kTarget.y);
  std::printf(
      "PP_ENDPOINT event=complete ok=%d arrived=%d x=%.3f y=%.3f h=%.3f "
      "target=%.2f,%.2f error=%.3f\n",
      static_cast<int>(ok), static_cast<int>(arrived), final_pose.x_in,
      final_pose.y_in, final_pose.heading_deg, kTarget.x, kTarget.y,
      final_error_in);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return ok && arrived && final_error_in <= 2.0;
}

bool localization_simple_red_goal_hotkey_auton() {
  // Repositioned one-pin route. A twelve-inch backward vector at +60 degrees
  // terminates at the requested Goal coordinate (48,-24), so its matching
  // start anchor is (54,-13.6077) at heading zero after returning from Toggle.
  constexpr localization::FieldPose kStart{54.0, -13.6076952, 0.0};
  constexpr Waypoint kGoal{48.0, -24.0};
  constexpr Waypoint kFinalStack{24.0, 0.0};
  constexpr double kGoalTurnDeg = 60.0;
  constexpr double kGoalContactIn = 12.0;
  constexpr double kGoalExitIn = 13.5;
  constexpr double kStackReverseIn = 24.0;
  constexpr double kStackClampLeadIn = 1.0;

  default_constants();
  stop_drive_motors();
  set_claw_piston(false);
  if (!navigation::init(kStart.x_in, kStart.y_in,
                        kStart.heading_deg, 0.75)) {
    return false;
  }

  // Preserve the required Toggle contact, then return to the exact starting
  // position before turning toward the nearby Goal. The repositioned start is
  // inside the mapped near-Goal buffer, so both short legs explicitly allow
  // Goal proximity; otherwise navigation rejects them before motor output.
  const navigation::Result toggle_ram = navigation::drive_relative(
      6.0, 78, 1800, false, true, true);
  const bool toggle_contacted =
      toggle_ram == navigation::Result::kSuccess ||
      toggle_ram == navigation::Result::kDriveFailed;
  if (!toggle_contacted) {
    navigation::stop();
    return false;
  }
  const navigation::Result toggle_return = navigation::drive_relative(
      -6.0, 68, 2600, true, true, true);
  const bool toggle_returned =
      toggle_return == navigation::Result::kSuccess ||
      toggle_return == navigation::Result::kDriveFailed;
  if (!toggle_returned) {
    navigation::stop();
    return false;
  }

  // Turn directly toward the Goal only after completing the Toggle sequence.
  const navigation::Result goal_turn = navigation::turn_to(
      kGoalTurnDeg, 100, 1800, true, true);
  const bool goal_turn_attempted =
      goal_turn == navigation::Result::kSuccess ||
      goal_turn == navigation::Result::kTurnFailed;
  if (!goal_turn_attempted) {
    navigation::stop();
    return false;
  }

  // Deliberate Goal contact. A stall at twelve inches is a valid finish.
  const navigation::Result goal_contact = navigation::drive_relative(
      -kGoalContactIn, 80, 2200, true, true, true);
  const bool contacted = goal_contact == navigation::Result::kSuccess ||
                         goal_contact == navigation::Result::kDriveFailed;
  if (!contacted) {
    navigation::stop();
    return false;
  }
  set_claw_piston(true);
  pros::delay(250);

  // Drive forward 13.5 inches away from the Goal to create turning space.
  const navigation::Result goal_retreat = navigation::drive_relative(
      kGoalExitIn, 72, 3000, true, true, true);
  const bool retreated = goal_retreat == navigation::Result::kSuccess ||
                         goal_retreat == navigation::Result::kDriveFailed;
  if (!retreated) {
    navigation::stop();
    return false;
  }
  // One lower-power turn toward zero. Do not retry or wait on a precision
  // settle: those gates were stopping the remainder of the autonomous.
  const navigation::Result zero_turn = navigation::turn_to(
      0.0, 70, 1400, true, true);
  const bool zero_turn_attempted =
      zero_turn == navigation::Result::kSuccess ||
      zero_turn == navigation::Result::kTurnFailed;
  if (!zero_turn_attempted) {
    navigation::stop();
    return false;
  }

  // Reverse 23 inches with the claw open, clamp one inch before the requested
  // 24-inch endpoint, then finish the last inch while closed around the stack.
  const navigation::Result stack_approach = navigation::drive_relative(
      -(kStackReverseIn - kStackClampLeadIn),
      55, 6000, true, true, true);
  const bool stack_approach_usable =
      stack_approach == navigation::Result::kSuccess ||
      stack_approach == navigation::Result::kDriveFailed;
  if (!stack_approach_usable) {
    navigation::stop();
    return false;
  }
  set_claw_piston(false);
  pros::delay(100);
  const navigation::Result final_capture = navigation::drive_relative(
      -kStackClampLeadIn, 38, 1600, true, true, true);
  const bool stack_reached =
      final_capture == navigation::Result::kSuccess ||
      final_capture == navigation::Result::kDriveFailed;
  pros::delay(50);

  std::printf(
      "ONE_PIN_NEW toggle=%s/%s goal=%.2f,%.2f turn=%s contact=%s "
      "retreat=%s zero=%s stack=%.2f,%.2f approach=%s capture=%s ok=%d\n",
      navigation::result_name(toggle_ram),
      navigation::result_name(toggle_return), kGoal.x, kGoal.y,
      navigation::result_name(goal_turn),
      navigation::result_name(goal_contact),
      navigation::result_name(goal_retreat), navigation::result_name(zero_turn),
      kFinalStack.x, kFinalStack.y,
      navigation::result_name(stack_approach),
      navigation::result_name(final_capture),
      static_cast<int>(stack_reached));
  std::fflush(stdout);
  navigation::stop();
  return stack_reached;
}

bool localization_simple_red_goal_hotkey_auton_legacy() {
  // Surveyed global field frame: +X points toward the starting Toggle.
  constexpr localization::FieldPose kStart{60.5, 0.25, 0.0};
  // Requested global coordinate: shift X 1.5 inches more negative while
  // retaining the prior Y correction.
  constexpr Waypoint kGoal{kStart.x_in - 6.5, kStart.y_in - 8.0};
  constexpr Waypoint kRetreat{49.0, 3.0};
  constexpr Waypoint kFinalStack{24.0, 0.0};
  constexpr double kRearPickupReachIn = 10.0;
  constexpr Waypoint kFinalStackRobotCenter{
      kFinalStack.x + kRearPickupReachIn, kFinalStack.y};
  // Live trace proved both IMU and GPS reached the old 90-degree request,
  // while the mechanism was still physically a few degrees short of the
  // scoring alignment. This is a calibrated scoring heading, not PID error.
  constexpr double kDepositHeadingDeg = 96.0;
  // Raw PROS IMU rotation is clockwise-positive, so physical -90 degrees is
  // navigation +90 degrees in this file's CCW-positive field convention.
  constexpr double kPhysicalRetreatHeadingDeg = -90.0;
  constexpr double kNavigationRetreatHeadingDeg =
      -kPhysicalRetreatHeadingDeg;

  default_constants();
  stop_drive_motors();
  set_claw_piston(false);
  if (!navigation::init(kStart.x_in, kStart.y_in,
                        kStart.heading_deg, 0.75)) {
    return false;
  }

  // P1 intentionally sees the Toggle, so this one command explicitly allows
  // contact. Encoder, IMU, GPS, vision, stall, and timeout checks remain live.
  const navigation::Result ram = navigation::drive_relative(
      6.0, 78, 1800, false, false, true);
  // Contact with the Toggle normally ends through stall detection, which is
  // the expected outcome for this one short ram command.
  const bool ram_ok = ram == navigation::Result::kSuccess ||
                      ram == navigation::Result::kDriveFailed;
  const navigation::Result backout = ram_ok
      ? navigation::drive_relative(-5.0, 68, 3500, true, false, true)
      : navigation::Result::kDriveFailed;
  navigation::update();
  const navigation::Pose approach_start = navigation::current_pose();
  const double kGoalApproachHeadingDeg = normalize_deg(
      rad_to_deg(std::atan2(kGoal.y - approach_start.y_in,
                           kGoal.x - approach_start.x_in)) + 180.0);
  // Reverse to the Goal coordinate. The target heading is the rear-facing
  // bearing calculated from the current fused pose, so this leg does not add
  // a second hard-coded turn.
  const navigation::Result approach = navigation::go_to_pose(
      kGoal.x, kGoal.y, kGoalApproachHeadingDeg,
      88, 2800, true, true, true);

  navigation::update();
  navigation::Pose chained_pose = navigation::current_pose();
  const double goal_error_in = std::hypot(
      chained_pose.x_in - kGoal.x, chained_pose.y_in - kGoal.y);
  // A controller can report a settle timeout after reaching the coordinate.
  // Continue the chain when the fused pose confirms the robot is at the Goal.
  const bool near_goal = approach == navigation::Result::kSuccess ||
                         goal_error_in <= 4.0;
  const navigation::Result turn = navigation::turn_to(
      kDepositHeadingDeg, 100, 1400, true, true);

  navigation::update();
  chained_pose = navigation::current_pose();
  const double deposit_heading_error_deg = std::fabs(
      signed_angle_diff_deg(kDepositHeadingDeg, chained_pose.heading_deg));

  navigation::Result goal_nudge = navigation::Result::kDriveFailed;
  navigation::Result straighten = navigation::Result::kTurnFailed;
  navigation::Result retreat = navigation::Result::kDriveFailed;
  navigation::Result retreat_extra = navigation::Result::kDriveFailed;
  navigation::Result face_start = navigation::Result::kTurnFailed;
  navigation::Result stack_approach = navigation::Result::kDriveFailed;
  bool dropped = false;
  bool pin_intaked = false;
  bool straighten_usable = false;
  bool retreat_reached = false;
  bool face_start_usable = false;
  const bool route_enabled =
      !pros::competition::is_connected() ||
      !pros::competition::is_disabled();
  // A normal kDriveFailed/kTurnFailed here means the blocking controller
  // exhausted its settle window after executing the motion. Treat those as a
  // completed chain step; only invalid/unsafe/unavailable preflight results or
  // competition disable may suppress the following mechanism sequence.
  const bool approach_completed =
      approach == navigation::Result::kSuccess ||
      approach == navigation::Result::kDriveFailed ||
      approach == navigation::Result::kTurnFailed;
  const bool turn_usable =
      turn == navigation::Result::kSuccess ||
      turn == navigation::Result::kTurnFailed;
  std::printf(
      "SIMPLE_RED chain_after_boomerang enabled=%d pose=%d approach=%s "
      "turn=%s near=%d heading_error=%.2f continue=%d\n",
      static_cast<int>(route_enabled), static_cast<int>(chained_pose.valid),
      navigation::result_name(approach), navigation::result_name(turn),
      static_cast<int>(near_goal), deposit_heading_error_deg,
      static_cast<int>(route_enabled && approach_completed && turn_usable));
  std::fflush(stdout);
  if (route_enabled && approach_completed && turn_usable) {
    // Move the rear of the robot one inch closer before starting the rollers.
    goal_nudge = navigation::drive_relative(
        -1.0, 52, 1000, true, true, true);
    // The nudge may report pose-unavailable after the completed boomerang.
    // It is an optional one-inch seating motion and must never suppress the
    // pneumatic preload release at the already-reached Goal.
    const bool nudge_ok = route_enabled;
    if (nudge_ok) {
      // ADI-E extended is release/open. The preload is dropped only after the
      // chassis completes its final one-inch Goal nudge.
      set_claw_piston(true);
      pros::delay(250);
      dropped = true;

      // The scoring alignment is calibrated to 96 degrees. Return to the
      // field-axis 90-degree heading before beginning the retreat.
      straighten = navigation::turn_to(
          kNavigationRetreatHeadingDeg, 90, 900, true, true);
      navigation::update();
      const navigation::Pose straightened_pose = navigation::current_pose();
      straighten_usable = straightened_pose.valid &&
          (straighten == navigation::Result::kSuccess ||
           straighten == navigation::Result::kTurnFailed);
      if (straighten_usable) {
        retreat = navigation::go_to_pose(
            kRetreat.x, kRetreat.y, kNavigationRetreatHeadingDeg,
            75, 4500, false, true, true);
      }

      navigation::update();
      const navigation::Pose retreat_pose = navigation::current_pose();
      retreat_reached = retreat_pose.valid &&
          (retreat == navigation::Result::kSuccess ||
           retreat == navigation::Result::kDriveFailed ||
           std::hypot(retreat_pose.x_in - kRetreat.x,
                      retreat_pose.y_in - kRetreat.y) <= 2.0);
      if (retreat_reached) {
        // Create ten additional inches of clearance from the scored Goal.
        // The extra four inches correct the observed turn approach ending near
        // (24,-2) instead of squarely reaching the stack at (24,0).
        // This is robot-relative and does not depend on the fused endpoint.
        retreat_extra = navigation::drive_relative(
            10.0, 75, 3200, true, true, true);
      }
      const bool extra_retreat_usable =
          retreat_extra == navigation::Result::kSuccess ||
          retreat_extra == navigation::Result::kDriveFailed;
      if (extra_retreat_usable) {
        face_start = navigation::turn_to(0.0, 90, 1800, true, true);
        navigation::update();
        const navigation::Pose faced_pose = navigation::current_pose();
        face_start_usable = faced_pose.valid &&
            (face_start == navigation::Result::kSuccess ||
             face_start == navigation::Result::kTurnFailed);
      }
      if (face_start_usable) {
        // Target the actual stack at (24,0), not a fixed distance from the
        // retreat. With ten inches of rear reach, the robot center endpoint is
        // (34,0), heading 0, approached in reverse.
        stack_approach = navigation::go_to_pose(
            kFinalStackRobotCenter.x, kFinalStackRobotCenter.y, 0.0,
            48, 6000, true, true, true);
        navigation::update();
        const navigation::Pose stack_pose = navigation::current_pose();
        const double stack_center_error_in = stack_pose.valid
            ? std::hypot(stack_pose.x_in - kFinalStackRobotCenter.x,
                         stack_pose.y_in - kFinalStackRobotCenter.y)
            : std::numeric_limits<double>::infinity();
        const bool stack_reached =
            stack_approach == navigation::Result::kSuccess ||
            stack_center_error_in <= 3.0;
        std::printf(
            "SIMPLE_RED final_stack object=%.2f,%.2f center=%.2f,%.2f "
            "result=%s error=%.2f reached=%d\n",
            kFinalStack.x, kFinalStack.y, kFinalStackRobotCenter.x,
            kFinalStackRobotCenter.y,
            navigation::result_name(stack_approach), stack_center_error_in,
            static_cast<int>(stack_reached));
        std::fflush(stdout);
        if (stack_reached) {
          // Stay open throughout the coordinate approach and clamp only once
          // the rear tool has reached the stack.
          set_claw_piston(false);
          pros::delay(150);
          pin_intaked = true;
        }
      }
    }
  }

  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const bool success = ram_ok &&
      backout == navigation::Result::kSuccess &&
      approach_completed && turn_usable &&
      (goal_nudge == navigation::Result::kSuccess ||
       goal_nudge == navigation::Result::kDriveFailed) &&
      dropped && straighten_usable && retreat_reached &&
      face_start_usable && pin_intaked;
  std::printf(
      "SIMPLE_RED complete ok=%d ram=%s backout=%s approach=%s "
      "near=%d turn=%s heading_error=%.2f nudge=%s dropped=%d "
      "straighten=%s retreat=%s extra=%s face_start=%s stack=%s intake=%d "
      "x=%.2f y=%.2f heading=%.2f\n",
      static_cast<int>(success), navigation::result_name(ram),
      navigation::result_name(backout), navigation::result_name(approach),
      static_cast<int>(near_goal), navigation::result_name(turn),
      deposit_heading_error_deg,
      navigation::result_name(goal_nudge),
      static_cast<int>(dropped), navigation::result_name(straighten),
      navigation::result_name(retreat), navigation::result_name(retreat_extra),
      navigation::result_name(face_start),
      navigation::result_name(stack_approach),
      static_cast<int>(pin_intaked), final_pose.x_in, final_pose.y_in,
      final_pose.heading_deg);
  std::fflush(stdout);
  navigation::stop();
  return success;
}

bool localization_simple_red_goal_finish_correction() {
  // Measured endpoint of the current missed-Goal trial. Uploading resets the
  // local IMU/encoders, so re-anchor that physical pose before correcting it.
  constexpr localization::FieldPose kCurrent{-42.38, -5.16, 81.66};
  constexpr double kTargetHeadingDeg = 90.0;
  constexpr double kBackoffIn = 3.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  counter_rollers.move(0);
  if (!navigation::init(kCurrent.x_in, kCurrent.y_in,
                        kCurrent.heading_deg, 0.75)) {
    std::printf("SIMPLE_FINISH abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }

  navigation_stop_requested.store(false, std::memory_order_release);
  bool turn_ok = false;
  std::uint32_t turn_settled_since_ms = 0;
  const std::uint32_t turn_started_ms = pros::millis();
  while (pros::millis() - turn_started_ms < 2500) {
    update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
    const double error_deg = signed_angle_diff_deg(
        kTargetHeadingDeg, pose_heading_deg(telemetry_pose));
    if (std::fabs(error_deg) <= 1.0) {
      if (turn_settled_since_ms == 0) turn_settled_since_ms = pros::millis();
      if (pros::millis() - turn_settled_since_ms >= 120) {
        turn_ok = true;
        break;
      }
    } else {
      turn_settled_since_ms = 0;
    }
    const int turn_power = std::fabs(error_deg) > 3.0 ? 70 : 50;
    set_physical_drive_power(
        0, error_deg >= 0.0 ? turn_power : -turn_power);
    pros::delay(10);
  }
  stop_drive_motors();
  update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
  const MotorSideReading left_start = read_motor_side(chassis.left_motors);
  const MotorSideReading right_start = read_motor_side(chassis.right_motors);
  const double outtake_start_deg = counter_rollers.get_position();
  counter_rollers.move(45);  // Slow L1/outtake starts before backing off.
  pros::delay(350);
  bool backoff_ok = left_start.trustworthy && right_start.trustworthy;
  double reverse_travel_in = 0.0;
  const std::uint32_t reverse_started_ms = pros::millis();
  while (backoff_ok && pros::millis() - reverse_started_ms < 2600) {
    update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    if (!left.trustworthy || !right.trustworthy) {
      backoff_ok = false;
      break;
    }
    reverse_travel_in = -0.5 * (
        ((left.position_deg - left_start.position_deg) / 360.0) *
            kWheelCircumferenceIn * kLeftEncoderSign +
        ((right.position_deg - right_start.position_deg) / 360.0) *
            kWheelCircumferenceIn * kRightEncoderSign);
    if (reverse_travel_in >= kBackoffIn) break;
    const double heading_error_deg = signed_angle_diff_deg(
        kTargetHeadingDeg, pose_heading_deg(telemetry_pose));
    set_physical_drive_power(-38, clamp_power(heading_error_deg * 1.5));
    counter_rollers.move(45);
    pros::delay(10);
  }
  stop_drive_motors();
  backoff_ok = backoff_ok && reverse_travel_in >= 2.5;
  counter_rollers.move(45);
  pros::delay(700);
  counter_rollers.move(0);
  update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
  const double outtake_delta_deg =
      counter_rollers.get_position() - outtake_start_deg;
  const bool outtake_ok = std::isfinite(outtake_delta_deg) &&
                          std::fabs(outtake_delta_deg) >= 20.0;
  std::printf(
      "SIMPLE_FINISH event=complete turn_ok=%d backoff_ok=%d outtake_ok=%d "
      "reverse=%.3f outtake_delta=%.2f x=%.3f y=%.3f heading=%.3f\n",
      static_cast<int>(turn_ok), static_cast<int>(backoff_ok),
      static_cast<int>(outtake_ok), reverse_travel_in, outtake_delta_deg,
      telemetry_pose.x, telemetry_pose.y,
      pose_heading_deg(telemetry_pose));
  std::fflush(stdout);
  navigation::stop();
  stop_drive_motors();
  counter_rollers.move(0);
  return turn_ok && backoff_ok && outtake_ok;
}

bool localization_toggle_far_goal_hotkey_auton() {
  // User-surveyed field frame: +X points from center toward the starting
  // Toggle, +Y points toward the nearby black Goal. The two measured side
  // gaps (17.0 and 16.5 in) across Goal centers at Y=-24/+24 imply a 14.5-in
  // robot width and place its start center at Y=+0.25 in. The prior live
  // anchor supplies X=60.5 in; heading zero faces the Toggle.
  constexpr double kStartXIn = 60.5;
  constexpr double kStartYIn = 0.25;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kStartErrorEnvelopeIn = 1.0;
  constexpr double kRobotHalfWidthIn = 7.25;
  constexpr double kMeasuredSafetyAllowanceIn = 1.0;
  // The first live wall-side route proved the inferred footprint margin was
  // too optimistic. The replacement curve uses the field-center side and
  // keeps every non-target Goal at least 17 inches from robot center.
  constexpr double kMinimumCenterGoalClearanceIn = 17.0;
  constexpr double kMinimumCenterWallClearanceIn =
      kRobotHalfWidthIn + kMeasuredSafetyAllowanceIn;

  // Exact requested Goal center. Do not substitute the nearby Goal at
  // (47.10,-23.54), which is the obstacle this curve goes around.
  constexpr Waypoint kFarGoalCenter{23.55, -47.09};
  constexpr Waypoint kNearRedGoalCenter{47.10, -23.54};
  constexpr Waypoint kNearBlackGoalCenter{47.10, 23.55};
  // Drive inward between the two close Goals before bending around the
  // field-center side of the red Goal. End 20 inches west of the far Goal,
  // then point directly east at its exact center. This is longer than the
  // failed wall-side shortcut but has materially larger physical clearance.
  constexpr double kTurnClearanceRetreatIn = 12.0;
  constexpr Waypoint kPursuitStart{
      kStartXIn - kTurnClearanceRetreatIn, kStartYIn};
  constexpr Waypoint kPathEnd{3.55, -47.09};
  constexpr Waypoint kBezierControl1{34.0, 0.25};
  constexpr Waypoint kBezierControl2{-8.0, -35.0};
  constexpr std::size_t kPathPointCount = 61;
  constexpr double kLookaheadIn = 10.0;
  constexpr double kMaximumCrossTrackIn = 4.0;
  constexpr int kPathMaximumPower = 85;
  constexpr std::uint32_t kPathTimeoutMs = 15000;
  constexpr double kRamMaximumTravelIn = 7.0;
  constexpr std::uint32_t kRamMaximumMs = 1200;
  constexpr std::int32_t kRamCurrentLimitMa = 2400;
  constexpr double kMillimetersPerInch = 25.4;
  constexpr double kPreflightMaximumRangeIn = 15.0;
  constexpr double kPrealignMaximumTravelIn = 8.0;

  struct PursuitPoint {
    double x = 0.0;
    double y = 0.0;
    double s = 0.0;
  };
  std::array<PursuitPoint, kPathPointCount> path{};
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double t = static_cast<double>(index) /
        static_cast<double>(path.size() - 1);
    const double u = 1.0 - t;
    path[index].x =
        u * u * u * kPursuitStart.x +
        3.0 * u * u * t * kBezierControl1.x +
        3.0 * u * t * t * kBezierControl2.x +
        t * t * t * kPathEnd.x;
    path[index].y =
        u * u * u * kPursuitStart.y +
        3.0 * u * u * t * kBezierControl1.y +
        3.0 * u * t * t * kBezierControl2.y +
        t * t * t * kPathEnd.y;
    if (index > 0) {
      path[index].s = path[index - 1].s + std::hypot(
          path[index].x - path[index - 1].x,
          path[index].y - path[index - 1].y);
    }
  }

  double minimum_near_red_clearance_in =
      std::numeric_limits<double>::infinity();
  double minimum_near_black_clearance_in =
      std::numeric_limits<double>::infinity();
  double minimum_wall_clearance_in =
      std::numeric_limits<double>::infinity();
  bool path_geometry_safe = true;
  for (const auto& point : path) {
    minimum_near_red_clearance_in = std::min(
        minimum_near_red_clearance_in,
        std::hypot(point.x - kNearRedGoalCenter.x,
                   point.y - kNearRedGoalCenter.y));
    minimum_near_black_clearance_in = std::min(
        minimum_near_black_clearance_in,
        std::hypot(point.x - kNearBlackGoalCenter.x,
                   point.y - kNearBlackGoalCenter.y));
    minimum_wall_clearance_in = std::min(
        minimum_wall_clearance_in,
        localization::kPhysicalWallHalfSpanIn -
            std::max(std::fabs(point.x), std::fabs(point.y)));
    for (const auto& goal : localization::kGoalTagLandmarks) {
      const double clearance = std::hypot(
          point.x - goal.x_in, point.y - goal.y_in);
      if (clearance < kMinimumCenterGoalClearanceIn) {
        path_geometry_safe = false;
      }
    }
  }
  path_geometry_safe = path_geometry_safe &&
      minimum_wall_clearance_in >= kMinimumCenterWallClearanceIn;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  double p1_start_in =
      static_cast<double>(distance_1.get_distance()) /
      kMillimetersPerInch;
  const MotorSideReading left_start = read_motor_side(chassis.left_motors);
  const MotorSideReading right_start = read_motor_side(chassis.right_motors);
  const bool preflight_ok = path_geometry_safe &&
      left_start.trustworthy && right_start.trustworthy &&
      chassis.imu.is_installed() && !chassis.imu.is_calibrating() &&
      distance_1.is_installed() && std::isfinite(p1_start_in) &&
      p1_start_in >= 2.0 && p1_start_in <= kPreflightMaximumRangeIn;
  std::printf(
      "FAR_GOAL event=preflight ok=%d start=%.2f,%.2f,%.2f "
      "target=%.2f,%.2f p1=%.2f path_length=%.2f red_clear=%.2f "
      "black_clear=%.2f wall_clear=%.2f geometry=%d\n",
      static_cast<int>(preflight_ok), kStartXIn, kStartYIn,
      kStartHeadingDeg, kFarGoalCenter.x, kFarGoalCenter.y, p1_start_in,
      path.back().s, minimum_near_red_clearance_in,
      minimum_near_black_clearance_in, minimum_wall_clearance_in,
      static_cast<int>(path_geometry_safe));
  std::fflush(stdout);
  if (!preflight_ok) return false;

  if (!navigation::init(kStartXIn, kStartYIn, kStartHeadingDeg,
                        kStartErrorEnvelopeIn)) {
    std::printf("FAR_GOAL abort=navigation_init\n");
    std::fflush(stdout);
    return false;
  }

  // Nominal field placement is six inches behind the Toggle. Permit a small
  // setup error without changing the route frame: P1 closes only the local
  // contact gap, while encoders bound travel and the IMU holds heading.
  if (p1_start_in > 8.5 || p1_start_in < 4.0) {
    const bool move_toward_toggle = p1_start_in > 8.5;
    navigation_stop_requested.store(false, std::memory_order_release);
    const MotorSideReading align_left =
        read_motor_side(chassis.left_motors);
    const MotorSideReading align_right =
        read_motor_side(chassis.right_motors);
    if (!align_left.trustworthy || !align_right.trustworthy) return false;
    const std::uint32_t align_started_ms = pros::millis();
    while (pros::millis() - align_started_ms < 1800) {
      update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
      p1_start_in = static_cast<double>(distance_1.get_distance()) /
          kMillimetersPerInch;
      const MotorSideReading left = read_motor_side(chassis.left_motors);
      const MotorSideReading right = read_motor_side(chassis.right_motors);
      if (!std::isfinite(p1_start_in) || !left.trustworthy ||
          !right.trustworthy) {
        stop_drive_motors();
        return false;
      }
      const double travel_in = 0.5 * (
          ((left.position_deg - align_left.position_deg) / 360.0) *
              kWheelCircumferenceIn * kLeftEncoderSign +
          ((right.position_deg - align_right.position_deg) / 360.0) *
              kWheelCircumferenceIn * kRightEncoderSign);
      if ((move_toward_toggle && p1_start_in <= 7.0) ||
          (!move_toward_toggle && p1_start_in >= 6.0) ||
          std::fabs(travel_in) >= kPrealignMaximumTravelIn) {
        break;
      }
      const double heading_error_deg = signed_angle_diff_deg(
          kStartHeadingDeg, pose_heading_deg(telemetry_pose));
      const double range_error_in = move_toward_toggle
          ? std::max(0.0, p1_start_in - 6.5)
          : std::max(0.0, 6.5 - p1_start_in);
      const int align_magnitude = clamp_power(clamp(
          24.0 + range_error_in * 6.0, 24.0, 45.0));
      const int align_power = move_toward_toggle
          ? align_magnitude : -align_magnitude;
      set_physical_drive_power(
          align_power, clamp_power(heading_error_deg * 1.2));
      pros::delay(10);
    }
    stop_drive_motors();
    p1_start_in = static_cast<double>(distance_1.get_distance()) /
        kMillimetersPerInch;
    std::printf("FAR_GOAL event=prealign p1=%.3f\n", p1_start_in);
    std::fflush(stdout);
    if (p1_start_in < 4.0 || p1_start_in > 8.5) return false;
    // The P1 adjustment changes the physical start but not the surveyed field
    // definition. Re-anchor encoders, IMU, GPS alignment, and the route pose
    // only after the robot is actually in its 4-8.5-inch Toggle window.
    pros::delay(600);
    if (!navigation::init(kStartXIn, kStartYIn, kStartHeadingDeg,
                          kStartErrorEnvelopeIn)) {
      std::printf("FAR_GOAL abort=post_prealign_navigation_init\n");
      std::fflush(stdout);
      return false;
    }
  }
  auto begin_internal_motion = [&](const char* phase) {
    navigation_stop_requested.store(false, std::memory_order_release);
    return !blocking_motion_abort_requested(phase);
  };

  auto ram_and_return = [&](int cycle_index) {
    if (!begin_internal_motion("far_goal_toggle_ram")) return false;
    navigation::update();
    const navigation::Pose cycle_start = navigation::current_pose();
    const MotorSideReading left_baseline =
        read_motor_side(chassis.left_motors);
    const MotorSideReading right_baseline =
        read_motor_side(chassis.right_motors);
    if (!left_baseline.trustworthy || !right_baseline.trustworthy) {
      return false;
    }
    std::int32_t maximum_current_ma = 0;
    bool current_limit = false;
    bool travel_limit = false;
    const std::uint32_t started_ms = pros::millis();
    while (pros::millis() - started_ms < kRamMaximumMs) {
      update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
      if (motion_pose_invalid(telemetry_pose, "far_goal_toggle_ram") ||
          blocking_motion_abort_requested("far_goal_toggle_ram")) {
        stop_drive_motors();
        return false;
      }
      const MotorSideReading left = read_motor_side(chassis.left_motors);
      const MotorSideReading right = read_motor_side(chassis.right_motors);
      if (!left.trustworthy || !right.trustworthy) break;
      const double left_travel_in =
          ((left.position_deg - left_baseline.position_deg) / 360.0) *
          kWheelCircumferenceIn * kLeftEncoderSign;
      const double right_travel_in =
          ((right.position_deg - right_baseline.position_deg) / 360.0) *
          kWheelCircumferenceIn * kRightEncoderSign;
      const double forward_travel_in =
          0.5 * (left_travel_in + right_travel_in);
      if (forward_travel_in >= kRamMaximumTravelIn) {
        travel_limit = true;
        break;
      }
      for (auto& motor : chassis.left_motors) {
        maximum_current_ma = std::max(
            maximum_current_ma, motor.get_current_draw());
      }
      for (auto& motor : chassis.right_motors) {
        maximum_current_ma = std::max(
            maximum_current_ma, motor.get_current_draw());
      }
      // Full-power acceleration can briefly exceed the contact threshold.
      // Only treat current as Toggle contact after the robot has actually had
      // time (or encoder travel) to leave its starting point.
      const std::uint32_t elapsed_ms = pros::millis() - started_ms;
      if (maximum_current_ma >= kRamCurrentLimitMa &&
          (elapsed_ms >= 150 || forward_travel_in >= 1.0)) {
        current_limit = true;
        break;
      }
      const double heading_error_deg = signed_angle_diff_deg(
          kStartHeadingDeg, pose_heading_deg(telemetry_pose));
      // P1 intentionally sees the Toggle during this bounded contact phase.
      set_physical_drive_power(
          127, clamp_power(heading_error_deg * 1.2));
      pros::delay(10);
    }
    stop_drive_motors();
    update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
    const MotorSideReading left_stop = read_motor_side(chassis.left_motors);
    const MotorSideReading right_stop = read_motor_side(chassis.right_motors);
    if (!left_stop.trustworthy || !right_stop.trustworthy) return false;
    const double travel_in = 0.5 * (
        ((left_stop.position_deg - left_baseline.position_deg) / 360.0) *
            kWheelCircumferenceIn * kLeftEncoderSign +
        ((right_stop.position_deg - right_baseline.position_deg) / 360.0) *
            kWheelCircumferenceIn * kRightEncoderSign);
    const bool contact_motion_proven =
        travel_in >= 1.0 || current_limit;
    std::printf(
        "FAR_GOAL event=ram cycle=%d travel=%.3f current=%ld "
        "current_limit=%d travel_limit=%d proven=%d\n",
        cycle_index, travel_in, static_cast<long>(maximum_current_ma),
        static_cast<int>(current_limit), static_cast<int>(travel_limit),
        static_cast<int>(contact_motion_proven));
    std::fflush(stdout);
    if (!contact_motion_proven) return false;

    if (!begin_internal_motion("far_goal_toggle_return")) return false;
    const bool returned = fused_drive_to_point(
        telemetry_pose, "far_goal_toggle_return",
        Waypoint{cycle_start.x_in, cycle_start.y_in},
        kStartHeadingDeg, 70, 5000, 2.5, -1,
        LidarFusionMode::kBiasOnly);
    stop_drive_motors();
    navigation::update();
    const navigation::Pose after = navigation::current_pose();
    const double return_error_in = std::hypot(
        after.x_in - cycle_start.x_in,
        after.y_in - cycle_start.y_in);
    std::printf(
        "FAR_GOAL event=cycle_complete cycle=%d returned=%d "
        "error=%.3f x=%.3f y=%.3f h=%.3f\n",
        cycle_index, static_cast<int>(returned), return_error_in,
        after.x_in, after.y_in, after.heading_deg);
    std::fflush(stdout);
    return returned && return_error_in <= 1.25;
  };

  bool ok = ram_and_return(1);
  if (ok) ok = ram_and_return(2);
  if (ok) {
    ok = begin_internal_motion("far_goal_turn_clearance") &&
        fused_drive_to_point(
            telemetry_pose, "far_goal_turn_clearance", kPursuitStart,
            kStartHeadingDeg, 85, 5000, 3.0, -1,
            LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }
  if (ok) {
    ok = begin_internal_motion("far_goal_launch_turn") &&
        fused_turn_to_heading(telemetry_pose, "far_goal_launch_turn",
                              180.0, 60, 7000,
                              LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }

  if (ok && begin_internal_motion("far_goal_pure_pursuit")) {
    std::uint32_t started_ms = pros::millis();
    std::uint32_t last_loop_ms = started_ms;
    std::uint32_t last_progress_ms = started_ms;
    std::uint32_t last_log_ms = 0;
    std::size_t nearest_index = 0;
    double last_progress_in = 0.0;
    double forward_command = 0.0;
    double turn_command = 0.0;
    bool arrived = false;
    int stall_recoveries = 0;
    while (pros::millis() - started_ms < kPathTimeoutMs) {
      if (blocking_motion_abort_requested("far_goal_pure_pursuit")) {
        ok = false;
        break;
      }
      update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
      if (motion_pose_invalid(telemetry_pose, "far_goal_pure_pursuit")) {
        ok = false;
        break;
      }
      const std::uint32_t now = pros::millis();
      const double dt_s = std::max(
          0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
      last_loop_ms = now;

      double nearest_distance_in = std::numeric_limits<double>::infinity();
      for (std::size_t index = nearest_index; index < path.size(); ++index) {
        const double distance = std::hypot(
            telemetry_pose.x - path[index].x,
            telemetry_pose.y - path[index].y);
        if (distance < nearest_distance_in) {
          nearest_distance_in = distance;
          nearest_index = index;
        }
      }
      const double progress_in = path[nearest_index].s;
      if (progress_in >= last_progress_in + kFusedDriveStallProgressIn) {
        last_progress_in = progress_in;
        last_progress_ms = now;
      }
      if (nearest_distance_in > kMaximumCrossTrackIn) {
        std::printf(
            "FAR_GOAL abort=cross_track distance=%.3f limit=%.3f\n",
            nearest_distance_in, kMaximumCrossTrackIn);
        std::fflush(stdout);
        ok = false;
        break;
      }

      const double endpoint_distance_in = std::hypot(
          telemetry_pose.x - kPathEnd.x,
          telemetry_pose.y - kPathEnd.y);
      if (endpoint_distance_in <= 1.5 &&
          nearest_index + 2 >= path.size()) {
        arrived = true;
        break;
      }
      std::size_t carrot_index = nearest_index;
      const double carrot_s = progress_in + kLookaheadIn;
      while (carrot_index + 1 < path.size() &&
             path[carrot_index].s < carrot_s) {
        ++carrot_index;
      }
      const double dx = path[carrot_index].x - telemetry_pose.x;
      const double dy = path[carrot_index].y - telemetry_pose.y;
      const double desired_heading_deg = normalize_deg(rad_to_deg(
          std::atan2(dy, dx)));
      const double heading_error_deg = signed_angle_diff_deg(
          desired_heading_deg, pose_heading_deg(telemetry_pose));
      const double remaining_in = std::max(
          0.0, path.back().s - progress_in);
      double target_forward = std::min(
          static_cast<double>(kPathMaximumPower),
          std::max(kFusedDriveMinPower, remaining_in * 3.0));
      target_forward *= clamp(
          std::cos(deg_to_rad(std::min(75.0,
                                      std::fabs(heading_error_deg)))),
          0.30, 1.0);
      const double lookahead_distance_in = std::max(
          1.0, std::hypot(dx, dy));
      const double curvature_per_in =
          2.0 * std::sin(deg_to_rad(heading_error_deg)) /
          lookahead_distance_in;
      double target_turn = target_forward * curvature_per_in *
          kTrackWidthIn / 2.0;
      target_turn = clamp(target_turn, -35.0, 35.0);
      forward_command = apply_slew(
          target_forward, forward_command,
          kFusedDriveForwardSlewPowerPerSec * dt_s);
      turn_command = apply_slew(
          target_turn, turn_command,
          kFusedDriveTurnSlewPowerPerSec * dt_s);
      turn_command = clamp(turn_command, -std::fabs(forward_command),
                           std::fabs(forward_command));

      if (forward_obstacle_requires_stop("far_goal_pure_pursuit")) {
        ok = false;
        break;
      }
      if (std::fabs(forward_command) >= kFusedDriveMinPower * 0.75 &&
          now - last_progress_ms >= kFusedDriveStallTimeoutMs) {
        log_drive_health("far_goal_pure_pursuit_stall");
        if (stall_recoveries == 0) {
          ++stall_recoveries;
          stop_drive_motors();
          const MotorSideReading recovery_left =
              read_motor_side(chassis.left_motors);
          const MotorSideReading recovery_right =
              read_motor_side(chassis.right_motors);
          const std::uint32_t recovery_started_ms = pros::millis();
          while (recovery_left.trustworthy && recovery_right.trustworthy &&
                 pros::millis() - recovery_started_ms < 650) {
            update_pose(telemetry_pose, LidarFusionMode::kBiasOnly);
            const MotorSideReading left =
                read_motor_side(chassis.left_motors);
            const MotorSideReading right =
                read_motor_side(chassis.right_motors);
            if (!left.trustworthy || !right.trustworthy) break;
            const double backoff_in = -0.5 * (
                ((left.position_deg - recovery_left.position_deg) / 360.0) *
                    kWheelCircumferenceIn * kLeftEncoderSign +
                ((right.position_deg - recovery_right.position_deg) / 360.0) *
                    kWheelCircumferenceIn * kRightEncoderSign);
            if (backoff_in >= 4.0) break;
            set_physical_drive_power(-55, 0);
            pros::delay(10);
          }
          stop_drive_motors();
          nearest_index = 0;
          last_progress_in = 0.0;
          last_progress_ms = pros::millis();
          forward_command = 0.0;
          turn_command = 0.0;
          std::printf("FAR_GOAL event=stall_recovery count=%d\n",
                      stall_recoveries);
          std::fflush(stdout);
          continue;
        }
        std::printf(
            "FAR_GOAL abort=path_stall progress=%.3f command=%.2f\n",
            progress_in, forward_command);
        std::fflush(stdout);
        ok = false;
        break;
      }
      set_physical_drive_power(
          clamp_power(forward_command), clamp_power(turn_command));
      if (now - last_log_ms >= kFusionTestLogPeriodMs) {
        std::printf(
            "FAR_GOAL controller=pure_pursuit progress=%.2f/%.2f "
            "nearest=%u carrot=%u cross=%.2f endpoint=%.2f "
            "heading_error=%.2f forward=%.1f turn=%.1f x=%.2f y=%.2f "
            "h=%.2f\n",
            progress_in, path.back().s,
            static_cast<unsigned>(nearest_index),
            static_cast<unsigned>(carrot_index), nearest_distance_in,
            endpoint_distance_in, heading_error_deg, forward_command,
            turn_command, telemetry_pose.x, telemetry_pose.y,
            pose_heading_deg(telemetry_pose));
        std::fflush(stdout);
        last_log_ms = now;
      }
      pros::delay(20);
    }
    stop_drive_motors();
    ok = ok && arrived;
    std::printf("FAR_GOAL event=path_complete arrived=%d\n",
                static_cast<int>(arrived));
    std::fflush(stdout);
  } else if (ok) {
    ok = false;
  }

  if (ok) {
    const double target_heading_deg = normalize_deg(rad_to_deg(std::atan2(
        kFarGoalCenter.y - telemetry_pose.y,
        kFarGoalCenter.x - telemetry_pose.x)));
    ok = begin_internal_motion("far_goal_final_face") &&
        fused_turn_to_heading(telemetry_pose, "far_goal_final_face",
                              target_heading_deg, 90, 5000,
                              LidarFusionMode::kBiasOnly);
    stop_drive_motors();
  }
  navigation::update();
  const navigation::Pose final_pose = navigation::current_pose();
  const double final_goal_distance_in = std::hypot(
      final_pose.x_in - kFarGoalCenter.x,
      final_pose.y_in - kFarGoalCenter.y);
  const double final_goal_bearing_deg = normalize_deg(rad_to_deg(std::atan2(
      kFarGoalCenter.y - final_pose.y_in,
      kFarGoalCenter.x - final_pose.x_in)));
  const double final_heading_error_deg = std::fabs(signed_angle_diff_deg(
      final_goal_bearing_deg, final_pose.heading_deg));
  std::printf(
      "FAR_GOAL event=complete ok=%d target=%.2f,%.2f x=%.3f y=%.3f "
      "h=%.3f goal_distance=%.3f goal_bearing=%.3f heading_error=%.3f\n",
      static_cast<int>(ok), kFarGoalCenter.x, kFarGoalCenter.y,
      final_pose.x_in, final_pose.y_in, final_pose.heading_deg,
      final_goal_distance_in, final_goal_bearing_deg,
      final_heading_error_deg);
  std::fflush(stdout);
  navigation::stop();
  navigation::update();
  stop_drive_motors();
  return ok && final_goal_distance_in <= 22.0 &&
      final_heading_error_deg <= 3.0;
}

void localization_navigation_api_boundary_test() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kTargetHeadingDeg = 3.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  std::printf("NAV_BOUNDARY stage=init ok=%d\n",
              static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  std::printf("NAV_BOUNDARY stage=before_go_to_pose\n");
  std::fflush(stdout);
  const navigation::Result result = navigation::go_to_pose(
      kStartXIn, kStartYIn, kTargetHeadingDeg, 40, 4000);
  std::printf("NAV_BOUNDARY stage=after_go_to_pose result=%s\n",
              navigation::result_name(result));
  std::fflush(stdout);
  navigation::update();
  std::printf("NAV_BOUNDARY stage=after_update\n");
  std::fflush(stdout);
  const navigation::Pose pose = navigation::current_pose();
  std::printf("NAV_BOUNDARY stage=after_pose x=%.3f y=%.3f heading=%.3f\n",
              pose.x_in, pose.y_in, pose.heading_deg);
  std::fflush(stdout);
  const navigation::SensorHealth health = navigation::sensor_health();
  std::printf(
      "NAV_BOUNDARY stage=after_health pose=%d imu=%d encoders=%d gps=%s\n",
      static_cast<int>(health.pose_valid), static_cast<int>(health.imu_valid),
      static_cast<int>(health.drive_encoders_valid), health.gps_state);
  std::fflush(stdout);
  navigation::stop();
}

void localization_navigation_dropout_abort_test() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kTargetXIn = -29.0;
  constexpr double kInjectedTravelIn = 1.5;
  constexpr double kWheelCircumferenceIn = kPi * kWheelDiameterIn;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(kStartXIn, kStartYIn, 0.0, 1.0);
  std::printf("NAV_DROPOUT event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  navigation_test_imu_dropout_after_in = kInjectedTravelIn;
  navigation_test_imu_dropout_latched = false;
  navigation_test_inject_imu_dropout = true;
  const navigation::Result result = navigation::go_straight_to(
      kTargetXIn, kStartYIn, 50, 5000);
  stop_drive_motors();
  const double stopped_left_deg = average_motor_position(chassis.left_motors);
  const double stopped_right_deg = average_motor_position(chassis.right_motors);
  const double abort_x = telemetry_pose.x;
  const double abort_y = telemetry_pose.y;
  const double abort_heading = pose_heading_deg(telemetry_pose);
  const double abort_travel = telemetry_pose.dead_reckoning_distance_in;
  navigation_test_inject_imu_dropout = false;
  pros::delay(250);
  const double final_left_deg = average_motor_position(chassis.left_motors);
  const double final_right_deg = average_motor_position(chassis.right_motors);
  const double coast_in = 0.5 *
      (std::fabs(final_left_deg - stopped_left_deg) +
       std::fabs(final_right_deg - stopped_right_deg)) /
      360.0 * kWheelCircumferenceIn;
  const bool reinitialized = navigation::init(
      abort_x, abort_y, abort_heading,
      telemetry_pose.position_error_envelope_in);
  std::printf(
      "NAV_DROPOUT event=complete result=%s injected=%d abort_travel=%.3f "
      "coast_in=%.4f reinitialized=%d x=%.3f y=%.3f heading=%.3f\n",
      navigation::result_name(result),
      static_cast<int>(navigation_test_imu_dropout_latched), abort_travel,
      coast_in, static_cast<int>(reinitialized), abort_x, abort_y,
      abort_heading);
  std::fflush(stdout);
  navigation::stop();
}

void localization_navigation_obstacle_abort_test() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kTargetXIn = -29.0;
  constexpr double kWheelCircumferenceIn = kPi * kWheelDiameterIn;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const ForwardObstacleObservation obstacle = read_forward_obstacle();
  const double preflight_distance_in =
      obstacle.valid ? obstacle.distance_mm / 25.4 : NAN;
  if (!obstacle.valid || obstacle.distance_mm == 9999 ||
      preflight_distance_in > localization::kForwardObstacleStopIn) {
    std::printf(
        "NAV_OBSTACLE event=preflight_reject valid=%d distance_in=%.3f "
        "required_max_in=%.3f\n",
        static_cast<int>(obstacle.valid), preflight_distance_in,
        localization::kForwardObstacleStopIn);
    std::fflush(stdout);
    return;
  }
  const bool initialized = navigation::init(kStartXIn, kStartYIn, 0.0, 1.0);
  std::printf("NAV_OBSTACLE event=init ok=%d distance_in=%.3f\n",
              static_cast<int>(initialized), preflight_distance_in);
  std::fflush(stdout);
  if (!initialized) return;
  const double start_left_deg = average_motor_position(chassis.left_motors);
  const double start_right_deg = average_motor_position(chassis.right_motors);
  const navigation::Result result = navigation::go_straight_to(
      kTargetXIn, kStartYIn, 50, 3000);
  stop_drive_motors();
  const double travel_in = 0.5 *
      (std::fabs(average_motor_position(chassis.left_motors) - start_left_deg) +
       std::fabs(average_motor_position(chassis.right_motors) - start_right_deg)) /
      360.0 * kWheelCircumferenceIn;
  std::printf(
      "NAV_OBSTACLE event=complete result=%s travel_in=%.4f distance_in=%.3f\n",
      navigation::result_name(result), travel_in,
      static_cast<double>(distance_1.get_distance()) / 25.4);
  std::fflush(stdout);
  navigation::stop();
}

void localization_navigation_straight_qualification() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr std::array<double, 3> kCumulativeTargetsIn{{3.0, 6.0, 9.0}};

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(kStartXIn, kStartYIn, 0.0, 1.0);
  std::printf("NAV_STRAIGHT event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  bool all_ok = true;
  for (std::size_t index = 0; index < kCumulativeTargetsIn.size(); ++index) {
    const double target_x = kStartXIn + kCumulativeTargetsIn[index];
    const std::uint32_t started_ms = pros::millis();
    const navigation::Result result = navigation::go_straight_to(
        target_x, kStartYIn, 50, 5000);
    navigation::update();
    const navigation::Pose pose = navigation::current_pose();
    const double position_error = std::hypot(
        pose.x_in - target_x, pose.y_in - kStartYIn);
    std::printf(
        "NAV_STRAIGHT event=leg_done index=%u target=%.3f result=%s "
        "elapsed_ms=%lu x=%.3f y=%.3f heading=%.3f error=%.3f "
        "envelope=%.3f\n",
        static_cast<unsigned>(index), kCumulativeTargetsIn[index],
        navigation::result_name(result),
        static_cast<unsigned long>(pros::millis() - started_ms), pose.x_in,
        pose.y_in, pose.heading_deg, position_error,
        pose.position_error_envelope_in);
    std::fflush(stdout);
    if (result != navigation::Result::kSuccess) {
      all_ok = false;
      break;
    }
    pros::delay(500);
  }
  navigation::stop();
  const navigation::Pose final_pose = navigation::current_pose();
  std::printf(
      "NAV_STRAIGHT event=complete ok=%d x=%.3f y=%.3f heading=%.3f "
      "total_error=%.3f\n",
      static_cast<int>(all_ok), final_pose.x_in, final_pose.y_in,
      final_pose.heading_deg,
      std::hypot(final_pose.x_in - (kStartXIn + kCumulativeTargetsIn.back()),
                 final_pose.y_in - kStartYIn));
  std::fflush(stdout);
}

void localization_navigation_return_route() {
  // Fused endpoint recorded by live_02 after its stall watchdog stopped the
  // first curve. Re-anchor there, use the public straight/turn API to return
  // to the qualification start, and never repeat this recovery automatically.
  constexpr double kMeasuredXIn = -25.066;
  constexpr double kMeasuredYIn = 9.289;
  constexpr double kMeasuredHeadingDeg = 62.938;
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kClearanceForwardIn = 8.0;
  const double kClearanceXIn =
      kMeasuredXIn + kClearanceForwardIn *
          std::cos(kMeasuredHeadingDeg * kPi / 180.0);
  const double kClearanceYIn =
      kMeasuredYIn + kClearanceForwardIn *
          std::sin(kMeasuredHeadingDeg * kPi / 180.0);

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  const bool initialized = navigation::init(
      kMeasuredXIn, kMeasuredYIn, kMeasuredHeadingDeg, 2.0);
  std::printf("NAV_RETURN event=init ok=%d\n", static_cast<int>(initialized));
  std::fflush(stdout);
  if (!initialized) return;

  const navigation::Result clearance = navigation::go_straight_to(
      kClearanceXIn, kClearanceYIn, 35, 6000);
  const navigation::Result drive = clearance == navigation::Result::kSuccess
      ? navigation::go_straight_to(kStartXIn, kStartYIn, 45, 12000)
      : navigation::Result::kDriveFailed;
  const navigation::Result turn = drive == navigation::Result::kSuccess
      ? navigation::turn_to(kStartHeadingDeg, 40, 6000)
      : navigation::Result::kDriveFailed;
  navigation::update();
  const navigation::Pose pose = navigation::current_pose();
  std::printf(
      "NAV_RETURN event=complete clearance=%s drive=%s turn=%s x=%.3f y=%.3f "
      "heading=%.3f position_error=%.3f heading_error=%.3f\n",
      navigation::result_name(clearance), navigation::result_name(drive),
      navigation::result_name(turn),
      pose.x_in, pose.y_in, pose.heading_deg,
      std::hypot(pose.x_in - kStartXIn, pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg, pose.heading_deg)));
  std::fflush(stdout);
  navigation::stop();
  stop_drive_motors();
}

void localization_drive_response_test() {
  constexpr double kStartXIn = -35.0;
  constexpr double kStartYIn = 0.0;
  constexpr double kStartHeadingDeg = 0.0;
  constexpr double kTargetXIn = -33.0;
  constexpr int kMaxPower = 35;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  log_drive_health("response_start");
  const bool initialized = navigation::init(
      kStartXIn, kStartYIn, kStartHeadingDeg, 1.0);
  const navigation::Result result = initialized
      ? navigation::go_straight_to(
            kTargetXIn, kStartYIn, kMaxPower, 4000)
      : navigation::Result::kPoseUnavailable;
  navigation::update();
  const navigation::Pose pose = navigation::current_pose();
  log_drive_health("response_done");
  std::printf(
      "NAV_RESPONSE initialized=%d result=%s x=%.3f y=%.3f heading=%.3f "
      "distance_error=%.3f heading_error=%.3f\n",
      static_cast<int>(initialized), navigation::result_name(result),
      pose.x_in, pose.y_in, pose.heading_deg,
      std::hypot(pose.x_in - kTargetXIn, pose.y_in - kStartYIn),
      std::fabs(signed_angle_diff_deg(kStartHeadingDeg, pose.heading_deg)));
  std::fflush(stdout);
  navigation::stop();
  stop_drive_motors();
}

void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.pid_angle_behavior_set(ez::shortest);
  pid_autotune_apply_if_ready();
}

void simple_goal_avoidance_auton() {
  default_constants();
  chassis.pid_targets_reset();
  pros::lcd::set_text(6, "Auton fused bypass");

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  for (int i = 0; i < 10; ++i) {
    update_pose(pose);
    pros::delay(20);
  }

  const double start_x = pose.x;
  const double start_y = pose.y;
  const std::array<Waypoint, 4> route = {{
      {start_x, start_y + 8.0},
      {start_x + 16.0, start_y + 20.0},
      {start_x + 16.0, start_y + 30.0},
      {start_x, start_y + 36.0},
  }};

  for (const auto& point : route) {
    drive_to_point(pose, point, 55, 4500);
    pros::delay(120);
  }

  stop_drive_motors();
  pros::lcd::set_text(6, "Auton done");
}

void fusion_test_auton() {
  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  chassis.drive_sensor_reset();
  horizontal_odom.reset_position();
  chassis.drive_imu_reset(0.0);
  pros::delay(80);
  pros::lcd::set_text(6, "Slow 20R x6");

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  sample_fusion_for(pose, "start", 200);

  const Waypoint route_start{pose.x, pose.y};
  const double route_start_heading_deg = pose_heading_deg(pose);
  printf(
      "FUSE_TEST route=slow_long_start x=%.2f y=%.2f heading=%.2f travel_total=240.0 turn_total=60.0 max_drive=35\n",
      route_start.x,
      route_start.y,
      route_start_heading_deg);
  fflush(stdout);

  bool route_ok = true;
  const char* back_phases[] = {
      "slow_back_1", "slow_back_2", "slow_back_3",
      "slow_back_4", "slow_back_5", "slow_back_6"};
  const char* return_phases[] = {
      "slow_return_1", "slow_return_2", "slow_return_3",
      "slow_return_4", "slow_return_5", "slow_return_6"};
  for (int pair = 0; pair < 6 && route_ok; ++pair) {
    route_ok = drive_forward_test_leg(
        pose, back_phases[pair], -20.0, 35, 7500);
    sample_fusion_for(pose, "slow_endpoint", 350);
    if (route_ok) {
      route_ok = drive_forward_test_leg(
          pose, return_phases[pair], 20.0, 35, 7500);
    }
    sample_fusion_for(pose, "slow_home", 500);

    // Two bounded out-and-back heading excursions exercise the rear-wheel
    // rotation compensation and camera/LiDAR reacquisition without changing
    // the route's spatial envelope.
    if (route_ok && pair == 1) {
      route_ok = fused_turn_to_heading(
          pose, "slow_turn_plus_15", route_start_heading_deg + 15.0, 35, 4000);
      sample_fusion_for(pose, "slow_plus_15_hold", 500);
      if (route_ok) route_ok = fused_turn_to_heading(
          pose, "slow_turn_plus_return", route_start_heading_deg, 35, 4000);
      sample_fusion_for(pose, "slow_plus_return_hold", 500);
    }
    if (route_ok && pair == 3) {
      route_ok = fused_turn_to_heading(
          pose, "slow_turn_minus_15", route_start_heading_deg - 15.0, 35, 4000);
      sample_fusion_for(pose, "slow_minus_15_hold", 500);
      if (route_ok) route_ok = fused_turn_to_heading(
          pose, "slow_turn_minus_return", route_start_heading_deg, 35, 4000);
      sample_fusion_for(pose, "slow_minus_return_hold", 500);
    }
  }
  sample_fusion_for(pose, "long_final", 500);

  stop_drive_motors();
  const double return_error_in = std::hypot(
      pose.x - route_start.x, pose.y - route_start.y);
  const double return_heading_error_deg = std::fabs(signed_angle_diff_deg(
      pose_heading_deg(pose), route_start_heading_deg));
  printf(
      "FUSE_TEST route=slow_long_done ok=%d x=%.2f y=%.2f heading=%.2f return_error=%.2f heading_error=%.2f\n",
      static_cast<int>(route_ok), pose.x, pose.y, pose_heading_deg(pose),
      return_error_in, return_heading_error_deg);
  fflush(stdout);

  // Hand the exact estimator state and sensor baselines to normal telemetry;
  // otherwise the UI reinitializes at the entered start after a diagnostic.
  telemetry_pose = pose;
  telemetry_pose_initialized = true;
  publish_telemetry_snapshot();
  telemetry_last_log_ms = 0;

  pros::lcd::print(6,
                   "Long %d e%.1f h%.1f",
                   static_cast<int>(route_ok),
                   return_error_in,
                   return_heading_error_deg,
                   pose.x,
                   pose.y);
}

void gps_obstacle_aware_route_test() {
  constexpr double kInchesPerMeter = 39.37007874015748;
  constexpr int kDrivePower = 40;
  constexpr int kTurnPower = 45;
  auto& gps = gps_7;

  auto sample_robot_pose = [&]() {
    constexpr int kSamples = 20;
    double sensor_x_in = 0.0;
    double sensor_y_in = 0.0;
    double heading_sin = 0.0;
    double heading_cos = 0.0;
    double error_in = 0.0;
    for (int i = 0; i < kSamples; ++i) {
      const auto position = gps.get_position();
      sensor_x_in += position.x * kInchesPerMeter;
      sensor_y_in += position.y * kInchesPerMeter;
      const double heading_rad = deg_to_rad(gps.get_heading());
      heading_sin += std::sin(heading_rad);
      heading_cos += std::cos(heading_rad);
      error_in += gps.get_error() * kInchesPerMeter;
      pros::delay(50);
    }
    sensor_x_in /= kSamples;
    sensor_y_in /= kSamples;
    error_in /= kSamples;
    double sensor_heading_cw_deg =
        rad_to_deg(std::atan2(heading_sin, heading_cos));
    sensor_heading_cw_deg = normalize_deg(sensor_heading_cw_deg);
    const auto project_pose = localization::vex_gps_to_project_robot_pose(
        sensor_x_in / kInchesPerMeter,
        sensor_y_in / kInchesPerMeter,
        sensor_heading_cw_deg);
    return std::array<double, 4>{project_pose.x_in,
                                 project_pose.y_in,
                                 project_pose.heading_deg,
                                 error_in};
  };

  auto point_segment_distance = [](Waypoint point, Waypoint a, Waypoint b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= 1e-9) return std::hypot(point.x - a.x, point.y - a.y);
    const double t = clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) /
                               length_sq,
                           0.0, 1.0);
    return std::hypot(point.x - (a.x + t * dx),
                      point.y - (a.y + t * dy));
  };
  auto point_inside_walls = [](Waypoint point) {
    const double limit = localization::kPhysicalWallHalfSpanIn -
                         localization::kNavigationProvisionalWallClearanceIn;
    return std::fabs(point.x) <= limit && std::fabs(point.y) <= limit;
  };
  double required_goal_clearance =
      localization::kNavigationProvisionalGoalClearanceIn;
  auto segment_is_safe = [&](Waypoint a, Waypoint b, double& minimum_clearance) {
    if (!point_inside_walls(a) || !point_inside_walls(b)) return false;
    minimum_clearance = std::numeric_limits<double>::infinity();
    for (const auto& landmark : localization::kGoalTagLandmarks) {
      const Waypoint goal{landmark.x_in, landmark.y_in};
      minimum_clearance = std::min(
          minimum_clearance, point_segment_distance(goal, a, b));
    }
    return minimum_clearance >= required_goal_clearance;
  };

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_drive_motors();
  chassis.drive_sensor_reset();
  horizontal_odom.reset_position();
  if (!gps.is_installed() || !chassis.imu.is_installed()) {
    printf("GPS_ROUTE abort=sensor_missing gps=%d imu=%d\n",
           static_cast<int>(gps.is_installed()),
           static_cast<int>(chassis.imu.is_installed()));
    fflush(stdout);
    pros::lcd::set_text(6, "GPS route SENSOR FAIL");
    return;
  }

  chassis.imu.reset();
  const std::uint32_t imu_calibration_started = pros::millis();
  while (chassis.imu.is_calibrating() &&
         pros::millis() - imu_calibration_started < 5000) {
    pros::delay(20);
  }
  if (chassis.imu.is_calibrating() ||
      chassis.imu.get_status() == pros::ImuStatus::error) {
    printf("GPS_ROUTE abort=imu_calibration status=%d\n",
           static_cast<int>(chassis.imu.get_status()));
    fflush(stdout);
    pros::lcd::set_text(6, "GPS route IMU FAIL");
    return;
  }
  chassis.drive_imu_reset(0.0);

  const auto start_reading = sample_robot_pose();
  const localization::FieldPose start_pose{
      start_reading[0], start_reading[1], start_reading[2]};
  const Waypoint start{start_pose.x_in, start_pose.y_in};
  double start_goal_clearance = std::numeric_limits<double>::infinity();
  for (const auto& landmark : localization::kGoalTagLandmarks) {
    const Waypoint goal{landmark.x_in, landmark.y_in};
    start_goal_clearance = std::min(
        start_goal_clearance, std::hypot(goal.x - start.x, goal.y - start.y));
  }
  required_goal_clearance = std::min(
      localization::kNavigationProvisionalGoalClearanceIn,
      std::max(localization::kNavigationMinimumGoalEscapeClearanceIn,
               start_goal_clearance - 0.5));
  const std::array<Waypoint, 4> route = {{
      start,
      {-22.0, -10.0},
      {-22.0, 20.0},
      start,
  }};
  printf("GPS_ROUTE start x=%.2f y=%.2f heading=%.2f gps_error=%.2f nearest_goal=%.2f required_clearance=%.2f\n",
         start_pose.x_in, start_pose.y_in, start_pose.heading_deg,
         start_reading[3], start_goal_clearance, required_goal_clearance);

  bool safe = start_reading[3] <= 2.0;
  double planned_distance = 0.0;
  for (std::size_t i = 1; i < route.size(); ++i) {
    double clearance = 0.0;
    const bool segment_safe = segment_is_safe(route[i - 1], route[i], clearance);
    const double distance = std::hypot(route[i].x - route[i - 1].x,
                                       route[i].y - route[i - 1].y);
    planned_distance += distance;
    printf("GPS_ROUTE plan segment=%u from=%.2f,%.2f to=%.2f,%.2f distance=%.2f goal_clearance=%.2f safe=%d\n",
           static_cast<unsigned>(i), route[i - 1].x, route[i - 1].y,
           route[i].x, route[i].y, distance, clearance,
           static_cast<int>(segment_safe));
    safe = safe && segment_safe;
  }
  printf("GPS_ROUTE plan total_distance=%.2f safe=%d\n",
         planned_distance, static_cast<int>(safe));
  fflush(stdout);
  if (!safe) {
    pros::lcd::set_text(6, "GPS route PLAN ABORT");
    stop_drive_motors();
    return;
  }

  PoseEstimate pose;
  init_pose(pose, start_pose);
  sample_fusion_for(pose, "gps_route_start", 200, LidarFusionMode::kDisabled);
  bool route_ok = true;
  for (std::size_t i = 1; i < route.size() && route_ok; ++i) {
    const double bearing_deg = normalize_deg(
        rad_to_deg(std::atan2(route[i].y - pose.y, route[i].x - pose.x)));
    char turn_phase[24];
    char drive_phase[24];
    std::snprintf(turn_phase, sizeof(turn_phase), "gps_turn_%u",
                  static_cast<unsigned>(i));
    std::snprintf(drive_phase, sizeof(drive_phase), "gps_drive_%u",
                  static_cast<unsigned>(i));
    route_ok = fused_turn_to_heading(
        pose, turn_phase, bearing_deg, kTurnPower, 6000);
    if (route_ok) {
      route_ok = fused_drive_to_point(
          pose, drive_phase, route[i], bearing_deg, kDrivePower, 12000);
    }
    stop_drive_motors();
    const auto gps_reading = sample_robot_pose();
    const double odom_gps_error = std::hypot(
        pose.x - gps_reading[0], pose.y - gps_reading[1]);
    printf("GPS_ROUTE checkpoint=%u ok=%d odom_x=%.2f odom_y=%.2f gps_x=%.2f gps_y=%.2f disagreement=%.2f gps_error=%.2f\n",
           static_cast<unsigned>(i), static_cast<int>(route_ok),
           pose.x, pose.y, gps_reading[0], gps_reading[1],
           odom_gps_error, gps_reading[3]);
    fflush(stdout);
    if (gps_reading[3] <= 2.0 && odom_gps_error <= 6.0) {
      pose.x = gps_reading[0];
      pose.y = gps_reading[1];
    } else {
      route_ok = false;
    }
  }

  stop_drive_motors();
  const auto final_reading = sample_robot_pose();
  const double return_error = std::hypot(
      final_reading[0] - start_pose.x_in,
      final_reading[1] - start_pose.y_in);
  const double heading_error = std::fabs(signed_angle_diff_deg(
      final_reading[2], start_pose.heading_deg));
  printf("GPS_ROUTE done ok=%d final_x=%.2f final_y=%.2f final_heading=%.2f return_error=%.2f heading_error=%.2f\n",
         static_cast<int>(route_ok), final_reading[0], final_reading[1],
         final_reading[2], return_error, heading_error);
  fflush(stdout);
  pros::lcd::print(6, "Route %s e%.1f h%.1f",
                   route_ok ? "OK" : "FAIL", return_error, heading_error);
}

void localization_slow_rotation_calibration() {
  constexpr double kCalibrationTurnDeg = 12.0;
  constexpr double kCalibrationTurnRpm = 8.0;
  constexpr std::uint32_t kCalibrationTurnTimeoutMs = 4000;
  constexpr std::uint32_t kCalibrationSampleMs = 1800;

  struct CalibrationEndpoint {
    double lidar_theta_deg = NAN;
    double field_imu_heading_deg = NAN;
    double left_motor_deg = NAN;
    double right_motor_deg = NAN;
    double left_spread_deg = NAN;
    double right_spread_deg = NAN;
    std::int32_t side_centideg = 0;
    int lidar_samples = 0;
    bool lidar_valid = false;
    bool imu_valid = false;
    bool drive_valid = false;
    bool side_valid = false;
  };

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  stop_drive_motors();
  chassis.drive_sensor_reset();
  horizontal_odom.reset_position();
  chassis.drive_imu_reset(0.0);
  pros::delay(100);

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  auto median = [](std::vector<double> values) -> double {
    if (values.empty()) return NAN;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0
               ? 0.5 * (values[middle - 1] + values[middle])
               : values[middle];
  };
  auto sample_stationary = [&](const char* phase) {
    std::vector<double> lidar_theta_samples;
    const std::uint32_t start = pros::millis();
    std::uint32_t last_log = 0;
    std::uint32_t last_lidar_sample_ms = 0;
    do {
      update_pose(pose, LidarFusionMode::kBiasOnly);
      const std::uint32_t now = pros::millis();
      if (pose.last_lidar_sample_ms != last_lidar_sample_ms) {
        last_lidar_sample_ms = pose.last_lidar_sample_ms;
        if (std::isfinite(pose.lidar_theta_deg) &&
            std::isfinite(pose.lidar_rmse_in) &&
            pose.lidar_rmse_in <= kMaxLidarRmseIn) {
          lidar_theta_samples.push_back(pose.lidar_theta_deg);
        }
      }
      if (now - last_log >= 100) {
        log_pose(phase, pose);
        last_log = now;
      }
      pros::delay(20);
    } while (pros::millis() - start < kCalibrationSampleMs);
    CalibrationEndpoint endpoint;
    endpoint.lidar_theta_deg = median(lidar_theta_samples);
    endpoint.field_imu_heading_deg = read_field_imu_heading_deg(pose);
    const MotorSideReading left = read_motor_side(chassis.left_motors);
    const MotorSideReading right = read_motor_side(chassis.right_motors);
    endpoint.left_motor_deg = left.position_deg;
    endpoint.right_motor_deg = right.position_deg;
    endpoint.left_spread_deg = left.spread_deg;
    endpoint.right_spread_deg = right.spread_deg;
    endpoint.side_centideg = horizontal_odom.get_position();
    endpoint.lidar_samples = static_cast<int>(lidar_theta_samples.size());
    endpoint.lidar_valid = std::isfinite(endpoint.lidar_theta_deg) &&
                           endpoint.lidar_samples >= 5;
    endpoint.imu_valid = std::isfinite(endpoint.field_imu_heading_deg);
    endpoint.drive_valid = left.trustworthy && right.trustworthy;
    endpoint.side_valid = endpoint.side_centideg != PROS_ERR;
    printf("FUSE_ODOM_CAL endpoint=%s theta=%.3f field_imu=%.3f h5=%ld left_deg=%.3f right_deg=%.3f spread=%.3f/%.3f samples=%d valid=L%d/I%d/D%d/S%d\n",
           phase,
           endpoint.lidar_theta_deg,
           endpoint.field_imu_heading_deg,
           static_cast<long>(endpoint.side_centideg),
           endpoint.left_motor_deg,
           endpoint.right_motor_deg,
           endpoint.left_spread_deg,
           endpoint.right_spread_deg,
           endpoint.lidar_samples,
           static_cast<int>(endpoint.lidar_valid),
           static_cast<int>(endpoint.imu_valid),
           static_cast<int>(endpoint.drive_valid),
           static_cast<int>(endpoint.side_valid));
    fflush(stdout);
    return endpoint;
  };

  pros::lcd::set_text(6, "Slow LiDAR calibration");
  printf("FUSE_CAL event=start turn_deg=%.1f power=%d\n",
         kCalibrationTurnDeg,
         static_cast<int>(kCalibrationTurnRpm));
  fflush(stdout);
  std::array<CalibrationEndpoint, 5> endpoints;
  endpoints[0] = sample_stationary("cal_start");

  auto turn_slow_velocity = [&](const char* phase, double target_delta_cw_deg) {
    const double start_imu_cw_deg = chassis.drive_imu_get();
    const std::uint32_t start = pros::millis();
    std::uint32_t settled_since = 0;
    std::uint32_t last_log = 0;
    bool success = false;
    while (pros::millis() - start < kCalibrationTurnTimeoutMs) {
      update_pose(pose, LidarFusionMode::kDisabled);
      const double imu_cw_deg = chassis.drive_imu_get();
      if (!std::isfinite(imu_cw_deg)) break;
      const double traveled_cw_deg = imu_cw_deg - start_imu_cw_deg;
      const double error_deg = target_delta_cw_deg - traveled_cw_deg;
      if (std::fabs(traveled_cw_deg) > std::fabs(target_delta_cw_deg) + 3.0) {
        break;
      }
      if (std::fabs(error_deg) <= 0.7) {
        if (settled_since == 0) settled_since = pros::millis();
        stop_drive_motors();
        if (pros::millis() - settled_since >= 100) {
          success = true;
          break;
        }
        pros::delay(20);
        continue;
      } else {
        settled_since = 0;
      }
      const double command_cw_rpm =
          (error_deg >= 0.0 ? 1.0 : -1.0) * kCalibrationTurnRpm;
      for (auto& motor : chassis.left_motors) {
        motor.move_velocity(command_cw_rpm);
      }
      for (auto& motor : chassis.right_motors) {
        motor.move_velocity(-command_cw_rpm);
      }
      const std::uint32_t now = pros::millis();
      if (now - last_log >= 100) {
        log_pose(phase, pose);
        last_log = now;
      }
      pros::delay(20);
    }
    stop_drive_motors();
    return success;
  };

  const bool cw_ok = turn_slow_velocity("cal_cw_motion", kCalibrationTurnDeg);
  stop_drive_motors();
  endpoints[1] = sample_stationary("cal_cw_stop");

  const bool cw_return_ok =
      turn_slow_velocity("cal_cw_return", -kCalibrationTurnDeg);
  stop_drive_motors();
  endpoints[2] = sample_stationary("cal_center_1");

  const bool ccw_ok =
      turn_slow_velocity("cal_ccw_motion", -kCalibrationTurnDeg);
  stop_drive_motors();
  endpoints[3] = sample_stationary("cal_ccw_stop");

  const bool ccw_return_ok =
      turn_slow_velocity("cal_ccw_return", kCalibrationTurnDeg);
  stop_drive_motors();
  endpoints[4] = sample_stationary("cal_final");

  double side_lidar_numerator = 0.0;
  double side_lidar_denominator = 0.0;
  double side_imu_numerator = 0.0;
  double side_imu_denominator = 0.0;
  double drive_imu_numerator = 0.0;
  double drive_imu_denominator = 0.0;
  double lidar_imu_numerator = 0.0;
  double lidar_imu_denominator = 0.0;
  double lidar_imu_residual_sq_sum = 0.0;
  int valid_lidar_side_segments = 0;
  int valid_imu_side_segments = 0;
  int valid_drive_segments = 0;
  int valid_lidar_imu_segments = 0;
  for (std::size_t i = 1; i < endpoints.size(); ++i) {
    const auto& previous = endpoints[i - 1];
    const auto& current = endpoints[i];
    const bool side_pair_valid = previous.side_valid && current.side_valid;
    const bool imu_pair_valid = previous.imu_valid && current.imu_valid;
    const bool lidar_pair_valid = previous.lidar_valid && current.lidar_valid;
    const bool drive_pair_valid = previous.drive_valid && current.drive_valid;
    const double delta_lidar_deg = lidar_pair_valid
                                         ? signed_angle_diff_deg(
                                               current.lidar_theta_deg,
                                               previous.lidar_theta_deg)
                                         : NAN;
    const double delta_field_imu_deg = imu_pair_valid
                                           ? signed_angle_diff_deg(
                                                 current.field_imu_heading_deg,
                                                 previous.field_imu_heading_deg)
                                           : NAN;
    const double delta_field_imu_rad = deg_to_rad(delta_field_imu_deg);
    const double delta_side_in = side_pair_valid
                                     ? ((static_cast<double>(current.side_centideg) -
                                         static_cast<double>(previous.side_centideg)) /
                                        100.0 / 360.0) *
                                           kSideOdomCircumferenceIn * kSideOdomRawSign
                                     : NAN;
    const double delta_left_in = drive_pair_valid
                                     ? ((current.left_motor_deg - previous.left_motor_deg) /
                                        360.0) *
                                           kWheelCircumferenceIn * kLeftEncoderSign
                                     : NAN;
    const double delta_right_in = drive_pair_valid
                                      ? ((current.right_motor_deg - previous.right_motor_deg) /
                                         360.0) *
                                            kWheelCircumferenceIn * kRightEncoderSign
                                      : NAN;
    const double drive_heading_numerator_in = delta_right_in - delta_left_in;

    if (side_pair_valid && lidar_pair_valid &&
        std::fabs(delta_lidar_deg) >= 4.0) {
      const double delta_lidar_rad = deg_to_rad(delta_lidar_deg);
      side_lidar_numerator += delta_lidar_rad * delta_side_in;
      side_lidar_denominator += delta_lidar_rad * delta_lidar_rad;
      ++valid_lidar_side_segments;
    }
    if (side_pair_valid && imu_pair_valid &&
        std::fabs(delta_field_imu_deg) >= 4.0) {
      side_imu_numerator += delta_field_imu_rad * delta_side_in;
      side_imu_denominator += delta_field_imu_rad * delta_field_imu_rad;
      ++valid_imu_side_segments;
    }
    if (drive_pair_valid && imu_pair_valid &&
        std::fabs(delta_field_imu_deg) >= 4.0) {
      drive_imu_numerator += delta_field_imu_rad * drive_heading_numerator_in;
      drive_imu_denominator += delta_field_imu_rad * delta_field_imu_rad;
      ++valid_drive_segments;
    }
    if (lidar_pair_valid && imu_pair_valid &&
        std::fabs(delta_field_imu_deg) >= 4.0) {
      lidar_imu_numerator += delta_field_imu_deg * delta_lidar_deg;
      lidar_imu_denominator += delta_field_imu_deg * delta_field_imu_deg;
      const double residual_deg = delta_lidar_deg - delta_field_imu_deg;
      lidar_imu_residual_sq_sum += residual_deg * residual_deg;
      ++valid_lidar_imu_segments;
    }

    const double segment_track_width_in =
        drive_pair_valid && imu_pair_valid &&
                std::fabs(delta_field_imu_rad) >= deg_to_rad(4.0)
            ? drive_heading_numerator_in / delta_field_imu_rad
            : NAN;
    const double segment_rear_offset_in =
        side_pair_valid && imu_pair_valid &&
                std::fabs(delta_field_imu_rad) >= deg_to_rad(4.0)
            ? delta_side_in / delta_field_imu_rad
            : NAN;
    printf("FUSE_ODOM_CAL segment=%d dlidar=%.3f dimu=%.3f dh5=%ld side_in=%.4f left_in=%.4f right_in=%.4f track_in=%.4f rear_in=%.4f lidar_imu_resid=%.3f\n",
           static_cast<int>(i),
           delta_lidar_deg,
           delta_field_imu_deg,
           static_cast<long>(current.side_centideg - previous.side_centideg),
           delta_side_in,
           delta_left_in,
           delta_right_in,
           segment_track_width_in,
           segment_rear_offset_in,
           delta_lidar_deg - delta_field_imu_deg);
  }
  const double side_lidar_slope_in_per_rad =
      side_lidar_denominator > 0.0
          ? side_lidar_numerator / side_lidar_denominator
          : NAN;
  const double side_imu_slope_in_per_rad =
      side_imu_denominator > 0.0
          ? side_imu_numerator / side_imu_denominator
          : NAN;
  const double measured_track_width_in =
      drive_imu_denominator > 0.0
          ? drive_imu_numerator / drive_imu_denominator
          : NAN;
  const double lidar_per_imu_scale =
      lidar_imu_denominator > 0.0
          ? lidar_imu_numerator / lidar_imu_denominator
          : NAN;
  const double lidar_imu_rmse_deg = valid_lidar_imu_segments > 0
                                        ? std::sqrt(lidar_imu_residual_sq_sum /
                                                    valid_lidar_imu_segments)
                                        : NAN;
  const double measured_sign = side_imu_slope_in_per_rad >= 0.0 ? 1.0 : -1.0;
  const double measured_rear_offset_in = std::fabs(side_imu_slope_in_per_rad);

  printf("FUSE_CAL event=done cw=%d cw_return=%d ccw=%d ccw_return=%d x=%.2f y=%.2f heading=%.2f lidar_side_segments=%d imu_side_segments=%d drive_segments=%d lidar_imu_segments=%d odom_sign=%+.0f odom_offset_lidar_in=%.4f odom_offset_imu_in=%.4f track_width_imu_in=%.4f lidar_per_imu=%.5f lidar_imu_rmse_deg=%.3f configured_sign=%+.0f configured_offset_in=%.4f configured_track_in=%.4f\n",
         static_cast<int>(cw_ok),
         static_cast<int>(cw_return_ok),
         static_cast<int>(ccw_ok),
         static_cast<int>(ccw_return_ok),
         pose.x,
         pose.y,
         pose_heading_deg(pose),
         valid_lidar_side_segments,
         valid_imu_side_segments,
         valid_drive_segments,
         valid_lidar_imu_segments,
         measured_sign,
         std::fabs(side_lidar_slope_in_per_rad),
         measured_rear_offset_in,
         measured_track_width_in,
         lidar_per_imu_scale,
         lidar_imu_rmse_deg,
         kSideOdomRawSign,
         kSideOdomOffsetBackIn,
         kTrackWidthIn);
  fflush(stdout);
  stop_drive_motors();
  pros::lcd::set_text(6, "Slow calibration done");
}

void localization_slow_forward_calibration() {
  constexpr double kTargetIn = 0.5;
  constexpr double kVelocityRpm = 6.0;
  constexpr std::uint32_t kTimeoutMs = 2000;
  constexpr double kReturnToleranceDeg = 5.0;
  constexpr bool kReturnAfterMove = false;
  const double target_motor_deg =
      (std::fabs(kTargetIn) / kWheelCircumferenceIn) * 360.0;
  const double motion_sign = kTargetIn >= 0.0 ? 1.0 : -1.0;

  default_constants();
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  stop_drive_motors();
  chassis.drive_sensor_reset();
  horizontal_odom.reset_position();
  chassis.drive_imu_reset(0.0);
  pros::delay(100);

  PoseEstimate pose;
  init_pose(pose, runtime_start_pose);
  const std::array<double, 4> baseline = {
      chassis.left_motors[0].get_position(),
      chassis.left_motors[1].get_position(),
      chassis.right_motors[0].get_position(),
      chassis.right_motors[1].get_position(),
  };
  const double start_imu_cw_deg = chassis.drive_imu_get();

  auto motor_deltas = [&]() {
    return std::array<double, 4>{
        chassis.left_motors[0].get_position() - baseline[0],
        chassis.left_motors[1].get_position() - baseline[1],
        chassis.right_motors[0].get_position() - baseline[2],
        chassis.right_motors[1].get_position() - baseline[3],
    };
  };
  auto max_abs_delta = [](const std::array<double, 4>& deltas) {
    double maximum = 0.0;
    for (double delta : deltas) maximum = std::max(maximum, std::fabs(delta));
    return maximum;
  };
  auto set_velocity = [&](double rpm) {
    for (auto& motor : chassis.left_motors) motor.move_velocity(rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(rpm);
  };

  std::uint32_t start = pros::millis();
  std::uint32_t last_log = 0;
  bool forward_ok = false;
  while (pros::millis() - start < kTimeoutMs) {
    update_pose(pose, LidarFusionMode::kDisabled);
    const auto deltas = motor_deltas();
    const double imu_delta = chassis.drive_imu_get() - start_imu_cw_deg;
    if (!std::isfinite(imu_delta) || std::fabs(imu_delta) > 5.0) break;
    if (max_abs_delta(deltas) >= target_motor_deg) {
      forward_ok = true;
      break;
    }
    set_velocity(motion_sign * kVelocityRpm);
    const std::uint32_t now = pros::millis();
    if (now - last_log >= 100) {
      log_pose("drive_cal_forward", pose);
      last_log = now;
    }
    pros::delay(20);
  }
  stop_drive_motors();
  const auto forward_deltas = motor_deltas();
  printf("FUSE_TEST phase=drive_cal_peak x=%.2f y=%.2f heading=%.2f m17_delta=%.1f m18_delta=%.1f m11_delta=%.1f m13_delta=%.1f h5_delta=%ld imu_delta=%.2f ok=%d\n",
         pose.x,
         pose.y,
         pose_heading_deg(pose),
         forward_deltas[0],
         forward_deltas[1],
         forward_deltas[2],
         forward_deltas[3],
         static_cast<long>(horizontal_odom.get_position()),
         chassis.drive_imu_get() - start_imu_cw_deg,
         static_cast<int>(forward_ok));
  fflush(stdout);
  pros::delay(500);

  bool return_ok = !kReturnAfterMove;
  if (kReturnAfterMove) {
    start = pros::millis();
    last_log = 0;
    while (pros::millis() - start < kTimeoutMs) {
      update_pose(pose, LidarFusionMode::kDisabled);
      const auto deltas = motor_deltas();
      if (max_abs_delta(deltas) <= kReturnToleranceDeg) {
        return_ok = true;
        break;
      }
      set_velocity(-kVelocityRpm);
      const std::uint32_t now = pros::millis();
      if (now - last_log >= 100) {
        log_pose("drive_cal_return", pose);
        last_log = now;
      }
      pros::delay(20);
    }
  }
  stop_drive_motors();
  const auto final_deltas = motor_deltas();
  printf("FUSE_TEST phase=drive_cal_final x=%.2f y=%.2f heading=%.2f m17_delta=%.1f m18_delta=%.1f m11_delta=%.1f m13_delta=%.1f h5_delta=%ld imu_delta=%.2f forward_ok=%d return_ok=%d\n",
         pose.x,
         pose.y,
         pose_heading_deg(pose),
         final_deltas[0],
         final_deltas[1],
         final_deltas[2],
         final_deltas[3],
         static_cast<long>(horizontal_odom.get_position()),
         chassis.drive_imu_get() - start_imu_cw_deg,
         static_cast<int>(forward_ok),
         static_cast<int>(return_ok));
  fflush(stdout);
  pros::delay(500);
}
