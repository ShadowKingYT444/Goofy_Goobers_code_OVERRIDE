#include "main.h"
#include "localization_config.hpp"
#include "pid_autotune.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
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
constexpr double kFusedDriveToleranceIn = 1.0;
constexpr double kFusedDriveSettleMs = 80;
constexpr double kFusedDriveKp = 7.5;
constexpr double kFusedDriveMinPower = 18.0;
constexpr double kFusedDriveHeadingKp = 1.15;
constexpr double kFusedDriveFinalHeadingKp = 0.35;
constexpr double kFusedDriveMaxTurnPower = 32.0;
constexpr double kFusedDriveForwardSlewPowerPerSec = 280.0;
constexpr double kFusedDriveTurnSlewPowerPerSec = 420.0;
constexpr double kFusedTurnToleranceDeg = 2.0;
constexpr double kFusedTurnSettleMs = 100;
constexpr double kFusedTurnKp = 0.95;
constexpr double kFusedTurnKd = 0.08;
constexpr double kFusedTurnMinPower = 18.0;
// Do not leave a 2-4 degree dead zone where the controller is outside settle
// tolerance but below the minimum-static-friction command threshold.
constexpr double kFusedTurnMinPowerErrorDeg = kFusedTurnToleranceDeg;
constexpr double kFusedTurnSlewPowerPerSec = 360.0;
constexpr std::uint32_t kFusedTurnStallTimeoutMs = 1000;

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
  std::uint32_t last_ai_correction_ms = 0;
  int consistent_ai_observations = 0;
  const char* last_ai_goal = "none";
  const char* last_ai_face = "none";
  const char* ai_goal = "none";
  const char* ai_face = "none";
  const char* ai_reject = "not_initialized";
};

PoseEstimate telemetry_pose;
bool telemetry_pose_initialized = false;
std::uint32_t telemetry_last_log_ms = 0;
localization::FieldPose runtime_start_pose = localization::kEnteredStartPose;

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

void update_ai_vision_shadow(PoseEstimate& pose, std::uint32_t now) {
  const auto& observation = ai_vision_shadow_snapshot();
  pose.ai_tag_id = observation.tag_id;
  pose.ai_bearing_right_deg = observation.bearing_deg;
  pose.ai_goal = "none";
  pose.ai_face = "none";
  pose.ai_candidate_residual_deg = NAN;
  pose.ai_candidate_margin_deg = NAN;
  pose.ai_observed_range_in = observation.range_estimate_in;
  pose.ai_predicted_range_in = NAN;
  pose.ai_range_residual_in = NAN;
  pose.ai_position_innovation_in = NAN;
  pose.ai_position_step_in = 0.0;
  pose.ai_heading_step_deg = 0.0;
  pose.ai_age_ms = now >= observation.brain_ms ? now - observation.brain_ms : 0;
  pose.ai_reject = observation.reason;

  if (!observation.installed || !observation.configured ||
      !observation.tag_valid) {
    return;
  }
  if (pose.ai_age_ms > localization::kAiVisionPollPeriodMs * 3) {
    pose.ai_reject = "stale";
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
  std::vector<AiLandmarkCandidate> candidates;
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
          std::fabs(observation.range_estimate_in - predicted_range_in);
      candidates.push_back(AiLandmarkCandidate{
          landmark.name,
          kFaceNames[face_index],
          predicted_bearing_right_deg,
          residual_deg,
          predicted_range_in,
          range_residual_in,
          face_x,
          face_y,
      });
    }
  }

  if (candidates.empty()) {
    pose.ai_reject = "no_map_candidate";
    return;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.residual_deg < b.residual_deg; });
  const auto& best = candidates.front();
  // Adjacent faces on one physical Goal may be nearly tied. That is face
  // uncertainty, not ambiguity between two duplicate-ID field landmarks.
  const auto different_goal = std::find_if(
      candidates.begin() + 1, candidates.end(),
      [&best](const auto& candidate) {
        return std::strcmp(candidate.goal, best.goal) != 0;
      });
  const double margin = different_goal != candidates.end()
                            ? different_goal->residual_deg - best.residual_deg
                            : INFINITY;
  pose.ai_goal = best.goal;
  pose.ai_face = best.face;
  pose.ai_candidate_residual_deg = best.residual_deg;
  pose.ai_candidate_margin_deg = margin;
  pose.ai_predicted_range_in = best.predicted_range_in;
  pose.ai_range_residual_in = best.range_residual_in;
  if (best.residual_deg > localization::kAiMaxCandidateBearingResidualDeg) {
    pose.ai_reject = "bearing_residual";
  } else if (!std::isfinite(observation.range_estimate_in) ||
             observation.range_estimate_in < localization::kAiMinUsableRangeIn ||
             observation.range_estimate_in > localization::kAiMaxUsableRangeIn) {
    pose.ai_reject = "range_invalid";
  } else if (best.range_residual_in >
             localization::kAiMaxCandidateRangeResidualIn) {
    pose.ai_reject = "range_residual";
  } else if (margin < localization::kAiMinCandidateWinnerMarginDeg) {
    pose.ai_reject = "ambiguous";
  } else if (!localization::kAiVisionPoseCorrectionEnabled) {
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
        observation.range_estimate_in * std::cos(observed_global_bearing_rad);
    const double measured_camera_y = best.face_y -
        observation.range_estimate_in * std::sin(observed_global_bearing_rad);
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

double heading_rad_from_deg(double deg) {
  return deg_to_rad(normalize_deg(deg));
}

double read_field_imu_heading_deg(const PoseEstimate& pose) {
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
  return MotorSideReading{
      count > 0 ? sum / count : NAN,
      spread,
      count,
      count > 0 && spread <= kMaxSameSideMotorSpreadDeg,
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

void stop_drive() {
  chassis.drive_set(0, 0);
}

void stop_drive_motors() {
  chassis.pid_targets_reset();
  for (auto& motor : chassis.left_motors) {
    motor.move(0);
  }
  for (auto& motor : chassis.right_motors) {
    motor.move(0);
  }
  stop_drive();
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

void update_pose(PoseEstimate& pose, LidarFusionMode lidar_mode = LidarFusionMode::kBiasOnly) {
  const MotorSideReading left = read_motor_side(chassis.left_motors);
  const MotorSideReading right = read_motor_side(chassis.right_motors);
  std::int32_t side_centideg = 0;
  const bool side_sample_valid = read_side_odom_position(side_centideg);
  const std::uint32_t now = pros::millis();
  pose.left_motor_count = left.valid_count;
  pose.right_motor_count = right.valid_count;
  pose.left_motor_spread_deg = left.spread_deg;
  pose.right_motor_spread_deg = right.spread_deg;
  if (!left.trustworthy || !right.trustworthy) {
    update_ai_vision_shadow(pose, now);
    return;
  }

  const double dt_s = pose.last_update_ms > 0 ? static_cast<double>(now - pose.last_update_ms) / 1000.0 : 0.02;
  pose.last_update_ms = now;

  const double delta_left_in = ((left.position_deg - pose.left_deg) / 360.0) * kWheelCircumferenceIn * kLeftEncoderSign;
  const double delta_right_in = ((right.position_deg - pose.right_deg) / 360.0) * kWheelCircumferenceIn * kRightEncoderSign;
  double delta_side_wheel_in = 0.0;
  bool side_delta_accepted = false;
  pose.side_odom_reject = "none";
  if (!side_sample_valid) {
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
  const double delta_center_in = (delta_left_in + delta_right_in) / 2.0;
  const double field_imu_heading_deg = read_field_imu_heading_deg(pose);
  double delta_heading_rad = drivetrain_delta_heading_rad;
  double angular_rate_deg_s = std::fabs(rad_to_deg(drivetrain_delta_heading_rad)) / std::max(0.001, dt_s);
  if (std::isfinite(field_imu_heading_deg) && pose.imu_ready) {
    const double delta_heading_deg = signed_angle_diff_deg(field_imu_heading_deg, pose.imu_heading_deg);
    delta_heading_rad = deg_to_rad(delta_heading_deg);
    angular_rate_deg_s = std::fabs(delta_heading_deg) / std::max(0.001, dt_s);
    pose.imu_heading_deg = field_imu_heading_deg;
  } else if (std::isfinite(field_imu_heading_deg)) {
    pose.imu_ready = true;
    pose.imu_heading_deg = field_imu_heading_deg;
    delta_heading_rad = 0.0;
    angular_rate_deg_s = 0.0;
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
  if (std::isfinite(field_imu_heading_deg)) {
    pose.heading_rad = heading_rad_from_deg(field_imu_heading_deg + pose.imu_bias_deg);
  } else {
    pose.heading_rad += delta_heading_rad;
  }
  update_ai_vision_shadow(pose, now);

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
}

void log_pose(const char* phase, const PoseEstimate& pose) {
  printf(
      "FUSE_TEST phase=%s x=%.2f y=%.2f heading=%.2f imu=%.2f bias=%.2f lidar=%s reject=%s theta=%.2f distance=%.2f rmse=%.3f lidar_axis=%s axis_step=%.2f drive=L%d/R%d spread=%.1f/%.1f side=%s track=%.4f rear=%.4f lidar_scale=%.6f ai_id=%d ai_bearing=%.2f ai_range=%.2f ai_pred_range=%.2f ai_range_residual=%.2f ai_innovation=%.2f ai_pos_step=%.2f ai_heading_step=%.2f ai_goal=%s ai_face=%s ai_residual=%.2f ai_margin=%.2f ai_age=%lu ai_reject=%s\n",
      phase,
      pose.x,
      pose.y,
      normalize_deg(rad_to_deg(pose.heading_rad)),
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
      pose.ai_reject);
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

void set_physical_drive_power(int forward_power, int turn_power) {
  const int left_power = clamp_power(forward_power - turn_power);
  const int right_power = clamp_power(forward_power + turn_power);
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

  runtime_start_pose = {x_in, y_in, normalize_deg(heading_deg)};
  telemetry_pose_initialized = false;
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

bool fused_drive_to_point(PoseEstimate& pose,
                          const char* phase,
                          Waypoint target,
                          double final_heading_deg,
                          int max_forward_power,
                          std::uint32_t timeout_ms) {
  const std::uint32_t start = pros::millis();
  std::uint32_t last_loop_ms = start;
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;
  double forward_command = 0.0;
  double turn_command = 0.0;

  printf("FUSE_TEST command=%s type=fused_drive_to target_x=%.2f target_y=%.2f final_heading=%.2f max_power=%d\n",
         phase,
         target.x,
         target.y,
         normalize_deg(final_heading_deg),
         max_forward_power);
  fflush(stdout);

  while (pros::millis() - start < timeout_ms) {
    update_pose(pose, LidarFusionMode::kBiasOnly);
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);
    const double heading_deg = pose_heading_deg(pose);
    const double bearing_deg = normalize_deg(std::atan2(dy, dx) * 180.0 / kPi);
    const double bearing_error_deg = signed_angle_diff_deg(bearing_deg, heading_deg);
    const double final_heading_error_deg = signed_angle_diff_deg(final_heading_deg, heading_deg);

    if (distance <= kFusedDriveToleranceIn) {
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >= static_cast<std::uint32_t>(kFusedDriveSettleMs)) {
        stop_drive_motors();
        return true;
      }
    } else {
      settled_since = 0;
    }

    double target_forward = clamp(distance * kFusedDriveKp, 0.0, static_cast<double>(max_forward_power));
    if (distance > kFusedDriveToleranceIn * 1.5 && target_forward < kFusedDriveMinPower) {
      target_forward = kFusedDriveMinPower;
    }
    const double heading_scale = clamp(std::cos(deg_to_rad(clamp(std::fabs(bearing_error_deg), 0.0, 80.0))),
                                       0.25,
                                       1.0);
    target_forward *= heading_scale;
    if (std::fabs(bearing_error_deg) > 55.0) {
      target_forward = std::min(target_forward, 32.0);
    }

    const double drive_heading_error_deg =
        distance < 4.0 ? final_heading_error_deg : bearing_error_deg;
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

    set_physical_drive_power(clamp_power(forward_command), clamp_power(turn_command));

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf(
          "FUSE_TEST phase=%s controller=fused_drive dist=%.2f bearing=%.2f bearing_err=%.2f final_err=%.2f fwd=%.1f turn=%.1f x=%.2f y=%.2f h=%.2f lidar=%s reject=%s\n",
          phase,
          distance,
          bearing_deg,
          bearing_error_deg,
          final_heading_error_deg,
          forward_command,
          turn_command,
          pose.x,
          pose.y,
          heading_deg,
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

bool fused_turn_to_heading(PoseEstimate& pose,
                           const char* phase,
                           double target_heading_deg,
                           int max_turn_power,
                           std::uint32_t timeout_ms) {
  const std::uint32_t start = pros::millis();
  std::uint32_t last_loop_ms = start;
  std::uint32_t settled_since = 0;
  std::uint32_t last_log = 0;
  double turn_command = 0.0;
  double last_error_deg = signed_angle_diff_deg(target_heading_deg, pose_heading_deg(pose));
  double last_motion_heading_deg = pose_heading_deg(pose);
  std::uint32_t last_motion_ms = start;

  printf("FUSE_TEST command=%s type=fused_turn_to target_heading=%.2f max_power=%d\n",
         phase,
         normalize_deg(target_heading_deg),
         max_turn_power);
  fflush(stdout);

  while (pros::millis() - start < timeout_ms) {
    update_pose(pose, LidarFusionMode::kBiasOnly);
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;

    const double heading_deg = pose_heading_deg(pose);
    const double error_deg = signed_angle_diff_deg(target_heading_deg, heading_deg);
    const double error_rate_deg_s = signed_angle_diff_deg(error_deg, last_error_deg) / dt_s;
    last_error_deg = error_deg;

    if (std::fabs(signed_angle_diff_deg(heading_deg,
                                        last_motion_heading_deg)) >= 0.20) {
      last_motion_heading_deg = heading_deg;
      last_motion_ms = now;
    } else if (std::fabs(error_deg) > kFusedTurnToleranceDeg &&
               now - last_motion_ms >= kFusedTurnStallTimeoutMs) {
      printf("FUSE_TEST phase=%s abort=turn_stall error=%.2f heading=%.2f\n",
             phase, error_deg, heading_deg);
      fflush(stdout);
      stop_drive_motors();
      return false;
    }

    if (std::fabs(error_deg) <= kFusedTurnToleranceDeg && std::fabs(error_rate_deg_s) < 35.0) {
      if (settled_since == 0) {
        settled_since = now;
      } else if (now - settled_since >= static_cast<std::uint32_t>(kFusedTurnSettleMs)) {
        stop_drive_motors();
        return true;
      }
    } else {
      settled_since = 0;
    }

    double target_turn = kFusedTurnKp * error_deg + kFusedTurnKd * error_rate_deg_s;
    target_turn = clamp(target_turn, -static_cast<double>(max_turn_power), static_cast<double>(max_turn_power));
    if (std::fabs(error_deg) > kFusedTurnMinPowerErrorDeg &&
        std::fabs(target_turn) < kFusedTurnMinPower) {
      target_turn = target_turn >= 0.0 ? kFusedTurnMinPower : -kFusedTurnMinPower;
    }
    turn_command = apply_slew(target_turn, turn_command, kFusedTurnSlewPowerPerSec * dt_s);
    set_physical_drive_power(0, clamp_power(turn_command));

    if (now - last_log >= kFusionTestLogPeriodMs) {
      printf(
          "FUSE_TEST phase=%s controller=fused_turn target_heading=%.2f error=%.2f rate=%.2f turn=%.1f x=%.2f y=%.2f h=%.2f lidar=%s reject=%s\n",
          phase,
          normalize_deg(target_heading_deg),
          error_deg,
          error_rate_deg_s,
          turn_command,
          pose.x,
          pose.y,
          heading_deg,
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
      stop_drive();
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

  stop_drive();
  return false;
}
}  // namespace

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

  stop_drive();
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
  telemetry_last_log_ms = 0;

  pros::lcd::print(6,
                   "Long %d e%.1f h%.1f",
                   static_cast<int>(route_ok),
                   return_error_in,
                   return_heading_error_deg,
                   pose.x,
                   pose.y);
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
