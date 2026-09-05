#include "main.h"

#include "cascade_lift.hpp"
#include "gps_frame.hpp"
#include "localization_config.hpp"
#include "path2_auton_config.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

namespace {

using Path2Point = path2_config::Point;
constexpr double kPath2Pi = 3.14159265358979323846;
double path2_imu_anchor_cw_deg = 0.0;
double path2_field_anchor_heading_deg = 0.0;
bool path2_imu_anchor_valid = false;
double path2_gps_heading_rotation_deg = 0.0;
bool path2_gps_heading_anchor_valid = false;
std::uint32_t path2_prefer_gps_until_ms = 0;

double path2_field_heading_from_imu(double imu_cw_deg) {
  if (!path2_imu_anchor_valid || !std::isfinite(imu_cw_deg)) return NAN;
  return std::remainder(
      path2_field_anchor_heading_deg -
          (imu_cw_deg - path2_imu_anchor_cw_deg),
      360.0);
}

bool path2_field_heading_from_gps(double& heading_deg) {
  if (!path2_gps_heading_anchor_valid || !gps_7.is_installed()) return false;
  const auto position = gps_7.get_position();
  const double sensor_heading_cw_deg = gps_7.get_heading();
  const double error_in = gps_7.get_error() * 39.37007874015748;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(sensor_heading_cw_deg) || !std::isfinite(error_in) ||
      error_in < 0.0 ||
      error_in > localization::kGpsMaxReportedErrorIn) {
    return false;
  }
  const auto project_pose = localization::vex_gps_to_project_robot_pose(
      position.x, position.y, sensor_heading_cw_deg);
  heading_deg = std::remainder(
      project_pose.heading_deg + path2_gps_heading_rotation_deg, 360.0);
  return std::isfinite(heading_deg);
}

void path2_blank_impact_imu(unsigned duration_ms) {
  path2_prefer_gps_until_ms = pros::millis() + 5000;
  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < duration_ms) {
    navigation::update();
    pros::delay(20);
  }
}

struct Path2FastDriveResult {
  bool reached{false};
  bool disabled{false};
  double traveled_in{0.0};
};

double path2_average_drive_position_deg() {
  return 0.25 * (chassis.left_motors[0].get_position() +
                 chassis.left_motors[1].get_position() +
                 chassis.right_motors[0].get_position() +
                 chassis.right_motors[1].get_position());
}

void path2_brake_drive() {
  chassis.drive_set(0, 0);
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motor.brake();
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motor.brake();
  }
}

// Phase 1 uses a deliberately small encoder-chain controller. The general
// fused navigator waits for a precision settle and reports a short timeout as
// failure; that behavior is correct for long field paths but used to abort
// this entire sequence between six-inch moves. This controller exits as soon
// as the encoder finish plane is crossed and reports actual travel so the
// Toggle backout can return exactly as far as the ram moved.
Path2FastDriveResult path2_fast_drive(double distance_in, int full_power,
                                      unsigned timeout_ms,
                                      bool contact_is_finish = false,
                                      bool progressive_goal_decel = false) {
  Path2FastDriveResult result;
  if (!std::isfinite(distance_in) || std::fabs(distance_in) < 0.05 ||
      full_power <= 0) {
    result.reached = true;
    return result;
  }
  constexpr double kStallProgressIn = 0.06;
  constexpr unsigned kStallMs = 260;
  constexpr double kBrakeZoneIn = 2.0;
  const double circumference =
      localization::kDriveWheelDiameterIn * kPath2Pi;
  const double baseline_deg = path2_average_drive_position_deg();
  const double target_in = std::fabs(distance_in);
  const int direction = distance_in >= 0.0 ? 1 : -1;
  const std::uint32_t started = pros::millis();
  std::uint32_t last_progress_ms = started;
  double last_progress_in = 0.0;

  chassis.drive_mode_set(ez::DISABLE, true);

  while (pros::millis() - started < timeout_ms) {
    if (pros::competition::is_connected() &&
        pros::competition::is_disabled()) {
      result.disabled = true;
      break;
    }
    result.traveled_in = std::fabs(
        path2_average_drive_position_deg() - baseline_deg) /
        360.0 * circumference;
    const double remaining_in = target_in - result.traveled_in;
    if (remaining_in <= 0.0) {
      result.reached = true;
      break;
    }
    if (result.traveled_in >= last_progress_in + kStallProgressIn) {
      last_progress_in = result.traveled_in;
      last_progress_ms = pros::millis();
    } else if (pros::millis() - last_progress_ms >= kStallMs) {
      result.reached = contact_is_finish;
      break;
    }
    int power = remaining_in <= kBrakeZoneIn
                    ? std::min(full_power, 50)
                    : full_power;
    if (progressive_goal_decel) {
      // Goal approaches retain useful transit speed, then taper over the last
      // foot so the rear mechanism seats instead of slamming into the Goal.
      constexpr double kGoalDecelZoneIn = 12.0;
      constexpr int kGoalContactPower = 28;
      if (remaining_in <= kGoalDecelZoneIn) {
        const double fraction = std::clamp(
            remaining_in / kGoalDecelZoneIn, 0.0, 1.0);
        power = static_cast<int>(std::lround(
            kGoalContactPower + (full_power - kGoalContactPower) * fraction));
      }
    }
    chassis.drive_set(direction * power, direction * power);
    pros::delay(10);
  }
  path2_brake_drive();
  // Include the distance traveled while the drivetrain was physically
  // braking. The Toggle return uses this final value, not the last value from
  // before the brake command, so it cannot accumulate coast as start error.
  result.traveled_in = std::fabs(
      path2_average_drive_position_deg() - baseline_deg) /
      360.0 * circumference;
  navigation::update();
  std::printf(
      "PATH2_FAST_DRIVE target=%.2f traveled=%.2f reached=%d disabled=%d\n",
      distance_in, result.traveled_in, static_cast<int>(result.reached),
      static_cast<int>(result.disabled));
  std::fflush(stdout);
  return result;
}

bool path2_fast_turn(double heading_deg, bool relative = false,
                     double settle_tolerance_deg = 2.5,
                     unsigned timeout_ms = 2600,
                     unsigned settle_ms = 40) {
  // Execute the requested global delta against the raw, continuous IMU
  // rotation. The fused pose establishes the field target once, but GPS/LiDAR
  // corrections cannot move the target underneath the controller mid-turn.
  // This also avoids a strict public-API timeout suppressing the rest of Path2.
  const double tolerance_deg =
      std::clamp(settle_tolerance_deg, 1.0, 8.0);
  constexpr double kTurnKp = 1.65;
  constexpr double kTurnKd = 0.055;
  constexpr double kRateFilter = 0.30;
  constexpr double kMinimumMovingPower = 30.0;
  constexpr double kBrakeZoneDeg = 15.0;
  constexpr double kBrakeZonePower = 52.0;

  navigation::update();
  const auto start_pose = navigation::current_pose();
  const double start_imu_cw_deg = chassis.drive_imu_get();
  if (!std::isfinite(start_imu_cw_deg)) return false;
  // A Goal impact can momentarily rattle the IMU. During the bounded blanking
  // window, use the wall-facing GPS for the absolute heading once, then
  // re-anchor the fast IMU rate signal to that heading for the actual turn.
  double gps_reanchor_heading_deg = NAN;
  if (pros::millis() < path2_prefer_gps_until_ms) {
    if (path2_field_heading_from_gps(gps_reanchor_heading_deg)) {
      path2_imu_anchor_cw_deg = start_imu_cw_deg;
      path2_field_anchor_heading_deg = gps_reanchor_heading_deg;
      path2_imu_anchor_valid = true;
      std::printf("PATH2_TURN heading_reanchor=gps heading=%.2f imu=%.2f\n",
                  gps_reanchor_heading_deg, start_imu_cw_deg);
      std::fflush(stdout);
    }
  }
  const double current_field_heading_deg =
      std::isfinite(gps_reanchor_heading_deg)
          ? gps_reanchor_heading_deg
          : (start_pose.valid
                 ? start_pose.heading_deg
                 : path2_field_heading_from_imu(start_imu_cw_deg));
  if (!std::isfinite(current_field_heading_deg)) return false;
  if (!start_pose.valid) {
    std::printf("PATH2_TURN heading_source=imu_fallback heading=%.2f\n",
                current_field_heading_deg);
    std::fflush(stdout);
  }
  const double target_heading_deg = relative
      ? std::remainder(current_field_heading_deg + heading_deg, 360.0)
      : heading_deg;
  const double initial_field_error_deg = relative
      ? std::remainder(heading_deg, 360.0)
      : std::remainder(heading_deg - current_field_heading_deg, 360.0);
  const double target_imu_cw_deg =
      start_imu_cw_deg - initial_field_error_deg;
  double last_error_deg = initial_field_error_deg;
  double filtered_rate_deg_s = 0.0;
  std::uint32_t settled_since = 0;
  std::uint32_t last_loop_ms = pros::millis();
  const std::uint32_t started_ms = last_loop_ms;
  chassis.drive_mode_set(ez::DISABLE, true);

  while (pros::millis() - started_ms < timeout_ms) {
    if (pros::competition::is_connected() &&
        pros::competition::is_disabled()) {
      path2_brake_drive();
      return false;
    }
    const double imu_cw_deg = chassis.drive_imu_get();
    if (!std::isfinite(imu_cw_deg)) {
      path2_brake_drive();
      return false;
    }
    const std::uint32_t now = pros::millis();
    const double dt_s = std::max(
        0.001, static_cast<double>(now - last_loop_ms) / 1000.0);
    last_loop_ms = now;
    // Raw IMU rotation is clockwise-positive; field heading and drivetrain
    // turn power are counterclockwise-positive.
    const double error_deg = -(target_imu_cw_deg - imu_cw_deg);
    const double raw_rate_deg_s = (error_deg - last_error_deg) / dt_s;
    filtered_rate_deg_s +=
        kRateFilter * (raw_rate_deg_s - filtered_rate_deg_s);
    const bool crossed_target = error_deg * last_error_deg < 0.0;
    last_error_deg = error_deg;

    if (std::fabs(error_deg) <= tolerance_deg) {
      path2_brake_drive();
      if (settled_since == 0) settled_since = now;
      if (now - settled_since >= settle_ms) {
        navigation::update();
        std::printf(
            "PATH2_TURN result=success target=%.2f raw_error=%.2f elapsed=%lu\n",
            target_heading_deg, error_deg,
            static_cast<unsigned long>(now - started_ms));
        std::fflush(stdout);
        return true;
      }
      pros::delay(10);
      continue;
    }
    settled_since = 0;
    if (crossed_target) {
      path2_brake_drive();
      filtered_rate_deg_s = 0.0;
      pros::delay(15);
      continue;
    }

    double command =
        kTurnKp * error_deg + kTurnKd * filtered_rate_deg_s;
    const double active_limit = std::fabs(error_deg) <= kBrakeZoneDeg
        ? std::min(static_cast<double>(path2_config::kTurnPower),
                   kBrakeZonePower)
        : static_cast<double>(path2_config::kTurnPower);
    command = std::clamp(command, -active_limit, active_limit);
    if (std::fabs(command) < kMinimumMovingPower) {
      command = error_deg > 0.0 ? kMinimumMovingPower
                                : -kMinimumMovingPower;
    }
    const int turn_power = static_cast<int>(std::lround(command));
    chassis.drive_set(turn_power, -turn_power);
    pros::delay(10);
  }

  path2_brake_drive();
  navigation::update();
  const auto final_pose = navigation::current_pose();
  const double final_imu_cw_deg = chassis.drive_imu_get();
  const double final_heading_deg = final_pose.valid
      ? final_pose.heading_deg
      : path2_field_heading_from_imu(final_imu_cw_deg);
  const double final_error_deg = std::isfinite(final_heading_deg)
      ? std::fabs(std::remainder(
            target_heading_deg - final_heading_deg, 360.0))
      : std::numeric_limits<double>::infinity();
  std::printf(
      "PATH2_TURN result=timeout target=%.2f heading=%.2f error=%.2f\n",
      target_heading_deg, final_heading_deg, final_error_deg);
  std::fflush(stdout);
  return final_error_deg <= tolerance_deg + 1.5;
}

// A transient turn settle timeout must not silently discard every later phase.
// Retry against the absolute target so the second attempt corrects only the
// remaining error (rather than applying the relative delta twice). If the IMU
// and fused pose remain healthy and the retry ends reasonably close, chain on.
bool path2_chain_turn(double heading_deg, bool relative = false,
                      double settle_tolerance_deg = 4.0) {
  navigation::update();
  const auto start_pose = navigation::current_pose();
  const double start_imu_cw_deg = chassis.drive_imu_get();
  const double current_field_heading_deg = start_pose.valid
      ? start_pose.heading_deg
      : path2_field_heading_from_imu(start_imu_cw_deg);
  if (!std::isfinite(current_field_heading_deg)) return false;
  const double absolute_target_deg = relative
      ? std::remainder(current_field_heading_deg + heading_deg, 360.0)
      : heading_deg;
  if (path2_fast_turn(heading_deg, relative, settle_tolerance_deg)) {
    return true;
  }
  if (pros::competition::is_connected() &&
      pros::competition::is_disabled()) {
    return false;
  }
  std::printf("PATH2_CHAIN retry_turn target=%.2f\n", absolute_target_deg);
  std::fflush(stdout);
  if (path2_fast_turn(absolute_target_deg, false, 5.0)) return true;
  navigation::update();
  const auto final_pose = navigation::current_pose();
  const double imu_deg = chassis.drive_imu_get();
  const double final_heading_deg = final_pose.valid
      ? final_pose.heading_deg
      : path2_field_heading_from_imu(imu_deg);
  if (!std::isfinite(final_heading_deg)) return false;
  const double final_error_deg = std::fabs(std::remainder(
      absolute_target_deg - final_heading_deg, 360.0));
  const bool safe_to_chain = final_error_deg <= 10.0;
  std::printf(
      "PATH2_CHAIN turn_retry_end target=%.2f heading=%.2f error=%.2f "
      "continue=%d\n",
      absolute_target_deg, final_heading_deg, final_error_deg,
      static_cast<int>(safe_to_chain));
  std::fflush(stdout);
  return safe_to_chain;
}

double path2_normalize_deg(double value) {
  while (value >= 360.0) value -= 360.0;
  while (value < 0.0) value += 360.0;
  return value;
}

double path2_front_heading(const navigation::Pose& pose, Path2Point target) {
  return path2_normalize_deg(
      std::atan2(target.y - pose.y_in, target.x - pose.x_in) *
      180.0 / kPath2Pi);
}

double path2_rear_heading(const navigation::Pose& pose, Path2Point target) {
  return path2_normalize_deg(path2_front_heading(pose, target) + 180.0);
}

bool path2_nav_ok(navigation::Result result, const char* step) {
  if (result == navigation::Result::kSuccess) return true;
  std::printf("PATH2 abort step=%s result=%s\n",
              step, navigation::result_name(result));
  std::fflush(stdout);
  navigation::stop();
  return false;
}

void path2_stop_all() {
  navigation::stop();
}

bool path2_config_ready() {
  using namespace path2_config;
  bool ready = true;
  const auto need_double = [&](const char* name, double value) {
    if (std::isfinite(value)) return;
    std::printf("PATH2_CONFIG missing=%s\n", name);
    ready = false;
  };
  const auto need_power = [&](const char* name, int value) {
    if (value != 0) return;
    std::printf("PATH2_CONFIG missing=%s\n", name);
    ready = false;
  };
  const auto need_time = [&](const char* name, unsigned value) {
    if (value != 0) return;
    std::printf("PATH2_CONFIG missing=%s\n", name);
    ready = false;
  };

  need_double("start_heading_deg", kStartHeadingDeg);
  need_double("blue_start_heading_deg", kBlueStartHeadingDeg);
  need_double("blue_start_x", kBlueStart.x);
  need_double("blue_start_y", kBlueStart.y);
  need_double("blue_goal_x", kBlueGoal.x);
  need_double("blue_goal_y", kBlueGoal.y);
  need_double("blue_stack_a_x", kBlueStackA.x);
  need_double("blue_stack_a_y", kBlueStackA.y);
  need_double("blue_stack_b_x", kBlueStackB.x);
  need_double("blue_stack_b_y", kBlueStackB.y);
  need_double("start_position_error_in", kStartPositionErrorIn);
  need_double("toggle_signed_distance_in", kToggleSignedDistanceIn);
  need_double("toggle_return_distance_in", kToggleReturnDistanceIn);
  need_power("toggle_power", kTogglePower);
  need_power("toggle_return_power", kToggleReturnPower);
  need_double("goal_staging_x", kGoalStaging.x);
  need_double("goal_staging_y", kGoalStaging.y);
  need_double("goal_dock_reverse_in", kGoalDockReverseIn);
  need_power("goal_dock_power", kGoalDockPower);
  need_power("goal_alignment_bump_power", kGoalAlignmentBumpPower);
  need_time("goal_alignment_bump_ms", kGoalAlignmentBumpMs);
  need_double("goal_contact_exit_in", kGoalContactExitIn);
  need_double("stack_fast_fraction", kStackFastFraction);
  need_double("rear_pickup_reach_in", kRearPickupReachIn);
  need_power("fast_drive_power", kFastDrivePower);
  need_power("stack_approach_power", kStackApproachPower);
  need_power("slow_stack_power", kSlowStackPower);
  need_power("second_stack_approach_power", kSecondStackApproachPower);
  need_power("second_stack_slow_power", kSecondStackSlowPower);
  need_power("turn_power", kTurnPower);
  if (kIntakeUpperPower == 0 && kIntakeCounterPower == 0) {
    std::printf("PATH2_CONFIG missing=intake_motor_powers\n");
    ready = false;
  }
  if (kOuttakeUpperPower == 0 && kOuttakeCounterPower == 0) {
    std::printf("PATH2_CONFIG missing=outtake_motor_powers\n");
    ready = false;
  }
  need_time("preload_deposit_ms", kPreloadDepositMs);
  need_double("stage_1_extra_capture_in", kStage1ExtraCaptureIn);
  need_power("stage_1_extra_capture_power", kStage1ExtraCapturePower);
  need_double("stage_1_goal_drive_in", kStage1GoalDriveIn);
  need_power("stage_1_goal_drive_power", kStage1GoalDrivePower);
  need_power("score_drop_power", kScoreDropPower);
  need_time("score_drop_pulse_ms", kScoreDropPulseMs);
  need_time("score_drop_wait_timeout_ms", kScoreDropWaitTimeoutMs);
  need_time("stage_1_outtake_lead_ms", kStage1OuttakeLeadMs);
  need_double("stage_1_score_retreat_in", kStage1ScoreRetreatIn);
  need_power("stage_1_score_retreat_power", kStage1ScoreRetreatPower);
  need_time("stage_1_home_wait_ms", kStage1HomeWaitMs);
  need_double("loaded_turn_clearance_deg", kLoadedTurnClearanceDeg);
  need_time("loaded_turn_clearance_timeout_ms",
            kLoadedTurnClearanceTimeoutMs);
  need_time("score_stage_ready_timeout_ms", kScoreStageReadyTimeoutMs);
  need_double("stage_2_extra_capture_in", kStage2ExtraCaptureIn);
  need_power("stage_2_extra_capture_power", kStage2ExtraCapturePower);
  need_double("stage_2_goal_drive_in", kStage2GoalDriveIn);
  need_power("stage_2_goal_drive_power", kStage2GoalDrivePower);
  need_double("stage_2_score_retreat_in", kStage2ScoreRetreatIn);
  need_power("stage_2_score_retreat_power", kStage2ScoreRetreatPower);
  need_double("lift_ready_tolerance_deg", kLiftReadyToleranceDeg);
  need_time("lift_ready_timeout_ms", kLiftReadyTimeoutMs);
  if (kSafeFinalLiftStage < 0 || kSafeFinalLiftStage > 5) {
    std::printf("PATH2_CONFIG missing=safe_final_lift_stage\n");
    ready = false;
  }
  if (kStartPositionErrorIn < 0.0 || kStartPositionErrorIn > 3.0 ||
      kStackFastFraction <= 0.0 || kStackFastFraction >= 1.0 ||
      kRearPickupReachIn <= 0.0 || kRearPickupReachIn > 18.0 ||
      kGoalAlignmentBumpMs > 1000 ||
      kFirstCupLiftStage < 1 || kFirstCupLiftStage > 5 ||
      kSecondCupLiftStage < 1 || kSecondCupLiftStage > 5 ||
      kLoadedTurnClearanceDeg < 0.0 ||
      kLoadedTurnClearanceDeg >= cascade_lift::stage_position_deg(
          kFirstCupLiftStage) ||
      kTestStopAfterPhase < 0 || kTestStopAfterPhase > 4) {
    std::printf("PATH2_CONFIG invalid=range_check\n");
    ready = false;
  }
  const int configured_powers[] = {
      kTogglePower, kGoalDockPower, kGoalAlignmentBumpPower,
      kToggleReturnPower,
      kFastDrivePower, kStackApproachPower, kSlowStackPower,
      kSecondStackApproachPower, kSecondStackSlowPower, kTurnPower,
      kStage1GoalDrivePower, kStage1ScoreRetreatPower,
      kStage1ExtraCapturePower,
      kStage2ExtraCapturePower, kStage2GoalDrivePower,
      kStage2ScoreRetreatPower,
      kIntakeUpperPower, kIntakeCounterPower,
      kOuttakeUpperPower, kOuttakeCounterPower,
      kPreloadPinUpperPower, kPreloadPinCounterPower};
  for (const int power : configured_powers) {
    if (power < -127 || power > 127) {
      std::printf("PATH2_CONFIG invalid=motor_power value=%d\n", power);
      ready = false;
    }
  }
  std::fflush(stdout);
  return ready;
}

class Path2LiftService {
 public:
  Path2LiftService()
      : task_([this] {
          std::uint32_t full_down_started_ms = 0;
          std::uint32_t score_drop_started_ms = 0;
          while (active_.load(std::memory_order_acquire)) {
            if (score_drop_requested_.load(std::memory_order_acquire)) {
              if (!score_drop_active_.load(std::memory_order_acquire)) {
                score_drop_started_ms = pros::millis();
                score_drop_active_.store(true, std::memory_order_release);
              }
              if (pros::millis() - score_drop_started_ms >=
                  score_drop_duration_ms_.load(std::memory_order_acquire)) {
                score_drop_requested_.store(false, std::memory_order_release);
                continue;
              }
              rest_lock_.store(false, std::memory_order_release);
              cascade_lift::set_manual_power(
                  -score_drop_power_.load(std::memory_order_acquire));
              cascade_lift::update();
              position_deg_.store(cascade_lift::snapshot().position_deg,
                                  std::memory_order_release);
              pros::delay(10);
              continue;
            }
            if (score_drop_active_.exchange(false,
                                            std::memory_order_acq_rel)) {
              cascade_lift::set_manual_power(0);
              cascade_lift::update();
              slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_right.brake();
              slider_left.brake();
              score_drop_completed_.store(true, std::memory_order_release);
            }
            if (full_down_requested_.load(std::memory_order_acquire)) {
              if (!full_down_active_.load(std::memory_order_acquire)) {
                full_down_started_ms = pros::millis();
                full_down_active_.store(true, std::memory_order_release);
              }
              if (pros::millis() - full_down_started_ms >=
                  full_down_duration_ms_.load(std::memory_order_acquire)) {
                full_down_requested_.store(false, std::memory_order_release);
                continue;
              }
              rest_lock_.store(false, std::memory_order_release);
              cascade_lift::set_manual_power(-127);
              cascade_lift::update();
              position_deg_.store(cascade_lift::snapshot().position_deg,
                                  std::memory_order_release);
              pros::delay(10);
              continue;
            }
            if (full_down_active_.exchange(false,
                                           std::memory_order_acq_rel)) {
              cascade_lift::set_manual_power(0);
              // The full-power pulse has seated the cascade at its mechanical
              // bottom. Re-zero port 16 and all PID state here so subsequent
              // stage targets are measured from a fresh physical Stage 0.
              cascade_lift::initialize_at_rest();
              rest_lock_.store(true, std::memory_order_release);
              active_stage_.store(0, std::memory_order_release);
              accepted_.store(true, std::memory_order_release);
              slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_right.brake();
              slider_left.brake();
              full_down_completed_.store(true, std::memory_order_release);
            }
            const double position_request = requested_position_deg_.exchange(
                std::numeric_limits<double>::quiet_NaN(),
                std::memory_order_acq_rel);
            const int request = requested_stage_.exchange(
                -1, std::memory_order_acq_rel);
            if (std::isfinite(position_request)) {
              rest_lock_.store(false, std::memory_order_release);
              accepted_.store(
                  cascade_lift::set_target_position_deg(position_request),
                  std::memory_order_release);
              active_stage_.store(-2, std::memory_order_release);
              target_position_deg_.store(position_request,
                                         std::memory_order_release);
            } else if (request == 0) {
              rest_lock_.store(true, std::memory_order_release);
              cascade_lift::disable_pid();
              accepted_.store(true, std::memory_order_release);
              active_stage_.store(0, std::memory_order_release);
              const auto stopped = cascade_lift::snapshot();
              std::printf(
                  "PATH2_LIFT command=rest_lock position=%.1f target=%.1f\n",
                  stopped.position_deg, stopped.target_deg);
              std::fflush(stdout);
            } else if (request > 0) {
              rest_lock_.store(false, std::memory_order_release);
              accepted_.store(cascade_lift::set_target_stage(request),
                              std::memory_order_release);
              active_stage_.store(request, std::memory_order_release);
              std::printf("PATH2_LIFT command=stage_%d position=%.1f\n",
                          request, cascade_lift::snapshot().position_deg);
              std::fflush(stdout);
            }
            // Stage 0 is a physical HOLD, not a PID target. A boot-time sensor
            // offset must never turn a nominal zero-degree error into upward
            // motor voltage. Until a captured stack explicitly requests a
            // scoring stage, force PID/manual output off on every cycle.
            if (rest_lock_.load(std::memory_order_acquire)) {
              cascade_lift::disable_pid();
              slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
              slider_right.brake();
              slider_left.brake();
            }
            cascade_lift::update();
            const auto lift = cascade_lift::snapshot();
            position_deg_.store(lift.position_deg,
                                std::memory_order_release);
            faulted_.store(lift.faulted || !lift.sensor_ok,
                           std::memory_order_release);
            error_deg_.store(lift.error_deg, std::memory_order_release);
            pros::delay(20);
          }
          cascade_lift::disable_pid();
        }, "path2 lift") {}

  ~Path2LiftService() {
    active_.store(false, std::memory_order_release);
    task_.join();
    // Preserve the last verified height mechanically after this route gives up
    // ownership of the cascade controller.
    slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_right.brake();
    slider_left.brake();
  }

  void request(int stage) {
    accepted_.store(false, std::memory_order_release);
    requested_stage_.store(stage, std::memory_order_release);
  }

  void request_position(double position_deg) {
    accepted_.store(false, std::memory_order_release);
    requested_position_deg_.store(position_deg, std::memory_order_release);
  }

  void start_full_down_for(unsigned duration_ms) {
    full_down_completed_.store(false, std::memory_order_release);
    full_down_duration_ms_.store(duration_ms, std::memory_order_release);
    full_down_requested_.store(true, std::memory_order_release);
  }

  void start_score_drop_for(int power, unsigned duration_ms) {
    score_drop_completed_.store(false, std::memory_order_release);
    score_drop_power_.store(std::clamp(power, 1, 127),
                            std::memory_order_release);
    score_drop_duration_ms_.store(duration_ms, std::memory_order_release);
    score_drop_requested_.store(true, std::memory_order_release);
  }

  bool wait_score_drop_complete(unsigned timeout_ms) {
    const std::uint32_t start_wait_ms = pros::millis();
    while (pros::millis() - start_wait_ms < timeout_ms) {
      if (score_drop_completed_.load(std::memory_order_acquire)) return true;
      pros::delay(5);
    }
    return false;
  }

  bool wait_full_down_complete(unsigned timeout_ms) {
    const std::uint32_t start_wait_ms = pros::millis();
    while (pros::millis() - start_wait_ms < timeout_ms) {
      if (full_down_completed_.load(std::memory_order_acquire)) {
        return true;
      }
      pros::delay(5);
    }
    return false;
  }

  bool wait_ready(int stage, double tolerance_deg, unsigned timeout_ms) {
    const std::uint32_t started = pros::millis();
    while (pros::millis() - started < timeout_ms) {
      if (faulted_.load(std::memory_order_acquire)) return false;
      if (active_stage_.load(std::memory_order_acquire) == stage &&
          accepted_.load(std::memory_order_acquire) &&
          std::fabs(error_deg_.load(std::memory_order_acquire)) <=
              tolerance_deg) {
        return true;
      }
      pros::delay(20);
    }
    return false;
  }

  bool wait_position(double position_deg, double tolerance_deg,
                     unsigned timeout_ms) {
    const std::uint32_t started = pros::millis();
    while (pros::millis() - started < timeout_ms) {
      if (faulted_.load(std::memory_order_acquire)) return false;
      if (active_stage_.load(std::memory_order_acquire) == -2 &&
          accepted_.load(std::memory_order_acquire) &&
          std::fabs(target_position_deg_.load(std::memory_order_acquire) -
                    position_deg) <= 0.1 &&
          std::fabs(error_deg_.load(std::memory_order_acquire)) <=
              tolerance_deg) {
        return true;
      }
      pros::delay(20);
    }
    return false;
  }

  bool wait_at_least(double position_deg, unsigned timeout_ms) {
    const std::uint32_t started = pros::millis();
    while (pros::millis() - started < timeout_ms) {
      if (faulted_.load(std::memory_order_acquire)) return false;
      if (position_deg_.load(std::memory_order_acquire) >= position_deg) {
        return true;
      }
      pros::delay(10);
    }
    return position_deg_.load(std::memory_order_acquire) >= position_deg;
  }

 private:
  std::atomic<bool> active_{true};
  // Stage 0 is the autonomous startup state, not an idle/no-request state.
  std::atomic<int> requested_stage_{0};
  std::atomic<bool> rest_lock_{true};
  std::atomic<bool> full_down_requested_{false};
  std::atomic<bool> full_down_active_{false};
  std::atomic<bool> full_down_completed_{false};
  std::atomic<unsigned> full_down_duration_ms_{0};
  std::atomic<bool> score_drop_requested_{false};
  std::atomic<bool> score_drop_active_{false};
  std::atomic<bool> score_drop_completed_{false};
  std::atomic<int> score_drop_power_{0};
  std::atomic<unsigned> score_drop_duration_ms_{0};
  std::atomic<double> requested_position_deg_{
      std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> target_position_deg_{0.0};
  std::atomic<double> position_deg_{0.0};
  std::atomic<int> active_stage_{-1};
  std::atomic<bool> accepted_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<double> error_deg_{0.0};
  pros::Task task_;
};

bool path2_turn_rear_to(Path2Point target, const char* step) {
  navigation::update();
  const auto pose = navigation::current_pose();
  return pose.valid && path2_nav_ok(
      navigation::turn_to(path2_rear_heading(pose, target),
                          path2_config::kTurnPower, 5000,
                          true, true),
      step);
}

bool path2_goal_dock(const char* step) {
  using namespace path2_config;
  navigation::update();
  auto pose = navigation::current_pose();
  if (!pose.valid) return false;
  const double heading = path2_rear_heading(pose, kGoal);
  if (!path2_nav_ok(navigation::go_to_pose(
          kGoalStaging.x, kGoalStaging.y, heading,
          kFastDrivePower, 9000, true, true, true), step)) {
    return false;
  }
  if (!path2_turn_rear_to(kGoal, "rear toward goal")) return false;
  if (!path2_nav_ok(navigation::drive_relative(
          -std::fabs(kGoalDockReverseIn), kGoalDockPower, 4000,
          true, true, true), "goal dock reverse")) {
    return false;
  }
  const std::uint32_t bump_started = pros::millis();
  while (pros::millis() - bump_started < kGoalAlignmentBumpMs) {
    if (pros::competition::is_connected() &&
        pros::competition::is_disabled()) {
      chassis.drive_set(0, 0);
      return false;
    }
    chassis.drive_set(kGoalAlignmentBumpPower, kGoalAlignmentBumpPower);
    pros::delay(10);
  }
  chassis.drive_set(0, 0);
  return true;
}

bool path2_approach_stack(Path2Point target, bool reverse,
                          const char* step) {
  using namespace path2_config;
  navigation::update();
  auto pose = navigation::current_pose();
  if (!pose.valid) return false;
  const double dx = target.x - pose.x_in;
  const double dy = target.y - pose.y_in;
  const double distance = std::hypot(dx, dy);
  const double pickup_reach = reverse ? kRearPickupReachIn : 0.0;
  if (distance <= pickup_reach + 1.0) return false;
  const Path2Point robot_center_target{
      target.x - dx / distance * pickup_reach,
      target.y - dy / distance * pickup_reach};
  const double center_dx = robot_center_target.x - pose.x_in;
  const double center_dy = robot_center_target.y - pose.y_in;
  const double scale = kStackFastFraction;
  const Path2Point pre{pose.x_in + center_dx * scale,
                       pose.y_in + center_dy * scale};
  const double heading = reverse ? path2_rear_heading(pose, target)
                                 : path2_front_heading(pose, target);
  const auto curve_result = navigation::go_to_pose(
      pre.x, pre.y, heading, kStackApproachPower, 9000,
      reverse, true, true);
  if (curve_result != navigation::Result::kSuccess) {
    // Leaving the preload Goal begins inside its mapped exclusion. Goal/wall
    // proximity is explicitly allowed above. A timeout after reaching the
    // halfway point is also recoverable; the dedicated heading/capture commands
    // below finish the maneuver instead of cancelling the entire autonomous.
    navigation::update();
    const auto recovered_pose = navigation::current_pose();
    const double pre_error = recovered_pose.valid
        ? std::hypot(pre.x - recovered_pose.x_in,
                     pre.y - recovered_pose.y_in)
        : std::numeric_limits<double>::infinity();
    const bool recoverable = recovered_pose.valid &&
        (curve_result == navigation::Result::kDriveFailed ||
         curve_result == navigation::Result::kTurnFailed);
    std::printf(
        "PATH2 curve step=%s result=%s pre_error=%.2f recover=%d\n",
        step, navigation::result_name(curve_result), pre_error,
        static_cast<int>(recoverable));
    std::fflush(stdout);
    if (!recoverable) return false;
  }
  navigation::update();
  pose = navigation::current_pose();
  if (!pose.valid) return false;
  // Preserve the boomerang's arrival tangent into the first stack. A separate
  // final heading correction was turning the rear claw away immediately before
  // contact and causing an otherwise aligned approach to miss.
  navigation::update();
  pose = navigation::current_pose();
  if (!pose.valid) return false;
  const double final_distance = std::hypot(
      robot_center_target.x - pose.x_in,
      robot_center_target.y - pose.y_in);
  std::printf(
      "PATH2 pickup object=%.2f,%.2f center_target=%.2f,%.2f "
      "rear_reach=%.2f remaining=%.2f\n",
      target.x, target.y, robot_center_target.x, robot_center_target.y,
      pickup_reach, final_distance);
  std::fflush(stdout);
  // Close the longer pneumatic claw 9.7 inches before its calibrated contact
  // point, then finish the approach while it is already closing around the
  // stack. This avoids driving past the stack before the piston can capture it.
  constexpr double kPneumaticGrabLeadIn = 9.7;
  const double before_grab =
      std::max(0.0, final_distance - kPneumaticGrabLeadIn);
  if (before_grab > 0.05) {
    const auto approach_result = navigation::drive_relative(
        reverse ? -before_grab : before_grab,
        kSlowStackPower, 5000, false, true, true);
    if (approach_result != navigation::Result::kSuccess &&
        approach_result != navigation::Result::kDriveFailed) {
      return path2_nav_ok(approach_result, "slow stack approach");
    }
  }
  set_claw_piston(false);
  pros::delay(100);
  const double final_capture =
      std::min(kPneumaticGrabLeadIn, final_distance);
  const auto capture_result = navigation::drive_relative(
      reverse ? -final_capture : final_capture,
      kSlowStackPower, 1800, false, true, true);
  if (capture_result == navigation::Result::kSuccess ||
      capture_result == navigation::Result::kDriveFailed) {
    pros::delay(50);
    return true;
  }
  return path2_nav_ok(capture_result, "final pneumatic stack capture");
}

void path2_score(unsigned dwell_ms) {
  // Extended releases the preload/stack; retracted is the holding state.
  set_claw_piston(true);
  pros::delay(dwell_ms);
}

bool path2_exit_preload_goal(double distance_in, const char* step) {
  set_claw_piston(true);
  const bool ok = path2_nav_ok(navigation::drive_relative(
      std::fabs(distance_in), path2_config::kFastDrivePower, 5000,
      true, true, true), step);
  return ok;
}

bool path2_score_cup_and_exit(Path2LiftService& lift, int stage,
                              double exit_distance_in, const char* step) {
  using namespace path2_config;
  const double lowered_target =
      cascade_lift::stage_position_deg(stage) - kScoreLoweringDeg;
  lift.request_position(lowered_target);
  if (!lift.wait_position(lowered_target, kLiftReadyToleranceDeg,
                          kLiftReadyTimeoutMs)) {
    std::printf("PATH2 abort=score_lowering stage=%d\n", stage);
    std::fflush(stdout);
    return false;
  }
  set_claw_piston(true);
  lift.request(0);
  const bool exited = path2_nav_ok(navigation::drive_relative(
      exit_distance_in, kFastDrivePower, 6000, true, true, true), step);
  return exited;
}

}  // namespace

bool localization_two_cup_auton(bool blue_side) {
  using namespace path2_config;
  const Path2Point route_start = blue_side ? kBlueStart : kStart;
  const Path2Point route_goal = blue_side ? kBlueGoal : kGoal;
  const Path2Point route_stack_a = blue_side ? kBlueStackA : kStackA;
  const Path2Point route_stack_b = blue_side ? kBlueStackB : kStackB;
  const double route_start_heading_deg =
      blue_side ? kBlueStartHeadingDeg : kStartHeadingDeg;
  // Reflection across field X=Y maps heading h to 90-h and reverses the sign
  // of relative turns. Distances and mechanism actions remain unchanged.
  // Two extra degrees move the reverse approach about 0.4 in away from the
  // observed left-side miss over the 12-in preload drive.
  const double first_goal_turn_delta_deg = blue_side ? 77.0 : -77.0;
  const double first_stack_score_heading_deg = blue_side ? 270.0 : 180.0;
  const double second_stack_pickup_heading_deg = 225.0;
  const double second_stack_score_heading_deg = blue_side ? 0.0 : 90.0;
  const double final_heading_deg = blue_side ? 270.0 : 180.0;
  const double phase2_goal_clearance_in = blue_side
      ? kBluePhase2GoalClearanceIn : kPhase2GoalClearanceIn;
  // Close another 0.25 in later than the previous tune on both alliances.
  const double second_stack_grab_lead_in = blue_side ? 4.95 : 4.20;
  default_constants();
  path2_stop_all();
  // Competition setup starts the lift physically at rest, and initialize()
  // establishes that state as zero. Do not block the entire route on a second
  // current-threshold homing pass before the first drivetrain command.
  cascade_lift::clear_fault();
  slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  slider_right.brake();
  slider_left.brake();
  if (!path2_config_ready()) {
    std::printf("PATH2 disabled=missing_physical_configuration\n");
    std::fflush(stdout);
    return false;
  }
  // run_selected_auton() owns the arm and continuously holds its recorded
  // normal position throughout this route.
  if (!navigation::init(route_start.x, route_start.y,
                        route_start_heading_deg, kStartPositionErrorIn)) {
    return false;
  }
  path2_imu_anchor_cw_deg = chassis.drive_imu_get();
  path2_field_anchor_heading_deg = route_start_heading_deg;
  path2_imu_anchor_valid = std::isfinite(path2_imu_anchor_cw_deg);
  const auto gps_start_position = gps_7.get_position();
  const double gps_start_heading_cw_deg = gps_7.get_heading();
  const double gps_start_error_in = gps_7.get_error() * 39.37007874015748;
  path2_gps_heading_anchor_valid = gps_7.is_installed() &&
      std::isfinite(gps_start_position.x) &&
      std::isfinite(gps_start_position.y) &&
      std::isfinite(gps_start_heading_cw_deg) &&
      std::isfinite(gps_start_error_in) && gps_start_error_in >= 0.0 &&
      gps_start_error_in <= localization::kGpsMaxReportedErrorIn;
  if (path2_gps_heading_anchor_valid) {
    const auto gps_project_start =
        localization::vex_gps_to_project_robot_pose(
            gps_start_position.x, gps_start_position.y,
            gps_start_heading_cw_deg);
    path2_gps_heading_rotation_deg = std::remainder(
        route_start_heading_deg - gps_project_start.heading_deg, 360.0);
  }
  path2_prefer_gps_until_ms = 0;

  Path2LiftService lift;
  lift.request(0);

  // A. Front hits the Toggle once, immediately backs all the way to the
  // starting position to create turning clearance, then turns 75 degrees
  // toward its alliance's preload Goal and reverses into it. These short legs
  // deliberately use the aggressive Path 2 power settings and tight
  // deadlines so Phase 1 chains quickly.
  const auto toggle = path2_fast_drive(
      kToggleSignedDistanceIn, kTogglePower, 1100, true);
  if (toggle.disabled || toggle.traveled_in < 0.5) return false;
  // The robot now starts one inch farther from the Toggle. Ram the extra inch,
  // but return only the original six-inch offset so every later field waypoint
  // begins from the already-qualified route anchor instead of remaining shifted.
  const auto toggle_return = path2_fast_drive(
      -kToggleReturnDistanceIn, kToggleReturnPower, 1600);
  if (toggle_return.disabled) return false;
  // This alignment only needs to be accurate enough for the short preload
  // docking leg. Do not spend multiple seconds chasing sub-degree settling.
  if (!path2_fast_turn(first_goal_turn_delta_deg, true, 5.0, 1400, 30)) {
    return false;
  }
  const auto preload_reverse = path2_fast_drive(
      -kPhase1PreloadReverseIn, kPhase1DrivePower, 1900);
  if (preload_reverse.disabled) return false;
  path2_blank_impact_imu(600);
  navigation::update();
  const auto first_goal_pose = navigation::current_pose();
  std::printf("PATH2 first_goal x=%.3f y=%.3f heading=%.3f\n",
              first_goal_pose.x_in, first_goal_pose.y_in,
              first_goal_pose.heading_deg);
  std::fflush(stdout);
  path2_score(kPreloadDepositMs);
  std::printf("PATH2_PHASE completed=preload_deposit next=first_stack\n");
  std::fflush(stdout);

  if constexpr (kTestStopAfterPhase == 1) {
    path2_stop_all();
    std::printf("PATH2 test_stop=phase_1_after_preload_drop\n");
    std::fflush(stdout);
    return true;
  }

  // B. The preload alignment leaves the chassis parallel to the path X axis
  // above the Goal. Retreat farther away from the Goal before curving rear-first
  // toward the next cup so the boomerang has enough room to align its approach.
  auto goal_clearance = path2_fast_drive(
      phase2_goal_clearance_in, kPhase1DrivePower, 2200);
  if (goal_clearance.disabled) return false;
  if (goal_clearance.traveled_in < phase2_goal_clearance_in - 0.25) {
    goal_clearance = path2_fast_drive(
        phase2_goal_clearance_in - goal_clearance.traveled_in,
        kPhase1DrivePower, 1200);
    if (goal_clearance.disabled) return false;
  }
  if (!path2_approach_stack(route_stack_a, true,
                            "stack A boomerang approach")) return false;

  // Phase 2 ends at the stack coordinate. The ADI-E claw begins closing for
  // the final 9.7 inches based on the latest loaded test.
  if constexpr (kTestStopAfterPhase == 2) {
    path2_stop_all();
    std::printf("PATH2 test_stop=phase_2_after_first_cup_capture\n");
    std::fflush(stdout);
    return true;
  }

  // C. Pull the captured stack fully into the claw, then begin raising Stage 1
  // asynchronously. Turn to the alliance-mirrored scoring heading, reverse 24
  // inches into the Goal, pulse the lift down, then outtake and retreat.
  auto extra_capture = path2_fast_drive(
      -kStage1ExtraCaptureIn, kStage1ExtraCapturePower, 2000);
  if (extra_capture.disabled) return false;
  double extra_capture_total_in = extra_capture.traveled_in;
  for (int retry = 0;
       retry < 2 &&
       extra_capture_total_in < kStage1ExtraCaptureIn - 0.25;
       ++retry) {
    const double remaining_capture =
        kStage1ExtraCaptureIn - extra_capture_total_in;
    extra_capture = path2_fast_drive(
        -remaining_capture, kStage1ExtraCapturePower, 1400);
    if (extra_capture.disabled) return false;
    extra_capture_total_in += extra_capture.traveled_in;
  }
  if (extra_capture_total_in < kStage1ExtraCaptureIn - 0.25) {
    std::printf(
        "PATH2 abort=first_extra_capture_short target=%.2f traveled=%.2f\n",
        kStage1ExtraCaptureIn, extra_capture_total_in);
    std::fflush(stdout);
    return false;
  }
  lift.request(kFirstCupLiftStage);
  // Wait only for enough physical clearance to keep the loaded claw from
  // dragging during rotation. Never wait for Stage 2 to settle: it continues
  // rising concurrently through the turn and drive into the Goal.
  if (!lift.wait_at_least(kLoadedTurnClearanceDeg,
                          kLoadedTurnClearanceTimeoutMs)) {
    std::printf("PATH2 continue=first_lift_clearance_timeout position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  if (!path2_fast_turn(first_stack_score_heading_deg)) return false;
  const auto goal_drive = path2_fast_drive(
      -kStage1GoalDriveIn, kStage1GoalDrivePower, 3000, true, true);
  if (goal_drive.disabled || goal_drive.traveled_in < 1.0) return false;
  path2_blank_impact_imu(600);
  // Confirm Stage 1 at the first Goal. It has been rising asynchronously
  // throughout the turn and drive, so this normally returns immediately.
  if (!lift.wait_ready(kFirstCupLiftStage, kLiftReadyToleranceDeg,
                       kScoreStageReadyTimeoutMs)) {
    std::printf("PATH2 continue=first_score_stage_not_ready position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  // Deposit with a short direct pulse instead of waiting up to 1.1 seconds
  // for the position PID to lower and settle. The lift task owns the motors,
  // applies power 100 for 0.1 seconds, and brakes before the claw releases.
  lift.start_score_drop_for(kScoreDropPower, kScoreDropPulseMs);
  if (!lift.wait_score_drop_complete(kScoreDropWaitTimeoutMs)) {
    std::printf("PATH2 continue=first_score_drop_pulse_timeout position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  set_claw_piston(true);
  pros::delay(kStage1OuttakeLeadMs);
  // Clear the scoring height immediately: bypass the slow position controller
  // and run both cascade motors downward at full power for 0.2 seconds.
  lift.start_full_down_for(500);
  const auto score_retreat = path2_fast_drive(
      kStage1ScoreRetreatIn, kStage1ScoreRetreatPower, 2400);
  if (score_retreat.disabled) return false;
  if (!lift.wait_full_down_complete(800)) {
    // The downward pulse is asynchronous and the next phase does not require
    // an exact zero before beginning its turn. Never discard the remainder of
    // autonomous solely because the completion flag arrived late.
    std::printf("PATH2 continue=first_full_down_pulse_timeout position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  pros::lcd::set_text(6, "P2: FIRST SCORE DONE");
  std::printf("PATH2_PHASE completed=first_stack_deposit next=second_stack\n");
  std::fflush(stdout);
  // Chain directly into the next phase while the lift returns home in its
  // background task. The next pickup can override that target when captured.
  if constexpr (kTestStopAfterPhase == 3) {
    path2_stop_all();
    std::printf("PATH2 test_stop=phase_3_after_first_cup_score\n");
    std::fflush(stdout);
    return true;
  }
  // D. Face stack B with the rear claw. In the production heading frame the
  // correct tested direction is absolute 225 degrees (the user's 135-degree
  // frame). Computing it from the noisy post-retreat pose over-rotated toward
  // the mirrored Goal at path (48,-24), while a relative +45 inherited error.
  std::printf(
      "PATH2_ALLIANCE side=%s goal=%.2f,%.2f stack_a=%.2f,%.2f "
      "stack_b=%.2f,%.2f\n",
      blue_side ? "blue" : "red", route_goal.x, route_goal.y,
      route_stack_a.x, route_stack_a.y, route_stack_b.x, route_stack_b.y);
  std::fflush(stdout);
  // From score 1 onward, wall-strip GPS heading is authoritative. Never cancel
  // the remaining route merely because the impact invalidated IMU/fused pose.
  if (!path2_fast_turn(second_stack_pickup_heading_deg, false, 3.5)) {
    std::printf("PATH2 continue=second_stack_turn_degraded\n");
    std::fflush(stdout);
  }
  constexpr double kSecondStackCenterTravelIn =
      kStackBTravelIn - kRearPickupReachIn;
  const double second_stack_fast_in =
      kSecondStackCenterTravelIn * kStackFastFraction;
  auto second_stack_drive = path2_fast_drive(
      -second_stack_fast_in, kSecondStackApproachPower, 2200);
  if (second_stack_drive.disabled) return false;
  const double second_slow_in =
      kSecondStackCenterTravelIn - second_stack_fast_in;
  second_stack_drive = path2_fast_drive(
      -std::max(0.0, second_slow_in - second_stack_grab_lead_in),
      kSecondStackSlowPower, 2600);
  if (second_stack_drive.disabled) return false;
  // Begin closing at the alliance-tuned lead distance, then carry the claw
  // through the remaining approach so it cannot pass the stack before closing.
  set_claw_piston(false);
  pros::delay(100);
  second_stack_drive = path2_fast_drive(
      -second_stack_grab_lead_in, kSecondStackSlowPower, 1200);
  if (second_stack_drive.disabled) return false;
  pros::delay(50);

  // Pull the stack 4.5 more inches fully into the claw before raising it.
  auto second_extra = path2_fast_drive(
      -kStage2ExtraCaptureIn, kStage2ExtraCapturePower, 1600);
  if (second_extra.disabled) return false;
  if (second_extra.traveled_in < kStage2ExtraCaptureIn - 0.5) {
    const double second_remaining =
        kStage2ExtraCaptureIn - second_extra.traveled_in;
    second_extra = path2_fast_drive(
        -second_remaining, kStage2ExtraCapturePower, 1200);
    if (second_extra.disabled) return false;
  }

  lift.request(kSecondCupLiftStage);
  if (!lift.wait_at_least(kLoadedTurnClearanceDeg,
                          kLoadedTurnClearanceTimeoutMs)) {
    std::printf("PATH2 continue=second_lift_clearance_timeout position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  // Ignore accumulated relative-heading error here. The prior -90-degree
  // absolute target faced directly away from the Goal, so the correct field
  // alignment after the (48,-48) pickup is its 180-degree opposite: +90.
  // E. Complete that rear-facing alignment, reverse into the Goal, pulse the
  // lift down, then outtake and retreat.
  if (!path2_fast_turn(second_stack_score_heading_deg, false, 3.5)) {
    std::printf("PATH2 continue=second_goal_turn_degraded\n");
    std::fflush(stdout);
  }
  const auto second_goal_drive = path2_fast_drive(
      -kStage2GoalDriveIn, kStage2GoalDrivePower, 3600, true, true);
  if (second_goal_drive.disabled || second_goal_drive.traveled_in < 1.0) {
    return false;
  }
  path2_blank_impact_imu(600);
  // Confirm Stage 2 at the second Goal before performing the 100-degree drop.
  if (!lift.wait_ready(kSecondCupLiftStage, kLiftReadyToleranceDeg,
                       kScoreStageReadyTimeoutMs)) {
    std::printf("PATH2 continue=second_score_stage_not_ready position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  lift.start_score_drop_for(kScoreDropPower, kScoreDropPulseMs);
  if (!lift.wait_score_drop_complete(kScoreDropWaitTimeoutMs)) {
    std::printf("PATH2 continue=second_score_drop_pulse_timeout position=%.1f\n",
                cascade_lift::snapshot().position_deg);
    std::fflush(stdout);
  }
  set_claw_piston(true);
  pros::delay(kStage1OuttakeLeadMs);
  lift.start_full_down_for(500);
  const auto second_retreat = path2_fast_drive(
      kStage2ScoreRetreatIn, kStage2ScoreRetreatPower, 2400);
  if (second_retreat.disabled) return false;
  if (!lift.wait_full_down_complete(800)) {
    std::printf("PATH2 abort=second_full_down_pulse_timeout\n");
    std::fflush(stdout);
    return false;
  }
  // End the tested route 24 inches clear of the Goal, facing the mirrored
  // alliance-specific final heading and holding position.
  if (!path2_fast_turn(final_heading_deg, false, 3.5)) {
    std::printf("PATH2 continue=final_turn_degraded\n");
    std::fflush(stdout);
  }
  // This is the requested competition endpoint: 24 inches clear of the second
  // Goal, lift returned by the full-down pulse, and chassis facing 180 degrees.
  // The old development build called this a phase-4 test stop; it is now the
  // explicit successful end of the complete two-cup route.
  path2_stop_all();
  std::printf("PATH2 complete=1\n");
  std::fflush(stdout);
  return true;
}

bool localization_two_cup_red_auton() {
  return localization_two_cup_auton(false);
}

bool localization_two_cup_blue_auton() {
  return localization_two_cup_auton(true);
}
