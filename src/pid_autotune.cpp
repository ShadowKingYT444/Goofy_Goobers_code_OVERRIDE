#include "main.h"
#include "pid_autotune.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
// Turn autotune experiment.
constexpr double kTurnKp = 1.8;
constexpr double kTurnKi = 0.0;
constexpr double kTurnStartI = 0.0;
constexpr double kKdLow = 0.0;
constexpr double kKdHigh = 0.35;
constexpr double kTurnTargetDeg = 45.0;
constexpr int kTurnSpeed = 112;
constexpr double kTurnMinimumPower = 14.0;
constexpr double kTurnSlewPowerPerSec = 800.0;
constexpr int kGoldenReductions = 10;
constexpr int kCandidateCount = kGoldenReductions + 2;
constexpr double kInversePhi = 0.6180339887498948482;

// Sampling and settling.
constexpr std::uint32_t kTrialHorizonMs = 2500;
constexpr std::uint32_t kSamplePeriodMs = 10;
constexpr std::uint32_t kMaximumSampleGapMs = 40;
constexpr std::uint32_t kStopTaskDrainMs = 20;
constexpr std::uint32_t kTailWindowMs = 200;
constexpr double kSettlingErrorDeg = 2.0;
constexpr double kSettlingVelocityDegPerSec = 5.0;
constexpr double kVelocityFilterTimeConstantSec = 0.06;
constexpr double kCrossingHysteresisDeg = 1.0;

// Safety limits.  These end a bad experiment before its score is considered.
constexpr std::uint32_t kWrongWayGraceMs = 250;
constexpr double kWrongWayDeg = 6.0;
constexpr double kSevereWrongWayDeg = 15.0;
constexpr std::uint32_t kStallCheckMs = 600;
constexpr double kMinimumWindowTravelDeg = 3.0;
constexpr double kStallMinimumRemainingErrorDeg = 10.0;
constexpr double kMaximumExcursionDeg = 90.0;
constexpr double kMaximumTotalTravelDeg = 180.0;
constexpr double kMaximumSampleJumpDeg = 20.0;
constexpr int kMaximumTargetCrossings = 6;
constexpr double kQuiescentVelocityDegPerSec = 3.0;
constexpr std::uint32_t kQuiescentHoldMs = 150;
constexpr std::uint32_t kQuiescentTimeoutMs = 1000;
constexpr int kMaximumConsecutiveUnsafeCandidates = 2;
constexpr double kRejectedCandidateCost = 1.0e9;

// Centralized objective weights.
constexpr double kNitaeWeight = 100.0;
constexpr double kOvershootWeight = 150.0;
constexpr double kSettlingWeight = 40.0;
constexpr double kFinalErrorWeight = 80.0;
constexpr double kFinalVelocityWeight = 10.0;
constexpr double kRingingWeight = 8.0;
constexpr double kDirectionDisagreementWeight = 0.2;

struct Gains {
  double kp;
  double ki;
  double kd;
  double start_i;
};

enum class RejectReason {
  none,
  not_run,
  field_control,
  imu_not_ready,
  invalid_sensor,
  sensor_jump,
  sample_gap,
  wrong_way,
  stalled,
  excessive_excursion,
  oscillatory,
  not_quiescent,
  insufficient_samples,
  unsafe_bracket,
  no_valid_candidate,
};

struct TrialResult {
  double cost = kRejectedCandidateCost;
  double nitae = 0.0;
  double overshoot = 0.0;
  double settling = 1.0;
  double final_error_deg = kTurnTargetDeg;
  double final_velocity_deg_per_sec = 0.0;
  double ringing = 0.0;
  double elapsed_ms = 0.0;
  int target_crossings = 0;
  int samples = 0;
  int direction = 0;
  RejectReason reason = RejectReason::not_run;
  bool valid = false;
};

struct CandidateResult {
  double kd = 0.0;
  double cost = kRejectedCandidateCost;
  TrialResult positive;
  TrialResult negative;
  RejectReason reason = RejectReason::not_run;
  bool valid = false;
};

bool tuned_turn_ready = false;
Gains tuned_turn = {kTurnKp, kTurnKi, 0.08, kTurnStartI};

const char* reject_reason_name(RejectReason reason) {
  switch (reason) {
    case RejectReason::none:
      return "none";
    case RejectReason::not_run:
      return "not-run";
    case RejectReason::field_control:
      return "field-control";
    case RejectReason::imu_not_ready:
      return "imu-not-ready";
    case RejectReason::invalid_sensor:
      return "invalid-sensor";
    case RejectReason::sensor_jump:
      return "sensor-jump";
    case RejectReason::sample_gap:
      return "sample-gap";
    case RejectReason::wrong_way:
      return "wrong-way";
    case RejectReason::stalled:
      return "stalled";
    case RejectReason::excessive_excursion:
      return "excessive-excursion";
    case RejectReason::oscillatory:
      return "oscillatory";
    case RejectReason::not_quiescent:
      return "not-quiescent";
    case RejectReason::insufficient_samples:
      return "insufficient-samples";
    case RejectReason::unsafe_bracket:
      return "unsafe-bracket";
    case RejectReason::no_valid_candidate:
      return "no-valid-candidate";
  }
  return "unknown";
}

bool is_candidate_specific_rejection(RejectReason reason) {
  return reason == RejectReason::stalled ||
         reason == RejectReason::excessive_excursion ||
         reason == RejectReason::oscillatory;
}

bool field_control_connected() {
  return pros::competition::is_connected();
}

bool wait_for_live_imu_ready(std::uint32_t timeout_ms = 5000) {
  const std::uint32_t start_ms = pros::millis();

  while (chassis.imu.is_calibrating()) {
    if (pros::millis() - start_ms >= timeout_ms) return false;
    pros::lcd::set_text(5, "waiting for IMU...");
    pros::delay(10);
  }

  const pros::ImuStatus status = chassis.imu.get_status();
  const double angle = chassis.drive_imu_get();
  std::printf(
      "PID_TUNE IMU ez_flag=%d status=%d calibrating=%d angle=%.3f\n",
      static_cast<int>(chassis.drive_imu_calibrated()),
      static_cast<int>(status),
      static_cast<int>(chassis.imu.is_calibrating()),
      angle);
  std::fflush(stdout);

  return status != pros::ImuStatus::error && std::isfinite(angle);
}

void hard_stop() {
  // Disable the background task before resetting its target.  Resetting the
  // target first can allow one task tick that commands the robot back to zero.
  chassis.drive_mode_set(ez::DISABLE, true);
  chassis.drive_set(0, 0);
  // Drain any TURN iteration that selected its mode immediately before the
  // disable.  Its late motor write is overwritten by the final zero below.
  pros::delay(kStopTaskDrainMs);
  chassis.drive_set(0, 0);
  chassis.pid_targets_reset();
}

void command_turn_power(double turn_power) {
  const int power = static_cast<int>(std::clamp(turn_power, -127.0, 127.0));
  // Positive field-math turn is CCW. The raw V5 IMU rotation used below is
  // clockwise-positive, so a positive response makes raw rotation decrease.
  for (auto& motor : chassis.left_motors) motor.move(power);
  for (auto& motor : chassis.right_motors) motor.move(-power);
}

RejectReason wait_until_quiescent() {
  hard_stop();
  if (field_control_connected()) return RejectReason::field_control;

  double previous = chassis.drive_imu_get();
  if (!std::isfinite(previous)) return RejectReason::invalid_sensor;

  const std::uint32_t start_ms = pros::millis();
  std::uint32_t previous_ms = start_ms;
  std::uint32_t quiet_since_ms = 0;

  while (pros::millis() - start_ms < kQuiescentTimeoutMs) {
    pros::delay(kSamplePeriodMs);
    if (field_control_connected()) return RejectReason::field_control;

    const std::uint32_t now_ms = pros::millis();
    const double current = chassis.drive_imu_get();
    if (!std::isfinite(current)) return RejectReason::invalid_sensor;

    const std::uint32_t dt_ms = now_ms - previous_ms;
    if (dt_ms == 0) continue;
    const double velocity =
        std::fabs(current - previous) * 1000.0 / static_cast<double>(dt_ms);

    if (velocity <= kQuiescentVelocityDegPerSec) {
      if (quiet_since_ms == 0) quiet_since_ms = now_ms;
      if (now_ms - quiet_since_ms >= kQuiescentHoldMs) {
        return RejectReason::none;
      }
    } else {
      quiet_since_ms = 0;
    }

    previous = current;
    previous_ms = now_ms;
  }

  return RejectReason::not_quiescent;
}

int hysteretic_sign(double value) {
  if (value > kCrossingHysteresisDeg) return 1;
  if (value < -kCrossingHysteresisDeg) return -1;
  return 0;
}

TrialResult run_turn_trial(double kd, int direction) {
  TrialResult result;
  result.direction = direction;

  const RejectReason ready = wait_until_quiescent();
  if (ready != RejectReason::none) {
    result.reason = ready;
    hard_stop();
    return result;
  }

  const double baseline_deg = chassis.drive_imu_get();
  if (!std::isfinite(baseline_deg)) {
    result.reason = RejectReason::invalid_sensor;
    hard_stop();
    return result;
  }

  const std::uint32_t start_ms = pros::millis();
  std::uint32_t previous_ms = start_ms;
  double previous_measurement_deg = baseline_deg;
  double previous_response_deg = 0.0;
  double filtered_velocity_deg_per_sec = 0.0;
  double weighted_error_integral = 0.0;
  double previous_weighted_error = 0.0;
  double maximum_response_deg = 0.0;
  double total_travel_deg = 0.0;
  double stall_window_travel_deg = 0.0;
  double stall_window_start_ms = 0.0;
  double last_unsettled_ms = 0.0;
  double tail_velocity_integral = 0.0;
  double tail_duration_sec = 0.0;
  int previous_error_side = 1;
  double turn_command = 0.0;

  if (field_control_connected()) {
    result.reason = RejectReason::field_control;
    hard_stop();
    return result;
  }
  chassis.drive_mode_set(ez::DISABLE, true);
  result.reason = RejectReason::none;

  while (true) {
    pros::delay(kSamplePeriodMs);
    if (field_control_connected()) {
      result.reason = RejectReason::field_control;
      break;
    }

    const std::uint32_t now_ms = pros::millis();
    const std::uint32_t dt_ms = now_ms - previous_ms;
    if (dt_ms == 0) continue;
    if (dt_ms > kMaximumSampleGapMs) {
      result.reason = RejectReason::sample_gap;
      break;
    }

    const double measurement_deg = chassis.drive_imu_get();
    if (!std::isfinite(measurement_deg)) {
      result.reason = RejectReason::invalid_sensor;
      break;
    }

    const double measurement_step_deg =
        measurement_deg - previous_measurement_deg;
    if (std::fabs(measurement_step_deg) > kMaximumSampleJumpDeg) {
      result.reason = RejectReason::sensor_jump;
      break;
    }

    const double raw_elapsed_ms = static_cast<double>(now_ms - start_ms);
    const double elapsed_ms =
        std::min(raw_elapsed_ms, static_cast<double>(kTrialHorizonMs));
    const double effective_dt_ms = elapsed_ms - result.elapsed_ms;
    if (effective_dt_ms <= 0.0) break;
    const double elapsed_sec = elapsed_ms / 1000.0;
    const double dt_sec = effective_dt_ms / 1000.0;
    const double effective_step_deg =
        measurement_step_deg *
        (effective_dt_ms / static_cast<double>(dt_ms));
    const double effective_measurement_deg =
        previous_measurement_deg + effective_step_deg;
    const double response_deg =
        static_cast<double>(direction) *
        (baseline_deg - effective_measurement_deg);
    const double error_deg = kTurnTargetDeg - response_deg;
    const double raw_velocity_deg_per_sec =
        -static_cast<double>(direction) * effective_step_deg / dt_sec;
    const double velocity_alpha =
        dt_sec / (kVelocityFilterTimeConstantSec + dt_sec);
    filtered_velocity_deg_per_sec +=
        velocity_alpha *
        (raw_velocity_deg_per_sec - filtered_velocity_deg_per_sec);

    const double normalized_error =
        std::fabs(error_deg) / kTurnTargetDeg;
    const double weighted_error = elapsed_sec * normalized_error;
    weighted_error_integral +=
        0.5 * (previous_weighted_error + weighted_error) * dt_sec;
    previous_weighted_error = weighted_error;

    maximum_response_deg = std::max(maximum_response_deg, response_deg);
    const double response_step_deg =
        std::fabs(response_deg - previous_response_deg);
    total_travel_deg += response_step_deg;
    stall_window_travel_deg += response_step_deg;

    const int error_side = hysteretic_sign(error_deg);
    if (error_side != 0 && error_side != previous_error_side) {
      ++result.target_crossings;
      previous_error_side = error_side;
    }

    if (std::fabs(error_deg) > kSettlingErrorDeg ||
        std::fabs(filtered_velocity_deg_per_sec) >
            kSettlingVelocityDegPerSec) {
      last_unsettled_ms = elapsed_ms;
    }

    const double tail_start_ms =
        static_cast<double>(kTrialHorizonMs - kTailWindowMs);
    const double tail_dt_ms =
        std::max(0.0,
                 elapsed_ms - std::max(result.elapsed_ms, tail_start_ms));
    if (tail_dt_ms > 0.0) {
      tail_velocity_integral +=
          std::fabs(filtered_velocity_deg_per_sec) * tail_dt_ms / 1000.0;
      tail_duration_sec += tail_dt_ms / 1000.0;
    }

    result.elapsed_ms = elapsed_ms;
    result.final_error_deg = error_deg;
    ++result.samples;

    if (response_deg < -kSevereWrongWayDeg ||
        (elapsed_ms >= static_cast<double>(kWrongWayGraceMs) &&
         response_deg < -kWrongWayDeg)) {
      result.reason = RejectReason::wrong_way;
      break;
    }
    if (elapsed_ms - stall_window_start_ms >=
        static_cast<double>(kStallCheckMs)) {
      // A controller holding near the target is not stalled.  Its remaining
      // error belongs in the cost; only reject a motion that stops far short.
      if (std::fabs(error_deg) > kStallMinimumRemainingErrorDeg &&
          stall_window_travel_deg < kMinimumWindowTravelDeg) {
        result.reason = RejectReason::stalled;
        break;
      }
      stall_window_start_ms = elapsed_ms;
      stall_window_travel_deg = 0.0;
    }
    if (std::fabs(response_deg) > kMaximumExcursionDeg) {
      result.reason = RejectReason::excessive_excursion;
      break;
    }
    if (result.target_crossings > kMaximumTargetCrossings ||
        total_travel_deg > kMaximumTotalTravelDeg) {
      result.reason = RejectReason::oscillatory;
      break;
    }

    double requested_effort =
        kTurnKp * error_deg - kd * filtered_velocity_deg_per_sec;
    requested_effort = std::clamp(
        requested_effort,
        -static_cast<double>(kTurnSpeed),
        static_cast<double>(kTurnSpeed));
    if (std::fabs(error_deg) > kSettlingErrorDeg &&
        std::fabs(requested_effort) < kTurnMinimumPower) {
      requested_effort = requested_effort >= 0.0
                             ? kTurnMinimumPower
                             : -kTurnMinimumPower;
    }
    const double maximum_command_step = kTurnSlewPowerPerSec * dt_sec;
    turn_command = std::clamp(requested_effort,
                              turn_command - maximum_command_step,
                              turn_command + maximum_command_step);
    command_turn_power(static_cast<double>(direction) * turn_command);

    previous_measurement_deg = effective_measurement_deg;
    previous_response_deg = response_deg;
    previous_ms = now_ms;

    if (raw_elapsed_ms >= static_cast<double>(kTrialHorizonMs)) break;
  }

  hard_stop();

  constexpr double kTrialHorizonSec =
      static_cast<double>(kTrialHorizonMs) / 1000.0;
  if (result.samples < 2 || result.elapsed_ms <= 0.0) {
    if (result.reason == RejectReason::none) {
      result.reason = RejectReason::insufficient_samples;
    }
    return result;
  }

  result.nitae =
      2.0 * weighted_error_integral /
      (kTrialHorizonSec * kTrialHorizonSec);
  result.overshoot =
      std::max(0.0, maximum_response_deg - kTurnTargetDeg) /
      kTurnTargetDeg;
  result.settling =
      std::min(1.0,
               last_unsettled_ms /
                   static_cast<double>(kTrialHorizonMs));
  result.final_velocity_deg_per_sec =
      tail_duration_sec > 0.0
          ? tail_velocity_integral / tail_duration_sec
          : std::fabs(filtered_velocity_deg_per_sec);
  result.ringing =
      static_cast<double>(std::max(0, result.target_crossings - 1));

  if (result.reason != RejectReason::none) return result;

  const double normalized_final_error =
      std::fabs(result.final_error_deg) / kTurnTargetDeg;
  const double normalized_final_velocity =
      result.final_velocity_deg_per_sec / kTurnTargetDeg;
  result.cost =
      kNitaeWeight * result.nitae +
      kOvershootWeight * result.overshoot +
      kSettlingWeight * result.settling +
      kFinalErrorWeight * normalized_final_error +
      kFinalVelocityWeight * normalized_final_velocity +
      kRingingWeight * result.ringing;
  result.valid = std::isfinite(result.cost);
  if (!result.valid) {
    result.cost = kRejectedCandidateCost;
    result.reason = RejectReason::invalid_sensor;
  }
  return result;
}

void print_trial(int candidate_number,
                 double kd,
                 const TrialResult& trial) {
  std::printf(
      "PID_TUNE trial=%d/%d kd=%.6f dir=%+d valid=%d cost=%.3f "
      "nitae=%.4f overshoot=%.4f settling=%.4f final_error=%.3f "
      "final_velocity=%.3f ringing=%.0f crossings=%d elapsed=%.0fms "
      "samples=%d reason=%s\n",
      candidate_number,
      kCandidateCount,
      kd,
      trial.direction,
      static_cast<int>(trial.valid),
      trial.cost,
      trial.nitae,
      trial.overshoot,
      trial.settling,
      trial.final_error_deg,
      trial.final_velocity_deg_per_sec,
      trial.ringing,
      trial.target_crossings,
      trial.elapsed_ms,
      trial.samples,
      reject_reason_name(trial.reason));
  std::fflush(stdout);
}

CandidateResult evaluate_candidate(double kd, int candidate_number) {
  CandidateResult candidate;
  candidate.kd = kd;
  const int first_direction = candidate_number % 2 == 1 ? 1 : -1;
  const int directions[2] = {first_direction, -first_direction};

  pros::lcd::print(4,
                   "Turn kD %d/%d",
                   candidate_number,
                   kCandidateCount);
  pros::lcd::print(5, "kD %.3f", kd);

  for (int direction : directions) {
    pros::lcd::print(6, "direction %+d", direction);
    const TrialResult trial = run_turn_trial(kd, direction);
    if (direction > 0) {
      candidate.positive = trial;
    } else {
      candidate.negative = trial;
    }
    print_trial(candidate_number, kd, trial);

    if (!trial.valid) {
      candidate.reason = trial.reason;
      candidate.cost = kRejectedCandidateCost;
      hard_stop();
      return candidate;
    }
  }

  candidate.cost =
      0.5 * (candidate.positive.cost + candidate.negative.cost) +
      kDirectionDisagreementWeight *
          std::fabs(candidate.positive.cost - candidate.negative.cost);
  candidate.valid = std::isfinite(candidate.cost);
  candidate.reason = candidate.valid ? RejectReason::none
                                     : RejectReason::invalid_sensor;
  if (!candidate.valid) candidate.cost = kRejectedCandidateCost;

  std::printf(
      "PID_TUNE candidate=%d/%d kP=%.3f kI=%.3f kD=%.6f valid=%d "
      "pair_cost=%.3f positive=%.3f negative=%.3f reason=%s\n",
      candidate_number,
      kCandidateCount,
      kTurnKp,
      kTurnKi,
      kd,
      static_cast<int>(candidate.valid),
      candidate.cost,
      candidate.positive.cost,
      candidate.negative.cost,
      reject_reason_name(candidate.reason));
  std::fflush(stdout);
  return candidate;
}

double comparison_cost(const CandidateResult& candidate) {
  return candidate.valid ? candidate.cost : kRejectedCandidateCost;
}

void consider_best(CandidateResult& best,
                   const CandidateResult& candidate) {
  if (!candidate.valid) return;
  if (!best.valid || candidate.cost < best.cost) best = candidate;
}

void print_abort(RejectReason reason) {
  std::printf("PID_TUNE ABORTED reason=%s; turn constants preserved\n",
              reject_reason_name(reason));
  std::fflush(stdout);
  pros::lcd::set_text(4, "PID TUNE ABORTED");
  pros::lcd::print(5, "reason %s", reject_reason_name(reason));
  pros::lcd::set_text(6, "turn gains preserved");
}
}  // namespace

bool pid_autotune_apply_if_ready() {
  // The measured controller is the fused-navigation controller in autons.cpp,
  // not EZ-Template's differently-signed turn PID. Gains are printed for a
  // deliberate source update after the supervised run; never apply them to
  // the wrong controller at runtime.
  return false;
}

bool pid_autotune_auton() {
  hard_stop();
  // Never preserve a previous result across a failed preflight or rerun.
  tuned_turn_ready = false;

  if (field_control_connected()) {
    print_abort(RejectReason::field_control);
    return false;
  }
  if (!wait_for_live_imu_ready()) {
    print_abort(RejectReason::imu_not_ready);
    return false;
  }

  pros::lcd::set_text(4, "PID TURN AUTOTUNE");
  pros::lcd::set_text(5, "fused kP 1.800");
  pros::lcd::set_text(6, "24 turns maximum");
  std::printf(
      "PID_TUNE START kP=%.3f kI=%.3f kD_low=%.3f kD_high=%.3f "
      "target=+/-%.1f speed=%d horizon=%lums candidates=%d\n",
      kTurnKp,
      kTurnKi,
      kKdLow,
      kKdHigh,
      kTurnTargetDeg,
      kTurnSpeed,
      static_cast<unsigned long>(kTrialHorizonMs),
      kCandidateCount);
  std::fflush(stdout);

  double low = kKdLow;
  double high = kKdHigh;
  double c = high - kInversePhi * (high - low);
  double d = low + kInversePhi * (high - low);
  int candidate_number = 0;
  int consecutive_unsafe_candidates = 0;
  RejectReason abort_reason = RejectReason::none;
  RejectReason last_candidate_rejection = RejectReason::none;
  CandidateResult best;

  auto record_candidate = [&](const CandidateResult& candidate) {
    consider_best(best, candidate);
    if (candidate.valid) {
      consecutive_unsafe_candidates = 0;
      return true;
    }
    if (!is_candidate_specific_rejection(candidate.reason)) {
      abort_reason = candidate.reason;
      return false;
    }
    last_candidate_rejection = candidate.reason;
    ++consecutive_unsafe_candidates;
    if (consecutive_unsafe_candidates >=
        kMaximumConsecutiveUnsafeCandidates) {
      abort_reason = last_candidate_rejection;
      return false;
    }
    return true;
  };

  CandidateResult cost_c = evaluate_candidate(c, ++candidate_number);
  bool keep_searching = record_candidate(cost_c);
  CandidateResult cost_d;
  if (keep_searching) {
    cost_d = evaluate_candidate(d, ++candidate_number);
    keep_searching = record_candidate(cost_d);
  }
  if (keep_searching && !cost_c.valid && !cost_d.valid) {
    abort_reason = last_candidate_rejection != RejectReason::none
                       ? last_candidate_rejection
                       : RejectReason::unsafe_bracket;
    keep_searching = false;
  }

  for (int reduction = 0;
       keep_searching && reduction < kGoldenReductions;
       ++reduction) {
    if (comparison_cost(cost_c) < comparison_cost(cost_d)) {
      high = d;
      d = c;
      cost_d = cost_c;
      c = high - kInversePhi * (high - low);
      cost_c = evaluate_candidate(c, ++candidate_number);
      keep_searching = record_candidate(cost_c);
    } else {
      low = c;
      c = d;
      cost_c = cost_d;
      d = low + kInversePhi * (high - low);
      cost_d = evaluate_candidate(d, ++candidate_number);
      keep_searching = record_candidate(cost_d);
    }

    if (keep_searching && !cost_c.valid && !cost_d.valid) {
      abort_reason = last_candidate_rejection != RejectReason::none
                         ? last_candidate_rejection
                         : RejectReason::unsafe_bracket;
      keep_searching = false;
    }
  }

  hard_stop();
  if (field_control_connected()) {
    abort_reason = RejectReason::field_control;
    keep_searching = false;
  }

  if (!keep_searching || !best.valid) {
    if (abort_reason == RejectReason::none) {
      abort_reason = RejectReason::no_valid_candidate;
    }
    print_abort(abort_reason);
    return false;
  }

  tuned_turn = {kTurnKp, kTurnKi, best.kd, kTurnStartI};
  tuned_turn_ready = true;
  std::printf(
      "PID_TUNE COMPLETE kP=%.3f kI=%.3f kD=%.6f start_i=%.1f "
      "cost=%.3f interval=[%.6f,%.6f] evaluated=%d\n",
      tuned_turn.kp,
      tuned_turn.ki,
      tuned_turn.kd,
      tuned_turn.start_i,
      best.cost,
      low,
      high,
      candidate_number);
  std::printf(
      "PID_TUNE PASTE fused kP=%.3f kI=%.3f kD=%.6f start_i=%.1f "
      "min_power=%.1f slew=%.1f\n",
      tuned_turn.kp,
      tuned_turn.ki,
      tuned_turn.kd,
      tuned_turn.start_i,
      kTurnMinimumPower,
      kTurnSlewPowerPerSec);
  std::fflush(stdout);

  pros::lcd::set_text(4, "TUNING COMPLETE");
  pros::lcd::print(5, "kP %.3f kD %.3f", tuned_turn.kp, tuned_turn.kd);
  pros::lcd::print(6, "cost %.2f", best.cost);
  hard_stop();
  return true;
}
