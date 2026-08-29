#include "cascade_lift.hpp"

#include "subsystems.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace cascade_lift {
namespace {

// The installed port-16 sensor decreases while positive motor power raises the
// lift, so expose upward travel as positive throughout the lift subsystem.
constexpr double kSensorDirection = -1.0;
constexpr double kPmvPerDeg = 30.0;
constexpr double kImvPerDegS = 0.0;
constexpr double kDmvPerDegPerS = 3.0;
constexpr double kGravityMv = 0.0;  // Tune after the five heights are validated.
constexpr double kMinimumUpwardPidMv = 2500.0;
constexpr double kMinimumDownwardPidMv = 3000.0;
constexpr double kPidVoltageLimitMv = 12000.0;
constexpr double kPidDownwardVoltageLimitMv = 5000.0;
constexpr double kShortStageUpwardLimitMv = 6000.0;
constexpr double kVelocityFilterAlpha = 0.20;
constexpr double kIntegralZoneDeg = 40.0;
constexpr double kIntegralLimitDegS = 100.0;
constexpr double kPositionToleranceDeg = 5.0;
constexpr double kVelocityToleranceDegS = 8.0;
constexpr double kUpperSoftLimitDeg = 2000.0;
constexpr double kCalibrationTravelLimitDeg = kUpperSoftLimitDeg;
constexpr double kBottomRezeroWindowDeg = 40.0;
constexpr double kBottomStillVelocityDegS = 6.0;
constexpr std::int32_t kBottomLoadedCurrentMa = 1200;
constexpr std::uint32_t kBottomConfirmMs = 300;
constexpr double kStallVelocityDegS = 4.0;
constexpr double kStallOutputMv = 7000.0;
constexpr std::uint32_t kStallTimeoutMs = 350;
// Empirical port-16 multi-turn positions, measured upward from stage 0/rest.
// Unmeasured stages remain NaN until the current calibration session records
// them. Keep these values in ascending stage order.
constexpr std::array<double, kStageCount> kStageCalibrationDeg = {
    427.59, 757.62, 1140.56, 1524.20, 1950.00};

struct State {
  Snapshot data;
  std::array<double, kStageCount> stages{};
  int manual_power = 0;
  double integral = 0.0;
  double previous_error = 0.0;
  double previous_position = 0.0;
  std::uint32_t previous_ms = 0;
  std::uint32_t bottom_candidate_ms = 0;
  std::uint32_t stall_candidate_ms = 0;
  std::uint32_t still_candidate_ms = 0;
  std::uint32_t last_telemetry_ms = 0;
  double last_stop_candidate_deg = std::numeric_limits<double>::quiet_NaN();
};

State state;

bool valid_stage(int stage) {
  return stage >= 1 && stage <= static_cast<int>(kStageCount);
}

double read_position_deg(std::int32_t raw_centidegrees) {
  return kSensorDirection * static_cast<double>(raw_centidegrees) / 100.0;
}

std::int32_t average_current_ma() {
  return (slider_right.get_current_draw() + slider_left.get_current_draw()) / 2;
}

void command_power(int power) {
  slider_right.move(power);
  slider_left.move(power);
}

void command_voltage(double millivolts) {
  const auto voltage = static_cast<std::int32_t>(std::lround(
      std::clamp(millivolts, -12000.0, 12000.0)));
  slider_right.move_voltage(voltage);
  slider_left.move_voltage(voltage);
}

void reset_controller_terms() {
  state.integral = 0.0;
  state.previous_error = state.data.target_deg - state.data.position_deg;
  state.stall_candidate_ms = 0;
}

}  // namespace

void initialize_at_rest() {
  slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  command_power(0);
  slider_rotation_sensor.reset_position();

  state = State{};
  state.stages = kStageCalibrationDeg;
  state.data.sensor_ok = slider_rotation_sensor.is_installed();
  state.data.homed = state.data.sensor_ok;
  state.previous_ms = pros::millis();
  std::printf(
      "CASCADE_HOME mode=%s sensor_port=16 sensor_ok=%d "
      "right_port=2 left_port=9 stages=%lu\n",
      "start_assumed_rest",
      static_cast<int>(state.data.sensor_ok),
      static_cast<unsigned long>(kStageCount));
  std::fflush(stdout);
}

void set_manual_power(int power) {
  state.data.pid_active = false;
  state.data.target_stage = 0;
  state.manual_power = std::clamp(power, -127, 127);
  reset_controller_terms();
}

bool record_stage(int stage, double position_deg) {
  if (!valid_stage(stage) || !std::isfinite(position_deg) ||
      position_deg <= 0.0) {
    return false;
  }
  if (stage > 1 && state.stages[stage - 2] == state.stages[stage - 2] &&
      position_deg <= state.stages[stage - 2]) {
    return false;
  }
  state.stages[stage - 1] = position_deg;
  return true;
}

bool record_stage_here(int stage) {
  return record_stage(stage, state.data.position_deg);
}

bool stage_is_recorded(int stage) {
  return valid_stage(stage) && std::isfinite(state.stages[stage - 1]);
}

double stage_position_deg(int stage) {
  return stage_is_recorded(stage) ? state.stages[stage - 1]
                                  : std::numeric_limits<double>::quiet_NaN();
}

bool set_target_stage(int stage) {
  if (!state.data.homed || state.data.faulted || !stage_is_recorded(stage)) {
    return false;
  }
  state.data.pid_active = true;
  state.data.target_stage = stage;
  state.data.target_deg = state.stages[stage - 1];
  state.manual_power = 0;
  reset_controller_terms();
  return true;
}

bool set_target_position_deg(double position_deg) {
  if (!state.data.homed || state.data.faulted || !std::isfinite(position_deg) ||
      position_deg < 0.0 || position_deg > state.stages.back()) {
    return false;
  }
  state.data.pid_active = true;
  state.data.target_stage = 0;
  state.data.target_deg = position_deg;
  state.manual_power = 0;
  reset_controller_terms();
  return true;
}

void disable_pid() {
  state.data.pid_active = false;
  state.data.target_stage = 0;
  state.manual_power = 0;
  reset_controller_terms();
  command_power(0);
}

void clear_fault() {
  state.data.faulted = false;
  reset_controller_terms();
}

void update() {
  const std::uint32_t now_ms = pros::millis();
  const double dt_s = std::clamp(
      static_cast<double>(now_ms - state.previous_ms) / 1000.0, 0.001, 0.100);
  state.previous_ms = now_ms;

  state.data.sensor_ok = slider_rotation_sensor.is_installed();
  if (!state.data.sensor_ok) {
    state.data.faulted = true;
    state.data.homed = false;
    command_power(0);
    return;
  }

  state.data.raw_centidegrees = slider_rotation_sensor.get_position();
  state.data.position_deg = read_position_deg(state.data.raw_centidegrees);
  const double raw_velocity_deg_s =
      (state.data.position_deg - state.previous_position) / dt_s;
  state.data.velocity_deg_s += kVelocityFilterAlpha *
      (raw_velocity_deg_s - state.data.velocity_deg_s);
  state.previous_position = state.data.position_deg;

  state.data.nearest_stage = 0;
  double nearest_error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < state.stages.size(); ++i) {
    if (!std::isfinite(state.stages[i])) continue;
    const double error = std::fabs(state.data.position_deg - state.stages[i]);
    if (error < nearest_error) {
      nearest_error = error;
      state.data.nearest_stage = static_cast<int>(i + 1);
    }
  }

  const std::int32_t current_ma = average_current_ma();
  const bool seeking_bottom =
      state.manual_power < 0 ||
      (state.data.pid_active && state.data.target_deg <= 0.0);
  const bool guarded_bottom =
      seeking_bottom &&
      std::fabs(state.data.position_deg) <= kBottomRezeroWindowDeg &&
      std::fabs(state.data.velocity_deg_s) <= kBottomStillVelocityDegS &&
      current_ma >= kBottomLoadedCurrentMa;
  if (guarded_bottom) {
    if (state.bottom_candidate_ms == 0) state.bottom_candidate_ms = now_ms;
    if (now_ms - state.bottom_candidate_ms >= kBottomConfirmMs) {
      command_power(0);
      slider_rotation_sensor.reset_position();
      state.data.raw_centidegrees = 0;
      state.data.position_deg = 0.0;
      state.previous_position = 0.0;
      state.manual_power = 0;
      state.data.pid_active = false;
      state.data.target_stage = 0;
      state.data.target_deg = 0.0;
      state.data.error_deg = 0.0;
      state.data.output_mv = 0.0;
      state.bottom_candidate_ms = 0;
      std::printf("CASCADE_HOME mode=guarded_bottom_rezero current_ma=%ld\n",
                  static_cast<long>(current_ma));
      std::fflush(stdout);
      return;
    }
  } else {
    state.bottom_candidate_ms = 0;
  }

  if (!state.data.homed || state.data.faulted) {
    state.data.output_mv = 0.0;
    command_power(0);
    return;
  }

  if (!state.data.pid_active) {
    int safe_power = state.manual_power;
    if (state.data.position_deg >= kCalibrationTravelLimitDeg && safe_power > 0)
      safe_power = 0;
    state.data.output_mv = safe_power * (12000.0 / 127.0);
    command_power(safe_power);

    if (safe_power == 0 && std::fabs(state.data.velocity_deg_s) < 2.0) {
      if (state.still_candidate_ms == 0) state.still_candidate_ms = now_ms;
      const bool new_candidate =
          !std::isfinite(state.last_stop_candidate_deg) ||
          std::fabs(state.data.position_deg - state.last_stop_candidate_deg) >
              20.0;
      if (new_candidate && now_ms - state.still_candidate_ms >= 500) {
        state.last_stop_candidate_deg = state.data.position_deg;
        std::printf(
            "CASCADE_STOP_CANDIDATE position_deg=%.2f raw_centideg=%ld\n",
            state.data.position_deg,
            static_cast<long>(state.data.raw_centidegrees));
        std::fflush(stdout);
      }
    } else {
      state.still_candidate_ms = 0;
    }
    return;
  }

  state.data.error_deg = state.data.target_deg - state.data.position_deg;
  if (std::fabs(state.data.error_deg) < kIntegralZoneDeg) {
    state.integral = std::clamp(
        state.integral + state.data.error_deg * dt_s,
        -kIntegralLimitDegS, kIntegralLimitDegS);
  } else {
    state.integral = 0.0;
  }
  // Differentiate the filtered measurement instead of the error so target
  // changes do not create a derivative kick.
  const double derivative = -state.data.velocity_deg_s;
  state.previous_error = state.data.error_deg;
  state.data.output_mv =
      kPmvPerDeg * state.data.error_deg +
      kImvPerDegS * state.integral +
      kDmvPerDegPerS * derivative + kGravityMv;
  const double upward_voltage_limit =
      state.data.target_deg <= state.stages[1]
          ? kShortStageUpwardLimitMv
          : kPidVoltageLimitMv;
  state.data.output_mv = std::clamp(state.data.output_mv,
                                    -kPidDownwardVoltageLimitMv,
                                    upward_voltage_limit);
  if (state.data.error_deg > kPositionToleranceDeg &&
      state.data.output_mv > 0.0 &&
      state.data.output_mv < kMinimumUpwardPidMv) {
    state.data.output_mv = kMinimumUpwardPidMv;
  }
  if (state.data.error_deg < -kPositionToleranceDeg &&
      state.data.output_mv < 0.0 &&
      state.data.output_mv > -kMinimumDownwardPidMv) {
    state.data.output_mv = -kMinimumDownwardPidMv;
  }

  const double max_position = kUpperSoftLimitDeg;
  if (state.data.position_deg <= 0.0 && state.data.output_mv < 0.0)
    state.data.output_mv = 0.0;
  if (state.data.position_deg >= max_position && state.data.output_mv > 0.0)
    state.data.output_mv = 0.0;

  const bool stalled = std::fabs(state.data.output_mv) >= kStallOutputMv &&
                       std::fabs(state.data.velocity_deg_s) < kStallVelocityDegS;
  if (stalled) {
    if (state.stall_candidate_ms == 0) state.stall_candidate_ms = now_ms;
    if (now_ms - state.stall_candidate_ms >= kStallTimeoutMs) {
      state.data.faulted = true;
      state.data.pid_active = false;
      state.data.output_mv = 0.0;
      command_power(0);
      std::printf("CASCADE_FAULT reason=stall position_deg=%.2f\n",
                  state.data.position_deg);
      std::fflush(stdout);
      return;
    }
  } else {
    state.stall_candidate_ms = 0;
  }

  command_voltage(state.data.output_mv);
}

Snapshot snapshot() { return state.data; }

void print_telemetry_if_due(std::uint32_t period_ms) {
  const std::uint32_t now_ms = pros::millis();
  if (now_ms - state.last_telemetry_ms < period_ms) return;
  state.last_telemetry_ms = now_ms;
  std::printf(
      "CASCADE_TRACK t=%lu homed=%d sensor=%d fault=%d pid=%d "
      "target_stage=%d nearest_stage=%d raw_centideg=%ld position_deg=%.2f "
      "velocity_deg_s=%.2f target_deg=%.2f error_deg=%.2f output_mv=%.0f "
      "right_current_ma=%ld left_current_ma=%ld right_temp_c=%.1f "
      "left_temp_c=%.1f\n",
      static_cast<unsigned long>(now_ms), static_cast<int>(state.data.homed),
      static_cast<int>(state.data.sensor_ok),
      static_cast<int>(state.data.faulted),
      static_cast<int>(state.data.pid_active), state.data.target_stage,
      state.data.nearest_stage,
      static_cast<long>(state.data.raw_centidegrees), state.data.position_deg,
      state.data.velocity_deg_s, state.data.target_deg, state.data.error_deg,
      state.data.output_mv, static_cast<long>(slider_right.get_current_draw()),
      static_cast<long>(slider_left.get_current_draw()),
      slider_right.get_temperature(), slider_left.get_temperature());
  std::fflush(stdout);
}

}  // namespace cascade_lift
