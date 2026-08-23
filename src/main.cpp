#include "main.h"
#include "localization_config.hpp"
#include "pid_autotune.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>

// Drivetrain directions are calibrated so positive controller commands move
// the robot forward.
Drive chassis({17, 18},
              {-11, -13},
              6,
              localization::kDriveWheelDiameterIn,
              localization::kDriveRpm,
              localization::kDriveExternalRatio);
pros::Distance distance_1(localization::kForwardDistancePort);
pros::Gps gps_7(localization::kGpsPort);
pros::Rotation horizontal_odom(5);

namespace {
constexpr std::uint32_t SAMPLE_PERIOD_MS = 20;
constexpr std::uint32_t TELEMETRY_PERIOD_MS = 50;
constexpr bool RUN_STARTUP_LIDAR_CALIBRATION = false;
constexpr bool RUN_STARTUP_FORWARD_CALIBRATION = false;
// Diagnostic-only. Enable for one supervised boot, then restore false.
constexpr bool RUN_STARTUP_AI_VISION_SCAN = false;
// Diagnostic motion is opt-in for one supervised boot only. Tournament and
// normal telemetry images must never move the robot during startup.
constexpr bool RUN_STARTUP_SCAN_RECOVERY = false;
// One supervised tether-managed acceptance boot only. Restore false after the
// route and upload a no-motion production image.
constexpr bool RUN_STARTUP_LONG_FUSION_ROUTE = false;
// One supervised diagnostic boot; restore false immediately after the test.
constexpr bool RUN_STARTUP_DISTANCE_SWEEP_TEST = false;
// One supervised diagnostic boot; restore false immediately after the test.
constexpr bool RUN_STARTUP_ROTATION_SWEEP_TEST = false;
// One supervised obstacle-aware route; restore false immediately afterward.
constexpr bool RUN_STARTUP_GPS_SAFE_ROUTE = false;
// One-shot, pose-gated move away from the left wall. The pose gate prevents a
// reboot from repeating the translation after a successful recovery.
constexpr bool RUN_STARTUP_WALL_RECOVERY = false;
constexpr int CONTROLLER_DRIVE_DEADBAND = 5;

struct RuntimePoseEditor {
  bool was_active = false;
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_deg = 0.0;
  std::uint32_t last_display_ms = 0;
};

RuntimePoseEditor runtime_pose_editor;

double normalized_heading(double heading_deg) {
  while (heading_deg >= 360.0) heading_deg -= 360.0;
  while (heading_deg < 0.0) heading_deg += 360.0;
  return heading_deg;
}

bool update_runtime_pose_editor(pros::Controller& master) {
  const bool active =
      master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
      master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
  if (active && !runtime_pose_editor.was_active) {
    localization_get_runtime_start_pose(runtime_pose_editor.x_in,
                                        runtime_pose_editor.y_in,
                                        runtime_pose_editor.heading_deg);
    master.clear();
  }
  runtime_pose_editor.was_active = active;
  if (!active) return false;

  constexpr double kPositionStepIn = 0.5;
  constexpr double kFineHeadingStepDeg = 1.0;
  constexpr double kCoarseHeadingStepDeg = 15.0;
  constexpr double kWallIn = localization::kPhysicalWallHalfSpanIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    runtime_pose_editor.x_in -= kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    runtime_pose_editor.x_in += kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    runtime_pose_editor.y_in -= kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    runtime_pose_editor.y_in += kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    runtime_pose_editor.heading_deg -= kFineHeadingStepDeg;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    runtime_pose_editor.heading_deg += kFineHeadingStepDeg;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    runtime_pose_editor.heading_deg -= kCoarseHeadingStepDeg;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    runtime_pose_editor.heading_deg += kCoarseHeadingStepDeg;
  runtime_pose_editor.x_in = std::clamp(runtime_pose_editor.x_in, -kWallIn, kWallIn);
  runtime_pose_editor.y_in = std::clamp(runtime_pose_editor.y_in, -kWallIn, kWallIn);
  runtime_pose_editor.heading_deg = normalized_heading(runtime_pose_editor.heading_deg);

  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    localization_set_runtime_start_pose(runtime_pose_editor.x_in,
                                        runtime_pose_editor.y_in,
                                        runtime_pose_editor.heading_deg);
    master.rumble(".");
  }
  const std::uint32_t now = pros::millis();
  if (now - runtime_pose_editor.last_display_ms >= 100) {
    master.print(0, 0, "POSE EDIT X+Y held ");
    master.print(1, 0, "X%5.1f Y%5.1f    ", runtime_pose_editor.x_in,
                 runtime_pose_editor.y_in);
    master.print(2, 0, "H%5.1f A=SAVE    ", runtime_pose_editor.heading_deg);
    runtime_pose_editor.last_display_ms = now;
  }
  return true;
}

volatile bool opcontrol_auton_running = false;

int apply_controller_deadband(int value) {
  return std::abs(value) <= CONTROLLER_DRIVE_DEADBAND ? 0 : value;
}

bool drive_positions_are_zeroed() {
  for (const auto& motor : chassis.left_motors) {
    const double position = motor.get_position();
    if (!std::isfinite(position) || std::fabs(position) > 10.0) return false;
  }
  for (const auto& motor : chassis.right_motors) {
    const double position = motor.get_position();
    if (!std::isfinite(position) || std::fabs(position) > 10.0) return false;
  }
  return true;
}

void run_distance_sweep_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr double kVelocityRpm = 15.0;
  constexpr std::uint32_t kTimeoutPerInchMs = 1800;
  constexpr std::array<int, 3> kTargetsIn = {2, 5, 10};
  pros::Gps gps(localization::kGpsPort);
  pros::Imu imu6(6);

  struct GpsSample {
    double x_m;
    double y_m;
    double heading_deg;
    double error_m;
  };
  auto sample_gps = [&]() {
    constexpr int kSamples = 20;
    GpsSample sample{0.0, 0.0, 0.0, 0.0};
    double heading_sin = 0.0;
    double heading_cos = 0.0;
    for (int i = 0; i < kSamples; ++i) {
      const auto position = gps.get_position();
      sample.x_m += position.x;
      sample.y_m += position.y;
      sample.error_m += gps.get_error();
      const double heading_rad = gps.get_heading() * kPi / 180.0;
      heading_sin += std::sin(heading_rad);
      heading_cos += std::cos(heading_rad);
      pros::delay(50);
    }
    sample.x_m /= kSamples;
    sample.y_m /= kSamples;
    sample.error_m /= kSamples;
    sample.heading_deg = std::atan2(heading_sin, heading_cos) * 180.0 / kPi;
    if (sample.heading_deg < 0.0) sample.heading_deg += 360.0;
    return sample;
  };
  auto motor_positions = [&]() {
    return std::array<double, 4>{
        chassis.left_motors[0].get_position(),
        chassis.left_motors[1].get_position(),
        chassis.right_motors[0].get_position(),
        chassis.right_motors[1].get_position(),
    };
  };
  auto average_delta_deg = [](const std::array<double, 4>& now,
                              const std::array<double, 4>& baseline) {
    double total = 0.0;
    for (std::size_t i = 0; i < now.size(); ++i) total += now[i] - baseline[i];
    return total / static_cast<double>(now.size());
  };
  auto stop = [&]() {
    for (auto& motor : chassis.left_motors) motor.move_velocity(0);
    for (auto& motor : chassis.right_motors) motor.move_velocity(0);
  };
  auto command = [&](double rpm) {
    for (auto& motor : chassis.left_motors) motor.move_velocity(rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(rpm);
  };
  auto heading_delta = [](double a, double b) {
    double delta = a - b;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
  };

  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  for (auto& motor : chassis.right_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  stop();

  const bool imu_installed = imu6.is_installed();
  if (imu_installed) {
    imu6.reset();
    const std::uint32_t calibration_started = pros::millis();
    while (imu6.is_calibrating() && pros::millis() - calibration_started < 5000) {
      pros::delay(20);
    }
  }
  printf("STRAIGHT_IMU_INIT port=6 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu6.get_status()));
  fflush(stdout);

  for (int target_in : kTargetsIn) {
    pros::lcd::print(6, "Sweep %din: sampling", target_in);
    const auto baseline = motor_positions();
    const GpsSample start_gps = sample_gps();
    const double start_imu_deg = imu6.get_rotation();
    double max_abs_imu_delta_deg = 0.0;
    const double target_deg =
        static_cast<double>(target_in) * 360.0 / kWheelCircumferenceIn;

    pros::lcd::print(6, "Sweep %din: BACK", target_in);
    const std::uint32_t back_started = pros::millis();
    while (pros::millis() - back_started <
               kTimeoutPerInchMs * static_cast<std::uint32_t>(target_in) &&
           average_delta_deg(motor_positions(), baseline) > -target_deg) {
      command(-kVelocityRpm);
      max_abs_imu_delta_deg = std::max(
          max_abs_imu_delta_deg, std::fabs(imu6.get_rotation() - start_imu_deg));
      pros::delay(10);
    }
    stop();
    pros::delay(300);
    const auto back_motors = motor_positions();
    const GpsSample back_gps = sample_gps();
    const double encoder_back_in =
        -average_delta_deg(back_motors, baseline) * kWheelCircumferenceIn / 360.0;
    const double gps_back_in =
        std::hypot(back_gps.x_m - start_gps.x_m,
                   back_gps.y_m - start_gps.y_m) * 39.37007874;
    printf("SWEEP target_in=%d phase=back encoder_in=%.3f gps_in=%.3f difference_in=%.3f gps_heading_delta=%.3f gps_error_in=%.3f imu_delta_deg=%.3f imu_max_abs_deg=%.3f\n",
           target_in, encoder_back_in, gps_back_in,
           gps_back_in - encoder_back_in,
           heading_delta(back_gps.heading_deg, start_gps.heading_deg),
           back_gps.error_m * 39.37007874,
           imu6.get_rotation() - start_imu_deg, max_abs_imu_delta_deg);
    fflush(stdout);

    pros::lcd::print(6, "Sweep %din: RETURN", target_in);
    const std::uint32_t return_started = pros::millis();
    while (pros::millis() - return_started <
               kTimeoutPerInchMs * static_cast<std::uint32_t>(target_in) &&
           average_delta_deg(motor_positions(), baseline) < -1.5) {
      command(kVelocityRpm);
      max_abs_imu_delta_deg = std::max(
          max_abs_imu_delta_deg, std::fabs(imu6.get_rotation() - start_imu_deg));
      pros::delay(10);
    }
    stop();
    pros::delay(300);
    const auto final_motors = motor_positions();
    const GpsSample final_gps = sample_gps();
    const double encoder_residual_in =
        average_delta_deg(final_motors, baseline) * kWheelCircumferenceIn / 360.0;
    const double gps_residual_in =
        std::hypot(final_gps.x_m - start_gps.x_m,
                   final_gps.y_m - start_gps.y_m) * 39.37007874;
    printf("SWEEP target_in=%d phase=return encoder_residual_in=%.3f gps_residual_in=%.3f gps_heading_delta=%.3f gps_error_in=%.3f imu_residual_deg=%.3f imu_max_abs_deg=%.3f\n",
           target_in, encoder_residual_in, gps_residual_in,
           heading_delta(final_gps.heading_deg, start_gps.heading_deg),
           final_gps.error_m * 39.37007874,
           imu6.get_rotation() - start_imu_deg, max_abs_imu_delta_deg);
    fflush(stdout);
    pros::delay(500);
  }
  pros::lcd::set_text(6, "Distance sweep DONE");
}

void run_rotation_sweep_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr std::array<double, 5> kTargetsDeg = {15.0, 30.0, 45.0, 60.0, 90.0};
  constexpr double kVelocityRpm = 12.0;
  pros::Gps gps(localization::kGpsPort);
  pros::Imu imu6(6);

  struct HeadingSample {
    double gps_deg;
    double imu_deg;
    double gps_x_m;
    double gps_y_m;
    double gps_error_m;
  };
  auto angle_delta = [](double a, double b) {
    double delta = a - b;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
  };
  auto sample_headings = [&]() {
    constexpr int kSamples = 20;
    double gps_sin = 0.0, gps_cos = 0.0;
    double imu_sin = 0.0, imu_cos = 0.0;
    HeadingSample result{0.0, 0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < kSamples; ++i) {
      const auto position = gps.get_position();
      const double gps_rad = gps.get_heading() * kPi / 180.0;
      const double imu_rad = imu6.get_heading() * kPi / 180.0;
      gps_sin += std::sin(gps_rad);
      gps_cos += std::cos(gps_rad);
      imu_sin += std::sin(imu_rad);
      imu_cos += std::cos(imu_rad);
      result.gps_x_m += position.x;
      result.gps_y_m += position.y;
      result.gps_error_m += gps.get_error();
      pros::delay(50);
    }
    result.gps_deg = std::atan2(gps_sin, gps_cos) * 180.0 / kPi;
    result.imu_deg = std::atan2(imu_sin, imu_cos) * 180.0 / kPi;
    if (result.gps_deg < 0.0) result.gps_deg += 360.0;
    if (result.imu_deg < 0.0) result.imu_deg += 360.0;
    result.gps_x_m /= kSamples;
    result.gps_y_m /= kSamples;
    result.gps_error_m /= kSamples;
    return result;
  };
  auto motor_positions = [&]() {
    return std::array<double, 4>{
        chassis.left_motors[0].get_position(),
        chassis.left_motors[1].get_position(),
        chassis.right_motors[0].get_position(),
        chassis.right_motors[1].get_position(),
    };
  };
  auto turn_motor_delta = [](const std::array<double, 4>& now,
                             const std::array<double, 4>& baseline) {
    const double left = ((now[0] - baseline[0]) + (now[1] - baseline[1])) / 2.0;
    const double right = ((now[2] - baseline[2]) + (now[3] - baseline[3])) / 2.0;
    return (left - right) / 2.0;
  };
  auto stop = [&]() {
    for (auto& motor : chassis.left_motors) motor.move_velocity(0);
    for (auto& motor : chassis.right_motors) motor.move_velocity(0);
  };
  auto command_turn = [&](double rpm) {
    for (auto& motor : chassis.left_motors) motor.move_velocity(rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(-rpm);
  };

  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  for (auto& motor : chassis.right_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  stop();
  const bool imu_installed = imu6.is_installed();
  if (imu_installed) {
    imu6.reset();
    const std::uint32_t calibration_started = pros::millis();
    while (imu6.is_calibrating() && pros::millis() - calibration_started < 5000) {
      pros::delay(20);
    }
  }
  printf("ROTATION_SWEEP_INIT imu_port=6 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu6.get_status()));
  fflush(stdout);

  for (double target_deg : kTargetsDeg) {
    pros::lcd::print(6, "Turn %.0f: sampling", target_deg);
    const auto baseline = motor_positions();
    const HeadingSample start = sample_headings();
    const double side_arc_target_in =
        target_deg * kPi / 180.0 * localization::kDriveTrackWidthIn / 2.0;
    const double motor_target_deg = side_arc_target_in * 360.0 / kWheelCircumferenceIn;

    pros::lcd::print(6, "Turn %.0f: CW", target_deg);
    const std::uint32_t turn_started = pros::millis();
    while (pros::millis() - turn_started < 8000 &&
           turn_motor_delta(motor_positions(), baseline) < motor_target_deg) {
      command_turn(kVelocityRpm);
      pros::delay(10);
    }
    stop();
    pros::delay(300);
    const auto peak_motors = motor_positions();
    const HeadingSample peak = sample_headings();
    const double motor_turn_deg = turn_motor_delta(peak_motors, baseline);
    const double encoder_heading_deg =
        (motor_turn_deg * kWheelCircumferenceIn / 360.0) * 2.0 /
        localization::kDriveTrackWidthIn * 180.0 / kPi;
    const double gps_heading_deg = angle_delta(peak.gps_deg, start.gps_deg);
    const double imu_heading_deg = imu_installed
        ? angle_delta(peak.imu_deg, start.imu_deg) : NAN;
    const double effective_track_in = std::fabs(gps_heading_deg) > 1.0
        ? (motor_turn_deg * kWheelCircumferenceIn / 360.0) * 2.0 /
              (std::fabs(gps_heading_deg) * kPi / 180.0)
        : NAN;
    printf("ROT_SWEEP target_deg=%.0f phase=turn encoder_deg=%.3f gps_deg=%.3f imu_deg=%.3f encoder_minus_gps_deg=%.3f effective_track_in=%.3f gps_error_in=%.3f\n",
           target_deg, encoder_heading_deg, gps_heading_deg, imu_heading_deg,
           encoder_heading_deg - std::fabs(gps_heading_deg), effective_track_in,
           peak.gps_error_m * 39.37007874);
    fflush(stdout);

    pros::lcd::print(6, "Turn %.0f: RETURN", target_deg);
    const std::uint32_t return_started = pros::millis();
    while (pros::millis() - return_started < 8000 &&
           turn_motor_delta(motor_positions(), baseline) > 1.5) {
      command_turn(-kVelocityRpm);
      pros::delay(10);
    }
    stop();
    pros::delay(300);
    const auto final_motors = motor_positions();
    const HeadingSample final = sample_headings();
    const double gps_position_residual_in =
        std::hypot(final.gps_x_m - start.gps_x_m,
                   final.gps_y_m - start.gps_y_m) * 39.37007874;
    printf("ROT_SWEEP target_deg=%.0f phase=return encoder_residual_motor_deg=%.3f gps_heading_residual_deg=%.3f imu_heading_residual_deg=%.3f gps_position_residual_in=%.3f\n",
           target_deg, turn_motor_delta(final_motors, baseline),
           angle_delta(final.gps_deg, start.gps_deg),
           imu_installed ? angle_delta(final.imu_deg, start.imu_deg) : NAN,
           gps_position_residual_in);
    fflush(stdout);
    pros::delay(500);
  }
  pros::lcd::set_text(6, "Rotation sweep DONE");
}

void run_wall_recovery() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kInchesPerMeter = 39.37007874015748;
  constexpr double kTargetRobotHeadingCwDeg = 90.0;
  constexpr double kDriveDistanceIn = 10.0;
  constexpr double kMeasuredEncoderScale = 0.887;
  constexpr double kTurnRpm = 12.0;
  constexpr double kDriveRpm = 15.0;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  pros::Gps gps(localization::kGpsPort);
  pros::Imu imu(6);

  struct RobotPose {
    double x_in;
    double y_in;
    double heading_cw_deg;
    double error_in;
    double position_span_in;
  };
  auto normalize = [](double degrees) {
    while (degrees >= 360.0) degrees -= 360.0;
    while (degrees < 0.0) degrees += 360.0;
    return degrees;
  };
  auto signed_delta = [](double target, double current) {
    double delta = target - current;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
  };
  auto sample_pose = [&]() {
    constexpr int kSamples = 25;
    double sensor_x_in = 0.0;
    double sensor_y_in = 0.0;
    double heading_sin = 0.0;
    double heading_cos = 0.0;
    double error_in = 0.0;
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (int i = 0; i < kSamples; ++i) {
      const auto position = gps.get_position();
      const double x_in = position.x * kInchesPerMeter;
      const double y_in = position.y * kInchesPerMeter;
      const double heading_rad = gps.get_heading() * kPi / 180.0;
      sensor_x_in += x_in;
      sensor_y_in += y_in;
      heading_sin += std::sin(heading_rad);
      heading_cos += std::cos(heading_rad);
      error_in += gps.get_error() * kInchesPerMeter;
      min_x = std::min(min_x, x_in);
      max_x = std::max(max_x, x_in);
      min_y = std::min(min_y, y_in);
      max_y = std::max(max_y, y_in);
      pros::delay(40);
    }
    sensor_x_in /= kSamples;
    sensor_y_in /= kSamples;
    error_in /= kSamples;
    const double sensor_heading_cw_deg = normalize(
        std::atan2(heading_sin, heading_cos) * 180.0 / kPi);
    const double robot_heading_cw_deg = normalize(
        sensor_heading_cw_deg - localization::kGpsSensorHeadingOffsetCwDeg);
    const double heading_rad = robot_heading_cw_deg * kPi / 180.0;
    const double forward_x = std::sin(heading_rad);
    const double forward_y = std::cos(heading_rad);
    const double right_x = std::cos(heading_rad);
    const double right_y = -std::sin(heading_rad);
    const double center_x = sensor_x_in -
        (localization::kGpsRightOffsetIn * right_x +
         localization::kGpsForwardOffsetIn * forward_x);
    const double center_y = sensor_y_in -
        (localization::kGpsRightOffsetIn * right_y +
         localization::kGpsForwardOffsetIn * forward_y);
    return RobotPose{center_x, center_y, robot_heading_cw_deg, error_in,
                     std::hypot(max_x - min_x, max_y - min_y)};
  };
  auto stop = [&]() {
    for (auto& motor : chassis.left_motors) motor.move_velocity(0);
    for (auto& motor : chassis.right_motors) motor.move_velocity(0);
  };
  auto command = [&](double left_rpm, double right_rpm) {
    for (auto& motor : chassis.left_motors) motor.move_velocity(left_rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(right_rpm);
  };
  auto average_motor_delta = [](const std::array<double, 4>& baseline) {
    const std::array<double, 4> now = {
        chassis.left_motors[0].get_position(),
        chassis.left_motors[1].get_position(),
        chassis.right_motors[0].get_position(),
        chassis.right_motors[1].get_position(),
    };
    double total = 0.0;
    for (std::size_t i = 0; i < now.size(); ++i) total += now[i] - baseline[i];
    return total / static_cast<double>(now.size());
  };

  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  for (auto& motor : chassis.right_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  stop();
  if (!gps.is_installed() || !imu.is_installed()) {
    printf("WALL_RECOVERY abort=sensor_missing gps=%d imu=%d\n",
           static_cast<int>(gps.is_installed()),
           static_cast<int>(imu.is_installed()));
    fflush(stdout);
    return;
  }

  const RobotPose start = sample_pose();
  const bool start_gate = start.x_in >= -53.0 && start.x_in <= -43.0 &&
                          start.y_in >= -12.0 && start.y_in <= 5.0 &&
                          start.heading_cw_deg >= 130.0 &&
                          start.heading_cw_deg <= 165.0 &&
                          start.error_in <= 2.0 &&
                          start.position_span_in <= 2.0;
  printf("WALL_RECOVERY start x=%.2f y=%.2f heading_cw=%.2f error=%.2f span=%.2f gate=%d\n",
         start.x_in, start.y_in, start.heading_cw_deg, start.error_in,
         start.position_span_in, static_cast<int>(start_gate));
  fflush(stdout);
  if (!start_gate) {
    pros::lcd::set_text(6, "Recovery pose ABORT");
    return;
  }

  imu.reset();
  const std::uint32_t calibration_started = pros::millis();
  while (imu.is_calibrating() && pros::millis() - calibration_started < 5000)
    pros::delay(20);
  if (imu.is_calibrating() || imu.get_status() == pros::ImuStatus::error) {
    printf("WALL_RECOVERY abort=imu_calibration status=%d\n",
           static_cast<int>(imu.get_status()));
    fflush(stdout);
    return;
  }

  const double turn_target_deg = signed_delta(
      kTargetRobotHeadingCwDeg, start.heading_cw_deg);
  const std::uint32_t turn_started = pros::millis();
  while (pros::millis() - turn_started < 10000 &&
         imu.get_rotation() > turn_target_deg) {
    command(-kTurnRpm, kTurnRpm);
    pros::delay(10);
  }
  stop();
  pros::delay(350);
  const double turn_result_deg = imu.get_rotation();
  const RobotPose after_turn = sample_pose();
  const double turn_position_shift = std::hypot(
      after_turn.x_in - start.x_in, after_turn.y_in - start.y_in);
  const bool turn_ok = std::fabs(turn_result_deg - turn_target_deg) <= 3.0 &&
                       after_turn.error_in <= 2.0 &&
                       after_turn.position_span_in <= 2.0 &&
                       turn_position_shift <= 4.0;
  printf("WALL_RECOVERY turn target=%.2f imu=%.2f x=%.2f y=%.2f heading_cw=%.2f shift=%.2f error=%.2f ok=%d\n",
         turn_target_deg, turn_result_deg, after_turn.x_in, after_turn.y_in,
         after_turn.heading_cw_deg, turn_position_shift, after_turn.error_in,
         static_cast<int>(turn_ok));
  fflush(stdout);
  if (!turn_ok) {
    pros::lcd::set_text(6, "Recovery turn ABORT");
    return;
  }

  const std::array<double, 4> drive_baseline = {
      chassis.left_motors[0].get_position(),
      chassis.left_motors[1].get_position(),
      chassis.right_motors[0].get_position(),
      chassis.right_motors[1].get_position(),
  };
  const double encoder_target_deg =
      (kDriveDistanceIn / kMeasuredEncoderScale) * 360.0 /
      kWheelCircumferenceIn;
  bool drive_quality_ok = true;
  const std::uint32_t drive_started = pros::millis();
  while (pros::millis() - drive_started < 9000 &&
         average_motor_delta(drive_baseline) < encoder_target_deg) {
    const double heading_error_deg = turn_target_deg - imu.get_rotation();
    const double turn_rpm = std::clamp(heading_error_deg * 0.25, -3.0, 3.0);
    command(kDriveRpm + turn_rpm, kDriveRpm - turn_rpm);
    if (gps.get_error() * kInchesPerMeter > 2.5 ||
        std::fabs(heading_error_deg) > 6.0) {
      drive_quality_ok = false;
      break;
    }
    pros::delay(10);
  }
  stop();
  pros::delay(350);
  const double encoder_deg = average_motor_delta(drive_baseline);
  const RobotPose final_pose = sample_pose();
  const double gps_distance_in = std::hypot(
      final_pose.x_in - after_turn.x_in, final_pose.y_in - after_turn.y_in);
  const bool drive_ok = drive_quality_ok &&
                        encoder_deg >= encoder_target_deg - 3.0 &&
                        final_pose.error_in <= 2.0;
  printf("WALL_RECOVERY drive encoder_deg=%.2f target_deg=%.2f gps_distance=%.2f final_x=%.2f final_y=%.2f heading_cw=%.2f error=%.2f ok=%d\n",
         encoder_deg, encoder_target_deg, gps_distance_in, final_pose.x_in,
         final_pose.y_in, final_pose.heading_cw_deg, final_pose.error_in,
         static_cast<int>(drive_ok));
  fflush(stdout);
  pros::lcd::print(6, "Recovery %s x%.0f y%.0f",
                   drive_ok ? "OK" : "FAIL", final_pose.x_in,
                   final_pose.y_in);
}

struct DistanceReading {
  long mm;
  long confidence;
  bool installed;
};

DistanceReading read_sensor(pros::Distance& sensor) {
  errno = 0;
  DistanceReading reading{
      static_cast<long>(sensor.get_distance()),
      static_cast<long>(sensor.get_confidence()),
      sensor.is_installed(),
  };
  return reading;
}

void move_intake(int power) {
  upper_intake.move(-power);
}

void print_distance_frame() {
  static std::uint32_t sample = 0;
  static std::uint32_t last_frame_ms = 0;
  const std::uint32_t now = pros::millis();
  if (last_frame_ms != 0 && now - last_frame_ms < TELEMETRY_PERIOD_MS) return;
  last_frame_ms = now;
  const DistanceReading forward_distance = read_sensor(distance_1);
  const auto gps_position = gps_7.get_position();
  const double gps_heading_deg = gps_7.get_heading();
  const double gps_error_m = gps_7.get_error();

  printf(
      "D4 s=%lu t=%lu "
      "p1=%ld,%ld,%d "
      "m17=%.1f m18=%.1f m11=%.1f m13=%.1f h5=%ld "
      "imu=%.2f rawimu=%.2f imust=%d "
      "gps7=%.4f,%.4f,%.2f,%.4f,%d errno=%d\n",
      static_cast<unsigned long>(sample++),
      static_cast<unsigned long>(now),
      forward_distance.mm,
      forward_distance.confidence,
      static_cast<int>(forward_distance.installed),
      chassis.left_motors[0].get_position(),
      chassis.left_motors[1].get_position(),
      chassis.right_motors[0].get_position(),
      chassis.right_motors[1].get_position(),
      static_cast<long>(horizontal_odom.get_position()),
      chassis.drive_imu_get(),
      chassis.imu.get_rotation(),
      static_cast<int>(chassis.imu.get_status()),
      gps_position.x,
      gps_position.y,
      gps_heading_deg,
      gps_error_m,
      static_cast<int>(gps_7.is_installed()),
      errno);
  fflush(stdout);

  if (sample % 5 == 0) {
    pros::lcd::print(0, "P1 FWD %4ldmm c%2ld %s", forward_distance.mm,
                     forward_distance.confidence,
                     forward_distance.installed ? "ok" : "no");
    pros::lcd::print(1, "GPS P7 e%.3fm", gps_error_m);
    pros::lcd::set_text(2, "IMU P6 / AI P8");
    pros::lcd::set_text(3, "P9 left slider");
    const auto& vision = ai_vision_shadow_snapshot();
    pros::lcd::print(4, "AI P%u tag=%d %s",
                     static_cast<unsigned>(vision.port),
                     vision.tag_id,
                     vision.tag_valid ? "VALID" : vision.reason);
    master.print(2, 0, "AI P%u T%d %-3s",
                 static_cast<unsigned>(vision.port),
                 vision.tag_id,
                 vision.tag_valid ? "YES" : "NO");
  }
}

void start_opcontrol_auton() {
  if (opcontrol_auton_running) {
    return;
  }

  opcontrol_auton_running = true;
  pros::Task::create([] {
    pros::lcd::set_text(6, "B+Down auton");
    move_intake(0);
    simple_goal_avoidance_auton();
    chassis.drive_set(0, 0);
    move_intake(0);
    opcontrol_auton_running = false;
    pros::lcd::set_text(6, "Auton done");
  }, "opcontrol auton");
}

void start_fusion_test_auton() {
  if (opcontrol_auton_running) {
    return;
  }

  opcontrol_auton_running = true;
  pros::Task::create([] {
    pros::lcd::set_text(6, "X+Down real fusion");
    move_intake(0);
    fusion_test_auton();
    chassis.drive_set(0, 0);
    move_intake(0);
    opcontrol_auton_running = false;
    pros::lcd::set_text(6, "Fusion test done");
  }, "fusion test");
}

void start_pid_autotune() {
  if (opcontrol_auton_running) {
    return;
  }

  opcontrol_auton_running = true;
  pros::Task::create([] {
    pros::lcd::set_text(7, "X+Up PID autotune");
    move_intake(0);
    chassis.drive_set(0, 0);
    const bool success = pid_autotune_auton();
    chassis.drive_set(0, 0);
    move_intake(0);
    opcontrol_auton_running = false;
    std::printf("PID autotune %s\n", success ? "completed" : "FAILED");
    std::fflush(stdout);
    pros::lcd::set_text(
        7, success ? "PID autotune complete" : "PID autotune FAILED");
  }, "pid autotune");
}

void stop_drive_velocity() {
  for (auto& motor : chassis.left_motors) motor.move_velocity(0);
  for (auto& motor : chassis.right_motors) motor.move_velocity(0);
}

bool ai_vision_bounded_startup_scan() {
  constexpr double kLimitFromStartDeg = 32.0;
  constexpr double kTurnVelocityRpm = 8.0;
  constexpr double kTargetToleranceDeg = 1.2;
  constexpr std::uint32_t kLegTimeoutMs = 24000;
  constexpr std::uint32_t kStallTimeoutMs = 2200;
  constexpr int kRequiredValidFrames = 3;
  constexpr std::array<double, 3> kTargetsFromStartDeg{30.0, -30.0, 0.0};

  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  stop_drive_velocity();
  const double start_imu_deg = chassis.drive_imu_get();
  if (!std::isfinite(start_imu_deg)) {
    std::printf("VISION_SCAN event=abort reason=imu_invalid\n");
    std::fflush(stdout);
    return false;
  }

  std::printf(
      "VISION_SCAN event=start start_imu=%.2f limit=%.1f velocity_rpm=%.1f "
      "translation_limit_in=0\n",
      start_imu_deg, kLimitFromStartDeg, kTurnVelocityRpm);
  std::fflush(stdout);
  bool found = false;
  double found_delta_deg = NAN;
  int consecutive_valid = 0;

  for (double target_delta_deg : kTargetsFromStartDeg) {
    const std::uint32_t leg_start_ms = pros::millis();
    std::uint32_t last_motion_ms = leg_start_ms;
    std::uint32_t last_log_ms = 0;
    double last_delta_deg = chassis.drive_imu_get() - start_imu_deg;
    bool leg_done = false;
    while (pros::millis() - leg_start_ms < kLegTimeoutMs) {
      const double current_imu_deg = chassis.drive_imu_get();
      if (!std::isfinite(current_imu_deg)) {
        std::printf("VISION_SCAN event=abort reason=imu_invalid_motion\n");
        stop_drive_velocity();
        std::fflush(stdout);
        return false;
      }
      const double delta_deg = current_imu_deg - start_imu_deg;
      if (std::fabs(delta_deg) > kLimitFromStartDeg) {
        std::printf("VISION_SCAN event=abort reason=angle_guard delta=%.2f\n",
                    delta_deg);
        stop_drive_velocity();
        std::fflush(stdout);
        return false;
      }

      ai_vision_shadow_update();
      print_distance_frame();
      const auto& vision = ai_vision_shadow_snapshot();
      consecutive_valid = vision.tag_valid ? consecutive_valid + 1 : 0;
      if (!found && consecutive_valid >= kRequiredValidFrames) {
        found = true;
        found_delta_deg = delta_deg;
        stop_drive_velocity();
        std::printf(
            "VISION_SCAN event=tag_found delta=%.2f tag=%d bearing=%.2f "
            "area=%.1f geometry_age=%lu\n",
            found_delta_deg, vision.tag_id, vision.bearing_deg,
            vision.area_px2,
            static_cast<unsigned long>(vision.geometry_age_ms));
        std::fflush(stdout);
        const std::uint32_t hold_start_ms = pros::millis();
        while (pros::millis() - hold_start_ms < 1500) {
          ai_vision_shadow_update();
          print_distance_frame();
          pros::delay(SAMPLE_PERIOD_MS);
        }
        target_delta_deg = 0.0;
      }

      const double error_deg = target_delta_deg - delta_deg;
      if (std::fabs(error_deg) <= kTargetToleranceDeg) {
        stop_drive_velocity();
        leg_done = true;
        break;
      }
      const double command_rpm = error_deg > 0.0 ? kTurnVelocityRpm
                                                 : -kTurnVelocityRpm;
      for (auto& motor : chassis.left_motors) motor.move_velocity(command_rpm);
      for (auto& motor : chassis.right_motors) motor.move_velocity(-command_rpm);

      const std::uint32_t now = pros::millis();
      if (std::fabs(delta_deg - last_delta_deg) >= 0.20) {
        last_delta_deg = delta_deg;
        last_motion_ms = now;
      } else if (now - last_motion_ms > kStallTimeoutMs) {
        std::printf("VISION_SCAN event=abort reason=stall delta=%.2f\n",
                    delta_deg);
        stop_drive_velocity();
        std::fflush(stdout);
        return false;
      }
      if (now - last_log_ms >= 500) {
        std::printf(
            "VISION_SCAN event=motion target=%.1f delta=%.2f valid=%d "
            "tag=%d reason=%s\n",
            target_delta_deg, delta_deg, static_cast<int>(vision.tag_valid),
            vision.tag_id, vision.reason);
        std::fflush(stdout);
        last_log_ms = now;
      }
      pros::delay(SAMPLE_PERIOD_MS);
    }
    stop_drive_velocity();
    if (!leg_done) {
      std::printf("VISION_SCAN event=abort reason=leg_timeout target=%.1f\n",
                  target_delta_deg);
      std::fflush(stdout);
      return false;
    }
    if (found) break;
  }

  // If a tag ended a leg early, return to the exact starting IMU heading.
  if (found && std::fabs(chassis.drive_imu_get() - start_imu_deg) >
                   kTargetToleranceDeg) {
    const std::uint32_t return_start_ms = pros::millis();
    while (pros::millis() - return_start_ms < kLegTimeoutMs) {
      const double delta_deg = chassis.drive_imu_get() - start_imu_deg;
      if (!std::isfinite(delta_deg) || std::fabs(delta_deg) > kLimitFromStartDeg) {
        stop_drive_velocity();
        std::printf("VISION_SCAN event=abort reason=return_guard delta=%.2f\n",
                    delta_deg);
        std::fflush(stdout);
        return false;
      }
      if (std::fabs(delta_deg) <= kTargetToleranceDeg) break;
      const double command_rpm = delta_deg > 0.0 ? -kTurnVelocityRpm
                                                 : kTurnVelocityRpm;
      for (auto& motor : chassis.left_motors) motor.move_velocity(command_rpm);
      for (auto& motor : chassis.right_motors) motor.move_velocity(-command_rpm);
      ai_vision_shadow_update();
      print_distance_frame();
      pros::delay(SAMPLE_PERIOD_MS);
    }
  }
  stop_drive_velocity();
  const double final_delta_deg = chassis.drive_imu_get() - start_imu_deg;
  std::printf(
      "VISION_SCAN event=done found=%d found_delta=%.2f final_delta=%.2f\n",
      static_cast<int>(found), found_delta_deg, final_delta_deg);
  std::fflush(stdout);
  return found && std::fabs(final_delta_deg) <= 2.0;
}

bool recover_scan_start_heading() {
  constexpr double kTargetDeltaDeg = -8.0;
  constexpr double kAngleGuardDeg = 12.0;
  constexpr double kToleranceDeg = 0.55;
  constexpr std::uint32_t kTimeoutMs = 16000;
  constexpr std::uint32_t kStallTimeoutMs = 2500;
  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  stop_drive_velocity();
  const double start_imu_deg = chassis.drive_imu_get();
  std::uint32_t start_ms = pros::millis();
  std::uint32_t last_motion_ms = start_ms;
  double last_delta_deg = 0.0;
  bool success = false;
  std::printf("SCAN_RECOVERY event=start target=%.2f imu=%.2f\n",
              kTargetDeltaDeg, start_imu_deg);
  std::fflush(stdout);
  while (std::isfinite(start_imu_deg) && pros::millis() - start_ms < kTimeoutMs) {
    const double delta_deg = chassis.drive_imu_get() - start_imu_deg;
    if (!std::isfinite(delta_deg) || std::fabs(delta_deg) > kAngleGuardDeg) {
      std::printf("SCAN_RECOVERY event=abort reason=angle_or_imu delta=%.2f\n",
                  delta_deg);
      break;
    }
    const double error_deg = kTargetDeltaDeg - delta_deg;
    if (std::fabs(error_deg) <= kToleranceDeg) {
      success = true;
      break;
    }
    const double magnitude_rpm = std::clamp(std::fabs(error_deg) * 0.45, 2.5, 8.0);
    const double command_rpm = error_deg > 0.0 ? magnitude_rpm : -magnitude_rpm;
    for (auto& motor : chassis.left_motors) motor.move_velocity(command_rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(-command_rpm);
    const std::uint32_t now = pros::millis();
    if (std::fabs(delta_deg - last_delta_deg) >= 0.15) {
      last_delta_deg = delta_deg;
      last_motion_ms = now;
    } else if (now - last_motion_ms > kStallTimeoutMs) {
      std::printf("SCAN_RECOVERY event=abort reason=stall delta=%.2f\n",
                  delta_deg);
      break;
    }
    ai_vision_shadow_update();
    print_distance_frame();
    pros::delay(SAMPLE_PERIOD_MS);
  }
  stop_drive_velocity();
  pros::delay(250);
  const double final_delta_deg = chassis.drive_imu_get() - start_imu_deg;
  std::printf("SCAN_RECOVERY event=done success=%d final_delta=%.2f\n",
              static_cast<int>(success), final_delta_deg);
  std::fflush(stdout);
  return success && std::fabs(final_delta_deg - kTargetDeltaDeg) <= 1.2;
}
}  // namespace

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Forward Distance P1");
  pros::lcd::set_text(1, "GPS P7 / IMU P6");
  pros::lcd::set_text(2, "IMU calibrating...");
  chassis.drive_sensor_reset();
  bool imu_ready = false;
  if (chassis.imu.is_installed()) {
    chassis.imu.reset();
    const std::uint32_t imu_calibration_started = pros::millis();
    while (chassis.imu.is_calibrating() &&
           pros::millis() - imu_calibration_started < 5000) {
      pros::delay(20);
    }
    imu_ready = !chassis.imu.is_calibrating() &&
                chassis.imu.get_status() != pros::ImuStatus::error;
    if (imu_ready) chassis.drive_imu_reset(0.0);
  }
  pros::lcd::set_text(2, imu_ready ? "IMU ready" : "IMU check failed");
  horizontal_odom.reset_position();
  localization_telemetry_reset();
  ai_vision_shadow_initialize();
  pros::delay(500);

  printf("D4 init distance_port=1 direction=forward period_ms=%lu installed=%d\n",
         static_cast<unsigned long>(TELEMETRY_PERIOD_MS),
         static_cast<int>(distance_1.is_installed()));
  printf("IMU_INIT calibrated=%d raw_heading=%.2f\n",
         static_cast<int>(imu_ready),
         chassis.drive_imu_get());
  // One boot-time inventory makes wiring changes explicit and prevents a
  // stale configured port from silently disabling a localization sensor.
  for (std::uint8_t port = 1; port <= 21; ++port) {
    const int type = static_cast<int>(pros::c::get_plugged_type(port));
    if (type != 0) {
      printf("DEVICE_PORT port=%u type=%d\n", static_cast<unsigned>(port), type);
    }
  }
  fflush(stdout);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  pros::Task telemetry_task([] {
    while (true) {
      print_distance_frame();
      ai_vision_shadow_update();
      pros::delay(SAMPLE_PERIOD_MS);
    }
  });

  fusion_test_auton();

  while (true) {
    pros::delay(100);
  }
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  if (drive_positions_are_zeroed()) {
    localization_telemetry_reset();
  }
  if (RUN_STARTUP_WALL_RECOVERY) {
    pros::lcd::set_text(6, "Wall recovery in 10s");
    pros::delay(10000);
    run_wall_recovery();
  }
  if (RUN_STARTUP_DISTANCE_SWEEP_TEST) {
    pros::lcd::set_text(6, "Sweep starts in 3 sec");
    pros::delay(3000);
    run_distance_sweep_test();
  }
  if (RUN_STARTUP_ROTATION_SWEEP_TEST) {
    pros::lcd::set_text(6, "Turns start in 3 sec");
    pros::delay(3000);
    run_rotation_sweep_test();
  }
  if (RUN_STARTUP_GPS_SAFE_ROUTE) {
    pros::lcd::set_text(6, "GPS route in 5 sec");
    pros::delay(5000);
    gps_obstacle_aware_route_test();
  }
  if (RUN_STARTUP_LIDAR_CALIBRATION) {
    pros::lcd::set_text(6, "Cal starts in 5 sec");
    pros::delay(5000);
    localization_slow_rotation_calibration();
  }
  if (RUN_STARTUP_FORWARD_CALIBRATION) {
    pros::lcd::set_text(6, "Drive check in 5 sec");
    pros::delay(5000);
    localization_slow_forward_calibration();
  }
  if (RUN_STARTUP_SCAN_RECOVERY) {
    pros::lcd::set_text(6, "Scan recovery in 5 sec");
    pros::delay(5000);
    const bool recovery_ok = recover_scan_start_heading();
    pros::lcd::set_text(6, recovery_ok ? "Scan recovery done" : "Recovery FAILED");
  }
  if (RUN_STARTUP_AI_VISION_SCAN) {
    pros::lcd::set_text(6, "Vision scan in 5 sec");
    pros::delay(5000);
    const bool vision_scan_ok = ai_vision_bounded_startup_scan();
    pros::lcd::set_text(6, vision_scan_ok ? "Vision tag found" : "Vision scan no tag");
  }
  if (RUN_STARTUP_LONG_FUSION_ROUTE) {
    pros::lcd::set_text(6, "Long fusion in 10 sec");
    pros::delay(10000);
    fusion_test_auton();
    pros::lcd::set_text(6, "Long fusion done");
  }
  bool auton_combo_was_pressed = false;
  bool fusion_test_combo_was_pressed = false;
  bool pid_tune_combo_was_pressed = false;

  while (true) {
    const bool pose_editor_active = update_runtime_pose_editor(master);
    const bool auton_combo_pressed =
        !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (auton_combo_pressed && !auton_combo_was_pressed) {
      start_opcontrol_auton();
    }
    auton_combo_was_pressed = auton_combo_pressed;

    const bool fusion_test_combo_pressed =
        !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (fusion_test_combo_pressed && !fusion_test_combo_was_pressed) {
      start_fusion_test_auton();
    }
    fusion_test_combo_was_pressed = fusion_test_combo_pressed;

    const bool pid_tune_combo_pressed =
        !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    if (pid_tune_combo_pressed && !pid_tune_combo_was_pressed) {
      start_pid_autotune();
    }
    pid_tune_combo_was_pressed = pid_tune_combo_pressed;

    if (!opcontrol_auton_running) {
      constexpr double kOpcontrolDriveScale = 0.70;
      const int forward_power = pose_editor_active
          ? 0
          : apply_controller_deadband(
                master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      const int turn_power = pose_editor_active
          ? 0
          : -apply_controller_deadband(
                master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
      const int left_drive_power = static_cast<int>(
          std::clamp(forward_power + turn_power, -127, 127) *
          kOpcontrolDriveScale);
      const int right_drive_power = static_cast<int>(
          std::clamp(forward_power - turn_power, -127, 127) *
          kOpcontrolDriveScale);
      for (auto& motor : chassis.left_motors) {
        motor.move(left_drive_power);
      }
      for (auto& motor : chassis.right_motors) {
        motor.move(right_drive_power);
      }

      const int requested_intake_power = pose_editor_active ? 0 :
          master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)
              ? 127
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !auton_combo_pressed ? -127 : 0);
      move_intake(requested_intake_power);

      const int slider_power =
          master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
              ? 127
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? -127 : 0);
      slider_right.move(!pose_editor_active ? slider_power : 0);
      slider_left.move(!pose_editor_active ? slider_power : 0);

      if (!pose_editor_active) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
          clamp_piston.set_value(true);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
          clamp_piston.set_value(false);
        }
      }

      const int claw_wrist_power = pose_editor_active
          ? 0
          : master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
              ? 40
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ? -40 : 0);
      claw_arm.move(claw_wrist_power);
    }

    print_distance_frame();
    ai_vision_shadow_update();
    if (!opcontrol_auton_running) {
      localization_telemetry_update();
    }
    pros::delay(SAMPLE_PERIOD_MS);
  }
}
