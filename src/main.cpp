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
              0,
              localization::kDriveWheelDiameterIn,
              localization::kDriveRpm,
              localization::kDriveExternalRatio);
pros::Distance distance_6(6);
pros::Distance distance_7(7);
pros::Distance distance_8(8);
pros::Rotation horizontal_odom(5);

namespace {
constexpr std::array<std::uint8_t, 3> DISTANCE_PORTS = {6, 7, 8};
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

std::array<pros::Distance*, 3> distance_sensors = {
    &distance_6,
    &distance_7,
    &distance_8,
};

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
  std::array<DistanceReading, 3> readings{};

  for (std::size_t i = 0; i < distance_sensors.size(); ++i) {
    readings[i] = read_sensor(*distance_sensors[i]);
  }

  printf(
      "D4 s=%lu t=%lu "
      "p6=%ld,%ld,%d p7=%ld,%ld,%d p8=%ld,%ld,%d p9=%ld,%ld,%d "
      "m17=%.1f m18=%.1f m11=%.1f m13=%.1f h5=%ld "
      "imu=%.2f rawimu=%.2f imust=%d errno=%d\n",
      static_cast<unsigned long>(sample++),
      static_cast<unsigned long>(now),
      readings[0].mm,
      readings[0].confidence,
      static_cast<int>(readings[0].installed),
      readings[1].mm,
      readings[1].confidence,
      static_cast<int>(readings[1].installed),
      readings[2].mm,
      readings[2].confidence,
      static_cast<int>(readings[2].installed),
      -1L,
      0L,
      0,
      chassis.left_motors[0].get_position(),
      chassis.left_motors[1].get_position(),
      chassis.right_motors[0].get_position(),
      chassis.right_motors[1].get_position(),
      static_cast<long>(horizontal_odom.get_position()),
      chassis.drive_imu_get(),
      chassis.imu.get_rotation(),
      static_cast<int>(chassis.imu.get_status()),
      errno);
  fflush(stdout);

  if (sample % 5 == 0) {
    pros::lcd::print(0, "P6 %4ldmm c%2ld %s", readings[0].mm, readings[0].confidence,
                     readings[0].installed ? "ok" : "no");
    pros::lcd::print(1, "P7 %4ldmm c%2ld %s", readings[1].mm, readings[1].confidence,
                     readings[1].installed ? "ok" : "no");
    pros::lcd::print(2, "P8 %4ldmm c%2ld %s", readings[2].mm, readings[2].confidence,
                     readings[2].installed ? "ok" : "no");
    pros::lcd::set_text(3, "P9 right slider");
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
  pros::lcd::set_text(0, "3x Distance sensors");
  pros::lcd::set_text(1, "Ports 6, 7, 8");
  pros::lcd::set_text(2, "IMU calibrating...");
  chassis.drive_sensor_reset();
  const bool imu_ready = chassis.drive_imu_calibrate(false);
  chassis.drive_imu_reset(0.0);
  pros::lcd::set_text(2, imu_ready ? "IMU ready" : "IMU check failed");
  horizontal_odom.reset_position();
  localization_telemetry_reset();
  ai_vision_shadow_initialize();
  pros::delay(500);

  printf("D4 init ports=6,7,8 period_ms=%lu installed=%d,%d,%d\n",
         static_cast<unsigned long>(TELEMETRY_PERIOD_MS),
         static_cast<int>(distance_6.is_installed()),
         static_cast<int>(distance_7.is_installed()),
         static_cast<int>(distance_8.is_installed()));
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
    pros::lcd::set_text(6, "Long fusion in 5 sec");
    pros::delay(5000);
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

      const int claw_arm_power = pose_editor_active
          ? 0
          : master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
              ? 127
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? -127 : 0);
      counter_rollers.move(claw_arm_power);

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
