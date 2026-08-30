#include "main.h"
#include "cascade_lift.hpp"
#include "gps_frame.hpp"
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
// One supervised reversible heading sweep for live P8 characterization.
constexpr bool RUN_STARTUP_AI_VISION_HEADING_SWEEP = false;
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
// Short fused-estimator turn/return diagnostic. Enable for one supervised boot.
constexpr bool RUN_STARTUP_FUSED_ROTATION_HEALTH_TEST = false;
// One supervised curved relative endpoint acceptance route.
constexpr bool RUN_STARTUP_FUSED_BOOMERANG_TEST = false;
// One supervised public-API qualification route. It traces mirrored compact
// curves and returns to the same anchor; restore false after the single boot.
constexpr bool RUN_STARTUP_NAVIGATION_QUALIFICATION = false;
// One supervised 3-degree, zero-translation public-API boundary diagnostic.
constexpr bool RUN_STARTUP_NAVIGATION_API_BOUNDARY_TEST = false;
// One supervised 1.5-inch live motion with a deterministic IMU-dropout hook.
constexpr bool RUN_STARTUP_NAVIGATION_DROPOUT_TEST = false;
// One supervised P1 stop test. Its harness refuses motion unless a target is
// already inside the configured forward stop boundary.
constexpr bool RUN_STARTUP_NAVIGATION_OBSTACLE_TEST = false;
// Three supervised cumulative 3-inch public straight legs.
constexpr bool RUN_STARTUP_NAVIGATION_STRAIGHT_QUALIFICATION = false;
// Two forward/reverse pairs through the public signed-relative API.
constexpr bool RUN_STARTUP_NAVIGATION_BIDIRECTIONAL_QUALIFICATION = false;
// Reposition to open center, run paired public turns, then reverse home.
constexpr bool RUN_STARTUP_NAVIGATION_TURN_QUALIFICATION = false;
// From the open-center endpoint of a failed turn trial, retest and reverse home.
constexpr bool RUN_STARTUP_NAVIGATION_TURN_RECOVERY = false;
// Camera-safe shallow mirrored curves from a forward-repositioned center.
constexpr bool RUN_STARTUP_NAVIGATION_MIRRORED_CURVE = false;
// Bounded P1 approach to the visible forward Goal, then reverse by actual travel.
constexpr bool RUN_STARTUP_NAVIGATION_OBSTACLE_APPROACH = false;
// Camera-verified reverse-only recovery after the obstacle-approach trial.
constexpr bool RUN_STARTUP_NAVIGATION_REVERSE_RECOVERY = false;
// One 0.5-second, low-power mechanism-only slider test. Drivetrain stays off.
constexpr bool RUN_STARTUP_SLIDER_SLOW_TEST = false;
// Sequential low-power mechanism checkout with long camera snapshot pauses.
constexpr bool RUN_STARTUP_ALL_MECHANISMS_TEST = false;
// Dedicated ADI-H clamp state pictures; always finishes in the false state.
constexpr bool RUN_STARTUP_CLAMP_PICTURE_TEST = false;
// One supervised toggle-contact and far-Goal example route.
constexpr bool RUN_STARTUP_TOGGLE_GOAL_EXAMPLE = false;
// One supervised Path 1 opening trial: toggle contact and six-inch reverse.
// Restore false and reinstall the safe image after every recorded run.
constexpr bool RUN_STARTUP_PATH1_OPENING_TUNING = false;
// Resume from the measured Path 1 opening endpoint and tune only the -90 deg
// in-place turn. Restore false and reinstall the safe image after each run.
constexpr bool RUN_STARTUP_PATH1_GOAL_TURN_TUNING = false;
// Resume after the verified -90 deg turn and perform only the bounded Goal 3
// contact approach. Restore false and reinstall the safe image after the run.
constexpr bool RUN_STARTUP_PATH1_GOAL_APPROACH_TUNING = false;
// Drive-disabled, low-power L1-direction pin outtake at the verified rear Goal.
constexpr bool RUN_STARTUP_PATH1_GOAL_OUTTAKE = false;
// Recover from the rejected front-side Goal contact and return to the exact
// Path 1 start for a camera-verified single Toggle flip.
constexpr bool RUN_STARTUP_PATH1_RETURN_TO_START = false;
// Return from the measured Toggle verification endpoint to the exact start.
constexpr bool RUN_STARTUP_PATH1_OPENING_RETURN_TO_START = false;
// Low-power final Toggle contact from the measured near-Toggle endpoint.
constexpr bool RUN_STARTUP_PATH1_TOGGLE_FINISH_PROBE = false;
// Resume-only leg from the measured fail-closed endpoint of the toggle route.
constexpr bool RUN_STARTUP_TOGGLE_GOAL_CONTINUE = false;
// One supervised end-to-end Left+X route trial. Enable only for a recorded
// tuning boot, then restore false before the competition image is installed.
constexpr bool RUN_STARTUP_TOGGLE_FAR_GOAL_TEST = false;
// Temporary supervised full-route trace. Must be false in the normal image.
constexpr bool RUN_STARTUP_SIMPLE_RED_TRACE = false;
constexpr bool RUN_STARTUP_FAR_GOAL_RECOVERY_TEST = false;
// One supervised coordinate-to-coordinate pursuit trial. Restore false after
// the recorded run and reinstall the hotkey-only image.
constexpr bool RUN_STARTUP_PURE_PURSUIT_ENDPOINT_TEST = false;
// One supervised recovery from the measured endpoint of a failed compact
// qualification leg. Restore false after the single boot.
constexpr bool RUN_STARTUP_NAVIGATION_RETURN = false;
// One supervised two-inch straight response check. Restore false after boot.
constexpr bool RUN_STARTUP_DRIVE_RESPONSE_TEST = false;
// One supervised sub-inch incremental static-friction characterization.
constexpr bool RUN_STARTUP_DRIVE_BREAKAWAY_TEST = false;
// One supervised encoder/IMU propagation test with opportunistic stationary
// GPS/P8 correction. Restore false immediately after the single run.
constexpr bool RUN_STARTUP_FUSED_RELATIVE_MOTION_TEST = false;
// One supervised bounded turn-autotune boot. Restore false immediately after
// collecting the paired +/-45 degree trials.
constexpr bool RUN_STARTUP_PID_AUTOTUNE = false;
// One supervised obstacle-aware route; restore false immediately afterward.
constexpr bool RUN_STARTUP_GPS_SAFE_ROUTE = false;
// One-shot, pose-gated move away from the left wall. The pose gate prevents a
// reboot from repeating the translation after a successful recovery.
constexpr bool RUN_STARTUP_WALL_RECOVERY = false;
// Never deploy a multi-foot diagnostic route as the competition callback.
constexpr bool RUN_COMPETITION_DIAGNOSTIC_ROUTE = false;
// Development routes and PID autotune must never be reachable from mechanism
// button combinations in a competition image.
constexpr bool ENABLE_CONTROLLER_MOTION_DIAGNOSTICS = false;
// Temporary slot-2 boot isolator. It disables every opcontrol actuator write;
// only initialize() progress markers run. Restore false after diagnosis.
constexpr bool RUN_BOOT_DIAGNOSTIC_NO_ACTUATION = false;
// One supervised cascade verification from physical rest to calibrated stage 4.
// Keep disabled in the normal competition image.
constexpr bool RUN_STARTUP_CASCADE_STAGE4_TEST = false;
// One supervised all-stage PID sequence from physical rest. It visits each
// stage, dips 75 degrees, returns to that stage, and finally returns to zero.
// Keep disabled in the normal competition image.
constexpr bool RUN_STARTUP_CASCADE_SEQUENCE_TEST = false;
constexpr int CONTROLLER_DRIVE_DEADBAND = 5;
constexpr int kAutonArmLoweringPower = 127;

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
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    runtime_pose_editor.x_in += kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    runtime_pose_editor.y_in -= kPositionStepIn;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
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
// ADI-D is physically inverted: high retracts the cylinder, low extends it.
std::atomic<bool> clamp_output_high{true};

enum class RedAutonSelection : int {
  kOnePin = 0,
  kTwoCup = 1,
};

std::atomic<int> selected_red_auton{
    static_cast<int>(RedAutonSelection::kTwoCup)};
std::atomic<bool> auton_selection_locked{false};

const char* selected_red_auton_name() {
  return selected_red_auton.load(std::memory_order_acquire) ==
                 static_cast<int>(RedAutonSelection::kTwoCup)
             ? "2 Cup Auto Red"
             : "1 Pin Auto Red";
}

void render_auton_selection() {
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase_rect(0, 0, 479, 239);
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "AUTON SELECT");
  pros::screen::set_pen(pros::Color::red);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "%s",
                      selected_red_auton_name());
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5,
                      "< LEFT       RIGHT >");
  if (selected_red_auton.load(std::memory_order_acquire) ==
      static_cast<int>(RedAutonSelection::kTwoCup)) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7,
                        "TEST LIMIT: THROUGH 2ND SCORE");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7,
                        "FULL 1-PIN ROUTE");
  }
}

void select_red_auton(int direction) {
  if (auton_selection_locked.load(std::memory_order_acquire) ||
      opcontrol_auton_running) return;
  constexpr int kAutonCount = 2;
  const int current = selected_red_auton.load(std::memory_order_acquire);
  const int step = direction >= 0 ? 1 : -1;
  const int selected = (current + step + kAutonCount) % kAutonCount;
  selected_red_auton.store(selected, std::memory_order_release);
  render_auton_selection();
  std::printf("AUTON_SELECTOR selected=%d name=%s\n",
              selected, selected_red_auton_name());
  std::fflush(stdout);
}

bool run_selected_red_auton() {
  const auto selected = static_cast<RedAutonSelection>(
      selected_red_auton.load(std::memory_order_acquire));
  // Retracted is the default/holding state for the ADI-E claw and retains the
  // preload until the route explicitly releases it at the first Goal.
  set_claw_piston(false);
  // First autonomous action shared by both routes: an independent task drives
  // the arm at full positive power (the exact Down-arrow direction) for 300ms.
  // It runs concurrently while D gets 1.0s to extend and 0.2s to retract.
  pros::Task arm_lowering_task([] {
    claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    claw_arm.move(kAutonArmLoweringPower);
    pros::delay(300);
    claw_arm.move(0);
    // The full-power pulse seats the arm against its physical lower stop. Use
    // that repeatable position as relative zero for the upcoming loaded PID.
    horizontal_odom.reset_position();
  }, "auton arm home");
  clamp_piston.set_value(false);
  clamp_output_high.store(false, std::memory_order_release);
  pros::delay(1000);
  clamp_piston.set_value(true);
  clamp_output_high.store(true, std::memory_order_release);
  pros::delay(200);
  arm_lowering_task.join();
  std::printf("ARM_HOME relative_deg=%.2f absolute_deg=%.2f\n",
              static_cast<double>(horizontal_odom.get_position()) / 100.0,
              static_cast<double>(horizontal_odom.get_angle()) / 100.0);
  std::printf("AUTON_SELECTOR run=%d name=%s\n",
              static_cast<int>(selected), selected_red_auton_name());
  std::fflush(stdout);
  return selected == RedAutonSelection::kTwoCup
      ? localization_two_cup_red_auton()
      : localization_simple_red_goal_hotkey_auton();
}

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

void print_drive_motor_health(const char* phase) {
  const std::array<int, 4> ports = {17, 18, 11, 13};
  const std::array<pros::Motor*, 4> motors = {
      &chassis.left_motors[0], &chassis.left_motors[1],
      &chassis.right_motors[0], &chassis.right_motors[1]};
  std::printf(
      "DRIVE_POWER phase=%s battery_mv=%ld battery_ma=%ld capacity=%.1f\n",
      phase, static_cast<long>(pros::battery::get_voltage()),
      static_cast<long>(pros::battery::get_current()),
      pros::battery::get_capacity());
  for (std::size_t i = 0; i < motors.size(); ++i) {
    std::printf(
        "DRIVE_MOTOR phase=%s port=%d pos=%.2f velocity=%.2f current_ma=%ld "
        "command_mv=%ld voltage_limit_mv=%ld temp_c=%.1f faults=%lu flags=%lu\n",
        phase, ports[i], motors[i]->get_position(),
        motors[i]->get_actual_velocity(),
        static_cast<long>(motors[i]->get_current_draw()),
        static_cast<long>(motors[i]->get_voltage()),
        static_cast<long>(motors[i]->get_voltage_limit()),
        motors[i]->get_temperature(),
        static_cast<unsigned long>(motors[i]->get_faults()),
        static_cast<unsigned long>(motors[i]->get_flags()));
  }
  std::fflush(stdout);
}

void run_drive_breakaway_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr std::array<int, 5> kPowerSteps{{18, 24, 30, 36, 42}};
  constexpr double kBreakawayTravelIn = 0.15;
  constexpr double kPerStepTravelLimitIn = 0.50;
  constexpr std::uint32_t kStepTimeLimitMs = 300;
  constexpr long kObstacleStopMm = static_cast<long>(
      localization::kForwardObstacleStopIn * 25.4);

  auto stop_and_brake = []() {
    for (auto& motor : chassis.left_motors) {
      motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      motor.brake();
    }
    for (auto& motor : chassis.right_motors) {
      motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      motor.brake();
    }
  };
  auto average_position_deg = []() {
    return 0.25 * (
        chassis.left_motors[0].get_position() +
        chassis.left_motors[1].get_position() +
        chassis.right_motors[0].get_position() +
        chassis.right_motors[1].get_position());
  };

  stop_and_brake();
  print_drive_motor_health("breakaway_start");
  bool found = false;
  int selected_power = 0;
  double selected_travel_in = 0.0;
  for (const int power : kPowerSteps) {
    const long p1_mm = static_cast<long>(distance_1.get_distance());
    if (!distance_1.is_installed() ||
        (p1_mm != 9999 &&
         (p1_mm < kObstacleStopMm ||
          p1_mm > localization::kForwardObstacleMaxRangeMm))) {
      std::printf("BREAKAWAY abort=forward_preflight power=%d p1_mm=%ld\n",
                  power, p1_mm);
      break;
    }
    pros::delay(200);
    const double baseline_deg = average_position_deg();
    double travel_in = 0.0;
    const std::uint32_t started_ms = pros::millis();
    for (auto& motor : chassis.left_motors) motor.move(power);
    for (auto& motor : chassis.right_motors) motor.move(power);
    while (pros::millis() - started_ms < kStepTimeLimitMs) {
      travel_in = std::fabs(average_position_deg() - baseline_deg) / 360.0 *
                  kWheelCircumferenceIn;
      const long live_p1_mm = static_cast<long>(distance_1.get_distance());
      if (travel_in >= kPerStepTravelLimitIn ||
          (live_p1_mm != 9999 &&
           live_p1_mm <= kObstacleStopMm)) {
        break;
      }
      pros::delay(20);
    }
    stop_and_brake();
    std::printf(
        "BREAKAWAY power=%d elapsed_ms=%lu travel_in=%.4f p1_mm=%ld\n",
        power, static_cast<unsigned long>(pros::millis() - started_ms),
        travel_in, static_cast<long>(distance_1.get_distance()));
    std::fflush(stdout);
    print_drive_motor_health("breakaway_step_done");
    if (travel_in >= kBreakawayTravelIn) {
      found = true;
      selected_power = power;
      selected_travel_in = travel_in;
      break;
    }
  }
  stop_and_brake();
  std::printf("BREAKAWAY complete found=%d selected_power=%d travel_in=%.4f\n",
              static_cast<int>(found), selected_power, selected_travel_in);
  std::fflush(stdout);
}

void run_distance_sweep_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr std::array<double, 3> kVelocityRpm = {10.0, 20.0, 35.0};
  constexpr std::uint32_t kTimeoutPerInchMs = 1800;
  constexpr std::array<int, 3> kTargetsIn = {2, 5, 10};
  auto& gps = gps_7;
  auto& imu6 = chassis.imu;

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
  auto side_delta_deg = [](const std::array<double, 4>& now,
                           const std::array<double, 4>& baseline,
                           std::size_t first) {
    return ((now[first] - baseline[first]) +
            (now[first + 1] - baseline[first + 1])) / 2.0;
  };
  auto tracking_delta_in = [](std::int32_t now, std::int32_t baseline) {
    return (static_cast<double>(now - baseline) / 100.0 / 360.0) *
           (kPi * localization::kSideOdomWheelDiameterIn);
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

  const bool imu_installed = imu6.is_installed() && !imu6.is_calibrating();
  printf("STRAIGHT_IMU_INIT port=6 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu6.get_status()));
  fflush(stdout);

  for (double velocity_rpm : kVelocityRpm) {
  for (int target_in : kTargetsIn) {
    pros::lcd::print(6, "Sweep %din @ %.0f", target_in, velocity_rpm);
    const auto baseline = motor_positions();
    const std::int32_t baseline_h5 = horizontal_odom.get_position();
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
      command(-velocity_rpm);
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
    const std::int32_t back_h5 = horizontal_odom.get_position();
    const double back_left_deg = side_delta_deg(back_motors, baseline, 0);
    const double back_right_deg = side_delta_deg(back_motors, baseline, 2);
    printf("SWEEP rpm=%.0f target_in=%d phase=back encoder_in=%.3f gps_in=%.3f difference_in=%.3f gps_heading_delta=%.3f gps_error_in=%.3f imu_delta_deg=%.3f imu_max_abs_deg=%.3f left_deg=%.2f right_deg=%.2f drive_side_diff_deg=%.2f h5_delta_cdeg=%ld h5_in=%.3f p1_mm=%ld p1_conf=%ld\n",
           velocity_rpm, target_in, encoder_back_in, gps_back_in,
           gps_back_in - encoder_back_in,
           heading_delta(back_gps.heading_deg, start_gps.heading_deg),
           back_gps.error_m * 39.37007874,
           imu6.get_rotation() - start_imu_deg, max_abs_imu_delta_deg,
           back_left_deg, back_right_deg, back_left_deg - back_right_deg,
           static_cast<long>(back_h5 - baseline_h5),
           tracking_delta_in(back_h5, baseline_h5),
           distance_1.get(), distance_1.get_confidence());
    fflush(stdout);

    pros::lcd::print(6, "Sweep %din: RETURN", target_in);
    const std::uint32_t return_started = pros::millis();
    while (pros::millis() - return_started <
               kTimeoutPerInchMs * static_cast<std::uint32_t>(target_in) &&
           average_delta_deg(motor_positions(), baseline) < -1.5) {
      command(velocity_rpm);
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
    const std::int32_t final_h5 = horizontal_odom.get_position();
    const double final_left_deg = side_delta_deg(final_motors, baseline, 0);
    const double final_right_deg = side_delta_deg(final_motors, baseline, 2);
    printf("SWEEP rpm=%.0f target_in=%d phase=return encoder_residual_in=%.3f gps_residual_in=%.3f gps_heading_delta=%.3f gps_error_in=%.3f imu_residual_deg=%.3f imu_max_abs_deg=%.3f left_deg=%.2f right_deg=%.2f drive_side_diff_deg=%.2f h5_delta_cdeg=%ld h5_in=%.3f p1_mm=%ld p1_conf=%ld\n",
           velocity_rpm, target_in, encoder_residual_in, gps_residual_in,
           heading_delta(final_gps.heading_deg, start_gps.heading_deg),
           final_gps.error_m * 39.37007874,
           imu6.get_rotation() - start_imu_deg, max_abs_imu_delta_deg,
           final_left_deg, final_right_deg, final_left_deg - final_right_deg,
           static_cast<long>(final_h5 - baseline_h5),
           tracking_delta_in(final_h5, baseline_h5),
           distance_1.get(), distance_1.get_confidence());
    fflush(stdout);
    pros::delay(500);
  }
  }
  pros::lcd::set_text(6, "Distance sweep DONE");
}

void run_rotation_sweep_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr std::array<double, 1> kTargetsDeg = {15.0};
  constexpr double kVelocityRpm = 12.0;
  auto& gps = gps_7;
  auto& imu6 = chassis.imu;

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
  auto tracking_delta_in = [](std::int32_t now, std::int32_t baseline) {
    return (static_cast<double>(now - baseline) / 100.0 / 360.0) *
           (kPi * localization::kSideOdomWheelDiameterIn);
  };
  auto stop = [&]() {
    for (auto& motor : chassis.left_motors) motor.move_velocity(0);
    for (auto& motor : chassis.right_motors) motor.move_velocity(0);
  };
  auto command_turn = [&](double rpm) {
    for (auto& motor : chassis.left_motors) motor.move_velocity(rpm);
    for (auto& motor : chassis.right_motors) motor.move_velocity(-rpm);
  };
  auto print_motor_health = [&](const char* phase) {
    const std::array<int, 4> ports = {17, 18, 11, 13};
    const std::array<pros::Motor*, 4> motors = {
        &chassis.left_motors[0], &chassis.left_motors[1],
        &chassis.right_motors[0], &chassis.right_motors[1]};
    for (std::size_t i = 0; i < motors.size(); ++i) {
      printf("MOTOR_HEALTH phase=%s port=%d position_deg=%.2f velocity_rpm=%.2f current_ma=%ld temperature_c=%.1f faults=%lu flags=%lu\n",
             phase, ports[i], motors[i]->get_position(),
             motors[i]->get_actual_velocity(),
             static_cast<long>(motors[i]->get_current_draw()),
             motors[i]->get_temperature(),
             static_cast<unsigned long>(motors[i]->get_faults()),
             static_cast<unsigned long>(motors[i]->get_flags()));
    }
    fflush(stdout);
  };

  chassis.drive_mode_set(ez::DISABLE, true);
  for (auto& motor : chassis.left_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  for (auto& motor : chassis.right_motors)
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  stop();
  const bool imu_installed = imu6.is_installed() && !imu6.is_calibrating();
  printf("ROTATION_SWEEP_INIT imu_port=6 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu6.get_status()));
  fflush(stdout);
  print_motor_health("start");

  for (double target_deg : kTargetsDeg) {
    pros::lcd::print(6, "Turn %.0f: sampling", target_deg);
    const auto baseline = motor_positions();
    const std::int32_t baseline_h5 = horizontal_odom.get_position();
    const HeadingSample start = sample_headings();
    const double side_arc_target_in =
        target_deg * kPi / 180.0 * localization::kDriveTrackWidthIn / 2.0;
    const double motor_target_deg = side_arc_target_in * 360.0 / kWheelCircumferenceIn;

    // Positive left / negative right is counterclockwise in the robot frame.
    pros::lcd::print(6, "Turn %.0f: CCW", target_deg);
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
    const std::int32_t peak_h5 = horizontal_odom.get_position();
    const double peak_h5_in = tracking_delta_in(peak_h5, baseline_h5);
    const double h5_offset_from_imu_in = std::fabs(imu_heading_deg) > 1.0
        ? peak_h5_in / (imu_heading_deg * kPi / 180.0)
        : NAN;
    printf("ROT_SWEEP target_deg=%.0f phase=turn direction=ccw encoder_deg=%.3f gps_deg=%.3f imu_deg=%.3f encoder_minus_gps_deg=%.3f effective_track_in=%.3f gps_error_in=%.3f h5_delta_cdeg=%ld h5_in=%.3f h5_offset_from_imu_in=%.3f p1_mm=%ld p1_conf=%ld\n",
           target_deg, encoder_heading_deg, gps_heading_deg, imu_heading_deg,
           encoder_heading_deg - std::fabs(gps_heading_deg), effective_track_in,
           peak.gps_error_m * 39.37007874,
           static_cast<long>(peak_h5 - baseline_h5), peak_h5_in,
           h5_offset_from_imu_in, distance_1.get(),
           distance_1.get_confidence());
    fflush(stdout);
    print_motor_health("peak");

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
    const std::int32_t final_h5 = horizontal_odom.get_position();
    printf("ROT_SWEEP target_deg=%.0f phase=return encoder_residual_motor_deg=%.3f gps_heading_residual_deg=%.3f imu_heading_residual_deg=%.3f gps_position_residual_in=%.3f h5_residual_cdeg=%ld h5_residual_in=%.3f p1_mm=%ld p1_conf=%ld\n",
           target_deg, turn_motor_delta(final_motors, baseline),
           angle_delta(final.gps_deg, start.gps_deg),
           imu_installed ? angle_delta(final.imu_deg, start.imu_deg) : NAN,
           gps_position_residual_in,
           static_cast<long>(final_h5 - baseline_h5),
           tracking_delta_in(final_h5, baseline_h5),
           distance_1.get(), distance_1.get_confidence());
    fflush(stdout);
    print_motor_health("return");
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
  // Reuse the single shared hardware owners. Constructing a second device on
  // an occupied Smart Port makes diagnostic behavior dependent on object
  // lifetime and can contend with normal telemetry.
  auto& gps = gps_7;
  auto& imu = chassis.imu;

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
    const auto project_pose = localization::vex_gps_to_project_robot_pose(
        sensor_x_in / kInchesPerMeter,
        sensor_y_in / kInchesPerMeter,
        sensor_heading_cw_deg);
    return RobotPose{project_pose.x_in,
                     project_pose.y_in,
                     project_pose.robot_heading_cw_deg,
                     error_in,
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
  const bool start_gate = start.x_in >= -12.0 && start.x_in <= 5.0 &&
                          start.y_in >= 43.0 && start.y_in <= 53.0 &&
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
  const double gps_gyro_z = gps_7.get_gyro_rate_z();
  const auto imu_gyro = chassis.imu.get_gyro_rate();
  const auto imu_accel = chassis.imu.get_accel();
  const int gps_errno = errno;

  printf(
      "D4 s=%lu t=%lu "
      "p1=%ld,%ld,%d "
      "m17=%.1f m18=%.1f m11=%.1f m13=%.1f h5=%ld h5abs=%.2f "
      "imu=%.2f rawimu=%.2f imust=%d "
      "imugyro=%.4f,%.4f,%.4f imuacc=%.4f,%.4f,%.4f "
      "gps7=%.4f,%.4f,%.2f,%.4f,%d errno=%d gpsgyro=%.2f\n",
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
      static_cast<double>(horizontal_odom.get_angle()) / 100.0,
      chassis.drive_imu_get(),
      chassis.imu.get_rotation(),
      static_cast<int>(chassis.imu.get_status()),
      imu_gyro.x,
      imu_gyro.y,
      imu_gyro.z,
      imu_accel.x,
      imu_accel.y,
      imu_accel.z,
      gps_position.x,
      gps_position.y,
      gps_heading_deg,
      gps_error_m,
      static_cast<int>(gps_7.is_installed()),
      gps_errno,
      gps_gyro_z);
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

void start_toggle_far_goal_auton() {
  if (opcontrol_auton_running) return;

  // Cancel the last operator lift write before handing control to the new
  // autonomous task. Without this synchronous handoff, an R1/upward command
  // from the preceding opcontrol cycle can remain latched until the spawned
  // task is scheduled.
  cascade_lift::set_manual_power(0);
  cascade_lift::update();
  slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  slider_right.brake();
  slider_left.brake();
  opcontrol_auton_running = true;
  pros::Task::create([] {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::lcd::print(6, "Run: %s", selected_red_auton_name());
    controller.print(0, 0, "%-18s", selected_red_auton_name());
    move_intake(0);
    chassis.drive_set(0, 0);
    const bool success = run_selected_red_auton();
    chassis.drive_set(0, 0);
    move_intake(0);
    opcontrol_auton_running = false;
    controller.rumble(success ? "." : "---");
    pros::lcd::set_text(
        6, success ? "Simple auton PASS" : "Simple auton FAIL");
    controller.print(
        0, 0, success ? "AUTON PASS        " : "AUTON FAILED      ");
  }, "toggle far goal");
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
  constexpr double kLimitFromStartDeg = 36.0;
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
      // Positive IMU rotation on this calibrated drivetrain requires the
      // left wheels backward and right wheels forward.
      const double magnitude_rpm = std::clamp(
          std::fabs(error_deg) * 0.50, 2.5, kTurnVelocityRpm);
      const double command_rpm = error_deg > 0.0 ? -magnitude_rpm
                                                 : magnitude_rpm;
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
    pros::delay(300);
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
      const double magnitude_rpm = std::clamp(
          std::fabs(delta_deg) * 0.50, 2.5, kTurnVelocityRpm);
      const double command_rpm = delta_deg > 0.0 ? magnitude_rpm
                                                 : -magnitude_rpm;
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

bool ai_vision_heading_characterization() {
  constexpr double kAngleGuardDeg = 70.0;
  constexpr double kTurnVelocityRpm = 8.0;
  constexpr double kToleranceDeg = 0.8;
  constexpr std::uint32_t kLegTimeoutMs = 18000;
  constexpr std::uint32_t kStallTimeoutMs = 2200;
  constexpr std::uint32_t kHoldMs = 1000;
  constexpr std::array<double, 24> kTargetsDeg{
      -10.0, -15.0, -20.0, -25.0, -30.0, -35.0, -40.0, -45.0,
      -50.0, -55.0, -60.0, -65.0, -60.0, -55.0, -50.0, -45.0,
      -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, 0.0};

  for (auto& motor : chassis.left_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  for (auto& motor : chassis.right_motors) {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  stop_drive_velocity();
  if (!chassis.imu.is_installed() || chassis.imu.is_calibrating() ||
      chassis.imu.get_status() == pros::ImuStatus::error) {
    std::printf("P8_HEADING event=abort reason=imu_invalid\n");
    std::fflush(stdout);
    return false;
  }
  const double start_imu_deg = chassis.drive_imu_get();
  if (!std::isfinite(start_imu_deg)) {
    std::printf("P8_HEADING event=abort reason=imu_nonfinite\n");
    std::fflush(stdout);
    return false;
  }
  std::printf(
      "P8_HEADING event=start start_imu=%.2f guard_deg=%.1f velocity_rpm=%.1f\n",
      start_imu_deg, kAngleGuardDeg, kTurnVelocityRpm);
  std::fflush(stdout);

  int valid_samples = 0;
  int changed_samples = 0;
  int repeated_samples = 0;
  int visible_legs = 0;
  for (std::size_t target_index = 0; target_index < kTargetsDeg.size();
       ++target_index) {
    const double target_deg = kTargetsDeg[target_index];
    const std::uint32_t leg_start_ms = pros::millis();
    std::uint32_t last_motion_ms = leg_start_ms;
    double last_delta_deg = chassis.drive_imu_get() - start_imu_deg;
    bool reached = false;
    while (pros::millis() - leg_start_ms < kLegTimeoutMs) {
      const double delta_deg = chassis.drive_imu_get() - start_imu_deg;
      if (!std::isfinite(delta_deg) || std::fabs(delta_deg) > kAngleGuardDeg) {
        stop_drive_velocity();
        std::printf("P8_HEADING event=abort reason=angle_guard delta=%.2f\n",
                    delta_deg);
        std::fflush(stdout);
        return false;
      }
      ai_vision_shadow_update();
      print_distance_frame();
      const double error_deg = target_deg - delta_deg;
      if (std::fabs(error_deg) <= kToleranceDeg) {
        stop_drive_velocity();
        reached = true;
        break;
      }
      const double magnitude_rpm = std::clamp(
          std::fabs(error_deg) * 0.50, 2.5, kTurnVelocityRpm);
      const double command_rpm = error_deg > 0.0 ? -magnitude_rpm
                                                 : magnitude_rpm;
      for (auto& motor : chassis.left_motors) motor.move_velocity(command_rpm);
      for (auto& motor : chassis.right_motors) motor.move_velocity(-command_rpm);
      const std::uint32_t now = pros::millis();
      if (std::fabs(delta_deg - last_delta_deg) >= 0.20) {
        last_delta_deg = delta_deg;
        last_motion_ms = now;
      } else if (now - last_motion_ms > kStallTimeoutMs) {
        stop_drive_velocity();
        std::printf("P8_HEADING event=abort reason=stall target=%.1f delta=%.2f\n",
                    target_deg, delta_deg);
        std::fflush(stdout);
        return false;
      }
      pros::delay(SAMPLE_PERIOD_MS);
    }
    stop_drive_velocity();
    if (!reached) {
      std::printf("P8_HEADING event=abort reason=timeout target=%.1f\n",
                  target_deg);
      std::fflush(stdout);
      return false;
    }

    int leg_valid = 0;
    int leg_changed = 0;
    int leg_repeated = 0;
    const std::uint32_t hold_start_ms = pros::millis();
    while (pros::millis() - hold_start_ms < kHoldMs) {
      ai_vision_shadow_update();
      print_distance_frame();
      const AiVisionShadowSnapshot vision = ai_vision_shadow_snapshot();
      if (vision.tag_valid) {
        ++leg_valid;
        ++valid_samples;
        if (vision.repeated_geometry) {
          ++leg_repeated;
          ++repeated_samples;
        } else {
          ++leg_changed;
          ++changed_samples;
        }
      }
      pros::delay(50);
    }
    const double achieved_deg = chassis.drive_imu_get() - start_imu_deg;
    const AiVisionShadowSnapshot vision = ai_vision_shadow_snapshot();
    if (leg_valid > 0) ++visible_legs;
    std::printf(
        "P8_HEADING event=hold target=%.1f achieved=%.2f valid=%d changed=%d "
        "repeat=%d tag=%d bearing=%.2f horizontal=%.2f range=%.2f area=%.1f "
        "reason=%s p1_mm=%ld p1_conf=%ld\n",
        target_deg, achieved_deg, leg_valid, leg_changed, leg_repeated,
        vision.tag_id, vision.bearing_deg, vision.horizontal_range_in,
        vision.range_estimate_in, vision.area_px2, vision.reason,
        static_cast<long>(distance_1.get()),
        static_cast<long>(distance_1.get_confidence()));
    std::fflush(stdout);
  }
  stop_drive_velocity();
  const double final_delta_deg = chassis.drive_imu_get() - start_imu_deg;
  const bool returned = std::fabs(final_delta_deg) <= 2.0;
  std::printf(
      "P8_HEADING event=done returned=%d final_delta=%.2f visible_legs=%d "
      "valid=%d changed=%d repeat=%d\n",
      static_cast<int>(returned), final_delta_deg, visible_legs,
      valid_samples, changed_samples, repeated_samples);
  std::fflush(stdout);
  return returned && visible_legs >= 3;
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
  // Emit progress before each device call. A Smart Port driver must never be
  // able to make a black Brain screen look like a dead user program.
  std::printf("BOOT_STAGE t=%lu stage=entry\n",
              static_cast<unsigned long>(pros::millis()));
  for (std::uint8_t port = 1; port <= 21; ++port) {
    const int type = static_cast<int>(pros::c::get_plugged_type(port));
    if (type != 0) {
      std::printf("DEVICE_PORT port=%u type=%d\n",
                  static_cast<unsigned>(port), type);
    }
  }
  std::fflush(stdout);

  // Full-size 11 W port-4 arm motor. Keep it passive until its external
  // rotation-sensor PID is retuned for the changed mechanism and load.
  claw_arm.set_current_limit(2500);
  claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  claw_arm.move(0);
  // Port D is wired active-low: high is the physical retracted state. A then
  // toggles from this known state instead of inheriting stale mode state.
  clamp_piston.set_value(true);
  clamp_output_high.store(true, std::memory_order_release);

  std::printf("BOOT_STAGE t=%lu stage=lcd_begin\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Forward Distance P1");
  pros::lcd::set_text(1, "GPS P7 / IMU P6");
  pros::lcd::set_text(2, "IMU calibrating...");
  std::printf("BOOT_STAGE t=%lu stage=drive_reset_begin\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  chassis.drive_sensor_reset();
  std::printf("BOOT_STAGE t=%lu stage=drive_reset_done\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  bool imu_ready = false;
  if (chassis.imu.is_installed()) {
    std::printf("BOOT_STAGE t=%lu stage=imu_reset_begin\n",
                static_cast<unsigned long>(pros::millis()));
    std::fflush(stdout);
    chassis.imu.reset();
    std::printf("BOOT_STAGE t=%lu stage=imu_reset_returned\n",
                static_cast<unsigned long>(pros::millis()));
    std::fflush(stdout);
    const std::uint32_t imu_calibration_started = pros::millis();
    while (chassis.imu.is_calibrating() &&
           pros::millis() - imu_calibration_started < 5000) {
      pros::delay(20);
    }
    imu_ready = !chassis.imu.is_calibrating() &&
                chassis.imu.get_status() != pros::ImuStatus::error;
    if (imu_ready) chassis.drive_imu_reset(0.0);
  }
  std::printf("BOOT_STAGE t=%lu stage=imu_done ready=%d\n",
              static_cast<unsigned long>(pros::millis()),
              static_cast<int>(imu_ready));
  std::fflush(stdout);
  pros::lcd::set_text(2, imu_ready ? "IMU ready" : "IMU check failed");
  std::printf("BOOT_STAGE t=%lu stage=aux_reset_begin\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  horizontal_odom.reset_position();
  cascade_lift::initialize_at_rest();
  set_claw_piston(false);
  localization_telemetry_reset();
  std::printf("BOOT_STAGE t=%lu stage=vision_init_begin\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  ai_vision_shadow_initialize();
  std::printf("BOOT_STAGE t=%lu stage=vision_init_done\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  pros::delay(500);
  auton_selection_locked.store(false, std::memory_order_release);
  render_auton_selection();

  printf("D4 init distance_port=1 direction=forward period_ms=%lu installed=%d\n",
         static_cast<unsigned long>(TELEMETRY_PERIOD_MS),
         static_cast<int>(distance_1.is_installed()));
  printf("IMU_INIT calibrated=%d raw_heading=%.2f\n",
         static_cast<int>(imu_ready),
         chassis.drive_imu_get());
  print_drive_motor_health("startup_stationary");
  fflush(stdout);

  // Keep the autonomous hotkey independent of the competition callback state.
  // This listener is the single owner of Left+X and remains alive whenever the
  // user program is running, even if opcontrol() is restarted.
  pros::Task::create([] {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool armed = true;
    bool last_left = false;
    bool last_x = false;
    std::uint32_t chord_started_ms = 0;
    while (true) {
      const bool left = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_LEFT);
      const bool x = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_X);
      if (left != last_left || x != last_x) {
        std::printf("HOTKEY_INPUT left=%d x=%d connected=%d\n",
                    static_cast<int>(left), static_cast<int>(x),
                    static_cast<int>(controller.is_connected()));
        std::fflush(stdout);
        last_left = left;
        last_x = x;
      }
      if (left && x) {
        if (chord_started_ms == 0) chord_started_ms = pros::millis();
        if (armed && pros::millis() - chord_started_ms >= 100) {
          armed = false;
          std::printf("SIMPLE_RED event=hotkey_left_x_listener\n");
          std::fflush(stdout);
          controller.rumble(".-");
          start_toggle_far_goal_auton();
        }
      } else {
        chord_started_ms = 0;
        if (!left && !x) armed = true;
      }
      pros::delay(20);
    }
  }, "left x auton listener");

  // Left cycles to the next autonomous while disabled or in normal opcontrol.
  // Ignore it while X is held so the Left+X launch chord cannot also change
  // the selected route. Selection locks when competition autonomous begins.
  pros::Task::create([] {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool last_left = false;
    bool left_press_had_x = false;
    bool last_right = false;
    bool last_brain_left = false;
    bool last_brain_right = false;
    bool last_touch_pressed = false;
    std::uint32_t last_render_ms = 0;
    while (!auton_selection_locked.load(std::memory_order_acquire)) {
      const bool x_held = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_X);
      const bool left = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_LEFT);
      const bool selectable = !opcontrol_auton_running;
      const bool right = selectable && controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_RIGHT);
      const std::uint8_t brain_buttons = pros::lcd::read_buttons();
      const bool brain_left = (brain_buttons & LCD_BTN_LEFT) != 0;
      const bool brain_right = (brain_buttons & LCD_BTN_RIGHT) != 0;
      const auto touch = pros::screen::touch_status();
      const bool touch_pressed =
          touch.touch_status == pros::E_TOUCH_PRESSED ||
          touch.touch_status == pros::E_TOUCH_HELD;
      // Commit a Left selection only on release, and only if X was never part
      // of that press. This makes Left+X atomic even when controller packets
      // report Left a loop earlier than X.
      if (left && !last_left) left_press_had_x = x_held;
      if (left && x_held) left_press_had_x = true;
      if (!left && last_left) {
        if (!left_press_had_x && selectable) select_red_auton(+1);
        left_press_had_x = false;
      }
      if (right && !last_right) select_red_auton(-1);
      if (brain_left && !last_brain_left) select_red_auton(-1);
      if (brain_right && !last_brain_right) select_red_auton(+1);
      if (touch_pressed && !last_touch_pressed) {
        select_red_auton(touch.x < 240 ? -1 : +1);
      }
      last_left = left;
      last_right = right;
      last_brain_left = brain_left;
      last_brain_right = brain_right;
      last_touch_pressed = touch_pressed;
      if (pros::millis() - last_render_ms >= 250) {
        render_auton_selection();
        last_render_ms = pros::millis();
      }
      if (selectable) {
        controller.print(1, 0, "< %-15s >", selected_red_auton_name());
      }
      pros::delay(20);
    }
  }, "auton selector");
}

void disabled() {
  navigation::stop();
}

void competition_initialize() { render_auton_selection(); }

void autonomous() {
  auton_selection_locked.store(true, std::memory_order_release);
  if constexpr (RUN_COMPETITION_DIAGNOSTIC_ROUTE) {
    fusion_test_auton();
  } else {
    run_selected_red_auton();
  }
}

void opcontrol() {
  if constexpr (RUN_BOOT_DIAGNOSTIC_NO_ACTUATION) {
    while (true) {
      std::printf("BOOT_HEARTBEAT t=%lu\n",
                  static_cast<unsigned long>(pros::millis()));
      std::fflush(stdout);
      pros::delay(1000);
    }
  }
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  master.print(0, 0, "READY: UP+X AUTON ");
  pros::lcd::set_text(6, "Ready: press Up + X");
  if (drive_positions_are_zeroed()) {
    localization_telemetry_reset();
  }
  if (RUN_STARTUP_SIMPLE_RED_TRACE) {
    pros::lcd::set_text(6, "Simple red trace in 5s");
    std::printf("SIMPLE_RED_TRACE event=countdown duration_ms=5000\n");
    std::fflush(stdout);
    pros::delay(5000);
    const bool trace_ok = localization_simple_red_goal_hotkey_auton();
    std::printf("SIMPLE_RED_TRACE event=done ok=%d\n",
                static_cast<int>(trace_ok));
    std::fflush(stdout);
    pros::lcd::set_text(6, trace_ok ? "Trace PASS" : "Trace FAIL");
  }
  if (RUN_STARTUP_CASCADE_SEQUENCE_TEST) {
    constexpr double kDipDeg = 75.0;
    constexpr double kToleranceDeg = 10.0;
    constexpr std::uint32_t kSettleMs = 250;
    constexpr std::uint32_t kLegTimeoutMs = 10000;
    constexpr std::int32_t kCurrentAbortMa = 2450;
    constexpr std::uint32_t kCurrentAbortConfirmMs = 200;

    const auto run_target = [&](double target_deg, const char* label) {
      const bool accepted = cascade_lift::set_target_position_deg(target_deg);
      const std::uint32_t started_ms = pros::millis();
      std::uint32_t settled_since_ms = 0;
      std::uint32_t loaded_since_ms = 0;
      bool reached = false;
      bool current_abort = false;
      std::printf("CASCADE_SEQUENCE begin label=%s target_deg=%.2f accepted=%d\n",
                  label, target_deg, static_cast<int>(accepted));
      std::fflush(stdout);
      while (accepted && pros::millis() - started_ms < kLegTimeoutMs) {
        cascade_lift::update();
        cascade_lift::print_telemetry_if_due(100);
        const auto lift = cascade_lift::snapshot();
        const std::uint32_t now_ms = pros::millis();
        const std::int32_t max_current_ma = std::max(
            slider_right.get_current_draw(), slider_left.get_current_draw());
        if (max_current_ma >= kCurrentAbortMa) {
          if (loaded_since_ms == 0) loaded_since_ms = now_ms;
          if (now_ms - loaded_since_ms >= kCurrentAbortConfirmMs) {
            current_abort = true;
            break;
          }
        } else {
          loaded_since_ms = 0;
        }
        if (std::fabs(lift.error_deg) <= kToleranceDeg &&
            std::fabs(lift.velocity_deg_s) <= 15.0) {
          if (settled_since_ms == 0) settled_since_ms = now_ms;
          if (now_ms - settled_since_ms >= kSettleMs) {
            reached = true;
            break;
          }
        } else {
          settled_since_ms = 0;
        }
        if (lift.faulted) break;
        pros::delay(20);
      }
      const auto final_lift = cascade_lift::snapshot();
      std::printf(
          "CASCADE_SEQUENCE end label=%s reached=%d current_abort=%d "
          "fault=%d position_deg=%.2f error_deg=%.2f elapsed_ms=%lu\n",
          label, static_cast<int>(reached), static_cast<int>(current_abort),
          static_cast<int>(final_lift.faulted), final_lift.position_deg,
          target_deg - final_lift.position_deg,
          static_cast<unsigned long>(pros::millis() - started_ms));
      std::fflush(stdout);
      return reached && !current_abort && !final_lift.faulted;
    };

    bool sequence_ok = true;
    for (int stage = 1; stage <= static_cast<int>(cascade_lift::kStageCount);
         ++stage) {
      const double stage_deg = cascade_lift::stage_position_deg(stage);
      char stage_label[20];
      char dip_label[20];
      std::snprintf(stage_label, sizeof(stage_label), "stage_%d", stage);
      std::snprintf(dip_label, sizeof(dip_label), "stage_%d_dip", stage);
      sequence_ok = run_target(stage_deg, stage_label) && sequence_ok;
      if (!sequence_ok) break;
      pros::delay(500);
      sequence_ok = run_target(std::max(0.0, stage_deg - kDipDeg), dip_label) &&
                    sequence_ok;
      if (!sequence_ok) break;
      sequence_ok = run_target(stage_deg, stage_label) && sequence_ok;
      if (!sequence_ok) break;
      pros::delay(500);
    }

    if (!sequence_ok) cascade_lift::clear_fault();
    const bool zero_ok = run_target(0.0, "zero");
    cascade_lift::disable_pid();
    slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_right.move(0);
    slider_left.move(0);
    std::printf("CASCADE_SEQUENCE complete sequence_ok=%d zero_ok=%d\n",
                static_cast<int>(sequence_ok), static_cast<int>(zero_ok));
    std::fflush(stdout);
  }
  if (RUN_STARTUP_CASCADE_STAGE4_TEST) {
    constexpr std::uint32_t kCascadeTestTimeoutMs = 10000;
    constexpr std::int32_t kCascadeTestCurrentAbortMa = 2400;
    constexpr double kCascadeTestToleranceDeg = 8.0;
    const bool target_ok = cascade_lift::set_target_stage(4);
    const std::uint32_t started_ms = pros::millis();
    bool reached = false;
    bool current_abort = false;
    while (target_ok &&
           pros::millis() - started_ms < kCascadeTestTimeoutMs) {
      cascade_lift::update();
      cascade_lift::print_telemetry_if_due(100);
      const auto lift = cascade_lift::snapshot();
      const std::int32_t max_current_ma = std::max(
          slider_right.get_current_draw(), slider_left.get_current_draw());
      if (max_current_ma >= kCascadeTestCurrentAbortMa) {
        current_abort = true;
        break;
      }
      if (std::fabs(lift.target_deg - lift.position_deg) <=
          kCascadeTestToleranceDeg) {
        reached = true;
        break;
      }
      if (lift.faulted) break;
      pros::delay(20);
    }
    cascade_lift::disable_pid();
    slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_right.move(0);
    slider_left.move(0);
    const auto final_lift = cascade_lift::snapshot();
    std::printf(
        "CASCADE_STAGE4_TEST complete target_ok=%d reached=%d "
        "current_abort=%d fault=%d target_deg=%.2f position_deg=%.2f "
        "error_deg=%.2f elapsed_ms=%lu\n",
        static_cast<int>(target_ok), static_cast<int>(reached),
        static_cast<int>(current_abort), static_cast<int>(final_lift.faulted),
        final_lift.target_deg, final_lift.position_deg,
        final_lift.target_deg - final_lift.position_deg,
        static_cast<unsigned long>(pros::millis() - started_ms));
    std::fflush(stdout);
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
  if (RUN_STARTUP_FUSED_ROTATION_HEALTH_TEST) {
    pros::lcd::set_text(6, "Fused turns in 3 sec");
    pros::delay(3000);
    localization_fused_rotation_health_test();
  }
  if (RUN_STARTUP_FUSED_BOOMERANG_TEST) {
    pros::lcd::set_text(6, "12,12 route in 5 sec");
    pros::delay(5000);
    localization_fused_boomerang_test();
  }
  if (RUN_STARTUP_NAVIGATION_QUALIFICATION) {
    pros::lcd::set_text(6, "Nav route in 5 sec");
    pros::delay(5000);
    localization_navigation_qualification_route();
  }
  if (RUN_STARTUP_NAVIGATION_API_BOUNDARY_TEST) {
    pros::lcd::set_text(6, "Nav API check in 5 sec");
    pros::delay(5000);
    localization_navigation_api_boundary_test();
  }
  if (RUN_STARTUP_NAVIGATION_DROPOUT_TEST) {
    pros::lcd::set_text(6, "Dropout test in 5 sec");
    pros::delay(5000);
    localization_navigation_dropout_abort_test();
  }
  if (RUN_STARTUP_NAVIGATION_OBSTACLE_TEST) {
    pros::lcd::set_text(6, "Obstacle test in 5 sec");
    pros::delay(5000);
    localization_navigation_obstacle_abort_test();
  }
  if (RUN_STARTUP_NAVIGATION_STRAIGHT_QUALIFICATION) {
    pros::lcd::set_text(6, "Straight trial in 5 sec");
    pros::delay(5000);
    localization_navigation_straight_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_BIDIRECTIONAL_QUALIFICATION) {
    pros::lcd::set_text(6, "Fwd/rev trial in 5s");
    pros::delay(5000);
    localization_navigation_bidirectional_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_TURN_QUALIFICATION) {
    pros::lcd::set_text(6, "Turn trial in 5 sec");
    pros::delay(5000);
    localization_navigation_turn_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_TURN_RECOVERY) {
    pros::lcd::set_text(6, "Turn recovery in 5s");
    pros::delay(5000);
    localization_navigation_turn_recovery_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_MIRRORED_CURVE) {
    pros::lcd::set_text(6, "Mirror curves in 5s");
    pros::delay(5000);
    localization_navigation_mirrored_curve_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_OBSTACLE_APPROACH) {
    pros::lcd::set_text(6, "P1 approach in 5 sec");
    pros::delay(5000);
    localization_navigation_obstacle_approach_qualification();
  }
  if (RUN_STARTUP_NAVIGATION_REVERSE_RECOVERY) {
    pros::lcd::set_text(6, "Reverse recovery in 5s");
    pros::delay(5000);
    localization_navigation_reverse_recovery();
  }
  if (RUN_STARTUP_SLIDER_SLOW_TEST) {
    pros::lcd::set_text(6, "Slider test in 5 sec");
    pros::delay(5000);
    localization_slider_slow_picture_test();
  }
  if (RUN_STARTUP_ALL_MECHANISMS_TEST) {
    pros::lcd::set_text(6, "Mechanisms in 5 sec");
    pros::delay(5000);
    localization_all_mechanisms_test();
  }
  if (RUN_STARTUP_CLAMP_PICTURE_TEST) {
    pros::lcd::set_text(6, "Clamp pictures in 5s");
    pros::delay(5000);
    localization_clamp_picture_test();
  }
  if (RUN_STARTUP_TOGGLE_GOAL_EXAMPLE) {
    pros::lcd::set_text(6, "Toggle route in 5s");
    pros::delay(5000);
    localization_toggle_goal_example_auton();
  }
  if (RUN_STARTUP_PATH1_OPENING_TUNING) {
    pros::lcd::set_text(6, "Path1 opening in 3s");
    pros::delay(3000);
    const bool path1_opening_ok = localization_path1_opening_tuning_test();
    pros::lcd::set_text(6, path1_opening_ok ? "Path1 opening PASS"
                                           : "Path1 opening STOP");
  }
  if (RUN_STARTUP_PATH1_GOAL_TURN_TUNING) {
    pros::lcd::set_text(6, "Path1 turn in 3s");
    pros::delay(3000);
    const bool path1_turn_ok = localization_path1_goal_turn_tuning_test();
    pros::lcd::set_text(6, path1_turn_ok ? "Path1 turn PASS"
                                        : "Path1 turn STOP");
  }
  if (RUN_STARTUP_PATH1_GOAL_APPROACH_TUNING) {
    pros::lcd::set_text(6, "Path1 approach in 3s");
    pros::delay(3000);
    const bool path1_approach_ok =
        localization_path1_goal_approach_tuning_test();
    pros::lcd::set_text(6, path1_approach_ok ? "Path1 approach PASS"
                                            : "Path1 approach STOP");
  }
  if (RUN_STARTUP_PATH1_GOAL_OUTTAKE) {
    pros::lcd::set_text(6, "Path1 outtake in 3s");
    pros::delay(3000);
    const bool outtake_ok = localization_path1_goal_outtake_test();
    pros::lcd::set_text(6, outtake_ok ? "Path1 outtake PASS"
                                     : "Path1 outtake STOP");
  }
  if (RUN_STARTUP_PATH1_RETURN_TO_START) {
    pros::lcd::set_text(6, "Path1 reset in 3s");
    pros::delay(3000);
    const bool path1_reset_ok = localization_path1_return_to_exact_start_test();
    pros::lcd::set_text(6, path1_reset_ok ? "Path1 reset PASS"
                                         : "Path1 reset STOP");
  }
  if (RUN_STARTUP_PATH1_OPENING_RETURN_TO_START) {
    pros::lcd::set_text(6, "Toggle reset in 3s");
    pros::delay(3000);
    const bool reset_ok = localization_path1_opening_return_to_start_test();
    pros::lcd::set_text(6, reset_ok ? "Toggle reset PASS"
                                   : "Toggle reset STOP");
  }
  if (RUN_STARTUP_PATH1_TOGGLE_FINISH_PROBE) {
    pros::lcd::set_text(6, "Toggle probe in 3s");
    pros::delay(3000);
    const bool probe_ok = localization_path1_toggle_finish_probe_test();
    pros::lcd::set_text(6, probe_ok ? "Toggle probe PASS"
                                   : "Toggle probe STOP");
  }
  if (RUN_STARTUP_TOGGLE_GOAL_CONTINUE) {
    pros::lcd::set_text(6, "Goal continuation in 5s");
    pros::delay(5000);
    localization_toggle_goal_continue_auton();
  }
  if (RUN_STARTUP_TOGGLE_FAR_GOAL_TEST) {
    pros::lcd::set_text(6, "Far Goal trial in 5s");
    pros::delay(5000);
    const bool far_goal_ok = localization_toggle_far_goal_hotkey_auton();
    pros::lcd::set_text(6, far_goal_ok ? "Far Goal trial PASS"
                                      : "Far Goal trial FAIL");
  }
  if (RUN_STARTUP_FAR_GOAL_RECOVERY_TEST) {
    pros::lcd::set_text(6, "Return to Toggle in 5s");
    pros::delay(5000);
    const bool recovery_ok = localization_far_goal_trial_recover_to_start();
    pros::lcd::set_text(6, recovery_ok ? "Toggle return PASS"
                                      : "Toggle return STOP");
  }
  if (RUN_STARTUP_PURE_PURSUIT_ENDPOINT_TEST) {
    pros::delay(1500);
    const bool pursuit_ok = localization_pure_pursuit_endpoint_test();
    pros::lcd::set_text(6, pursuit_ok ? "Pursuit endpoint OK"
                                     : "Pursuit endpoint FAIL");
  }
  if (RUN_STARTUP_NAVIGATION_RETURN) {
    pros::lcd::set_text(6, "Return in 5 sec");
    pros::delay(5000);
    localization_navigation_return_route();
  }
  if (RUN_STARTUP_DRIVE_RESPONSE_TEST) {
    pros::lcd::set_text(6, "2in response in 5s");
    pros::delay(5000);
    localization_drive_response_test();
  }
  if (RUN_STARTUP_DRIVE_BREAKAWAY_TEST) {
    pros::lcd::set_text(6, "Breakaway in 5 sec");
    pros::delay(5000);
    run_drive_breakaway_test();
  }
  if (RUN_STARTUP_FUSED_RELATIVE_MOTION_TEST) {
    pros::lcd::set_text(6, "12in test in 3 sec");
    pros::delay(3000);
    localization_fused_relative_motion_test();
  }
  if (RUN_STARTUP_PID_AUTOTUNE) {
    pros::lcd::set_text(6, "PID tune in 5 sec");
    pros::delay(5000);
    const bool pid_tune_ok = pid_autotune_auton();
    pros::lcd::set_text(6, pid_tune_ok ? "PID tune complete"
                                      : "PID tune FAILED");
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
  if (RUN_STARTUP_AI_VISION_HEADING_SWEEP) {
    pros::lcd::set_text(6, "P8 sweep in 5 sec");
    pros::delay(5000);
    const bool p8_sweep_ok = ai_vision_heading_characterization();
    pros::lcd::set_text(6, p8_sweep_ok ? "P8 heading sweep OK"
                                      : "P8 heading sweep FAIL");
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
  std::uint32_t last_drive_health_ms = 0;
  bool claw_toggle_was_pressed = false;
  clamp_piston.set_value(true);
  clamp_output_high.store(true, std::memory_order_release);

  while (true) {
    const bool pose_editor_active = update_runtime_pose_editor(master);
    const bool piston_toggle_pressed =
        !pose_editor_active && !opcontrol_auton_running &&
        !master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    if (piston_toggle_pressed) {
      const bool next_output_high =
          !clamp_output_high.load(std::memory_order_acquire);
      clamp_output_high.store(next_output_high, std::memory_order_release);
      clamp_piston.set_value(next_output_high);
    }
    const bool auton_combo_pressed =
        ENABLE_CONTROLLER_MOTION_DIAGNOSTICS && !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (auton_combo_pressed && !auton_combo_was_pressed) {
      start_opcontrol_auton();
    }
    auton_combo_was_pressed = auton_combo_pressed;

    const bool fusion_test_combo_pressed =
        ENABLE_CONTROLLER_MOTION_DIAGNOSTICS && !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (fusion_test_combo_pressed && !fusion_test_combo_was_pressed) {
      start_fusion_test_auton();
    }
    fusion_test_combo_was_pressed = fusion_test_combo_pressed;

    const bool pid_tune_combo_pressed =
        ENABLE_CONTROLLER_MOTION_DIAGNOSTICS && !pose_editor_active &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (pid_tune_combo_pressed && !pid_tune_combo_was_pressed) {
      start_pid_autotune();
    }
    pid_tune_combo_was_pressed = pid_tune_combo_pressed;

    if (!opcontrol_auton_running) {
      constexpr double kOpcontrolDriveScale = 1.0;
      constexpr int kMechanismPower = 127;
      const std::uint32_t control_now_ms = pros::millis();
      const bool claw_toggle_pressed =
          !pose_editor_active &&
          master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
      if (claw_toggle_pressed && !claw_toggle_was_pressed) {
        set_claw_piston(!claw_piston_extended.load(
            std::memory_order_acquire));
      }
      claw_toggle_was_pressed = claw_toggle_pressed;
      const bool wrist_up_manual_negative =
          !pose_editor_active &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
          master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
      const bool wrist_down_manual_positive =
          !pose_editor_active &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
          master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
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
              ? kMechanismPower
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !auton_combo_pressed
                     ? -kMechanismPower
                     : 0);
      move_intake(requested_intake_power);

      const int slider_power =
          master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
              ? kMechanismPower
              : (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
                     ? -kMechanismPower
                     : 0);
      cascade_lift::set_manual_power(!pose_editor_active ? slider_power : 0);
      cascade_lift::update();
      cascade_lift::print_telemetry_if_due();

      if (wrist_up_manual_negative) {
        // PID is intentionally disabled while the new 11 W motor is tuned.
        // Up and Down provide direct motion in either direction.
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        claw_arm.move(-kMechanismPower);
      } else if (wrist_down_manual_positive) {
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        claw_arm.move(kMechanismPower);
      } else if (pose_editor_active) {
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        claw_arm.move(0);
      } else {
        // Passive braking only: do not run either of the previous wrist PID
        // targets until gains are retuned for the heavier 11 W mechanism.
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        claw_arm.move(0);
      }
    }

    print_distance_frame();
    const std::uint32_t drive_health_now_ms = pros::millis();
    if (drive_health_now_ms - last_drive_health_ms >= 5000) {
      print_drive_motor_health("periodic_stationary_or_opcontrol");
      std::printf("CLAW_ROTATION port=5 absolute_deg=%.2f\n",
                  static_cast<double>(horizontal_odom.get_angle()) / 100.0);
      std::fflush(stdout);
      last_drive_health_ms = drive_health_now_ms;
    }
    ai_vision_shadow_update();
    if (!opcontrol_auton_running) {
      localization_telemetry_update();
    }
    pros::delay(SAMPLE_PERIOD_MS);
  }
}
