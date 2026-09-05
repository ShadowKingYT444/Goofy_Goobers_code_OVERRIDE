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
              14,
              localization::kDriveWheelDiameterIn,
              localization::kDriveRpm,
              localization::kDriveExternalRatio);
pros::Distance distance_1(localization::kRearDistancePort);
pros::Gps gps_7(localization::kGpsPort);
pros::Rotation claw_arm_rotation(5);
pros::Rotation horizontal_odom(15);

namespace {
constexpr std::uint32_t SAMPLE_PERIOD_MS = 20;
constexpr std::uint32_t TELEMETRY_PERIOD_MS = 50;
constexpr bool RUN_STARTUP_LIDAR_CALIBRATION = false;
constexpr bool RUN_STARTUP_FORWARD_CALIBRATION = false;
// Diagnostic-only. Enable for one supervised boot, then restore false.
constexpr bool RUN_STARTUP_AI_VISION_SCAN = false;
// One supervised reversible heading sweep for live P6 characterization.
constexpr bool RUN_STARTUP_AI_VISION_HEADING_SWEEP = false;
// Stationary two-camera AprilTag characterization. This never commands an
// actuator and is enabled only for a supervised diagnostic boot.
constexpr bool RUN_STARTUP_DUAL_AI_VISION_TEST = false;
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
// One supervised end-to-end L2+B route trial. Enable only for a recorded
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
// One supervised, webcam-recorded P15 closed-loop 12-inch forward check.
// Restore false and reinstall the safe image immediately after the run.
constexpr bool RUN_STARTUP_P15_FORWARD_12_TEST = false;
// Straight encoder-only retreat used only when P6 is unavailable during a
// supervised Toggle verification. Restore false immediately after one run.
constexpr bool RUN_STARTUP_TOGGLE_RETREAT_TEST = false;
// Second half of the supervised verification: return to the P1 target and
// perform one current/travel-bounded Toggle strike.
constexpr bool RUN_STARTUP_TOGGLE_STRIKE_TEST = false;
// One supervised sub-inch incremental static-friction characterization.
constexpr bool RUN_STARTUP_DRIVE_BREAKAWAY_TEST = false;
// One supervised encoder/IMU propagation test with opportunistic stationary
// GPS/P6 correction. Restore false immediately after the single run.
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

std::atomic<bool> opcontrol_auton_running{false};
// ADI-D is physically inverted: high retracts the cylinder, low extends it.
std::atomic<bool> clamp_output_high{true};

enum class AutonSelection : int {
  kOnePinRed = 0,
  kTwoCupRed = 1,
  kOnePinBlue = 2,
  kTwoCupBlue = 3,
};

constexpr int kAutonCount = 4;

constexpr AutonSelection next_auton_selection(AutonSelection current) {
  return static_cast<AutonSelection>(
      (static_cast<int>(current) + 1) % kAutonCount);
}

// Compile-time proof of the exact competition selector order and wraparound.
static_assert(next_auton_selection(AutonSelection::kOnePinRed) ==
              AutonSelection::kTwoCupRed);
static_assert(next_auton_selection(AutonSelection::kTwoCupRed) ==
              AutonSelection::kOnePinBlue);
static_assert(next_auton_selection(AutonSelection::kOnePinBlue) ==
              AutonSelection::kTwoCupBlue);
static_assert(next_auton_selection(AutonSelection::kTwoCupBlue) ==
              AutonSelection::kOnePinRed);

std::atomic<int> selected_auton{
    static_cast<int>(AutonSelection::kTwoCupRed)};
std::atomic<bool> auton_selection_locked{false};
std::atomic<int> detected_distance_port{0};
std::atomic<int> detected_ai_vision_port{0};

const char* selected_auton_name() {
  switch (static_cast<AutonSelection>(
      selected_auton.load(std::memory_order_acquire))) {
    case AutonSelection::kOnePinRed: return "1 Pin Auto Red";
    case AutonSelection::kTwoCupRed: return "2 Cup Auto Red";
    case AutonSelection::kOnePinBlue: return "1 Pin Auto Blue";
    case AutonSelection::kTwoCupBlue: return "2 Cup Auto Blue";
  }
  return "UNKNOWN AUTON";
}

void render_auton_selection() {
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase_rect(0, 0, 479, 239);
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "AUTON SELECT");
  const auto selection = static_cast<AutonSelection>(
      selected_auton.load(std::memory_order_acquire));
  pros::screen::set_pen(
      selection == AutonSelection::kOnePinBlue ||
              selection == AutonSelection::kTwoCupBlue
                            ? pros::Color::blue
                            : pros::Color::red);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "%s",
                      selected_auton_name());
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5,
                      "< LEFT       RIGHT >");
  if (selection == AutonSelection::kTwoCupRed ||
      selection == AutonSelection::kTwoCupBlue) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7,
                        "TEST LIMIT: THROUGH 2ND SCORE");
  } else if (selection == AutonSelection::kOnePinBlue) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7,
                        "FULL 1-PIN BLUE MIRROR");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7,
                        "FULL 1-PIN ROUTE");
  }
  pros::screen::print(
      pros::E_TEXT_MEDIUM_CENTER, 9, "DIST P%d   AI P%d",
      detected_distance_port.load(std::memory_order_acquire),
      detected_ai_vision_port.load(std::memory_order_acquire));
}

void advance_auton() {
  if (auton_selection_locked.load(std::memory_order_acquire) ||
      opcontrol_auton_running) return;
  const auto current = static_cast<AutonSelection>(
      selected_auton.load(std::memory_order_acquire));
  const int selected = static_cast<int>(next_auton_selection(current));
  selected_auton.store(selected, std::memory_order_release);
  render_auton_selection();
  std::printf("AUTON_SELECTOR selected=%d name=%s\n",
              selected, selected_auton_name());
  std::fflush(stdout);
}

bool run_selected_auton();

int apply_controller_deadband(int value) {
  return std::abs(value) <= CONTROLLER_DRIVE_DEADBAND ? 0 : value;
}

// Port-5 absolute angle recorded with the arm physically held at the desired
// right-arrow pickup position on 2026-09-04. The 11 W green-cartridge motor
// and claw mechanism hold their load mechanically, so this controller uses no
// gravity or static feedforward.
constexpr double kArmRightTargetDeg = 21.18;
// Port-5 absolute angle recorded at the desired default/start-of-match pickup
// orientation on 2026-09-04. Autonomous and opcontrol actively hold it.
constexpr double kArmNormalTargetDeg = 64.7;
constexpr double kArmNormalToleranceDeg = 5.0;
constexpr double kArmPositionKp = 1.35;
constexpr double kArmPositionKd = 0.10;
constexpr double kArmPositionMaxPower = 70.0;
constexpr double kArmPositionMinPower = 16.0;
// P5 angle increases when the arm returns down toward its 66-degree
// match/default pose. Give only that direction a quicker response.
constexpr double kArmPositionDownKp = 2.10;
constexpr double kArmPositionDownMaxPower = 110.0;
constexpr double kArmPositionDownMinPower = 24.0;
constexpr double kArmPositionBrakeBandDeg = 1.25;

double shortest_arm_error_deg(double target_deg, double current_deg) {
  double error_deg = target_deg - current_deg;
  while (error_deg > 180.0) error_deg -= 360.0;
  while (error_deg < -180.0) error_deg += 360.0;
  return error_deg;
}

struct ArmPositionController {
  bool initialized = false;
  std::uint32_t previous_ms = 0;
  std::uint32_t last_log_ms = 0;
  double previous_error_deg = 0.0;
  double filtered_derivative_deg_s = 0.0;

  void reset() {
    initialized = false;
    filtered_derivative_deg_s = 0.0;
  }

  double update(double target_deg) {
    const std::uint32_t now_ms = pros::millis();
    const double angle_deg =
        static_cast<double>(claw_arm_rotation.get_angle()) / 100.0;
    const double error_deg = shortest_arm_error_deg(target_deg, angle_deg);
    claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    if (!initialized) {
      initialized = true;
      previous_ms = now_ms;
      previous_error_deg = error_deg;
      filtered_derivative_deg_s = 0.0;
    }
    const double dt_s = std::clamp(
        static_cast<double>(now_ms - previous_ms) / 1000.0, 0.005, 0.100);
    const double raw_derivative_deg_s =
        (error_deg - previous_error_deg) / dt_s;
    // Filtering the sensor derivative is important on the stronger motor: an
    // unfiltered D term caused power reversals and visible oscillation.
    filtered_derivative_deg_s +=
        0.18 * (raw_derivative_deg_s - filtered_derivative_deg_s);

    const bool moving_down_toward_normal = error_deg > 0.0;
    const double proportional_gain = moving_down_toward_normal
        ? kArmPositionDownKp : kArmPositionKp;
    const double maximum_power = moving_down_toward_normal
        ? kArmPositionDownMaxPower : kArmPositionMaxPower;
    const double minimum_power = moving_down_toward_normal
        ? kArmPositionDownMinPower : kArmPositionMinPower;
    double output = proportional_gain * error_deg +
        kArmPositionKd * filtered_derivative_deg_s;
    output = std::clamp(output, -maximum_power, maximum_power);
    if (std::fabs(error_deg) <= kArmPositionBrakeBandDeg) {
      output = 0.0;
    } else if (std::fabs(output) < minimum_power) {
      output = std::copysign(minimum_power, error_deg);
    }
    claw_arm.move(static_cast<int>(std::lround(output)));

    if (now_ms - last_log_ms >= 100) {
      std::printf(
          "ARM_PID target=%.2f angle=%.2f error=%.2f derivative=%.2f "
          "output=%.0f\n",
          target_deg, angle_deg, error_deg, filtered_derivative_deg_s,
          output);
      std::fflush(stdout);
      last_log_ms = now_ms;
    }
    previous_ms = now_ms;
    previous_error_deg = error_deg;
    return error_deg;
  }
};

bool run_selected_auton() {
  const auto selected = static_cast<AutonSelection>(
      selected_auton.load(std::memory_order_acquire));
  // Retracted is the default/holding state for the ADI-E claw and retains the
  // preload until the route explicitly releases it at the first Goal.
  set_claw_piston(false);
  // Competition setup places the cascade at its physical bottom. Establish
  // that exact position as Port-16 zero at the autonomous handoff so Stage 1
  // cannot inherit a stale relative offset and run toward the upper limit.
  cascade_lift::initialize_at_rest();

  // Hold the arm at its absolute normal orientation for the complete route.
  // This replaces the old timed full-power lowering pulse and avoids resetting
  // the port-5 rotation sensor's relative position.
  std::atomic<bool> arm_hold_running{true};
  pros::Task arm_normal_hold_task([&arm_hold_running] {
    ArmPositionController controller;
    while (arm_hold_running.load(std::memory_order_acquire)) {
      controller.update(kArmNormalTargetDeg);
      pros::delay(20);
    }
    claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw_arm.move(0);
  }, "auton arm hold");

  // ADI-D is active-low. Extend before either autonomous, allow the cylinder
  // 0.6 seconds to reach full travel, and leave it extended for the route.
  clamp_piston.set_value(false);
  clamp_output_high.store(false, std::memory_order_release);
  pros::delay(600);

  // A hotkey test often begins after the robot was repositioned by hand. Motor
  // encoders retain that motion and can have harmless unequal absolute offsets,
  // which the fused-navigation preflight otherwise rejects before any drive
  // command. Let the faster asynchronous arm reach its normal pose before
  // anchoring navigation; arm motion during this handoff can shake the IMU and
  // make the first turn fail before it ever commands the drivetrain.
  const std::uint32_t arm_settle_started_ms = pros::millis();
  while (pros::millis() - arm_settle_started_ms < 800) {
    const double arm_angle_deg =
        static_cast<double>(claw_arm_rotation.get_angle()) / 100.0;
    if (std::fabs(shortest_arm_error_deg(
            kArmNormalTargetDeg, arm_angle_deg)) <=
        kArmNormalToleranceDeg) {
      break;
    }
    pros::delay(20);
  }
  chassis.drive_set(0, 0);
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  int drive_tare_errors = 0;
  for (auto& motor : chassis.left_motors) {
    if (motor.tare_position() != 1) ++drive_tare_errors;
  }
  for (auto& motor : chassis.right_motors) {
    if (motor.tare_position() != 1) ++drive_tare_errors;
  }
  pros::delay(100);
  std::printf("AUTON_DRIVE_ANCHOR tare_errors=%d L=%.2f/%.2f R=%.2f/%.2f\n",
              drive_tare_errors,
              chassis.left_motors[0].get_position(),
              chassis.left_motors[1].get_position(),
              chassis.right_motors[0].get_position(),
              chassis.right_motors[1].get_position());
  std::printf("ARM_NORMAL target=%.2f absolute_deg=%.2f tolerance=%.1f\n",
              kArmNormalTargetDeg,
              static_cast<double>(claw_arm_rotation.get_angle()) / 100.0,
              kArmNormalToleranceDeg);
  std::printf("AUTON_SELECTOR run=%d name=%s\n",
              static_cast<int>(selected), selected_auton_name());
  std::fflush(stdout);
  bool success = false;
  switch (selected) {
    case AutonSelection::kOnePinRed:
      success = localization_simple_red_goal_hotkey_auton();
      break;
    case AutonSelection::kTwoCupRed:
      success = localization_two_cup_red_auton();
      break;
    case AutonSelection::kOnePinBlue:
      success = localization_simple_blue_goal_hotkey_auton();
      break;
    case AutonSelection::kTwoCupBlue:
      success = localization_two_cup_blue_auton();
      break;
  }
  arm_hold_running.store(false, std::memory_order_release);
  arm_normal_hold_task.join();
  return success;
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

void run_toggle_retreat_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTargetIn = 18.0;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr int kBasePower = -45;
  constexpr std::uint32_t kTimeoutMs = 6000;
  constexpr std::uint32_t kStallMs = 650;
  constexpr double kProgressStepIn = 0.20;
  constexpr double kMaximumSideMismatchIn = 2.0;

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
  auto side_position = [](const auto& motors) {
    return 0.5 * (motors[0].get_position() + motors[1].get_position());
  };
  auto side_spread = [](const auto& motors) {
    return std::fabs(motors[0].get_position() - motors[1].get_position());
  };

  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_and_brake();
  const double left_zero = side_position(chassis.left_motors);
  const double right_zero = side_position(chassis.right_motors);
  const long p1_start_mm = distance_1.get_distance();
  const bool preflight = distance_1.is_installed() &&
      p1_start_mm >= 10 && p1_start_mm <= 220 &&
      std::isfinite(left_zero) && std::isfinite(right_zero) &&
      side_spread(chassis.left_motors) <= 15.0 &&
      side_spread(chassis.right_motors) <= 15.0;
  std::printf("TOGGLE_RETREAT event=preflight ok=%d p1_mm=%ld\n",
              static_cast<int>(preflight), p1_start_mm);
  std::fflush(stdout);
  if (!preflight) return;

  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_progress_ms = started_ms;
  std::uint32_t last_log_ms = 0;
  double last_progress_in = 0.0;
  bool reached = false;
  const char* abort_reason = "none";
  while (pros::millis() - started_ms < kTimeoutMs) {
    const double left_in =
        (side_position(chassis.left_motors) - left_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double right_in =
        (side_position(chassis.right_motors) - right_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double retreat_in = -0.5 * (left_in + right_in);
    const double side_mismatch_in = std::fabs(left_in - right_in);
    if (retreat_in >= kTargetIn) {
      reached = true;
      break;
    }
    if (side_spread(chassis.left_motors) > 15.0 ||
        side_spread(chassis.right_motors) > 15.0 ||
        side_mismatch_in > kMaximumSideMismatchIn) {
      abort_reason = "encoder_mismatch";
      break;
    }
    const std::uint32_t now = pros::millis();
    if (retreat_in >= last_progress_in + kProgressStepIn) {
      last_progress_in = retreat_in;
      last_progress_ms = now;
    }
    if (now - last_progress_ms >= kStallMs) {
      abort_reason = "stall";
      break;
    }
    const int correction = static_cast<int>(std::clamp(
        -(left_in - right_in) * 8.0, -12.0, 12.0));
    for (auto& motor : chassis.left_motors) motor.move(kBasePower + correction);
    for (auto& motor : chassis.right_motors) motor.move(kBasePower - correction);
    if (now - last_log_ms >= 100) {
      std::printf(
          "TOGGLE_RETREAT event=motion retreat=%.3f left=%.3f right=%.3f "
          "mismatch=%.3f correction=%d\n",
          retreat_in, left_in, right_in, side_mismatch_in, correction);
      std::fflush(stdout);
      last_log_ms = now;
    }
    pros::delay(10);
  }
  stop_and_brake();
  const double left_final_in =
      (side_position(chassis.left_motors) - left_zero) / 360.0 *
      kWheelCircumferenceIn;
  const double right_final_in =
      (side_position(chassis.right_motors) - right_zero) / 360.0 *
      kWheelCircumferenceIn;
  std::printf(
      "TOGGLE_RETREAT event=complete reached=%d retreat=%.3f left=%.3f "
      "right=%.3f abort=%s p1_mm=%ld\n",
      static_cast<int>(reached), -0.5 * (left_final_in + right_final_in),
      left_final_in, right_final_in, abort_reason, distance_1.get_distance());
  std::fflush(stdout);
  print_drive_motor_health("toggle_retreat_complete");
}

void run_toggle_strike_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWheelCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr double kMaximumTotalTravelIn = 32.0;
  constexpr double kMaximumRamTravelIn = 5.0;
  constexpr long kNearToggleMm = 170;
  constexpr int kApproachPower = 55;
  constexpr int kStrikePower = 127;
  constexpr double kMandatoryRetreatIn = 10.0;
  constexpr std::int32_t kContactCurrentMa = 2200;
  constexpr std::uint32_t kTimeoutMs = 6500;
  constexpr std::uint32_t kStallMs = 650;

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
  auto side_position = [](const auto& motors) {
    return 0.5 * (motors[0].get_position() + motors[1].get_position());
  };
  auto side_spread = [](const auto& motors) {
    return std::fabs(motors[0].get_position() - motors[1].get_position());
  };
  auto maximum_drive_current = []() {
    std::int32_t maximum = 0;
    for (auto& motor : chassis.left_motors)
      maximum = std::max(maximum, motor.get_current_draw());
    for (auto& motor : chassis.right_motors)
      maximum = std::max(maximum, motor.get_current_draw());
    return maximum;
  };

  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE, true);
  stop_and_brake();
  const double left_zero = side_position(chassis.left_motors);
  const double right_zero = side_position(chassis.right_motors);
  const long p1_start_mm = distance_1.get_distance();
  const bool preflight = distance_1.is_installed() &&
      p1_start_mm > kNearToggleMm && p1_start_mm <= 2000 &&
      std::isfinite(left_zero) && std::isfinite(right_zero) &&
      side_spread(chassis.left_motors) <= 15.0 &&
      side_spread(chassis.right_motors) <= 15.0;
  std::printf("TOGGLE_STRIKE event=preflight ok=%d p1_mm=%ld\n",
              static_cast<int>(preflight), p1_start_mm);
  std::fflush(stdout);
  if (!preflight) return;

  // Recreate the along-wall contact location from the camera-verified
  // 2026-08-28 blue->red hit. P6 is unavailable, so the short dog-leg is
  // bounded independently by the paired drivetrain encoders.
  auto run_encoder_segment = [&](const char* phase, int left_power,
                                 int right_power, double target_in) {
    const double left_start = side_position(chassis.left_motors);
    const double right_start = side_position(chassis.right_motors);
    const std::uint32_t segment_start_ms = pros::millis();
    bool reached = false;
    while (pros::millis() - segment_start_ms < 4000) {
      const double left_in =
          (side_position(chassis.left_motors) - left_start) / 360.0 *
          kWheelCircumferenceIn;
      const double right_in =
          (side_position(chassis.right_motors) - right_start) / 360.0 *
          kWheelCircumferenceIn;
      const double progress_in =
          0.5 * (std::fabs(left_in) + std::fabs(right_in));
      if (progress_in >= target_in) {
        reached = true;
        break;
      }
      if (side_spread(chassis.left_motors) > 15.0 ||
          side_spread(chassis.right_motors) > 15.0 ||
          std::fabs(std::fabs(left_in) - std::fabs(right_in)) > 2.0) {
        break;
      }
      for (auto& motor : chassis.left_motors) motor.move(left_power);
      for (auto& motor : chassis.right_motors) motor.move(right_power);
      pros::delay(10);
    }
    stop_and_brake();
    pros::delay(150);
    std::printf("TOGGLE_STRIKE event=shift phase=%s reached=%d p1_mm=%ld\n",
                phase, static_cast<int>(reached), distance_1.get_distance());
    std::fflush(stdout);
    return reached;
  };
  constexpr double kThirtyDegreeArcIn =
      3.14159265358979323846 * localization::kDriveTrackWidthIn / 12.0;
  bool shift_ok = run_encoder_segment("clear_reverse", -45, -45, 10.0);
  if (shift_ok) {
    shift_ok = run_encoder_segment(
        "turn_left_30", -45, 45, kThirtyDegreeArcIn);
  }
  if (shift_ok) {
    shift_ok = run_encoder_segment("diagonal_forward", 50, 50, 17.0);
  }
  if (shift_ok) {
    shift_ok = run_encoder_segment(
        "turn_right_30", 45, -45, kThirtyDegreeArcIn);
  }

  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_progress_ms = started_ms;
  std::uint32_t last_log_ms = 0;
  std::uint32_t strike_started_ms = 0;
  double last_progress_in = 0.0;
  double strike_start_in = 0.0;
  bool near_toggle = false;
  bool contact_current = false;
  bool ram_limit = false;
  const char* abort_reason = shift_ok ? "none" : "shift_failed";
  while (shift_ok && pros::millis() - started_ms < kTimeoutMs) {
    const double left_in =
        (side_position(chassis.left_motors) - left_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double right_in =
        (side_position(chassis.right_motors) - right_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double forward_in = 0.5 * (left_in + right_in);
    const double mismatch_in = std::fabs(left_in - right_in);
    const long p1_mm = distance_1.get_distance();
    const std::uint32_t now = pros::millis();
    if (!near_toggle && forward_in >= 8.0 && p1_mm >= 20 &&
        p1_mm <= kNearToggleMm) {
      near_toggle = true;
      strike_start_in = forward_in;
      strike_started_ms = now;
      std::printf("TOGGLE_STRIKE event=near travel=%.3f p1_mm=%ld\n",
                  forward_in, p1_mm);
      std::fflush(stdout);
    }
    const double ram_travel_in = near_toggle
        ? forward_in - strike_start_in : 0.0;
    const std::int32_t current_ma = maximum_drive_current();
    if (near_toggle && now - strike_started_ms >= 100 &&
        current_ma >= kContactCurrentMa) {
      contact_current = true;
      break;
    }
    if (near_toggle && ram_travel_in >= kMaximumRamTravelIn) {
      ram_limit = true;
      break;
    }
    if (forward_in >= kMaximumTotalTravelIn) {
      abort_reason = "travel_limit";
      break;
    }
    if (side_spread(chassis.left_motors) > 15.0 ||
        side_spread(chassis.right_motors) > 15.0 || mismatch_in > 2.0) {
      abort_reason = "encoder_mismatch";
      break;
    }
    if (forward_in >= last_progress_in + 0.20) {
      last_progress_in = forward_in;
      last_progress_ms = now;
    }
    if (now - last_progress_ms >= kStallMs) {
      abort_reason = "stall";
      break;
    }
    const int base_power = near_toggle ? kStrikePower : kApproachPower;
    const int correction = static_cast<int>(std::clamp(
        -(left_in - right_in) * 8.0, -12.0, 12.0));
    for (auto& motor : chassis.left_motors) motor.move(base_power + correction);
    for (auto& motor : chassis.right_motors) motor.move(base_power - correction);
    if (now - last_log_ms >= 100) {
      std::printf(
          "TOGGLE_STRIKE event=motion travel=%.3f left=%.3f right=%.3f "
          "mismatch=%.3f p1_mm=%ld near=%d ram=%.3f current=%ld\n",
          forward_in, left_in, right_in, mismatch_in, p1_mm,
          static_cast<int>(near_toggle), ram_travel_in,
          static_cast<long>(current_ma));
      std::fflush(stdout);
      last_log_ms = now;
    }
    pros::delay(10);
  }
  stop_and_brake();
  pros::delay(250);
  const double left_final_in =
      (side_position(chassis.left_motors) - left_zero) / 360.0 *
      kWheelCircumferenceIn;
  const double right_final_in =
      (side_position(chassis.right_motors) - right_zero) / 360.0 *
      kWheelCircumferenceIn;

  // Every physical Toggle attempt must finish by clearing the field element,
  // regardless of whether contact current, travel, or another guard ended the
  // forward phase. This also makes the camera's final color check unobstructed.
  const double retreat_left_zero = side_position(chassis.left_motors);
  const double retreat_right_zero = side_position(chassis.right_motors);
  const std::uint32_t retreat_started_ms = pros::millis();
  bool retreat_complete = false;
  while (pros::millis() - retreat_started_ms < 4000) {
    const double retreat_left_in =
        -(side_position(chassis.left_motors) - retreat_left_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double retreat_right_in =
        -(side_position(chassis.right_motors) - retreat_right_zero) / 360.0 *
        kWheelCircumferenceIn;
    const double retreat_in = 0.5 * (retreat_left_in + retreat_right_in);
    if (retreat_in >= kMandatoryRetreatIn) {
      retreat_complete = true;
      break;
    }
    if (side_spread(chassis.left_motors) > 15.0 ||
        side_spread(chassis.right_motors) > 15.0 ||
        std::fabs(retreat_left_in - retreat_right_in) > 2.0) {
      break;
    }
    const int correction = static_cast<int>(std::clamp(
        -(retreat_left_in - retreat_right_in) * 8.0, -12.0, 12.0));
    for (auto& motor : chassis.left_motors) motor.move(-55 - correction);
    for (auto& motor : chassis.right_motors) motor.move(-55 + correction);
    pros::delay(10);
  }
  stop_and_brake();
  const double retreat_left_in =
      -(side_position(chassis.left_motors) - retreat_left_zero) / 360.0 *
      kWheelCircumferenceIn;
  const double retreat_right_in =
      -(side_position(chassis.right_motors) - retreat_right_zero) / 360.0 *
      kWheelCircumferenceIn;
  std::printf(
      "TOGGLE_STRIKE event=complete success=%d near=%d current_stop=%d "
      "ram_limit=%d travel=%.3f left=%.3f right=%.3f abort=%s "
      "retreat_ok=%d retreat=%.3f retreat_mismatch=%.3f p1_mm=%ld\n",
      static_cast<int>(near_toggle && (contact_current || ram_limit)),
      static_cast<int>(near_toggle), static_cast<int>(contact_current),
      static_cast<int>(ram_limit), 0.5 * (left_final_in + right_final_in),
      left_final_in, right_final_in, abort_reason,
      static_cast<int>(retreat_complete),
      0.5 * (retreat_left_in + retreat_right_in),
      std::fabs(retreat_left_in - retreat_right_in),
      distance_1.get_distance());
  std::fflush(stdout);
  print_drive_motor_health("toggle_strike_complete");
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

void run_p15_forward_12_test() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kOdomCircumferenceIn = kPi * 2.0;
  constexpr double kDriveCircumferenceIn =
      kPi * localization::kDriveWheelDiameterIn;
  constexpr double kOdomTransferRatio =
      localization::kForwardOdomSensorToGroundRatio;
  constexpr double kTargetIn = 6.0;
  // The first run coasted 0.50 corrected inches after brake engagement.
  constexpr double kStopLeadIn = 0.48;
  constexpr double kMaximumEncoderTravelIn = 10.0;
  constexpr std::uint32_t kTimeoutMs = 10000;
  constexpr std::uint32_t kStallTimeoutMs = 700;

  auto stop_and_brake = [] {
    for (auto& motor : chassis.left_motors) {
      motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      motor.brake();
    }
    for (auto& motor : chassis.right_motors) {
      motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      motor.brake();
    }
  };
  auto side_position = [](const auto& motors) {
    return 0.5 * (motors[0].get_position() + motors[1].get_position());
  };

  stop_and_brake();
  if (!horizontal_odom.is_installed()) {
    std::printf("P15_FORWARD abort=odom_missing\n");
    std::fflush(stdout);
    return;
  }
  const std::int32_t odom_zero = horizontal_odom.get_position();
  const double left_zero = side_position(chassis.left_motors);
  const double right_zero = side_position(chassis.right_motors);
  if (odom_zero == static_cast<std::int32_t>(PROS_ERR) ||
      !std::isfinite(left_zero) || !std::isfinite(right_zero)) {
    std::printf("P15_FORWARD abort=preflight_invalid\n");
    std::fflush(stdout);
    return;
  }

  std::printf("P15_FORWARD event=start odom_zero=%ld target=%.2f\n",
              static_cast<long>(odom_zero), kTargetIn);
  std::fflush(stdout);
  const std::uint32_t started_ms = pros::millis();
  std::uint32_t last_progress_ms = started_ms;
  std::uint32_t last_log_ms = 0;
  double last_progress_in = 0.0;
  const char* abort_reason = "none";
  bool reached = false;
  while (pros::millis() - started_ms < kTimeoutMs) {
    const std::int32_t odom_now = horizontal_odom.get_position();
    if (odom_now == static_cast<std::int32_t>(PROS_ERR)) {
      abort_reason = "odom_fault";
      break;
    }
    const double odom_in =
        std::fabs(static_cast<double>(odom_now - odom_zero)) / 100.0 /
        360.0 * kOdomCircumferenceIn * kOdomTransferRatio;
    const double left_in = std::fabs(side_position(chassis.left_motors) -
                                     left_zero) /
                           360.0 * kDriveCircumferenceIn;
    const double right_in = std::fabs(side_position(chassis.right_motors) -
                                      right_zero) /
                            360.0 * kDriveCircumferenceIn;
    const double encoder_in = 0.5 * (left_in + right_in);
    if (odom_in >= kTargetIn - kStopLeadIn) {
      reached = true;
      break;
    }
    if (encoder_in > kMaximumEncoderTravelIn) {
      abort_reason = "encoder_travel_guard";
      break;
    }
    const std::uint32_t now = pros::millis();
    if (odom_in >= last_progress_in + 0.02) {
      last_progress_in = odom_in;
      last_progress_ms = now;
    }
    if (now - last_progress_ms >= kStallTimeoutMs) {
      abort_reason = "odom_stall";
      break;
    }
    const double remaining_in = kTargetIn - odom_in;
    const int base_power = remaining_in > 3.0 ? 48
                           : remaining_in > 1.0 ? 32
                                                : 20;
    const int correction = static_cast<int>(std::clamp(
        (right_in - left_in) * 8.0, -10.0, 10.0));
    for (auto& motor : chassis.left_motors)
      motor.move(base_power + correction);
    for (auto& motor : chassis.right_motors)
      motor.move(base_power - correction);
    if (now - last_log_ms >= 100) {
      std::printf(
          "P15_FORWARD event=motion odom=%.3f encoder=%.3f left=%.3f "
          "right=%.3f power=%d\n",
          odom_in, encoder_in, left_in, right_in, base_power);
      std::fflush(stdout);
      last_log_ms = now;
    }
    pros::delay(10);
  }
  stop_and_brake();
  pros::delay(500);
  const std::int32_t odom_final = horizontal_odom.get_position();
  const double final_odom_in =
      std::fabs(static_cast<double>(odom_final - odom_zero)) / 100.0 /
      360.0 * kOdomCircumferenceIn * kOdomTransferRatio;
  const double final_left_in =
      std::fabs(side_position(chassis.left_motors) - left_zero) / 360.0 *
      kDriveCircumferenceIn;
  const double final_right_in =
      std::fabs(side_position(chassis.right_motors) - right_zero) / 360.0 *
      kDriveCircumferenceIn;
  std::printf(
      "P15_FORWARD event=complete reached=%d odom=%.3f encoder=%.3f "
      "left=%.3f right=%.3f abort=%s\n",
      static_cast<int>(reached), final_odom_in,
      0.5 * (final_left_in + final_right_in), final_left_in, final_right_in,
      abort_reason);
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
  auto& imu14 = chassis.imu;

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

  const bool imu_installed = imu14.is_installed() && !imu14.is_calibrating();
  printf("STRAIGHT_IMU_INIT port=12 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu14.get_status()));
  fflush(stdout);

  for (double velocity_rpm : kVelocityRpm) {
  for (int target_in : kTargetsIn) {
    pros::lcd::print(6, "Sweep %din @ %.0f", target_in, velocity_rpm);
    const auto baseline = motor_positions();
    const std::int32_t baseline_h5 = horizontal_odom.get_position();
    const GpsSample start_gps = sample_gps();
    const double start_imu_deg = imu14.get_rotation();
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
          max_abs_imu_delta_deg, std::fabs(imu14.get_rotation() - start_imu_deg));
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
           imu14.get_rotation() - start_imu_deg, max_abs_imu_delta_deg,
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
          max_abs_imu_delta_deg, std::fabs(imu14.get_rotation() - start_imu_deg));
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
           imu14.get_rotation() - start_imu_deg, max_abs_imu_delta_deg,
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
  auto& imu14 = chassis.imu;

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
      const double imu_rad = imu14.get_heading() * kPi / 180.0;
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
  const bool imu_installed = imu14.is_installed() && !imu14.is_calibrating();
  printf("ROTATION_SWEEP_INIT imu_port=14 installed=%d status=%d\n",
         static_cast<int>(imu_installed), static_cast<int>(imu14.get_status()));
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

void print_distance_frame() {
  static std::uint32_t sample = 0;
  static std::uint32_t last_frame_ms = 0;
  const std::uint32_t now = pros::millis();
  if (last_frame_ms != 0 && now - last_frame_ms < TELEMETRY_PERIOD_MS) return;
  last_frame_ms = now;
  const DistanceReading rear_distance = read_sensor(distance_1);
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
      "m17=%.1f m18=%.1f m11=%.1f m13=%.1f h15=%ld h15abs=%.2f "
      "imu=%.2f rawimu=%.2f imust=%d "
      "imugyro=%.4f,%.4f,%.4f imuacc=%.4f,%.4f,%.4f "
      "gps7=%.4f,%.4f,%.2f,%.4f,%d errno=%d gpsgyro=%.2f\n",
      static_cast<unsigned long>(sample++),
      static_cast<unsigned long>(now),
      rear_distance.mm,
      rear_distance.confidence,
      static_cast<int>(rear_distance.installed),
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
    pros::lcd::print(0, "P1 REAR %4ldmm c%2ld %s", rear_distance.mm,
                     rear_distance.confidence,
                     rear_distance.installed ? "ok" : "no");
    pros::lcd::print(1, "GPS P7 e%.3fm", gps_error_m);
    pros::lcd::set_text(2, "IMU P14 / AI P6");
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
    simple_goal_avoidance_auton();
    chassis.drive_set(0, 0);
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
    fusion_test_auton();
    chassis.drive_set(0, 0);
    opcontrol_auton_running = false;
    pros::lcd::set_text(6, "Fusion test done");
  }, "fusion test");
}

void start_toggle_far_goal_auton() {
  if (opcontrol_auton_running) return;

  // This is the controller-launched equivalent of autonomous(): deploy at
  // the launch edge, then leave the active-low output latched for driver mode.
  clamp_piston.set_value(false);
  clamp_output_high.store(false, std::memory_order_release);
  std::printf("TOGGLE_STATE phase=hotkey_auton output=0 physical=extended\n");
  std::fflush(stdout);

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
  // Freeze the displayed selection before the asynchronous route starts.
  // Selection is unlocked only after this exact run has finished.
  auton_selection_locked.store(true, std::memory_order_release);
  opcontrol_auton_running = true;
  pros::Task::create([] {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::lcd::print(6, "Run: %s", selected_auton_name());
    controller.print(0, 0, "%-18s", selected_auton_name());
    chassis.drive_set(0, 0);
    const bool success = run_selected_auton();
    chassis.drive_set(0, 0);
    opcontrol_auton_running = false;
    auton_selection_locked.store(false, std::memory_order_release);
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
    chassis.drive_set(0, 0);
    const bool success = pid_autotune_auton();
    chassis.drive_set(0, 0);
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
    std::printf("P6_HEADING event=abort reason=imu_invalid\n");
    std::fflush(stdout);
    return false;
  }
  const double start_imu_deg = chassis.drive_imu_get();
  if (!std::isfinite(start_imu_deg)) {
    std::printf("P6_HEADING event=abort reason=imu_nonfinite\n");
    std::fflush(stdout);
    return false;
  }
  std::printf(
      "P6_HEADING event=start start_imu=%.2f guard_deg=%.1f velocity_rpm=%.1f\n",
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
        std::printf("P6_HEADING event=abort reason=angle_guard delta=%.2f\n",
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
        std::printf("P6_HEADING event=abort reason=stall target=%.1f delta=%.2f\n",
                    target_deg, delta_deg);
        std::fflush(stdout);
        return false;
      }
      pros::delay(SAMPLE_PERIOD_MS);
    }
    stop_drive_velocity();
    if (!reached) {
      std::printf("P6_HEADING event=abort reason=timeout target=%.1f\n",
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
        "P6_HEADING event=hold target=%.1f achieved=%.2f valid=%d changed=%d "
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
      "P6_HEADING event=done returned=%d final_delta=%.2f visible_legs=%d "
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

void print_smart_port_inventory(const char* phase) {
  int distance_port = 0;
  int ai_vision_port = 0;
  for (std::uint8_t port = 1; port <= 21; ++port) {
    const int type = static_cast<int>(pros::c::get_plugged_type(port));
    if (type == static_cast<int>(pros::c::E_DEVICE_DISTANCE)) {
      distance_port = port;
    } else if (type == static_cast<int>(pros::c::E_DEVICE_AIVISION)) {
      ai_vision_port = port;
    }
    if (type != 0) {
      std::printf("DEVICE_PORT phase=%s port=%u type=%d\n", phase,
                  static_cast<unsigned>(port), type);
    }
  }
  detected_distance_port.store(distance_port, std::memory_order_release);
  detected_ai_vision_port.store(ai_vision_port, std::memory_order_release);
  std::fflush(stdout);
}

struct DualAiDiagnosticObservation {
  bool valid{false};
  std::uint8_t port{0};
  int tag_id{-1};
  int object_count{0};
  double horizontal_range_in{NAN};
  double bearing_right_deg{NAN};
  const char* reason{"not_read"};
};

bool configure_dual_ai_diagnostic_port(std::uint8_t port) {
  if (static_cast<int>(pros::c::get_plugged_type(port)) !=
      static_cast<int>(pros::c::E_DEVICE_AIVISION)) {
    std::printf("AI_COMPARE event=config port=%u configured=0 reason=missing\n",
                static_cast<unsigned>(port));
    std::fflush(stdout);
    return false;
  }
  errno = 0;
  const int reset = pros::c::aivision_reset(port);
  const int family = pros::c::aivision_set_tag_family_override(
      port, pros::TAG_CIRCLE_21H7);
  const int disable = pros::c::aivision_disable_detection_types(
      port, pros::E_AIVISION_MODE_COLORS |
                pros::E_AIVISION_MODE_OBJECTS |
                pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enable = pros::c::aivision_enable_detection_types(
      port, pros::E_AIVISION_MODE_TAGS);
  const int enabled = pros::c::aivision_get_enabled_detection_types(port);
  const bool configured = reset == 1 && family == 1 && disable == 1 &&
                          enable == 1 &&
                          enabled == pros::E_AIVISION_MODE_TAGS;
  std::printf(
      "AI_COMPARE event=config port=%u configured=%d reset=%d family=%d "
      "disable=%d enable=%d enabled=%d errno=%d\n",
      static_cast<unsigned>(port), static_cast<int>(configured), reset,
      family, disable, enable, enabled, errno);
  std::fflush(stdout);
  return configured;
}

DualAiDiagnosticObservation read_dual_ai_diagnostic(std::uint8_t port) {
  DualAiDiagnosticObservation result;
  result.port = port;
  if (static_cast<int>(pros::c::get_plugged_type(port)) !=
      static_cast<int>(pros::c::E_DEVICE_AIVISION)) {
    result.reason = "missing";
    return result;
  }
  errno = 0;
  const int count = pros::c::aivision_get_object_count(port);
  result.object_count = count;
  if (count < 0 || count > 24) {
    result.reason = "count_error";
    return result;
  }
  bool found = false;
  pros::aivision_object_s_t best{};
  double best_area = -1.0;
  for (int index = 0; index < count; ++index) {
    const auto object = pros::c::aivision_get_object(
        port, static_cast<std::uint32_t>(index));
    if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
    const auto& tag = object.object.tag;
    const double twice_area =
        static_cast<double>(tag.x0) * tag.y1 -
        static_cast<double>(tag.y0) * tag.x1 +
        static_cast<double>(tag.x1) * tag.y2 -
        static_cast<double>(tag.y1) * tag.x2 +
        static_cast<double>(tag.x2) * tag.y3 -
        static_cast<double>(tag.y2) * tag.x3 +
        static_cast<double>(tag.x3) * tag.y0 -
        static_cast<double>(tag.y3) * tag.x0;
    const double area = std::fabs(twice_area) * 0.5;
    if (area > best_area) {
      best = object;
      best_area = area;
      found = true;
    }
  }
  if (!found) {
    result.reason = count == 0 ? "no_object" : "no_tag";
    return result;
  }

  const auto& tag = best.object.tag;
  const auto edge = [](int x0, int y0, int x1, int y1) {
    return std::hypot(static_cast<double>(x1 - x0),
                      static_cast<double>(y1 - y0));
  };
  const std::array<double, 4> edges{
      edge(tag.x0, tag.y0, tag.x1, tag.y1),
      edge(tag.x1, tag.y1, tag.x2, tag.y2),
      edge(tag.x2, tag.y2, tag.x3, tag.y3),
      edge(tag.x3, tag.y3, tag.x0, tag.y0)};
  const auto [minimum_edge, maximum_edge] =
      std::minmax_element(edges.begin(), edges.end());
  if (*minimum_edge < localization::kAiMinTagEdgePx ||
      *minimum_edge <= 0.0 ||
      *maximum_edge / *minimum_edge > localization::kAiMaxTagEdgeRatio ||
      best_area < 40.0) {
    result.reason = "geometry";
    return result;
  }
  const double horizontal_edge = 0.5 * (edges[0] + edges[2]);
  const double vertical_edge = 0.5 * (edges[1] + edges[3]);
  const double center_x = 0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
  const double depth = 0.5 *
      (localization::kAiFocalLengthXPx *
           localization::kAiTagDetectedSizeIn / horizontal_edge +
       localization::kAiFocalLengthYPx *
           localization::kAiTagDetectedSizeIn / vertical_edge);
  const double right = depth *
      (center_x - localization::kAiImageWidthPx * 0.5) /
      localization::kAiFocalLengthXPx;
  result.tag_id = best.id;
  result.horizontal_range_in = std::hypot(depth, right);
  result.bearing_right_deg = std::atan2(right, depth) * 180.0 /
                             3.14159265358979323846;
  if (!std::isfinite(result.horizontal_range_in) ||
      result.horizontal_range_in < localization::kAiMinUsableRangeIn ||
      result.horizontal_range_in > localization::kAiMaxUsableRangeIn) {
    result.reason = "range";
    return result;
  }
  result.valid = true;
  result.reason = "valid";
  return result;
}

bool dual_ai_robot_stationary() {
  constexpr double kMaximumMotorRpm = 2.0;
  for (const auto& motor : chassis.left_motors) {
    const double rpm = motor.get_actual_velocity();
    if (!std::isfinite(rpm) || std::fabs(rpm) > kMaximumMotorRpm) return false;
  }
  for (const auto& motor : chassis.right_motors) {
    const double rpm = motor.get_actual_velocity();
    if (!std::isfinite(rpm) || std::fabs(rpm) > kMaximumMotorRpm) return false;
  }
  const auto gyro = chassis.imu.get_gyro_rate();
  return std::isfinite(gyro.z) && std::fabs(gyro.z) <= 3.0;
}

void emit_dual_ai_pose_comparison(const DualAiDiagnosticObservation& vision,
                                  std::uint32_t stationary_ms) {
  const auto fused = navigation::current_pose();
  const auto gps_position = gps_7.get_position();
  const double gps_heading_cw_deg = gps_7.get_heading();
  const double gps_error_in = gps_7.get_error() * 39.37007874015748;
  const auto gps = localization::vex_gps_to_project_robot_pose(
      gps_position.x, gps_position.y, gps_heading_cw_deg);
  const bool gps_valid = gps_7.is_installed() &&
      std::isfinite(gps.x_in) && std::isfinite(gps.y_in) &&
      std::isfinite(gps.heading_deg) && std::isfinite(gps_error_in) &&
      gps_error_in >= 0.0 &&
      gps_error_in <= localization::kGpsMaxReportedErrorIn;
  const bool fused_valid = fused.valid && std::isfinite(fused.x_in) &&
                           std::isfinite(fused.y_in) &&
                           std::isfinite(fused.heading_deg);
  const double reference_heading_deg = fused_valid
      ? fused.heading_deg : (gps_valid ? gps.heading_deg : NAN);
  if (!vision.valid || !std::isfinite(reference_heading_deg)) {
    std::printf(
        "AI_COMPARE t=%lu stationary_ms=%lu port=%u tag=%d valid=0 "
        "reason=%s gps_valid=%d fused_valid=%d\n",
        static_cast<unsigned long>(pros::millis()),
        static_cast<unsigned long>(stationary_ms),
        static_cast<unsigned>(vision.port), vision.tag_id, vision.reason,
        static_cast<int>(gps_valid), static_cast<int>(fused_valid));
    std::fflush(stdout);
    return;
  }

  // P6 points robot-forward; P8 points robot-right. Camera translations are
  // intentionally zero until measured, so this qualification reports the
  // resulting disagreement instead of feeding it into autonomous control.
  const double camera_yaw_right_deg = vision.port == 8 ? 90.0 : 0.0;
  const double camera_heading_deg =
      reference_heading_deg - camera_yaw_right_deg;
  const double global_bearing_rad =
      (camera_heading_deg + vision.bearing_right_deg) *
      3.14159265358979323846 / 180.0;
  constexpr std::array<std::array<double, 2>, 4> kFaceNormals{{
      {{1.0, 0.0}}, {{0.0, 1.0}}, {{-1.0, 0.0}}, {{0.0, -1.0}},
  }};
  bool found_candidate = false;
  double best_x = NAN;
  double best_y = NAN;
  double best_gps_delta = NAN;
  double best_fused_delta = NAN;
  double best_score = INFINITY;
  int candidate_count = 0;
  for (const auto& landmark : localization::kGoalTagLandmarks) {
    if (landmark.tag_id != vision.tag_id) continue;
    for (const auto& normal : kFaceNormals) {
      const double face_x = landmark.x_in +
          normal[0] * localization::kAiGoalFaceOffsetIn;
      const double face_y = landmark.y_in +
          normal[1] * localization::kAiGoalFaceOffsetIn;
      const double camera_x = face_x - vision.horizontal_range_in *
          std::cos(global_bearing_rad);
      const double camera_y = face_y - vision.horizontal_range_in *
          std::sin(global_bearing_rad);
      // Only the outward side of a Goal face can be observed.
      if ((camera_x - face_x) * normal[0] +
              (camera_y - face_y) * normal[1] <= 0.0) {
        continue;
      }
      ++candidate_count;
      const double gps_delta = gps_valid
          ? std::hypot(camera_x - gps.x_in, camera_y - gps.y_in) : NAN;
      const double fused_delta = fused_valid
          ? std::hypot(camera_x - fused.x_in, camera_y - fused.y_in) : NAN;
      const double score = (gps_valid ? gps_delta : 0.0) +
                           (fused_valid ? fused_delta : 0.0);
      if (score < best_score) {
        best_score = score;
        best_x = camera_x;
        best_y = camera_y;
        best_gps_delta = gps_delta;
        best_fused_delta = fused_delta;
        found_candidate = true;
      }
    }
  }
  const double gps_fused_delta = gps_valid && fused_valid
      ? std::hypot(gps.x_in - fused.x_in, gps.y_in - fused.y_in) : NAN;
  const int reference_count = static_cast<int>(gps_valid) +
                              static_cast<int>(fused_valid);
  const double mean_delta = reference_count > 0
      ? ((gps_valid ? best_gps_delta : 0.0) +
         (fused_valid ? best_fused_delta : 0.0)) / reference_count
      : NAN;
  // Diagnostic display scale: 100% at zero disagreement and 0% at 12 inches.
  const double similarity_pct = found_candidate && std::isfinite(mean_delta)
      ? std::clamp(100.0 * (1.0 - mean_delta / 12.0), 0.0, 100.0)
      : 0.0;
  std::printf(
      "AI_COMPARE t=%lu stationary_ms=%lu port=%u mount=%s tag=%d "
      "range=%.2f bearing=%.2f candidates=%d vision_xy=%.2f,%.2f "
      "gps_xy=%.2f,%.2f gps_rms=%.2f vision_gps_delta=%.2f "
      "fused_xy=%.2f,%.2f vision_fused_delta=%.2f gps_fused_delta=%.2f "
      "similarity_pct=%.1f correction_applied=0 valid=%d\n",
      static_cast<unsigned long>(pros::millis()),
      static_cast<unsigned long>(stationary_ms),
      static_cast<unsigned>(vision.port), vision.port == 8 ? "right" : "front",
      vision.tag_id, vision.horizontal_range_in, vision.bearing_right_deg,
      candidate_count, best_x, best_y, gps.x_in, gps.y_in, gps_error_in,
      best_gps_delta, fused.x_in, fused.y_in, best_fused_delta,
      gps_fused_delta, similarity_pct, static_cast<int>(found_candidate));
  std::fflush(stdout);
  const int line = vision.port == 8 ? 5 : 4;
  pros::lcd::print(line, "AI P%u G%.1f F%.1f %3.0f%%",
                   static_cast<unsigned>(vision.port), best_gps_delta,
                   best_fused_delta, similarity_pct);
}

void dual_ai_stationary_comparison_loop() {
  constexpr std::array<std::uint8_t, 2> kPorts{6, 8};
  for (const auto port : kPorts) configure_dual_ai_diagnostic_port(port);
  pros::delay(750);
  std::uint32_t stationary_since_ms = 0;
  while (true) {
    const bool autonomous_active = opcontrol_auton_running ||
        (pros::competition::is_autonomous() &&
         !pros::competition::is_disabled());
    if (!autonomous_active || !dual_ai_robot_stationary()) {
      stationary_since_ms = 0;
      pros::delay(50);
      continue;
    }
    const std::uint32_t now = pros::millis();
    if (stationary_since_ms == 0) stationary_since_ms = now;
    const std::uint32_t stationary_ms = now - stationary_since_ms;
    if (stationary_ms < 300) {
      pros::delay(50);
      continue;
    }
    for (const auto port : kPorts) {
      emit_dual_ai_pose_comparison(read_dual_ai_diagnostic(port),
                                   stationary_ms);
    }
    pros::delay(100);
  }
}

void arm_calibration_logger_loop() {
  std::uint32_t last_screen_ms = 0;
  while (true) {
    claw_arm.move(0);
    const double angle_deg =
        static_cast<double>(claw_arm_rotation.get_angle()) / 100.0;
    std::printf(
        "ARM_CAL t=%lu port=5 installed=%d angle=%.2f position=%ld "
        "motor_command_mv=%ld\n",
        static_cast<unsigned long>(pros::millis()),
        static_cast<int>(claw_arm_rotation.is_installed()),
        angle_deg,
        static_cast<long>(claw_arm_rotation.get_position()),
        static_cast<long>(claw_arm.get_voltage()));
    std::fflush(stdout);
    if (pros::millis() - last_screen_ms >= 250) {
      pros::screen::set_eraser(pros::Color::black);
      pros::screen::erase_rect(0, 0, 479, 239);
      pros::screen::set_pen(pros::Color::white);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 2, "ARM CAL P5");
      pros::screen::set_pen(pros::Color::green);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "%.2f deg",
                          angle_deg);
      pros::screen::set_pen(pros::Color::white);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8, "P4 MOTOR ZERO");
      last_screen_ms = pros::millis();
    }
    pros::delay(50);
  }
}

void run_dual_ai_vision_stationary_test() {
  constexpr std::array<std::uint8_t, 2> kPorts{6, 8};
  constexpr std::uint32_t kDurationMs = 12000;
  constexpr std::uint32_t kPeriodMs = 100;
  constexpr double kPi = 3.14159265358979323846;

  stop_drive_velocity();
  std::printf(
      "DUAL_AI event=start duration_ms=%lu p6_mount=forward "
      "p8_mount=robot_right expected_goal=24,-48\n",
      static_cast<unsigned long>(kDurationMs));
  for (const std::uint8_t port : kPorts) {
    const int type = static_cast<int>(pros::c::get_plugged_type(port));
    errno = 0;
    int reset = 0;
    int family = 0;
    int disable = 0;
    int enable = 0;
    int enabled = 0;
    if (type == static_cast<int>(pros::c::E_DEVICE_AIVISION)) {
      reset = pros::c::aivision_reset(port);
      family = pros::c::aivision_set_tag_family_override(
          port, pros::TAG_CIRCLE_21H7);
      disable = pros::c::aivision_disable_detection_types(
          port, pros::E_AIVISION_MODE_COLORS |
                    pros::E_AIVISION_MODE_OBJECTS |
                    pros::E_AIVISION_MODE_COLOR_MERGE);
      enable = pros::c::aivision_enable_detection_types(
          port, pros::E_AIVISION_MODE_TAGS);
      enabled = pros::c::aivision_get_enabled_detection_types(port);
    }
    std::printf(
        "DUAL_AI event=config port=%u type=%d reset=%d family=%d "
        "disable=%d enable=%d enabled=%d errno=%d\n",
        static_cast<unsigned>(port), type, reset, family, disable, enable,
        enabled, errno);
  }
  std::fflush(stdout);
  pros::delay(750);

  const std::uint32_t started_ms = pros::millis();
  while (pros::millis() - started_ms < kDurationMs) {
    const std::uint32_t elapsed_ms = pros::millis() - started_ms;
    const auto gps_position = gps_7.get_position();
    const double gps_heading_cw = gps_7.get_heading();
    const double gps_error_in = gps_7.get_error() * 39.37007874015748;
    const auto gps_project = localization::vex_gps_to_project_robot_pose(
        gps_position.x, gps_position.y, gps_heading_cw);
    std::printf(
        "DUAL_GPS t=%lu native=%.5f,%.5f heading_cw=%.2f error=%.2f "
        "project_robot=%.2f,%.2f,%.2f installed=%d\n",
        static_cast<unsigned long>(elapsed_ms), gps_position.x,
        gps_position.y, gps_heading_cw, gps_error_in, gps_project.x_in,
        gps_project.y_in, gps_project.heading_deg,
        static_cast<int>(gps_7.is_installed()));
    for (const std::uint8_t port : kPorts) {
      errno = 0;
      const int count = pros::c::aivision_get_object_count(port);
      bool found = false;
      pros::aivision_object_s_t best{};
      double best_area = -1.0;
      if (count >= 0 && count <= 24) {
        for (int index = 0; index < count; ++index) {
          const auto object = pros::c::aivision_get_object(
              port, static_cast<std::uint32_t>(index));
          if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
          const auto& tag = object.object.tag;
          const double twice_area =
              static_cast<double>(tag.x0) * tag.y1 -
              static_cast<double>(tag.y0) * tag.x1 +
              static_cast<double>(tag.x1) * tag.y2 -
              static_cast<double>(tag.y1) * tag.x2 +
              static_cast<double>(tag.x2) * tag.y3 -
              static_cast<double>(tag.y2) * tag.x3 +
              static_cast<double>(tag.x3) * tag.y0 -
              static_cast<double>(tag.y3) * tag.x0;
          const double area = std::fabs(twice_area) * 0.5;
          if (area > best_area) {
            best = object;
            best_area = area;
            found = true;
          }
        }
      }
      if (!found) {
        std::printf(
            "DUAL_AI event=sample t=%lu port=%u count=%d tag=-1 "
            "valid=0 errno=%d\n",
            static_cast<unsigned long>(elapsed_ms),
            static_cast<unsigned>(port), count, errno);
        continue;
      }

      const auto& tag = best.object.tag;
      const auto edge = [](int x0, int y0, int x1, int y1) {
        return std::hypot(static_cast<double>(x1 - x0),
                          static_cast<double>(y1 - y0));
      };
      const double e0 = edge(tag.x0, tag.y0, tag.x1, tag.y1);
      const double e1 = edge(tag.x1, tag.y1, tag.x2, tag.y2);
      const double e2 = edge(tag.x2, tag.y2, tag.x3, tag.y3);
      const double e3 = edge(tag.x3, tag.y3, tag.x0, tag.y0);
      const double horizontal_edge = 0.5 * (e0 + e2);
      const double vertical_edge = 0.5 * (e1 + e3);
      const double center_x =
          0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
      const double center_y =
          0.25 * (tag.y0 + tag.y1 + tag.y2 + tag.y3);
      double depth = 0.0;
      if (horizontal_edge > 0.0 && vertical_edge > 0.0) {
        depth = 0.5 *
                (localization::kAiFocalLengthXPx *
                     localization::kAiTagDetectedSizeIn / horizontal_edge +
                 localization::kAiFocalLengthYPx *
                     localization::kAiTagDetectedSizeIn / vertical_edge);
      }
      const double right = depth *
                           (center_x - localization::kAiImageWidthPx * 0.5) /
                           localization::kAiFocalLengthXPx;
      const double up = -depth *
                        (center_y - localization::kAiImageHeightPx * 0.5) /
                        localization::kAiFocalLengthYPx;
      const double horizontal_range = std::hypot(depth, right);
      const double bearing = std::atan2(right, depth) * 180.0 / kPi;
      const double elevation =
          std::atan2(up, horizontal_range) * 180.0 / kPi;
      std::printf(
          "DUAL_AI event=sample t=%lu port=%u count=%d tag=%d "
          "corners=%d,%d,%d,%d,%d,%d,%d,%d center=%.1f,%.1f area=%.1f "
          "edge_h=%.2f edge_v=%.2f depth=%.2f right=%.2f up=%.2f "
          "horizontal=%.2f bearing=%.2f elevation=%.2f valid=1\n",
          static_cast<unsigned long>(elapsed_ms),
          static_cast<unsigned>(port), count, best.id, tag.x0, tag.y0,
          tag.x1, tag.y1, tag.x2, tag.y2, tag.x3, tag.y3, center_x,
          center_y, best_area, horizontal_edge, vertical_edge, depth, right,
          up, horizontal_range, bearing, elevation);
    }
    std::fflush(stdout);
    pros::delay(kPeriodMs);
  }
  stop_drive_velocity();
  std::printf("DUAL_AI event=done elapsed_ms=%lu\n",
              static_cast<unsigned long>(pros::millis() - started_ms));
  std::fflush(stdout);
}
}  // namespace

void initialize() {
  // Emit progress before each device call. A Smart Port driver must never be
  // able to make a black Brain screen look like a dead user program.
  std::printf("BOOT_STAGE t=%lu stage=entry\n",
              static_cast<unsigned long>(pros::millis()));
  print_smart_port_inventory("boot");

  // Full-size 11 W port-4 arm motor. Port 5 supplies the absolute position
  // feedback used by the normal/right position holds in autonomous/opcontrol.
  claw_arm.set_current_limit(2500);
  claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  claw_arm.move(0);
  // Port D is wired active-low. Every program start must command high so the
  // robot remains inside its pre-match size limit. Only an autonomous launch
  // may command low/extended; that state is then left latched through driver.
  clamp_piston.set_value(true);
  clamp_output_high.store(true, std::memory_order_release);
  std::printf("TOGGLE_STATE phase=initialize output=1 physical=retracted\n");

  std::printf("BOOT_STAGE t=%lu stage=lcd_begin\n",
              static_cast<unsigned long>(pros::millis()));
  std::fflush(stdout);
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Rear Distance P1");
  pros::lcd::set_text(1, "GPS P7 / IMU P14");
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
  // Independent diagnostics owner for P6/P8. It remains idle unless an
  // autonomous routine is active and the robot has been stationary for 300ms.
  // Results are display/log only; AI pose correction is disabled in config.
  pros::Task::create(dual_ai_stationary_comparison_loop,
                     "dual AI compare");
  // Repeat after slower sensor initialization so a terminal attached just
  // after upload still receives the live wiring inventory.
  print_smart_port_inventory("post_init");
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

  // Calibration must remain observable while the Brain is disabled and no
  // controller is paired; competition callbacks are not entered in that state.
  if constexpr (RUN_BOOT_DIAGNOSTIC_NO_ACTUATION) {
    pros::Task::create(arm_calibration_logger_loop, "arm calibration logger");
    return;
  }

  // One task owns both L2 selection and L2+B launch. Keeping both sides of the
  // chord in one state machine prevents a delayed L2 release from changing the
  // selected route during or immediately after autonomous.
  pros::Task::create([] {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool launch_armed = true;
    bool last_l2 = false;
    bool last_b = false;
    bool l2_press_had_b = false;
    int l2_tap_count = 0;
    std::uint32_t l2_sequence_started_ms = 0;
    constexpr std::uint32_t kSelectionTapWindowMs = 2000;
    std::uint32_t chord_started_ms = 0;
    std::uint32_t last_render_ms = 0;
    while (true) {
      const bool selectable =
          !auton_selection_locked.load(std::memory_order_acquire) &&
          !opcontrol_auton_running;
      const std::uint32_t input_now_ms = pros::millis();
      const bool b_held = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_B);
      const bool l2 = controller.get_digital(
          pros::E_CONTROLLER_DIGITAL_L2);
      if (l2 != last_l2 || b_held != last_b) {
        std::printf("HOTKEY_INPUT l2=%d b=%d connected=%d\n",
                    static_cast<int>(l2), static_cast<int>(b_held),
                    static_cast<int>(controller.is_connected()));
        std::fflush(stdout);
      }
      if (l2_tap_count > 0 &&
          input_now_ms - l2_sequence_started_ms > kSelectionTapWindowMs) {
        l2_tap_count = 0;
        l2_sequence_started_ms = 0;
        controller.print(0, 0, "AUTON SELECT RESET ");
      }
      if (b_held && !last_b) {
        // B belongs to the launch chord. Even if L2 was pressed slightly
        // earlier, B must cancel every partially entered selection gesture.
        l2_tap_count = 0;
        l2_sequence_started_ms = 0;
      }
      if (l2 && !last_l2) {
        l2_press_had_b = b_held;
      }
      if (l2 && b_held) {
        l2_press_had_b = true;
        // A launch chord invalidates the entire selection-tap sequence.
        l2_tap_count = 0;
        l2_sequence_started_ms = 0;
      }
      if (l2 && b_held) {
        if (chord_started_ms == 0) chord_started_ms = pros::millis();
        if (launch_armed && selectable &&
            pros::millis() - chord_started_ms >= 100) {
          launch_armed = false;
          l2_press_had_b = true;
          std::printf("AUTON_HOTKEY event=launch_l2_b selected=%d name=%s\n",
                      selected_auton.load(std::memory_order_acquire),
                      selected_auton_name());
          std::fflush(stdout);
          controller.rumble(".-");
          start_toggle_far_goal_auton();
        }
      } else {
        chord_started_ms = 0;
        if (!l2 && !b_held) launch_armed = true;
      }
      if (!l2 && last_l2) {
        // Three ordinary completed taps inside one two-second window advance
        // exactly one entry. There are no hold-time or rearm-delay gates.
        if (!l2_press_had_b && selectable) {
          const std::uint32_t now = pros::millis();
          if (l2_tap_count == 0 ||
              now - l2_sequence_started_ms > kSelectionTapWindowMs) {
            l2_sequence_started_ms = now;
            l2_tap_count = 1;
          } else {
            ++l2_tap_count;
          }
          std::printf("AUTON_SELECTOR l2_tap=%d/3\n", l2_tap_count);
          std::fflush(stdout);
          controller.print(0, 0, "AUTON SELECT %d/3   ", l2_tap_count);
          if (l2_tap_count >= 3) {
            advance_auton();
            l2_tap_count = 0;
            l2_sequence_started_ms = 0;
            controller.rumble(".-");
          } else {
            controller.rumble(".");
          }
        }
        l2_press_had_b = false;
      }
      last_l2 = l2;
      last_b = b_held;
      if (pros::millis() - last_render_ms >= 250) {
        render_auton_selection();
        last_render_ms = pros::millis();
      }
      if (selectable) {
        controller.print(1, 0, "< %-15s >", selected_auton_name());
      }
      pros::delay(20);
    }
  }, "auton select and launch");
}

void disabled() {
  navigation::stop();
  // autonomous() locks the route selector to prevent changes while moving.
  // Field disable must release that lock so L2 x3 works before the next run.
  auton_selection_locked.store(false, std::memory_order_release);
  render_auton_selection();
}

void competition_initialize() {
  auton_selection_locked.store(false, std::memory_order_release);
  render_auton_selection();
}

void autonomous() {
  // The Toggle mechanism must remain deployed for the complete game once
  // autonomous begins. ADI-D is active-low, so false is physically extended.
  clamp_piston.set_value(false);
  clamp_output_high.store(false, std::memory_order_release);
  std::printf("TOGGLE_STATE phase=competition_auton output=0 physical=extended\n");
  std::fflush(stdout);
  auton_selection_locked.store(true, std::memory_order_release);
  if constexpr (RUN_COMPETITION_DIAGNOSTIC_ROUTE) {
    fusion_test_auton();
  } else {
    run_selected_auton();
  }
}

void opcontrol() {
  if constexpr (RUN_BOOT_DIAGNOSTIC_NO_ACTUATION) {
    while (true) {
      claw_arm.move(0);
      pros::delay(50);
    }
  }
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  // Do not deploy here: entering driver control before autonomous must remain
  // competition-size legal. If autonomous already ran, its low output remains
  // physically latched extended through this callback.
  // Competition autonomous locks selection only for its own run. Re-enable
  // selection on entry to driver control so repeated field/testing runs can
  // choose another route without restarting the program.
  auton_selection_locked.store(false, std::memory_order_release);
  master.print(0, 0, "L2x3=SEL L2+B=GO ");
  pros::lcd::set_text(6, "L2 x3 select / L2+B run");
  if (drive_positions_are_zeroed()) {
    localization_telemetry_reset();
  }
  if (RUN_STARTUP_DUAL_AI_VISION_TEST) {
    pros::lcd::set_text(6, "Dual AI stationary test");
    run_dual_ai_vision_stationary_test();
    pros::lcd::set_text(6, "Dual AI test complete");
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
    constexpr double kDipDeg = 100.0;
    constexpr double kToleranceDeg = 16.0;
    constexpr std::uint32_t kSettleMs = 250;
    constexpr std::uint32_t kLegTimeoutMs = 10000;
    constexpr std::int32_t kCurrentAbortMa = 2450;
    constexpr std::uint32_t kCurrentAbortConfirmMs = 200;

    pros::lcd::set_text(6, "Lift sequence in 2 sec");
    pros::delay(2000);

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
      pros::delay(250);
      sequence_ok = run_target(std::max(0.0, stage_deg - kDipDeg), dip_label) &&
                    sequence_ok;
      if (!sequence_ok) break;
      pros::delay(100);
      sequence_ok = run_target(0.0, "zero_between_stages") && sequence_ok;
      if (!sequence_ok) break;
      pros::delay(250);
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
  if (RUN_STARTUP_P15_FORWARD_12_TEST) {
    pros::lcd::set_text(6, "P15 6in in 3s");
    pros::delay(3000);
    run_p15_forward_12_test();
    pros::lcd::set_text(6, "P15 6in complete");
  }
  if (RUN_STARTUP_TOGGLE_RETREAT_TEST) {
    pros::lcd::set_text(6, "24in retreat in 3s");
    pros::delay(3000);
    run_toggle_retreat_test();
    pros::lcd::set_text(6, "Retreat complete");
  }
  if (RUN_STARTUP_TOGGLE_STRIKE_TEST) {
    pros::lcd::set_text(6, "Toggle strike in 3s");
    pros::delay(3000);
    run_toggle_strike_test();
    pros::lcd::set_text(6, "Toggle strike complete");
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
    pros::lcd::set_text(6, "P6 sweep in 5 sec");
    pros::delay(5000);
    const bool p8_sweep_ok = ai_vision_heading_characterization();
    pros::lcd::set_text(6, p8_sweep_ok ? "P6 heading sweep OK"
                                      : "P6 heading sweep FAIL");
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
  bool toggle_piston_was_pressed = false;
  bool arm_position_hold_active = false;
  double arm_position_target_deg = kArmNormalTargetDeg;
  ArmPositionController arm_position_controller;

  while (true) {
    const bool pose_editor_active = update_runtime_pose_editor(master);
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
      const bool toggle_piston_pressed =
          !pose_editor_active &&
          master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
      if (toggle_piston_pressed && !toggle_piston_was_pressed) {
        const bool next_output_high = !clamp_output_high.load(
            std::memory_order_acquire);
        clamp_piston.set_value(next_output_high);
        clamp_output_high.store(next_output_high, std::memory_order_release);
        std::printf("TOGGLE_STATE phase=opcontrol_a output=%d physical=%s\n",
                    static_cast<int>(next_output_high),
                    next_output_high ? "retracted" : "extended");
        std::fflush(stdout);
      }
      toggle_piston_was_pressed = toggle_piston_pressed;
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
      const bool wrist_right_position_pressed =
          !pose_editor_active &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
          master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
      const bool wrist_normal_position_pressed =
          !pose_editor_active &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
          !master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
          master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
      if (wrist_right_position_pressed) {
        arm_position_hold_active = true;
        arm_position_target_deg = kArmRightTargetDeg;
        arm_position_controller.reset();
        std::printf("ARM_PID event=start target=%.2f\n",
                    kArmRightTargetDeg);
        std::fflush(stdout);
      }
      if (wrist_normal_position_pressed) {
        arm_position_hold_active = true;
        arm_position_target_deg = kArmNormalTargetDeg;
        arm_position_controller.reset();
        std::printf("ARM_PID event=normal target=%.2f tolerance=%.1f\n",
                    kArmNormalTargetDeg, kArmNormalToleranceDeg);
        std::fflush(stdout);
      }
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
        arm_position_hold_active = false;
        arm_position_controller.reset();
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        claw_arm.move(-kMechanismPower);
      } else if (wrist_down_manual_positive) {
        arm_position_hold_active = false;
        arm_position_controller.reset();
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        claw_arm.move(kMechanismPower);
      } else if (pose_editor_active) {
        arm_position_hold_active = false;
        arm_position_controller.reset();
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        claw_arm.move(0);
      } else if (arm_position_hold_active) {
        // Keep correcting after arrival so the mechanism cannot wobble or sag.
        // Normal mode's requested acceptance is +/-5 degrees; the shared
        // controller's tighter brake band comfortably satisfies that bound.
        arm_position_controller.update(arm_position_target_deg);
      } else {
        // A manual Up/Down move remains at its released location until Left or
        // Right explicitly selects a held position again.
        claw_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        claw_arm.move(0);
      }
    } else {
      // Do not turn an A press held through autonomous completion into a
      // surprise piston edge when driver control ownership returns.
      toggle_piston_was_pressed = master.get_digital(
          pros::E_CONTROLLER_DIGITAL_A);
    }

    print_distance_frame();
    const std::uint32_t drive_health_now_ms = pros::millis();
    if (drive_health_now_ms - last_drive_health_ms >= 5000) {
      print_drive_motor_health("periodic_stationary_or_opcontrol");
      std::printf("CLAW_ROTATION port=5 absolute_deg=%.2f\n",
                  static_cast<double>(claw_arm_rotation.get_angle()) / 100.0);
      std::fflush(stdout);
      last_drive_health_ms = drive_health_now_ms;
    }
    // During a hotkey autonomous, the stationary diagnostic task owns P6/P8
    // reads. Avoid concurrent Smart Port calls from the driver-control loop.
    if (!opcontrol_auton_running) ai_vision_shadow_update();
    if (!opcontrol_auton_running) {
      localization_telemetry_update();
    }
    pros::delay(SAMPLE_PERIOD_MS);
  }
}
