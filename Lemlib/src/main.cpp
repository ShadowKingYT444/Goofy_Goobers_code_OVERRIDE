#include "main.h"
#include "autons.hpp"
#include "gps_reset/gps_reset.hpp"
#include "field_map.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
namespace {
// Competition control deletes/reuses the opcontrol stack on mode changes.
// Background tasks must only refer to state that outlives that stack.
std::atomic_bool arm_auto_moving{false};
std::atomic_bool arm_auto_cancelled{false};
std::atomic_bool lift_auto_cancelled{false};
std::atomic_bool claw_pressed{false};

// RIGHT macro: the arm comes down to arm_position::release_deg, the claw
// piston lets go of the element halfway there, then the lift lowers to rest.
// The skills route repeats this sequence (put_on in src/autons.cpp).
constexpr double kReleaseArmDeg = arm_position::release_deg;
constexpr std::uint32_t kReleaseSettleMs = arm_position::release_settle_ms;

void cancel_driver_macros() {
    arm_auto_cancelled.store(true);
    lift_auto_cancelled.store(true);
}

void release_and_lower() {
    const double start_deg = claw_sensor.get_position() / 100.0;
    const double release_within_deg =
        std::abs(start_deg - kReleaseArmDeg) / 2.0;
    pros::Task([] {
        moveArm(kReleaseArmDeg, &arm_auto_cancelled);
        arm_auto_moving.store(false);
    });
    while (std::abs(claw_sensor.get_position() / 100.0 - kReleaseArmDeg) >
           release_within_deg) {
        // The arm stopped short of halfway: keep hold of the element.
        if (arm_auto_cancelled.load() || !arm_auto_moving.load()) return;
        pros::delay(10);
    }
    // Open, whatever L1 last did: autonomous moves this piston too, so the
    // L1 toggle's idea of its state cannot be trusted here.
    claw_pressed.store(false);
    claw_piston.set_value(false);
    pros::delay(kReleaseSettleMs);
    lowerLiftToRest(&lift_auto_cancelled);
}

// Ends any automatic arm or lift move still running, so a new one can start.
// moveArm() can sit just short of its target indefinitely, which used to
// leave Y and RIGHT dead until DOWN was pressed.
void take_over_arm() {
    cancel_driver_macros();
    for (int i = 0; i < 10 && arm_auto_moving.load(); ++i) pros::delay(10);
    pros::delay(30);  // every macro loop has seen the cancel by now
    arm_auto_moving.store(true);
    arm_auto_cancelled.store(false);
    lift_auto_cancelled.store(false);
}

void boot_stage(const char* stage) {
    std::printf("BOOT_STAGE,%s\n", stage);
    std::fflush(stdout);
    fieldviz::status(stage);
}
}
// Drive wiring and direction match the old Goofy Goobers project.
pros::MotorGroup left_motors({-17, -18}, pros::MotorGears::blue);
pros::MotorGroup right_motors({11, 13}, pros::MotorGears::blue);
pros::Imu imu(14);
pros::Motor side_toggle(4);
pros::adi::DigitalOut claw_piston('B');
lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              12, lemlib::Omniwheel::NEW_275, 450, 2);

// No external tracking wheels are installed. LemLib creates its vertical
// odometry inputs from the drivetrain motor encoders during calibrate().
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

// Hand-set values. The first autotuner's result (lateral 42/0/93, angular
// 8.4/0.67/93) was far too aggressive on the robot and was reverted.
lemlib::ControllerSettings lateral_controller(6.0, 0, 3, 0, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::ExpoDriveCurve throttle_curve(5, 0, 1.0);
lemlib::ExpoDriveCurve steer_curve(5, 0, 1.0);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

pros::Motor slider_left(-19, pros::MotorGears::green);
pros::Motor slider_right(2, pros::MotorGears::green);
pros::Rotation lift_sensor(16);
pros::Rotation claw_sensor(12);
pros::Motor claw_arm(20, pros::MotorGears::green);
pros::adi::DigitalOut clamp_piston('A');
void initialize() {
    fieldviz::start();
    boot_stage("initialize_begin");
    boot_stage("field_map_begin");
    boot_stage("aux_reset_begin");
    claw_arm.tare_position();
    claw_sensor.reset_position();
    lift_sensor.reset_position();
    // HOLD is used by moveLift() after it reaches a target; check motor heat
    // and paired-motor behavior with the loaded mechanism during validation.
    slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    boot_stage("aux_reset_done");
    boot_stage("chassis_calibrate_begin");
    chassis.calibrate();
    boot_stage("chassis_calibrate_done");
    claw_piston.set_value(true);
    clamp_piston.set_value(false);
    boot_stage("gps_init_begin");
    // The camera looks out of the robot's right side. If GPS X/Y come out
    // mirrored on both axes relative to the motor pose, use 270 instead.
    gpsreset::init(chassis, 10, /*forward_in=*/3.0, /*right_in=*/3.7,
                   /*facing_deg=*/90.0);
    gpsreset::set_anchor_observer(fieldviz::set_anchor);
    boot_stage("gps_init_done");

    chassis.setPose(0, 0, 180);  // Same starting pose as the 3-pin auton.
    boot_stage("gps_anchor_begin");
    const bool gps_anchor_ok = gpsreset::capture_start_as(0, 0, 180);
    boot_stage(gps_anchor_ok ? "gps_anchor_ok" : "gps_anchor_no_fix");

    boot_stage("screen_task_begin");
    static pros::Task screen_task([]() {
        // Read only the sensor here; anchor metadata arrives via the observer.
        while (true) {
            const auto pose = chassis.getPose();
            const auto gps_pose = gpsreset::live_pose();
            fieldviz::publish({
                {pose.x, pose.y, pose.theta},
                {gps_pose.x_in, gps_pose.y_in, gps_pose.theta_deg},
                gps_pose.ok,
                gps_pose.jump,
                imu.get_rotation(),
                lift_sensor.get_position() / 100.0,
                claw_sensor.get_position() / 100.0
            });
            pros::delay(50);  // also the rate GPS route checkpoints are served
        }
    });
    boot_stage("initialize_done");
}

void disabled() {
    cancel_driver_macros();
    setLiftLock(false);
}
void competition_initialize() { cancel_driver_macros(); }

// Routine run by autonomous() and the LEFT hotkey. Build another one into a
// slot without editing this file:
//   pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=two_pin_far_auton
#ifndef AUTON_ROUTINE
#define AUTON_ROUTINE three_pin_auton
#endif
#define AUTON_NAME_(name) #name
#define AUTON_NAME(name) AUTON_NAME_(name)

void autonomous() {
    cancel_driver_macros();
    const std::uint32_t startedAt = pros::millis();
    setLiftLock(true);
    AUTON_ROUTINE();
    setLiftLock(false);
    const std::uint32_t elapsedMs = pros::millis() - startedAt;
    // Shown on the Brain so a run can be checked against the 15 s period.
    fieldviz::print(7, "Auto took %.1f s", elapsedMs / 1000.0);
    const lemlib::Pose pose = chassis.getPose();
    const std::int32_t liftPosition = lift_sensor.get_position();
    const std::int32_t liftVelocity = lift_sensor.get_velocity();
    // CSV-style record for repeated field trials. Pose is odometry output;
    // compare endpoint error against an independent field measurement.
    std::printf(
        "AUTON_RESULT," AUTON_NAME(AUTON_ROUTINE) ",%lu,%.2f,%.2f,%.1f,%d,%d\n",
        static_cast<unsigned long>(elapsedMs),
        static_cast<double>(pose.x),
        static_cast<double>(pose.y),
        static_cast<double>(pose.theta),
        static_cast<int>(liftPosition),
        static_cast<int>(liftVelocity)
    );
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    bool clamp_pressed = false;
    uint32_t next_lift_report = 0;
    cancel_driver_macros();
    // Driver control runs the lift lock itself, below.
    setLiftLock(false);

    while (true) {
        // Preserve the old single-stick arcade behavior and turn direction.
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        const int lift = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
                             ? 127
                             : (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
                                    ? -127
                                    : 0);
        // R1/R2 take the lift back from the RIGHT macro. With neither held,
        // the lift is locked at max height once it is 95% of the way up.
        if (lift != 0) lift_auto_cancelled.store(true);
        if (lift != 0 || (!liftMoveActive() && !holdLiftAtMax())) {
            slider_left.move(-lift);
            slider_right.move(-lift);
        }
        const int spin = (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)
                             ? 100
                
                                    : 0);
        side_toggle.move(spin);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            clamp_pressed = !clamp_pressed;
            clamp_piston.set_value(clamp_pressed);
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            take_over_arm();
            pros::Task([] {
                moveArm(arm_position::max_deg, &arm_auto_cancelled);
                arm_auto_moving.store(false);
            });
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            take_over_arm();
            pros::Task release_task(release_and_lower);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            lift_auto_cancelled.store(true);
            slider_left.move(-127);
            slider_right.move(-127);
            pros::delay(150);
        }
        // LEFT is otherwise unbound. Runs the selected autonomous from driver
        // control, so routines longer than the 15 s autonomous period finish.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonomous();
        }
        // UP records a hand-driven curve for the two-pin far route.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
            master.rumble(curveCaptureToggle() ? "." : "-");
        curveCaptureSample();
        // X notes the lift and arm heights on the Brain, for the skills
        // route's stacking levels.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            master.print(0, 0, "Height %d noted  ", recordHeights());
            pros::delay(60);  // the controller drops commands sent closer
            master.rumble(".");
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            claw_pressed.store(!claw_pressed.load());
            claw_piston.set_value(claw_pressed.load());
        }
        // B prints a snapshot of the arm degrees on demand.
        
        constexpr double ARM_MIN_DEG = 0.0;
        const bool manual_arm_input =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        if (manual_arm_input && arm_auto_moving) {
            // Manual input has priority over the automatic Y move.
            arm_auto_cancelled = true;
            pros::delay(20);
        }
        double arm_deg = claw_sensor.get_position()/100.0;
        if (!arm_auto_moving) {
            double arm_deg = claw_sensor.get_position() / 100.0;

                int arm =
                    (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
                     arm_deg > ARM_MIN_DEG)
                        ? 127
                        : 0;

                claw_arm.move(arm);
            }
        pros::delay(20);
    }
}
