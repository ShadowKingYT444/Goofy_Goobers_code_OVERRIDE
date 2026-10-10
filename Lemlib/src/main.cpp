#include "main.h"
#include "autons.hpp"
#include "gps_reset/gps_reset.hpp"
#include "field_map.hpp"
#include <cstdint>
#include <cstdio>
namespace {
// Competition control deletes/reuses the opcontrol stack on mode changes.
// Background tasks must only refer to state that outlives that stack.
std::atomic_bool arm_auto_moving{false};
std::atomic_bool arm_auto_cancelled{false};

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

pros::Motor slider_left(-9, pros::MotorGears::green);
pros::Motor slider_right(2, pros::MotorGears::green);
pros::Rotation lift_sensor(16);
pros::Rotation claw_sensor(12);
pros::Motor claw_arm(20, pros::MotorGears::green);
pros::adi::DigitalOut clamp_piston('A');
// Prints the arm rotation sensor (claw_sensor, port 7) degrees to the terminal
// so default/max/1-pin arm heights can be read off and set for auton.
void print_arm_degrees() {
    printf("Arm angle: %.2f deg | position: %.2f deg\n",
           claw_sensor.get_angle() / 100.0,
           claw_sensor.get_position() / 100.0);
}
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

void disabled() { arm_auto_cancelled.store(true); }
void competition_initialize() { arm_auto_cancelled.store(true); }

// Routine run by autonomous() and the LEFT hotkey. Build another one into a
// slot without editing this file:
//   pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=pid_autotune_auton
#ifndef AUTON_ROUTINE
#define AUTON_ROUTINE three_pin_auton
#endif
#define AUTON_NAME_(name) #name
#define AUTON_NAME(name) AUTON_NAME_(name)

void autonomous() {
    arm_auto_cancelled.store(true);
    const std::uint32_t startedAt = pros::millis();
    AUTON_ROUTINE();
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
    bool claw_pressed = false;
    uint32_t next_lift_report = 0;
    arm_auto_cancelled.store(true);

    while (true) {
        // Preserve the old single-stick arcade behavior and turn direction.
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        const int lift = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
                             ? 127
                             : (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
                                    ? -127
                                    : 0);
    
        
        slider_left.move(-lift);
        slider_right.move(-lift);
        const int spin = (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)
                             ? 100
                
                                    : 0);
        side_toggle.move(spin);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            clamp_pressed = !clamp_pressed;
            clamp_piston.set_value(clamp_pressed);
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) &&
            !arm_auto_moving.exchange(true)) {
            arm_auto_cancelled.store(false);
            pros::Task([] {
                moveArm(530, &arm_auto_cancelled);
                arm_auto_moving.store(false);
            });
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            slider_left.move(-127);
            slider_right.move(-127);
            pros::delay(150);
        }
        // LEFT is otherwise unbound. Runs the selected autonomous from driver
        // control, so routines longer than the 15 s autonomous period finish.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonomous();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            claw_pressed = !claw_pressed;
            claw_piston.set_value(claw_pressed);
        }
        // B prints a snapshot of the arm degrees on demand.
        
        constexpr double ARM_MAX_DEG = 550.2;
        constexpr double ARM_MIN_DEG = 0.0;
        const bool manual_arm_input =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ||
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
                    (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
                    arm_deg < ARM_MAX_DEG)
                        ? -127
                        : ((master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
                            arm_deg > ARM_MIN_DEG)
                            ? 127
                            : 0);

                claw_arm.move(arm);
            }
        pros::delay(20);
    }
}
