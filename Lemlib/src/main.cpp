#include "main.h"
#include "autons.hpp"
#include "gps_reset/gps_reset.hpp"
#include <cstdint>
#include <cstdio>
namespace {
// Competition control deletes/reuses the opcontrol stack on mode changes.
// Background tasks must only refer to state that outlives that stack.
std::atomic_bool arm_auto_moving{false};
std::atomic_bool arm_auto_cancelled{false};

// Image: footprint spans 16 in inward from the right wall and 12 in above
// the centerline. Its center is (+72 - 16/2, +12/2) in field coordinates.
constexpr double kGpsDisplayStartXIn = 64.0;
constexpr double kGpsDisplayStartYIn = 6.0;

void boot_stage(const char* stage) {
    std::printf("BOOT_STAGE,%s\n", stage);
    std::fflush(stdout);
}
}
// Drive wiring and direction match the old Goofy Goobers project.
pros::MotorGroup left_motors({-17, -18}, pros::MotorGears::blue);
pros::MotorGroup right_motors({11, 13}, pros::MotorGears::blue);
pros::Imu imu(14);
// Reversed because the raw sensors decreased for front/right motion.
pros::Rotation vertical_odom(-15);
pros::Rotation horizontal_odom(-1);
pros::Motor side_toggle(4);
pros::adi::DigitalOut claw_piston('B');
lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              12, lemlib::Omniwheel::NEW_275, 450, 2);

// Temporary diagnostic geometry. The offsets are measured from the LemLib
// tracking center: vertical is 0.5 in left, horizontal is 5.1 in forward.
lemlib::TrackingWheel vertical_wheel(
    &vertical_odom,
    lemlib::Omniwheel::NEW_2,
    -0.5,
    1.0
);
lemlib::TrackingWheel horizontal_wheel(
    &horizontal_odom,
    lemlib::Omniwheel::NEW_2,
    5.1,
    1.0
);
lemlib::OdomSensors sensors(
    &vertical_wheel,
    nullptr,
    &horizontal_wheel,
    nullptr,
    &imu
);

// First lateral P speed experiment; verify endpoint accuracy and slip physically.
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
    boot_stage("initialize_begin");
    boot_stage("aux_reset_begin");
    claw_arm.tare_position();
    claw_sensor.reset_position();
    lift_sensor.reset_position();
    // HOLD is used by moveLift() after it reaches a target; check motor heat
    // and paired-motor behavior with the loaded mechanism during validation.
    slider_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slider_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    boot_stage("aux_reset_done");
    boot_stage("lcd_begin");
    pros::lcd::initialize();
    boot_stage("lcd_done");
    boot_stage("chassis_calibrate_begin");
    chassis.calibrate();
    boot_stage("chassis_calibrate_done");
    claw_piston.set_value(true);
    clamp_piston.set_value(false);
    boot_stage("gps_init_begin");
    gpsreset::init(chassis, 10, /*forward_in=*/7.0, /*right_in=*/4.5);
    boot_stage("gps_init_done");

    chassis.setPose(0, 0, 180);  // Same starting pose as the 3-pin auton.
    boot_stage("gps_anchor_begin");
    const bool gps_anchor_ok = gpsreset::capture_start_as(0, 0, 180);
    boot_stage(gps_anchor_ok ? "gps_anchor_ok" : "gps_anchor_no_fix");

    boot_stage("screen_task_begin");
    static pros::Task screen_task([]() {
        // Rebooting the Brain starts a fresh baseline. The delta values below
        // are therefore the signed sensor changes since initialization.
        bool have_baseline = false;
        double baseline_vertical = 0.0;
        double baseline_horizontal = 0.0;
        double baseline_rotation = 0.0;

        while (true) {
            const double lift_deg = lift_sensor.get_position()/100.0;
            const double claw_deg = claw_sensor.get_position()/100.0;
            const double vertical_distance =
                vertical_wheel.getDistanceTraveled();
            const double horizontal_distance =
                horizontal_wheel.getDistanceTraveled();
            const double imu_rotation = imu.get_rotation();

            if (!have_baseline) {
                baseline_vertical = vertical_distance;
                baseline_horizontal = horizontal_distance;
                baseline_rotation = imu_rotation;
                have_baseline = true;
            }

            auto pose = chassis.getPose();

            pros::lcd::print(0, "X %.2f Y %.2f", pose.x, pose.y);
            pros::lcd::print(1, "T %.2f", pose.theta);

            pros::lcd::print(
                2, "IMU %.2f dT %.1f",
                imu_rotation,
                imu_rotation - baseline_rotation
            );
            pros::lcd::print(
                3, "Lift %.2f deg", lift_deg
            );
            pros::lcd::print(
                4, "Claw %.2f deg", claw_deg
            );
            pros::lcd::print(
                5, "V %.2f H %.2f",
                vertical_distance,
                horizontal_distance
            );
            pros::lcd::print(
                6, "dV %.2f dH %.2f",
                vertical_distance - baseline_vertical,
                horizontal_distance - baseline_horizontal
            );
            pros::delay(100);
        }
    });
    boot_stage("initialize_done");
}

void disabled() { arm_auto_cancelled.store(true); }
void competition_initialize() { arm_auto_cancelled.store(true); }

void autonomous() {
    arm_auto_cancelled.store(true);
    const std::uint32_t startedAt = pros::millis();
    three_pin_auton();
    const std::uint32_t elapsedMs = pros::millis() - startedAt;
    const lemlib::Pose pose = chassis.getPose();
    const std::int32_t liftPosition = lift_sensor.get_position();
    const std::int32_t liftVelocity = lift_sensor.get_velocity();
    // CSV-style record for repeated field trials. Pose is odometry output;
    // compare endpoint error against an independent field measurement.
    std::printf(
        "AUTON_RESULT,three_pin,%lu,%.2f,%.2f,%.1f,%d,%d\n",
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
    std::uint32_t last_gps_display_ms = pros::millis() - 100;
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
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
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
        const std::uint32_t now = pros::millis();
        if (now - last_gps_display_ms >= 100) {
            last_gps_display_ms = now;
            gpsreset::print_position_from_start(kGpsDisplayStartXIn,
                                                kGpsDisplayStartYIn);
        }
        pros::delay(20);
    }
}
