#include "autons.hpp"
#include "main.h"
#include "aivision_reset/aivision_reset.hpp"
#include "lemlib/pid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
void moveLift(double targetDeg) {
    const double kP = 0.35;
    const double tolerance = 8.0;

    while (true) {
        double current = lift_sensor.get_position() / 100.0;
        double error = targetDeg - current;

        if (fabs(error) <= tolerance) break;

        int power = std::clamp(
            static_cast<int>(error * kP),
            -127,
            127
        );

        // Lift needs minimum force, especially when moving DOWN.
        if (error > 0 && power < 90)
            power = 90;
        else if (error < 0 && power > -60)
            power = -60;

        slider_left.move(power);
        slider_right.move(power);

        pros::delay(10);
    }

    slider_left.brake();
    slider_right.brake();
}

namespace {

constexpr std::uint32_t LOOP_MS = 10;
constexpr std::uint32_t TRIAL_TIMEOUT_MS = 1800;
constexpr std::uint32_t SETTLE_MS = 180;

constexpr float DRIVE_TARGET_IN = 18.0f;
constexpr float TURN_TARGET_DEG = 60.0f;

constexpr float DRIVE_MAX_POWER = 65.0f;
constexpr float TURN_MAX_POWER = 65.0f;

constexpr float BAD_SCORE = 1.0e8f;

enum class Axis {
    LATERAL,
    ANGULAR
};

struct TrialResult {
    float score;
    bool fatal;
};

struct Gains {
    float p;
    float d;
    float score;
    bool valid;
};

void stop_drive() {
    left_motors.move(0);
    right_motors.move(0);
}

const char* axis_name(Axis axis) {
    return axis == Axis::LATERAL ? "LAT" : "ANG";
}

float read_axis(Axis axis) {
    if (axis == Axis::LATERAL) {
        return vertical_wheel.getDistanceTraveled();
    }
    return static_cast<float>(imu.get_rotation());
}

void command_axis(Axis axis, float power) {
    const int p = static_cast<int>(std::clamp(power, -127.0f, 127.0f));

    if (axis == Axis::LATERAL) {
        left_motors.move(p);
        right_motors.move(p);
    } else {
        // Positive power should produce positive IMU rotation on this drivetrain.
        left_motors.move(p);
        right_motors.move(-p);
    }
}

TrialResult run_trial(Axis axis, float kP, float kD, float target) {
    stop_drive();
    pros::delay(250);

    const float start = read_axis(axis);
    if (!std::isfinite(start)) {
        pros::lcd::print(7, "%s SENSOR ERROR", axis_name(axis));
        std::printf("AUTOTUNE FATAL: %s sensor is invalid\n", axis_name(axis));
        return {BAD_SCORE, true};
    }

    // Use LemLib's own PID class so the candidate gains are evaluated with
    // the same P/D implementation as LemLib 0.5.6.
    lemlib::PID pid(kP, 0.0f, kD, 0.0f, true);

    const float settle_error = axis == Axis::LATERAL ? 0.50f : 1.50f;
    const float sign_check_distance = axis == Axis::LATERAL ? 1.0f : 5.0f;
    const float unsafe_extra = axis == Axis::LATERAL ? 10.0f : 40.0f;
    const float max_power = axis == Axis::LATERAL ? DRIVE_MAX_POWER : TURN_MAX_POWER;

    float previous_error = target;
    float iae = 0.0f;
    float overshoot = 0.0f;
    int crossings = 0;
    std::uint32_t settled_for = 0;
    bool settled = false;

    const std::uint32_t began = pros::millis();

    while (pros::millis() - began < TRIAL_TIMEOUT_MS) {
        const float position = read_axis(axis) - start;
        const float error = target - position;

        if (!std::isfinite(position) || !std::isfinite(error)) {
            stop_drive();
            pros::lcd::print(7, "%s SENSOR ERROR", axis_name(axis));
            return {BAD_SCORE, true};
        }

        // If motion is clearly going opposite the sensor target, this is not
        // a tuning problem. Stop before a bad sensor sign launches the robot.
        const std::uint32_t elapsed = pros::millis() - began;
        if (elapsed > 250 &&
            std::fabs(position) > sign_check_distance &&
            position * target < 0.0f) {
            stop_drive();
            pros::lcd::print(7, "%s SIGN WRONG", axis_name(axis));
            std::printf(
                "AUTOTUNE FATAL: %s sign wrong. target=%.2f measured=%.2f\n",
                axis_name(axis), target, position);
            return {BAD_SCORE, true};
        }

        // Bad candidate, but not a hardware/configuration failure.
        if (std::fabs(position) > std::fabs(target) + unsafe_extra) {
            stop_drive();
            return {BAD_SCORE, false};
        }

        float output = pid.update(error);
        output = std::clamp(output, -max_power, max_power);
        command_axis(axis, output);

        if ((error > 0.0f) != (previous_error > 0.0f) &&
            std::fabs(previous_error) > settle_error) {
            ++crossings;
        }

        const float this_overshoot =
            target > 0.0f
                ? std::max(0.0f, position - target)
                : std::max(0.0f, target - position);
        overshoot = std::max(overshoot, this_overshoot);

        iae += std::fabs(error) * (LOOP_MS / 1000.0f);

        if (std::fabs(error) <= settle_error) {
            settled_for += LOOP_MS;
        } else {
            settled_for = 0;
        }

        previous_error = error;

        if (settled_for >= SETTLE_MS) {
            settled = true;
            break;
        }

        pros::delay(LOOP_MS);
    }

    stop_drive();

    // Include coast / mechanical settling in the score.
    pros::delay(120);

    const float final_position = read_axis(axis) - start;
    const float final_error = std::fabs(target - final_position);
    const float seconds =
        static_cast<float>(pros::millis() - began) / 1000.0f;

    // Lower is better. This strongly penalizes final error and overshoot,
    // while still rewarding fast settling with little oscillation.
    float score =
        20.0f * final_error +
        12.0f * overshoot +
        5.0f * static_cast<float>(crossings) +
        0.5f * iae +
        2.0f * seconds;

    if (!settled) score += 80.0f;

    return {score, false};
}

Gains evaluate(Axis axis, float kP, float kD, float target) {
    pros::lcd::print(7, "%s P%.2f D%.2f", axis_name(axis), kP, kD);
    std::printf("AUTOTUNE %s testing P=%.3f D=%.3f\n",
                axis_name(axis), kP, kD);

    const TrialResult positive = run_trial(axis, kP, kD, target);
    if (positive.fatal) return {kP, kD, BAD_SCORE, false};

    pros::delay(250);

    const TrialResult negative = run_trial(axis, kP, kD, -target);
    if (negative.fatal) return {kP, kD, BAD_SCORE, false};

    const float score = 0.5f * (positive.score + negative.score);

    std::printf("AUTOTUNE %s P=%.3f D=%.3f score=%.2f\n",
                axis_name(axis), kP, kD, score);

    return {kP, kD, score, true};
}

// Small bounded coordinate search ("twiddle-lite").
// We start at LemLib's normal baseline and repeatedly test +/-P and +/-D.
// Two refinement rounds keeps the whole tuner short enough to run on-field.
Gains tune_axis(Axis axis,
                float start_p,
                float start_d,
                float p_step,
                float d_step,
                float target) {
    Gains best = evaluate(axis, start_p, start_d, target);
    if (!best.valid) return best;

    for (int round = 0; round < 2; ++round) {
        const float base_p = best.p;
        const float base_d = best.d;

        const float candidates[4][2] = {
            {base_p + p_step, base_d},
            {std::max(0.05f, base_p - p_step), base_d},
            {base_p, base_d + d_step},
            {base_p, std::max(0.0f, base_d - d_step)}
        };

        for (const auto& candidate : candidates) {
            Gains test = evaluate(
                axis,
                candidate[0],
                candidate[1],
                target
            );

            if (!test.valid) return test;

            if (test.score < best.score) {
                best = test;
            }
        }

        p_step *= 0.5f;
        d_step *= 0.5f;
    }

    // Re-run the winner once so a lucky/noisy single trial does not win.
    Gains validation = evaluate(axis, best.p, best.d, target);
    if (!validation.valid) return validation;

    if (validation.score < BAD_SCORE) {
        best.score = 0.5f * (best.score + validation.score);
    }

    return best;
}

} // namespace

// This is now the only tuning autonomous you need.
void pid_autotune_auton() {
    chassis.cancelAllMotions();
    stop_drive();

    std::printf("\n=== LEMLIB PID AUTOTUNE START ===\n");
    pros::lcd::print(7, "AUTOTUNE START");
    pros::delay(500);

    // LemLib's documented baseline is approximately LAT 10/3, ANG 2/10.
    Gains lateral = tune_axis(
        Axis::LATERAL,
        10.0f, 3.0f,
        4.0f, 3.0f,
        DRIVE_TARGET_IN
    );

    if (!lateral.valid) {
        stop_drive();
        pros::lcd::print(7, "AUTOTUNE ABORT LAT");
        return;
    }

    pros::delay(600);

    Gains angular = tune_axis(
        Axis::ANGULAR,
        2.0f, 10.0f,
        0.8f, 5.0f,
        TURN_TARGET_DEG
    );

    stop_drive();

    if (!angular.valid) {
        pros::lcd::print(7, "AUTOTUNE ABORT ANG");
        return;
    }

    std::printf("\n=== LEMLIB PID AUTOTUNE DONE ===\n");
    std::printf("Lateral: kP=%.3f kI=0 kD=%.3f\n",
                lateral.p, lateral.d);
    std::printf("Angular: kP=%.3f kI=0 kD=%.3f\n",
                angular.p, angular.d);

    std::printf(
        "lemlib::ControllerSettings lateral_controller("
        "%.3f, 0, %.3f, 0, 1, 100, 3, 500, 0);\n",
        lateral.p, lateral.d
    );

    std::printf(
        "lemlib::ControllerSettings angular_controller("
        "%.3f, 0, %.3f, 0, 1, 100, 3, 500, 0);\n",
        angular.p, angular.d
    );

    // main.cpp owns LCD lines 0-6, so line 7 is intentionally reserved
    // for the tuner and will not be overwritten by the pose debug task.
    pros::lcd::print(
        7,
        "LP%.1f D%.1f AP%.1f D%.1f",
        lateral.p, lateral.d,
        angular.p, angular.d
    );
}

// Kept only so the existing header/main still links if this symbol is declared.
// The old separate manual sign-test autonomous is no longer part of tuning.
void test_shi() {
    moveLift(lift_position::stage_1_deg);
    pros::delay(500);
    moveLift(lift_position::stage_0_deg);
    pros::delay(500);


}
void motion_test_auton() {
    chassis.setPose(0, 0, 0);

    // ===== LATERAL PID TEST =====

    // Drive forward 24"
    chassis.moveToPoint(
        0, 24,
        3000,
        {.maxSpeed = 80},
        false
    );

    pros::delay(500);

    // Back straight to start without turning around
    chassis.moveToPoint(
        0, 0,
        3000,
        {.forwards = false, .maxSpeed = 80},
        false
    );

    pros::delay(700);


    // ===== ANGULAR PID TEST =====

    // 0 -> 90
    chassis.turnToHeading(
        90,
        2500,
        {.maxSpeed = 70},
        false
    );

    pros::delay(500);

    // 90 -> 180
    chassis.turnToHeading(
        180,
        2500,
        {.maxSpeed = 70},
        false
    );

    pros::delay(500);

    // 180 -> 90
    chassis.turnToHeading(
        90,
        2500,
        {.maxSpeed = 70},
        false
    );

    pros::delay(500);

    // 90 -> 0
    chassis.turnToHeading(
        0,
        2500,
        {.maxSpeed = 70},
        false
    );
}
constexpr double CLAW_AFTER_GOAL = -500.0;
// Relative frame measured on the robot:
//   start = (0, 0, 0)
//   +X = robot-left at start
//   +Y = robot-rear / into field at start
//   +theta = right turn
//
// IMPORTANT:
// The REAR / AI-Vision side is BOTH the Pin pickup side and the scoring side.
// Therefore every Pin approach and every Goal approach uses .forwards = false.
//
// Robot footprint used here: 12" wide x 16" long.
// Rear-center is 8" behind the LemLib tracking center.

void one_pin_auton() {
    chassis.setPose(0, 0, 180);

    // Toggle: physical rear moves into field, then physical front returns.
    chassis.moveToPoint(0, 6, 700,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(false);
    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);
    // REAR goes into blue Goal.
    chassis.moveToPoint(0, 16, 1100,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(-9, 16, 1100,
                        {.forwards = false},
                        false);
    //go back a bit cuz arm.
    chassis.moveToPoint(-3.5, 16, 1100,
                        {.forwards = true},
                        false);
    claw_arm.move_absolute(2250, -120);
    pros::delay(1200);
    claw_piston.set_value(false);
    pros::delay(100);
    
    // Pull straight out only enough to clear the Goal.
    chassis.moveToPoint(9, 16.11, 650,
                        {.forwards = true},
                        false);
    //turn to face CUP
    chassis.turnToHeading(90,500);
    claw_arm.move_absolute(3000, -120);
    pros::delay(500);
    // CENTER PIN: same REAR/camera side approaches and touches the Pin.
    chassis.moveToPoint(7, 30.7, 1200,
                        {.forwards = false, .maxSpeed = 90},
                        false);

    // Add Pin pickup action here.
    claw_piston.set_value(true);
    chassis.moveToPoint(8, 16, 650,
                        {.forwards = true},
                        false);
    // Turn the SAME rear side toward the BLACK Goal and score.
    chassis.turnToPoint(30.45, 17.11, 700,
                        {.forwards = false},
                        false);
    moveLift(lift_position::stage_2_deg);
    pros::delay(600);

    chassis.moveToPoint(24.08, 17.11, 1100,
                        {.forwards = false},
                        false);
    claw_piston.set_value(false);
}


void three_pin_auton() {
    chassis.setPose(0, 0, 180);
    // First motion test: use LemLib's normal output while PID and odometry are
    // being validated.
    chassis.moveToPoint(0, 6, 500,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(false);
    pros::delay(10);

    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);

    // PRELOAD -> BLUE Goal. REAR/camera is the scoring side.
    

    chassis.moveToPoint(0, 16.5, 700,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(11, 16.5, 700,
                        {.forwards = false, .maxSpeed = 85},
                        false);
    chassis.moveToPoint(5, 16.5, 700,
                        {.forwards = false},
                        false);
    claw_arm.move_absolute(2250, -120);
    pros::delay(1000);
    claw_piston.set_value(false);
    pros::delay(100);
    // Pull straight out only enough to clear the Goal.
    chassis.moveToPoint(-9, 17, 650,
                        {.forwards = true},
                        false);
    
    // PIN #2 BIG ISSUE HERE: rear/camera side faces and enters the Pin.
    chassis.turnToPoint(15.2, 32.5, 700,
                        {.forwards = false},
                        false);
    claw_arm.move_absolute(3050, -120);
    pros::delay(200);
    chassis.moveToPoint(15.2 , 32.5, 1200,
                        {.forwards = false, .maxSpeed = 90},
                        false);
    
    //PICKUP
    claw_piston.set_value(true);
    
    chassis.moveToPoint(24, 37.41, 1200,
                        {.forwards = false},
                        false);
    // Same side turns back toward Goal.
    chassis.turnToPoint(24, 14.11, 700,
                        {.forwards = false},
                        false);
    moveLift(lift_position::stage_1_deg);
    pros::delay(500);
    chassis.moveToPoint(24, 17.11, 1050,
                        {.forwards = false},
                        false);
    
    //SCORE CUP #1
    claw_piston.set_value(false);
    pros::delay(20);
    
    //backup
    chassis.moveToPoint(24, 37.41, 1200,
                        {.forwards = true},
                        false);
    moveLift(lift_position::stage_0_deg);
    pros::delay(300);
    // PIN #3: direct after clearing Goal; REAR/camera side picks it up.
    chassis.turnToPoint(40.20, 14.5, 700,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(40.20, 18.40, 1200,
                        {.forwards = false},
                        false);

    // Add Pin #3 pickup action here.
    claw_piston.set_value(true);
    pros::delay(50);
    moveLift(lift_position::stage_2_deg);
    pros::delay(700);
    chassis.moveToPoint(43.20, 16, 1200,
                        {.forwards = true},
                        false);
    chassis.turnToPoint(7, 16, 700,
                        {.forwards = false},
                        false);
    
    chassis.moveToPoint(15, 16, 1050,
                        {.forwards = false},
                        false);
    claw_piston.set_value(false);
    

    

}
