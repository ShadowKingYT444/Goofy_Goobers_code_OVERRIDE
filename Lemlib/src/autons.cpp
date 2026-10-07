#include "autons.hpp"
#include "main.h"
#include "lemlib/pid.hpp"
#include "gps_reset/gps_reset.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

bool moveLift(double targetDeg, std::uint32_t timeoutMs) {
    constexpr double kP = 0.35;
    constexpr double kPositionToleranceDeg = 8.0;
    constexpr std::uint32_t kLoopDelayMs = 10;
    constexpr std::uint32_t kMaxTimeoutMs = 8000;
    constexpr double kPositionSafetyMarginDeg = 100.0;

    auto stop = [](bool success) {
        slider_left.brake();
        slider_right.brake();
        return success;
    };

    if (!std::isfinite(targetDeg) || timeoutMs == 0 ||
        timeoutMs > kMaxTimeoutMs ||
        targetDeg < lift_position::max_height_deg ||
        targetDeg > lift_position::stage_0_deg) {
        std::printf("LIFT_FAIL,invalid_target_or_timeout,%.1f,%lu\n",
                    targetDeg,
                    static_cast<unsigned long>(timeoutMs));
        return stop(false);
    }

    const std::uint32_t startedAt = pros::millis();
    double lastCurrentDeg = std::numeric_limits<double>::quiet_NaN();
    double lastVelocityDegPerSec = std::numeric_limits<double>::quiet_NaN();
    auto fail = [&](const char* reason, double currentDeg,
                    double velocityDegPerSec) {
        const std::uint32_t elapsedMs = pros::millis() - startedAt;
        std::printf("LIFT_FAIL,%s,target=%.1f,current=%.1f,velocity=%.1f,elapsed_ms=%lu\n",
                    reason, targetDeg, currentDeg, velocityDegPerSec,
                    static_cast<unsigned long>(elapsedMs));
        return stop(false);
    };

    while (pros::millis() - startedAt < timeoutMs) {
        const std::int32_t positionRaw = lift_sensor.get_position();
        const std::int32_t velocityRaw = lift_sensor.get_velocity();
        if (positionRaw == PROS_ERR || velocityRaw == PROS_ERR)
            return fail("sensor", lastCurrentDeg, lastVelocityDegPerSec);

        const double currentDeg = positionRaw / 100.0;
        const double velocityDegPerSec = velocityRaw / 100.0;
        if (!std::isfinite(currentDeg) || !std::isfinite(velocityDegPerSec))
            return fail("non_finite_sensor", currentDeg, velocityDegPerSec);
        if (currentDeg <
                lift_position::max_height_deg - kPositionSafetyMarginDeg ||
            currentDeg >
                lift_position::stage_0_deg + kPositionSafetyMarginDeg)
            return fail("position_bound", currentDeg, velocityDegPerSec);

        lastCurrentDeg = currentDeg;
        lastVelocityDegPerSec = velocityDegPerSec;

        const double errorDeg = targetDeg - currentDeg;
        const std::uint32_t now = pros::millis();
        if (std::abs(errorDeg) <= kPositionToleranceDeg) {
            std::printf("LIFT_OK,target=%.1f,current=%.1f,velocity=%.1f,elapsed_ms=%lu\n",
                        targetDeg, currentDeg, velocityDegPerSec,
                        static_cast<unsigned long>(now - startedAt));
            return stop(true);
        }

        // Restore the previous proportional response and motor power floors.
        int power = std::clamp(static_cast<int>(errorDeg * kP), -127, 127);
        if (errorDeg > 0 && power < 90)
            power = 90;
        else if (errorDeg < 0 && power > -60)
            power = -60;

        slider_left.move(power);
        slider_right.move(power);
        pros::delay(kLoopDelayMs);
    }

    return fail("timeout", lastCurrentDeg, lastVelocityDegPerSec);
}

namespace {
// Scheduling guards only; validate the minimum reliable pneumatic time and
// mechanism clearance on the assembled robot before shortening either one.
constexpr std::uint32_t kPneumaticSettleMs = 75;
constexpr float kGoalClearanceIn = 6.0f;
constexpr float kPinClearanceIn = 3.0f;

bool liftOrStop(double targetDeg) {
    if (moveLift(targetDeg)) return true;
    chassis.cancelAllMotions();
    left_motors.move(0);
    right_motors.move(0);
    return false;
}
}  // namespace

void moveArm(double targetDeg, volatile bool* cancel) {
    const double kP = 0.55;
    const double tolerance = 3.0;

    while (true) {
        if (cancel != nullptr && *cancel)
            break;
        double current = claw_sensor.get_position() / 100.0;
        double error = targetDeg - current;

        if (fabs(error) <= tolerance)
            break;

        int power = static_cast<int>(error * kP);

        // Normal speed.
        power = std::clamp(power, -110, 110);

        // Soft landing near either endpoint.
        if (fabs(error) < 60)
            power = std::clamp(power, -35, 35);

        if (fabs(error) < 25)
            power = std::clamp(power, -20, 20);

        // Minimum power only when sufficiently far away.
        if (fabs(error) > 25) {
            if (power > 0 && power < 25)
                power = 25;
            else if (power < 0 && power > -25)
                power = -25;
        }

        claw_arm.move(-power);

        pros::delay(10);
    }
    claw_arm.brake();
}
/*

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
*/

// Kept only so the existing header/main still links if this symbol is declared.
// The old separate manual sign-test autonomous is no longer part of tuning.
void test_shi() {
    gps_reset::capture_start_as(0,0,180);
    pros::delay(1000);
    gps_reset::reset();
    chassis.moveToPoint(
        0, 24,
        3000,
        {.maxSpeed = 80},
        false
    );  
    gps_reset::reset();
    pros::delay(500);

    // Back straight to start without turning around
    
    chassis.moveToPoint(
        0, 0,
        3000,
        {.forwards = false, .maxSpeed = 80},
        false
    );
    gps_reset::reset();
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
// Robot footprint used here: 12" wide x11 16" long.
// Rear-center is 8" behind the LemLib tracking center.

void one_pin_auton() {
    chassis.setPose(0, 0, 180);
    const bool gps_anchor_ok = gpsreset::capture_start_as(0, 0, 180);
    if (!liftOrStop(lift_position::stage_2_deg)) return;
    // Toggle: physical rear moves into field, then physical front returns.
    chassis.moveToPoint(0, 6, 700,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);
    // REAR goes into blue Goal.
    chassis.moveToPoint(0, 15, 1100,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(-11, 15, 1100,
                        {.forwards = false},
                        false);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    if (gps_anchor_ok) gpsreset::reset();
    claw_piston.set_value(false);
    pros::delay(100);
    
    // Pull straight out only enough to clear the Goal.
    
    chassis.moveToPoint(0, 15, 650,
                        {.forwards = true},
                        false);
    
    //turn to face CUP ANOTHA BIG ISSUE:
    chassis.turnToPoint(-21, -4, 700,
                        {.forwards = false},
                        false);
    

    chassis.moveToPoint(-15, -2, 1200,
                        {.forwards = false, .maxSpeed=80},
                        false);
    // Finish alignment before the final reverse approach. Starting this turn
    // from here leaves enough distance for moveToPoint() to steer normally.
    chassis.turnToPoint(-19.7, -5, 500,
                        {.forwards = false, .maxSpeed = 95},
                        false);
    chassis.moveToPoint(-19.7, -5, 1200,
                        {.forwards = false, .maxSpeed=80},
                        false);

    // Add Pin pickup action here.
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    // First travel clear of the pickup before raising with the cup held.
    chassis.moveToPoint(5, 14.5, 650,
                        {.forwards = true},
                        true);
    chassis.waitUntil(8.0);
    if (!liftOrStop(lift_position::stage_1_deg)) return;
    chassis.waitUntilDone();
    chassis.turnToPoint(-14, 14.5, 700,
                        {.forwards = false},
                        false);
    
    chassis.moveToPoint(-18, 14.5, 700,
                        {.forwards = false},
                        false);
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    


}
void one_pin_close(){
    chassis.setPose(0, 0, 180);
    const bool gps_anchor_ok = gps_reset::capture_start_as(0,0,180);
    // First motion test: use LemLib's normal output while PID and odometry are
    // being validated.
    if (!liftOrStop(lift_position::matchload - 125)) return;
    chassis.moveToPoint(0, 8, 500,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);

    // PRELOAD -> BLUE Goal. REAR/camera is the scoring side.
    

    chassis.moveToPoint(0, 15, 700,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(13, 15, 700,
                        {.forwards = false, .maxSpeed = 95},
                        false);
    if (!liftOrStop(lift_position::normal_shi)) return;
    if (gps_anchor_ok) gps_reset::reset();
    claw_piston.set_value(false);
    pros::delay(100);
    // Pull straight out only enough to clear the Goal.
    chassis.moveToPoint(0, 15, 650,
                        {.forwards = true},
                        false);
}

void three_pin_auton() {
    chassis.setPose(0, 0, 180);
    // First motion test: use LemLib's normal output while PID and odometry are
    // being validated.
    if (!liftOrStop(lift_position::matchload - 125)) return;
    chassis.moveToPoint(0, 8, 500,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(true);
    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);

    // PRELOAD -> BLUE Goal. REAR/camera is the scoring side.
    

    chassis.moveToPoint(0, 15, 700,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(13, 15, 700,
                        {.forwards = false, .maxSpeed = 80},
                        false);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    claw_piston.set_value(false);
    
    // Pull straight out only enough to clear the Goal.
    chassis.moveToPoint(0, 16.5, 650,
                        {.forwards = true},
                        false);
    
    // PIN #2 BIG ISSUE HERE: rear/camera side faces and enters the Pin.
    chassis.turnToPoint(13.5,31,650,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(13.5 , 31 , 700,
                        {.forwards = false},
                        false);
    
    //PICKUP
    claw_piston.set_value(true);
    
    chassis.moveToPoint(20, 36, 1200,
                        {.forwards = false},
                        false);
    // Same side turns back toward Goal.
    chassis.turnToPoint(20, 14.11, 700,
                        {.forwards = false},
                        false);
    if (!liftOrStop(lift_position::stage_1_deg)) return;

    
    chassis.moveToPoint(20, 17, 1050,
                        {.forwards = false},
                        false);
    
    //SCORE CUP #1
    claw_piston.set_value(false);

    // Back clear, then lower before starting the next turn.
    chassis.moveToPoint(17, 35, 1200,
                        {.forwards = true},
                        false);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    
    chassis.turnToPoint(-12, 35, 700,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(0.5, 35, 700,
                        {.forwards = false},
                        false);
    claw_piston.set_value(true);
    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(0.5, 40, 1200,
                        {.forwards = false},
                        false);
    /*
    // PIN #3: direct after clearing Goal; REAR/camera side picks it up.
    chassis.turnToPoint(38, 14.5, 700,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(38, 15, 1200,
                        {.forwards = false},
                        false);

    // Add Pin #3 pickup action here.
    claw_piston.set_value(true);
    if (!liftOrStop(lift_position::stage_2_deg)) return;
    chassis.moveToPoint(43.20, 16, 1200,
                        {.forwards = true},
                        false);
    chassis.turnToPoint(7, 15, 700,
                        {.forwards = false},
                        false);
    
    chassis.moveToPoint(15, 16, 1050,
                        {.forwards = false},
                        false);
    claw_piston.set_value(false);
    chassis.moveToPoint(43, 16, 1050,
                        {.forwards = true},
                        false);
    chassis.turnToPoint(0, 45, 1050,{.forwards = false}, false);
    chassis.moveToPoint(0,45,2000, {.forwards = false}, false);

    
*/
}

    

void skills() {
    chassis.setPose(0, 0, 180);
    const bool gps_anchor_ok = gpsreset::capture_start_as(0, 0, 180);
    // First motion test: use LemLib's normal output while PID and odometry are
    // being validated.
    if (!liftOrStop(lift_position::matchload - 125)) return;
    chassis.moveToPoint(0, 8, 500,
                        {.forwards = false},
                        false);
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(0, -2, 700,
                        {.forwards = true},
                        false);

    // PRELOAD -> BLUE Goal. REAR/camera is the scoring side.
    

    chassis.moveToPoint(0, 15, 700,
                        {.forwards = false},
                        false);
    chassis.turnToHeading(-90,500);
    chassis.moveToPoint(13, 15, 700,
                        {.forwards = false, .maxSpeed = 80},
                        false);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    if (gps_anchor_ok) gpsreset::reset();
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    
    // Pull straight out only enough to clear the Goal.
    chassis.moveToPoint(0, 16.5, 650,
                        {.forwards = true},
                        false);
    
    // PIN #2 BIG ISSUE HERE: rear/camera side faces and enters the Pin.
    chassis.turnToPoint(13.5,31,650,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(13.5 , 31 , 700,
                        {.forwards = false},
                        false);
    
    //PICKUP
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    
    chassis.moveToPoint(20, 36, 1200,
                        {.forwards = false},
                        false);
    // Same side turns back toward Goal.
    chassis.turnToPoint(20, 14.11, 700,
                        {.forwards = false},
                        true);
    if (!liftOrStop(lift_position::stage_1_deg)) return;
    chassis.waitUntilDone();

    
    chassis.moveToPoint(20, 17, 1050,
                        {.forwards = false},
                        false);
    
    //SCORE CUP #1
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    
    // Clear the Goal before lowering, while overlapping the remaining drive.
    chassis.moveToPoint(17, 35, 1200,
                        {.forwards = true},
                        true);
    chassis.waitUntil(kGoalClearanceIn);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    chassis.waitUntilDone();
    /*
    chassis.turnToPoint(-12, 35, 700,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(0.5, 35, 700,
                        {.forwards = false},
                        false);
    claw_piston.set_value(true);
    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(0.5, 40, 1200,
                        {.forwards = false},
                        false);
    */
    // PIN #3: direct after clearing Goal; REAR/camera side picks it up.
    chassis.turnToPoint(38, 14.5, 700,
                        {.forwards = false},
                        false);
    chassis.moveToPoint(38, 15, 1200,
                        {.forwards = false},
                        false);

    // Add Pin #3 pickup action here.
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    // Move clear with the cup low, then raise during the remainder of the
    // retreat. waitUntilDone() preserves the next turn's endpoint ordering.
    chassis.moveToPoint(43.20, 16, 1200,
                        {.forwards = true},
                        true);
    chassis.waitUntil(kPinClearanceIn);
    if (!liftOrStop(lift_position::stage_2_deg)) return;
    chassis.waitUntilDone();
    chassis.turnToPoint(7, 15, 700,
                        {.forwards = false},
                        false);
    
    chassis.moveToPoint(15, 16, 1050,
                        {.forwards = false},
                        false);
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(43, 16, 1050,
                        {.forwards = true},
                        false);
    chassis.turnToPoint(0, 45, 1050,{.forwards = false}, false);
    chassis.moveToPoint(0,45,2000, {.forwards = false}, false);

    

}
