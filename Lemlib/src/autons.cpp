#include "autons.hpp"
#include "main.h"
#include "lemlib/pid.hpp"
#include "gps_reset/gps_reset.hpp"
#include "field_display_status.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <new>

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

void moveArm(double targetDeg, std::atomic_bool* cancel) {
    const double kP = 0.55;
    const double tolerance = 3.0;
    const auto starting_mode = pros::competition::get_status();

    while (true) {
        if ((cancel != nullptr && cancel->load()) ||
            pros::competition::get_status() != starting_mode)
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
// pid_autotune_auton() lives in src/pid_autotune.cpp.

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

// GPS mount check. Run it from driver control (LEFT) with room to spin and
// about 2 ft clear ahead.
//   1. Stops at six headings around a full turn in place. The rotation center
//      does not move, so the circle the lens traces gives the lens offsets
//      (least squares over all six stops) and how well the stops agree.
//   2. Drives straight forward. The GPS must report travel in the direction
//      the robot is facing; any difference is error in facing_deg.
// The panel shows the values now in main.cpp next to the measured ones.
void gps_reset_test_auton() {
    constexpr int kStops = 6;
    constexpr double kDriveIn = 18.0;
    constexpr int kTimeoutMs = 2500;
    constexpr std::uint32_t kSettleMs = 600;

    auto halt = [] {
        chassis.cancelAllMotions();
        left_motors.move(0);
        right_motors.move(0);
    };
    auto measure = [&](gpsreset::GpsPose& lens, const char* what) {
        halt();
        pros::delay(kSettleMs);
        if (gpsreset::average_lens_pose(lens, 16, 50)) return true;
        fieldviz::report("GPS MOUNT TEST\n\nFAILED: no GPS fix\n%s.\n"
                         "Move away from the wall\nthe camera faces.", what);
        std::printf("GPS_MOUNT,NO_FIX,%s\n", what);
        return false;
    };

    halt();
    gpsreset::set_auto_reset(false);
    const auto& mount = gpsreset::shared_state();
    const double forward = mount.mount_forward_in;
    const double right = mount.mount_right_in;
    const double facing = mount.mount_facing_deg;
    const lemlib::Pose start = chassis.getPose();

    // 1. Six stops around a turn in place.
    std::vector<gpsreset::GpsPose> stops;
    for (int i = 0; i < kStops; ++i) {
        fieldviz::report("GPS MOUNT TEST\n\nStop %d of %d...", i + 1, kStops);
        if (i > 0)
            chassis.turnToHeading(start.theta + i * 360.0 / kStops, kTimeoutMs,
                                  {}, false);
        gpsreset::GpsPose lens{};
        char what[24];
        std::snprintf(what, sizeof(what), "at stop %d", i + 1);
        if (!measure(lens, what)) return;
        stops.push_back(lens);
    }
    chassis.turnToHeading(start.theta, kTimeoutMs, {}, false);

    // How far the reported center wanders with the offsets in use now.
    double mean_x = 0.0, mean_y = 0.0, wobble = 0.0;
    for (const auto& lens : stops) {
        const auto c = gpsreset::lens_to_center(lens, forward, right);
        mean_x += c.x_in / kStops;
        mean_y += c.y_in / kStops;
    }
    for (const auto& lens : stops) {
        const auto c = gpsreset::lens_to_center(lens, forward, right);
        wobble = std::max(wobble, std::hypot(c.x_in - mean_x, c.y_in - mean_y));
    }

    // 2. Straight drive: direction of travel versus reported heading.
    fieldviz::report("GPS MOUNT TEST\n\nDriving forward...");
    gpsreset::GpsPose before{}, after{};
    if (!measure(before, "before the drive")) return;
    const double heading = chassis.getPose().theta * M_PI / 180.0;
    const lemlib::Pose origin = chassis.getPose();
    chassis.moveToPoint(origin.x + kDriveIn * std::sin(heading),
                        origin.y + kDriveIn * std::cos(heading), kTimeoutMs,
                        {.maxSpeed = 70}, false);
    if (!measure(after, "after the drive")) return;
    chassis.moveToPoint(origin.x, origin.y, kTimeoutMs,
                        {.forwards = false, .maxSpeed = 70}, false);
    halt();

    const double dx = after.x_in - before.x_in, dy = after.y_in - before.y_in;
    const double travelled = std::hypot(dx, dy);
    const double travel_deg = std::atan2(dx, dy) * 180.0 / M_PI;
    const bool facing_measured = travelled >= 0.5 * kDriveIn;
    // Reported heading minus true heading; the same error is in facing_deg.
    const double facing_error = facing_measured
        ? gpsreset::wrap_deg(before.theta_deg - travel_deg) : 0.0;
    const bool facing_ok = std::abs(facing_error) <= 10.0;
    const double facing_use =
        facing_ok ? facing : gpsreset::normalize_deg(facing + facing_error);

    // Offsets are only meaningful with the right camera direction, so refit
    // with corrected headings when the drive showed it was wrong.
    if (!facing_ok)
        for (auto& lens : stops)
            lens.theta_deg = gpsreset::normalize_deg(lens.theta_deg - facing_error);
    const auto fit = gpsreset::fit_mount(stops);
    std::printf("GPS_MOUNT,%d,facing=%.1f,facing_error=%.1f,travel=%.1f,"
                "now=%.2f/%.2f,use=%.2f/%.2f,wobble=%.2f,rms=%.2f\n", fit.ok,
                facing, facing_error, travelled, forward, right,
                fit.forward_in, fit.right_in, wobble, fit.rms_in);
    if (!fit.ok) {
        fieldviz::report("GPS MOUNT TEST\n\nFAILED: stops did not\nform a turn.");
        return;
    }
    const bool offsets_ok = std::hypot(fit.forward_in - forward,
                                       fit.right_in - right) <= 1.0;
    char facing_line[48];
    if (facing_measured)
        std::snprintf(facing_line, sizeof(facing_line), "Facing %s: now %.0f use %.0f",
                      facing_ok ? "OK" : "WRONG", facing, facing_use);
    else
        std::snprintf(facing_line, sizeof(facing_line), "Facing: not measured");
    fieldviz::report(
        "GPS MOUNT TEST\n"
        "%s\n"
        " (drive read %+.0f deg off)\n"
        "Offsets %s\n"
        " now fwd %.1f right %.1f\n"
        " use fwd %.1f right %.1f\n"
        "Spin wobble now %.1f in\n"
        "Fit scatter %.1f in",
        facing_line, facing_error, offsets_ok && facing_ok ? "OK" : "CHANGE",
        forward, right, fit.forward_in, fit.right_in, wobble, fit.rms_in);
}

// Three-pin route, start-relative: start is (0, 0) facing 180 (toward the
// wall with the toggle), +Y runs away from that wall and +X toward the goal.
// The claw is on the BACK, so every pickup and score is driven in reverse.
//
// Element positions below are estimates (one field tile from the goal, per
// the route sketch in Auto_paths) and are the numbers to adjust on the field.
// GPS corrections during the route are OFF by default: in back-to-back runs
// the route was more accurate on encoders + IMU alone. To build the version
// with them:  pros make EXTRA_CXXFLAGS=-DTHREE_PIN_GPS_RESETS=1
#ifndef THREE_PIN_GPS_RESETS
#define THREE_PIN_GPS_RESETS 0
#endif

namespace three_pin {
constexpr bool kGpsResets = THREE_PIN_GPS_RESETS;
struct Point {
    float x, y;
};
constexpr Point kGoal{20.0f, 15.0f};
constexpr Point kStack1{20.0f, 37.5f};  // about one tile farther from the wall
constexpr Point kStack2{42.0f, 15.0f};  // about one tile beyond the goal
// Rotation center to the claw's grip when picking a stack up. The pickup now
// drives all the way to its stop point instead of stalling an inch short, so
// this is an inch longer than when it was first found.
constexpr float kReach = 8.0f;
// ...and how close the center is sent to the goal's center when scoring a
// stack. Smaller = deeper into the goal.
constexpr float kScoreReach = 4.0f;
// Stack 1 is clamped slightly off-center, so it is scored at a spot a little
// farther from the wall than the goal's center and driven an inch deeper.
constexpr Point kGoalForStack1{20.0f, 15.75f};
constexpr float kScoreReachStack1 = 3.0f;
constexpr float kGoalSpeed = 80.0f;     // into the goal (LemLib 0-127 scale)

// Slowing down onto a stack. How fast LemLib arrives is set by the lateral
// kP (its output near a target is kP x distance), not by maxSpeed, so the
// only way to arrive slower is a gentler kP for the last stretch. LemLib
// exposes its PIDs for this ("gain scheduling"). The pickup therefore runs
// the last kGentleZoneIn on kGentleKp, with a power floor so the weaker
// output still carries the claw all the way onto the stack.
constexpr float kLateralKp = 6.0f, kLateralKd = 3.0f;  // must match main.cpp
constexpr float kGentleKp = 3.0f;
constexpr float kGentleZoneIn = 10.0f;
constexpr float kGentleEntrySpeed = 45.0f;  // power handed over at the zone
constexpr float kGentleFloor = 24.0f;       // slowest power while closing in
// Motions that belong together hand over without settling in between.
constexpr float kChainSpeed = 40.0f;
constexpr float kChainExitIn = 2.0f;
constexpr int kTurnChainSpeed = 20;
constexpr float kTurnChainExitDeg = 4.0f;   // the leg that follows re-aims
constexpr float kGoalStandbyIn = 6.0f;      // wait here if the lift is not up
// After clamping a stack the robot keeps driving claw-first along the same
// line (lift already rising) before it turns for the goal. Each stack sits
// on one of the goal's axes (stack 1 shares its X, stack 2 its Y), so
// carrying on toward where the stack stood brings the robot's center onto
// that axis and the run into the goal is close to straight instead of a
// diagonal. kReach would put the center exactly on the axis.
constexpr float kSeatIn = 5.0f;
// Lift travel (sensor degrees, up is negative) that counts as "off the tiles".
constexpr double kLiftClearDeg = 60.0;

// Short or slow legs end still rolling (their power floor carries them to
// the target); braking stops the robot where it was sent instead of letting
// it coast on. Coasting is restored for the normal legs.
void brake_drive() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.brake();
    right_motors.brake();
}
void coast_drive() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void set_lateral_kp(float kP) {
    lemlib::PID& pid = chassis.lateralPID;
    pid.~PID();
    new (&pid) lemlib::PID(kP, 0.0f, kLateralKd, 0.0f, true);
}

// A lift move running alongside the drive. wait() blocks until it is done
// and, like liftOrStop(), stops the chassis if the lift failed.
class LiftJob {
public:
    explicit LiftJob(double targetDeg) {
        state() = 0;
        pros::Task([targetDeg] { state() = moveLift(targetDeg) ? 1 : -1; });
    }
    bool wait() {
        while (state() == 0) pros::delay(10);
        if (state() == 1) return true;
        chassis.cancelAllMotions();
        left_motors.move(0);
        right_motors.move(0);
        return false;
    }

private:
    static std::atomic<int>& state() {
        static std::atomic<int> value{1};
        return value;
    }
};

// GPS checkpoint at a waypoint the robot has just driven to. See
// gpsreset::checkpoint(): it only corrects when the GPS is confident and
// agrees the robot is near `expected`.
void fix_at(Point expected) {
    if (kGpsResets) gpsreset::checkpoint(expected.x, expected.y);
}

// The point `distance` short of `target` on the line from `from`.
Point standoff(Point from, Point target, float distance) {
    const float dx = target.x - from.x, dy = target.y - from.y;
    const float length = std::hypot(dx, dy);
    return {target.x - dx / length * distance, target.y - dy / length * distance};
}

// Straight transit leg that hands over to the next motion without settling.
void chain_to(Point target, bool forwards, int timeoutMs, bool async = false) {
    chassis.moveToPoint(target.x, target.y, timeoutMs,
                        {.forwards = forwards, .minSpeed = kChainSpeed,
                         .earlyExitRange = kChainExitIn}, async);
}

// Turn the claw toward `target`, handing straight over to the drive.
void turn_claw_to(Point target) {
    chassis.turnToPoint(target.x, target.y, 900,
                        {.forwards = false, .minSpeed = kTurnChainSpeed,
                         .earlyExitRange = kTurnChainExitDeg}, false);
}

// Back the claw onto a stack and clamp it. If the lift is still coming down
// (`lowering`), it must be down before the robot drives at the stack.
// Returns false if the lift failed; `at` becomes where the robot stopped and
// `seat` the point kSeatIn farther along the same line.
bool pick_up(Point& at, Point& seat, Point stack, LiftJob* lowering = nullptr) {
    const Point gentle_from = standoff(at, stack, kReach + kGentleZoneIn);
    const Point stop = standoff(at, stack, kReach);
    seat = standoff(at, stack, kReach - kSeatIn);
    turn_claw_to(stack);
    if (lowering && !lowering->wait()) return false;
    chassis.moveToPoint(gentle_from.x, gentle_from.y, 1500,
                        {.forwards = false, .minSpeed = kGentleEntrySpeed},
                        false);
    set_lateral_kp(kGentleKp);
    chassis.moveToPoint(stop.x, stop.y, 1500,
                        {.forwards = false, .minSpeed = kGentleFloor}, false);
    set_lateral_kp(kLateralKp);
    brake_drive();
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    coast_drive();
    fix_at(stop);
    at = stop;
    return true;
}

// Holding a stack: start the lift, keep going claw-first to `seat`, then
// turn the claw to the goal and drive up to kGoalStandbyIn short of the
// scoring spot. The robot goes the rest of the way only once the lift is up
// (without pausing if it already is), stops on the spot, and releases.
// `at` becomes where the robot stopped.
bool score(Point& at, Point seat, float liftDeg, Point goal = kGoal,
           float reach = kScoreReach) {
    // The lift starts the instant the stack is clamped, and the robot does
    // not move until the stack is off the tiles, so it is never dragged.
    const double lift_start = lift_sensor.get_position() / 100.0;
    LiftJob raise(liftDeg);
    const std::uint32_t lifting_since = pros::millis();
    while (lift_sensor.get_position() / 100.0 > lift_start - kLiftClearDeg &&
           pros::millis() - lifting_since < 400)
        pros::delay(10);
    chassis.moveToPoint(seat.x, seat.y, 1200,
                        {.forwards = false, .minSpeed = kGentleFloor}, false);
    brake_drive();
    coast_drive();
    const Point standby = standoff(seat, goal, reach + kGoalStandbyIn);
    const Point stop = standoff(seat, goal, reach);
    turn_claw_to(goal);
    chassis.moveToPoint(standby.x, standby.y, 1500,
                        {.forwards = false, .minSpeed = kChainSpeed}, false);
    if (!raise.wait()) return false;
    // A floor on this short leg so it cannot stall short of the goal.
    chassis.moveToPoint(stop.x, stop.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed,
                         .minSpeed = kGentleFloor}, false);
    brake_drive();
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    coast_drive();
    fix_at(stop);
    at = stop;
    return true;
}
}  // namespace three_pin

void three_pin_auton() {
    using namespace three_pin;
    // In case an earlier run was cut off mid-pickup.
    set_lateral_kp(kLateralKp);
    coast_drive();
    chassis.setPose(0, 0, 180);
    struct AutoReset {
        explicit AutoReset(bool on) { gpsreset::set_auto_reset(on); }
        ~AutoReset() { gpsreset::set_auto_reset(false); }
    } auto_reset(gpsreset::capture_start_as(0, 0, 180) && kGpsResets);

    // TOGGLE: back off the wall while the lift rises, then drive into it.
    chassis.moveToPoint(0, 8, 500, {.forwards = false}, true);
    if (!liftOrStop(lift_position::matchload - 125)) return;
    chassis.waitUntilDone();
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(0, -2, 700, {.forwards = true}, false);

    // PRELOAD: back out to the goal's lane, turn the claw to the goal, back
    // in with the lift still raised, then lower the lift to seat the pin.
    // Plain, unchained moves: chaining the drive out to the lane made the
    // robot roll past it and miss the goal.
    chassis.moveToPoint(0, kGoal.y, 1000, {.forwards = false}, false);
    chassis.turnToHeading(270, 800, {}, false);
    const Point preload{kGoal.x - kReach, kGoal.y};
    chassis.moveToPoint(preload.x, preload.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed}, false);
    if (!liftOrStop(lift_position::stage_0_deg)) return;
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    fix_at(preload);

    // Pull straight out of the goal to the start line (chained).
    Point at{0.0f, kGoal.y};
    chain_to(at, true, 1000);

    // STACK 1: lift is already down. Pick up, score one level up.
    Point seat{};
    if (!pick_up(at, seat, kStack1)) return;
    if (!score(at, seat, lift_position::stage_1_deg, kGoalForStack1,
               kScoreReachStack1))
        return;

    // Back away from the goal and stop; once the claw is clear, start
    // lowering the lift for stack 2.
    const Point retreat = standoff(
        at, kGoal, std::hypot(kGoal.x - at.x, kGoal.y - at.y) + 16.0f);
    chassis.moveToPoint(retreat.x, retreat.y, 1200, {.forwards = true}, true);
    chassis.waitUntil(kGoalClearanceIn);
    LiftJob lower(lift_position::stage_0_deg);
    chassis.waitUntilDone();
    at = retreat;

    // GPS reset before the long diagonal to stack 2, in every build: by now
    // odometry has absorbed a pickup and a score. Applied only if the GPS is
    // confident, the robot is stopped, and the GPS agrees it is within 6 in
    // of this spot and 8 in of odometry; otherwise odometry is kept.
    const bool gps_fixed =
        gpsreset::checkpoint(retreat.x, retreat.y, 6.0, 350);
    fieldviz::print(7, gps_fixed ? "GPS reset applied" : "GPS reset skipped");

    // STACK 2: pick up, score two levels up.
    if (!pick_up(at, seat, kStack2, &lower)) return;
    if (!score(at, seat, lift_position::stage_2_deg)) return;

    // Leave the goal with the lift still up so nothing drags the stack.
    const Point done = standoff(
        at, kGoal, std::hypot(kGoal.x - at.x, kGoal.y - at.y) + 8.0f);
    chassis.moveToPoint(done.x, done.y, 1000, {.forwards = true}, false);
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
