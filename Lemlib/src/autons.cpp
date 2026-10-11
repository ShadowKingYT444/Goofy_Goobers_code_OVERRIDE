#include "autons.hpp"
#include "curve_motion.hpp"
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
// GPS use is set at build time:
//   THREE_PIN_KEY_RESETS (default 0): one GPS reset at each key stop.
//     Off: run back to back, the route with them missed stack 1 and the
//     route without them did not. To build it for comparison:
//     pros make EXTRA_CXXFLAGS=-DTHREE_PIN_KEY_RESETS=1
//   THREE_PIN_GPS_RESETS (default 0): additionally correct continuously
//     between stops. Off because it made the route less accurate.
#ifndef THREE_PIN_GPS_RESETS
#define THREE_PIN_GPS_RESETS 0
#endif
#ifndef THREE_PIN_KEY_RESETS
#define THREE_PIN_KEY_RESETS 0
#endif

namespace three_pin {
constexpr bool kGpsResets = THREE_PIN_GPS_RESETS;
constexpr bool kKeyResets = THREE_PIN_KEY_RESETS;
struct Point {
    float x, y;
};
constexpr Point kGoal{20.0f, 15.0f};
constexpr Point kStack1{20.0f, 37.5f};  // about one tile farther from the wall
// One tile beyond the goal, the same spacing as stack 1 (which is picked up
// reliably at 22.5 in from the goal). The route sketch agrees (x about 44).
constexpr Point kStack2{42.5f, 14.0f};
// Rotation center to the claw's grip when picking a stack up. The pickup now
// drives all the way to its stop point instead of stalling an inch short, so
// this is an inch longer than when it was first found.
constexpr float kReach = 8.0f;
// ...and how close the center is sent to the goal's center when scoring a
// stack. Smaller = deeper into the goal.
constexpr float kScoreReach = -1.0f;
// Stack 1 is clamped slightly off-center, so it is scored at a spot a little
// farther from the wall than the goal's center and driven an inch deeper
// than stack 2.
constexpr Point kGoalForStack1{20.0f, 15.75f};
constexpr float kScoreReachStack1 = 0.0f;
constexpr float kGoalSpeed = 90.0f;     // into the goal (LemLib 0-127 scale)
// Longest a GPS reset may hold the robot when the GPS has nothing to offer;
// a good fix takes about 250 ms.
constexpr std::uint32_t kFixWaitMs = 300;
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
constexpr float kGoalAimBeyondIn = 40.0f;   // turn aims this far past the goal
// After clamping a stack the robot keeps driving claw-first along the same
// line (lift already rising) before it turns for the goal. Each stack sits
// on one of the goal's axes (stack 1 shares its X, stack 2 its Y), so
// carrying on toward where the stack stood brings the robot's center onto
// that axis and the run into the goal is close to straight instead of a
// diagonal. kReach would put the center exactly on the axis.
constexpr float kSeatIn = 5.0f;
constexpr float kSeatInStack2 = 6.75f;  // stack 2 needs more to line up
// How much of the starting lift raise must be done before the toggle hit.
constexpr double kToggleLiftFraction = 0.8;
constexpr float kRetreatIn = 10.0f;  // straight out of the goal after stack 1
constexpr float kSeatSpeed = 45.0f;  // floor for that claw-first drive
// Turn to the goal with a stack raised: ends this close, at this floor.
constexpr int kGoalTurnFloor = 10;
constexpr float kGoalTurnExitDeg = 2.0f;
// Lift drop (sensor degrees, down is positive) that sets a stack down on the
// goal before the claw opens: a third of one level.
constexpr double kSeatDropDeg =
    (lift_position::stage_0_deg - lift_position::stage_1_deg) / 3.0;
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
    // A handle on whichever lift move is already running (or last ran).
    struct Running {};
    explicit LiftJob(Running) {}
    explicit LiftJob(double targetDeg) {
        state() = 0;
        pros::Task([targetDeg] { state() = moveLift(targetDeg) ? 1 : -1; });
    }
    bool done() const { return state() != 0; }
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

// GPS reset at a key stop (a clamp or a score). See gpsreset::checkpoint():
// odometry is corrected only if the robot is parked, the GPS is confident,
// and it agrees the robot is within 6 in of `expected` and 8 in of odometry;
// otherwise odometry is kept. Every outcome is kept for the summary shown
// after the route and printed as a GPS_FIX line.
enum KeyStop { kAtPreload, kAtClamp1, kAtScore1, kAtClamp2, kKeyStops };
struct FixLog {
    const char* outcome = "skipped";  // "moved", "agreed", "off", or why not
    double gap_in = 0.0;              // GPS vs odometry at that stop
};
FixLog fix_log[kKeyStops];

void fix_at(Point expected, KeyStop stop) {
    if (!kKeyResets) {
        fix_log[stop] = {"off", 0.0};
        return;
    }
    const gpsreset::Correction fix =
        gpsreset::checkpoint(expected.x, expected.y, 6.0, kFixWaitMs);
    const char* outcome =
        fix.status == gpsreset::Fix::APPLIED ? "moved"
        : fix.status == gpsreset::Fix::ALREADY_ALIGNED ? "agreed"
        : gpsreset::fix_name(fix.status);
    fix_log[stop] = {outcome, fix.gap_in};
}

// Wait for the current move to finish, ending it early if the drive has
// stopped turning: the robot is against the goal and pushing longer only
// wastes time. Returns true if it was ended that way.
bool wait_done_or_stalled() {
    constexpr double kStalledRpm = 12.0;
    constexpr std::uint32_t kStalledMs = 150, kLaunchMs = 300;
    const std::uint32_t began = pros::millis();
    std::uint32_t slow_since = 0;
    while (chassis.isInMotion()) {
        double rpm = 0.0;
        int motors = 0;
        for (double v : left_motors.get_actual_velocity_all())
            rpm += std::abs(v), ++motors;
        for (double v : right_motors.get_actual_velocity_all())
            rpm += std::abs(v), ++motors;
        const std::uint32_t now = pros::millis();
        if (motors == 0 || rpm / motors > kStalledRpm || now - began < kLaunchMs)
            slow_since = 0;
        else if (slow_since == 0)
            slow_since = now;
        else if (now - slow_since >= kStalledMs) {
            chassis.cancelMotion();
            return true;
        }
        pros::delay(10);
    }
    return false;
}

// The point `distance` straight ahead of the robot (front first), by its
// current odometry pose.
Point straight_ahead(float distance) {
    const lemlib::Pose pose = chassis.getPose();
    const float heading = pose.theta * static_cast<float>(M_PI) / 180.0f;
    return {pose.x + std::sin(heading) * distance,
            pose.y + std::cos(heading) * distance};
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

// Turn the claw toward `target`, handing straight over to the drive. Only
// for turns with nothing in the claw: the hand-over leaves the robot still
// rotating, which the leg onto a stack has room to steer out.
void turn_claw_to(Point target) {
    chassis.turnToPoint(target.x, target.y, 900,
                        {.forwards = false, .minSpeed = kTurnChainSpeed,
                         .earlyExitRange = kTurnChainExitDeg}, false);
}

// Back the claw onto a stack and clamp it. If the lift is still coming down
// (`lowering`), it must be down before the robot drives at the stack.
// Returns false if the lift failed; `at` becomes where the robot stopped and
// `seat` the point kSeatIn farther along the same line.
// What one pick_up() actually did, for the summary shown after the route.
struct PickLog {
    float short_in = 0.0f;       // odometry distance left to the clamp spot
    std::uint32_t creep_ms = 0;  // time spent on the slow last stretch
};
constexpr int kCreepTimeoutMs = 2500;

bool pick_up(Point& at, Point& seat, Point stack, PickLog& log,
             LiftJob* lowering = nullptr, float seatIn = kSeatIn) {
    const Point gentle_from = standoff(at, stack, kReach + kGentleZoneIn);
    const Point stop = standoff(at, stack, kReach);
    seat = standoff(at, stack, kReach - seatIn);
    turn_claw_to(stack);
    if (lowering && !lowering->wait()) return false;
    chassis.moveToPoint(gentle_from.x, gentle_from.y, 1500,
                        {.forwards = false, .minSpeed = kGentleEntrySpeed},
                        false);
    set_lateral_kp(kGentleKp);
    const std::uint32_t creeping = pros::millis();
    chassis.moveToPoint(stop.x, stop.y, kCreepTimeoutMs,
                        {.forwards = false, .minSpeed = kGentleFloor}, false);
    set_lateral_kp(kLateralKp);
    brake_drive();
    log.creep_ms = pros::millis() - creeping;
    const lemlib::Pose clamping = chassis.getPose();
    log.short_in = std::hypot(stop.x - clamping.x, stop.y - clamping.y);
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    at = stop;
    return true;
}

// What one score() actually did, for the summary shown after the route.
struct ScoreLog {
    float turn_error_deg = 0.0f;  // claw aim vs. the goal after the turn
    float short_in = 0.0f;        // odometry distance left to the scoring spot
    std::uint32_t entry_ms = 0;   // time spent on the last leg into the goal
    bool stalled = false;         // ended against the goal, not on the spot
    // How far the goal reference moved odometry (route X and Y).
    float reference_dx_in = 0.0f, reference_dy_in = 0.0f;
};
constexpr int kGoalEntryTimeoutMs = 2000;

// Holding a stack: start the lift, keep going claw-first to `seat`, then
// turn the claw to the goal and drive up to kGoalStandbyIn short of the
// scoring spot. The robot goes the rest of the way only once the lift is up
// (without pausing if it already is), stops on the spot, and releases.
// `at` becomes where the robot stopped.
// The lift then starts down to `lowerTo`; once it has dropped kSeatDropDeg
// (the stack is resting on the goal) the claw opens, and the lift keeps
// going down while the caller drives away.
bool score(Point& at, Point seat, float liftDeg, float lowerTo, ScoreLog& log,
           KeyStop clampStop, bool fixAfter, Point goal = kGoal,
           float reach = kScoreReach) {
    // The lift starts the instant the stack is clamped, and the robot does
    // not move until the stack is off the tiles, so it is never dragged.
    const double lift_start = lift_sensor.get_position() / 100.0;
    LiftJob raise(liftDeg);
    const std::uint32_t lifting_since = pros::millis();
    // GPS reset on the clamped stack, in the time the lift needs anyway.
    fix_at(at, clampStop);
    coast_drive();
    while (lift_sensor.get_position() / 100.0 > lift_start - kLiftClearDeg &&
           pros::millis() - lifting_since < 400)
        pros::delay(10);
    chassis.moveToPoint(seat.x, seat.y, 1200,
                        {.forwards = false, .minSpeed = kSeatSpeed}, false);
    brake_drive();
    coast_drive();
    // Everything from here is laid out from where the robot really stopped.
    const lemlib::Pose turning = chassis.getPose();
    const Point here{turning.x, turning.y};
    const Point standby = standoff(here, goal, reach + kGoalStandbyIn);
    const Point stop = standoff(here, goal, reach);
    // A stationary turn that all but stops before the robot drives: with a
    // raised stack a turn handed over while still swinging goes well past.
    // It ends once the claw is within kGoalTurnExitDeg of the goal, moving
    // slowly, instead of waiting to be declared settled; the run in steers
    // out what is left. It aims at a point far beyond the goal on the same
    // line, so the aim does not shift as the robot's position estimate moves
    // during the pivot.
    const Point aim = standoff(here, goal, -kGoalAimBeyondIn);
    chassis.turnToPoint(aim.x, aim.y, 1100,
                        {.forwards = false, .minSpeed = kGoalTurnFloor,
                         .earlyExitRange = kGoalTurnExitDeg}, false);
    const lemlib::Pose turned = chassis.getPose();
    const float claw_deg = turned.theta + 180.0f;
    const float goal_deg = std::atan2(goal.x - turned.x, goal.y - turned.y) *
                           180.0f / static_cast<float>(M_PI);
    log.turn_error_deg = std::remainder(claw_deg - goal_deg, 360.0f);
    // One move all the way to the scoring spot, so LemLib keeps steering at
    // the goal for most of the run (it only stops steering in the last
    // 7.5 in of a move; a separate leg to the standby point used to end the
    // steering after the first few inches). The floor keeps it from stalling
    // short. If the lift is not up by the standby point, stop there, wait,
    // and finish with a second move.
    const lemlib::MoveToPointParams entry{.forwards = false,
                                          .maxSpeed = kGoalSpeed,
                                          .minSpeed = kGentleFloor};
    const std::uint32_t entering = pros::millis();
    chassis.moveToPoint(stop.x, stop.y, kGoalEntryTimeoutMs, entry, true);
    chassis.waitUntil(std::hypot(standby.x - here.x, standby.y - here.y));
    if (chassis.isInMotion() && !raise.done()) {
        chassis.cancelMotion();
        brake_drive();
        if (!raise.wait()) return false;
        coast_drive();
        chassis.moveToPoint(stop.x, stop.y, kGoalEntryTimeoutMs, entry, true);
    }
    log.stalled = wait_done_or_stalled();
    if (!raise.wait()) return false;
    brake_drive();
    log.entry_ms = pros::millis() - entering;
    const lemlib::Pose arrived = chassis.getPose();
    log.short_in = std::hypot(stop.x - arrived.x, stop.y - arrived.y);
    // Set the stack down on the goal, then let go: the claw opens once the
    // lift has dropped kSeatDropDeg and the lift carries on down.
    const double scored_at = lift_sensor.get_position() / 100.0;
    LiftJob lower(lowerTo);
    const std::uint32_t lowering_since = pros::millis();
    while (!lower.done() &&
           lift_sensor.get_position() / 100.0 < scored_at + kSeatDropDeg &&
           pros::millis() - lowering_since < 800)
        pros::delay(10);
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    if (fixAfter) fix_at(stop, kAtScore1);
    // Take position from the goal itself. A stack that is on the goal puts
    // the claw's grip on the goal's center, and the grip is kReach from the
    // robot's center (the same distance the preload scores from and both
    // stacks are picked up at), so the center is kReach from the goal along
    // the robot's heading, whatever odometry says. And odometry is wrong
    // here: it reports the robot's center on the goal's center, which the
    // robot cannot physically reach, and the GPS independently put it 8 in
    // away from odometry just after. The likely source is the turn with a
    // raised stack: with the weight on the claw end the robot pivots about a
    // point behind its center, which swings the center several inches, and
    // without tracking wheels odometry cannot see that. Heading stays with
    // the IMU.
    {
        const lemlib::Pose pressed = chassis.getPose();
        const float heading = pressed.theta * static_cast<float>(M_PI) / 180.0f;
        const Point center{kGoal.x + kReach * std::sin(heading),
                           kGoal.y + kReach * std::cos(heading)};
        log.reference_dx_in = center.x - pressed.x;
        log.reference_dy_in = center.y - pressed.y;
        chassis.setPose(center.x, center.y, pressed.theta);
    }
    coast_drive();
    // Where odometry now says the robot is (a reset may have moved it).
    const lemlib::Pose released = chassis.getPose();
    at = {released.x, released.y};
    return true;
}
}  // namespace three_pin

void three_pin_auton() {
    using namespace three_pin;
    const std::uint32_t started = pros::millis();
    for (FixLog& entry : fix_log) entry = {};
    // In case an earlier run was cut off mid-pickup.
    set_lateral_kp(kLateralKp);
    coast_drive();
    chassis.setPose(0, 0, 180);
    struct AutoReset {
        explicit AutoReset(bool on) { gpsreset::set_auto_reset(on); }
        ~AutoReset() { gpsreset::set_auto_reset(false); }
    } auto_reset(gpsreset::capture_start_as(0, 0, 180) && kGpsResets);

    // TOGGLE: back off the wall while the lift rises, then drive into it.
    // The robot cannot reach the toggle with the lift low, so the hit waits
    // until the lift is most of the way up (kToggleLiftFraction), not for it
    // to finish; it finishes during the hit.
    const double lift_from = lift_sensor.get_position() / 100.0;
    const double lift_to = lift_position::matchload - 125;
    LiftJob raise_preload(lift_to);
    chassis.moveToPoint(0, 8, 500, {.forwards = false}, false);
    const double lift_enough =
        lift_from + kToggleLiftFraction * (lift_to - lift_from);
    const std::uint32_t waiting_since = pros::millis();
    while (!raise_preload.done() && pros::millis() - waiting_since < 1500 &&
           (lift_to < lift_from
                ? lift_sensor.get_position() / 100.0 > lift_enough
                : lift_sensor.get_position() / 100.0 < lift_enough))
        pros::delay(10);
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    // Ends as soon as the robot is stopped against the wall.
    chassis.moveToPoint(0, -2, 700, {.forwards = true}, true);
    wait_done_or_stalled();

    // PRELOAD: back into the goal with the lift still raised, then lower the
    // lift to seat the pin.
    const Point preload{kGoal.x - kReach, kGoal.y};
    // Back out to the lane, turn, back in. Plain, unchained moves, and it
    // stays that way: handing these over without stopping (tried twice) and
    // a single curve (tried twice) both threw the rest of the route off.
    chassis.moveToPoint(0, kGoal.y, 1000, {.forwards = false}, false);
    chassis.turnToHeading(270, 800, {}, false);
    chassis.moveToPoint(preload.x, preload.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed}, false);
    // The lift must be all the way up before it is sent back down.
    if (!raise_preload.wait()) return;
    {
        // GPS reset while the lift comes down.
        LiftJob seat_pin(lift_position::stage_0_deg);
        fix_at(preload, kAtPreload);
        if (!seat_pin.wait()) return;
    }
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);

    // Pull straight out of the goal to the start line (chained).
    Point at{0.0f, kGoal.y};
    chain_to(at, true, 1000);

    // STACK 1: lift is already down. Pick up, score one level up.
    Point seat{};
    PickLog picked1, picked2;
    if (!pick_up(at, seat, kStack1, picked1)) return;
    ScoreLog scored1, scored2;
    if (!score(at, seat, lift_position::stage_1_deg,
               lift_position::stage_0_deg, scored1, kAtClamp1, true,
               kGoalForStack1, kScoreReachStack1))
        return;

    // Pull straight out of the goal, front first, along the way the robot is
    // pointing, while the lift finishes coming down for stack 2, and hand
    // straight over to the turn for stack 2 without stopping. (Not "away
    // from the goal's center": the robot stops close to the center, so that
    // direction is meaningless and made it swing round.)
    chain_to(straight_ahead(kRetreatIn), true, 1200);
    // Stack 2 is laid out from wherever odometry now has the robot.
    const lemlib::Pose retreated = chassis.getPose();
    at = {retreated.x, retreated.y};
    LiftJob lower{LiftJob::Running{}};

    // STACK 2: pick up, score two levels up.
    if (!pick_up(at, seat, kStack2, picked2, &lower, kSeatInStack2)) return;
    // No reset after the last score: nothing is left to aim at.
    if (!score(at, seat, lift_position::stage_2_deg,
               lift_position::stage_1_deg, scored2, kAtClamp2, false))
        return;

    // Leave the goal. Chained so the route ends as soon as the claw is clear.
    chain_to(straight_ahead(8.0f), true, 800);

    // How each score went, by odometry: how far off the goal the claw was
    // aimed after the turn, and how far short of the scoring spot the robot
    // was when it released (BLOCKED: it pushed at the goal until the 2 s limit).
    // "wall" means the drive stopped against the goal before the spot.
    const auto blocked = [](const ScoreLog& s) {
        return s.stalled ? "wall"
               : s.entry_ms + 50 >= static_cast<std::uint32_t>(kGoalEntryTimeoutMs)
                   ? "BLOCKED" : "ok";
    };
    // GPS lines: what each reset did and how far GPS and odometry disagreed.
    // "Grab" lines: how far short of the clamp spot the robot was when it
    // clamped, and how long the slow last stretch took (2.0 s = it ran out
    // of time before getting there).
    fieldviz::report("3 PIN  took %.1f s\n"
                     "Grab 1 short %.1f  %.1f s\n"
                     "Grab 2 short %.1f  %.1f s\n"
                     "S1 aim %+.1f short %.1f %s\n"
                     "S2 aim %+.1f short %.1f %s\n"
                     "Goal ref moved x%+.1f y%+.1f",
                     (pros::millis() - started) / 1000.0,
                     picked1.short_in, picked1.creep_ms / 1000.0,
                     picked2.short_in, picked2.creep_ms / 1000.0,
                     scored1.turn_error_deg, scored1.short_in, blocked(scored1),
                     scored2.turn_error_deg, scored2.short_in, blocked(scored2),
                     scored1.reference_dx_in, scored1.reference_dy_in);
    std::printf("SCORE,1,%.1f,%.1f,%u\nSCORE,2,%.1f,%.1f,%u\n",
                scored1.turn_error_deg, scored1.short_in,
                static_cast<unsigned>(scored1.entry_ms),
                scored2.turn_error_deg, scored2.short_in,
                static_cast<unsigned>(scored2.entry_ms));
}



// Two-pin far route, first sequence only so far: toggle, preload onto the
// goal, back out, curve claw-first onto the wall stack, clamp, lift while
// curving back out, score the stack. Laid out from
// Auto_paths/goofy_goobers_wallpin_v3_pathjerry.txt.
//
// Start-relative like the three-pin: start is (0, 0) facing 180 (toward the
// toggle wall), +Y runs away from that wall. This is the mirror-image side:
// the goal is on -X here (the robot's right as it faces the wall).
//
// Every position below is converted from the path file, which is itself
// traced from a screenshot. They are starting estimates to correct on the
// field, not measurements. The path file starts the robot 9.8 in back from
// the toggle, while this robot starts 2 in from it, so positions are taken
// relative to the toggle contact point (which both agree on) rather than to
// the file's start; that puts the goal's lane 14 in out, close to the 15 in
// the three-pin found on the field.
namespace two_pin_far {
using three_pin::Point;
constexpr float kLaneY = 15.0f;                // both goals' lane
constexpr Point kLane{0.0f, kLaneY};           // on the lane, at the start's X
constexpr float kScoreHeading = 90.0f;         // claw toward the blue goal (-X)
// The blue goal's center is 20 in out (the three-pin's distance, mirrored).
// Robot center when scoring on it: 0.75 in short of a full claw's reach,
// because any closer the robot presses on the goal with its wheels turning.
constexpr float kGoalX = -20.0f;
constexpr Point kScoreSpot{-11.25f, kLaneY};
// Where the robot pulls out to before going for the right wall stack, and
// where it curves back to with it.
constexpr Point kStaging{8.0f, kLaneY};
// The right wall stack: directly behind the blue goal (same X, on the wall
// side) and 6 in toward the wall from the start line.
constexpr Point kWallStack{kGoalX, -6.0f};
// Heading the robot ends on, backed onto that stack (it faces away from the
// stack; the claw points at heading + 180). There are only 21 in between the
// goal and the stack and the robot is 16 in long, so this is a narrow
// window: at the path file's 26.6 the robot's front corner ends up 4.4 in
// from the goal's center (it clipped the goal), and beyond about 40 its back
// corner reaches the wall. 36 leaves about 6.4 in to the goal's center and
// half an inch to the wall.
constexpr float kWallStackHeading = 36.0f;
// How the robot gets onto a wall stack: after pivoting to point the claw at
// it, a curve with a small lead, which runs nearly straight at the stack and
// leaves the swing onto the final heading to the last few inches. A larger
// lead bows the path out toward the goal first.
constexpr float kWallCurveLead = 0.2f;
// The curves are driven slower than the tuner's speed: one ends on a stack
// against the wall, the other carries a raised stack.
constexpr float kCurveSpeed = 70.0f;

// PART 2 (off until part 1 is reliable). The field is taken as symmetric:
// the goals are two tiles (48 in) apart with a wall stack behind each. The
// path file agrees once its 0.9 scale is taken out, but none of these have
// been on the field.
constexpr bool kRunPartTwo = false;
constexpr float kNeutralGoalX = kGoalX + 48.0f;
constexpr Point kLeftWallStack{kNeutralGoalX, -6.0f};
constexpr float kLeftWallStackHeading = -kWallStackHeading;
constexpr float kNeutralScoreHeading = 270.0f;  // claw toward +X
constexpr Point kNeutralScoreSpot{kNeutralGoalX - 8.75f, kLaneY};
// Mirror image of kStaging: the same 28 in from the neutral goal.
constexpr Point kNeutralStaging{kNeutralGoalX - 28.0f, kLaneY};

// Where the robot's center must be for the claw's grip (kReach behind the
// center) to be on `stack` when the robot faces `headingDeg`.
Point spot_for(Point stack, float headingDeg) {
    const float h = headingDeg * static_cast<float>(M_PI) / 180.0f;
    return {stack.x + three_pin::kReach * std::sin(h),
            stack.y + three_pin::kReach * std::cos(h)};
}

// From a staging point on the lane: pivot the claw toward a wall stack, back
// onto it, clamp, start the lift to `liftDeg`, and once the stack is off the
// tiles curve back out nose-first to `out`, ending on `outHeading` (claw
// already toward the goal, so there is no pivot with a raised stack).
// `clamped` is the odometry pose at the clamp. The lift is still rising on
// return; set_down() waits for it.
void take_wall_stack(Point stack, float headingDeg, float liftDeg, Point out,
                     float outHeading, lemlib::Pose& clamped) {
    using namespace three_pin;
    const Point spot = spot_for(stack, headingDeg);
    curve::Settings in = curve::tuned;
    in.lead = kWallCurveLead;
    in.max_speed = kCurveSpeed;
    chassis.turnToPoint(spot.x, spot.y, 900, {.forwards = false}, false);
    curve::to(spot.x, spot.y, headingDeg, false, 2500, in);
    brake_drive();
    clamped = chassis.getPose();
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);

    const double lift_start = lift_sensor.get_position() / 100.0;
    LiftJob raise(liftDeg);
    const std::uint32_t lifting_since = pros::millis();
    while (lift_sensor.get_position() / 100.0 > lift_start - kLiftClearDeg &&
           pros::millis() - lifting_since < 400)
        pros::delay(10);
    coast_drive();
    curve::Settings leaving = curve::tuned;
    leaving.max_speed = kCurveSpeed;
    curve::to(out.x, out.y, outHeading, true, 2500, leaving);
}

// With the lift up, back straight down the lane onto `spot`, lower the lift
// toward `lowerTo`, and open the claw once it has dropped a third of a
// level. Returns false if the lift failed. `scoring` is the odometry pose
// when the robot stopped.
bool set_down(Point spot, float lowerTo, lemlib::Pose& scoring) {
    using namespace three_pin;
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    chassis.moveToPoint(spot.x, spot.y, 1500,
                        {.forwards = false, .maxSpeed = kGoalSpeed}, false);
    brake_drive();
    scoring = chassis.getPose();
    const double scored_at = lift_sensor.get_position() / 100.0;
    LiftJob lower(lowerTo);
    const std::uint32_t lowering_since = pros::millis();
    while (!lower.done() &&
           lift_sensor.get_position() / 100.0 < scored_at + kSeatDropDeg &&
           pros::millis() - lowering_since < 800)
        pros::delay(10);
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    coast_drive();
    return true;
}
}  // namespace two_pin_far

void two_pin_far_auton() {
    using namespace three_pin;
    using namespace two_pin_far;
    const std::uint32_t started = pros::millis();
    set_lateral_kp(kLateralKp);
    coast_drive();
    gpsreset::set_auto_reset(false);
    chassis.setPose(0, 0, 180);
    gpsreset::capture_start_as(0, 0, 180);

    // TOGGLE: identical to the three-pin. Back off the wall while the lift
    // rises, hit once the lift is most of the way up, stop on the wall.
    const double lift_from = lift_sensor.get_position() / 100.0;
    const double lift_to = lift_position::matchload - 125;
    LiftJob raise_preload(lift_to);
    chassis.moveToPoint(0, 8, 500, {.forwards = false}, false);
    const double lift_enough =
        lift_from + kToggleLiftFraction * (lift_to - lift_from);
    const std::uint32_t waiting_since = pros::millis();
    while (!raise_preload.done() && pros::millis() - waiting_since < 1500 &&
           lift_sensor.get_position() / 100.0 > lift_enough)
        pros::delay(10);
    clamp_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    chassis.moveToPoint(0, -2, 700, {.forwards = true}, true);
    wait_done_or_stalled();

    // PRELOAD: back out to the lane, turn the claw to the goal, back in.
    // Plain settled moves, as in the three-pin, where anything else here
    // threw the rest of the route off.
    chassis.moveToPoint(kLane.x, kLane.y, 1000, {.forwards = false}, false);
    chassis.turnToHeading(kScoreHeading, 800, {}, false);
    chassis.moveToPoint(kScoreSpot.x, kScoreSpot.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed}, false);
    if (!raise_preload.wait()) return;
    if (!LiftJob(lift_position::stage_0_deg).wait()) return;
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);

    // Back out along the lane, front first, well clear of the goal.
    chassis.moveToPoint(kStaging.x, kStaging.y, 1200, {.forwards = true}, false);

    // RIGHT WALL STACK onto the blue goal, one level up. The lift is already
    // down from seating the preload.
    lemlib::Pose clamped(0, 0, 0), scoring(0, 0, 0);
    take_wall_stack(kWallStack, kWallStackHeading, lift_position::stage_1_deg,
                    kStaging, kScoreHeading, clamped);
    if (!set_down(kScoreSpot, lift_position::stage_0_deg, scoring)) return;

    // Back out along the lane and take a GPS reset there, parked: the camera
    // looks straight at the toggle wall. The lift finishes coming down.
    chassis.moveToPoint(kStaging.x, kStaging.y, 1200, {.forwards = true}, false);
    brake_drive();
    const gpsreset::Correction fix =
        gpsreset::checkpoint(kStaging.x, kStaging.y);
    coast_drive();

    const Point wall_spot = spot_for(kWallStack, kWallStackHeading);
    fieldviz::report(
        "2 PIN FAR  took %.1f s\n\n"
        "Wall stack\n off %.1f in  %+.1f deg\n"
        "Score\n off %.1f in  %+.1f deg\n\n"
        "GPS reset at the end\n"
        " %s  %.1f in apart",
        (pros::millis() - started) / 1000.0,
        std::hypot(clamped.x - wall_spot.x, clamped.y - wall_spot.y),
        std::remainder(clamped.theta - kWallStackHeading, 360.0f),
        std::hypot(scoring.x - kScoreSpot.x, scoring.y - kScoreSpot.y),
        std::remainder(scoring.theta - kScoreHeading, 360.0f),
        gpsreset::fix_name(fix.status), fix.gap_in);

    // PART 2, off for now (kRunPartTwo): the left wall stack onto the
    // neutral goal. Kept as live code behind the switch, rather than in a
    // comment, so it keeps compiling as the helpers above change.
    if (!kRunPartTwo) return;

    // Along the lane to the mirror-image staging point, lift fully down.
    if (!LiftJob{LiftJob::Running{}}.wait()) return;
    chassis.moveToPoint(kNeutralStaging.x, kNeutralStaging.y, 1000,
                        {.forwards = false}, false);
    // The neutral goal is empty, so the stack is carried just high enough to
    // clear it (the height the preload is carried at) and set all the way
    // down, like the preload.
    take_wall_stack(kLeftWallStack, kLeftWallStackHeading,
                    lift_position::matchload - 125, kNeutralStaging,
                    kNeutralScoreHeading, clamped);
    if (!set_down(kNeutralScoreSpot, lift_position::stage_0_deg, scoring))
        return;
    // Leave the goal so nothing is touching the stack.
    chassis.moveToPoint(kNeutralStaging.x, kNeutralStaging.y, 1200,
                        {.forwards = true}, false);
    fieldviz::report(
        "2 PIN FAR  took %.1f s\n\n"
        "Left wall stack\n off %.1f in  %+.1f deg\n"
        "Neutral score\n off %.1f in  %+.1f deg",
        (pros::millis() - started) / 1000.0,
        std::hypot(clamped.x - spot_for(kLeftWallStack, kLeftWallStackHeading).x,
                   clamped.y - spot_for(kLeftWallStack, kLeftWallStackHeading).y),
        std::remainder(clamped.theta - kLeftWallStackHeading, 360.0f),
        std::hypot(scoring.x - kNeutralScoreSpot.x,
                   scoring.y - kNeutralScoreSpot.y),
        std::remainder(scoring.theta - kNeutralScoreHeading, 360.0f));
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
