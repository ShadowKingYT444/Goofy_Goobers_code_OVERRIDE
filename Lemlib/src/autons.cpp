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

namespace {
// Time of the last moveLift() loop pass. A timestamp rather than a flag, so
// a moveLift() killed with its task (end of the autonomous period) cannot
// leave the lift marked busy.
std::atomic<std::uint32_t> lift_move_seen_ms{0};
std::atomic_bool lift_move_seen{false};
std::atomic_bool lift_lock_enabled{false};
}  // namespace

bool liftMoveActive() {
    return lift_move_seen.load() &&
           pros::millis() - lift_move_seen_ms.load() < 50;
}

bool holdLiftAtMax() {
    // Starting values: raise kHoldPower if the loaded lift still sags at max
    // height, lower it if the motors run hot.
    constexpr int kHoldPower = 20;  // feedforward against gravity
    constexpr double kP = 1.0;      // extra power per degree below max height

    const std::int32_t positionRaw = lift_sensor.get_position();
    if (positionRaw == PROS_ERR) return false;
    const double currentDeg = positionRaw / 100.0;
    if (currentDeg > lift_position::lock_from_deg) return false;

    // Up is negative, for both the sensor and the motors.
    const double belowMaxDeg = currentDeg - lift_position::max_height_deg;
    const int power =
        std::clamp(kHoldPower + static_cast<int>(belowMaxDeg * kP), 0, 127);
    slider_left.move(-power);
    slider_right.move(-power);
    return true;
}

void setLiftLock(bool enabled) {
    lift_lock_enabled.store(enabled);
    if (!enabled) return;
    static pros::Task lock_task([] {
        bool holding = false;
        while (true) {
            const bool hold = lift_lock_enabled.load() && !liftMoveActive() &&
                              holdLiftAtMax();
            // Hand the motors back once, then leave them to their owner.
            if (holding && !hold && !liftMoveActive()) {
                slider_left.brake();
                slider_right.brake();
            }
            holding = hold;
            pros::delay(20);
        }
    });
}

bool moveLift(double targetDeg, std::uint32_t timeoutMs,
              std::atomic_bool* cancel) {
    constexpr double kP = 0.45;
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
        lift_move_seen_ms.store(pros::millis());
        lift_move_seen.store(true);
        if (cancel != nullptr && cancel->load())
            return fail("cancelled", lastCurrentDeg, lastVelocityDegPerSec);
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

        // Power floors. Down is the stronger one: the lift's rubber bands
        // pull it up.
        int power = std::clamp(static_cast<int>(errorDeg * kP), -127, 127);
        if (errorDeg > 0 && power < 110)
            power = 110;
        else if (errorDeg < 0 && power > -60)
            power = -60;

        slider_left.move(power);
        slider_right.move(power);
        pros::delay(kLoopDelayMs);
    }

    return fail("timeout", lastCurrentDeg, lastVelocityDegPerSec);
}

double lowerLiftToRest(std::atomic_bool* cancel) {
    // Full power: the lift's rubber bands pull hardest near the bottom, and
    // less than this left it partway down. It has stopped once the sensor
    // has moved less than kProgressDeg in kStoppedMs.
    constexpr int kDownPower = 127;
    constexpr double kRestToleranceDeg = 8.0, kProgressDeg = 3.0;
    constexpr std::uint32_t kStoppedMs = 300, kTimeoutMs = 6000;
    const std::uint32_t began = pros::millis();
    double current = lift_sensor.get_position() / 100.0;
    double progress_from = current;
    std::uint32_t progress_at = began;
    while (pros::millis() - began < kTimeoutMs &&
           current < lift_position::stage_0_deg - kRestToleranceDeg &&
           !(cancel != nullptr && cancel->load())) {
        lift_move_seen_ms.store(pros::millis());
        lift_move_seen.store(true);
        slider_left.move(kDownPower);
        slider_right.move(kDownPower);
        pros::delay(10);
        current = lift_sensor.get_position() / 100.0;
        const std::uint32_t now = pros::millis();
        if (std::abs(current - progress_from) >= kProgressDeg) {
            progress_from = current;
            progress_at = now;
        } else if (now - progress_at >= kStoppedMs) {
            break;
        }
    }
    slider_left.brake();
    slider_right.brake();
    std::printf("LIFT_SETTLED,current=%.1f,elapsed_ms=%lu\n", current,
                static_cast<unsigned long>(pros::millis() - began));
    return current;
}

namespace {
// Scheduling guards only; validate the minimum reliable pneumatic time and
// mechanism clearance on the assembled robot before shortening either one.
constexpr std::uint32_t kPneumaticSettleMs = 75;
constexpr float kGoalClearanceIn = 6.0f;
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
//   THREE_PIN_KEY_RESETS (default 0): one GPS reset at each key stop. The
//     one after clamping stack 2 is taken either way.
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
constexpr Point kStack1{20.1f, 36.79f};  // about one tile farther from the wall
// One tile beyond the goal, the same spacing as stack 1 (which is picked up
// reliably at 22.5 in from the goal). The route sketch agrees (x about 44).
constexpr Point kStack2{42.5f, 13.25f};
// Rotation center to the claw's grip when picking a stack up. The pickup now
// drives all the way to its stop point instead of stalling an inch short, so
// this is an inch longer than when it was first found.
constexpr float kReach = 8.0f;
// ...and how close the center is sent to the goal's center when scoring a
// stack. Smaller = deeper into the goal.
constexpr float kScoreReach = -1.0f;
// Stack 1 is clamped slightly off-center, so it is scored at a spot a little
// farther from the wall than the goal's center. It was an inch shallower
// than stack 2 while the pickup still overran the stack; with the stack's
// position corrected it needs the same depth.
constexpr Point kGoalForStack1{20.0f, 15.75f};
// Stack 2 comes into the goal along the line the preload did, from the other
// side. Its run ends a touch to the far side of that line, and its turn to
// the goal aims at the same y back where the preload's run started.
constexpr Point kGoalForStack2{kGoal.x, 14.75f};
constexpr Point kAimForStack2{0.0f, kGoalForStack2.y};
constexpr float kScoreReachStack1 = -1.0f;
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
// Lift height the preload is lowered to on the goal before the claw opens.
// moveLift() ends 8 degrees early, so the lift stops between -82 and -75.
constexpr double kPreloadSeatDeg = -75.0;
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
// and stops the chassis if the lift failed.
class LiftJob {
public:
    // A handle on whichever lift move is already running (or last ran).
    struct Running {};
    explicit LiftJob(Running) {}
    // lowerLiftToRest() from the preload height instead of a move to an
    // exact height; anything no higher than the preload height counts.
    struct Settle {};
    explicit LiftJob(Settle) {
        state() = 0;
        pros::Task([] {
            state() = lowerLiftToRest() >= kPreloadSeatDeg - 8.0 ? 1 : -1;
        });
    }
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
    // The reset on the clamped stack 2 is always on: by then odometry has
    // drifted too far for the last run into the goal.
    if (!kKeyResets && stop != kAtClamp2) {
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

// Mean wheel speed of the drive, either direction. Reads as moving if no
// motor answers.
constexpr double kStalledRpm = 12.0;
double drive_rpm() {
    double rpm = 0.0;
    int motors = 0;
    for (double v : left_motors.get_actual_velocity_all())
        rpm += std::abs(v), ++motors;
    for (double v : right_motors.get_actual_velocity_all())
        rpm += std::abs(v), ++motors;
    return motors ? rpm / motors : 2.0 * kStalledRpm;
}

// Wait for the current move to finish, ending it early if the drive has
// stopped turning: the robot is against the goal and pushing longer only
// wastes time. Returns true if it was ended that way.
bool wait_done_or_stalled() {
    constexpr std::uint32_t kStalledMs = 150, kLaunchMs = 300;
    const std::uint32_t began = pros::millis();
    std::uint32_t slow_since = 0;
    while (chassis.isInMotion()) {
        const std::uint32_t now = pros::millis();
        if (drive_rpm() > kStalledRpm || now - began < kLaunchMs)
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
           float reach = kScoreReach, const Point* aimAt = nullptr) {
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
    // during the pivot. `aimAt` replaces it with a fixed point.
    const Point aim = aimAt ? *aimAt : standoff(here, goal, -kGoalAimBeyondIn);
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

// The three-pin route. False if it stopped early because the lift failed.
bool run_three_pin() {
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
    // lift just far enough (kPreloadSeatDeg) to seat the pin.
    const Point preload{kGoal.x - kReach, kGoal.y};
    // Back out to the lane, turn, back in. Plain, unchained moves, and it
    // stays that way: handing these over without stopping (tried twice) and
    // a single curve (tried twice) both threw the rest of the route off.
    chassis.moveToPoint(0, kGoal.y, 1000, {.forwards = false}, false);
    chassis.turnToHeading(270, 800, {}, false);
    chassis.moveToPoint(preload.x, preload.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed}, false);
    // The lift must be all the way up before it is sent back down.
    if (!raise_preload.wait()) return false;
    {
        // GPS reset while the lift comes down.
        LiftJob seat_pin(kPreloadSeatDeg);
        fix_at(preload, kAtPreload);
        if (!seat_pin.wait()) return false;
    }
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);

    // Pull straight out of the goal to the start line (chained).
    Point at{0.0f, kGoal.y};
    chain_to(at, true, 1000);

    // STACK 1: the lift comes down as far as it will go now that the robot
    // is clear of the goal. Pick up, score one level up.
    LiftJob lower_for_stack1{LiftJob::Settle{}};
    Point seat{};
    PickLog picked1, picked2;
    if (!pick_up(at, seat, kStack1, picked1, &lower_for_stack1)) return false;
    ScoreLog scored1, scored2;
    if (!score(at, seat, lift_position::stage_1_deg,
               lift_position::stage_0_deg, scored1, kAtClamp1, true,
               kGoalForStack1, kScoreReachStack1))
        return false;

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
    if (!pick_up(at, seat, kStack2, picked2, &lower)) return false;
    // With the stack clamped, carry on along the same line only as far as
    // the goal's axis (y = kGoal.y), then turn along that axis.
    seat = {at.x + (kStack2.x - at.x) * (kGoal.y - at.y) / (kStack2.y - at.y),
            kGoal.y};
    // No reset after the last score: nothing is left to aim at.
    if (!score(at, seat, lift_position::stage_2_deg,
               lift_position::stage_1_deg, scored2, kAtClamp2, false,
               kGoalForStack2, kScoreReach, &kAimForStack2))
        return false;

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
    // GPS line: what the reset on the clamped stack 2 did ("moved" = odometry
    // was corrected) and how far GPS and odometry disagreed there.
    // "Grab" lines: how far short of the clamp spot the robot was when it
    // clamped, and how long the slow last stretch took (2.0 s = it ran out
    // of time before getting there).
    fieldviz::report("3 PIN  took %.1f s\n"
                     "Grab 1 short %.1f  %.1f s\n"
                     "Grab 2 short %.1f  %.1f s\n"
                     "S1 aim %+.1f short %.1f %s\n"
                     "S2 aim %+.1f short %.1f %s\n"
                     "Goal ref moved x%+.1f y%+.1f\n"
                     "GPS at grab 2: %s %.1f in",
                     (pros::millis() - started) / 1000.0,
                     picked1.short_in, picked1.creep_ms / 1000.0,
                     picked2.short_in, picked2.creep_ms / 1000.0,
                     scored1.turn_error_deg, scored1.short_in, blocked(scored1),
                     scored2.turn_error_deg, scored2.short_in, blocked(scored2),
                     scored1.reference_dx_in, scored1.reference_dy_in,
                     fix_log[kAtClamp2].outcome, fix_log[kAtClamp2].gap_in);
    std::printf("SCORE,1,%.1f,%.1f,%u\nSCORE,2,%.1f,%.1f,%u\n",
                scored1.turn_error_deg, scored1.short_in,
                static_cast<unsigned>(scored1.entry_ms),
                scored2.turn_error_deg, scored2.short_in,
                static_cast<unsigned>(scored2.entry_ms));
    return true;
}

void three_pin_auton() { run_three_pin(); }



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
// The right wall stack, from a recording of the approach driven with the
// controller (UP in driver control, see curve_capture): where the claw's
// grip ended. An earlier recording, pushed by hand, put the stack 6 in
// nearer in X and is not used. X is 1.5 in nearer than recorded (-19.2):
// with the approach sent deeper the robot ended that far past both stacks.
constexpr Point kWallStack{-17.7f, -8.6f};
// Heading the robot ends on, backed onto that stack (it faces away from the
// stack; the claw points at heading + 180). The recording came in at about
// 32 and twisted to 24 on the spot; 28 is the heading of the straight line
// from the recorded three-quarter point onto the stack, so the route arrives
// on it without a twist.
constexpr float kWallStackHeading = 28.0f;
// The approach from kStaging, claw first, in two motions fitted to the
// recorded path (within an inch and about 4 degrees of it in simulation):
// a drive at the recorded point 37% of the way along, which swings the robot
// about 16 degrees straight away, as the recording did, then a curve onto
// the stack. The earlier chain of three recorded points turned too little at
// the start (LemLib stops steering within 7.5 in of each point) and left
// the last curve to make it up, where the robot hung for half a second.
// Y is half an inch nearer the wall than recorded (11.7) for a slightly
// harder first swing: the mirrored run was just clipping the neutral goal.
constexpr Point kWallTurnIn{-3.4f, 11.2f};
constexpr float kWallCurveLead = 0.45f;
// The field's mirror line between the two goals, for the left wall stack.
constexpr float kMirrorX = 4.0f;
constexpr Point mirrored(Point p) { return {2.0f * kMirrorX - p.x, p.y}; }
// The curves are driven slower than the tuner's speed: one ends on a stack
// against the wall, the other carries a raised stack.
constexpr float kCurveSpeed = 70.0f;

// PART 2: the left wall stack onto the neutral (middle) goal, as the mirror
// image of part 1. The field is taken as symmetric: the goals are two tiles
// (48 in) apart with a wall stack behind each. The path file agrees once its
// 0.9 scale is taken out, but none of part 2 has been on the field.
constexpr bool kRunPartTwo = true;
constexpr float kNeutralGoalX = kGoalX + 48.0f;
constexpr Point kLeftWallStack = mirrored(kWallStack);
constexpr float kLeftWallStackHeading = -kWallStackHeading;
constexpr float kNeutralScoreHeading = 270.0f;  // claw toward +X
constexpr Point kNeutralScoreSpot{kNeutralGoalX - 8.75f, kLaneY};
// Mirror image of kStaging: the same 28 in from the neutral goal.
constexpr Point kNeutralStaging{kNeutralGoalX - 28.0f, kLaneY};

// Driven by the route, the approach stops short of both wall stacks (right
// direction, not deep enough), so it is sent this much farther in: found on
// the field, separately for each stack.
constexpr float kWallDeeperIn = 4.65f;      // right stack (part 1)
constexpr float kLeftWallDeeperIn = 5.0f;   // left stack (part 2)
// Hand-overs out of the curve with a stack and out of the goal: they end
// this far short and still rolling, and the next motion takes it from there.
constexpr float kLeaveExitIn = 3.0f;
// The run into a goal starts while the lift is still rising and holds this
// far short if the lift is not up by then.
constexpr float kGoalHoldIn = 6.0f;

// Where the robot's center is sent to take `stack` facing `headingDeg`: the
// claw's grip (kReach behind the center) on the stack, `deeperIn` deeper.
Point spot_for(Point stack, float headingDeg, float deeperIn) {
    const float h = headingDeg * static_cast<float>(M_PI) / 180.0f;
    const float reach = three_pin::kReach - deeperIn;
    return {stack.x + reach * std::sin(h), stack.y + reach * std::cos(h)};
}

// What one approach onto a wall stack did, measured alongside the motion
// without touching it: the numbers that would tell a clean pickup from a jam
// on the cups or an empty claw. Shown in the report; nothing acts on them
// yet.
constexpr int kWallApproachTimeoutMs = 1500;
struct WallLog {
    std::uint32_t approach_ms = 0;  // the curve onto the stack
    std::uint32_t stalled_ms = 0;   // of that, with the drive not turning
    // Odometry at the clamp against the spot it was sent to: still to go
    // along the approach, off to the robot's right of it, and heading.
    float short_in = 0.0f, right_in = 0.0f, heading_off_deg = 0.0f;
    // The lift's first kLiftClearDeg with whatever the claw holds: how long
    // it took and the mean current of both motors together.
    std::uint32_t lift_ms = 0;
    float lift_ma = 0.0f;
};

// Counts the time the drive is not turning, from its own task so the motion
// being watched runs exactly as it did.
std::atomic_bool stall_watch_on{false};
std::atomic<std::uint32_t> stall_watch_ms{0};
void watch_for_stall() {
    stall_watch_ms = 0;
    stall_watch_on = true;
    pros::Task([] {
        std::uint32_t last = pros::millis();
        while (stall_watch_on) {
            pros::delay(10);
            const std::uint32_t now = pros::millis();
            if (three_pin::drive_rpm() < three_pin::kStalledRpm)
                stall_watch_ms += now - last;
            last = now;
        }
    });
}

// From a staging point on the lane: back onto a wall stack along the recorded
// approach (through `via`, ending `deeperIn` past the recorded stop), clamp,
// start the lift to `liftDeg`, and once the stack is off the tiles curve back out nose-first toward `out` on `outHeading` (claw already
// toward the goal, so there is no pivot with a raised stack). The curve ends
// kLeaveExitIn short and still rolling: set_down() reverses out of it.
// The lift is still rising on return; set_down() holds short of the goal for
// it if it has to.
void take_wall_stack(Point stack, float headingDeg, Point via, float deeperIn,
                     float liftDeg, Point out, float outHeading,
                     WallLog& log) {
    using namespace three_pin;
    const Point spot = spot_for(stack, headingDeg, deeperIn);
    chassis.moveToPoint(via.x, via.y, 1000,
                        {.forwards = false, .maxSpeed = kCurveSpeed,
                         .minSpeed = kChainSpeed,
                         .earlyExitRange = kChainExitIn}, false);
    // The floor makes this end as the robot reaches the stack instead of
    // settling there, like the three-pin's pickups.
    const std::uint32_t approaching = pros::millis();
    watch_for_stall();
    chassis.moveToPose(spot.x, spot.y, headingDeg, kWallApproachTimeoutMs,
                       {.forwards = false,
                        .horizontalDrift = curve::tuned.drift,
                        .lead = kWallCurveLead, .maxSpeed = kCurveSpeed,
                        .minSpeed = kGentleFloor}, false);
    stall_watch_on = false;
    brake_drive();
    const lemlib::Pose clamped = chassis.getPose();
    claw_piston.set_value(true);
    log.approach_ms = pros::millis() - approaching;
    log.stalled_ms = stall_watch_ms;
    const float h = headingDeg * static_cast<float>(M_PI) / 180.0f;
    const float dx = clamped.x - spot.x, dy = clamped.y - spot.y;
    log.short_in = dx * std::sin(h) + dy * std::cos(h);
    log.right_in = dx * std::cos(h) - dy * std::sin(h);
    log.heading_off_deg = std::remainder(clamped.theta - headingDeg, 360.0f);
    pros::delay(kPneumaticSettleMs);

    const double lift_start = lift_sensor.get_position() / 100.0;
    LiftJob raise(liftDeg);
    const std::uint32_t lifting_since = pros::millis();
    double lift_ma = 0.0;
    int lift_samples = 0;
    while (lift_sensor.get_position() / 100.0 > lift_start - kLiftClearDeg &&
           pros::millis() - lifting_since < 400) {
        lift_ma += slider_left.get_current_draw() +
                   slider_right.get_current_draw();
        ++lift_samples;
        pros::delay(10);
    }
    log.lift_ms = pros::millis() - lifting_since;
    log.lift_ma = lift_samples ? lift_ma / lift_samples : 0.0;
    coast_drive();
    chassis.moveToPose(out.x, out.y, outHeading, 2500,
                       {.forwards = true,
                        .horizontalDrift = curve::tuned.drift,
                        .lead = curve::tuned.lead, .maxSpeed = kCurveSpeed,
                        .minSpeed = kChainSpeed,
                        .earlyExitRange = kLeaveExitIn}, false);
}

// Back straight down the lane onto `spot` while the lift finishes rising
// (holding kGoalHoldIn short if it is not up by then), lower the lift toward
// `lowerTo`, and open the claw once it has dropped a third of a level.
// Returns false if the lift failed. `scoring` is the odometry pose when the
// robot stopped.
bool set_down(Point spot, float lowerTo, lemlib::Pose& scoring) {
    using namespace three_pin;
    LiftJob raise{LiftJob::Running{}};
    // The floor keeps it from stopping short and waiting to be declared
    // settled, and the run ends at once if the robot is against the goal;
    // without either it sat at the goal for up to the whole timeout.
    const lemlib::MoveToPointParams entry{.forwards = false,
                                          .maxSpeed = kGoalSpeed,
                                          .minSpeed = kGentleFloor};
    const lemlib::Pose from = chassis.getPose();
    chassis.moveToPoint(spot.x, spot.y, 1500, entry, true);
    chassis.waitUntil(std::hypot(spot.x - from.x, spot.y - from.y) -
                      kGoalHoldIn);
    if (chassis.isInMotion() && !raise.done()) {
        chassis.cancelMotion();
        brake_drive();
        if (!raise.wait()) return false;
        coast_drive();
        chassis.moveToPoint(spot.x, spot.y, 1500, entry, true);
    }
    wait_done_or_stalled();
    if (!raise.wait()) return false;
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

namespace two_pin_far {
// The route on either side of the field. The close side (where the three-pin
// starts) is taken as the far side's mirror image in X about the start: its
// goal 20 in to +X, the middle goal 28 in to -X, a wall stack behind each.
// Nothing on the close side has been measured.
struct Side {
    bool close;
    Point at(Point p) const { return close ? Point{-p.x, p.y} : p; }
    float heading(float deg) const { return close ? 360.0f - deg : deg; }
    float wall_heading(float deg) const { return close ? -deg : deg; }
};

void run(Side side) {
    using namespace three_pin;
    const std::uint32_t started = pros::millis();
    const char* const title = side.close ? "2 PIN CLOSE" : "2 PIN FAR";
    const float score_heading = side.heading(kScoreHeading);
    const Point score_spot = side.at(kScoreSpot);
    const Point staging = side.at(kStaging);
    // Seconds into the route at each step, for the report: toggle hit,
    // preload released, then each stack back on the lane and released.
    enum { kToggle, kPreload, kStack1, kScore1, kStack2, kScore2, kMarks };
    float at_s[kMarks] = {};
    const auto mark = [&](int step) {
        at_s[step] = (pros::millis() - started) / 1000.0f;
    };
    set_lateral_kp(kLateralKp);
    coast_drive();
    // No GPS in this route: taking the start anchor can hold the robot for
    // up to 0.6 s before it moves, and nothing here uses it.
    gpsreset::set_auto_reset(false);
    chassis.setPose(0, 0, 180);

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
    mark(kToggle);

    // PRELOAD: back out to the lane, turn the claw to the goal, back in.
    // The move out to the lane is a plain settled one, as in the three-pin,
    // where handing these over without stopping threw the rest of the route
    // off.
    chassis.moveToPoint(kLane.x, kLane.y, 1000, {.forwards = false}, false);
    // The turn ends once it is within kGoalTurnExitDeg and moving slowly,
    // and the run in steers out the rest, as for the three-pin's scores.
    // The run in keeps a floor and ends at once if the robot is against the
    // goal, instead of waiting there to be declared settled.
    chassis.turnToHeading(score_heading, 800,
                          {.minSpeed = kGoalTurnFloor,
                           .earlyExitRange = kGoalTurnExitDeg}, false);
    chassis.moveToPoint(score_spot.x, score_spot.y, 1200,
                        {.forwards = false, .maxSpeed = kGoalSpeed,
                         .minSpeed = kGentleFloor}, true);
    wait_done_or_stalled();
    brake_drive();
    // Lower only as far as seats the pin, as in the three-pin.
    if (!raise_preload.wait()) return;
    if (!LiftJob(kPreloadSeatDeg).wait()) return;
    claw_piston.set_value(false);
    pros::delay(kPneumaticSettleMs);
    coast_drive();
    mark(kPreload);

    // Back out along the lane, front first, well clear of the goal; once
    // clear of it the lift comes the rest of the way down. No stop at the
    // end: the approach to the stack reverses straight out of it.
    chassis.moveToPoint(staging.x, staging.y, 1200,
                        {.forwards = true, .minSpeed = kChainSpeed,
                         .earlyExitRange = kChainExitIn}, true);
    chassis.waitUntil(kGoalClearanceIn);
    LiftJob lower_for_stack{LiftJob::Settle{}};
    chassis.waitUntilDone();
    if (!lower_for_stack.wait()) return;

    // FIRST WALL STACK (behind the alliance goal) onto that goal, one level
    // up.
    WallLog wall[2];
    lemlib::Pose scoring[2] = {{0, 0, 0}, {0, 0, 0}};
    const Point score_spots[2] = {score_spot, side.at(kNeutralScoreSpot)};
    const float score_headings[2] = {score_heading,
                                     side.heading(kNeutralScoreHeading)};
    // Shown again after every step with whatever is known by then (the rest
    // reads 0), so a route cut off by the clock still leaves its numbers.
    const auto show = [&] {
        char text[768];
        int used = std::snprintf(
            text, sizeof(text),
            "%s  took %.1f s\n"
            "At: toggle %.1f  preload %.1f\n"
            " stack %.1f  score %.1f    stack %.1f  score %.1f",
            title, (pros::millis() - started) / 1000.0, at_s[kToggle],
            at_s[kPreload], at_s[kStack1], at_s[kScore1], at_s[kStack2],
            at_s[kScore2]);
        for (int i = 0; i < 2 && used < static_cast<int>(sizeof(text)); ++i)
            used += std::snprintf(
                text + used, sizeof(text) - used,
                "\nWall %d: approach %lu ms, stalled %lu ms\n"
                " short %.1f in  right %+.1f in  %+.1f deg\n"
                " lift %lu ms  %.0f mA\n"
                " score off %.1f in  %+.1f deg",
                i + 1, static_cast<unsigned long>(wall[i].approach_ms),
                static_cast<unsigned long>(wall[i].stalled_ms),
                wall[i].short_in, wall[i].right_in, wall[i].heading_off_deg,
                static_cast<unsigned long>(wall[i].lift_ms), wall[i].lift_ma,
                at_s[i ? kScore2 : kScore1] == 0.0f
                    ? 0.0f
                    : std::hypot(scoring[i].x - score_spots[i].x,
                                 scoring[i].y - score_spots[i].y),
                at_s[i ? kScore2 : kScore1] == 0.0f
                    ? 0.0f
                    : std::remainder(scoring[i].theta - score_headings[i],
                                     360.0f));
        fieldviz::report_full(text);
    };
    const Point wall_stack = side.at(kWallStack);
    const float wall_heading = side.wall_heading(kWallStackHeading);
    take_wall_stack(wall_stack, wall_heading, side.at(kWallTurnIn),
                    kWallDeeperIn, lift_position::stage_1_deg, staging,
                    score_heading, wall[0]);
    mark(kStack1);
    show();
    if (!set_down(score_spot, lift_position::stage_0_deg, scoring[0])) return;
    mark(kScore1);
    show();

    if (!kRunPartTwo) {
        // Back out along the lane; the lift finishes coming down.
        chassis.moveToPoint(staging.x, staging.y, 1200, {.forwards = true},
                            false);
        show();
        return;
    }

    // PART 2 (kRunPartTwo): the second wall stack onto the middle goal, the
    // same approach as part 1, mirrored. Pull out to its staging point and
    // turn the claw round to lead, each handing over to the next without
    // settling; the approach re-aims. The lift must be down before it goes
    // in. The stack is carried at the middle goal's first level.
    const Point middle_staging = side.at(kNeutralStaging);
    const Point second_stack = side.at(kLeftWallStack);
    const float second_heading = side.wall_heading(kLeftWallStackHeading);
    chassis.moveToPoint(middle_staging.x, middle_staging.y, 1200,
                        {.forwards = true, .minSpeed = kChainSpeed,
                         .earlyExitRange = kLeaveExitIn}, false);
    const Point turn_in = side.at(mirrored(kWallTurnIn));
    turn_claw_to(turn_in);
    if (!LiftJob{LiftJob::Running{}}.wait()) return;
    take_wall_stack(second_stack, second_heading, turn_in, kLeftWallDeeperIn,
                    lift_position::middle_goal_stage_1_deg, middle_staging,
                    score_headings[1], wall[1]);
    mark(kStack2);
    show();
    if (!set_down(score_spots[1], lift_position::stage_0_deg, scoring[1]))
        return;
    mark(kScore2);
    // Leave the goal so nothing is touching the stack; the route ends as
    // soon as the claw is clear.
    chain_to(straight_ahead(8.0f), true, 800);
    show();
}
}  // namespace two_pin_far

void two_pin_far_auton() { two_pin_far::run({.close = false}); }
void two_pin_close_auton() { two_pin_far::run({.close = true}); }

// Driver-control recorder for the two-pin far's curve onto the wall stack
// (UP starts it, UP again ends it). Park the robot where the route stands
// after backing out of the goal (kStaging, nose away from the goal), start,
// drive the curve by hand, and end with the claw on the stack.
//
// Odometry is left alone: each sample is the raw LemLib pose plus the same
// pose in route coordinates, taking the start as kStaging at kScoreHeading.
// Samples go to the terminal as CURVE_CAP lines. The Brain shows the result
// over the whole screen: the end pose, where that puts the claw's grip, and
// the path as kWaypoints poses with the wheel speeds driven at each.
namespace curve_capture {
struct Sample {
    std::uint32_t t_ms;
    float x, y, heading;  // route coordinates
    float travelled_in;   // path length so far
    float left_rpm, right_rpm;
};
constexpr int kWaypoints = 8;
constexpr int kMaxSamples = 600;
constexpr float kSampleEveryIn = 0.5f, kSampleEveryDeg = 2.0f;
Sample samples[kMaxSamples];
int count = 0;
bool recording = false;
std::uint32_t began = 0;
lemlib::Pose origin(0, 0, 0);

Sample to_route(const lemlib::Pose& raw) {
    using namespace two_pin_far;
    // Rotate the raw displacement by the difference between the headings.
    const float turn = (kScoreHeading - origin.theta) *
                       static_cast<float>(M_PI) / 180.0f;
    const float dx = raw.x - origin.x, dy = raw.y - origin.y;
    return {pros::millis() - began,
            kStaging.x + dx * std::cos(turn) + dy * std::sin(turn),
            kStaging.y - dx * std::sin(turn) + dy * std::cos(turn),
            kScoreHeading + (raw.theta - origin.theta), 0.0f, 0.0f, 0.0f};
}

double mean_rpm(pros::MotorGroup& side) {
    double sum = 0.0;
    int motors = 0;
    for (double v : side.get_actual_velocity_all()) sum += v, ++motors;
    return motors ? sum / motors : 0.0;
}

void record(const lemlib::Pose& raw) {
    if (count >= kMaxSamples) return;
    Sample now = to_route(raw);
    now.left_rpm = static_cast<float>(mean_rpm(left_motors));
    now.right_rpm = static_cast<float>(mean_rpm(right_motors));
    if (count > 0) {
        const Sample& last = samples[count - 1];
        now.travelled_in = last.travelled_in +
                           std::hypot(now.x - last.x, now.y - last.y);
    }
    samples[count] = now;
    std::printf("CURVE_CAP,%d,%lu,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.0f,%.0f\n",
                count, static_cast<unsigned long>(now.t_ms), now.x, now.y,
                now.heading, raw.x, raw.y, raw.theta, now.left_rpm,
                now.right_rpm);
    ++count;
}

// The sample taken once `fraction` of the path had been driven.
const Sample& sample_at(float fraction) {
    const float target = samples[count - 1].travelled_in * fraction;
    for (int i = 0; i < count; ++i)
        if (samples[i].travelled_in >= target) return samples[i];
    return samples[count - 1];
}

void finish() {
    using namespace two_pin_far;
    record(chassis.getPose());
    const Sample& end = samples[count - 1];
    const float h = end.heading * static_cast<float>(M_PI) / 180.0f;
    // Where the claw's grip ended: the wall stack, if the claw is on it.
    const Point grip{end.x - three_pin::kReach * std::sin(h),
                     end.y - three_pin::kReach * std::cos(h)};
    // Where the turn was tightest: most heading change over kWindowIn of path.
    constexpr float kWindowIn = 3.0f;
    float sharpest = 0.0f;
    const Sample* sharpest_at = &samples[0];
    for (int i = 0, j = 0; i < count; ++i) {
        while (j < count - 1 &&
               samples[j].travelled_in < samples[i].travelled_in + kWindowIn)
            ++j;
        const float span = samples[j].travelled_in - samples[i].travelled_in;
        if (span < kWindowIn) break;
        const float rate = std::abs(samples[j].heading - samples[i].heading) / span;
        if (rate > sharpest) sharpest = rate, sharpest_at = &samples[i];
    }
    std::printf("CURVE_CAP_END,samples=%d,ms=%lu,path_in=%.1f,end=%.2f/%.2f/%.1f,"
                "grip=%.2f/%.2f,sharpest=%.1f@%.1f/%.1f\n",
                count, static_cast<unsigned long>(end.t_ms), end.travelled_in,
                end.x, end.y, end.heading, grip.x, grip.y, sharpest,
                sharpest_at->x, sharpest_at->y);
    std::fflush(stdout);

    static char text[768];
    int used = std::snprintf(
        text, sizeof(text),
        "CURVE CAPTURE  %.1f s, %.1f in, %d samples\n"
        "Start %.1f, %.1f at %.0f     End %.1f, %.1f at %.0f\n"
        "Claw grip %.1f, %.1f     route has %.1f, %.1f at %.0f\n"
        "Tightest %.1f deg/in from %.1f, %.1f\n"
        "Path  %%:  x, y  heading  left/right rpm",
        end.t_ms / 1000.0, end.travelled_in, count, kStaging.x, kStaging.y,
        kScoreHeading, end.x, end.y, end.heading, grip.x, grip.y, kWallStack.x,
        kWallStack.y, kWallStackHeading, sharpest, sharpest_at->x,
        sharpest_at->y);
    for (int i = 1; i <= kWaypoints; ++i) {
        if (used < 0 || used >= static_cast<int>(sizeof(text))) break;
        const Sample& at = sample_at(static_cast<float>(i) / kWaypoints);
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\n%d:  %.1f, %.1f   %.0f   %.0f/%.0f",
                              100 * i / kWaypoints, at.x, at.y, at.heading,
                              at.left_rpm, at.right_rpm);
    }
    fieldviz::report_full(text);
}
}  // namespace curve_capture

bool curveCaptureToggle() {
    using namespace curve_capture;
    if (recording) {
        recording = false;
        finish();
        return false;
    }
    count = 0;
    began = pros::millis();
    origin = chassis.getPose();
    recording = true;
    std::printf("CURVE_CAP_BEGIN,index,ms,x,y,heading,raw_x,raw_y,raw_heading,"
                "left_rpm,right_rpm\n");
    fieldviz::report("CURVE CAPTURE\n\nRecording...\nUP again to finish.");
    record(origin);
    return true;
}

void curveCaptureSample() {
    using namespace curve_capture;
    if (!recording || count == 0) return;
    const lemlib::Pose raw = chassis.getPose();
    const Sample now = to_route(raw);
    const Sample& last = samples[count - 1];
    if (std::hypot(now.x - last.x, now.y - last.y) >= kSampleEveryIn ||
        std::abs(now.heading - last.heading) >= kSampleEveryDeg)
        record(raw);
}

// SKILLS: the three-pin, which starts at the west wall and puts its preload
// and both field stacks on G2, then in this order:
//   1. two match loads from the lower red loader onto G2,
//   2. the west wall stack onto G2,
//   3. five match loads onto G3 (a pin, then four cups),
//   4. the bottom wall stack onto G3,
//   5. the bottom toggle, and park in the middle of the field.
// Each goal ends with a pin and five cups on it. Each wall stack comes after
// its goal's match loads, so a failed attempt costs nothing already scored. The clock decides how far down the list the
// run gets. Positions are the field coordinates of Auto_paths/SKILLS
// (inches from the field's center, +X east, +Y north, heading 0 = north),
// which that guide traced from screenshots. None of this has been on the
// field.
//
// Every run into the loader is the same last 14 in: the robot comes to
// kLineUp, straight in front of the loader, nose south, takes its position
// from the GPS, makes a quarter turn and backs in. G2 is due north of
// kLineUp (the pin that stood between them went onto G2 in the three-pin),
// so a G2 trip is a straight line each way and arrives already nose south.
//
// Position resets, so the 4 in claw meets the loader the same way each time:
//  - Before every run into the loader, at kLineUp, from the GPS. Same spot
//    and heading every time, so whatever error the GPS has is the same on
//    every pickup and comes out in kLoader. It cannot be later than that:
//    once the claw faces the loader the camera looks at the south wall from
//    about 10 in.
//  - At every goal: position from the goal itself, as in the three-pin.
//  - Against the loader, during the wait before the clamp: the GPS is read
//    but not used, to show in the report whether it sees anything there and
//    how far the quarter turn and run in moved the robot off.
//
// The toggle is hit nose first with the lift raised, as at the start of the
// three-pin.
//
// The claw end of the chassis has a rounded cut-out that a goal or a stack
// sits in, so the robot goes several inches deeper onto one than its 16 in
// length suggests. Runs into a goal and the loader are therefore sent past
// the contact point and end when the drive stops; the wall stacks use the
// depth the two-pin's needed.
namespace skills_route {
using three_pin::Point;

// The three-pin's frame is the field's turned a quarter round: it starts
// facing the west wall on its heading 180, so its +Y is east and its +X is
// south, and it takes its position from G2 each time it scores there, which
// pins its goal (20, 15) to G2.
constexpr Point kG2Field{-48.0f, -24.0f};
constexpr Point on_field(Point field) {
    return {kG2Field.y + three_pin::kGoal.x - field.y,
            field.x - kG2Field.x + three_pin::kGoal.y};
}
constexpr float field_heading(float deg) { return deg - 90.0f; }

constexpr Point kG2 = on_field(kG2Field);
constexpr Point kG3 = on_field({-24.0f, -48.0f});
static_assert(kG2.x == three_pin::kGoal.x && kG2.y == three_pin::kGoal.y);
// The guide stops the robot's center at x = -62 in the loader. It is sent
// 4 in past that and the run ends when the robot is against the loader.
constexpr Point kLoader = on_field({-66.0f, -59.0f});
// Parked against the loader this long before the claw closes.
constexpr std::uint32_t kLoaderWaitMs = 500;
// Longest a GPS reset may hold the robot; a good one takes about 250 ms
// from the moment the robot is still.
constexpr std::uint32_t kResetWaitMs = 450;
// Where every run into the loader starts: straight in front of it and due
// south of G2.
constexpr Point kLineUp = on_field({-48.0f, -59.0f});
// Straight out of G2 to the south, far enough for the claw to be clear.
constexpr Point kG2Out = on_field({-48.0f, -44.0f});
// G3 trip: east along the south lane to here, claw to the goal, and back out
// to here.
constexpr Point kG3Turn = on_field({-37.0f, -58.5f});
// Bottom toggle: the lane point under it and the point the nose is driven
// at, 2 in past where the robot meets the wall. X is as far along the bar
// as the three-pin starts along the west one (the guide has -12, its end).
constexpr Point kToggleLane = on_field({-6.0f, -59.5f});
constexpr Point kToggleHit = on_field({-6.0f, -66.0f});
// Park: the middle of the field, inside the four grey goals. In from the
// south-west between the goals at (-24, 0) and (0, -24), stopping with the
// nose just short of the pin at the center. A first guess: where the robot
// has to end to count as parked has not been checked.
constexpr Point kParkEntry = on_field({-15.0f, -28.0f});
constexpr Point kPark = on_field({-11.0f, -8.0f});

// A wall stack: where it stands, the heading the robot ends on backed onto
// it, where the robot turns its claw to lead, the point it backs through,
// and the pose it curves back out to with the stack, which is also where it
// turns the claw to the goal from and pulls back out to. The guide's poses,
// untuned: the two-pin's wall stacks each took several field sessions.
struct WallPin {
    Point stack;
    float heading;
    Point stage, via, out;
    float out_heading;
};
constexpr WallPin kWallPin2{on_field({-72.0f, -24.0f}), field_heading(90.0f),
                            on_field({-60.0f, -46.0f}),
                            on_field({-60.0f, -38.0f}),
                            on_field({-60.0f, -42.0f}), field_heading(180.0f)};
constexpr WallPin kWallPin3{on_field({-24.0f, -72.0f}), field_heading(0.0f),
                            on_field({-44.0f, -59.0f}),
                            on_field({-38.0f, -59.5f}),
                            on_field({-42.0f, -59.0f}), field_heading(270.0f)};

// How much of each step the route attempts. A step is skipped unless the
// time it is expected to take, plus the toggle and park, is still left of
// the minute, or if its goal is already as high as the robot can reach.
// The three-pin leaves G2 holding three: its preload pin and two cups.
constexpr int kOnG2AfterThreePin = 3;
constexpr int kG2Loads = 2;
constexpr int kG3Loads = 5;
constexpr bool kWallPins = true;
constexpr bool kToggleAndPark = true;
constexpr int kMostOnGoal = 6;  // a pin and five cups
// Expected seconds for each step: estimates from distances and the two-pin's
// measured pace, to be replaced by the times the report shows.
constexpr float kRunS = 60.0f;
constexpr float kG2TripS = 7.5f;
constexpr float kG3TripS = 8.0f;
constexpr float kWallPin2S = 8.5f;
constexpr float kWallPin3S = 8.0f;
constexpr float kEndgameS = 7.0f;

// How close the robot's center is sent to a goal's center. It cannot get
// nearer than three_pin::kReach, so the run in ends against the goal even
// with odometry a few inches out.
constexpr float kScoreReachIn = 3.0f;
// Lift height a match load is carried at, the same as the preload.
constexpr double kCarryDeg = lift_position::matchload - 125;
constexpr float kStageStep =
    lift_position::stage_2_deg - lift_position::stage_1_deg;
// moveLift() has to arrive within 8 degrees of its target, so it is not sent
// to the very top; the lift lock holds it there from here.
constexpr float kLiftTopDeg = 0.97f * lift_position::max_height_deg;

// Where the lift and arm carry the stack that goes onto a goal already
// holding 0, 1, ... 5.
//
// PLACEHOLDERS. The first two are the three-pin's (lift alone). From the
// third up the arm is raised, which is worth about one stage, so the lift
// stops a stage lower plus 15% of a stage; the top two are simply the top of
// the lift and have never been tried. Replace them with heights read on the
// robot: X in driver control lists the lift and arm readings on the Brain.
struct Level {
    float lift_deg;
    float arm_deg;
};
constexpr Level kLevels[kMostOnGoal] = {
    // The pin on an empty goal; it is then lowered to kPreloadSeatDeg.
    {static_cast<float>(kCarryDeg), arm_position::rest_deg},
    {lift_position::stage_1_deg, arm_position::rest_deg},
    {lift_position::stage_1_deg + 0.15f * kStageStep, arm_position::max_deg},
    {lift_position::stage_2_deg + 0.15f * kStageStep, arm_position::max_deg},
    {kLiftTopDeg, arm_position::max_deg},
    {kLiftTopDeg, arm_position::max_deg},
};
constexpr bool with_arm(int held) {
    return kLevels[held].arm_deg > arm_position::rest_deg;
}

// An arm move running alongside the drive, like LiftJob. moveArm() has no
// time limit and can sit just short of its target, so each move is given one
// here and counts as done when it runs out.
class ArmJob {
public:
    explicit ArmJob(double targetDeg, std::uint32_t limitMs = 1500) {
        end();
        cancelled() = false;
        running() = true;
        deadline() = pros::millis() + limitMs;
        pros::Task([targetDeg] {
            moveArm(targetDeg, &cancelled());
            running() = false;
        });
    }
    static bool done() {
        if (running() && pros::millis() >= deadline()) cancelled() = true;
        return !running();
    }
    static void wait() {
        while (!done()) pros::delay(10);
    }
    static void end() {
        cancelled() = true;
        while (running()) pros::delay(5);
    }

private:
    static std::atomic_bool& cancelled() {
        static std::atomic_bool value{false};
        return value;
    }
    static std::atomic_bool& running() {
        static std::atomic_bool value{false};
        return value;
    }
    static std::atomic<std::uint32_t>& deadline() {
        static std::atomic<std::uint32_t> value{0};
        return value;
    }
};

// Turn the nose toward `target`, handing straight over to the drive. For the
// corners a single drive would swing wide on, next to a goal or the cups.
void turn_nose_to(Point target) {
    using namespace three_pin;
    chassis.turnToPoint(target.x, target.y, 900,
                        {.forwards = true, .minSpeed = kTurnChainSpeed,
                         .earlyExitRange = kTurnChainExitDeg}, false);
}

// GPS readings taken at one kind of stop, for the report.
struct FixTally {
    int tried = 0, read = 0;
    double last_gap_in = 0.0;  // GPS against odometry at the last one read
};

// One GPS reading with the robot parked at `expected`. With `apply`,
// odometry X/Y is set from it however small the gap (gpsreset's own limits
// still hold: within 6 in of `expected` and 8 in of odometry, or odometry is
// kept). Without, it is only measured.
void gps_fix(Point expected, bool apply, std::uint32_t waitMs,
             FixTally& tally) {
    const gpsreset::Correction fix = gpsreset::checkpoint(
        expected.x, expected.y, 6.0, waitMs, apply ? 0.0 : 100.0);
    ++tally.tried;
    // Counted: a reset that moved odometry, or a measurement that got a
    // reading at all (whatever gpsreset then made of it).
    const bool read = fix.status == gpsreset::Fix::APPLIED ||
                      (!apply && fix.samples > 0 &&
                       (fix.status == gpsreset::Fix::ALREADY_ALIGNED ||
                        fix.status == gpsreset::Fix::OFF_TARGET ||
                        fix.status == gpsreset::Fix::TOO_LARGE));
    if (read) {
        ++tally.read;
        tally.last_gap_in = fix.gap_in;
    }
}

// Nose first to `to` from wherever the robot is, turning to face it first.
// Nothing if it is already there.
void go_to(Point to) {
    const lemlib::Pose pose = chassis.getPose();
    if (std::hypot(to.x - pose.x, to.y - pose.y) < 3.0f) return;
    turn_nose_to(to);
    three_pin::chain_to(to, true, 1500);
}

// With the claw empty: to kLineUp, square up nose south, reset position from
// the GPS, turn the claw to the loader, back in until the robot is against
// it, wait kLoaderWaitMs, clamp, and lift the pin to carrying height. The
// lift (and the arm, after a stack set down from it) must be back down
// before the robot backs in.
bool take_load(FixTally& lined_up, FixTally& at_loader) {
    using namespace three_pin;
    go_to(kLineUp);
    chassis.turnToHeading(field_heading(180.0f), 700,
                          {.minSpeed = kTurnChainSpeed,
                           .earlyExitRange = kTurnChainExitDeg}, false);
    brake_drive();
    gps_fix(kLineUp, true, kResetWaitMs, lined_up);
    coast_drive();
    turn_claw_to(kLoader);
    ArmJob::wait();
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    chassis.moveToPoint(kLoader.x, kLoader.y, 1800,
                        {.forwards = false, .minSpeed = kGentleFloor}, true);
    wait_done_or_stalled();
    brake_drive();
    const std::uint32_t parked = pros::millis();
    const lemlib::Pose stopped = chassis.getPose();
    gps_fix({stopped.x, stopped.y}, false, kLoaderWaitMs - 100, at_loader);
    while (pros::millis() - parked < kLoaderWaitMs) pros::delay(10);
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    const double lift_start = lift_sensor.get_position() / 100.0;
    LiftJob raise(kCarryDeg);
    const std::uint32_t lifting_since = pros::millis();
    while (lift_sensor.get_position() / 100.0 > lift_start - kLiftClearDeg &&
           pros::millis() - lifting_since < 400)
        pros::delay(10);
    coast_drive();
    return true;
}

// Start the lift and arm toward the height the stack is carried at for a
// goal already holding `held`, so they are up by the time the robot gets
// there. False if the lift move before it failed.
bool raise_for(int held) {
    using three_pin::LiftJob;
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    LiftJob raise(kLevels[held].lift_deg);
    if (with_arm(held)) ArmJob arm(kLevels[held].arm_deg);
    return true;
}

// Out of the loader, nose first, to `turn`, raising the lift and arm for the
// goal once the pin is clear of the loader. False if the lift failed.
bool carry_to(Point turn, int held) {
    three_pin::chain_to(turn, true, 1500, true);
    chassis.waitUntil(kGoalClearanceIn);
    const bool raising = raise_for(held);
    chassis.waitUntilDone();
    return raising;
}

// From where the robot stands with a stack in the claw and the lift and arm
// already on their way up (raise_for): turn the claw to `goal`, which
// already holds `held`, back in until the robot is against the goal
// (holding short if the lift or arm is not up yet), let go, and take
// position from the goal as the three-pin does. False if the lift failed.
// The robot may leave as soon as this returns: the claw is open.
bool put_on(Point goal, int held) {
    using namespace three_pin;
    LiftJob raise{LiftJob::Running{}};
    chassis.turnToPoint(goal.x, goal.y, 1100,
                        {.forwards = false, .minSpeed = kGoalTurnFloor,
                         .earlyExitRange = kGoalTurnExitDeg}, false);
    const lemlib::Pose turned = chassis.getPose();
    const Point here{turned.x, turned.y};
    const Point stop = standoff(here, goal, kScoreReachIn);
    const lemlib::MoveToPointParams entry{.forwards = false,
                                          .maxSpeed = kGoalSpeed,
                                          .minSpeed = kGentleFloor};
    chassis.moveToPoint(stop.x, stop.y, 1500, entry, true);
    // The robot meets the goal kReach out, before `stop`; it holds
    // kGoalStandbyIn short of that.
    chassis.waitUntil(std::hypot(stop.x - here.x, stop.y - here.y) -
                      (kReach - kScoreReachIn) - kGoalStandbyIn);
    if (chassis.isInMotion() && !(raise.done() && ArmJob::done())) {
        chassis.cancelMotion();
        brake_drive();
        if (!raise.wait()) return false;
        ArmJob::wait();
        coast_drive();
        chassis.moveToPoint(stop.x, stop.y, 1500, entry, true);
    }
    wait_done_or_stalled();
    if (!raise.wait()) return false;
    ArmJob::wait();
    brake_drive();

    if (with_arm(held)) {
        // RIGHT in driver control: the arm comes down, the claw lets go
        // halfway there, and the lift starts down. Unlike the macro, the claw
        // opens even if the arm stops short of halfway: a route cannot wait.
        const double release_within =
            std::abs(claw_sensor.get_position() / 100.0 -
                     arm_position::release_deg) / 2.0;
        ArmJob lower_arm(arm_position::release_deg);
        while (!ArmJob::done() &&
               std::abs(claw_sensor.get_position() / 100.0 -
                        arm_position::release_deg) > release_within)
            pros::delay(10);
        claw_piston.set_value(false);
        pros::delay(arm_position::release_settle_ms);
        LiftJob lower(lift_position::matchload);
    } else if (held == 0) {
        // On an empty goal: lower only as far as seats the pin.
        if (!LiftJob(kPreloadSeatDeg).wait()) return false;
        claw_piston.set_value(false);
        pros::delay(kPneumaticSettleMs);
    } else {
        // Set the stack down, opening the claw a third of a level down; the
        // lift carries on to the loader's height.
        const double scored_at = lift_sensor.get_position() / 100.0;
        LiftJob lower(lift_position::matchload);
        const std::uint32_t lowering_since = pros::millis();
        while (!lower.done() &&
               lift_sensor.get_position() / 100.0 < scored_at + kSeatDropDeg &&
               pros::millis() - lowering_since < 800)
            pros::delay(10);
        claw_piston.set_value(false);
        pros::delay(kPneumaticSettleMs);
    }
    // The claw's grip is on the goal's center, kReach from the robot's.
    const lemlib::Pose pressed = chassis.getPose();
    const float heading = pressed.theta * static_cast<float>(M_PI) / 180.0f;
    chassis.setPose(goal.x + kReach * std::sin(heading),
                    goal.y + kReach * std::cos(heading), pressed.theta);
    coast_drive();
    return true;
}

// Forward out of a goal to `to`. Once the claw is clear, the lift and arm go
// back to where the loader needs them (setting a stack down has already
// started the lift there for everything but the first on a goal).
void leave_to(Point to, int held) {
    three_pin::chain_to(to, true, 1500, true);
    chassis.waitUntil(kGoalClearanceIn);
    if (held == 0) three_pin::LiftJob to_loader(lift_position::matchload);
    if (with_arm(held)) ArmJob to_rest(arm_position::rest_deg);
    chassis.waitUntilDone();
}

// With the claw empty: to the wall stack's staging point, back onto the
// stack and curve back out with it, the lift at the first level.
bool take_wall_pin(const WallPin& pin, two_pin_far::WallLog& log) {
    using namespace three_pin;
    go_to(pin.stage);
    ArmJob::wait();
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    LiftJob down{LiftJob::Settle{}};
    turn_claw_to(pin.via);
    if (!down.wait()) return false;
    two_pin_far::take_wall_stack(pin.stack, pin.heading, pin.via,
                                 two_pin_far::kWallDeeperIn,
                                 lift_position::stage_1_deg, pin.out,
                                 pin.out_heading, log);
    return true;
}

void run() {
    using namespace three_pin;
    const std::uint32_t started = pros::millis();
    const auto now_s = [&] { return (pros::millis() - started) / 1000.0f; };
    if (!run_three_pin()) return;
    const float three_pin_s = now_s();
    set_lateral_kp(kLateralKp);
    coast_drive();

    constexpr int kLoads = kG2Loads + kG3Loads;
    float toggle_s = 0.0f, park_s = 0.0f;
    float trip_s[kLoads] = {};
    int trips = 0, on_g2 = kOnG2AfterThreePin, on_g3 = 0;
    FixTally lined_up, at_loader;
    const char* wall_note[2] = {"-", "-"};
    float wall_s[2] = {};
    two_pin_far::WallLog wall[2];
    // Shown again after every step with whatever is known by then, so a run
    // cut off by the clock still leaves its numbers.
    const auto show = [&] {
        char text[768];
        int used = std::snprintf(
            text, sizeof(text),
            "SKILLS  %.1f s   (3 pin %.1f)\n"
            "Resets before loader: %d of %d, last moved %.1f in\n"
            "GPS at loader: read %d of %d, last %.1f in off\n"
            "On G2: %d   on G3: %d\nTrips (s):",
            now_s(), three_pin_s, lined_up.read,
            lined_up.tried, lined_up.last_gap_in, at_loader.read,
            at_loader.tried, at_loader.last_gap_in, on_g2, on_g3);
        for (int i = 0; i < trips && used < static_cast<int>(sizeof(text)); ++i)
            used += std::snprintf(text + used, sizeof(text) - used, " %.1f",
                                  trip_s[i]);
        for (int i = 0; i < 2 && used < static_cast<int>(sizeof(text)); ++i)
            used += std::snprintf(
                text + used, sizeof(text) - used,
                "\nWall pin %d: %s %.1f s  approach %lu ms, stalled %lu ms",
                i + 2, wall_note[i], wall_s[i],
                static_cast<unsigned long>(wall[i].approach_ms),
                static_cast<unsigned long>(wall[i].stalled_ms));
        if (used < static_cast<int>(sizeof(text)))
            std::snprintf(text + used, sizeof(text) - used,
                          "\nToggle at %.1f s   parked at %.1f s", toggle_s,
                          park_s);
        fieldviz::report_full(text);
    };
    // True if `seconds` more, and the toggle and park after it, still fit.
    const auto fits = [&](float seconds) {
        return now_s() + seconds + (kToggleAndPark ? kEndgameS : 0.0f) <= kRunS;
    };

    // One match load: from the loader nose first to `turn`, onto `goal`, and
    // straight back out to `out`. False if the lift failed.
    const auto load = [&](Point goal, Point turn, Point out, int& on) {
        const float began = now_s();
        if (!take_load(lined_up, at_loader)) return false;
        if (!carry_to(turn, on)) return false;
        if (!put_on(goal, on)) return false;
        leave_to(out, on);
        ++on;
        trip_s[trips++] = now_s() - began;
        show();
        return true;
    };
    // One wall stack onto the goal beside it, if the goal has room and the
    // clock allows.
    const auto wall_pin = [&](const WallPin& pin, Point goal, int& on,
                              float seconds, int slot) {
        if (!kWallPins) return true;
        if (on >= kMostOnGoal) wall_note[slot] = "goal full";
        else if (!fits(seconds)) wall_note[slot] = "no time";
        else {
            const float began = now_s();
            if (!take_wall_pin(pin, wall[slot])) return false;
            if (!raise_for(on)) return false;
            if (!put_on(goal, on)) return false;
            leave_to(pin.out, on);
            ++on;
            wall_note[slot] = "done";
            wall_s[slot] = now_s() - began;
        }
        show();
        return true;
    };

    // The three-pin ends 8 in south of G2, nose south and rolling, on the
    // line to kLineUp. The lift comes down to the loader's height.
    if (!LiftJob{LiftJob::Running{}}.wait()) return;
    LiftJob to_loader(lift_position::matchload);

    // 1. MATCH LOADS ONTO G2: straight north from kLineUp and straight back.
    for (int i = 0; i < kG2Loads; ++i) {
        if (on_g2 >= kMostOnGoal || !fits(kG2TripS)) break;
        if (!load(kG2, kLineUp, kG2Out, on_g2)) return;
    }
    // 2. WEST WALL STACK ONTO G2, taken from the south-west.
    if (!wall_pin(kWallPin2, kG2, on_g2, kWallPin2S, 0)) return;
    // 3. MATCH LOADS ONTO G3.
    for (int i = 0; i < kG3Loads; ++i) {
        if (on_g3 >= kMostOnGoal || !fits(kG3TripS)) break;
        if (!load(kG3, kG3Turn, kG3Turn, on_g3)) return;
    }
    // 4. BOTTOM WALL STACK ONTO G3.
    if (!wall_pin(kWallPin3, kG3, on_g3, kWallPin3S, 1)) return;

    // 5. TOGGLE AND PARK: east along the south lane under G3, nose into the
    // bottom wall with the lift raised (the toggle cannot be reached with it
    // low), back out, north past G3 into the middle of the field.
    if (kToggleAndPark) {
        if (!LiftJob{LiftJob::Running{}}.wait()) return;
        LiftJob for_toggle(kCarryDeg);
        go_to(kG3Turn);
        turn_nose_to(kToggleLane);
        chain_to(kToggleLane, true, 2500);
        turn_nose_to(kToggleHit);
        if (!for_toggle.wait()) return;
        clamp_piston.set_value(true);
        chassis.moveToPoint(kToggleHit.x, kToggleHit.y, 900, {.forwards = true},
                            true);
        wait_done_or_stalled();
        toggle_s = now_s();
        chain_to(kToggleLane, false, 800);
        turn_nose_to(kParkEntry);
        chain_to(kParkEntry, true, 2500);
        chassis.moveToPoint(kPark.x, kPark.y, 2000, {.forwards = true}, true);
        wait_done_or_stalled();
        brake_drive();
        park_s = now_s();
    }
    show();
}
}  // namespace skills_route

void skills() { skills_route::run(); }

// X in driver control: note where the lift and arm are now. The list is
// shown over the whole Brain screen, for setting skills_route::kLevels.
namespace height_notes {
constexpr int kMost = 10;
struct Note {
    float lift_deg, arm_deg;
};
Note notes[kMost];
int count = 0;
}  // namespace height_notes

int recordHeights() {
    using namespace height_notes;
    if (count >= kMost) count = 0;
    notes[count++] = {lift_sensor.get_position() / 100.0f,
                      claw_sensor.get_position() / 100.0f};
    char text[768];
    int used = std::snprintf(text, sizeof(text),
                             "HEIGHTS  (X adds one; starts over after %d)\n"
                             " #     lift      arm",
                             kMost);
    for (int i = 0; i < count && used < static_cast<int>(sizeof(text)); ++i)
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\n%2d   %7.1f   %6.1f", i + 1,
                              notes[i].lift_deg, notes[i].arm_deg);
    fieldviz::report_full(text);
    std::printf("HEIGHT_NOTE,%d,%.1f,%.1f\n", count,
                notes[count - 1].lift_deg, notes[count - 1].arm_deg);
    return count;
}
