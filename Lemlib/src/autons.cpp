#include "autons.hpp"
#include "curve_motion.hpp"
#include "main.h"
#include "lemlib/pid.hpp"
#include "gps_reset/gps_reset.hpp"
#include "field_display_status.hpp"
#include "skills_control.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

// fastUp (skills only): a stiffer gain going up, so the lift stays at full
// power until about 125 degrees from the target instead of about 280. The
// rubber bands help it up and the move ends the moment it is within 8
// degrees, with the motors on HOLD, so it does not hunt. The match routes
// keep the gain they were tuned with.
bool moveLift(double targetDeg, std::uint32_t timeoutMs,
              std::atomic_bool* cancel, bool fastUp) {
    constexpr double kP = 0.45;
    constexpr double kFastUpP = 1.0;
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
        const double gain = fastUp && errorDeg < 0 ? kFastUpP : kP;
        int power = std::clamp(static_cast<int>(errorDeg * gain), -127, 127);
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
    const auto read_position = [] {
        const std::int32_t raw = lift_sensor.get_position();
        return raw == PROS_ERR ? NAN : raw / 100.0;
    };
    double current = read_position();
    double progress_from = current;
    std::uint32_t progress_at = began;
    while (std::isfinite(current) && pros::millis() - began < kTimeoutMs &&
           current < lift_position::stage_0_deg - kRestToleranceDeg &&
           !(cancel != nullptr && cancel->load())) {
        lift_move_seen_ms.store(pros::millis());
        lift_move_seen.store(true);
        slider_left.move(kDownPower);
        slider_right.move(kDownPower);
        pros::delay(10);
        current = read_position();
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
    if (!std::isfinite(current) || (cancel != nullptr && cancel->load())) {
        std::printf("LIFT_SETTLE_FAIL,%s\n",
                    std::isfinite(current) ? "cancelled" : "sensor");
        return NAN;
    }
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

namespace {
skills_control::ArmOutcome move_arm_bounded(double target,
                                           std::atomic_bool* cancel,
                                           std::uint32_t timeout_ms) {
    struct ArmIO {
        std::atomic_bool* cancel;
        std::uint8_t mode;
        std::uint32_t now() const { return pros::millis(); }
        void delay(std::uint32_t ms) const { pros::delay(ms); }
        double position_deg() const {
            const std::int32_t raw = claw_sensor.get_position();
            return raw == PROS_ERR ? NAN : raw / 100.0;
        }
        bool cancelled() const {
            return (cancel != nullptr && cancel->load()) ||
                   pros::competition::get_status() != mode;
        }
        void command(int power) const { claw_arm.move(power); }
        void brake() const { claw_arm.brake(); }
    } io{cancel, pros::competition::get_status()};
    return skills_control::run_arm(target, timeout_ms, io);
}
}  // namespace

void moveArm(double targetDeg, std::atomic_bool* cancel) {
    // Driver macros retain their existing power profile and cancel ownership.
    (void)move_arm_bounded(targetDeg, cancel, 0);
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
// An inch farther out than 42.5: the claw closed just short of it.
constexpr Point kStack2{43.5f, 13.25f};
// The claw still closed a hair short of stack 2 (odometry 0.5 in short of
// the clamp spot), so its last stretch goes this much farther into it.
constexpr float kStack2DeeperIn = 0.3f;
// Rotation center to the claw's grip when picking a stack up. The pickup now
// drives all the way to its stop point instead of stalling an inch short, so
// this is an inch longer than when it was first found.
constexpr float kReach = 8.0f;
// ...and how close the center is sent to the goal's center when scoring a
// stack. Smaller = deeper into the goal. Past the center on purpose: the
// robot ends on the stall against the goal. Sending it only to the contact
// point (7 in) left every score after the first short, and the goal
// reference then put odometry inches off for the rest of the route.
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
constexpr float kScoreReachStack1 = kScoreReach;
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
        cancelled() = false;
        state() = 0;
        pros::Task([] {
            const double at = lowerLiftToRest(&cancelled());
            state() = std::isfinite(at) && at >= kPreloadSeatDeg - 8.0 ? 1 : -1;
        });
    }
    explicit LiftJob(double targetDeg, bool fastUp = false) {
        cancelled() = false;
        state() = 0;
        pros::Task([targetDeg, fastUp] {
            state() = moveLift(targetDeg, 6000, &cancelled(), fastUp) ? 1 : -1;
        });
    }
    bool done() const { return state() != 0; }
    bool succeeded() const { return state() == 1; }
    bool failed() const { return state() == -1; }
    // Used when Skills aborts. Join before another worker may own the motors.
    static void end() {
        cancelled() = true;
        while (state() == 0) pros::delay(10);
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
    static std::atomic_bool& cancelled() {
        static std::atomic_bool value{false};
        return value;
    }
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
    for (double v : left_motors.get_actual_velocity_all()) {
        if (!std::isfinite(v)) return 2.0 * kStalledRpm;
        rpm += std::abs(v), ++motors;
    }
    for (double v : right_motors.get_actual_velocity_all()) {
        if (!std::isfinite(v)) return 2.0 * kStalledRpm;
        rpm += std::abs(v), ++motors;
    }
    return motors ? rpm / motors : 2.0 * kStalledRpm;
}

// Wait for the current move to finish, ending it early if the drive has
// stopped turning: the robot is against the goal and pushing longer only
// wastes time. Returns true if it was ended that way. Delayed callers pass
// the start of the active motion, so waiting for the lift does not buy that
// same motion another launch grace. A restarted motion needs a new timestamp.
bool wait_done_or_stalled(std::uint32_t motion_started = pros::millis()) {
    constexpr std::uint32_t kStalledMs = 150, kLaunchMs = 300;
    std::uint32_t slow_since = 0;
    while (chassis.isInMotion()) {
        const std::uint32_t now = pros::millis();
        if (drive_rpm() > kStalledRpm || now - motion_started < kLaunchMs)
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
             LiftJob* lowering = nullptr, float seatIn = kSeatIn,
             float deeperIn = 0.0f) {
    const Point gentle_from = standoff(at, stack, kReach + kGentleZoneIn);
    const Point stop = standoff(at, stack, kReach - deeperIn);
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
// Brake and wait (at most `ms`) until the drive has all but stopped.
void stop_rolling(std::uint32_t ms) {
    brake_drive();
    const std::uint32_t began = pros::millis();
    while (drive_rpm() > 15.0 && pros::millis() - began < ms) pros::delay(10);
}

bool score(Point& at, Point seat, float liftDeg, float lowerTo, ScoreLog& log,
           KeyStop clampStop, bool fixAfter, Point goal = kGoal,
           float reach = kScoreReach, const Point* aimAt = nullptr,
           bool stopAroundTurn = false) {
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
    // brake_drive() followed at once by coast_drive() does not stop anything:
    // the robot slid into the turn at the seat move's floor (45). With
    // stopAroundTurn it really stops first (at most 0.15 s), and again after
    // the turn, which otherwise ends 2 degrees early still rotating and,
    // with the stack raised, coasts past its aim before the run in.
    if (stopAroundTurn) stop_rolling(150);
    else brake_drive();
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
    if (stopAroundTurn) {
        stop_rolling(150);
        coast_drive();
    }
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
    if (!pick_up(at, seat, kStack2, picked2, &lower, kSeatIn, kStack2DeeperIn))
        return false;
    // With the stack clamped, carry on along the same line only as far as
    // the goal's axis (y = kGoal.y), then turn along that axis.
    seat = {at.x + (kStack2.x - at.x) * (kGoal.y - at.y) / (kStack2.y - at.y),
            kGoal.y};
    // No reset after the last score: nothing is left to aim at.
    if (!score(at, seat, lift_position::stage_2_deg,
               lift_position::stage_1_deg, scored2, kAtClamp2, false,
               kGoalForStack2, kScoreReach, &kAimForStack2,
               /*stopAroundTurn=*/true))
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



// Four-pin far route, first sequence only so far: toggle, preload onto the
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
namespace four_pin_far {
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
// Both wall stacks use the same curve. 0.45 and 0.5 turned into the wall too
// early and ended short and to the inside of the pin; 0.4 (tried on the
// second stack) came onto it square.
constexpr float kWallCurveLead = 0.4f;
// The field's mirror line between the two goals, for the left wall stack.
constexpr float kMirrorX = 4.0f;
constexpr Point mirrored(Point p) { return {2.0f * kMirrorX - p.x, p.y}; }
// Both stacks were reached about an inch to the inside of the pin (toward
// the other stack), with the claw not fully round it, so both are aimed this
// much farther out. Part 2 gets it by mirroring.
constexpr float kWallOutIn = 1.0f;
constexpr Point kWallStackAim{kWallStack.x - kWallOutIn, kWallStack.y};
// The curves are driven slower than the tuner's speed: one ends on a stack
// against the wall, the other carries a raised stack.
constexpr float kCurveSpeed = 70.0f;

// PART 2: the left wall stack onto the neutral (middle) goal, as the mirror
// image of part 1. The field is taken as symmetric: the goals are two tiles
// (48 in) apart with a wall stack behind each. The path file agrees once its
// 0.9 scale is taken out, but none of part 2 has been on the field.
constexpr bool kRunPartTwo = true;
constexpr float kNeutralGoalX = kGoalX + 48.0f;
constexpr Point kLeftWallStack = mirrored(kWallStackAim);
constexpr float kLeftWallStackHeading = -kWallStackHeading;
constexpr float kNeutralScoreHeading = 270.0f;  // claw toward +X
// The neutral goal is wider than the alliance goal where the robot meets it:
// both have the same base (about 5.9 in across, measured off the field
// image), but the neutral one is 5.8 in tall against 3.25 and stays wide
// where the alliance goal has already tapered to its top ring. The game
// manual gives no widths, so the extra radius is an estimate to tune: the
// robot stops this much farther out than on the alliance goal instead of
// running into the wider goal and being pushed off line.
constexpr float kNeutralGoalExtraIn = 0.5f;
constexpr Point kNeutralScoreSpot{kNeutralGoalX - 8.75f - kNeutralGoalExtraIn,
                                  kLaneY};
// Mirror image of kStaging: the same 28 in from the neutral goal.
constexpr Point kNeutralStaging{kNeutralGoalX - 28.0f, kLaneY};

// Driven by the route, the approach stops short of both wall stacks (right
// direction, not deep enough), so it is sent this much farther in. One value
// for both stacks (it was 4.65 and 5.0), so they are tuned together.
constexpr float kWallDeeperIn = 5.0f;
// A stall onto a wall stack ends the approach only once odometry has the
// claw at least this far past where it meets the stack (0 = exactly there).
// The claw has to be all the way round the pin, so no margin short of it.
constexpr float kWallStallWithinIn = 0.0f;
// Speed cap for the half turn that brings the claw round for part 2.
constexpr int kSecondTurnSpeed = 80;
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
    // One GPS reading (field inches, robot center) as the approach starts
    // and at the clamp, to check that part 2 really mirrors part 1. Not used
    // to correct anything; zero if the camera had no confident fix.
    gpsreset::GpsPose gps_start{}, gps_clamp{};
};

gpsreset::GpsPose gps_now() {
    auto& st = gpsreset::shared_state();
    if (!st.initialized || !st.gps) return {};
    return gpsreset::read_pose(*st.gps);
}

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
                     WallLog& log, float lead = kWallCurveLead) {
    using namespace three_pin;
    const Point spot = spot_for(stack, headingDeg, deeperIn);
    log.gps_start = gps_now();
    chassis.moveToPoint(via.x, via.y, 1000,
                        {.forwards = false, .maxSpeed = kCurveSpeed,
                         .minSpeed = kChainSpeed,
                         .earlyExitRange = kChainExitIn}, false);
    // The floor makes this end as the robot reaches the stack instead of
    // settling there, like the three-pin's pickups. The spot is past the
    // contact point (deeperIn), so the robot usually meets the stack and wall
    // before crossing it. A stall there ends the move at once, but only
    // within kWallStallWithinIn of where the claw meets the stack: a stall
    // farther out is the claw caught on a cup (8.6 in short on the field,
    // with the robot 3 in to the inside of the pin), and pushing on is what
    // slides it over onto the pin.
    const std::uint32_t approaching = pros::millis();
    watch_for_stall();
    chassis.moveToPose(spot.x, spot.y, headingDeg, kWallApproachTimeoutMs,
                       {.forwards = false,
                        .horizontalDrift = curve::tuned.drift,
                        .lead = lead, .maxSpeed = kCurveSpeed,
                        .minSpeed = kGentleFloor}, true);
    {
        const float h = headingDeg * static_cast<float>(M_PI) / 180.0f;
        constexpr std::uint32_t kStalledMs = 150, kLaunchMs = 300;
        std::uint32_t slow_since = 0;
        while (chassis.isInMotion()) {
            const std::uint32_t now = pros::millis();
            const lemlib::Pose p = chassis.getPose();
            // Still to go along the approach to the spot.
            const float to_go = (p.x - spot.x) * std::sin(h) +
                                (p.y - spot.y) * std::cos(h);
            const bool near = to_go <= deeperIn + kWallStallWithinIn;
            if (!near || drive_rpm() > kStalledRpm ||
                now - approaching < kLaunchMs)
                slow_since = 0;
            else if (slow_since == 0)
                slow_since = now;
            else if (now - slow_since >= kStalledMs) {
                chassis.cancelMotion();
                break;
            }
            pros::delay(10);
        }
    }
    stall_watch_on = false;
    brake_drive();
    const lemlib::Pose clamped = chassis.getPose();
    claw_piston.set_value(true);
    log.gps_clamp = gps_now();
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
}  // namespace four_pin_far

namespace four_pin_far {
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
    const char* const title = side.close ? "4 PIN CLOSE" : "4 PIN FAR";
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
    // Until the route finishes, the time is that of the latest step: a run
    // cut off by the 15 s period stops on that screen.
    bool finished = false;
    const auto show = [&] {
        char text[768];
        int used = std::snprintf(
            text, sizeof(text),
            "%s  %s %.1f s\n"
            "At: toggle %.1f  preload %.1f\n"
            " stack %.1f  score %.1f    stack %.1f  score %.1f",
            title, finished ? "took" : "last step at",
            (pros::millis() - started) / 1000.0, at_s[kToggle],
            at_s[kPreload], at_s[kStack1], at_s[kScore1], at_s[kStack2],
            at_s[kScore2]);
        for (int i = 0; i < 2 && used < static_cast<int>(sizeof(text)); ++i) {
            // GPS reading as "x,y heading", or "none" without a fix.
            char g[2][24];
            const gpsreset::GpsPose* gp[2] = {&wall[i].gps_start,
                                              &wall[i].gps_clamp};
            for (int k = 0; k < 2; ++k)
                if (gp[k]->ok)
                    std::snprintf(g[k], sizeof(g[k]), "%.1f,%.1f %.0f",
                                  gp[k]->x_in, gp[k]->y_in, gp[k]->theta_deg);
                else
                    std::snprintf(g[k], sizeof(g[k]), "none");
            used += std::snprintf(
                text + used, sizeof(text) - used,
                "\nWall %d: approach %lu ms, stalled %lu ms\n"
                " short %.1f in  right %+.1f in  %+.1f deg\n"
                " lift %lu ms %.0f mA  score off %.1f in %+.1f\n"
                " GPS go %s  clamp %s",
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
                                     360.0f),
                g[0], g[1]);
            if (used < 0) break;
        }
        fieldviz::report_full(text);
    };
    const Point wall_stack = side.at(kWallStackAim);
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
    // Same hand-over onto the staging point as part 1's.
    chassis.moveToPoint(middle_staging.x, middle_staging.y, 1200,
                        {.forwards = true, .minSpeed = kChainSpeed,
                         .earlyExitRange = kChainExitIn}, false);
    const Point turn_in = side.at(mirrored(kWallTurnIn));
    // Part 1 starts its approach from kStaging facing score_heading; part 2
    // turns round to the mirror image of that (claw toward the middle goal),
    // so both approaches start from mirrored poses. The half turn slows and
    // all but stops on the heading: handed over still rotating it overturned.
    chassis.turnToHeading(score_headings[1], 1000,
                          {.maxSpeed = kSecondTurnSpeed,
                           .minSpeed = kGoalTurnFloor,
                           .earlyExitRange = kGoalTurnExitDeg}, false);
    if (!LiftJob{LiftJob::Running{}}.wait()) return;
    take_wall_stack(second_stack, second_heading, turn_in, kWallDeeperIn,
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
    finished = true;
    show();
}
}  // namespace four_pin_far

void four_pin_far_auton() { four_pin_far::run({.close = false}); }
void four_pin_close_auton() { four_pin_far::run({.close = true}); }

// Driver-control recorder for the four-pin far's curve onto the wall stack
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
    using namespace four_pin_far;
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
    using namespace four_pin_far;
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
// depth the four-pin's needed.
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
// G3 is one tile east and one tile south of G2. Tiles are 600 mm (23.62 in);
// this frame is anchored on G2, so G3 is placed from it.
constexpr float kTileIn = 23.62f;
constexpr Point kG3Field{kG2Field.x + kTileIn, kG2Field.y - kTileIn};
constexpr Point kG3 = on_field(kG3Field);
static_assert(kG2.x == three_pin::kGoal.x && kG2.y == three_pin::kGoal.y);
// The guide stops the robot's center at x = -62 in the loader. It is sent
// 4 in past that and the run ends when the robot is against the loader.
// Y: the guide had -59. With the wall reset working, the robot reached the
// loader about 1 in to its left at -56 and was still on the loader's edge at
// -57. The robot backs in facing east, so its right is south (-Y). The
// line-up spot moves with it so the run in stays straight.
// Which way is which: this is field Y (the field map's frame). In the
// three-pin's start frame (x along the toggle wall, y away from it) the same
// direction is x: three-pin x = -4 - field Y. More negative here = farther
// along the wall toward the loader's corner.
// History: -56 and -57 short of the loader, -58.5 "centimetres" short;
// -57.75 (a misread "go back") went the wrong way and was further off.
// So 0.75 in past -58.5, toward the corner.
constexpr float kLoaderLaneY = -59.25f;
constexpr Point kLoaderField{-66.0f, kLoaderLaneY};
constexpr Point kLoader = on_field(kLoaderField);
// Field X of the robot's center when it is backed against the loader. The
// loader stands about 4.3 in out from the wall (field drawing) and the robot
// center ends 8 in in front of where its claw end meets it, give or take
// the cut-out: -59.7 to -57.7. -58.6 is where odometry had it on the first
// trip, which the GPS confirmed to 0.6 in just before.
constexpr float kLoaderContactFieldX = -58.6f;
// How wide the curve back onto the loader lane swings (LemLib lead).
constexpr float kLoaderLead = 0.3f;
// Parked against the loader this long before the claw closes.
constexpr std::uint32_t kLoaderWaitMs = 250;
// Longest a GPS reset may hold the robot; a good one takes about 250 ms
// from the moment the robot is still.
constexpr std::uint32_t kResetWaitMs = 450;
// Where every run into the loader starts: straight in front of it and due
// south of G2.
constexpr Point kLineUp = on_field({-48.0f, kLoaderLaneY});
// G3 trips: from the line-up spot the robot backs straight north to here
// (level with G3, due north of the line-up spot), turns its claw east and
// backs into G3; afterwards it comes back out here, nose first.
constexpr Point kG3Lane = on_field({-48.0f, kG3Field.y});
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
// Updated 2026-10-17 from the team's drawing ("end", just south of the
// center goal). The robot backs north into it claw first from the toggle:
// through kParkEntry, which keeps it 2 in clear of the grey goal at (0, -24)
// and well clear of G3, then to kPark (claw toward the center goal).
constexpr Point kParkEntry = on_field({-12.0f, -30.0f});
constexpr Point kPark = on_field({-8.0f, -12.0f});

// A wall stack: where it stands, the heading the robot ends on backed onto
// it, where the robot turns its claw to lead, the point it backs through,
// and the pose it curves back out to with the stack, which is also where it
// turns the claw to the goal from and pulls back out to. The guide's poses,
// untuned: the four-pin's wall stacks each took several field sessions.
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
constexpr int kG2Loads = 3;  // fills G2: a pin and five cups
constexpr int kG3Loads = 5;
constexpr bool kWallPins = false;  // not in the current plan (2026-10-17)
constexpr bool kToggleAndPark = true;
constexpr int kMostOnGoal = 6;  // a pin and five cups
// Expected seconds for each step: estimates from distances and the four-pin's
// measured pace, to be replaced by the times the report shows.
constexpr float kRunS = 60.0f;
// The clock check that skips steps which would not leave time for the
// toggle and park. Off while testing, so every step runs and shows its time.
constexpr bool kUseClock = false;
constexpr float kG2TripS = 7.5f;
constexpr float kG3TripS = 8.0f;
constexpr float kWallPin2S = 8.5f;
constexpr float kWallPin3S = 8.0f;
constexpr float kEndgameS = 7.0f;

// How close the robot's center is sent to a goal's center. It cannot get
// nearer than three_pin::kReach, so the run in ends against the goal even
// with odometry a few inches out.
constexpr float kScoreReachIn = 3.0f;
// Largest move the goal reference may make to odometry (see put_on).
constexpr float kGoalRefMaxIn = 4.0f;
// Lift height a match load is carried at, the same as the preload.
constexpr double kCarryDeg = lift_position::matchload - 125;
// Just off the floor, so a load does not drag while the robot pulls it out
// of the loader; any higher there and it clips the loader. The lift goes the
// rest of the way up once the robot has turned to the goal (carry_to).
constexpr double kLoadOffFloorDeg = -150.0;
constexpr float kStageStep =
    lift_position::stage_2_deg - lift_position::stage_1_deg;
// moveLift() has to arrive within 8 degrees of its target, so it is not sent
// to the very top; the lift lock holds it there from here.
constexpr float kLiftTopDeg = 0.97f * lift_position::max_height_deg;

// Lift height for the third cup on a goal: one more cup's step above the
// second, the step being the three-pin's from its first cup to its second.
// It is above 95% of the lift's travel, so once the move ends the lift lock
// holds it at the very top (-1670, 31 degrees higher).
constexpr float kThirdCupDeg =
    lift_position::stage_2_deg +
    (lift_position::stage_2_deg - lift_position::stage_1_deg);
// All the way up. moveLift() ends within 8 degrees, so it is sent 8 short of
// the top (it cannot be sent to the very top); the lift lock then holds it at
// the top.
constexpr float kFullLiftDeg = lift_position::max_height_deg + 8.0f;
// Where the lift and arm carry the stack that goes onto a goal already
// holding 0, 1, ... 5 (the pin, then cups).
//
// The first three are the three-pin's heights; the third cup is a step
// above the second; the fourth is the third cup's lift with the arm raised
// to the Y macro's position and released like RIGHT; the fifth is the same
// with the lift all the way up (the fourth's was too low for it).
struct Level {
    float lift_deg;
    float arm_deg;
};
constexpr Level kLevels[kMostOnGoal] = {
    // The pin on an empty goal; it is then lowered to kPreloadSeatDeg.
    {static_cast<float>(kCarryDeg), arm_position::rest_deg},
    {lift_position::stage_1_deg, arm_position::rest_deg},   // first cup
    {lift_position::stage_2_deg, arm_position::rest_deg},   // second cup
    {kThirdCupDeg, arm_position::rest_deg},                 // third cup
    {kThirdCupDeg, arm_position::max_deg},                  // fourth cup
    {kFullLiftDeg, arm_position::max_deg},                  // fifth cup
};
constexpr bool with_arm(int held) {
    return kLevels[held].arm_deg > arm_position::rest_deg;
}

// The worker owns its deadline even while the route is inside a chassis
// motion. A finished job is not proof that the arm reached its target.
class ArmJob {
public:
    explicit ArmJob(double targetDeg, std::uint32_t limitMs = 1500) {
        end();
        cancelled() = false;
        outcome() = skills_control::ArmOutcome::running;
        pros::Task([targetDeg, limitMs] {
            outcome() = move_arm_bounded(targetDeg, &cancelled(), limitMs);
        });
    }
    static skills_control::ArmOutcome result() { return outcome().load(); }
    static bool done() { return result() != skills_control::ArmOutcome::running; }
    static bool failed() {
        return result() == skills_control::ArmOutcome::sensor_error ||
               result() == skills_control::ArmOutcome::cancelled;
    }
    static void wait() {
        while (!done()) pros::delay(10);
    }
    static void end() {
        cancelled() = true;
        while (!done()) pros::delay(5);
    }

private:
    static std::atomic_bool& cancelled() {
        static std::atomic_bool value{false};
        return value;
    }
    static std::atomic<skills_control::ArmOutcome>& outcome() {
        static std::atomic<skills_control::ArmOutcome> value{
            skills_control::ArmOutcome::reached};
        return value;
    }
};

// Blocking time only: mechanism work that overlaps driving is not added
// here. These counters are also shown on the Brain, without a terminal.
struct TripTiming {
    std::uint32_t wall_ms = 0, down_ms = 0, stack_ms = 0, pickup_ms = 0;
    std::uint32_t loader_ms = 0, aim_ms = 0, goal_ms = 0, release_ms = 0;
    std::uint32_t return_ms = 0;
};

double lift_degrees() {
    const std::int32_t raw = lift_sensor.get_position();
    return raw == PROS_ERR ? NAN : raw / 100.0;
}
double arm_degrees() {
    const std::int32_t raw = claw_sensor.get_position();
    return raw == PROS_ERR ? NAN : raw / 100.0;
}

// A false readiness result ends the attempt with the element still held.
// run() joins both workers on every return, including these aborts.
const char* mechanism_stop_reason = "";
bool mechanism_failure(const char* stage) {
    mechanism_stop_reason = stage;
    chassis.cancelAllMotions();
    three_pin::brake_drive();
    std::printf("SKILLS_MECHANISM_FAIL,%s,lift=%.1f,arm=%.1f\n", stage,
                lift_degrees(), arm_degrees());
    return false;
}

// Turn the nose toward `target`, handing straight over to the drive. For the
// corners a single drive would swing wide on, next to a goal or the cups.
void turn_nose_to(Point target) {
    using namespace three_pin;
    chassis.turnToPoint(target.x, target.y, 900,
                        {.forwards = true, .minSpeed = kTurnChainSpeed,
                         .earlyExitRange = kTurnChainExitDeg}, false);
}

// The match-load loop moves like the three-pin's preload drop-off, a little
// faster since its runs are longer (the preload run is capped at 90). Each
// move stops before the next one starts, but crisply: it ends at a low power
// floor as it reaches the point, and the brakes stop it, instead of waiting
// out LemLib's settle (within 1 in for 100 ms, which is slow at the end of a
// move). Moves and turns handed over still rolling overshot and drifted;
// capped at 60 and fully settled, the loop was far too slow.
// LemLib 0-127 scale.
constexpr float kCalmDrive = 60.0f;
constexpr int kCalmTurn = 50;
constexpr float kStopFloor = 20.0f;      // power at the end of a move
constexpr float kStopExitIn = 0.5f;
constexpr int kTurnStopFloor = 10;
constexpr float kTurnStopExitDeg = 1.0f;
constexpr float kTurnFixDeg = 1.5f;
void calm_turn_to_heading(float heading) {
    chassis.turnToHeading(heading, 1500,
                          {.maxSpeed = kCalmTurn, .minSpeed = kTurnStopFloor,
                           .earlyExitRange = kTurnStopExitDeg}, false);
    three_pin::stop_rolling(150);
    // If it still went past by more than kTurnFixDeg, a slow settled touch-up
    // (this is the turn the robot backs into the loader on).
    if (std::abs(std::remainder(chassis.getPose().theta - heading, 360.0f)) >
        kTurnFixDeg) {
        chassis.turnToHeading(heading, 600, {.maxSpeed = 30}, false);
        three_pin::brake_drive();
    }
}
void calm_turn_to_point(Point target, bool forwards) {
    chassis.turnToPoint(target.x, target.y, 1500,
                        {.forwards = forwards, .maxSpeed = kCalmTurn,
                         .minSpeed = kTurnStopFloor,
                         .earlyExitRange = kTurnStopExitDeg}, false);
    three_pin::brake_drive();
}

// One aim, with the load low. The previous second turn ran after raising
// had begun and was also being used to hide residual rotation from the first.
bool aim_for_score(Point goal) {
    calm_turn_to_point(goal, false);
    three_pin::stop_rolling(150);
    const auto heading_error = [&] {
        const lemlib::Pose p = chassis.getPose();
        const float target = std::atan2(goal.x - p.x, goal.y - p.y) *
                                 180.0f / static_cast<float>(M_PI) + 180.0f;
        return std::abs(std::remainder(p.theta - target, 360.0f));
    };
    if (heading_error() > kTurnFixDeg) {
        calm_turn_to_point(goal, false);
        three_pin::stop_rolling(150);
    }
    return std::isfinite(heading_error()) && heading_error() <= kTurnFixDeg &&
           three_pin::drive_rpm() <= 15.0;
}
// With async the caller brakes once the move is done.
void calm_drive_to(Point target, bool forwards, bool async = false) {
    chassis.moveToPoint(target.x, target.y, 2500,
                        {.forwards = forwards, .maxSpeed = kCalmDrive,
                         .minSpeed = kStopFloor,
                         .earlyExitRange = kStopExitIn}, async);
    if (!async) three_pin::brake_drive();
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
    calm_turn_to_point(to, true);
    calm_drive_to(to, true);
}

// ---- Wall resets with the two front distance sensors ----------------------
//
// The usual VEX approach (VEX Forum; other teams' code): heading stays with
// the IMU, the robot faces a wall square, and the distance sensors set only
// the one coordinate along that wall's normal:
//   wall + (reading + sensor offset) x cos(angle off square).
// A reset is refused when the robot is moving, is not square, the sensors
// disagree, or the result is far from where odometry expects it.
//
// Heading is NOT taken from the two sensors' difference. Each V5 distance
// sensor can read about 15 mm off on its own, its cone is 24 degrees wide
// (so at an angle it reports something closer to the perpendicular distance
// than the along-beam one), and it updates at only about 30 Hz. The slope
// between two sensors a few inches apart is therefore worth several degrees
// of error; the first version that used it sent the robot off line. The
// second sensor is used to cross-check the first instead: both should see
// the same wall at nearly the same distance.
//
// Mounting to check: the sensors must sit high enough that the lower edge of
// the 24 degree cone (12 degrees down) reaches the wall before the floor:
// at least about 2.5 in up for a wall 10 in away.
constexpr float kSensorFwdIn = 4.5f;  // robot center to the sensors' faces
// The field is about 140.5 in between the walls (VEX field spec) and its
// tiles are 600 mm (23.62 in), so G2 at (-1200, -600) mm is 46.63 in north
// of the south wall. This frame puts G2 at y = -24.
constexpr float kSouthWallY = kG2Field.y - (70.25f - 23.62f);
// Samples: six polls at 0..50 ms, about two sensor updates at 30 Hz; the
// median of each sensor's samples is used, so one bad sample does nothing.
// The robot has just braked to a stop at the end of a turn, so there is no
// separate wait for it to be still unless the wheels are still turning.
constexpr int kWallSamples = 6;
constexpr std::uint32_t kWallSampleMs = 10;
// Gates.
constexpr float kWallMaxSquareDeg = 5.0f;  // IMU heading off square
constexpr float kWallAgreeIn = 1.5f;       // the two sensors
constexpr float kWallMaxRmsIn = 0.35f;     // scatter of a sensor's samples
constexpr float kWallMaxMoveIn = 4.0f;     // change to odometry
constexpr float kLineUpFixIn = 0.3f;       // re-place on the line-up spot past this
constexpr double kWallStillRpm = 5.0;      // drive wheels, to count as parked

float median(float* v, int n) {
    std::sort(v, v + n);
    return n % 2 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

struct WallReading {
    bool ok = false;
    float a_in = 0.0f, b_in = 0.0f;  // medians
    float rms_in = 0.0f;             // worse sensor's scatter about its median
    int samples = 0;
};

WallReading read_wall() {
    float a[kWallSamples], b[kWallSamples];
    int n = 0;
    // Samples count only once the wheels are still; the robot has just
    // braked, so this is usually at once (at most 0.2 s in all).
    for (int tries = 0; tries < 20 && n < kWallSamples; ++tries) {
        if (three_pin::drive_rpm() > kWallStillRpm) {
            // Every accepted batch must describe one continuous stop.
            n = 0;
            pros::delay(kWallSampleMs);
            continue;
        }
        const std::int32_t da = wall_dist_a.get_distance();
        const std::int32_t db = wall_dist_b.get_distance();
        // 9999 = nothing in range; PROS_ERR = sensor missing.
        if (da != PROS_ERR && db != PROS_ERR && da >= 20 && db >= 20 &&
            da < 2000 && db < 2000) {
            a[n] = da / 25.4f;
            b[n] = db / 25.4f;
            ++n;
        }
        if (n == kWallSamples) break;  // do not sleep after the final reading
        pros::delay(kWallSampleMs);
    }
    WallReading r;
    r.samples = n;
    if (n < kWallSamples / 2) return r;
    const auto scatter = [n](const float* v, float mid) {
        float ss = 0.0f;
        for (int i = 0; i < n; ++i) ss += (v[i] - mid) * (v[i] - mid);
        return std::sqrt(ss / n);
    };
    float sa[kWallSamples], sb[kWallSamples];
    std::copy(a, a + n, sa);
    std::copy(b, b + n, sb);
    r.a_in = median(sa, n);
    r.b_in = median(sb, n);
    r.rms_in = std::max(scatter(a, r.a_in), scatter(b, r.b_in));
    r.ok = r.rms_in <= kWallMaxRmsIn;
    return r;
}

// What one wall reset did, for the report.
struct WallFix {
    bool applied = false;
    const char* why = "not run";
    float dy_in = 0.0f;     // change to the wall-normal coordinate
    float a_in = 0.0f, b_in = 0.0f;
    float rms_in = 0.0f;
};

// Robot parked nose south toward the south wall: set its distance from that
// wall (field Y, route X). Heading and the other coordinate are untouched.
WallFix south_wall_reset() {
    using namespace three_pin;
    WallFix fix;
    brake_drive();
    const lemlib::Pose before = chassis.getPose();
    const float square = field_heading(180.0f);
    if (!std::isfinite(before.theta) ||
        std::abs(std::remainder(before.theta - square, 360.0f)) >
            kWallMaxSquareDeg) {
        fix.why = "not square";
        coast_drive();
        return fix;
    }
    const WallReading r = read_wall();
    fix.a_in = r.a_in;
    fix.b_in = r.b_in;
    fix.rms_in = r.rms_in;
    // Sampling can take 200 ms. Preserve the current unconstrained axis and
    // heading, and gate the correction against the current pose, not the
    // pose from before the wheels finished stopping.
    const lemlib::Pose pose = chassis.getPose();
    const float off_deg = std::remainder(pose.theta - square, 360.0f);
    if (drive_rpm() > kWallStillRpm || !std::isfinite(pose.x) ||
        !std::isfinite(pose.y) || !std::isfinite(off_deg)) {
        fix.why = "moving or invalid pose";
        coast_drive();
        return fix;
    }
    if (std::abs(off_deg) > kWallMaxSquareDeg) {
        fix.why = "not square";
        coast_drive();
        return fix;
    }
    coast_drive();
    if (r.samples < kWallSamples / 2) {
        fix.why = "no reading";
        return fix;
    }
    if (!r.ok) {
        fix.why = "scattered";
        return fix;
    }
    if (std::abs(r.a_in - r.b_in) > kWallAgreeIn) {
        fix.why = "sensors disagree";
        return fix;
    }
    const float off = off_deg * static_cast<float>(M_PI) / 180.0f;
    const float along =
        (kSensorFwdIn + 0.5f * (r.a_in + r.b_in)) * std::cos(off);
    const float field_y = kSouthWallY + along;
    const float route_x = kG2Field.y + three_pin::kGoal.x - field_y;
    fix.dy_in = route_x - pose.x;
    if (std::abs(fix.dy_in) > kWallMaxMoveIn) {
        fix.why = "too far from odometry";
        return fix;
    }
    chassis.setPose(route_x, pose.y, pose.theta);
    fix.applied = true;
    fix.why = "ok";
    return fix;
}

// "Down" for the loader: the lift within this of the bottom (the same line
// LiftJob::Settle counts as success) and the arm within kArmCloseDeg of rest.
constexpr double kLiftDownDeg = -83.0;
constexpr double kArmCloseDeg = 25.0;
// "Up" before the robot meets a goal (put_on): lift within kLiftAtDeg of its
// height, arm within kArmAtDeg of its own; the robot holds 6 in short for at
// most kStackWaitMs for that.
constexpr double kLiftAtDeg = 25.0;
constexpr double kArmAtDeg = 20.0;
constexpr std::uint32_t kStackWaitMs = 2500;
// Physical clearance lets driving overlap the tail of a lowering job.
// Job completion alone never supplies clearance, and expiration is failure.
bool wait_until_down(TripTiming& timing) {
    using three_pin::LiftJob;
    using skills_control::Readiness;
    const std::uint32_t began = pros::millis();
    LiftJob running{LiftJob::Running{}};
    const auto outcome = skills_control::wait_until(began, kStackWaitMs, [&] {
        const double lift = lift_degrees(), arm = arm_degrees();
        if (!std::isfinite(lift) || !std::isfinite(arm) ||
            running.failed() || ArmJob::failed()) return Readiness::failed;
        if (skills_control::loader_position_ready(
                lift, arm, kLiftDownDeg, arm_position::rest_deg, kArmCloseDeg))
            return Readiness::ready;
        if ((running.done() && lift < kLiftDownDeg) ||
            (ArmJob::done() && arm > arm_position::rest_deg + kArmCloseDeg))
            return Readiness::failed;
        return Readiness::waiting;
    }, [] { return pros::millis(); }, [](std::uint32_t ms) { pros::delay(ms); });
    timing.down_ms += pros::millis() - began;
    return outcome == skills_control::WaitOutcome::ready ||
           mechanism_failure("loader clearance");
}

// With the claw empty: to kLineUp, square up nose south, reset position from
// the south wall, turn the claw to the loader, back in until the robot is against
// it, wait kLoaderWaitMs, clamp, and lift the pin to carrying height. The
// lift (and the arm, after a stack set down from it) must be back down
// before the robot backs in.
bool take_load(FixTally& lined_up, FixTally& at_loader, bool measure_gps,
               lemlib::Pose* stopped_at, float* reference_in,
               WallFix* wall_fix, TripTiming& timing) {
    using namespace three_pin;
    go_to(kLineUp);
    // Nose south to the south wall, 7 in away: the distance sensors set the
    // distance from that wall (the robot's sideways place
    // in front of the loader). The GPS is no longer used here: parked nose
    // south its camera looks straight at the loader, which hides most of
    // the wall code.
    (void)lined_up;
    (void)measure_gps;
    (void)at_loader;
    // Square to the south wall for the reset. Coming back from a goal the
    // robot already faces south within a degree or two: then no turn.
    if (std::abs(std::remainder(chassis.getPose().theta -
                                field_heading(180.0f), 360.0f)) > 3.0f)
        calm_turn_to_heading(field_heading(180.0f));
    const std::uint32_t resetting_since = pros::millis();
    const WallFix reset = south_wall_reset();
    timing.wall_ms += pros::millis() - resetting_since;
    if (wall_fix) *wall_fix = reset;
    // The reset sets the robot's distance from the south wall, which is
    // along its own axis (nose south). Before, the robot turned to the
    // loader from where it stood and the curve into the loader then made up
    // the difference sideways, swinging in and ending in the same wrong
    // place. Now it first drives straight forward or back onto the line-up
    // spot, so the run into the loader is straight.
    const lemlib::Pose p = chassis.getPose();
    // The correction size is not the remaining placement error. A large
    // reset can put us exactly on the spot; do not command a zero-length
    // minimum-power move in that case. Only route X was measured here.
    if (reset.applied && std::abs(kLineUp.x - p.x) > kLineUpFixIn) {
        const float h = p.theta * static_cast<float>(M_PI) / 180.0f;
        const float ahead = (kLineUp.x - p.x) * std::sin(h) +
                            (kLineUp.y - p.y) * std::cos(h);
        calm_drive_to(kLineUp, ahead > 0.0f);
    }
    // Claw square to the loader (west), by heading rather than by aiming at
    // the loader point: after the GPS reset the robot can be a couple of
    // inches off the lane, and aiming at the point would put it in at an
    // angle.
    calm_turn_to_heading(field_heading(90.0f));
    // Back in as soon as the lift and arm are physically down, not when
    // their jobs end: lowering the lift only ends 0.3 s after it stops at
    // the bottom, and the arm often stops just short of 0 and then only
    // ends at its 1.5 s limit. Both kept the robot standing here.
    if (!wait_until_down(timing)) return false;
    // A gentle curve back onto the lane that arrives square: inside 7.5 in
    // of the target LemLib only holds the heading, so the last stretch into
    // the loader is straight.
    const std::uint32_t loader_started = pros::millis();
    chassis.moveToPose(kLoader.x, kLoader.y, field_heading(90.0f), 2500,
                       {.forwards = false, .lead = kLoaderLead,
                        .maxSpeed = kCalmDrive, .minSpeed = kGentleFloor},
                       true);
    // Ends against the loader either way: on the stall, or pushing on it
    // until the time limit (the target is 4 in past it, so it cannot be
    // reached). Only a run that odometry says got all the way there is not
    // trusted.
    wait_done_or_stalled(loader_started);
    timing.loader_ms += pros::millis() - loader_started;
    const bool against_loader =
        std::hypot(kLoader.x - chassis.getPose().x,
                   kLoader.y - chassis.getPose().y) > 1.0f;
    brake_drive();
    const std::uint32_t parked = pros::millis();
    const lemlib::Pose stopped = chassis.getPose();
    if (stopped_at) *stopped_at = stopped;
    // Pressed against the loader, the robot's distance from the wall is
    // known: set that one coordinate (field X) and leave the other two. This
    // is what lines the robot up on G2 for the run north, so it no longer
    // misses the goal; the goal in turn sets the other axis (put_on). Two
    // contacts at right angles, like squaring up on two walls. The camera
    // cannot help here: its side faces the south wall from about 12 in.
    *reference_in = NAN;
    // Refused if it would move odometry more than kWallMaxMoveIn: then the
    // robot is probably stopped on something before the loader.
    const float loader_route_y =
        kLoaderContactFieldX - kG2Field.x + three_pin::kGoal.y;
    if (against_loader &&
        std::abs(loader_route_y - stopped.y) <= kWallMaxMoveIn) {
        *reference_in = loader_route_y - stopped.y;
        chassis.setPose(stopped.x, loader_route_y, stopped.theta);
    }
    while (pros::millis() - parked < kLoaderWaitMs) pros::delay(10);
    claw_piston.set_value(true);
    pros::delay(kPneumaticSettleMs);
    // The lowering job has long finished by now; one lift job at a time.
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    const double lift_start = lift_degrees();
    if (!std::isfinite(lift_start)) return mechanism_failure("pickup sensor");
    LiftJob raise(kLoadOffFloorDeg);
    const std::uint32_t lifting_since = pros::millis();
    const auto picked_up = skills_control::wait_until(lifting_since, 400, [&] {
        const double lift = lift_degrees();
        if (!std::isfinite(lift) || raise.failed())
            return skills_control::Readiness::failed;
        // A successful -150 target may end at -142. From an allowed -83
        // start that is 59 degrees: it is already at the accepted carrying
        // height and must not burn the whole 400 ms waiting for degree 60.
        if (skills_control::pickup_clear(lift, lift_start, raise.succeeded(),
                                         kLoadOffFloorDeg, kLiftClearDeg, 8.0))
            return skills_control::Readiness::ready;
        return raise.done() ? skills_control::Readiness::failed
                            : skills_control::Readiness::waiting;
    }, [] { return pros::millis(); }, [](std::uint32_t ms) { pros::delay(ms); });
    timing.pickup_ms += pros::millis() - lifting_since;
    if (picked_up != skills_control::WaitOutcome::ready)
        return mechanism_failure("pickup clearance");
    coast_drive();
    return true;
}

// Start the lift and arm toward the height the stack is carried at for a
// goal already holding `held`, so they are up by the time the robot gets
// there. False if the lift move before it failed.
bool raise_for(int held) {
    using three_pin::LiftJob;
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    LiftJob raise(kLevels[held].lift_deg, /*fastUp=*/true);
    // A previous high-score return may have stopped within the loader's
    // 25-degree clearance but outside the score's 20-degree window. Every
    // score gets its required target, including rest on the ordinary levels.
    ArmJob arm(kLevels[held].arm_deg);
    return true;
}


// From where the robot stands with a stack in the claw and the lift and arm
// already on their way up (raise_for), after aim_for_score: `goal`
// already holds `held`, back in until the robot is against the goal
// (holding short if the lift or arm is not up yet), let go, and take
// position from the goal as the three-pin does. False if the lift failed.
// The robot may leave as soon as this returns: the claw is open.
bool put_on(Point goal, int held, float* reference_in, TripTiming& timing,
            bool lower_for_toggle = false) {
    using namespace three_pin;
    using skills_control::Readiness;
    LiftJob raise{LiftJob::Running{}};
    // The one low-load aim has already stopped and been checked. Do not
    // issue another turn after starting the raise.
    const lemlib::Pose turned = chassis.getPose();
    const Point here{turned.x, turned.y};
    const Point stop = standoff(here, goal, kScoreReachIn);
    const lemlib::MoveToPointParams entry{.forwards = false,
                                          .maxSpeed = kCalmDrive,
                                          .minSpeed = kGentleFloor};
    const std::uint32_t approaching_since = pros::millis();
    std::uint32_t motion_started = approaching_since;
    chassis.moveToPoint(stop.x, stop.y, 2500, entry, true);
    // The robot meets the goal kReach out, before `stop`; it holds
    // kGoalStandbyIn short of that.
    chassis.waitUntil(std::hypot(stop.x - here.x, stop.y - here.y) -
                      (kReach - kScoreReachIn) - kGoalStandbyIn);
    // The stack must be up before the robot meets the goal, or it knocks
    // things over: hold here until the lift has finished and the arm is at
    // its height.
    // Ready means measured, not "the job ended": a lift job also ends when it
    // fails, and an arm job ends at its 1.5 s limit even if the arm hung
    // short. The lift must be within kLiftAtDeg of its height (or in the top
    // band the lift lock holds), the arm within kArmAtDeg of its own.
    const double lift_target = kLevels[held].lift_deg;
    const double arm_target = kLevels[held].arm_deg;
    const auto stack_state = [&] {
        const double lift = lift_degrees(), arm = arm_degrees();
        if (!std::isfinite(lift) || !std::isfinite(arm) || raise.failed() ||
            ArmJob::failed()) return Readiness::failed;
        const bool positioned = skills_control::score_position_ready(
            lift, arm, lift_target, arm_target, kLiftAtDeg, kArmAtDeg,
            lift_position::lock_from_deg);
        // Joining the raise is still required before a lowering job takes
        // the motors. A completed failure never satisfies this gate.
        if (raise.succeeded() && positioned) return Readiness::ready;
        if ((raise.done() && lift > lift_target + kLiftAtDeg &&
             lift > lift_position::lock_from_deg) ||
            (ArmJob::done() && std::abs(arm - arm_target) > kArmAtDeg))
            return Readiness::failed;
        return Readiness::waiting;
    };
    // One deadline for this scoring attempt, shared by the short-of-goal
    // gate and the final release gate. No second 2.5 s wait, and no hidden
    // unbounded LiftJob::wait() after an expired readiness gate.
    const std::uint32_t ready_since = pros::millis();
    const auto wait_for_stack = [&] {
        const std::uint32_t began = pros::millis();
        const auto result = skills_control::wait_until(
            ready_since, kStackWaitMs, stack_state,
            [] { return pros::millis(); },
            [](std::uint32_t ms) { pros::delay(ms); });
        timing.stack_ms += pros::millis() - began;
        return result == skills_control::WaitOutcome::ready;
    };
    if (stack_state() != Readiness::ready) {
        if (chassis.isInMotion()) chassis.cancelMotion();
        brake_drive();
        if (!wait_for_stack()) {
            timing.goal_ms += pros::millis() - approaching_since;
            return mechanism_failure("score height");
        }
        coast_drive();
        motion_started = pros::millis();
        chassis.moveToPoint(stop.x, stop.y, 2500, entry, true);
    }
    const bool against_goal = wait_done_or_stalled(motion_started);
    if (!wait_for_stack()) {
        timing.goal_ms += pros::millis() - approaching_since;
        return mechanism_failure("release height");
    }
    brake_drive();
    timing.goal_ms += pros::millis() - approaching_since;
    const std::uint32_t releasing_since = pros::millis();
    const auto release_failed = [&](const char* stage) {
        timing.release_ms += pros::millis() - releasing_since;
        return mechanism_failure(stage);
    };

    if (with_arm(held)) {
        // RIGHT in driver control: the arm comes down, the claw lets go
        // halfway there, and the lift starts down. An expired worker is not
        // evidence that the arm has reached that release position.
        const double release_start = arm_degrees();
        if (!std::isfinite(release_start))
            return release_failed("arm release sensor");
        const double release_within =
            std::abs(release_start - arm_position::release_deg) / 2.0;
        ArmJob lower_arm(arm_position::release_deg);
        while (!ArmJob::done() &&
               std::isfinite(arm_degrees()) &&
               std::abs(arm_degrees() - arm_position::release_deg) > release_within)
            pros::delay(10);
        const double release_at = arm_degrees();
        if (ArmJob::failed() || !std::isfinite(release_at) ||
            std::abs(release_at - arm_position::release_deg) > release_within)
            return release_failed("arm release travel");
        claw_piston.set_value(false);
        pros::delay(arm_position::release_settle_ms);
        // Last planned G3 load: the next lift destination is the toggle,
        // not the floor. Avoid lowering to rest and raising back to -375.
        // Both destinations overlap departure; arm return still waits for
        // the existing 6-inch claw clearance in leave_to().
        if (lower_for_toggle) LiftJob lower(kCarryDeg);
        else LiftJob lower{LiftJob::Settle{}};
    } else if (held == 0) {
        // On an empty goal: lower only as far as seats the pin.
        if (!LiftJob(kPreloadSeatDeg).wait()) return release_failed("pin seat");
        claw_piston.set_value(false);
        pros::delay(kPneumaticSettleMs);
    } else {
        // Set the stack down, opening the claw a third of a level down; the
        // lift carries on all the way down while the robot heads back to the
        // loader.
        const double scored_at = lift_degrees();
        if (!std::isfinite(scored_at)) return release_failed("seat sensor");
        LiftJob lower{LiftJob::Settle{}};
        const std::uint32_t lowering_since = pros::millis();
        const auto seated = skills_control::wait_until(lowering_since, 800, [&] {
            const double lift = lift_degrees();
            if (!std::isfinite(lift) || lower.failed()) return Readiness::failed;
            if (lift >= scored_at + kSeatDropDeg) return Readiness::ready;
            return lower.done() ? Readiness::failed : Readiness::waiting;
        }, [] { return pros::millis(); }, [](std::uint32_t ms) { pros::delay(ms); });
        if (seated != skills_control::WaitOutcome::ready)
            return release_failed("seat travel");
        claw_piston.set_value(false);
        pros::delay(kPneumaticSettleMs);
    }
    // The claw's grip is on the goal's center, kReach from the robot's. Only
    // if the run in ended against the goal: a run that reached its target
    // instead went past the goal, and resetting from it then would put
    // odometry inches out for every trip after (that is how the loader
    // spot drifted from trip to trip).
    // Also only if it moves odometry less than kGoalRefMaxIn: the run in is
    // sent to 3 in from the center and stops at about 8, so normally the
    // reference moves odometry very little; a big move means the robot met
    // the goal off-center or slid on it.
    const lemlib::Pose pressed = chassis.getPose();
    const float heading = pressed.theta * static_cast<float>(M_PI) / 180.0f;
    const float ref_x = goal.x + kReach * std::sin(heading);
    const float ref_y = goal.y + kReach * std::cos(heading);
    const float ref_move = std::hypot(ref_x - pressed.x, ref_y - pressed.y);
    if (against_goal && ref_move <= kGoalRefMaxIn) {
        if (reference_in) *reference_in = ref_move;
        chassis.setPose(ref_x, ref_y, pressed.theta);
    } else if (reference_in) {
        *reference_in = -1.0f;  // skipped
    }
    timing.release_ms += pros::millis() - releasing_since;
    coast_drive();
    return true;
}

// Forward out of a goal to `to`. Once the claw is clear, the lift and arm go
// back to where the loader needs them (setting a stack down has already
// started the lift there for everything but the first on a goal).
void leave_to(Point to, int held) {
    // Settled, like the three-pin's moves around the preload: a hand-over
    // still rolling made the next turn swing.
    calm_drive_to(to, true, true);
    chassis.waitUntil(kGoalClearanceIn);
    if (held == 0) three_pin::LiftJob to_loader{three_pin::LiftJob::Settle{}};
    if (with_arm(held)) ArmJob to_rest(arm_position::rest_deg);
    chassis.waitUntilDone();
    three_pin::brake_drive();
}

// With the claw empty: to the wall stack's staging point, back onto the
// stack and curve back out with it, the lift at the first level.
bool take_wall_pin(const WallPin& pin, four_pin_far::WallLog& log) {
    using namespace three_pin;
    go_to(pin.stage);
    ArmJob::wait();
    if (!LiftJob{LiftJob::Running{}}.wait()) return false;
    LiftJob down{LiftJob::Settle{}};
    turn_claw_to(pin.via);
    if (!down.wait()) return false;
    four_pin_far::take_wall_stack(pin.stack, pin.heading, pin.via,
                                 four_pin_far::kWallDeeperIn,
                                 lift_position::stage_1_deg, pin.out,
                                 pin.out_heading, log);
    return true;
}

void run() {
    using namespace three_pin;
    // Cooperative cancellation and join on every return. No background
    // raise/lower is allowed to outlive a failed Skills attempt.
    struct FinishJobs {
        ~FinishJobs() {
            chassis.cancelAllMotions();
            brake_drive();
            ArmJob::end();
            LiftJob::end();
        }
    } finish_jobs;
    ArmJob::end();
    LiftJob::end();
    mechanism_stop_reason = "";
    const std::uint32_t started = pros::millis();
    const auto now_s = [&] { return (pros::millis() - started) / 1000.0f; };
    if (!run_three_pin()) return;
    const float three_pin_s = now_s();
    set_lateral_kp(kLateralKp);
    coast_drive();

    constexpr int kLoads = kG2Loads + kG3Loads;
    static_assert(kG2Loads >= 0 && kOnG2AfterThreePin + kG2Loads <= kMostOnGoal);
    static_assert(kG3Loads >= 0 && kG3Loads <= kMostOnGoal);
    float toggle_s = 0.0f, park_s = 0.0f;
    int trips = 0, on_g2 = kOnG2AfterThreePin, on_g3 = 0;
    bool lowering_for_toggle = false;
    const char* stop_note = "";
    FixTally lined_up, at_loader;  // kept for take_load's signature
    // One line per trip on the report.
    struct Trip {
        int goal = 0;              // 2 or 3
        float seconds = 0.0f;
        float loader_ref = NAN;    // field X change at the loader contact
        float goal_ref = -1.0f;    // reset from the goal (-1 = skipped)
        WallFix wall[2];           // before the loader; before the G3 run
        TripTiming timing;
    };
    std::array<Trip, kLoads> trip{};
    // A wall reset in a few characters: the Y change, or why it was refused.
    const auto wall_short = [](const WallFix& w, char* out, int size) {
        const char* why = w.why;
        if (w.applied) {
            std::snprintf(out, size, "%+.1f", w.dy_in);
            return;
        }
        if (std::strcmp(why, "not run") == 0) why = "-";
        else if (std::strcmp(why, "not square") == 0) why = "sq";
        else if (std::strcmp(why, "scattered") == 0) why = "scat";
        else if (std::strcmp(why, "sensors disagree") == 0) why = "dis";
        else if (std::strcmp(why, "no reading") == 0) why = "none";
        else if (std::strcmp(why, "too far from odometry") == 0) why = "far";
        else if (std::strcmp(why, "moving or invalid pose") == 0) why = "move";
        std::snprintf(out, size, "%s", why);
    };
    // Shown after every step, so a run cut off part way still leaves its
    // numbers. Per trip: goal, seconds, loader contact change (field X), goal
    // reset (or skip), wall reset before the loader, wall reset before G3.
    const auto show = [&] {
        char text[1024];
        TripTiming waits;
        int used = std::snprintf(
            text, sizeof(text), "SKILLS %.1f s (3 pin %.1f)  G2 %d  G3 %d%s%s",
            now_s(), three_pin_s, on_g2, on_g3, *stop_note ? "  STOP " : "",
            stop_note);
        for (int i = 0; i < trips && used < static_cast<int>(sizeof(text)); ++i) {
            const Trip& t = trip[i];
            waits.wall_ms += t.timing.wall_ms;
            waits.down_ms += t.timing.down_ms;
            waits.stack_ms += t.timing.stack_ms;
            waits.pickup_ms += t.timing.pickup_ms;
            char w0[16], w1[16], g[16];
            wall_short(t.wall[0], w0, sizeof(w0));
            wall_short(t.wall[1], w1, sizeof(w1));
            if (t.goal_ref < 0.0f) std::snprintf(g, sizeof(g), "skip");
            else std::snprintf(g, sizeof(g), "%.1f", t.goal_ref);
            used += std::snprintf(
                text + used, sizeof(text) - used,
                "\nG%d #%d %.1fs ldr%+.1f goal %s wall %s %s", t.goal, i + 1,
                t.seconds, std::isnan(t.loader_ref) ? 0.0f : t.loader_ref, g,
                w0, w1);
        }
        if (used < static_cast<int>(sizeof(text)))
            std::snprintf(text + used, sizeof(text) - used,
                          "\nWait wall %.2fs  down %.2fs"
                          "\nWait up %.2fs  pickup %.2fs"
                          "\nToggle at %.1f s   parked at %.1f s",
                          waits.wall_ms / 1000.0f, waits.down_ms / 1000.0f,
                          waits.stack_ms / 1000.0f, waits.pickup_ms / 1000.0f,
                          toggle_s,
                          park_s);
        fieldviz::report_full(text);
    };
    // True if `seconds` more, and the toggle and park after it, still fit.
    // Off while the route is being tested, so every step runs and its time
    // shows; turn it on (kUseClock) once the cut list is decided.
    const auto fits = [&](float seconds) {
        return !kUseClock ||
               now_s() + seconds + (kToggleAndPark ? kEndgameS : 0.0f) <= kRunS;
    };

    // One match load. Both goals start the same way: into the loader from
    // the line-up spot (wall reset, contact reset), then nose first back out
    // to the line-up spot with the load just off the floor.
    //   G2: due north of the line-up spot. Claw north, raise, back straight
    //       in, come straight back out to the line-up spot.
    //   G3: east of kG3Lane, which is due north of the line-up spot. Nose
    //       south to the south wall for a second wall reset (G3's run is
    //       east-west, so this Y is its sideways place), back straight north
    //       to kG3Lane, claw east, raise, back in, come back out to kG3Lane.
    // Turns are made with the load low; the lift and arm go up after.
    const auto load = [&](bool to_g3) {
        Trip& t = trip[trips++];  // include a failed attempt in the report
        t.goal = to_g3 ? 3 : 2;
        const float began = now_s();
        const auto finished = [&](bool success) {
            t.seconds = now_s() - began;
            const TripTiming& m = t.timing;
            std::printf("SKILLS_PHASE,trip=%d,goal=%d,ok=%d,total_s=%.3f,"
                        "wall_ms=%lu,down_ms=%lu,stack_ms=%lu,pickup_ms=%lu,"
                        "loader_ms=%lu,aim_ms=%lu,goal_ms=%lu,release_ms=%lu,"
                        "return_ms=%lu\n", trips, t.goal, success, t.seconds,
                        static_cast<unsigned long>(m.wall_ms),
                        static_cast<unsigned long>(m.down_ms),
                        static_cast<unsigned long>(m.stack_ms),
                        static_cast<unsigned long>(m.pickup_ms),
                        static_cast<unsigned long>(m.loader_ms),
                        static_cast<unsigned long>(m.aim_ms),
                        static_cast<unsigned long>(m.goal_ms),
                        static_cast<unsigned long>(m.release_ms),
                        static_cast<unsigned long>(m.return_ms));
            return success;
        };
        int& on = to_g3 ? on_g3 : on_g2;
        const Point goal = to_g3 ? kG3 : kG2;
        lemlib::Pose stopped(0, 0, 0);
        if (!take_load(lined_up, at_loader, false, &stopped, &t.loader_ref,
                       &t.wall[0], t.timing))
            return finished(false);
        calm_drive_to(kLineUp, true);
        if (to_g3) {
            calm_turn_to_heading(field_heading(180.0f));
            const std::uint32_t resetting_since = pros::millis();
            t.wall[1] = south_wall_reset();
            t.timing.wall_ms += pros::millis() - resetting_since;
            calm_drive_to(kG3Lane, false);
        }
        const std::uint32_t aiming_since = pros::millis();
        const bool aimed = aim_for_score(goal);
        t.timing.aim_ms += pros::millis() - aiming_since;
        if (!aimed) {
            mechanism_failure("goal alignment");
            return finished(false);
        }
        if (!raise_for(on)) return finished(false);
        const bool final_arm_score = to_g3 && on + 1 == kG3Loads &&
                                     with_arm(on) && kToggleAndPark;
        if (!put_on(goal, on, &t.goal_ref, t.timing, final_arm_score))
            return finished(false);
        lowering_for_toggle = final_arm_score;
        const std::uint32_t returning_since = pros::millis();
        leave_to(to_g3 ? kG3Lane : kLineUp, on);
        t.timing.return_ms += pros::millis() - returning_since;
        ++on;
        finished(true);
        show();
        return true;
    };

    // The three-pin ends 8 in south of G2, nose south and rolling, on the
    // line to kLineUp. The lift comes all the way down: the claw takes the
    // load off the loader at floor height.
    if (!LiftJob{LiftJob::Running{}}.wait()) return;
    LiftJob to_loader{LiftJob::Settle{}};
    // A fresh command belongs to this attempt; a cancelled/failed arm job
    // from the last Skills run must not poison the first loader gate. The
    // opening has left the goal, so returning to rest can overlap this leg.
    ArmJob arm_to_loader(arm_position::rest_deg);

    // 1. MATCH LOADS ONTO G2 until it holds a pin and five cups (the
    //    three-pin left the pin and two).
    while (on_g2 < kOnG2AfterThreePin + kG2Loads) {
        if (!fits(kG2TripS)) { stop_note = "time (G2)"; break; }
        if (!load(false)) {
            stop_note = *mechanism_stop_reason ? mechanism_stop_reason : "lift (G2)";
            show(); return;
        }
    }
    // 2. MATCH LOADS ONTO G3: the first load is a pin (dropped like the
    //    three-pin's preload), then four cups at levels 1 to 4.
    while (on_g3 < kG3Loads) {
        if (!fits(kG3TripS)) { stop_note = "time (G3)"; break; }
        if (!load(true)) {
            stop_note = *mechanism_stop_reason ? mechanism_stop_reason : "lift (G3)";
            show(); return;
        }
    }

    // 3. TOGGLE AND PARK. From kG3Lane south to the line-up spot, east along
    //    the lane under G3 to the toggle, nose into the wall with the lift
    //    raised as in the three-pin (the toggle cannot be reached with it
    //    low), then back north claw first into the park.
    if (kToggleAndPark) {
        if (!lowering_for_toggle) {
            if (!LiftJob{LiftJob::Running{}}.wait()) {
                stop_note = "lift"; show(); return;
            }
            LiftJob for_toggle(kCarryDeg);
        }
        // The final direct lowering job can finish during this travel.
        go_to(kLineUp);
        calm_turn_to_point(kToggleLane, true);
        calm_drive_to(kToggleLane, true);
        calm_turn_to_heading(field_heading(180.0f));
        if (!LiftJob{LiftJob::Running{}}.wait()) {
            stop_note = "lift"; show(); return;
        }
        clamp_piston.set_value(true);
        coast_drive();
        chassis.moveToPoint(kToggleHit.x, kToggleHit.y, 900, {.forwards = true},
                            true);
        wait_done_or_stalled();
        toggle_s = now_s();
        calm_drive_to(kParkEntry, false);
        chassis.moveToPoint(kPark.x, kPark.y, 2000,
                            {.forwards = false, .maxSpeed = kCalmDrive,
                             .minSpeed = kGentleFloor}, true);
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
