// Disabled for now: the hand-set gains in main.cpp are being kept. To use the
// tuner again, change "#if 0" to "#if 1", restore its declaration in
// include/autons.hpp, and build with
//   pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=pid_autotune_auton
#if 0
#include "autons.hpp"
#include "main.h"
#include "subsystems.hpp"
#include "field_display_status.hpp"
#include "gps_reset/gps_reset.hpp"
#include "pid_autotune_model.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <new>
#include <vector>

// PID autotuner. It automates LemLib's own tuning procedure, on the real
// robot, with LemLib's real motions (turnToHeading / moveToPoint):
//   start from gentle gains, raise kP one step, and if the move then
//   overshoots, wobbles or brakes too hard, add kD; stop at the last gains
//   that were both smooth and faster.
// Every candidate is installed into the live chassis PID and driven with the
// same calls a route uses, so what is measured is what a route will get.
// Turning is tuned first because moveToPoint also steers with the angular PID.
//
// Needs about 3 ft of clear floor ahead of the robot and room to spin.
// Each move is measured relative to where it started, so slow IMU drift over
// the run does not accumulate into the results; drift is still measured and
// reported because a drifting IMU means a bad calibration.
namespace {
using pidtune::Gains;
using pidtune::Metrics;
using pidtune::Result;
using pidtune::Sample;

constexpr std::uint32_t kLoopMs = 10;
constexpr int kMoveTimeoutMs = 2500;
constexpr std::uint32_t kCoastMs = 300;
// Drivetrain geometry from main.cpp: 600 rpm motors geared to 450 rpm on
// 2.75 in wheels, 12 in track width.
constexpr double kInPerSecPerRpm = (450.0 / 600.0) * M_PI * 2.75 / 60.0;
constexpr double kTrackWidthIn = 12.0;

// How hard the robot may brake. These decide how aggressive the result is:
// raise them for faster, harsher stops; lower them if the robot still shakes
// or the wheels chirp. 170 in/s^2 is about 0.45 g.
constexpr double kLateralDecelLimit = 170.0;    // in/s^2
constexpr double kAngularDecelLimit = 2200.0;   // deg/s^2

struct Axis {
    const char* name;
    bool lateral;
    pidtune::Limits limits;
    pidtune::SearchConfig search;
    std::vector<double> targets;  // signed moves driven for every candidate
    Gains fallback;               // main.cpp's gains, restored on failure
    double runaway;               // cancel a move this far past its target
};

void stop_drive() {
    chassis.cancelAllMotions();
    left_motors.move(0);
    right_motors.move(0);
}

// lemlib::PID's gains are const, so "changing" them means rebuilding the
// object in place. LemLib exposes these two PIDs for exactly this purpose.
void install(const Axis& axis, Gains gains) {
    lemlib::PID& pid = axis.lateral ? chassis.lateralPID : chassis.angularPID;
    pid.~PID();
    new (&pid) lemlib::PID(static_cast<float>(gains.kP), 0.0f,
                           static_cast<float>(gains.kD), 0.0f, true);
}

bool fail(const Axis& axis, const char* reason) {
    stop_drive();
    install(axis, axis.fallback);
    fieldviz::report("PID AUTOTUNE\n\nFAILED (%s)\n%s", axis.name, reason);
    std::printf("AUTOTUNE,FAIL,%s,%s\n", axis.name, reason);
    return false;
}

// One real LemLib move of `amount` from wherever the robot is now.
// Returns false only for a fault that must end tuning.
bool run_move(const Axis& axis, double amount, Metrics& metrics) {
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(250);
    const lemlib::Pose start = chassis.getPose();
    const double heading = start.theta * M_PI / 180.0;
    // Wheel speed from the motors' own velocity estimate (see analyze()).
    auto velocity = [&]() -> double {
        const double left = left_motors.get_actual_velocity() * kInPerSecPerRpm;
        const double right = right_motors.get_actual_velocity() * kInPerSecPerRpm;
        if (axis.lateral) return 0.5 * (left + right);
        return (left - right) / kTrackWidthIn * 180.0 / M_PI;
    };
    auto position = [&]() -> double {
        const lemlib::Pose pose = chassis.getPose();
        if (!axis.lateral) return pose.theta - start.theta;
        return (pose.x - start.x) * std::sin(heading) +
               (pose.y - start.y) * std::cos(heading);
    };

    if (axis.lateral) {
        chassis.moveToPoint(start.x + amount * std::sin(heading),
                            start.y + amount * std::cos(heading),
                            kMoveTimeoutMs, {.forwards = amount > 0}, true);
    } else {
        chassis.turnToHeading(start.theta + amount, kMoveTimeoutMs, {}, true);
    }

    std::vector<Sample> samples;
    std::uint32_t now = pros::millis();
    const std::uint32_t began = now;
    double ended_s = -1.0;
    bool runaway = false;
    while (true) {
        const double seconds = (pros::millis() - began) / 1000.0;
        const double x = position();
        if (!std::isfinite(x)) return fail(axis, "sensor reading invalid");
        samples.push_back({seconds, x, velocity()});
        if (ended_s < 0.0) {
            if (std::abs(x) > std::abs(amount) + axis.runaway) {
                stop_drive();
                runaway = true;
                ended_s = seconds;
            } else if (seconds > 0.05 && !chassis.isInMotion()) {
                ended_s = seconds;
            } else if (seconds > kMoveTimeoutMs / 1000.0 + 1.0) {
                return fail(axis, "LemLib motion never ended");
            }
        } else if (seconds - ended_s >= kCoastMs / 1000.0) {
            break;
        }
        pros::Task::delay_until(&now, kLoopMs);
    }
    left_motors.move(0);
    right_motors.move(0);
    metrics = pidtune::analyze(samples, amount, ended_s, axis.limits.noise);
    // A cancelled move counts as a large overshoot, whatever was recorded.
    if (runaway) metrics.overshoot = std::max(metrics.overshoot, axis.runaway);
    return true;
}

bool fault = false;  // a move reported a fault and already called fail()

Result evaluate(const Axis& axis, Gains gains, int number) {
    if (number == 0)
        fieldviz::report("PID AUTOTUNE\n\n%s: measuring the\ncurrent gains\n"
                         "kP %.2f  kD %.1f", axis.name, gains.kP, gains.kD);
    else
        fieldviz::report("PID AUTOTUNE\n\n%s: test %d\nkP %.2f  kD %.1f",
                         axis.name, number, gains.kP, gains.kD);
    install(axis, gains);
    std::vector<Metrics> moves;
    for (double amount : axis.targets) {
        Metrics metrics;
        if (!run_move(axis, amount, metrics)) {
            fault = true;
            Result fatal;
            fatal.fatal = true;
            return fatal;
        }
        moves.push_back(metrics);
    }
    const Result r = pidtune::summarize(moves, axis.limits);
    std::printf("AUTOTUNE,TEST,%s,kP=%.3f,kD=%.3f,%s,cost=%.3f,time=%.2f,"
                "overshoot=%.2f,final=%.2f,decel=%.0f,crossings=%d\n",
                axis.name, gains.kP, gains.kD, r.acceptable ? "ok" : "rejected",
                r.cost, r.time_s, r.overshoot, r.final_error, r.peak_decel,
                r.crossings);
    return r;
}

// kept = true when nothing beat the gains already in main.cpp.
bool tune(const Axis& base_axis, pidtune::Outcome& outcome, bool& kept) {
    Axis axis = base_axis;
    const double timeout_s = kMoveTimeoutMs / 1000.0;

    // The gains already in main.cpp are the reference: braking up to 30%
    // harder than they produce is allowed (but never less than the absolute
    // limit), and the result has to beat them or they are kept.
    Result baseline = evaluate(axis, axis.fallback, 0);
    if (baseline.fatal) return false;
    axis.limits.decel = std::max(axis.limits.decel, 1.3 * baseline.peak_decel);
    baseline.acceptable = baseline.overshoot <= axis.limits.overshoot &&
                          baseline.crossings <= 1;
    const bool baseline_usable = pidtune::usable(baseline, axis.limits, timeout_s);
    std::printf("AUTOTUNE,LIMIT,%s,decel=%.0f,baseline_usable=%d\n", axis.name,
                axis.limits.decel, baseline_usable);

    // Climb from well below the current gains.
    axis.search.start = {0.6 * axis.fallback.kP, 0.6 * axis.fallback.kD};
    int number = 0;
    outcome = pidtune::tune_gains(axis.search, [&](Gains gains) {
        return evaluate(axis, gains, ++number);
    });
    if (fault) return false;  // already reported, gains already restored

    const bool tuned_usable =
        outcome.ok && pidtune::usable(outcome.result, axis.limits, timeout_s);
    kept = !tuned_usable ||
           (baseline_usable && baseline.cost <= outcome.result.cost);
    if (kept) {
        if (!baseline_usable)
            return fail(axis, "no gains tested were\nboth smooth and accurate");
        outcome.ok = true;
        outcome.gains = axis.fallback;
        outcome.result = baseline;
    }
    install(axis, outcome.gains);
    std::printf("AUTOTUNE,BEST,%s,kP=%.3f,kD=%.3f,tests=%d,%s\n", axis.name,
                outcome.gains.kP, outcome.gains.kD, outcome.evaluations,
                kept ? "kept_current" : "new");
    return true;
}

// Degrees per minute the heading moves while the robot is parked.
double measure_drift() {
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(500);
    const double before = chassis.getPose().theta;
    pros::delay(2000);
    return (chassis.getPose().theta - before) * 30.0;
}

// Chained moves with the final gains: out, turn, turn back, return. Odometry
// cannot see wheel slip, so where a GPS fix exists the physical return error
// is measured with it. Start and end headings match, which cancels any error
// in the GPS mounting offsets.
void chain_check(char* line, std::size_t size) {
    const lemlib::Pose start = chassis.getPose();
    const double heading = start.theta * M_PI / 180.0;
    gpsreset::GpsPose gps_before{}, gps_after{};
    const bool gps_ok = gpsreset::average_lens_pose(gps_before, 10, 50);

    chassis.moveToPoint(start.x + 24 * std::sin(heading),
                        start.y + 24 * std::cos(heading), kMoveTimeoutMs,
                        {}, false);
    chassis.turnToHeading(start.theta + 90, kMoveTimeoutMs, {}, false);
    chassis.turnToHeading(start.theta, kMoveTimeoutMs, {}, false);
    chassis.moveToPoint(start.x, start.y, kMoveTimeoutMs, {.forwards = false},
                        false);
    chassis.turnToHeading(start.theta, kMoveTimeoutMs, {}, false);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(400);

    const lemlib::Pose end = chassis.getPose();
    const double odom = std::hypot(end.x - start.x, end.y - start.y);
    if (gps_ok && gpsreset::average_lens_pose(gps_after, 10, 50)) {
        const double gps = std::hypot(gps_after.x_in - gps_before.x_in,
                                      gps_after.y_in - gps_before.y_in);
        std::snprintf(line, size, "Chain: odom %.1f GPS %.1f in", odom, gps);
    } else {
        std::snprintf(line, size, "Chain: odom %.1f in (no GPS)", odom);
    }
    std::printf("AUTOTUNE,CHAIN,%s\n", line);
}
}  // namespace

void pid_autotune_auton() {
    const Axis angular{
        "ANGULAR", false,
        {1.5, kAngularDecelLimit, 1.0, 0.15},
        {{1.0, 5.0}, 4.0},
        {90, -90, 160, -160},
        {2.0, 10.0},
        60.0};
    const Axis lateral{
        "LATERAL", true,
        {0.4, kLateralDecelLimit, 0.5, 0.05},
        {{4.0, 3.0}, 1.0},
        {24, -24, 8, -8},
        {6.0, 3.0},
        10.0};

    stop_drive();
    fault = false;
    std::printf("AUTOTUNE,START\n");
    fieldviz::report("PID AUTOTUNE\n\nChecking IMU drift.\nDo not touch the robot.");
    const double drift = measure_drift();
    std::printf("AUTOTUNE,DRIFT,%.2f deg/min\n", drift);

    pidtune::Outcome turn, straight;
    bool kept_turn = false, kept_straight = false;
    if (!tune(angular, turn, kept_turn)) {
        install(angular, angular.fallback);
        return;
    }
    if (!tune(lateral, straight, kept_straight)) {
        install(lateral, lateral.fallback);
        return;
    }

    fieldviz::report("PID AUTOTUNE\n\nChained-move check...");
    char chain[48];
    chain_check(chain, sizeof(chain));
    stop_drive();

    std::printf("AUTOTUNE,DONE\n"
                "lemlib::ControllerSettings lateral_controller(%.2f, 0, %.1f, 0, 1, 100, 3, 500, 0);\n"
                "lemlib::ControllerSettings angular_controller(%.2f, 0, %.1f, 0, 1, 100, 3, 500, 0);\n",
                straight.gains.kP, straight.gains.kD, turn.gains.kP,
                turn.gains.kD);
    fieldviz::report(
        "PID AUTOTUNE DONE\n"
        "LATERAL kP %.2f kD %.1f%s\n"
        " %.2fs over %.1f err %.1f in\n"
        "ANGULAR kP %.2f kD %.1f%s\n"
        " %.2fs over %.1f err %.1f deg\n"
        "%s\n"
        "IMU drift %.1f deg/min%s",
        straight.gains.kP, straight.gains.kD, kept_straight ? " KEPT" : "",
        straight.result.time_s,
        straight.result.overshoot, straight.result.final_error,
        turn.gains.kP, turn.gains.kD, kept_turn ? " KEPT" : "",
        turn.result.time_s,
        turn.result.overshoot, turn.result.final_error, chain, drift,
        std::abs(drift) > 3.0 ? " HIGH" : "");
}
#endif
