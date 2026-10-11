#include "autons.hpp"
#include "main.h"
#include "subsystems.hpp"
#include "field_display_status.hpp"
#include "gps_reset/gps_reset.hpp"
#include "curve_motion.hpp"
#include "curve_tune_model.hpp"
#include <atomic>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

// Curve tuner. Drives curve::to() (include/curve_motion.hpp), the curved
// drive routes use, with different settings and reports which were fastest,
// most accurate and smoothest. It tunes that move's three knobs (see
// include/curve_tune_model.hpp): `straight_in`, `lead` and `drift`. The PID
// gains in main.cpp are not touched.
//
// For each setting the robot drives two out-and-back pairs from where it
// started: a wide curve to the right and a tight one to the left, each out
// nose-first and back claw-first, so both directions and both tightnesses
// count. After the pairs it squares up at the start and the GPS is compared
// with odometry there: the robot is facing the way it started, so the GPS
// mount offsets cancel and the gap is sideways sliding the encoders missed.
// The best setting is then driven again against the current one and only
// recommended if it wins a second time.
//
// Build:  pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=curve_autotune_auton
// Run from driver control with LEFT; B aborts. Takes about two minutes.
// Nothing is saved: the result is shown on the Brain and printed.
//
// Floor needed: about 3 ft clear ahead of the robot and 2.5 ft to each side.
namespace {
using curvetune::Move;
using curvetune::Result;
using curvetune::Sample;
using curvetune::Settings;

constexpr std::uint32_t kLoopMs = 10;
constexpr int kMoveTimeoutMs = 3500;
constexpr std::uint32_t kRestMs = 250;
constexpr float kMaxSpeed = 100.0f;
// The two test curves: {x, y, final heading} from the start pose (0, 0, 0).
struct Target {
    float x, y, heading;
};
constexpr Target kWide{18.0f, 24.0f, 90.0f};
constexpr Target kTight{-12.0f, 14.0f, -90.0f};
constexpr float kRunawayIn = 45.0f;  // stop everything this far from the start
const double kStraightIns[] = {10.0, 14.0};
const double kLeads[] = {0.3, 0.6};
const double kDrifts[] = {10.0, 4.0};

bool fault = false;

void stop_drive() {
    chassis.cancelAllMotions();
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.brake();
    right_motors.brake();
}

void fail(const char* reason) {
    stop_drive();
    fault = true;
    std::printf("CURVETUNE,FAILED,%s\n", reason);
    fieldviz::report("CURVE TUNE\n\nSTOPPED\n%s", reason);
}

curve::Settings motion(Settings s) {
    curve::Settings m;
    m.lead = static_cast<float>(s.lead);
    m.drift = static_cast<float>(s.drift);
    m.straight_in = static_cast<float>(s.straight_in);
    m.max_speed = kMaxSpeed;
    return m;
}

// One measured curve to `target`. curve::to() blocks, so it runs in its own
// task while this one samples the gyro and watches for an abort.
bool drive_curve(Settings s, Target target, bool forwards, Move& move) {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    std::vector<Sample> samples;
    samples.reserve(kMoveTimeoutMs / kLoopMs + 8);
    static std::atomic<bool> finished{true};
    finished = false;
    const curve::Settings m = motion(s);
    const std::uint32_t began = pros::millis();
    pros::Task([=] {
        curve::to(target.x, target.y, target.heading, forwards, kMoveTimeoutMs, m);
        finished = true;
    });
    const char* stopped = nullptr;
    while (!finished) {
        if (!stopped) {
            const lemlib::Pose pose = chassis.getPose();
            const double rate = imu.get_gyro_rate().z;
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
                stopped = "Aborted with B";
            else if (std::hypot(pose.x, pose.y) > kRunawayIn)
                stopped = "Robot left the test area";
            else if (!std::isfinite(rate))
                stopped = "IMU not reading";
            else
                samples.push_back({(pros::millis() - began) / 1000.0, rate});
        }
        // Keep cancelling until the move's task has wound down, so its
        // second leg cannot start after the first was cancelled.
        if (stopped) chassis.cancelAllMotions();
        pros::delay(kLoopMs);
    }
    if (stopped) {
        fail(stopped);
        return false;
    }
    const std::uint32_t elapsed = pros::millis() - began;
    stop_drive();
    pros::delay(kRestMs);
    const lemlib::Pose rest = chassis.getPose();
    move.time_s = elapsed / 1000.0;
    move.timed_out = elapsed + 30 >= static_cast<std::uint32_t>(kMoveTimeoutMs);
    move.position_error_in = std::hypot(rest.x - target.x, rest.y - target.y);
    move.heading_error_deg =
        std::abs(std::remainder(rest.theta - target.heading, 360.0f));
    move.roughness = curvetune::roughness(samples);
    move.reversals = curvetune::reversals(samples);
    return true;
}

// Both out-and-back pairs with one group of settings, then the GPS check at
// the start, which also re-homes odometry for the next candidate.
bool evaluate(Settings s, int number, int total, Result& result) {
    fieldviz::report("CURVE TUNE\n\nTest %d of about %d\n"
                     "straight %.0f in\nlead %.2f  drift %.0f\n\n"
                     "B on the controller\nstops the test.",
                     number, total, s.straight_in, s.lead, s.drift);
    std::vector<Move> moves(4);
    const Target targets[] = {kWide, kTight};
    for (int i = 0; i < 2; ++i) {
        if (!drive_curve(s, targets[i], true, moves[2 * i])) return false;
        if (!drive_curve(s, {0.0f, 0.0f, 0.0f}, false, moves[2 * i + 1]))
            return false;
    }
    chassis.turnToHeading(0, 900, {}, false);
    stop_drive();
    const gpsreset::Correction fix = gpsreset::checkpoint(0.0, 0.0, 10.0, 700);
    const bool measured = fix.status == gpsreset::Fix::APPLIED ||
                          fix.status == gpsreset::Fix::ALREADY_ALIGNED;
    result = curvetune::summarize(s, moves, measured ? fix.gap_in : -1.0);
    std::printf("CURVETUNE,RESULT,%.0f,%.2f,%.1f,%.2f,%.2f,%.1f,%.1f,%.0f,%d,%.1f\n",
                s.straight_in, s.lead, s.drift, result.cost, result.time_s,
                result.position_error_in, result.heading_error_deg,
                result.roughness, result.reversals, result.slip_in);
    for (const Move& m : moves)
        std::printf("CURVETUNE,MOVE,%.2f,%.1f,%.1f,%.0f,%d,%d\n", m.time_s,
                    m.position_error_in, m.heading_error_deg, m.roughness,
                    m.reversals, m.timed_out ? 1 : 0);
    return true;
}
}  // namespace

void curve_autotune_auton() {
    fault = false;
    gpsreset::set_auto_reset(false);
    chassis.setPose(0, 0, 0);
    // The slip measurement needs a GPS start anchor; without one the tuner
    // still runs on time, accuracy and smoothness alone.
    const bool anchored = gpsreset::capture_start_as(0, 0, 0);
    std::printf("CURVETUNE,START,gps_anchor=%d\n", anchored ? 1 : 0);

    // The settings routes use today are the baseline to beat.
    const Settings baseline{curve::tuned.lead, curve::tuned.drift,
                            curve::tuned.straight_in};
    const int total = 3 + std::size(kStraightIns) + std::size(kLeads) +
                      std::size(kDrifts);
    std::vector<Result> results;
    int number = 0;
    auto run = [&](Settings s) {
        Result r;
        if (!evaluate(s, ++number, total, r)) return false;
        results.push_back(r);
        return true;
    };
    auto best_so_far = [&] { return results[curvetune::choose(results, 0)]; };

    // One knob at a time, each starting from the best found so far: first
    // whether a straight finish helps, then how wide to swing, then how much
    // speed to hold in the curve.
    if (!run(baseline)) return;
    for (double straight_in : kStraightIns)
        if (straight_in != baseline.straight_in &&
            !run({baseline.lead, baseline.drift, straight_in}))
            return;
    Settings from = best_so_far().settings;
    for (double lead : kLeads)
        if (lead != from.lead && !run({lead, from.drift, from.straight_in}))
            return;
    from = best_so_far().settings;
    for (double drift : kDrifts)
        if (drift != from.drift && !run({from.lead, drift, from.straight_in}))
            return;

    // Drive the winner and the baseline once more; the winner is only
    // recommended if it wins again.
    const int winner = curvetune::choose(results, 0);
    const Result before = results[0];
    Result best = before;
    if (winner != 0) {
        Result baseline_again, winner_again;
        if (!evaluate(baseline, ++number, total, baseline_again)) return;
        if (!evaluate(results[winner].settings, ++number, total, winner_again))
            return;
        if (curvetune::confirmed(before, baseline_again, results[winner],
                                 winner_again))
            best = winner_again;
        std::printf("CURVETUNE,CONFIRM,%.2f,%.2f,%s\n", baseline_again.cost,
                    winner_again.cost,
                    best.settings.lead == before.settings.lead &&
                            best.settings.drift == before.settings.drift &&
                            best.settings.straight_in ==
                                before.settings.straight_in
                        ? "rejected" : "kept");
    }
    const bool changed = best.settings.lead != before.settings.lead ||
                         best.settings.drift != before.settings.drift ||
                         best.settings.straight_in != before.settings.straight_in;

    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    std::printf("CURVETUNE,BEST,%.0f,%.2f,%.1f,%.2f,baseline_cost=%.2f\n",
                best.settings.straight_in, best.settings.lead,
                best.settings.drift, best.cost, before.cost);
    char slip[24] = "no GPS";
    if (best.slip_in >= 0.0)
        std::snprintf(slip, sizeof(slip), "%.1f in", best.slip_in);
    fieldviz::report(
        "CURVE TUNE DONE\n%s\nstraight %.0f in\nlead %.2f  drift %.0f\n\n"
        "        was    best\n"
        "time   %4.1f   %4.1f s\n"
        "miss   %4.1f   %4.1f in\n"
        "angle  %4.1f   %4.1f deg\n"
        "rough  %4.0f   %4.0f\n"
        "slip %s",
        changed ? "Use:" : "Keep current:", best.settings.straight_in,
        best.settings.lead, best.settings.drift, before.time_s, best.time_s,
        before.position_error_in, best.position_error_in,
        before.heading_error_deg, best.heading_error_deg, before.roughness,
        best.roughness, slip);
}
