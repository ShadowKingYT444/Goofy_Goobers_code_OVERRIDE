#include "autons.hpp"
#include "main.h"
#include "subsystems.hpp"
#include "field_display_status.hpp"
#include "curve_motion.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// Motion tuner for the solo route (2026-10-18). Instead of tuning blind on
// the route, it drives solo's own kinds of motion with several settings and
// shows which were fastest while still accurate.
//
//   1. CURVE: the preload curve. Backward boomerang from (0, 0) heading 180
//      to (13, 15) heading 270 (a quarter turn while backing 15 in and
//      sliding 13 in to the robot's left), as solo drives it after the
//      toggle. Settings: lead, top speed, and power floor (0 = settle at the
//      end, 24 = end on reaching the point, as solo does now).
//   2. CHAIN: the run after the wall pin. From heading -28: turn to 90, drive
//      22 in, turn to 130.6, back 12 in. Settings: how the turns and drives
//      hand over (fully settled, or with power floors and early exits).
//
// Each trial re-zeroes odometry where it starts, runs, brakes, waits 0.25 s
// and measures where the robot came to rest: time, distance from the
// target, heading error, and overshoot (how far past the target along the
// way it was travelling; + is past). Then it drives back to its start.
// Cost = time + 0.5 x distance + 0.5 x overshoot past + 0.05 x heading.
// Odometry is the only measure: it shows overshoot, swing and time, not
// wheel slip.
//
// Build: pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=motion_tune_auton
// Run from driver control with LEFT; B stops it. About 2 minutes.
// Floor: about 3 ft x 3 ft clear, mostly behind the robot and to its left.
//
// 2026-10-18 first run: curve best 0.30/80/0 (now in solo). The chain part
// stopped after its first setting (B or runaway), so the curve part is off
// (kRunCurves) and the chain part runs alone, each run listed on its own,
// with why it stopped. Chain floor: about 2 ft clear to the robot's right
// (and a little behind it).
namespace {
constexpr bool kRunCurves = false;
constexpr float kRunawayIn = 50.0f;
bool aborted = false;
char stop_why[48] = "";

pros::Controller& pad() {
    static pros::Controller controller(pros::E_CONTROLLER_MASTER);
    return controller;
}
void brake_all() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.brake();
    right_motors.brake();
}
void coast_all() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}
// Wait for the current async motion, stopping everything on B or if the
// robot runs away from where the trial started.
bool watch() {
    while (chassis.isInMotion()) {
        const lemlib::Pose p = chassis.getPose();
        const bool b = pad().get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (b || std::hypot(p.x, p.y) > kRunawayIn) {
            if (b) std::snprintf(stop_why, sizeof(stop_why), "B pressed");
            else
                std::snprintf(stop_why, sizeof(stop_why),
                              "runaway at %.0f,%.0f", p.x, p.y);
            chassis.cancelAllMotions();
            brake_all();
            aborted = true;
            return false;
        }
        pros::delay(10);
    }
    return !aborted;
}

struct Trial {
    char name[24] = "";
    float time_s = 0, err_in = 0, head_deg = 0, over_in = 0;
    float cost() const {
        return time_s + 0.5f * err_in + 0.5f * std::max(0.0f, over_in) +
               0.05f * head_deg;
    }
};

// Where the robot came to rest, against target (x, y, heading). `backward`:
// it was travelling claw first, so "past the target" is behind its nose.
void measure(Trial& t, float x, float y, float heading, bool backward,
             std::uint32_t began) {
    t.time_s = (pros::millis() - began) / 1000.0f;
    brake_all();
    pros::delay(250);
    const lemlib::Pose r = chassis.getPose();
    t.err_in = std::hypot(r.x - x, r.y - y);
    t.head_deg = std::abs(std::remainder(r.theta - heading, 360.0f));
    const float h = heading * static_cast<float>(M_PI) / 180.0f;
    const float sign = backward ? -1.0f : 1.0f;
    t.over_in = (r.x - x) * sign * std::sin(h) + (r.y - y) * sign * std::cos(h);
}

// 1. CURVE --------------------------------------------------------------
struct CurveSet {
    float lead, speed, floor;
};
constexpr float kCurveX = 13.0f, kCurveY = 15.0f, kCurveH = 270.0f;

bool curve_trial(const CurveSet& c, Trial& t) {
    std::snprintf(t.name, sizeof(t.name), "%.2f/%.0f/%.0f", c.lead, c.speed,
                  c.floor);
    chassis.setPose(0, 0, 180);
    coast_all();
    const std::uint32_t began = pros::millis();
    chassis.moveToPose(kCurveX, kCurveY, kCurveH, 2500,
                       {.forwards = false,
                        .horizontalDrift = curve::tuned.drift,
                        .lead = c.lead, .maxSpeed = c.speed,
                        .minSpeed = c.floor}, true);
    if (!watch()) return false;
    measure(t, kCurveX, kCurveY, kCurveH, true, began);
    // Back to the start: the same curve forwards, then square up.
    coast_all();
    chassis.moveToPose(0, 0, 180, 3000,
                       {.forwards = true, .lead = 0.45f, .maxSpeed = 80}, true);
    if (!watch()) return false;
    chassis.turnToHeading(180, 900, {}, true);
    if (!watch()) return false;
    brake_all();
    pros::delay(150);
    return true;
}

// 2. CHAIN --------------------------------------------------------------
struct ChainSet {
    const char* name;
    int turn_floor;
    float turn_exit_deg;
    float drive_floor, drive_exit_in;
};
constexpr float kChainStartH = -28.0f, kChainTurn1 = 90.0f, kChainTurn2 = 130.6f;
constexpr float kChainDriveIn = 22.0f, kChainBackIn = 12.0f;

bool chain_trial(const ChainSet& c, Trial& t) {
    std::snprintf(t.name, sizeof(t.name), "%s", c.name);
    chassis.setPose(0, 0, kChainStartH);
    coast_all();
    const float h2 = kChainTurn2 * static_cast<float>(M_PI) / 180.0f;
    const float end_x = kChainDriveIn - kChainBackIn * std::sin(h2);
    const float end_y = -kChainBackIn * std::cos(h2);
    const std::uint32_t began = pros::millis();
    chassis.turnToHeading(kChainTurn1, 1200,
                          {.minSpeed = c.turn_floor,
                           .earlyExitRange = c.turn_exit_deg}, true);
    if (!watch()) return false;
    chassis.moveToPoint(kChainDriveIn, 0, 1500,
                        {.forwards = true, .minSpeed = c.drive_floor,
                         .earlyExitRange = c.drive_exit_in}, true);
    if (!watch()) return false;
    chassis.turnToHeading(kChainTurn2, 1200,
                          {.minSpeed = c.turn_floor,
                           .earlyExitRange = c.turn_exit_deg}, true);
    if (!watch()) return false;
    // The last leg settles, as a run into a goal ends on contact: every
    // setting is timed to the same kind of finish.
    chassis.moveToPoint(end_x, end_y, 1500, {.forwards = false}, true);
    if (!watch()) return false;
    measure(t, end_x, end_y, kChainTurn2, true, began);
    // Back to the start.
    coast_all();
    chassis.turnToPoint(0, 0, 1000, {}, true);
    if (!watch()) return false;
    chassis.moveToPoint(0, 0, 2000, {.maxSpeed = 80}, true);
    if (!watch()) return false;
    chassis.turnToHeading(kChainStartH, 1000, {}, true);
    if (!watch()) return false;
    brake_all();
    pros::delay(150);
    return true;
}

void show(const char* status, const std::vector<Trial>& curves,
          const std::vector<Trial>& chains) {
    char text[900];
    int used = std::snprintf(text, sizeof(text), "MOTION TUNE  %s", status);
    if (!curves.empty())
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\nCurve lead/speed/floor: s err over deg");
    std::vector<Trial> c = curves;
    std::sort(c.begin(), c.end(),
              [](const Trial& a, const Trial& b) { return a.cost() < b.cost(); });
    for (std::size_t i = 0; i < c.size() && i < 5; ++i)
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\n %s  %.2f %.1f %+.1f %.0f", c[i].name,
                              c[i].time_s, c[i].err_in, c[i].over_in,
                              c[i].head_deg);
    used += std::snprintf(text + used, sizeof(text) - used,
                          "\nChain runs: s err over deg");
    // Every run on its own line, so one odd run stands out.
    for (const Trial& t : chains)
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\n %-9s %.2f %.1f %+.1f %.0f", t.name, t.time_s,
                              t.err_in, t.over_in, t.head_deg);
    if (!chains.empty()) {
        const Trial* best = &chains[0];
        for (const Trial& t : chains)
            if (t.cost() < best->cost()) best = &t;
        used += std::snprintf(text + used, sizeof(text) - used,
                              "\nBest run: chain %s", best->name);
    }
    if (*stop_why)
        std::snprintf(text + used, sizeof(text) - used, "\nStopped: %s",
                      stop_why);
    fieldviz::report_full(text);
}
}  // namespace

void motion_tune_auton() {
    aborted = false;
    stop_why[0] = '\0';
    const CurveSet curve_sets[] = {
        {0.30f, 80, 0},  {0.30f, 110, 0},  {0.30f, 80, 24},  {0.30f, 110, 24},
        {0.45f, 80, 0},  {0.45f, 110, 0},  {0.45f, 80, 24},  {0.45f, 110, 24},
        {0.60f, 80, 0},  {0.60f, 110, 0},  {0.60f, 80, 24},  {0.60f, 110, 24},
    };
    const ChainSet chain_sets[] = {
        {"settled", 0, 0.0f, 0.0f, 0.0f},
        {"gentle", 10, 2.0f, 25.0f, 1.0f},
        {"solo", 20, 4.0f, 40.0f, 2.0f},   // what solo uses now
        {"fast", 30, 6.0f, 60.0f, 3.0f},
    };
    std::vector<Trial> curves, chains;
    show("starting", curves, chains);
    for (const CurveSet& c : curve_sets) {
        if (!kRunCurves) break;
        Trial t;
        if (!curve_trial(c, t)) return show("STOPPED (B or runaway)", curves, chains);
        curves.push_back(t);
        std::printf("MOTIONTUNE,CURVE,%s,%.3f,%.2f,%.2f,%.1f\n", t.name, t.time_s,
                    t.err_in, t.over_in, t.head_deg);
        show("curves", curves, chains);
    }
    for (int round = 0; round < 2; ++round)
        for (const ChainSet& c : chain_sets) {
            Trial t;
            if (!chain_trial(c, t))
                return show("STOPPED (B or runaway)", curves, chains);
            chains.push_back(t);
            std::printf("MOTIONTUNE,CHAIN,%s,%.3f,%.2f,%.2f,%.1f\n", t.name,
                        t.time_s, t.err_in, t.over_in, t.head_deg);
            show("chains", curves, chains);
        }
    show("done", curves, chains);
}
