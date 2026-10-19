"""Execute current skills scoring functions with scripted hardware.

The C++ under test is extracted from autons.cpp, not a second implementation.
Only PROS, chassis motion and background workers are replaced. The real
skills_control.hpp supplies the physical predicates and deadline loops.
"""

from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def code_mask(source):
    """Hide comments/ordinary literals so braces inside them are not code."""
    pattern = r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    return re.sub(
        pattern,
        lambda match: "".join("\n" if c == "\n" else " " for c in match[0]),
        source,
        flags=re.DOTALL,
    )


def extract_block(source, pattern, *, semicolon=False):
    masked = code_mask(source)
    matches = list(re.finditer(pattern, masked, re.MULTILINE))
    if len(matches) != 1:
        raise AssertionError(
            f"Expected one production block matching {pattern!r}; "
            f"found {len(matches)}. Update the harness for the changed API."
        )
    start = matches[0].start()
    opening = masked.find("{", matches[0].end())
    if opening < 0:
        raise AssertionError(f"Missing production body for {pattern!r}")
    depth = 0
    for at in range(opening, len(masked)):
        if masked[at] == "{":
            depth += 1
        elif masked[at] == "}":
            depth -= 1
            if depth == 0:
                end = at + 1
                if semicolon:
                    end = masked.index(";", end) + 1
                return source[start:end]
    raise AssertionError(f"Unbalanced production body for {pattern!r}")


def declaration(source, name):
    masked = code_mask(source)
    pattern = (rf"^(?:constexpr|const) [\w:]+(?:[ \t]*\*)?"
               rf"[ \t]+{re.escape(name)}\b[^=]*=")
    matches = list(re.finditer(pattern, masked, re.MULTILINE))
    if len(matches) != 1:
        raise AssertionError(f"Expected one production constant {name}")
    start = matches[0].start()
    return source[start:masked.index(";", matches[0].end()) + 1]


PRELUDE = r'''
#include "skills_control.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using skills_control::ArmOutcome;
constexpr std::int32_t PROS_ERR = std::numeric_limits<std::int32_t>::max();

namespace sim {
struct LiftCommand {
    bool settle;
    double target;
    bool fast;
    std::uint32_t at;
};
struct ArmCommand { double target; std::uint32_t at; };
struct State {
    std::uint32_t tick = 0;
    double lift = 0, arm = 0;
    int lift_state = 1;  // 0 running, 1 success, -1 failure
    ArmOutcome arm_outcome = ArmOutcome::reached;
    bool claw_closed = true, motion = false;
    int moves = 0, stops = 0, cancelled_moves = 0, contact_calls = 0;
    int lift_ends = 0;
    std::vector<LiftCommand> lifts;
    std::vector<ArmCommand> arms;
    std::vector<std::uint32_t> opened_at;
    std::function<void()> after_delay, at_contact;
    std::function<void(bool, double, bool)> lift_started;
    std::function<void(double)> arm_started;
} state;
void reset() { state = State{}; }
void advance(std::uint32_t ms) {
    state.tick += ms;
    assert(state.tick <= 10000 && "A production wait did not terminate");
    if (state.after_delay) state.after_delay();
}
}  // namespace sim

namespace pros {
std::uint32_t millis() { return sim::state.tick; }
void delay(std::uint32_t ms) { sim::advance(ms); }
}
struct Sensor {
    bool arm;
    std::int32_t get_position() const {
        const double value = arm ? sim::state.arm : sim::state.lift;
        return std::isfinite(value)
                   ? static_cast<std::int32_t>(std::llround(value * 100.0))
                   : PROS_ERR;
    }
};
Sensor lift_sensor{false}, claw_sensor{true};
struct Claw {
    void set_value(bool closed) {
        sim::state.claw_closed = closed;
        if (!closed) sim::state.opened_at.push_back(pros::millis());
    }
} claw_piston;
namespace lemlib {
struct Pose { float x = 0, y = 0, theta = 0; };
struct MoveToPointParams {
    bool forwards = true;
    float maxSpeed = 127, minSpeed = 0;
};
struct MoveToPoseParams {
    bool forwards = true;
    float horizontalDrift = 0, lead = 0.6f, maxSpeed = 127, minSpeed = 0;
    float earlyExitRange = 0;
};
}
struct Chassis {
    lemlib::Pose getPose() const { return {}; }
    void setPose(float, float, float) {}
    void moveToPoint(float, float, int, lemlib::MoveToPointParams, bool) {
        ++sim::state.moves;
        sim::state.motion = true;
    }
    void moveToPose(float, float, float, int, lemlib::MoveToPoseParams, bool) {
        ++sim::state.moves;
        sim::state.motion = true;
    }
    void waitUntil(double) {}
    bool isInMotion() const { return sim::state.motion; }
    void cancelMotion() {
        sim::state.motion = false;
        ++sim::state.cancelled_moves;
    }
    void cancelAllMotions() {
        sim::state.motion = false;
        ++sim::state.stops;
    }
} chassis;
'''


LIFT_FAKE = r'''
class LiftJob {
public:
    struct Running {};
    struct Settle {};
    explicit LiftJob(Running) {}
    explicit LiftJob(Settle) { start(true, NAN, false); }
    explicit LiftJob(double target, bool fast = false) {
        start(false, target, fast);
    }
    bool done() const { return sim::state.lift_state != 0; }
    // Cancel and join: a running worker ends as a failure.
    static void end() {
        if (sim::state.lift_state == 0) sim::state.lift_state = -1;
        ++sim::state.lift_ends;
    }
    bool succeeded() const { return sim::state.lift_state == 1; }
    bool failed() const { return sim::state.lift_state == -1; }
    bool wait() {
        while (!done()) pros::delay(10);
        return succeeded();
    }
private:
    void start(bool settle, double target, bool fast) {
        assert(sim::state.lift_state != 0 &&
               "Two lift workers would own the motors at the same time");
        sim::state.lifts.push_back({settle, target, fast, pros::millis()});
        sim::state.lift_state = 0;
        if (sim::state.lift_started)
            sim::state.lift_started(settle, target, fast);
    }
};
void brake_drive() {}
void coast_drive() {}
struct PastLine {
    float x, y, ux, uy, at_in;
};
bool wait_done_or_stalled(std::uint32_t, std::uint32_t = 150, double = 12.0,
                          std::uint32_t = 300, const PastLine* = nullptr) {
    ++sim::state.contact_calls;
    if (sim::state.at_contact) sim::state.at_contact();
    sim::state.motion = false;
    return true;
}
'''


ARM_FAKE = r'''
class ArmJob {
public:
    explicit ArmJob(double target, std::uint32_t = 1500) {
        sim::state.arms.push_back({target, pros::millis()});
        sim::state.arm_outcome = ArmOutcome::running;
        if (sim::state.arm_started) sim::state.arm_started(target);
    }
    static bool done() { return sim::state.arm_outcome != ArmOutcome::running; }
    static ArmOutcome result() { return sim::state.arm_outcome; }
    static bool failed() {
        return sim::state.arm_outcome == ArmOutcome::cancelled ||
               sim::state.arm_outcome == ArmOutcome::sensor_error;
    }
};
'''


CHECKS = r'''
using namespace skills_route;

void at_score(int held) {
    sim::reset();
    sim::state.lift = kLevels[held].lift_deg;
    sim::state.arm = kLevels[held].arm_deg;
}

void loader_clearance() {
    TripTiming timing;
    sim::reset();
    sim::state.lift = -140;  // task ended above the loader clearance
    sim::state.arm = 22;
    sim::state.arm_outcome = ArmOutcome::timed_out;
    assert(!wait_until_down(timing));
    assert(sim::state.tick == 0 && sim::state.stops == 1);

    sim::reset();
    sim::state.lift = -83;
    sim::state.arm = 22;
    sim::state.arm_outcome = ArmOutcome::timed_out;
    assert(wait_until_down(timing));  // measured inside loader's 25 degrees
    assert(sim::state.tick == 0);

    for (ArmOutcome failure : {ArmOutcome::cancelled, ArmOutcome::sensor_error}) {
        sim::reset();
        sim::state.arm_outcome = failure;
        assert(!wait_until_down(timing));  // even with both sensors at rest
        assert(sim::state.stops == 1);
    }
    sim::reset();
    sim::state.arm = 26;
    sim::state.arm_outcome = ArmOutcome::timed_out;
    assert(!wait_until_down(timing));

    for (bool arm_sensor : {false, true}) {
        sim::reset();
        if (arm_sensor) sim::state.arm = NAN;
        else sim::state.lift = NAN;
        assert(!wait_until_down(timing));  // exercises actual PROS_ERR adapters
    }

    sim::reset();
    sim::state.lift = -140;
    sim::state.arm = 35;
    sim::state.lift_state = 0;
    sim::state.arm_outcome = ArmOutcome::running;
    sim::state.after_delay = [] {
        if (pros::millis() >= 30) {
            sim::state.lift = -83;
            sim::state.arm = 25;
        }
    };
    assert(wait_until_down(timing));
    assert(sim::state.tick == 30 && sim::state.lift_state == 0);
}

void raise_ownership_and_rest() {
    sim::reset();
    sim::state.lift_state = 0;  // the prior pickup still owns the lift
    sim::state.arm = 22;
    sim::state.after_delay = [] {
        if (pros::millis() >= 30) sim::state.lift_state = 1;
    };
    assert(raise_for(1));
    assert(sim::state.tick == 30);
    assert(sim::state.lifts.size() == 1 && sim::state.arms.size() == 1);
    assert(sim::state.lifts[0].target == kLevels[1].lift_deg);
    assert(sim::state.lifts[0].fast && !sim::state.lifts[0].settle);
    assert(sim::state.arms[0].target == arm_position::rest_deg);

    sim::reset();
    sim::state.lift_state = -1;
    assert(!raise_for(1));
    assert(sim::state.lifts.empty() && sim::state.arms.empty());
}

void score_failure_keeps_claw() {
    if (kStackGates) for (ArmOutcome failure : {ArmOutcome::cancelled, ArmOutcome::sensor_error}) {
        at_score(1);
        sim::state.arm_outcome = failure;
        TripTiming timing;
        assert(!put_on({0, 30}, 1, nullptr, timing));
        assert(sim::state.claw_closed && sim::state.opened_at.empty());
        assert(sim::state.lifts.empty() && sim::state.stops == 1);
    }
    if (!kStackGates) {  // height checks off (2026-10-18): nothing at the goal waits
        // A failed raise no longer stops the score: the lift is taken to be
        // at height and the stack is set down.
        at_score(1);
        sim::state.lift_state = -1;
        TripTiming t2;
        assert(put_on({0, 30}, 1, nullptr, t2));
        assert(!sim::state.claw_closed && sim::state.lifts.size() == 1);
        // A raise still running is stopped, not waited for.
        at_score(1);
        sim::state.lift_state = 0;
        TripTiming t3;
        assert(put_on({0, 30}, 1, nullptr, t3));
        assert(sim::state.lift_ends == 1 && sim::state.lifts.size() == 1);
        // The lowering starts once the robot has settled on the goal.
        assert(sim::state.lifts[0].settle &&
               sim::state.lifts[0].at == kGoalSettleMs);
        return;
    }
    at_score(1);
    sim::state.arm = 22;  // legal for the loader, outside score's 20 degrees
    sim::state.arm_outcome = ArmOutcome::timed_out;
    TripTiming timing;
    // One retry of the arm move, then the shared deadline; the claw stays
    // closed and nothing starts lowering.
    assert(!put_on({0, 30}, 1, nullptr, timing));
    assert(sim::state.claw_closed && sim::state.tick == kStackWaitMs);
    assert(sim::state.arms.size() == 1 &&
           sim::state.arms[0].target == arm_position::rest_deg);

    at_score(1);
    sim::state.lift_state = -1;
    assert(!put_on({0, 30}, 1, nullptr, timing));
    assert(sim::state.claw_closed && sim::state.lifts.empty());
}

void score_timeout_within_tolerance() {
    at_score(1);
    sim::state.arm = 18;
    sim::state.arm_outcome = ArmOutcome::timed_out;
    const double scored_at = sim::state.lift;
    sim::state.after_delay = [scored_at] {
        if (pros::millis() >= kGoalSettleMs + 20)
            sim::state.lift = scored_at + three_pin::kSeatDropDeg + 0.01;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 1, nullptr, timing));
    assert(!sim::state.claw_closed &&
           sim::state.opened_at[0] == kGoalSettleMs + 20);
    assert(sim::state.tick == kGoalSettleMs + 20 + kPneumaticSettleMs);
    assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
}

void one_score_deadline() {
    if (!kStackGates) return;  // height checks off (2026-10-18)
    at_score(1);
    sim::state.lift_state = 0;
    // Fault injection: a worker never publishes completion. The scoring
    // deadline must still bound both waits, independently of worker timing.
    sim::state.arm_outcome = ArmOutcome::running;
    sim::state.after_delay = [] {
        if (pros::millis() >= 2200) sim::state.lift_state = 1;
    };
    sim::state.at_contact = [] {
        sim::advance(200);
        sim::state.arm = 22;  // moved outside the gate during final approach
    };
    TripTiming timing;
    assert(!put_on({0, 30}, 1, nullptr, timing));
    assert(kStackWaitMs == 4000);
    assert(sim::state.tick == 4000);  // 2200 wait + 200 drive + 1600 wait
    assert(timing.stack_ms == 3800);
    assert(sim::state.moves == 2 && sim::state.contact_calls == 1);
    assert(sim::state.claw_closed && sim::state.lifts.empty());
}

// 2026-10-18: the arm levels (4th and 5th cups) are set down like the
// others: drop a third of a level, open, lift down. No arm move at the goal
// (the RIGHT macro lowered the arm first and swung the claw past the goal);
// the arm goes back to rest in leave_to().
void arm_release_failure() {
    at_score(4);
    const double scored_at = sim::state.lift;
    sim::state.after_delay = [scored_at] {
        if (pros::millis() >= kGoalSettleMs + 40)
            sim::state.lift = scored_at + three_pin::kSeatDropDeg + 0.01;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 4, nullptr, timing));
    assert(!sim::state.claw_closed && sim::state.opened_at.size() == 1);
    assert(sim::state.opened_at[0] == kGoalSettleMs + 40);
    assert(sim::state.arms.empty());
    assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
    assert(sim::state.lifts[0].at == kGoalSettleMs);
}

void ordinary_seat_failure() {
    // Timeout, failed worker, successful-but-short worker, and sensor loss.
    for (int scenario = 0; scenario < 4; ++scenario) {
        at_score(1);
        const double scored_at = sim::state.lift;
        sim::state.after_delay = [scenario, scored_at] {
            if (pros::millis() < kGoalSettleMs + 40) return;
            if (scenario == 1) {
                sim::state.lift_state = -1;
                sim::state.lift = scored_at + three_pin::kSeatDropDeg + 0.01;
            } else if (scenario == 2) sim::state.lift_state = 1;
            else if (scenario == 3) sim::state.lift = NAN;
        };
        // The claw opens anyway: after 800 ms, or as soon as the lowering
        // ends early or the sensor drops out (the behaviour before the PR).
        TripTiming timing;
        assert(put_on({0, 30}, 1, nullptr, timing));
        assert(!sim::state.claw_closed && sim::state.opened_at.size() == 1);
        assert(sim::state.opened_at[0] ==
               kGoalSettleMs + (scenario == 0 ? 800u : 40u));
        assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
    }
}

// An arm that stopped 30 degrees short of 530 gets one retry; once it is
// there the score goes ahead.
void arm_retry_then_score() {
    if (!kStackGates) return;  // height checks off (2026-10-18)
    at_score(4);
    sim::state.arm = kLevels[4].arm_deg - 30;
    sim::state.arm_outcome = ArmOutcome::timed_out;
    sim::state.after_delay = [] {
        if (sim::state.arms.size() == 1 && pros::millis() >= 300 &&
            sim::state.arm_outcome == ArmOutcome::running) {
            sim::state.arm = kLevels[4].arm_deg - 5;
            sim::state.arm_outcome = ArmOutcome::reached;
        }
        if (sim::state.arms.size() == 2 && pros::millis() >= 400)
            sim::state.arm = arm_position::release_deg + 150;  // past halfway
    };
    TripTiming timing;
    assert(put_on({0, 30}, 4, nullptr, timing));
    assert(sim::state.arms.size() == 2);
    assert(sim::state.arms[0].target == kLevels[4].arm_deg);
    assert(sim::state.arms[1].target == arm_position::release_deg);
    assert(sim::state.opened_at.size() == 1);
}

// raise_for(take_over): a pre-raise still running is cancelled and the full
// raise starts at once, without waiting for it.
void raise_takes_over() {
    sim::reset();
    sim::state.lift_state = 0;  // pre-raise still running
    assert(raise_for(1, true));
    assert(sim::state.tick == 0 && sim::state.lift_ends == 1);
    assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].fast);
    assert(sim::state.lifts[0].target == kLevels[1].lift_deg);
    assert(sim::state.arms.size() == 1);
}

void final_score_carry_sequence() {
    for (bool last_load : {false, true}) {
        at_score(4);
        const double scored_at = sim::state.lift;
        sim::state.after_delay = [scored_at] {
            if (pros::millis() >= kGoalSettleMs + 40)
                sim::state.lift = scored_at + three_pin::kSeatDropDeg + 0.01;
        };
        TripTiming timing;
        assert(put_on({0, 30}, 4, nullptr, timing, last_load));
        assert(sim::state.opened_at.size() == 1 &&
               sim::state.opened_at[0] == kGoalSettleMs + 40);
        assert(sim::state.arms.empty());
        assert(sim::state.lifts.size() == 1);
        const auto& next = sim::state.lifts[0];
        // The lowering starts once settled; the claw opens on the way down.
        assert(next.at == kGoalSettleMs && !next.fast);
        if (last_load) assert(!next.settle && next.target == kCarryDeg);
        else assert(next.settle);
    }
}

// 2026-10-18: with the gates off, the robot holds short of the goal only
// while the stack is really not up (here: the top level still climbing),
// and goes on once the lift is within kLiftNearDeg of its height. The lift
// lock's band (95% of travel) no longer counts as up: from there the load
// clipped the top of the goal's stack.
void light_hold_until_up() {
    if (kStackGates) return;
    at_score(5);
    sim::state.lift = -1600;  // in the lock's band, 62 below the height
    sim::state.lift_state = 0;
    sim::state.after_delay = [] {
        if (pros::millis() >= 300) sim::state.lift = kLevels[5].lift_deg + 5;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 5, nullptr, timing));
    assert(sim::state.moves == 2 && sim::state.cancelled_moves == 1);
    assert(timing.stack_ms >= 300 && timing.stack_ms <= 310);
    assert(sim::state.lift_ends == 1);  // the raise is stopped at contact
    // Already up: no hold at all.
    at_score(5);
    TripTiming t2;
    assert(put_on({0, 30}, 5, nullptr, t2));
    // (One cancel and a second move: the hand-over to the slow last
    // stretch, kGoalFinalSpeed.)
    assert(sim::state.moves == 2 && sim::state.cancelled_moves == 1 &&
           t2.stack_ms == 0);
}

// 2026-10-18: the claw opens after half the old drop (kReleaseDropDeg, a
// sixth of a level), after the robot has settled on the goal.
void half_drop_release() {
    at_score(2);
    const double scored_at = sim::state.lift;
    sim::state.after_delay = [scored_at] {
        if (pros::millis() >= kGoalSettleMs + 30)
            sim::state.lift = scored_at + kReleaseDropDeg + 0.01;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 2, nullptr, timing));
    assert(sim::state.opened_at.size() == 1 &&
           sim::state.opened_at[0] == kGoalSettleMs + 30);
    assert(kReleaseDropDeg < three_pin::kSeatDropDeg * 0.51);
}

// 2026-10-18: a hold for a lift that never comes up ends at kHoldCapMs
// (no more 3 s stops); the robot goes on and the report counts it.
void hold_is_capped() {
    if (kStackGates) return;
    at_score(5);
    sim::state.lift = -1400;
    sim::state.lift_state = 0;
    TripTiming timing;
    assert(put_on({0, 30}, 5, nullptr, timing));
    // Once, short of the goal: no second wait at contact after a capped hold.
    assert(timing.capped == 1);
    assert(timing.stack_ms >= kHoldCapMs && timing.stack_ms <= kHoldCapMs + 30);
    assert(sim::state.opened_at.size() == 1);
}

// 2026-10-18 (fourth video): no arm reading (sensor silent). An arm level
// is not taken to be up: the hold short of the goal runs to its cap instead
// of the robot going straight in. A level with the arm at rest goes by the
// lift alone.
void missing_arm_reading() {
    if (kStackGates) return;
    at_score(4);
    sim::state.arm = NAN;
    sim::state.lift_state = 0;  // the raise still running
    TripTiming timing;
    assert(put_on({0, 30}, 4, nullptr, timing));
    assert(timing.capped == 1);
    assert(timing.stack_ms >= kHoldCapMs && timing.stack_ms <= kHoldCapMs + 30);
    at_score(3);
    sim::state.arm = NAN;
    TripTiming t2;
    assert(put_on({0, 30}, 3, nullptr, t2));
    assert(t2.capped == 0 && t2.stack_ms == 0);
    assert(sim::state.opened_at.size() == 1);
}

// 2026-10-18 (team): height 4 (the third cup) 180 degrees lower than the
// third cup's step, and 0.5 s more standing on the goal before it is set
// down; the other levels keep the plain settle.
void height_four_padding() {
    assert(kLevels[3].lift_deg == kThirdCupDeg + 180.0f);
    assert(kLevels[3].extra_settle_ms == 500 && kLevels[2].extra_settle_ms == 0);
    at_score(3);
    const double scored_at = sim::state.lift;
    sim::state.after_delay = [scored_at] {
        if (pros::millis() >= kGoalSettleMs + 530)
            sim::state.lift = scored_at + kReleaseDropDeg + 0.01;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 3, nullptr, timing));
    assert(sim::state.lifts.size() == 1 &&
           sim::state.lifts[0].at == kGoalSettleMs + 500);
    assert(sim::state.opened_at.size() == 1 &&
           sim::state.opened_at[0] == kGoalSettleMs + 530);
}

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string name(argv[1]);
    if (name == "loader") loader_clearance();
    else if (name == "raise") raise_ownership_and_rest();
    else if (name == "score_failure") score_failure_keeps_claw();
    else if (name == "score_timeout_ok") score_timeout_within_tolerance();
    else if (name == "deadline") one_score_deadline();
    else if (name == "arm_release") arm_release_failure();
    else if (name == "seat") ordinary_seat_failure();
    else if (name == "carry") final_score_carry_sequence();
    else if (name == "light_hold") light_hold_until_up();
    else if (name == "half_drop") half_drop_release();
    else if (name == "hold_cap") hold_is_capped();
    else if (name == "no_arm") missing_arm_reading();
    else if (name == "height4") height_four_padding();
    else if (name == "arm_retry") arm_retry_then_score();
    else if (name == "take_over") raise_takes_over();
    else return 2;
}
'''


def production_harness():
    source = (ROOT / "src" / "autons.cpp").read_text()
    subsystems = (ROOT / "include" / "subsystems.hpp").read_text()
    parts = [PRELUDE]
    for name in ("lift_position", "arm_position"):
        parts.append(extract_block(subsystems, rf"^namespace {name}\b"))
    parts.append(declaration(source, "kPneumaticSettleMs"))
    parts.append("namespace three_pin {")
    parts.append(extract_block(source, r"^struct Point\b", semicolon=True))
    for name in ("kReach", "kGentleFloor", "kGoalStandbyIn",
                 "kSeatDropDeg", "kPreloadSeatDeg"):
        parts.append(declaration(source, name))
    parts.append(LIFT_FAKE)
    parts.append(extract_block(source, r"^Point standoff\("))
    parts.append("} namespace skills_route { using three_pin::Point;")
    for name in ("kMostOnGoal", "kScoreReachIn", "kGoalRefMaxIn", "kCarryDeg",
                 "kThirdCupDeg", "kFullLiftDeg", "kCalmDrive", "kLiftDownDeg",
                 "kArmCloseDeg", "kLiftAtDeg", "kArmAtDeg", "kStackWaitMs",
                 "kStackGates", "kArmNearDeg", "kLiftNearDeg", "kGoalContactMs",
                 "kGoalContactLaunchMs", "kGoalContactRpm",
                 "kGoalFinalSpeed", "kPastContactIn", "kGoalSettleMs", "kHoldCapMs",
                 "kReleaseDropDeg", "kPinSeatDeg",
                 "mechanism_stop_reason"):
        parts.append(declaration(source, name))
    parts.append(extract_block(source, r"^struct Level\b", semicolon=True))
    parts.append(declaration(source, "kLevels"))
    parts.append(extract_block(source, r"^constexpr bool with_arm\("))
    parts.append(extract_block(source, r"^struct TripTiming\b", semicolon=True))
    parts.append(ARM_FAKE)
    for name in ("lift_degrees", "arm_degrees"):
        parts.append(extract_block(source, rf"^double {name}\("))
    for name in ("mechanism_failure", "wait_until_down", "raise_for", "put_on"):
        parts.append(extract_block(source, rf"^bool {name}\("))
    parts.extend(["}  // namespace skills_route", CHECKS])
    return "\n".join(parts)


class SkillsScoringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = tempfile.TemporaryDirectory(prefix="skills-scoring-test-")
        cls.addClassCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        source = directory / "scoring.cpp"
        source.write_text(production_harness())
        cls.binary = directory / "scoring"
        compiled = subprocess.run(
            ["g++", "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "include"), str(source), "-o", str(cls.binary)],
            capture_output=True, text=True,
        )
        if compiled.returncode:
            raise AssertionError(f"Production scoring harness failed to compile:\n"
                                 f"{compiled.stdout}{compiled.stderr}")

    def run_case(self, name):
        result = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_loader_requires_measurements_and_rejects_explicit_failures(self):
        self.run_case("loader")

    def test_raise_joins_previous_owner_and_commands_arm_rest(self):
        self.run_case("raise")

    def test_failed_score_does_not_open_claw_or_start_lowering(self):
        # Lift failure, arm failure, or an arm still out of place after one
        # retry: the robot does not drive in or release.
        self.run_case("score_failure")

    def test_timed_out_arm_can_satisfy_measured_score_tolerance(self):
        self.run_case("score_timeout_ok")

    def test_both_score_checks_share_one_deadline(self):
        self.run_case("deadline")

    def test_arm_levels_release_like_the_others(self):
        self.run_case("arm_release")

    def test_cup_release_opens_even_if_the_seat_drop_ends_early(self):
        self.run_case("seat")

    def test_arm_short_of_height_gets_one_retry(self):
        self.run_case("arm_retry")

    def test_raise_takes_over_a_running_pre_raise(self):
        self.run_case("take_over")

    def test_claw_opens_after_half_the_old_drop(self):
        self.run_case("half_drop")

    def test_holds_for_the_lift_are_capped(self):
        self.run_case("hold_cap")

    def test_holds_short_only_while_the_stack_is_not_up(self):
        self.run_case("light_hold")

    def test_last_high_score_lowers_to_carry_while_releasing(self):
        self.run_case("carry")

    def test_missing_arm_reading_does_not_count_as_up(self):
        self.run_case("no_arm")

    def test_height_four_lower_with_extra_settle(self):
        self.run_case("height4")


if __name__ == "__main__":
    unittest.main()
