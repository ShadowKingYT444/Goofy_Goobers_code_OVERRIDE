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
}
struct Chassis {
    lemlib::Pose getPose() const { return {}; }
    void setPose(float, float, float) {}
    void moveToPoint(float, float, int, lemlib::MoveToPointParams, bool) {
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
bool wait_done_or_stalled(std::uint32_t) {
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
    for (ArmOutcome failure : {ArmOutcome::cancelled, ArmOutcome::sensor_error}) {
        at_score(1);
        sim::state.arm_outcome = failure;
        TripTiming timing;
        assert(!put_on({0, 30}, 1, nullptr, timing));
        assert(sim::state.claw_closed && sim::state.opened_at.empty());
        assert(sim::state.lifts.empty() && sim::state.stops == 1);
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
        if (pros::millis() >= 20)
            sim::state.lift = scored_at + three_pin::kSeatDropDeg + 0.01;
    };
    TripTiming timing;
    assert(put_on({0, 30}, 1, nullptr, timing));
    assert(!sim::state.claw_closed && sim::state.opened_at[0] == 20);
    assert(sim::state.tick == 20 + kPneumaticSettleMs);
    assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
}

void one_score_deadline() {
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

// The stack is on the goal: if the arm job ends before halfway, the claw
// opens anyway (the behaviour before the PR) and the lift starts down.
void arm_release_failure() {
    for (ArmOutcome failure : {ArmOutcome::timed_out, ArmOutcome::cancelled,
                               ArmOutcome::sensor_error}) {
        at_score(4);
        sim::state.after_delay = [failure] {
            const std::uint32_t end = failure == ArmOutcome::timed_out ? 1500 : 20;
            if (pros::millis() >= end) {
                sim::state.arm_outcome = failure;
                // Explicit failure cannot open even at the halfway point.
                if (failure != ArmOutcome::timed_out)
                    sim::state.arm = (arm_position::max_deg +
                                      arm_position::release_deg) / 2.0;
            }
        };
        TripTiming timing;
        assert(put_on({0, 30}, 4, nullptr, timing));
        assert(!sim::state.claw_closed && sim::state.opened_at.size() == 1);
        assert(sim::state.opened_at[0] ==
               (failure == ArmOutcome::timed_out ? 1500u : 20u));
        assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
    }
}

void ordinary_seat_failure() {
    // Timeout, failed worker, successful-but-short worker, and sensor loss.
    for (int scenario = 0; scenario < 4; ++scenario) {
        at_score(1);
        const double scored_at = sim::state.lift;
        sim::state.after_delay = [scenario, scored_at] {
            if (pros::millis() < 40) return;
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
        assert(sim::state.opened_at[0] == (scenario == 0 ? 800u : 40u));
        assert(sim::state.lifts.size() == 1 && sim::state.lifts[0].settle);
    }
}

// An arm that stopped 30 degrees short of 530 gets one retry; once it is
// there the score goes ahead.
void arm_retry_then_score() {
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
        sim::state.after_delay = [] {
            if (pros::millis() >= 40)
                sim::state.arm = (arm_position::max_deg +
                                  arm_position::release_deg) / 2.0;
        };
        TripTiming timing;
        assert(put_on({0, 30}, 4, nullptr, timing, last_load));
        assert(sim::state.opened_at.size() == 1 && sim::state.opened_at[0] == 40);
        assert(sim::state.lifts.size() == 1);
        const auto& next = sim::state.lifts[0];
        assert(next.at == 40 + arm_position::release_settle_ms);
        assert(arm_position::release_settle_ms == 100);
        assert(sim::state.tick == next.at && !next.fast);
        if (last_load) assert(!next.settle && next.target == kCarryDeg);
        else assert(next.settle);
    }
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

    def test_arm_release_opens_even_if_the_arm_job_ends_early(self):
        self.run_case("arm_release")

    def test_cup_release_opens_even_if_the_seat_drop_ends_early(self):
        self.run_case("seat")

    def test_arm_short_of_height_gets_one_retry(self):
        self.run_case("arm_retry")

    def test_raise_takes_over_a_running_pre_raise(self):
        self.run_case("take_over")

    def test_last_high_score_lowers_to_carry_after_release_settle(self):
        self.run_case("carry")


if __name__ == "__main__":
    unittest.main()
