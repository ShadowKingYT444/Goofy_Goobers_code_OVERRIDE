"""Run production skills control helpers with fake sensors and a fake clock."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CHECKS = r'''
#include "skills_control.hpp"
#include <cassert>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace skills_control;

struct FakeArm {
    std::uint32_t tick = 0;
    double position = 22.0;
    bool cancel = false;
    int brakes = 0;
    std::vector<int> commands;
    std::vector<std::uint32_t> command_times;
    std::function<void(FakeArm&)> after_delay;

    std::uint32_t now() const { return tick; }
    void delay(std::uint32_t ms) {
        assert(ms > 0 && ms <= 10);
        tick += ms;
        if (after_delay) after_delay(*this);
    }
    double position_deg() const { return position; }
    bool cancelled() const { return cancel; }
    void command(int power) {
        commands.push_back(power);
        command_times.push_back(tick);
    }
    void brake() { ++brakes; }
};

void arm_deadline() {
    // A stalled rest move must end without any parent calling done(). It
    // must not receive extra motor commands after its own deadline.
    FakeArm arm;
    assert(run_arm(0, 1500, arm) == ArmOutcome::timed_out);
    assert(arm.tick == 1500 && arm.brakes == 1);
    assert(arm.commands.size() == 150);
    assert(arm.command_times.back() == 1490);
    for (int power : arm.commands) assert(power == 12);

    // A limit not divisible by the control period is still respected.
    FakeArm short_move;
    assert(run_arm(0, 35, short_move) == ArmOutcome::timed_out);
    assert(short_move.tick == 35 && short_move.brakes == 1);
    assert(short_move.command_times.back() == 30);
}

void driver_unlimited_and_cancellation() {
    FakeArm driver;
    driver.after_delay = [](FakeArm& arm) {
        if (arm.tick >= 1800) arm.position = 0;
    };
    assert(run_arm(0, 0, driver) == ArmOutcome::reached);
    assert(driver.tick == 1800 && driver.brakes == 1);

    FakeArm stopped;
    stopped.after_delay = [](FakeArm& arm) { arm.cancel = arm.tick >= 40; };
    assert(run_arm(0, 1500, stopped) == ArmOutcome::cancelled);
    assert(stopped.tick == 40 && stopped.brakes == 1);
    assert(stopped.command_times.back() == 30);

    FakeArm cancelled_before_start;
    cancelled_before_start.cancel = true;
    assert(run_arm(0, 1500, cancelled_before_start) == ArmOutcome::cancelled);
    assert(cancelled_before_start.commands.empty());
    assert(cancelled_before_start.brakes == 1);
}

void sensor_errors() {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
        FakeArm arm;
        arm.position = invalid;
        assert(run_arm(0, 1500, arm) == ArmOutcome::sensor_error);
        assert(arm.tick == 0 && arm.commands.empty() && arm.brakes == 1);
    }
    FakeArm lost_sensor;
    lost_sensor.after_delay = [](FakeArm& arm) {
        if (arm.tick >= 20)
            arm.position = std::numeric_limits<double>::quiet_NaN();
    };
    assert(run_arm(0, 1500, lost_sensor) == ArmOutcome::sensor_error);
    assert(lost_sensor.tick == 20 && lost_sensor.commands.size() == 2);
    assert(lost_sensor.brakes == 1);

    FakeArm bad_target;
    assert(run_arm(std::numeric_limits<double>::quiet_NaN(), 1500,
                   bad_target) == ArmOutcome::sensor_error);
    assert(bad_target.commands.empty() && bad_target.brakes == 1);
}

void arm_power_profile() {
    // Fixed points from the existing controller, including the two sides
    // of the 25-degree power-floor boundary and both motor directions.
    assert(arm_motor_power(4) == -2);
    assert(arm_motor_power(24) == -13);
    assert(arm_motor_power(25) == -13);
    assert(arm_motor_power(26) == -25);
    assert(arm_motor_power(59) == -32);
    assert(arm_motor_power(60) == -33);
    assert(arm_motor_power(100) == -55);
    assert(arm_motor_power(530) == -110);
    assert(arm_motor_power(-24) == 13);
    assert(arm_motor_power(-26) == 25);
    assert(arm_motor_power(-530) == 110);

    FakeArm at_target;
    at_target.position = 527;
    assert(run_arm(530, 1500, at_target) == ArmOutcome::reached);
    assert(at_target.commands.empty() && at_target.tick == 0);
    assert(at_target.brakes == 1);
}

void physical_gates() {
    constexpr double down = -83, rest = 0, loader_arm = 25, score_arm = 20;
    constexpr double lock_from = -1586.5;
    // Loader and score keep their own physical tolerances. Neither takes
    // job.done(), so a completed task cannot override a bad measurement.
    assert(loader_position_ready(-83, 22, down, rest, loader_arm));
    assert(!score_position_ready(-1150, 22, -1150, rest, 25,
                                  score_arm, lock_from));
    assert(!loader_position_ready(-140, 0, down, rest, loader_arm));
    assert(!loader_position_ready(-75, 100, down, rest, loader_arm));
    assert(loader_position_ready(-75, 25, down, rest, loader_arm));
    assert(!loader_position_ready(-75, 25.01, down, rest, loader_arm));
    assert(score_position_ready(-1150, 20, -1150, rest, 25,
                                 score_arm, lock_from));
    assert(!score_position_ready(-1124, 0, -1150, rest, 25,
                                  score_arm, lock_from));

    // A worker can time out short of its 3-degree target yet meet the
    // separate 20-degree physical gate. Timeout itself proves nothing.
    FakeArm near_target;
    near_target.position = 525;
    assert(run_arm(530, 1500, near_target) == ArmOutcome::timed_out);
    assert(score_position_ready(-1639.3, near_target.position, -1639.3,
                                 530, 25, score_arm, lock_from));
    assert(!score_position_ready(-1639.3, 508, -1639.3, 530,
                                  25, score_arm, lock_from));
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    assert(!loader_position_ready(invalid, 0, down, rest, loader_arm));
    assert(!loader_position_ready(-75, invalid, down, rest, loader_arm));
    assert(!score_position_ready(-1150, invalid, -1150, rest,
                                  25, score_arm, lock_from));
}

void pickup_target_band() {
    assert(!pickup_clear(-142, -83, false));
    assert(pickup_clear(-142, -83, true));
    assert(!pickup_clear(-130, -83, true));
    assert(pickup_clear(-143, -83, false));
    assert(!pickup_clear(-59, 0, false));
    assert(pickup_clear(-60, 0, false));
    assert(!pickup_clear(std::numeric_limits<double>::quiet_NaN(), -83, true));
}

void shared_wait_budget() {
    std::uint32_t tick = 0;
    int delays = 0;
    const auto now = [&] { return tick; };
    const auto delay = [&](std::uint32_t ms) { tick += ms; ++delays; };
    const auto late_ready = [&] {
        return tick >= 2200 ? Readiness::ready : Readiness::waiting;
    };
    assert(wait_until(0, 2500, late_ready, now, delay) == WaitOutcome::ready);
    assert(tick == 2200);
    tick += 200;  // final approach; the original readiness clock is retained
    const auto still_waiting = [] { return Readiness::waiting; };
    assert(wait_until(0, 2500, still_waiting, now, delay) == WaitOutcome::timed_out);
    assert(tick == 2500);
    const int previous_delays = delays;
    assert(wait_until(0, 2500, still_waiting, now, delay) == WaitOutcome::timed_out);
    assert(delays == previous_delays);  // no second 2.5-second allowance
    assert(wait_until(0, 2500, [] { return Readiness::ready; }, now, delay) ==
           WaitOutcome::ready);  // already ready: no more waiting
    assert(delays == previous_delays);

    tick = 0;
    const auto boundary_ready = [&] {
        return tick >= 25 ? Readiness::ready : Readiness::waiting;
    };
    assert(wait_until(0, 25, boundary_ready, now, delay) == WaitOutcome::ready);
    assert(tick == 25);
    tick = 0;
    const auto too_late = [&] {
        return tick >= 26 ? Readiness::ready : Readiness::waiting;
    };
    assert(wait_until(0, 25, too_late, now, delay) == WaitOutcome::timed_out);
    assert(tick == 25);

    tick = 0;
    assert(wait_until(0, 2500, [] { return Readiness::failed; }, now, delay) ==
           WaitOutcome::failed);
    assert(tick == 0);  // explicit failure is not a reason to keep sleeping
    assert(wait_until(0, 0, still_waiting, now, delay) == WaitOutcome::timed_out);
    assert(tick == 0);
}

void clock_rollover() {
    const std::uint32_t began = std::numeric_limits<std::uint32_t>::max() - 15;
    FakeArm arm;
    arm.tick = began;
    assert(run_arm(0, 35, arm) == ArmOutcome::timed_out);
    assert(arm.tick - began == 35 && arm.brakes == 1);

    std::uint32_t tick = began;
    const auto now = [&] { return tick; };
    const auto delay = [&](std::uint32_t ms) { tick += ms; };
    assert(wait_until(began, 35, [] { return Readiness::waiting; }, now, delay) ==
           WaitOutcome::timed_out);
    assert(tick - began == 35);
}

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string name(argv[1]);
    if (name == "arm_deadline") arm_deadline();
    else if (name == "driver_unlimited_and_cancellation") driver_unlimited_and_cancellation();
    else if (name == "sensor_errors") sensor_errors();
    else if (name == "arm_power_profile") arm_power_profile();
    else if (name == "physical_gates") physical_gates();
    else if (name == "pickup_target_band") pickup_target_band();
    else if (name == "shared_wait_budget") shared_wait_budget();
    else if (name == "clock_rollover") clock_rollover();
    else return 2;
}
'''


class SkillsControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="skills-control-test-")
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        source = directory / "checks.cpp"
        source.write_text(CHECKS)
        cls.binary = directory / "checks"
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "include"), str(source), "-o", str(cls.binary)],
            check=True,
        )

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=5)

    def test_arm_deadline_without_parent_polling(self):
        self.run_case("arm_deadline")

    def test_unlimited_driver_move_and_cancellation(self):
        self.run_case("driver_unlimited_and_cancellation")

    def test_sensor_errors_brake_without_false_success(self):
        self.run_case("sensor_errors")

    def test_existing_arm_power_profile(self):
        self.run_case("arm_power_profile")

    def test_measured_loader_and_score_gates(self):
        self.run_case("physical_gates")

    def test_pickup_success_requires_the_measured_target_band(self):
        self.run_case("pickup_target_band")

    def test_repeated_checks_share_one_waiting_budget(self):
        self.run_case("shared_wait_budget")

    def test_deadlines_across_clock_rollover(self):
        self.run_case("clock_rollover")


if __name__ == "__main__":
    unittest.main()
