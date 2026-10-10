"""Host checks for the autotuner's trajectory analysis and gain search."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CHECKS = r'''
#include "pid_autotune_model.hpp"
#include <cassert>
#include <cstdio>
#include <random>
using namespace pidtune;

// Stand-in drivetrain: first order in velocity with friction and latency.
struct Plant {
    double gain, tau, delay, deadband, breakaway;
};
class Simulator {
public:
    explicit Simulator(const Plant& plant) : plant_(plant) {
        pipeline_.assign(std::max(0L, std::lround(plant.delay / kSub)) + 1, 0.0);
    }
    double position() const { return x_; }
    double velocity() const { return v_; }
    void step(double u) {  // one 10 ms control period
        for (int i = 0; i < 5; ++i) {
            pipeline_[head_] = u;
            head_ = (head_ + 1) % pipeline_.size();
            substep(pipeline_[head_]);
        }
    }
private:
    static constexpr double kSub = 0.002;
    void substep(double u) {
        const bool stopped = std::abs(v_) < 1e-9;
        if (stopped && std::abs(u) <= plant_.breakaway) return;
        const double dir = stopped ? (u > 0 ? 1.0 : -1.0) : (v_ > 0 ? 1.0 : -1.0);
        const double next =
            v_ + (plant_.gain * (u - plant_.deadband * dir) - v_) * kSub / plant_.tau;
        if (!stopped && next * v_ <= 0.0 && std::abs(u) <= plant_.breakaway) {
            v_ = 0.0;
            return;
        }
        x_ += 0.5 * (v_ + next) * kSub;
        v_ = next;
    }
    Plant plant_;
    std::vector<double> pipeline_;
    std::size_t head_ = 0;
    double x_ = 0.0, v_ = 0.0;
};

// LemLib-style move: PD on error (derivative = per-loop difference), output
// clamped to 127, ends inside 1 for 100 ms or inside 3 for 500 ms.
Metrics drive(const Plant& plant, Gains gains, double target, double noise,
              std::mt19937& rng) {
    std::normal_distribution<double> jitter(0.0, noise);
    Simulator sim(plant);
    std::vector<Sample> samples;
    double previous = 0.0, small = 0.0, large = 0.0, t = 0.0, ended = 0.0;
    bool done = false;
    for (; t < 2.5 + 0.3; t += 0.01) {
        const double x = sim.position() + jitter(rng);
        samples.push_back({t, x, sim.velocity()});
        if (!done && t >= 2.5) { done = true; ended = t; }
        if (done) { sim.step(0.0); if (t > ended + 0.3) break; continue; }
        const double error = target - x;
        const double out = std::clamp(
            gains.kP * error + gains.kD * (error - previous), -127.0, 127.0);
        previous = error;
        small = std::abs(error) < 1.0 ? small + 0.01 : 0.0;
        large = std::abs(error) < 3.0 ? large + 0.01 : 0.0;
        if (small >= 0.1 || large >= 0.5) { done = true; ended = t; }
        sim.step(out);
    }
    return analyze(samples, target, ended, 4 * noise + 1e-6);
}

Outcome run(const char* name, const Plant& plant, SearchConfig config,
            Limits limits, const std::vector<double>& targets, double noise) {
    std::mt19937 rng(11);
    auto evaluate = [&](Gains gains) {
        std::vector<Metrics> moves;
        for (double target : targets)
            moves.push_back(drive(plant, gains, target, noise, rng));
        const Result r = summarize(moves, limits);
        std::printf("  %s kP %6.2f kD %6.1f: %s cost %.2f time %.2f over %.2f "
                    "final %.2f decel %.0f cross %d\n", name, gains.kP, gains.kD,
                    r.acceptable ? "ok " : "BAD", r.cost, r.time_s, r.overshoot,
                    r.final_error, r.peak_decel, r.crossings);
        return r;
    };
    const Result start = evaluate(config.start);
    const Outcome out = tune_gains(config, evaluate);
    std::printf("%s -> kP %.2f kD %.1f after %d evaluations\n", name,
                out.gains.kP, out.gains.kD, out.evaluations);
    assert(out.ok && out.result.acceptable);
    assert(out.evaluations <= config.max_evaluations);
    assert(out.result.peak_decel <= limits.decel);
    assert(out.result.overshoot <= limits.overshoot);
    assert(out.result.cost <= start.cost);
    assert(out.gains.kP >= config.start.kP);
    return out;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    // analyze(): constant 100 units/s^2 braking from 40 units/s to rest.
    std::vector<Sample> braking;
    for (double t = 0.0; t <= 0.8; t += 0.01) {
        const double tb = std::min(t, 0.4);
        braking.push_back({t, 40 * tb - 50 * tb * tb, 40 - 100 * tb});  // stops at x = 8
    }
    Metrics m = analyze(braking, 8.0, 0.4, 0.05);
    assert(std::abs(m.peak_decel - 100) < 8 && m.crossings == 0);
    assert(m.overshoot == 0.0 && m.final_error < 1e-9 && m.time_s == 0.4);
    // A hard launch is not braking, even with velocity noise around zero
    // before it (this once got every candidate rejected on the robot).
    std::vector<Sample> launch;
    for (double t = 0.0; t <= 1.2; t += 0.01) {
        const double v = t < 0.05 ? (std::fmod(t, 0.02) < 0.01 ? -0.6 : 0.6)
                       : t < 0.15 ? 400 * (t - 0.05)
                       : std::max(0.0, 40 - 50 * (t - 0.15));
        launch.push_back({t, 0, v});
    }
    m = analyze(launch, 20.0, 1.0, 0.05);
    assert(std::abs(m.peak_decel - 50) < 6);
    Result arrives; arrives.acceptable = true; arrives.final_error = 0.4; arrives.time_s = 1.0;
    assert(usable(arrives, {0.4, 170, 0.5, 0.05}, 2.5));
    arrives.final_error = 15; assert(!usable(arrives, {0.4, 170, 0.5, 0.05}, 2.5));
    arrives.final_error = 0.4; arrives.time_s = 2.5;
    assert(!usable(arrives, {0.4, 170, 0.5, 0.05}, 2.5));
    // Overshoot then a return through the target: two crossings.
    std::vector<Sample> wobble;
    for (double t = 0.0; t <= 2.0; t += 0.01)
        wobble.push_back({t, 10 * (1 - std::exp(-3 * t) * std::cos(8 * t)), 0});
    m = analyze(wobble, 10.0, 2.0, 0.05);
    assert(m.overshoot > 1.0 && m.crossings >= 2);
    // Negative targets measure the same way.
    for (Sample& s : wobble) s.x = -s.x;
    const Metrics mirrored = analyze(wobble, -10.0, 2.0, 0.05);
    assert(std::abs(mirrored.overshoot - m.overshoot) < 1e-9);
    assert(mirrored.crossings == m.crossings);
    // Jitter around the target is not a crossing.
    std::vector<Sample> parked(100, Sample{0, 0, 0});
    for (int i = 0; i < 100; ++i) parked[i] = {i * 0.01, 10 + (i % 2 ? 0.02 : -0.02), 0};
    assert(analyze(parked, 10.0, 0.5, 0.05).crossings == 0);
    assert(!summarize({}, {1, 1, 1, 1}).acceptable);

    // Search on drivetrains in the range expected here (about 65 in/s and
    // 500 deg/s at full power) and on a heavier, laggier one.
    const Limits lateral{0.4, 170, 0.5, 0.02}, angular{1.5, 2200, 1.0, 0.1};
    const SearchConfig lat{{4, 3}, 1.0}, ang{{1, 5}, 4.0};
    Outcome o = run("LAT", {0.52, 0.14, 0.025, 7, 10}, lat, lateral,
                    {24, -24, 8, -8}, 0.005);
    // Nothing like the bang-bang gains (kP 42) the first tuner produced.
    assert(o.gains.kP < 20);
    o = run("ANG", {4.2, 0.10, 0.025, 9, 13}, ang, angular,
            {90, -90, 160, -160}, 0.02);
    assert(o.gains.kP < 6);
    run("LAT_HEAVY", {0.40, 0.30, 0.045, 12, 16}, lat, lateral,
        {24, -24, 8, -8}, 0.005);
    run("ANG_HEAVY", {2.5, 0.30, 0.045, 14, 20}, ang, angular,
        {90, -90, 160, -160}, 0.02);

    // A fault stops the search; a robot that misbehaves even at the gentlest
    // gains is reported as a failure, not tuned anyway.
    int calls = 0;
    Outcome bad = tune_gains(lat, [&](Gains) { ++calls; Result r; r.fatal = true; return r; });
    assert(!bad.ok && calls == 1);
    bad = tune_gains(lat, [&](Gains) { Result r; r.cost = 1; return r; });
    assert(!bad.ok && bad.evaluations == 4);
}
'''


class PidAutotuneTests(unittest.TestCase):
    def test_analysis_and_search(self):
        with tempfile.TemporaryDirectory(prefix="pid-autotune-test-") as temp:
            directory = Path(temp)
            (directory / "checks.cpp").write_text(CHECKS)
            binary = directory / "checks"
            subprocess.run(["g++", "-std=c++20", "-O2", "-Wall", "-Wextra",
                            "-Werror", "-I", str(ROOT / "include"),
                            str(directory / "checks.cpp"), "-o", str(binary)],
                           check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
