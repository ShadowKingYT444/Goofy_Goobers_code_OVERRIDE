"""Host checks for how the curve tuner judges moves and picks settings."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CHECKS = r'''
#include "curve_tune_model.hpp"
#include <cassert>
using namespace curvetune;

std::vector<Sample> turn(double wobble_deg_per_sec) {
    // A 1 s right-hand curve at 90 deg/s, optionally hunting at 5 Hz.
    std::vector<Sample> samples;
    for (int i = 0; i <= 100; ++i) {
        const double t = i * 0.01;
        samples.push_back(
            {t, 90.0 + wobble_deg_per_sec * std::sin(2 * M_PI * 5 * t)});
    }
    return samples;
}

Result result(double lead, double time_s, double error_in, double slip_in) {
    Move m;
    m.time_s = time_s;
    m.position_error_in = error_in;
    return summarize({lead, 2.0}, {m, m}, slip_in);
}

int main() {
    // A steady curve is smooth and never reverses; a hunting one is neither.
    assert(roughness(turn(0)) < 1.0 && reversals(turn(0)) == 0);
    assert(roughness(turn(150)) > 1500.0 && reversals(turn(150)) >= 4);
    // Gyro noise around zero is not a change of direction.
    assert(reversals({{0, 5}, {0.01, -5}, {0.02, 8}, {0.03, -12}}) == 0);
    assert(roughness({}) == 0.0);

    // Slower, less accurate, rougher, wobblier or timed out all cost more.
    Move base;
    base.time_s = 2.0;
    Move worse = base;
    worse.position_error_in = 1.0;
    assert(cost(worse) > cost(base));
    worse = base;
    worse.heading_error_deg = 4.0;
    assert(cost(worse) > cost(base));
    worse = base;
    worse.reversals = 1;
    assert(cost(worse) > cost(base));
    worse = base;
    worse.timed_out = true;
    assert(cost(worse) > cost(base) + 1.5);

    // Sliding the encoders cannot see counts against a setting; an unknown
    // amount (no GPS fix) does not.
    assert(result(0.6, 2.0, 0.5, 3.0).cost > result(0.6, 2.0, 0.5, 0.0).cost);
    assert(result(0.6, 2.0, 0.5, -1.0).cost == result(0.6, 2.0, 0.5, 0.0).cost);
    assert(!summarize({}, {}, 0.0).valid);

    // The baseline is kept unless something is clearly better.
    std::vector<Result> all{result(0.6, 2.00, 0.5, 0.0),
                            result(0.45, 1.95, 0.5, 0.0),   // within noise
                            result(0.3, 2.60, 0.5, 0.0)};
    assert(choose(all, 0) == 0);
    all.push_back(result(0.75, 1.60, 0.4, 0.0));
    assert(choose(all, 0) == 3);
    // An unmeasured candidate is never chosen.
    all.push_back(summarize({0.9, 2.0}, {}, 0.0));
    assert(choose(all, 0) == 3);

    // A winner has to win again on a repeat run, not just once.
    const Result usual = result(0.6, 2.0, 0.5, 0.0);
    const Result fast = result(0.45, 1.6, 0.5, 0.0);
    assert(confirmed(usual, usual, fast, fast));
    assert(!confirmed(usual, usual, fast, result(0.45, 2.1, 0.5, 0.0)));
    assert(!confirmed(usual, usual, fast, summarize({0.45, 2.0}, {}, 0.0)));
    // ...and a lucky first run cannot carry a repeat that only just wins.
    assert(!confirmed(result(0.6, 1.5, 0.5, 0.0), usual, fast,
                      result(0.45, 1.85, 0.5, 0.0)));
}
'''


class CurveTuneTests(unittest.TestCase):
    def test_scoring_and_choice(self):
        with tempfile.TemporaryDirectory(prefix="curve-tune-test-") as temp:
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
