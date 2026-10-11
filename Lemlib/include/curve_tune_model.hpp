#pragma once

// Hardware-free core of the curve tuner: how one curved move is judged and
// how the settings to keep are chosen. The robot-side code (src/
// curve_autotune.cpp) drives LemLib's moveToPose with each candidate and
// fills in what it measured; host tests feed made-up measurements instead.
//
// A curved move (curve::to in include/curve_motion.hpp: a LemLib
// "boomerang" curve with an optional straight finish) has three knobs:
//   lead   how far the robot swings out before bending onto its final
//          heading (0-1). Low = cuts the corner, high = wide sweep.
//   drift  LemLib's horizontalDrift: how much sideways grip it assumes, which
//          sets how much it slows down in the curve. Low = slow and careful,
//          high = fast but the wheels may slide sideways.
//   straight_in  length of the straight finish onto the target (0 = none).
//          The curve then ends that far back, already on the final heading.
//
// Units: inches, degrees, seconds.

#include <algorithm>
#include <cmath>
#include <vector>

namespace curvetune {

struct Settings {
    double lead = 0.6, drift = 2.0, straight_in = 0.0;
};

struct Sample {
    double t;          // seconds since the move began
    double turn_rate;  // deg/s from the gyro, clockwise positive
};

// One driven curve.
struct Move {
    double time_s = 0.0;
    double position_error_in = 0.0;  // after coming to rest, by odometry
    double heading_error_deg = 0.0;
    double roughness = 0.0;          // RMS change in turn rate, deg/s^2
    int reversals = 0;               // times the robot swung back the other way
    bool timed_out = false;
};

// Ignore turn rates below this when counting direction changes: gyro noise
// around zero has no reliable sign.
constexpr double kTurnDeadbandDegPerSec = 20.0;

// How jerky the turning was: RMS angular acceleration, taken over 60 ms
// windows so that sensor jitter between samples is not read as jerk.
inline double roughness(const std::vector<Sample>& samples) {
    constexpr int kHalf = 3;
    const int n = static_cast<int>(samples.size());
    double sum = 0.0;
    int count = 0;
    for (int i = kHalf; i < n - kHalf; ++i) {
        const double dt = samples[i + kHalf].t - samples[i - kHalf].t;
        if (dt <= 0.0) continue;
        const double accel =
            (samples[i + kHalf].turn_rate - samples[i - kHalf].turn_rate) / dt;
        sum += accel * accel;
        ++count;
    }
    return count ? std::sqrt(sum / count) : 0.0;
}

// How many times the robot changed which way it was turning. A clean curve
// to one side has none; steering that hunts left and right has several.
inline int reversals(const std::vector<Sample>& samples) {
    int direction = 0, changes = 0;
    for (const Sample& s : samples) {
        if (std::abs(s.turn_rate) < kTurnDeadbandDegPerSec) continue;
        const int now = s.turn_rate > 0.0 ? 1 : -1;
        if (direction != 0 && now != direction) ++changes;
        direction = now;
    }
    return changes;
}

// Seconds-equivalent; lower is better. Each term is scaled so that the
// amount named costs as much as half a second of driving:
// 1 in off target, 4 deg off heading, 1500 deg/s^2 of jerk, one reversal.
inline double cost(const Move& m) {
    return m.time_s + 0.5 * m.position_error_in +
           0.5 * m.heading_error_deg / 4.0 + 0.5 * m.roughness / 1500.0 +
           0.5 * m.reversals + (m.timed_out ? 2.0 : 0.0);
}

// Everything measured for one pair of settings.
struct Result {
    Settings settings;
    bool valid = false;        // every curve was driven and measured
    double cost = INFINITY;
    double time_s = 0.0;       // mean per curve
    double position_error_in = 0.0;  // worst
    double heading_error_deg = 0.0;  // worst
    double roughness = 0.0;    // mean
    int reversals = 0;         // total
    double slip_in = -1.0;     // GPS vs odometry afterwards; < 0 = unknown
};

// slip_in is how far odometry had wandered from the GPS after the curves:
// sideways sliding the encoders cannot see. Pass < 0 if the GPS had no fix.
inline Result summarize(Settings settings, const std::vector<Move>& moves,
                        double slip_in) {
    Result r;
    r.settings = settings;
    r.slip_in = slip_in;
    if (moves.empty()) return r;
    r.valid = true;
    r.cost = 0.0;
    for (const Move& m : moves) {
        r.cost += cost(m) / moves.size();
        r.time_s += m.time_s / moves.size();
        r.roughness += m.roughness / moves.size();
        r.position_error_in = std::max(r.position_error_in, m.position_error_in);
        r.heading_error_deg = std::max(r.heading_error_deg, m.heading_error_deg);
        r.reversals += m.reversals;
    }
    if (slip_in > 0.0) r.cost += 0.3 * slip_in;
    return r;
}

// The result to keep. The baseline (the settings in use today) stays unless
// something beats it by a clear margin, so run-to-run noise cannot talk the
// robot into a change. Returns the index into `results`; `baseline` if
// nothing is clearly better.
inline int choose(const std::vector<Result>& results, int baseline,
                  double min_gain = 0.05) {
    int best = baseline;
    const bool have_baseline = baseline >= 0 &&
                               baseline < static_cast<int>(results.size()) &&
                               results[baseline].valid;
    const double bar = have_baseline
                           ? results[baseline].cost * (1.0 - min_gain)
                           : INFINITY;
    double best_cost = bar;
    for (int i = 0; i < static_cast<int>(results.size()); ++i) {
        if (i == baseline || !results[i].valid) continue;
        if (results[i].cost < best_cost) {
            best = i;
            best_cost = results[i].cost;
        }
    }
    return best;
}

// A winner is only believed if it wins again: both are driven a second time
// and the challenger must still beat the baseline by the margin on that
// repeat, and by the margin on the two runs taken together.
inline bool confirmed(const Result& baseline_first, const Result& baseline_again,
                      const Result& winner_first, const Result& winner_again,
                      double min_gain = 0.05) {
    if (!baseline_again.valid || !winner_again.valid) return false;
    const bool again = winner_again.cost < baseline_again.cost * (1.0 - min_gain);
    const bool together = winner_first.cost + winner_again.cost <
                          (baseline_first.cost + baseline_again.cost) *
                              (1.0 - min_gain);
    return again && together;
}

}  // namespace curvetune
