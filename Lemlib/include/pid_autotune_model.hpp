#pragma once

// Hardware-free core of the PID autotuner: trajectory analysis and the gain
// search policy. The robot-side code supplies "run these gains and give me
// the measured result"; host tests supply a simulated drivetrain instead.
//
// The search automates LemLib's manual tuning procedure (raise kP until the
// move misbehaves, add kD to calm it, repeat) with one addition: a move is
// also rejected when it brakes harder than a set limit. Hard braking is what
// makes wheels slip and the robot shake, and slip is invisible to encoder
// odometry, so it has to be prevented rather than detected.
//
// Units: position is inches (lateral) or degrees (angular), time is seconds.

#include <algorithm>
#include <cmath>
#include <vector>

namespace pidtune {

struct Gains {
    double kP = 0.0, kD = 0.0;
};

struct Sample {
    double t;  // seconds since the move began
    double x;  // position along the move, relative to where it began
    double v;  // velocity along the move from a rate sensor (not d(x)/dt)
};

struct Limits {
    double overshoot;  // largest acceptable travel past the target
    double decel;      // largest acceptable braking, units/s^2
    double accuracy;   // final error that costs as much as 0.5 s
    double noise;      // position jitter ignored when counting crossings
};

struct Metrics {
    double time_s = 0.0;       // until LemLib ended the motion
    double overshoot = 0.0;
    double final_error = 0.0;  // after coasting to rest
    double peak_decel = 0.0;
    int crossings = 0;         // times the position passed through the target
};

// samples cover the motion plus the coast afterwards; motion_time_s is when
// LemLib reported the motion finished.
inline Metrics analyze(const std::vector<Sample>& samples, double target,
                       double motion_time_s, double noise) {
    Metrics m;
    m.time_s = motion_time_s;
    if (samples.empty()) return m;
    const double sign = target >= 0.0 ? 1.0 : -1.0;
    // +1 before the target, -1 past it; a crossing only counts once the
    // position is clear of the target by more than sensor jitter.
    int side = 1;
    for (const Sample& s : samples) {
        const double error = sign * (target - s.x);
        m.overshoot = std::max(m.overshoot, -error);
        if (side > 0 && error < -noise) {
            side = -1;
            ++m.crossings;
        } else if (side < 0 && error > noise) {
            side = 1;
            ++m.crossings;
        }
    }
    m.final_error = std::abs(target - samples.back().x);

    // Braking: how fast speed falls after its peak, from measured velocity
    // over a 60 ms window. Position is not differentiated twice (sensor
    // updates are not synchronized with the sampling loop, and that jitter
    // would read as huge accelerations), and the launch and the near-zero
    // tail are excluded (velocity noise around zero has no reliable sign, so
    // the hard launch used to be mistaken for braking).
    constexpr int kHalf = 3;
    const int n = static_cast<int>(samples.size());
    int peak = 0;
    for (int i = 1; i < n; ++i)
        if (std::abs(samples[i].v) > std::abs(samples[peak].v)) peak = i;
    const double floor_speed = 0.15 * std::abs(samples[peak].v);
    for (int i = std::max(peak, kHalf); i < n - kHalf; ++i) {
        if (std::abs(samples[i].v) < floor_speed) continue;
        const double dt = samples[i + kHalf].t - samples[i - kHalf].t;
        if (dt <= 0.0) continue;
        const double slowing = (std::abs(samples[i - kHalf].v) -
                                std::abs(samples[i + kHalf].v)) / dt;
        m.peak_decel = std::max(m.peak_decel, slowing);
    }
    return m;
}

// Summary of every move driven with one set of gains.
struct Result {
    bool fatal = false;       // robot/sensor fault: stop tuning
    bool acceptable = false;  // smooth: no overshoot, wobble or hard braking
    double cost = INFINITY;   // seconds-equivalent; lower is better
    double time_s = 0.0;      // mean
    double overshoot = 0.0;   // worst
    double final_error = 0.0; // mean
    double peak_decel = 0.0;  // worst
    int crossings = 0;        // worst
};

inline Result summarize(const std::vector<Metrics>& moves, const Limits& limits) {
    Result r;
    if (moves.empty()) return r;
    for (const Metrics& m : moves) {
        r.time_s += m.time_s / moves.size();
        r.final_error += m.final_error / moves.size();
        r.overshoot = std::max(r.overshoot, m.overshoot);
        r.peak_decel = std::max(r.peak_decel, m.peak_decel);
        r.crossings = std::max(r.crossings, m.crossings);
    }
    // One small pass through the target inside the overshoot allowance is
    // fine; coming back through it again is a wobble.
    r.acceptable = r.overshoot <= limits.overshoot &&
                   r.peak_decel <= limits.decel && r.crossings <= 1;
    r.cost = r.time_s + 0.5 * r.final_error / limits.accuracy;
    return r;
}

// Good enough to drive a route with: it actually arrives.
inline bool usable(const Result& r, const Limits& limits, double timeout_s) {
    return r.acceptable && r.final_error <= 3.0 * limits.accuracy &&
           r.time_s < 0.95 * timeout_s;
}

struct SearchConfig {
    Gains start;             // gentle gains that are safe to run first
    double kd_per_kp;        // kD first tried when damping is needed
    double kp_step = 1.3;
    double kd_step = 1.5;
    int kd_levels = 3;       // damping increases tried per kP step
    double min_gain = 0.02;  // required fractional cost improvement
    int max_evaluations = 24;
};

struct Outcome {
    bool ok = false;
    Gains gains;
    Result result;
    int evaluations = 0;
};

// evaluate(Gains) -> Result drives the gains and measures them. Gains only
// ever increase one step at a time from an accepted set, so the harshest
// thing the robot is asked to do is one step beyond something already smooth.
template <typename Evaluate>
inline Outcome tune_gains(const SearchConfig& config, Evaluate evaluate) {
    Outcome out;
    Gains best = config.start;
    Result best_result = evaluate(best);
    out.evaluations = 1;
    // The start is meant to be sluggish. If even that misbehaves, back off.
    for (int i = 0; i < 3 && !best_result.fatal && !best_result.acceptable; ++i) {
        best = {0.6 * best.kP, best.kD};
        best_result = evaluate(best);
        ++out.evaluations;
    }
    if (best_result.fatal || !best_result.acceptable) return out;

    while (out.evaluations < config.max_evaluations) {
        // More kP alone first; while that is not a smooth improvement, the
        // same kP with progressively more damping.
        const double kp = best.kP * config.kp_step;
        double kd = best.kD;
        bool improved = false;
        for (int level = 0; level <= config.kd_levels; ++level) {
            if (out.evaluations >= config.max_evaluations) break;
            const Gains candidate{kp, kd};
            const Result result = evaluate(candidate);
            ++out.evaluations;
            if (result.fatal) return out;
            if (result.acceptable &&
                result.cost < best_result.cost * (1.0 - config.min_gain)) {
                best = candidate;
                best_result = result;
                improved = true;
                break;
            }
            kd = std::max(kd * config.kd_step, kp * config.kd_per_kp);
        }
        if (!improved) break;
    }
    out.ok = true;
    out.gains = best;
    out.result = best_result;
    return out;
}

}  // namespace pidtune
