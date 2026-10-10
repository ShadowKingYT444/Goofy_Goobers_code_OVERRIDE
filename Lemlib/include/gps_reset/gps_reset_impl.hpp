#pragma once

#include "gps_reset.hpp"
#include "pose_transform.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "api.h"
#include "pros/gps.hpp"
#include "field_display_status.hpp"

namespace gpsreset {

constexpr double kMetersToInches = 39.37007874015748;
constexpr double kInchesToMeters = 0.0254;
constexpr double kPi = 3.14159265358979323846;

// Confidence gate: only reset when the GPS's own RMS error estimate is below
// this (meters). From your vex-gps-probe branch, < 0.0762 m (3 in) = valid.
constexpr double kMaxRmsErrorM = 0.0762;

// ---- Live estimate (Estimator) ---------------------------------------------
// Starting values; tune on the field.
constexpr int kLockCount = 3;            // agreeing fixes needed to start
constexpr int kRelockCount = 8;          // agreeing "outliers" that win
constexpr double kClusterIn = 3.0;       // "agreeing" = within this distance
constexpr double kGateIn = 6.0;          // largest believable GPS-vs-estimate gap
constexpr double kGateLatencyS = 0.2;    // extra gap allowed per in/s of speed
constexpr double kGainParked = 0.20;     // pull toward GPS per 100 ms, at rest
constexpr double kGainMoving = 0.06;     // ... while driving
constexpr double kParkedSpeedIn = 5.0;   // in/s
constexpr double kParkedTurnDeg = 30.0;  // deg/s
constexpr double kMaxTurnForFixDeg = 150.0;  // camera blurs above this
constexpr double kFrameGain = 0.05;      // heading-frame tracking per update
constexpr double kCoastS = 2.0;          // dead-reckon this long without a fix
constexpr double kMaxSpeedIn = 90.0;     // faster "motion" is a setPose()
constexpr double kMaxTurnDeg = 700.0;

// ---- Automatic odometry correction (set_auto_reset) -------------------------
constexpr double kRestSpeedIn = 1.0;         // "at rest": slower than this,
constexpr double kRestTurnDeg = 5.0;         // turning slower than this,
constexpr double kRestTimeS = 0.3;           // for at least this long
constexpr double kAutoResetMinIn = 0.75;     // smaller gaps are just noise
constexpr std::uint32_t kAutoResetPeriodMs = 500;

// ---- Route checkpoints (checkpoint) -----------------------------------------
constexpr double kCheckpointMinIn = 1.5;     // leave odometry alone below this
constexpr double kCheckpointRadiusIn = 6.0;  // GPS must agree the robot is here

constexpr std::uint32_t kStableReadBudgetMs = 250;
constexpr std::uint32_t kResetPollMs = 50;
constexpr std::uint32_t kStableReadCount = 3;
// These stability gates are starting candidates. Validate them on the field;
// PROS's RMS position error is not a bound on the true position error.
constexpr double kStablePositionDeltaIn = 3.0;
constexpr double kStableHeadingDeltaDeg = 3.0;
constexpr double kStableOdomPositionDeltaIn = 0.5;
constexpr double kStableOdomHeadingDeltaDeg = 1.5;
// Reject unexpectedly large GPS corrections until their safe range has been
// measured on the field. This is a conservative starting limit, not a tuned
// localization threshold.
constexpr double kMaxPositionCorrectionIn = 8.0;
// Legacy row 7 routes status to the field map without cross-task drawing.
constexpr int kGpsLcdLine = 7;

struct GpsPose {
    bool ok = false;
    double x_in = 0.0;      // field frame, inches, origin = field center
    double y_in = 0.0;
    double theta_deg = 0.0;  // 0 = north, clockwise, matches LemLib theta
    double err_m = 0.0;
    bool jump = false;       // live_pose() only: latest raw fix was ignored
};

inline double normalize_deg(double deg) {
    deg = std::fmod(deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

// Signed smallest rotation from b to a, in (-180, 180].
inline double wrap_deg(double a_minus_b) {
    double d = std::fmod(a_minus_b + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

inline double heading_delta_deg(double lhs, double rhs) {
    return std::abs(wrap_deg(lhs - rhs));
}

// The noise screen. Raw GPS fixes jitter by an inch or more, lag while the
// robot moves, blur while it spins and occasionally jump; drivetrain
// odometry is smooth but drifts. This keeps one field-frame estimate that
// moves with the odometry between fixes and is pulled gently toward each
// believable fix, so it is smooth like odometry and absolute like GPS.
// Pure arithmetic: no hardware, one caller at a time.
struct Estimator {
    bool locked = false;
    double x = 0.0, y = 0.0;        // field inches, robot center
    double frame_deg = 0.0;         // field heading minus odometry heading
    bool rejecting = false;         // latest fix disagreed and was ignored
    bool fix_used = false;          // latest update was corrected by a fix
    double speed = 0.0;             // in/s and deg/s from odometry, latest update
    double turn_rate = 0.0;

    GpsPose update(double odom_x, double odom_y, double odom_theta,
                   const GpsPose& fix, double dt_s) {
        if (dt_s <= 0.0) dt_s = 0.01;
        double dx = odom_x - last_x_, dy = odom_y - last_y_;
        double dtheta = odom_theta - last_theta_;
        // chassis.setPose() moves odometry without moving the robot.
        const bool teleport = !have_odom_ ||
            std::hypot(dx, dy) > kMaxSpeedIn * dt_s + 1.0 ||
            std::abs(dtheta) > kMaxTurnDeg * dt_s + 5.0;
        if (teleport) {
            dx = dy = dtheta = 0.0;
            frame_stale_ = true;
        }
        have_odom_ = true;
        last_x_ = odom_x; last_y_ = odom_y; last_theta_ = odom_theta;
        speed = std::hypot(dx, dy) / dt_s;
        turn_rate = std::abs(dtheta) / dt_s;
        fix_used = false;
        const bool parked = speed < kParkedSpeedIn && turn_rate < kParkedTurnDeg;

        // Dead-reckon: rotate the odometry step into the field frame.
        const double frame = frame_deg * kPi / 180.0;
        const double fx = dx * std::cos(frame) + dy * std::sin(frame);
        const double fy = -dx * std::sin(frame) + dy * std::cos(frame);
        if (locked) { x += fx; y += fy; }
        if (cluster_ > 0) { cluster_x_ += fx; cluster_y_ += fy; }

        if (!fix.ok) {
            rejecting = false;
            no_fix_s_ += dt_s;
            if (no_fix_s_ > kCoastS) { locked = false; cluster_ = 0; }
            return output(odom_theta, 0.0);
        }
        no_fix_s_ = 0.0;
        const double frame_now = wrap_deg(fix.theta_deg - odom_theta);

        if (!locked) {
            frame_deg = frame_now;
            frame_stale_ = false;
            if (add_to_cluster(fix) >= kLockCount) lock_to_cluster();
            return output(odom_theta, fix.err_m);
        }
        if (frame_stale_) {
            frame_deg = frame_now;
            frame_stale_ = false;
        } else if (turn_rate < kParkedTurnDeg) {
            frame_deg += (1.0 - std::pow(1.0 - kFrameGain, dt_s / 0.1)) *
                         wrap_deg(frame_now - frame_deg);
        }

        const double gap = std::hypot(fix.x_in - x, fix.y_in - y);
        if (gap <= kGateIn + kGateLatencyS * speed) {
            rejecting = false;
            cluster_ = 0;
            if (turn_rate <= kMaxTurnForFixDeg) {
                fix_used = true;
                // Scaled so the smoothing does not depend on the call rate.
                const double gain = 1.0 - std::pow(
                    1.0 - (parked ? kGainParked : kGainMoving), dt_s / 0.1);
                x += gain * (fix.x_in - x);
                y += gain * (fix.y_in - y);
            }
        } else {
            rejecting = true;
            // If the "outliers" keep agreeing with each other while the
            // robot is steady, the estimate is what is wrong (a shove, wheel
            // slip): start over from them.
            if (!parked) cluster_ = 0;
            else if (add_to_cluster(fix) >= kRelockCount) lock_to_cluster();
        }
        return output(odom_theta, fix.err_m);
    }

    void reset() { *this = {}; }
    // Odometry was just rewritten (setPose) by a known amount: carry on from
    // the new values without treating the change as robot motion.
    void rebase(double odom_x, double odom_y, double odom_theta) {
        last_x_ = odom_x; last_y_ = odom_y; last_theta_ = odom_theta;
    }

private:
    int add_to_cluster(const GpsPose& fix) {
        if (cluster_ > 0 && std::hypot(fix.x_in - cluster_x_,
                                       fix.y_in - cluster_y_) <= kClusterIn) {
            ++cluster_;
            cluster_x_ += (fix.x_in - cluster_x_) / cluster_;
            cluster_y_ += (fix.y_in - cluster_y_) / cluster_;
        } else {
            cluster_ = 1;
            cluster_x_ = fix.x_in;
            cluster_y_ = fix.y_in;
        }
        return cluster_;
    }
    void lock_to_cluster() {
        locked = true;
        rejecting = false;
        x = cluster_x_;
        y = cluster_y_;
        cluster_ = 0;
    }
    GpsPose output(double odom_theta, double err_m) const {
        GpsPose p{};
        p.ok = locked;
        p.x_in = x;
        p.y_in = y;
        p.theta_deg = normalize_deg(odom_theta + frame_deg);
        p.err_m = err_m;
        p.jump = rejecting;
        return p;
    }
    bool have_odom_ = false, frame_stale_ = true;
    double last_x_ = 0.0, last_y_ = 0.0, last_theta_ = 0.0;
    double no_fix_s_ = 0.0;
    int cluster_ = 0;
    double cluster_x_ = 0.0, cluster_y_ = 0.0;
};

struct SharedState {
    AnchorObserver anchor_observer = nullptr;
    lemlib::Chassis* chassis = nullptr;
    std::unique_ptr<pros::Gps> gps;
    bool initialized = false;
    // Sensor mounting, applied in read_pose(). See init().
    double mount_forward_in = 0.0, mount_right_in = 0.0;
    double mount_facing_deg = 0.0;
    Estimator estimator;
    std::uint32_t estimator_ms = 0;
    bool auto_reset = false;
    double rest_s = 0.0;
    std::uint32_t auto_reset_ms = 0;
    int auto_resets = 0;
    // Pending route checkpoint, served by the telemetry task in live_pose().
    bool checkpoint_pending = false;
    bool checkpoint_applied = false;
    double checkpoint_x = 0.0, checkpoint_y = 0.0, checkpoint_radius = 0.0;
    // Start anchor from capture_start_as(): the field pose the robot had at
    // the start, and the route-frame pose that spot is declared to be.
    bool relative = false;
    bool anchor_valid = false;
    double anchor_x_in = 0.0, anchor_y_in = 0.0, anchor_theta_deg = 0.0;
    double decl_x_in = 0.0, decl_y_in = 0.0, decl_theta_deg = 0.0;
};

// Function-local static: one shared object even though init() lives in
// main.cpp and reset() is called from elsewhere. Same trick as
// avreset, so this stays header-only with no .cpp file.
inline SharedState& shared_state() {
    static SharedState state;
    return state;
}

inline void set_anchor_observer(AnchorObserver observer) {
    shared_state().anchor_observer = observer;
}

// Camera-lens position with the robot's heading (facing_deg applied, lever
// arm not). read_pose() and the mount calibration both build on this.
inline GpsPose read_lens_pose(const pros::Gps& gps) {
    GpsPose p{};
    const double err = gps.get_error();
    if (!std::isfinite(err) || err <= 0.0 || err > kMaxRmsErrorM) return p;
    const pros::gps_status_s_t s = gps.get_position_and_orientation();
    const double raw_heading = gps.get_heading();
    if (!std::isfinite(s.x) || !std::isfinite(s.y) ||
        !std::isfinite(raw_heading))
        return p;
    const auto& st = shared_state();
    p.ok = true;
    p.theta_deg = normalize_deg(raw_heading - st.mount_facing_deg);
    p.x_in = s.x * kMetersToInches;
    p.y_in = s.y * kMetersToInches;
    p.err_m = err;
    return p;
}

// Rotation center for a lens pose and a (forward, right) lens offset. Robot
// forward is (sin, cos) and robot right is (cos, -sin) on the field.
inline GpsPose lens_to_center(GpsPose lens, double forward_in,
                              double right_in) {
    const double heading_rad = lens.theta_deg * kPi / 180.0;
    const double sin_h = std::sin(heading_rad), cos_h = std::cos(heading_rad);
    lens.x_in -= forward_in * sin_h + right_in * cos_h;
    lens.y_in -= forward_in * cos_h - right_in * sin_h;
    return lens;
}

// Robot-center pose in the field frame. The sensor reports its own lens
// position and the direction its camera faces; the mounting from init()
// converts that to the robot's heading and rotation center. The firmware
// offset is left at zero so this is the only place the lever arm is applied.
inline GpsPose read_pose(const pros::Gps& gps) {
    const GpsPose lens = read_lens_pose(gps);
    if (!lens.ok) return lens;
    const auto& st = shared_state();
    return lens_to_center(lens, st.mount_forward_in, st.mount_right_in);
}

// Blocking: mean lens pose of a stationary robot. False if fewer than half
// of the samples had a confident fix.
inline bool average_lens_pose(GpsPose& result, int samples = 20,
                              std::uint32_t period_ms = 50) {
    const auto& st = shared_state();
    if (!st.initialized || st.gps == nullptr) return false;
    int good = 0;
    double sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
    for (int i = 0; i < samples; ++i) {
        const GpsPose p = read_lens_pose(*st.gps);
        if (p.ok) {
            ++good;
            sum_x += p.x_in;
            sum_y += p.y_in;
            sum_sin += std::sin(p.theta_deg * kPi / 180.0);
            sum_cos += std::cos(p.theta_deg * kPi / 180.0);
        }
        pros::delay(period_ms);
    }
    if (good * 2 < samples || good == 0) return false;
    result = {};
    result.ok = true;
    result.x_in = sum_x / good;
    result.y_in = sum_y / good;
    result.theta_deg = normalize_deg(std::atan2(sum_sin, sum_cos) * 180.0 / kPi);
    return true;
}

struct MountFit {
    bool ok = false;
    double forward_in = 0.0, right_in = 0.0;  // lens offset from the center
    double center_x_in = 0.0, center_y_in = 0.0;
    double rms_in = 0.0;                      // how well the stops agree
};

// Lens offset from lens poses taken at several headings while the robot turns
// in place. The rotation center C did not move, so every stop obeys
//   lens = C + forward * (sin h, cos h) + right * (cos h, -sin h)
// which is linear in (Cx, Cy, forward, right): solved by least squares over
// all stops. Needs a correct facing_deg and a real spread of headings.
inline MountFit fit_mount(const std::vector<GpsPose>& stops) {
    MountFit fit{};
    const int n = static_cast<int>(stops.size());
    if (n < 3) return fit;
    double m[4][5] = {};
    double sum_sin = 0.0, sum_cos = 0.0;
    for (const GpsPose& p : stops) {
        if (!p.ok) return fit;
        const double h = p.theta_deg * kPi / 180.0;
        const double sn = std::sin(h), cs = std::cos(h);
        sum_sin += sn;
        sum_cos += cs;
        const double rows[2][5] = {{1, 0, sn, cs, p.x_in},
                                   {0, 1, cs, -sn, p.y_in}};
        for (const auto& row : rows)
            for (int a = 0; a < 4; ++a)
                for (int b = 0; b < 5; ++b) m[a][b] += row[a] * row[b];
    }
    // Headings bunched together cannot separate the center from the offset.
    if (std::hypot(sum_sin, sum_cos) / n > 0.9) return fit;
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 4; ++row)
            if (std::abs(m[row][col]) > std::abs(m[pivot][col])) pivot = row;
        if (std::abs(m[pivot][col]) < 1e-9) return fit;
        for (int b = 0; b < 5; ++b) std::swap(m[col][b], m[pivot][b]);
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            const double factor = m[row][col] / m[col][col];
            for (int b = col; b < 5; ++b) m[row][b] -= factor * m[col][b];
        }
    }
    fit.center_x_in = m[0][4] / m[0][0];
    fit.center_y_in = m[1][4] / m[1][1];
    fit.forward_in = m[2][4] / m[2][2];
    fit.right_in = m[3][4] / m[3][3];
    double sse = 0.0;
    for (const GpsPose& p : stops) {
        const GpsPose c = lens_to_center(p, fit.forward_in, fit.right_in);
        sse += (c.x_in - fit.center_x_in) * (c.x_in - fit.center_x_in) +
               (c.y_in - fit.center_y_in) * (c.y_in - fit.center_y_in);
    }
    fit.rms_in = std::sqrt(sse / n);
    fit.ok = std::isfinite(fit.forward_in) && std::isfinite(fit.right_in) &&
             std::isfinite(fit.rms_in);
    return fit;
}

// Convert an absolute (field-frame) reading into the start-relative frame.
// The frame rotation comes from the declared start heading relative to the
// GPS heading at anchor capture. Headings are clockwise from north.
inline GpsPose to_auton_frame(const GpsPose& p) {
    auto& st = shared_state();
    if (!st.relative) return p;
    GpsPose r = p;
    const FramePose transformed = transform_pose(
        {p.x_in, p.y_in, p.theta_deg},
        {st.anchor_x_in, st.anchor_y_in, st.anchor_theta_deg},
        {st.decl_x_in, st.decl_y_in, st.decl_theta_deg});
    r.x_in = transformed.x_in;
    r.y_in = transformed.y_in;
    r.theta_deg = transformed.theta_deg;
    return r;
}

// Screened live estimate of the robot center in the field frame. Call it
// regularly from ONE task (the telemetry task); each call advances the
// Estimator by the odometry since the previous call.
//
// With set_auto_reset(true) and a valid start anchor it also corrects LemLib's
// X/Y from that estimate whenever the robot has been at rest between motions
// and the GPS is confident. Heading is left to the IMU.
inline GpsPose live_pose() {
    auto& st = shared_state();
    if (!st.initialized || st.gps == nullptr || st.chassis == nullptr)
        return {};
    const std::uint32_t now = pros::millis();
    const double dt_s = st.estimator_ms == 0 ? 0.0
                                             : (now - st.estimator_ms) / 1000.0;
    st.estimator_ms = now;
    const lemlib::Pose odom = st.chassis->getPose();
    const GpsPose estimate = st.estimator.update(
        odom.x, odom.y, odom.theta, read_pose(*st.gps), dt_s);

    const bool resting = st.estimator.speed < kRestSpeedIn &&
                         st.estimator.turn_rate < kRestTurnDeg &&
                         !st.chassis->isInMotion();
    st.rest_s = resting ? st.rest_s + dt_s : 0.0;
    // Both kinds of correction need the same evidence: a start anchor, a
    // locked estimate that a fresh confident fix just agreed with, and a
    // robot that is not moving.
    const bool confident = st.relative && st.anchor_valid && estimate.ok &&
                           st.estimator.fix_used && resting;
    const GpsPose route = to_auton_frame(estimate);
    const double gap = std::hypot(route.x_in - odom.x, route.y_in - odom.y);
    auto apply = [&](const char* why) {
        st.chassis->setPose(route.x_in, route.y_in, odom.theta);
        st.estimator.rebase(route.x_in, route.y_in, odom.theta);
        st.auto_reset_ms = now;
        ++st.auto_resets;
        std::printf("%s,%.2f,%.2f,%.2f,%.2f\n", why,
                    static_cast<double>(odom.x), static_cast<double>(odom.y),
                    route.x_in, route.y_in);
    };
    if (st.checkpoint_pending && confident) {
        // The GPS has to place the robot at the waypoint the route just
        // drove to, and not wildly far from where odometry has it.
        const double off_target = std::hypot(route.x_in - st.checkpoint_x,
                                             route.y_in - st.checkpoint_y);
        if (std::isfinite(gap) && gap >= kCheckpointMinIn &&
            gap <= kMaxPositionCorrectionIn &&
            off_target <= st.checkpoint_radius) {
            apply("GPS_CHECKPOINT");
            st.checkpoint_applied = true;
        }
        st.checkpoint_pending = false;
    } else if (st.auto_reset && confident && st.rest_s >= kRestTimeS &&
               now - st.auto_reset_ms >= kAutoResetPeriodMs &&
               std::isfinite(gap) && gap >= kAutoResetMinIn &&
               gap <= kMaxPositionCorrectionIn) {
        apply("GPS_AUTO_RESET");
    }
    return estimate;
}

// Ask the telemetry task for one correction at a route waypoint.
inline void request_checkpoint(double x_in, double y_in, double radius_in) {
    auto& st = shared_state();
    st.checkpoint_x = x_in;
    st.checkpoint_y = y_in;
    st.checkpoint_radius = radius_in;
    st.checkpoint_applied = false;
    st.checkpoint_pending = true;
}

inline bool checkpoint(double x_in, double y_in, double radius_in,
                       std::uint32_t wait_ms) {
    auto& st = shared_state();
    if (!st.initialized || !st.relative || !st.anchor_valid) return false;
    request_checkpoint(x_in, y_in, radius_in);
    const std::uint32_t began = pros::millis();
    while (st.checkpoint_pending && pros::millis() - began < wait_ms)
        pros::delay(10);
    st.checkpoint_pending = false;
    return st.checkpoint_applied;
}

inline void set_auto_reset(bool enabled) {
    auto& st = shared_state();
    st.auto_reset = enabled;
    st.rest_s = 0.0;
}

inline bool read_stable_pose(const pros::Gps& gps,
                             lemlib::Chassis& chassis,
                             GpsPose& result) {
    const std::uint32_t start = pros::millis();
    std::uint32_t stable_count = 0;
    double reference_x = 0.0;
    double reference_y = 0.0;
    double reference_theta = 0.0;
    lemlib::Pose odom_reference{0.0f, 0.0f, 0.0f};
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_heading_sin = 0.0;
    double sum_heading_cos = 0.0;
    GpsPose latest{};

    do {
        latest = read_pose(gps);
        const lemlib::Pose odom = chassis.getPose();
        if (latest.ok && !chassis.isInMotion()) {
            const double dx = latest.x_in - reference_x;
            const double dy = latest.y_in - reference_y;
            const double odom_dx = odom.x - odom_reference.x;
            const double odom_dy = odom.y - odom_reference.y;
            const bool stable_position =
                std::hypot(dx, dy) <= kStablePositionDeltaIn;
            const bool stable_heading =
                heading_delta_deg(latest.theta_deg, reference_theta) <=
                kStableHeadingDeltaDeg;
            const bool stable_odometry =
                std::hypot(odom_dx, odom_dy) <= kStableOdomPositionDeltaIn &&
                heading_delta_deg(odom.theta, odom_reference.theta) <=
                    kStableOdomHeadingDeltaDeg;
            if (stable_count == 0 ||
                (stable_position && stable_heading && stable_odometry)) {
                if (stable_count == 0) {
                    reference_x = latest.x_in;
                    reference_y = latest.y_in;
                    reference_theta = latest.theta_deg;
                    odom_reference = odom;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_heading_sin = 0.0;
                    sum_heading_cos = 0.0;
                }
                ++stable_count;
                sum_x += latest.x_in;
                sum_y += latest.y_in;
                const double heading_rad = latest.theta_deg * kPi / 180.0;
                sum_heading_sin += std::sin(heading_rad);
                sum_heading_cos += std::cos(heading_rad);
                if (stable_count >= kStableReadCount) {
                    latest.x_in = sum_x / stable_count;
                    latest.y_in = sum_y / stable_count;
                    latest.theta_deg = normalize_deg(
                        std::atan2(sum_heading_sin, sum_heading_cos) *
                        180.0 / kPi
                    );
                    result = latest;
                    // Repeated API reads are not guaranteed to be distinct
                    // camera fixes; this window is only a stability filter.
                    return true;
                }
            } else {
                reference_x = latest.x_in;
                reference_y = latest.y_in;
                reference_theta = latest.theta_deg;
                odom_reference = odom;
                sum_x = latest.x_in;
                sum_y = latest.y_in;
                const double heading_rad = latest.theta_deg * kPi / 180.0;
                sum_heading_sin = std::sin(heading_rad);
                sum_heading_cos = std::cos(heading_rad);
                stable_count = 1;
            }
        } else {
            stable_count = 0;
        }

        if (pros::millis() - start >= kStableReadBudgetMs) break;
        pros::delay(kResetPollMs);
    } while (pros::millis() - start <= kStableReadBudgetMs);

    return false;
}

inline bool apply_reset(const GpsPose& p) {
    auto& st = shared_state();
    if (!p.ok || st.chassis == nullptr || st.chassis->isInMotion() ||
        (st.relative && !st.anchor_valid))
        return false;

    const GpsPose f = to_auton_frame(p);
    if (!std::isfinite(f.x_in) || !std::isfinite(f.y_in) ||
        !std::isfinite(f.theta_deg))
        return false;
    const lemlib::Pose current = st.chassis->getPose();
    if (std::hypot(f.x_in - current.x, f.y_in - current.y) >
        kMaxPositionCorrectionIn)
        return false;

    // Position is corrected first; retain LemLib's validated odometry/IMU
    // heading until GPS heading has been independently qualified.
    st.chassis->setPose(f.x_in, f.y_in, current.theta);
    // Brain screen: the pose the GPS just reset the robot to.
    // "GPSR" = relative (auton-frame) mode, "GPS" = absolute field frame.
    fieldviz::print(kGpsLcdLine, st.relative ? "GPSR (%.1f, %.1f, %.0f)"
                                           : "GPS (%.1f, %.1f, %.0f)",
                     f.x_in, f.y_in, current.theta);
    return true;
}

inline void init(lemlib::Chassis& chassis, std::uint8_t gps_port,
                 double forward_in, double right_in, double facing_deg) {
    auto& st = shared_state();
    st.chassis = &chassis;
    st.initialized = false;
    st.relative = false;
    st.anchor_valid = false;
    st.estimator.reset();
    st.estimator_ms = 0;
    st.mount_forward_in = forward_in;
    st.mount_right_in = right_in;
    st.mount_facing_deg = facing_deg;
    // Zero the firmware's own offset: it is applied along the camera's axes,
    // which are not the robot's unless the sensor faces straight ahead.
    st.gps = std::make_unique<pros::Gps>(gps_port, 0.0, 0.0);
    st.initialized = true;
}

// Blocking: capture the current confident GPS reading as the anchor (P0) for
// relative (auton-frame) mode. Call ONCE per match, with the robot sitting
// stationary at its start pose, before autonomous() runs. (x_in, y_in,
// theta_deg) is the pose that physical spot should have in your auton
// coordinates -- e.g. capture_start_as(0, 0, 180) to match autons written
// around setPose(0, 0, 180).
//
// After this, reset() applies readings in the declared start-relative
// frame. Failure invalidates the current anchor; relative resets then fail
// closed. The acquisition window is bounded to avoid stalling an autonomous.
inline bool capture_start_as(double x_in, double y_in, double theta_deg) {
    auto& st = shared_state();
    if (!st.initialized || st.gps == nullptr) return false;

    // A capture begins a new localization run. Clear every piece of state
    // that could otherwise reuse data from an earlier
    // autonomous invocation. Invalid or unavailable anchors remain in
    // relative mode but fail closed; they never fall back to absolute field
    // coordinates behind a start-relative route's back.
    st.relative = true;
    st.anchor_valid = false;
    if (st.anchor_observer) st.anchor_observer(false, 0.0, 0.0, 0.0);
    if (!std::isfinite(x_in) || !std::isfinite(y_in) ||
        !std::isfinite(theta_deg) || st.chassis == nullptr ||
        st.chassis->isInMotion()) {
        fieldviz::print(kGpsLcdLine,
                            "GPS no anchor");
        return false;
    }

    GpsPose p{};
    if (read_stable_pose(*st.gps, *st.chassis, p)) {
        st.anchor_x_in = p.x_in;
        st.anchor_y_in = p.y_in;
        st.anchor_theta_deg = p.theta_deg;
        st.decl_x_in = x_in;
        st.decl_y_in = y_in;
        st.decl_theta_deg = normalize_deg(theta_deg);
        st.anchor_valid = true;
        if (st.anchor_observer) st.anchor_observer(true, p.x_in, p.y_in, p.theta_deg);
        fieldviz::print(kGpsLcdLine, "GPS anchor ok");
        return true;
    }
    fieldviz::print(kGpsLcdLine, "GPS no fix");
    return false;
}

inline bool reset() {
    auto& st = shared_state();
    if (!st.initialized || st.chassis == nullptr || st.gps == nullptr)
        return false;
    if (st.relative && !st.anchor_valid) {
        fieldviz::print(kGpsLcdLine,
                            "GPS no anchor");
        return false;
    }
    if (st.chassis->isInMotion()) return false;

    GpsPose p{};
    if (read_stable_pose(*st.gps, *st.chassis, p) && apply_reset(p)) return true;
    // No stable, plausible, stationary fix within 250 ms: skip this correction.
    fieldviz::print(kGpsLcdLine, "GPS no fix");
    return false;
}

}  // namespace gpsreset
