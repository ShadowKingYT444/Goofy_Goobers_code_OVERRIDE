#pragma once

#include "gps_reset.hpp"
#include "pose_transform.hpp"

#include <cmath>
#include <memory>

#include "api.h"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"

namespace gpsreset {

constexpr double kMetersToInches = 39.37007874015748;
constexpr double kInchesToMeters = 0.0254;
constexpr double kPi = 3.14159265358979323846;

// Confidence gate: only reset when the GPS's own RMS error estimate is below
// this (meters). From your vex-gps-probe branch, < 0.0762 m (3 in) = valid.
constexpr double kMaxRmsErrorM = 0.0762;

// Constant heading correction (degrees, clockwise). 0 assumes the GPS is
// mounted in the standard orientation (VEX logo up/out, camera rearward).
// Your gps-probe code subtracted 90 from yaw, so if headings look off by a
// fixed amount on the field, put that amount here instead of touching math.
constexpr double kHeadingOffsetDeg = 0.0;

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
constexpr std::uint32_t kTestPollMs = 100;      // test() sensor poll rate
constexpr std::uint32_t kTestResetPeriodMs = 500;  // min gap between test resets
// The telemetry task owns lines 0-6; all GPS status uses the same LCD API.
constexpr int kGpsLcdLine = 7;

struct SharedState {
    lemlib::Chassis* chassis = nullptr;
    std::unique_ptr<pros::Gps> gps;
    bool initialized = false;
    std::uint32_t last_test_poll_ms = 0;
    std::uint32_t last_test_reset_ms = 0;
    std::uint32_t test_stable_count = 0;
    bool test_has_reference = false;
    double test_gps_x = 0.0, test_gps_y = 0.0, test_gps_theta = 0.0;
    double test_odom_x = 0.0, test_odom_y = 0.0, test_odom_theta = 0.0;
    bool have_reset = false;
    double last_x_in = 0.0, last_y_in = 0.0, last_theta_deg = 0.0;
    // Relative (auton-frame) mode, anchored by capture_start_as():
    // P0 = confident absolute reading at the start pose, D = the pose that
    // physical spot should have in auton coordinates (e.g. 0, 0, 180).
    bool relative = false;
    bool anchor_valid = false;
    double anchor_x_in = 0.0, anchor_y_in = 0.0, anchor_theta_deg = 0.0;
    double decl_x_in = 0.0, decl_y_in = 0.0, decl_theta_deg = 0.0;
};

// Function-local static: one shared object even though init() lives in
// main.cpp and reset()/test() are called from elsewhere. Same trick as
// avreset, so this stays header-only with no .cpp file.
inline SharedState& shared_state() {
    static SharedState state;
    return state;
}

inline double normalize_deg(double deg) {
    while (deg >= 360.0) deg -= 360.0;
    while (deg < 0.0) deg += 360.0;
    return deg;
}

inline double heading_delta_deg(double lhs, double rhs) {
    double delta = std::abs(normalize_deg(lhs) - normalize_deg(rhs));
    if (delta > 180.0) delta = 360.0 - delta;
    return delta;
}

// True when the GPS itself says "I'm sure": finite, positive RMS error under
// the gate. A calibrating/disconnected sensor reports inf -> rejected.
inline bool confident(const pros::Gps& gps) {
    const double err = gps.get_error();
    return std::isfinite(err) && err > 0.0 && err <= kMaxRmsErrorM;
}

struct GpsPose {
    bool ok = false;
    double x_in = 0.0;      // field frame, inches, origin = field center
    double y_in = 0.0;
    double theta_deg = 0.0;  // 0 = north, clockwise, matches LemLib theta
    double err_m = 0.0;
};

inline GpsPose read_pose(const pros::Gps& gps) {
    GpsPose p{};
    if (!confident(gps)) return p;
    const pros::gps_status_s_t s = gps.get_position_and_orientation();
    const double heading = gps.get_heading();
    if (!std::isfinite(s.x) || !std::isfinite(s.y) ||
        !std::isfinite(heading))
        return p;
    p.ok = true;
    p.x_in = s.x * kMetersToInches;
    p.y_in = s.y * kMetersToInches;
    p.theta_deg = normalize_deg(heading + kHeadingOffsetDeg);
    p.err_m = gps.get_error();
    return p;
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

inline PositionEstimate position_from_start(double field_start_x_in,
                                            double field_start_y_in) {
    const auto& st = shared_state();
    PositionEstimate estimate{};
    if (!st.initialized || st.gps == nullptr || !st.relative ||
        !st.anchor_valid)
        return estimate;
    estimate.anchor_ok = true;
    const GpsPose sample = read_pose(*st.gps);
    if (!sample.ok || !std::isfinite(field_start_x_in) ||
        !std::isfinite(field_start_y_in))
        return estimate;
    const FramePose transformed = transform_pose(
        {sample.x_in, sample.y_in, sample.theta_deg},
        {field_start_x_in, field_start_y_in, st.anchor_theta_deg},
        {0.0, 0.0, st.decl_theta_deg});
    estimate.ok = true;
    estimate.x_in = transformed.x_in;
    estimate.y_in = transformed.y_in;
    estimate.theta_deg = transformed.theta_deg;
    return estimate;
}

inline void print_position_from_start(double field_start_x_in,
                                      double field_start_y_in) {
    const PositionEstimate estimate =
        position_from_start(field_start_x_in, field_start_y_in);
    if (!estimate.anchor_ok) {
        pros::lcd::print(kGpsLcdLine, "GPS no anchor");
    } else if (!estimate.ok) {
        pros::lcd::print(kGpsLcdLine, "GPS no fix");
    } else {
        pros::lcd::print(kGpsLcdLine, "GPS X%.1f Y%.1f T%.0f",
                         estimate.x_in, estimate.y_in, estimate.theta_deg);
    }
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
    st.have_reset = true;
    st.last_x_in = f.x_in;
    st.last_y_in = f.y_in;
    st.last_theta_deg = current.theta;
    // Brain screen: the pose the GPS just reset the robot to.
    // "GPSR" = relative (auton-frame) mode, "GPS" = absolute field frame.
    pros::lcd::print(kGpsLcdLine, st.relative ? "GPSR (%.1f, %.1f, %.0f)"
                                           : "GPS (%.1f, %.1f, %.0f)",
                     f.x_in, f.y_in, current.theta);
    return true;
}

inline void init(lemlib::Chassis& chassis, std::uint8_t gps_port,
                 double forward_in, double right_in) {
    auto& st = shared_state();
    st.chassis = &chassis;
    st.initialized = false;
    st.relative = false;
    st.anchor_valid = false;
    st.have_reset = false;
    st.last_test_poll_ms = 0;
    st.last_test_reset_ms = 0;
    st.test_stable_count = 0;
    st.test_has_reference = false;
    // PROS offset frame: +x = robot right, +y = robot front (meters),
    // so the sensor reports the rotation center, not its own position.
    st.gps = std::make_unique<pros::Gps>(gps_port, right_in * kInchesToMeters,
                                         forward_in * kInchesToMeters);
    st.initialized = true;
}

// Blocking: capture the current confident GPS reading as the anchor (P0) for
// relative (auton-frame) mode. Call ONCE per match, with the robot sitting
// stationary at its start pose, before autonomous() runs. (x_in, y_in,
// theta_deg) is the pose that physical spot should have in your auton
// coordinates -- e.g. capture_start_as(0, 0, 180) to match autons written
// around setPose(0, 0, 180).
//
// After this, reset()/test() apply readings in the declared start-relative
// frame. Failure invalidates the current anchor; relative resets then fail
// closed. The acquisition window is bounded to avoid stalling an autonomous.
inline bool capture_start_as(double x_in, double y_in, double theta_deg) {
    auto& st = shared_state();
    if (!st.initialized || st.gps == nullptr) return false;

    // A capture begins a new localization run. Clear every piece of state
    // that could otherwise make test() display or reuse data from an earlier
    // autonomous invocation. Invalid or unavailable anchors remain in
    // relative mode but fail closed; they never fall back to absolute field
    // coordinates behind a start-relative route's back.
    st.relative = true;
    st.anchor_valid = false;
    st.have_reset = false;
    st.last_test_poll_ms = 0;
    st.last_test_reset_ms = 0;
    st.test_stable_count = 0;
    st.test_has_reference = false;
    if (!std::isfinite(x_in) || !std::isfinite(y_in) ||
        !std::isfinite(theta_deg) || st.chassis == nullptr ||
        st.chassis->isInMotion()) {
        pros::lcd::print(kGpsLcdLine,
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
        pros::lcd::print(kGpsLcdLine, "GPS anchor ok");
        return true;
    }
    pros::lcd::print(kGpsLcdLine, "GPS no fix");
    return false;
}

inline bool reset() {
    auto& st = shared_state();
    if (!st.initialized || st.chassis == nullptr || st.gps == nullptr)
        return false;
    if (st.relative && !st.anchor_valid) {
        pros::lcd::print(kGpsLcdLine,
                            "GPS no anchor");
        return false;
    }
    if (st.chassis->isInMotion()) return false;

    GpsPose p{};
    if (read_stable_pose(*st.gps, *st.chassis, p) && apply_reset(p)) return true;
    // No stable, plausible, stationary fix within 250 ms: skip this correction.
    pros::lcd::print(kGpsLcdLine, "GPS no fix");
    return false;
}

inline void test() {
    auto& st = shared_state();
    if (!st.initialized || st.chassis == nullptr || st.gps == nullptr) return;
    if (st.relative && !st.anchor_valid) {
        pros::lcd::print(kGpsLcdLine,
                            "GPS no anchor");
        return;
    }
    const std::uint32_t now = pros::millis();
    if (now - st.last_test_poll_ms < kTestPollMs) return;
    st.last_test_poll_ms = now;

    const GpsPose p = read_pose(*st.gps);
    const lemlib::Pose odom = st.chassis->getPose();
    if (p.ok && !st.chassis->isInMotion() &&
        (!st.relative || st.anchor_valid)) {
        const bool stable = st.test_has_reference &&
            std::hypot(p.x_in - st.test_gps_x, p.y_in - st.test_gps_y) <=
                kStablePositionDeltaIn &&
            heading_delta_deg(p.theta_deg, st.test_gps_theta) <=
                kStableHeadingDeltaDeg &&
            std::hypot(odom.x - st.test_odom_x, odom.y - st.test_odom_y) <=
                kStableOdomPositionDeltaIn &&
            heading_delta_deg(odom.theta, st.test_odom_theta) <=
                kStableOdomHeadingDeltaDeg;
        if (stable) {
            ++st.test_stable_count;
        } else {
            st.test_stable_count = 1;
        }
        st.test_has_reference = true;
        st.test_gps_x = p.x_in;
        st.test_gps_y = p.y_in;
        st.test_gps_theta = p.theta_deg;
        st.test_odom_x = odom.x;
        st.test_odom_y = odom.y;
        st.test_odom_theta = odom.theta;
    } else {
        st.test_stable_count = 0;
        st.test_has_reference = false;
    }

    if (p.ok && st.test_stable_count >= kStableReadCount &&
        now - st.last_test_reset_ms >= kTestResetPeriodMs) {
        // A rejected correction should be eligible for the next good sample;
        // only throttle corrections that were actually applied.
        if (apply_reset(p)) st.last_test_reset_ms = now;
    } else if (st.have_reset) {
        // Keep the last applied pose on screen between resets.
        pros::lcd::print(kGpsLcdLine, st.relative ? "GPSR (%.1f, %.1f, %.0f)"
                                               : "GPS (%.1f, %.1f, %.0f)",
                         st.last_x_in, st.last_y_in, st.last_theta_deg);
    } else {
        // No reset yet: show live error so you can see the sensor is alive
        // and how far it is from the confidence gate.
        const double err = st.gps->get_error();
        pros::lcd::print(kGpsLcdLine, "GPS no fix err=%.3fm", err);
    }
}

}  // namespace gpsreset
