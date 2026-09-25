#pragma once

#include "gps_reset.hpp"

#include <cmath>
#include <memory>

#include "api.h"
#include "pros/gps.hpp"
#include "pros/screen.hpp"

namespace gpsreset {
namespace {

constexpr double kMetersToInches = 39.37007874015748;
constexpr double kInchesToMeters = 0.0254;

// Confidence gate: only reset when the GPS's own RMS error estimate is below
// this (meters). From your vex-gps-probe branch, < 0.0762 m (3 in) = valid.
constexpr double kMaxRmsErrorM = 0.0762;

// Constant heading correction (degrees, clockwise). 0 assumes the GPS is
// mounted in the standard orientation (VEX logo up/out, camera rearward).
// Your gps-probe code subtracted 90 from yaw, so if headings look off by a
// fixed amount on the field, put that amount here instead of touching math.
constexpr double kHeadingOffsetDeg = 0.0;

constexpr int kResetAttempts = 20;      // blocking reset() tries
constexpr std::uint32_t kResetPollMs = 50;
constexpr std::uint32_t kTestPollMs = 100;      // test() sensor poll rate
constexpr std::uint32_t kTestResetPeriodMs = 500;  // min gap between test resets
// PROS 4 removed pros::lcd; the brain text API is now pros::screen. We print
// with explicit pixel coordinates (x=8, y=208, near the bottom of the 480x240
// screen) instead of a line number, so this doesn't depend on how many text
// lines a given font size provides.
constexpr std::int16_t kLcdX = 8;
constexpr std::int16_t kLcdY = 208;

struct SharedState {
    lemlib::Chassis* chassis = nullptr;
    std::unique_ptr<pros::Gps> gps;
    bool initialized = false;
    std::uint32_t last_test_poll_ms = 0;
    std::uint32_t last_test_reset_ms = 0;
    bool have_reset = false;
    double last_x_in = 0.0, last_y_in = 0.0, last_theta_deg = 0.0;
    // Relative (auton-frame) mode, anchored by capture_start_as():
    // P0 = confident absolute reading at the start pose, D = the pose that
    // physical spot should have in auton coordinates (e.g. 0, 0, 180).
    bool relative = false;
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

// Convert an absolute (field-frame) reading into the auton frame anchored by
// capture_start_as():  P_rel = P_abs - P0 + D, component-wise, with the
// heading wrapped to [0, 360). No rotation matrix is needed because the two
// frames are axis-aligned (GPS x = right, y = up in both); it is a pure
// translation plus a heading re-zero -- a rigid frame shift.
inline GpsPose to_auton_frame(const GpsPose& p) {
    auto& st = shared_state();
    if (!st.relative) return p;
    GpsPose r = p;
    r.x_in = p.x_in - st.anchor_x_in + st.decl_x_in;
    r.y_in = p.y_in - st.anchor_y_in + st.decl_y_in;
    r.theta_deg =
        normalize_deg(p.theta_deg - st.anchor_theta_deg + st.decl_theta_deg);
    return r;
}

inline void apply_reset(const GpsPose& p) {
    auto& st = shared_state();
    const GpsPose f = to_auton_frame(p);
    st.chassis->setPose(f.x_in, f.y_in, f.theta_deg);
    st.have_reset = true;
    st.last_x_in = f.x_in;
    st.last_y_in = f.y_in;
    st.last_theta_deg = f.theta_deg;
    // Brain screen: the pose the GPS just reset the robot to.
    // "GPSR" = relative (auton-frame) mode, "GPS" = absolute field frame.
    pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, st.relative ? "GPSR (%.1f, %.1f, %.0f)"
                                           : "GPS (%.1f, %.1f, %.0f)",
                     f.x_in, f.y_in, f.theta_deg);
}

}  // namespace

inline void init(lemlib::Chassis& chassis, std::uint8_t gps_port,
                 double forward_in, double right_in) {
    auto& st = shared_state();
    st.chassis = &chassis;
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
// After this, every reset()/test() applies P_rel = P_abs - P0 + D instead of
// the raw absolute reading, so your start-relative paths keep working while
// still getting drift-free GPS corrections. Returns false (mode unchanged)
// if no confident reading appears within ~2 s.
inline bool capture_start_as(double x_in, double y_in, double theta_deg) {
    auto& st = shared_state();
    if (!st.initialized || st.gps == nullptr) return false;
    for (int i = 0; i < 40; ++i) {
        const GpsPose p = read_pose(*st.gps);
        if (p.ok) {
            st.anchor_x_in = p.x_in;
            st.anchor_y_in = p.y_in;
            st.anchor_theta_deg = p.theta_deg;
            st.decl_x_in = x_in;
            st.decl_y_in = y_in;
            st.decl_theta_deg = theta_deg;
            st.relative = true;
            pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, "GPS anchor ok");
            return true;
        }
        pros::delay(kResetPollMs);
    }
    pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, "GPS no fix");
    return false;
}

inline bool reset() {
    auto& st = shared_state();
    if (!st.initialized || st.chassis == nullptr || st.gps == nullptr)
        return false;
    for (int i = 0; i < kResetAttempts; ++i) {
        const GpsPose p = read_pose(*st.gps);
        if (p.ok) {
            apply_reset(p);
            return true;
        }
        pros::delay(kResetPollMs);
    }
    // Nothing confident within ~1 s: say so on the brain (no serial here).
    pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, "GPS no fix");
    return false;
}

inline void test() {
    auto& st = shared_state();
    if (!st.initialized || st.chassis == nullptr || st.gps == nullptr) return;
    const std::uint32_t now = pros::millis();
    if (now - st.last_test_poll_ms < kTestPollMs) return;
    st.last_test_poll_ms = now;

    const GpsPose p = read_pose(*st.gps);
    if (p.ok && now - st.last_test_reset_ms >= kTestResetPeriodMs) {
        st.last_test_reset_ms = now;
        apply_reset(p);
    } else if (st.have_reset) {
        // Keep the last applied pose on screen between resets.
        pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, st.relative ? "GPSR (%.1f, %.1f, %.0f)"
                                               : "GPS (%.1f, %.1f, %.0f)",
                         st.last_x_in, st.last_y_in, st.last_theta_deg);
    } else {
        // No reset yet: show live error so you can see the sensor is alive
        // and how far it is from the confidence gate.
        const double err = st.gps->get_error();
        pros::screen::print(pros::E_TEXT_SMALL, kLcdX, kLcdY, "GPS no fix err=%.3fm", err);
    }
}

}  // namespace gpsreset
