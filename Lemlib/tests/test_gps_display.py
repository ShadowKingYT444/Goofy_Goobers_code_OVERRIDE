"""Compile the production GPS header against small host-side hardware fakes."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

FAKES = r'''
#pragma once
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>
namespace pros {
inline std::uint32_t clock_ms = 0;
inline std::uint32_t millis() { return clock_ms; }
inline void delay(std::uint32_t ms) { clock_ms += ms; }
struct gps_status_s_t { double x = 0, y = 0; };
class Gps {
public:
    inline static double error = 0.01, heading = 90;
    inline static gps_status_s_t position{};
    Gps(std::uint8_t, double, double) {}
    double get_error() const { return error; }
    double get_heading() const { return heading; }
    gps_status_s_t get_position_and_orientation() const { return position; }
};
namespace lcd {
inline std::string text;
inline int line = -1;
inline void print(int row, const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    line = row;
    text = buffer;
}
}
}
namespace lemlib {
struct Pose { float x, y, theta; };
class Chassis {
public:
    int set_pose_calls = 0;
    bool moving = false;
    Pose pose{0, 0, 180};
    bool isInMotion() const { return moving; }
    Pose getPose() const { return pose; }
    void setPose(float x, float y, float theta) {
        ++set_pose_calls;
        pose = {x, y, theta};
    }
};
}
'''

CHECKS = r'''
#include "gps_reset/gps_reset.hpp"
#include <cassert>
#include <limits>

void check_near(double actual, double expected, int line) {
    if (std::abs(actual - expected) < 1e-6) return;
    std::fprintf(stderr, "line %d: got %.6f, expected %.6f\n", line, actual, expected);
    std::abort();
}
#define near(actual, expected) check_near(actual, expected, __LINE__)
void reading(double x, double y, double heading) {
    pros::Gps::position = {x * 0.0254, y * 0.0254};
    pros::Gps::heading = heading;
}
int anchor_notifications = 0;
bool notified_valid = false;
double notified_x = 0, notified_y = 0, notified_heading = 0;
void anchor_changed(bool valid, double x, double y, double heading) {
    ++anchor_notifications;
    notified_valid = valid;
    notified_x = x;
    notified_y = y;
    notified_heading = heading;
}
int main() {
    using namespace gpsreset;
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    lemlib::Chassis chassis;

    // Mounting: camera looks out of the robot's right side, 7 in ahead and
    // 4.5 in right of the rotation center. The sensor reports its own lens
    // position and its camera's heading.
    init(chassis, 10, 7, 4.5, 90);
    pros::Gps::error = infinity;
    assert(!live_pose().ok && !live_pose().jump);  // no fix is not a jump
    pros::Gps::error = 0.01;
    reading(14.5, 27, 90);      // robot at (10,20) facing north
    auto m = read_pose(*shared_state().gps);
    assert(m.ok); near(m.x_in, 10); near(m.y_in, 20); near(m.theta_deg, 0);
    reading(17, 15.5, 180);     // same spot, robot turned to face east
    m = read_pose(*shared_state().gps);
    near(m.x_in, 10); near(m.y_in, 20); near(m.theta_deg, 90);

    // Mount calibration: stops around a turn in place recover the lens
    // offset and the rotation center, whatever offsets init() was given.
    auto lens_at = [](double cx, double cy, double fwd, double right, double h) {
        GpsPose p{}; p.ok = true; p.theta_deg = h;
        const double r = h * kPi / 180.0;
        p.x_in = cx + fwd * std::sin(r) + right * std::cos(r);
        p.y_in = cy + fwd * std::cos(r) - right * std::sin(r);
        return p;
    };
    std::vector<GpsPose> stops;
    for (double h : {0., 60., 120., 180., 240., 300.})
        stops.push_back(lens_at(10, 20, 7, 4.5, h));
    auto fit = fit_mount(stops);
    assert(fit.ok); near(fit.forward_in, 7); near(fit.right_in, 4.5);
    near(fit.center_x_in, 10); near(fit.center_y_in, 20); near(fit.rms_in, 0);
    // Negative offsets, uneven headings, and +/-0.5 in of scatter.
    stops.clear();
    int k = 0;
    for (double h : {10., 95., 140., 230., 290., 350.}) {
        GpsPose p = lens_at(-30, 5, -3, 2, h);
        p.x_in += (k % 2 ? 0.5 : -0.5); p.y_in += (k % 3 ? 0.4 : -0.5); ++k;
        stops.push_back(p);
    }
    fit = fit_mount(stops);
    assert(fit.ok && std::abs(fit.forward_in + 3) < 0.6 &&
           std::abs(fit.right_in - 2) < 0.6 && fit.rms_in > 0.2 && fit.rms_in < 1.0);
    // No real turn, too few stops, or a lost fix: no answer.
    assert(!fit_mount({lens_at(0,0,7,4,0), lens_at(0,0,7,4,5), lens_at(0,0,7,4,10)}).ok);
    assert(!fit_mount({lens_at(0,0,7,4,0), lens_at(0,0,7,4,180)}).ok);
    stops[2].ok = false;
    assert(!fit_mount(stops).ok);
    // average_lens_pose reads the lens (camera direction applied, no offset).
    GpsPose lens_a{};
    reading(14.5, 27, 90);
    assert(average_lens_pose(lens_a, 4, 10));
    near(lens_a.x_in, 14.5); near(lens_a.y_in, 27); near(lens_a.theta_deg, 0);
    pros::Gps::error = infinity;
    assert(!average_lens_pose(lens_a, 4, 10));
    pros::Gps::error = 0.01;

    // Local axes must match the drivetrain's. Robot physically faces field
    // west (270) at the start, declared 180: driving forward 10 in is field
    // (-10, 0) and must read local (0, -10), not a swapped axis.
    init(chassis, 10, 0, 0, 90);
    reading(30, 40, 0);         // camera heading = 270 + 90
    assert(capture_start_as(0, 0, 180));
    reading(20, 40, 0);
    auto local = to_auton_frame(read_pose(*shared_state().gps));
    near(local.x_in, 0); near(local.y_in, -10); near(local.theta_deg, 180);
    reading(30, 47, 0);         // field +Y is the robot's right: local -X
    local = to_auton_frame(read_pose(*shared_state().gps));
    near(local.x_in, -7); near(local.y_in, 0);

    // ---- Estimator: the noise screen ------------------------------------
    auto fix = [](double x, double y, double heading) {
        GpsPose p{}; p.ok = true; p.x_in = x; p.y_in = y; p.theta_deg = heading;
        return p;
    };
    const GpsPose none{};
    const double dt = 0.1;
    {   // Needs three agreeing fixes before it reports anything.
        Estimator e;
        assert(!e.update(0, 0, 180, fix(40, 10, 90), dt).ok);
        assert(!e.update(0, 0, 180, fix(70, 10, 90), dt).ok);   // disagrees
        assert(!e.update(0, 0, 180, fix(40.5, 10, 90), dt).ok);
        assert(!e.update(0, 0, 180, fix(39.5, 10, 90), dt).ok);
        GpsPose p = e.update(0, 0, 180, fix(40, 10, 90), dt);
        assert(p.ok && !p.jump);
        near(p.x_in, 40); near(p.y_in, 10); near(p.theta_deg, 90);

        // Parked: +/-1.5 in of GPS jitter moves the estimate far less.
        double low = 1e9, high = -1e9;
        for (int i = 0; i < 60; ++i) {
            p = e.update(0, 0, 180, fix(40 + (i % 2 ? 1.5 : -1.5), 10, 90), dt);
            if (i > 10) { low = std::min(low, p.x_in); high = std::max(high, p.x_in); }
        }
        assert(high - low < 0.7 && std::abs(0.5 * (low + high) - 40) < 0.3);

        // One wild fix is ignored; the estimate holds and says so.
        const double before = p.x_in;
        p = e.update(0, 0, 180, fix(75, -20, 90), dt);
        assert(p.ok && p.jump); near(p.x_in, before);
        p = e.update(0, 0, 180, fix(40, 10, 90), dt);
        assert(p.ok && !p.jump);

        // No fix: follow odometry, rotated into the field frame. Odometry
        // heading 180 is field heading 90, so local -Y (forward) is field +X.
        const double x0 = p.x_in, y0 = p.y_in;
        for (int i = 1; i <= 10; ++i) p = e.update(0, -i, 180, none, dt);
        assert(p.ok && !p.jump);
        near(p.x_in, x0 + 10); near(p.y_in, y0); near(p.theta_deg, 90);
        // GPS returns, lagging 3 in behind a moving robot: still accepted.
        p = e.update(0, -12, 180, fix(x0 + 9, y0, 90), dt);
        assert(p.ok && !p.jump && p.x_in > x0 + 11 && p.x_in < x0 + 12);

        // Blur while spinning fast: fixes are not applied.
        const double spin_x = p.x_in;
        p = e.update(0, -12, 180 + 40, fix(spin_x + 4, y0, 130), dt);
        near(p.x_in, spin_x);

        // chassis.setPose() mid-route: odometry teleports, the robot did not.
        p = e.update(-0.5, 15, 90, none, dt);
        near(p.x_in, spin_x);
        p = e.update(-0.5, 15, 90, fix(spin_x, y0, 130), dt);
        near(p.theta_deg, 130);                 // frame re-derived from the fix
        p = e.update(-0.5, 16, 90, none, dt);   // local +Y, frame now +40 deg
        near(p.x_in, spin_x + std::sin(40 * kPi / 180));

        // Shoved while parked: the "outliers" agree with each other, so
        // after kRelockCount of them the estimate starts over from them.
        for (int i = 0; i < kRelockCount - 1; ++i) {
            p = e.update(-0.5, 16, 90, fix(-20, -30, 130), dt);
            assert(p.jump && p.x_in > 0);
        }
        p = e.update(-0.5, 16, 90, fix(-20, -30, 130), dt);
        assert(p.ok && !p.jump); near(p.x_in, -20); near(p.y_in, -30);

        // Long outage: stop reporting rather than dead-reckon forever.
        for (int i = 0; i < 18; ++i) p = e.update(-0.5, 16, 90, none, dt);
        assert(p.ok);                                // 1.8 s: still coasting
        for (int i = 0; i < 4; ++i) p = e.update(-0.5, 16, 90, none, dt);
        assert(!p.ok);                               // past kCoastS
    }

    // live_pose(): the same screen on the real read path.
    init(chassis, 10, 0, 0, 0);
    chassis.pose = {0, 0, 180};
    pros::Gps::error = infinity;
    assert(!live_pose().ok && !live_pose().jump);
    pros::Gps::error = 0.01;
    reading(10, 10, 0);
    for (int i = 0; i < kLockCount - 1; ++i) { pros::delay(100); assert(!live_pose().ok); }
    pros::delay(100);
    assert(live_pose().ok);
    reading(50, 10, 0);
    pros::delay(100);
    auto live = live_pose();
    assert(live.ok && live.jump); near(live.x_in, 10);
    reading(11, 10, 0);
    pros::delay(100);
    live = live_pose();
    assert(live.ok && !live.jump && live.x_in > 10 && live.x_in < 10.5);

    // ---- Start anchor and resets ----------------------------------------
    shared_state() = {};
    reading(64, 6, 90);
    init(chassis, 10, 0, 0);
    set_anchor_observer(anchor_changed);
    assert(!reset());   // absolute mode: a 64 in correction is not believable
    assert(capture_start_as(0, 0, 180));
    assert(anchor_notifications == 2 && notified_valid);
    near(notified_x, 64); near(notified_y, 6); near(notified_heading, 90);

    // Field (48, 21) is 16 in and 15 in from the start: route (15, 16).
    reading(48, 21, 90);
    auto route = to_auton_frame(read_pose(*shared_state().gps));
    near(route.x_in, 15); near(route.y_in, 16); near(route.theta_deg, 180);

    // reset(): applies a small stationary correction, keeps LemLib heading,
    // and refuses implausible ones or a moving chassis.
    chassis.pose = {14, 15, 177};
    assert(reset());
    near(chassis.pose.x, 15); near(chassis.pose.y, 16); near(chassis.pose.theta, 177);
    chassis.pose = {0, 0, 180};
    const int calls = chassis.set_pose_calls;
    assert(!reset());                              // 22 in away: not believable
    chassis.pose = {15, 16, 180};
    chassis.moving = true;
    assert(!reset());
    chassis.moving = false;
    pros::Gps::error = infinity;
    assert(!reset());
    reading(nan, 6, 90);
    pros::Gps::error = 0.01;
    assert(!reset());
    assert(chassis.set_pose_calls == calls);

    // Automatic correction: only when enabled, anchored, at rest and the
    // gap is believable; the estimate itself must not move when it fires.
    reading(48, 21, 90);                       // route (15, 16)
    chassis.pose = {15, 16, 180};
    for (int i = 0; i < 8; ++i) { pros::delay(100); live_pose(); }
    chassis.pose = {13, 16, 177};              // odometry is 2 in off
    shared_state().estimator.rebase(13, 16, 177);
    const int before_auto = chassis.set_pose_calls;
    for (int i = 0; i < 8; ++i) { pros::delay(100); live_pose(); }
    assert(chassis.set_pose_calls == before_auto);          // disabled
    set_auto_reset(true);
    chassis.moving = true;
    for (int i = 0; i < 8; ++i) { pros::delay(100); live_pose(); }
    assert(chassis.set_pose_calls == before_auto);          // in a motion
    chassis.moving = false;
    for (int i = 0; i < 8; ++i) { pros::delay(100); live = live_pose(); }
    assert(chassis.set_pose_calls == before_auto + 1);      // once, not repeatedly
    near(chassis.pose.x, 15); near(chassis.pose.y, 16); near(chassis.pose.theta, 177);
    near(live.x_in, 48); near(live.y_in, 21);
    chassis.pose = {15, 4, 177};               // 12 in off: not believable
    shared_state().estimator.rebase(15, 4, 177);
    for (int i = 0; i < 8; ++i) { pros::delay(100); live_pose(); }
    assert(chassis.set_pose_calls == before_auto + 1);
    set_auto_reset(false);

    // Route checkpoints: one correction, only if the GPS agrees the robot is
    // at the waypoint and not wildly far from odometry.
    const int before_cp = chassis.set_pose_calls;
    chassis.pose = {13, 16, 177};                         // 2 in off
    shared_state().estimator.rebase(13, 16, 177);
    request_checkpoint(30, 16, 6);                        // wrong waypoint
    pros::delay(50); live_pose();
    assert(chassis.set_pose_calls == before_cp && !shared_state().checkpoint_pending);
    request_checkpoint(15, 16, 6);
    chassis.moving = true;                                // not stopped yet
    pros::delay(50); live_pose();
    assert(chassis.set_pose_calls == before_cp && shared_state().checkpoint_pending);
    chassis.moving = false;
    pros::delay(50); live_pose();
    assert(chassis.set_pose_calls == before_cp + 1 && shared_state().checkpoint_applied);
    near(chassis.pose.x, 15); near(chassis.pose.y, 16); near(chassis.pose.theta, 177);
    chassis.pose = {14, 16, 177};                         // 1 in: leave it
    shared_state().estimator.rebase(14, 16, 177);
    request_checkpoint(15, 16, 6);
    pros::delay(50); live_pose();
    assert(chassis.set_pose_calls == before_cp + 1 && !shared_state().checkpoint_applied);
    chassis.pose = {15, 6, 177};                          // 10 in: not believable
    shared_state().estimator.rebase(15, 6, 177);
    request_checkpoint(15, 16, 6);
    pros::delay(50); live_pose();
    assert(chassis.set_pose_calls == before_cp + 1);
    // Nobody serving requests (or no anchor): checkpoint() gives up, no hang.
    assert(!checkpoint(15, 16));
    assert(!shared_state().checkpoint_pending);
    chassis.pose = {15, 16, 180};

    // A failed recapture invalidates the anchor (resets fail closed and the
    // map hides the motor marker); a successful one republishes it.
    reading(64, 6, 90);
    pros::Gps::error = infinity;
    assert(!capture_start_as(0, 0, 180));
    assert(anchor_notifications == 3 && !notified_valid);
    assert(!reset());
    pros::Gps::error = 0.01;
    assert(capture_start_as(0, 0, 180));
    assert(anchor_notifications == 5 && notified_valid);

    // The route frame honors a nonzero declared origin.
    shared_state().anchor_x_in = 64;
    shared_state().anchor_y_in = 6;
    shared_state().anchor_theta_deg = 180;
    shared_state().decl_x_in = 10;
    shared_state().decl_y_in = 20;
    shared_state().decl_theta_deg = 0;
    const auto r = to_auton_frame({true, 68, 9, 180, 0.01});
    near(r.x_in, 6); near(r.y_in, 17); near(r.theta_deg, 0);
}
'''


class GpsDisplayTests(unittest.TestCase):
    def test_live_display_and_coordinate_frame(self):
        with tempfile.TemporaryDirectory(prefix="gps-display-test-") as temp:
            directory = Path(temp)
            for relative in ("api.h", "lemlib/api.hpp", "pros/gps.hpp",
                             "pros/llemu.hpp"):
                header = directory / relative
                header.parent.mkdir(parents=True, exist_ok=True)
                header.write_text(FAKES if relative == "api.h" else
                                  '#pragma once\n#include "api.h"\n')
            (directory / "field_display_status.hpp").write_text(r'''
#pragma once
#include "api.h"
namespace fieldviz {
inline bool print(std::int16_t row, const char* format, ...) {
    char text[128];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    pros::lcd::print(row, "%s", text);
    return true;
}
}
''')
            source = directory / "checks.cpp"
            source.write_text(CHECKS)
            binary = directory / "checks"
            subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                            "-I", str(directory), "-I", str(ROOT / "include"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
