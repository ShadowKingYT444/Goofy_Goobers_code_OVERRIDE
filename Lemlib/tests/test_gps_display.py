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

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-6);
}
void reading(double x, double y, double heading) {
    pros::Gps::position = {x * 0.0254, y * 0.0254};
    pros::Gps::heading = heading;
}
int main() {
    using namespace gpsreset;
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    lemlib::Chassis chassis;

    // Startup/no anchor must not pretend that GPS has a local-frame pose.
    assert(!position_from_start(64, 6).anchor_ok);
    print_position_from_start(64, 6);
    assert(pros::lcd::line == 7 && pros::lcd::text == "GPS no anchor");

    // Physically facing the right wall (90 degrees), declared as 180 degrees.
    reading(64, 6, 90);
    init(chassis, 10, 7, 4.5);
    assert(capture_start_as(0, 0, 180));
    auto p = position_from_start(64, 6);
    assert(p.ok && p.anchor_ok);
    near(p.x_in, 0); near(p.y_in, 0); near(p.theta_deg, 180);

    // Inward 16 and up 15 field inches becomes local (15,16), not (-15,-16).
    reading(48, 21, 90);
    chassis.moving = true;  // Estimates must keep updating during driving.
    p = position_from_start(64, 6);
    assert(p.ok);
    near(p.x_in, 15); near(p.y_in, 16); near(p.theta_deg, 180);
    print_position_from_start(64, 6);
    assert(pros::lcd::text == "GPS X15.0 Y16.0 T180");

    // Turning in place changes only heading, including wraparound at 360.
    reading(64, 6, 350);
    p = position_from_start(64, 6);
    near(p.x_in, 0); near(p.y_in, 0); near(p.theta_deg, 80);

    // Use the physical start, not the noisy measured anchor, for the display.
    shared_state().anchor_x_in = 65;
    shared_state().anchor_y_in = 7;
    p = position_from_start(64, 6);
    near(p.x_in, 0); near(p.y_in, 0);

    // Lost fixes cannot keep displaying the preceding valid coordinates.
    pros::Gps::error = infinity;
    p = position_from_start(64, 6);
    assert(p.anchor_ok && !p.ok);
    print_position_from_start(64, 6);
    assert(pros::lcd::text == "GPS no fix");
    pros::Gps::error = 0.01;
    reading(nan, 6, 90);
    assert(!position_from_start(64, 6).ok);
    reading(64, 6, 90);
    assert(!position_from_start(nan, 6).ok);
    shared_state().anchor_valid = false;
    assert(!position_from_start(64, 6).anchor_ok);

    // The query/print path must never apply a reset or replace the anchor.
    assert(chassis.set_pose_calls == 0);
    assert(!shared_state().have_reset);
    near(shared_state().anchor_x_in, 65);
    near(shared_state().anchor_y_in, 7);

    // Existing reset transform still honors a nonzero declared origin.
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
            source = directory / "checks.cpp"
            source.write_text(CHECKS)
            binary = directory / "checks"
            subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                            "-I", str(directory), "-I", str(ROOT / "include"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
