"""Execute the production wall/reset and contact helpers on a scripted clock.

The bounded source sections are compiled verbatim, not reimplemented here.
Robot motion, range accuracy and wheel slip still require field validation.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

FAKES = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>
constexpr std::int32_t PROS_ERR = 2147483647;
namespace pros {
inline std::uint32_t clock_ms = 0;
inline std::function<void()> tick = [] {};
inline std::uint32_t millis() { return clock_ms; }
inline void delay(std::uint32_t ms) { clock_ms += ms; tick(); }
}
struct Cycle { double rpm = 0; int a = 175, b = 175; };
std::vector<Cycle> cycles(20);
std::function<double()> rpm_override;
Cycle current_cycle() {
    return cycles[std::min<std::size_t>(pros::clock_ms / 10, cycles.size()-1)];
}
struct MotorGroup {
    std::vector<double> get_actual_velocity_all() const {
        const double v = rpm_override ? rpm_override() : current_cycle().rpm;
        return {v, v};
    }
} left_motors, right_motors;
struct Distance {
    bool a;
    std::vector<std::uint32_t> read_times;
    int get_distance() {
        read_times.push_back(pros::millis());
        return a ? current_cycle().a : current_cycle().b;
    }
} wall_dist_a{true, {}}, wall_dist_b{false, {}};
namespace lemlib { struct Pose { float x, y, theta; }; }
struct Chassis {
    lemlib::Pose pose{55,15,90};
    int sets=0, cancels=0;
    bool moving=false;
    std::function<bool()> motion_override;
    bool isInMotion() { return motion_override ? motion_override() : moving; }
    void cancelMotion() { ++cancels; moving=false; pros::delay(10); }
    lemlib::Pose getPose() const { return pose; }
    void setPose(float x, float y, float theta) { ++sets; pose={x,y,theta}; }
} chassis;
namespace three_pin {
struct Point { float x,y; };
constexpr Point kGoal{20,15};
int brakes=0, coasts=0;
void brake_drive() { ++brakes; }
void coast_drive() { ++coasts; }
'''

CHECKS = r'''
void reset() {
    pros::clock_ms=0;
    pros::tick=[] {};
    cycles.assign(20, {});
    rpm_override={};
    wall_dist_a.read_times.clear(); wall_dist_b.read_times.clear();
    chassis=Chassis{};
    three_pin::brakes=three_pin::coasts=0;
}
void near(double a, double b) { assert(std::abs(a-b)<0.0001); }
int main(int argc, char** argv) {
    assert(argc==2);
    const std::string test=argv[1];
    reset();
    if (test=="complete") {
        const auto r=skills_route::read_wall();
        assert(r.ok && r.samples==6);
        assert(pros::clock_ms==50);
        const std::vector<std::uint32_t> expected{0,10,20,30,40,50};
        assert(wall_dist_a.read_times==expected);
        assert(wall_dist_b.read_times==expected);
        near(r.a_in,175/25.4f); near(r.b_in,175/25.4f); near(r.rms_in,0);
    } else if (test=="invalid") {
        for (auto& c:cycles) c.a=PROS_ERR;
        const auto r=skills_route::read_wall();
        assert(!r.ok && r.samples==0 && pros::clock_ms==200);
        assert(wall_dist_a.read_times.size()==20);
    } else if (test=="motion_clears") {
        for (auto& c:cycles) c.a=c.b=PROS_ERR;
        cycles[0]=cycles[1]=cycles[2]={0,254,254};
        cycles[3]={20,254,254};
        cycles[4]=cycles[5]={0,267,267};
        const auto r=skills_route::read_wall();
        assert(r.samples==2 && !r.ok && pros::clock_ms==200);
    } else if (test=="one_stop_only") {
        for (auto& c:cycles) c.a=c.b=PROS_ERR;
        cycles[0]=cycles[1]=cycles[2]={0,254,254};
        cycles[3]={20,254,254};
        cycles[4]=cycles[5]=cycles[6]={0,267,267};
        const auto r=skills_route::read_wall();
        assert(r.samples==3 && r.ok);
        near(r.a_in,267/25.4f);  // no plausible-looking pre/post-motion average
    } else if (test=="fresh_pose") {
        chassis.pose={45,15,90}; // stale X would reject the correction (>4 in)
        pros::tick=[] { chassis.pose={56,17,92}; };
        const auto fix=skills_route::south_wall_reset();
        assert(fix.applied && chassis.sets==1);
        const float expected_x=-4-(-70.63f+(4.5f+175/25.4f)*std::cos(2*M_PI/180));
        near(chassis.pose.x,expected_x);
        near(chassis.pose.y,17); near(chassis.pose.theta,92);
        near(fix.dy_in,expected_x-56); // innovation uses the current X
        assert(three_pin::brakes==1 && three_pin::coasts==1);
    } else if (test=="heading_drift") {
        pros::tick=[] { chassis.pose.theta=96; };
        const auto fix=skills_route::south_wall_reset();
        assert(!fix.applied && chassis.sets==0);
        assert(std::string(fix.why)=="not square");
    } else if (test=="late_motion") {
        rpm_override=[] { return wall_dist_b.read_times.size()>=6 ? 20.0 : 0.0; };
        const auto fix=skills_route::south_wall_reset();
        assert(!fix.applied && chassis.sets==0);
        assert(std::string(fix.why)=="moving or invalid pose");
    } else if (test=="bad_velocity") {
        rpm_override=[] { return std::numeric_limits<double>::quiet_NaN(); };
        const auto fix=skills_route::south_wall_reset();
        assert(!fix.applied && chassis.sets==0);
    } else if (test=="late_contact") {
        pros::clock_ms=500; chassis.moving=true;
        assert(three_pin::wait_done_or_stalled(0));
        assert(pros::clock_ms==660 && chassis.cancels==1);
    } else if (test=="restart_contact") {
        pros::clock_ms=500; chassis.moving=true;
        assert(three_pin::wait_done_or_stalled(500));
        assert(pros::clock_ms==960 && chassis.cancels==1);
    } else if (test=="contact_rollover") {
        const std::uint32_t started=std::numeric_limits<std::uint32_t>::max()-200;
        pros::clock_ms=started+500; chassis.moving=true;
        const std::uint32_t entered=pros::clock_ms;
        assert(three_pin::wait_done_or_stalled(started));
        assert(pros::clock_ms-entered==160);
    } else if (test=="normal_completion") {
        chassis.motion_override=[] { return pros::clock_ms<40; };
        assert(!three_pin::wait_done_or_stalled());
        assert(pros::clock_ms==40 && chassis.cancels==0);
    } else if (test=="interrupted_contact") {
        pros::clock_ms=500; chassis.moving=true;
        rpm_override=[] { return pros::clock_ms>=580 && pros::clock_ms<600 ? 20.0 : 0.0; };
        assert(three_pin::wait_done_or_stalled(0));
        assert(pros::clock_ms==760); // a fresh 150 ms low-speed observation
    } else assert(false);
}
'''


class SkillsWallContactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / "src/autons.cpp").read_text()
        contact = source[source.index("constexpr double kStalledRpm"):
                         source.index("// The point `distance` straight ahead")]
        wall = source[source.index("constexpr float kSensorFwdIn"):
                      source.index('// "Down" for the loader:')]
        frame = r'''
} // namespace three_pin
namespace skills_route {
using three_pin::Point;
constexpr Point kG2Field{-48,-24};
constexpr float field_heading(float deg) { return deg-90; }
'''
        cls.tmp = tempfile.TemporaryDirectory()
        path = Path(cls.tmp.name)
        cpp = path / "checks.cpp"
        cpp.write_text(FAKES + contact + frame + wall + "\n}\n" + CHECKS)
        cls.binary = path / "checks"
        subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                        str(cpp), "-o", str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def check(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=5)

    def test_complete_batch_keeps_six_sample_times(self): self.check("complete")
    def test_invalid_readings_keep_the_bounded_retry_budget(self): self.check("invalid")
    def test_motion_discards_incomplete_batch(self): self.check("motion_clears")
    def test_samples_describe_only_the_latest_stop(self): self.check("one_stop_only")
    def test_reset_preserves_fresh_pose_components(self): self.check("fresh_pose")
    def test_heading_is_rechecked_after_sampling(self): self.check("heading_drift")
    def test_motion_after_final_sample_rejects_reset(self): self.check("late_motion")
    def test_invalid_velocity_cannot_prove_stillness(self): self.check("bad_velocity")
    def test_late_contact_wait_does_not_restart_launch_grace(self): self.check("late_contact")
    def test_restarted_motion_gets_its_own_launch_grace(self): self.check("restart_contact")
    def test_contact_elapsed_handles_clock_rollover(self): self.check("contact_rollover")
    def test_normal_motion_completion_is_not_contact(self): self.check("normal_completion")
    def test_motion_interrupts_sustained_contact_observation(self): self.check("interrupted_contact")


if __name__ == "__main__":
    unittest.main()
