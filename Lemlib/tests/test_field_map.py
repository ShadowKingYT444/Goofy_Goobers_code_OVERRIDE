"""Run production map math/rendering on host fakes; hardware still needs a boot test."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
API = r'''
#pragma once
#include <mutex>
#include <cstdint>
namespace pros {
inline std::uint32_t clock_ms = 0;
inline std::uint32_t millis() { return clock_ms; }
inline void delay(std::uint32_t ms) { clock_ms += ms; }
using task_t = void*;
constexpr int E_TASK_STATE_BLOCKED = 2;
namespace c {
inline int daemon_polls = 0;
inline task_t task_get_by_name(const char*) { static int daemon; return &daemon; }
// The fake daemon is mid-handler for the first two polls.
inline int task_get_state(task_t) { return ++daemon_polls > 2 ? E_TASK_STATE_BLOCKED : 0; }
}
class Mutex {
    std::mutex value;
public:
    void lock() { value.lock(); }
    void unlock() { value.unlock(); }
};
}
'''
LVGL = r'''
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
using lv_value_precise_t = float;
struct lv_point_precise_t { float x, y; };
struct lv_obj_t {
    std::string text;
    int x=0, y=0, w=0, h=0, flags=0;
    std::uint32_t color=0;
    const lv_point_precise_t* points=nullptr;
};
inline std::vector<std::unique_ptr<lv_obj_t>> objects;
inline lv_obj_t* new_object() {
    objects.emplace_back(new lv_obj_t);
    return objects.back().get();
}
inline lv_obj_t* lv_obj_create(lv_obj_t*) { return new_object(); }
inline lv_obj_t* lv_label_create(lv_obj_t*) { return new_object(); }
inline lv_obj_t* lv_line_create(lv_obj_t*) { return new_object(); }
inline void lv_obj_remove_style_all(lv_obj_t*) {}
inline void lv_obj_remove_flag(lv_obj_t* p, int f) { p->flags &= ~f; }
inline void lv_obj_add_flag(lv_obj_t* p, int f) { p->flags |= f; }
inline void lv_obj_set_pos(lv_obj_t* p, int x, int y) { p->x=x; p->y=y; }
inline void lv_obj_set_size(lv_obj_t* p, int w, int h) { p->w=w; p->h=h; }
inline void lv_obj_set_width(lv_obj_t* p, int w) { p->w=w; }
inline void lv_obj_set_style_text_color(lv_obj_t* p, std::uint32_t c, int) { p->color=c; }
inline void lv_label_set_long_mode(lv_obj_t*, int) {}
inline void lv_label_set_text(lv_obj_t* p, const char* t) { p->text=t; }
inline std::uint32_t lv_color_hex(std::uint32_t c) { return c; }
inline void lv_obj_set_style_bg_color(lv_obj_t* p, std::uint32_t c, int) { p->color=c; }
inline void lv_obj_set_style_bg_opa(lv_obj_t*, int, int) {}
inline void lv_obj_set_style_radius(lv_obj_t*, int, int) {}
inline void lv_obj_set_style_border_color(lv_obj_t*, std::uint32_t, int) {}
inline void lv_obj_set_style_border_width(lv_obj_t*, int, int) {}
inline void lv_obj_set_style_line_color(lv_obj_t* p, std::uint32_t c, int) { p->color=c; }
inline void lv_obj_set_style_line_width(lv_obj_t*, int, int) {}
inline void lv_obj_set_style_text_font(lv_obj_t*, const void*, int) {}
inline void lv_screen_load(lv_obj_t*) {}
inline void lv_obj_set_style_pad_all(lv_obj_t*, int, int) {}
struct lv_timer_t {};
using lv_timer_cb_t = void (*)(lv_timer_t*);
inline lv_timer_cb_t timer_cb = nullptr;
inline std::uint32_t timer_period = 0;
inline int timers_created = 0;
inline lv_timer_t* lv_timer_create(lv_timer_cb_t cb, std::uint32_t period, void*) {
    static lv_timer_t timer;
    timer_cb = cb; timer_period = period; ++timers_created;
    return &timer;
}
inline void lv_line_set_points(lv_obj_t* p, const lv_point_precise_t* a, std::uint32_t) { p->points=a; }
constexpr int LV_OBJ_FLAG_HIDDEN=1, LV_OBJ_FLAG_SCROLLABLE=2, LV_LABEL_LONG_CLIP=0;
constexpr int LV_OPA_COVER=255, LV_OPA_TRANSP=0, LV_RADIUS_CIRCLE=100;
constexpr const void* LV_FONT_DEFAULT=nullptr;
'''
CHECKS = r'''
#include "field_map.hpp"
#include "api.h"
#include "liblvgl/lvgl.h"
#include <cassert>
#include <limits>
void near(double a, double b) { assert(std::abs(a-b)<1e-5); }
bool text(const char* expected) {
    for (const auto& p : objects) if (p->text==expected) return true;
    return false;
}
lv_obj_t* dot(std::uint32_t color, int size) {
    for (const auto& p : objects) if (p->color==color && p->w==size && p->h==size) return p.get();
    assert(false); return nullptr;
}
void tick() { pros::clock_ms+=100; if (timer_cb) timer_cb(nullptr); }
lv_obj_t* labelled(const char* expected) {
    for (const auto& p : objects) if (p->text==expected) return p.get();
    assert(false); return nullptr;
}
int main() {
    using namespace fieldviz;
    // Local (0,0,180) is wherever the GPS was captured at the start.
    for (gpsreset::FramePose a : {gpsreset::FramePose{64,6,90}, {-30,41.5,275}, {0,0,0}}) {
        auto origin=to_field({0,0,180},a);
        near(origin.x_in,a.x_in); near(origin.y_in,a.y_in); near(origin.theta_deg,a.theta_deg);
        auto back=to_local(a,a);
        near(back.x_in,0); near(back.y_in,0); near(back.theta_deg,180);
    }
    auto field=to_field({15,16,180},{64,6,90});
    near(field.x_in,48); near(field.y_in,21); near(field.theta_deg,90);
    for (double h : {0.,45.,90.,135.,180.,225.,270.,359.}) {
        const gpsreset::FramePose a{-20,33,h};
        auto local=to_local(to_field({12,-7,250},a),a);
        near(local.x_in,12); near(local.y_in,-7); near(local.theta_deg,250);
        // A turn in place must not alter X/Y after coordinate conversion.
        auto turned=to_field({12,-7,17},a);
        auto first=to_field({12,-7,250},a);
        near(turned.x_in,first.x_in); near(turned.y_in,first.y_in);
    }
    const auto corner=to_pixel({-72,72,0});
    assert(corner.visible && corner.x==kMapLeft && corner.y==kMapTop);
    const auto opposite=to_pixel({72,-72,180});
    assert(opposite.x==kMapLeft+kMapSize && opposite.y==kMapTop+kMapSize);
    assert(!to_pixel({72.1,0,0}).visible);
    assert(!to_pixel({0,std::numeric_limits<double>::quiet_NaN(),0}).visible);
    assert(!to_pixel({0,0,std::numeric_limits<double>::infinity()}).visible);

    // Nothing draws before start(); start() waits for the daemon to park
    // and installs exactly one render timer, however often it is called.
    tick(); assert(objects.empty() && timers_created==0);
    start(); start();
    assert(timers_created==1 && timer_period==100 && pros::c::daemon_polls==3);
    assert(objects.empty());  // Widgets are only ever created by the daemon.
    // A start somewhere other than the old assumed (64, 6): both markers
    // and the start cross must coincide there.
    set_anchor(true,-30,41.5,275);
    publish({{0,0,180},{-30,41.5,275},true,false,90,0,0});
    tick(); assert(text("Difference: 0.0 in"));
    assert(text("Field -30.0, 41.5"));
    auto* motor=dot(0x36d9ee,7);
    auto* gps=dot(0xffcf40,13);
    auto* cross=labelled("start");
    assert(!(cross->flags & LV_OBJ_FLAG_HIDDEN));
    assert(motor->x+3==gps->x+6 && motor->y+3==gps->y+6);
    assert(cross->x+16==motor->x+3 && cross->y-6==motor->y+3);
    set_anchor(true,64,6,90);
    publish({{0,0,180},{64,6,90},true,false,90,0,0});
    tick(); assert(text("Difference: 0.0 in"));
    assert(text("X 0.0  Y 0.0\nHeading 180.0 deg"));
    assert(!(motor->flags & LV_OBJ_FLAG_HIDDEN));
    assert(!(gps->flags & LV_OBJ_FLAG_HIDDEN));
    assert(motor->x+3==gps->x+6 && motor->y+3==gps->y+6);
    assert(cross->x+16==motor->x+3);
    const auto allocated=objects.size();
    publish({{10,16,180},{48,21,90},true,false,90,0,0});
    tick(); assert(text("Difference: 5.0 in"));
    assert(text("X 15.0  Y 16.0\nHeading 180.0 deg"));
    publish({{10,16,180},{80,21,90},true,false,90,0,0});
    tick(); assert(text("Field 80.0, 21.0 (off)"));
    assert(gps->flags & LV_OBJ_FLAG_HIDDEN);
    publish({{10,16,180},{0,0,0},false,false,90,0,0});
    tick(); assert(text("No confident GPS fix"));
    assert(text("Difference: --"));
    assert(gps->flags & LV_OBJ_FLAG_HIDDEN);
    publish({{10,16,180},{48,21,90},true,true,90,0,0});
    tick(); assert(text("Field 48.0, 21.0 (holding)"));
    assert(!(gps->flags & LV_OBJ_FLAG_HIDDEN));
    set_anchor(false,0,0,0);
    publish({{0,0,180},{64,6,90},true,false,90,0,0});
    tick(); assert(text("Local: no start anchor"));
    assert(motor->flags & LV_OBJ_FLAG_HIDDEN);
    assert(cross->flags & LV_OBJ_FLAG_HIDDEN);
    assert(!(gps->flags & LV_OBJ_FLAG_HIDDEN));
    report("TEST %d\nline two", 7);
    tick(); auto* panel=labelled("TEST 7\nline two");
    assert(!(panel->flags & LV_OBJ_FLAG_HIDDEN) && panel->w==258 && panel->h==240);
    report("");
    tick(); assert(panel->flags & LV_OBJ_FLAG_HIDDEN);
    assert(print(7,"GPS err %.2f",1.25));
    assert(!print(8,"bad"));
    tick(); assert(text("GPS err 1.25"));
    for (int i=0;i<100;++i) tick();
    assert(objects.size()==allocated);  // No repeated widget allocation.
}
'''

class FieldMapTests(unittest.TestCase):
    def test_frames_markers_confidence_and_rendering(self):
        with tempfile.TemporaryDirectory(prefix="field-map-test-") as temp:
            directory = Path(temp)
            (directory / "api.h").write_text(API)
            (directory / "liblvgl").mkdir()
            (directory / "liblvgl/lvgl.h").write_text(LVGL)
            (directory / "checks.cpp").write_text(CHECKS)
            binary = directory / "checks"
            subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                            "-pthread", "-I", str(directory), "-I", str(ROOT / "include"),
                            str(ROOT / "src/field_map.cpp"), str(directory / "checks.cpp"),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    unittest.main()
