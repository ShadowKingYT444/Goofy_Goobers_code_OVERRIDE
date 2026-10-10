#include "field_map.hpp"
#include "api.h"
#include "liblvgl/lvgl.h"
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace fieldviz {
namespace {
struct State {
    pros::Mutex mutex;
    Telemetry sample;
    bool anchor_ok = false;
    gpsreset::FramePose anchor{0, 0, 0};
    char status[96] = "Starting sensors...";
    char report[256] = "";
};
State& state() { static State value; return value; }
constexpr std::uint32_t kMotorColor = 0x36d9ee;
constexpr std::uint32_t kGpsColor = 0xffcf40;
constexpr std::uint32_t kRefreshMs = 100;

// The installed LVGL formatter has floating-point formatting disabled.
// libc's vsnprintf supports decimals; LVGL copies this temporary buffer.
void label_format(lv_obj_t* object, const char* format, ...) {
    char text[160];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    lv_label_set_text(object, text);
}

struct Marker {
    lv_obj_t* dot = nullptr;
    lv_obj_t* arrow = nullptr;
    lv_point_precise_t points[2]{};  // LVGL keeps the pointer; never use a stack array.
};
struct View {
    lv_obj_t* screen = nullptr;
    lv_obj_t* motor = nullptr;
    lv_obj_t* motor_field = nullptr;
    lv_obj_t* gps = nullptr;
    lv_obj_t* gps_field = nullptr;
    lv_obj_t* delta = nullptr;
    lv_obj_t* status = nullptr;
    lv_obj_t* auxiliary = nullptr;
    lv_obj_t* start_h = nullptr;
    lv_obj_t* start_v = nullptr;
    lv_obj_t* start_label = nullptr;
    lv_obj_t* report = nullptr;
    Marker motor_marker, gps_marker;
};

lv_obj_t* label(lv_obj_t* parent, int x, int y, int width,
                const char* text, std::uint32_t color = 0xe4edf7) {
    auto* object = lv_label_create(parent);
    lv_obj_set_pos(object, x, y);
    lv_obj_set_width(object, width);
    lv_obj_set_style_text_color(object, lv_color_hex(color), 0);
    lv_label_set_long_mode(object, LV_LABEL_LONG_CLIP);
    lv_label_set_text(object, text);
    return object;
}

lv_obj_t* box(lv_obj_t* parent, int x, int y, int w, int h,
              std::uint32_t color) {
    auto* object = lv_obj_create(parent);
    lv_obj_remove_style_all(object);
    lv_obj_remove_flag(object, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(object, x, y);
    lv_obj_set_size(object, w, h);
    lv_obj_set_style_bg_color(object, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, 0);
    return object;
}

void create_marker(lv_obj_t* parent, Marker& marker, std::uint32_t color,
                   int size, bool ring) {
    marker.arrow = lv_line_create(parent);
    lv_obj_set_style_line_color(marker.arrow, lv_color_hex(color), 0);
    lv_obj_set_style_line_width(marker.arrow, 2, 0);
    marker.dot = box(parent, 0, 0, size, size, color);
    lv_obj_set_style_radius(marker.dot, ring ? LV_RADIUS_CIRCLE : 1, 0);
    if (ring) {
        lv_obj_set_style_bg_opa(marker.dot, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(marker.dot, lv_color_hex(color), 0);
        lv_obj_set_style_border_width(marker.dot, 2, 0);
    }
    lv_obj_add_flag(marker.dot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(marker.arrow, LV_OBJ_FLAG_HIDDEN);
}

void create_view(View& v) {
    v.screen = lv_obj_create(nullptr);
    lv_obj_remove_style_all(v.screen);
    lv_obj_remove_flag(v.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(v.screen, lv_color_hex(0x101923), 0);
    lv_obj_set_style_bg_opa(v.screen, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(v.screen, LV_FONT_DEFAULT, 0);
    label(v.screen, 18, 5, 210, "FIELD XY (inches)");
    box(v.screen, kMapLeft, kMapTop, kMapSize + 1, kMapSize + 1, 0x1b2938);
    for (int i = 0; i <= 6; ++i) {
        const int offset = i * kMapSize / 6;
        const auto color = i == 3 ? 0x728397 : 0x3b4c60;
        box(v.screen, kMapLeft + offset, kMapTop, 1, kMapSize + 1, color);
        box(v.screen, kMapLeft, kMapTop + offset, kMapSize + 1, 1, color);
    }
    label(v.screen, 2, 28, 16, "+Y");
    label(v.screen, 85, 224, 85, "0       +72 X", 0xa4b5c9);
    label(v.screen, 18, 224, 45, "-72", 0xa4b5c9);
    // Positioned by move_start() once the GPS start anchor is known.
    v.start_h = box(v.screen, 0, 0, 9, 1, 0xffffff);
    v.start_v = box(v.screen, 0, 0, 1, 9, 0xffffff);
    v.start_label = label(v.screen, 0, 0, 35, "start", 0xffffff);
    label(v.screen, 230, 5, 246, "CYAN motor  /  YELLOW GPS");
    label(v.screen, 230, 28, 246, "MOTOR + IMU (local)", kMotorColor);
    v.motor = label(v.screen, 230, 46, 246, "Waiting for odometry", kMotorColor);
    v.motor_field = label(v.screen, 230, 82, 246, "Field: needs start anchor", kMotorColor);
    label(v.screen, 230, 106, 246, "GPS (local)", kGpsColor);
    v.gps = label(v.screen, 230, 124, 246, "Waiting for GPS", kGpsColor);
    v.gps_field = label(v.screen, 230, 160, 246, "Field: no fix", kGpsColor);
    v.delta = label(v.screen, 230, 181, 246, "Difference: --");
    v.status = label(v.screen, 230, 203, 246, "Starting...");
    v.auxiliary = label(v.screen, 230, 223, 246, "");
    create_marker(v.screen, v.motor_marker, kMotorColor, 7, false);
    create_marker(v.screen, v.gps_marker, kGpsColor, 13, true);
    // Created last so it covers the readout column while a report is shown.
    v.report = label(v.screen, 222, 0, 258, "", 0xffffff);
    lv_obj_set_size(v.report, 258, 240);
    lv_obj_set_style_bg_color(v.report, lv_color_hex(0x101923), 0);
    lv_obj_set_style_bg_opa(v.report, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(v.report, 6, 0);
    lv_obj_add_flag(v.report, LV_OBJ_FLAG_HIDDEN);
    lv_screen_load(v.screen);
}

void move_marker(Marker& marker, gpsreset::FramePose field, bool valid, int size) {
    const auto p = to_pixel(field);
    if (!valid || !p.visible) {
        lv_obj_add_flag(marker.dot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(marker.arrow, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_remove_flag(marker.dot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(marker.arrow, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(marker.dot, p.x - size / 2, p.y - size / 2);
    const double angle = field.theta_deg * 3.14159265358979323846 / 180.0;
    marker.points[0] = {static_cast<lv_value_precise_t>(p.x), static_cast<lv_value_precise_t>(p.y)};
    marker.points[1] = {static_cast<lv_value_precise_t>(p.x + 16 * std::sin(angle)),
                        static_cast<lv_value_precise_t>(p.y - 16 * std::cos(angle))};
    lv_line_set_points(marker.arrow, marker.points, 2);
}

void move_start(View& v, gpsreset::FramePose anchor, bool valid) {
    const auto p = to_pixel(anchor);
    for (auto* object : {v.start_h, v.start_v, v.start_label}) {
        if (valid && p.visible) lv_obj_remove_flag(object, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
    }
    if (!valid || !p.visible) return;
    lv_obj_set_pos(v.start_h, p.x - 4, p.y);
    lv_obj_set_pos(v.start_v, p.x, p.y - 4);
    lv_obj_set_pos(v.start_label, p.x - 16, p.y + 6);
}

// LVGL timer callback: runs exclusively on the display daemon, inside
// lv_timer_handler. No sensor calls or delays.
void render(lv_timer_t*) {
    static View view;
    Telemetry sample;
    bool anchor_ok;
    gpsreset::FramePose anchor;
    char message[96];
    char report[256];
    auto& shared = state();
    {
        std::lock_guard<pros::Mutex> guard(shared.mutex);
        sample = shared.sample;
        anchor_ok = shared.anchor_ok;
        anchor = shared.anchor;
        std::snprintf(message, sizeof(message), "%s", shared.status);
        std::snprintf(report, sizeof(report), "%s", shared.report);
    }
    if (!view.screen) create_view(view);
    auto motor_field = to_field(sample.motor, anchor);
    auto gps_local = to_local(sample.gps_field, anchor);
    label_format(view.motor, "X %.1f  Y %.1f\nHeading %.1f deg",
                          sample.motor.x_in, sample.motor.y_in, sample.motor.theta_deg);
    if (anchor_ok) {
        label_format(view.motor_field, "Field %.1f, %.1f%s", motor_field.x_in,
                              motor_field.y_in, to_pixel(motor_field).visible ? "" : " (off)");
    } else lv_label_set_text(view.motor_field, "Field: needs start anchor");
    if (sample.gps_ok) {
        if (anchor_ok)
            label_format(view.gps, "X %.1f  Y %.1f\nHeading %.1f deg",
                                  gps_local.x_in, gps_local.y_in, gps_local.theta_deg);
        else lv_label_set_text(view.gps, "Local: no start anchor");
        // "holding": the latest raw fix disagreed and is being ignored.
        label_format(view.gps_field, "Field %.1f, %.1f%s", sample.gps_field.x_in,
                              sample.gps_field.y_in,
                              !to_pixel(sample.gps_field).visible ? " (off)"
                              : sample.gps_jump ? " (holding)" : "");
    } else {
        lv_label_set_text(view.gps, "No confident GPS fix");
        lv_label_set_text(view.gps_field, "Field: no fix");
    }
    if (anchor_ok && sample.gps_ok)
        label_format(view.delta, "Difference: %.1f in",
                              std::hypot(motor_field.x_in - sample.gps_field.x_in,
                                         motor_field.y_in - sample.gps_field.y_in));
    else lv_label_set_text(view.delta, "Difference: --");
    lv_label_set_text(view.status, message);
    label_format(view.auxiliary, "IMU %.0f  Lift %.0f  Claw %.0f",
                          sample.imu_deg, sample.lift_deg, sample.claw_deg);
    if (report[0]) {
        lv_label_set_text(view.report, report);
        lv_obj_remove_flag(view.report, LV_OBJ_FLAG_HIDDEN);
    } else lv_obj_add_flag(view.report, LV_OBJ_FLAG_HIDDEN);
    move_start(view, anchor, anchor_ok);
    move_marker(view.motor_marker, motor_field, anchor_ok, 7);
    move_marker(view.gps_marker, sample.gps_field, sample.gps_ok, 13);
}
}  // namespace

// LVGL has no locking here, so the render timer may only be added while the
// display daemon is parked in its delay between lv_timer_handler calls. The
// daemon runs below user-task priority and LVGL never blocks inside the
// handler, so once it is seen blocked it cannot resume until this task yields.
void start() {
    static bool started = false;
    if (started) return;
    started = true;
    state();
    const pros::task_t daemon = pros::c::task_get_by_name("Display Daemon (PROS)");
    for (int tries = 0; daemon && tries < 200 &&
             pros::c::task_get_state(daemon) != pros::E_TASK_STATE_BLOCKED; ++tries)
        pros::delay(1);
    lv_timer_create(render, kRefreshMs, nullptr);
}
void set_anchor(bool valid, double field_x_in, double field_y_in,
                double field_heading) {
    auto& shared = state();
    std::lock_guard<pros::Mutex> guard(shared.mutex);
    shared.anchor_ok = valid && std::isfinite(field_x_in) &&
                       std::isfinite(field_y_in) && std::isfinite(field_heading);
    shared.anchor = {field_x_in, field_y_in, field_heading};
}
void publish(const Telemetry& sample) {
    auto& shared = state();
    std::lock_guard<pros::Mutex> guard(shared.mutex);
    shared.sample = sample;
}
void status(const char* text) {
    auto& shared = state();
    std::lock_guard<pros::Mutex> guard(shared.mutex);
    std::snprintf(shared.status, sizeof(shared.status), "%s", text);
}
}  // namespace fieldviz

// GPS/autotune tasks publish status text without touching any LVGL objects.
bool fieldviz::print(std::int16_t line, const char* format, ...) {
    if (line < 0 || line > 7) return false;
    char message[96];
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    fieldviz::status(message);
    return true;
}

void fieldviz::report(const char* format, ...) {
    char text[256];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    auto& shared = state();
    std::lock_guard<pros::Mutex> guard(shared.mutex);
    std::snprintf(shared.report, sizeof(shared.report), "%s", text);
}
