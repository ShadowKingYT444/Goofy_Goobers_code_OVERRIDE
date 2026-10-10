#pragma once

#include "field_map_geometry.hpp"
#include "field_display_status.hpp"

namespace fieldviz {
struct Telemetry {
    gpsreset::FramePose motor{0, 0, 180};
    gpsreset::FramePose gps_field{0, 0, 0};
    bool gps_ok = false;
    bool gps_jump = false;  // estimate is holding; latest raw fix ignored
    double imu_deg = 0;
    double lift_deg = 0;
    double claw_deg = 0;
};

// Publish data only. All LVGL work runs in the display daemon, never in the
// sensor/control task. The GPS pose captured at the start (field inches,
// heading) is what local (0, 0, 180) means on the field.
void start();
void set_anchor(bool valid, double field_x_in, double field_y_in,
                double field_heading);
void publish(const Telemetry& sample);
void status(const char* text);
}  // namespace fieldviz
