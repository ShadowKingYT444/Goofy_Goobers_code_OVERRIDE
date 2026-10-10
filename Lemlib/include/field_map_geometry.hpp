#pragma once

#include "gps_reset/pose_transform.hpp"
#include <cmath>

namespace fieldviz {
// Autonomous routes are written relative to the start, declared as local
// (0, 0, 180). The GPS pose captured at the start is where that local origin
// sits on the field, so both estimates begin at the same point on the map.
constexpr gpsreset::FramePose kLocalStart{0.0, 0.0, 180.0};
constexpr int kMapLeft = 18;
constexpr int kMapTop = 28;
constexpr int kMapSize = 192;

inline gpsreset::FramePose to_field(gpsreset::FramePose local,
                                  gpsreset::FramePose field_start) {
    return gpsreset::transform_pose(local, kLocalStart, field_start);
}

inline gpsreset::FramePose to_local(gpsreset::FramePose field,
                                  gpsreset::FramePose field_start) {
    return gpsreset::transform_pose(field, field_start, kLocalStart);
}

struct Pixel { bool visible; int x; int y; };
inline Pixel to_pixel(gpsreset::FramePose field) {
    if (!std::isfinite(field.x_in) || !std::isfinite(field.y_in) ||
        !std::isfinite(field.theta_deg) || std::abs(field.x_in) > 72 ||
        std::abs(field.y_in) > 72)
        return {false, 0, 0};
    return {true, kMapLeft + static_cast<int>(std::lround((field.x_in + 72) * kMapSize / 144.0)),
            kMapTop + static_cast<int>(std::lround((72 - field.y_in) * kMapSize / 144.0))};
}
}  // namespace fieldviz
