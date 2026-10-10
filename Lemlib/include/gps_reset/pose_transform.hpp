#pragma once

#include <cmath>

namespace gpsreset {

struct FramePose {
    double x_in;
    double y_in;
    double theta_deg;
};

// Headings are clockwise from +Y. Use the captured heading to align field
// axes with the auton's declared heading, even when the robot faces a wall.
inline FramePose transform_pose(FramePose field_pose, FramePose field_origin,
                                FramePose declared_origin) {
    constexpr double pi = 3.14159265358979323846;
    const double angle =
        (declared_origin.theta_deg - field_origin.theta_deg) * pi / 180.0;
    const double dx = field_pose.x_in - field_origin.x_in;
    const double dy = field_pose.y_in - field_origin.y_in;
    double heading = std::fmod(field_pose.theta_deg - field_origin.theta_deg +
                                  declared_origin.theta_deg,
                              360.0);
    if (heading < 0.0) heading += 360.0;
    return {declared_origin.x_in + dx * std::cos(angle) + dy * std::sin(angle),
            declared_origin.y_in - dx * std::sin(angle) + dy * std::cos(angle),
            heading};
}

}  // namespace gpsreset
