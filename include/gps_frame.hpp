#pragma once

#include "localization_config.hpp"

#include <cmath>

namespace localization {

// VEX's absolute GPS field frame is a conventional top-down field view:
// native +X points right, native +Y points toward the 0-degree/top wall, and
// heading increases clockwise from that wall. The navigation frame instead
// uses +X toward the 0-degree wall, +Y toward the red/left side, and heading
// increases counterclockwise. Keep this conversion in one place; silently
// treating the two X/Y pairs as interchangeable rotates every fix by 90 deg.
struct ProjectGpsPose {
  double x_in;
  double y_in;
  double heading_deg;
  double robot_heading_cw_deg;
};

inline double wrap_gps_degrees(double degrees) {
  while (degrees >= 360.0) degrees -= 360.0;
  while (degrees < 0.0) degrees += 360.0;
  return degrees;
}

inline ProjectGpsPose vex_gps_to_project_robot_pose(
    double native_sensor_x_m,
    double native_sensor_y_m,
    double native_sensor_heading_cw_deg) {
  constexpr double kInchesPerMeter = 39.37007874015748;

  const double robot_heading_cw_deg = wrap_gps_degrees(
      native_sensor_heading_cw_deg - kGpsSensorHeadingOffsetCwDeg);
  const double project_heading_deg =
      wrap_gps_degrees(-robot_heading_cw_deg);

  // Rotate VEX native coordinates 90 degrees counterclockwise into the
  // project's axes: project X = native Y, project Y = -native X.
  const double sensor_project_x_in = native_sensor_y_m * kInchesPerMeter;
  const double sensor_project_y_in = -native_sensor_x_m * kInchesPerMeter;

  const double heading_rad = project_heading_deg *
      3.14159265358979323846 / 180.0;
  const double forward_x = std::cos(heading_rad);
  const double forward_y = std::sin(heading_rad);
  const double right_x = std::sin(heading_rad);
  const double right_y = -std::cos(heading_rad);

  return {
      sensor_project_x_in - kGpsForwardOffsetIn * forward_x -
          kGpsRightOffsetIn * right_x,
      sensor_project_y_in - kGpsForwardOffsetIn * forward_y -
          kGpsRightOffsetIn * right_y,
      project_heading_deg,
      robot_heading_cw_deg,
  };
}

}  // namespace localization
