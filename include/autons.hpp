#pragma once

void default_constants();
void localization_telemetry_update();
void localization_telemetry_reset();
bool localization_set_runtime_start_pose(double x_in, double y_in, double heading_deg);
void localization_get_runtime_start_pose(double& x_in, double& y_in, double& heading_deg);
void localization_slow_rotation_calibration();
void localization_slow_forward_calibration();
void simple_goal_avoidance_auton();
void fusion_test_auton();
void gps_obstacle_aware_route_test();
