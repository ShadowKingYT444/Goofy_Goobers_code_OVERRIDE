Before changes, tools/lidar_bar_server.py already parsed optional m17/m18/m11/m13 motor positions and displayed motor degrees, but it had no field map or differential-drive odometry view.

Before the horizontal odometer change, the dashboard field dot only used left/right drive motor deltas. Sideways omni-wheel movement could not move the dot because it can happen without drive motor rotation. LiDAR line fitting displayed distance, angle, and RMSE, but it did not constrain or smooth the field pose.
