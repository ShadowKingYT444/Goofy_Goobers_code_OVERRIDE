Current behavior found before any localization code changes:

- `src/main.cpp` constructs the drivetrain on ports 17/18 and 11/13, IMU port 4, four Distance sensors on ports 6-9, and `horizontal_odom` on port 5.
- `src/main.cpp` emits D4 frames at 20 ms cadence with distance mm/confidence/installed fields, drive motor positions, and `h5=<centidegrees>`.
- `tools/lidar_bar_server.py` parses D4 frames and duplicates the same relative pose math in browser JavaScript.
- `src/autons.cpp` estimates pose only inside `simple_goal_avoidance_auton()`.
- The current estimator initializes x=0, y=0, heading=90 deg, then integrates drivetrain encoder deltas for forward and heading.
- The current estimator integrates port 5 horizontal odometer deltas for lateral movement and compensates for its rear offset during turns.
- The current onboard LiDAR correction fits a line across four sensors and rejects missing, negative, far, high-RMSE, or high-point-error readings.
- The current onboard LiDAR correction does not check distance confidence, robot rotation speed, or a wall-hypothesis winner margin.
- The current onboard LiDAR correction projects x or y directly onto the selected wall distance. It does not use wall theta to correct IMU bias.
- The IMU exists in the EZ-Template `Drive` object, but `src/autons.cpp` does not call `chassis.drive_imu_get()`, `chassis.imu.get_heading()`, or `chassis.imu.get_rotation()`.
