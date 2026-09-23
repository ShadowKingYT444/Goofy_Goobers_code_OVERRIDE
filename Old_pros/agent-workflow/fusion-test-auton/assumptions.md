Assumptions before editing:

- The current fused pose code in `src/autons.cpp` is the source of truth for onboard LiDAR/odom/IMU/motor encoder fusion.
- The IMU is still configured through `Drive chassis(..., imu_port=4, ...)`.
- The horizontal odometer is still port 5 and reports centidegrees through `horizontal_odom`.
- The four distance sensors are still ports 6, 7, 8, and 9.
- The requested "turning 45 degrees" means a clockwise 45 degree turn from the current heading unless testing shows the sign is wrong.
- X+Down currently triggers PID autotune, but the new user request takes priority, so X+Down should trigger the fusion test auton instead.
