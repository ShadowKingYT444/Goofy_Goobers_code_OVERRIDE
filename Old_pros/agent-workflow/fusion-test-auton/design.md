Design:

`fusion_test_auton()` is a coordinate-first 12-45-12 test.

The routine:

1. Resets the drive sensors, horizontal odom wheel, and drive IMU.
2. Initializes the fused pose at the test origin.
3. Builds an absolute route from the current pose:
   - `wp1 = start + 12 in at start heading`
   - `turn_heading = start heading - 45 deg`
   - `wp2 = wp1 + 12 in at turn_heading`
4. Logs the route with `FUSE_TEST route=start ...`.
5. Drives to each waypoint with `fused_drive_to_point(...)`, which recomputes distance, bearing, heading error, and motor power from fused x/y/heading every 20 ms.
6. Turns with `fused_turn_to_heading(...)`, which uses fused field heading rather than raw IMU-relative heading.
7. Uses smoothed direct PROS motor power with slew limits for both forward and turn commands.
8. Avoids EZ-Template turn commands in this diagnostic because EZ's default drive signs made the physical robot back up during the turn.
9. Uses LiDAR wall theta as soft IMU bias during active fused movement, so clean wall observations can help heading without hard-resetting the drive IMU mid-command.
10. Samples fusion during every movement wait and during short 40 ms chain samples between commands.

LiDAR wall fusion is heading-only for this test:

- Distance readings still gate and disambiguate the wall hypothesis.
- Accepted wall-theta observations reset the drive IMU through `chassis.drive_imu_reset(...)` on a 1 second cadence.
- Active fused movement loops call `update_pose(..., LidarFusionMode::kBiasOnly)`, so LiDAR correction can bias fused heading but cannot hard-reset the IMU during motion.
- LiDAR no longer writes `pose.x` or `pose.y`; translational pose stays from drivetrain encoders plus horizontal odom until a real vertical odom wheel exists.

Full-system movement:

- `update_pose()` fuses drivetrain encoders, port 5 horizontal odom, IMU heading, and optional LiDAR wall theta.
- `fused_drive_to_point()` controls forward and turn power from fused x/y/heading.
- `fused_turn_to_heading()` controls turn power from fused heading.
- The old encoder-bounded drive and raw-IMU turn helpers remain in the file for fallback/debugging, but `fusion_test_auton()` no longer calls them.

Turn stabilization:

- The turn loop no longer uses the old `abs(error) * 2 + minPower` controller.
- It uses `command_cw = kP * error + kD * errorRate`, clamps to the requested max turn speed, and slews power frame-to-frame.
- Minimum turn power only applies when the error is more than 4 degrees, so the robot can settle near 45 degrees without hard reversing.

Entry points:

- `autonomous()` runs `fusion_test_auton()`.
- X+Down in opcontrol starts `fusion_test_auton()` in a task.
- B+Down still runs the older bypass autonomous.
