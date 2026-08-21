Baseline inspection on 2026-07-08:

- `src/main.cpp::autonomous()` currently runs `simple_goal_avoidance_auton()`.
- `src/main.cpp::opcontrol()` currently uses B+Down for `simple_goal_avoidance_auton()`.
- `src/main.cpp::opcontrol()` currently uses X+Down for `pid_autotune_auton()`.
- `src/autons.cpp` already contains onboard fused pose code with:
  - drivetrain motor encoder forward estimate
  - port 5 horizontal odometer side estimate
  - IMU heading through `chassis.drive_imu_get()`
  - IMU bias correction from gated LiDAR wall theta
  - wall hypothesis winner-margin rejection
  - bounded one-axis LiDAR distance correction

The missing piece is a small repeatable test routine that exercises this fused pose estimate directly.

Observed robot failure after first implementation:

- User manually uploaded the first version and pressed X+Down.
- Robot drove backward/turned immediately instead of driving a clean first 12 inch forward leg.
- Code cause: the first implementation used same-sign `chassis.drive_set()` power. Existing opcontrol uses left negative and right positive for physical forward, so same-sign drive power is not a valid physical forward command on this robot.
- The screen label also used the older fusion-test wording, making it harder to confirm the corrected firmware was loaded.
