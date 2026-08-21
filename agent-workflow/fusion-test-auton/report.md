Changed files:

- `src/autons.cpp`
- `src/main.cpp`
- `include/autons.hpp`
- `agent-workflow/fusion-test-auton/*`

Latest update:

- Added full fused movement controllers for the X+Down test.
- `fused_drive_to_point(...)` now controls waypoint movement from fused x/y/heading every loop.
- `fused_turn_to_heading(...)` now controls heading from fused field heading every loop.
- Active fused movement uses `LidarFusionMode::kBiasOnly`, so clean LiDAR wall theta can adjust heading bias without hard-resetting the IMU mid-command.
- `fusion_test_auton()` no longer calls the old `drive_to_field_target(...)` / `turn_to_field_heading(...)` path.
- Source check passed, PROS build was current, and the binary uploaded to V5 slot 1.

Previous update:

- Fixed the turn oscillation path observed on the pose grid.
- `update_pose(...)` now accepts `LidarFusionMode`.
- Active drive and turn loops call `update_pose(..., false)`, so LiDAR wall theta cannot reset the drive IMU while a fixed movement target is executing.
- Replaced the old hard-minimum turn controller with damped `kP/kD` control, `command_cw`, and power slew limiting.
- Turn logs now include `rate`, `command_cw`, and `power`.
- Source check, PROS build, and upload to V5 slot 1 passed after this fix.

What changed:

- `fusion_test_auton()` now builds a field-coordinate route first:
  - start pose
  - waypoint 1 = 12 inches forward from start
  - target heading = start heading - 45 degrees
  - waypoint 2 = 12 inches from waypoint 1 along the new heading
- Movement commands are derived from those coordinates:
  - `drive_to_field_target(...)` computes route/bearing from the fused pose, but caps each command to the planned 12 inch segment.
  - `drive_forward_test_leg(...)` stops from measured drivetrain encoder travel instead of waiting on `move_relative()` targets.
  - `turn_to_field_heading(...)` computes field heading error from the fused pose, converts it to clockwise IMU-relative degrees, then drives the motors directly with the robot's physical turn signs.
- Movements are chained with only a 40 ms fusion sample between phases.
- The fused pose loop still runs during every movement wait, so drivetrain encoders, horizontal odom, IMU, and LiDAR are logged together.
- LiDAR wall fusion is heading-only for this test:
  - wall distance still gates and chooses the wall hypothesis
  - accepted wall theta corrections reset the drive IMU every 1 second
  - LiDAR no longer corrects `pose.x` or `pose.y`
- X+Down still starts `fusion_test_auton()`, with the screen label `X+Down real fusion`.
- Competition `autonomous()` still runs `fusion_test_auton()`.
- B+Down still runs the older bypass autonomous.

Observed failures that drove the fixes:

- First version used same-sign `chassis.drive_set()` and the robot drove backward/turned immediately.
- Second version tracked degrees/sensor values, but the manual power loop was too slow, left large gaps, and did not complete the full 12 inch / 45 degree intended movements.
- The latest correction makes odometry matter by planning field-coordinate targets before issuing bounded direct PROS motor movements.
- The next correction removed `move_relative()` and EZ turns from this diagnostic because they caused long waits, backward turn behavior, and inflated second-leg distance when pose error made the remaining distance larger than 12 inches.

Commands run:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
```

Result: passed.

```powershell
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Result: passed. `src/autons.cpp` compiled with existing LVGL enum warnings and the linker emitted an RWX segment warning, but `bin/hot.package.bin` was created.

```powershell
pros lsusb --target v5
```

Result: V5 detected on `COM9` communications and `COM8` user ports.

```powershell
.\.venv\Scripts\python.exe -m pros.cli.main upload --slot 1 --name MoreVex --description MoreVex --after none
```

Result: passed. Program `MoreVex` uploaded to V5 slot 1 on `COM9`.

The final upload in this report is the bounded-drive/direct-IMU-turn version.

Failed attempts:

- Fused waypoint follower control path: wrong physical drive signs.
- Direct power loop: too slow and did not complete the intended moves cleanly.
- `move_relative()` plus EZ turn version: long pause after first drive, turn backed up, and second leg could command more than 12 inches from pose error.
- Previous direct turn version: UI showed heading/path squiggles and the robot oscillated around 10-30 degrees instead of reaching 45. Likely contributors were LiDAR IMU reset during the active turn and a hard minimum turn power that reversed too aggressively near target.
- Previous movement version: pose was fused, but drive control still stopped from encoder inches and turn control still used raw IMU-relative heading. It was not a true fused motion controller.
- Earlier PROS build attempts failed on the external `make.exe` access issue; the current run built successfully.

Remaining risks:

- Forward distance still uses drivetrain encoders until a vertical tracking wheel exists.
- LiDAR heading correction is conditional on a clean, unambiguous wall observation.
- If the direct IMU turn is physically reversed, flip the sign of `signed_turn_power` in `turn_clockwise_test_leg()`.
- If the turn still undershoots without oscillating, increase `turn_speed` or `kFusionTestTurnKp` slightly.
- If the turn still oscillates, lower `kFusionTestTurnKp` or `kFusionTestTurnMinPower`.
- If logged `traveled` does not match real inches, calibrate `kWheelDiameterIn`, `kLeftEncoderSign`, `kRightEncoderSign`, or drivetrain gearing before trusting drivetrain encoder distance.
- Physical 12-45-12 accuracy still needs to be measured after this slot-1 upload.

What was not done:

- No physical run was performed by Codex.
- No vertical odom wheel integration was added.
- No AI Vision landmark correction was added.

Manual verification steps:

1. Run slot 1 on the V5 Brain.
2. Confirm the Brain shows `X+Down real fusion`.
3. Press X+Down.
4. Confirm the first command drives to waypoint 1, about 12 inches forward.
5. Confirm the turn goes clockwise about 45 degrees.
6. Confirm the second command drives toward waypoint 2 along the new field heading.
7. Watch terminal/dashboard logs for `type=fused_drive_to`, `controller=fused_drive`, `type=fused_turn_to`, `controller=fused_turn`, shrinking `dist`, shrinking heading `error`, and optional `imu_correction mode=bias`.
