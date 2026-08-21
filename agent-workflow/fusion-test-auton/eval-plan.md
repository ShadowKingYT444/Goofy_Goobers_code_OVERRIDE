Static checks:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
```

Build check:

```powershell
.\.venv\Scripts\pros.exe --version
.\.venv\Scripts\pros.exe lsusb --target v5
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Upload check:

```powershell
.\.venv\Scripts\python.exe -m pros.cli.main upload --slot 1 --name MoreVex --description MoreVex --after none
```

Manual robot test:

1. Upload the program.
2. Put the robot in a known starting pose.
3. Open the LiDAR dashboard if useful.
4. Press X+Down in opcontrol.
   - The Brain should show `X+Down real fusion` before the routine starts.
5. Confirm the robot attempts:
   - drive to the first field waypoint, about 12 inches forward
   - turn to the field heading 45 degrees clockwise from the start heading
   - drive to the second field waypoint, about 12 inches along the new heading
6. Watch the terminal/dashboard for `FUSE_TEST` route, `type=fused_drive_to`, `type=fused_turn_to`, `controller=fused_drive`, `controller=fused_turn`, and optional bias-mode `imu_correction` lines.
7. Measure final physical pose against the expected path.

Pass criteria:

- Source check passes.
- PROS CLI runs and can see the V5 Brain.
- PROS build passes or any toolchain failure is documented.
- `autonomous()` calls `fusion_test_auton()`.
- X+Down calls `fusion_test_auton()`.
- The test routine logs start, after-first-leg, after-turn, and final pose.
- `fusion_test_auton()` defines field-coordinate waypoints before commanding movement.
- `fusion_test_auton()` uses `fused_drive_to_point()` and `fused_turn_to_heading()`.
- Fused drive computes motor power from fused x/y/heading every loop.
- Fused turn computes motor power from fused field heading every loop.
- Active fused movement loops use `LidarFusionMode::kBiasOnly`.
- LiDAR can apply soft heading bias during motion, but cannot hard-reset the IMU during motion.
- Turn loop uses damped `kP/kD` control, command slew limiting, and no hard minimum power near the target.
- During movement, logs should show `controller=fused_*` and may show `FUSE_TEST imu_correction mode=bias` when a clean wall is accepted.
- The pose grid should not show the heading bouncing between about 10 and 30 degrees during the 45 degree command.
- Accepted LiDAR wall-theta corrections reset the drive IMU on a 1 second cadence.
- LiDAR wall correction does not write `pose.x` or `pose.y` in this test.
- Build and upload complete through PROS CLI.
- Manual test confirms whether signs/distances are correct; if not, record the correction needed.
