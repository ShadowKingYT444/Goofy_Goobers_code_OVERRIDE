Commands:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
```

Manual:

Open http://127.0.0.1:8774/ with robot streaming D4 frames. Confirm the field panel appears, pose updates when drivetrain moves, and LiDAR readouts still update.

Reset origin, slide the robot right without spinning the drive wheels, and confirm the dot moves right while Side Odom changes. Rotate in place and confirm the 12 cm odometer offset compensation does not create large fake side movement. Put a clean left-side wall under 50 in and confirm LiDAR Assist shows heading/wall context without changing x/y; move past 50 in and confirm assist turns off.

Pass criteria:

- Python syntax passes.
- Odometry sign regression passes.
- Dashboard loads.
- Field map does not break LiDAR readouts.
- Pose waits gracefully when motor telemetry is absent.
- Telemetry parser accepts both old motor-only frames and new frames with h5 centidegrees.
- Horizontal odometer sign regression passes.
- LiDAR assist/context only appears when RMSE <= 0.20 in and all wall distances are under 50 in.

Latest pose-grid criteria:

- Open `http://127.0.0.1:8774/pose-grid` or `http://127.0.0.1:8774/playing-field`.
- While the fusion autonomous is running, confirm the HUD switches to `Onboard fused pose`.
- Confirm the HUD x/y/heading match the latest `FUSE_TEST` log values.
- When no fresh `FUSE_TEST` line has arrived for about 2.5 seconds, confirm the HUD says `Browser fallback odometry`.
- Confirm `/data` includes `onboard_pose` when `FUSE_TEST` pose lines are present.
- Confirm browser-side LiDAR does not write x/y; LiDAR is display/heading context only.
- Confirm the field-only view shows x/y/heading in the canvas HUD even though the side pose card is hidden.
