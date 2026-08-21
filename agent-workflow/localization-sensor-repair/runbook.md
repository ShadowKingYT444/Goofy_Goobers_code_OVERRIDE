# Localization runbook

## Normal UI use

```powershell
cd C:\Users\terry\Downloads\MoreVex
.\start_localization_ui.cmd
```

Open `http://127.0.0.1:8774/localization`. The dashboard should say **Fused pose + fixed camera Y**, show `Drive sensors L 2/2 | R 2/2`, and update while the controller moves the robot. The current configured anchor is approximately `X=-48.1 in`, `Y=0.5 in`, `H=94.2 deg`.

Use **Reset Start** only while the robot is physically at the configured anchor. It also saves the camera marker's current pixel as field `Y=0.5 in`. A reset changes the browser/camera reference; it cannot correct a robot that is placed somewhere else.

## Build and upload

```powershell
cd C:\Users\terry\Downloads\MoreVex
$env:APPDATA=(Resolve-Path .\.pros-appdata).Path
$env:PROS_TOOLCHAIN=(Resolve-Path .\.pros-toolchain\usr).Path
.\.venv\Scripts\pros.exe make
.\.venv\Scripts\pros.exe upload --port COM9 --slot 1 --after run
```

## Regression checks

```powershell
.\.venv\Scripts\python.exe agent-workflow\localization-sensor-repair\localization_repair_check.py
.\.venv\Scripts\python.exe agent-workflow\localization-architecture-review\localization_failure_checks.py
.\.venv\Scripts\python.exe agent-workflow\localization-architecture-review\autons_fusion_source_check.py
.\.venv\Scripts\python.exe agent-workflow\fusion-test-auton\fusion_test_source_check.py
.\.venv\Scripts\python.exe agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe agent-workflow\field-odometry-ui\odometry_sign_check.py
.\.venv\Scripts\python.exe agent-workflow\field-odometry-ui\telemetry_parser_check.py
.\.venv\Scripts\python.exe agent-workflow\master-fusion-design\master_fusion_note_check.py
.\.venv\Scripts\python.exe agent-workflow\localization-sensor-repair\camera_tracker_check.py
.\.venv\Scripts\python.exe -m py_compile tools\lidar_bar_server.py
```

## Quick live acceptance

1. Leave the robot still for 30 seconds. Raw IMU should remain nearly fixed and fused heading must not rapidly oscillate.
2. Drive straight a short measured distance. Left/right synchronized deltas should have the same sign and similar magnitude.
3. Rotate slowly a few degrees. The LiDAR wall label should remain stable when a clean wall is in range, and heading correction should move in the same direction as the physical turn.
4. Confirm the UI pose follows the motion and its telemetry age stays below one second.

The LiDAR fit runs at 10 Hz; do not increase it for idle operation without a measured need.
