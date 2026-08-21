# Runbook

Edit the robot-center start pose in:

```text
include/localization_config.hpp
```

Run checks and build:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
$env:APPDATA=(Resolve-Path .\.pros-appdata).Path
$env:PROS_TOOLCHAIN=(Resolve-Path .\.pros-toolchain\usr).Path
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Hardware validation is required before relying on the entered pose. Measure to the robot rotation center, not a bumper corner.

Start the field dashboard:

```powershell
.\.venv\Scripts\python.exe .\tools\lidar_bar_server.py
```

Open `http://127.0.0.1:8774/playing-field`.
