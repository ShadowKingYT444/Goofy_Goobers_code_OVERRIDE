# Evaluation Plan

Automated checks:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
$env:APPDATA=(Resolve-Path .\.pros-appdata).Path
$env:PROS_TOOLCHAIN=(Resolve-Path .\.pros-toolchain\usr).Path
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Adversarial source cases:

- A nonzero configured start heading must not change the physical IMU reset value from zero.
- Every duplicate tag ID must have exactly two map entries; tag 0 must have exactly one.
- Coordinates and IDs must match the supplied image layout at 24 inches per tile step.
- The four user examples must exist exactly: ID 4 blue `(24,-48)`, ID 3 lower blue `(-24,-48)`, ID 2 bottom blue `(-48,-24)`, and ID 1 bottom red `(-48,24)`.
- No 90-degree start-heading constant may remain in IMU conversion helpers.

Manual robot check:

1. Enter measured robot-center X/Y and field heading in `include/localization_config.hpp`.
2. Place the robot at that pose and start X+Down.
3. Confirm the first `FUSE_TEST` pose equals the entered pose.
4. Confirm the raw IMU was reset to zero while displayed field heading equals the entered heading.
5. Rotate clockwise about 20 degrees and confirm field heading decreases by about 20 degrees.
