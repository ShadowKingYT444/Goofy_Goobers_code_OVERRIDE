Commands:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py
.\.venv\Scripts\pros.exe make
```

Manual field checks required before calling localization reliable:

- Initialize from the known field start and log x/y/heading while the robot is stationary.
- Drive forward 24 in slowly, then aggressively, and compare estimated y against tape-measured displacement.
- Push or skid the robot forward with drivetrain wheels slipping and confirm whether vertical odom, if installed, reports true displacement.
- Slide right/left with drive wheels stationary and confirm horizontal odometer sign and scale.
- Turn clockwise/counterclockwise 90 deg and compare IMU heading against visible robot heading.
- Put the left-facing LiDAR bar 12-36 in from a known wall and sweep through small heading errors. Confirm wall theta sign and bias correction direction.
- Repeat the wall test while two wall hypotheses are plausible and confirm the winner-margin gate rejects correction.
- Run a long scoring macro and compare final pose against measured ground truth.

Pass criteria:

- Static checks pass.
- Ambiguous wall hypotheses are rejected, not arbitrarily selected.
- IMU heading is the heading source used by the estimator and controller.
- Wall theta changes `imu_bias_deg` slowly and only under strict gates.
- Forward displacement comes from a non-driven vertical odometer or is explicitly labeled fallback-quality.
- Physical tests show bounded error over the scoring macro route.
