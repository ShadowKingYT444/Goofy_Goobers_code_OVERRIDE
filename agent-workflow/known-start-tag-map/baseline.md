# Baseline

Date: 2026-07-09

- `src/autons.cpp` hard-codes localization start pose as `(0, 0, 90 deg)`.
- IMU conversion is also hard-coded around 90 degrees, so a different entered start heading cannot currently work correctly.
- `initialize()` calibrates the IMU but does not explicitly reset the post-calibration reading to zero.
- No shared field landmark table exists in the main firmware.
- The separate `ai_vision_smoke` project uses AI Vision port 1 and Circle21h7 tags, but that smoke project is not fused into main localization.

Baseline checks:

- `fusion_test_source_check.py`: passed.
- `localization_failure_checks.py`: passed.
- `telemetry_parser_check.py`: passed.
- `odometry_sign_check.py`: passed.
- `autons_fusion_source_check.py`: failed because it still expects the older 2500 ms LiDAR cadence while current firmware uses 1000 ms.
- PROS build with repo-local APPDATA/toolchain: passed with `make: Nothing to be done for 'quick'`.
