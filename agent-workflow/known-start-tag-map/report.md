# Report

## Outcome

Implemented a center-origin, inch-based Override coordinate frame and fixed nine-Goal AprilTag landmark map. Robot start X, Y, and field heading are now entered in one config object. The physical IMU is reset to raw zero while localization preserves the entered field heading as the raw-zero reference.

Live AI Vision position correction is intentionally deferred. The map records that AI Vision will be opposite the left LiDAR array, but the final camera yaw and robot-center offsets must be measured first.

## Changed files

- `include/localization_config.hpp`: editable start pose, coordinate constants, sensor sides, fixed Goal/tag map.
- `src/autons.cpp`: arbitrary start pose, raw-zero IMU field reference, center-origin LiDAR wall hypotheses.
- `src/main.cpp`: explicit post-calibration IMU reset and startup log.
- `tools/lidar_bar_server.py`: signed center-origin rendering and nine Goal/tag markers.
- `NEXT_CHANGES.md`: current hardware and localization direction.
- `agent-workflow/known-start-tag-map/*`: design, tests, runbook, evidence, and acceptance state.
- `agent-workflow/localization-architecture-review/autons_fusion_source_check.py`: current one-second, heading-only LiDAR contract.

## Commands run

- New map/start-pose adversarial check: passed.
- Fusion autonomous source check: passed.
- Localization fusion source check: passed after replacing stale expectations.
- Localization failure checks: passed.
- Telemetry parser and odometry sign checks: passed.
- Python syntax checks: passed.
- Served JavaScript `node --check`: passed.
- `/playing-field` and `/data` foreground HTTP checks: passed.
- PROS firmware build: passed and produced `bin/hot.package.bin`.

## Failed attempts

- Baseline architecture check failed because it required the removed wall-axis correction and old 2500 ms cadence; the checker was updated to the current heading-only 1000 ms design.
- In-app browser discovery returned unavailable, so no browser screenshot was captured.
- Detached dashboard processes are terminated by this execution environment. A foreground server was used for HTTP verification and then stopped cleanly.

## Remaining risks

- No physical robot test proves the IMU sign, entered start pose, encoder scale, or LiDAR wall-name mapping yet.
- Drivetrain encoder slip still causes robot-forward translation drift.
- Tag IDs 1-4 are duplicated, so future camera fusion must reject ambiguous Goal candidates.
- Goal coordinates are ideal 24-inch tile coordinates. Actual field placement tolerance must be measured if sub-inch correction is expected.
- AI Vision correction is not active yet.

## Manual verification

1. Enter measured robot-center X, Y, and field heading in `include/localization_config.hpp`.
2. Build/upload and place the robot at that pose.
3. Start X+Down and confirm `FUSE_INIT` reports the entered pose with raw IMU near zero.
4. Rotate clockwise and confirm field heading decreases.
5. Open `http://127.0.0.1:8774/playing-field`; confirm +X is top, +Y/red is left, and the Goal IDs match `FIELD_2d_VIEW.jpg`.

Status: implemented and source/build validated; physical robot and visual browser validation remain.
