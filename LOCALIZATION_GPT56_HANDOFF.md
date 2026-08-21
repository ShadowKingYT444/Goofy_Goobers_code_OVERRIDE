# MoreVex localization handoff

## Current verified state (2026-07-10)

The robot firmware and local web UI are running. The latest stationary reading after a UI-server restart was approximately `X=-48.14 in`, `Y=0.50 in`, `H=94.1 deg`; all four drive encoders were finite, the audience-wall LiDAR candidate was selected, camera confidence was about 86%, and telemetry arrived at 16.67 Hz.

Run `start_localization_ui.cmd`, then open `http://127.0.0.1:8774/localization`.

## Implemented behavior

- IMU prediction runs every 20 ms. LiDAR no longer hard-resets the IMU; it applies a bounded software bias after three consistent fits.
- The LiDAR array fits all four sensors at 10 Hz, evaluates the four modulo-90 field headings, selects using heading continuity and observable-axis innovation, and corrects the coordinate perpendicular to the observed wall.
- Live-calibrated LiDAR geometry is: sensor distance corrections `{+7, 0, -5, -2} mm`, left mount offset `5.29 in`, forward offset `0 in`, and field heading relationship `wall base + theta`.
- Drive encoder normalization is left `+1`, right `-1`. Health telemetry reports finite motor count and same-side spread. Absolute motor positions are not compared as if they shared a zero; synchronized deltas are used.
- Port-5 horizontal odometry rejects device errors, non-finite values, and implausible jumps and compensates its configured `12 cm` rear offset during rotation.
- The web UI uses onboard X/heading and fixed-camera Y when the marker is fresh and confident. Camera calibration is stored in `agent-workflow/localization-sensor-repair/camera_calibration.json`, survives server restarts, and is re-anchored by **Reset Start**.
- Both automatic startup motion/calibration flags are false. Controller deadband is 5.

## Coordinate/start contract

`include/localization_config.hpp` is authoritative:

```text
origin: field center
+X: toward the configured 0-degree wall
-X: toward the audience wall
+Y: red side
-Y: blue side
current entered pose: (-48.1, +0.5, 94.2 degrees)
physical wall half-span: 70.2 in
```

Only press **Reset Start** while the robot is physically at that configured pose. It re-anchors the browser fallback and fixed-camera Y; it cannot infer a manually relocated robot.

## Evidence

- Stationary: raw IMU span `0.02 deg`; fused heading span `1.08 deg` over 45 seconds, with stable audience-wall selection in 384/387 samples.
- Rotation: slow live trials proved the LiDAR sign and kept LiDAR/fused heading within roughly 1.4 degrees of the encoder/IMU turn estimate.
- Forward: a powered 1.5-inch trial produced motor deltas `+64,+63,+62,+63 deg`, `1.51 in` onboard travel, and under 0.4-degree heading change.
- Camera: a 0.5-inch test moved the fixed marker 14 px (`0.47 in` at 30 px/in). Restart and Reset Start tests both preserved/replaced the camera anchor correctly.
- Rendered UI: showed fused pose + camera Y, L2/2 R2/2 drive health, fresh audience-wall lock, and no browser warnings/errors.
- All localization adversarial/source/parser checks listed in `agent-workflow/localization-sensor-repair/runbook.md` pass.

## Known limitations

- One wall observes heading and only the axis perpendicular to that wall. The parallel axis is odometry-derived on the Brain; the fixed camera corrects that along-wall axis only in the PC UI.
- Camera Y assumes the camera, lighting, marker, and 30 px/in calibration remain fixed. Low-confidence or stale camera frames automatically fall back to onboard Y. Large rotations/occlusion may reduce confidence.
- This drivetrain cannot perform a powered pure lateral slide, so port-5 translation scale was not physically recalibrated during this session.
- Only the audience-wall hypothesis was live-accessible. Other wall candidates have synthetic/adversarial coverage, not a live drive test.
- A convincing flat obstacle can still look like a wall; confidence, fit, range, temporal consistency, continuity, and innovation gates reduce but cannot eliminate that risk.

## Important files

- `src/autons.cpp`: estimator, four-candidate LiDAR fusion, encoder/odom guards.
- `src/main.cpp`: hardware, telemetry rates, controller loop.
- `include/localization_config.hpp`: start pose and field contract.
- `tools/lidar_bar_server.py`: serial parser, web UI, fixed-camera Y correction.
- `agent-workflow/localization-sensor-repair/`: complete discovery/design/eval/evidence/report/runbook artifacts.
- `NEXT_CHANGES.md`: future covariance-based master-estimator design; it is not a description of fully implemented behavior.
