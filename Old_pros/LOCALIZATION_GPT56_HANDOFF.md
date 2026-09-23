# MoreVex localization handoff

## Live resume (2026-08-25; supersedes the runtime warning below)

The Brain user runtime recovered after a real power cycle and slot 4 emitted
live D4/FUSE/VISION telemetry with no startup motion. During a later stationary
soak, the complete VEX USB device disappeared at 20:56:52 after 19.28 min of
valid data. The Brio also shows only the field corner/storage area, not the
robot. Reconnect the Brain and restore visual coverage before any actuation.

The most important new result is a P6 calibration-quality failure: one
API-ready calibration crossed 0.10 degrees of false rotation after 126.8 s and
0.50 degrees after 183.4 s, then drifted 4.25 to 15.10 degrees over ten minutes
(1.074 degrees/min) while every drive encoder remained fixed and P7 heading
spanned 0.17 degrees. A warm stationary recalibration then held 0.02 degrees
peak-to-peak over 11.89 min. Installed/non-calibrating/finite status is not
sufficient motion authority. A candidate relative P6/P7/encoder stopped gate
detects the saved failure, but must be estimator-session scoped and reset on
every deliberate tare; do not deploy it until real turns validate its reset
and false-positive behavior.

D4 now carries `imugyro=x,y,z` and `imuacc=x,y,z`; both host consumers parse
them. P7 raw Z gyro remains diagnostic-only because it reports a persistent
roughly -0.4 to -0.6 stopped bias. P8 geometry uses a 0.625-in inner detected
square; the older 0.875-in ranges below are superseded. Current evidence:

- `reports/sensor_campaign_2026-08-25/live_resume_addendum.md`
- `reports/sensor_campaign_2026-08-25/imu_calibration_experiment/summary.json`
- `reports/sensor_campaign_2026-08-25/goal_completion_audit.md`

## Current verified state (2026-08-23; supersedes the 2026-07-10 section)

This is a replacement robot. Do not use the older port map or claim that its
four-sensor LiDAR/P20 camera is present.

- Drivetrain: P17/P18 left, P11/P13 right; differential eight-wheel 2.75-in
  omni drive. Encoder scale is `0.8847477281`, fit RMSE 0.215 in across nine
  2/5/10-in trials. This is a provisional P7-GPS-referenced fit, not independent
  tape/laser ground calibration; preserve the paired 2.433055-in effective
  wheel and 10.624582-in effective track until both are surveyed together.
- P6 IMU: primary heading. Stationary std 0.00463 degrees; one completed fused
  four-leg turn returned within 0.088 degrees. The 0.91-degree commanded sweep
  return residual is not external accuracy evidence; production uses a
  provisional 2.0-degree heading/controller allowance.
- P7 GPS: 6 in right, 6 in behind, lens robot-right. It can be extremely quiet
  while stationary but produced 15.63-in false turn displacement and up to
  27.85-in corrected-frame innovations. It is now only a bounded, heavily
  gated correction.
- P1 Distance: forward 8-in autonomous stop only. It fails closed on missing,
  API-error, and malformed values, but cannot distinguish a real missed object
  from the documented healthy 9999-mm/no-target result. It is not a certified
  collision envelope.
- P8 AI Vision: Circle21h7, official medium outer black square 0.875 in. Tag 4
  onboard edge/focal horizontal range was 25.559-in median; offline PnP was
  25.763 in and gave 0.204-px reprojection RMSE. PnP/plane-normal solving is
  offline only. Camera field correction is disabled pending calibrated
  intrinsics, extrinsics, tape validation, and face transforms. Deterministic
  obliquity analysis found +7.7% range bias at 30-degree tag yaw and +20.7% at
  45 degrees even when current gates pass.
- Official Override v1.1 Appendix A proved the old Goal coordinates/colors were
  right but all eight nonzero tag IDs were shifted by two. Firmware and UI now
  use `4,3,1,2 / 2,1,3,4` around the field (center 0). Correction was disabled,
  so the stale map never moved the fused pose.
- P5 Rotation tracker: mechanically inactive (0–0.002 in), fusion disabled.
- P9 is the left slider motor; the old LiDAR array is unavailable.

`include/navigation.hpp` is the public blocking API: init (including a
caller-supplied start-position error bound), 20-ms idle update, fresh current
pose, sensor health, fixed 512-point path history, turn, fixed-line go-to,
thread-safe stop, and result names.
The competition `autonomous()` callback is no-motion until an actual route is
defined; controller-launched diagnostic motion is compile-time disabled.

Critical deployment warning: all source motion flags are false, 50/50 host
tests pass, and the current safe binary was uploaded to slot 1 with no
automatic run. Its SHA-256 is
`192c4adc4f16805a4a3fb8c26d82867ed26b546c1070fafc21e8fefd143554a1`.
The V5 system channel `/dev/ttyACM0` remains healthy, but two bounded run checks
produced no D4/user telemetry on `/dev/ttyACM1`, including after the final safe
upload. The program was explicitly stopped. Do not attempt navigation or fault
injection until reliable user-program execution/telemetry is restored and the
boot inventory is verified.

Recovery order is safety-critical:

1. diagnose the Brain user-runtime/user-CDC failure without commanding motors;
2. obtain a current illuminated external view and confirm the tether/field is
   clear before any program run;
3. rerun the 50 host checks, confirm every startup/competition diagnostic flag
   is false, and verify the binary hash above;
4. read live boot inventory and confirm P17/P18/P11/P13/P6/P7/P1/P8 (plus
   optional inactive P5);
5. keep movement disabled until pose initialization, current sensor health,
   Brio visibility, robot footprint, and the specific short test path are
   independently checked.

Drive-side encoder loss or IMU loss now aborts blocking motion and latches the
pose invalid until an explicit `navigation::init()`. P8 association consistency
counts each camera poll at most once; rereading one cached snapshot cannot
satisfy its temporal gate. Fresh failed or ambiguous P8 polls reset the chain,
so accepted observations must be consecutive.
Exact repeated P8 corners are additionally rejected if independent encoders
move over 0.5 in or P6 changes over 2 degrees, guarding against cached optical
data despite a fresh API poll ID.
The web UI no longer uses the old robot's P8 extrinsics: it displays `mount
UNMEASURED` and suppresses field-position hypotheses until the new mount is
physically measured.

P1 forward stopping explicitly distinguishes the documented 9999-mm
healthy/no-target result from PROS's positive `PROS_ERR` failure sentinel. A
failed distance or confidence call aborts forward autonomous motion.

P7 correction is explicitly stopped-robot only. Its consistency chain resets
above 0.5 in/s encoder speed or 12 deg/s rotation; drive encoders plus P6 own
moving legs, and deliberate brief pauses permit bounded GPS correction.

The straight-line controller now brakes immediately during its finish-window
settle timer. An offline 90,000-trial sweep found 1.94-in 95th-percentile actual
error and 96.25% within 2 in for a 48-in/power-40 leg across the provisional
scale-plus-heading envelope; only 50.55% of 84-in/power-40 trials stayed within
2 in. Unobserved slip/push reached 7.14-in p95 / 8.57-in max while the estimator
still claimed 0.67 in. The minimum reported outage envelope grows by 4.104% of
travel and still excludes common scale-reference bias, slip/push, collision,
braking, and placement error beyond the supplied init bound.
Do not call it competition-qualified until the revised live out/return passes.
The production map also contains only walls and nine Goals; official Pins,
Loaders, Toggles, movable Blocks, other robots, and the measured swept robot
footprint must be added or independently avoided before arbitrary navigation.

Authoritative evidence:

- `reports/sensor_campaign_2026-08-23/qualification_report.md`
- `reports/sensor_campaign_2026-08-23/sensor_qualification_dashboard.png`
- `reports/sensor_campaign_2026-08-23/current_gps_gate_replay_dashboard.png`
- `reports/sensor_campaign_2026-08-23/gps_orientation_reliability_dashboard.png`
- `reports/sensor_campaign_2026-08-23/live_reconnect_addendum.md`
- `reports/sensor_campaign_2026-08-23/live_reconnect_sensor_dashboard.png`
- `NAVIGATION.md`

The older section below is retained only as historical evidence.

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
