# Baseline

Captured 2026-07-10 before edits.

- V5 Brain is connected: communications COM9, user COM8.
- `/data` streams D4 at 50 Hz.
- Live drive sample: `m17=inf`, `m18=582.0`, `m11=-826.0`, `m13=-825.0`, with `errno=19`. The UI drops non-finite port 17 and silently averages only port 18 on the left.
- Live stationary five-second IMU sample was stable: EZ/raw range `-49.79..-49.78` degrees (0.01 degree span). This makes the reported rapid heading oscillation more consistent with the 100 ms hard-reset fusion loop than stationary raw sensor noise.
- Live LiDAR was clean/high confidence (`63`) but onboard fusion rejected it as `theta`; the raw field heading was roughly 140 degrees while the wall fit implied a correction beyond the current 35 degree gate.
- Current fusion hard-resets the EZ IMU from one LiDAR fit every 100 ms with full correction gain and no multi-sample consistency, smoothing, deadband, or correction-rate limit.
- Port 5 is converted correctly from centidegrees in the nominal case, but `PROS_ERR` is accepted as a real position and subtraction occurs in 32-bit integer arithmetic.
- Existing parser and synthetic odometry checks pass. Two localization source checks fail because they expect the prior bias-only/1000 ms design while production now uses 100 ms hard resets.

## Full-pose continuation baseline, 2026-07-10

- Current firmware still chooses a wall from predicted pose distance first, considers only that wall's 180-degree heading pair, and never corrects X/Y from wall distance.
- Live old estimator state had drifted to `x=-71.40`, `y=42.56`, heading about `4.33`, with bias saturated at `-90`, despite the robot being placed back at the known start geometry. Absolute motor positions were not synchronized to that pose.
- All four drivetrain encoders are currently finite and same-side matched; the earlier port-17 fault is not present in the latest live sample.
- The known anchor is the configured start `(-48, 0, 90 deg)`: centered along the audience wall and perpendicular to it. Robot-center distance to that wall is `22.2 in`.
- 81 unique stationary LiDAR frames measured perpendicular distance median `16.874 in`, implying a leftward LiDAR-center offset of `5.326 in` from the robot rotation center.
- The nominal perpendicular fit falsely reported median theta `3.155 deg` and RMSE `0.110 in` because sensor mounting/range offsets differ. Per-sensor corrections `{+7, 0, -5, -2} mm` reduced a follow-up sample to median theta approximately `0.0 deg`, median RMSE `0.051 in`, and median distance `16.913 in`.
- Literal field center `(0,0)` is 70.2 inches from every boundary, beyond the 50-inch sensor gate. Here “center” is the configured `y=0` centerline along the audience wall, not the field origin.
