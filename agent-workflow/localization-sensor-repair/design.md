# Design

1. Keep the IMU raw zero stable after initialization. Convert LiDAR into a software heading-bias observation; never repeatedly reset the physical/EZ IMU in the pose loop.
2. Require three mutually consistent clean LiDAR headings, then apply a bounded low-pass bias step with a deadband. Permit wider initial reacquisition while retaining wall-distance, confidence, fit-quality, and spin gates.
3. Read drivetrain sides with health metadata. Allow explicit one-motor degraded forward integration so the currently connected robot remains observable, but reject a side when two finite motor deltas disagree beyond a tight threshold. Log health instead of hiding it.
4. Reset drivetrain encoder baselines during robot initialization so raw diagnostics begin together. Preserve current `+1/+1` logical signs because PROS normalizes negative motor ports; verify with a controlled push before changing them.
5. Validate port 5 installation and `PROS_ERR`, subtract in 64-bit/double space, and reject physically impossible sample jumps. Keep the previous valid baseline on invalid input.
6. Keep Brain and browser conversion constants aligned and update regression tests to test the production signs and fault behavior.
7. Fix wrapped-angle derivative math in fused turning.

## Full wall-pose fusion design

1. Apply the measured zero-mean distance corrections `{+7, 0, -5, -2} mm` before fitting the four-sensor line. Configure the LiDAR center `5.29 in` left of the robot rotation center; forward offset remains zero pending measurement.
2. From every clean fit angle `theta`, generate headings `normalize(theta + 90*k)` for `k=0..3`. Live slow CW/CCW calibration established this sensor-order sign. For a left-facing bar these correspond to red, audience, blue, and zero walls.
3. For each candidate, transform the measured perpendicular wall distance back to a robot-center X or Y using the rotated LiDAR mount offset. Score all four candidates from full heading continuity plus observable-axis innovation. Do not select a wall before generating the four headings.
4. Require a unique score winner, a valid in-field coordinate, and three consecutive observations with the same wall, directed heading, and nearby axis coordinate.
5. Apply the existing bounded heading-bias correction and an additional bounded axis correction. Never modify the coordinate parallel to the observed wall.
6. Log best/second scores, margin, measured distance, observed axis coordinate, and applied axis step for live diagnosis.
7. Validate stationary anchor first. Only after the field is clear, run a one-shot low-power calibration: small CW/return/CCW/return turns and a 3–4 inch forward move with LiDAR disabled during motion and hard watchdogs.
# Fixed-camera restart contract

The PC-side camera correction must use a calibration anchored to the fixed camera, not an in-memory first frame. Store the marker's anchor pixel, field Y, and pixel scale in `camera_calibration.json`; load and validate it before tracking. A server restart must retain that anchor. The existing **Reset Start** action may replace the stored marker pixel only when the robot is physically at the configured start pose. Missing or malformed calibration falls back to one deliberate first-frame initialization and records that initialization for later restarts.
