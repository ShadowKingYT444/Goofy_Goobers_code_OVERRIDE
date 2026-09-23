# Evaluation plan

- Hardware inventory: scan ports 1-21 and prove AI Vision port/type before production integration.
- Rear wheel: stationary noise, forward/back cross-axis leakage, repeated paired 10-15 degree CW/CCW turns, fitted sign/offset, and pure-rotation cancellation.
- Turn correlation: for every returned-to-center segment record all four drive encoders, field-frame IMU delta, modulo-wall LiDAR delta, and rear-wheel delta. Fit `W=(dR-dL)/dtheta_imu`, rear lever arm, LiDAR/IMU scale, and LiDAR/IMU RMS residual. Repeat both signs and reject invalid LiDAR fits rather than filling missing data.
- Differential micro-turn: verify left +1.000 inch/right -1.000 inch and the inverse against the fitted effective track width; measured travel, not the command target, is authoritative.
- LiDAR: clean wall correction, deliberate obstruction/dropout, stale-data rejection, and smooth reacquisition.
- Vision: known-pose tag detection, timestamp/freshness, duplicate-ID ambiguity, back-face/FOV checks, shadow residuals, then bounded correction.
- Start pose: enter a non-default pose without recompiling and prove Brain/UI agreement after reset.
- UI: operational onboard pose remains unchanged when the Brio tracker is stopped; camera appears only as diagnostic comparison.
- Regression: all existing localization checks plus new geometry, outage, ambiguity, and telemetry tests.
- Live route: controller-driven movement within the allowed 3 feet and 720 degrees, with independent camera comparison and heading error below 10 degrees.
