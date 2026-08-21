# Current Baseline

Date: 2026-07-09

Confirmed from the active firmware:

- `update_pose()` runs inside fused movement loops every 20 ms.
- Drivetrain motor encoders provide forward displacement.
- Port 5 horizontal odometry provides sideways displacement after rotational-offset compensation.
- IMU is the high-rate heading source with an additive `imu_bias_deg`.
- The four distance sensors are line-fit on every eligible pose update.
- An accepted LiDAR correction is limited to once every 1000 ms.
- LiDAR applies 20% of heading innovation, capped at 1 degree per update and 15 degrees total.
- LiDAR rejects heading innovations above 18 degrees and uses fit, distance, confidence, angular-rate, wall-hypothesis, and winner-margin gates.
- Active fused drive and turn controllers use `LidarFusionMode::kBiasOnly`, so LiDAR does not hard-reset the IMU during those movements.
- AI Vision is not instantiated in the main project and performs no pose correction. AprilTag handling exists only in `ai_vision_smoke` and PC tools.
- No covariance, uncertainty growth, observation age, temporal-consistency window, normalized innovation gate, or recovery state exists in the active estimator.

Verdict: the current firmware is partial fusion, not the proposed master fusion system.
