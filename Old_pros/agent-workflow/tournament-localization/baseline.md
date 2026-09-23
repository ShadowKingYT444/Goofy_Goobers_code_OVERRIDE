# Baseline

Captured 2026-07-10 before tournament-localization production edits.

- Production firmware uses PROS 4.1.1 and contains no AI Vision instance or fusion.
- AI Vision smoke project uses PROS 4.2.2 and assumed port 1. A live device scan disproved that assumption: port 1 type 0 (empty), port 20 type 29 (AI Vision).
- Reconfigured stationary smoke test on port 20 reports `installed=1`, tag detection enabled (`enabled=1`), temperature about 42-43 C, and zero detections at the current orientation.
- Current webcam frame shows the on-robot camera facing away from the nearby Goal/tag, explaining the zero detections.
- Production UI currently substitutes fixed-webcam Y for onboard Y, violating the tournament-only sensor contract.
- Current nine-landmark map stores Goal centers and IDs but not 36 tag-face transforms, tag height/size, camera intrinsics, or robot-to-camera extrinsics.
- Current LiDAR/IMU/odometry fusion is live, but LiDAR-blocked AprilTag fallback is absent and unproved.
- No automatic startup movement is enabled.
