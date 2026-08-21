# Failure cases

- Laptop webcam missing, stale, or deliberately wrong changes official pose.
- All LiDAR beams are blocked and pose freezes, resets, becomes NaN, or falsely claims absolute lock.
- Duplicate tag ID selects the wrong Goal or wrong face and teleports pose.
- Same camera frame is applied repeatedly as independent evidence.
- Tiny, clipped, skewed, blurred, back-facing, or unknown tag is accepted.
- A moved Goal or flat obstacle overrides consistent IMU/odometry without corroboration.
- Camera yaw, offset, tag size, face radius, or field map is guessed rather than calibrated.
- IMU wrap, encoder fault, or port-5 error creates a discontinuity during LiDAR outage.
- Reset Start is pressed away from the entered pose and silently appears valid.
- Test motion exceeds 36 inches or 720 degrees cumulative.
