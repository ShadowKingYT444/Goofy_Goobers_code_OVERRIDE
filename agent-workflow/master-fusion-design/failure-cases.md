# Failure Cases

- A clean line on another robot is accepted as a field wall.
- A duplicate AprilTag ID selects the wrong Goal and teleports the pose.
- A clipped, skewed, stale, or motion-blurred tag frame is counted as fresh truth.
- Repeated polling of one cached sensor frame falsely collapses uncertainty.
- A large innovation is trusted merely because it is large.
- Corrections are applied during fast turns and create heading oscillation.
- Per-frame corrections are too aggressive and make movement controllers jerk.
- Camera mounting yaw or offset error becomes systematic forward-position error.
- Goal-center coordinates are used without accounting for which of four Goal faces contains the visible tag.
- No valid observation is available, but the system continues reporting high confidence.
- LiDAR and camera disagree and one is selected without independent corroboration.
- Sensor latency compares a delayed measurement against the current pose instead of the pose at capture time.
- The movement controller and localization task race over mutable pose state.
- A filter passes simulation but has no recorded-log replay or 90-second ground-truth test.
