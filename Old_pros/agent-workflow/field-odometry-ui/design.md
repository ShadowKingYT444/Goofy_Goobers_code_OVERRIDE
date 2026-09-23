Add a field panel to the existing dashboard. Compute left/right wheel travel from averaged motor position deltas, wheel circumference, and track width. Integrate differential-drive pose from bottom-left facing upward. Render a 144 in square field, path trail, robot marker, heading arrow, and pose text.

Add port 5 horizontal odometer telemetry as `h5=<centidegrees>` in the existing D4 serial line. Convert centidegrees through the 2 in odometer wheel circumference and integrate the local right/left delta with the forward drive delta. Subtract the rotational contribution from the odometer's 12 cm rear offset before transforming local movement into field X/Y.

Use the left-facing LiDAR fit as a small wall-assist correction only when the wall fit is clean: all four sensors present, confidence passes the existing gate, RMSE <= 0.20 in, max point error <= 0.75 in, and all wall distances are under 50 in. Pick the nearest field wall along the robot-left ray and blend 15% of the distance error, capped to 3 in per telemetry frame.

2026-07-08 update:

- Parse onboard `FUSE_TEST` pose lines from the same serial stream as `D4`.
- Expose the latest onboard fused pose as `onboard_pose` in `/data`.
- Make the field grid prefer fresh onboard fused pose for x/y/heading and fall back to browser odometry only when no fresh onboard pose exists.
- Add a field HUD so `/playing-field` and `/pose-grid` show x/y/heading directly on the canvas.
- Keep LiDAR wall handling heading-only in the display path; the browser no longer projects `pose.x` or `pose.y` from LiDAR.
