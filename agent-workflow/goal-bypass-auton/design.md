Use an onboard fused-pose waypoint follower for a conservative right-side bypass:

1. Estimate pose from drivetrain motor encoders for forward/heading and port 5 horizontal odometer for side displacement.
2. When the four LiDAR sensors form a clean wall line under 50 in with RMSE <= 0.20 in, project the pose onto the selected field wall line.
3. Select the wall hypothesis using both odometry distance agreement and theta agreement with current odometry heading.
4. Drive relative waypoints from the corrected start pose: up 8 in, right/up around the goal, back toward centerline, then finish 36 in upfield.

Run telemetry in a background task during autonomous so the web field view can keep showing the same sensor stream.
