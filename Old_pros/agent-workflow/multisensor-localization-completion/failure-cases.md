# Failure cases

- Straight forward/backward motion incorrectly changes the lateral odometer estimate.
- Pure rotation creates fake X/Y because rear offset or sign is wrong.
- A nominal drivetrain track width predicts the wrong turn because tire scrub and gearing make the effective width different.
- LiDAR theta is compared without same-wall/modulo-90 association, producing a false 90-degree disagreement.
- Same-side coupled motors disagree, but their average is accepted as trustworthy drive travel.
- CW and CCW calibration yield incompatible offsets because of slip or sign error.
- LiDAR sees a goal, robot, or obstruction as a field wall.
- LiDAR is blocked or out of its 50 in useful range and a stale correction is reused.
- A repeated tag ID selects the wrong one of two goals or the wrong tag face.
- A clipped, blurred, back-facing, stale, or cached AI Vision detection moves pose.
- Laptop webcam unavailability changes the operational fused pose.
- Operator resets to a compiled pose different from the actual round start.
- Port 9 mechanism traffic interferes with the fourth distance sensor.
- A failed drive encoder leaves stale `L2/R2` health visible.
