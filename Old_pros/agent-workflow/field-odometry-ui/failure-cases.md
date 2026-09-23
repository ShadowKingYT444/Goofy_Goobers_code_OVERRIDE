- No motor telemetry: field view should stay visible and say waiting for motor telemetry.
- One side missing: do not integrate pose until both left and right averages are available.
- Page reload: reset pose baseline cleanly instead of jumping from absolute encoder values.
- Large movement or turn: use arc integration rather than simple straight-only math.
- Robot leaves field bounds: clamp drawing position to the field while still reporting pose.
- The dashboard shows browser-estimated odometry while the robot firmware has a different fused x/y/heading estimate.
- LiDAR wall geometry in the browser changes x/y even though the current robot architecture only uses LiDAR wall theta to correct IMU heading.
- A full-screen field route hides the side pose card, making x/y/heading unreadable during a quick autonomous test.
## Horizontal odometer and LiDAR assist

- Sideways omni movement does not change left/right motor encoders, so a drive-only dot stays still.
- Reapplying the same LiDAR frame on every browser animation frame causes visual drift even if numbers are stable.
- Port 5 centidegrees treated as degrees makes side travel 100x too large.
- Ignoring the odometer's 12 cm rear offset creates fake side movement during in-place turns.
- LiDAR readings above 50 in can pull the pose toward a bad wall estimate.
- A stale port conflict in code can command port 5 as a motor while current hardware uses port 5 for the horizontal odometer.
