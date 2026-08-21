# Assumptions

- The current negative left motor ports normalize left encoder direction in PROS; production signs remain `+1/+1` until a controlled forward push proves otherwise.
- One motor revolution equals one 2.75 inch drive-wheel revolution, per the prior user-confirmed hardware note.
- Port 5 reports centidegrees, drives a direct 2 inch wheel, is 12 cm behind robot center, and positive motion is robot-right. Scale/sign/offset still require physical calibration.
- LiDAR ports 6,7,8,9 are ordered consistently along the bar. The exact physical order/sign still requires a clockwise/counterclockwise wall test.
- The live `m17=inf` result is a physical port/device/cable fault that software can expose and safely degrade around, but cannot repair.
- There is no separate vertical tracking wheel in the code; forward/vertical displacement comes from drivetrain motor encoders.
