Verdict design target, not implemented in this review:

1. Keep known starting pose as an explicit field-pose initialization, including x, y, heading, and expected wall visibility.
2. Use IMU heading as the high-rate heading source. Keep a small `imu_bias_deg` term instead of deriving heading from left/right wheel deltas.
3. Use a vertical non-driven tracking wheel for forward displacement if mechanically possible. Until then, treat drivetrain encoders as fallback forward distance only.
4. Keep the existing horizontal odometer for lateral displacement and turn-offset compensation.
5. Fit the four distance sensors exactly as the current code does, but gate harder: installed, finite distance, under max range, confidence threshold where meaningful, RMSE, max point error, low angular velocity, known wall plausibility, and winner margin over the second-best wall.
6. Use wall theta to make a bounded IMU bias correction. Do not replace IMU heading directly.
7. Use wall distance only as secondary evidence: wall choice scoring and occasional one-axis correction when wall identity is confident.
8. Use AI Vision only as an opportunistic landmark or wall-identity confirmation input.
9. Add field-test logging that compares estimated pose against measured ground truth before trusting long scoring macros.
