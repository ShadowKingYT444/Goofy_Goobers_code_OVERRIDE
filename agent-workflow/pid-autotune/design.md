Research basis:

- Ziegler-Nichols closed-loop tuning uses ultimate gain and oscillation period, but it is aggressive and can overshoot.
- Relay autotune can estimate ultimate gain from forced oscillation, but it still intentionally oscillates the robot.
- For this robot, a safer first implementation is an experiment-based candidate scorer: run short bounded step trials, score overshoot, RMS error, and settle time, then apply the best observed constants.
- Derivative action is kept, integral starts at zero, and anti-windup clamps are included. This matches the practical guidance that derivative damps oscillation while integral can create windup and is often unnecessary for short position moves.

Implementation:

- `src/pid_autotune.cpp` owns the autotuner.
- Drive candidates run alternating +12 in and -12 in tests using all four drivetrain encoders through left/right averages.
- Turn candidates run alternating +35 deg and -35 deg tests using the IMU.
- Each trial has speed caps, timeouts, overshoot measurement, RMS error, and settled detection.
- The best drive constants are applied with `chassis.pid_drive_constants_set(...)`.
- The best turn constants are applied with `chassis.pid_turn_constants_set(...)`; heading constants are derived conservatively from the selected turn P/D.
- `src/main.cpp` exposes the routine through `X + Down` in opcontrol.
