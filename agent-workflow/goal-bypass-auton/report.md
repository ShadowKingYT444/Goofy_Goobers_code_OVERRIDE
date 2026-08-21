Changed files:

- `src/autons.cpp`
- `include/autons.hpp`
- `src/main.cpp`

What changed:

- Added `simple_goal_avoidance_auton()` with a right-side obstacle bypass route.
- The autonomous now estimates pose onboard from drivetrain encoders, port 5 horizontal odometer, and gated LiDAR wall projection.
- LiDAR correction only runs when all four sensors produce a clean wall line under 50 in with RMSE <= 0.20 in.
- Wall hypothesis choice uses both odometry distance agreement and theta agreement with current odometry heading.
- `main.cpp::autonomous()` starts a telemetry task and runs the fused autonomous.
- Updated chassis config to 2.75 in drive wheels and 450 rpm motor cartridge.

Verification:

- `pros make` passed with existing LVGL/linker warnings.
- Upload to COM15 succeeded.
- Live COM14 D4 telemetry returned at 50 Hz after upload.

Remaining risk:

- The low-level waypoint controller direction signs, gains, and bypass clearance still need one physical field test.
