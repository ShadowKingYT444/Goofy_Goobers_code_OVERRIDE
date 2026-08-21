# Failure cases

1. A single clean but noisy LiDAR fit causes an immediate hard IMU reset and steering oscillation.
2. Alternating wall-fit noise drives heading back and forth instead of converging with bounded corrections.
3. A large but plausible initial IMU error is permanently rejected by the 35 degree theta gate.
4. Missing motor 17 is silently treated as a healthy two-motor left side.
5. Same-side encoders disagree substantially but their average is still integrated.
6. Absolute left/right positions are compared even though they were not synchronously reset.
7. Port 5 returns `PROS_ERR`; integer subtraction overflows and creates huge lateral travel.
8. Port 5 produces an implausible one-frame jump; pose accepts it.
9. Browser fallback constants drift from Brain constants/tests.
10. Angular derivative crosses wrap and produces a controller spike.
11. Pose-first wall selection rejects every wall when odometry has drifted.
12. A clean fit produces only two headings instead of all four modulo-90 candidates.
13. A 180-degree opposite wall switch passes an unoriented line-angle consistency test.
14. LiDAR mounting translation is ignored, shifting absolute X/Y by about five inches.
15. Per-sensor depth/range offsets create a false nonzero wall angle while physically perpendicular.
16. Wall distance incorrectly changes both X and Y instead of only the observable axis.
17. A candidate wall changes without three consistent directional wins.
18. PROS hot upload preserves a function-local telemetry pose while hardware encoders reset, publishing a stale position with zeroed sensors.
19. A hot restart skips the expected initialization callback; estimator state must detect the backwards program clock before integrating reset encoders.
