# Evaluation plan

- Baseline: record live USB, `/data`, individual motor values, IMU range, LiDAR confidence/reject state.
- Static checks: Python compile, parser tests, embedded JavaScript syntax, new source/algorithm regression.
- Adversarial simulation: noisy LiDAR, large initial heading error, inconsistent consecutive fits, missing motor, opposite same-side motor deltas, port-5 error/jump, heading wrap.
- Regression: existing field odometry, localization failure, known-start, and fusion source checks updated only where the intended design changed.
- Firmware: run PROS build with the repo-local toolchain.
- Live/manual: upload only if build passes and Brain remains connected; then verify stationary heading, 12 inch forward push, pure right/left slide, and small CW/CCW wall-angle test.

Pass criteria:

- No repeated `drive_imu_reset` in LiDAR correction.
- Clean repeated LiDAR observations monotonically reduce heading error with each update capped.
- One bad/alternating fit cannot move bias.
- Invalid port-5 samples add zero pose displacement and do not poison the baseline.
- Missing/disagreeing motors produce explicit health state; no silent healthy average.
- Build and all current acceptance scripts pass, or blockers are reported precisely.

Full-pose criteria:

- A theta sweep generates exactly four unique headings separated by 90 degrees, including wraparound.
- The known anchor selects the audience wall and converges to heading `90 +/- 1 deg`, X `-48 +/- 1 in`, while leaving Y unchanged.
- A nonzero mount offset test fails under zero-offset math and passes with the configured transform.
- Symmetric/tied hypotheses reject unless heading/start continuity makes one candidate a clear winner.
- Alternating walls/headings and one-frame distance/theta outliers cannot update pose.
- Stationary ten-second peak-to-peak target is at most 0.5 degrees and 0.5 inches after settling.
- Slow turn tests confirm theta sign and no 90-degree candidate swap. Slow perpendicular movement changes the wall-derived axis by measured travel; parallel coordinate is not claimed by LiDAR.
